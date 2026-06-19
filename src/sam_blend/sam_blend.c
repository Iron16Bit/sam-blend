#include "sam/sam_ticks.h"
#include "zephyr/net_buf.h"
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app_blend, CONFIG_LOG_DEFAULT_LEVEL);

#include "zephyr/logging/log.h"
#include "zephyr/toolchain/gcc.h"
#include <zephyr/sys/printk.h>

#include "sam/buf/sam_bufpool.h"
#include "sam/core/sam_core.h"
#include "sam/radio/sam_radio.h"
#include "sam/radio/sam_sem.h"
#include "sam/sam_module.h"
#include "sam/sam_result.h"
#include "sam/modules/logging/log.h"
#include <sam/core/sam_timeslot.h>
#include <sam_blend/sam_blend.h>

#include <hal/nrf_radio.h>

/*--------------------------------------------------------------------------*/
// BLEnd SHARED STATE
// Computed once during init, read-only afterwards.
/*--------------------------------------------------------------------------*/
static sam_radioticks_t blend_rt;

/*--------------------------------------------------------------------------*/
// BLEnd DEDICATED WORKQUEUE
//
// A dedicated workqueue is used instead of the system workqueue because
// scan_work and tx_work block internally (sam_core_rx, sam_core_tx_wait).
// Blocking the system workqueue would stall unrelated Zephyr subsystems.
/*--------------------------------------------------------------------------*/
#define BLEND_WQ_STACK_SIZE 2048
#define BLEND_WQ_PRIORITY   7

K_THREAD_STACK_DEFINE(blend_wq_stack, BLEND_WQ_STACK_SIZE);
static struct k_work_q blend_wq;

/*--------------------------------------------------------------------------*/
// WORK ITEMS AND EPOCH TIMER
/*--------------------------------------------------------------------------*/
static struct k_work  scan_work;
static struct k_work  tx_work;
static struct k_timer epoch_timer;

/*--------------------------------------------------------------------------*/
// SCAN WORK
//
// Runs at the start of every epoch. Executes the BLEnd scan window, then
// immediately submits tx_work to begin beacon transmission.
/*--------------------------------------------------------------------------*/
static void scan_work_fn(struct k_work *w)
{
    sam_result_t res;

    sam_rx_sched_args_t sa = { .rx_timeout_us = SCAN_DURATION_US,
                               .rx_guard_us   = 50 };
    sam_rx_wait_args_t wa = {};

    res = sam_core_rx(sa, &wa);
    if (res == SAM_SUCCESS) {
        LOG_INF("RX len=%u", wa.rx_buf->len);
        LOG_HEXDUMP_INF(wa.rx_buf->data, wa.rx_buf->len, "RX data");
        sam_buf_unref(wa.rx_buf);
        wa.rx_buf = NULL;
    } else if (res == SAM_TIMEOUT) {
        LOG_INF("No beacon received in the SCAN");
    } else {
        LOG_INF("! Unexpected result for the SCAN: %s",
                sam_get_std_result_name(res));
    }

    k_work_submit_to_queue(&blend_wq, &tx_work);
}

/*--------------------------------------------------------------------------*/
// TX WORK
//
// Transmits BEACONS_PER_EPOCH beacons using the same pipelined pattern as
// the original main_thread: schedule beacon[i+1], wait for beacon[i], apply
// random slack, advance prev_op_id. Structured to match main.c exactly.
//
// On early exit (alloc/sched failure for beacon 0), the work function simply
// returns — the epoch timer will start the next epoch regardless.
//
// No sami_core_epoch_restart_* is called here. The k_timer owns the epoch
// boundary and fires unconditionally every EPOCH_DURATION_US.
/*--------------------------------------------------------------------------*/
static void tx_work_fn(struct k_work *w)
{
    sam_result_t res;
    struct net_buf *buf;
    int prev_op_id, next_op_id;

    SAM_MODULE_DATA_TYPE(core) *core_data =
        SAM_MODULE_DEFAULT_INSTANCE_GET(core)->data;

    /* ---- First beacon (index 0) ---------------------------------------- */
    res = sam_bufpool_alloc(&buf, K_NO_WAIT);
    if (!res || buf == NULL) {
        return; /* epoch abandoned; timer restarts next one */
    }
    net_buf_add_u8(buf, 0);
    sam_core_tx_enqueue(buf);
    sam_buf_unref(buf);

    sam_tx_sched_args_t tsa = {};
    tsa.tx_buf = buf;
    core_data->action_context.tx_delay = blend_rt;
    res = sam_core_tx_sched(tsa, &prev_op_id);
    if (!res) {
        LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
        return; /* epoch abandoned; timer restarts next one */
    }

    /* ---- Remaining beacons (index 1..BEACONS_PER_EPOCH-1) --------------- */
    for (int i = 1; i < BEACONS_PER_EPOCH; i++) {
        res = sam_bufpool_alloc(&buf, K_NO_WAIT);
        if (!res || buf == NULL) {
            continue;
        }
        net_buf_add_u8(buf, i);
        sam_core_tx_enqueue(buf);
        sam_buf_unref(buf);

        sam_tx_sched_args_t sa = {};
        sa.tx_buf = buf;
        core_data->action_context.tx_delay = blend_rt;

        res = sam_core_tx_sched(sa, &next_op_id);
        if (!res) {
            LOG_ERR("! sam_core_tx_sched returned %s",
                    sam_get_std_result_name(res));
            continue;
        }

        sam_core_tx_wait(prev_op_id);

        int random_slack_us = rand() % RANDOM_SLACK_US;
        sam_radioticks_t slack;
        sam_radio_us_to_rt(random_slack_us, &slack);
        int slack_in_slots = slack / core_data->context.slot_duration;
        sam_core_skip_slots(slack_in_slots);

        prev_op_id = next_op_id;
    }

    /* ---- Wait for the last beacon -------------------------------------- */
    sam_core_tx_wait(prev_op_id);

    /* ---- Log flush (same order as original) ---------------------------- */
    while (log_process()) {}
    sami_log_flush(SAM_MODULE_DEFAULT_INSTANCE_GET(logging), 0, NULL);

    /*
     * No sami_core_epoch_restart_sched/wait here.
     * The k_timer fires unconditionally every EPOCH_DURATION_US and submits
     * scan_work for the next epoch. tx_work simply exits and the workqueue
     * thread goes idle until then.
     */
}

/*--------------------------------------------------------------------------*/
// EPOCH TIMER EXPIRY  (ISR context — submit only, never block)
//
// k_work_submit_to_queue returns:
//   1  → work was freshly submitted (normal case)
//   0  → work was already pending in the queue (very unlikely overrun)
//  <0  → work is currently running (-EBUSY): previous epoch not yet done
/*--------------------------------------------------------------------------*/
static void epoch_timer_expiry(struct k_timer *timer)
{
    int ret = k_work_submit_to_queue(&blend_wq, &scan_work);
    if (ret != 1) {
        LOG_WRN("BLEnd epoch overrun (ret=%d): previous epoch still running", ret);
    }
}

/*--------------------------------------------------------------------------*/
// MAIN THREAD  (init only — exits after starting the timer)
//
// Mirrors the original main_thread startup sequence exactly (two 2-second
// sleeps, sam_module_init, radio disable). After setup it starts the epoch
// timer and exits; the blend_wq workqueue thread takes ownership from here.
/*--------------------------------------------------------------------------*/
void start_blend(void)
{
    sam_radio_us_to_rt(TIMESLOT_REQUEST_DISTANCE_US, &blend_rt);

    k_work_queue_init(&blend_wq);
    k_work_queue_start(&blend_wq, blend_wq_stack,
                       K_THREAD_STACK_SIZEOF(blend_wq_stack),
                       K_PRIO_PREEMPT(BLEND_WQ_PRIORITY), NULL);

    k_work_init(&scan_work, scan_work_fn);
    k_work_init(&tx_work,   tx_work_fn);

    k_timer_init(&epoch_timer, epoch_timer_expiry, NULL);
    k_timer_start(&epoch_timer, K_NO_WAIT, K_USEC(EPOCH_DURATION_US));
}

/*--------------------------------------------------------------------------*/