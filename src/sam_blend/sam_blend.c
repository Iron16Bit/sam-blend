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

void sam_blend_loop(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    void (*cb)(void) = (void (*)(void))arg1;

    uint8_t current_op = 0;

    SAM_MODULE_DATA_TYPE(core) *core_data =
        SAM_MODULE_DEFAULT_INSTANCE_GET(core)->data;

    sam_radioticks_t rt;
    sam_radio_us_to_rt(TIMESLOT_REQUEST_DISTANCE_US + 4000, &rt);

	while(1) {
        current_op = 0;

        // First SCAN
        struct net_buf *buf;
        sam_rx_sched_args_t sa = {.rx_timeout_us = SCAN_DURATION_US,
                                  .rx_guard_us = 50};
        sam_rx_wait_args_t wa = {};

        sam_result_t res;

        res = sam_core_rx(sa, &wa);
        if (res == SAM_SUCCESS) {
            LOG_INF("RX len=%u", wa.rx_buf->len);
            LOG_HEXDUMP_INF(wa.rx_buf->data, wa.rx_buf->len, "RX data");
            sam_buf_unref(wa.rx_buf);
            wa.rx_buf = NULL;
            (*cb)();
        } else if (res == SAM_TIMEOUT) {
            LOG_INF("No beacon received in the SCAN");
        } else {
            LOG_INF("! Unexpected result for the SCAN: %s", sam_get_std_result_name(res));
        }

        int prev_op_id, next_op_id;

        // Transmit beacons
        res = sam_bufpool_alloc(&buf, K_NO_WAIT);
        if (!res || buf == NULL) {
            continue;
        }
        net_buf_add_u8(buf, 0);
        sam_core_tx_enqueue(buf);
        sam_buf_unref(buf); // we can unref (the core has now ownership)

        sam_tx_sched_args_t tsa = {};
        tsa.tx_buf = buf;
        core_data->action_context.tx_delay = rt;
        res = sam_core_tx_sched(tsa, &prev_op_id);
        if (!res) {
            LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
            continue;
        }

        for(int i=1; i < BEACONS_PER_EPOCH; i++) {
            // Transmit beacons
            res = sam_bufpool_alloc(&buf, K_NO_WAIT);
            if (!res || buf == NULL) {
                continue;
            }
            net_buf_add_u8(buf, i);
            sam_core_tx_enqueue(buf);
            sam_buf_unref(buf); // we can unref (the core has now ownership)

            sam_tx_sched_args_t sa = {};
            sa.tx_buf = buf;
            core_data->action_context.tx_delay = rt;

            res = sam_core_tx_sched(sa, &next_op_id);
            if (!res) {
                LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
                continue;
            }

            sam_core_tx_wait(prev_op_id);

            int random_slack_us = (rand() % RANDOM_SLACK_US);
            sam_radioticks_t slack;
            sam_radio_us_to_rt(random_slack_us, &slack);
            int slack_in_slots = slack / core_data->context.slot_duration;
            sam_core_skip_slots(slack_in_slots);

            prev_op_id = next_op_id;
        }
        sam_core_tx_wait(prev_op_id);
        // Wait for the end of the epoch
        while (log_process()) {}

        // Reset the epoch
        sami_log_flush(SAM_MODULE_DEFAULT_INSTANCE_GET(logging), 0, NULL);

        sami_core_epoch_restart_sched(SAM_MODULE_DEFAULT_INSTANCE_GET(core), EPOCH_DURATION_US);
		sami_core_epoch_restart_wait(SAM_MODULE_DEFAULT_INSTANCE_GET(core));
	}
}

#define SAM_OP_THREAD_STACK_SIZE 1024
#define SAM_OP_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(sam_blend_stack,
                      SAM_OP_THREAD_STACK_SIZE);

static struct k_thread sam_blend_thread_data;

void sam_start_blend(void (*cb)(void))
{
    k_thread_create(
        &sam_blend_thread_data,
        sam_blend_stack,
        K_THREAD_STACK_SIZEOF(sam_blend_stack),
        sam_blend_loop,
        cb,
        NULL,
        NULL,
        SAM_OP_THREAD_PRIORITY,
        0,
        K_NO_WAIT);
}