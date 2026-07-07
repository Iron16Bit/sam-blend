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

#include <sam_nrf52833/sam_radio.h>
#include <sam_nrf52833/sam_nrf52833_util.h>

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

    SAM_MODULE_DATA_TYPE(core) *core_data =
        SAM_MODULE_DEFAULT_INSTANCE_GET(core)->data;

    uint8_t channel = 37;

	while(1) {
        /* Start epoch timer */
        int64_t epoch_end = k_uptime_get() + EPOCH_DURATION_US / 1000;

        // First SCAN
        struct net_buf *buf;
        sam_rx_sched_args_t sa = {.rx_timeout_us = 0,
                                  .rx_guard_us = 0,
                                  .channel = channel + 200};
                                  
        if (++channel > 39) {
            channel = 37;
        }
        sam_rx_wait_args_t wa = {};

        sam_result_t res;

        res = sam_core_rx(sa, &wa);
        if (res == SAM_SUCCESS) {
            sam_buf_unref(wa.rx_buf);
            wa.rx_buf = NULL;
            (*cb)();
        } else if (res == SAM_TIMEOUT) {
            LOG_INF("No beacon received in the SCAN");
        } else {
            LOG_INF("! Unexpected result for the SCAN: %s", sam_get_std_result_name(res));
        }

        for (int i=0; i < BEACONS_PER_EPOCH; i++) {
            int last_op_id;
            // Advertise on channels 37, 38, 39
            res = sam_bufpool_alloc(&buf, K_NO_WAIT);
            if (!res || buf == NULL) {
                k_oops();
            }
            net_buf_add_u8(buf, i); //TODO add BLEnd packet structure
            sam_core_tx_enqueue(buf);
            sam_buf_unref(buf); // we can unref (the core has now ownership)

            sam_tx_sched_args_t tsa = {};
            tsa.tx_buf = buf;
            tsa.channel = ADVERTISE;

            if (i != 0) {
                // Random slack
                int random_slack_us = (rand() % RANDOM_SLACK_US);
                k_sleep(K_USEC(random_slack_us));
            }

            core_data->action_context.tx_delay = 0;
            res = sam_core_tx_sched(tsa, &last_op_id);
            if (!res) {
                LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
                k_oops();
            }

            res = sam_core_tx_wait(last_op_id);
            k_sleep(K_USEC(TIMESLOT_REQUEST_DISTANCE_US));
            if (res != SAM_SUCCESS) {
                LOG_ERR("Failed TX");
            }
        }

        /* Wait until the epoch duration has elapsed */
        int64_t remaining_ms = epoch_end - k_uptime_get();
        if (remaining_ms > 0) {
            k_sleep(K_MSEC(remaining_ms));
        }

        while (log_process()) {}
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

void sam_stop_blend(void) {
    k_thread_abort(&sam_blend_thread_data);
}