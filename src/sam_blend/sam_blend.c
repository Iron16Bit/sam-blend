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
#include <zephyr/sys/byteorder.h>

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

static const uint8_t blend_protocol_id[5] = { 'B', 'L', 'E', 'N', 'D' };

static inline uint16_t blend_node_id(void)
{
    return (uint16_t)(
        ((NRF_FICR->DEVICEID[0] & 0xFF)) |
        ((NRF_FICR->DEVICEID[1] & 0xFF) << 8));
}

int64_t epoch_end;
uint16_t next_epoch_start;
bool schedule_precise_beacon;

void create_blend_packet(struct net_buf *buf, uint8_t payload[22]) {
    net_buf_add_u8(buf, 'B');
    net_buf_add_u8(buf, 'L');
    net_buf_add_u8(buf, 'E');
    net_buf_add_u8(buf, 'N');
    net_buf_add_u8(buf, 'D');
    net_buf_add_u8(buf, NRF_FICR->DEVICEID[0]);
    net_buf_add_u8(buf, NRF_FICR->DEVICEID[1]);

    // Get time until epoch end and store it
    uint16_t remaining_ms = (uint16_t)(epoch_end - k_uptime_get());
    net_buf_add_u8(buf, remaining_ms & 0xFF);
    net_buf_add_u8(buf, remaining_ms >> 8);

    for (int i=0; i<22; i++)
        net_buf_add_u8(buf, payload[i]);
}

void sam_blend_loop(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    void (*cb)(struct net_buf *rx_buf) = (void (*)(struct net_buf *))arg1;
    
    SAM_MODULE_DATA_TYPE(core) *core_data =
    SAM_MODULE_DEFAULT_INSTANCE_GET(core)->data;
    
    uint8_t channel = 37;
    core_data->action_context.tx_delay = 0;

    LOG_INF("Starting BLEnd on node %u", blend_node_id());

	while(1) {
        schedule_precise_beacon = false;
        /* Start epoch timer */
        epoch_end = k_uptime_get() + EPOCH_DURATION_US / 1000;t();

        // First SCAN
        struct net_buf *buf;
        sam_rx_sched_args_t sa = {.rx_timeout_us = SCAN_DURATION_US,
                                  .rx_guard_us = 0,
                                  .channel = channel + 200};
                                  
        if (++channel > 39) {
            channel = 37;
        }
        sam_rx_wait_args_t wa = {};

        sam_result_t res;

        res = sam_core_rx(sa, &wa);
        if (res == SAM_SUCCESS) {
            // Check if it is a BLEND packet

            if (wa.rx_buf->len < 5) {
                LOG_ERR("Received packet too short");
                sam_buf_unref(wa.rx_buf);
                wa.rx_buf = NULL;
            } else if (memcmp(wa.rx_buf->data, blend_protocol_id, 5) != 0) {
                LOG_ERR("Ignoring non-BLEnd packet");
                sam_buf_unref(wa.rx_buf);
                wa.rx_buf = NULL;
            } else {
                /* Remove the protocol identifier before passing the packet on */
                net_buf_pull_mem(wa.rx_buf, 5);

                if (BLEND_VERSION == bblend) {
                    // Retrieve neighbor's next epoch start
                    next_epoch_start = sys_get_le16(wa.rx_buf->data);
                    schedule_precise_beacon;
                }

                (*cb)(wa.rx_buf);
                sam_buf_unref(wa.rx_buf);
                wa.rx_buf = NULL;
            }
        } else if (res == SAM_TIMEOUT) {
            LOG_INF("No beacon received in the SCAN");
        } else {
            LOG_INF("! Unexpected result for the SCAN: %s", sam_get_std_result_name(res));
        }

        if (!schedule_precise_beacon){
            for (int i=0; i < (BLEND_VERSION == ublend ? BEACONS_PER_EPOCH_U : BEACONS_PER_EPOCH_B); i++) {
                int last_op_id;
                // Advertise on channels 37, 38, 39
                res = sam_bufpool_alloc(&buf, K_NO_WAIT);
                if (!res || buf == NULL) {
                    k_oops();
                }

                // Data payload of BLEnd packet is empty at the moment. Modify it as needed
                uint8_t payload[22];
                memset(&payload, 0, 22);
                create_blend_packet(buf, payload);

                sam_core_tx_enqueue(buf);

                sam_tx_sched_args_t tsa = {};
                tsa.tx_buf = buf;
                tsa.channel = ADVERTISE;

                int random_slack_us = 0;
                if (i != 0) {
                    random_slack_us = (rand() % RANDOM_SLACK_US);
                    // Random slack
                    k_sleep(K_USEC(random_slack_us));
                }
                res = sam_core_tx_sched(tsa, &last_op_id);
                if (!res) {
                    LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
                    k_oops();
                }

                res = sam_core_tx_wait(last_op_id);
                sam_buf_unref(buf); // we can unref (the core has now ownership)
                if (res != SAM_SUCCESS) {
                    LOG_ERR("Failed TX");
                }
                k_sleep(K_USEC(TIMESLOT_REQUEST_DISTANCE_US - TIMESLOT_LENGTH_US - random_slack_us));
            }

            while (log_process()) {}

            /* Wait until the epoch duration has elapsed */
            int64_t remaining_ms = epoch_end - k_uptime_get();
            if (remaining_ms > 0) {
                k_sleep(K_MSEC(remaining_ms));
            }

        } else {
            // Send a single beacon exactly when the neighbor's listen insterval starts
            int op_id;
            // Advertise on channels 37, 38, 39
            res = sam_bufpool_alloc(&buf, K_NO_WAIT);
            if (!res || buf == NULL) {
                k_oops();
            }

            // Data payload of BLEnd packet is empty at the moment. Modify it as needed
            uint8_t payload[22];
            memset(&payload, 0, 22);
            create_blend_packet(buf, payload);

            sam_core_tx_enqueue(buf);

            sam_tx_sched_args_t tsa = {};
            tsa.tx_buf = buf;
            tsa.channel = ADVERTISE;

            k_sleep(K_MSEC(next_epoch_start));

            res = sam_core_tx_sched(tsa, &op_id);
            if (!res) {
                LOG_ERR("! sam_core_tx_sched returned %s", sam_get_std_result_name(res));
                k_oops();
            }

            res = sam_core_tx_wait(op_id);
            sam_buf_unref(buf); // we can unref (the core has now ownership)
            if (res != SAM_SUCCESS) {
                LOG_ERR("Failed TX");
            }
            while (log_process()) {}
        }
	}
}

#define SAM_OP_THREAD_STACK_SIZE 1024
#define SAM_OP_THREAD_PRIORITY   5

K_THREAD_STACK_DEFINE(sam_blend_stack,
                      SAM_OP_THREAD_STACK_SIZE);

static struct k_thread sam_blend_thread_data;

void sam_start_blend(void (*cb)(struct net_buf *rx_buf))
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