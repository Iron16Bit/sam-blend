#ifndef SAM_BLEND_H
#define SAM_BLEND_H
/*--------------------------------------------------------------------------*/
#include "sam/core/sam_timeslot.h"
#define TIMESLOT_LENGTH_US (3500)   // b
/*--------------------------------------------------------------------------*/
// Configurable parameters for the BLEnd protocol
#define EPOCH_DURATION_US (2000000) // E
#define SCAN_DURATION_US (30000)    // L
#define RANDOM_SLACK_US (10000) // s
#define TIMESLOT_REQUEST_DISTANCE_US (SCAN_DURATION_US - TIMESLOT_LENGTH_US - RANDOM_SLACK_US)  // A
/*--------------------------------------------------------------------------*/
#define BEACONS_PER_EPOCH ((uint8_t)(EPOCH_DURATION_US/(2*TIMESLOT_REQUEST_DISTANCE_US)))   // nb
/*--------------------------------------------------------------------------*/
#define BLEND_PROTOCOL_ID_LEN 5
#define BLEND_PAYLOAD_LEN     22
#define BLEND_PACKET_LEN 31

struct __packed blend_packet {
    uint8_t  protocol_id[BLEND_PROTOCOL_ID_LEN];
    uint16_t node_id;
    uint16_t epoch_offset_ticks;                // B-BLEnd only
    uint8_t  payload[BLEND_PAYLOAD_LEN];
};

BUILD_ASSERT(sizeof(struct blend_packet) == 31);
/*--------------------------------------------------------------------------*/
/**
 * @brief Start the BLEnd protocol
 *
 * @param cb Callback function to call when another node is detected
 *
 */
void sam_start_blend(void (*cb)(struct net_buf *rx_buf));
/**
 * @brief Stop the BLEnd protocol
 *
 */
void sam_stop_blend(void);
/*--------------------------------------------------------------------------*/
#endif