#ifndef SAM_BLEND_H
#define SAM_BLEND_H
/*--------------------------------------------------------------------------*/
// Configurable parameters for the BLEnd protocol
#define EPOCH_DURATION_US (3983000) // E
#define TIMESLOT_LENGTH_US (1800)   // b
#define RANDOM_SLACK_US (10000) // s
#define TIMESLOT_REQUEST_DISTANCE_US (83000)  // A
#define SCAN_DURATION_US (TIMESLOT_REQUEST_DISTANCE_US + TIMESLOT_LENGTH_US + RANDOM_SLACK_US)    // L
#define BEACONS_PER_EPOCH_B (EPOCH_DURATION_US/(TIMESLOT_REQUEST_DISTANCE_US)-1)   // nb
#define BEACONS_PER_EPOCH_U (EPOCH_DURATION_US/(2*TIMESLOT_REQUEST_DISTANCE_US)-1)   // nb
/*--------------------------------------------------------------------------*/
/**
 * @brief Start the BLEnd protocol
 *
 * @param cb Callback function to call when another node is detected
 *
 */
void sam_start_blend(void (*cb)(struct net_buf *rx_buf));
/*--------------------------------------------------------------------------*/
#endif