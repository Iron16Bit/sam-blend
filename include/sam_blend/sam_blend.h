#ifndef SAM_BLEND_H
#define SAM_BLEND_H
/*--------------------------------------------------------------------------*/
#define BEACONS_PER_EPOCH (61)   // nb
#define EPOCH_DURATION_US (2000000) // E
#define TIMESLOT_LENGTH_US (3200)   // b
#define SCAN_DURATION_US (30000)    // L
#define RANDOM_SLACK_US (10000) // s
#define TIMESLOT_REQUEST_DISTANCE_US (SCAN_DURATION_US - TIMESLOT_LENGTH_US - RANDOM_SLACK_US)  // A
/*--------------------------------------------------------------------------*/
void start_blend(void);
/*--------------------------------------------------------------------------*/
#endif