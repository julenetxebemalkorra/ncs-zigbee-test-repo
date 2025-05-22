/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef ZIGBEE_BDB_H_
#define ZIGBEE_BDB_H_

#define MAX_TIME_NO_FRAMES_FROM_COORDINATOR  300000     // 5 minutes = 300000 ms
#define TIME_INTERVAL_BETWEEN_IEEE_ADD_REQ_TO_ZC 90000  // 90 seconds = 90000 ms
#define MAX_ATTEMPS_DETECT_COORDINATOR  3

/* Function prototypes (used only internally)                                 */
bool request_coordinator_ieee_address(void);

/* Function prototypes (used externally)                                      */
void zigbee_bdb_init(void);
void zigbee_bdb_coordinator_activity_detected(void);
void zigbee_bdb_network_watchdog(void);

#endif /* ZIGBEE_BDB_H_ */







