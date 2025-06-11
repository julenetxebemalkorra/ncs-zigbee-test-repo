/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

int8_t watchdog_init(void);
void task_wdt_callback(int channel_id, void *user_data);
void periodic_feed_of_main_loop_watchdog(void);

#endif /* SYSTEM_H_ */
