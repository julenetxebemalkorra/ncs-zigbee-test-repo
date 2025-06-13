/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

int8_t watchdog_init(void);
void task_wdt_callback(int channel_id, void *user_data);
void periodic_feed_of_main_loop_watchdog(void);
void display_system_information(void);
void display_boot_status(void);
void confirm_image(void);

#endif /* SYSTEM_H_ */
