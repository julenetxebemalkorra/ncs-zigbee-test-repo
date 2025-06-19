/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <nrfx_timer.h>

#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

#define LED0_NODE DT_ALIAS(led0)                //The devicetree node identifier for the "led0" alias.
#define TIMER1_FREQUENCY_HZ 1000000             // 1MHz timer frequency
#define TIMER1_PERIOD_US 100                    // 100 microseconds period for the timer
#define MAIN_LOOP_WDT_TIMEOUT_MS 2000U          // 2 seconds timeout for the main loop watchdog
#define DEBUG_LED_TOGGLE_INTERVAL 10000         // 10000 x 0.1ms (10us) = 1 second for the debug LED toggle interval
#define MAIN_LOOP_WDT_FEED_INTERVAL_MS 1000     // 1000 milliseconds (1 second) for feeding the main loop watchdog

typedef enum {
    SYSTEM_RET_OK = 0,
    SYSTEM_RET_ERR = -1,
    SYSTEM_WDT_RET_ERR = -2,
    SYSTEM_TASK_WDT_ADD_RET_ERR = -3,
} system_ret_t;

int8_t gpio_init(void);
void timer1_init(void);
int8_t watchdog_init(void);
void diagnostic_toogle_pin(void);
void task_wdt_callback(int channel_id, void *user_data);
void periodic_feed_of_main_loop_watchdog(void);
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context);
void timer1_repeated_timer_start(uint32_t timeout_us);
void display_system_information(void);
void display_boot_status(void);
void confirm_image(void);

#endif /* SYSTEM_H_ */
