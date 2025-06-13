/*
 * Copyright (c) 2025 IED
 *
 */

/** @file
 *
 * @brief Definition of auxiliar functions used to configure and manage peripherals, and to display system information at start up.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_timer.h>
//#include <zephyr/drivers/watchdog.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#include "app_version.h"
#include <zboss_api.h>
#include "system.h"
#include "Tcu_Uart.h"
#include "zigbee_aps.h"

LOG_MODULE_REGISTER(system, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static volatile uint16_t debug_led_ms_x10 = 0;
static const struct device *const hw_wdt_dev = DEVICE_DT_GET_OR_NULL(WDT_NODE);
static int task_wdt_id = -1;
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(1); // Reference to the TIMER1 instance

//------------------------------------------------------------------------------
/**@brief Initialization of the GPIO pins.
 *        Currently, only one pin is initialized. Configured as output to drive a led.
 * @retval -1 Error
 * @retval 0 OK
 */
int8_t gpio_init(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

    return 0;
}


/// @brief Initialization of the TIMER1 peripheral using the nrfx driver
/// 
/// @param None

void timer1_init(void)
{
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000); //1MHz
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	int err = nrfx_timer_init(&my_timer, &timer_config, timer1_event_handler);
	if (err != NRFX_SUCCESS) {
		LOG_WRN("Error initializing timer: %x\n", err);
	}

	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);

	timer1_repeated_timer_start(100); //100 us
}

/// @brief Function for initializing the task watchdog
/// 
/// The task watchdog task is initialized and a channel is created for the main loop thread.
/// HW watchdog set as fallback.
/// 
///  @retval -1 Error
///  @retval 0 OK
int8_t watchdog_init(void)
{
	int ret;

    if (!device_is_ready(hw_wdt_dev)) {
		LOG_ERR("HW WDT not available.\n");
        return -1;
	}

    ret = task_wdt_init(hw_wdt_dev);
	if (ret != 0) {
		LOG_ERR("Task watchdog init failure: %d\n", ret);
        return -2;
	}

    // Register this thread
    k_tid_t my_tid = k_current_get();

    LOG_INF("Registering task watchdog for thread: %p", my_tid);
    LOG_INF("Thread name: %s", k_thread_name_get(my_tid));
 
	task_wdt_id = task_wdt_add(2000U, task_wdt_callback , my_tid); //2 seconds timeout
    if (task_wdt_id < 0) {
		LOG_ERR("task_wdt_add failed: %d", task_wdt_id);
		return -3;
	}

	LOG_INF("Task WDT initialized with channel %d", task_wdt_id);

    return 0;
}

//------------------------------------------------------------------------------
/**@brief This function toggles and output pin at 1Hz. That output pin is connected to a LED
 *
 * @note Executed the in main loop
 *
 */
void diagnostic_toogle_pin(void)
{
    if(debug_led_ms_x10 >= 10000)
    {
        debug_led_ms_x10 = 0;
        gpio_pin_toggle_dt(&led);
    }
}

/// @brief Callback function to be executed when the watchdog of the main thread times out.
///
/// It just resets the device. There is not attempt to resolve the issue and continue operation.
///
/// @param channel_id :Id of the whatdog channel 
/// @param user_data ; Pointer to strucure with thread information
void task_wdt_callback(int channel_id, void *user_data)
{
    LOG_WRN("Task watchdog channel %d callback, thread: %s\n", channel_id, k_thread_name_get((k_tid_t)user_data));
    LOG_WRN("Resetting device...\n");
    sys_reboot(SYS_REBOOT_COLD);
}

/// @brief Feed the main loop thread watchdog every 1000 ms
/// @param None
void periodic_feed_of_main_loop_watchdog(void)
{
    static uint64_t time_last_ms_wdt = 0;
    uint64_t time_now_ms = k_uptime_get();
    
    if ((uint64_t)( time_now_ms - time_last_ms_wdt ) > 1000)
    {
        int err = task_wdt_feed(task_wdt_id); // Feed the watchdog
        if (err != 0) {
            LOG_ERR("task_wdt_feed failed: %d", err);
        }
        time_last_ms_wdt = time_now_ms;
    }
}

/**
 * @brief Interrupt handler for TIMER1
 * NOTE: This callback is triggered by an interrupt. Many drivers or modules in Zephyr can not be accessed directly from interrupts, 
 * and if you need to access one of these from the timer callback it is necessary to use something like a k_work item to move execution out of the interrupt
 * context.
 */
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
            if(debug_led_ms_x10 < 10000) debug_led_ms_x10++;
            tcu_uart_timers_10kHz();
            check_scheduling_cb_timeout();
			break;
		default:
			break;
	}
}

/**
 * @brief Schedule repeated callbacks from TIMER1.
 * @param timeout_us Period in microseconds.
 */
int miFuncion(int param1, float param2);

void timer1_repeated_timer_start(uint32_t timeout_us)
{
	nrfx_timer_enable(&my_timer);

	nrfx_timer_extended_compare(&my_timer, NRF_TIMER_CC_CHANNEL0, timeout_us, 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

/// @brief Display system information.
///
/// This function is called at startup. It displays system information (firmware version,
/// ZBOSS version, reason of last reset...)
///
/// @param None
void display_system_information(void)
{
    const zb_char_t ZB_IAR_CODE *zb_version;
    uint32_t reset_cause;
    int result;

    // Display the last reset cause
    result = hwinfo_get_reset_cause(&reset_cause);
    if (result == 0)
	{
        LOG_ERR("RESET:");
        if (reset_cause & RESET_PIN)              LOG_WRN("Reset cause: RESET_PIN");
        if (reset_cause & RESET_SOFTWARE)         LOG_WRN("Reset cause: RESET_SOFTWARE");
        if (reset_cause & RESET_BROWNOUT)         LOG_WRN("Reset cause: RESET_BROWNOUT");
        if (reset_cause & RESET_POR)              LOG_WRN("Reset cause: RESET_POR");
        if (reset_cause & RESET_WATCHDOG)         LOG_WRN("Reset cause: RESET_WATCHDOG");
        if (reset_cause & RESET_DEBUG)            LOG_WRN("Reset cause: RESET_DEBUG");
        if (reset_cause & RESET_SECURITY)         LOG_WRN("Reset cause: RESET_SECURITY");
        if (reset_cause & RESET_LOW_POWER_WAKE)   LOG_WRN("Reset cause: RESET_LOW_POWER_WAKE");
        if (reset_cause & RESET_CPU_LOCKUP)       LOG_WRN("Reset cause: RESET_CPU_LOCKUP");
        if (reset_cause & RESET_PARITY)           LOG_WRN("Reset cause: RESET_PARITY");
        if (reset_cause & RESET_PLL)              LOG_WRN("Reset cause: RESET_PLL");
        if (reset_cause & RESET_CLOCK)            LOG_WRN("Reset cause: RESET_CLOCK");
        if (reset_cause & RESET_HARDWARE)         LOG_WRN("Reset cause: RESET_HARDWARE");
        if (reset_cause & RESET_USER)             LOG_WRN("Reset cause: RESET_USER");
        if (reset_cause & RESET_TEMPERATURE)      LOG_WRN("Reset cause: RESET_TEMPERATURE");

        if (reset_cause == 0) {
            LOG_WRN("No known reset causes detected");
        }
    }
    else
	{
		LOG_ERR("\n\n It was not possible to read the last reset causes \n\n");
    }

    hwinfo_clear_reset_cause(); // Clear the reset cause flags. In that way we will see only the cause of last reset

    // Display the APP firmware version
    LOG_INF("APP firmware version is %u.%u,with patch level %u", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL);
    // Display the ZBOSS version
    zb_version = zb_get_version();
    LOG_INF("ZBOSS Version: %s\n", zb_version);
}


/// @brief Display boot status information
///
/// This function is called at startup. It displays the boot status (if there is a swap pending,
/// or if the image running is only for test and needs confirmation...)
///
/// @param  None
void display_boot_status(void)
{
    int swap_type = mcuboot_swap_type();
    switch (swap_type) {
        case BOOT_SWAP_TYPE_NONE:
            LOG_INF("No swap pending.\n");
            break;
        case BOOT_SWAP_TYPE_TEST:
            LOG_INF("New image in slot1 is scheduled for test.\n");
            break;
        case BOOT_SWAP_TYPE_PERM:
            LOG_INF("New image will be made permanent.\n");
            break;
        case BOOT_SWAP_TYPE_REVERT:
            LOG_INF("Reverting to previous image.\n");
            break;
        default:
            LOG_INF("Unknown swap type: %d\n", swap_type);
            break;
    }

    struct mcuboot_img_header header;
    size_t header_size = sizeof(header);

    int ret = boot_read_bank_header(FIXED_PARTITION_ID(slot0_partition), &header, header_size);
    if (ret == 0) {
        if (header.mcuboot_version == 1) {
            LOG_INF("MCUBoot image header (v1):");
            LOG_INF("  Version: %d.%d.%d+%d",
                    header.h.v1.sem_ver.major,
                    header.h.v1.sem_ver.minor,
                    header.h.v1.sem_ver.revision,
                    header.h.v1.sem_ver.build_num);
            LOG_INF("  Image size:  %u bytes", header.h.v1.image_size);
        } else {
            LOG_WRN("Unsupported mcuboot_version: %u", header.mcuboot_version);
        }
    } else {
        LOG_ERR("Failed to read MCUBoot header, error: %d", ret);
    }

    // Optionally log trailer offset
    ssize_t trailer_offset = boot_get_area_trailer_status_offset(0);
    if (trailer_offset >= 0) {
        LOG_INF("Trailer status offset: 0x%08zx", trailer_offset);
    } else {
        LOG_ERR("Failed to get trailer offset");
    }
}

/// @brief Confirms the current firmware image.
///
/// This function is called at startup. It checks whether the current image is already confirmed or is in test mode.
/// If the images in slot 0 and slot 1 were recently swapped, it confirms the new image.
/// @param  None
void confirm_image(void)
{
	if (!boot_is_img_confirmed()) {
        LOG_WRN("Image is in test mode");
		int ret = boot_write_img_confirmed();

		if (ret) {
			LOG_ERR("Couldn't confirm image. Error code: %d", ret);
		} else {
			LOG_WRN("Image confirmed");
		}
	}
}
