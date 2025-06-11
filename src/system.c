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
//#include <zephyr/drivers/watchdog.h>
#include <zephyr/task_wdt/task_wdt.h>
        #include <zephyr/dfu/mcuboot.h>
        #include <zephyr/storage/flash_map.h>
#include "app_version.h"
#include <zboss_api.h>
#include "system.h"

LOG_MODULE_REGISTER(system, LOG_LEVEL_DBG);

static const struct device *const hw_wdt_dev = DEVICE_DT_GET_OR_NULL(WDT_NODE);
static int task_wdt_id = -1;

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
