/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of the UART0, which is connected to the TCU.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>



#include "zigbee_configuration.h"
#include "Digi_At_commands.h"
#include "tcu_Uart.h"
#include "nvram.h"

// NVRAM file system instance for persisting application data
static struct nvs_fs fs;

LOG_MODULE_REGISTER(nvram_app, LOG_LEVEL_DBG);

/**
 * @brief This function initializes the NVRAM file system.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
uint8_t init_nvram(void)
{
    int rc = 0;
    struct flash_pages_info info;

    /* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at NVS_PARTITION_OFFSET
	 */
	fs.flash_device = NVS_PARTITION_DEVICE;
    // Check if the flash device is ready
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return -1;
	}
	fs.offset = NVS_PARTITION_OFFSET;
    // Get the size and start offset of flash page at certain flash offset.
    // returns 0 on success, -EINVAL if page of the offset doesn’t exist.
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc == 0) {
        LOG_INF("Page info: size %d, start offset %d\n", info.size, info.start_offset);
    } 
    else if (rc == EINVAL) // Invalid argument
    {
		LOG_ERR("Unable to get page info. EINVAL \n");
		return rc;
	}
    else
    {
        LOG_INF("Unknown error getting page info; %d\n", rc);
        return rc;
    }

    // Set the sector size and count
	fs.sector_size = info.size;
	fs.sector_count = 3U;

    // Mount the NVRAM file system
    // 0 – Success
    //-ERRNO – errno code if error
	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed: ERRNO %d\n", rc);
		return rc;
	}

    LOG_INF("NVRAM initialized successfully\n");

    return rc;
}

/**
 * @brief This function reads data from non-volatile storage (NVRAM).
 * 
 * @param id The ID of the data to be read.
 * @param data A pointer to a buffer where the data will be stored.
 * @param len The size of the data to be read.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */

uint8_t read_nvram(uint16_t id, uint8_t *data, size_t len)
{
    int rc =0;
    rc = nvs_read(&fs, id, data, len);
    return rc;
}

/**
 * @brief This function writes the PAN_ID to non-volatile storage (NVRAM).
 * 
 * @param id The ID of the data to be written.
 * @param panid A pointer to the buffer containing the PAN_ID to be written.
 * @param panid_size The size of the PAN_ID to be written.
 * 
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
void write_nvram(uint16_t id, uint8_t *data, size_t len)
{
    // Write the PAN_ID to NVRAM
    (void)nvs_write(&fs, id, data, len);
}

size_t calc_free_space(void)
{
    size_t free_space = nvs_calc_free_space(&fs);
    return free_space;
}

