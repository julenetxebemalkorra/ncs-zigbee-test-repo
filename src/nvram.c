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

static struct zb_user_conf_t zb_user_conf; // zigbee user configuration

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

    LOG_INF("NVRAM initialized\n");

    return rc;
}

/**
 * @brief This function reads the first ID from non-volatile storage (NVRAM).
 * 
 * @param nvram_first_id A pointer to a buffer where the first ID will be stored.
 * @param nvram_first_id_size The size of the buffer pointed to by nvram_first_id.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
uint8_t read_nvram_first_id(uint8_t *nvram_first_id, size_t nvram_first_id_size)
{
    int rc =0;
    rc = nvs_read(&fs, ZB_NVRAM_CHECK_ID, nvram_first_id, nvram_first_id_size);
    return rc;
}

/**
 * @brief This function reads the Node Identifier (NI) from non-volatile storage (NVRAM).
 * 
 * @param ni A pointer to a buffer where the NI will be stored.
 * @param ni_size The size of the buffer pointed to by ni.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
uint8_t read_nvram_NI(uint8_t *ni, size_t ni_size)
{
    int rc = 0;
    
    // Returns: Number of bytes read. On success, it will be equal to the number of bytes requested to be read. 
    // When the return value is larger than the number of bytes requested to read this indicates not all bytes were read, 
    // and more data is available. On error, returns negative value of errno.h defined error codes.
    rc = nvs_read(&fs, ZB_NODE_IDENTIFIER, ni, ni_size);

    return rc;
}

/**
 * @brief This function reads the PAN_ID from non-volatile storage (NVRAM).
 * 
 * @param panid A pointer to a buffer where the PAN_ID will be stored.
 * @param panid_size The size of the buffer pointed to by panid.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
uint8_t read_nvram_PAN_ID(uint64_t *panid, size_t panid_size)
{
    int rc = 0;

    // Read the PAN_ID from NVRAM returns the number of bytes read
    rc = nvs_read(&fs, ZB_EXT_PANID, panid, panid_size); 
    return rc;
}

/**
 * @brief This function writes the PAN_ID to non-volatile storage (NVRAM).
 * 
 * @param panid A pointer to the buffer containing the PAN_ID to be written.
 * @param panid_size The size of the PAN_ID to be written.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
void write_nvram_first_id(uint64_t *firstid, size_t firstid_size)
{
    // Write the PAN_ID to NVRAM
    (void)nvs_write(&fs, ZB_NVRAM_CHECK_ID, firstid, firstid_size);
}

/**
 * @brief This function writes the Node Identifier (NI) to non-volatile storage (NVRAM).
 * 
 * @param ni A pointer to the buffer containing the NI to be written.
 * @param ni_size The size of the NI to be written.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
void write_nvram_NI(uint8_t *ni, size_t ni_size)
{
    // Write the Node Identifier to NVRAM
    (void)nvs_write(&fs, ZB_NODE_IDENTIFIER, ni, ni_size);
}

/**
 * @brief This function writes the PAN_ID to non-volatile storage (NVRAM).
 * 
 * @param panid A pointer to the buffer containing the PAN_ID to be written.
 * @param panid_size The size of the PAN_ID to be written.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
void write_nvram_PAN_ID(uint64_t *panid, size_t panid_size)
{
    // Write the PAN_ID to NVRAM
    (void)nvs_write(&fs, ZB_EXT_PANID, panid, panid_size);
}
