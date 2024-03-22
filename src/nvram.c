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

#include "global_defines.h"
#include "zigbee_configuration.h"
#include "Digi_At_commands.h"
#include "tcu_Uart.h"
#include "nvram.h"

// NVRAM file system instance for persisting application data
static struct nvs_fs fs;

// NVRAM partition where application data is stored in flash memory 
#define NVS_PARTITION		    storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define ZB_EXT_PANID 1
#define ZB_NODE_IDENTIFIER 2

static struct zb_user_conf_t zb_user_conf; // zigbee user configuration

// Define default extended PAN ID for the network
uint64_t extended_pan_id = 0x0000000000009999;

LOG_MODULE_REGISTER(nvram_app, LOG_LEVEL_DBG);

/**
 * @brief This function initializes the NVRAM file system.
 * 
 * @return The function returns a uint8_t value. (You should describe what this return value represents)
 */
uint8_t init_nvram(void)
{
    int rc;
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
		return 0;
	}
	fs.offset = NVS_PARTITION_OFFSET;
    // Get the page info
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return rc;
	}

    // Set the sector size and count
	fs.sector_size = info.size;
	fs.sector_count = 3U;

    // Mount the NVRAM file system
	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return rc;
	}

    // initialize the NVRAM with default values if thewy are not present
    //sprintf(zb_user_conf.at_ni, "NORDIC_JULEN_NVRAM_TEST"); // Node identifier string
    zb_user_conf.at_ni[0] = 'N';
    zb_user_conf.at_ni[1] = 'O';
    zb_user_conf.at_ni[2] = 'R';
    zb_user_conf.at_ni[3] = 'D';
    zb_user_conf.at_ni[4] = 'I';
    zb_user_conf.at_ni[5] = 'K';
    zb_user_conf.at_ni[6] = '_';
    zb_user_conf.at_ni[7] = 'D';
    zb_user_conf.at_ni[8] = 'K';
    zb_user_conf.at_ni[9] = '3';
    zb_user_conf.at_ni[10] = '1';
    zb_user_conf.at_ni[11] = 0;

    LOG_INF("NVRAM initialized\n");

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
    int rc;
    
    // Read the Node Identifier from NVRAM returns the number of bytes read
    rc = nvs_read(&fs, ZB_NODE_IDENTIFIER, ni, ni_size);

    if (rc > 0) 
    {
        // Node Identifier is set, so log it
        LOG_INF("Id: %d, Node Identifier: %s\n", ZB_NODE_IDENTIFIER, ni);
    } 
    else 
    {
        // Node Identifier not found in NVRAM, write the default Node Identifier to NVRAM
        LOG_WRN("No Node Identifier found, adding %s at id %d\n", zb_user_conf.at_ni, ZB_NODE_IDENTIFIER);
        (void)nvs_write(&fs, ZB_NODE_IDENTIFIER, &zb_user_conf.at_ni, strlen(zb_user_conf.at_ni)+1);
        // Copy the default Node Identifier to the buffer pointed to by ni
        memcpy(ni, &zb_user_conf.at_ni, sizeof(zb_user_conf.at_ni));
    }

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
    int rc;

    // Read the PAN_ID from NVRAM returns the number of bytes read
    rc = nvs_read(&fs, ZB_EXT_PANID, panid, panid_size); 

    // PAN_ID found in NVRAM
    if (rc == panid_size) 
    {        
        LOG_INF("Memmory Id: %d. ", ZB_EXT_PANID);
        LOG_HEXDUMP_DBG(panid, panid_size, "Extended PAN ID: ");
    } 
    // PAN_ID not found in NVRAM, write the default PAN_ID to NVRAM
    else 
    {
        LOG_WRN("No PAN_ID found, adding the default one at id %d\n", ZB_EXT_PANID);
        (void)nvs_write(&fs, ZB_EXT_PANID, &extended_pan_id, sizeof(extended_pan_id));
        // Copy the default PAN_ID to the buffer pointed to by panid
        memcpy(panid, &extended_pan_id, sizeof(extended_pan_id));
    }
    return rc;
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
