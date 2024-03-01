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

#include "Digi_At_commands.h"
#include "tcu_Uart.h"
#include "nvram.h"

// NVRAM file system instance for persisting application data
static struct nvs_fs fs;

// NVRAM partition where application data is stored in flash memory 
#define NVS_PARTITION		    storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define ZB_NODE_IDENTIFIER 1
#define ZB_EXT_PANID 2

static struct xbee_parameters_t xbee_parameters_nvram; // Xbee's parameters

// Define the exttended PAN ID for the network
uint8_t pan_id_default[8] = {0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Define a distributed key for the network
uint8_t distributed_key_default[16] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};


LOG_MODULE_REGISTER(nvram_app, LOG_LEVEL_DBG);

/**@brief This function initializes nvram file system
 * 
 */
void init_nvram(void)
{
    int rc;
    struct flash_pages_info info;

    /* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at NVS_PARTITION_OFFSET
	 */
	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return 0;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return 0;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return 0;
	}

    sprintf(xbee_parameters_nvram.at_ni, "NORDIC_JULEN_NVRAM_TEST"); // Node identifier string

    LOG_INF("NVRAM initialized\n");
}

/**@brief This function read nvram for Node Identifier
 * 
 */
void read_nvram_NI(void)
{
    int rc;
    uint8_t temp[33];

    rc = nvs_read(&fs, ZB_NODE_IDENTIFIER, &temp, sizeof(temp));
    if (rc > 0) {
        if (strlen(temp) == 0)
        {  
            (void)nvs_write(&fs, ZB_NODE_IDENTIFIER, &xbee_parameters_nvram.at_ni, strlen(xbee_parameters_nvram.at_ni)+1);

            //xbee_parameters_nvram.at_ni[strlen(xbee_parameters_nvram.at_ni)] = '\0'; // Add null terminator
            LOG_WRN("No Node Identifier found, adding %s at id %d\n", xbee_parameters_nvram.at_ni, ZB_NODE_IDENTIFIER);
        }
        else if(strlen(temp) > 0)
        {
            LOG_INF("Id: %d, Node Identifier: %s\n", ZB_NODE_IDENTIFIER, temp);
        }
        else
        {
            LOG_ERR("Error reading Node Identifier\n");
        }
    } 
    else 
    {
        LOG_WRN("No Node Identifier found, adding %s at id %d\n", xbee_parameters_nvram.at_ni, ZB_NODE_IDENTIFIER);
        (void)nvs_write(&fs, ZB_NODE_IDENTIFIER, &xbee_parameters_nvram.at_ni, strlen(xbee_parameters_nvram.at_ni)+1);
    }
}

/// @brief This function read nvram for PAN_ID
/// @param  
/**@brief This function reads nvram for PAN_ID
 * 
 */
void read_nvram_PAN_ID(void)
{
    int rc;
    uint8_t temp[8];

    rc = nvs_read(&fs, ZB_EXT_PANID, &temp, sizeof(temp));
    if (rc > 0) {
        LOG_INF("Id: %d, PAN_ID: ", ZB_EXT_PANID);
        for (int n = 0; n < 8; n++) {
            LOG_INF("[%d]: 0x%02x ", n, temp[n]);
        }
    } else {
        LOG_WRN("No PAN_ID found, adding it at id %d\n", ZB_EXT_PANID);
        (void)nvs_write(&fs, ZB_EXT_PANID, &pan_id_default, sizeof(pan_id_default));
    }
}

/// @brief 
void nvram_configuration()
{
    init_nvram();
    read_nvram_NI();
    read_nvram_PAN_ID();
}
