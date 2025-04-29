/*
 * Copyright (c) 2025 IED
 *
 */

/** @file
 *
 * @brief Managment of AT commands received through Zigbee.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/storage/flash_map.h>

#include "Digi_profile.h"
#include "OTA_dfu_target.h"

LOG_MODULE_REGISTER(OTA_dfu_target, LOG_LEVEL_DBG);

static char bin_header[512] = {0x3D, 0xB8, 0xF3, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x51, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define STAGING_BUF_SIZE 512
static uint8_t staging_buf[STAGING_BUF_SIZE];


int_fast8_t check_flash_area(void)
{
    const struct flash_area *fa;
    int rc = flash_area_open(FLASH_AREA_ID(image_1), &fa);
    if (rc != 0) 
    {
        LOG_ERR("image_1 not found in flash map! Error: %d", rc);
    } else 
    {
        LOG_INF("image_1 offset: 0x%x, size: %d", fa->fa_off, fa->fa_size);
        flash_area_close(fa);
    }
    return rc;
}

static void dfu_target_callback_handler(int evt)
{
    switch (evt) {
    case DFU_TARGET_EVT_TIMEOUT:
        LOG_WRN("DFU target timeout event");
		break;
	case DFU_TARGET_EVT_ERASE_DONE:
        LOG_WRN("DFU target erase done event");
		break;
    case DFU_TARGET_EVT_ERASE_PENDING:
        LOG_WRN("DFU target erase pending event");
        break;
    default:
        LOG_WRN("DFU target unknown event: %d", evt);
        break;
	}
}


/**@brief This function initializes the Digi_wireless_at_commands firmware module
 *
 */
int OTA_dfu_target_init()
{

    int ret = 0;

    ret = check_flash_area();
    if(ret != 0) {
        LOG_ERR("check_flash_area error: %d", ret);
        return;
    }
    
    // Just for sanity
    LOG_WRN("staging_buf addr: %p, size: %d\n", staging_buf, STAGING_BUF_SIZE);
    ret = dfu_target_mcuboot_set_buf(staging_buf, STAGING_BUF_SIZE);
    if (ret < 0) {
        LOG_WRN("set_buf failed: 0x%x", ret);
    }

    int img_type = dfu_target_img_type(bin_header, sizeof(bin_header));
    if (img_type < 0) {
        LOG_WRN("error: 0x%x\n", img_type);
    }
    else if (DFU_TARGET_IMAGE_TYPE_MCUBOOT == img_type)
    {
        LOG_WRN("image type: mcuboot\n");
    }
    else
    {
        LOG_WRN("image type: 0x%x\n", img_type);
    }

    ret = dfu_target_init(img_type, 0, 0x1000, dfu_target_callback_handler);
    if (ret) {
        LOG_WRN("dfu_target_init error: 0x%x\n", ret);
    }
    else
    {
        LOG_WRN("init ok\n");
    }
    return ret;
}

int handle_fota_chunk(uint8_t *payload, size_t len, uint32_t *file_offset)
{
    size_t offset_before, offset_after;

    if (len <= 0) {
        LOG_ERR("FOTA chunk too short");
        return -1;
    }

    int ret = dfu_target_offset_get(&offset_before);
    if (ret != 0) {
        LOG_ERR("error: failed to get offset before write\n");
    } else {
        LOG_WRN("offset before write: 0x%2x\n", offset_before);
    }

    if(offset_before != *file_offset) {
        LOG_ERR("Offset mismatch! Expected: 0x%08x, Got: 0x%08x", *file_offset, offset_before);
        *file_offset = offset_before;  // Update file_offset to match the current offset
        return -1;
    }

    ret = dfu_target_write(payload, len);
    if (ret != 0) {
        LOG_ERR("dfu_target_write failed: %d", ret);
        *file_offset = offset_before;  // Update file_offset to match the current offset
        return ret;
    } else {
        LOG_WRN("FOTA chunk written: 0x%2x bytes", len);
    }

    ret = dfu_target_offset_get(&offset_after);
    if (ret != 0) {
        LOG_WRN("error: failed to get offset after write\n");
    } else {
        LOG_WRN("offset after write: 0x%2x\n", offset_after);
    }

    if (offset_after != offset_before + len) {
        *file_offset = offset_before;  // Update file_offset to match the current offset
        LOG_WRN("error: offset mismatch after write\n");
        ret = -1;  // Indicate error
    } else {
        LOG_WRN("offset match after write\n");
    }

    *file_offset = offset_after;  // Update file_offset for next chunk
    LOG_WRN("file_offset updated: 0x%2x\n", *file_offset);

    return ret;
}
