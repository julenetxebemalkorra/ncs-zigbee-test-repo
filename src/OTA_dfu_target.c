/*
 * Copyright (c) 2025 IED
 *
 */

/** @file
 *
 * @brief   Management of the OTA DFU target for firmware updates.
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

LOG_MODULE_REGISTER(OTA_dfu_target, LOG_LEVEL_INF);

#define STAGING_BUF_SIZE 512                                // Size of the staging buffer for dfu target mcuboot 
static uint8_t staging_buf[STAGING_BUF_SIZE] __aligned(32); // The array should be aligned to 32 bits,
static uint8_t dfu_target_initialized = false;

/**@brief Check that there is a partition available in flash for storing the new image.
 *
 * @retval 0 If successful, negative error code otherwise.
 */
int_fast8_t check_flash_area(void)
{
    const struct flash_area *fa;
    int rc = flash_area_open(FLASH_AREA_ID(image_1), &fa);

    if (rc != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("image_1 not found in flash map! Error: %d", rc);
    } 
    else 
    {
        LOG_INF("image_1 offset: 0x%lx, size: %d", fa->fa_off, fa->fa_size);
        flash_area_close(fa);
    }
    return rc;
}

/**@brief Initialization of dfu target
 *
 * @param[in] file_size of the file is getting downloaded.
 *
 * @retval 0 If successful, negative error code otherwise.
 */
int OTA_dfu_target_init(size_t file_size)
{
    int ret;
    size_t initial_offset;

    if (dfu_target_initialized)
    {
        dfu_target_mcuboot_done(false); // Cancel and release resources
        dfu_target_initialized = false;
    }

    ret = check_flash_area();
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("check_flash_area() failed: %d", ret);
        return OTA_DFU_TARGET_ERR_FLASH_AREA;
    }
    ret = dfu_target_mcuboot_set_buf(staging_buf, STAGING_BUF_SIZE);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_set_buf() failed: %d", ret);
        return OTA_DFU_TARGET_SET_BUF_ERR; 
    }
    ret = dfu_target_mcuboot_init(file_size, 0, NULL);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_init() failed: %d", ret);
        return OTA_DFU_TARGET_ERR_INIT;
    }
    dfu_target_initialized = true;
    ret = dfu_target_mcuboot_reset();
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_reset() failed: %d", ret);
        dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
        dfu_target_initialized = false;
        return OTA_DFU_TARGET_RESET_ERR; 
    }
    ret = dfu_target_mcuboot_offset_get(&initial_offset);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_offset_get() failed: %d", ret);
        dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
        dfu_target_initialized = false;
        return OTA_DFU_TARGET_OFFSET_ERR;
    }
    {
        LOG_ERR("dfu_target_mcuboot_offset_get() failed: %d", ret);
        dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
        dfu_target_initialized = false;
        return OTA_DFU_TARGET_OFFSET_ERR;
    }
    if (initial_offset != 0)
    {
        LOG_ERR("The initial dfu target offset is not 0");
        dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
        dfu_target_initialized = false;
        return OTA_DFU_TARGET_ERR_WRITE;
    }
    return OTA_DFU_TARGET_OK;
}

/**@brief Initialization of dfu target when we think that there was fuota upgrade process already started
 *
 * @param[in] file_size of the file is getting downloaded.
 *
 * @retval Offset of the already started fuota upgrade process. 0 if the initialization process did not success.
 */
uint32_t OTA_dfu_target_init_resume_previous_upgrade(size_t file_size)
{
    int ret;
    size_t initial_offset;

    ret = check_flash_area();
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("check_flash_area() failed: %d", ret);
        return OTA_DFU_TARGET_OK;
    }
    ret = dfu_target_mcuboot_set_buf(staging_buf, STAGING_BUF_SIZE);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_set_buf() failed: %d", ret);
        return OTA_DFU_TARGET_OK;
    }
    ret = dfu_target_mcuboot_init(file_size, 0, NULL);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_init() failed: %d", ret);
        return OTA_DFU_TARGET_OK;
    }
    dfu_target_initialized = true;
    ret = dfu_target_mcuboot_offset_get(&initial_offset);
    if (ret != OTA_DFU_TARGET_OK)
    {
        LOG_ERR("dfu_target_mcuboot_offset_get() failed: %d", ret);
        dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
        dfu_target_initialized = false;
        return OTA_DFU_TARGET_OK;
    }
    if (initial_offset > 0)
    {
        LOG_WRN("There was a previous FW upgrade in process");
        return (uint32_t)initial_offset;
    }
    dfu_target_mcuboot_done(false); // Cancel upgrade and release resources
    dfu_target_initialized = false;
    return OTA_DFU_TARGET_OK;
}

/**@brief Abort current dfu and release resources
 *
 * @details This function is called when the user decides to cancel the firmware update process.
 *          It will release the resources allocated for the dfu target and reset the state.
 */
void abort_dfu(void)
{
    if (dfu_target_initialized)
    {
        dfu_target_initialized = false;
        dfu_target_mcuboot_done(false);
    }
}

/**
 * @brief Handle a FOTA chunk received from the server.
 *
 * @param[in] payload Pointer to the data chunk received.
 * @param[in] len Length of the data chunk.
 * @param[in,out] file_offset Pointer to the current file offset, which will be updated after writing the chunk.
 *
 * @retval 0 If successful, negative error code otherwise.
 */
int handle_fota_chunk(const uint8_t *payload, size_t len, uint32_t *file_offset)
{
    size_t offset_before, offset_after;

    if (len <= 0) {
        LOG_ERR("FOTA chunk too short");
        return FUOTA_HANDLE_ERR;
    }

    int ret = dfu_target_mcuboot_offset_get(&offset_before);
    if (ret != FUOTA_HANDLE_OK) {
        LOG_ERR("error: failed to get offset before write\n");
    } else {
        LOG_DBG("offset before write: 0x%2x\n", offset_before);
    }

    if(offset_before != *file_offset) {
        LOG_ERR("Offset mismatch! Expected: 0x%08x, Got: 0x%08x", *file_offset, offset_before);
        *file_offset = offset_before;  // Update file_offset to match the current offset
        return FUOTA_HANDLE_ERR;
    }

    ret = dfu_target_mcuboot_write(payload, len);
    if (ret != FUOTA_HANDLE_OK) {
        LOG_ERR("dfu_target_write failed: %d", ret);
        *file_offset = offset_before;  // Update file_offset to match the current offset
        return ret;
    } else {
        LOG_DBG("FOTA chunk written: 0x%2x bytes", len);
    }

    ret = dfu_target_mcuboot_offset_get(&offset_after);
    if (ret != FUOTA_HANDLE_OK) {
        LOG_WRN("error: failed to get offset after write\n");
    } else {
        LOG_DBG("offset after write: 0x%2x\n", offset_after);
    }

    if (offset_after != offset_before + len) {
        *file_offset = offset_before;  // Update file_offset to match the current offset
        LOG_WRN("error: offset mismatch after write\n");
        ret = FUOTA_HANDLE_ERR;  // Indicate error
    } else {
        LOG_DBG("offset match after write\n");
    }

    *file_offset = offset_after;  // Update file_offset for next chunk
    LOG_INF("File_offset updated: 0x%2x\n", *file_offset);

    return ret;
}
