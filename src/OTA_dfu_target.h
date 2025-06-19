/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef OTA_DFU_TARGET_H_
#define OTA_DFU_TARGET_H_

#include "global_defines.h"

typedef enum {
    OTA_DFU_TARGET_OK = 0,
    OTA_DFU_TARGET_ERR_FLASH_AREA = -1,
    OTA_DFU_TARGET_SET_BUF_ERR = -2,
    OTA_DFU_TARGET_ERR_INIT = -3,
    OTA_DFU_TARGET_RESET_ERR = -4,
    OTA_DFU_TARGET_OFFSET_ERR = -5,
    OTA_DFU_TARGET_ERR_WRITE = -6,
    OTA_DFU_TARGET_ERR_UNKNOWN = -100
} ota_dfu_target_err_t;

typedef enum {
    FUOTA_HANDLE_OK = 0,
    FUOTA_HANDLE_ERR = -1,
} fuota_handle_err_t;


int OTA_dfu_target_init(size_t file_size);
uint32_t OTA_dfu_target_init_resume_previous_upgrade(size_t file_size);
void abort_dfu(void);
int handle_fota_chunk(const uint8_t *payload, size_t len, uint32_t *file_offset);

#endif /* OTA_DFU_TARGET_H_ */







