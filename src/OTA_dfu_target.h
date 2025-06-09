/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef OTA_DFU_TARGET_H_
#define OTA_DFU_TARGET_H_

#include "global_defines.h"

int OTA_dfu_target_init(size_t file_size);
uint32_t OTA_dfu_target_init_resume_previous_upgrade(size_t file_size);
void abort_dfu(void);
int handle_fota_chunk(const uint8_t *payload, size_t len, uint32_t *file_offset);

#endif /* OTA_DFU_TARGET_H_ */







