/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef OTA_DFU_TARGET_H_
#define OTA_DFU_TARGET_H_

#include "global_defines.h"

#define MAX_SIZE_AT_COMMAND_REPLY MAXIMUM_SIZE_NODE_IDENTIFIER

int OTA_dfu_target_init(void);
int handle_fota_chunk(uint8_t *payload, size_t len, uint32_t *file_offset);
void dfu_target_test(void);


#endif /* OTA_DFU_TARGET_H_ */







