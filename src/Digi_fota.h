/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef DIGI_FOTA_H_
#define DIGI_FOTA_H_

#include "global_defines.h"

#define MAX_SIZE_AT_COMMAND_REPLY MAXIMUM_SIZE_NODE_IDENTIFIER

#define NO_HW_VERSION 0x00
#define FOTA_STATUS_SUCCESS 0x00
#define QUERY_NEXT_IMAGE_REQUEST 0x01
#define QUERY_NEXT_IMAGE_RESPONSE 0x02
#define IMAGE_NOTIFY 0x03
#define IMAGE_BLOCK_REQUEST 0x05
#define UPGRADE_AND_REQUEST 0x06
#define UPGRADE_AND_RESPONSE 0x07
#define MANUFACTORER_CODE 0x1E10
#define FILE_VERSION 0x02001012
#define NEXT_IMAGE_SIZE 0xee
#define FOTA_FIRST_BLOCK_HEADER_SIZE 79 // Adjust based on actual protocol
#define FOTA_BLOCK_HEADER_SIZE 17 // Adjust based on actual protocol



/* Enumerative with the supported Xbee wireless AT commands used to read parameters */
enum digi_fota_read_cmd_e{
    READ_FOTA_IMAGE_NOTIFY,
    READ_FOTA_QUERY_NEXT_IMAGE_REQUEST,
    BLOCK_REQUEST,
    LAST_BLOCK_REQUEST_COMMAND,
    NUMBER_OF_FOTA_COMMANDS,
    NO_SUPPORTED_FOTA_CMD
};

void digi_fota_init(void);
bool is_a_digi_fota_command(uint8_t* input_data, int16_t size_of_input_data);
bool digi_fota_cmd_reply(void);
void digi_fota_command_manager(void);

#endif /* DIGI_FOTA_H_ */







