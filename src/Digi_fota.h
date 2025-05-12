/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef DIGI_FOTA_H_
#define DIGI_FOTA_H_

#include "global_defines.h"

#define MAX_SIZE_AT_COMMAND_REPLY MAXIMUM_SIZE_NODE_IDENTIFIER

#define FOTA_STATUS_SUCCESS 0x00
#define QUERY_NEXT_IMAGE_RESPONSE 0x02
#define IMAGE_NOTIFY 0x03
#define IMAGE_BLOCK_REQUEST 0x05
#define UPGRADE_AND_REQUEST 0x06
#define UPGRADE_AND_RESPONSE 0x07
#define DIGI_MANUFACTURER_ID 0x101E
#define CURRENT_FW_VERSION 0x02001010
#define DIGI_FILE_HEADER_SIZE 62 // A 62 byte header has been added to the bin file to be compatible with DIGI
#define FILE_BLOCK_MAX_SIZE 49 // With this size, the file fragment can be sent in a single Zigbee packet.
#define FOTA_FIRST_BLOCK_HEADER_SIZE 79 // Adjust based on actual protocol
#define FOTA_BLOCK_HEADER_SIZE 17 // Adjust based on actual protocol



// FUOTA SERVER COMMANDS
#define IMAGE_NOTIFY_CMD               0x00
#define QUERY_NEXT_IMAGE_RESPONDE_CMD  0x02
#define IMAGE_BLOCK_RESPONSE_CMD       0x05
#define UPGRADE_END_RESPONSE_CMD       0x07

// FUOTA CLIENT COMMANDS
#define QUERY_NEXT_IMAGE_REQUEST_CMD   0x01
#define IMAGE_BLOCK_REQUEST_CMD        0x03
#define UPGRADE_END_REQUEST_CMD        0x06

// SIZE OF FUOTA SERVER COMMANDS
#define IMAGE_NOTIFY_CMD_SIZE               13
#define QUERY_NEXT_IMAGE_RESPONDE_CMD_SIZE  16
#define IMAGE_BLOCK_RESPONSE_HEADER_SIZE    17
#define IMAGE_BLOCK_RESPONSE_CMD_SIZE_MIN   IMAGE_BLOCK_RESPONSE_HEADER_SIZE + 1
#define IMAGE_BLOCK_RESPONSE_CMD_SIZE_MAX   IMAGE_BLOCK_RESPONSE_HEADER_SIZE + FILE_BLOCK_MAX_SIZE
#define UPGRADE_END_RESPONSE_CMD_SIZE       19

#define FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER 0x01
#define FIELD_CONTROL_HW_VERSION_NO_PRESENT                   0x01


struct firmware_image_t {
    uint16_t manufacturer_code;
    uint16_t image_type;
    uint32_t firmware_version;
    uint32_t file_size;
};

/* Enumerative with the FUOTA state machine states */
enum fuota_state_machine_e{
    FUOTA_INIT_STATE_ST,
    FUOTA_NO_UPGRADE_IN_PROCESS_ST,
    FUOTA_IMAGE_NOTIFY_RECEIVED_ST,
    FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST,
    FUOTA_WAITING_FOR_NEXT_IMAGE_RESPONSE_ST,
    FUOTA_NEXT_IMAGE_RESPONDED_ST,
    FUOTA_INIT_DFU_TARGET,
    FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST,
    FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST,
    FUOTA_IMAGE_BLOCK_RESPONDED_ST,
    FUOTA_MAKE_AN_UPGRADE_END_REQUEST_ST,
    FUOTA_WAITING_FOR_UPGRADE_END_RESPONSE_ST,
    FUOTA_UPGRADE_END_RESPONDED_ST,
    FUOTA_NUMBER_OF_STATES
};

void digi_fota_init(void);
bool is_a_digi_fota_command(uint8_t* input_data, int16_t size_of_input_data);

bool digi_fota_send_query_next_image_request_cmd(void);
bool digi_fota_send_image_block_request_cmd(void);
bool digi_fota_send_upgrade_end_request_cmd(void);
void digi_fota_manager(void);
void digi_fota_switch_state(enum fuota_state_machine_e new_state);


#endif /* DIGI_FOTA_H_ */







