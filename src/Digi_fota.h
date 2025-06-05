/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef DIGI_FOTA_H_
#define DIGI_FOTA_H_

#include "global_defines.h"

#define DIGI_FILE_HEADER_SIZE 62      // DIGI .ota files include a 62-byte header before the binary file contents
#define FILE_BLOCK_MAX_SIZE 47        // With this size, a file fragment can be transmitted in a single Zigbee packet

// FUOTA SERVER COMMANDS
#define IMAGE_NOTIFY_CMD               0x00
#define QUERY_NEXT_IMAGE_RESPONSE_CMD  0x02
#define IMAGE_BLOCK_RESPONSE_CMD       0x05
#define UPGRADE_END_RESPONSE_CMD       0x07

// FUOTA CLIENT COMMANDS
#define QUERY_NEXT_IMAGE_REQUEST_CMD   0x01
#define IMAGE_BLOCK_REQUEST_CMD        0x03
#define UPGRADE_END_REQUEST_CMD        0x06

// FUOTA SERVER AND CLIENT COMMANDS
#define DEFAULT_RESPONSE_CMD           0x0B

// FUOTA SERVER COMMAND STATUS CODES
#define FOTA_STATUS_SUCCESS 0x00
#define FOTA_STATUS_NO_IMAGE_AVAILABLE 0x98
#define FOTA_STATUS_NOT_AUTHORIZED 0x7E

// SIZE OF FUOTA SERVER COMMANDS
#define IMAGE_NOTIFY_CMD_SIZE               13
#define QUERY_NEXT_IMAGE_RESPONSE_CMD_SIZE  16
#define IMAGE_BLOCK_RESPONSE_HEADER_SIZE    17
#define IMAGE_BLOCK_RESPONSE_CMD_SIZE_MIN   IMAGE_BLOCK_RESPONSE_HEADER_SIZE + 1
#define IMAGE_BLOCK_RESPONSE_CMD_SIZE_MAX   IMAGE_BLOCK_RESPONSE_HEADER_SIZE + FILE_BLOCK_MAX_SIZE
#define UPGRADE_END_RESPONSE_CMD_SIZE       19
#define DEFAULT_RESPONSE_CMD_SIZE           5


#define DIGI_MANUFACTURER_ID 0x101E   // DIGI manufacturer code
#define DIGI_IMAGE_TYPE      0x0001   // DIGI's manufacturer specific image type
#define FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER 0x01
#define FIELD_CONTROL_HW_VERSION_NO_PRESENT                   0x01

#define MAX_NEXT_IMAGE_CMD_RESPONSE_TIME_MS    10000 // Maximum duration the client will wait for a response to this command
#define MAX_IMAGE_BLOCK_CMD_RESPONSE_TIME_MS    5000 // Maximum duration the client will wait for a response to this command
#define MAX_UPGRADE_CMD_RESPONSE_TIME_MS       10000 // Maximum duration the client will wait for a response to this command

#define MAX_ATTEMPTS_DFU_INIT                3  // FUOTA process is canceled after 3 unsuccessful DFU initialization attempts
#define MAX_ATTEMPTS_NEXT_IMAGE_REQUEST      3  // FUOTA process is canceled after 3 unanswered Next Image requests
#define MAX_ATTEMPTS_IMAGE_BLOCK_REQUEST    10  // FUOTA process is canceled after 10 unanswered Image Block requests
#define MAX_ATTEMPTS_UPGRADE_END_REQUEST     3  // FUOTA process is canceled after 3 unanswered Upgrade and End requests

struct firmware_image_t {
    uint16_t manufacturer_code;
    uint16_t image_type;
    uint32_t firmware_version;
    uint32_t file_size;
};

/* Enumerative with the FUOTA state machine states */
enum fuota_state_machine_e{
    FUOTA_INIT_STATE_ST,
    FUOTA_UPGRADE_WAS_INTERRUPTED_ST,
    FUOTA_WAITING_FOR_IMAGE_NOTIFY_ST,
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

/* Function prototypes used only internally                                   */
bool process_image_notify_cmd(uint8_t* input_data, int16_t size_of_input_data);
bool process_next_image_response_cmd(uint8_t* input_data, int16_t size_of_input_data);
bool process_image_block_response_cmd(uint8_t* input_data, int16_t size_of_input_data);
bool process_upgrade_end_response_cmd(uint8_t* input_data, int16_t size_of_input_data);
bool process_default_response_cmd(uint8_t* input_data, int16_t size_of_input_data);
bool digi_fota_send_query_next_image_request_cmd(void);
bool digi_fota_send_image_block_request_cmd(void);
bool digi_fota_send_upgrade_end_request_cmd(void);
void digi_fota_switch_state(enum fuota_state_machine_e new_state);
void set_in_nvram_fota_is_active(void);
void set_in_nvram_fota_is_not_active(void);
bool read_from_nvram_if_fota_was_interrupted(void);

/* Function prototypes used externally                                        */
void digi_fota_init(void);
bool is_a_digi_fota_command(uint8_t* input_data, int16_t size_of_input_data);
void digi_fota_manager(void);

#endif /* DIGI_FOTA_H_ */







