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
#include <zephyr/sys/reboot.h>

#include <zboss_api.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>

#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_fota.h"
#include "OTA_dfu_target.h"

LOG_MODULE_REGISTER(Digi_fota, LOG_LEVEL_INF);

static enum fuota_state_machine_e fuota_state;     // FUOTA state machine state
static uint64_t time_last_state_transition_ms;     // Time of FUOTA last state transition
static uint64_t time_last_attempt_ms;              // Time of FUOTA last attempt to perform an action
static uint16_t attempt_counter;                   // Number of attempts to perform an action in the FUOTA process
static struct firmware_image_t firmware_image;     // Structure describing firmware image
static uint8_t command_sequence_number;
static uint32_t file_offset;
static uint32_t requested_file_offset;

/**@brief This function initializes the Digi_fota firmware module
 *
 */
void digi_fota_init(void)
{
    fuota_state = FUOTA_INIT_STATE_ST;
    time_last_state_transition_ms = k_uptime_get();
    time_last_attempt_ms = 0;
    attempt_counter = 0;
    command_sequence_number = 0;
}

/**@brief This function evaluates if the last received APS frame is a Digi's read AT command
 *
 * @param[in]   input_data   Pointer to payload of received APS frame
 * @param[in]   size_of_input_data   Payload size
 *
 * @retval True It is a read AT command
 * @retval False Otherwise
 */
bool is_a_digi_fota_command(uint8_t* input_data, int16_t size_of_input_data)
{
    bool b_return = false;

    //LOG_WRN("Received fota command %d", size_of_input_data);
    //LOG_HEXDUMP_DBG(input_data, size_of_input_data, "Received fota command in hex:");

    if ((size_of_input_data == IMAGE_NOTIFY_CMD_SIZE) && (input_data[2] == IMAGE_NOTIFY_CMD))
    {
        LOG_WRN("Received fota command READ_FOTA_IMAGE_NOTIFY");
        command_sequence_number = 0;
        digi_fota_switch_state(FUOTA_IMAGE_NOTIFY_RECEIVED_ST);
        b_return = true;
    }
    else if ((input_data[2] == QUERY_NEXT_IMAGE_RESPONDE_CMD) && (input_data[3] == FOTA_STATUS_SUCCESS) &&
             (size_of_input_data == QUERY_NEXT_IMAGE_RESPONDE_CMD_SIZE))
    {
        LOG_WRN("Received fota command QUERY NEXT IMAGE RESPONSE");
        if (fuota_state == FUOTA_WAITING_FOR_NEXT_IMAGE_RESPONSE_ST) // Ignore the command if we were not expecting its reception
        {
            firmware_image.manufacturer_code = (input_data[5] << 8) | input_data[4];
            firmware_image.image_type = (input_data[7] << 8) | input_data[6];
            firmware_image.firmware_version = (input_data[11] << 24) | (input_data[10] << 16) | (input_data[9] << 8) | input_data[8];
            firmware_image.file_size = (input_data[15] << 24) | (input_data[14] << 16) | (input_data[13] << 8) | input_data[12];
            if (firmware_image.file_size > DIGI_FILE_HEADER_SIZE)
            {
                firmware_image.file_size = firmware_image.file_size - DIGI_FILE_HEADER_SIZE; // Do not consider the bytes of the header.
            }
            else
            {
                firmware_image.file_size = 0;
            }
            command_sequence_number = input_data[1];
            if ((firmware_image.manufacturer_code == DIGI_MANUFACTURER_ID) && (firmware_image.file_size > 0))
            {
                LOG_WRN("It is a valid image for this device");
                file_offset = 0;
                digi_fota_switch_state(FUOTA_NEXT_IMAGE_RESPONDED_ST);

            }
            else
            {
                LOG_WRN("It is not a valid image for this device");
                digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
            }
        }
        b_return = true;
    }
    else if ((input_data[2] == IMAGE_BLOCK_RESPONSE_CMD) && (input_data[3] == FOTA_STATUS_SUCCESS) &&
             (size_of_input_data <= IMAGE_BLOCK_RESPONSE_CMD_SIZE_MAX) && (size_of_input_data >= IMAGE_BLOCK_RESPONSE_CMD_SIZE_MIN))
    {
        LOG_WRN("Received fota command IMAGE BLOCK RESPONSE");
        if (fuota_state == FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST) // Ignore the command if we were not expecting its reception
        {
            uint8_t chunk_len = size_of_input_data - IMAGE_BLOCK_RESPONSE_HEADER_SIZE;
            uint32_t received_file_offset = (input_data[15] << 24) | (input_data[14] << 16) | (input_data[13] << 8) | input_data[12];
            command_sequence_number = input_data[1];

            // Check if file offset has the expected value
            if (received_file_offset == requested_file_offset)  // Ignore the command if the offset is not correct
            {
                const uint8_t *chunk_data = &input_data[IMAGE_BLOCK_RESPONSE_HEADER_SIZE];  // Remove the header from the received payload
                int ret = handle_fota_chunk(chunk_data, chunk_len, &file_offset); //Store the chuck in NVRAM and update file_offset
                if (ret != 0)
                {
                    LOG_WRN("File offset + NEXT_IMAGE_SIZE: 0x%08X", file_offset);
                    LOG_ERR("handle_fota_chunk error: %d", ret);
                }
                else
                {
                    LOG_WRN("File offset + NEXT_IMAGE_SIZE: 0x%08X", file_offset);
                    LOG_WRN("handle_fota_chunk ok");
                }
                digi_fota_switch_state(FUOTA_IMAGE_BLOCK_RESPONDED_ST);
            }
        }
        b_return = true;
    }
    else if ((size_of_input_data == UPGRADE_END_RESPONSE_CMD_SIZE) && (input_data[2] == UPGRADE_END_RESPONSE_CMD))
    {
        LOG_WRN("Received fota command UPGRADE_AND_RESPONSE");
        LOG_WRN("OTA finished successfully, now we can reset the device");
        digi_fota_switch_state(FUOTA_UPGRADE_END_RESPONDED_ST);
        b_return = true;
    }
    else
    {
        LOG_WRN("Received unkown fota command");
        b_return = false;
    }
    return b_return;
}

/**@brief This function places in the APS output frame queue a frame containing the query next image request command.
 *
 * @retval True if the frame could be placed in the APS output frame
 * @retval False otherwise
 */
bool digi_fota_send_query_next_image_request_cmd(void)
{
    bool b_return = false;

    if (zigbee_aps_get_output_frame_buffer_free_space())
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = DIGI_PROFILE_ID;
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number;
        element.payload[i++] = QUERY_NEXT_IMAGE_REQUEST_CMD;
        element.payload[i++] = FIELD_CONTROL_HW_VERSION_NO_PRESENT;
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID >> 8); // High byte of manufacturer code
        element.payload[i++] = 0x01; // Image type 0x0001
        element.payload[i++] = 0x00;
        element.payload[i++] = (uint8_t)(CURRENT_FW_VERSION & 0xFF);
        element.payload[i++] = (uint8_t)((CURRENT_FW_VERSION >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((CURRENT_FW_VERSION >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)(CURRENT_FW_VERSION >> 24);

        element.payload_size = (zb_uint8_t)i;
        if( enqueue_aps_frame(&element) ) b_return = true;
    }
    return b_return;
}

/**@brief This function places in the APS output frame queue a frame containing the image block request command.
 *
 * @retval True if the frame could be placed in the APS output frame
 * @retval False otherwise
 */
bool digi_fota_send_image_block_request_cmd(void)
{
    bool b_return = false;
    requested_file_offset = file_offset + DIGI_FILE_HEADER_SIZE;  // Correct the offset. The first 62 bytes of the file are metadata.

    if (zigbee_aps_get_output_frame_buffer_free_space())
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = DIGI_PROFILE_ID;
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number +1;
        element.payload[i++] = IMAGE_BLOCK_REQUEST_CMD;
        element.payload[i++] = FOTA_STATUS_SUCCESS;
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID >> 8); // High byte of manufacturer code
        element.payload[i++] = 0x01; // Image type 0x0001
        element.payload[i++] = 0x00;
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version >> 24);
        element.payload[i++] = (uint8_t)(requested_file_offset & 0xFF); // Low byte of file offset
        element.payload[i++] = (uint8_t)((requested_file_offset >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((requested_file_offset >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)((requested_file_offset >> 24) & 0xFF); // High byte of file offset
        element.payload[i++] = FILE_BLOCK_MAX_SIZE;
        element.payload_size = (zb_uint8_t)i;

        if( enqueue_aps_frame(&element) ) b_return = true;
    }
    return b_return;
}

/**@brief This function places in the APS output frame queue a frame containing the upgrade end request command.
 *
 * @retval True if the frame could be placed in the APS output frame
 * @retval False otherwise
 */
bool digi_fota_send_upgrade_end_request_cmd(void)
{
    bool b_return = false;

    if (zigbee_aps_get_output_frame_buffer_free_space())
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = DIGI_PROFILE_ID;
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number + 1;
        element.payload[i++] = UPGRADE_END_REQUEST_CMD;
        element.payload[i++] = FOTA_STATUS_SUCCESS;
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = (uint8_t)(DIGI_MANUFACTURER_ID >> 8); // High byte of manufacturer code
        element.payload[i++] = 0x01; // Image type 0x0001
        element.payload[i++] = 0x00;
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version >> 24);
        element.payload_size = (zb_uint8_t)i;

        if( enqueue_aps_frame(&element) ) b_return = true;
    }
    return b_return;
}

/**@brief This function implements the state machine that handles the reception of the bin file during a FUOTA.
 *
 */
void digi_fota_manager(void)
{
    uint64_t time_now_ms = k_uptime_get();
    int ret;

    switch(fuota_state)
    {
        case FUOTA_INIT_STATE_ST:
            digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
            break;
        case FUOTA_NO_UPGRADE_IN_PROCESS_ST:
            break;
        case FUOTA_IMAGE_NOTIFY_RECEIVED_ST:
            digi_fota_switch_state(FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST);
            break;
        case FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST:
            if (digi_fota_send_query_next_image_request_cmd())
            {
                digi_fota_switch_state(FUOTA_WAITING_FOR_NEXT_IMAGE_RESPONSE_ST);
            }
            break;
        case FUOTA_WAITING_FOR_NEXT_IMAGE_RESPONSE_ST:
            if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
            {
                digi_fota_switch_state(FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST);
            }
            break;
        case FUOTA_NEXT_IMAGE_RESPONDED_ST:
            digi_fota_switch_state(FUOTA_INIT_DFU_TARGET);
            break;
        case FUOTA_INIT_DFU_TARGET:
            if ((time_now_ms - time_last_attempt_ms) > 2000) // Leave 2 seconds between attempts
            {
                time_last_attempt_ms = time_now_ms;
                ret = OTA_dfu_target_init(firmware_image.file_size);
                if (ret == 0)
                {
                    LOG_WRN("OTA_dfu_target_init() succeeded");
                    digi_fota_switch_state(FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST);
                }
                else
                {
                    attempt_counter++;
                    if (attempt_counter >= 3) // Maximum 3 attempts
                    {
                        digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                    }
                }
            }
            break;
        case FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST:
            if ((time_now_ms - time_last_state_transition_ms) > 200) // Artificial delay of 200 ms to avoid network congestion
            {
                if (digi_fota_send_image_block_request_cmd())
                {
                    digi_fota_switch_state(FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST);
                }
            }
            break;
        case FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST:
            if ((time_now_ms - time_last_state_transition_ms) > 5000) // More than 5 seconds waiting for reply
            {
                digi_fota_switch_state(FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST);
            }
            break;
        case FUOTA_IMAGE_BLOCK_RESPONDED_ST:
            if (file_offset < firmware_image.file_size)
            {
                digi_fota_switch_state(FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST);
            }
            else
            {
                digi_fota_switch_state(FUOTA_MAKE_AN_UPGRADE_END_REQUEST_ST);
            }
            break;
        case FUOTA_MAKE_AN_UPGRADE_END_REQUEST_ST:
            if (digi_fota_send_upgrade_end_request_cmd())
            {
                digi_fota_switch_state(FUOTA_WAITING_FOR_UPGRADE_END_RESPONSE_ST);
            }
            break;
        case FUOTA_WAITING_FOR_UPGRADE_END_RESPONSE_ST:
            if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
            {
                digi_fota_switch_state(FUOTA_MAKE_AN_UPGRADE_END_REQUEST_ST);
            }
            break;
        case FUOTA_UPGRADE_END_RESPONDED_ST:
            //digi_fota_switch_state(FUOTA_INIT_STATE_ST);
            int ret = dfu_target_mcuboot_done(true);
            if (ret)
            {
                LOG_ERR("dfu done failed: 0x%x\n", ret);
                digi_fota_switch_state(FUOTA_INIT_STATE_ST);
            }
            else
            {
                LOG_WRN("dfu done ok\n");
                dfu_target_mcuboot_schedule_update(0);  // Schedule the update image 0
                sys_reboot(SYS_REBOOT_COLD);
            }
            break;
        default:
            digi_fota_switch_state(FUOTA_INIT_STATE_ST);
            LOG_ERR("FUOTA state machine entered in a no existing state");
            break;
    }
}


/**@brief This function switches the FUOTA state machine to the state passed as argument
 *
 * @param[in]   new_state   New state of FUOTA state machine
 */
void digi_fota_switch_state(enum fuota_state_machine_e new_state)
{
    fuota_state = new_state;
    time_last_state_transition_ms = k_uptime_get();
    time_last_attempt_ms = 0;
    attempt_counter = 0;
}
