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

#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_fota.h"
#include "OTA_dfu_target.h"
#include "nvram.h"

LOG_MODULE_REGISTER(Digi_fota, LOG_LEVEL_INF);

static enum fuota_state_machine_e fuota_state;     // FUOTA state machine state
static uint64_t time_last_state_transition_ms;     // Time of FUOTA last state transition
static uint64_t time_last_attempt_ms;              // Time of FUOTA last attempt to make an action
static struct firmware_image_t firmware_image;     // Structure describing firmware image
static uint8_t command_sequence_number;
static uint32_t file_offset;
static uint32_t requested_file_offset;
static bool b_dfu_target_init_done;
static bool b_dfu_target_reset_done;
static uint8_t DFUOTA_status[1] = {0};
static uint32_t firmware_version[1] = {0};
static uint32_t firmware_size[1] = {0};

/**@brief This function initializes the Digi_fota firmware module
 *
 */
void digi_fota_init(void)
{
    fuota_state = FUOTA_INIT_STATE_ST;
    time_last_state_transition_ms = k_uptime_get();
    time_last_attempt_ms = 0;
    command_sequence_number = 0;
    b_dfu_target_init_done = false;
    b_dfu_target_reset_done = false;
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

    if ((size_of_input_data == FOTA_STATUS_NOT_OK_CMD_SIZE) && (input_data[4] == NO_IMAGE_AVAILABLE_CMD))
    {
        LOG_WRN("Received fota command NO_IMAGE_AVAILABLE_CMD");
        //digi_fota_switch_state(FUOTA_INIT_STATE_ST);
    }
    else if ((size_of_input_data == FOTA_STATUS_NOT_OK_CMD_SIZE) && (input_data[4] == SERVER_NOT_AUTHORIZED_CMD))
    {
        LOG_WRN("Received fota command SERVER_NOT_AUTHORIZED_CMD");
        //digi_fota_switch_state(FUOTA_INIT_STATE_ST);
    }
    else if ((size_of_input_data == FOTA_STATUS_NOT_OK_CMD_SIZE) && (input_data[4] == UNSUP_CLUSTER_COMMAND))
    {
        LOG_WRN("Received fota command SERVER_NOT_AUTHORIZED_CMD: : unexpected direction in the Frame Control Field of the ZCL Header; unexpected command identifier code value in the ZCL header");
        //digi_fota_switch_state(FUOTA_INIT_STATE_ST);
    } 
    else if ((size_of_input_data == IMAGE_NOTIFY_CMD_SIZE) && (input_data[2] == IMAGE_NOTIFY_CMD))
    {
        LOG_WRN("Received fota command READ_FOTA_IMAGE_NOTIFY");
        command_sequence_number = 0;
        firmware_image.firmware_version = (input_data[12] << 24) | (input_data[11] << 16) | (input_data[10] << 8) | input_data[9];
        LOG_INF("firmware version: %d", firmware_image.firmware_version);
        digi_fota_switch_state(FUOTA_IMAGE_NOTIFY_RECEIVED_ST);
        b_return = true;
    }
    else if ((size_of_input_data == QUERY_NEXT_IMAGE_RESPONDE_CMD_SIZE) && (input_data[2] == QUERY_NEXT_IMAGE_RESPONDE_CMD) && (input_data[3] == FOTA_STATUS_SUCCESS))
    {
        firmware_image.manufacturer_code = (input_data[5] << 8) | input_data[4];
        firmware_image.image_type = (input_data[7] << 8) | input_data[6];
        firmware_image.firmware_version = (input_data[11] << 24) | (input_data[10] << 16) | (input_data[9] << 8) | input_data[8];
        firmware_image.file_size = (input_data[15] << 24) | (input_data[14] << 16) | (input_data[13] << 8) | input_data[12];
        LOG_INF("firmware image size: 0x%08X", firmware_image.file_size);
        LOG_INF("firmware image type: %d", firmware_image.image_type);
        LOG_INF("firmware image manufacturer code: %d", firmware_image.manufacturer_code);
        LOG_INF("firmware image version: %d", firmware_image.firmware_version);

        LOG_INF("Received fota command QUERY NEXT IMAGE RESPONSE");
        command_sequence_number = input_data[1];
        digi_fota_switch_state(FUOTA_NEXT_IMAGE_RESPONDED_ST);
        b_return = true;
    }
    else if ((size_of_input_data <= IMAGE_BLOCK_RESPONSE_CMD_SIZE_MAX) && (size_of_input_data >= IMAGE_BLOCK_RESPONSE_CMD_SIZE_MIN) &&
             (input_data[2] == IMAGE_BLOCK_RESPONSE_CMD) && (input_data[3] == FOTA_STATUS_SUCCESS))
    {
        uint8_t chunk_len = size_of_input_data - IMAGE_BLOCK_RESPONSE_HEADER_SIZE;
        uint32_t received_file_offset = (input_data[15] << 24) | (input_data[14] << 16) | (input_data[13] << 8) | input_data[12];
        firmware_image.manufacturer_code = (input_data[5] << 8) | input_data[4];
        firmware_image.image_type = (input_data[7] << 8) | input_data[6];
        firmware_image.firmware_version = (input_data[11] << 24) | (input_data[10] << 16) | (input_data[9] << 8) | input_data[8];

        LOG_INF("firmware image size: 0x%08X", firmware_image.file_size);
        LOG_INF("firmware image type: %d", firmware_image.image_type);
        LOG_INF("firmware image manufacturer code: %d", firmware_image.manufacturer_code);
        LOG_INF("firmware image version: %d", firmware_image.firmware_version);
        LOG_WRN("Received file offset: 0x%08X", received_file_offset);
        // Calculate and log the OTA progress percentage
        if (firmware_image.file_size > 0) {
            uint8_t progress_percentage = (received_file_offset * 100) / firmware_image.file_size;
            LOG_WRN("OTA Progress: %d%%", progress_percentage);
        } else {
            LOG_ERR("Invalid firmware image size, cannot calculate progress");
            b_return = true;
            return;
        }

        LOG_INF("Received fota command IMAGE BLOCK RESPONSE");
        command_sequence_number = input_data[1];

        // Check if file offset has the expected value
        if (received_file_offset == requested_file_offset)  // Don't do anything with this frame if offset is not correct.
        {   
            const uint8_t *chunk_data = &input_data[IMAGE_BLOCK_RESPONSE_HEADER_SIZE];  // Remove the header from the received payload
            int ret = handle_fota_chunk(chunk_data, chunk_len, &file_offset); //Store the chuck in NVRAM and update file_offset
            if (ret != 0)
            {
                LOG_ERR("File offset + NEXT_IMAGE_SIZE: 0x%08X", file_offset);
                LOG_ERR("handle_fota_chunk error: %d", ret);
            }
            else
            {
                LOG_INF("File offset + NEXT_IMAGE_SIZE: 0x%08X", file_offset);
                LOG_INF("handle_fota_chunk ok");
            }
            digi_fota_switch_state(FUOTA_IMAGE_BLOCK_RESPONDED_ST);
        }
        else
        {
            LOG_ERR("Received fota command IMAGE BLOCK RESPONSE with wrong file offset %d", received_file_offset);
            LOG_ERR("Expected file offset 0x%08X", requested_file_offset);
            digi_fota_switch_state(FUOTA_IMAGE_BLOCK_RESPONDED_ST);
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
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number;
        element.payload[i++] = QUERY_NEXT_IMAGE_REQUEST_CMD;
        element.payload[i++] = FIELD_CONTROL_HW_VERSION_NO_PRESENT;
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = DFU_TARGET_IMAGE_TYPE_MCUBOOT;
        element.payload[i++] = 0x00;   // file version different may not work neither
        element.payload[i++] = 0x0e;
        element.payload[i++] = 0x10;
        element.payload[i++] = 0x00;
        element.payload[i++] = 0x00;
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

    LOG_WRN("Requested file offset: 0x%08X", requested_file_offset);

    if (zigbee_aps_get_output_frame_buffer_free_space())
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number +1;
        element.payload[i++] = IMAGE_BLOCK_REQUEST_CMD;
        element.payload[i++] = FOTA_STATUS_SUCCESS;
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = DFU_TARGET_IMAGE_TYPE_MCUBOOT;
        element.payload[i++] = 0x00;
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version >> 24); // High byte of file version
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
        element.cluster_id = DIGI_FOTA_CLUSTER;
        element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = FRAME_CONTROL_FIELD_CLUSTER_SPECIFIC_CLIENT_TO_SERVER;
        element.payload[i++] = command_sequence_number + 1;
        element.payload[i++] = UPGRADE_END_REQUEST_CMD;
        element.payload[i++] = FOTA_STATUS_SUCCESS;
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
        element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
        element.payload[i++] = DFU_TARGET_IMAGE_TYPE_NONE;
        element.payload[i++] = 0x00;
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 8) & 0xFF);
        element.payload[i++] = (uint8_t)((firmware_image.firmware_version >> 16) & 0xFF);
        element.payload[i++] = (uint8_t)(firmware_image.firmware_version >> 24); // High byte of file version
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
            digi_fota_switch_state(FUOTA_CHECK_FOR_UPGRADE_ST);
            break;
        case FUOTA_CHECK_FOR_UPGRADE_ST:
            // read if there is an upgrade in process unfinished and there is a new image available
            ret = read_nvram(DUFOTA_STATUS, DFUOTA_status, sizeof(DFUOTA_status));
            if (ret <= 0)     
            {
                LOG_ERR("read_nvram error %d", ret);
                if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                {
                    LOG_ERR("NVRAM error, starting OTA for the beggining");
                    b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                    digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                }
            }
            else
            {
                if(DFUOTA_status[0] == 0x01) // OTA ongoing
                {
                    LOG_WRN("OTA ongoing already, no need to start it again. check FW version and size");
                    // Check if the image is the same as the one we have, if not, then we can start the download
                    ret = read_nvram(DUFOTA_FW_VERSION, firmware_version, sizeof(firmware_version));
                    if (ret <= 0)     
                    {
                        LOG_ERR("read_nvram error %d", ret);
                        if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                        {
                            LOG_ERR("NVRAM error, starting OTA for the beggining");
                            digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                        }
                    }
                    else
                    {
                        LOG_INF("Read firmware version succesfully from nvram");
                    }
                    ret = read_nvram(DUFOTA_FW_SIZE, firmware_size, sizeof(firmware_size));
                    if (ret <= 0)     
                    {
                        LOG_ERR("read_nvram error %d", ret);
                        if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                        {
                            LOG_ERR("NVRAM error, starting OTA for the beggining");
                            digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                        }
                    }
                    else
                    {
                        firmware_image.file_size = firmware_size[0];
                        LOG_WRN("Read firmware size succesfully from nvram");
                        digi_fota_switch_state(FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST);
                    }
                }
                else if (DFUOTA_status[0] == 0x00) 
                {
                    LOG_WRN("No OTA ongoing, waiting until image notify command is received");
                    digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                }
                else 
                {
                    LOG_ERR("Unknown OTA status: %d", DFUOTA_status[0]);
                    if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                    {
                        LOG_ERR("NVRAM error, starting OTA for the beggining");
                        digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
                    }
                }
            }
            break;
        case FUOTA_NO_UPGRADE_IN_PROCESS_ST:
            break;
        case FUOTA_IMAGE_NOTIFY_RECEIVED_ST:
            if(DFUOTA_status[0] == 0x01) // OTA ongoing
            {
                LOG_INF("OTA ongoing already, no need to start it again. check FW version and size");
                // Check if the image is the same as the one we have, if not, then we can start the download
                if(firmware_version[0] == firmware_image.firmware_version) // Check if the firmware version is the same
                {
                    LOG_INF("Firmware version is the same, no need to start OTA again");
                    digi_fota_switch_state(FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST);
                    // Do nothing, wait for a new image
                }
                else 
                {
                    LOG_ERR("Firmware version installed is different, start OTA again");
                    firmware_version[0] = firmware_image.firmware_version;
                    write_nvram(DUFOTA_FW_VERSION, firmware_version, sizeof(firmware_version));
                    b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                    digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
                    // Start OTA again, we can do it here because the image is different
                }
            }
            else if (DFUOTA_status[0] == 0x00) 
            {
                LOG_INF("No OTA ongoing, starting it now");
                b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                firmware_version[0] = firmware_image.firmware_version;
                write_nvram(DUFOTA_FW_VERSION, firmware_version, sizeof(firmware_version));
                digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
            }
            else 
            {
                LOG_ERR("Unknown OTA status: %d", DFUOTA_status[0]);
                if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                {
                    LOG_ERR("NVRAM error, starting OTA for the beggining");
                    b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                    digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
                }
            }
            break;
        case FUOTA_INIT_FUOTA_ST:
            // to do here: check if the image is the same as the one we have, if not, then we can start the download
            // if the image is the same, then we can leave the state machine in this state and wait for a new image
            // nvram OTA ongoing??
            // if yes firmware version and size compare
            // if no, then we can start the download
            if (!b_dfu_target_reset_done) // We should reset previous dfu
            {
                ret = dfu_target_mcuboot_reset();
                if (ret == -ENOENT) 
                {
                    LOG_INF("No existing DFU progress found (nothing to delete)");
                    b_dfu_target_reset_done = true;
                } 
                else if (ret != 0) 
                {
                    LOG_ERR("Failed to delete DFU settings: %d", ret);
                    b_dfu_target_reset_done = false;
                    if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                    {
                        LOG_ERR("Failed to delete DFU settings during 30 minutes go to FUOTA_PROCESSING_ERROR_ST ");
                        digi_fota_switch_state(FUOTA_PROCESSING_ERROR_ST);
                    }
                }
                else
                {
                    LOG_INF("dfu_target_mcuboot_reset ok");
                    b_dfu_target_reset_done = true;
                }
            }
            else
            {
                LOG_INF("dfu_target_mcuboot_reset already done");
                digi_fota_switch_state(FUOTA_MAKE_NEXT_IMAGE_REQUEST_ST);
            }
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
            digi_fota_switch_state(FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST);
            break;
        case FUOTA_MAKE_NEW_IMAGE_BLOCK_REQUEST_ST:
            if(DFUOTA_status[0] == 0x01) // OTA ongoing
            {
                LOG_INF("OTA ongoing already, no need to start it again. check FW version and size");
                // Check if the image is the same as the one we have, if not, then we can start the download
                if(firmware_size[0] == firmware_image.file_size) // Check if the firmware version is the same
                {
                    LOG_INF("Firmware size is the same, no need to start OTA again");
                    // Do nothing, wait for a new image
                }
                else 
                {
                    LOG_ERR("Firmware version installed is different, start OTA again");
                    firmware_size[0] = firmware_image.file_size;
                    write_nvram(DUFOTA_FW_SIZE, firmware_size, sizeof(firmware_size));
                    b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                    digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
                    // Start OTA again, we can do it here because the image is different
                }
            }
            else if (DFUOTA_status[0] == 0x00) 
            {
                LOG_INF("No OTA ongoing, starting it now");
                DFUOTA_status[0] = 0x01; // OTA ongoing                    
                write_nvram(DUFOTA_STATUS, DFUOTA_status, sizeof(DFUOTA_status));
                firmware_size[0] = firmware_image.file_size;
                write_nvram(DUFOTA_FW_SIZE, firmware_size, sizeof(firmware_size));
                b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
            }
            else 
            {
                LOG_ERR("Unknown OTA status: %d", DFUOTA_status[0]);
                if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
                {
                    LOG_ERR("NVRAM error, starting OTA for the beggining");
                    b_dfu_target_reset_done = false; // Reset the flag to false, we need to reset the dfu target
                    digi_fota_switch_state(FUOTA_INIT_FUOTA_ST);
                }
            }
            if (b_dfu_target_init_done == false) // Do not leave this state unless dfu target has been initiated
            {
                if ((time_now_ms - time_last_attempt_ms) > 5000) // Leave 5 seconds between attempts
                {
                    LOG_INF("Trying to init dfu target; file size: %d", firmware_image.file_size);
                    ret = OTA_dfu_target_init(firmware_image.file_size);
                    if (ret) {
                        LOG_ERR("dfu_target_init error: %d", ret);
                    }
                    else
                    {
                        LOG_INF("dfu_target_init ok");
                        if(DFUOTA_status[0] == 0x01) // OTA ongoing
                        {
                            ret = OTA_dfu_target_get_offset(&file_offset);
                            if (ret) {
                                file_offset = 0;
                                LOG_ERR("dfu_target_get_offset error: %d", ret);
                            }
                            else
                            {
                                //file_offset = file_offset - 2*STAGING_BUF_SIZE; // Remove the header from the file offset
                                LOG_WRN("file offset after reset and OTA_dfu_target_init: 0x%2x\n", file_offset);
                            }
                        }
                        b_dfu_target_init_done = true;
                    }
                    time_last_attempt_ms = time_now_ms;
                }
            }
            else
            {
                if (digi_fota_send_image_block_request_cmd())
                {
                    LOG_INF("digi_fota_send_image_block_request_cmd ok");
                    digi_fota_switch_state(FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST);
                }
            }
            break;
        case FUOTA_WAITING_FOR_IMAGE_BLOCK_RESPONSE_ST:
            if ((time_now_ms - time_last_state_transition_ms) > 30000) // More than 30 seconds waiting for reply
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
                LOG_WRN("File offset: 0x%08X", file_offset);
                LOG_WRN("Firmware image size: 0x%08X", firmware_image.file_size);
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
                DFUOTA_status[0] = 0x00; // OTA ongoing                    
                write_nvram(DUFOTA_STATUS, DFUOTA_status, sizeof(DFUOTA_status));
                dfu_target_mcuboot_schedule_update(0);  // Schedule the update image 0
                sys_reboot(SYS_REBOOT_COLD);
            }
            break;
        case FUOTA_PROCESSING_ERROR_ST:
            // TO DO what if the proccess goes to error state? we need to reset the dfu target and start again
            LOG_ERR("FUOTA_PROCESSING_ERROR_ST, starting OTA from the beggining");
            digi_fota_switch_state(FUOTA_NO_UPGRADE_IN_PROCESS_ST);
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
}
