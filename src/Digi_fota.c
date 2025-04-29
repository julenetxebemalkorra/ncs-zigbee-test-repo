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

LOG_MODULE_REGISTER(Digi_fota, LOG_LEVEL_DBG);

static bool b_pending_read_cmd;             // A read AT command has been received and is pending to be replied
static bool b_first_block_request = false; // Flag to check if the DFU target has been initialized
static enum digi_fota_read_cmd_e read_cmd;       // Enumerative of read AT command
static uint32_t file_offset;                     // File offset of the image to be downloaded
static uint8_t read_cmd_first_char;                // First char of read AT command
static uint8_t read_cmd_size;               // Second char of read AT command

/**@brief This function initializes the Digi_wireless_at_commands firmware module
 *
 */
void digi_fota_init(void)
{
    b_pending_read_cmd = false;
    read_cmd = NO_SUPPORTED_FOTA_CMD;
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
    enum digi_fota_read_cmd_e received_cmd = NO_SUPPORTED_FOTA_CMD;

    LOG_WRN("Received fota command %d", size_of_input_data);
    LOG_HEXDUMP_DBG(input_data, size_of_input_data, "Received fota command in hex:");

    if ((size_of_input_data == 13) && (input_data[3] == IMAGE_NOTIFY)) // We got this value with the sniffer and reverse engineering
    {
        LOG_WRN("Received fota command READ_FOTA_IMAGE_NOTIFY");
        received_cmd = READ_FOTA_IMAGE_NOTIFY;
        read_cmd_first_char = input_data[1];
        b_pending_read_cmd = true;
        b_return = true;
    }
    else if(size_of_input_data == 16 && input_data[2] == QUERY_NEXT_IMAGE_RESPONSE && (input_data[3] == FOTA_STATUS_SUCCESS))
    {
        LOG_WRN("Received fota command QUERY_NEXT_IMAGE_RESPONSE");
        received_cmd = BLOCK_REQUEST;
        read_cmd_first_char = input_data[1];
        read_cmd_size = NEXT_IMAGE_SIZE;
        file_offset = 0;
        b_pending_read_cmd = true;
        b_first_block_request = true;
        b_return = true;

    }
    else if(size_of_input_data <= 255 && input_data[2] == IMAGE_BLOCK_REQUEST && (input_data[3] == FOTA_STATUS_SUCCESS))
    {
        LOG_WRN("Received image block response fota command IMAGE_BLOCK_REQUEST");
        if (input_data[4] == (uint8_t)(MANUFACTORER_CODE >> 8) && input_data[5] == (uint8_t)(MANUFACTORER_CODE & 0xFF))
        {
            LOG_DBG(" Manufacturer code is correct %d", MANUFACTORER_CODE);
        }
        else
        {
            LOG_ERR(" Manufacturer code is NOT correct 0x%02X, 0x%02X", input_data[4], input_data[5]);
        }

        if (input_data[8] == 0x12 && input_data[9] == 0x10 && input_data[10] == 0x00 && input_data[11] == 0x02)
        {
            LOG_DBG(" Image version is correct %d", FILE_VERSION);
        }
        else
        {
            LOG_ERR(" Image version is NOT correct 0x%02X, 0x%02X, 0x%02X, 0x%02X", input_data[8], input_data[9], input_data[10], input_data[11]);
        }
        // Check if the image type is correct
        if (input_data[6] == DFU_TARGET_IMAGE_TYPE_NONE)
        {
            // if the image type is 0x00, it means that the image is not supported
            LOG_ERR("Image type is NOT correct 0x%02X", input_data[6]);
            // shpould we send a command requesting the last block?
            received_cmd = LAST_BLOCK_REQUEST_COMMAND;
        }
        else if(input_data[6] == DFU_TARGET_IMAGE_TYPE_MCUBOOT)
        {
            received_cmd = BLOCK_REQUEST;
        }
        else
        {
            LOG_ERR("Image type is NOT correct 0x%02X", input_data[6]);
            received_cmd = NO_SUPPORTED_FOTA_CMD;
        }

        // Save the file offset to resend later in the answer
        file_offset = (input_data[15] << 24) | (input_data[14] << 16) | (input_data[13] << 8) | input_data[12];
        LOG_WRN("Saved file offset: 0x%08X", file_offset);

        if (input_data[16] == NEXT_IMAGE_SIZE)
        {
            read_cmd_size = NEXT_IMAGE_SIZE;
            LOG_WRN("file size is correct 0x%2x", input_data[16]);
        }
        else
        {   
            read_cmd_size = input_data[16];
            LOG_WRN("file size less than 0xEE but it is OK 0x%02X", input_data[16]);
            if (input_data[16] == 0x00)
            {
                LOG_WRN("file size is 0x00, it is the last block");
                received_cmd = LAST_BLOCK_REQUEST_COMMAND;
                b_pending_read_cmd = true;
                b_return = true;
            }
        }
        // Check if dfu target is initialized correctly
        if (g_b_dfu_target_init_done == false)
        {
            int ret = OTA_dfu_target_init();
            if (ret) {
                LOG_ERR("dfu_target_init error: %d", ret);
            }
            else
            {
                LOG_WRN("init ok");
                g_b_dfu_target_init_done = true;
            }
        }
        else if (b_pending_read_cmd == false)
        {
            // TO DO is this enough to know it is the first block request?
            if (b_first_block_request = true && size_of_input_data == 255)
            {   
                size_t header_size = FOTA_FIRST_BLOCK_HEADER_SIZE;  // Remove the header before writeng to the flash
                const uint8_t *chunk_data = &input_data[header_size];

                if(chunk_data == NULL)
                {
                    LOG_ERR("b_first_block_request Chunk data is NULL");
                    return false;
                }
                if(size_of_input_data < header_size)
                {
                    LOG_ERR("b_first_block_request Chunk data size is less than header size %d < %d", size_of_input_data, header_size);
                    return false;
                }

                size_t chunk_len = size_of_input_data - header_size;

                int ret = handle_fota_chunk(chunk_data, chunk_len, &file_offset);
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
                
                b_first_block_request = false;
                LOG_WRN("First block request, file offset: 0x%08X", file_offset);
            }
            else
            {
                size_t header_size = FOTA_BLOCK_HEADER_SIZE;  // Remove the header before writeng to the flash
                const uint8_t *chunk_data = &input_data[header_size];

                if(chunk_data == NULL)
                {
                    LOG_ERR("Chunk data is NULL");
                    return false;
                }
                if(size_of_input_data < header_size)
                {
                    LOG_ERR("Chunk data size is less than header size %d < %d", size_of_input_data, header_size);
                    return false;
                }

                size_t chunk_len = size_of_input_data - header_size;

                int ret = handle_fota_chunk(chunk_data, chunk_len, &file_offset);
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

                b_first_block_request = false;
            }
        }
        else
        {
            b_pending_read_cmd = false;
            b_return = true;
            return b_return;
        }

        read_cmd_first_char = input_data[1];
        b_pending_read_cmd = true;
        b_return = false;
    }
    else if(size_of_input_data <= 255 && input_data[2] == UPGRADE_AND_RESPONSE)
    {
        LOG_WRN("Received fota command UPGRADE_AND_RESPONSE");
        LOG_WRN("OTA finished successfully, now we can reset the device");
        // answer to the last block request command
        // dfu_target_done(true);
        int ret = dfu_target_done(true);
        if (ret) 
        {
            LOG_ERR("dfu done failed: 0x%x\n", ret);
            return true;
        }
        else
        {
            LOG_WRN("dfu done ok\n");
            //    dfu_target_schedule_update(0);  // Schedule the update image 0 or reboot is enough
            // Reset the device TODO
            sys_reboot(SYS_REBOOT_COLD);
            return true;
        }    
    }
    else
    {
        LOG_WRN("Received unkown fota command");
        received_cmd = NO_SUPPORTED_FOTA_CMD;
        b_return = false;
    }
    read_cmd = received_cmd;
    return b_return;
}

/**@brief This function checks if there is a read AT command received through Zigbee pending to be replied, and, in that case
 *        schedules the function that will reply to that command.
 *
 * @retval True If the transmission of a reply to a read AT command has been scheduled
 * @retval False Otherwise
 */
void digi_fota_command_manager(void)
{
    if (b_pending_read_cmd)
    {
        b_pending_read_cmd = false;
        digi_fota_cmd_reply();
    }
}



/**@brief This function places in the APS output frame queue the reply to a read AT command received through Zigbee.
*
*/
bool digi_fota_cmd_reply(void)
{
    bool b_return = false;
    if (read_cmd >= NUMBER_OF_FOTA_COMMANDS)
    {
        LOG_ERR("Not supported command");
        return b_return;
    }
    switch(read_cmd)
    {
        case READ_FOTA_IMAGE_NOTIFY:
            LOG_WRN("FOTA command reply READ_FOTA_IMAGE_NOTIFY");
            if( zigbee_aps_get_output_frame_buffer_free_space() )
            {
                uint8_t i;
                aps_output_frame_t element;

                element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
                element.cluster_id = DIGI_FOTA_CLUSTER;
                element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
                element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
                i = 0;
                element.payload[i++] = QUERY_NEXT_IMAGE_REQUEST;    
                element.payload[i++] = NO_HW_VERSION;
                element.payload[i++] = 0x01;    // I don´t know what is this, but if I cahnge it does not work
                element.payload[i++] = FOTA_STATUS_SUCCESS;
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
                element.payload[i++] = DFU_TARGET_IMAGE_TYPE_MCUBOOT;
                element.payload[i++] = 0x00;   // file version different may not work neither
                element.payload[i++] = 0x0e;
                element.payload[i++] = 0x10;
                element.payload[i++] = 0x00;
                element.payload[i++] = 0x00;
                element.payload_size = (zb_uint8_t)i;
                LOG_WRN("FOTA command reply");
                if( enqueue_aps_frame(&element) ) b_return = true;
            }
            break;
        case BLOCK_REQUEST:
            LOG_WRN("FOTA command reply BLOCK_REQUEST");
            if( zigbee_aps_get_output_frame_buffer_free_space() )
            {
                uint8_t i;
                aps_output_frame_t element;

                element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
                element.cluster_id = DIGI_FOTA_CLUSTER;
                element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
                element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
                i = 0;
                element.payload[i++] = QUERY_NEXT_IMAGE_REQUEST;    
                element.payload[i++] = read_cmd_first_char +1;
                element.payload[i++] = IMAGE_NOTIFY; 
                element.payload[i++] = FOTA_STATUS_SUCCESS;
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
                element.payload[i++] = DFU_TARGET_IMAGE_TYPE_MCUBOOT;
                element.payload[i++] = 0x00;
                element.payload[i++] = (uint8_t)(FILE_VERSION & 0xFF);
                element.payload[i++] = (uint8_t)((FILE_VERSION >> 8) & 0xFF);
                element.payload[i++] = (uint8_t)((FILE_VERSION >> 16) & 0xFF);
                element.payload[i++] = (uint8_t)(FILE_VERSION >> 24); // High byte of file version
                element.payload[i++] = (uint8_t)(file_offset & 0xFF); // Low byte of file offset
                element.payload[i++] = (uint8_t)((file_offset >> 8) & 0xFF);
                element.payload[i++] = (uint8_t)((file_offset >> 16) & 0xFF);
                element.payload[i++] = (uint8_t)((file_offset >> 24) & 0xFF); // High byte of file offset
                element.payload[i++] = read_cmd_size;
                element.payload_size = (zb_uint8_t)i;
                LOG_WRN("FOTA command reply");
                if( enqueue_aps_frame(&element) ) b_return = true;
            }
            break;
        case LAST_BLOCK_REQUEST_COMMAND:
            LOG_WRN("FOTA command reply BLOCK_REQUEST");
            if( zigbee_aps_get_output_frame_buffer_free_space() )
            {
                uint8_t i;
                aps_output_frame_t element;

                element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
                element.cluster_id = DIGI_FOTA_CLUSTER;
                element.src_endpoint = DIGI_BINARY_VALUE_SOURCE_ENDPOINT;
                element.dst_endpoint = DIGI_BINARY_VALUE_DESTINATION_ENDPOINT;
                i = 0;
                element.payload[i++] = QUERY_NEXT_IMAGE_REQUEST;    
                element.payload[i++] = read_cmd_first_char + 1;
                element.payload[i++] = UPGRADE_AND_REQUEST; 
                element.payload[i++] = FOTA_STATUS_SUCCESS;
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE >> 8); // High byte of manufacturer code
                element.payload[i++] = (uint8_t)(MANUFACTORER_CODE & 0xFF); // Low byte of manufacturer code
                element.payload[i++] = DFU_TARGET_IMAGE_TYPE_NONE;
                element.payload[i++] = 0x00;
                element.payload[i++] = (uint8_t)(FILE_VERSION & 0xFF);
                element.payload[i++] = (uint8_t)((FILE_VERSION >> 8) & 0xFF);
                element.payload[i++] = (uint8_t)((FILE_VERSION >> 16) & 0xFF);
                element.payload[i++] = (uint8_t)(FILE_VERSION >> 24); // High byte of file version
                element.payload_size = (zb_uint8_t)i;
                LOG_WRN("FOTA command reply");
                if( enqueue_aps_frame(&element) ) b_return = true;
            }        
            break;  
        default:
            LOG_ERR("Not supported command");
            break;
    }
    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}