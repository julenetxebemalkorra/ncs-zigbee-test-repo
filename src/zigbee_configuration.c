/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Management of user configurable zigbee parameters.
 */

#include <zephyr/logging/log.h>
#include <zboss_api.h>

#include "zigbee_configuration.h"
#include "Digi_At_commands.h"
#include "nvram.h"
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(zb_conf, LOG_LEVEL_DBG);
/* Local variables                                                            */
static struct zb_user_conf_t zb_user_conf; // zigbee user configuration


enum nvram_status_t status = NVRAM_WRONG_DATA;

bool g_b_flash_write_cmd = false; //   Flag to indicate that a write command has been received
bool g_b_reset_zigbee_cmd = false; // Flag to indicate that a reset command has been received
bool g_b_reset_cmd = false; // Flag to indicate that a reset command has been received
bool g_b_nvram_write_done = true; // Flag to indicate that the NVRAM write operation has been completed


/* Function definition                                                        */
//------------------------------------------------------------------------------
/**@brief Check if the NVRAM is being used for the first time
 *
 * This function checks if the NVRAM is being used for the first time.
 * It reads the first 6 bytes of the NVRAM and compares them with the expected values.
 * If the values are correct, it means that the NVRAM is being used for the first time.
 * In this case, the function writes the expected values to the NVRAM.
 *
 * @return The function returns a int8_t value. If the NVRAM is being used for the first time, 
 * it returns 0. If the NVRAM is not being used for the first time, it returns a negative error code.
 */
int8_t zb_nvram_check_usage(void)
{
    // Buffer to store the first 6 bytes of the NVRAM
    int8_t rc = 0;
    uint8_t nvram_first_id[6];
    uint8_t nvram_first_id_expected[6];
    uint8_t number_restarts[1] = {0};
    uint8_t reset_reason[1] = {0};

    // Expected values for the first 6 bytes of the NVRAM
    nvram_first_id_expected[0] = 0xAA;
    nvram_first_id_expected[1] = 0xBB;
    nvram_first_id_expected[2] = 0xCC;
    nvram_first_id_expected[3] = 0xDD;
    nvram_first_id_expected[4] = 0xEE;
    nvram_first_id_expected[5] = 0xFF;

    // Read the first 6 bytes of the NVRAM
    rc = read_nvram(ZB_NVRAM_CHECK_ID, nvram_first_id, sizeof(nvram_first_id));
    // Check if the first 6 bytes of the NVRAM were successfully read
    if (rc != sizeof(nvram_first_id)) 
    {
        LOG_WRN("NVRAM first id is missing, write the default values");
        zb_user_conf.extended_pan_id = 0x0000000000000000;
        zb_user_conf.at_ni[0] = ' ';
        zb_user_conf.at_ni[1] = 0;
        uint8_t network_link_key[16] = {0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39};  // defalut Zigbee Alliance key
        memcpy(zb_user_conf.network_link_key, network_link_key, sizeof(network_link_key));
        write_nvram(ZB_NVRAM_CHECK_ID, nvram_first_id_expected, sizeof(nvram_first_id_expected));
        write_nvram(RBT_CNT_ID, number_restarts, sizeof(number_restarts));
        write_nvram(RBT_CNT_REASON, reset_reason, sizeof(reset_reason));
        return NVRAM_NOT_WRITTEN;
    }
    else if (rc == sizeof(nvram_first_id))
    {
        if(nvram_first_id[0] == 0xAA && nvram_first_id[1] == 0xBB && nvram_first_id[2] == 0xCC && nvram_first_id[3] == 0xDD && nvram_first_id[4] == 0xEE && nvram_first_id[5] == 0xFF)
        {
            LOG_INF("NVRAM first id is correct");
            return SUCCESS;
        }
        else
        {
            LOG_WRN("NVRAM read data is not correct, write again and work with the default configuration");
            /* Get the device pointer of the UART hardware */
            #ifdef CONFIG_PAN1770EVB
                zb_user_conf.extended_pan_id = 0x0000000000000000;
                const char *node_identifier = "pan1780_dk";
                strncpy(zb_user_conf.at_ni, node_identifier, sizeof(zb_user_conf.at_ni) - 1);
                zb_user_conf.at_ni[sizeof(zb_user_conf.at_ni) - 1] = '\0';
                uint8_t network_link_key[16] = {0x88, 0x88, 0x99, 0x99, 0x88, 0x89, 0x45, 0x67, 0xAC, 0xCD, 0xA7, 0xA7, 0x66, 0xD3, 0xD3, 0xD6};   // custom Zigbee key
                memcpy(zb_user_conf.network_link_key, network_link_key, sizeof(network_link_key));
                write_nvram(ZB_NVRAM_CHECK_ID, nvram_first_id_expected, sizeof(nvram_first_id_expected));
                return NVRAM_WRONG_DATA;            
            #else
                zb_user_conf.extended_pan_id = 0x0000000000000000;
                zb_user_conf.at_ni[0] = ' ';
                zb_user_conf.at_ni[1] = 0;
                uint8_t network_link_key[16] = {0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39};   // defalut Zigbee Alliance key
                memcpy(zb_user_conf.network_link_key, network_link_key, sizeof(network_link_key));
                write_nvram(ZB_NVRAM_CHECK_ID, nvram_first_id_expected, sizeof(nvram_first_id_expected));
                return NVRAM_WRONG_DATA;
            #endif
        }
    }
    else
    {
        LOG_ERR("Unkwon error reading NVRAM first id");
        return NVRAM_UNKNOWN_ERR;
    }
}

//------------------------------------------------------------------------------
/**@brief Load zigbee user configuration from NVRAM
 *
 * This function reads the Zigbee user configuration from NVRAM.
 * It reads the PAN_ID and stores it in the zb_user_conf structure.
 *
 * @return The function returns a uint8_t value. If the read operation was successful, 
 * it returns the number of bytes read. If the read operation failed, it returns an error code.
 */
 
uint8_t zb_conf_read_from_nvram (void)
{
    uint8_t panid[8];   // Buffer to store the PAN_ID read from NVRAM
    uint8_t rc;         // Return code from the read operation

    // Read the PAN_ID from NVRAM
    rc = read_nvram(ZB_EXT_PANID, panid, sizeof(panid));
    // Check if the PAN_ID was successfully read from NVRAM
    if (rc == sizeof(panid)) {
        // PAN_ID was successfully read from NVRAM
        // Convert the PAN_ID to a uint64_t and store it in the zb_user_conf structure
        zb_user_conf.extended_pan_id = *(uint64_t *)panid;
        // Log the PAN_ID read from NVRAM
        LOG_HEXDUMP_DBG(&zb_user_conf.extended_pan_id,sizeof(zb_user_conf.extended_pan_id),"Extended PAN ID: ");
    } 
    else 
    {
        // Failed to read the PAN_ID from NVRAM
        LOG_ERR("Error reading PAN ID");
        return NVRAM_ERROR_READING;
    }

    // Read the Node Identifier (NI) from NVRAM
    rc = read_nvram(ZB_NODE_IDENTIFIER, zb_user_conf.at_ni, sizeof(zb_user_conf.at_ni));
    // Check if the Node Identifier was successfully read from NVRAM
    if (rc == sizeof(zb_user_conf.at_ni)) {
        // Node Identifier was successfully read from NVRAM
        LOG_INF("Node Identifier: %s", zb_user_conf.at_ni);
    } 
    else 
    {
        // Failed to read the Node Identifier from NVRAM
        LOG_ERR("Error reading Node Identifier");
        return NVRAM_ERROR_READING;
    }

    // Read the network key from NVRAM
    rc = read_nvram(ZB_NETWORK_ENCRYPTION_KEY, zb_user_conf.network_link_key, sizeof(zb_user_conf.network_link_key));
    // Check if the network key was successfully read from NVRAM
    if (rc == sizeof(zb_user_conf.network_link_key)) {
        // Network key was successfully read from NVRAM
        //LOG_HEXDUMP_DBG(zb_user_conf.network_link_key,sizeof(zb_user_conf.network_link_key),"Network Key: ");
        LOG_WRN("JULEN Network Key: %s", zb_user_conf.network_link_key);
    } 
    else 
    {
        // Failed to read the network key from NVRAM
        LOG_ERR("Error reading Network Key");
        LOG_ERR("rc: %d", rc);
        return NVRAM_ERROR_READING;
    }

    uint32_t stored_checksum = 0;
    uint32_t calculated_checksum = calculate_checksum(&zb_user_conf, sizeof(zb_user_conf));

    rc = read_nvram(ZB_CHECKSUM, &stored_checksum, sizeof(stored_checksum));
    if (rc == sizeof(stored_checksum)) {
        if (calculated_checksum != stored_checksum) {
            LOG_ERR("Checksum does not match");
            LOG_WRN(" stored_checksum: %d", stored_checksum);
            LOG_WRN(" calculated_checksum: %d", calculated_checksum);
            return NVRAM_WRONG_DATA;
        }
    } 
    else 
    {
        LOG_ERR("Error reading checksum");
        return NVRAM_ERROR_READING;
    }

    return SUCCESS;
}

//------------------------------------------------------------------------------
/**@brief Write zigbee user configuration to NVRAM
 *
 * This function writes the current Zigbee user configuration to NVRAM.
 * It writes both the PAN_ID and the Node Identifier (NI).
 */
void zb_conf_write_to_nvram (void)
{
    g_b_nvram_write_done = false;
    // Calculate checksum
    uint32_t checksum = calculate_checksum(&zb_user_conf, sizeof(zb_user_conf));
    
    // Write the PAN_ID to NVRAM
    write_nvram(ZB_EXT_PANID, &zb_user_conf.extended_pan_id, sizeof(zb_user_conf.extended_pan_id));

    // Write the Node Identifier (NI) to NVRAM
    write_nvram(ZB_NODE_IDENTIFIER, &zb_user_conf.at_ni, sizeof(zb_user_conf.at_ni));

    // Write the network key to NVRAM
    write_nvram(ZB_NETWORK_ENCRYPTION_KEY, &zb_user_conf.network_link_key, sizeof(zb_user_conf.network_link_key));
    LOG_WRN("JULEN Network Key: %s", zb_user_conf.network_link_key);

    write_nvram(ZB_CHECKSUM, &checksum, sizeof(checksum));

    LOG_WRN("Zigbee configuration written to NVRAM");
    LOG_WRN(" Checksum: %d", checksum);

    g_b_nvram_write_done = true;
    g_b_reset_zigbee_cmd = true;
}

//------------------------------------------------------------------------------
/**@brief Update the zigbee user configuration structure with the last values 
 *        introduced by the user.
 *
 * This function updates the Zigbee user configuration structure with the latest
 * values provided by the user. It updates both the PAN_ID and the Node Identifier (NI).
 */
void zb_conf_update (void)
{
    zb_user_conf.extended_pan_id = digi_at_get_parameter_id();
    digi_at_get_parameter_ni(&zb_user_conf.at_ni[0]);
    digi_at_get_parameter_ky(&zb_user_conf.network_link_key[0]);
    LOG_WRN("Updating Zigbee configuration");
    LOG_WRN("Extended PAN ID: %llx", zb_user_conf.extended_pan_id);
    LOG_WRN("Node Identifier: %s", zb_user_conf.at_ni);
    LOG_WRN("Network Key: ");
    LOG_HEXDUMP_DBG(zb_user_conf.network_link_key, sizeof(zb_user_conf.network_link_key), " ");
}

//------------------------------------------------------------------------------
/**@brief Get the used configurable parameter "extended pan id"
 *
 * @retval User configured extended pan id
 */
uint64_t zb_conf_get_extended_pan_id (void)
{
    return zb_user_conf.extended_pan_id;
}

//------------------------------------------------------------------------------
/**@brief Get the used configurable parameter network key
 *
 * @retval User configured network key
 */
void zb_conf_get_network_link_key (uint8_t *network_key)
{
    for(uint8_t i = 0; i < MAXIMUM_SIZE_LINK_KEY; i++)
    {
        network_key[i] = zb_user_conf.network_link_key[i];
    }
}

//------------------------------------------------------------------------------
/**@brief Invert the order of bytes in a 32-bit value 
 *      (e.g., 0x12345678 becomes 0x78563412)
 * 
 * @param value The 32-bit value to invert
*/
uint32_t invert_bytes(uint32_t value) {
    uint32_t result = 0;
    result |= (value & 0xFF000000) >> 24;
    result |= (value & 0x00FF0000) >> 8;
    result |= (value & 0x0000FF00) << 8;
    result |= (value & 0x000000FF) << 24;
    return result;
}

//------------------------------------------------------------------------------
/**@brief Get the used configurable parameter "mac address"
 *
 * @retval User configured extended pan id
 */
uint32_t zb_get_mac_addr_low (void)
{
        zb_ieee_addr_t zb_long_address; // MAC address of this node
        // Retrieve and display MAC address
        zb_get_long_address(zb_long_address);

        //Display MAC address
        uint8_t temp[8];
        uint32_t low = 0;
        
        for(uint8_t i = 0; i<8; i++)
        {
            temp[i] = zb_long_address[7-i];
        }

        // Convert extended PAN ID to short address
        for (uint8_t i=0; i<4; i++) {
            low = (low << 8) | zb_long_address[i];
        }

        low = invert_bytes(low);

        return low;
}

//------------------------------------------------------------------------------
/**@brief Get the used configurable parameter "mac address"
 *
 * @retval User configured extended pan id
 */
uint32_t zb_get_mac_addr_high (void)
{
        zb_ieee_addr_t zb_long_address;
        // Retrieve and display MAC address
        zb_get_long_address(zb_long_address);

        //Display MAC address
        uint8_t temp[8];
        uint32_t high = 0;
        for(uint8_t i = 0; i<8; i++)
        {
            temp[i] = zb_long_address[i];
        }

        // Convert extended PAN ID to short address
        for (uint8_t i=0; i<4; i++) {
            high = (high << 8) | zb_long_address[i +4];
        }

        // Invert the order of bits in high
        high = invert_bytes(high);

        return high;
}

//------------------------------------------------------------------------------
/**@brief Get the used configurable parameter "node identifier".
 *        It gets stored in the buffer passed as argument
 *
 * @param  ni  Pointer to buffer where the node identifier will be stored.
 *
 * @retval Size of node identifier string
 */
uint8_t zb_conf_get_extended_node_identifier (uint8_t *ni)
{
    uint8_t i;
    for(i=0; i < MAXIMUM_SIZE_NODE_IDENTIFIER; i++)
    {
        ni[i] = zb_user_conf.at_ni[i];
        if( ni[i] == '\0' ) break;
    }
    if( ni[i] != '\0' ) ni[i] = '\0';
    LOG_WRN("Node Identifier: %s", ni);
    return(i);
}

//------------------------------------------------------------------------------
/** @brief Calculate checksum for given data
 * 
 * @param data A pointer to the data to calculate the checksum.
 * @param size The size of the data to calculate the checksum.
*/
// Calculate checksum for given data
uint32_t calculate_checksum(char* data, int size) {
    uint32_t checksum = 0;
    for (int i = 0; i < size; i++) {
        checksum += (uint32_t)data[i];
    }
    return checksum;
}

void nvram_manager(void)
{
    if((!g_b_flash_error) && (g_b_flash_write_cmd))
    {
        LOG_WRN("Flash write command received");
        zb_conf_update(); // Update the values in the zb_user_conf structure
        zb_conf_write_to_nvram(); // Write the new values to NVRAM
        g_b_flash_write_cmd = false;
    }     
}

void zigbee_thread_manager(void)
{
    // The flag g_b_reset_zigbee_cmd realices the reset of the zigbee stack
    // and the g_b_reset_cmd realices the reset of the whole system
    if(g_b_reset_zigbee_cmd)
    {
        g_b_reset_zigbee_cmd = false;
        int ret = ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);
        if(ret != RET_OK)
        {
            LOG_ERR("zb_bdb_reset_via_local_action failed, ret %d", ret);
        }
        LOG_WRN("Zigbee reset");
    }

    if (g_b_reset_cmd)
    {
        g_b_reset_cmd = false;
        LOG_WRN("Reset command received from TCU, rebooting...");
        zb_reset(true);
    }
}