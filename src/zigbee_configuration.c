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

LOG_MODULE_REGISTER(zb_conf, LOG_LEVEL_DBG);
/* Local variables                                                            */
static struct zb_user_conf_t zb_user_conf; // zigbee user configuration

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
    rc = read_nvram_PAN_ID(panid, sizeof(panid));
    // Check if the PAN_ID was successfully read from NVRAM
    if (rc > 0) {
        // PAN_ID was successfully read from NVRAM
        // Convert the PAN_ID to a uint64_t and store it in the zb_user_conf structure
        zb_user_conf.extended_pan_id = *(uint64_t *)panid;
        // Log the PAN_ID read from NVRAM
        LOG_HEXDUMP_DBG(&zb_user_conf.extended_pan_id,sizeof(zb_user_conf.extended_pan_id),"Extended PAN ID: ");
    } 
    else 
    {
        // Failed to read the PAN_ID from NVRAM
        LOG_ERR("Error reading PAN ID\n");
    }

    // Read the Node Identifier (NI) from NVRAM
    rc = read_nvram_NI(zb_user_conf.at_ni, sizeof(zb_user_conf.at_ni));
    // Check if the Node Identifier was successfully read from NVRAM
    if (rc > 0) {
        // Node Identifier was successfully read from NVRAM
        LOG_INF("Node Identifier: %s\n", zb_user_conf.at_ni);
    } 
    else 
    {
        // Failed to read the Node Identifier from NVRAM
        LOG_ERR("Error reading Node Identifier\n");
    }

    return rc;

}

//------------------------------------------------------------------------------
/**@brief Write zigbee user configuration to NVRAM
 *
 * This function writes the current Zigbee user configuration to NVRAM.
 * It writes both the PAN_ID and the Node Identifier (NI).
 */
void zb_conf_copy_to_nvram (void)
{
    // Write the PAN_ID to NVRAM
    write_nvram_PAN_ID(&zb_user_conf.extended_pan_id, sizeof(zb_user_conf.extended_pan_id));

    // Write the Node Identifier (NI) to NVRAM
    write_nvram_NI(zb_user_conf.at_ni, sizeof(zb_user_conf.at_ni));
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
/**@brief Get the used configurable parameter "node identifier".
 *        It gets stored in the buffer passed as argument
 *
 * @param  ni  Pointer to buffer where the node identifier will be stored.
 */
void zb_conf_get_extended_node_identifier (uint8_t *ni)
{
    uint8_t i;
    for(i=0; i < MAXIMUM_SIZE_NODE_IDENTIFIER; i++)
    {
        ni[i] = zb_user_conf.at_ni[i];
        if( ni[i] == '\0' ) break;
    }
    if( ni[i] != '\0' ) ni[i+1] = '\0';
}
