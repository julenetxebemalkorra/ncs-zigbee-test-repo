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

LOG_MODULE_REGISTER(zb_conf, LOG_LEVEL_DBG);
/* Local variables                                                            */
static struct zb_user_conf_t zb_user_conf; // zigbee user configuration

/* Function definition                                                        */


//------------------------------------------------------------------------------
/**@brief Load zigbee user configuration from NVRAM
 *
 *
 */
void zb_conf_read_from_nvram (void) // TODO
{
    zb_user_conf.extended_pan_id = 0x000000000000abce;
    zb_user_conf.at_ni[0] = ' ';
    zb_user_conf.at_ni[1] = '\r';
}

//------------------------------------------------------------------------------
/**@brief Write zigbee user configuration to NVRAM
 *
 *
 */
void zb_conf_copy_to_nvram (void) // TODO
{

}

//------------------------------------------------------------------------------
/**@brief Update the zigbee user configuration structure with the last values 
 *        introduced by the user.
 *
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
