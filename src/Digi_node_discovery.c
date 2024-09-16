/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of replies to the Node Discovery command from Digi.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>


#include <zboss_api.h>

#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_At_commands.h"
#include "Digi_node_discovery.h"
#include "zigbee_configuration.h"


/* Local variables                                                            */
static struct node_discovery_reply_t node_discovery_reply;

LOG_MODULE_REGISTER(Digi_node_discovery, LOG_LEVEL_DBG);

/**@brief This function initializes the Digi_node_discovery's firmware module
 *
 */
void digi_node_discovery_init(void)
{
    node_discovery_reply.b_pending_request = false;
    node_discovery_reply.first_character = 0;
    node_discovery_reply.max_reply_time_ms = 0;
    node_discovery_reply.time_request_ms = 0;
    node_discovery_reply.time_reply_ms = 0;

    zb_conf_get_extended_node_identifier(node_discovery_reply.at_ni);

}

/**@brief This function evaluates if the last received APS frame is a Digi's Node Discover request
 *
 * @param[in]   input_data   Pointer to payload of received APS frame
 * @param[in]   size_of_input_data   Payload size
 *
 * @retval True It is a Digi's Node Discover request
 * @retval False Otherwise
 */
bool is_a_digi_node_discovery_request(uint8_t* input_data, int16_t size_of_input_data)
{
    bool b_return = false;
    if( size_of_input_data >= 12 ) // TODO. I don't know if the size of this frame is constant. I know it should have at least 12 chars
    {
        if( ( input_data[10] == 'N' ) && ( input_data[11] == 'D' ) ) // ND command
        {
            if( input_data[1] >= 32 ) // Minimum valid discovery timeout
            {
                b_return = true;
                node_discovery_reply.b_pending_request = true;
                node_discovery_reply.first_character = input_data[0];
                node_discovery_reply.max_reply_time_ms = (input_data[1] + 10) * 100;
                node_discovery_reply.time_request_ms = k_uptime_get();
                node_discovery_reply.time_reply_ms = node_discovery_reply.time_request_ms + node_discovery_reply.max_reply_time_ms/2; //TODO. It should be random, so not all TSC reply at the same time
            }
        }
    }
    return b_return;
}

/**@brief This function places in the APS output frame queue the reply to a node discovery request.
*
*/
bool digi_node_discovery_reply(void)
{
    bool b_return = false;
    
    if( zigbee_aps_get_output_frame_buffer_free_space() )
    {
        uint8_t i,j;
        aps_output_frame_t element;
        zb_ieee_addr_t zb_long_address; // MAC address of this node

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.cluster_id = DIGI_COMMISSIONING_REPLY_CLUSTER;
        element.src_endpoint = DIGI_COMMISSIONING_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_COMMISSIONING_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = (zb_uint8_t)node_discovery_reply.first_character; //First character of the Node Discovery request
        element.payload[i++] = 'N'; //ND, node discovery
        element.payload[i++] = 'D';
        element.payload[i++] = 0;
        element.payload[i++] = (zb_uint8_t)((uint16_t)(zb_get_short_address())>>8); //Short address, high byte
        element.payload[i++] = (zb_uint8_t)(zb_get_short_address());//Short address, low byte

        zb_get_long_address(zb_long_address); //MAC address of device
        for(j = 0; j<8; j++)
        {
            element.payload[i++] = zb_long_address[7-j];
        }       
        j = 0;
        while(j < 32)
        {
            if( node_discovery_reply.at_ni[j] == 0 ) break; //Null character found
            else
            {
                element.payload[i++] = node_discovery_reply.at_ni[j]; //Node identifier string
                j++;
            }
        }
        element.payload[i++] = 0;
        element.payload[i++] = 0xFF; //Parent address of node (always 0xFFFE for routers)
        element.payload[i++] = 0xFE;
        element.payload[i++] = 0x01; // Node type = 1 (router)
        element.payload[i++] = 0;
        element.payload[i++] = 0xC1; // Profile ID = 0xC105;
        element.payload[i++] = 0x05;
        element.payload[i++] = 0x10; // Manufacturer ID = 0x101E (We will start using a value from DIGI's Xbees)
        element.payload[i++] = 0x1E;
        element.payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 24) & 0xFF; // Product type
        element.payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 16) & 0xFF;
        element.payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 8) & 0xFF;
        element.payload[i++] = (uint32_t)(PRODUCT_TYPE) & 0xFF;
        element.payload[i++] = ((uint16_t)(MANUFACTURED_ID) >> 8) & 0xFF; // Manufactured ID
        element.payload[i++] = (uint16_t)(MANUFACTURED_ID) & 0xFF;
        element.payload[i++] = 0x2e; // TODO. Not sure what it means. It represents the RSSI or a related magnitude.
        element.payload_size = (zb_uint8_t)i;
        if( enqueue_aps_frame(&element) ) b_return = true;
    }

    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}

/**@brief This function checks if there is a node discovery request pending to be replied, and, in that case
 *        schedules the function that will reply to that request.
 *
 * @retval True If the transmission of a reply to a Node Discover request has been scheduled
 * @retval False Otherwise
 */
void digi_node_discovery_request_manager(void)
{
    if( node_discovery_reply.b_pending_request )
    {
        uint64_t time_now_ms = k_uptime_get();
        if( time_now_ms >= node_discovery_reply.time_reply_ms )
        {
            node_discovery_reply.b_pending_request = false;
            digi_node_discovery_reply();
        }
    }
}
