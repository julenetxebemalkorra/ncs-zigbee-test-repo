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

#include <zboss_api.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>

#include <string.h>


#include "Digi_profile.h"
#include "Digi_At_commands.h"
#include "Digi_node_discovery.h"

static struct node_discovery_reply_t node_discovery_reply;

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
    node_discovery_reply.at_ni[0] = 'F';
    node_discovery_reply.at_ni[1] = 'A';
    node_discovery_reply.at_ni[2] = 'N';
    node_discovery_reply.at_ni[3] = 'S';
    node_discovery_reply.at_ni[4] = 'T';
    node_discovery_reply.at_ni[5] = 'E';
    node_discovery_reply.at_ni[6] = 'L';
    node_discovery_reply.at_ni[7] = '_';
    node_discovery_reply.at_ni[8] = '1';
    node_discovery_reply.at_ni[9] = '3';
    node_discovery_reply.at_ni[10] = '5';
    node_discovery_reply.at_ni[11] = 0;
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

/**@brief This function generates the APS frame to reply to a node discovery request and schedules its transmission.
*
 * @retval True if the transmission could be scehduled
 * @retval False Otherwise
 */
bool digi_node_discovery_reply(void)
{
    bool b_return = false;
    zb_uint8_t payload[DIGI_NODE_DISCOVERY_REPLY_PAYLOAD_SIZE_MAX];
    zb_ieee_addr_t zb_long_address; // MAC address of this node
    zb_addr_u dst_addr; // Address of destination Node
    uint8_t i, j;
    zb_bufid_t bufid = NULL;
    
    bufid = zb_buf_get_out(); // Allocate output buffer

/*  If buffer could be allocated, generate APS frame and schedule its transmission */
    if( bufid )
    {
        i = 0;
        payload[i++] = (zb_uint8_t)node_discovery_reply.first_character; //First character of the Node Discovery request
        payload[i++] = 'N'; //ND, node discovery
        payload[i++] = 'D';
        payload[i++] = 0;
        payload[i++] = (zb_uint8_t)((uint16_t)(zb_get_short_address())>>8); //Short address, high byte
        payload[i++] = (zb_uint8_t)(zb_get_short_address());//Short address, low byte

        zb_get_long_address(zb_long_address); //MAC address of device
        for(uint8_t j = 0; j<8; j++)
        {
            payload[i++] = zb_long_address[7-j];
        }
        
        j = 0;
        while(j < 32)
        {
            if( node_discovery_reply.at_ni[j] == 0 ) break; //Null character found
            else
            {
                payload[i++] = node_discovery_reply.at_ni[j]; //Node identifier string
                j++;
            }
        }
        payload[i++] = 0;
        payload[i++] = 0xFF; //Parent address of node (always 0xFFFE for routers)
        payload[i++] = 0xFE;
        payload[i++] = 0x01; // Node type = 1 (router)
        payload[i++] = 0;
        payload[i++] = 0xC1; // Profile ID = 0xC105;
        payload[i++] = 0x05;
        payload[i++] = 0x10; // Manufacturer ID = 0x101E (We will start using a value from DIGI's Xbees)
        payload[i++] = 0x1E;
        payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 24) & 0xFF; // Product type
        payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 16) & 0xFF;
        payload[i++] = ((uint32_t)(PRODUCT_TYPE) >> 8) & 0xFF;
        payload[i++] = (uint32_t)(PRODUCT_TYPE) & 0xFF;
        payload[i++] = ((uint16_t)(MANUFACTURED_ID) >> 8) & 0xFF; // Manufactured ID
        payload[i++] = (uint16_t)(MANUFACTURED_ID) & 0xFF;
        payload[i++] = 0x2e; // TODO. Not sure what it means. It represents the RSSI or a related magnitude.

        dst_addr.addr_short = 0x0000; // Destination address of the coordinator

        zb_ret_t ret = zb_aps_send_user_payload(bufid,
                                                dst_addr,
                                                DIGI_PROFILE_ID,
                                                DIGI_COMMISSIONING_REPLY_CLUSTER,
                                                DIGI_COMMISSIONING_SOURCE_ENDPOINT,
                                                DIGI_COMMISSIONING_DESTINATION_ENDPOINT,
                                                ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                                ZB_TRUE,
                                                payload,
                                                (size_t)i);
        if(ret == RET_OK) b_return = true;
    }
    return b_return;
}

/**@brief This function checks if there is a node discovery request pending to be replied, and, in that case
 *        schedules the function that will reply to that request.
 *
 * @retval True If the transmission of a reply to a Node Discover request has been scheduled
 * @retval False Otherwise
 */
bool digi_node_discovery_request_manager(void)
{
    bool b_return = false;
    if( node_discovery_reply.b_pending_request )
    {
        uint64_t time_now_ms = k_uptime_get();
        if( time_now_ms >= node_discovery_reply.time_reply_ms )
        {
            node_discovery_reply.b_pending_request = false;
            if( digi_node_discovery_reply() ) return true;
        }
    }
    return b_return;
}
