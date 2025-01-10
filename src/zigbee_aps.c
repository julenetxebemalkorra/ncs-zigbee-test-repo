/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Generation and reception of APS frames.
 */

#include <zephyr/logging/log.h>

#include <zboss_api.h>
#include <zigbee/zigbee_error_handler.h>
#include "zigbee_aps.h"
#include "Digi_profile.h"

#define SCHEDULING_CB_TIMEOUT_MS 50000 // Tiempo límite en milisegundos para enviar un frame APS
#define SYSTEM_TICK_MS 1              // Tiempo de tick del sistema en milisegundos

/* Local variables                                                            */
static aps_output_frame_circular_buffer_t aps_output_frame_buffer;
static bool b_scheduling_cb_pending = false;
static uint32_t scheduling_cb_timer = 0;

LOG_MODULE_REGISTER(zigbee_aps, LOG_LEVEL_DBG);

/* Function definition                                                        */

//------------------------------------------------------------------------------
/**@brief Initialization of Zigbee_aps firmware module
 *
 *
 */
void zigbee_aps_init(void)
{
    init_aps_output_frame_buffer();
}

//------------------------------------------------------------------------------
/**@brief Initialization of aps output frame circular buffer.
 *
 *
 */
void init_aps_output_frame_buffer(void)
{
    aps_output_frame_buffer.head = 0;
    aps_output_frame_buffer.tail = 0;
    aps_output_frame_buffer.free_space = APS_OUTPUT_FRAME_BUFFER_SIZE;
    aps_output_frame_buffer.used_space = 0;
}

// -----------------------------------------------------------------------------
/**@brief Function to get the free space of the aps output frame buffer.
 *
 * @return  The free space of the aps output frame buffer.
 */
void check_scheduling_cb_timeout(void)
{
    if (b_scheduling_cb_pending && scheduling_cb_timer > 0)
    {
        scheduling_cb_timer -= SYSTEM_TICK_MS; // Decrementa cada 10KHz
        if (scheduling_cb_timer <= 0)
        {
            b_scheduling_cb_pending = false;
            LOG_DBG("Scheduling callback flag reset after timeout");
        }
    }
}

//------------------------------------------------------------------------------
/**@brief Callback function executed when APS frame transmission is completed.
 *        The buffer is released.
 *        A log message showing APS, NWK and MAC counters is printed.
 *
 * @param   bufid   Reference to the Zigbee stack buffer used to transmit the APS frame
 *
 * @note The callback function will be executed when the transmission process is considered completed,
 *       even if it was not succesful.
 *       The transmission process is considered completed when the APS ACK is received (transmission succesful)
 *       or when the maximum number of attempts has been reached.
 *       If there is not ACK at APS layer it will retransmit once, and if there is not ACK at MAC level,
 *       it will retransmit up to three times. But the callback function is not executed on every attempt,
 *       it is executed only when the process is considered completed.
 *
 */
void zigbee_aps_user_data_tx_cb(zb_bufid_t bufid)
{
    if( bufid )
    {
        zb_uint8_t *pointerToBeginOfBuffer;
        zb_uint8_t aps_counter;
        zb_uint8_t nwk_sequence_number;
        zb_uint8_t mac_sequence_number;

        pointerToBeginOfBuffer = zb_buf_begin(bufid);
        pointerToBeginOfBuffer = pointerToBeginOfBuffer - 17;

        mac_sequence_number = pointerToBeginOfBuffer[2];
        nwk_sequence_number = pointerToBeginOfBuffer[16];
        aps_counter = pointerToBeginOfBuffer[24];
        LOG_DBG("Transmission completed, MAC seq = %d, NWK seq = %d, APS counter = %d", mac_sequence_number, nwk_sequence_number, aps_counter);

        // safe way to free buffer
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    }
}

//------------------------------------------------------------------------------
/**@brief Add new element to circular buffer used to store pending aps output frames.
 *
 * @param  element Pointer to struct containing new element to be added
 *
 * @retval true The new element could be added
 * @retval false The new element could not be added (buffer is full or size of payload too big)
 */
bool enqueue_aps_frame(aps_output_frame_t *element)
{
    if( aps_output_frame_buffer.free_space > 0 )
    {
        if( element->payload_size <= APS_UNENCRYPTED_PAYLOAD_MAX )
        {
            aps_output_frame_buffer.data[aps_output_frame_buffer.head].dst_addr = element->dst_addr;
            aps_output_frame_buffer.data[aps_output_frame_buffer.head].cluster_id = element->cluster_id;
            aps_output_frame_buffer.data[aps_output_frame_buffer.head].dst_endpoint = element->dst_endpoint;
            aps_output_frame_buffer.data[aps_output_frame_buffer.head].src_endpoint = element->src_endpoint;
            aps_output_frame_buffer.data[aps_output_frame_buffer.head].payload_size = element->payload_size;            
            for( uint8_t i = 0; i < element->payload_size; i++ )
            {
                aps_output_frame_buffer.data[aps_output_frame_buffer.head].payload[i] = element->payload[i];
            }
            aps_output_frame_buffer.head = aps_output_frame_buffer.head + 1;
            if( aps_output_frame_buffer.head >= APS_OUTPUT_FRAME_BUFFER_SIZE ) aps_output_frame_buffer.head = 0;
            aps_output_frame_buffer.used_space = aps_output_frame_buffer.used_space + 1;
            aps_output_frame_buffer.free_space = aps_output_frame_buffer.free_space - 1;
            return true;
        }
        else
        {
            LOG_ERR("Payload size too big to be sent in a single frame %d", element->payload_size);
            return false;
        }
    }
    else
    {
        LOG_ERR("Not free space of aps output frame queue");
        return false;
    }
}

//------------------------------------------------------------------------------
/**@brief extract element from circular buffer used to store pending aps output frames.
 *
 * @param   element Pointer to struct where extracted element will be stored.
 *
 * @retval true The element could be extracted.
 * @retval false The new element could not be extracted (buffer is empty).
 */
bool dequeue_aps_frame(aps_output_frame_t *element)
{
    if( aps_output_frame_buffer.used_space > 0 )
    {
        element->dst_addr = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].dst_addr;
        element->cluster_id = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].cluster_id;
        element->dst_endpoint = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].dst_endpoint;
        element->src_endpoint = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].src_endpoint;
        element->payload_size = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].payload_size;
        if( element->payload_size > APS_UNENCRYPTED_PAYLOAD_MAX ) element->payload_size = APS_UNENCRYPTED_PAYLOAD_MAX;
        for( uint8_t i = 0; i < element->payload_size; i++ )
        {
            element->payload[i] = aps_output_frame_buffer.data[aps_output_frame_buffer.tail].payload[i];
        }
        aps_output_frame_buffer.tail = aps_output_frame_buffer.tail + 1;
        if( aps_output_frame_buffer.tail >= APS_OUTPUT_FRAME_BUFFER_SIZE ) aps_output_frame_buffer.tail = 0;
        aps_output_frame_buffer.used_space = aps_output_frame_buffer.used_space - 1;
        aps_output_frame_buffer.free_space = aps_output_frame_buffer.free_space + 1;
        return true;
    }
    else
    {
        return false;
    }
}

//------------------------------------------------------------------------------
/**@brief Return the number of free positions on the aps output frame circular buffer.
 *
 * @retval Number of free positions on the aps output frame circular buffer.
 */
uint16_t zigbee_aps_get_output_frame_buffer_free_space(void)
{
    return aps_output_frame_buffer.free_space;
}

//------------------------------------------------------------------------------
/**@brief Generation and scheduling on the first APS frame of the queue
 *
 *
 */
void zigbee_aps_frame_scheduling_cb(zb_uint8_t param)
{
    ZVUNUSED(param);
    zb_ret_t zb_err_code;

    if (zb_buf_is_oom_state()) {
        // Handle Out Of Memory state
        LOG_ERR("Buffer pool is out of memory!\n");
        return;
    }
    if (zb_buf_memory_low()) 
    {
        // Handle low memory state
        LOG_WRN("Warning: Buffer pool memory is running low!\n");
        return;
    }

    // Trace the buffer statistics for debugging purposes
    zb_buf_oom_trace();

    zb_bufid_t bufid = zb_buf_get_out();

    if( bufid ) // If buffer could be allocated, generate frame and schedule the transmission
    {
        aps_output_frame_t aps_frame;
        if( dequeue_aps_frame(&aps_frame) ) // First pending frame from the queue
        {
            zb_ret_t ret = zb_aps_send_user_payload(bufid,
                                                    aps_frame.dst_addr,
                                                    DIGI_PROFILE_ID,
                                                    aps_frame.cluster_id,
                                                    aps_frame.src_endpoint,
                                                    aps_frame.dst_endpoint,
                                                    ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                                    ZB_TRUE,
                                                    aps_frame.payload,
                                                    aps_frame.payload_size);
            if(ret == RET_OK) LOG_WRN("Scheduled APS Frame with cluster 0x%x and payload %d bytes", aps_frame.cluster_id, (uint16_t)aps_frame.payload_size);
            else if(ret == RET_INVALID_PARAMETER_1) LOG_ERR("Transmission could not be scheduled: The buffer is invalid");
            else if(ret == RET_INVALID_PARAMETER_2) LOG_ERR("Transmission could not be scheduled: The payload_ptr parameter is invalid");
            else if(ret == RET_INVALID_PARAMETER_3) LOG_ERR("Transmission could not be scheduled: The payload_size parameter is too large");
            else LOG_ERR("Transmission could not be scheduled: Unkown error");                                                    
        }
        else
        {
            zb_osif_disable_all_inter();
            zb_buf_free(bufid); // No need to use the buffer is there was not a pending frame in the queue.
            zb_osif_enable_all_inter();
            LOG_ERR("Transmission could not be scheduled: No pending output frame in queue");
        }                
    }
    else
    {
        //TODO: Implement a mechanism to try to allocate the buffer again after a while
        zb_err_code = zb_buf_get_out_delayed(zigbee_aps_frame_scheduling_cb);
        ZB_ERROR_CHECK(zb_err_code);
        LOG_ERR("Transmission could not be scheduled: Zigbee Out buffer not allocated");
    }

    b_scheduling_cb_pending = false;
}

//------------------------------------------------------------------------------
/**@brief Management of APS layer. Generation of APS frames and scheduling of their transmission
 *
 *
 */
void zigbee_aps_manager(void)
{
    int ret = 0;
    if( aps_output_frame_buffer.used_space > 0 ) // There is at least an output aps frame pending to be scheduled
    {
        if( !b_scheduling_cb_pending )
        {        
            b_scheduling_cb_pending = true;
            scheduling_cb_timer = SCHEDULING_CB_TIMEOUT_MS;

            ret = ZB_SCHEDULE_APP_CALLBACK(zigbee_aps_frame_scheduling_cb,0);
            if(ret == RET_OK) LOG_DBG("Transmission scheduled");
            else if(ret == RET_OVERFLOW) LOG_ERR("Transmission could not be scheduled: Scheduling failed RET_OVERFLOW");
            else LOG_ERR("Transmission could not be scheduled: Unkown error");
        } 
        else
        {
            LOG_WRN("Transmission could not be scheduled: Scheduling callback already pending");
        }       
    }
}
