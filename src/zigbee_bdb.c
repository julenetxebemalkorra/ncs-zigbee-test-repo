/*
 * Copyright (c) 2025 IED
 *
 */

/** @file
 *
 * @brief Zigbee BDB (Basic device behavior).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zboss_api.h>

#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
//#include <zboss_api_zdo.h>
//#include <zb_osif_platform.h>
//#include <zboss_api_buf.h>
#include "zigbee_bdb.h"
#include "global_defines.h"
#include "zigbee_device_profile.h"
#include "zigbee_aps.h"

/* Local variables                                                            */
static bool b_not_detected_coordinator_activity_since_boot;
static bool b_coordinator_is_active;
static uint64_t time_last_frame_received_from_coordinator_ms;


LOG_MODULE_REGISTER(BDB, LOG_LEVEL_DBG);

/**@brief Initializes local variables of the zigbee_bdb module.
 *
 */
void zigbee_bdb_init(void)
{
    b_not_detected_coordinator_activity_since_boot = true;
    b_coordinator_is_active = false;
    time_last_frame_received_from_coordinator_ms = 0;
}


/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t   *sg_p = NULL;
    zb_zdo_app_signal_type_t  signal_type;
    zb_ret_t                  signal_status_code;

    if (bufid)
    {
        signal_type         = zb_get_app_signal(bufid, &sg_p);
        signal_status_code  = ZB_GET_APP_SIGNAL_STATUS(bufid);

        if (signal_type != ZB_COMMON_SIGNAL_CAN_SLEEP ) // Do not log this annoying signal. It is sent every second approx.
        {
            LOG_WRN("Event signal. Type %u, status code %" PRId32, signal_type, signal_status_code);
        }

        if (signal_type == ZB_ZDO_SIGNAL_LEAVE)
        {
            if (signal_status_code == RET_OK)
            {
                if (g_b_reset_mcu_after_leaving_network)
                {
                    sys_reboot(SYS_REBOOT_COLD);
                }
            }
        }

        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    }
}

/** @brief Resets the variables used for detecting coordinator activity.
 *
 */
void zigbee_bdb_coordinator_activity_detected(void)
{
    if (b_not_detected_coordinator_activity_since_boot) b_not_detected_coordinator_activity_since_boot = false;
    b_coordinator_is_active = true;
    time_last_frame_received_from_coordinator_ms = k_uptime_get();
}

/** @brief Evaluates the presence of the coordinator and resets the module if no coordinator activity is detected.
 *
 */
void zigbee_bdb_network_watchdog(void)
{
    static uint64_t time_last_ieee_address_request_sent_to_coordinator_ms = 0;
    static uint8_t num_attempts_detect_coordinator = 0;
    uint64_t time_now_ms = k_uptime_get();

    if (zb_zdo_joined())
    {
        if (b_coordinator_is_active)
        {
            if ((time_now_ms - time_last_frame_received_from_coordinator_ms) > MAX_TIME_NO_FRAMES_FROM_COORDINATOR)
            {
                b_coordinator_is_active = false;
                time_last_ieee_address_request_sent_to_coordinator_ms = 0;
                num_attempts_detect_coordinator = 0;
            }
        }
        else
        {
            if (num_attempts_detect_coordinator < MAX_ATTEMPS_DETECT_COORDINATOR)
            {
                if ((time_now_ms - time_last_ieee_address_request_sent_to_coordinator_ms) > TIME_INTERVAL_BETWEEN_IEEE_ADD_REQ_TO_ZC)
                {
                   if (request_coordinator_ieee_address())
                    {
                        time_last_ieee_address_request_sent_to_coordinator_ms = time_now_ms;
                        num_attempts_detect_coordinator++;
                    }
                }
            }
            else
            {
                g_b_reset_zigbee_cmd = true;  // Leave the network and start steering process
                //zb_set_nvram_erase_at_start(ZB_TRUE);
                //sys_reboot(SYS_REBOOT_COLD);
            }
        }
    }
    else
    {
        b_coordinator_is_active = false;
        time_last_ieee_address_request_sent_to_coordinator_ms = time_now_ms;
        num_attempts_detect_coordinator = 0;
    }
}

/**@brief This function adds to the APS output frame queue a coordinator IEEE request address.
*
* @return The function returns a bool. TRUE if frame could be added to the queue.
*/
bool request_coordinator_ieee_address(void)
{
    static uint8_t sequence_number = 0;
    bool b_return = false;

    if( zigbee_aps_get_output_frame_buffer_free_space() )
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = ZIGBEE_DEVICE_PROFILE_ID;
        element.cluster_id = IEEE_ADDRESS_REQUEST_CLUSTER;
        element.src_endpoint = ZIGBEE_DEVICE_OBJECT_SOURCE_ENDPOINT;
        element.dst_endpoint = ZIGBEE_DEVICE_OBJECT_DESTINATION_ENDPOINT;
        i = 0;
        sequence_number++;
        element.payload[i++] = sequence_number;
        element.payload[i++] = COORDINATOR_SHORT_ADDRESS_LOWER_BYTE;
        element.payload[i++] = COORDINATOR_SHORT_ADDRESS_HIGHER_BYTE;
        element.payload[i++] = IEEE_ADDRESS_REQUEST_TYPE;
        element.payload[i++] = SINGLE_REPLY_START_INDEX;
        element.payload_size = (zb_uint8_t)i;
        LOG_WRN("Request coordinator IEEE address");
        if( enqueue_aps_frame(&element) ) b_return = true;
    }

    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}
