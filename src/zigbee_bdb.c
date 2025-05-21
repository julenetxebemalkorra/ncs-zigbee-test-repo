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

#include <zboss_api.h>

#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
//#include <zboss_api_zdo.h>
//#include <zb_osif_platform.h>
//#include <zboss_api_buf.h>
#include "zigbee_bdb.h"

/* Local variables                                                            */

LOG_MODULE_REGISTER(BDB, LOG_LEVEL_DBG);

#define BAD_KEY_SEQ_THRESHOLD 5

#define ZIGBEE_COORDINATOR_SHORT_ADDR 0x0000  // Update with actual coordinator address

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

static uint8_t bad_key_seq_counter = 0;


void simple_desc_response_cb(zb_bufid_t bufid)
{
    if (!bufid)
    {
        if (zb_buf_is_oom_state()) {
        // Handle Out Of Memory state
        LOG_ERR("Buffer pool is out of memory!\n");
        }
        if (zb_buf_memory_low()) 
        {
            // Handle low memory state
            LOG_WRN("Warning: Buffer pool memory is running low!\n");
        }
    
        // Trace the buffer statistics for debugging purposes
        zb_buf_oom_trace();
        LOG_ERR("Error: bufid is NULL simple_desc_response_cb beggining: bufid status %d", zb_buf_get_status(bufid));
        return ;
    }
    else
    {
        zb_zdo_simple_desc_resp_t *resp = (zb_zdo_simple_desc_resp_t *)zb_buf_begin(bufid);

        if (resp->hdr.status != ZB_ZDP_STATUS_SUCCESS)
        {
            LOG_WRN("Device seems disconnected. Initiating TC rejoin...");
            zb_bdb_initiate_tc_rejoin(bufid);
        }
        else
        {
            LOG_INF("Device is still connected. Resetting bad key counter");
            bad_key_seq_counter = 0;
        }

        if (bufid)
        {
        	// safe way to free buffer
            zb_osif_disable_all_inter();
            zb_buf_free(bufid);
            zb_osif_enable_all_inter();
        	return ;
	    }

    }

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

        if(signal_type != ZB_COMMON_SIGNAL_CAN_SLEEP ) // Do not log this annoying signal. It is sent every second approx.
        {
            LOG_WRN("Event signal. Type %u, status code %" PRId32, signal_type, signal_status_code);
        }

           // https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/releases_and_maturity/known_issues.html#zigbee
        // NCSIDB-1411: Clearing configuration data is not fully performed when processing the Leave Network command


        if(signal_type == ZB_ZDO_SIGNAL_LEAVE)
        // Device leaves the network.
        if (signal_status_code == RET_OK) 
        {
            LOG_WRN("Device leaves the network");
            zb_zdo_signal_leave_params_t *leave_params =
                ZB_ZDO_SIGNAL_GET_PARAMS(signal_type, zb_zdo_signal_leave_params_t);

            if (leave_params->leave_type == ZB_NWK_LEAVE_TYPE_RESET) {
               // Workaround for NCSIDB-1411 - clearing ZCL Reporting parameters. 
               LOG_WRN("Clearing ZCL Reporting parameters");
               zb_zcl_init_reporting_info();
            }
        }
    

        if(signal_type == ZB_NLME_STATUS_INDICATION)
        {
            zb_zdo_signal_nlme_status_indication_params_t *nlme_status_ind =
                ZB_ZDO_SIGNAL_GET_PARAMS(signal_type, zb_zdo_signal_nlme_status_indication_params_t);
            /* Device leaves the network. */
            if (signal_status_code == RET_OK) 
            {


                if (nlme_status_ind->nlme_status.status == ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER)
                {
                    LOG_WRN("Received BAD_KEY_SEQUENCE_NUMBER status");
                    bad_key_seq_counter++;
                    if (bad_key_seq_counter >= BAD_KEY_SEQ_THRESHOLD)
                    {
                        LOG_WRN("Threshold reached. Checking network connection...");
                        // Allocate buffer to send Simple Descriptor Request
                        zb_bufid_t req_buf = zb_buf_get_out();
                        if (req_buf)
                        {
                            LOG_WRN("Sending Simple Desc Req to coordinator");
                            // Example: Send to coordinator (short addr = 0x0000, endpoint 0x00)
                            zb_bufid_t req_buf = zb_buf_get_out();
                            zb_zdo_simple_desc_req_t *req = ZB_BUF_GET_PARAM(req_buf, zb_zdo_simple_desc_req_t);
                            req->nwk_addr = ZIGBEE_COORDINATOR_SHORT_ADDR;       // address of coordinator
                            req->endpoint = APP_TEMPLATE_ENDPOINT;
                            zb_zdo_simple_desc_req(req_buf, simple_desc_response_cb);
                        }
                        else
                        {
                            LOG_ERR("Unable to allocate buffer for simple desc request");
                        }
                    }
                }
            }
        }

        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    }
}
