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
        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    }
}
