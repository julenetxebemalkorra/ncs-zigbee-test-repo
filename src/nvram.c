/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of the UART0, which is connected to the TCU.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zboss_api.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>
#include <zb_nrf_platform.h>
#include "zb_range_extender.h"

#include "Digi_At_commands.h"
#include "tcu_Uart.h"
#include "nvram.h"

static struct xbee_parameters_t xbee_parameters_nvram; // Xbee's parameters
static struct xbee_parameters_t ds; // Xbee's parameters

LOG_MODULE_REGISTER(nvram_app, LOG_LEVEL_DBG);

uint8_t xbee_nvram_write_app_data(uint8_t page, uint32_t pos)
{
    LOG_DBG("xbee_nvram_write_app_data\n");
    uint8_t ret;

    // Populate 'ds' with your XBee parameters
    // Example: ds.at_vr = xbee_parameters.at_vr;

    ret = ZB_DEFAULT_MANUFACTURER_CODE;

    ds.at_vr = 1;     // Xbee's FW version
    ds.at_hv = 1;     // Xbee's HW version
    ds.at_sh = 0;     // High part of the MAC address
    ds.at_sl = 1;     // Low part of the MAC address
    ds.at_jv = 1;     // Node join verification
    ds.at_nj = 0xFF;  // Node join time
    ds.at_nw = 10;    // Network watchdog
    ds.at_id = 0;     // Extended pan id
    ds.at_ce = 0;     // Coordinator enabled
    ds.at_ai = 0xFF;  // Association indication
    ds.at_ch = 0;     // Operation channel
    ds.at_my = 0;     // Short address
    ds.at_zs = 2;     // Xbee's Zigbee stack profile (2 = ZigBee-PRO)
    ds.at_bd = 4;     // Xbee's UART baud rate (4 = 19200)
    ds.at_nb = 1;     // Xbee's UART parity
    sprintf(ds.at_ni, "NORDIC"); // Node identifier string

    ret = zb_nvram_write_data(page, pos, (zb_uint8_t*)&ds, sizeof(ds));
    return ret;
}

void xbee_nvram_read_app_data(uint8_t page, uint32_t pos, uint16_t payload_length)
{
    LOG_DBG("xbee_nvram_read_app_data\n");
    zb_ret_t ret;

    ret = zb_nvram_read_data(page, pos, (zb_uint8_t*)&ds, sizeof(ds));

    if (ret == RET_OK) {
        // Populate your XBee parameters from 'ds'
        // Example: xbee_parameters.at_vr = ds.at_vr;
        xbee_parameters_nvram.at_vr = ds.at_vr;
        xbee_parameters_nvram.at_hv = ds.at_hv;
        xbee_parameters_nvram.at_sh = ds.at_sh;
        xbee_parameters_nvram.at_sl = ds.at_sl;
        xbee_parameters_nvram.at_jv = ds.at_jv;
        xbee_parameters_nvram.at_nj = ds.at_nj;
        xbee_parameters_nvram.at_nw = ds.at_nw;
        xbee_parameters_nvram.at_id = ds.at_id;
        xbee_parameters_nvram.at_ce = ds.at_ce;
        xbee_parameters_nvram.at_ai = ds.at_ai;
        xbee_parameters_nvram.at_ch = ds.at_ch;
        xbee_parameters_nvram.at_my = ds.at_my;
        xbee_parameters_nvram.at_zs = ds.at_zs;
        xbee_parameters_nvram.at_bd = ds.at_bd;
        xbee_parameters_nvram.at_nb = ds.at_nb;
        memcpy(xbee_parameters_nvram.at_ni, ds.at_ni, sizeof(xbee_parameters_nvram.at_ni));

        LOG_DBG("xbee_parameters_nvram.at_vr = %d\n", xbee_parameters_nvram.at_vr);
        LOG_DBG("xbee_parameters_nvram.at_hv = %d\n", xbee_parameters_nvram.at_hv);
        LOG_DBG("xbee_parameters_nvram.at_sh = %d\n", xbee_parameters_nvram.at_sh);
        LOG_DBG("xbee_parameters_nvram.at_sl = %d\n", xbee_parameters_nvram.at_sl);
        LOG_DBG("xbee_parameters_nvram.at_jv = %d\n", xbee_parameters_nvram.at_jv);
        LOG_DBG("xbee_parameters_nvram.at_nj = %d\n", xbee_parameters_nvram.at_nj);
        LOG_DBG("xbee_parameters_nvram.at_nw = %d\n", xbee_parameters_nvram.at_nw);
        LOG_DBG("xbee_parameters_nvram.at_id = %d\n", xbee_parameters_nvram.at_id);
        LOG_DBG("xbee_parameters_nvram.at_ce = %d\n", xbee_parameters_nvram.at_ce);
        LOG_DBG("xbee_parameters_nvram.at_ai = %d\n", xbee_parameters_nvram.at_ai);
        LOG_DBG("xbee_parameters_nvram.at_ch = %d\n", xbee_parameters_nvram.at_ch);
        LOG_DBG("xbee_parameters_nvram.at_my = %d\n", xbee_parameters_nvram.at_my);
        LOG_DBG("xbee_parameters_nvram.at_zs = %d\n", xbee_parameters_nvram.at_zs);
        LOG_DBG("xbee_parameters_nvram.at_bd = %d\n", xbee_parameters_nvram.at_bd);
        LOG_DBG("xbee_parameters_nvram.at_nb = %d\n", xbee_parameters_nvram.at_nb);
        for(int i = 0; i < sizeof(xbee_parameters_nvram.at_ni); i++) {
            LOG_DBG("xbee_parameters_nvram.at_ni[%d] = %c\n", i, xbee_parameters_nvram.at_ni[i]);
        }
    }
}

uint16_t xbee_get_nvram_data_size()
{
    LOG_DBG("xbee_get_nvram_data_size\n");
    return sizeof(xbee_parameters_nvram);
}

