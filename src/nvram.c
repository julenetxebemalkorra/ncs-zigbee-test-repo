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

/* Application dataset for persisting into nvram */
typedef ZB_PACKED_PRE struct nvram_dataset_s
{
 zb_uint8_t at_vr;
 zb_uint8_t at_hv;
  /* Size of the structure in bytes must be divisible by 4 */
 zb_uint8_t reserved[2];
} ZB_PACKED_STRUCT
nvram_dataset_t;

LOG_MODULE_REGISTER(nvram_app, LOG_LEVEL_DBG);

/* Application callback for writing application data to NVRAM */
uint8_t xbee_nvram_write_app_data(uint8_t page, uint32_t pos)
{
    LOG_WRN("xbee_nvram_write_app_data. page: %d pos: %d\n", page, pos);

    uint8_t ret;
    nvram_dataset_t ds;

    // Populate 'ds' with your XBee parameters
    // Example: ds.at_vr = xbee_parameters.at_vr;

    ds.at_vr = 1;     // Xbee's FW version
    ds.at_hv = 2;     // Xbee's HW version
    ds.reserved[0] = 3;
    ds.reserved[1] = 4;

    ret = zb_nvram_write_data(page, pos, (zb_uint8_t*)&ds, sizeof(ds));
    return ret;
}

/* Application callback for reading application data from NVRAM */
void xbee_nvram_read_app_data(uint8_t page, uint32_t pos, uint16_t payload_length)
{
    LOG_DBG("xbee_nvram_read_app_data\n");
    zb_ret_t ret;

    nvram_dataset_t ds;

    /* [trace_msg] */
    ZB_ASSERT(payload_length == sizeof(ds));

    ret = zb_nvram_read_data(page, pos, (zb_uint8_t*)&ds, sizeof(ds));

    if (ret == RET_OK) {
        // Populate your XBee parameters from 'ds'
        // Example: xbee_parameters.at_vr = ds.at_vr;
        xbee_parameters_nvram.at_vr = ds.at_vr;
        xbee_parameters_nvram.at_hv = ds.at_hv;
        xbee_parameters_nvram.at_ni[0] = ds.reserved[0];
        xbee_parameters_nvram.at_ni[1] = ds.reserved[1];

        LOG_DBG("xbee_parameters_nvram.at_vr = %d\n", xbee_parameters_nvram.at_vr);
        LOG_DBG("xbee_parameters_nvram.at_hv = %d\n", xbee_parameters_nvram.at_hv);
        LOG_DBG("xbee_parameters_nvram.reserved[0] = %d\n", xbee_parameters_nvram.at_ni[0]);
        LOG_DBG("xbee_parameters_nvram.reserved[1] = %d\n", xbee_parameters_nvram.at_ni[1]);
    }
}

/* Application callback to determine application NVRAM stored dataset size */
uint16_t xbee_get_nvram_data_size()
{
    LOG_DBG("xbee_get_nvram_data_size\n");
    return sizeof(nvram_dataset_t);
}

