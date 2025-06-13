/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief C main source file.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zboss_api.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>
#include <zb_nrf_platform.h>
#include "zb_range_extender.h"
#include "zb_mem_config_max.h" // This file has to be included after zboss_api.h

// UART async includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/uart.h>
#include <string.h>

#include <nrfx_timer.h>

#include "global_defines.h"
#include "zigbee_configuration.h"
#include "tcu_Uart.h"
#include "Digi_profile.h"
#include "zigbee_device_profile.h"
#include "zigbee_bdb.h"
#include "zigbee_aps.h"
#include "Digi_At_commands.h"
#include "Digi_node_discovery.h"
#include "Digi_wireless_at_commands.h"
#include "nvram.h"
#include "Digi_fota.h"
#include "OTA_dfu_target.h"
#include "system.h"

#include <stdbool.h>

#include <zboss_api_addons.h>
#include "zb_config.h"
#include "zb_address.h"
#include "zboss_api_buf.h"
#include "zb_types.h"
#include "zboss_api_zgp.h"

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

bool g_b_flash_error = false; //   Flag to indicate if there was an error when reading the NVRAM

/* Zigbee messagge info*/
static bool b_infit_info_flag = PRINT_ZIGBEE_INFO;

// boolean flags for detecting modbus request handling
bool b_Zigbe_Connected = false;
bool bTimeToSendFrame = false;

static struct xbee_parameters_t xbee_parameters; // Xbee's parameters


/*----------------------------------------------------------------------------*/
/*                           FUNCTION DEFINITIONS                             */
/*----------------------------------------------------------------------------*/


//------------------------------------------------------------------------------
/**@brief This function prints the value of several Zigbee parameters to the console.
 *
 *
 * @details Information is printed only once, after the device has joined a Network
 *
 * @note Executed the in main loop
 *
 */
void diagnostic_zigbee_info()
{
    zb_ext_pan_id_t zb_ext_pan_id;

    if(zb_zdo_joined() && b_infit_info_flag == ZB_TRUE)
    {
        b_infit_info_flag = ZB_FALSE;
        b_Zigbe_Connected = true;
        if(PRINT_ZIGBEE_INFO) LOG_DBG("Zigbee application joined the network: bellow some info : \n");

        xbee_parameters.at_my = zb_get_short_address();
        if(PRINT_ZIGBEE_INFO) LOG_DBG("zigbee shrot addr:  0x%x\n", xbee_parameters.at_my);

        zb_get_extended_pan_id(zb_ext_pan_id);
        
        // Display extended PAN ID
        uint8_t temp[8];
        for(uint8_t i = 0; i<8; i++)
        {
            temp[i] = zb_ext_pan_id[7-i];
        }
        if(PRINT_ZIGBEE_INFO) LOG_HEXDUMP_DBG(temp,8,"Extended PAN ID: ");

        switch(zb_get_network_role())
        {
        case 0:
            if(PRINT_ZIGBEE_INFO) LOG_DBG("zigbee role coordinator\n");
            break;
        case 1:
            if(PRINT_ZIGBEE_INFO) LOG_DBG("zigbee role router\n");
            break;
        case 2:
            if(PRINT_ZIGBEE_INFO) LOG_DBG("zigbee role end device\n");
            break;
        default:
            if(PRINT_ZIGBEE_INFO) LOG_DBG("Zigbee role NOT found \n");
            break;
        }

    xbee_parameters.at_ch = zb_get_current_channel();
    if(PRINT_ZIGBEE_INFO) LOG_DBG("zigbee channel: %d \n", xbee_parameters.at_ch);

    }
}

// Custom logging function to print extended address and PAN ID
void log_ext_address(zb_uint8_t *addr)
{
    LOG_INF("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
        addr[7], addr[6], addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

//------------------------------------------------------------------------------
/**@brief Main function
 *
 */
int main(void)
{
    int8_t ret = 0;

    display_system_information();
    display_boot_status();

    ret = init_nvram();              // Initialize NVRAM
    if( ret != 0)
    {
        LOG_ERR("init_nvram error %d", ret);
        g_b_flash_error = ZB_TRUE;
    }
    
    if (!g_b_flash_error)
    {
        ret = zb_nvram_check_usage();   // Check NVRAM usage

        if( ret == -1) // NVRAM is not used, so write default data
        {
            zb_conf_write_to_nvram(); // Write user configurable zigbee parameters to NVRAM
        }
        else if(ret == 0) // read NVRAM data and use it
        {
            ret = zb_conf_read_from_nvram(); // Read user configurable zigbee parameters from NVRAM
            if(ret < 0)
            {
                LOG_ERR("zb_conf_read_from_nvram error %d", ret);
                //g_b_flash_error = ZB_TRUE;
            }
            else if(ret == 0)
            {
                LOG_INF("NVRAM data read successfully");
                //g_b_flash_error = ZB_FALSE;
            }
        }
        else
        {
            LOG_ERR("zb_nvram_check_usage error %d", ret);
            g_b_flash_error = ZB_TRUE;
        }	
    }

    zigbee_aps_init();
    digi_at_init();
    digi_node_discovery_init();
    digi_wireless_at_init();
    digi_fota_init();
    zigbee_bdb_init();

    ret = watchdog_init();
    if( ret < 0)
    {
        LOG_ERR("watchdog_init error %d", ret);
    }

    ret = tcu_uart_init();
    if( ret < 0)
    {
        LOG_ERR("tcu_uart_init error %d", ret);
    }

    // Initialize TIMER1
    timer1_init();

    // Initialize GPIO
    ret = gpio_init();
    if( ret < 0)
    {
        LOG_ERR("gpio_init error %d", ret);
    }

    LOG_WRN("Starting Zigbee Router");
    zigbee_configuration(); //Zigbee configuration
    zigbee_enable(); // Start Zigbee default thread
    zb_af_set_data_indication(data_indication_cb); // Set call back function for APS frame received
    zb_aps_set_user_data_tx_cb(zigbee_aps_user_data_tx_cb); // Set call back function for APS frame transmitted

    LOG_INF("Router started successfully");

    confirm_image(); // Confirm the image if it is not already confirmed

    while(1)
    {
        periodic_feed_of_main_loop_watchdog();
        diagnostic_toogle_pin();
        diagnostic_zigbee_info();   
        tcu_uart_transparent_mode_manager();     // Manage the frames received from the TCU uart when module is in transparent mode
        digi_node_discovery_request_manager();   // Manage the device discovery requests
        digi_wireless_read_at_command_manager(); // Manage the read AT commands received through Zigbee
        digi_fota_manager();                     // FUOTA state machine
        zigbee_aps_manager();                    // Manage the aps output frame queue
        zigbee_bdb_network_watchdog();           // Network watchdog
        zigbee_reset_manager();                  // Manage reset requests of ZBOSS stack or MCU
        nvram_manager();                         // Manage the NVRAM
        tcu_uart_manager();                      // Manage the TCU UART
        k_sleep(K_MSEC(5));                      // Required to see log messages on console
    }

    return 0;
}
