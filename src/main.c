/*
 * Copyright (c) 2025 IED
 *
 */

/**
 * @file main.c
 * @brief Main source file for the dual communication Nordic project.
 *
 *
 * This firmware is designed for the Nordic nRF52840 platform and provides the following features:
 *
 * - Implements a Zigbee Router node using the ZBOSS stack, fully integrated with the Zephyr RTOS.
 * - Provides an AT command interface over UART, supporting both command mode and transparent mode.
 * - Supports wireless AT command reception via Zigbee APS frames, compatible with XBee/Digi style communication.
 * - Enables Firmware Upgrade Over-The-Air (FUOTA) using a custom APS-based transport protocol.
 * - Utilizes Zephyr's flash-backed settings subsystem for NVRAM configuration storage.
 * 
 * @author jetxeberria
 * @date 2025
 */

// TODO: Add only the necessary #include directives for this file
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zboss_api.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>
#include <zb_nrf_platform.h>
#include "zb_range_extender.h"
#include "zb_mem_config_max.h" // This file has to be included after zboss_api.h

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
 * @details Information is printed only once and if PRINT_ZIGBEE_INFO is true, after the device has joined a Network
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


/**
 *
 * @section main_diagram Main Logic Flow
 *
 * @dot
 * digraph main_logic {
 *   node [shape=box, fontname="Arial"];
 *   Main [label="main()"];
 *   DisplaySysInfo [label="display_system_information()"];
 *   DisplayBootStatus [label="display_boot_status()"];
 *   InitNVRAM [label="init_nvram()"];
 *   CheckNVRAM [label="zb_nvram_check_usage()"];
 *   WriteDefaults [label="zb_conf_write_to_nvram()"];
 *   ReadNVRAM [label="zb_conf_read_from_nvram()"];
 *   ZigbeeAPSInit [label="zigbee_aps_init()"];
 *   DigiATInit [label="digi_at_init()"];
 *   NodeDiscoveryInit [label="digi_node_discovery_init()"];
 *   WirelessATInit [label="digi_wireless_at_init()"];
 *   DigiFOTAInit [label="digi_fota_init()"];
 *   ZigbeeBDBInit [label="zigbee_bdb_init()"];
 *   WatchdogInit [label="watchdog_init()"];
 *   UARTInit [label="tcu_uart_init()"];
 *   TimerInit [label="timer1_init()"];
 *   GPIOInit [label="gpio_init()"];
 *   ZigbeeConfig [label="zigbee_configuration()"];
 *   ZigbeeEnable [label="zigbee_enable()"];
 *   SetDataIndCB [label="zb_af_set_data_indication()"];
 *   SetUserDataTxCB [label="zb_aps_set_user_data_tx_cb()"];
 *   ConfirmImage [label="confirm_image()"];
 *   MainLoop [label="while(1)"];
 *   FeedWatchdog [label="periodic_feed_of_main_loop_watchdog()"];
 *   TogglePin [label="diagnostic_toogle_pin()"];
 *   ZigbeeDiag [label="diagnostic_zigbee_info()"];
 *   UARTTranspMgr [label="tcu_uart_transparent_mode_manager()"];
 *   NodeDiscMgr [label="digi_node_discovery_request_manager()"];
 *   WirelessATMgr [label="digi_wireless_read_at_command_manager()"];
 *   DigiFOTAMgr [label="digi_fota_manager()"];
 *   APSMgr [label="zigbee_aps_manager()"];
 *   BDBWatchdog [label="zigbee_bdb_network_watchdog()"];
 *   ZigbeeResetMgr [label="zigbee_reset_manager()"];
 *   NVRAMMgr [label="nvram_manager()"];
 *   UARTMgr [label="tcu_uart_manager()"];
 *   Sleep [label="k_sleep(5ms)"];
 *
 *   Main -> DisplaySysInfo -> DisplayBootStatus -> InitNVRAM -> CheckNVRAM;
 *   CheckNVRAM -> WriteDefaults [label="if unused"];
 *   CheckNVRAM -> ReadNVRAM [label="if used"];
 *   WriteDefaults -> ZigbeeAPSInit;
 *   ReadNVRAM -> ZigbeeAPSInit;
 *   ZigbeeAPSInit -> DigiATInit -> NodeDiscoveryInit -> WirelessATInit -> DigiFOTAInit -> ZigbeeBDBInit -> WatchdogInit -> UARTInit -> TimerInit -> GPIOInit;
 *   GPIOInit -> ZigbeeConfig -> ZigbeeEnable -> SetDataIndCB -> SetUserDataTxCB -> ConfirmImage -> MainLoop;
 *   MainLoop -> FeedWatchdog -> TogglePin -> ZigbeeDiag -> UARTTranspMgr -> NodeDiscMgr -> WirelessATMgr -> DigiFOTAMgr -> APSMgr -> BDBWatchdog -> ZigbeeResetMgr -> NVRAMMgr -> UARTMgr -> Sleep;
 *   Sleep -> MainLoop [style=dotted];
 * }
 * @enddot
 *
 * @section description Description
 * The main logic initializes system information and boot status, then sets up NVRAM.
 * Depending on NVRAM usage, it writes default Zigbee parameters or reads existing ones.
 * It then initializes all Zigbee, UART, watchdog, timer, and GPIO subsystems.
 * After configuration, it enables Zigbee networking, sets up callbacks, and confirms firmware image.
 * The main loop continuously:
 *   - Feeds the watchdog
 *   - Toggles a diagnostic pin
 *   - Prints Zigbee diagnostic info after joining
 *   - Manages UART transparent mode and node discovery
 *   - Handles wireless AT commands and FOTA state machine
 *   - Processes Zigbee APS output queue and BDB network watchdog
 *   - Handles Zigbee and MCU reset requests
 *   - Manages NVRAM and UART
 *   - Sleeps briefly to allow log processing
*
 * @brief Main function of the firmware.
 *
 * Initializes system peripherals and enters the main execution loop.
 * Handles Zigbee network join attempts, application state logic,
 * and watchdog registration.
 *
 * @return int Should never return.
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
