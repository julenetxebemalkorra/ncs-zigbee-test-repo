/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @brief Zigbee application template.
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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <nrfx_timer.h>
#include <zephyr/sys/reboot.h>

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

#include <zephyr/debug/coredump.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>
#include <zephyr/sys/reboot.h>

#define ZIGBEE_COORDINATOR_SHORT_ADDR 0x0000

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

//#define ZB_APS_MAX_IN_FRAGMENT_TRANSMISSIONS 5U  // Example: Increase to handle 5 fragments in parallel
//#define ZB_APS_ACK_WAIT_TIMEOUT             1000U // Example: Increase to 1000 ms

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static volatile uint16_t debug_led_ms_x10 = 0; // 10000 ms timer to control the debug led

bool g_b_flash_error = false; //   Flag to indicate if there was an error when reading the NVRAM

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Get a reference to the TIMER1 instance
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(1);

/* Zigbee messagge info*/
static bool b_infit_info_flag = PRINT_ZIGBEE_INFO;

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// boolean flags for detecting modbus request handling
bool b_Zigbe_Connected = false;
bool bTimeToSendFrame = false;

static struct xbee_parameters_t xbee_parameters; // Xbee's parameters


/*----------------------------------------------------------------------------*/
/*                           FUNCTION DEFINITIONS                             */
/*----------------------------------------------------------------------------*/

// Interrupt handler for the timer
// NOTE: This callback is triggered by an interrupt. Many drivers or modules in Zephyr can not be accessed directly from interrupts, 
//		 and if you need to access one of these from the timer callback it is necessary to use something like a k_work item to move execution out of the interrupt context. 
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
            if(debug_led_ms_x10 < 10000) debug_led_ms_x10++;
            tcu_uart_timers_10kHz();
            check_scheduling_cb_timeout();
			break;
		default:
			break;
	}
}

// Function for scheduling repeated callbacks from TIMER1
static void timer1_repeated_timer_start(uint32_t timeout_us)
{
	nrfx_timer_enable(&my_timer);

	nrfx_timer_extended_compare(&my_timer, NRF_TIMER_CC_CHANNEL0, timeout_us, 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

// Function for initializing the TIMER1 peripheral using the nrfx driver
static void timer1_init(void)
{
	//nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_1MHz);
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	int err = nrfx_timer_init(&my_timer, &timer_config, timer1_event_handler);
	if (err != NRFX_SUCCESS) {
		LOG_WRN("Error initializing timer: %x\n", err);
	}

	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);

	// Setup TIMER1 to generate callbacks every second 1000000
	// Setup TIMER1 to generate callbacks every 100ms 100000
	timer1_repeated_timer_start(100); //100 us
	//timer1_repeated_timer_start(200000); // 200 ms
}

//------------------------------------------------------------------------------
/**@brief Function for initializing the GPIO pins.
 *        Currently, only one pin is initialized. Configured as output to drive a led.
 * @retval -1 Error
 * @retval 0 OK
 */
static int8_t gpio_init(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

    return 0;
}

void set_extended_pan_id_in_stack(void)
{
    zb_uint8_t extended_pan_id[8] = {0};
    uint64_t ui_temp = zb_conf_get_extended_pan_id();

    for(uint8_t i=0; i<8; i++)
    {
        extended_pan_id[i] = (zb_uint8_t)(ui_temp & 0x00000000000000FF);
        ui_temp = ui_temp>>8;
    }

	zb_set_extended_pan_id(extended_pan_id);
}

// Function for initializing the Zigbee configuration
void zigbee_configuration()
{
    zb_uint8_t network_link_key[16];
    /* disable NVRAM erasing on every application startup*/
    zb_set_nvram_erase_at_start(ZB_FALSE);

    //TRUE to disable trust center, FALSE to enable trust center
    zb_bdb_set_legacy_device_support(ZB_FALSE);

    zb_conf_get_network_link_key(network_link_key);

    LOG_WRN("Link key: ");
    LOG_HEXDUMP_DBG(network_link_key, 16, " ");

    // Set the network link key. This action can help us choose between different link keys
    zb_zdo_set_tc_standard_distributed_key(network_link_key);

    // Enable distributed Trust Center
    zb_enable_distributed();
    zb_zdo_setup_network_as_distributed();

    set_extended_pan_id_in_stack();

    // Set the device link key. This action can help us choose between different link keys it has to be distributed TC to all devices
    if(zb_is_network_distributed()) LOG_WRN("Network key is distributed");
    else LOG_WRN("Network key is NOT distributed");
}

//------------------------------------------------------------------------------
/**@brief This function toggles and output pin at 1Hz. That output pin is connected to a LED
 *
 * @note Executed the in main loop
 *
 */
void diagnostic_toogle_pin()
{
    if(debug_led_ms_x10 >= 10000)
    {
        debug_led_ms_x10 = 0;
        gpio_pin_toggle_dt(&led);
    }
}

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
