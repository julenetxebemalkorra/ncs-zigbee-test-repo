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

// UART async includes
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <nrfx_timer.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>

#include "global_defines.h"
#include "zigbee_configuration.h"
#include "tcu_Uart.h"
#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_At_commands.h"
#include "Digi_node_discovery.h"
#include "nvram.h"

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON             1

/* Flag  used to print zigbee info once the device joins a network. */
#define PRINT_ZIGBEE_INFO                ZB_TRUE
#define PRINT_UART_INFO                  ZB_TRUE
#define CRYPTO_ENABLE                    ZB_FALSE

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*UART Modbus and zigbee buffer size definitions*/
#define MODBUS_MIN_RX_LENGTH                8   // if the messagge has 8 bytes we consider it a modbus frame

#define SIGNAL_STEERING_ATEMP_COUNT_MAX     3  // Number of steering attempts before local reset
#define RESTART_ATEMP_COUNT_MAX             3  // Number of restart attempts before hardware reset

uint32_t reset_cause; // Bit register containg the last reset cause.

uint16_t aps_frames_received_total_counter = 0;
uint16_t aps_frames_received_binary_cluster_counter = 0;
uint16_t aps_frames_received_commissioning_cluster_counter = 0;

uint16_t tcu_uart_frames_transmitted_counter = 0;
uint16_t tcu_uart_frames_received_counter = 0;

uint8_t soft_reset_counter = 0;
uint8_t hard_reset_counter = 0;

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

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// boolean flags for detecting modbus request handling
bool b_Zigbe_Connected = false;
bool bTimeToSendFrame = false;

/*----------------------------------------------------------------------------*/
/*                           FUNCTION DEFINITIONS                             */
/*----------------------------------------------------------------------------*/

/**@brief Function to read the reason for the last reset. The reason is printed in the console. */
void get_reset_reason(void)
{
	int result = hwinfo_get_reset_cause(&reset_cause);
    if (result == 0) // Success, reset_cause now contains the reset cause flags
	{
        LOG_ERR("RESET");

		if (reset_cause & RESET_PIN) LOG_DBG("Reset cause is: RESET_PIN\n"); // 0
		else if (reset_cause & RESET_SOFTWARE) LOG_DBG("Reset cause is: RESET_SOFTWARE\n"); // 1
		else if (reset_cause & RESET_BROWNOUT) LOG_DBG("Reset cause is: RESET_BROWNOUT\n"); // 2
		else if(reset_cause & RESET_POR) LOG_DBG("Reset cause is: RESET_POR\n"); // 3
		else if(reset_cause & RESET_WATCHDOG) LOG_DBG("Reset cause is: RESET_WATCHDOG\n"); // 4
		else if(reset_cause & RESET_DEBUG) LOG_DBG("Reset cause is: RESET_DEBUG\n"); // 5
    	else if(reset_cause & RESET_SECURITY) LOG_DBG("Reset cause is: RESET_SECURITY\n"); // 6
		else if(reset_cause & RESET_LOW_POWER_WAKE) LOG_DBG("Reset cause is: RESET_LOW_POWER_WAKE\n"); // 7
		else if(reset_cause & RESET_CPU_LOCKUP) LOG_DBG("Reset cause is: RESET_CPU_LOCKUP\n"); // 8
		else if(reset_cause & RESET_PARITY) LOG_DBG("Reset cause is: RESET_PARITY\n"); // 9
		else if(reset_cause & RESET_PLL) LOG_DBG("Reset cause is: RESET_PLL\n"); // 10
		else if(reset_cause & RESET_CLOCK) LOG_DBG("Reset cause is: RESET_CLOCK\n"); // 11
		else if(reset_cause & RESET_HARDWARE) LOG_DBG("Reset cause is: RESET_HARDWARE\n"); // 12
		else if(reset_cause & RESET_USER) LOG_DBG("Reset cause is: RESET_USER\n"); // 13
		else if (reset_cause & RESET_TEMPERATURE) LOG_DBG("Reset cause is: RESET_TEMPERATURE\n\n"); // 14
		else LOG_DBG("\n\n reset cause is: %d\n\n",reset_cause);

    	hwinfo_clear_reset_cause(); // Clear the hardware flags. In that way we see only the cause of last reset
    } else if (result == -ENOSYS) 
	{
		LOG_ERR("\n\n there is no implementation for the particular device.\n\n");
    } else 
	{
		LOG_ERR("\n\n negative value on driver specific errors\n\n");
    }
}

//------------------------------------------------------------------------------
/**@brief Callback function excuted when AF gets APS packet.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 *
 */
zb_uint8_t data_indication_cb(zb_bufid_t bufid)
{
    if (bufid)
	{
        zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header

        zb_uint8_t *pointerToBeginOfBuffer;
        zb_uint8_t *pointerToEndOfBuffer;
        zb_int32_t sizeOfPayload;
        pointerToBeginOfBuffer = zb_buf_begin(bufid);
        pointerToEndOfBuffer = zb_buf_end(bufid);
        sizeOfPayload = pointerToEndOfBuffer - pointerToBeginOfBuffer;

        if(PRINT_ZIGBEE_INFO) LOG_DBG("Rx APS Frame with profile 0x%x, cluster 0x%x, src_ep %d, dest_ep %d, payload %d bytes",
                          (uint16_t)ind->profileid, (uint16_t)ind->clusterid, (uint8_t)ind->src_endpoint,
                                                    (uint8_t)ind->dst_endpoint, (uint16_t)sizeOfPayload);

        aps_frames_received_total_counter++;
        if( ind->clusterid == DIGI_BINARY_VALUE_CLUSTER )aps_frames_received_binary_cluster_counter++;
        if( ind->clusterid == DIGI_COMMISSIONING_CLUSTER )aps_frames_received_commissioning_cluster_counter++;

        if( (ind->clusterid == DIGI_BINARY_VALUE_CLUSTER) &&
            ( ind->src_endpoint == DIGI_BINARY_VALUE_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_BINARY_VALUE_DESTINATION_ENDPOINT ) )
        {
            if ((sizeOfPayload > 0) && (sizeOfPayload < UART_RX_BUFFER_SIZE))
            {
                /*if(PRINT_ZIGBEE_INFO)
                {
                    LOG_DBG("Size of received payload is %d bytes \n", sizeOfPayload);
                    LOG_HEXDUMP_DBG(pointerToBeginOfBuffer,sizeOfPayload,"Payload of input RF packet");
                }*/

                if( !is_tcu_uart_in_command_mode() && (sizeOfPayload >= MODBUS_MIN_RX_LENGTH) )
                {
                    if(PRINT_ZIGBEE_INFO) LOG_DBG("Payload of input RF packet sent to TCU UART");
                    tcu_uart_frames_transmitted_counter++;
                    sendFrameToTcu((uint8_t *)pointerToBeginOfBuffer, sizeOfPayload);
                }
            }
        }

        if( ( ind->clusterid == DIGI_COMMISSIONING_CLUSTER ) &&
            ( ind->src_endpoint == DIGI_COMMISSIONING_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_COMMISSIONING_DESTINATION_ENDPOINT ) )
        {
            if( is_a_digi_node_discovery_request((uint8_t *)pointerToBeginOfBuffer, (uint16_t)sizeOfPayload) )
            {
                if(PRINT_ZIGBEE_INFO) LOG_DBG("Xbee Node Discovery Device Request");
            }
        }
	}

    if (bufid)
    {
    	// safe way to free buffer
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    	return ZB_TRUE;
	}
	return ZB_FALSE;
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	//Read signal description out of memory buffer. */
	zb_zdo_app_signal_hdr_t *sg_p = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &sg_p);

    if( PRINT_ZIGBEE_INFO )
    {
        if( sig != ZB_COMMON_SIGNAL_CAN_SLEEP ) // Do not show information about this one, it happens too often!
        {
			if( sig == ZB_BDB_SIGNAL_DEVICE_FIRST_START ) LOG_WRN( "SIGNAL 5: Device started for the first time after the NVRAM erase");
            else if( sig == ZB_BDB_SIGNAL_DEVICE_REBOOT ) LOG_WRN( "SIGNAL 6: Device started using the NVRAM contents");
			else if( sig == ZB_BDB_SIGNAL_STEERING_CANCELLED ) LOG_WRN( "SIGNAL 55: BDB steering cancel request processed");
            else if( sig == ZB_ZDO_SIGNAL_LEAVE ) LOG_WRN( "SIGNAL 3: The device has left the network");
            else if( sig == ZB_ZDO_SIGNAL_DEVICE_ANNCE ) LOG_WRN(" SIGNAL 2:  Notifies the application about the new device appearance.");
			else
			{
                if (ZB_GET_APP_SIGNAL_STATUS(bufid) == 0)
                {
                    LOG_WRN( "SIGNAL %d , state OK",sig);
                    soft_reset_counter = 0;
                    hard_reset_counter = 0;
                }
                else
                {
                    LOG_WRN( "SIGNAL %d , state not OK",sig);
                }
			}
        }
    }

    if(sig == ZB_BDB_SIGNAL_STEERING || sig == ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT)
    {
        soft_reset_counter++;
        LOG_WRN("Device is encountering issues during steering.");
        if (soft_reset_counter >= SIGNAL_STEERING_ATEMP_COUNT_MAX)
        {
 			if(PRINT_ZIGBEE_INFO)
 			{	
            	zb_uint16_t parnet_node = zb_nwk_get_parent();
                if(parnet_node != 0xffff)
                {
                    LOG_WRN("Device has joined a coordinator, but there might be an issue. Parent Node: 0x%04x", parnet_node);
                }
                else
                {
                    LOG_WRN( "device has not joined a coordinator.");
                }
        	}	

            ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);

            LOG_WRN( "The device will perform the NLME leave and clean all Zigbee persistent data except the outgoing NWK frame counter and application datasets");

            soft_reset_counter = 0;
            hard_reset_counter++;
        }
        if(hard_reset_counter >= RESTART_ATEMP_COUNT_MAX)
        {
            LOG_ERR( "device restarted since it is not able to join any network");
            
            // Reboot the device
		    sys_reboot(0);
            hard_reset_counter = 0;
        }
    }

	/* No application-specific behavior is required.
	 * Call default signal handler.
	 */
    ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	/* All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid)
	{
		// safe way to free buffer
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();	
    }
	
}

// Interrupt handler for the timer
// NOTE: This callback is triggered by an interrupt. Many drivers or modules in Zephyr can not be accessed directly from interrupts, 
//		 and if you need to access one of these from the timer callback it is necessary to use something like a k_work item to move execution out of the interrupt context. 
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
            if(debug_led_ms_x10 < 10000) debug_led_ms_x10++;
            tcu_uart_timers_10kHz();
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
	/* disable NVRAM erasing on every application startup*/
	zb_set_nvram_erase_at_start(ZB_TRUE);

	if(!CRYPTO_ENABLE)
	{
		zb_disable_nwk_security();
		zb_nwk_set_ieee_policy(ZB_FALSE);
	}

	// Define a distributed key (assuming key size of 16 bytes, you might need to adjust based on documentation)
    zb_uint8_t distributed_key[16] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};

    // Set the network key
	zb_secur_setup_nwk_key((zb_uint8_t *) distributed_key, 0);

    set_extended_pan_id_in_stack();

	//TRUE to disable trust center, legacy support for
	zb_bdb_set_legacy_device_support(ZB_TRUE);
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
    zb_ieee_addr_t zb_long_address;
    zb_ext_pan_id_t zb_ext_pan_id;
    zb_uint8_t zb_channel;
    zb_uint16_t zb_shrot_addr;

    if(zb_zdo_joined() && b_infit_info_flag == ZB_TRUE)
    {
        b_infit_info_flag = ZB_FALSE;
        b_Zigbe_Connected = true;
        LOG_DBG("Zigbee application joined the network: bellow some info: \n");
        zb_get_long_address(zb_long_address);
        // Display MAC address
        {
            uint8_t temp[8];
            for(uint8_t i = 0; i<8; i++)
            {
                temp[i] = zb_long_address[7-i];
            }
            LOG_HEXDUMP_DBG(temp,8,"MAC address: ");
        }

        zb_shrot_addr = zb_get_short_address();
        LOG_DBG("zigbee shrot addr:  0x%x\n", zb_shrot_addr);
        zb_get_extended_pan_id(zb_ext_pan_id);
        // Display extended PAN ID
        {
            uint8_t temp[8];
            for(uint8_t i = 0; i<8; i++)
            {
                temp[i] = zb_ext_pan_id[7-i];
            }
            LOG_HEXDUMP_DBG(temp,8,"Extended PAN ID: ");
        }
        switch(zb_get_network_role())
        {
        case 0:
            LOG_DBG("zigbee role coordinator\n");
            break;
        case 1:
            LOG_DBG("zigbee role router\n");
            break;
        case 2:
            LOG_DBG("zigbee role end device\n");
            break;
        default:
            LOG_DBG("Zigbee role NOT found \n");
            break;
        }

    zb_channel = zb_get_current_channel();
    LOG_DBG("zigbee channel: %d \n", zb_channel);

    }
}

//------------------------------------------------------------------------------
/**@brief This function prints the contents of multiple counters to the console.
 *
 *
 * @details Information is printed once per minute. Those counters contain information about
 *        the number of RF packets received, RF packets schedule for transmission,...
 *
 * @note Executed in the main loop
 *
 */
void display_counters(void)
{
    static uint64_t time_last_ms = 0;
    uint64_t time_now_ms = k_uptime_get();
    if( (uint64_t)( time_now_ms - time_last_ms ) > 60000 )
    {
        time_last_ms = time_now_ms;
        LOG_DBG("APS RX COUNTERS: Total %d, Binary %d, Commis %d",
                               aps_frames_received_total_counter,
                               aps_frames_received_binary_cluster_counter,
                               aps_frames_received_commissioning_cluster_counter);
        LOG_DBG("Uart frames: Tx %d, Rx %d",
                               tcu_uart_frames_transmitted_counter,
                               tcu_uart_frames_received_counter);
    }
}

//------------------------------------------------------------------------------
/**@brief Main function
 *
 */
int main(void)
{
    int8_t ret = 0;

    get_reset_reason();        // Read last reset reason

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
            if(ret <= 0)
            {
                LOG_ERR("zb_conf_read_from_nvram error %d", ret);
                g_b_flash_error = ZB_TRUE;
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

    zigbee_configuration(); //Zigbee configuration
    zigbee_enable(); // Start Zigbee default thread
    zb_af_set_data_indication(data_indication_cb); // Set call back function for APS frame received
    zb_aps_set_user_data_tx_cb(zigbee_aps_user_data_tx_cb); // Set call back function for APS frame transmitted
            
    LOG_INF("Router started successfully");

    while(1)
    {
        // run diagnostic functions
        diagnostic_toogle_pin();
        diagnostic_zigbee_info();
        display_counters();

        tcu_uart_transparent_mode_manager();   // Manage the frames received from the TCU uart when module is in transparent mode
        digi_node_discovery_request_manager(); // Manage the device discovery requests
        zigbee_aps_manager();                  // Manage the aps output frame queue

        nvram_manager();                       // Manage the NVRAM

        zigbee_thread_manager();               // Manage the Zigbee thread

        k_sleep(K_MSEC(5));                    // Required to see log messages on console
    }

    return 0;
}
