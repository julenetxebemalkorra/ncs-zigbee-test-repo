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

#include <zephyr/drivers/watchdog.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <stdbool.h>

#include <zboss_api_addons.h>
#include "zb_config.h"
#include "zb_address.h"
#include "zboss_api_buf.h"
#include "zb_types.h"
#include "zboss_api_zgp.h"

#include <zephyr/debug/coredump.h>

#define ZIGBEE_COORDINATOR_SHORT_ADDR 0x0000  // Update with actual coordinator address

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

//#define ZB_APS_MAX_IN_FRAGMENT_TRANSMISSIONS 5U  // Example: Increase to handle 5 fragments in parallel
//#define ZB_APS_ACK_WAIT_TIMEOUT             1000U // Example: Increase to 1000 ms

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON             1

/* Flag  used to print zigbee info once the device joins a network. */
#define PRINT_ZIGBEE_INFO                ZB_TRUE
#define PRINT_UART_INFO                  ZB_TRUE
#define CRYPTO_ENABLE                    ZB_TRUE
#define DEFAULT_ENCRIPTION_KEY           0
#define NEW_ENCRIPTION_KEY               1

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

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

// Get reference to watchdog device
static const struct device *const hw_wdt_dev = DEVICE_DT_GET_OR_NULL(WDT_NODE);
int task_wdt_id;

/* Zigbee messagge info*/
static bool b_infit_info_flag = PRINT_ZIGBEE_INFO;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// boolean flags for detecting modbus request handling
bool b_Zigbe_Connected = false;
bool bTimeToSendFrame = false;

static struct xbee_parameters_t xbee_parameters; // Xbee's parameters


/*----------------------------------------------------------------------------*/
/*                           FUNCTION DEFINITIONS                             */
/*----------------------------------------------------------------------------*/

/**@brief Function to read the reason for the last reset. The reason is printed in the console. */
void get_reset_reason(void)
{
    int8_t rc = 0;
    uint8_t restart_number[1] = {0};
    uint8_t reset_cause_flags[1] = {0};

    const zb_char_t *zb_version;
    // Call the zb_get_version function to get the ZBOSS version string
    zb_version = zb_get_version();

    // Print the version string
    LOG_DBG("ZBOSS Version: %s\n", zb_version);

    zb_uint8_t max_children_allowed = zb_get_max_children();

    LOG_DBG("Max children allowed: %d\n", max_children_allowed);

	int result = hwinfo_get_reset_cause(&reset_cause);

    if (result == 0) // Success, reset_cause now contains the reset cause flags
	{
        LOG_ERR("RESET");

		if (reset_cause & RESET_BROWNOUT) LOG_WRN("Reset cause is: RESET_BROWNOUT\n"); // 2
        else if (reset_cause & RESET_PIN) LOG_DBG("Reset cause is: RESET_PIN\n"); // 0
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
        else if (reset_cause & RESET_SOFTWARE) LOG_DBG("Reset cause is: RESET_SOFTWARE\n"); // 1
		else LOG_DBG("\n\n reset cause is: %d\n\n",reset_cause);

    	hwinfo_clear_reset_cause(); // Clear the hardware flags. In that way we see only the cause of last reset
    } 
    else if (result == -ENOSYS) 
	{
		LOG_ERR("\n\n there is no implementation for the particular device.\n\n");
    } 
    else 
	{
		LOG_ERR("\n\n negative value on driver specific errors\n\n");
    }
    rc = read_nvram(RBT_CNT_ID, restart_number, sizeof(restart_number));
    if (rc <= 0)     
    {
        LOG_ERR("read_nvram error %d", rc);
    }
    else
    {
        restart_number[0]++;
        write_nvram(RBT_CNT_ID, restart_number, sizeof(restart_number));
        LOG_WRN("restart_number, %d\n", restart_number[0]);
    }

    rc = read_nvram(RBT_CNT_REASON, reset_cause_flags, sizeof(reset_cause_flags));
    if (rc <= 0)     
    {
        LOG_ERR("read_nvram error %d", rc);
    }
    else
    {
        write_nvram(RBT_CNT_REASON, (uint8_t *)&reset_cause, sizeof(reset_cause));
        LOG_WRN("reset_cause_flags, %d\n", reset_cause_flags[0]);
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
    if(!bufid)
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
        LOG_ERR("Error: bufid is NULL data_indication_cb beggining: bufid status %d", zb_buf_get_status(bufid));
        return ZB_TRUE;
    } 
    else
	{
        zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header

        zb_uint8_t *pointerToBeginOfBuffer;
        zb_uint8_t *pointerToEndOfBuffer;
        zb_int32_t sizeOfPayload;
        pointerToBeginOfBuffer = zb_buf_begin(bufid);
        pointerToEndOfBuffer = zb_buf_end(bufid);
        sizeOfPayload = pointerToEndOfBuffer - pointerToBeginOfBuffer;

        if(PRINT_ZIGBEE_INFO) {
            LOG_DBG("Rx APS Frame with profile 0x%x, cluster 0x%x, src_ep %d, dest_ep %d, payload %d bytes, status %d",
                                                    (uint16_t)ind->profileid, (uint16_t)ind->clusterid, (uint8_t)ind->src_endpoint,
                                                    (uint8_t)ind->dst_endpoint, (uint16_t)sizeOfPayload, zb_buf_get_status(bufid));
            ////LOG_DBG("  Rx APS Frame:\n");
            //LOG_DBG("  Frame control (fc): %d\n", ind->fc);
            ////LOG_DBG("  Destination address: 0x%04X\n", ind->dst_addr);
            ////LOG_DBG("  Group address: 0x%04X\n", ind->group_addr);
            //LOG_DBG("  APS counter: %d\n", ind->aps_counter);
            ////LOG_DBG("  MAC source address: 0x%04X\n", ind->mac_src_addr);
            ////LOG_DBG("  MAC destination address: 0x%04X\n", ind->mac_dst_addr);
            ////LOG_DBG("  LQI: %d\n", ind->lqi);
            ////LOG_DBG("  RSSI: %d\n", ind->rssi);
            ////LOG_DBG("  APS key source: %d\n", ind->aps_key_source);
            ////LOG_DBG("  APS key attributes: %d\n", ind->aps_key_attrs);
            ////LOG_DBG("  APS key from TC: %d\n", ind->aps_key_from_tc);
            //LOG_DBG("  Extended frame control: %d\n", ind->extended_fc);
            //LOG_DBG("  Reserved: %d\n", ind->reserved);
            //LOG_DBG("  Transaction sequence number (TSN): %d\n", ind->tsn);
            //LOG_DBG("  Block number: %d\n", ind->block_num);
            //LOG_DBG("  Block acknowledgment: %d\n", ind->block_ack);
            ////LOG_DBG("  Radius: %d\n", ind->radius);
            ////LOG_DBG("  Payload size: %d bytes\n", sizeOfPayload);
            ////LOG_DBG("  Status: %d\n", zb_buf_get_status(bufid));
        }

        aps_frames_received_total_counter++;
        if( ind->clusterid == DIGI_BINARY_VALUE_CLUSTER )aps_frames_received_binary_cluster_counter++;
        if( ind->clusterid == DIGI_COMMISSIONING_CLUSTER )aps_frames_received_commissioning_cluster_counter++;

        if( (ind->clusterid == DIGI_BINARY_VALUE_CLUSTER) &&
            ( ind->src_endpoint == DIGI_BINARY_VALUE_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_BINARY_VALUE_DESTINATION_ENDPOINT ) )
        {
            if ((sizeOfPayload > 0) && (sizeOfPayload < UART_RX_BUFFER_SIZE))
            {
                if(PRINT_ZIGBEE_INFO)
                {
                    LOG_DBG("Size of received payload is %d bytes \n", sizeOfPayload);
                    LOG_HEXDUMP_DBG(pointerToBeginOfBuffer,sizeOfPayload,"Payload of input RF packet");
                }
                

                if( !is_tcu_uart_in_command_mode() && (sizeOfPayload >= MODBUS_MIN_RX_LENGTH) )
                {
                    if (queue_zigbee_Message(pointerToBeginOfBuffer, sizeOfPayload) == SUCCESS) // Assuming queueMessage returns SUCCESS or error
                    {
                        tcu_uart_frames_transmitted_counter++;
                        //if (PRINT_ZIGBEE_INFO) LOG_DBG("Payload of input RF packet sent to TCU UART: counter %d", tcu_uart_frames_transmitted_counter);
                    }
                    else
                    {
                        LOG_ERR("Failed to send payload to TCU UART");
                    }                    
                }
                else
                {
                    if(PRINT_ZIGBEE_INFO) LOG_ERR("Payload of input RF packet NOT sent to TCU UART: counter %d", tcu_uart_frames_transmitted_counter);
                }
            }
            else
            {
                if(PRINT_ZIGBEE_INFO) LOG_ERR("Payload of input RF packet is too big or too small: %d bytes", sizeOfPayload);
            }
        }

        else if( ( ind->clusterid == DIGI_COMMISSIONING_CLUSTER ) &&
            ( ind->src_endpoint == DIGI_COMMISSIONING_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_COMMISSIONING_DESTINATION_ENDPOINT ) )
        {
            if( is_a_digi_node_discovery_request((uint8_t *)pointerToBeginOfBuffer, (uint16_t)sizeOfPayload) )
            {
                if(PRINT_ZIGBEE_INFO) LOG_DBG("Xbee Node Discovery Device Request");
            }
        }
        else
        {
            if(PRINT_ZIGBEE_INFO) LOG_ERR("Cluster ID not found");
            if(PRINT_ZIGBEE_INFO) LOG_WRN("Rx APS Frame with profile 0x%x, cluster 0x%x, src_ep %d, dest_ep %d, payload %d bytes",
                          (uint16_t)ind->profileid, (uint16_t)ind->clusterid, (uint8_t)ind->src_endpoint,(uint8_t)ind->dst_endpoint, (uint16_t)sizeOfPayload);
            if(PRINT_ZIGBEE_INFO)
                {
                    LOG_DBG("Size of received payload is %d bytes \n", sizeOfPayload);
                    LOG_HEXDUMP_DBG(pointerToBeginOfBuffer,sizeOfPayload,"Payload of input RF packet");
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
    LOG_ERR("Error: bufid is NULL data_indication_cb end");
	return ZB_TRUE;
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
    int ret = 0;

    if(ZB_GET_APP_SIGNAL_STATUS(bufid) != 0)
    {
        LOG_ERR("Signal %d failed, status %d", sig, ZB_GET_APP_SIGNAL_STATUS(bufid));
        return;
    }

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

            ret = ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);
            if(ret != RET_OK)
            {
                LOG_ERR("zb_bdb_reset_via_local_action failed, ret %d", ret);
            }

            LOG_WRN( "The device will perform the NLME leave and clean all Zigbee persistent data except the outgoing NWK frame counter and application datasets");

            soft_reset_counter = 0;
            hard_reset_counter++;
        }
        if(hard_reset_counter >= RESTART_ATEMP_COUNT_MAX)
        {
            LOG_ERR( "device restarted since it is not able to join any network");
            
            // Reboot the device
		    sys_reboot(0);
            g_b_reset_zigbee_cmd = ZB_TRUE;
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

//------------------------------------------------------------------------------
/**@brief Function for initializing the watchdog task.
 *      The watchdog task is used to monitor the main loop and reset the device if it gets stuck.
 * @retval -1 Error
 * @retval 0 OK
 */
static int8_t watchdog_init(void)
{
	int ret;

    if (!device_is_ready(hw_wdt_dev)) {
		LOG_ERR("Hardware watchdog not ready; ignoring it.\n");
        return -1;
	}
    /* This function sets up necessary kernel timers and the hardware watchdog 
    * (if desired as fallback). It has to be called before task_wdt_add() and task_wdt_feed().
    * 0 – If successful.
    * -ENOTSUP – If assigning a hardware watchdog is not supported.
    * -Errno – Negative errno if the fallback hw_wdt is used and the install timeout API
    * fails. See wdt_install_timeout() API for possible return values.
    * */
    ret = task_wdt_init(hw_wdt_dev);
	if (ret != 0) {
		LOG_ERR("task wdt init failure: %d\n", ret);
		return ret;
	}

    /*
	 * Add a new task watchdog channel with custom callback function and
	 * the current thread ID as user data.
	 */
	task_wdt_id = task_wdt_add(1000U, NULL, NULL);

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

    //TRUE to disable trust center, FALSE to enable trust center
    zb_bdb_set_legacy_device_support(ZB_FALSE);

    // Define a distributed key thsi is Zigbee Alliance key
    zb_uint8_t network_key[16] = {0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};
    zb_uint8_t network_link_key[16] = {0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, 0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};

    LOG_WRN("Link key: ");
    LOG_HEXDUMP_DBG(network_link_key, 16, " ");

    LOG_WRN("Network key: ");
    LOG_HEXDUMP_DBG(network_key, 16, " ");

    zb_conf_get_network_key(network_link_key);

    LOG_WRN("Link key: ");
    LOG_HEXDUMP_DBG(network_link_key, 16, " ");

    // Set the network link key. This action can help us choose between different link keys
    zb_zdo_set_tc_standard_distributed_key(network_link_key);

    // Enable distributed Trust Center
    zb_enable_distributed();
    zb_zdo_setup_network_as_distributed();

    // Set the network key Only for development proccess
    zb_secur_setup_nwk_key(network_key, DEFAULT_ENCRIPTION_KEY);

    set_extended_pan_id_in_stack();

    // Set the device nwkey. This action can help us choose between different nwk keys
    zb_secur_nwk_key_switch_procedure(DEFAULT_ENCRIPTION_KEY);

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

/* Callback for neighbor table response */
void get_lqi_cb(zb_uint8_t param)
{
    zb_bufid_t buf = param;
    zb_uint8_t *zdp_cmd = zb_buf_begin(buf);
    zb_zdo_mgmt_lqi_resp_t *resp = (zb_zdo_mgmt_lqi_resp_t *)(zdp_cmd);
    zb_zdo_neighbor_table_record_t *record = (zb_zdo_neighbor_table_record_t *)(resp + 1);
    zb_uint_t i;

    LOG_INF("LQI callback status: %hd, neighbor_table_entries: %hd, start_index: %hd, neighbor_table_list_count: %d",
        resp->status, resp->neighbor_table_entries, resp->start_index, resp->neighbor_table_list_count);

    /* Iterate over the list of neighbors */
    for (i = 0; i < resp->neighbor_table_list_count; i++)
    {
        LOG_INF("#%hd: Long Addr: ", i);
        log_ext_address(record->ext_addr);  // Print extended address
        LOG_INF("PAN ID: ");
        log_ext_address(record->ext_pan_id);  // Print PAN ID

        LOG_INF("Network Addr: %d, Dev Type: %hd, Rx On Idle: %hd, Relationship: %hd, Permit Join: %hd, Depth: %hd, LQI: %hd",
            record->network_addr,
            ZB_ZDO_RECORD_GET_DEVICE_TYPE(record->type_flags),
            ZB_ZDO_RECORD_GET_RX_ON_WHEN_IDLE(record->type_flags),
            ZB_ZDO_RECORD_GET_RELATIONSHIP(record->type_flags),
            record->permit_join,
            record->depth,
            record->lqi);

        record++;
    }

    zb_buf_free(buf);  // Free buffer after processing
}


/* Send LQI (Neighbor Table) request */
void send_lqi_request(zb_bufid_t buf)
{
    zb_uint8_t tsn;
    zb_zdo_mgmt_lqi_param_t *req_param;

    req_param = ZB_BUF_GET_PARAM(buf, zb_zdo_mgmt_lqi_param_t);
    req_param->start_index = 0;
    req_param->dst_addr = ZIGBEE_COORDINATOR_SHORT_ADDR;

    tsn = zb_zdo_mgmt_lqi_req(buf, get_lqi_cb);
    LOG_INF("LQI request sent, TSN: %d", tsn);
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

        /* Create buffer to send LQI request */
        /*
        zb_bufid_t buf = zb_buf_get_out();
        if (buf)
        {
            //Send LQI request to coordinator to get neighbor table
            send_lqi_request(buf);
        }
        else
        {
            LOG_ERR("Failed to get buffer for LQI request");
        }
        */
    }

    if( (uint64_t)( time_now_ms - time_last_ms ) > 1000 )
    {
        zigbee_thread_manager();               // Manage the Zigbee thread
    }


    if (zb_buf_is_oom_state()) {
        // Handle Out Of Memory state
        LOG_ERR("Buffer pool is out of memory!\n");
    }
    if (zb_buf_memory_low()) {
        // Handle low memory state
        LOG_WRN("Warning: Buffer pool memory is running low!\n");
    }

    // Trace the buffer statistics for debugging purposes
    zb_buf_oom_trace();
}


//------------------------------------------------------------------------------
/**@brief Main function
 *
 */
int main(void)
{
    int8_t ret = 0;

    LOG_WRN("Starting Zigbee Router");

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
                g_b_flash_error = ZB_TRUE;
            }
            else if(ret == 0)
            {
                LOG_INF("NVRAM data read successfully");
                g_b_flash_error = ZB_FALSE;
            }
            else
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

    get_reset_reason();        // Read last reset reason

    zigbee_aps_init();
    digi_at_init();
    digi_node_discovery_init();

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

    while(1)
    {
        // run diagnostic functions
        if(PRINT_ZIGBEE_INFO)
        {
            diagnostic_toogle_pin();
            diagnostic_zigbee_info();
            display_counters();
        }

		task_wdt_feed(task_wdt_id); // Feed the watchdog

        tcu_uart_transparent_mode_manager();   // Manage the frames received from the TCU uart when module is in transparent mode
        digi_node_discovery_request_manager(); // Manage the device discovery requests
        zigbee_aps_manager();                  // Manage the aps output frame queue
        nvram_manager();                       // Manage the NVRAM
        tcu_uart_manager();                   // Manage the TCU UART

        k_sleep(K_MSEC(5));                    // Required to see log messages on console
    }

    return 0;
}
