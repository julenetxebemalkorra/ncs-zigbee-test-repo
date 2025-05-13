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
#include "zigbee_bdb.h"
#include "zigbee_aps.h"
#include "Digi_At_commands.h"
#include "Digi_node_discovery.h"
#include "Digi_wireless_at_commands.h"
#include "nvram.h"
#include "Digi_fota.h"
#include "OTA_dfu_target.h"

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

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/storage/flash_map.h>

#define ZIGBEE_COORDINATOR_SHORT_ADDR 0x0000  // Update with actual coordinator address

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

//#define ZB_APS_MAX_IN_FRAGMENT_TRANSMISSIONS 5U  // Example: Increase to handle 5 fragments in parallel
//#define ZB_APS_ACK_WAIT_TIMEOUT             1000U // Example: Increase to 1000 ms

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Flag  used to print zigbee info once the device joins a network. */
#define PRINT_ZIGBEE_INFO                ZB_TRUE
#define PRINT_UART_INFO                  ZB_TRUE

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
#define WDT_NODE DT_ALIAS(watchdog0)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

/*UART Modbus and zigbee buffer size definitions*/
#define MODBUS_MIN_RX_LENGTH                8   // if the messagge has 8 bytes we consider it a modbus frame

uint32_t reset_cause; // Bit register containg the last reset cause.

uint16_t aps_frames_received_total_counter = 0;
uint16_t aps_frames_received_binary_cluster_counter = 0;
uint16_t aps_frames_received_commissioning_cluster_counter = 0;

uint16_t tcu_uart_frames_transmitted_counter = 0;
uint16_t tcu_uart_frames_received_counter = 0;


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
static int task_wdt_id = -1; // Task watchdog ID

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
        LOG_ERR("RESET:");    
        if (reset_cause & RESET_PIN)              LOG_WRN("Reset cause: RESET_PIN");
        if (reset_cause & RESET_SOFTWARE)         LOG_WRN("Reset cause: RESET_SOFTWARE");
        if (reset_cause & RESET_BROWNOUT)         LOG_WRN("Reset cause: RESET_BROWNOUT");
        if (reset_cause & RESET_POR)              LOG_WRN("Reset cause: RESET_POR");
        if (reset_cause & RESET_WATCHDOG)         LOG_WRN("Reset cause: RESET_WATCHDOG");
        if (reset_cause & RESET_DEBUG)            LOG_WRN("Reset cause: RESET_DEBUG");
        if (reset_cause & RESET_SECURITY)         LOG_WRN("Reset cause: RESET_SECURITY");
        if (reset_cause & RESET_LOW_POWER_WAKE)   LOG_WRN("Reset cause: RESET_LOW_POWER_WAKE");
        if (reset_cause & RESET_CPU_LOCKUP)       LOG_WRN("Reset cause: RESET_CPU_LOCKUP");
        if (reset_cause & RESET_PARITY)           LOG_WRN("Reset cause: RESET_PARITY");
        if (reset_cause & RESET_PLL)              LOG_WRN("Reset cause: RESET_PLL");
        if (reset_cause & RESET_CLOCK)            LOG_WRN("Reset cause: RESET_CLOCK");
        if (reset_cause & RESET_HARDWARE)         LOG_WRN("Reset cause: RESET_HARDWARE");
        if (reset_cause & RESET_USER)             LOG_WRN("Reset cause: RESET_USER");
        if (reset_cause & RESET_TEMPERATURE)      LOG_WRN("Reset cause: RESET_TEMPERATURE");
    
        if (reset_cause == 0) {
            LOG_WRN("No known reset causes detected");
        }

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

    hwinfo_clear_reset_cause(); // Clear the hardware flags. In that way we see only the cause of last reset

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
        LOG_ERR("Error: bufid is NULL data_indication_cb beggining");
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
        else if( ( ind->clusterid == DIGI_AT_COMMAND_CLUSTER ) &&
            ( ind->src_endpoint == DIGI_AT_COMMAND_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_AT_COMMAND_DESTINATION_ENDPOINT ) )
        {
            if( is_a_digi_read_at_command((uint8_t *)pointerToBeginOfBuffer, (uint16_t)sizeOfPayload) )
            {
                if(PRINT_ZIGBEE_INFO) LOG_DBG("Xbee read AT command received");
            }
        }
        else if( ( ind->clusterid == DIGI_FOTA_CLUSTER ) &&
            ( ind->src_endpoint == DIGI_BINARY_VALUE_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_BINARY_VALUE_SOURCE_ENDPOINT ) )
        {
            if( is_a_digi_fota_command((uint8_t *)pointerToBeginOfBuffer, (uint16_t)sizeOfPayload) )
            {
                if(PRINT_ZIGBEE_INFO) LOG_DBG("Xbee read FOTA related command received");
            }
        }
        else if( ( ind->clusterid == DIGI_AT_PING_CLUSTER ) &&
            ( ind->src_endpoint == DIGI_AT_PING_SOURCE_ENDPOINT ) &&
            ( ind->dst_endpoint == DIGI_AT_PING_DESTINATION_ENDPOINT ) )
        {
            if( is_a_ping_command((uint8_t *)pointerToBeginOfBuffer, (uint16_t)sizeOfPayload) )
            {
                if(PRINT_ZIGBEE_INFO) LOG_DBG("PING");
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
        // safe way to free buffer
        zb_osif_disable_all_inter();
        zb_buf_free(bufid);
        zb_osif_enable_all_inter();
    	return ZB_TRUE;
	}
}

static void task_wdt_callback(int channel_id, void *user_data)
{
    LOG_WRN("Task watchdog channel %d callback, thread: %s\n",
        channel_id, k_thread_name_get((k_tid_t)user_data));

    /*
     * If the issue could be resolved, call task_wdt_feed(channel_id) here
     * to continue operation.
     *
     * Otherwise we can perform some cleanup and reset the device.
     */

    LOG_WRN("Resetting device...\n");

    sys_reboot(SYS_REBOOT_COLD);
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
	}

    // Register this thread
    k_tid_t my_tid = k_current_get();

    LOG_INF("Registering task watchdog for thread: %p", my_tid);
    LOG_INF("Thread name: %s", k_thread_name_get(my_tid));

    /*
	 * Add a new task watchdog channel with custom callback function and
	 * the current thread ID as user data.
	*/
    // 60000U - 60 seconds timeout
    // task_wdt_callback - callback function to be called when the watchdog expires
    // my_tid - thread ID of the current thread
    
	task_wdt_id = task_wdt_add(60000U, task_wdt_callback , my_tid);
    if (task_wdt_id < 0) {
		LOG_ERR("task_wdt_add failed: %d", task_wdt_id);
		return task_wdt_id;
	}

	LOG_INF("Task WDT initialized with channel %d", task_wdt_id);

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
    static uint64_t time_last_ms_counter = 0;
    static uint64_t time_last_ms_thread_manager = 0;
    static uint64_t time_last_ms_wdt = 0;


    uint64_t time_now_ms = k_uptime_get();
    if( (uint64_t)( time_now_ms - time_last_ms_counter ) > 60000 )
    {
        time_last_ms_counter = time_now_ms;
        LOG_DBG("APS RX COUNTERS: Total %d, Binary %d, Commis %d",
                               aps_frames_received_total_counter,
                               aps_frames_received_binary_cluster_counter,
                               aps_frames_received_commissioning_cluster_counter);
        LOG_DBG("Uart frames: Tx %d, Rx %d",
                               tcu_uart_frames_transmitted_counter,
                               tcu_uart_frames_received_counter);

    }

    if( (uint64_t)( time_now_ms - time_last_ms_thread_manager ) > 1000 )
    {
        zigbee_thread_manager();               // Manage the Zigbee thread
        time_last_ms_thread_manager = time_now_ms;
    }
/*
    if ( (uint64_t)( time_now_ms - time_last_ms_wdt ) > 10000 )
    {
        int err = task_wdt_feed(task_wdt_id); // Feed the watchdog
        if (err != 0) {
            LOG_ERR("task_wdt_feed failed: %d", err);
        }
        time_last_ms_wdt = time_now_ms;
    }
 */



}

void check_boot_status(void)
{
    int swap_type = mcuboot_swap_type();
    switch (swap_type) {
        case BOOT_SWAP_TYPE_NONE:
            LOG_WRN("No swap pending.\n");
            break;
        case BOOT_SWAP_TYPE_TEST:
            LOG_WRN("New image in slot1 is scheduled for test.\n");
            break;
        case BOOT_SWAP_TYPE_PERM:
            LOG_WRN("New image will be made permanent.\n");
            break;
        case BOOT_SWAP_TYPE_REVERT:
            LOG_WRN("Reverting to previous image.\n");
            break;
        default:
            LOG_WRN("Unknown swap type: %d\n", swap_type);
            break;
    }

    struct mcuboot_img_header header;
    size_t header_size = sizeof(header);

    int ret = boot_read_bank_header(FIXED_PARTITION_ID(slot0_partition), &header, header_size);
    if (ret == 0) {
        if (header.mcuboot_version == 1) {
            LOG_INF("MCUBoot image header (v1):");
            LOG_INF("  Version: %d.%d.%d+%d",
                    header.h.v1.sem_ver.major,
                    header.h.v1.sem_ver.minor,
                    header.h.v1.sem_ver.revision,
                    header.h.v1.sem_ver.build_num);
            LOG_INF("  Image size:  %u bytes", header.h.v1.image_size);
        } else {
            LOG_WRN("Unsupported mcuboot_version: %u", header.mcuboot_version);
        }
    } else {
        LOG_ERR("Failed to read MCUBoot header, error: %d", ret);
    }

    // Optionally log trailer offset
    ssize_t trailer_offset = boot_get_area_trailer_status_offset(0);
    if (trailer_offset >= 0) {
        LOG_INF("Trailer status offset: 0x%08zx", trailer_offset);
    } else {
        LOG_ERR("Failed to get trailer offset");
    }
}

/**
 * @brief Confirms the current firmware image.
    int swap_type = mcuboot_swap_type();
    if (swap_type < 0) {
        LOG_ERR("Failed to get swap type: %d\n", swap_type);
    } else if (swap_type == BOOT_SWAP_TYPE_REVERT) {
        int rc = boot_write_img_confirmed();
        LOG_WRN("Confirmed image: %d\n", rc);
    }
 */
static void confirm_image(void)
{
	if (!boot_is_img_confirmed()) {
		int ret = boot_write_img_confirmed();

		if (ret) {
			LOG_ERR("Couldn't confirm image: %d", ret);
		} else {
			LOG_INF("Marked image as OK");
		}
	}
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
    digi_wireless_at_init();
    digi_fota_init();
/*
    ret = watchdog_init();
    if( ret < 0)
    {
        LOG_ERR("watchdog_init error %d", ret);
    }
*/
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

    confirm_image(); // Confirm the image if it is not already confirmed

    check_boot_status(); // Check the boot status

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

        tcu_uart_transparent_mode_manager();     // Manage the frames received from the TCU uart when module is in transparent mode
        digi_node_discovery_request_manager();  // Manage the device discovery requests
        digi_wireless_read_at_command_manager(); // Manage the read AT commands received through Zigbee
        digi_fota_manager();                     // FUOTA state machine
        zigbee_aps_manager();                    // Manage the aps output frame queue
        nvram_manager();                         // Manage the NVRAM
        tcu_uart_manager();                     // Manage the TCU UART

        k_sleep(K_MSEC(5));                    // Required to see log messages on console
    }

    return 0;
}
