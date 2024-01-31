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

#include "tcu_Uart.h"

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               232

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON             1

/* Flag  used to print zigbee info once the device joins a network. */
#define PRINT_ZIGBEE_INFO             	ZB_TRUE
#define PRINT_UART_INFO             	ZB_TRUE

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*UART Modbus and zigbee buffer size definitions*/
#define MODBUS_MIN_RX_LENGTH             8   // if the messagge has 8 bytes we consider it a modbus frame
#define MAX_ZIGBEE_PAYLOAD_SIZE 		 82 // Maximum payload size that can be sent at once via zigbee

/*Zigbee NWK configuration
Note: This part was added to wireshark encription issues
*/
#define ZB_ZGP_DEFAULT_SHARED_SECURITY_KEY_TYPE ZB_ZGP_SEC_KEY_TYPE_NWK
#define ZB_ZGP_DEFAULT_SECURITY_LEVEL ZB_ZGP_SEC_LEVEL_FULL_WITH_ENC
#define ZB_STANDARD_TC_KEY { 0x81, 0x42, < rest of key > };
#define ZB_DISTRIBUTED_GLOBAL_KEY { 0x81, 0x42, , < rest of key > };
#define ZB_TOUCHLINK_PRECONFIGURED_KEY { 0x81, 0x42, < rest of key > };

uint32_t reset_cause; // Bit register containg the last reset cause.

uint16_t correct_RF_packets_received_counter = 0;
uint16_t tcu_uart_frames_received_counter = 0;
uint16_t correct_RF_packets_received_counter_old = 0;
uint16_t tcu_uart_frames_received_counter_old = 0;

//
static const zb_uint8_t ext_pan_id[8] = {0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const zb_uint8_t * ptr_ext_pan_id = ext_pan_id;

static volatile uint16_t debug_led_ms_x10 = 0; // 10000 ms timer to control the debug led

uint8_t tcu_transmitted_frames_counter = 0;
	
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Get a reference to the TIMER1 instance
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(1);

/* UART configuration variables*/
volatile char UART_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t UART_rx_buffer_index;
volatile uint16_t UART_rx_buffer_index_max;

static volatile size_t offset = 0;

static volatile	zb_uint8_t *pointerToBeginOfBuffer;

/* Zigbee messagge info*/
static volatile uint16_t ZB_profileid;
static volatile uint16_t ZB_clusterid;
static volatile uint16_t ZB_src_endpoint;
static volatile uint16_t ZB_dst_endpoint;
static volatile uint16_t ZB_aps_counter;

static bool b_infit_info_flag = PRINT_ZIGBEE_INFO;

/*  This macro normally must be used after including <zephyr/logging/log.h> to
 * complete the initialization of the module. 
 
 Note: it may be removed
 */
LOG_MODULE_REGISTER(ZB_router_app, LOG_LEVEL_DBG);

// boolean flags for detecting modbus request handling
volatile bool b_Modbus_Ready_to_send = false;
bool b_Zigbe_Connected = false;
bool bTimeToSendFrame = false;

/* Main application customizable context.
 * Stores all settings and static values.
 */
struct zb_device_ctx {
	zb_zcl_basic_attrs_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
};

/* Zigbee device application context storage. */
static struct zb_device_ctx dev_ctx;

/*Declare attribute list for Identify cluster*/
ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

/*Declare Basic attribute list*/
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.power_source);

/*Declare cluster list for Range extender device*/
ZB_DECLARE_RANGE_EXTENDER_CLUSTER_LIST(
	app_template_clusters,
	basic_attr_list,
	identify_attr_list);

/*Declare endpoint for Range extender device*/
ZB_DECLARE_RANGE_EXTENDER_EP(
	app_template_ep,
	APP_TEMPLATE_ENDPOINT,
	app_template_clusters);

/*Declare application's device context for single-endpoint device*/
ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	app_template_ctx,
	app_template_ep);

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


/**@brief Function for initializing all clusters attributes. */
static void app_clusters_attr_init(void)
{
	/* Basic cluster attributes data */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source = TEMPLATE_INIT_BASIC_POWER_SOURCE;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
}

/**@brief Starts identifying the device.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void start_identifying(zb_bufid_t bufid)
{
	ZVUNUSED(bufid);

	if (ZB_JOINED()) {
		/* Check if endpoint is in identifying mode,
		 * if not put desired endpoint in identifying mode.
		 */
		if (dev_ctx.identify_attr.identify_time ==
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE) {

			zb_ret_t zb_err_code = zb_bdb_finding_binding_target(
				APP_TEMPLATE_ENDPOINT);

			if (zb_err_code == RET_OK) {
				LOG_DBG("Enter identify mode");
			} else if (zb_err_code == RET_INVALID_STATE) {
				LOG_ERR("RET_INVALID_STATE - Cannot enter identify mode");
			} else {
				ZB_ERROR_CHECK(zb_err_code);
			}
		} else {
			LOG_INF("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	} else {
		LOG_ERR("Device not in a network - cannot enter identify mode");
	}
}

//------------------------------------------------------------------------------
/**@brief Callback to call when AF got APS packet.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 *                      
 */
zb_uint8_t data_indication(zb_bufid_t bufid)
{
 	zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header
    
    if (bufid)
	{
        if( (ind->clusterid == 0x0011) && ( ind->src_endpoint == 232 ) && ( ind->dst_endpoint == 232 ) )
        {
            zb_uint8_t *pointerToEndOfBuffer;
            zb_int32_t sizeOfPayload;
            pointerToBeginOfBuffer = zb_buf_begin(bufid);
            pointerToEndOfBuffer = zb_buf_end(bufid);
            sizeOfPayload = pointerToEndOfBuffer - pointerToBeginOfBuffer;

            if ((sizeOfPayload > 0) && (sizeOfPayload < UART_RX_BUFFER_SIZE))
            {
                correct_RF_packets_received_counter ++;
                if(PRINT_ZIGBEE_INFO)
                {
                    LOG_DBG("Size of received payload is %d bytes \n", sizeOfPayload);
                    LOG_HEXDUMP_DBG(pointerToBeginOfBuffer,sizeOfPayload,"Payload of input RF packet");
                    //LOG_DBG("\n Frame control field: %d \n", pointerToBeginOfBuffer[0]);
                    //LOG_DBG("Sequence number: %d - \n", pointerToBeginOfBuffer[1]);
                    //LOG_DBG("Zigbee Command: 0x%02x - \n", pointerToBeginOfBuffer[2]);
                    //LOG_DBG("ind APS counter %d \n", ind->aps_counter);
                }

                if(sizeOfPayload >= MODBUS_MIN_RX_LENGTH)
                {
                    if(PRINT_ZIGBEE_INFO)
                    {
                        LOG_DBG("bModbusRequestReceived via zigbee and send via UART \n");
                    }
                    // safe zigbee source endpoint info to send the asnwer
                    ZB_profileid = ind->profileid;
                    ZB_clusterid = ind->clusterid;
                    ZB_src_endpoint = ind->src_endpoint;
                    ZB_dst_endpoint = ind->dst_endpoint;
                    ZB_aps_counter = ind->aps_counter;

                    //sendFrameToTcu(&pointerToBeginOfBuffer[0], sizeOfPayload);
                    sendFrameToTcu((uint8_t *)pointerToBeginOfBuffer, sizeOfPayload);
 
                }
            }

        }
	}

    if (bufid)
    {
		zb_buf_free(bufid); // JESUS: I don't know if this is needed, but I try, just in case.
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

 	if (ZB_GET_APP_SIGNAL_STATUS(bufid) == 0)
 	{

		switch (sig) {
		case ZB_ZDO_SIGNAL_DEFAULT_START:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "Zigbee Device STARTED OK\n");
 		break;
		case ZB_ZDO_SIGNAL_SKIP_STARTUP:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_SKIP_STARTUP\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_ANNCE:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_DEVICE_ANNCE\n");
		break;
		case ZB_ZDO_SIGNAL_LEAVE:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_LEAVE\n");
		break;
		case ZB_ZDO_SIGNAL_ERROR:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_ERROR\n");
		break;
		case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_DEVICE_FIRST_START\n");
		break;
		case ZB_BDB_SIGNAL_DEVICE_REBOOT:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_DEVICE_REBOOT\n");
		break;
		case ZB_BDB_SIGNAL_STEERING:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_STEERING\n");
			start_identifying(bufid);
		break;
		case ZB_BDB_SIGNAL_FORMATION:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_FORMATION\n");
		break;
		case ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED\n");
		break;
		case ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED\n");
		break;
		case ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_NWK_SIGNAL_DEVICE_ASSOCIATED\n");
		break;
		case ZB_ZDO_SIGNAL_LEAVE_INDICATION:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_LEAVE_INDICATION\n");
		break;
		case ZB_BDB_SIGNAL_WWAH_REJOIN_STARTED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_WWAH_REJOIN_STARTED\n");
		break;
		case ZB_ZGP_SIGNAL_COMMISSIONING:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZGP_SIGNAL_COMMISSIONING\n");
		break;
		case ZB_COMMON_SIGNAL_CAN_SLEEP:
			//LOG_DBG( "JULEN ZB_COMMON_SIGNAL_CAN_SLEEP\n");
		break;
		case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY\n");
		break;
		case ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_UPDATE:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_ZDO_SIGNAL_DEVICE_UPDATE\n");
		break;
		case ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED\n");
		break;
		case ZB_NLME_STATUS_INDICATION:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_NLME_STATUS_INDICATION\n");
		break;
		case ZB_TCSWAP_DB_BACKUP_REQUIRED_SIGNAL:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_TCSWAP_DB_BACKUP_REQUIRED_SIGNAL\n");
		break;
		case ZB_BDB_SIGNAL_TC_REJOIN_DONE:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_TC_REJOIN_DONE\n");
		break;
		case ZB_BDB_SIGNAL_STEERING_CANCELLED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_STEERING_CANCELLED\n");
		break;
		case ZB_BDB_SIGNAL_FORMATION_CANCELLED:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_BDB_SIGNAL_FORMATION_CANCELLED\n");
		break;
		case ZB_SIGNAL_READY_TO_SHUT:
			if(PRINT_ZIGBEE_INFO) LOG_DBG( "JULEN ZB_SIGNAL_READY_TO_SHUT\n");
		break;
 		default:
			//LOG_DBG( "JULEN Unknown signal\n");
			zb_af_set_data_indication(data_indication);
 		break;
 		}
	}

	/* No application-specific behavior is required.
	 * Call default signal handler.
	 */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	/* All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid) {
		zb_buf_free(bufid);
	}
}

//------------------------------------------------------------------------------
/**@brief Zigbee helper function  to send user payload
 *
 * @param[in]   outputPayload   Reference to the user payload 
 * @param[in]   chunk_size   payload size to send 
 *
 */
void send_user_payload(zb_uint8_t *outputPayload ,size_t chunk_size)
{
	    /*Allocate OUT buffer of the default size.*/
		zb_bufid_t bufid;
		bufid = zb_buf_get_out();

		/*destination address: for the first test the network coordinator will be the destination */
		zb_addr_u dst_addr;
		dst_addr.addr_short = 0x0000;

		if(PRINT_UART_INFO)
		{
			LOG_DBG("send_user_payload: chunk_size %d\n", chunk_size);
            LOG_HEXDUMP_DBG(outputPayload,chunk_size,"Payload of output RF packet");
		}

	    //zb_uint8_t outputCustomPayload[255] = {0xC2, 0x04, 0x04, 0x00, 0x4A, 0x75, 0x6C, 0x65, 0x6E, 0x4A, 0x75, 0x6C, 0x65, 0x6E};

		/*zb_aps_send_user_payload(bufid, 
		    					 dst_addr,
			    				 0xc105,
				    			 0X0011,
					    		 232,
						    	 232,
								 ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		    					 ZB_FALSE,
			    				 outputPayload,
				    			 9); */
		zb_ret_t ret = zb_aps_send_user_payload(bufid, 
		    					                dst_addr,
			    				                ZB_profileid,
				    			                ZB_clusterid,
					    		                ZB_src_endpoint,
						    	                ZB_dst_endpoint,
								                ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		    					                ZB_FALSE,
			    				                outputPayload,
				    			                chunk_size);
        if(bufid)
        {
            if(ret == RET_OK) LOG_DBG("RET_OK - if transmission was successful scheduled;\n");
		    else if(ret == RET_INVALID_PARAMETER_1) LOG_ERR("RET_INVALID_PARAMETER_1 - if the buffer is invalid\n");
		    else if(ret == RET_INVALID_PARAMETER_2) LOG_ERR("RET_INVALID_PARAMETER_2 - if the payload_ptr parameter is invalid;\n");
		    else if(ret == RET_INVALID_PARAMETER_3) LOG_ERR("RET_INVALID_PARAMETER_3 - if the payload_size parameter is too large;\n");
		    else LOG_ERR("Unkown error zb_aps_send_user_payload ;\n");
        }
        else
        {
            LOG_ERR("\n\n bufid NULL error ZB not answered \n\n");
        }

	/*Free packet buffer and put it into free list after payload is sent*/
	if (bufid) 
	{
	    zb_buf_free(bufid);
	}

/*
  Note: This code includes a one-second delay between Zigbee package transmissions for stability.

  To adjust the delay time:
  - Locate the 'sleep' function.
  - Modify the argument value in 'sleep' for a different delay duration.
  - Test the code after changing the delay to ensure proper functionality.
  
  Caution: Be mindful when altering the delay to prevent network congestion.
*/
	//k_msleep(SLEEP_TIME_MS);

}

//------------------------------------------------------------------------------
/**@brief Function to send the received modbus answer. Logic to split the messages in MAX_ZIGBEE_PAYLOAD_SIZE
 *
 *
 */
void send_zigbee_modbus_answer(void)
{
	zb_uint8_t outputPayload[MAX_ZIGBEE_PAYLOAD_SIZE];

	size_t remaining_length = UART_rx_buffer_index_max +1;

    // Transmit the modified payload in chunks via Zigbee
	while(remaining_length > 0) 
	{
    	size_t chunk_size = MIN(remaining_length, MAX_ZIGBEE_PAYLOAD_SIZE);

		memcpy(outputPayload, &UART_rx_buffer[offset], chunk_size);
/**
		for (uint8_t i = 0; i < chunk_size; i++)
		{
		LOG_DBG("%c- ", outputPayload[i]);
		}
*/
		//send_user_payload(&outputPayload, chunk_size);
        send_user_payload(outputPayload, chunk_size);

		// Update offset and remaining length for the next chunk
    	offset += chunk_size;
    	remaining_length -= chunk_size;
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

// Function for initializing the Zigbee configuration
void zigbee_configuration()
{
	/* disable NVRAM erasing on every application startup*/
	zb_set_nvram_erase_at_start(ZB_TRUE);

	/* Register device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&app_template_ctx);

	// Define a distributed key (assuming key size of 16 bytes, you might need to adjust based on documentation)
    zb_uint8_t distributed_key[16] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};

    // Set the network key
	zb_secur_setup_nwk_key((zb_uint8_t *) distributed_key, 0);

	zb_set_extended_pan_id(ptr_ext_pan_id);

	//TRUE to disable trust center, legacy support for 
	zb_bdb_set_legacy_device_support(ZB_TRUE);
	
	//Call Function for initializing all clusters attributes
	app_clusters_attr_init();
}

// Function for giagnostic purposes. toogle the diagnostic led every second to be sure HW is OK and app is running
void diagnostic_toogle_pin()
{
    if(debug_led_ms_x10 >= 10000)
    {
        debug_led_ms_x10 = 0;
        gpio_pin_toggle_dt(&led);
    }
}

// Function to join the Zigbee network
void get_endpoint_descriptor(zb_uint8_t endpoint) 
{
	
	// After successful joining, get the endpoint descriptor for a specific endpoint ID
    zb_af_endpoint_desc_t *endpoint_desc = zb_af_get_endpoint_desc(endpoint);

    // Check if the endpoint descriptor is found
    if (endpoint_desc != NULL) {
        // Successfully obtained the endpoint descriptor
        // Access endpoint descriptor fields as needed
        zb_uint8_t profile_id = endpoint_desc->simple_desc->app_profile_id;
        zb_uint16_t device_id = endpoint_desc->simple_desc->app_device_id;
		zb_uint16_t device_version = endpoint_desc->simple_desc->app_device_version;
		
		LOG_DBG("Profileid 0x%04x \n", profile_id);
    	LOG_DBG("Clusterid 0x%04x \n", device_id);
		LOG_DBG("device_version 0x%04x \n", device_version);

    } else 
	{
    	LOG_ERR("endpoint descriptor not available\n");
    }
}

// Function for giagnostic purposes. Print zigbee info when the device joins a network
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
		// Log Long Address
		LOG_DBG("zigbee long addr: ");
		for (int i = 7; i >= 0; i--) {
		    LOG_DBG("%02x", zb_long_address[i]);
		}
		LOG_DBG("\n");
		zb_shrot_addr = zb_get_short_address();
		LOG_DBG("zigbee shrot addr:  0x%x\n", zb_shrot_addr);
		zb_get_extended_pan_id(zb_ext_pan_id);	
		// Log Extended PAN ID
		LOG_DBG("zigbee extended pan id: ");
		for (int i = 7; i >= 0; i--) {
		    LOG_DBG("%02x", zb_ext_pan_id[i]);
		}
		LOG_DBG("\n");
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

	get_endpoint_descriptor(232);

	}
}

void display_counters(void)
{
    if(correct_RF_packets_received_counter != correct_RF_packets_received_counter_old)
    {
        correct_RF_packets_received_counter_old = correct_RF_packets_received_counter;
        printk("Input counter: %d \n", correct_RF_packets_received_counter);
    }

    if(tcu_uart_frames_received_counter != tcu_uart_frames_received_counter_old)
    {
        tcu_uart_frames_received_counter_old = tcu_uart_frames_received_counter;
        printk("Output counter: %d \n", tcu_uart_frames_received_counter);
    }
}

int main(void)
{
	LOG_INF("Router started successfully");
	int ret = 0;

    get_reset_reason();

	ret = tcu_uart_init();
    if( ret < 0)
    {
		LOG_ERR("tcu_uart_init error %d", ret);
    }
    
	// Initialize TIMER1
	timer1_init();
	ret = gpio_init();

	if( ret < 0)
	{
		LOG_ERR("gpio_init error %d", ret);
	}

	zigbee_configuration();

	/* Start Zigbee default thread */
	zigbee_enable();

    while(1)
    {
        // run diagnostic functions
        diagnostic_toogle_pin();
        diagnostic_zigbee_info();
        display_counters();

        // when modbus answer is ready send the answer
        if(b_Modbus_Ready_to_send)
        {
            tcu_uart_frames_received_counter++;
            send_zigbee_modbus_answer();
            b_Modbus_Ready_to_send = false;
            UART_rx_buffer_index = 0;
            offset=0;
        }
        k_sleep(K_MSEC(5));
    }

    return 0;
}
