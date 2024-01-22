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
#define SLEEP_TIME_MS   10000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* Default tick to consider a modbus frame completed*/
#define DEFAULT_TICKS_TO_CONSIDER_FRAME_COMPLETED 100 // At 19200 bps, 4T = 2083 us --> 21 ticks of 100 us

/*UART Modbus and zigbee buffer size definitions*/
#define UART_RX_BUFFER_SIZE              255 //253 bytes + CRC (2 bytes) = 255
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

/* Get the device pointer of the UART hardware */
static const struct device *dev_uart= DEVICE_DT_GET(DT_NODELABEL(uart0));;
	
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Get a reference to the TIMER1 instance
static const nrfx_timer_t my_timer = NRFX_TIMER_INSTANCE(1);

/* UART configuration variables*/
static volatile char UART_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile bool b_UART_receiving_frame;
static volatile uint16_t UART_ticks_since_last_byte;
static uint16_t UART_ticks_to_consider_frame_completed;
static volatile bool b_UART_overflow;
static volatile uint16_t UART_rx_buffer_index;
static volatile uint16_t UART_rx_buffer_index_max;
static volatile bool b_Modbus_Request_algorithm_finished = false;


static volatile uint32_t start;

static volatile uint16_t uart_frame_received_count = 0;
static volatile uint16_t uart_frame_send_count = 0;

static volatile uint16_t zb_frame_received_count = 0;
static volatile uint16_t zb_frame_send_count = 0;

static volatile	zb_uint8_t ZB_rx_buffer[MAX_ZIGBEE_PAYLOAD_SIZE];
static volatile	zb_int32_t sizeOfPayload;

  // Create a UART configuration structure
static struct uart_config uart_cfg = {
        .baudrate = 19200,                    // Set baudrate to 9600 bps
        .parity = UART_CFG_PARITY_EVEN,      // No parity
        .stop_bits = UART_CFG_STOP_BITS_1,   // 1 stop bit
        .data_bits = UART_CFG_DATA_BITS_8,   // 8 data bits
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE // No flow control
 };

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
LOG_MODULE_REGISTER(app, LOG_LEVEL_NONE);

// boolean flags for detecting modbus request handling
bool b_UART_get_Modbus_frame = false;
bool b_ZB_send_Modbus_frame = false;
bool b_Zigbe_Connected = false;
bool b_UART_send_Modbus_frame = false;

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


/*----------------------------------------------------------------------------*/
/* Function: UART_driver_init()                                              */
/*                                                                            */
/* This function initializes the local variables of the UART module          */
/*                                                                            */
/* Parameters: None                                                           */
/* Returns: None                                                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/
void UART_conf_init(void)
{ 
    b_UART_receiving_frame = false;
    UART_ticks_since_last_byte = 0;
    UART_ticks_to_consider_frame_completed = DEFAULT_TICKS_TO_CONSIDER_FRAME_COMPLETED;
	b_UART_overflow = false;
	UART_rx_buffer_index = 0;
	UART_rx_buffer_index_max = 0;
	b_ZB_send_Modbus_frame = false;
	b_UART_get_Modbus_frame = false;
	b_UART_send_Modbus_frame = false;
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
				//printk("Enter identify mode");
			} else if (zb_err_code == RET_INVALID_STATE) {
				printk("RET_INVALID_STATE - Cannot enter identify mode");
			} else {
				ZB_ERROR_CHECK(zb_err_code);
			}
		} else {
			printk("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	} else {
		printk("Device not in a network - cannot enter identify mode");
	}
}

//------------------------------------------------------------------------------
/**@brief Callback to call when AF got APS packet.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
zb_uint8_t data_indication(zb_bufid_t bufid)
{
 	zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header
    
    if (bufid)
	{
		if(b_Modbus_Request_algorithm_finished)
		{
			UART_conf_init();
			//if( (ind->clusterid == 0x0011) && ( ind->src_endpoint == 1 ) && ( ind->dst_endpoint == 1 ) )
			if( (ind->clusterid == 0x0011) && ( ind->src_endpoint == 232 ) && ( ind->dst_endpoint == 232 ) )
			{
				zb_uint8_t *pointerToBeginOfBuffer;
				zb_uint8_t *pointerToEndOfBuffer;
				pointerToBeginOfBuffer = zb_buf_begin(bufid);
				pointerToEndOfBuffer = zb_buf_end(bufid);
				sizeOfPayload = pointerToEndOfBuffer - pointerToBeginOfBuffer;

				if ((sizeOfPayload > 0) && (sizeOfPayload < MAX_ZIGBEE_PAYLOAD_SIZE))
				{
					if(PRINT_ZIGBEE_INFO)
					{
						printk("Size of received payload is %d bytes \n", sizeOfPayload);
						for (uint8_t i = 0; i < sizeOfPayload; i++)
						{
							printk("0x%02x - ", pointerToBeginOfBuffer[i]);
						}
							printk("\n Frame control field: %d \n", pointerToBeginOfBuffer[0]);
							printk("Sequence number: %d - \n", pointerToBeginOfBuffer[1]);
							printk("Zigbee Command: 0x%02x - \n", pointerToBeginOfBuffer[2]);

							printk("ind Profileid 0x%04x \n", ind->profileid);
    						printk("ind Clusterid 0x%04x \n", ind->clusterid);
    						printk("ind Source Endpoint %d \n", ind->src_endpoint);
    						printk("ind Destination Endpoint %d \n", ind->dst_endpoint);
    						printk("ind APS counter %d \n", ind->aps_counter);
					}

					//if ((sizeOfPayload == 8) && (pointerToBeginOfBuffer[0] == 0xC2))
					if ((sizeOfPayload >= MODBUS_MIN_RX_LENGTH) && (pointerToBeginOfBuffer[0] == 0xC4))
					{

						if(PRINT_ZIGBEE_INFO)
						{
							printk("bModbusRequestReceived via zigbee and send via UART \n");
						}
						zb_frame_received_count++;
						// safe zigbee source endpoint info to send the asnwer
						ZB_profileid = ind->profileid;
    					ZB_clusterid = ind->clusterid;
    					ZB_src_endpoint = ind->src_endpoint;
    					ZB_dst_endpoint = ind->dst_endpoint;
    					ZB_aps_counter = ind->aps_counter;

						for (uint8_t i = 0; i < sizeOfPayload; i++)
						{
							ZB_rx_buffer[i] = pointerToBeginOfBuffer[i];
						}

						b_UART_send_Modbus_frame = true;
						b_Modbus_Request_algorithm_finished = false;
					}
				}
			}
		}
	}
    if (bufid)
    {
		zb_buf_free(bufid); // JESUS: I don't know if this is needed, but I try, just in case.
	}
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
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_DEFAULT_START\n");
 		break;
		case ZB_ZDO_SIGNAL_SKIP_STARTUP:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_SKIP_STARTUP\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_ANNCE:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_DEVICE_ANNCE\n");
		break;
		case ZB_ZDO_SIGNAL_LEAVE:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_LEAVE\n");
		break;
		case ZB_ZDO_SIGNAL_ERROR:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_ERROR\n");
		break;
		case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_DEVICE_FIRST_START\n");
		break;
		case ZB_BDB_SIGNAL_DEVICE_REBOOT:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_DEVICE_REBOOT\n");
		break;
		case ZB_BDB_SIGNAL_STEERING:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_STEERING\n");
			start_identifying(bufid);
		break;
		case ZB_BDB_SIGNAL_FORMATION:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_FORMATION\n");
		break;
		case ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED\n");
		break;
		case ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED\n");
		break;
		case ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_NWK_SIGNAL_DEVICE_ASSOCIATED\n");
		break;
		case ZB_ZDO_SIGNAL_LEAVE_INDICATION:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_LEAVE_INDICATION\n");
		break;
		case ZB_BDB_SIGNAL_WWAH_REJOIN_STARTED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_WWAH_REJOIN_STARTED\n");
		break;
		case ZB_ZGP_SIGNAL_COMMISSIONING:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZGP_SIGNAL_COMMISSIONING\n");
		break;
		case ZB_COMMON_SIGNAL_CAN_SLEEP:
			//printk( "JULEN ZB_COMMON_SIGNAL_CAN_SLEEP\n");
		break;
		case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY\n");
		break;
		case ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED\n");
		break;
		case ZB_ZDO_SIGNAL_DEVICE_UPDATE:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_ZDO_SIGNAL_DEVICE_UPDATE\n");
		break;
		case ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED\n");
		break;
		case ZB_NLME_STATUS_INDICATION:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_NLME_STATUS_INDICATION\n");
		break;
		case ZB_TCSWAP_DB_BACKUP_REQUIRED_SIGNAL:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_TCSWAP_DB_BACKUP_REQUIRED_SIGNAL\n");
		break;
		case ZB_BDB_SIGNAL_TC_REJOIN_DONE:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_TC_REJOIN_DONE\n");
		break;
		case ZB_BDB_SIGNAL_STEERING_CANCELLED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_STEERING_CANCELLED\n");
		break;
		case ZB_BDB_SIGNAL_FORMATION_CANCELLED:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_BDB_SIGNAL_FORMATION_CANCELLED\n");
		break;
		case ZB_SIGNAL_READY_TO_SHUT:
			if(PRINT_ZIGBEE_INFO) printk( "JULEN ZB_SIGNAL_READY_TO_SHUT\n");
		break;
 		default:
			//printk( "JULEN Unknown signal\n");
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
			printk("send_user_payload: chunk_size %d\n", chunk_size);

			for (uint8_t i = 0; i < chunk_size; i++)
			{
				printk("0x%02x- ", outputPayload[i]);
			}
			printk("\n");
		}

		if(PRINT_ZIGBEE_INFO)
		{
				printk("ind Profileid 0x%04x \n", ZB_profileid);
    			printk("ind Clusterid 0x%04x \n", ZB_clusterid);
    			printk("ind Source Endpoint %d \n", ZB_src_endpoint);
    			printk("ind Destination Endpoint %d \n", ZB_dst_endpoint);
		}


		if( (MODBUS_MIN_RX_LENGTH <= chunk_size) && (chunk_size <= MAX_ZIGBEE_PAYLOAD_SIZE) )
		{
	    	//zb_uint8_t outputCustomPayload[255] = {0xC2, 0x04, 0x04, 0x00, 0x4A, 0x75, 0x6C, 0x65, 0x6E, 0x4A, 0x75, 0x6C, 0x65, 0x6E};
			if(bufid)
			{
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
				if(ret == RET_OK)
				{
					printk("RET_OK - if transmission was successful scheduled;\n");
				}
				else if(ret == RET_INVALID_PARAMETER_1)
				{
					printk("RET_INVALID_PARAMETER_1 - if the buffer is invalid\n");
				}
				else if(ret == RET_INVALID_PARAMETER_2)
				{
					printk("RET_INVALID_PARAMETER_2 - if the payload_ptr parameter is invalid;\n");
				}
				else if(ret == RET_INVALID_PARAMETER_3)
				{
					printk("RET_INVALID_PARAMETER_3 - if the payload_size parameter is too large;\n");
				}
				else
				{
					printk("Unkown error zb_aps_send_user_payload ;\n");
				}
				zb_frame_send_count++;
			}
			else
			{
				printk("\n\n bufid NULL error ZB not answered \n\n");
			}
		}
		else
		{
			printk("\n\n payload LENGTH error \n\n");
		}
		b_Modbus_Request_algorithm_finished=true;

	/*Free packet buffer and put it into free list after payload is sent*/
	if (bufid) 
	{
	    zb_buf_free(bufid);
	}

}

//------------------------------------------------------------------------------
/**@brief Function to send the received modbus answer. Logic to split the messages in MAX_ZIGBEE_PAYLOAD_SIZE
 *
 *
 */
void send_zigbee_modbus_answer(void)
{

	//zb_uint8_t outputPayload[255] = {0xC2, 0x04, 0x04, 0x00, 0x4A, 0x75, 0x6C, 0x65, 0x6E, 0x4A, 0x75, 0x6C, 0x65, 0x6E};
	//zb_uint8_t outputPayload[MAX_ZIGBEE_PAYLOAD_SIZE];

	//send_user_payload(&UART_rx_buffer, (UART_rx_buffer_index_max +1));
/*
	printk("send_zigbee_modbus_answer Size of answer send via zigbee is %d bytes \n", UART_rx_buffer_index_max);

	for (uint8_t i = 0; i < (UART_rx_buffer_index_max); i++)
	{
		printk("%c- ", UART_rx_buffer[i]);
	}

	printk("UART_rx_buffer \n ");

	


    // Manipulating the received buffer before processing this is to test big data packages
	for (uint8_t i = 0; i <= (UART_rx_buffer_index_max * 20); i = i+5)
	{
		UART_rx_buffer[i] = UART_rx_buffer[0];
		UART_rx_buffer[i+1] = UART_rx_buffer[1];
		UART_rx_buffer[i+2] = UART_rx_buffer[2];
		UART_rx_buffer[i+3] = UART_rx_buffer[3];
		UART_rx_buffer[i+4] = UART_rx_buffer[4];
	}

	UART_rx_buffer_index_max = (UART_rx_buffer_index_max * 20);


	printk("send_zigbee_modbus_answer Size of answer send via zigbee is %d bytes \n", UART_rx_buffer_index_max);

	for (uint8_t i = 0; i < (UART_rx_buffer_index_max); i++)
	{
		printk("%c- ", UART_rx_buffer[i]);
	}

	size_t remaining_length = UART_rx_buffer_index_max +1;

    // Transmit the modified payload in chunks via Zigbee
	while(remaining_length > 0) 
	{
    	size_t chunk_size = MIN(remaining_length, MAX_ZIGBEE_PAYLOAD_SIZE);

		memcpy(outputPayload, &UART_rx_buffer[offset], chunk_size);

		for (uint8_t i = 0; i < chunk_size; i++)
		{
		printk("%c- ", outputPayload[i]);
		}

		send_user_payload(&outputPayload, chunk_size);

        // Reset flags for the next chunk
		b_UART_get_Modbus_frame = false;

		// Update offset and remaining length for the next chunk
    	offset += chunk_size;
    	remaining_length -= chunk_size;
*/
		/*
	  	Note: This code includes a one-second delay between Zigbee package transmissions for stability.

	  	To adjust the delay time:
	  	- Locate the 'sleep' function.
	  	- Modify the argument value in 'sleep' for a different delay duration.
	  	- Test the code after changing the delay to ensure proper functionality.

	  	Caution: Be mindful when altering the delay to prevent network congestion.
	
		if(remaining_length)
		{
			k_msleep(SLEEP_TIME_MS);
		}
			*/
	//}

}

// Interrupt handler for the timer
// NOTE: This callback is triggered by an interrupt. Many drivers or modules in Zephyr can not be accessed directly from interrupts, 
//		 and if you need to access one of these from the timer callback it is necessary to use something like a k_work item to move execution out of the interrupt context. 
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	/* temp array for get_uart function*/
	char rx_buf[UART_RX_BUFFER_SIZE];
	int ret;

	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
			//if (!bModbusRequestReceived) printk("Timer 1 callback = %d bModbusRequestReceived %d\n", bModbusRequestReceived);
			if (b_UART_get_Modbus_frame)
			{
				UART_ticks_since_last_byte++;
				if( b_UART_receiving_frame )
    			{
    			    if( UART_ticks_since_last_byte > UART_ticks_to_consider_frame_completed )
    			    {
    			        b_UART_receiving_frame = false; 
						UART_ticks_since_last_byte = 0;
						b_ZB_send_Modbus_frame = true;
						b_UART_get_Modbus_frame = false;
						UART_rx_buffer_index=0;
						uart_frame_received_count++;
						uint32_t timeout_in_ms;
						timeout_in_ms = (k_uptime_get_32() - start);
						printk("time to GET via uart modbus request %lu\n", timeout_in_ms);
    			    }
				}
				if (UART_ticks_since_last_byte >= (UART_ticks_to_consider_frame_completed*10))
				{
					b_Modbus_Request_algorithm_finished = true;
				}
			}
			if(b_UART_send_Modbus_frame)
			{	
				start = k_uptime_get_32();

				for (int i = 0; i < sizeOfPayload; i++) 
				{
					uart_poll_out(dev_uart, ZB_rx_buffer[i]);
				}
				b_UART_send_Modbus_frame = false;
				b_UART_get_Modbus_frame = true;
				uart_frame_send_count++;
				uint32_t timeout_in_ms;
				timeout_in_ms = (k_uptime_get_32() - start);
				printk("time to SEND via uart modbus request %lu\n", timeout_in_ms);
			}
			// when modbus answer is ready send the answer
			if(b_ZB_send_Modbus_frame)
			{
				b_ZB_send_Modbus_frame = false;
				uint32_t timeout_in_ms;
				timeout_in_ms = (k_uptime_get_32() - start);
				printk("time to b_ZB_send_Modbus_frame %lu\n", timeout_in_ms);
	
				send_user_payload(&UART_rx_buffer, (UART_rx_buffer_index_max +1));	
				timeout_in_ms = (k_uptime_get_32() - start);
				printk("time to SENT b_ZB_send_Modbus_frame %lu\n", timeout_in_ms);
			}
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
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_1MHz);
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	int err = nrfx_timer_init(&my_timer, &timer_config, timer1_event_handler);
	if (err != NRFX_SUCCESS) {
		if(err == NRFX_ERROR_INVALID_PARAM)
		{
			printk("Specified frequency is not supported by the TIMER instance.\n");
		}
		else if(err == NRFX_ERROR_ALREADY_INITIALIZED)
		{
			printk("The driver is already initialized..\n");
		}
		else if(err == NRFX_ERROR_INVALID_STATE )
		{
			printk("NRFX_ERROR_INVALID_STATE \n");
		}
		else
		{
			printk("Error initializing timer: %x\n", err);
		}
	}
	else
	{
		printk("timer initialized correctly\n");
	}

	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);

	// Setup TIMER1 to generate callbacks every second 1000000
	// Setup TIMER1 to generate callbacks every 100ms 100000
	timer1_repeated_timer_start(100);
}

/*
* Write a character to the device for output.
 */
void send_uart(uint8_t out_uint8) 
{
/*
Write a character to the device for output.
This routine checks if the transmitter is full. When the transmitter is not full, it writes a character to the data register. 
It waits and blocks the calling thread, otherwise. This function is a blocking call.
To send a character when hardware flow control is enabled, the handshake signal CTS must be asserted.
*/
	uart_poll_out(dev_uart, (unsigned char)out_uint8);
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t rx_buf;
	//printk("serial_cb \n");

	if (!uart_irq_update(dev_uart)) {
		return;
	}

	if (!uart_irq_rx_ready(dev_uart)) {
		return;
	}

/* read until FIFO empty */
	int ret;
	ret = uart_fifo_read(dev_uart, &rx_buf, 1);

	if(ret >= 1) 
	{
		if (b_UART_get_Modbus_frame)
		{
			//printk("UART MODBUS response %02x, number of bytes %d \n", rx_buf, ret);     
			if( b_UART_receiving_frame )
    		{
				if( !b_UART_overflow )
    		    {
    		        if( UART_rx_buffer_index >= UART_RX_BUFFER_SIZE )
    		        {
    		            b_UART_overflow = true;
						printk("b_UART_overflow \n");
    		        }
    		        else
    		        {                       
    		            UART_rx_buffer[UART_rx_buffer_index] = rx_buf;     
						//printk("UART char arrived %02x index: %d \n",UART_rx_buffer[UART_rx_buffer_index], (UART_rx_buffer_index));     
						UART_rx_buffer_index_max = UART_rx_buffer_index;        
    		            UART_rx_buffer_index++;
    		        }
    		    }
				UART_ticks_since_last_byte = 0; //Reset 3.5T Modbus timer
			}
			else
			{
    		    b_UART_receiving_frame = true;
    		    b_UART_overflow = false;
    		    UART_rx_buffer[0] = rx_buf;
				//printk("UART char arrived %02x index: %d \n",UART_rx_buffer[UART_rx_buffer_index], (UART_rx_buffer_index));             
    		    UART_rx_buffer_index = 1;
				UART_rx_buffer_index_max = UART_rx_buffer_index;        
				UART_ticks_since_last_byte = 0; //Reset 3.5T Modbus timer
			}
		}
		else
		{
			printk("UART char arrived but is not MODBUS %02x \n", rx_buf);     
		}
		
	}
	else if (ret == 0)
	{
		UART_ticks_since_last_byte++;
		printk("uart_poll_in No character available. increment rs485_ticks_since_last_byte %d\n", UART_ticks_since_last_byte);
	}
	else if (ret == -ENOSYS)
	{
		printk("uart_poll_in ENOSYS UART polling operation not implemented \n");
	}
		else if (ret == -EBUSY)
	{
		printk("uart_poll_in -EBUSY Async reception was enabled.\n");
	}
	else
	{
		UART_ticks_since_last_byte++;
		printk("uart_poll_in Unknown error. \n");
	}
}

// Function for initializing the UART peripheral
static void app_uart_init(void)
{
	if (!device_is_ready(dev_uart)) {
		printk("UART device not found!");
		return 0;
	}

	// Call uart_configure to apply the configuration
    int result = uart_configure(dev_uart, &uart_cfg);

	// Check the result
    if (result == 0) {
        printf("UART configuration successful!\n");
    } else {
        printf("UART configuration failed with error code: %d\n", result);
    }
	
	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(dev_uart, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	else
	{
        printf("UART Interrupt configuration successful!\n");
	}

	uart_irq_rx_enable(dev_uart);

}


// Function for initializing the GPIO 
static void gpio_init(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return 1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 1;
	}
}

// Function for initializing the Zigbee configuration
void zigbee_configuration()
{
	/* disable NVRAM erasing on every application startup*/
	zb_set_nvram_erase_at_start(ZB_TRUE);

	/* Register device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&app_template_ctx);

		// 1. Define a distributed key (assuming key size of 16 bytes, you might need to adjust based on documentation)
    zb_uint8_t distributed_key[16] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};

    // 3. Call the function with this pointer
	zb_secur_setup_nwk_key((zb_uint8_t *) distributed_key, 0);

	zb_ext_pan_id_t ext_pan_id[8] = {0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	

	zb_set_extended_pan_id(ext_pan_id);

	//TRUE to disable trust center, legacy support for 
	zb_bdb_set_legacy_device_support(ZB_TRUE);
	
	//Call Function for initializing all clusters attributes
	app_clusters_attr_init();
}

// Function for giagnostic purposes. toogle the diagnostic led every second to be sure HW is OK and app is running
void diagnostic_toogle_pin()
{
	int ret = gpio_pin_toggle_dt(&led);
	if (ret < 0) {
		return;
	}

	printf("uart_frame_received_count %d\n", uart_frame_received_count);
	printf("uart_frame_send_count %d\n", uart_frame_send_count);
	printf("zb_frame_received_count %d\n", zb_frame_received_count);
	printf("zb_frame_send_count %d\n", zb_frame_send_count);

	k_msleep(SLEEP_TIME_MS);
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
		
		printk("Profileid 0x%04x \n", profile_id);
    	printk("Clusterid 0x%04x \n", device_id);
		printk("device_version 0x%04x \n", device_version);

    } else 
	{
    	printk("endpoint descriptor not available\n");
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
		printk("Zigbee application joined the network: bellow some info: \n");
		zb_get_long_address(zb_long_address);
		// Log Long Address
		printk("zigbee long addr: ");
		for (int i = 7; i >= 0; i--) {
		    printk("%02x", zb_long_address[i]);
		}
		printk("\n");
		zb_shrot_addr = zb_get_short_address();
		printk("zigbee shrot addr:  0x%x\n", zb_shrot_addr);
		zb_get_extended_pan_id(zb_ext_pan_id);	
		// Log Extended PAN ID
		printk("zigbee extended pan id: ");
		for (int i = 7; i >= 0; i--) {
		    printk("%02x", zb_ext_pan_id[i]);
		}
		printk("\n");
		switch(zb_get_network_role())
		{
		case 0:
			printk("zigbee role coordinator\n");
			break;
		case 1:
			printk("zigbee role router\n");
			break;
		case 2:
			printk("zigbee role end device\n");
			break;
		default:
			printk("Zigbee role NOT found \n");
			break;
		}
		
	zb_channel = zb_get_current_channel();	
	printk("zigbee channel: %d \n", zb_channel);

	get_endpoint_descriptor(232);

	}
}

void get_restet_reason(void)
{
	uint32_t reset_cause;
	int result = hwinfo_get_reset_cause(&reset_cause);
    if (result == 0) 
	{
    // Success, reset_cause now contains the reset cause flags
    // You can analyze the flags to determine the cause of the reset
    // ...
		printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nreset cause is:");

		if (reset_cause & RESET_PIN)
		{
			printf("RESET_PIN\n"); // 0
		}
		else if (reset_cause & RESET_SOFTWARE)
		{
    		printf("RESET_SOFTWARE\n"); // 1
		}
		else if (reset_cause & RESET_BROWNOUT)
		{
			printf("RESET_BROWNOUT\n"); // 2
		}
		else if(reset_cause & RESET_POR)
		{
    		printf("RESET_POR\n"); // 3
		}
		else if(reset_cause & RESET_WATCHDOG)
		{
			printf("RESET_WATCHDOG\n"); // 4
		}
		else if(reset_cause & RESET_DEBUG)
		{
			printf("RESET_DEBUG\n"); // 5
		}
    	else if(reset_cause & RESET_SECURITY)
		{
			printf("RESET_SECURITY\n"); // 6
		}
		else if(reset_cause & RESET_LOW_POWER_WAKE)
		{
			printf("RESET_LOW_POWER_WAKE\n"); // 7
		}
		else if(reset_cause & RESET_CPU_LOCKUP)
		{
			printf("RESET_CPU_LOCKUP\n"); // 8
		}
		else if(reset_cause & RESET_PARITY)
		{
			printf("RESET_PARITY\n"); // 9
		}
		else if(reset_cause & RESET_PLL)
		{
			printf("RESET_PLL\n"); // 10
		}
		else if(reset_cause & RESET_CLOCK)
		{
			printf("RESET_CLOCK\n"); // 11
		}
		else if(reset_cause & RESET_HARDWARE)
		{
			printf("RESET_HARDWARE\n"); // 12
		}
		else if(reset_cause & RESET_USER)
		{
			printf("RESET_USER\n"); // 13
		}
		else if (reset_cause & RESET_TEMPERATURE)
		{
			printf("RESET_TEMPERATURE\n\n"); // 14
		}
		else
		{
			printk("\n\n reset cause is %d\n\n",reset_cause);
		}
    // Optionally, you can call hwinfo_clear_reset_cause to clear the hardware flags
    // This is useful if you only want to consider the cause of the most recent reset
    	hwinfo_clear_reset_cause();

    } else if (result == -ENOSYS) 
	{
        // Handle the case where there is no implementation for the particular device
        // ...
		printk("\n\n there is no implementation for the particular device.\n\n");
    } else 
	{
        // Handle other errors
        // ...
		printk("\n\n negative value on driver specific errors\n\n");
    }
}

int main(void)
{
	/* Initialize UART*/
	app_uart_init();

	// Initialize TIMER1
	timer1_init();
	
	UART_conf_init();

	gpio_init();

	zigbee_configuration();

	/* Start Zigbee default thread */
	zigbee_enable();

	get_restet_reason();

	k_msleep(SLEEP_TIME_MS);

	b_Modbus_Request_algorithm_finished = true;

	while(1)
	{		
		// run diagnostic functions
		//diagnostic_toogle_pin();
		diagnostic_zigbee_info();
	}

	return 0;
}
