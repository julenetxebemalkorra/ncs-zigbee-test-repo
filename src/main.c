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
#include <zephyr/drivers/gpio.h>
#include <uart_handler.h>
#include <string.h>

#include <nrfx_timer.h>

/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               10

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON             1

#define PRINT_ZIGBEE_INFO             	ZB_TRUE

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

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

LOG_MODULE_REGISTER(app, LOG_LEVEL_NONE);

// boolean flag for detecting modbus request received from gateway
bool bModbusRequestReceived = false;
bool bConnected = false;

/* Main application customizable context.
 * Stores all settings and static values.
 */
struct zb_device_ctx {
	zb_zcl_basic_attrs_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
};

/* Zigbee device application context storage. */
static struct zb_device_ctx dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.power_source);

ZB_DECLARE_RANGE_EXTENDER_CLUSTER_LIST(
	app_template_clusters,
	basic_attr_list,
	identify_attr_list);

ZB_DECLARE_RANGE_EXTENDER_EP(
	app_template_ep,
	APP_TEMPLATE_ENDPOINT,
	app_template_clusters);

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
				printk("Enter identify mode");
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

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_zcl_device_callback_param_t  *device_cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	//printk("Received ZCL command %s where device cb id %hd", __func__, device_cb_param->device_cb_id);
}


//------------------------------------------------------------------------------
/**@brief Callback to call when AF got APS packet.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
zb_uint8_t data_indication(zb_bufid_t bufid)
{

 zb_uint8_t *ptr;
 zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header


/* commnets by JULEN, test this example with real set up
 if (ind->profileid == 0x0104 )
 {
 ptr = ZB_APS_HDR_CUT(bufid);
 
 TRACE_MSG(TRACE_APS3, "apsde_data_indication: packet %p len %hd status 0x%hx from %d",
 (FMT__P_D_D_D, bufid, zb_buf_len(bufid), zb_buf_get_status(bufid), ind->src_addr));
 
 for (zb_ushort_t i = 0 ; i < zb_buf_len(bufid) ; ++i)
 {
 TRACE_MSG(TRACE_APS3, "%x %c", (FMT__D_C, (int)ptr[i], ptr[i]));
 }
 zb_buf_free(bufid);
 return ZB_TRUE;
 }
 return ZB_FALSE;

 */

    zb_uint8_t aps_payload_size = 0;
    zb_uint8_t *aps_payload_ptr = zb_aps_get_aps_payload(bufid, &aps_payload_size); // Get APS payload
	//zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header
    
    if (bufid)
	{
		//printk("ind Profileid 0x%04x ", ind->profileid);
    	//printk("ind Clusterid 0x%04x ", ind->clusterid);
    	//printk("ind Source Endpoint %d ", ind->src_endpoint);
    	//printk("ind Destination Endpoint %d ", ind->dst_endpoint);
    	//printk("ind APS counter %d \n", ind->aps_counter);


		//printk("aps_payload_ptr %hhn ", aps_payload_ptr);

		//if( (ind->clusterid == 0x0011) && ( ind->src_endpoint == 232 ) && ( ind->dst_endpoint == 232 ) )
		//{
		if( (ind->clusterid == 0x0104) && ( ind->src_endpoint == 1 ) && ( ind->dst_endpoint == 10 ) )
		{
			zb_uint8_t *pointerToBeginOfBuffer;
			zb_uint8_t *pointerToEndOfBuffer;
			zb_int32_t sizeOfPayload;
			pointerToBeginOfBuffer = zb_buf_begin(bufid);
			pointerToEndOfBuffer = zb_buf_end(bufid);
			sizeOfPayload = pointerToEndOfBuffer - pointerToBeginOfBuffer;
			if ((sizeOfPayload > 0) && (sizeOfPayload < 255))
			{
				//printk("Size of payload is %d bytes \n", sizeOfPayload);
				for (uint8_t i = 0; i < sizeOfPayload; i++)
				{
					//printk("0x%02x - ", pointerToBeginOfBuffer[i]);
				}
			}
			//if ((sizeOfPayload == 8) && (pointerToBeginOfBuffer[0] == 0xC2))
			if ((sizeOfPayload == 8) && (pointerToBeginOfBuffer[0] == 0x11))
			{
				bModbusRequestReceived = true;
				//printk("bModbusRequestReceived this is the answer send via UART \n");

				uint8_t modbusArray[8];

				for (uint8_t i = 0; i < sizeOfPayload; i++)
				{
					modbusArray[i] = pointerToBeginOfBuffer[i];
					//printk("0x%02x - ", modbusArray[i]);
				}

				//printk("Size of payload is %d bytes \n", sizeof(modbusArray));

				app_uart_send(pointerToBeginOfBuffer, 8, K_NO_WAIT);
				//app_uart_send(modbusArray, sizeof(modbusArray));
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
	 	//LOG_INF( "JULEN Device STARTED OK");
 		zb_af_set_data_indication(data_indication);
 	break;
 	default:
 		//LOG_INF( "JULEN Unknown signal");
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


// Interrupt handler for the timer
// NOTE: This callback is triggered by an interrupt. Many drivers or modules in Zephyr can not be accessed directly from interrupts, 
//		 and if you need to access one of these from the timer callback it is necessary to use something like a k_work item to move execution out of the interrupt context. 
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	static int counter = 0;
	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
			// Do your work here
			int ret = gpio_pin_toggle_dt(&led);
			if (ret < 0) {
				return;
			}		
			break;
		default:
			break;
	}
}

// Function for initializing the TIMER1 peripheral using the nrfx driver
static void timer1_init(void)
{
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_config.frequency = NRF_TIMER_FREQ_1MHz;

	int err = nrfx_timer_init(&my_timer, &timer_config, timer1_event_handler);
	if (err != NRFX_SUCCESS) {
		printk("Error initializing timer: %x\n", err);
	}

	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
}

// Function for scheduling repeated callbacks from TIMER1
static void timer1_repeated_timer_start(uint32_t timeout_us)
{
	nrfx_timer_enable(&my_timer);

	nrfx_timer_extended_compare(&my_timer, NRF_TIMER_CC_CHANNEL0, timeout_us, 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}


// Function for initializing the TIMER1 peripheral using the nrfx driver
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

static void on_app_uart_event(struct app_uart_evt_t *evt)
{
	switch(evt->type) {
		// Data received over the UART interface. 
		// NOTE: The UART data buffers are only guaranteed to be retained until this function ends
		//       If the data can not be processed immediately, they should be copied to a different buffer
		case APP_UART_EVT_RX:
			//printk("RX (%i bytes): %.*s\n", evt->data.rx.length, evt->data.rx.length, evt->data.rx.bytes); 

				if ( bModbusRequestReceived )
				{

				    zb_bufid_t bufid;
				    zb_uint8_t outputPayload[9] = {0xC2, 0x04, 0x04, 0x00, 0x4A, 0x75, 0x6C, 0x65, 0x6E};
				    zb_addr_u dst_addr;
				    bufid = zb_buf_get_out();
				    dst_addr.addr_short = 0x0000;
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
					zb_aps_send_user_payload(bufid, 
					    					 dst_addr,
						    				 0x0104,
							    			 0x0104,
								    		 1,
									    	 10,
				    						 ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
					    					 ZB_FALSE,
						    				 outputPayload,
							    			 9);
	    	        bModbusRequestReceived = false;
					//printk("RESPONDIDA9");
					if (bufid) {
			            zb_buf_free(bufid);
	    	        }
			}
		
			break;

		// A UART error ocurred, such as a break or frame error condition
		case APP_UART_EVT_ERROR:
			printk("UART error: Reason %i\n", evt->data.error.reason);
			break;

		// The UART event queue has overflowed. If this happens consider increasing the UART_EVENT_QUEUE_SIZE (will increase RAM usage),
		// or increase the UART_RX_TIMEOUT_US parameter to avoid a lot of small RX packets filling up the event queue
		case APP_UART_EVT_QUEUE_OVERFLOW:
			printk("UART library error: Event queue overflow!\n");
			break;
	}
}

static void uart_init(void)
{

		int err = app_uart_init(on_app_uart_event);
		if(err != 0) {
		printk("app_uart_init failed: %i\n", err);
		return;
	}
}



int main(void)
{
	zb_ieee_addr_t zb_long_address;
	zb_ext_pan_id_t zb_ext_pan_id;
	zb_nwk_device_type_t zb_role;
	zb_uint8_t zb_channel;
	zb_uint16_t zb_shrot_addr;
	int infit_info_flag = PRINT_ZIGBEE_INFO;
	zb_ret_t zb_err_code;
    zb_ieee_addr_t ieee_addr;
	int blink_status = 0;

	/* Initialize */
	uart_init();

	// Initialize TIMER1
	timer1_init();

	gpio_init();
	
	// Setup TIMER1 to generate callbacks every second
	timer1_repeated_timer_start(1000000);

	zb_set_nvram_erase_at_start(ZB_TRUE);

	const uint8_t test_string[] = "Text example UART and Zigbee \r\n";
	uint8_t modbusArray[8];

	app_uart_send(test_string, strlen(test_string), K_NO_WAIT);

	/* Register device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&app_template_ctx);

		// 1. Define a distributed key (assuming key size of 16 bytes, you might need to adjust based on documentation)
    zb_uint8_t distributed_key[16] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 
                                     0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};

    // 2. Create a pointer to the key
    zb_uint8_t* key_ptr = distributed_key;

    // 3. Call the function with this pointer
	zb_secur_setup_nwk_key((zb_uint8_t *) distributed_key, 0);

	zb_bdb_set_legacy_device_support(ZB_TRUE);

	app_clusters_attr_init();

	/* Start Zigbee default thread */
	zigbee_enable();

	while(1)
	{		

		if(zb_zdo_joined() && infit_info_flag == ZB_TRUE)
		{
			infit_info_flag = ZB_FALSE;
			bConnected = true;

			const uint8_t ZBjoinedPayload[] = "ZB joined nw \r\n";
			app_uart_send(ZBjoinedPayload, strlen(ZBjoinedPayload), K_NO_WAIT); 

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

		}

	}

	return 0;
}
