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


/* Device endpoint, used to receive ZCL commands. */
#define APP_TEMPLATE_ENDPOINT               10

/* Function to set the Erase persistent storage
 *        depending on the erase pin */
#define ERASE_PERSISTENT_CONFIG    ZB_TRUE

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define TEMPLATE_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* LED indicating that device successfully joined Zigbee network. */
#define ZIGBEE_NETWORK_STATE_LED               DK_LED1

/* LED used for device identification. */
#define IDENTIFY_LED                     DK_LED2

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON             1

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON                1


bool bModbusRequestReceived = false;

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

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

/**@brief Function to toggle the identify LED
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void toggle_identify_led(zb_bufid_t bufid)
{
	static int blink_status;

	dk_set_led(IDENTIFY_LED, (++blink_status) % 2);
	ZB_SCHEDULE_APP_ALARM(toggle_identify_led, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(100));
}

/**@brief Function to handle identify notification events on the first endpoint.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void identify_cb(zb_bufid_t bufid)
{
	zb_ret_t zb_err_code;

	if (bufid) {
		/* Schedule a self-scheduling function that will toggle the LED */
		ZB_SCHEDULE_APP_CALLBACK(toggle_identify_led, bufid);
	} else {
		/* Cancel the toggling function alarm and turn off LED */
		zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(toggle_identify_led, ZB_ALARM_ANY_PARAM);
		ZVUNUSED(zb_err_code);

		dk_set_led(IDENTIFY_LED, 0);
	}
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
				LOG_INF("Enter identify mode");
			} else if (zb_err_code == RET_INVALID_STATE) {
				LOG_WRN("RET_INVALID_STATE - Cannot enter identify mode");
			} else {
				ZB_ERROR_CHECK(zb_err_code);
			}
		} else {
			LOG_INF("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	} else {
		LOG_WRN("Device not in a network - cannot enter identify mode");
	}
}

/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing buttons state.
 * @param[in]   has_changed   Bitmask containing buttons
 *                            that have changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (IDENTIFY_MODE_BUTTON & has_changed) {
		if (IDENTIFY_MODE_BUTTON & button_state) {
			/* Button changed its state to pressed */
		} else {
			/* Button changed its state to released */
			if (was_factory_reset_done()) {
				/* The long press was for Factory Reset */
				LOG_DBG("After Factory Reset - ignore button release");
			} else   {
				/* Button released before Factory Reset */

				/* Start identification mode */
				ZB_SCHEDULE_APP_CALLBACK(start_identifying, 0);
			}
		}
	}

	check_factory_reset_button(button_state, has_changed);
}

/**@brief Function for initializing LEDs and Buttons. */
static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
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

	LOG_INF("Received ZCL command %s where device cb id %hd", __func__, device_cb_param->device_cb_id);
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

    //zb_uint8_t aps_payload_size = 0;
    //zb_uint8_t *aps_payload_ptr = zb_aps_get_aps_payload(bufid, &aps_payload_size); // Get APS payload
	//zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);  // Get APS header
    
    if (bufid)
	{
		LOG_INF("Profileid 0x%04x ", ind->profileid);
    	LOG_INF("Clusterid 0x%04x ", ind->clusterid);
    	LOG_INF("Source Endpoint %d ", ind->src_endpoint);
    	LOG_INF("Destination Endpoint %d ", ind->dst_endpoint);
    	LOG_INF("APS counter %d \n", ind->aps_counter);
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
				LOG_INF("Size of payload is %d bytes \n", sizeOfPayload);
				for (uint8_t i = 0; i < sizeOfPayload; i++)
				{
					LOG_INF("0x%02x - ", pointerToBeginOfBuffer[i]);
				}
			}
			//if ((sizeOfPayload == 8) && (pointerToBeginOfBuffer[0] == 0xC2))
			if ((sizeOfPayload == 8) && (pointerToBeginOfBuffer[0] == 0x11))
			{
				bModbusRequestReceived = true;
				LOG_INF("bModbusRequestReceived \n");
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
	/* Update network status LED. */
	zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

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

int main(void)
{
	bool bConnected = false;
	zb_ieee_addr_t zb_long_address;
	zb_ext_pan_id_t zb_ext_pan_id;
	zb_nwk_device_type_t zb_role;
	zb_uint8_t zb_channel;
	zb_uint16_t zb_shrot_addr;
	int infit_info_flag = 0;
	zb_ret_t zb_err_code;
    zb_ieee_addr_t ieee_addr;
	int blink_status = 0;

	LOG_INF("Starting Zigbee application template example");

	/* Initialize */
	configure_gpio();
	register_factory_reset_button(FACTORY_RESET_BUTTON);

	/* Register device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&app_template_ctx);

	app_clusters_attr_init();

	/* Register handlers to identify notifications */
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(APP_TEMPLATE_ENDPOINT, identify_cb);

	/* Start Zigbee default thread */
	zigbee_enable();

	LOG_INF("Zigbee application template started");

	
	while(1)
	{		
		k_sleep(K_MSEC(500));

		dk_set_led(ZIGBEE_NETWORK_STATE_LED, (blink_status) % 2);
		dk_set_led(IDENTIFY_LED, (++blink_status) % 2);


		if(zb_zdo_joined() && infit_info_flag == 0)
		{
			infit_info_flag = 1;
			bConnected = true;
			LOG_INF("Zigbee application joined the network: bellow some info:");

			zb_get_long_address(zb_long_address);
			// Log Long Address
			LOG_INF("zigbee long addr: ");
			for (int i = 7; i >= 0; i--) {
			    printk("%02x", zb_long_address[i]);
			}
			printk("\n");

			zb_shrot_addr = zb_get_short_address();
			LOG_INF("zigbee shrot addr:  0x%x", zb_shrot_addr);

			zb_get_extended_pan_id(zb_ext_pan_id);	
			// Log Extended PAN ID
			LOG_INF("zigbee extended pan id: ");
			for (int i = 7; i >= 0; i--) {
			    printk("%02x", zb_ext_pan_id[i]);
			}
			printk("\n");

			switch(zb_get_network_role())
			{
			case 0:
				LOG_INF("zigbee role coordinator");
				break;
			case 1:
				LOG_INF("zigbee role router");
				break;
			case 2:
				LOG_INF("zigbee role end device");
				break;
			default:
				LOG_INF("Zigbee role NOT found");
				break;
			}
			
		zb_channel = zb_get_current_channel();	
		LOG_INF("zigbee channel: %d", zb_channel);

		}

        if ( bConnected )
        {
			if ( bModbusRequestReceived )
			{
			    zb_bufid_t bufid;
			    zb_uint8_t outputPayload[9] = {0xC2, 0x04, 0x04, 0x00, 0x1E, 0x00, 0x00, 0x68, 0x8E};
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
				LOG_INF("RESPONDIDA9");
				if (bufid) {
		            zb_buf_free(bufid);
	            }
			}
		}

	}

	return 0;
}
