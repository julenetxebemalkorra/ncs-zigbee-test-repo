/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Simple Zigbee network coordinator implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>

#include <zboss_api.h>
#include <zb_mem_config_max.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>
#include <zb_nrf_platform.h>
#include "zb_range_extender.h"


#define RUN_STATUS_LED                         DK_LED1
#define RUN_LED_BLINK_INTERVAL                 1000

/* Device endpoint, used to receive ZCL commands. */
#define ZIGBEE_COORDINATOR_ENDPOINT            232

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define COORDINATOR_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* LED indicating that device successfully joined Zigbee network. */
#define ZIGBEE_NETWORK_STATE_LED               DK_LED3

/* LED used for device identification. */
#define IDENTIFY_LED                           DK_LED4

/* Button which reopens the Zigbee Network. */
#define KEY_ZIGBEE_NETWORK_REOPEN              DK_BTN1_MSK

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON                   DK_BTN4_MSK

/* If set to ZB_TRUE then device will not open the network after forming or reboot. */
#define ZIGBEE_MANUAL_STEERING                 ZB_FALSE

#define ZIGBEE_PERMIT_LEGACY_DEVICES           ZB_FALSE

#ifndef ZB_COORDINATOR_ROLE
#error Define ZB_COORDINATOR_ROLE to compile coordinator source code.
#endif

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

#define ZB_ZGP_DEFAULT_SHARED_SECURITY_KEY_TYPE ZB_ZGP_SEC_KEY_TYPE_NWK
#define ZB_ZGP_DEFAULT_SECURITY_LEVEL ZB_ZGP_SEC_LEVEL_FULL_WITH_ENC
#define ZB_STANDARD_TC_KEY { 0x81, 0x42, < rest of key > };
#define ZB_DISTRIBUTED_GLOBAL_KEY { 0x81, 0x42, , < rest of key > };
#define ZB_TOUCHLINK_PRECONFIGURED_KEY { 0x81, 0x42, < rest of key > };

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static const zb_uint8_t ext_pan_id[8] = {0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const zb_uint8_t * ptr_ext_pan_id = ext_pan_id;

static volatile	zb_uint8_t *pointerToBeginOfBuffer;
static volatile	zb_uint16_t C4_count = 0;
static volatile	zb_uint16_t C2_count = 0;


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
	nwk_coordinator_clusters,
	basic_attr_list,
	identify_attr_list);

ZB_DECLARE_RANGE_EXTENDER_EP(
	nwk_coordinator_ep,
	ZIGBEE_COORDINATOR_ENDPOINT,
	nwk_coordinator_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	nwk_coordinator,
	nwk_coordinator_ep);


/**@brief Function for initializing all clusters attributes. */
static void app_clusters_attr_init(void)
{
	/* Basic cluster attributes data. */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source = COORDINATOR_INIT_BASIC_POWER_SOURCE;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
}

/**@brief Function to toggle the identify LED.
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
		/* Schedule a self-scheduling function that will toggle the LED. */
		ZB_SCHEDULE_APP_CALLBACK(toggle_identify_led, bufid);
	} else {
		/* Cancel the toggling function alarm and turn off LED. */
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
		 * if not, put desired endpoint in identifying mode.
		 */
		if (dev_ctx.identify_attr.identify_time ==
		    ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE) {

			zb_ret_t zb_err_code = zb_bdb_finding_binding_target(
				ZIGBEE_COORDINATOR_ENDPOINT);

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

/**@brief Callback used in order to visualise network steering period.
 *
 * @param[in]   param   Not used. Required by callback type definition.
 */
static void steering_finished(zb_uint8_t param)
{
	ARG_UNUSED(param);

	LOG_INF("Network steering finished");
	dk_set_led_off(ZIGBEE_NETWORK_STATE_LED);
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
    
    if (bufid && (zb_buf_get_status(bufid) == 0))
	{
		unsigned int key = irq_lock();
        if(( ind->src_endpoint == 232 ) && ( ind->dst_endpoint == 232 ) )
        {
            zb_int32_t sizeOfPayload;
            pointerToBeginOfBuffer = zb_buf_begin(bufid);
            sizeOfPayload = zb_buf_len(bufid);

            if (sizeOfPayload > 0)
            {
                LOG_INF("Size of received payload is %d bytes \n", sizeOfPayload);
                LOG_HEXDUMP_INF(pointerToBeginOfBuffer,sizeOfPayload,"Payload of input RF packet");
                //LOG_INF("\n Frame control field: 0x%02x \n", pointerToBeginOfBuffer[0]);
                //LOG_INF("Sequence number: 0x%02x - \n", pointerToBeginOfBuffer[1]);
            	//LOG_INF("Zigbee Command: 0x%02x - \n", pointerToBeginOfBuffer[2]);
            	//LOG_INF("ind APS counter %d \n", ind->aps_counter);
            	if(pointerToBeginOfBuffer[0] == 196)
				{
					C4_count++;
					LOG_INF("C4_count %d \n", C4_count);
				}
				else if(pointerToBeginOfBuffer[0] == 194)
				{
					C2_count++;
					LOG_INF("C2_count %d \n", C2_count);
				}
				else{
					LOG_ERR("WRONG message input");
				}
            	LOG_INF("Payload of input RF packet sent to Tcu UART \n");
                // safe zigbee source endpoint info to send the asnwer
            }
        }
		irq_unlock(key);
	}

    if (bufid)
    {
		zb_buf_free(bufid); // JESUS: I don't know if this is needed, but I try, just in case.
		return ZB_TRUE;
	}
	return ZB_FALSE;
}

/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing buttons state.
 * @param[in]   has_changed   Bitmask containing buttons that has changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	/* Calculate bitmask of buttons that are pressed and have changed their state. */
	uint32_t buttons = button_state & has_changed;
	zb_bool_t comm_status;

	if (buttons & KEY_ZIGBEE_NETWORK_REOPEN) {
		(void)(ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished, ZB_ALARM_ANY_PARAM));

		comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
		if (comm_status) {
			LOG_INF("Top level commissioning restated");
		} else {
			LOG_INF("Top level commissioning hasn't finished yet!");
		}
	}

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

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	/* Read signal description out of memory buffer. */
	zb_zdo_app_signal_hdr_t *sg_p = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &sg_p);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);
	zb_ret_t zb_err_code;
	zb_bool_t comm_status;
	zb_time_t timeout_bi;

	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		/* BDB initialization completed after device reboot,
		 * use NVRAM contents during initialization.
		 * Device joined/rejoined and started.
		 */
		if (status == RET_OK) {
			if (ZIGBEE_MANUAL_STEERING == ZB_FALSE) {
				LOG_INF("Start network steering");
				comm_status = bdb_start_top_level_commissioning(
					ZB_BDB_NETWORK_STEERING);
				ZB_COMM_STATUS_CHECK(comm_status);
			} else {
				LOG_INF("Coordinator restarted successfully");
			}
		} else {
			LOG_ERR("Failed to initialize Zigbee stack using NVRAM data (status: %d)",
				status);
		}
		break;

	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			if (ZIGBEE_PERMIT_LEGACY_DEVICES == ZB_TRUE) {
				LOG_INF("Allow pre-Zigbee 3.0 devices to join the network");
				zb_bdb_set_legacy_device_support(1);
			}

			/* Schedule an alarm to notify about the end of steering period.
			 */
			LOG_INF("Network steering started");
			zb_err_code = ZB_SCHEDULE_APP_ALARM(
				steering_finished, 0,
				ZB_TIME_ONE_SECOND *
				ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
			ZB_ERROR_CHECK(zb_err_code);
		}
		break;

	case ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
		zb_zdo_signal_device_annce_params_t *dev_annce_params =
			ZB_ZDO_SIGNAL_GET_PARAMS(
				sg_p, zb_zdo_signal_device_annce_params_t);

		LOG_INF("New device commissioned or rejoined (short: 0x%04hx)",
			dev_annce_params->device_short_addr);

		zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished, ZB_ALARM_ANY_PARAM);
		if (zb_err_code == RET_OK) {
			LOG_INF("Joining period extended.");
			zb_err_code = ZB_SCHEDULE_APP_ALARM(
				steering_finished, 0,
				ZB_TIME_ONE_SECOND *
				ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
			ZB_ERROR_CHECK(zb_err_code);
		}
	} break;

	default:
		/* Call default signal handler. */
		//LOG_WRN( "JULEN Unknown signal\n");
		zb_af_set_data_indication(data_indication);
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	}

	/* Update network status LED. */
	if (ZB_JOINED() &&
	    (ZB_SCHEDULE_GET_ALARM_TIME(steering_finished, ZB_ALARM_ANY_PARAM,
					&timeout_bi) == RET_OK)) {
		dk_set_led_on(ZIGBEE_NETWORK_STATE_LED);
	} else {
		dk_set_led_off(ZIGBEE_NETWORK_STATE_LED);
	}

	/*
	 * All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid) {
		zb_buf_free(bufid);
	}
}

int main(void)
{
	int blink_status = 0;

	LOG_INF("Starting ZBOSS Coordinator example");

	zb_set_nvram_erase_at_start(ZB_FALSE);

	/* Initialize */
	configure_gpio();
	register_factory_reset_button(FACTORY_RESET_BUTTON);

	/* Register device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&nwk_coordinator);

	static const zb_uint8_t g_key_nwk[16] = { 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0, 0, 0, 0, 0, 0, 0, 0};
 	zb_secur_setup_nwk_key((zb_uint8_t *) g_key_nwk, 0);

	zb_set_extended_pan_id(ptr_ext_pan_id);

	zb_bdb_set_legacy_device_support(ZB_TRUE);

	app_clusters_attr_init();

	/* Register handlers to identify notifications. */
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZIGBEE_COORDINATOR_ENDPOINT, identify_cb);

	/* Start Zigbee default thread */
	zigbee_enable();

	LOG_INF("ZBOSS Coordinator example started");

	while (1) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
