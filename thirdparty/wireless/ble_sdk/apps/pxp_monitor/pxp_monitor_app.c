/**
* \file
*
* \brief Proximity Monitor Profile Application
*
* Copyright (c) 2015 Atmel Corporation. All rights reserved.
*
* \asf_license_start
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel micro controller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \asf_license_stop
*
*/

/*
* Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
*Support</a>
*/

/**
* \mainpage
* \section preface Preface
* This is the reference manual for the Proximity Monitor Profile Application
*/
/*- Includes ---------------------------------------------------------------*/

#define DEBUG_LOG

#include <asf.h>
#include "platform.h"

#include "pxp_monitor_app.h"

#if defined IMMEDIATE_ALERT_SERVICE
#include "immediate_alert.h"
#endif

#if defined TX_POWER_SERVICE
#include "tx_power.h"
#endif

#if defined LINK_LOSS_SERVICE
#include "link_loss.h"
#endif

#if defined PROXIMITY_REPORTER
#include "pxp_reporter.h"
#endif

at_ble_addr_t peer_addr
= {AT_BLE_ADDRESS_PUBLIC, {0x03, 0x18, 0xf0, 0x05, 0xf0, 0xf8}};

extern gatt_txps_char_handler_t txps_handle;
extern gatt_lls_char_handler_t lls_handle;
extern gatt_ias_char_handler_t ias_handle;

extern at_ble_connected_t ble_connected_dev_info[MAX_DEVICE_CONNECTED];
extern bool pxp_connect_request_flag;

volatile bool app_timer_done = false;
pxp_current_alert_t alert_level = PXP_NO_ALERT;

volatile bool button_pressed = false;

void button_cb(void)
{
	button_pressed = true;
}

/**@brief Check for Link Loss and Path Loss alert
* check for Low Alert value if crossed write Low Alert value to Immediate Alert
*Service. High Alert value if crossed write High Alert value to IAS service
*
* @param[in] conn_handle Connection handle of a connected device
*/
static void rssi_update(at_ble_handle_t conn_handle)
{
	int8_t rssi_power = 0;
	at_ble_status_t status;
	app_timer_done = false;

	/* Get the Received signal strength intensity of the connect
	*device/handle*/
	if ((status = at_ble_rx_power_get(conn_handle,&rssi_power)) != AT_BLE_SUCCESS)
	{
		DBG_LOG("at_ble_rx_power_get failed,reason %d",status);
	}

	DBG_LOG("Rx Power(RSSI):%04d dBm", rssi_power);

	/* if received rssi is above no alert zone and below high alert zone */
	if ((rssi_power <= PXP_LOW_ALERT_RANGE) &&
			(rssi_power >= PXP_HIGH_ALERT_RANGE)) {
		if (!(alert_level == PXP_MID_ALERT)) {
			ias_alert_level_write(conn_handle,
					ias_handle.char_handle,
					IAS_MID_ALERT);
			alert_level = PXP_MID_ALERT;
		}

		DBG_LOG_CONT("---Mild Alert!");
		LED_Toggle(LED0);
	}
	/* if received rssi is above mild alert zone */
	else if (rssi_power < PXP_HIGH_ALERT_RANGE) {
		if (!(alert_level == PXP_HIGH_ALERT)) {
			ias_alert_level_write(conn_handle,
					ias_handle.char_handle,
					IAS_HIGH_ALERT);
			alert_level = PXP_HIGH_ALERT;
		}
		DBG_LOG_CONT("---High Alert!!!");
		LED_On(LED0);
	}
	/* if received rssi is below mild alert zone */
	else {
		if (!(alert_level == PXP_NO_ALERT)) {
			ias_alert_level_write(conn_handle,
					ias_handle.char_handle,
					IAS_NO_ALERT);
			alert_level = PXP_NO_ALERT;
			LED_Off(LED0);
		}
		DBG_LOG_CONT("---No Alert");
	}
}

/**@brief Proximity Application initialization
* start the device scanning process
*/
static void pxp_app_init(void)
{
	at_ble_status_t scan_status;

	/* Initialize the scanning procedure */
	scan_status = gap_dev_scan();

	/* Check for scan status */
	if (scan_status == AT_BLE_INVALID_PARAM) {
		DBG_LOG("Scan parameters are invalid");
	} else if (scan_status == AT_BLE_FAILURE) {
		DBG_LOG("Scanning Failed Generic error");
	}
}

/* @brief timer call back for rssi update
* enable the flags to execute the application taskc
*
*/
static void timer_callback_handler(void)
{
	/* Stop the timer */
	hw_timer_stop();

	/* Enable the flag the serve the task */
	app_timer_done = true;
}

int main(void)
{	
	#if SAMG55
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();
	#elif SAM0
	system_init();
	#endif

	button_init();
	
	/* Initialize serial console */
	serial_console_init();

	/* Initialize the hardware timer */
	hw_timer_init();

	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);

	/* initialize the BLE chip  and Set the device mac address */
	ble_device_init(NULL);

	DBG_LOG("Initializing Proximity Monitor Application");

	/* Initialize the pxp service */
	pxp_app_init();

	while (1) {
		/* BLE Event Task */
		ble_event_task();

		/* Application Task */
		if (app_timer_done) {
			if (pxp_connect_request_flag) {
				at_ble_disconnected_t pxp_connect_request_fail;
				pxp_connect_request_fail.reason
					= AT_BLE_TERMINATED_BY_USER;
				pxp_connect_request_flag = false;
				if (at_ble_connect_cancel() == AT_BLE_SUCCESS) {
					DBG_LOG("Connection Timeout");
					pxp_disconnect_event_handler(
							&pxp_connect_request_fail);
				} else {
					DBG_LOG(
							"Unable to connect with device. Reseting the device");
					ble_device_init(NULL);
					pxp_app_init();
				}
			} else {
				rssi_update(ble_connected_dev_info[0].handle);
				hw_timer_start(PXP_RSSI_UPDATE_INTERVAL);
			}

			app_timer_done = false;
		}
	}
}
