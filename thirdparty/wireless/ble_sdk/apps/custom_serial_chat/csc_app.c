/**
 * \file
 *
 * \brief Custom Serial Chat Application declarations
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
 * This is the reference manual for the Custom Serial Chat Application declarations
 */
/*- Includes -----------------------------------------------------------------------*/
#include <asf.h>
#include "platform.h"
#include "at_ble_api.h"
#include "ble_manager.h"
#include "csc_app.h"
#include "cscp.h"
#include "cscs.h"
#include "ble_utils.h"
#include "console_serial.h"
#include "conf_extint.h"
#include "sio2host.h"  

/* =========================== GLOBALS ============================================================ */

/* Received notification data structure */
csc_report_ntf_t recv_ntf_info;

/* Data length to be send over the air */
uint16_t send_length = 0;

/* Buffer data to be send over the air */
uint8_t send_data[APP_TX_BUF_SIZE];

/* Function used for receive data */
static void csc_app_recv_buf(uint8_t *recv_data, uint8_t recv_len)
{
	uint16_t ind = 0;
	if (recv_len){
		for (ind = 0; ind < recv_len; ind++){
			sio2host_putchar(recv_data[ind]);
		}
		DBG_LOG("\r\n");
	}
}

/* Callback called for new data from remote device */
static void csc_prf_report_ntf_cb(csc_report_ntf_t *report_info)
{
	DBG_LOG("\r\n");
	csc_app_recv_buf(report_info->recv_buff, report_info->recv_buff_len);
}

/* Function used for send data */
static void csc_app_send_buf(void)
{
	uint16_t ind = 0;
	uint16_t len = 0;
	uint8_t buff = 0;
	len = sio2host_rx(&buff, 1);
	if (len){
		for (ind = 0; ind < len; ind++){
			if(buff != ENTER_BUTTON_PRESS){
				sio2host_putchar(buff);  
				if(send_length < APP_TX_BUF_SIZE){
					send_data[send_length++] = buff;
				}else{
					csc_prf_send_data(&send_data[0], send_length);
					send_length = 0;
				}
			}else{ // User press enter to send data
				if(send_length){
					ind = send_length;
					send_length = 0;
					csc_prf_send_data(&send_data[0], ind);
					DBG_LOG("\r\n");
				}
			}
		}
	}
}

bool app_exec = true;
int main(void )
{
#if SAMG55
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();
#elif SAM0
	system_init();
#endif
	
	/* Initialize serial console */
	sio2host_init();
	
	DBG_LOG("Initializing Custom Serial Chat Application");
	
	/* Initialize the buffer address and buffer length based on user input */
	csc_prf_buf_init(&send_data[0], APP_TX_BUF_SIZE);
	
	/* initialize the ble chip  and Set the device mac address */
	ble_device_init(NULL);
	
	/* Register the notification handler */
	notify_recv_ntf_handler(csc_prf_report_ntf_cb);
	
	/* Capturing the events  */
	while(app_exec){
		ble_event_task();
		csc_app_send_buf();	
	}
	return 0;
}



