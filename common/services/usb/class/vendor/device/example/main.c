/**
 * \file
 *
 * \brief Main functions for USB Device vendor example
 *
 * Copyright (c) 2011 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
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
 *    Atmel microcontroller product.
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

#include "compiler.h"
#include "preprocessor.h"
#include "board.h"
#include "gpio.h"
#include "sysclk.h"
#include "sleepmgr.h"
#include "conf_usb.h"
#include "udd.h"
#include "udc.h"
#include "udi_vendor.h"
#include "ui.h"

static bool main_b_vendor_enable = false;

/**
 * \name Buffer for loopback
 */
//@{
//! Size of buffer used for the loopback
#define  MAIN_LOOPBACK_SIZE    1024
static uint8_t main_buf_loopback[MAIN_LOOPBACK_SIZE];
static uint8_t main_buf_iso_sel;
//@}

// check configuration
#if UDI_VENDOR_EPS_SIZE_ISO_FS>(MAIN_LOOPBACK_SIZE/2)
# error UDI_VENDOR_EPS_SIZE_ISO_FS must be <= MAIN_LOOPBACK_SIZE/2 in cond_usb.h
#endif
#ifdef USB_DEVICE_HS_SUPPORT
# if UDI_VENDOR_EPS_SIZE_ISO_HS>(MAIN_LOOPBACK_SIZE/2)
#   error UDI_VENDOR_EPS_SIZE_ISO_HS must be <= MAIN_LOOPBACK_SIZE/2 in cond_usb.h
# endif
#endif

void main_vendor_int_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered);
void main_vendor_int_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered);
void main_vendor_bulk_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered);
void main_vendor_bulk_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered);
void main_vendor_iso_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered);
void main_vendor_iso_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered);

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();
	sysclk_init();
	board_init();
	ui_init();

	// Start USB stack to authorize VBus monitoring
	udc_start();

	if (!udc_include_vbus_monitoring()) {
		// VBUS monitoring is not available on this product
		// thereby VBUS has to be considered as present
		main_vbus_action(true);
	}

	// The main loop manages only the power mode
	// because the USB management is done by interrupt
	while (true) {
		sleepmgr_enter_sleep();
	}
}

void main_vbus_action(bool b_high)
{
	if (b_high) {
		// Attach USB Device
		udc_attach();
	} else {
		// VBUS not present
		udc_detach();
	}
}

void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_vendor_enable)
		return;
	ui_process(udd_get_frame_number());
}

bool main_vendor_enable(void)
{
	main_b_vendor_enable = true;
	// Start data reception on OUT endpoints
	main_vendor_int_in_received(UDD_EP_TRANSFER_OK, 0);
	main_vendor_bulk_in_received(UDD_EP_TRANSFER_OK, 0);
	main_buf_iso_sel=0;
	main_vendor_iso_out_received(UDD_EP_TRANSFER_OK, 0);
	return true;
}

void main_vendor_disable(void)
{
	main_b_vendor_enable = false;
}

bool main_setup_out_received(void)
{
	ui_loop_back_state(true);
	udd_g_ctrlreq.payload = main_buf_loopback;
	udd_g_ctrlreq.payload_size = min(
			udd_g_ctrlreq.req.wLength,
			sizeof(main_buf_loopback));
	return true;
}

bool main_setup_in_received(void)
{
	ui_loop_back_state(false);
	udd_g_ctrlreq.payload = main_buf_loopback;
	udd_g_ctrlreq.payload_size =
			min( udd_g_ctrlreq.req.wLength,
			sizeof(main_buf_loopback) );
	return true;
}

void main_vendor_int_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Tranfert aborted, then stop loopback
	}
	ui_loop_back_state(false);
	// Wait a full buffer
	udi_vendor_interrupt_out_run(
			main_buf_loopback,
			sizeof(main_buf_loopback),
			main_vendor_int_out_received);
}

void main_vendor_int_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Tranfert aborted, then stop loopback
	}
	ui_loop_back_state(true);
	// Send on IN endpoint the data received on endpoint OUT
	udi_vendor_interrupt_in_run(
			main_buf_loopback,
			nb_transfered,
			main_vendor_int_in_received);
}

void main_vendor_bulk_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Tranfert aborted, then stop loopback
	}
	ui_loop_back_state(false);
	// Wait a full buffer
	udi_vendor_bulk_out_run(
			main_buf_loopback,
			sizeof(main_buf_loopback),
			main_vendor_bulk_out_received);
}

void main_vendor_bulk_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	if (UDD_EP_TRANSFER_OK != status) {
		return; // Tranfert aborted, then stop loopback
	}
	ui_loop_back_state(true);
	// Send on IN endpoint the data received on endpoint OUT
	udi_vendor_bulk_in_run(
			main_buf_loopback,
			nb_transfered,
			main_vendor_bulk_in_received);
}

void main_vendor_iso_in_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	ui_loop_back_state(false);
}

void main_vendor_iso_out_received(udd_ep_status_t status,
		iram_size_t nb_transfered)
{
	uint8_t *buf_ptr;

	if (UDD_EP_TRANSFER_OK != status) {
		return; // Tranfert aborted, then stop loopback
	}

	if (nb_transfered) {
		ui_loop_back_state(true);
		// Send on IN endpoint the data received on endpoint OUT
		buf_ptr = &main_buf_loopback[ main_buf_iso_sel
				*(sizeof(main_buf_loopback)/2) ];
		udi_vendor_iso_in_run(
				buf_ptr,
				nb_transfered,
				main_vendor_iso_in_received);
	}

	// Switch of buffer
	main_buf_iso_sel = main_buf_iso_sel? 0:1;

	// Immediatly enable a transfer on next USB isochronous OUT packet
	// to avoid to skip a USB packet.
	// NOTE:
	// Here the expected buffer size is equal to endpoint size.
	// Thus, this transfer request will end after reception of
	// one USB packet.
	//
	// When using buffer size larger than endpoint size,
	// the requested transfer is stopped when the buffer is = full*.
	// *on USBC and XMEGA USB driver, the buffer is full
	// when "number of data transfered" > "buffer size" - "endppoint size".
	buf_ptr = &main_buf_loopback[ main_buf_iso_sel
			*(sizeof(main_buf_loopback)/2) ];

	// Send on IN endpoint the data received on endpoint OUT
	udi_vendor_iso_out_run(
			buf_ptr,
			udd_is_high_speed()?
				UDI_VENDOR_EPS_SIZE_ISO_HS:UDI_VENDOR_EPS_SIZE_ISO_FS,
			main_vendor_iso_out_received);
}

/**
 * \mainpage ASF USB Device Vendor Example
 *
 * \section intro Introduction
 * This example shows how to implement a USB Device implementing Vendor Class
 * on Atmel MCU with USB module.
 *
 * \section startup Startup
 * The example uses a vendor class which implements a loopback on
 * all endpoints types: control, interrupt, bulk and isochronous.
 * After loading firmware, connect the board
 * (EVKxx,XPlain,...) to the USB Host. A Host applciation developped
 * on libusb library is provided in application note AVR4001.

 * \note
 * When the application is connected for the first time to the PC,
 * the operating system will detect a new peripheral:
 * - This will open a new hardware installation wizard on Windows operating systems.
 * - Choose "No, not this time" to connect to Windows Update for this installation
 * - click "Next"
 * - When requested to search the INF file, browse the avr4901\drivers\ folder
 *   provided package of the AVR4901 applicatinon note.
 * - click "Next"
*
 * \copydoc UI
 *
 * \section example About example
 *
 * The example uses the following module groups:
 * - Basic modules:
 *   Startup, board, clock, interrupt, power management
 * - USB Device stack and vendor modules:
 *   <br>services/usb/
 *   <br>services/usb/udc/
 *   <br>services/usb/class/vendor/
 * - Specific implementation:
 *    - main.c,
 *      <br>initializes clock
 *      <br>initializes interrupt
 *      <br>manages UI
 *    - specific implementation for each target "./examples/product_board/":
 *       - conf_foo.h   configuration of each module
 *       - ui.c        implement of user's interface (leds)
 */
