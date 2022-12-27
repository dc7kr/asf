/**
 * \file
 *
 * \brief This example uses the mXT143E Xplained kit as a touch pad
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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

/**
 * \mainpage
 *
 * \section intro Introduction
 * This example will create a HID class USB mouse on the host computer and
 * emulate a trackpad.
 *
 * \section files Main files:
 * - example_hid.c: maXTouch component HID example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usb.h: configuration of USART driver
 *
 * \section apiinfo maXTouch lowlevel component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and XMEGA devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * When the XMEGA-A3BU Xplained kit is connected to a host computer supporting
 * USB HID class it will create a mouse device. The surface of the maXTouch kit
 * will function as a trackpad. First finger on the trackpad will move the pointer
 * and the second finger will emulate left clicks on a mouse.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */

#include <asf.h>
#include "ui.h"
#include "conf_example.h"

uint8_t new_touch_event = 0;
static bool main_b_mouse_enable = false;

/**
 * \brief Setting predefined maXTouch configuration
 *
 * \ param *device Pointer to mxt_device struct
 */
static void mxt_set_config(struct mxt_device *device)
{
	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x10, 0x05, 0x0a, 0x14, 0x64, 0x00, 0x05,
		0x0a, 0x00, 0x00,
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8f, 0x00, 0x00, 0x0d, 0x0b, 0x00, 0x21,
		0x3c, 0x0f, 0x00, 0x32, 0x01, 0x01, 0x00,
		0x08, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x1c, 0x1c, 0x37, 0x37, 0x8f, 0x50,
		0xcf, 0x6e, 0x00, 0x02, 0x2f, 0x2c, 0x00
	};

	/* T48 configuration object data */
	uint8_t t48_object[] = {
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* Writing data to configuration register in T7 configuration
	 * object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0xff);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0xff);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x32);

	/* Writing configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCG_TOUCHSUPPRESSION_T48, 0), &t48_object);
}

int main(void)
{
#ifdef USB_DEVICE_LOW_SPEED
	uint16_t virtual_sof_sub;
	uint16_t virtual_sof;
#endif

	/* TWI configuration */
	twi_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_DEVICE_ADR
	};

	/* maXTouch data structure */
	static struct mxt_device device;

	/* Initialize system clocks */
	sysclk_init();

	/* Initialize interrupt vectors */
	irq_initialize_vectors();

	/* Enable interrupts */
	cpu_irq_enable();

	/* Initialize the sleep manager */
	sleepmgr_init();

	/* Initialize the board */
	board_init();

	/* Setup TWI master to communicate to maXTouch device */
	if (twi_master_setup(TWI_INTERFACE, &twi_opt) != STATUS_OK) {
		Assert(false);
	}

	/* Proble for any available maXTouch device on the defined bus address */
	if (mxt_probe_device(TWI_INTERFACE, MAXTOUCH_DEVICE_ADR) != STATUS_OK) {
		Assert(false);
	}

	/* Initialize the maXTouch device on the defined bus address */
	if (mxt_init_device(&device, TWI_INTERFACE, MAXTOUCH_DEVICE_ADR,
			MAXTOUCH_CHG_PIN) != STATUS_OK) {
		Assert(false);
	}

	/* Send softreset command to maXTouch device (send a non-zero value to
	 * the defined offset */
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0) + MXT_GEN_COMMANDPROCESSOR_RESET, 1);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Set configuration */
	mxt_set_config(&device);

	/* Set CHG pin mode to 1 (stay low until message queue is empty */
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_SPT_COMMSCONFIG_T18, 0),
			MXT_COMMSCONFIG_T18_CHG_MODE_bp << 1);

	/* Re-calibrate device (send non-zero value to the defined offset*/
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0) +
			MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x1);

	ui_init();
	ui_powerdown();

	/* Start USB stack to authorize VBus monitoring */
	udc_start();

	if (!udc_include_vbus_monitoring()) {
		/* VBUS monitoring is not available on this product
		 * thereby VBUS has to be considered as present */
		main_vbus_action(true);
	}

	/* The main loop manages only the power mode
	 * because the USB management is done by interrupt */
	while (true) {
		if (mxt_is_message_pending(&device)) {
			if (mxt_read_touch_event(&device, &touch_event) == STATUS_OK) {
				new_touch_event = 1;
			}
		}

		sleepmgr_enter_sleep();

#ifdef   USB_DEVICE_LOW_SPEED
		/* No USB "Keep alive" interrupt available in low speed
		 * to scan mouse interface then use main loop */
		if (main_b_mouse_enable) {
			virtual_sof_sub = 0;
			virtual_sof = 0;
			if (virtual_sof_sub++ == 700) {
				virtual_sof_sub = 0;
				ui_process(virtual_sof++);
			}
		}
#endif
	}
}

void main_vbus_action(bool b_high)
{
	if (b_high) {
		/* Attach USB Device */
		udc_attach();
	} else {
		/* VBUS not present */
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
	if (!main_b_mouse_enable) {
		return;
	}
	ui_process(udd_get_frame_number());
}

void main_remotewakeup_enable(void)
{
	ui_wakeup_enable();
}

void main_remotewakeup_disable(void)
{
	ui_wakeup_disable();
}

bool main_mouse_enable(void)
{
	main_b_mouse_enable = true;
	return true;
}

void main_mouse_disable(void)
{
	main_b_mouse_enable = false;
}
