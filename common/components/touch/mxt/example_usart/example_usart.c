/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
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
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch lowlevel compnent API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and XMega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sendt over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incomming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * BAUD: 115200
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_example.h"
#include "conf_usart_serial.h"

#define MAX_ENTRIES        3
#define STRING_LENGTH     40

#define USART_TX_MAX_LENGTH     0xff

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

static void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer length */
	uint8_t tx_len = STRING_LENGTH * MAX_ENTRIES;

	/* Text buffer */
	char buf[STRING_LENGTH];

	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Read next next touch event in the queue, if the read fails,
		 * check if there is more messages in the queue and try another
		 * read */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}

		/* Format a new entry in the data string that will be sent over
		 * USART */
		sprintf(buf, "Nr: %2d X:%4d Y:%4d Status:0x%2x\n\r",
				touch_event.id, touch_event.x, touch_event.y,
				touch_event.status);

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	tx_len = i * STRING_LENGTH; /* Find the length of the string to send */

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, tx_len);
	}
}

int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	/* TWI configuration */
	twi_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_DEVICE_ADR
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */

#if UC3
	gpio_enable_gpio_pin(MAXTOUCH_CHG_PIN);
#endif

	/* Setup TWI master to communicate to maXTouch device */
	if (twi_master_setup(TWI_INTERFACE, &twi_opt) != STATUS_OK) {
		Assert(false);
	}

	/* Check if the maXTouch device is available on TWI_INTERFACE bus
	 * and MAXTOUCH_DEVICE_ADR address */
	if (mxt_probe_device(TWI_INTERFACE, MAXTOUCH_DEVICE_ADR) != STATUS_OK) {
		Assert(false);
	}

	/* Initialize the maXTouch device */
	if (mxt_init_device(&device, TWI_INTERFACE, MAXTOUCH_DEVICE_ADR,
			MAXTOUCH_CHG_PIN) != STATUS_OK) {
		Assert(false);
	}

	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

#if (defined(__GNUC__) && defined(__AVR32__))
	setbuf(stdout, NULL);
#endif

	printf("\n\rmaXTouch data USART transmitter\n\r");

	/* Reset maXTouch device */
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0) + MXT_GEN_COMMANDPROCESSOR_RESET, 1);

	/* Wait for reset sequence to complete */
	delay_ms(MXT_RESET_TIME);

	/* Set maXTouch configuration */
	mxt_set_config(&device);

	/* Enable CHG mode 1, CHG pin will stay low until message queue is empty */
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_SPT_COMMSCONFIG_T18, 0),
			MXT_COMMSCONFIG_T18_CHG_MODE_bp << 1);

	/* Re-calibrate device (send non-zero value to the defined offset */
	mxt_write_config_reg(&device, mxt_get_object_address(&device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0) +
			MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x1);

	while (true) {
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		if(mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
	}

	return 0;
}
