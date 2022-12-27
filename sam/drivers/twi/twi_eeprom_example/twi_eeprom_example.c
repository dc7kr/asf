/**
 * \file
 *
 * \brief TWI EEPROM Example for SAM.
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

/**
 * \mainpage TWI EEPROM Example
 *
 * \section intro Introduction
 *
 * The application demonstrates how to use the SAM TWI driver to access an
 * external serial EEPROM chip.
 *
 * \section files Main files:
 *  - twi.c PMC SAM3 Two-Wire Interface driver implementation.
 *  - twi.h PMC SAM3 Two-Wire Interface driver definitions.
 *  - twi_eeprom_example.c Example application.
 *
 * \section exampledescription Description of the Example
 * Upon startup, the program configures PIOs for console UART, LEDs and TWI
 * connected to EEPROM on board. Then it configures the TWI driver and data
 * package. The clock of I2C bus is set as 400kHz.
 * After initializing the master mode, the example sends test pattern to the
 * EEPROM. When sending is complete, TWI driver reads the memory and saves the
 * content in the reception buffer. Then the program compares the content
 * received with the test pattern sent before and prints the comparsion result.
 * The corresponding LED is turned on.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR EWARM.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 * Support and FAQ: http://support.atmel.com/
 *
 */

#include <stdio.h>
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "pmc.h"
#include "twi.h"
#include "conf_board.h"
#include "led.h"
#include "sysclk.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** */
#define CONSOLE_BAUD_RATE  115200

/** EEPROM Wait Time */
#define WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWI_CLK     400000
/** Address of AT24C chips */
#define AT24C_ADDRESS           0x50
#define EEPROM_MEM_ADDR         0
#define EEPROM_MEM_ADDR_LENGTH  2

/** Data to be sent */
#define  TEST_DATA_LENGTH  (sizeof(test_data_tx)/sizeof(uint8_t))

#define STRING_EOL    "\r"
#define STRING_HEADER "--TWI EEPROM Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

static const uint8_t test_data_tx[] = {
	/** SAM TWI EEPROM EXAMPLE */
	'S', 'A', 'M', ' ', 'T', 'W', 'I', ' ',
	'E', 'E', 'P', 'R', 'O', 'M', ' ',
	'E', 'X', 'A', 'M', 'P', 'L', 'E'
};

/** Reception buffer */
static uint8_t test_data_rx[TEST_DATA_LENGTH] = { 0 };

/** Global timestamp in milliseconds since start of application */
volatile uint32_t dw_ms_ticks = 0;

/**
 *  \brief Handler for Sytem Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void)
{
	dw_ms_ticks++;
}

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings = {
		sysclk_get_main_hz(), CONSOLE_BAUD_RATE, UART_MR_PAR_NO
	};

	/* Enable the peripheral clock for console UART */
	pmc_enable_periph_clk(CONSOLE_UART_ID);

	/* Configure UART peripheral */
	uart_init(CONSOLE_UART, &uart_console_settings);
}

/**
 *  \brief Wait for the given number of milliseconds (using the dwTimeStamp
 *         generated by the SAM3 microcontrollers' system tick).
 *  \param dw_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t dw_dly_ticks)
{
	uint32_t dw_cur_ticks;

	dw_cur_ticks = dw_ms_ticks;
	while ((dw_ms_ticks - dw_cur_ticks) < dw_dly_ticks);
}

/**
 * \brief Application entry point for TWI EEPROM example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t i;
	twi_options_t opt;
	twi_packet_t packet_tx, packet_rx;

	/* Initialize the SAM3 system */
	sysclk_init();

	/* Initialize the board */
	board_init();

	/* Turn off LEDs */
	LED_Off(LED0_GPIO);
	LED_Off(LED1_GPIO);
	
	/* Initialize the console UART */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Configure systick for 1 ms */
	puts("Configure system tick to get 1ms tick period.\r");
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
		}
	}

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(BOARD_ID_TWI_EEPROM);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_main_hz();
	opt.speed      = TWI_CLK;

	/* Configure the data packet to be transmitted */
	packet_tx.chip        = AT24C_ADDRESS;
	packet_tx.addr        = EEPROM_MEM_ADDR;
	packet_tx.addr_length = EEPROM_MEM_ADDR_LENGTH;
	packet_tx.buffer      = (uint8_t *) test_data_tx;
	packet_tx.length      = TEST_DATA_LENGTH;

	/* Configure the data packet to be received */
	packet_rx.chip        = packet_tx.chip;
	packet_rx.addr        = packet_tx.addr;
	packet_rx.addr_length = packet_tx.addr_length;
	packet_rx.buffer      = test_data_rx;
	packet_rx.length      = packet_tx.length;

	if (twi_master_init(BOARD_BASE_TWI_EEPROM, &opt) != TWI_SUCCESS) {
		puts("-E-\tTWI master initialization failed.\r");
		LED_On(LED0_GPIO);
		LED_On(LED1_GPIO);
		while (1) {
			/* Capture error */
		}
	}

	/* Send test pattern to EEPROM */
	if (twi_master_write(BOARD_BASE_TWI_EEPROM, &packet_tx) != TWI_SUCCESS) {
		puts("-E-\tTWI master write packet failed.\r");
		LED_On(LED0_GPIO);
		LED_On(LED1_GPIO);
		while (1) {
			/* Capture error */
		}
	}
	printf("Write:\tOK!\n\r");

	/* Wait at least 10 ms */
	mdelay(WAIT_TIME);

	/* Get memory from EEPROM */
	if (twi_master_read(BOARD_BASE_TWI_EEPROM, &packet_rx) != TWI_SUCCESS) {
		puts("-E-\tTWI master read packet failed.\r");
		LED_On(LED0_GPIO);
		LED_On(LED1_GPIO);
		while (1) {
			/* Capture error */
		}
	}
	puts("Read:\tOK!\r");

	/* Compare the sent and the received */
	for (i = 0; i < TEST_DATA_LENGTH; i++) {
		if (test_data_tx[i] != test_data_rx[i]) {
			/* No match */
			puts("Data comparison:\tUnmatched!\r");
			LED_On(LED0_GPIO);
			while (1) {
				/* Capture error */
			}
		}
	}
	/* Match */
	puts("Data comparison:\tMatched!\r");
	LED_On(LED1_GPIO);

	while (1) {
	}
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
