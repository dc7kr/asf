/**
 * \file
 *
 * \brief SMC PSRAM Example for SAM.
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
 * \mainpage SMC PSRAM Example
 *
 * \section Purpose
 *
 * This example shows how to configure the Static Memory Controller (SMC)
 * for PSRAM, and do write and read operations to check the configuration.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S evaluation kits.
 *
 * \section Description
 *
 * An accurate one-to-one comparison is necessary between PSRAM and SMC waveforms for
 * a complete SMC configuration.
 * * The required steps are:
 * <ul>
 * <li> Configure EBI I/O for PSRAM connection </li>
 * <li> Configure PMC to enable the SMC clock.</li>
 * <li> Refer to the "AC Characteristics" section of the customer PSRAM datasheet.</li>
 * <li> Configure SMC timing to fix the characteristics</li>
 * <li>	Configure SMC mode register for page mode if any.</li>
 * <li>	Access the PSRAM for write or read.</li>
 * </ul>
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- SMC PSRAM Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *
 *     -I- Configure EBI I/O for PSRAM connection.
 *     -I- Configure PMC to enable the SMC clock.
 *     -I- Configure SMC timing and mode.
 *     -I- SMC Setup Register 0xxxxxxxxx.
 *     -I- SMC Pulse Register 0xxxxxxxxx.
 *     -I- SMC Cycle Register 0xxxxxxxxx.
 *     -I- SMC MODE  Register 0xxxxxxxxx.
 *     -I- CTest external PSRAM access.
 *    \endcode
 * 
 */

#include "board.h"
#include "sysclk.h"
#include "gpio.h"
#include "exceptions.h"
#include "pio.h"
#include "uart.h"
#include "pmc.h"
#include "smc.h"
#include "conf_board.h"
#include <stdio.h>

/** Base address of chip select */
#define PSRAM_BASE_ADDRESS         (0x60000000)

#define STRING_EOL    "\r"
#define STRING_HEADER "-- SMC PSRAM Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/**
 * \brief Go/No-Go test of the first 10K-Bytes of external PSRAM access.
   \return 0 if test is failed else 1.
 */
static uint8_t access_psram_test(void)
{
	uint32_t i;
	uint32_t *ptr = (uint32_t *) PSRAM_BASE_ADDRESS;

	for (i = 0; i < 10 * 1024; ++i) {
		if (i & 1) {
			ptr[i] = 0x55AA55AA | (1 << i);
		} else {
			ptr[i] = 0xAA55AA55 | (1 << i);
		}
	}
	for (i = 0; i < 10 * 1024; ++i) {
		if (i & 1) {
			if (ptr[i] != (0x55AA55AA | (1 << i))) {
				return 0;
			}
		} else {
			if (ptr[i] != (0xAA55AA55 | (1 << i))) {
				return 0;
			}
		}
	}
	return 1;
}

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ sysclk_get_cpu_hz(), 115200, UART_MR_PAR_NO };

	/* Configure PIO */
	pio_configure(PINS_UART_PIO, PINS_UART_TYPE, PINS_UART_MASK,
			PINS_UART_ATTR);

	/* Configure PMC */
	pmc_enable_periph_clk(CONSOLE_UART_ID);

	/* Configure UART */
	uart_init(CONSOLE_UART, &uart_console_settings);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/**
 * \brief Application entry point for smc_psram example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initilize the SAM3 system */
	sysclk_init();
	board_init();

	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Configure EBI I/O for PSRAM connection */
	puts("Configure EBI I/O for PSRAM connection.\r");

	pmc_enable_periph_clk(ID_SMC);
	/* complete SMC configuration between PSRAM and SMC waveforms. */
	smc_set_setup_timing(SMC, 0, SMC_SETUP_NWE_SETUP(1)
			| SMC_SETUP_NCS_WR_SETUP(0)
			| SMC_SETUP_NRD_SETUP(2)
			| SMC_SETUP_NCS_RD_SETUP(0));
	smc_set_pulse_timing(SMC, 0, SMC_PULSE_NWE_PULSE(3)
			| SMC_PULSE_NCS_WR_PULSE(4)
			| SMC_PULSE_NRD_PULSE(3)
			| SMC_PULSE_NCS_RD_PULSE(5));
	smc_set_cycle_timing(SMC, 0, SMC_CYCLE_NWE_CYCLE(4)
			| SMC_CYCLE_NRD_CYCLE(5));
	smc_set_mode(SMC, 0, SMC_MODE_READ_MODE | SMC_MODE_WRITE_MODE

#if SAM3S || SAM4S
			| SMC_MODE_DBW_8_BIT
#elif SAM3U
			| SMC_MODE_DBW_BIT_16 
#elif SAM3XA 
			| SMC_MODE_DBW_BIT_8
#endif
			);

	/* Test external PSRAM access */
	puts("Test external PSRAM access. \r");
	uint8_t *ptr = (uint8_t *) PSRAM_BASE_ADDRESS;
	ptr[0] = 0x50;
	ptr[1] = 0x51;
	ptr[2] = 0x52;
	ptr[3] = 0x53;
	if (access_psram_test()) {
		puts("Psram access successful.\r");
	} else {
		puts("Psram access failed.\r");
	}
}
