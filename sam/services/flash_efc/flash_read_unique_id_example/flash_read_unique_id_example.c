/**
 * \file
 *
 * \brief Flash read Unique Identifier example for SAM.
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
 * \mainpage Flash Read Unique Identifier Example
 *
 * \section Purpose
 *
 * This basic example shows how to use the Flash service available on the Atmel SAM flash
 * microcontrollers. It reads and displays the Unique Identifier stored in the Flash.
 *
 * \section Description
 * To read the Unique Identifier, the sequence is:
 * <ul>
 *  <li>Send the Start Read Unique Identifier command (STUI) by writing the Flash Command
 * Register with the STUI command.</li>
 *  <li>When the Unique Identifier is ready to be read, the FRDY bit in the Flash Programming
 * Status Register (EEFC_FSR) falls.</li>
 *  <li>The Unique Identifier is located in the first 128 bits of the Flash memory mapping. So, at the
 * address 0x80000-0x80003.</li>
 *  <li>To stop the Unique Identifier mode, the user needs to send the Stop Read Unique Identifier
 * command (SPUI) by writing the Flash Command Register with the SPUI command.</li>
 *  <li>When the Stop Read Unique Identifier command (SPUI) has been performed, the
 * FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises.</li>
 * </ul>
 *
 * \section Usage
 *
 * -# Build the program and download it into the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on the solutions that users choose.
 * -# On the computer, open and configure a terminal application
 *    (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- Flash Read Unique Identifier Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Reading 128 bits unique identifier
 *     -I- ID: 0xxxxxxxxx, 0xxxxxxxxx, 0xxxxxxxxx, 0xxxxxxxxx
 * \endcode
 * 
 */

#include "board.h"
#include "sysclk.h"
#include "uart.h"
#include "pmc.h"
#include "pio.h"
#include "gpio.h"
#include "flash_efc.h"
#include "conf_board.h"
#include "conf_clock.h"

#define STRING_EOL    "\r"
#define STRING_HEADER "-- Flash Read Unique Identifier Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/**
 *  Configure console using UART.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ BOARD_MCK, 115200, UART_MR_PAR_NO };

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
 *  \brief efc_unique_id_example Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t dw_rc;
	uint32_t unique_id[4];

	/* Initialize the SAM3 system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Initialize Flash service */
	dw_rc = flash_init(FLASH_ACCESS_MODE_128, 4);
	if (dw_rc != FLASH_RC_OK) {
		printf("-F- Initialization error %lu\n\r", dw_rc);
		return 0;
	}

	/* Read the unique ID */
	puts("-I- Reading 128 bits Unique Identifier\r");
	dw_rc = flash_read_unique_id(unique_id, 4);
	if (dw_rc != FLASH_RC_OK) {
		printf("-F- Read the Unique Identifier error %lu\n\r", dw_rc);
		return 0;
	}

	printf("-I- ID: 0x%08lu, 0x%08lu, 0x%08lu, 0x%08lu\n\r",
			unique_id[0], unique_id[1], unique_id[2], unique_id[3]);

	while (1) {
		/* Do nothing */
	}
}
