/**
 * \file
 *
 * \brief Flash program example for SAM.
 *
 * Copyright (c) 2011 - 2012 Atmel Corporation. All rights reserved.
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
 * \mainpage Flash Program Example
 *
 * \section Purpose
 *
 * This basic example shows how to use the Flash service available on the Atmel SAM3 
 * microcontrollers. It details steps required to program the internal flash, and manage secure 
 * and lock bits.
 *
 * \section Description
 *
 * The program performs the following set of commands:
 * - Unlock the last page.
 * - Program the last page of the embedded flash with walking bit pattern (0x1, 0x2, 0x4, ...).
 * - Check if the flash is correctly programmed by reading all the values programmed.
 * - Lock the last page and check if it has been locked correctly.
 * - Set the security bit.
 *
 * The SAM3 MCU features a security bit, based on a specific General Purpose NVM bit 0.
 * When the security bit is enabled, any access to the Flash, SRAM, Core Registers
 * and Internal Peripherals through the ICE interface is forbidden.
 * This example will reproduce this scene.
 *
 * The SAM3 MCU ROM code embeds small In-Application Programming (IAP) Procedure.
 * Since this function is executed from ROM, this allows Flash programming
 * (such as sector write) to be done when code is running out of Flash.
 * We will use IAP function by default in flash driver.
 *
 * \section Usage
 *
 * -# Build the program and download it into the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or the
 *    <a href="http://www.iar.com/website1/1.0.1.0/78/1/">
 *    IAR EWARM User and reference guides</a>, depending on the solutions that users choose.
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
 *     -- Flash Programm Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Unlocking last page
 *     -I- Writing last page with walking bit pattern
 *     -I- Checking page contents  ......................................... ok
 *     -I- Locking last page
 *     -I- Try to program the locked page...
 *     -I- Please open Segger's JMem program
 *     -I- Read memory at address 0x0043FF00 to check contents
 *     -I- Press any key to continue...
 *     -I- Good job!
 *     -I- Now set the security bit
 *     -I- Press any key to continue to see what happened...
 *     -I- Setting GPNVM #0
 *     -I- All tests done
 * \endcode
 * 
 */

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"

#define STRING_EOL    "\r"
#define STRING_HEADER "-- Flash Program Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/**
 *  Configure console using UART.
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
 * Perform initialization and tests on flash.
 */
int main(void)
{
	uint32_t ul_last_page_addr = LAST_PAGE_ADDRESS;
	uint32_t *pul_last_page = (uint32_t *) ul_last_page_addr;
	uint32_t ul_rc;
	uint32_t ul_idx;
	uint8_t uc_key;
	uint32_t ul_page_buffer[IFLASH_PAGE_SIZE / sizeof(uint32_t)];

	/* Initialize the SAM3 system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Initialize flash: 6 wait states for flash writing. */
	ul_rc = flash_init(FLASH_ACCESS_MODE_128, 6);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Initialization error %lu\n\r", ul_rc);
		return 0;
	}

	/* Unlock page */
	puts("-I- Unlocking last page\r");
	ul_rc = flash_unlock(ul_last_page_addr,
			ul_last_page_addr + IFLASH_PAGE_SIZE - 1, 0, 0);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Unlock error %lu\n\r", ul_rc);
		return 0;
	}

	/* Write page */
	puts("-I- Writing last page with walking bit pattern\r");
	for (ul_idx = 0; ul_idx < (IFLASH_PAGE_SIZE / 4); ul_idx++) {
		ul_page_buffer[ul_idx] = 1 << (ul_idx % 32);
	}

#if SAM4S
	/* For SAM4S, the EWP command is not supported, the pages requires
	   erased first. */
	ul_rc = flash_erase_page(ul_last_page_addr, IFLASH_ERASE_PAGES_4);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash programming error %lu\n\r", ul_rc);
		return 0;
	}

	ul_rc = flash_write(ul_last_page_addr, ul_page_buffer,
			IFLASH_PAGE_SIZE, 0);
#else
	ul_rc = flash_write(ul_last_page_addr, ul_page_buffer,
			IFLASH_PAGE_SIZE, 1);
#endif
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash programming error %lu\n\r", ul_rc);
		return 0;
	}

	/* Validate page */
	puts("-I- Checking page contents ");
	for (ul_idx = 0; ul_idx < (IFLASH_PAGE_SIZE / 4); ul_idx++) {
		puts(".");
		if (pul_last_page[ul_idx] != ul_page_buffer[ul_idx]) {
			puts("\n\r-F- data error\r");
			return 0;
		}
	}
	puts("OK\r");

	/* Lock page */
	puts("-I- Locking last page\r");
	ul_rc = flash_lock(ul_last_page_addr,
			ul_last_page_addr + IFLASH_PAGE_SIZE - 1, 0, 0);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash locking error %lu\n\r", ul_rc);
		return 0;
	}

	/* Check if the associated region is locked. */
	puts("-I- Try to program the locked page ...\r");
	ul_rc = flash_write(ul_last_page_addr, ul_page_buffer,
			IFLASH_PAGE_SIZE, 1);
	if (ul_rc != FLASH_RC_OK) {
		printf("-I- The page to be programmed belongs to locked region. Error %lu\n\r", ul_rc);
	}

	puts("-I- Please open Segger's JMem program \r");
	printf("-I- Read memory at address 0x%08lu to check contents\n\r",
			ul_last_page_addr);
	puts("-I- Press any key to continue...\r");
	while (0 != uart_read(CONSOLE_UART, &uc_key));

	puts("-I- Good job!\n\r"
			"-I- Now set the security bit \n\r"
			"-I- Press any key to continue to see what happened...\r");
	while (0 != uart_read(CONSOLE_UART, &uc_key));

	/* Set security bit */
	puts("-I- Setting security bit \r");
	ul_rc = flash_enable_security_bit();
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Set security bit error %lu\n\r", ul_rc);
	}

	puts("-I- All tests done\r");

	while (1) {
		/* Do nothing */
	}
}
