/**
 * \file
 *
 * \brief Matrix example for SAM.
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
 * \mainpage MATRIX Example
 *
 * \section Purpose
 *
 * This example demonstrates the Bus Matrix (MATRIX) provided on
 * SAM microcontrollers. 
 *
 * \section Requirements
 *
 * This package can be used with SAM3 evaluation kits.
 *
 * \section Description
 *
 * This example shows running speed between two matrix configuration.
 * LED toggle times in one second is measured.
 * The first test is done with Round-Robin arbitration without default master. 
 * The second test is done with Round-Robin arbitration with last access master. 
 * As expected, the LED toggle times of test1 is lower than that of test2.
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
 *     -- MATRIX Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Configure system tick to get 1ms tick period.
 *     -- Test1: configure Round-Robin arbitration without default master. --
 *         Led toggled xxx times in one second
 *     -- Test2: configure Round-Robin arbitration with last access master. --
 *         Led toggled xxx times in one second
 *    \endcode
 */

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"

/* Matrix slave number */
#if (SAM3S || SAM4S)
#define MATRIX_SLAVE_NUM    5
#elif (SAM3N)
#define MATRIX_SLAVE_NUM    4
#elif (SAM3XA)
#define MATRIX_SLAVE_NUM    9
#elif (SAM3U)
#define MATRIX_SLAVE_NUM    10
#else
#warning "Not define matrix slave number, set 1 for default."
#define MATRIX_SLAVE_NUM    1
#endif

#define STRING_EOL    "\r"
#define STRING_HEADER "-- MATRIX Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* Global g_ul_ms_ticks in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;

/**
 * \brief Configure the console UART.
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
 * \brief Toggle led at the given time.
 *
 * \param ul_dly_ticks  Delay to wait for, in milliseconds.
 *
 * \return Led toggle times.
 */
static uint32_t toggle_led_test(uint32_t ul_dly_ticks)
{
	int32_t ul_cnt = 0;
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	do {
		ul_cnt++;
		gpio_toggle_pin(LED0_GPIO);
	} while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);

	return ul_cnt;
}

/**
 * \brief Handler for System Tick interrupt.
 *
 * Process System Tick Event.
 * Increment the g_ul_ms_ticks counter.
 */
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t ul_slave_id;
	int32_t ul_cnt;

	/* Initialize the system */
	sysclk_init();
	board_init();

	/* Configure console UART */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Configure systick for 1 ms */
	puts("Configure system tick to get 1ms tick period.\r");
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-F- Systick configuration error\r");
		while (1);
	}

	/* First, test with Round-Robin arbitration without default master */
	puts("-- Test1: configure Round-Robin arbitration without default master. --\r");
	for (ul_slave_id = 0; ul_slave_id < MATRIX_SLAVE_NUM; ul_slave_id++) {
		matrix_set_slave_arbitration_type(ul_slave_id,
				MATRIX_ARBT_ROUND_ROBIN);
		matrix_set_slave_default_master_type(ul_slave_id,
				MATRIX_DEFMSTR_NO_DEFAULT_MASTER);
	}
	ul_cnt = toggle_led_test(1000);
	printf("    Led toggled %d times in one second\n\r", ul_cnt);

	/* Second, test with Round-Robin arbitration with last access master */
	puts("-- Test2: configure Round-Robin arbitration with last access master. --\r");
	for (ul_slave_id = 0; ul_slave_id < MATRIX_SLAVE_NUM; ul_slave_id++) {
		matrix_set_slave_arbitration_type(ul_slave_id,
				MATRIX_ARBT_ROUND_ROBIN);
		matrix_set_slave_default_master_type(ul_slave_id,
				MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	}
	ul_cnt = toggle_led_test(1000);
	printf("    Led toggled %d times in one second\n\r", ul_cnt);

	/* Endless loop */
	while (1) {
	}
}
