/**
 * \file
 *
 * \brief Real-time Timer (RTT) example for SAM.
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
 * \mainpage RTT Example
 *
 * \section Purpose
 *
 * This example demonstrates the Real-Time Timer (RTT) provided on
 * SAM3 microcontrollers. It enables the user to set an alarm and watch
 * it being triggered when the timer reaches the corresponding value.
 *
 * \section Requirements
 *
 * This package can be used with SAM evaluation kits.
 *
 * \section Description
 *
 * When launched, this program displays a timer count and a menu on the terminal,
 * enabling the user to choose between several options.
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
 *     -- RTT Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Time: 0
 *     Menu:
 *     r - Reset timer
 *     s - Set alarm
 *     Choice?
 *    \endcode
 */

#include "board.h"
#include "sysclk.h"
#include "gpio.h"
#include "uart.h"
#include "pmc.h"
#include "rtt.h"
#include "pio.h"
#include "conf_board.h"
#include "conf_clock.h"

/** Device state: in the main menu. */
#define STATE_MAIN_MENU      0
/** Device state: user is setting an alarm time. */
#define STATE_SET_ALARM      1

/** ASCII char definition for backspace. */
#define ASCII_BS    8
/** ASCII char definition for carriage return. */
#define ASCII_CR    13

#define STRING_EOL    "\r"
#define STRING_HEADER "-- RTT Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Current device state. */
volatile uint8_t uc_state;

/** New alarm time being currently entered. */
volatile uint32_t dw_new_alarm;

/** Indicate that an alarm has occured but has not been cleared. */
volatile uint8_t uc_alarmed;

/**
 * \brief Refresh display on terminal.
 *
 * Update the terminal display to show the current menu and the current time
 * depending on the device state.
 */
static void refresh_display(void)
{
	printf("%c[2J\r", 27);
	printf("Time: %u\n\r", (unsigned int)rtt_read_timer_value(RTT));

	/* Display alarm */
	if (uc_alarmed) {
		puts("!!! ALARM !!!\r");
	}

	/* Main menu */
	if (uc_state == STATE_MAIN_MENU) {
		puts("Menu:\n\r"
				" r - Reset timer\n\r"
				" s - Set alarm\r");
		if (uc_alarmed) {
			puts(" c - Clear alarm notification\r");
		}
		puts("\n\rChoice? ");
	} else {
		if (uc_state == STATE_SET_ALARM) {
			puts("Enter alarm time: ");
			if (dw_new_alarm != 0) {
				printf("%u", dw_new_alarm);
			}
		}
	}
}

/**
 * \brief RTT configuration function.
 *
 * Configure the RTT to generate a one second tick, which triggers the RTTINC
 * interrupt.
 */
static void configure_rtt(void)
{
	uint32_t previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_init(RTT, 32768);
	previous_time = rtt_read_timer_value(RTT);
	while (previous_time == rtt_read_timer_value(RTT));

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ SystemCoreClock, 115200, UART_MR_PAR_NO };

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
 * \brief Interrupt handler for the RTT.
 *
 * Display the current time on the terminal.
 */
void RTT_Handler(void)
{
	uint32_t status;

	/* Get RTT status */
	status = rtt_get_status(RTT);

	/* Time has changed, refresh display */
	if ((status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		refresh_display();
	}

	/* Alarm */
	if ((status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		uc_alarmed = 1;
		refresh_display();
	}
}

/**
 * \brief Application entry point for RTT example.
 *
 * Initialize the RTT, display the current time and allow the user to
 * perform several actions: clear the timer, set an alarm, etc.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t c;

	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Configure console UART */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Configure RTT */
	configure_rtt();

	/* Initialize state machine */
	uc_state = STATE_MAIN_MENU;
	uc_alarmed = 0;
	refresh_display();

	/* User input loop */
	while (1) {
		/* Wait for user input */
		while (uart_read(CONSOLE_UART, &c));

		/* Main menu mode */
		if (uc_state == STATE_MAIN_MENU) {
			/* Reset timer */
			if (c == 'r') {
				configure_rtt();
				refresh_display();
			} else if (c == 's') { /* Set alarm */
				uc_state = STATE_SET_ALARM;
				dw_new_alarm = 0;
				refresh_display();
			} else { /* Clear alarm */
				if ((c == 'c') && uc_alarmed) {
					uc_alarmed = 0;
					refresh_display();
				}
			}
		} else if (uc_state == STATE_SET_ALARM) { /* Set alarm mode */
			/* Number */
			if ((c >= '0') && (c <= '9')) {
				dw_new_alarm = dw_new_alarm * 10 + c - '0';
				refresh_display();
			} else if (c == ASCII_BS) {
				uart_write(CONSOLE_UART, c);
				dw_new_alarm /= 10;
				refresh_display();
			} else if (c == ASCII_CR) {
				/* Avoid newAlarm = 0 case */
				if (dw_new_alarm != 0) {
					rtt_write_alarm_time(RTT, dw_new_alarm);
				}

				uc_state = STATE_MAIN_MENU;
				refresh_display();
			}
		}
	}
}
