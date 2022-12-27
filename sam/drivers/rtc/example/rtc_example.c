/**
 * \file
 *
 * \brief Real-Time Clock (RTC) example for SAM.
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
 * \mainpage RTC Example
 *
 * \section Purpose
 *
 * This basic example shows how to use the Real-Time Clock (RTC) peripheral
 * available on the SAM3  microcontrollers. The RTC enables easy
 * time and date management and allows the user to monitor events like a
 * configurable alarm, second change, calendar change, and so on.
 *
 * \section Requirements
 *
 * This example can be used with SAM3 evaluation kits.
 *
 * \section Description
 *
 * Upon startup, the program displays the currently set time and date
 * and a menu to perform the following:
 *     \code
 *     Menu:
 *        t - Set time
 *        d - Set date
 *        i - Set time alarm
 *        m - Set date alarm
 *        c - Clear the alarm notification (only if it has been triggered)
 *        w - Generate Waveform
 *     \endcode
 *
 * "w" is an additional option for SAM3SD8. An RTC output can be programmed to
 * generate several waveforms, including a prescaled clock derived from slow clock.
 *
 * Setting the time, date and time alarm is done by using Menu option, and
 * the display is updated accordingly.
 *
 * The time alarm is triggered only when the second, minute and hour match the preset
 * values; the date alarm is triggered only when the month and date match the preset
 * values. 
 *
 * Generating waveform is done by using Menu option "w" and a menu to peform the following:
 *     \code
 *     Menu:
 *     0 - No Waveform
 *     1 - 1 Hz square wave
 *     2 - 32 Hz square wave
 *     3 - 64 Hz square wave
 *     4 - 512 Hz square wave
 *     5 - Toggles when alarm flag rise
 *     6 - Copy of the alarm flag
 *     7 - Duty cycle programmable pulse
 *     8 - Quit
 *     \endcode
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
 *     -- RTC Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *
 *     Menu:
 *     t - Set time
 *     d - Set date
 *     i - Set time alarm
 *     m - Set date alarm      
 *    \endcode
 * -# Press one of the keys listed in the menu to perform the corresponding action.
 * 
 */

#include "board.h"
#include "sysclk.h"
#include "uart.h"
#include "pmc.h"
#include "pio.h"
#include "gpio.h"
#include "rtc.h"
#include "conf_clock.h"
#include "conf_board.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/* Main menu is being displayed. */
#define STATE_MENU				0
/* Time is being edited. */
#define STATE_SET_TIME			1
/* Date is being edited. */
#define STATE_SET_DATE			2
/* Time alarm is being edited. */
#define STATE_SET_TIME_ALARM	3
/* Date alarm is being edited. */
#define STATE_SET_DATE_ALARM	4
/* Wave generating is being edited. */
#define STATE_WAVEFORM			5

/* Maximum size of edited string. */
#define MAX_EDIT_SIZE			10

/* Macro for converting char to digit. */
#define char_to_digit(c) ((c) - '0')

#define STRING_EOL    "\r"
#define STRING_HEADER "-- RTC Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* Current state of application. */
static uint32_t state = STATE_MENU;

/* Edited hour. */
static uint32_t ul_new_hour;
/* Edited minute. */
static uint32_t ul_new_minute;
/* Edited second. */
static uint32_t ul_new_second;

/* Edited year. */
static uint32_t ul_new_year;
/* Edited month. */
static uint32_t ul_new_month;
/* Edited day. */
static uint32_t ul_new_day;
/* Edited day-of-the-week. */
static uint32_t ul_new_week;

/* Indicate if alarm has been triggered and not yet cleared */
static uint32_t alarm_triggered = 0;

/* Time string */
static uint8_t uc_rtc_time[8 + 1] =
		{ '0', '0', ':', '0', '0', ':', '0', '0', '\0' };
/* Date string */
static uint8_t date[10 + 1] =
		{ '0', '0', '/', '0', '0', '/', '0', '0', '0', '0', '\0' };
/* Week string */
static uint8_t pDayNames[7][4] =
		{ "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };

/* Flag for refreshing menu */
static uint32_t menu_shown = 0;

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
 * \brief Get new time. Successful value is put in ul_new_hour, ul_new_minute, ul_new_second.
 */
static uint32_t get_new_time(void)
{
	uint8_t uc_key;
	uint32_t i = 0;

	/* Clear setting variable. */
	ul_new_hour = 0xFFFFFFFF;
	ul_new_minute = 0xFFFFFFFF;
	ul_new_second = 0xFFFFFFFF;

	/* Use uc_rtc_time[] as a format template. */
	while (1) {

		while (uart_read(CONSOLE_UART, &uc_key));

		/* End input */
		if (uc_key == 0x0d || uc_key == 0x0a) {
			puts("\r");
			break;
		}

		/* DEL or BACKSPACE */
		if (uc_key == 0x7f || uc_key == 0x08) {
			if (i > 0) {
				/* End of uc_rtc_time[], then one more back of index */
				if (!uc_rtc_time[i]) {
					--i;
				}

				puts("\b \b");
				--i;

				/* Delimitor ':' for time is uneditable */
				if (!((uc_rtc_time[i]) >= '0' && (uc_rtc_time[i]) <= '9') && i > 0) {
					puts("\b \b");
					--i;
				}
			}
		}

		/* End of uc_rtc_time[], no more input except the above DEL/BS, or enter to end. */
		if (!uc_rtc_time[i]) {
			continue;
		}

		if (!((uc_key) >= '0' && (uc_key) <= '9')) {
			continue;
		}

		while (uart_write(CONSOLE_UART, uc_key));
		uc_rtc_time[i++] = uc_key;

		/* Ignore non-digit position if not the end */
		if (!((uc_rtc_time[i]) >= '0' && (uc_rtc_time[i]) <='9') && i < 8) {
			while (uart_write(CONSOLE_UART, uc_rtc_time[i]));
			++i;
		}
	}

	if (i == 0) {
		return 0;
	}

	if (i != 0 && uc_rtc_time[i] != '\0') {
		/* Failure input */
		return 1;
	}

	ul_new_hour = char_to_digit(uc_rtc_time[0]) * 10 +
			char_to_digit(uc_rtc_time[1]);
	ul_new_minute = char_to_digit(uc_rtc_time[3]) * 10 +
			char_to_digit(uc_rtc_time[4]);
	ul_new_second = char_to_digit(uc_rtc_time[6]) * 10 +
			char_to_digit(uc_rtc_time[7]);

	/* Success input. Verification of data is left to RTC internal Error Checking. */
	return 0;
}

/**
 * \brief Calculate week from year, month, day.
 */
static uint32_t calculate_week(uint32_t ul_year, uint32_t ul_month,
		uint32_t ul_day)
{
	uint32_t ul_week;

	if (ul_month == 1 || ul_month == 2) {
		ul_month += 12;
		--ul_year;
	}

	ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year +
			ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;

	++ul_week;

	return ul_week;
}

/**
 * \brief Get new time. Successful value is put in ul_new_year, ul_new_month, ul_new_day, ul_new_week.
 */
static uint32_t get_new_date(void)
{
	uint8_t uc_key;
	uint32_t i = 0;

	/* Clear setting variable */
	ul_new_year = 0xFFFFFFFF;
	ul_new_month = 0xFFFFFFFF;
	ul_new_day = 0xFFFFFFFF;
	ul_new_week = 0xFFFFFFFF;

	/* Use uc_rtc_time[] as a format template */
	while (1) {

		while (uart_read(CONSOLE_UART, &uc_key));

		/* End input */
		if (uc_key == 0x0d || uc_key == 0x0a) {
			puts("\r");
			break;
		}

		/* DEL or BACKSPACE */
		if (uc_key == 0x7f || uc_key == 0x08) {
			if (i > 0) {
				/* End of date[], then one more back of index */
				if (!date[i]) {
					--i;
				}

				puts("\b \b");
				--i;

				/* Delimitor '/' for date is uneditable */
				if (!((date[i]) >= '0' && (date[i]) <='9') && i > 0) {
					puts("\b \b");
					--i;
				}
			}
		}

		/* End of uc_rtc_time[], no more input except the above DEL/BS, or enter to end. */
		if (!date[i]) {
			continue;
		}

		if (!((uc_key) >= '0' && (uc_key) <= '9')) {
			continue;
		}

		while (uart_write(CONSOLE_UART, uc_key));
		date[i++] = uc_key;

		/* Ignore non-digit position */
		if (!((date[i]) >= '0' && (date[i]) <= '9') && i < 10) {
			while (uart_write(CONSOLE_UART, date[i]));
			++i;
		}
	}

	if (i == 0) {
		return 0;
	}

	if (i != 0 && date[i] != '\0' && i != 6) {
		/* Failure input */
		return 1;
	}

	/* MM-DD-YY */
	ul_new_month = char_to_digit(date[0]) * 10 + char_to_digit(date[1]);
	ul_new_day = char_to_digit(date[3]) * 10 + char_to_digit(date[4]);
	if (i != 6) {
		/* For 'Set Date' option, get the input new year and new week. */
		ul_new_year = char_to_digit(date[6]) * 1000 +
				char_to_digit(date[7]) * 100 +
				char_to_digit(date[8]) * 10 +
				char_to_digit(date[9]);
		ul_new_week = calculate_week(ul_new_year, ul_new_month, ul_new_day);
	}

	/* Success input. Verification of data is left to RTC internal Error Checking. */
	return 0;
}


/**
 * \brief Display the user interface on the terminal.
 */
static void refresh_display(void)
{
	uint32_t ul_hour, ul_minute, ul_second;
	uint32_t ul_year, ul_month, ul_day, ul_week;

	if (state != STATE_MENU) {
		/* Not in menu display mode, in set mode. */
	} else {
		/* Retrieve date and time */
		rtc_get_time(RTC, &ul_hour, &ul_minute, &ul_second);
		rtc_get_date(RTC, &ul_year, &ul_month, &ul_day, &ul_week);

		/* Display */
		if (!menu_shown) {
			puts("\n\rMenu:\n\r"
					"  t - Set time\n\r"
					"  d - Set date\n\r"
					"  i - Set time alarm\n\r"
					"  m - Set date alarm\r");
#if ((SAM3S8) || (SAM3SD8) || (SAM4S))
			puts("  w - Generate Waveform\r");
#endif
			if (alarm_triggered) {
				puts("  c - Clear alarm notification\r");
			}

			printf("\n\r");

			menu_shown = 1;
		}

		/* Update current date and time */
		puts("\r");
		printf(" [Time/Date: %02u:%02u:%02u, %02u/%02u/%04u %s ][Alarm status:%s]", 
			ul_hour, ul_minute, ul_second, ul_month, ul_day, ul_year,
			pDayNames[ul_week-1], alarm_triggered?"Triggered!":"");
	}
}

/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		/* Disable RTC interrupt */
		rtc_disable_interrupt(RTC, RTC_IDR_SECDIS);

		refresh_display();

		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

		rtc_enable_interrupt(RTC, RTC_IER_SECEN);
	} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			/* Disable RTC interrupt */
			rtc_disable_interrupt(RTC, RTC_IDR_ALRDIS);

			alarm_triggered = 1;
			refresh_display();
			/* Show additional menu item for clear notification */
			menu_shown = 0;
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			rtc_enable_interrupt(RTC, RTC_IER_ALREN);
		}
	}
}


/**
 * \brief Application entry point for RTC example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_key;

	/* Initialize the SAM3 system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);
	rtc_enable_interrupt(RTC, RTC_IER_SECEN | RTC_IER_ALREN);

	/* Refresh display once */
	refresh_display();

	/* Handle keypresses */
	while (1) {

		while (uart_read(CONSOLE_UART, &uc_key));

		/* Set time */
		if (uc_key == 't') {
			state = STATE_SET_TIME;

			do {
				puts("\n\r\n\r Set time(hh:mm:ss): ");
			} while (get_new_time());

			/* If valid input, none of the variables for time is 0xff. */
			if (ul_new_hour != 0xFFFFFFFF) {
				if (rtc_set_time(RTC, ul_new_hour, ul_new_minute,
						ul_new_second)) {
					puts("\n\r Time not set, invalid input!\r");
				}
			}

			state = STATE_MENU;
			menu_shown = 0;
			refresh_display();
		}

		/* Set date */
		if (uc_key == 'd') {
			state = STATE_SET_DATE;

			do {
				puts("\n\r\n\r Set date(mm/dd/yyyy): ");
			} while (get_new_date());

			/* If valid input, none of the variables for date is 0xff(ff). */
			if (ul_new_year != 0xFFFFFFFF) {
				if (rtc_set_date(RTC, ul_new_year, ul_new_month,
						ul_new_day, ul_new_week)) {
					puts("\n\r Date not set, invalid input!\r");
				}
			}

			/* Only 'mm/dd' is input. */
			if (ul_new_month != 0xFFFFFFFF && ul_new_year == 0xFFFFFFFF) {
				puts("\n\r Not Set for no year field!\r");
			}

			state = STATE_MENU;
			menu_shown = 0;
			refresh_display();
		}

		/* Set time alarm */
		if (uc_key == 'i') {
			state = STATE_SET_TIME_ALARM;

			rtc_clear_data_alarm(RTC);

			do {
				puts("\n\r\n\r Set time alarm(hh:mm:ss): ");
			} while (get_new_time());

			if (ul_new_hour != 0xFFFFFFFF) {
				if (rtc_set_time_alarm(RTC, 1, ul_new_hour,
						1, ul_new_minute, 1, ul_new_second)) {
					puts("\n\r Time alarm not set, invalid input!\r");
				} else {
					printf("\n\r Time alarm is set at %02u:%02u:%02u!",
						ul_new_hour, ul_new_minute, ul_new_second);
				}
			}
			state = STATE_MENU;
			menu_shown = 0;
			alarm_triggered = 0;
			refresh_display();
		}

		/* Set date alarm */
		if (uc_key == 'm') {
			state = STATE_SET_DATE_ALARM;

			rtc_clear_time_alarm(RTC);

			do {
				puts("\n\r\n\r Set date alarm(mm/dd/): ");
			} while (get_new_date());

			if (ul_new_year == 0xFFFFFFFF && ul_new_month != 0xFFFFFFFF) {
				if (rtc_set_date_alarm(RTC, 1, ul_new_month, 1, ul_new_day)) {
					puts("\n\r Date alarm not set, invalid input!\r");
				} else {
					printf("\n\r Date alarm is set on %02u/%02u!",
							ul_new_month, ul_new_day);
				}
			}
			state = STATE_MENU;
			menu_shown = 0;
			alarm_triggered = 0;
			refresh_display();
		}

#if ((SAM3S8) || (SAM3SD8) || (SAM4S))
		/* Generate Waveform */
		if (uc_key == 'w') {
			state = STATE_WAVEFORM;
			puts("\n\rMenu:\n\r"
					"  0 - No Waveform\n\r"
					"  1 - 1 Hz square wave\n\r"
					"  2 - 32 Hz square wave\n\r"
					"  3 - 64 Hz square wave\n\r"
					"  4 - 512 Hz square wave\n\r"
					"  5 - Toggles when alarm flag rise\n\r"
					"  6 - Copy of the alarm flag\n\r"
					"  7 - Duty cycle programmable pulse\n\r"
					"  8 - Quit\r");

			while (1) {
				while (uart_read(CONSOLE_UART, &uc_key));

				if ((uc_key >= '0') && (uc_key <= '7')) {
					rtc_set_waveform(RTC, 0, char_to_digit(uc_key));
				}

				if (uc_key == '8') {
					state = STATE_MENU;
					menu_shown = 0;
					refresh_display();
					break;
				}
			}
		}
#endif
		/* Clear trigger flag */
		if (uc_key == 'c') {
			alarm_triggered = 0;
			menu_shown = 0;
			refresh_display();
		}

	}

}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

