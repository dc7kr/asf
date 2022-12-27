/**
 * \file
 *
 * \brief DAC Sinewave Example.
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
 * \mainpage DAC Sinewave Example
 *
 * \section Purpose
 *
 * The DAC Sinewave example demonstrates how to use DACC peripheral.
 *
 * \section Requirements
 *
 * This example can be used with sam3n-ek,sam3s-ek,sam3s-ek2,sam3x-ek,sam4s-ek.
 *
 * \section Description
 *
 * This application is aimed to demonstrate how to use DACC in free running
 * mode.
 *
 * The example allows to configure the frequency and amplitude of output
 * sinewave. The frequency could be set from 200Hz to 3KHz, and the peak
 * amplitude could be set from 100 to 1023/4095 in regard to 10/12bit
 * resolution.
 *
 * The example can also generate a full amplitude square wave for reference.
 *
 * The output could be monitored by connecting the DAC channel that is used to
 * one channel of an oscilloscope.
 *
 * \section Usage
 *
 * -# Build the program and download it into the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on the solutions that users choose.
 * -# On the computer, open and configure a terminal application
 *    (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
 *     -- DAC Sinewave Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -- Menu Choices for this example--
 *     -- 0: Set frequency(200Hz-3kHz).--
 *     -- 1: Set amplitude(100-2047).--
 *     -- i: Display present frequency and amplitude.--
 *     -- m: Display this menu.--
 *    \endcode
 * -# Input command according to the menu.
 *
 */

#include "conf_board.h"
#include "conf_clock.h"
#include "conf_dacc_sinewave_example.h"

#include "compiler.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "pmc.h"
#include "dacc.h"
#include "sysclk.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

//! Analog control value
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) \
						  | DACC_ACR_IBCTLCH1(0x02) \
						  | DACC_ACR_IBCTLDACCORE(0x01))

//! The maximal sine wave sample data (no sign)
#define MAX_DIGITAL   (0x7ff)
//! The maximal (peak-peak) amplitude value
#define MAX_AMPLITUDE (DACC_MAX_DATA)
//! The minimal (peak-peak) amplitude value
#define MIN_AMPLITUDE (100)

/** SAMPLES per cycle */
#define SAMPLES (100)

/** Default frequency */
#define DEFAULT_FREQUENCY 1000
/** Min frequency */
#define MIN_FREQUENCY   200
/** Max frequency */
#define MAX_FREQUENCY   3000

/** Invalid value */
#define VAL_INVALID     0xFFFFFFFF

#define STRING_EOL    "\r"
#define STRING_HEADER "-- DAC Sinewave Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/*! Convert wave data to DACC value
 *  Put the sinewave to an offset of max_amplitude/2.
 *  \param wave          Waveform data
 *  \param amplitude     Amplitude value
 *  \param max_digital   Maximal digital value of input data (no sign)
 *  \param max_amplitude Maximal amplitude value
 */
#define wave_to_dacc(wave, amplitude, max_digital, max_amplitude) \
	(((int)(wave)*(amplitude)/(max_digital)) + (max_amplitude/2))

/** Current index_sample */
uint32_t index_sample = 0;
/** Frequency */
uint32_t ul_frequency = 0;
/** Amplitude */
int32_t sl_amplitude = 0;

/** Waveform selector */
uint8_t uc_wave_sel = 0;

/** 100 points of sinewave samples, amplitude is MAX_DIGITAL*2 */
const int16_t sine_data[SAMPLES] = {
	0x0,   0x080, 0x100, 0x17f, 0x1fd, 0x278, 0x2f1, 0x367, 0x3da, 0x449,
	0x4b3, 0x519, 0x579, 0x5d4, 0x629, 0x678, 0x6c0, 0x702, 0x73c, 0x76f,
	0x79b, 0x7bf, 0x7db, 0x7ef, 0x7fb, 0x7ff, 0x7fb, 0x7ef, 0x7db, 0x7bf,
	0x79b, 0x76f, 0x73c, 0x702, 0x6c0, 0x678, 0x629, 0x5d4, 0x579, 0x519,
	0x4b3, 0x449, 0x3da, 0x367, 0x2f1, 0x278, 0x1fd, 0x17f, 0x100, 0x080,

	-0x0,   -0x080, -0x100, -0x17f, -0x1fd, -0x278, -0x2f1, -0x367, -0x3da, -0x449,
	-0x4b3, -0x519, -0x579, -0x5d4, -0x629, -0x678, -0x6c0, -0x702, -0x73c, -0x76f,
	-0x79b, -0x7bf, -0x7db, -0x7ef, -0x7fb, -0x7ff, -0x7fb, -0x7ef, -0x7db, -0x7bf,
	-0x79b, -0x76f, -0x73c, -0x702, -0x6c0, -0x678, -0x629, -0x5d4, -0x579, -0x519,
	-0x4b3, -0x449, -0x3da, -0x367, -0x2f1, -0x278, -0x1fd, -0x17f, -0x100, -0x080
};

/**
 * \brief Configure console.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
		{ sysclk_get_cpu_hz(), 115200, UART_MR_PAR_NO };

	/* Configure PIO */
	pio_configure(PINS_UART_PIO, PINS_UART_TYPE,
		      PINS_UART_MASK, PINS_UART_ATTR);

	/* Configure PMC */
	pmc_enable_periph_clk(CONSOLE_UART_ID);

	/* Configure UART */
	uart_init(CONSOLE_UART, &uart_console_settings);
}

/**
 * \brief Get input from user, and the biggest 4-digit decimal number is allowed.
 *
 * \param ul_lower_limit The lower limit of input
 * \param ul_upper_limit The upper limit of input
 */
static uint32_t get_input_value(uint32_t ul_lower_limit,
	uint32_t ul_upper_limit)
{
	uint32_t i = 0, length = 0, value = 0;
	uint8_t uc_key, str_temp[5] = { 0 };

	while (1) {

		while (uart_read(CONSOLE_UART, &uc_key)) {
		}

		if (uc_key == '\n' || uc_key == '\r') {
			puts("\r");
			break;
		}

		if ('0' <= uc_key && '9' >= uc_key) {
			printf("%c", uc_key);
			str_temp[i++] = uc_key;

			if (i >= 4) {
				break;
			}
		}
	}

	str_temp[i] = '\0';
	/* Input string length */
	length = i;
	value = 0;

	/* Convert string to integer */
	for (i = 0; i < 4; i++) {
		if (str_temp[i] != '0') {
			switch (length - i - 1) {
			case 0:
				value += (str_temp[i] - '0');
				break;

			case 1:
				value += (str_temp[i] - '0') * 10;
				break;

			case 2:
				value += (str_temp[i] - '0') * 100;
				break;

			case 3:
				value += (str_temp[i] - '0') * 1000;
				break;
			}
		}
	}

	if (value > ul_upper_limit || value < ul_lower_limit) {
		puts("\n\r-F- Input value is invalid!");
		return VAL_INVALID;
	}

	return value;
}

/**
 * \brief Display main menu.
 */
static void display_menu(void)
{
	puts("======== Menu Choices for this example ========\r");
	printf("-- 0: Set frequency(%dHz-%dkHz).\n\r",
		MIN_FREQUENCY, MAX_FREQUENCY / 1000);
	printf("-- 1: Set amplitude(%d-%d).\n\r", MIN_AMPLITUDE, MAX_AMPLITUDE);
	puts("-- i: Display present frequency and amplitude.\n\r"
			"-- w: Switch to full amplitude square wave or back.\n\r"
			"-- m: Display this menu.\n\r"
			"------------ Current configuration ------------\r");
	printf("-- DACC channel:\t%d\n\r", DACC_CHANNEL);
	printf("-- Amplitude   :\t%d\n\r", sl_amplitude);
	printf("-- Frequency   :\t%u\n\r", ul_frequency);
	printf("-- Wave        :\t%s\n\r", uc_wave_sel ? "SQUARE" : "SINE");
	puts("===============================================\r");
}

/**
 * \brief SysTick IRQ handler.
 */
void SysTick_Handler(void)
{
	uint32_t status;
	uint32_t dac_val;

	status = dacc_get_interrupt_status(DACC_BASE);

	/* If ready for new data */
	if ((status & DACC_ISR_TXRDY) == DACC_ISR_TXRDY) {
		index_sample++;
		if (index_sample >= SAMPLES) {
			index_sample = 0;
		}
		dac_val = uc_wave_sel ?
				((index_sample > SAMPLES / 2) ? 0 : MAX_AMPLITUDE)
				: wave_to_dacc(sine_data[index_sample],
					 sl_amplitude,
					 MAX_DIGITAL * 2, MAX_AMPLITUDE);

		dacc_write_conversion_data(DACC_BASE, dac_val);
	}
}

/**
 *  \brief DAC Sinewave application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_key;
	uint32_t ul_freq, ul_amp;

	/* Initialize the system */
	sysclk_init();
	board_init();

	/* Initialize debug console */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Enable clock for DACC */
	pmc_enable_periph_clk(DACC_ID);
	/* Reset DACC registers */
	dacc_reset(DACC_BASE);
	/* Half word transfer mode */
	dacc_set_transfer_mode(DACC_BASE, 0);

	/* Initialize timing, amplitude and frequency */
#if (SAM3N)
	/* Timing:
	 * startup                - 0x10 (17 clocks)
	 * internal trigger clock - 0x60 (96 clocks)
	 */
	dacc_set_timing(DACC_BASE, 0x10, 0x60);
	/* Enable DAC */
	dacc_enable(DACC_BASE);
#else
	/* Power save:
	 * sleep mode  - 0 (disabled)
	 * fast wakeup - 0 (disabled)
	 */
	dacc_set_power_save(DACC_BASE, 0, 0);
	/* Timing:
	 * refresh        - 0x08 (1024*8 dacc clocks)
	 * max speed mode -    0 (disabled)
	 * startup time   - 0x10 (1024 dacc clocks)
	 */
	dacc_set_timing(DACC_BASE, 0x08, 0, 0x10);
	/* Disable TAG and select output channel DACC_CHANNEL */
	dacc_set_channel_selection(DACC_BASE, DACC_CHANNEL);
	/* Enable output channel DACC_CHANNEL */
	dacc_enable_channel(DACC_BASE, DACC_CHANNEL);
	/* Set up analog current */
	dacc_set_analog_control(DACC_BASE, DACC_ANALOG_CONTROL);
#endif /* (SAM3N) */

	sl_amplitude = MAX_AMPLITUDE / 2;
	ul_frequency = DEFAULT_FREQUENCY;

	SysTick_Config(sysclk_get_cpu_hz() / (ul_frequency * SAMPLES));

	/* Main menu */
	display_menu();

	while (1) {
		while (uart_read(CONSOLE_UART, &uc_key)) {
		}

		switch (uc_key) {
		case '0':
			puts("Frequency:");
			ul_freq = get_input_value(MIN_FREQUENCY, MAX_FREQUENCY);
			puts("\r");

			if (ul_freq != VAL_INVALID) {
				printf("Set frequency to:%uHz\n\r", ul_freq);
				SysTick_Config(sysclk_get_cpu_hz() / (ul_freq*SAMPLES));
				ul_frequency = ul_freq;
			}
			break;

		case '1':
			puts("Amplitude:");
			ul_amp = get_input_value(MIN_AMPLITUDE, MAX_AMPLITUDE);
			puts("\r");
			if (ul_amp != VAL_INVALID) {
				printf("Set amplitude to %u \n\r", ul_amp);
				sl_amplitude = ul_amp;
			}
			break;

		case 'i':
		case 'I':
			printf("-I- Frequency:%u Hz Amplitude:%d\n\r",
				ul_frequency, sl_amplitude);
			break;

		case 'w':
		case 'W':
			printf("-I- Switch wave to : %s\n\r", uc_wave_sel ?
				"SINE" : "Full Amplitude SQUAQE");
			uc_wave_sel = (uc_wave_sel + 1) & 1;
			break;

		case 'm':
		case 'M':
			display_menu();
			break;
		}
		puts("Press \'m\' or \'M\' to display the main menu again!!\r");
	}
}
