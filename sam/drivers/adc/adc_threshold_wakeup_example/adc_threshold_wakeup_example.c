/**
 * \file
 *
 * \brief ADC12 threshold wakeup example for SAM.
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
 *  \mainpage ADC12 Threshold Wakeup Example
 *
 *  \section Purpose
 *
 *  The adc12_threshold_wakeup example demonstrates how to use ADC with
 *  threshold wakeup.
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3-EK. To enable full scale measurement
 *  of the potentiometer by default configuration, close jumper JP18 on 1 and 2.
 * 
 *
 *  \section Description
 *  This example uses TIOA0 as external trigger instead of software trigger for
 *  ADC conversion. The TIOA0 is a 1ms period square wave. The rising edge
 *  during each period would trigger the ADC to start a conversion on the given 
 *  channel which is connected to the potentiometer. This example shows a menu as 
 *  below upon running:
 *  \code
 *  -- Menu Choices for this example--
 *  -- 0: Display voltage on potentiometer.--
 *  -- 1: Display thresholds.--
 *  -- 2: Modify low threshold.--
 *  -- 3: Modify high threshold.--
 *  -- 4: Choose comparison mode.--
 *  -- i: Display ADC information.--
 *  -- m: Display this main menu.--
 *  -- c: Set Auto Calibration Mode. --
 *  -- s: Enter sleep mode.--
 *  \endcode
 *  With the user interface, comparison window and mode could be set. The ADC
 *  supports 4 kinds of comparison events as follows:
 *
 *  - Lower than the low threshold.
 *  - Higher than the high threshold.
 *  - In the comparison window.
 *  - Out of the comparison window.
 *
 *  If the target gets an 'S' or 's' from user's input, the core fall in sleep
 *  by __WFI. Tuning the potentiometer to enable the ADC input fall into the
 *  comparison window and then generate a trigger event. The comparison event
 *  will wake the system up.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or the
 *     <a href="http://www.iar.com/website1/1.0.1.0/78/1/">
 *     IAR EWARM User and reference guides</a>,
 *     depending on the solutions that users choose.
 *  -# On the computer, open and configure a terminal application
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and the chip used):
 *     \code
 *      -- ADC12 Threshold Wakeup Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      -- 0: Display voltage on potentiometer.--
 *      -- 1: Display thresholds.--
 *      -- 2: Modify low threshold.--
 *      -- 3: Modify high threshold.--
 *      -- 4: Choose comparison mode.--
 *      -- i: Display ADC information.--
 *      -- m: Display this main menu.--
 *      -- c: Set Auto Calibration Mode. --
 *      -- s: Enter sleep mode.--
 *     \endcode
 *  -# Input the command according to the menu.
 *
 */

#include "asf.h"
#include "conf_board.h"

/** ADC channel for potentiometer */
#if SAM3S || SAM3N || SAM4S
#define ADC_CHANNEL_POTENTIOMETER  ADC_CHANNEL_5
#elif SAM3XA
#define ADC_CHANNEL_POTENTIOMETER    ADC_CHANNEL_1
#endif
/** ADC clock */
#define BOARD_ADC_FREQ (6000000)

/** Reference voltage for ADC,in mv.*/
#define VOLT_REF   (3300)

/** The maximal digital value*/
#if SAM3S || SAM3XA || SAM4S
/** The maximal digital value */
#define MAX_DIGITAL     (4095)
#elif SAM3N
#define MAX_DIGITAL     (1023)
#endif

#define STRING_EOL    "\r"
#define STRING_HEADER "-- ADC12 Threshold Wakeup Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL
#define MENU_HEADER "-- Menu Choices for this example--\n\r" \
		"-- 0: Display voltage on potentiometer.--\n\r" \
		"-- 1: Display thresholds.--\n\r" \
		"-- 2: Modify low threshold.--\n\r" \
		"-- 3: Modify high threshold.--\n\r" \
		"-- 4: Choose comparison mode.--\n\r" \
		"-- i: Display ADC information.--\n\r" \
		"-- m: Display this main menu.--\n\r" \
		"-- s: Enter sleep mode.--\n\r"

/** Low threshold*/
static uint16_t gs_us_low_threshold = 0;
/** High threshold*/
static uint16_t gs_us_high_threshold = 0;

void ADC_Handler(void)
{
	uint32_t ul_mode;
	uint16_t us_adc;

	/* Disable Compare Interrupt. */
	adc_disable_interrupt(ADC, ADC_IDR_COMPE);

	if ((adc_get_status(ADC).isr_status & ADC_ISR_COMPE) == ADC_ISR_COMPE) {
		ul_mode = adc_get_comparison_mode(ADC);
		us_adc = adc_get_channel_value(ADC, ADC_CHANNEL_POTENTIOMETER);

		switch (ul_mode) {
		case 0:
			printf("-ISR-:Potentiometer voltage %d mv is below the low " 
				"threshold:%d mv!\n\r", us_adc * VOLT_REF / MAX_DIGITAL, 
				gs_us_low_threshold * VOLT_REF / MAX_DIGITAL);
			break;

		case 1:
			printf("-ISR-:Potentiometer voltage %d mv is above the high " 
				"threshold:%d mv!\n\r", us_adc * VOLT_REF / MAX_DIGITAL, 
				gs_us_high_threshold * VOLT_REF / MAX_DIGITAL);
			break;

		case 2:
			printf("-ISR-:Potentiometer voltage %d mv is in the comparison " 
				"window:%d mv-%d mv!\n\r", us_adc * VOLT_REF / MAX_DIGITAL, 
				gs_us_low_threshold * VOLT_REF / MAX_DIGITAL, gs_us_high_threshold * VOLT_REF / MAX_DIGITAL);
			break;

		case 3:
			printf("-ISR-:Potentiometer voltage %d mv is out of the comparison" 
				" window:%d mv-%d mv!\n\r", us_adc * VOLT_REF / MAX_DIGITAL, 
				gs_us_low_threshold * VOLT_REF / MAX_DIGITAL, gs_us_high_threshold * VOLT_REF / MAX_DIGITAL);
			break;
		}
	}
}

/**
 *  \brief TC0 configuration.
 *
 * Configure Timer Counter 0 (TC0) to generate an interrupt every second. This
 * interrupt will be used to display the number of bytes received on the USART.
 */
static void configure_tc0(void)
{
	/* Enable TC0 peripheral clock. */
	pmc_enable_periph_clk(ID_TC0);
	/* Configure TC for a 1s (= 1Hz) tick. */
	tc_init(TC0, 0, 0x4 | TC_CMR_ACPC_SET | TC_CMR_WAVE
			| TC_CMR_ACPA_CLEAR | (0x2 << 13));
	/* 50% duty, 1s frequency */
	TC0->TC_CHANNEL[0].TC_RA = 16384;
	TC0->TC_CHANNEL[0].TC_RC = 32768;

}

/** Display main menu. */
static void display_menu(void)
{
	puts(MENU_HEADER);
}

/** Display current information,including
 * voltage on potentiometer, thresholds and comparison mode.
 */
static void display_info(void)
{
	uint32_t ul_adc_value = adc_get_channel_value(ADC, ADC_CHANNEL_POTENTIOMETER);

	printf("-I- Thresholds: %d mv - %d mv.\n\r",
			gs_us_low_threshold * VOLT_REF / MAX_DIGITAL,
			gs_us_high_threshold * VOLT_REF / MAX_DIGITAL);
	printf("-I- Voltage on potentiometer: %u mv.\n\r",
			(unsigned int)(ul_adc_value * VOLT_REF / MAX_DIGITAL));
	printf("-I- Comparison mode is %u\n\r.",
			(unsigned int)(ADC->ADC_EMR & ADC_EMR_CMPMODE_Msk));
}

/** Fall asleep by __WFI.
 * Enable interrupt first, and disable it after wake up.
 */
static void enter_asleep(void)
{
	while (1) {
		puts("The device is going to fall in sleep!\r");
		/* Clear status register. */
		adc_get_status(ADC);

		/* Enable Compare Interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_COMPE);

		__WFI();

		/* Every time waked up, break out of the loop. */
		break;
	}
}

/**
 * Get comparison mode.
 */
static uint8_t get_comparison_mode(void)
{
	uint8_t uc_mode = adc_get_comparison_mode(ADC);
	uint8_t uc_char;

	while (1) {
		while (uart_read(CONSOLE_UART, &uc_char));
		switch (uc_char) {
		case 'a':
		case 'A':
			uc_mode = 0x0;
			break;
		case 'b':
		case 'B':
			uc_mode = 0x1;
			break;
		case 'c':
		case 'C':
			uc_mode = 0x2;
			break;
		case 'd':
		case 'D':
			uc_mode = 0x3;
			break;
		case 'q':
		case 'Q':
			break;
		default:
			continue;
		}
		return uc_mode;
	}
}

/**
 * \brief Get voltage from user input (the range
 * is 0~3300 (mv)).
 */
static int16_t get_voltage(void)
{
	uint8_t c_counter = 0, c_char;
	int16_t s_value = 0;
	int8_t c_length = 0;
	int8_t c_str_temp[5] = { 0 };

	while (1) {
		while (uart_read(CONSOLE_UART, &c_char));

		if (c_char == '\n' || c_char == '\r') {
			puts("\r");
			break;
		}

		if ('0' <= c_char && '9' >= c_char) {
			printf("%c", c_char);
			c_str_temp[c_counter++] = c_char;
#if defined (  __GNUC__  )
			fflush(stdout);
#endif

			if (c_counter >= 4) {
				break;
			}
		}
	}

	c_str_temp[c_counter] = '\0';
	/* Input string length. */
	c_length = c_counter;
	s_value = 0;

	/* Convert string to integer. */
	for (c_counter = 0; c_counter < 4; c_counter++) {
		if (c_str_temp[c_counter] != '0') {
			switch (c_length - c_counter - 1) {
			case 0:
				s_value += (c_str_temp[c_counter] - '0');
				break;

			case 1:
				s_value += (c_str_temp[c_counter] - '0') * 10;
				break;

			case 2:
				s_value += (c_str_temp[c_counter] - '0') * 100;
				break;

			case 3:
				s_value += (c_str_temp[c_counter] - '0') * 1000;
				break;
			}
		}
	}

	if (s_value > VOLT_REF) {
		puts("\n\r-F- Too big threshold!\r");
		return -1;
	}

	return s_value;
}

static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ sysclk_get_cpu_hz(), 115200, UART_MR_PAR_NO };

	/* Configure PIO. */
	pio_configure(PINS_UART_PIO, PINS_UART_TYPE, PINS_UART_MASK,
			PINS_UART_ATTR);

	/* Configure PMC. */
	pmc_enable_periph_clk(CONSOLE_UART_ID);

	/* Configure UART. */
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
 *  \brief adc12_threshold_wakeup Application entry point.
 *
 *  Initialize adc to 12-bit, enable channel 5
 *  , hardware trigger with TIOA0 every second
 *  and start conversion.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main(void)
{
	uint8_t c_choice;
	int16_t s_adc_value;
	int16_t s_threshold = 0;

	/* Initialize the SAM3 system. */
	sysclk_init();
	/* Disable watchdog. */
	WDT->WDT_MR = WDT_MR_WDDIS;
	configure_console();
	/* Output example information. */
	puts(STRING_HEADER);

	/* Initialize threshold. */
	gs_us_low_threshold = 0x0;
	gs_us_high_threshold = MAX_DIGITAL;

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);
	/* Initialize ADC. */
	/* startup = 10:    640 periods of ADCClock
	 * for prescal = 4
	 *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
	 *     ADC clock = 6.4 MHz
	 */
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, 10);
#if SAM3S ||  SAM3XA || SAM4S
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
#elif SAM3N
	adc_configure_timing(ADC, 0);
#endif
	adc_check(ADC, sysclk_get_cpu_hz());

	/* Hardware trigger TIOA0. */
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
	/*Enable channels for x,y and z. */
	adc_enable_channel(ADC, ADC_CHANNEL_POTENTIOMETER);

	/* Configure TC. */
	configure_tc0();

	/*Channel 5 has to be compared. */
	adc_set_comparison_channel(ADC, ADC_CHANNEL_POTENTIOMETER);
	/*Compare mode, in the window */
	adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_IN);

	/* Set up Threshold. */
	adc_set_comparison_window(ADC, gs_us_high_threshold, gs_us_low_threshold);

	/* Enable adc interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Disable Compare Interrupt. */
	adc_disable_interrupt(ADC, ADC_IDR_COMPE);

	/* Start TC0 and hardware trigger. */
	tc_start(TC0, 0);

	/*Display main menu. */
	display_menu();

	while (1) {
		while (uart_read(CONSOLE_UART, &c_choice));
		printf("%c\r\n", c_choice);

		switch (c_choice) {
		case '0':
			s_adc_value = adc_get_channel_value(ADC,
					ADC_CHANNEL_POTENTIOMETER);
			printf("-I- Current voltage is %d mv, %d%% of ADVREF\n\r", 
			(s_adc_value * VOLT_REF / MAX_DIGITAL), (s_adc_value * 100 / MAX_DIGITAL));
			break;

		case '1':
			printf("-I- Thresholds are 0x%x(%d%%) and 0x%x(%d%%).\n\r", gs_us_low_threshold, 
				gs_us_low_threshold * 100 / MAX_DIGITAL, gs_us_high_threshold, 
				gs_us_high_threshold * 100 / MAX_DIGITAL);
			break;

		case '2':
			puts("Low threshold is set to(mv):");
			s_threshold = get_voltage();
			puts("\r");

			if (s_threshold >= 0) {
				s_adc_value = s_threshold * MAX_DIGITAL /
						VOLT_REF;
				adc_set_comparison_window(ADC, s_adc_value,
						gs_us_high_threshold);
				/* Renew low threshold. */
				gs_us_low_threshold = s_adc_value;
				printf("Low threshold is set to 0x%x(%d%%)\n\r",
						gs_us_low_threshold,
						gs_us_low_threshold * 100 /
						MAX_DIGITAL);
			}
			break;

		case '3':
			puts("High threshold is set to(mv):");
			s_threshold = get_voltage();
			puts("\r");

			if (s_threshold >= 0) {
				s_adc_value = s_threshold * MAX_DIGITAL /
						VOLT_REF;
				adc_set_comparison_window(ADC, gs_us_low_threshold,
						s_adc_value);

				/* Renew high threshold. */
				gs_us_high_threshold = s_adc_value;
				printf("High threshold is set to 0x%x(%d%%)\n\r", gs_us_high_threshold, 
				gs_us_high_threshold * 100 / MAX_DIGITAL);
			}
			break;
		case '4':
			puts("-a. Below low threshold.\n\r"
					"-b. Above high threshold.\n\r"
					"-c. In the comparison window.\n\r"
					"-d. Out of the comparison window.\n\r"
					"-q. Quit the setting.\r");
			c_choice = get_comparison_mode();
			adc_set_comparison_mode(ADC, c_choice);
			printf("Comparison mode is %c.\n\r", 'a' + c_choice);
			break;

		case 'm':
		case 'M':
			display_menu();
			break;

		case 'i':
		case 'I':
			display_info();
			break;

		case 's':
		case 'S':
			enter_asleep();
			break;
		}
		puts("Press \'m\' or \'M\' to display the main menu again!!\r");
	}
}
