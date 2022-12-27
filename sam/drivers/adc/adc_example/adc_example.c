/**
 * \file
 *
 * \brief Analog-to-Digital Converter (ADC/ADC12B) example for SAM.
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
 *  \mainpage ADC12 Example
 *
 *  \section Purpose
 *
 *  The adc12 example demonstrates how to use ADC peripheral with several modes.
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3-EK. To enable full scale measurement
 *  of the potentiometer, jumper JP18 has to be closed.
 *
 *  We use "USRPB1" button for ADTGR, so please connect ADTRG to
 *  USRPB1 and solder in R9,R10 before running the example.
 *
 *  \section Description
 *
 *  This application shows how to use the ADC using the several modes:
 *  with/without PDC, several types of trigger (Software, ADTRG, Timer, etc.),
 *  gain and offset selection, using sequencer. Users can select different modes
 *  by configuration menu in the terminal.
 *
 *  \note
 *  The sequence number allowed for sam3s8 is 0 up to 7, so the channel 15 used for
 *  TempSensor is not allowed to be set as the sequencer. Two times of capture on
 *  channel 5 are used instead in the user sequence mode demo.
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
 *      -- ADC12 Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      =========================================================
 *      Menu: press a key to change the configuration.
 *      ---------------------------------------------------------
 *      [X] 0: Set ADC trigger mode: Software.
 *      [ ] 1: Set ADC trigger mode: ADTRG.
 *      [ ] 2: Set ADC trigger mode: Timer TIOA.
 *      [ ] 3: Set ADC trigger mode: PWM Event Line.
 *      [ ] 4: Set ADC trigger mode: Free run mode.
 *      [E] T: Enable/Disable to tranfer with PDC.
 *      [D] S: Enable/Disable to use user sequence mode.
 *      [D] P: Enable/Disable ADC power save mode.
 *      [D] G: Enable/Disable to set gain=2 for potentiometer channel.
 *      [D] O: Enable/Disable offset for potentiometer channel.
 *          Q: Quit configuration and start ADC.
 *      =========================================================
 *     \endcode
 *  -# The application will output converted value to hyperterminal and display
 *     a menu for users to set different modes.
 *
 */

#include "board.h"
#include "sysclk.h"
#include "gpio.h"
#include "exceptions.h"
#include "pio.h"
#include "tc.h"
#include "uart.h"
#include "pmc.h"
#if SAM3S || SAM3U || SAM3XA || SAM4S
#include "pwm.h"
#endif
#include "adc.h"
#include "conf_board.h"
#include "pio_handler.h"
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/*
 * We use two ADC channels for this example:
 *    ADC_CHANNEL_5  (potentiometer)
 *    ADC_CHANNEL_15 (temperature sensor)
 */
#define ADC_12B

/** Total number of ADC channels in use */
#if SAM3S || SAM3XA || SAM4S
#define NUM_CHANNELS    (2)
#elif SAM3U|| SAM3N
#define NUM_CHANNELS    (1)
#endif
/** ADC convention done mask. */
#define ADC_DONE_MASK   ( (1<<NUM_CHANNELS) - 1 )

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE     NUM_CHANNELS

/** Reference voltage for ADC, in mv. */
#define VOLT_REF        (3300)

#if SAM3S || SAM3XA || SAM4S
/** The maximal digital value */
#define MAX_DIGITAL     (4095)
#elif SAM3N
#define MAX_DIGITAL     (1023)
#elif SAM3U
#ifdef ADC_12B
#define MAX_DIGITAL     (4095)
#else
#define MAX_DIGITAL     (1023)
#endif
#endif

/** ADC channel for potentiometer */
#if SAM3S || SAM3N || SAM4S
#define ADC_CHANNEL_POTENTIOMETER  ADC_CHANNEL_5
#elif SAM3XA
#define ADC_CHANNEL_POTENTIOMETER  ADC_CHANNEL_1
#elif SAM3U
#define ADC_CHANNEL_POTENTIOMETER  ADC_CHANNEL_3
#endif

#define STRING_EOL    "\r"
#define STRING_HEADER "-- ADC12 Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL
#define MENU_HEADER "\n\r" \
		"===================================================\n\r" \
		"Menu: press a key to change the configuration.\n\r" \
		"---------------------------------------------------------\n\r"

/*----------------------------------------------------------------------------
 *        Local types
 *----------------------------------------------------------------------------*/

/** ADC test mode structure */
struct {
	uint8_t ucTriggerMode;
	uint8_t ucPdcEn;
	uint8_t ucSequenceEn;
	uint8_t ucGainEn;
	uint8_t ucOffsetEn;
	uint8_t ucPowerSaveEn;
#if  SAM3S8 || SAM3SD8 || SAM4S
	uint8_t ucAutoCalibEn;
#endif
} gAdcTestMode;

/** ADC trigger modes */
enum {
	TRIGGER_MODE_SOFTWARE = 0,
	TRIGGER_MODE_ADTRG,
	TRIGGER_MODE_TIMER,
#if SAM3S || SAM3U || SAM3XA || SAM4S
	TRIGGER_MODE_PWM,
#endif
#if SAM3S || SAM3N || SAM3XA || SAM4S
	TRIGGER_MODE_FREERUN
#endif
} eTriggerMode;

/** ADC sample data */
struct {
	uint8_t ucChNum[NUM_CHANNELS];
	uint16_t wValue[NUM_CHANNELS];
	uint16_t wDone;
} gAdcSampleData;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

#if SAM3S || SAM3XA || SAM3N || SAM4S
/**Channel list for sequence*/
enum adc_channel_num_t ch_list[2] = {
	ADC_TEMPERATURE_SENSOR,
	ADC_CHANNEL_POTENTIOMETER
};
#endif
/** Global timestamp in milliseconds since start of application */
volatile uint32_t dw_ms_ticks = 0;
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Display ADC configuration menu.
 */
static void _DisplayMenu(void)
{
	uint8_t ucChar;

	puts(MENU_HEADER);
	ucChar = (gAdcTestMode.ucTriggerMode ==
			TRIGGER_MODE_SOFTWARE) ? 'X' : ' ';
	printf("[%c] 0: Set ADC trigger mode: Software.\n\r", ucChar);
	ucChar = (gAdcTestMode.ucTriggerMode == TRIGGER_MODE_ADTRG) ? 'X' : ' ';
	printf("[%c] 1: Set ADC trigger mode: ADTRG.\n\r", ucChar);
	ucChar = (gAdcTestMode.ucTriggerMode == TRIGGER_MODE_TIMER) ? 'X' : ' ';
	printf("[%c] 2: Set ADC trigger mode: Timer TIOA.\n\r", ucChar);
#if SAM3S || SAM3U || SAM3XA || SAM4S
	ucChar = (gAdcTestMode.ucTriggerMode == TRIGGER_MODE_PWM) ? 'X' : ' ';
	printf("[%c] 3: Set ADC trigger mode: PWM Event Line.\n\r", ucChar);
#endif
#if SAM3S || SAM3N || SAM3XA || SAM4S
	ucChar = (gAdcTestMode.ucTriggerMode ==
			TRIGGER_MODE_FREERUN) ? 'X' : ' ';
	printf("[%c] 4: Set ADC trigger mode: Free run mode.\n\r", ucChar);
#endif
	ucChar = (gAdcTestMode.ucPdcEn) ? 'E' : 'D';
	printf("[%c] T: Enable/Disable to tranfer with PDC.\n\r", ucChar);
#if SAM3S || SAM3N || SAM3XA || SAM4S
	ucChar = (gAdcTestMode.ucSequenceEn) ? 'E' : 'D';
	printf("[%c] S: Enable/Disable to use user sequence mode.\n\r", ucChar);
#endif
	ucChar = (gAdcTestMode.ucPowerSaveEn) ? 'E' : 'D';
	printf("[%c] P: Enable/Disable ADC power save mode.\n\r", ucChar);
#if SAM3S || SAM3U || SAM3XA || SAM4S
	ucChar = (gAdcTestMode.ucGainEn) ? 'E' : 'D';
	printf("[%c] G: Enable/Disable to set gain=2 for potentiometer channel.\n\r",
		ucChar);
	ucChar = (gAdcTestMode.ucOffsetEn) ? 'E' : 'D';
	printf("[%c] O: Enable/Disable offset for potentiometer channel.\n\r",
			ucChar);
#endif
#if  SAM3S8 || SAM3SD8 || SAM4S
	ucChar = (gAdcTestMode.ucAutoCalibEn) ? 'E' : 'D';
	printf("[%c] C: Enable Auto Calibration Mode.\n\r", ucChar);
#endif
	puts("    Q: Quit configuration and start ADC.\r");
	puts("=========================================================\r");
}

/**
 * \brief Set ADC test mode.
 */
static void _SetAdcTestMode(void)
{
	uint8_t ucKey;
	uint8_t ucDone = 0;

	while (!ucDone) {
		while (uart_read(CONSOLE_UART, &ucKey));

		switch (ucKey) {
		case '0':
			gAdcTestMode.ucTriggerMode = TRIGGER_MODE_SOFTWARE;
			break;

		case '1':
			gAdcTestMode.ucTriggerMode = TRIGGER_MODE_ADTRG;
			break;

		case '2':
			gAdcTestMode.ucTriggerMode = TRIGGER_MODE_TIMER;
			break;
#if SAM3S || SAM3U || SAM3XA || SAM4S
		case '3':
			gAdcTestMode.ucTriggerMode = TRIGGER_MODE_PWM;
			break;
#endif
#if SAM3S || SAM3N || SAM3XA || SAM4S
		case '4':
			gAdcTestMode.ucTriggerMode = TRIGGER_MODE_FREERUN;
			break;
#endif
		case 't':
		case 'T':
			if (gAdcTestMode.ucPdcEn) {
				gAdcTestMode.ucPdcEn = 0;
			} else {
				gAdcTestMode.ucPdcEn = 1;
			}
			break;
#if SAM3S || SAM3N || SAM3XA || SAM4S
		case 's':
		case 'S':
			if (gAdcTestMode.ucSequenceEn) {
				gAdcTestMode.ucSequenceEn = 0;
			} else {
				gAdcTestMode.ucSequenceEn = 1;
			}
			break;
#endif

		case 'p':
		case 'P':
			if (gAdcTestMode.ucPowerSaveEn) {
				gAdcTestMode.ucPowerSaveEn = 0;
			} else {
				gAdcTestMode.ucPowerSaveEn = 1;
			}
			break;

#if SAM3S || SAM3U || SAM3XA || SAM4S
		case 'g':
		case 'G':
			if (gAdcTestMode.ucGainEn) {
				gAdcTestMode.ucGainEn = 0;
			} else {
				gAdcTestMode.ucGainEn = 1;
			}
			break;

		case 'o':
		case 'O':
			if (gAdcTestMode.ucOffsetEn) {
				gAdcTestMode.ucOffsetEn = 0;
			} else {
				gAdcTestMode.ucOffsetEn = 1;
			}
			break;
#endif
#if  SAM3S8 || SAM3SD8 || SAM4S
		case 'c':
		case 'C':
			if (gAdcTestMode.ucAutoCalibEn) {
				gAdcTestMode.ucAutoCalibEn = 0;
			} else {
				gAdcTestMode.ucAutoCalibEn = 1;
			}
			break;
#endif
		case 'q':
		case 'Q':
			ucDone = 1;
			break;

		default:
			break;
		}

		_DisplayMenu();
	}
}

/**
 * \brief Find the best MCK divisor.
 *
 * Find the best MCK divisor given the timer frequency and MCK. The result
 * is guaranteed to satisfy the following equation:
 * \code
 *   (MCK / (DIV * 65536)) <= freq <= (MCK / DIV)
 * \endcode
 * with DIV being the highest possible value.
 *
 * \param dwFreq  Desired timer frequency.
 * \param dwMCk  Master clock frequency.
 * \param dwDiv  Divisor value.
 * \param dwTcClks  TCCLKS field value for divisor.
 * \param dwBoardMCK  Board clock frequency.
 *
 * \return 1 if a proper divisor has been found, otherwise 0.
 */
static uint32_t TC_FindMckDivisor(uint32_t dwFreq, uint32_t dwMCk,
		uint32_t * dwDiv, uint32_t * dwTcClks, uint32_t dwBoardMCK)
{
	const uint32_t adwDivisors[5] = { 2, 8, 32, 128, dwBoardMCK / 32768 };

	uint32_t dwIndex = 0;

	/*  Satisfy lower bound. */
	while (dwFreq < ((dwMCk / adwDivisors[dwIndex]) / 65536)) {
		dwIndex++;

		/*  If no divisor can be found, return 0. */
		if (dwIndex == (sizeof(adwDivisors) / sizeof(adwDivisors[0]))) {
			return 0;
		}
	}

	/*  Try to maximize DIV while satisfying upper bound. */
	while (dwIndex < 4) {

		if (dwFreq > (dwMCk / adwDivisors[dwIndex + 1])) {
			break;
		}
		dwIndex++;
	}

	/*  Store results. */
	if (dwDiv) {
		*dwDiv = adwDivisors[dwIndex];
	}
	if (dwTcClks) {
		*dwTcClks = dwIndex;
	}

	return 1;
}

/**
 * \brief Configure to trigger ADC by TIOA output of timer.
 */
static void _ConfigureTimerTrigger(void)
{
	uint32_t dwDiv = 0;
	uint32_t dwTcClks = 0;

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_TC0);

	/* TIOA configuration */
	gpio_configure_pin(PIN_TC0_TIOA0, PIN_TC0_TIOA0_FLAGS);

	/* Configure TC for a 1Hz frequency and trigger on RC compare. */
	TC_FindMckDivisor(1, BOARD_MCK, &dwDiv, &dwTcClks, BOARD_MCK);
	tc_init(TC0, 0, dwTcClks | TC_CMR_CPCTRG | TC_CMR_WAVE |
			TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
	TC0->TC_CHANNEL[0].TC_RA = (BOARD_MCK / dwDiv) / 2;
	TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / dwDiv) / 1;

	/* Start the Timer. */
	tc_start(TC0, 0);
	/* Set TIOA0 trigger. */
#if SAM3S || SAM3N || SAM3XA || SAM4S
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
#elif SAM3U
#ifdef ADC_12B
	adc12b_configure_trigger(ADC12B, ADC_TRIG_TIO_CH_0);
#else
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0);
#endif
#endif
}

#if SAM3S || SAM3U || SAM3XA || SAM4S
/**
 * \brief Configure to trigger ADC by PWM Event Line.
 */
static void _ConfigurePwmTrigger(void)
{
	/* PWM frequency in Hz. */
#define PWM_FREQUENCY               2
	/* Maximum duty cycle value. */
#define MAX_DUTY_CYCLE              1000

	/* Enable PWMC peripheral clock. */
	pmc_enable_periph_clk(ID_PWM);

	gpio_configure_pin(PIN_PWMC_PWMH0_TRIG, PIN_PWMC_PWMH0_TRIG_FLAG);

	/* Set clock A to run at PWM_FREQUENCY * MAX_DUTY_CYCLE (clock B is not used). */
	pwm_clock_t pwm_clock_setting = {
		.dw_clka = PWM_FREQUENCY * MAX_DUTY_CYCLE,
		.dw_clkb = 0,
		.dw_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &pwm_clock_setting);

	/* Configure PWMC for channel 0 (left-aligned). */
		pwm_channel_t pwm_trigger_channel = {
			.channel = PWM_CHANNEL_0,
			.alignment = PWM_ALIGN_LEFT,
			.polarity = PWM_LOW,
		.dw_prescaler = PWM_CMR_CPRE_CLKA,
		.dw_period = MAX_DUTY_CYCLE,
		.dw_duty = MAX_DUTY_CYCLE / 2
	};
	pwm_channel_init(PWM, &pwm_trigger_channel);

	pwm_cmp_t pwm_comparison_setting = {
		.unit = PWM_CMP_UNIT_0,
		.b_enable = true,
		.dw_value = MAX_DUTY_CYCLE / 2,
		.b_pulse_on_line_0 = true
	};
	pwm_cmp_init(PWM, &pwm_comparison_setting);


	/* Enable PWM channel 0. */
	pwm_channel_enable(PWM, 0);
	/* Set PWM Event Line 0 trigger. */
#if SAM3S || SAM3XA || SAM4S
	adc_configure_trigger(ADC, ADC_TRIG_PWM_EVENT_LINE_0, 0);
#elif SAM3U
#ifdef ADC_12B
	adc12b_configure_trigger(ADC12B, ADC_TRIG_PWM_EVENT_LINE_0);
#else
	adc_configure_trigger(ADC, ADC_TRIG_PWM_EVENT_LINE_0);
#endif
#endif
}
#endif
/**
 * \brief Read converted data through PDC channel.
 *
 * \param pADC The pointer of adc peripheral.
 * \param pwBuffer The destination buffer.
 * \param dwSize The size of the buffer.
 */
#if SAM3S || SAM3N || SAM3XA || SAM4S
static uint32_t ADC_ReadBuffer(Adc * pADC, uint16_t * pwBuffer, uint32_t dwSize)
{
	/* Check if the first PDC bank is free. */
	if ((pADC->ADC_RCR == 0) && (pADC->ADC_RNCR == 0)) {
		pADC->ADC_RPR = (uint32_t) pwBuffer;
		pADC->ADC_RCR = dwSize;
		pADC->ADC_PTCR = ADC_PTCR_RXTEN;

		return 1;
	} else {	/* Check if the second PDC bank is free. */
		if (pADC->ADC_RNCR == 0) {
			pADC->ADC_RNPR = (uint32_t) pwBuffer;
			pADC->ADC_RNCR = dwSize;

			return 1;
		} else {
			return 0;
		}
	}
}
#elif SAM3U
#ifdef ADC_12B
static uint32_t ADC12b_ReadBuffer(Adc12b * pADC, uint16_t * pwBuffer,
		uint32_t dwSize)
{
	/* Check if the first PDC bank is free. */
	if ((pADC->ADC12B_RCR == 0) && (pADC->ADC12B_RNCR == 0)) {
		pADC->ADC12B_RPR = (uint32_t) pwBuffer;
		pADC->ADC12B_RCR = dwSize;
		pADC->ADC12B_PTCR = ADC12B_PTCR_RXTEN;

		return 1;
	} else {	/* Check if the second PDC bank is free. */
		if (pADC->ADC12B_RNCR == 0) {
			pADC->ADC12B_RNPR = (uint32_t) pwBuffer;
			pADC->ADC12B_RNCR = dwSize;

			return 1;
		} else {
			return 0;
		}
	}
}
#else
static uint32_t ADC_ReadBuffer(Adc * pADC, uint16_t * pwBuffer, uint32_t dwSize)
{
	/* Check if the first PDC bank is free. */
	if ((pADC->ADC_RCR == 0) && (pADC->ADC_RNCR == 0)) {
		pADC->ADC_RPR = (uint32_t) pwBuffer;
		pADC->ADC_RCR = dwSize;
		pADC->ADC_PTCR = ADC_PTCR_RXTEN;

		return 1;
	} else {	/* Check if the second PDC bank is free. */
		if (pADC->ADC_RNCR == 0) {
			pADC->ADC_RNPR = (uint32_t) pwBuffer;
			pADC->ADC_RNCR = dwSize;

			return 1;
		} else {
			return 0;
		}
	}
}
#endif
#endif
/**
 * \brief Start ADC sample.
 * Initialize ADC, set clock and timing, and set ADC to given mode.
 */
static void _StartAdc(void)
{
	uint32_t dw;
	/* Enable peripheral clock. */
#if SAM3S || SAM3N || SAM3XA || SAM4S
	pmc_enable_periph_clk(ID_ADC);
#elif SAM3U
#ifdef ADC_12B
	pmc_enable_periph_clk(ID_ADC12B);
#else
	pmc_enable_periph_clk(ID_ADC);
#endif
#endif

	/* Initialize ADC. */
#if SAM3S || SAM3N || SAM3XA || SAM4S
	adc_init(ADC, BOARD_MCK, 6400000, 8);
#elif SAM3U
#ifdef ADC_12B
	adc12b_init(ADC12B, BOARD_MCK, 6400000, 10, 10);
#else
	adc_init(ADC, BOARD_MCK, 6400000, 10);
#endif
#endif

	memset((void *)&gAdcSampleData, 0, sizeof(gAdcSampleData));

	/*
	 * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	 * For example, MCK = 64MHZ, PRESCAL = 4, then:
	 *     ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	 */
	/* Set ADC clock. */
	/* Formula:
	 *     Startup  Time = startup value / ADCClock
	 *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	 *     Tracking Time = (TRACKTIM + 1) / ADCClock
	 *     Settling Time = settling value / ADCClock
	 * For example, ADC clock = 6MHz (166.7 ns)
	 *     Startup time = 512 / 6MHz = 85.3 us
	 *     Transfer Time = (1 * 2 + 3) / 6MHz = 833.3 ns
	 *     Tracking Time = (0 + 1) / 6MHz = 166.7 ns
	 *     Settling Time = 3 / 6MHz = 500 ns
	 */
	/* Set ADC timing. */
#if SAM3S ||  SAM3XA || SAM4S
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
#elif  SAM3N
	adc_configure_timing(ADC, 0);
#elif SAM3U
#ifdef ADC_12B
	adc12b_configure_timing(ADC12B, 1200);
#else
	adc_configure_timing(ADC, 1200);
#endif
#endif

#if SAM3S || SAM3N || SAM3XA || SAM4S
	/* Enable channel number tag. */
	adc_enable_tag(ADC);
	/* Enable/disable sequencer. */
	if (gAdcTestMode.ucSequenceEn) {
		/* Set user defined channel sequence. */
		adc_configure_sequence(ADC, ch_list, 2);

		/* Enable sequencer. */
		adc_start_sequencer(ADC);

		/* Enable channels. */
		for (dw = 0; dw < 2; dw++) {
			adc_enable_channel(ADC, dw);
		}
		/* Update channel number. */
		gAdcSampleData.ucChNum[0] = ch_list[0];
		gAdcSampleData.ucChNum[1] = ch_list[1];
	} else {
		/* Disable sequencer. */
		adc_stop_sequencer(ADC);

		/* Enable channels. */
		adc_enable_channel(ADC, ADC_CHANNEL_POTENTIOMETER);
#if SAM3S || SAM3XA || SAM4S
		adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);
#endif
		/* Update channel number. */
		gAdcSampleData.ucChNum[0] = ADC_CHANNEL_POTENTIOMETER;
#if SAM3S || SAM3XA || SAM4S
		gAdcSampleData.ucChNum[1] = ADC_TEMPERATURE_SENSOR;
#else
		gAdcSampleData.ucChNum[1] = ADC_CHANNEL_POTENTIOMETER;
#endif
	}
#elif SAM3U
#ifdef ADC_12B
	adc12b_enable_channel(ADC12B, ADC_CHANNEL_POTENTIOMETER);
#else
	adc_enable_channel(ADC, ADC_CHANNEL_POTENTIOMETER);
#endif
	gAdcSampleData.ucChNum[0] = ADC_CHANNEL_POTENTIOMETER;
	gAdcSampleData.ucChNum[1] = ADC_CHANNEL_POTENTIOMETER;
#endif

#if SAM3S ||  SAM3XA || SAM4S
	/* Enable the temperature sensor (CH15). */
	adc_enable_ts(ADC);
#endif
	/* Set gain and offset (only single ended mode used here). */
#if SAM3S || SAM3XA || SAM4S
	adc_disable_anch(ADC);	/* Disable analog change. */
#endif
	if (gAdcTestMode.ucGainEn) {
#if SAM3S || SAM3XA || SAM4S
		adc_enable_anch(ADC);
		/* gain = 2 */
		adc_set_channel_input_gain(ADC, ADC_CHANNEL_POTENTIOMETER, ADC_GAINVALUE_2);	
#elif SAM3U
#ifdef ADC_12B
		adc12b_set_input_gain(ADC12B, ADC_GAINVALUE_2);
#endif
#endif
	} else {
#if SAM3S || SAM3XA || SAM4S
		/* gain = 1 */
		adc_set_channel_input_gain(ADC, ADC_CHANNEL_POTENTIOMETER, ADC_GAINVALUE_0);	
#elif SAM3U
#ifdef ADC_12B
		adc12b_set_input_gain(ADC12B, ADC_GAINVALUE_0);
#endif
#endif
	}

	if (gAdcTestMode.ucOffsetEn) {
#if SAM3S || SAM3XA || SAM4S
		adc_enable_anch(ADC);
		adc_enable_channel_input_offset(ADC, ADC_CHANNEL_POTENTIOMETER);
#elif 	SAM3U
#ifdef ADC_12B
		adc12b_enable_input_offset(ADC12B);
#endif
#endif
	} else {
#if SAM3S || SAM3XA || SAM4S
		adc_disable_channel_input_offset(ADC, ADC_CHANNEL_POTENTIOMETER);
#elif 	SAM3U
#ifdef ADC_12B
		adc12b_disable_input_offset(ADC12B);
#endif
#endif
	}
	/* Set Auto Calibration Mode. */
#if  SAM3S8 || SAM3SD8 || SAM4S
	if (gAdcTestMode.ucAutoCalibEn) {
		adc_set_calibmode(ADC);
		while (1) {
			if ((adc_get_status(ADC).isr_status & ADC_ISR_EOCAL) ==
					ADC_ISR_EOCAL)
				break;
		}
	}
#endif

#if SAM3S || SAM3N || SAM3XA || SAM4S
	/* Set power save. */
	if (gAdcTestMode.ucPowerSaveEn) {
		adc_configure_power_save(ADC, 1, 0);
	} else {
		adc_configure_power_save(ADC, 0, 0);;
	}

	/* Transfer with/without PDC. */
	if (gAdcTestMode.ucPdcEn) {
		ADC_ReadBuffer(ADC, gAdcSampleData.wValue, BUFFER_SIZE);
		/* Enable PDC channel interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_RXBUFF);
	} else {
		/* Enable Data ready interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_DRDY);
	}
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);
#elif SAM3U
#ifdef ADC_12B
	/* Set power save. */
	if (gAdcTestMode.ucPowerSaveEn) {
		adc12b_configure_power_save(ADC12B, 1, 0);
	} else {
		adc12b_configure_power_save(ADC12B, 0, 0);;
	}

	/* Transfer with/without PDC. */
	if (gAdcTestMode.ucPdcEn) {
		ADC12b_ReadBuffer(ADC12B, gAdcSampleData.wValue, BUFFER_SIZE);
		/* Enable PDC channel interrupt. */
		adc12b_enable_interrupt(ADC12B, ADC12B_IER_RXBUFF);
	} else {
		/* Enable Data ready interrupt. */
		adc12b_enable_interrupt(ADC12B, ADC12B_IER_DRDY);

	}
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC12B_IRQn);
#else
	/* Set power save. */
	if (gAdcTestMode.ucPowerSaveEn) {
		adc_configure_power_save(ADC, 1);
	} else {
		adc_configure_power_save(ADC, 0);;
	}

	/* Transfer with/without PDC. */
	if (gAdcTestMode.ucPdcEn) {
		ADC_ReadBuffer(ADC, gAdcSampleData.wValue, BUFFER_SIZE);
		/* Enable PDC channel interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_RXBUFF);
	} else {
		/* Enable Data ready interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_DRDY);

	}
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);
#endif
#endif
	/* Configure trigger mode and start convention. */
	switch (gAdcTestMode.ucTriggerMode) {
	case TRIGGER_MODE_SOFTWARE:
#if SAM3S || SAM3N || SAM3XA || SAM4S
		adc_configure_trigger(ADC, ADC_TRIG_SW, 0);	/* Disable hardware trigger. */
#elif SAM3U
#ifdef ADC_12B
		adc12b_configure_trigger(ADC12B, ADC_TRIG_SW);
#else
		adc_configure_trigger(ADC, ADC_TRIG_SW);
#endif
#endif
		break;

	case TRIGGER_MODE_ADTRG:
#if SAM3S || SAM3N || SAM3XA || SAM4S
		gpio_configure_pin(PINS_ADC_TRIG, PINS_ADC_TRIG_FLAG);
		adc_configure_trigger(ADC, ADC_TRIG_EXT, 0);
#elif SAM3U
#ifdef ADC_12B
		gpio_configure_pin(PINS_ADC12B_TRIG, PINS_ADC12B_TRIG_FLAG);
		adc12b_configure_trigger(ADC12B, ADC_TRIG_EXT);
#else
		gpio_configure_pin(PINS_ADC_TRIG, PINS_ADC_TRIG_FLAG);
		adc_configure_trigger(ADC, ADC_TRIG_EXT);
#endif
#endif
		break;

	case TRIGGER_MODE_TIMER:
		_ConfigureTimerTrigger();
		break;
#if SAM3S || SAM3U || SAM3XA || SAM4S
	case TRIGGER_MODE_PWM:
		_ConfigurePwmTrigger();
		break;
#endif
#if SAM3S || SAM3N || SAM3XA || SAM4S
	case TRIGGER_MODE_FREERUN:
		adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
		break;
#endif
	default:
		break;
	}
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Systick handler.
 */
void SysTick_Handler(void)
{
	dw_ms_ticks++;
}

#if SAM3S || SAM3N || SAM3XA || SAM4S
/**
 * \brief Interrupt handler for the ADC.
 */
void ADC_Handler(void)
{
	uint32_t i;
	uint32_t dwTemp;
	uint8_t ucChNum;

	/* With PDC transfer */
	if (gAdcTestMode.ucPdcEn) {
		if ((adc_get_status(ADC).isr_status & ADC_ISR_RXBUFF) ==
				ADC_ISR_RXBUFF) {
			gAdcSampleData.wDone = ADC_DONE_MASK;
			ADC_ReadBuffer(ADC, gAdcSampleData.wValue, BUFFER_SIZE);
			/* Only keep sample value, and discard channel number. */
			for (i = 0; i < NUM_CHANNELS; i++) {
				gAdcSampleData.wValue[i] &= ADC_LCDR_LDATA_Msk;
			}
		}
	} else {	/* Without PDC transfer */
		if ((adc_get_status(ADC).isr_status & ADC_ISR_DRDY) ==
				ADC_ISR_DRDY) {
			dwTemp = adc_get_latest_value(ADC);
			for (i = 0; i < NUM_CHANNELS; i++) {
				ucChNum = (dwTemp & ADC_LCDR_CHNB_Msk) >>
						ADC_LCDR_CHNB_Pos;
				if (gAdcSampleData.ucChNum[i] == ucChNum) {
					gAdcSampleData.wValue[i] =
							dwTemp &
							ADC_LCDR_LDATA_Msk;
					gAdcSampleData.wDone |= 1 << i;
				}
			}
		}
	}
}
#elif SAM3U
#ifdef ADC_12B
void ADC12B_Handler(void)
{
	uint32_t i;
	uint32_t dwTemp;

	/* With PDC transfer */
	if (gAdcTestMode.ucPdcEn) {
		if ((adc12b_get_status(ADC12B).all_status & ADC12B_SR_RXBUFF) ==
				ADC12B_SR_RXBUFF) {
			gAdcSampleData.wDone = ADC_DONE_MASK;
			ADC12b_ReadBuffer(ADC12B, gAdcSampleData.wValue,
					BUFFER_SIZE);

			/* Only keep sample value, and discard channel number. */
			for (i = 0; i < NUM_CHANNELS; i++) {
				gAdcSampleData.wValue[i] &=
						ADC12B_LCDR_LDATA_Msk;
			}
		}
	} else {	/* Without PDC transfer */
		if ((adc12b_get_status(ADC12B).all_status & ADC12B_SR_DRDY) ==
				ADC12B_SR_DRDY) {
			dwTemp = adc12b_get_latest_value(ADC12B);
			for (i = 0; i < NUM_CHANNELS; i++) {
				gAdcSampleData.wValue[i] =
						dwTemp & ADC12B_LCDR_LDATA_Msk;
				gAdcSampleData.wDone |= 1 << i;
			}
		}
	}
}
#else
void ADC_Handler(void)
{
	uint32_t i;
	uint32_t dwTemp;

	/* With PDC transfer */
	if (gAdcTestMode.ucPdcEn) {
		if ((adc_get_status(ADC).all_status & ADC_SR_RXBUFF) ==
				ADC_SR_RXBUFF) {
			gAdcSampleData.wDone = ADC_DONE_MASK;
			ADC_ReadBuffer(ADC, gAdcSampleData.wValue, BUFFER_SIZE);

			/* Only keep sample value, and discard channel number. */
			for (i = 0; i < NUM_CHANNELS; i++) {
				gAdcSampleData.wValue[i] &= ADC_LCDR_LDATA_Msk;
			}
		}
	} else {
		if ((adc_get_status(ADC).all_status & ADC_SR_DRDY) ==
				ADC_SR_DRDY) {
			dwTemp = adc_get_latest_value(ADC);
			for (i = 0; i < NUM_CHANNELS; i++) {
				gAdcSampleData.wValue[i] =
						dwTemp & ADC_LCDR_LDATA_Msk;
				gAdcSampleData.wDone |= 1 << i;
			}
		}
	}
}
#endif
#endif
/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ BOARD_MCK, 115200, UART_MR_PAR_NO };

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
 *  Wait for the given number of milliseconds (using the dwTimeStamp generated
 *  by the SAM3 microcontrollers' system tick).
 *  \param dw_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t dw_dly_ticks)
{
	uint32_t dw_cur_ticks;

	dw_cur_ticks = dw_ms_ticks;
	while ((dw_ms_ticks - dw_cur_ticks) < dw_dly_ticks);
}

/**
 *  \brief adc12 Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t dw;
	uint8_t key;
	/* Initialize the SAM3 system. */
	sysclk_init();
	board_init();

	WDT->WDT_MR = WDT_MR_WDDIS;

	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	puts("Configure system tick to get 1ms tick period.\r");
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-F- Systick configuration error\r");
		while (1);
	}

	/* Set default ADC test mode. */
	memset((void *)&gAdcTestMode, 0, sizeof(gAdcTestMode));
	gAdcTestMode.ucTriggerMode = TRIGGER_MODE_SOFTWARE;
	gAdcTestMode.ucPdcEn = 1;
	gAdcTestMode.ucSequenceEn = 0;
	gAdcTestMode.ucGainEn = 0;
	gAdcTestMode.ucOffsetEn = 0;

	_DisplayMenu();
	_StartAdc();

	puts("Press any key to display configuration menu.\r");
	while (1) {
		/* ADC software trigger per 1s */
		if (gAdcTestMode.ucTriggerMode == TRIGGER_MODE_SOFTWARE) {
			mdelay(1000);
#if SAM3S || SAM3N || SAM3XA || SAM4S
			adc_start(ADC);
#elif SAM3U
#ifdef ADC_12B
			adc12b_start(ADC12B);
#else
			adc_start(ADC);
#endif
#endif
		}

		/* Check if the user enters a key. */
		if (!uart_read(CONSOLE_UART, &key)) {
#if SAM3S || SAM3N || SAM3XA || SAM4S
			adc_disable_interrupt(ADC, 0xFFFFFFFF);	/* Disable all adc interrupt. */
#elif SAM3U
#ifdef ADC_12B
			adc12b_disable_interrupt(ADC12B, 0xFFFFFFFF);	/* Disable all adc interrupt. */
#else
			adc_disable_interrupt(ADC, 0xFFFFFFFF);	/* Disable all adc interrupt. */
#endif
#endif
			tc_start(TC0, 0);	/* Stop the Timer. */
#if SAM3S || SAM3U || SAM3XA || SAM4S
			pwm_channel_disable(PWM, 0);
#endif
			_DisplayMenu();
			_SetAdcTestMode();
			_StartAdc();
			puts("Press any key to display configuration menu.\r");
		}

		/* Check if ADC sample is done. */
		if (gAdcSampleData.wDone == ADC_DONE_MASK) {
			for (dw = 0; dw < NUM_CHANNELS; dw++) {
				printf("CH%02d: %04d mv.    ",
						(int)gAdcSampleData.ucChNum[dw],
						(int)(gAdcSampleData.
								wValue[dw] *
								VOLT_REF /
								MAX_DIGITAL));
			}
			puts("\r");
			gAdcSampleData.wDone = 0;
		}
	}
}
