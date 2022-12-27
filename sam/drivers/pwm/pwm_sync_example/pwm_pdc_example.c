/**
 * \file
 *
 * \brief PWM PDC example for SAM.
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
 * \mainpage PWM PDC Example
 *
 * \par Purpose
 *
 * This example demonstrates a simple configuration of 2 PWM synchronous 
 * channels to generate variable duty cycle signals. The duty cycle values are 
 * updated automatically by the Peripheral DMA Controller (PDC), which makes 2 
 * on-board LEDs glow repeatedly.
 *
 * \par Usage
 *
 * -# Initialize system clock and pins setting on board
 * -# Initialize PWM clock
 * -# Configure PWM_CHANNEL_0
 * -# Configure PWM_CHANNEL_1
 * -# Configure PDC transfer for PWM duty cycle update
 * -# Enable interrupt of PDC Tx end and PWM_CHANNEL_0
 * -# Update synchronous period, dead time and override output via UART Console
 * -# Restart PDC transfer in ISR
 *
 */

#include "asf.h"
#include "conf_board.h"

/** Baud rate of console UART */
#define CONSOLE_BAUD_RATE  115200

/** PWM frequency in Hz */
#define PWM_FREQUENCY  50
/** PWM period value */
#define PERIOD_VALUE   50
/** Initial duty cycle value */
#define INIT_DUTY_VALUE  0
/** Initial dead time value */
#define INIT_DEAD_TIME   5
/** Maximum synchronous update period */
#define MAX_SYNC_UPDATE_PERIOD  PWM_SCUP_UPR_Msk

/** Duty cycle buffer length for three channels */
#define DUTY_BUFFER_LENGTH      ((PERIOD_VALUE - INIT_DUTY_VALUE + 1) * 3)

#define STRING_EOL    "\r"
#define STRING_HEADER "-- PWM PDC Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Duty cycle buffer for PDC transfer */
uint16_t g_us_duty_buffer[DUTY_BUFFER_LENGTH];

/** PDC transfer packet */
pdc_packet_t g_pdc_tx_packet;

/**
 * \brief Display menu.
 */
static void display_menu(void)
{
	puts("\n\r"
			"===============================================================\n\r"
			"Menu: press a key to change the configuration.\n\r"
			"===============================================================\n\r"
			"  u : Change update period for synchronous channels \n\r"
			"  d : Change dead time of PWM outputs\n\r"
			"  o : Enable/disable output override\n\r"
			"\r");
}

/**
 * \brief Get 2 digit numkey values from terminal input.
 *
 * \return Numkey value.
 */
static uint8_t get_num_key_2_digit(void)
{
	uint32_t ul_numkey;
	uint8_t uc_key1, uc_key2;

	puts("\n\rEnter 2 digits : ");
#if defined (  __GNUC__  )
	fflush(stdout);
#endif
	while (uart_read(CONSOLE_UART, &uc_key1));
	printf("%c", uc_key1);
	while (uart_read(CONSOLE_UART, &uc_key2));
	printf("%c", uc_key2);
	puts("\r");

	ul_numkey = (uc_key1 - '0') * 10 + (uc_key2 - '0');

	return (uint8_t) ul_numkey;
}

/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM_Handler(void)
{
	uint32_t pdc_status = pwm_get_interrupt_status(PWM);

	if ((pdc_status & PWM_PDC_TX_END) == PWM_PDC_TX_END) {
		/* Set up the PDC controller */
		g_pdc_tx_packet.ul_addr = (uint32_t) (&g_us_duty_buffer[0]);
		g_pdc_tx_packet.ul_size = DUTY_BUFFER_LENGTH;

		/* Initialize the PDC transfer */
		pdc_tx_init(PDC_PWM, &g_pdc_tx_packet, 0);

		/* Send the PWM value */
		pdc_enable_transfer(PDC_PWM, PERIPH_PTCR_TXTEN);
	}
}

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ sysclk_get_cpu_hz(), CONSOLE_BAUD_RATE, UART_MR_PAR_NO };

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
 * \brief Application entry point for PWM PDC example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t i;
	uint8_t uc_key;
	int8_t c_numkey;

	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Configure the console uart for debug infomation */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM);

	/* Disable PWM channel 0 and 1 */
	pwm_channel_disable(PWM, PWM_CHANNEL_0 | PWM_CHANNEL_1);

	/* Set PWM clock A as PWM_FREQUENCY * PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);

	/* Initialize PWM channels outputs */
	pwm_output_t channel_output = {
		.b_override_pwmh = false,         /* Disable override PWMH outputs */
		.b_override_pwml = false,         /* Disable override PWML outputs */
		.override_level_pwmh = PWM_HIGH,  /* Set override PWMH output level as HIGH */
		.override_level_pwml = PWM_LOW    /* Set override PWML output level as LOW */
	};

	/* Initialize PWM synchronous channels */
	pwm_channel_t sync_channel = {
		.ul_prescaler = PWM_CMR_CPRE_CLKA, /* Use PWM clock A as source clock */
		.ul_period = PERIOD_VALUE,         /* Period value of output waveform */
		.ul_duty = INIT_DUTY_VALUE,        /* Duty cycle value of output waveform */
		.b_sync_ch = true,                 /* Set it as a synchronous channel */
		.b_deadtime_generator = true,      /* Enable dead-time generator */
		.us_deadtime_pwmh = INIT_DEAD_TIME, /* Dead-time value for PWMH output */
		.us_deadtime_pwml = INIT_DEAD_TIME, /* Dead-time value for PWML output */
		.output_selection.b_override_pwmh = false, /* Disable override PWMH outputs */
		.output_selection.b_override_pwml = false  /* Disable override PWML outputs */
	};

	/* Initialize PWM channel 1 */
	sync_channel.channel = PWM_CHANNEL_1;
	pwm_channel_init(PWM, &sync_channel);

	/* Initialize PWM channel 0 */
	sync_channel.channel = PWM_CHANNEL_0;
	pwm_channel_init(PWM, &sync_channel);

	/*
	 * Initialize PWM synchronous channels
	 * Synchronous Update Mode: Automatic update duty cycle value by the PDC and automatic update of synchronous channels. The update occurs when the Update Period elapses (MODE 2).
	 * Synchronous Update Period = MAX_SYNC_UPDATE_PERIOD.
	 */
	pwm_sync_init(PWM, PWM_SYNC_UPDATE_MODE_2, MAX_SYNC_UPDATE_PERIOD);

	/*
	 * Request PDC transfer as soon as the synchronous update period elapses (comparison unit is ignored).
	 */
	pwm_pdc_set_request_mode(PWM, PWM_PDC_UPDATE_PERIOD_ELAPSED,
			PWM_CMP_UNIT_0);

	/* Configure interrupt for PDC transfer */
	NVIC_DisableIRQ(PWM_IRQn);
	NVIC_ClearPendingIRQ(PWM_IRQn);
	NVIC_SetPriority(PWM_IRQn, 0);
	NVIC_EnableIRQ(PWM_IRQn);
	pwm_pdc_enable_interrupt(PWM, PWM_PDC_TX_END);

	/* Fill duty cycle buffer for channel #0 and #1 */
	/* For PWM channel 0 and 1, duty cycle ranges from MIN_DUTY_CYCLE to MAX_DUTY_CYCLE */
	for (i = 0; i < (DUTY_BUFFER_LENGTH / 3); i++) {
		g_us_duty_buffer[i * 3] = (i + INIT_DUTY_VALUE);
		g_us_duty_buffer[i * 3 + 1] = (i + INIT_DUTY_VALUE);
		g_us_duty_buffer[i * 3 + 2] = (i + INIT_DUTY_VALUE);
	}

	/* Configure the PDC transfer packet and enable PDC transfer */
	g_pdc_tx_packet.ul_addr = (uint32_t) (&(g_us_duty_buffer[0]));
	g_pdc_tx_packet.ul_size = DUTY_BUFFER_LENGTH;
	pdc_tx_init(PDC_PWM, &g_pdc_tx_packet, 0);
	pdc_enable_transfer(PDC_PWM, PERIPH_PTCR_TXTEN);

	/* Enable all synchronous channels by enabling channel 0 */
	pwm_channel_enable(PWM, PWM_CHANNEL_0);

	while (1) {
		display_menu();
		while (uart_read(CONSOLE_UART, &uc_key));

		switch (uc_key) {
		case 'u':
		case 'U':
			printf("Input update period between 0 and %d.\n\r",
					(int)MAX_SYNC_UPDATE_PERIOD);
			c_numkey = get_num_key_2_digit();
			if (c_numkey <= MAX_SYNC_UPDATE_PERIOD) {
				/* Set new synchronous update period value */
				pwm_sync_change_period(PWM, c_numkey);
				printf("Synchronous update period has been changed to %d.\n\r", (int)c_numkey);

			} else {
				printf("Invalid value (Must between 0 and %d).\n\r", (int)MAX_SYNC_UPDATE_PERIOD);
			}
			break;
		case 'd':
		case 'D':
			printf("Input dead time for outputs of channel #0 between %d and %d.\n\r", INIT_DUTY_VALUE, PERIOD_VALUE);
			c_numkey = get_num_key_2_digit();

			if ((c_numkey >= INIT_DUTY_VALUE)
					&& (c_numkey <= PERIOD_VALUE)) {
				/* Set new dead time value for channel 0 */
				pwm_channel_update_dead_time(PWM, &sync_channel,
						c_numkey, c_numkey);
				/* Update all synchronous channels */
				pwm_sync_unlock_update(PWM);
				printf("Dead time has been changed to %d.\n\r", (int)c_numkey);

			} else {
				printf("Invalid value (Must between %d and %d).\n\r", INIT_DUTY_VALUE, PERIOD_VALUE);
			}
			break;
		case 'o':
		case 'O':
			puts("0: Disable override output on channel #0\n\r"
					"1: Enable override output on channel #0\r");
			while (uart_read(CONSOLE_UART, &uc_key));

			if (uc_key == '1') {
				/* Enable override outputs of channel 0 synchronously */
				channel_output.b_override_pwmh = true;
				channel_output.b_override_pwml = true;
				pwm_channel_update_output(PWM, &sync_channel,
						&channel_output, true);
				puts("PWM Channel #0 outputs have been overridden.\r");

			} else if (uc_key == '0') {
				/* Disable override outputs of channel 0 synchronously */
				channel_output.b_override_pwmh = false;
				channel_output.b_override_pwml = false;
				pwm_channel_update_output(PWM, &sync_channel,
						&channel_output, true);
				puts("PWM Channel #0 output override has been disabled.\r");

			} else {
				puts("Unknow input\r");
			}
			break;
		default:
			puts("Unknow input\r");
			break;
		}
	}
}
