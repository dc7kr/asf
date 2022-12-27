/**
 * \file
 *
 * \brief USART Irda example for SAM.
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
 * \mainpage USART IrDA Example
 *
 * \par Purpose
 *
 * This example demonstrates the IrDA (Infrared Data Association) on sam3.
 *
 * \par Requirements
 *
 * This example can be used on all SAM3-EK with external IrDA transceiver component.
 * Connect the board and external component with the following paired
 * pins:
 *  - <b> SAM3-EK  --  IrDA transceiver </b>
 *   - 3V3 -- VCC
 *   - TXD -- TXD
 *   - RXD -- RXD
 *   - CTS -- SD
 *   - GND -- GND
 *
 * \par Description
 *
 * The provided program uses the USART in IrDA mode for transmitting and receiving
 * several octets. This example can be used with two SAM3-EK boards.
 * The program receives or transmits a series of octets according to its state of
 * either receiver or transmitter.
 *
 * \par Note
 * To receive IrDA signals, the following needs to be done:
 * <ul>
 * <li> Disable TX and Enable RX. </li>
 * <li> Configure the TXD pin as PIO and set it as an output at 0 (to avoid LED
 * emission). Disable the internal pull-up (better for power consumption).</li>
 * <li> Receive data. </li>
 * </ul>
 * \par Usage
 *
 * -# Build the program and download it into the two evaluation boards.
 * -# Connect a serial cable to the UART port for each evaluation kit.
 * -# On the computer, open and configure a terminal application (e.g.,
 *    HyperTerminal on Microsoft Windows) with these settings:
 *       - 115200 bauds
 *       - 8 data bits
 *       - No parity
 *       - 1 stop bit
 *       - No flow control
 * -# Start the application.
 * -# The following traces shall appear on the terminal:
 *    \code
 *     -- USART IrDA Example --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Press t to transmit, press r to receive...
 *    \endcode
 * -# Enable one board as transmitter and another as receiver to start the
 *    communication. If succeed, the side of receiver would output the received
 *    data.
 *
 */

#include <stdio.h>
#include <string.h>
#include "sysclk.h"
#include "board.h"
#include "exceptions.h"
#include "usart.h"
#include "uart.h"
#include "pmc.h"
#include "gpio.h"
#include "pio.h"
#include "pdc.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"


/** Buffer size for PDC transfer. */
#define BUFFER_SIZE                  10

/** Communication state. */
#define STATE_TRANSMIT               0
#define STATE_RECEIVE                1
#define STATE_IDLE                   3

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK           0xffffffff

#define STRING_EOL    "\r"
#define STRING_HEADER "--USART IrDA Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** State of BOARD_USART_BASE. */
uint8_t state = STATE_IDLE;

/** Receiving Done status. */
volatile uint32_t recv_done = false;

/** Transmitting Done status. */
volatile uint32_t sent_done = false;

/** Data buffer for receiving. */
uint8_t recv_data[BUFFER_SIZE] = { 0x0 };

/** Data buffer for transmitting. */
uint8_t send_data[BUFFER_SIZE] =
		{ 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89 };

/** Pointer to PDC register base. */
Pdc *p_pdc;

/** PDC data packet. */
pdc_packet_t st_packet;

/**
  * \brief USART IRQ Handler, handling RXBUFF and TXBUFE status.
  */
void USART_Handler(void)
{
	uint32_t dw_status;

	/* Read USART Status. */
	dw_status = usart_get_status(BOARD_USART);

	/* Receiving is done. */
	if ((dw_status & US_CSR_RXBUFF) && (state == STATE_RECEIVE)) {
		recv_done = true;
		usart_disable_interrupt(BOARD_USART, US_IDR_RXBUFF);
	}

	/* Transmitting is done. */
	if ((dw_status & US_CSR_TXBUFE) && (state == STATE_TRANSMIT)) {
		sent_done = true;
		usart_disable_interrupt(BOARD_USART, US_IDR_TXBUFE);
	}
}

/**
 * \brief Disable receiver and Enable transmitter.
 */
static void func_transmitter(void)
{
	/* Configure the TXD pin as peripheral. */
	pio_configure_pin(PIN_USART_TXD_IDX, PIN_USART_TXD_FLAGS);

	/* Disable Receiver. */
	usart_disable_rx(BOARD_USART);

	/* Enable transmitter. */
	usart_enable_tx(BOARD_USART);
}

/**
 * \brief Disable transmitter and Enable receiver.
 */
static void func_receiver(void)
{
	uint32_t ul_temp;

	/* Disable Transmitter. */
	usart_disable_tx(BOARD_USART);

	/* Configure the TXD pin as PIO. */
	pio_configure_pin(PIN_USART_TXD_IDX, PIN_USART_TXD_IO_FLAGS);
	pio_set_pin_low(PIN_USART_TXD_IDX);

	/* Enable receiver. */
	usart_enable_rx(BOARD_USART);

	/* Read dummy to make sure that there are no characters in US_THR! */
	if (usart_is_rx_ready(BOARD_USART)) {
		usart_read(BOARD_USART, &ul_temp);
		ul_temp = ul_temp;
	}
}

/**
 * \brief Dump received buffer to uart.
 * \param p_buf   Pointer to received buffer.
 * \param uc_size The size of the buffer.
 */
static void dump_recv_buf(uint8_t *p_buf, uint8_t uc_size)
{
	uint8_t uc_i = 0;

	for (uc_i = 0; uc_i < uc_size; uc_i++) {
		printf("0x%x\t", p_buf[uc_i]);
	}
	puts("\r");
}

/**
 *  Configure board USART communication with PC or other terminal.
 */
static void configure_usart(void)
{
	static uint32_t dw_sysclk;
	const sam_usart_opt_t usart_console_settings = {
		BOARD_USART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		BOARD_USART_IRDA_FILTER
	};

	/* Get system clock. */
	dw_sysclk = sysclk_get_main_hz();

	/* Enable peripheral. */
	pmc_enable_periph_clk(BOARD_ID_USART);

	/* Configure USART in IrDA mode. */
	usart_init_irda(BOARD_USART, &usart_console_settings, dw_sysclk);

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);
	
	/* Enable TX & RX function. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(USART_IRQn);
}

/**
 *  Configure UART for debug message output.
 */
static void configure_console(void)
{
	const sam_uart_opt_t uart_console_settings =
			{ sysclk_get_main_hz(), 115200, UART_MR_PAR_NO };

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
 * \brief Application entry point.
 *
 * Initialize the IrDA and start the main loop.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_char;

	/* Initialize the SAM3 system. */
	sysclk_init();
	board_init();

	/* Configure UART for debug message output. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	/* Initialize board USART. */
	configure_usart();

	/* Get board USART PDC base address and enable receiver and transmitter. */
	p_pdc = usart_get_pdc_base(BOARD_USART);
	pdc_enable_transfer(p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	puts("-I- Press t to transmit, press r to receive...\r");

	/* Main loop. */
	while (1) {
		uc_char = 0;
		uart_read(CONSOLE_UART, (uint8_t *)&uc_char);

		switch (uc_char) {
		case 't':
		case 'T':
			state = STATE_TRANSMIT;
			/* Enable transmitter. */
			func_transmitter();
			st_packet.dw_addr = (uint32_t)send_data;
			st_packet.dw_size = BUFFER_SIZE;
			pdc_tx_init(p_pdc, &st_packet, NULL);
			usart_enable_interrupt(BOARD_USART, US_IER_TXBUFE);
			while (!sent_done) {
			}

			puts("-I- Sent Done!\r");
			sent_done = false;
			state = STATE_IDLE;
			puts("-I- Press t to transmit, press r to receive...\r");
			break;

		case 'r':
		case 'R':
			state = STATE_RECEIVE;

			/* Enable receiver. */
			puts("-I- IrDA receive mode\r");
			func_receiver();
			st_packet.dw_addr = (uint32_t)recv_data;
			st_packet.dw_size = BUFFER_SIZE;
			pdc_rx_init(p_pdc, &st_packet, NULL);
			usart_enable_interrupt(BOARD_USART, US_IER_RXBUFF);

			while (!recv_done) {
			}

			puts("-I- Received Done! \r");
			dump_recv_buf(recv_data, BUFFER_SIZE);
			memset(recv_data, 0, BUFFER_SIZE);
			state = STATE_IDLE;
			recv_done = false;
			puts("-I- Press t to transmit, press r to receive...\r");
			break;

		default:
			break;
		}
	}
}
