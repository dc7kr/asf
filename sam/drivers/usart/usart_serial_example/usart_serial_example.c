/**
 * \file
 *
 * \brief USART Serial example for SAM.
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
 * \mainpage USART Serial Example
 *
 * \par Purpose
 * This example demonstrates the normal (serial) mode provided by the USART
 * peripherals.
 *
 * \par Requirements
 *  This package can be used with all SAM3-EK. 
 *
 * \par Description
 *
 * On start up, the debug information is dumped to on-board USART port.
 * A terminal application, such as hyperterminal, is used to monitor these
 * debug information. Open another hyperterminal to connect with
 * on-board USART port. Then the program works in ECHO mode, so USART will
 * send back anything it receives from the hyperterminal.  You can send a text
 * file from the hyperterminal connected with USART port to the device (without
 * any protocol such as X-modem).
 *
 * \par Usage
 *
 * -# Build the program and download it into the two evaluation boards.
 * -# Connect a serial cable to the UART port for each evaluation kit.
 * -# On the computer, open and configure a terminal application
 *    (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- Basic USART Serial Project --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -- Start to echo serial inputs --
 *    \endcode
 * -# Send a file in text format from the hyperterminal connected with USART port to
 *    the device. On HyperTerminal, this is done by selecting "Transfer -> Send Text File"
 *    (this does not prevent you from sending binary files). The transfer will start and then
 *    you could read the file in the hyperterminal.
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
#include "tc.h"
#include "pdc.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"

/** Size of the receive buffer used by the PDC, in bytes. */
#define BUFFER_SIZE         150

/** USART PDC transfer type definition. */
#define PDC_TRANSFER        1

/** USART FIFO transfer type definition. */
#define BYTE_TRANSFER       0

/** Max buffer number. */
#define MAX_BUF_NUM         1

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK  0xffffffff

/** Timer counter frequency in Hz. */
#define TC_FREQ             20

#define STRING_EOL    "\r"
#define STRING_HEADER "--USART Serial Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Receive buffer. */
static uint8_t p_buffer[2][BUFFER_SIZE];

/** Current bytes in buffer. */
static uint32_t dw_size_buffer = BUFFER_SIZE;

/** Byte mode read buffer. */
static uint16_t us_read_buffer = 0;

/** Current transfer mode. */
static uint8_t uc_trans_mode = PDC_TRANSFER;

/** Buffer number in use. */
static uint8_t uc_buf_num = 0;

/** PDC data packet. */
pdc_packet_t st_packet;

/** Pointer to PDC register base. */
Pdc *p_pdc;

/**
 * \brief Interrupt handler for USART. Echo the bytes received and start the next
 *        receive.
 */
void USART_Handler(void)
{
	uint32_t dw_status;

	/* Read USART Status. */
	dw_status = usart_get_status(BOARD_USART);

	if (uc_trans_mode == PDC_TRANSFER) {
		/* Receive buffer is full. */
		if (dw_status & US_CSR_ENDRX) {
			/* Disable timer. */
			tc_stop(TC0, 0);

			/* Echo back buffer. */
			st_packet.dw_addr = (uint32_t)p_buffer[uc_buf_num];
			st_packet.dw_size = dw_size_buffer;
			pdc_tx_init(p_pdc, &st_packet, NULL);
			pdc_enable_transfer(p_pdc, PERIPH_PTCR_TXTEN);
			uc_buf_num = MAX_BUF_NUM - uc_buf_num;
			dw_size_buffer = BUFFER_SIZE;

			/* Restart read on buffer. */
			st_packet.dw_addr = (uint32_t)p_buffer[uc_buf_num];
			st_packet.dw_size = BUFFER_SIZE;
			pdc_rx_init(p_pdc, &st_packet, NULL);

			/* Restart timer. */
			tc_start(TC0, 0);
		}
	} else {
		/* Transfer without PDC. */
		if (dw_status & US_CSR_RXRDY) {
			usart_getchar(BOARD_USART, (uint32_t *)&us_read_buffer);
			usart_write(BOARD_USART, us_read_buffer);
		}
	}
}

/**
 * \brief Interrupt handler for TC0. Record the number of bytes received,
 * and then restart a read transfer on the USART if the transfer was stopped.
 */
void TC0_Handler(void)
{
	uint32_t dw_status;
	uint32_t dw_byte_total = 0;

	/* Read TC0 Status. */
	dw_status = tc_get_status(TC0, 0);

	/* RC compare. */
	if (((dw_status & TC_SR_CPCS) == TC_SR_CPCS) &&
			(uc_trans_mode == PDC_TRANSFER)) {
		/* Flush PDC buffer. */
		dw_byte_total = BUFFER_SIZE - pdc_read_rx_counter(p_pdc);
		if (dw_byte_total == 0) {
			/* Return when no bytes received. */
			return;
		}

		/* Log current size. */
		dw_size_buffer = dw_byte_total;

		/* Trigger USART ENDRX. */
		st_packet.dw_size = 0;
		pdc_rx_init(p_pdc, &st_packet, NULL);
	}
}

/**
 * \brief Configure USART in normal (serial rs232) mode, asynchronous, 8 bits, 1 stop
 * bit, no parity, 115200 bauds and enable its transmitter and receiver.
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
		/* This field is only used in IrDA mode. */
		0
	};

	/* Get system clock. */
	dw_sysclk = sysclk_get_main_hz();

	/* Enable the peripheral clock in the PMC. */
	pmc_enable_periph_clk(BOARD_ID_USART);

	/* Configure USART in serial mode. */
	usart_init_rs232(BOARD_USART, &usart_console_settings, dw_sysclk);

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(USART_IRQn);
}

/**
 * \brief Configure Timer Counter 0 (TC0) to generate an interrupt every 200ms.
 * This interrupt will be used to flush USART input and echo back.
 */
static void configure_tc(void)
{
	uint32_t dw_div;
	uint32_t dw_tcclks;
	static uint32_t dw_sysclk;

	/* Get system clock. */
	dw_sysclk = sysclk_get_main_hz();

	/* Configure PMC. */
	pmc_enable_periph_clk(ID_TC0);

	/* Configure TC for a 50Hz frequency and trigger on RC compare. */
	tc_find_mck_divisor(TC_FREQ, dw_sysclk, &dw_div, &dw_tcclks, dw_sysclk);
	tc_init(TC0, 0, dw_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (dw_sysclk / dw_div) / TC_FREQ);

	/* Configure and enable interrupt on RC compare. */
	NVIC_EnableIRQ((IRQn_Type) ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
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
 * \brief Reset the TX & RX, and clear the PDC counter.
 */
static void usart_clear(void)
{
	/* Reset and disable receiver & transmitter. */
	usart_reset_rx(BOARD_USART);
	usart_reset_tx(BOARD_USART);

	/* Clear PDC counter. */
	st_packet.dw_addr = 0;
	st_packet.dw_size = 0;
	pdc_rx_init(p_pdc, &st_packet, NULL);

	/* Enable receiver & transmitter. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);
}

/**
 * \brief Display main menu.
 */
static void display_main_menu(void)
{
	puts("-- Menu Choices for this example --\r\n"
			"-- s: Switch mode for USART between PDC and without PDC.--\r\n"
			"-- m: Display this menu again.--\r");
}

/**
 * \brief Application entry point for usart_serial example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_char;
	uint8_t uc_flag;

	/* Initialize the SAM3 system. */
	sysclk_init();
	board_init();

	/* Configure UART for debug message output. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	/* Configure USART. */
	configure_usart();

	/* Get board USART PDC base address and enable receiver and transmitter. */
	p_pdc = usart_get_pdc_base(BOARD_USART);
	pdc_enable_transfer(p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Configure TC. */
	configure_tc();

	/* Start receiving data and start timer. */
	st_packet.dw_addr = (uint32_t)p_buffer[uc_buf_num];
	st_packet.dw_size = BUFFER_SIZE;
	pdc_rx_init(p_pdc, &st_packet, NULL);

	dw_size_buffer = BUFFER_SIZE;

	puts("-I- Default Transfer with PDC \r\n"
			"-I- Press 's' to switch transfer mode \r");
	uc_trans_mode = PDC_TRANSFER;

	usart_disable_interrupt(BOARD_USART, US_IDR_RXRDY);
	usart_enable_interrupt(BOARD_USART, US_IER_ENDRX);

	tc_start(TC0, 0);

	while (1) {
		uc_char = 0;
		uc_flag = uart_read(CONSOLE_UART, &uc_char);
		if (!uc_flag) {
			switch (uc_char) {
			case 's':
			case 'S':
				if (uc_trans_mode == PDC_TRANSFER) {
					/* Disable PDC controller. */
					pdc_disable_transfer(p_pdc,
							PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
					/* Transfer to no PDC communication mode, and disable the ENDRX interrupt. */
					usart_disable_interrupt(BOARD_USART, US_IDR_ENDRX);

					/* Clear USART controller. */
					usart_clear();
					
					/* Enable the RXRDY interrupt. */
					usart_enable_interrupt(BOARD_USART, US_IER_RXRDY);
					uc_trans_mode = BYTE_TRANSFER;

					puts("-I- Transfer without PDC \r");
				} else if (uc_trans_mode == BYTE_TRANSFER) {
					pdc_enable_transfer(p_pdc,
							PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
					/* Clear USART controller. */
					usart_clear();

					/* Reset pdc current buffer size. */
					dw_size_buffer = BUFFER_SIZE;
					uc_buf_num = 0;

					/* Start receiving data. */
					st_packet.dw_addr = (uint32_t)p_buffer[uc_buf_num];
					st_packet.dw_size = BUFFER_SIZE;
					pdc_rx_init(p_pdc, &st_packet, NULL);
					
					/* Transfer to PDC communication mode, disable RXRDY interrupt and enable ENDRX interrupt. */
					usart_disable_interrupt(BOARD_USART, US_IER_RXRDY);
					usart_enable_interrupt(BOARD_USART, US_IER_ENDRX);

					uc_trans_mode = PDC_TRANSFER;
					puts("-I- Transfer with PDC \r");
				}
				break;
			case 'm':
			case 'M':
				display_main_menu();
				break;
			default:
				break;
			}
		}
	}
}
