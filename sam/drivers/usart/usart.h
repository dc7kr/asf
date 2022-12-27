/**
 * \file
 *
 * \brief Universal Synchronous Asynchronous Receiver Transmitter (USART) driver for SAM.
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

#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** Clock phase. */
#define SPI_CPHA	(1 << 0)

/** Clock polarity. */
#define SPI_CPOL	(1 << 1)

/** SPI mode definition. */
#define SPI_MODE_0	(SPI_CPHA)
#define SPI_MODE_1	0
#define SPI_MODE_2	(SPI_CPOL | SPI_CPHA)
#define SPI_MODE_3	(SPI_CPOL)

//! Input parameters when initializing RS232 and similar modes.
typedef struct {
	//! Set baud rate of the USART (unused in slave modes).
	uint32_t baudrate;
	
	//! Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	//! US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or US_MR_MODE9.
	uint32_t char_length;
	
	//! Parity type, which should be one of the following: US_MR_PAR_EVEN, US_MR_PAR_ODD,
	//! US_MR_PAR_SPACE, US_MR_PAR_MARK, US_MR_PAR_NO or US_MR_PAR_MULTIDROP.
	uint32_t parity_type;

	//! Number of stop bits between two characters: US_MR_NBSTOP_1_BIT,
	//! US_MR_NBSTOP_1_5_BIT, US_MR_NBSTOP_2_BIT.
	//! \note US_MR_NBSTOP_1_5_BIT is supported in asynchronous modes only.
	uint32_t stop_bits;

	//! Run the channel in test mode, which should be one of following: US_MR_CHMODE_NORMAL,
	//! US_MR_CHMODE_AUTOMATIC, US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK
	uint32_t channel_mode;

	//! Filter of IrDA mode, useless in other modes. 
	uint32_t irda_filter;
} sam_usart_opt_t;

//! Input parameters when initializing ISO7816 mode.
typedef struct {
	//! Set the frequency of the ISO7816 clock.
	uint32_t iso7816_hz;
	
	//! The number of ISO7816 clock ticks in every bit period (1 to 2047, 0 = disable clock).
	//! Baudrate rate = iso7816_hz / fidi_ratio
	uint32_t fidi_ratio;

	//! How to calculate the parity bit: US_MR_PAR_EVEN for normal mode or
	//! US_MR_PAR_ODD for inverse mode.
	uint32_t parity_type;

	//! Inhibit Non Acknowledge:
	//!   - 0: the NACK is generated;
	//!   - 1: the NACK is not generated.
	//!
	//! \note This bit will be used only in ISO7816 mode, protocol T = 0 receiver.
	uint32_t inhibit_nack;

	//! Disable successive NACKs.
	//!  - 0: NACK is sent on the ISO line as soon as a parity error occurs in the received character.
	//! Successive parity errors are counted up to the value in the max_iterations field.
	//! These parity errors generate a NACK on the ISO line. As soon as this value is reached,
	//! No additional NACK is sent on the ISO line. The ITERATION flag is asserted.
	uint32_t dis_suc_nack;

	//! Max number of repetitions (0 to 7).
	uint32_t max_iterations;

	//! Bit order in transmitted characters:
	//!   - 0: LSB first;
	//!   - 1: MSB first.
	uint32_t bit_order;
	
	//! Which protocol is used:
	//!   - 0: T = 0;
	//!   - 1: T = 1.
	uint32_t protocol_type;
} usart_iso7816_opt_t;

//! Input parameters when initializing SPI mode.
typedef struct {
	//! Set the frequency of the SPI clock (unused in slave mode).
	uint32_t baudrate;

	//! Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	//! US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or US_MR_MODE9.
	uint32_t char_length;

	//! Which SPI mode to use, which should be one of the following:
	//! SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3.
	uint32_t spi_mode;

	//! Run the channel in test mode, which should be one of following: US_MR_CHMODE_NORMAL,
	//! US_MR_CHMODE_AUTOMATIC, US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK
	uint32_t channel_mode;
} usart_spi_opt_t;

void usart_reset(Usart *p_usart);
uint32_t usart_init_rs232(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_hw_handshaking(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
#if (SAM3S || SAM4S || SAM3U)
uint32_t usart_init_modem(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
#endif
uint32_t usart_init_sync_master(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_sync_slave(Usart *p_usart, const sam_usart_opt_t *p_usart_opt);
uint32_t usart_init_rs485(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_irda(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_iso7816(Usart *p_usart, const usart_iso7816_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_spi_master(Usart *p_usart, const usart_spi_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_spi_slave(Usart *p_usart, const usart_spi_opt_t *p_usart_opt);
#if SAM3XA
uint32_t usart_init_lin_master(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
uint32_t usart_init_lin_slave(Usart *p_usart, const sam_usart_opt_t *p_usart_opt, uint32_t ul_mck);
void usart_lin_abort_tx(Usart *p_usart);
void usart_lin_send_wakeup_signal(Usart *p_usart);
void usart_lin_set_node_action(Usart *p_usart, uint8_t uc_action);
void usart_lin_disable_parity(Usart *p_usart);
void usart_lin_enable_parity(Usart *p_usart);
void usart_lin_disable_checksum(Usart *p_usart);
void usart_lin_enable_checksum(Usart *p_usart);
void usart_lin_set_checksum_type(Usart *p_usart, uint8_t uc_type);
void usart_lin_set_data_len_mode(Usart *p_usart, uint8_t uc_mode);
void usart_lin_disable_frame_slot(Usart *p_usart);
void usart_lin_enable_frame_slot(Usart *p_usart);
void usart_lin_set_wakeup_signal_type(Usart *p_usart, uint8_t uc_type);
void usart_lin_set_response_data_len(Usart *p_usart, uint8_t uc_len);
void usart_lin_disable_pdc_mode(Usart *p_usart);
void usart_lin_enable_pdc_mode(Usart *p_usart);
void usart_lin_set_tx_identifier(Usart *p_usart, uint8_t uc_id);
uint8_t usart_lin_read_identifier(Usart *p_usart);
#endif
void usart_enable_tx(Usart *p_usart);
void usart_disable_tx(Usart *p_usart);
void usart_reset_tx(Usart *p_usart);
void usart_set_tx_timeguard(Usart *p_usart, uint32_t timeguard);
void usart_enable_rx(Usart *p_usart);
void usart_disable_rx(Usart *p_usart);
void usart_reset_rx(Usart *p_usart);
void usart_set_rx_timeout(Usart *p_usart, uint32_t timeout);
void usart_enable_interrupt(Usart *p_usart,uint32_t dw_sources);
void usart_disable_interrupt(Usart *p_usart,uint32_t dw_sources);
uint32_t usart_get_interrupt_mask(Usart *p_usart);
uint32_t usart_get_status(Usart *p_usart);
void usart_reset_status(Usart *p_usart);
void usart_start_tx_break(Usart *p_usart);
void usart_stop_tx_break(Usart *p_usart);
void usart_start_rx_timeout(Usart *p_usart);
uint32_t usart_send_address(Usart *p_usart, uint32_t ul_addr);
void usart_reset_iterations(Usart *p_usart);
void usart_reset_nack(Usart *p_usart);
void usart_restart_rx_timeout(Usart *p_usart);
#if (SAM3S || SAM4S || SAM3U)
void usart_drive_DTR_pin_low(Usart *p_usart);
void usart_drive_DTR_pin_high(Usart *p_usart);
#endif
void usart_drive_RTS_pin_low(Usart *p_usart);
void usart_drive_RTS_pin_high(Usart *p_usart);
void usart_spi_force_chip_select(Usart *p_usart);
void usart_spi_release_chip_select(Usart *p_usart);
uint32_t usart_is_tx_ready(Usart *p_usart);
uint32_t usart_is_tx_empty(Usart *p_usart);
uint32_t usart_is_rx_ready(Usart *p_usart);
uint32_t usart_is_rx_buf_end(Usart *p_usart);
uint32_t usart_is_tx_buf_end(Usart *p_usart);
uint32_t usart_is_rx_buf_full(Usart *p_usart);
uint32_t usart_is_tx_buf_empty(Usart *p_usart);
uint32_t usart_write(Usart *p_usart, uint32_t c);
uint32_t usart_putchar(Usart *p_usart, uint32_t c);
void usart_write_line(Usart *p_usart, const char *string);
uint32_t usart_read(Usart *p_usart, uint32_t *c);
uint32_t usart_getchar(Usart *p_usart, uint32_t *c);
#if (SAM3XA || SAM3U)
uint32_t * usart_get_tx_access(Usart *p_usart);
uint32_t * usart_get_rx_access(Usart *p_usart);
#endif
Pdc *usart_get_pdc_base(Usart *p_usart);
void usart_enable_writeprotect(Usart *p_usart);
void usart_disable_writeprotect(Usart *p_usart);
uint32_t usart_get_writeprotect_status(Usart *p_usart);
uint8_t usart_get_error_number(Usart *p_usart);
#if (SAM3S || SAM4S || SAM3U || SAM3XA)
void usart_man_set_tx_pre_len(Usart *p_usart, uint8_t uc_len);
void usart_man_set_tx_pre_pattern(Usart *p_usart, uint8_t uc_pattern);
void usart_man_set_tx_polarity(Usart *p_usart, uint8_t uc_polarity);
void usart_man_set_rx_pre_len(Usart *p_usart, uint8_t uc_len);
void usart_man_set_rx_pre_pattern(Usart *p_usart, uint8_t uc_pattern);
void usart_man_set_rx_polarity(Usart *p_usart, uint8_t uc_polarity);
void usart_man_enable_drift_compensation(Usart *p_usart);
void usart_man_disable_drift_compensation(Usart *p_usart);
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* USART_H_INCLUDED */
