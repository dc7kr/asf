/**
 * \file
 *
 * \brief Parallel Input/Output (PIO) Controller driver for SAM.
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

#ifndef PIO_H_INCLUDED
#define PIO_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/* GPIO Support */
#define PIO_TYPE_Pos 				27
#define PIO_TYPE_Msk 				(0xFu << PIO_TYPE_Pos) /* PIO Type */
#define   PIO_TYPE_NOT_A_PIN 		(0x0u << PIO_TYPE_Pos) /* The pin is not a function pin. */
#define   PIO_TYPE_PIO_PERIPH_A 	(0x1u << PIO_TYPE_Pos) /* The pin is controlled by the peripheral A. */
#define   PIO_TYPE_PIO_PERIPH_B 	(0x2u << PIO_TYPE_Pos) /* The pin is controlled by the peripheral B. */
#define   PIO_TYPE_PIO_PERIPH_C 	(0x3u << PIO_TYPE_Pos) /* The pin is controlled by the peripheral C. */
#define   PIO_TYPE_PIO_PERIPH_D 	(0x4u << PIO_TYPE_Pos) /* The pin is controlled by the peripheral D. */
#define   PIO_TYPE_PIO_INPUT 		(0x5u << PIO_TYPE_Pos) /* The pin is an input. */
#define   PIO_TYPE_PIO_OUTPUT_0 	(0x6u << PIO_TYPE_Pos) /* The pin is an output and has a default level of 0. */
#define   PIO_TYPE_PIO_OUTPUT_1 	(0x7u << PIO_TYPE_Pos) /* The pin is an output and has a default level of 1. */

typedef enum _pio_type {
	PIO_NOT_A_PIN	= PIO_TYPE_NOT_A_PIN,
	PIO_PERIPH_A 	= PIO_TYPE_PIO_PERIPH_A,
	PIO_PERIPH_B 	= PIO_TYPE_PIO_PERIPH_B,
#if (SAM3S || SAM3N || SAM4S)
	PIO_PERIPH_C 	= PIO_TYPE_PIO_PERIPH_C,
	PIO_PERIPH_D 	= PIO_TYPE_PIO_PERIPH_D,
#endif
	PIO_INPUT 		= PIO_TYPE_PIO_INPUT,
	PIO_OUTPUT_0 	= PIO_TYPE_PIO_OUTPUT_0,
	PIO_OUTPUT_1 	= PIO_TYPE_PIO_OUTPUT_1
} pio_type_t;

/*  Default pin configuration (no attribute). */
#define PIO_DEFAULT                 (0u << 0)
/*  The internal pin pull-up is active. */
#define PIO_PULLUP                  (1u << 0)
/*  The internal glitch filter is active. */
#define PIO_DEGLITCH                (1u << 1)
/*  The pin is open-drain. */
#define PIO_OPENDRAIN               (1u << 2)

/*  The internal debouncing filter is active. */
#define PIO_DEBOUNCE                (1u << 3)

/*  Enable additional interrupt modes. */
#define PIO_IT_AIME                 (1u << 4)

/*  Interrupt High Level/Rising Edge detection is active. */
#define PIO_IT_RE_OR_HL             (1u << 5)
/*  Interrupt Edge detection is active. */
#define PIO_IT_EDGE                 (1u << 6)

/*  Low level interrupt is active */
#define PIO_IT_LOW_LEVEL            (0               | 0 | PIO_IT_AIME)
/*  High level interrupt is active */
#define PIO_IT_HIGH_LEVEL           (PIO_IT_RE_OR_HL | 0 | PIO_IT_AIME)
/*  Falling edge interrupt is active */
#define PIO_IT_FALL_EDGE            (0               | PIO_IT_EDGE | PIO_IT_AIME)
/*  Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE            (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)

/*
 *  The #attribute# field is a bitmask that can either be set to PIO_DEFAULT,
 *  or combine (using bitwise OR '|') any number of the following constants:
 *     - PIO_PULLUP
 *     - PIO_DEGLITCH
 *     - PIO_DEBOUNCE
 *     - PIO_OPENDRAIN
 *     - PIO_IT_LOW_LEVEL
 *     - PIO_IT_HIGH_LEVEL
 *     - PIO_IT_FALL_EDGE
 *     - PIO_IT_RISE_EDGE
 */
void pio_pull_up(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_pull_up_enable);
void pio_set_debounce_filter(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_cut_off);
void pio_set(Pio *p_pio, const uint32_t dw_mask);
void pio_clear(Pio *p_pio, const uint32_t dw_mask);
uint32_t pio_get(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask);
void pio_set_peripheral(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask);
void pio_set_input(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_attribute);
void pio_set_output(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_default_level,
		const uint32_t dw_multidrive_enable,
		const uint32_t dw_pull_up_enable);
uint32_t pio_configure(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask, const uint32_t dw_attribute);
uint32_t pio_get_output_data_status(const Pio *p_pio,
		const uint32_t dw_mask);
void pio_set_multi_driver(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_multi_driver_enable);
uint32_t pio_get_multi_driver_status(const Pio *p_pio);

#if (SAM3S || SAM3N || SAM4S)
void pio_pull_down(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_pull_down_enable);
#endif

void pio_enable_output_write(Pio *p_pio, const uint32_t dw_mask);
void pio_disable_output_write(Pio *p_pio, const uint32_t dw_mask);
uint32_t pio_get_output_write_status(const Pio *p_pio);
void pio_sync_output_write(Pio *p_pio, const uint32_t dw_mask);

#if (SAM3S || SAM3N || SAM4S)
void pio_set_schmitt_trigger(Pio *p_pio, const uint32_t dw_mask);
uint32_t pio_get_schmitt_trigger(const Pio *p_pio);
#endif

void pio_configure_interrupt(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attr);
void pio_enable_interrupt(Pio *p_pio, const uint32_t dw_mask);
void pio_disable_interrupt(Pio *p_pio, const uint32_t dw_mask);
uint32_t pio_get_interrupt_status(const Pio *p_pio);
uint32_t pio_get_interrupt_mask(const Pio *p_pio);
void pio_set_additional_interrupt_mode(Pio *p_pio,
		const uint32_t dw_mask, const uint32_t dw_attribute);
void pio_set_writeprotect(Pio *p_pio, const uint32_t dw_enable);
uint32_t pio_get_writeprotect_status(const Pio *p_pio);

#if (SAM3S || SAM4S)
void pio_capture_set_mode(Pio *p_pio, uint32_t dw_mode);
void pio_capture_enable(Pio *p_pio);
void pio_capture_disable(Pio *p_pio);
uint32_t pio_capture_read(const Pio *p_pio, uint32_t * pdw_data);
void pio_capture_enable_interrupt(Pio *p_pio, const uint32_t dw_mask);
void pio_capture_disable_interrupt(Pio * p_pio, const uint32_t dw_mask);
uint32_t pio_capture_get_interrupt_status(const Pio *p_pio);
uint32_t pio_capture_get_interrupt_mask(const Pio *p_pio);
Pdc *pio_capture_get_pdc_base(const Pio *p_pio);
#endif

/* GPIO Support */
uint32_t pio_get_pin_value(uint32_t pin);
void pio_set_pin_high(uint32_t pin);
void pio_set_pin_low(uint32_t pin);
void pio_toggle_pin(uint32_t pin);
uint32_t pio_configure_pin(uint32_t dw_pin, const uint32_t dw_flags);
void pio_set_pin_group_high(Pio *p_pio, uint32_t dw_mask);
void pio_set_pin_group_low(Pio *p_pio, uint32_t dw_mask);
void pio_toggle_pin_group(Pio *p_pio, uint32_t dw_mask);
uint32_t pio_configure_pin_group(Pio *p_pio, uint32_t dw_mask, const uint32_t dw_flags);

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* PIO_H_INCLUDED */
