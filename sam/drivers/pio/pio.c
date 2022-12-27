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

#include "pio.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_drivers_pio_group Peripheral Parallel Input/Output (PIO) Controller
 *
 * \par Purpose
 *
 * The Parallel Input/Output Controller (PIO) manages up to 32 fully programmable 
 * input/output lines. Each I/O line may be dedicated as a general-purpose I/O or be 
 * assigned to a function of an embedded peripheral. This assures effective optimization 
 * of the pins of a product.
 *
 * @{
 */

#ifndef FREQ_SLOW_CLOCK_EXT
#define FREQ_SLOW_CLOCK_EXT 32768 /* External slow clock frequency (hz) */
#endif

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void pio_pull_up(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_pull_up_enable)
{
	/* Enable the pull-up(s) if necessary */
	if (dw_pull_up_enable) {
		p_pio->PIO_PUER = dw_mask;
	} else {
		p_pio->PIO_PUDR = dw_mask;
	}
}

/**
 * \brief Configure Glitch or Debouncing filter for the specified input(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_cut_off Cuts off frequency for debouncing filter.
 */
void pio_set_debounce_filter(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_cut_off)
{
#if (SAM3S || SAM3N || SAM4S)
	p_pio->PIO_IFSCER = dw_mask;	/* set Debouncing, 0 bit field no effect */
#elif (SAM3XA || SAM3U)
	p_pio->PIO_DIFSR = dw_mask;	/* set Debouncing, 0 bit field no effect */
#else
#error "Unsupported device"
#endif

	/* The debouncing filter can filter a pulse of less than 1/2 Period of a */
	/* programmable Divided Slow Clock: Tdiv_slclk = ((DIV+1)*2).Tslow_clock */
	p_pio->PIO_SCDR = PIO_SCDR_DIV((FREQ_SLOW_CLOCK_EXT / (2 * (dw_cut_off))) - 1);	
}

/**
 * \brief Set a high output level on all the PIOs defined in dw_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_set(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_SODR = dw_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in dw_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_clear(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_CODR = dw_mask;
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin instance currently have
 * a high level; otherwise returns 0. This method returns the actual value that
 * is being read on the pin. To return the supposed output value of a pin, use
 * pio_get_output_data_status() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_type PIO type.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 *
 * \retval 1 at least one PIO currently has a high level.
 * \retval 0 all PIOs have a low level.
 */
uint32_t pio_get(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask)
{
	uint32_t dw_reg;

	if ((dw_type == PIO_OUTPUT_0) || (dw_type == PIO_OUTPUT_1)) {
		dw_reg = p_pio->PIO_ODSR;
	} else {
		dw_reg = p_pio->PIO_PDSR;
	}

	if ((dw_reg & dw_mask) == 0) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * \brief Configure IO of a PIO controller as being controlled by a specific 
 * peripheral.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_type PIO type.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_set_peripheral(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask)
{
	uint32_t dw_sr;

	/* Disable interrupts on the pin(s) */
	p_pio->PIO_IDR = dw_mask;

#if (SAM3S || SAM3N || SAM4S)
	switch (dw_type) {
	case PIO_PERIPH_A:
		dw_sr = p_pio->PIO_ABCDSR[0];
		p_pio->PIO_ABCDSR[0] &= (~dw_mask & dw_sr);

		dw_sr = p_pio->PIO_ABCDSR[1];
		p_pio->PIO_ABCDSR[1] &= (~dw_mask & dw_sr);
		break;

	case PIO_PERIPH_B:
		dw_sr = p_pio->PIO_ABCDSR[0];
		p_pio->PIO_ABCDSR[0] = (dw_mask | dw_sr);

		dw_sr = p_pio->PIO_ABCDSR[1];
		p_pio->PIO_ABCDSR[1] &= (~dw_mask & dw_sr);
		break;

	case PIO_PERIPH_C:
		dw_sr = p_pio->PIO_ABCDSR[0];
		p_pio->PIO_ABCDSR[0] &= (~dw_mask & dw_sr);

		dw_sr = p_pio->PIO_ABCDSR[1];
		p_pio->PIO_ABCDSR[1] = (dw_mask | dw_sr);
		break;

	case PIO_PERIPH_D:
		dw_sr = p_pio->PIO_ABCDSR[0];
		p_pio->PIO_ABCDSR[0] = (dw_mask | dw_sr);

		dw_sr = p_pio->PIO_ABCDSR[1];
		p_pio->PIO_ABCDSR[1] = (dw_mask | dw_sr);
		break;

		// other types are invalid in this function
	case PIO_INPUT:
	case PIO_OUTPUT_0:
	case PIO_OUTPUT_1:
	case PIO_NOT_A_PIN:
		return;
	}
#elif (SAM3XA|| SAM3U)
	switch (dw_type) {
	case PIO_PERIPH_A:
		dw_sr = p_pio->PIO_ABSR;
		p_pio->PIO_ABSR &= (~dw_mask & dw_sr);
		break;

	case PIO_PERIPH_B:
		dw_sr = p_pio->PIO_ABSR;
		p_pio->PIO_ABSR = (dw_mask | dw_sr);
		break;

		// other types are invalid in this function
	case PIO_INPUT:
	case PIO_OUTPUT_0:
	case PIO_OUTPUT_1:
	case PIO_NOT_A_PIN:
		return;
	}
#else
#error "Unsupported device"
#endif

	// Remove the pins from under the control of PIO
	p_pio->PIO_PDR = dw_mask;
}

/**
 * \brief Configure one or more pin(s) or a PIO controller as inputs. 
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can 
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param dw_attribute PIO attribute(s).
 */
void pio_set_input(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_attribute)
{
	pio_disable_interrupt(p_pio, dw_mask);
	pio_pull_up(p_pio, dw_mask, dw_attribute & PIO_PULLUP);

	/* Enable Input Filter if necessary */
	if (dw_attribute & (PIO_DEGLITCH | PIO_DEBOUNCE)) {
		p_pio->PIO_IFER = dw_mask;
	} else {
		p_pio->PIO_IFDR = dw_mask;
	}

#if (SAM3S || SAM3N || SAM4S)
	/* Enable de-glitch or de-bounce if necessary */
	if (dw_attribute & PIO_DEGLITCH) {
		p_pio->PIO_IFSCDR = dw_mask;
	} else {
		if (dw_attribute & PIO_DEBOUNCE) {
			p_pio->PIO_IFSCER = dw_mask;
		}
	}
#elif (SAM3XA|| SAM3U)
	/* Enable de-glitch or de-bounce if necessary */
	if (dw_attribute & PIO_DEGLITCH) {
		p_pio->PIO_SCIFSR = dw_mask;
	} else {
		if (dw_attribute & PIO_DEBOUNCE) {
			p_pio->PIO_SCIFSR = dw_mask;
		}
	}
#else
#error "Unsupported device"
#endif

	/* Configure pin as input */
	p_pio->PIO_ODR = dw_mask;
	p_pio->PIO_PER = dw_mask;
}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with 
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask indicating which pin(s) to configure.
 * \param dw_default_level Default level on the pin(s).
 * \param dw_multidrive_enable Indicates if the pin(s) shall be configured as 
 * open-drain.
 * \param dw_pull_up_enable Indicates if the pin shall have its pull-up 
 * activated.
 */
void pio_set_output(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_default_level,
		const uint32_t dw_multidrive_enable,
		const uint32_t dw_pull_up_enable)
{
	pio_disable_interrupt(p_pio, dw_mask);
	pio_pull_up(p_pio, dw_mask, dw_pull_up_enable);

	/* Enable multi-drive if necessary */
	if (dw_multidrive_enable) {
		p_pio->PIO_MDER = dw_mask;
	} else {
		p_pio->PIO_MDDR = dw_mask;
	}

	/* Set default value */
	if (dw_default_level) {
		p_pio->PIO_SODR = dw_mask;
	} else {
		p_pio->PIO_CODR = dw_mask;
	}

	/* Configure pin(s) as output(s) */
	p_pio->PIO_OER = dw_mask;
	p_pio->PIO_PER = dw_mask;
}

/**
 * \brief Perform complete pin(s) configuration; general attributes and PIO init 
 * if necessary.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_type PIO type.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_attribute Pins attributes.
 *
 * \return Whether the pin(s) have been configured properly.
 */
uint32_t pio_configure(Pio *p_pio, const pio_type_t dw_type,
		const uint32_t dw_mask, const uint32_t dw_attribute)
{
	/* Configure pins */
	switch (dw_type) {
	case PIO_PERIPH_A:
	case PIO_PERIPH_B:
#     if (SAM3S || SAM3N || SAM4S)
	case PIO_PERIPH_C:
	case PIO_PERIPH_D:
#     endif
		pio_set_peripheral(p_pio, dw_type, dw_mask);
		pio_pull_up(p_pio, dw_mask, (dw_attribute & PIO_PULLUP));
		break;

	case PIO_INPUT:
		pio_set_input(p_pio, dw_mask, dw_attribute);
		break;

	case PIO_OUTPUT_0:
	case PIO_OUTPUT_1:
		pio_set_output(p_pio, dw_mask, (dw_type == PIO_OUTPUT_1),
				(dw_attribute & PIO_OPENDRAIN) ? 1 : 0,
				(dw_attribute & PIO_PULLUP) ? 1 : 0);
		break;

	default:
		return 0;
	}

	return 1;
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin are configured to 
 * output a high level (even if they are not output).
 * To get the actual value of the pin, use PIO_Get() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s).
 *
 * \retval 1 at least one PIO is configured to output a high level.
 * \retval 0 all PIOs are configued to output a low level.
 */
uint32_t pio_get_output_data_status(const Pio *p_pio,
		const uint32_t dw_mask)
{
	if ((p_pio->PIO_ODSR & dw_mask) == 0) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * \brief Configure PIO pin multi-driver.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_multi_driver_enable Indicates if the pin(s) multi-driver shall be 
 * configured.
 */
void pio_set_multi_driver(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_multi_driver_enable)
{
	/* Enable the multi-driver if necessary */
	if (dw_multi_driver_enable) {
		p_pio->PIO_MDER = dw_mask;
	} else {
		p_pio->PIO_MDDR = dw_mask;
	}
}

/**
 * \brief Get multi-driver status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The multi-driver mask value.
 */
uint32_t pio_get_multi_driver_status(const Pio *p_pio)
{
	return p_pio->PIO_MDSR;
}


#if (SAM3S || SAM3N || SAM4S)
/**
 * \brief Configure PIO pin internal pull-down.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_pull_down_enable Indicates if the pin(s) internal pull-down shall 
 * be configured.
 */
void pio_pull_down(Pio *p_pio, const uint32_t dw_mask,
		const uint32_t dw_pull_down_enable)
{
	/* Enable the pull-down if necessary */
	if (dw_pull_down_enable) {
		p_pio->PIO_PPDER = dw_mask;
	} else {
		p_pio->PIO_PPDDR = dw_mask;
	}
}
#endif

/**
 * \brief Enable PIO output write for synchronous data output.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_enable_output_write(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_OWER = dw_mask;
}

/**
 * \brief Disable PIO output write.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_disable_output_write(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_OWDR = dw_mask;
}

/**
 * \brief Read PIO output write status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The output write mask value.
 */
uint32_t pio_get_output_write_status(const Pio *p_pio)
{
	return p_pio->PIO_OWSR;
}

/**
 * \brief Synchronously write on output pins.
 * \note Only bits unmasked by PIO_OWSR (Output Write Status Register) are
 * written.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_sync_output_write(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_ODSR = dw_mask;
}

#if (SAM3S || SAM3N || SAM4S)
/**
 * \brief Configure PIO pin schmitt trigger. By default the Schmitt trigger is
 * active.
 * Disabling the Schmitt Trigger is requested when using the QTouch Library.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_set_schmitt_trigger(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_SCHMITT = dw_mask;
}

/**
 * \brief Get PIO pin schmitt trigger status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The schmitt trigger mask value.
 */
uint32_t pio_get_schmitt_trigger(const Pio *p_pio)
{
	return p_pio->PIO_SCHMITT;
}
#endif

/**
 * \brief Configure the given interrupt source.
 * Interrupt can be configured to trigger on rising edge, falling edge, high level,
 * low level or simply on level change.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Interrupt source bit map.
 * \param ul_attr Interrupt source attributes.
 */
void pio_configure_interrupt(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attr)
{
	/* Configure additional interrupt mode registers. */
	if (ul_attr & PIO_IT_AIME) {
		/* Enable additional interrupt mode. */
		p_pio->PIO_AIMER = ul_mask;

		/* if bit field of the selected pin is 1, set as Rising Edge/High level detection event. */
		if (ul_attr & PIO_IT_RE_OR_HL) {
			p_pio->PIO_REHLSR = ul_mask; /* Rising Edge or High Level */
		} else {
			p_pio->PIO_FELLSR = ul_mask; /* Falling Edge or Low Level */
		}

		/* if bit field of the selected pin is 1, set as edge detection source. */
		if (ul_attr & PIO_IT_EDGE)
			p_pio->PIO_ESR = ul_mask; /* Edge select */
		else
			p_pio->PIO_LSR = ul_mask; /* Level select */
	} else {
		/* Disable additional interrupt mode. */
		p_pio->PIO_AIMDR = ul_mask;
	}
}

/**
 * \brief Enable the given interrupt source.
 * The PIO must be configured as an NVIC interrupt source as well.
 * The status register of the corresponding PIO controller is cleared 
 * prior to enabling the interrupt.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Interrupt sources bit map.
 */
void pio_enable_interrupt(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_ISR;
	p_pio->PIO_IER = dw_mask;
}

/**
 * \brief Disable a given interrupt source, with no added side effects.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Interrupt sources bit map.
 */
void pio_disable_interrupt(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_IDR = dw_mask;
}

/**
 * \brief Read PIO interrupt status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt status mask value.
 */
uint32_t pio_get_interrupt_status(const Pio *p_pio)
{
	return p_pio->PIO_ISR;
}

/**
 * \brief Read PIO interrupt mask.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt mask value.
 */
uint32_t pio_get_interrupt_mask(const Pio *p_pio)
{
	return p_pio->PIO_IMR;
}

/**
 * \brief Set additional interrupt mode.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Interrupt sources bit map.
 * \param dw_attribute Pin(s) attributes.
 */
void pio_set_additional_interrupt_mode(Pio *p_pio,
		const uint32_t dw_mask, const uint32_t dw_attribute)
{
	/* Enables additional interrupt mode if needed */
	if (dw_attribute & PIO_IT_AIME) {
		/* Enables additional interrupt mode */
		p_pio->PIO_AIMER = dw_mask;

		/* Configures the Polarity of the event detection */
		/* (Rising/Falling Edge or High/Low Level) */
		if (dw_attribute & PIO_IT_RE_OR_HL) {
			p_pio->PIO_REHLSR = dw_mask; /* Rising Edge or High Level */
		} else {
			p_pio->PIO_FELLSR = dw_mask; /* Falling Edge or Low Level */
		}

		/* Configures the type of event detection (Edge or Level) */
		if (dw_attribute & PIO_IT_EDGE) {
			p_pio->PIO_ESR = dw_mask; /* Edge select */
		} else {
			p_pio->PIO_LSR = dw_mask; /* Level select */
		}
	} else {
		/* Disable additional interrupt mode */
		p_pio->PIO_AIMDR = dw_mask;
	}
}

#define PIO_WPMR_WPKEY_VALUE PIO_WPMR_WPKEY(0x50494Fu)

/**
 * \brief Enable or disable write protect of PIO registers.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_enable 1 to enable, 0 to disable.
 */
void pio_set_writeprotect(Pio *p_pio, const uint32_t dw_enable)
{
	p_pio->PIO_WPMR = PIO_WPMR_WPKEY_VALUE | dw_enable;
}

/**
 * \brief Read write protect status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return Return write protect status.
 */
uint32_t pio_get_writeprotect_status(const Pio *p_pio)
{
	return p_pio->PIO_WPSR;
}

#define PIO_DELTA	((uint32_t) PIOB - (uint32_t) PIOA)

/**
 * \brief Return the value of a pin.
 *
 * \param pin The pin number.
 *
 * \return The pin value.
 *
 * \note If pin is output: a pull-up or pull-down could hide the actual value.
 *       The function \ref pio_get can be called to get the actual pin output level.
 * \note If pin is input: PIOx must be clocked to sample the signal. See PMC driver.
 */
uint32_t pio_get_pin_value(uint32_t dw_pin)
{
	Pio *p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (dw_pin >> 5)));
	return (p_pio->PIO_PDSR >> (dw_pin & 0x1F)) & 1;
}

/**
 * \brief Drive a GPIO pin to 1.
 *
 * \param pin The pin index.
 *
 * \note The function \ref gpio_configure_pin must be called before.
 */
void pio_set_pin_high(uint32_t dw_pin)
{
	Pio *p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (dw_pin >> 5)));
	p_pio->PIO_SODR = 1 << (dw_pin & 0x1F); // Value to be driven on the I/O line: 1.
}

/**
 * \brief Drive a GPIO pin to 0.
 *
 * \param pin The pin index.
 *
 * \note The function \ref gpio_configure_pin must be called before.
 */
void pio_set_pin_low(uint32_t dw_pin)
{
	Pio *p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (dw_pin >> 5)));
	p_pio->PIO_CODR = 1 << (dw_pin & 0x1F); // Value to be driven on the I/O line: 0.
}

/**
 * \brief Toggle a GPIO pin.
 *
 * \param pin The pin index.
 *
 * \note The function \ref gpio_configure_pin must be called before.
 */
void pio_toggle_pin(uint32_t dw_pin)
{
	Pio *p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (dw_pin >> 5)));
	if (p_pio->PIO_ODSR & (1 << (dw_pin & 0x1F)))
		p_pio->PIO_CODR = 1 << (dw_pin & 0x1F); // Value to be driven on the I/O line: 0.
	else
		p_pio->PIO_SODR = 1 << (dw_pin & 0x1F); // Value to be driven on the I/O line: 1.
}

/**
 * \brief Perform complete pin(s) configuration; general attributes and PIO init 
 * if necessary.
 *
 * \param dw_pin Bitmask of one or more pin(s) to configure.
 * \param dw_flags Pins attributes.
 *
 * \return Whether the pin(s) have been configured properly.
 */
uint32_t pio_configure_pin(uint32_t dw_pin, const uint32_t dw_flags)
{
	Pio *p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (dw_pin >> 5)));

	/* Configure pins */
	switch (dw_flags & PIO_TYPE_Msk) {
	case PIO_TYPE_PIO_PERIPH_A:
		pio_set_peripheral(p_pio, PIO_PERIPH_A, (1 << (dw_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (dw_pin & 0x1F)), (dw_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_B:
		pio_set_peripheral(p_pio, PIO_PERIPH_B, (1 << (dw_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (dw_pin & 0x1F)), (dw_flags & PIO_PULLUP));
		break;
#     if (SAM3S || SAM3N || SAM4S)
	case PIO_TYPE_PIO_PERIPH_C:
		pio_set_peripheral(p_pio, PIO_PERIPH_C, (1 << (dw_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (dw_pin & 0x1F)), (dw_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_D:
		pio_set_peripheral(p_pio, PIO_PERIPH_D, (1 << (dw_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (dw_pin & 0x1F)), (dw_flags & PIO_PULLUP));
		break;
#     endif

	case PIO_TYPE_PIO_INPUT:
		pio_set_input(p_pio, (1 << (dw_pin & 0x1F)), dw_flags);
		break;

	case PIO_TYPE_PIO_OUTPUT_0:
	case PIO_TYPE_PIO_OUTPUT_1:
		pio_set_output(p_pio, (1 << (dw_pin & 0x1F)), (dw_flags & PIO_TYPE_PIO_OUTPUT_1),
				(dw_flags & PIO_OPENDRAIN) ? 1 : 0,
				(dw_flags & PIO_PULLUP) ? 1 : 0);
		break;

	default:
		return 0;
	}

	return 1;
}

/**
 * \brief Drive a GPIO port to 1.
 *
 * \param port The port number.
 * \param mask The mask.
 */
void pio_set_pin_group_high(Pio *p_pio, uint32_t dw_mask)
{
	p_pio->PIO_SODR = dw_mask; // Value to be driven on the I/O line: 1.
}

/**
 * \brief Drive a GPIO port to 0.
 *
 * \param port The port number.
 * \param mask The mask.
 */
void pio_set_pin_group_low(Pio *p_pio, uint32_t dw_mask)
{
	p_pio->PIO_CODR = dw_mask; // Value to be driven on the I/O line: 0.
}

/**
 * \brief Toggle a GPIO group.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 */
void pio_toggle_pin_group(Pio *p_pio, uint32_t dw_mask)
{
	if (p_pio->PIO_ODSR & dw_mask)
		p_pio->PIO_CODR = dw_mask; // Value to be driven on the I/O line: 0.
	else
		p_pio->PIO_SODR = dw_mask; // Value to be driven on the I/O line: 1.
}

/**
 * \brief Perform complete pin(s) configuration; general attributes and PIO init 
 * if necessary.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Bitmask of one or more pin(s) to configure.
 * \param dw_flags Pin(s) attributes.
 *
 * \return Whether the pin(s) have been configured properly.
 */
uint32_t pio_configure_pin_group(Pio *p_pio, uint32_t dw_mask, const uint32_t dw_flags)
{
	/* Configure pins */
	switch (dw_flags & PIO_TYPE_Msk) {
	case PIO_TYPE_PIO_PERIPH_A:
		pio_set_peripheral(p_pio, PIO_PERIPH_A, dw_mask);
		pio_pull_up(p_pio, dw_mask, (dw_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_B:
		pio_set_peripheral(p_pio, PIO_PERIPH_B, dw_mask);
		pio_pull_up(p_pio, dw_mask, (dw_flags & PIO_PULLUP));
		break;
#     if (SAM3S || SAM3N || SAM4S)
	case PIO_TYPE_PIO_PERIPH_C:
		pio_set_peripheral(p_pio, PIO_PERIPH_C, dw_mask);
		pio_pull_up(p_pio, dw_mask, (dw_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_D:
		pio_set_peripheral(p_pio, PIO_PERIPH_D, dw_mask);
		pio_pull_up(p_pio, dw_mask, (dw_flags & PIO_PULLUP));
		break;
#     endif

	case PIO_TYPE_PIO_INPUT:
		pio_set_input(p_pio, dw_mask, dw_flags);
		break;

	case PIO_TYPE_PIO_OUTPUT_0:
	case PIO_TYPE_PIO_OUTPUT_1:
		pio_set_output(p_pio, dw_mask, (dw_flags & PIO_TYPE_PIO_OUTPUT_1),
				(dw_flags & PIO_OPENDRAIN) ? 1 : 0,
				(dw_flags & PIO_PULLUP) ? 1 : 0);
		break;

	default:
		return 0;
	}

	return 1;
}

#if (SAM3S || SAM4S)
/**
 * \brief Configure PIO capture mode.
 * \note PIO capture mode will be disabled automatically.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mode Bitmask of one or more modes.
 */
void pio_capture_set_mode(Pio *p_pio, uint32_t dw_mode)
{
	dw_mode &= (~PIO_PCMR_PCEN); /* Disable PIO capture mode */
	p_pio->PIO_PCMR = dw_mode;
}

/**
 * \brief Enable PIO capture mode.
 *
 * \param p_pio Pointer to a PIO instance.
 */
void pio_capture_enable(Pio *p_pio)
{
	p_pio->PIO_PCMR |= PIO_PCMR_PCEN;
}

/**
 * \brief Disable PIO capture mode.
 *
 * \param p_pio Pointer to a PIO instance.
 */
void pio_capture_disable(Pio *p_pio)
{
	p_pio->PIO_PCMR &= (~PIO_PCMR_PCEN);
}

/**
 * \brief Read from Capture Reception Holding Register.
 * Data presence should be tested before any read attempt.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param pdw_data Pointer to store the data.
 *
 * \retval 0 Success.
 * \retval 1 I/O Failure, Capture data is not ready.
 */
uint32_t pio_capture_read(const Pio *p_pio, uint32_t *pdw_data)
{
	/* Check if the data is ready */
	if ((p_pio->PIO_PCISR & PIO_PCISR_DRDY) == 0) {
		return 1;
	}

	/* Read data */
	*pdw_data = p_pio->PIO_PCRHR;
	return 0;
}

/**
 * \brief Enable the given interrupt source of PIO capture. The status
 * register of the corresponding PIO capture controller is cleared prior
 * to enabling the interrupt.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Interrupt sources bit map.
 */
void pio_capture_enable_interrupt(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_PCISR;
	p_pio->PIO_PCIER = dw_mask;
}

/**
 * \brief Disable a given interrupt source of PIO capture.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param dw_mask Interrupt sources bit map.
 */
void pio_capture_disable_interrupt(Pio *p_pio, const uint32_t dw_mask)
{
	p_pio->PIO_PCIDR = dw_mask;
}

/**
 * \brief Read PIO interrupt status of PIO capture.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt status mask value.
 */
uint32_t pio_capture_get_interrupt_status(const Pio *p_pio)
{
	return p_pio->PIO_PCISR;
}

/**
 * \brief Read PIO interrupt mask of PIO capture.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt mask value.
 */
uint32_t pio_capture_get_interrupt_mask(const Pio *p_pio)
{
	return p_pio->PIO_PCIMR;
}

/**
 * \brief Get PDC registers base address.
 *
 * \param p_pio Pointer to an PIO peripheral.
 *
 * \return PIOA PDC register base address.
 */
Pdc *pio_capture_get_pdc_base(const Pio *p_pio)
{
	p_pio = p_pio; /* Stop warning */
	return PDC_PIOA;
}
#endif

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
