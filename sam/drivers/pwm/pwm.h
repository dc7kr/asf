/**
 * \file
 *
 * \brief Pulse Width Modulation (PWM) driver for SAM.
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

#ifndef PWM_H_INCLUDED
#define PWM_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#define PWM_INVALID_ARGUMENT  0xFFFF

/** Definitions for PWM channel number */
typedef enum _pwm_ch_t {
	PWM_CHANNEL_0 = (1 << 0),
	PWM_CHANNEL_1 = (1 << 1),
	PWM_CHANNEL_2 = (1 << 2),
	PWM_CHANNEL_3 = (1 << 3),
#if (SAM3XA)
	PWM_CHANNEL_4 = (1 << 4),
	PWM_CHANNEL_5 = (1 << 5),
	PWM_CHANNEL_6 = (1 << 6),
	PWM_CHANNEL_7 = (1 << 7)
#endif /* (SAM3XA) */
} pwm_ch_t;

/** Definitions for PWM channel alignment */
typedef enum {
	PWM_ALIGN_LEFT = (0 << 8),   /* The period is left aligned. */
	PWM_ALIGN_CENTER = (1 << 8)  /* The period is center aligned. */
} pwm_align_t;

/** Definitions for PWM level */
typedef enum {
	PWM_HIGH = HIGH,  /* High level */
	PWM_LOW = LOW     /* Low level */
} pwm_level_t;

/** Input parameters when initializing PWM */
typedef struct {
	/** Frequency of clock A in Hz (set 0 to turn it off) */
	uint32_t dw_clka;
	/** Frequency of clock B in Hz (set 0 to turn it off) */
	uint32_t dw_clkb;
	/** Frequency of master clock in Hz */
	uint32_t dw_mck;
} pwm_clock_t;

#if (SAM3U || SAM3S || SAM3XA || SAM4S)
/** Definitions for PWM channels used by motor stepper */
typedef enum {
	PWM_STEPPER_MOTOR_CH_0_1 = 0,  /* Channel 0 and 1 */
	PWM_STEPPER_MOTOR_CH_2_3 = 1,  /* Channel 2 and 3 */
#if (SAM3XA)
	PWM_STEPPER_MOTOR_CH_4_5 = 2,  /* Channel 4 and 5 */
	PWM_STEPPER_MOTOR_CH_6_7 = 3   /* Channel 6 and 7 */
#endif /* (SAM3XA) */
} pwm_stepper_motor_pair_t;

/** Definitions for PWM synchronous channels update mode */
typedef enum {
	PWM_SYNC_UPDATE_MODE_0 = PWM_SCM_UPDM_MODE0,
	PWM_SYNC_UPDATE_MODE_1 = PWM_SCM_UPDM_MODE1,
	PWM_SYNC_UPDATE_MODE_2 = PWM_SCM_UPDM_MODE2
} pwm_sync_update_mode_t;

/** Definitions for PWM event */
typedef enum {
	PWM_EVENT_PERIOD_END = (0 << 10),      /* The channel counter event occurs at the end of the PWM period. */
	PWM_EVENT_PERIOD_HALF_END = (1 << 10)  /* The channel counter event occurs at the half of the PWM period. */
} pwm_counter_event_t;

/** Definitions for PWM fault input ID */
typedef enum {
#if (SAM3U)
	PWM_FAULT_MAINOSC = (1 << 0),
	PWM_FAULT_PWMFI2 = (1 << 1),
	PWM_FAULT_PWMFI0 = (1 << 2),
	PWM_FAULT_PWMFI1 = (1 << 3)
#elif (SAM3S || SAM4S)
	PWM_FAULT_PWMFI1 = (1 << 0),
	PWM_FAULT_MAINOSC = (1 << 1),
	PWM_FAULT_ADC = (1 << 2),
	PWM_FAULT_ACC = (1 << 3),
	PWM_FAULT_TIMER_0 = (1 << 4),
	PWM_FAULT_TIMER_1 = (1 << 5)
#elif (SAM3XA)
	PWM_FAULT_PWMFI0 = (1 << 0),
	PWM_FAULT_PWMFI1 = (1 << 1),
	PWM_FAULT_PWMFI2 = (1 << 2),
	PWM_FAULT_MAINOSC = (1 << 3),
	PWM_FAULT_ADC = (1 << 4),
	PWM_FAULT_TIMER_0 = (1 << 5)
#endif
} pwm_fault_id_t;

/** Definitions of PWM register group */
typedef enum {
	PWM_GROUP_CLOCK = (1 << 0),
	PWM_GROUP_DISABLE = (1 << 1),
	PWM_GROUP_MODE = (1 << 2),
	PWM_GROUP_PERIOD = (1 << 3),
	PWM_GROUP_DEAD_TIME = (1 << 4),
	PWM_GROUP_FAULT = (1 << 5)
} pwm_protect_reg_group_t;

/** Definitions for PWM comparison interrupt */
typedef enum {
	PWM_CMP_MATCH = 8,   /* Comparison unit match */
	PWM_CMP_UPDATE = 16  /* Comparison unit update */
} pwm_cmp_interrupt_t;

/** Definitions for PWM comparison unit */
typedef enum {
	PWM_CMP_UNIT_0 = (1 << 0),
	PWM_CMP_UNIT_1 = (1 << 1),
	PWM_CMP_UNIT_2 = (1 << 2),
	PWM_CMP_UNIT_3 = (1 << 3),
	PWM_CMP_UNIT_4 = (1 << 4),
	PWM_CMP_UNIT_5 = (1 << 5),
	PWM_CMP_UNIT_6 = (1 << 6),
	PWM_CMP_UNIT_7 = (1 << 7)
} pmc_cmp_unit_t;

/** Definitions for PWM PDC transfer request mode */
typedef enum {
	PWM_PDC_UPDATE_PERIOD_ELAPSED = (0 << 20),  /* PDC transfer request is set as soon as the update period elapses. */
	PWM_PDC_COMPARISON_MATCH = (1 << 20)  /* PDC transfer request is set as soon as the selected comparison matches. */
} pwm_pdc_request_mode_t;

/** Definitions for PWM PDC transfer interrupt */
typedef enum {
	PWM_PDC_TX_END = (1 << 1),   /* PDC Tx end */
	PWM_PDC_TX_EMPTY = (1 << 2)  /* PDC Tx buffer empty */
} pwm_pdc_interrupt_t;

/** Definitions for PWM synchronous channels interrupt */
typedef enum {
	PWM_SYNC_WRITE_READY = (1 << 0),  /* Write Ready for Synchronous Channels Update */
	PWM_SYNC_UNDERRUN = (1 << 3)      /* Synchronous Channels Update Underrun Error */
} pwm_sync_interrupt_t;

/** Configurations of a PWM channel output */
typedef struct {
	/** Boolean of using override output as PWMH */
	bool b_override_pwmh;
	/** Boolean of using override output as PWML */
	bool b_override_pwml;
	/** Level of override output for PWMH */
	pwm_level_t override_level_pwmh;
	/** Level of override output for PWML */
	pwm_level_t override_level_pwml;
} pwm_output_t;

/** Configurations of PWM comparison */
typedef struct {
	/** Unit of comparison */
	pmc_cmp_unit_t unit;
	/** Boolean of comparison enable */
	bool b_enable;
	/** Comparison value */
	uint32_t dw_value;
	/** Comparison mode */
	bool b_is_decrementing;
	/** Comparison trigger value */
	uint32_t dw_trigger;
	/** Comparison period value */
	uint32_t dw_period;
	/** Comparison update period value */
	uint32_t dw_update_period;
	/** Boolean of generating a match pulse on PWM event line 0 */
	bool b_pulse_on_line_0;
	/** Boolean of generating a match pulse on PWM event line 1 */
	bool b_pulse_on_line_1;
} pwm_cmp_t;

/** Configuration of PWM fault input behaviors */
typedef struct {
	/** Fault ID */
	pwm_fault_id_t fault_id;
	/** Polarity of fault input */
	pwm_level_t polarity;
	/** Boolean of clearing fault flag */
	bool b_clear;
	/** Boolean of fault filtering */
	bool b_filtered;
} pwm_fault_t;

/** Structure of PWM write-protect information */
typedef struct {
	/** Bitmask of PWM register group for write protect hardware status */
	uint32_t dw_hw_status;
	/** Bitmask of PWM register group for write protect software status */
	uint32_t dw_sw_status;
	/** Offset address of PWM register in which a write access has been attempted */
	uint32_t dw_offset;
} pwm_protect_t;
#endif /* (SAM3U || SAM3S || SAM3XA) */

/** Input parameters when configuring a PWM channel mode */
typedef struct {
	/** Channel number */
	pwm_ch_t channel;
	/** Channel prescaler */
	uint32_t dw_prescaler;
    /** Channel alignment */
	pwm_align_t alignment;
    /** Channel initial polarity */
	pwm_level_t polarity;
	/** Duty Cycle Value */
	uint32_t dw_duty;
	/** Period Cycle Value */
	uint32_t dw_period;

#if (SAM3U || SAM3S || SAM3XA || SAM4S)
    /** Channel counter event */
	pwm_counter_event_t counter_event;
    /** Boolean of channel dead-time generator */
	bool b_deadtime_generator;
    /** Boolean of channel dead-time PWMH output inverted */
	bool b_pwmh_output_inverted;
    /** Boolean of channel dead-time PWML output inverted */
	bool b_pwml_output_inverted;
	/** Dead-time Value for PWMH Output */
	uint16_t w_deadtime_pwmh;
	/** Dead-time Value for PWML Output */
	uint16_t w_deadtime_pwml;
	/** Channel output */
	pwm_output_t output_selection;
	/** Boolean of Synchronous Channel */
	bool b_sync_ch;
	/** Fault ID of the channel */
	pwm_fault_id_t fault_id;
	/** Channel PWMH output level in fault protection */
	pwm_level_t dw_fault_output_pwmh;
	/** Channel PWML output level in fault protection */
	pwm_level_t dw_fault_output_pwml;
#endif /* (SAM3U || SAM3S || SAM3XA) */
} pwm_channel_t;


uint32_t pwm_init(Pwm *p_pwm, pwm_clock_t *clock_config);
uint32_t pwm_channel_init(Pwm *p_pwm, pwm_channel_t *p_channel);
uint32_t pwm_channel_update_period(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t dw_period);
uint32_t pwm_channel_update_duty(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t dw_duty);
uint32_t pwm_channel_get_counter(Pwm *p_pwm, pwm_channel_t *p_channel);
void pwm_channel_enable(Pwm *p_pwm, uint32_t dw_channel);
void pwm_channel_disable(Pwm *p_pwm, uint32_t dw_channel);
uint32_t pwm_channel_get_status(Pwm *p_pwm);
uint32_t pwm_channel_get_interrupt_status(Pwm *p_pwm);
uint32_t pwm_channel_get_interrupt_mask(Pwm *p_pwm);
void pwm_channel_enable_interrupt(Pwm *p_pwm, uint32_t dw_event,
		uint32_t dw_fault);
void pwm_channel_disable_interrupt(Pwm *p_pwm, uint32_t dw_event,
		uint32_t dw_fault);

#if (SAM3U || SAM3S || SAM3XA || SAM4S)
void pwm_channel_update_output(Pwm *p_pwm, pwm_channel_t *p_channel,
		pwm_output_t *p_output, bool b_sync);
void pwm_channel_update_dead_time(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint16_t w_deadtime_pwmh, uint16_t w_deadtime_pwml);

uint32_t pwm_fault_init(Pwm *p_pwm, pwm_fault_t *p_fault);
uint32_t pwm_fault_get_status(Pwm *p_pwm);
pwm_level_t pwm_fault_get_input_level(Pwm *p_pwm, pwm_fault_id_t id);
void pwm_fault_clear_status(Pwm *p_pwm, pwm_fault_id_t id);

uint32_t pwm_cmp_init(Pwm *p_pwm, pwm_cmp_t *p_cmp);
uint32_t pwm_cmp_change_setting(Pwm *p_pwm, pwm_cmp_t *p_cmp);
uint32_t pwm_cmp_get_period_counter(Pwm *p_pwm, pmc_cmp_unit_t unit);
uint32_t pwm_cmp_get_update_counter(Pwm *p_pwm, pmc_cmp_unit_t unit);
void pwm_cmp_enable_interrupt(Pwm *p_pwm, uint32_t dw_sources,
		pwm_cmp_interrupt_t type);
void pwm_cmp_disable_interrupt(Pwm *p_pwm, uint32_t dw_sources,
		pwm_cmp_interrupt_t type);
void pwm_pdc_set_request_mode(Pwm *p_pwm, pwm_pdc_request_mode_t request_mode,
		pmc_cmp_unit_t cmp_unit);

void pwm_pdc_enable_interrupt(Pwm *p_pwm, uint32_t dw_sources);
void pwm_pdc_disable_interrupt(Pwm *p_pwm, uint32_t dw_sources);

uint32_t pwm_sync_init(Pwm *p_pwm, pwm_sync_update_mode_t mode,
		uint32_t dw_update_period);
void pwm_sync_unlock_update(Pwm *p_pwm);
void pwm_sync_change_period(Pwm *p_pwm, uint32_t dw_update_period);
uint32_t pwm_sync_get_period_counter(Pwm * p_pwm);
void pwm_sync_enable_interrupt(Pwm *p_pwm, uint32_t dw_sources);
void pwm_sync_disable_interrupt(Pwm *p_pwm, uint32_t dw_sources);

void pwm_enable_protect(Pwm *p_pwm, uint32_t dw_group, bool b_sw);
void pwm_disable_protect(Pwm *p_pwm, uint32_t dw_group);
bool pwm_get_protect_status(Pwm *p_pwm, pwm_protect_t * p_protect);

uint32_t pwm_get_interrupt_status(Pwm *p_pwm);
uint32_t pwm_get_interrupt_mask(Pwm *p_pwm);
#endif /* (SAM3U || SAM3S || SAM3XA) */

#if (SAM3S || SAM3XA || SAM4S)
void pwm_stepper_motor_init(Pwm *p_pwm, pwm_stepper_motor_pair_t pair,
		bool b_enable_gray, bool b_down);
#endif /* (SAM3S || SAM3XA) */

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* PWM_H_INCLUDED */
