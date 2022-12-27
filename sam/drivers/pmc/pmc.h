/**
 * \file
 *
 * \brief Power Management Controller (PMC) driver for SAM.
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

#ifndef PMC_H_INCLUDED
#define PMC_H_INCLUDED

#include "compiler.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** Bit mask for peripheral clocks (PCER0) */
#define PMC_MASK_STATUS0    (0xFFFFFFFC)

/** Bit mask for peripheral clocks (PCER1) */
#define PMC_MASK_STATUS1    (0xFFFFFFFF)

/** Loop counter timeout value */
#define PMC_TIMEOUT			(2048)

/** Key to unlock CKGR_MOR register */
#define PMC_CKGR_MOR_KEY_VALUE CKGR_MOR_KEY(0x37)

/** Key used to write SUPC registers */
#define SUPC_KEY_VALUE ((uint32_t) 0xA5)

/** PMC xtal statup time */
#define PMC_XTAL_STARTUP_TIME (0x3F)

/** Mask to access fast startup input */
#define PMC_FAST_STARTUP_Msk (0xFFFF)

/** PMC_WPMR Write Protect KEY, unlock it */
#define PMC_WPMR_WPKEY_VALUE PMC_WPMR_WPKEY((uint32_t) 0x504D43)

/** Using external oscillator */
#define PMC_OSC_XTAL    0

/** Oscillator in bypass mode */
#define PMC_OSC_BYPASS  1

#define PMC_PCK_0   0	/* PCK0 ID */
#define PMC_PCK_1   1	/* PCK1 ID */
#define PMC_PCK_2   2	/* PCK2 ID */

uint32_t pmc_init(void);
void pmc_enable_pllack(uint32_t mula, uint32_t pllacount, uint32_t diva);
uint32_t pmc_is_locked_pllack(void);
void pmc_disable_pllack(void);
void pmc_pck_set_prescaler(uint32_t dw_id, uint32_t dw_pres);
void pmc_pck_set_source(uint32_t dw_id, uint32_t dw_source);
void pmc_mck_set_prescaler(uint32_t dw_pres);
void pmc_mck_set_source(uint32_t dw_source);
void pmc_switch_sclk_to_32kxtal(uint32_t dw_bypass);
uint32_t pmc_osc_is_ready_32kxtal(void);
void pmc_switch_mainck_to_fastrc(uint32_t dw_moscrcf);
void pmc_osc_enable_fastrc(uint32_t dw_rc);
void pmc_osc_disable_fastrc(void);
uint32_t pmc_osc_is_ready_fastrc(void);
void pmc_switch_mainck_to_xtal(uint32_t dw_bypass);
void pmc_osc_disable_xtal(uint32_t dw_bypass);
uint32_t pmc_osc_is_ready_xtal(void);
uint32_t pmc_switch_mck_to_sclk(uint32_t dw_pres);
uint32_t pmc_switch_mck_to_mainck(uint32_t dw_pres);
uint32_t pmc_switch_mck_to_pllack(uint32_t dw_pres);
uint32_t pmc_enable_periph_clk(uint32_t dw_id);
uint32_t pmc_disable_periph_clk(uint32_t dw_id);
void pmc_enable_all_periph_clk(void);
void pmc_disable_all_periph_clk(void);
uint32_t pmc_is_periph_clk_enabled(uint32_t dw_id);
uint32_t pmc_switch_pck_to_sclk(uint32_t dw_id, uint32_t dw_pres);
uint32_t pmc_switch_pck_to_mainck(uint32_t dw_id, uint32_t dw_pres);
uint32_t pmc_switch_pck_to_pllack(uint32_t dw_id, uint32_t dw_pres);
void pmc_enable_pck(uint32_t dw_id);
void pmc_disable_pck(uint32_t dw_id);
void pmc_enable_all_pck(void);
void pmc_disable_all_pck(void);
uint32_t pmc_is_pck_enabled(uint32_t dw_id);
void pmc_set_fast_startup_input(uint32_t dw_inputs);
void pmc_enable_sleepmode(uint8_t uc_type);
void pmc_enable_waitmode(void);
void pmc_enable_backupmode(void);
void pmc_enable_interrupt(uint32_t dw_sources);
void pmc_disable_interrupt(uint32_t dw_sources);
uint32_t pmc_get_interrupt_mask(void);
uint32_t pmc_get_status(void);
void pmc_set_writeprotect(uint32_t dw_enable);
uint32_t pmc_get_writeprotect_status(void);

#if (SAM3S || SAM4S)
void pmc_enable_pllbck(uint32_t mulb, uint32_t pllbcount, uint32_t divb);
uint32_t pmc_is_locked_pllbck(void);
void pmc_disable_pllbck(void);
uint32_t pmc_switch_mck_to_pllbck(uint32_t dw_pres);
uint32_t pmc_switch_pck_to_pllbck(uint32_t dw_id, uint32_t dw_pres);
void pmc_switch_udpck_to_pllbck(uint32_t dw_usbdiv);
#endif

#if (SAM3S || SAM3XA || SAM4S)
void pmc_switch_udpck_to_pllack(uint32_t dw_usbdiv);
void pmc_enable_udpck(void);
void pmc_disable_udpck(void);
#endif

#if (SAM3XA || SAM3U || SAM4S)
uint32_t pmc_is_locked_upll(void);
void pmc_enable_upll_clock(void);
void pmc_disable_upll_clock(void);
uint32_t pmc_switch_mck_to_upllck(uint32_t dw_pres);
uint32_t pmc_switch_pck_to_upllck(uint32_t dw_id, uint32_t dw_pres);
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* PMC_H_INCLUDED */
