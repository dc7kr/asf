/**
 * \file
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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

#ifndef _SAM3U_ADC_INSTANCE_
#define _SAM3U_ADC_INSTANCE_

/* ========== Register definition for ADC peripheral ========== */
#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#define REG_ADC_CR              (0x400AC000U) /**< \brief (ADC) Control Register */
#define REG_ADC_MR              (0x400AC004U) /**< \brief (ADC) Mode Register */
#define REG_ADC_CHER            (0x400AC010U) /**< \brief (ADC) Channel Enable Register */
#define REG_ADC_CHDR            (0x400AC014U) /**< \brief (ADC) Channel Disable Register */
#define REG_ADC_CHSR            (0x400AC018U) /**< \brief (ADC) Channel Status Register */
#define REG_ADC_SR              (0x400AC01CU) /**< \brief (ADC) Status Register */
#define REG_ADC_LCDR            (0x400AC020U) /**< \brief (ADC) Last Converted Data Register */
#define REG_ADC_IER             (0x400AC024U) /**< \brief (ADC) Interrupt Enable Register */
#define REG_ADC_IDR             (0x400AC028U) /**< \brief (ADC) Interrupt Disable Register */
#define REG_ADC_IMR             (0x400AC02CU) /**< \brief (ADC) Interrupt Mask Register */
#define REG_ADC_CDR             (0x400AC030U) /**< \brief (ADC) Channel Data Register */
#define REG_ADC_RPR             (0x400AC100U) /**< \brief (ADC) Receive Pointer Register */
#define REG_ADC_RCR             (0x400AC104U) /**< \brief (ADC) Receive Counter Register */
#define REG_ADC_TPR             (0x400AC108U) /**< \brief (ADC) Transmit Pointer Register */
#define REG_ADC_TCR             (0x400AC10CU) /**< \brief (ADC) Transmit Counter Register */
#define REG_ADC_RNPR            (0x400AC110U) /**< \brief (ADC) Receive Next Pointer Register */
#define REG_ADC_RNCR            (0x400AC114U) /**< \brief (ADC) Receive Next Counter Register */
#define REG_ADC_TNPR            (0x400AC118U) /**< \brief (ADC) Transmit Next Pointer Register */
#define REG_ADC_TNCR            (0x400AC11CU) /**< \brief (ADC) Transmit Next Counter Register */
#define REG_ADC_PTCR            (0x400AC120U) /**< \brief (ADC) Transfer Control Register */
#define REG_ADC_PTSR            (0x400AC124U) /**< \brief (ADC) Transfer Status Register */
#else
#define REG_ADC_CR     (*(WoReg*)0x400AC000U) /**< \brief (ADC) Control Register */
#define REG_ADC_MR     (*(RwReg*)0x400AC004U) /**< \brief (ADC) Mode Register */
#define REG_ADC_CHER   (*(WoReg*)0x400AC010U) /**< \brief (ADC) Channel Enable Register */
#define REG_ADC_CHDR   (*(WoReg*)0x400AC014U) /**< \brief (ADC) Channel Disable Register */
#define REG_ADC_CHSR   (*(RoReg*)0x400AC018U) /**< \brief (ADC) Channel Status Register */
#define REG_ADC_SR     (*(RoReg*)0x400AC01CU) /**< \brief (ADC) Status Register */
#define REG_ADC_LCDR   (*(RoReg*)0x400AC020U) /**< \brief (ADC) Last Converted Data Register */
#define REG_ADC_IER    (*(WoReg*)0x400AC024U) /**< \brief (ADC) Interrupt Enable Register */
#define REG_ADC_IDR    (*(WoReg*)0x400AC028U) /**< \brief (ADC) Interrupt Disable Register */
#define REG_ADC_IMR    (*(RoReg*)0x400AC02CU) /**< \brief (ADC) Interrupt Mask Register */
#define REG_ADC_CDR    (*(RoReg*)0x400AC030U) /**< \brief (ADC) Channel Data Register */
#define REG_ADC_RPR    (*(RwReg*)0x400AC100U) /**< \brief (ADC) Receive Pointer Register */
#define REG_ADC_RCR    (*(RwReg*)0x400AC104U) /**< \brief (ADC) Receive Counter Register */
#define REG_ADC_TPR    (*(RwReg*)0x400AC108U) /**< \brief (ADC) Transmit Pointer Register */
#define REG_ADC_TCR    (*(RwReg*)0x400AC10CU) /**< \brief (ADC) Transmit Counter Register */
#define REG_ADC_RNPR   (*(RwReg*)0x400AC110U) /**< \brief (ADC) Receive Next Pointer Register */
#define REG_ADC_RNCR   (*(RwReg*)0x400AC114U) /**< \brief (ADC) Receive Next Counter Register */
#define REG_ADC_TNPR   (*(RwReg*)0x400AC118U) /**< \brief (ADC) Transmit Next Pointer Register */
#define REG_ADC_TNCR   (*(RwReg*)0x400AC11CU) /**< \brief (ADC) Transmit Next Counter Register */
#define REG_ADC_PTCR   (*(WoReg*)0x400AC120U) /**< \brief (ADC) Transfer Control Register */
#define REG_ADC_PTSR   (*(RoReg*)0x400AC124U) /**< \brief (ADC) Transfer Status Register */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#endif /* _SAM3U_ADC_INSTANCE_ */
