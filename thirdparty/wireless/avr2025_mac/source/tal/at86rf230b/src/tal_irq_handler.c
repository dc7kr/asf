/**
 * @file tal_irq_handler.c
 *
 * @brief This file handles the interrupts generated by the transceiver.
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 */

/*
 * Copyright (c) 2013, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pal.h"
#include "return_val.h"
#include "tal.h"
#include "ieee_const.h"
#include "stack_config.h"
#include "bmm.h"
#include "qmm.h"
#include "tal_irq_handler.h"
#include "tal_rx.h"
#include "at86rf230b.h"
#include "tal_internal.h"
#include "tal_constants.h"
#include "tal_tx.h"
#include "mac_build_config.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts.
 */
void trx_irq_handler_cb(void)
{
	trx_irq_reason_t trx_irq_cause;

	trx_irq_cause = (trx_irq_reason_t)pal_trx_reg_read(RG_IRQ_STATUS);

	if (trx_irq_cause & TRX_IRQ_TRX_END) {
		/*
		 * TRX_END reason depends on if the trx is currently used for
		 * transmission or reception.
		 */
#if ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT))
		if ((tal_state == TAL_TX_AUTO) || (tal_state == TAL_TX_BASIC) ||
				(tal_state == TAL_TX_BEACON))
#else
		if ((tal_state == TAL_TX_AUTO) || (tal_state == TAL_TX_BASIC))
#endif
		{
			/* Switch to transceiver's default state: switch
			 *receiver on. */
			set_trx_state(CMD_RX_AACK_ON);

			/* Get the result and push it to the queue. */
			if (trx_irq_cause & TRX_IRQ_TRX_UR) {
				handle_tx_end_irq(true); /* see tal_tx.c */
			} else {
				handle_tx_end_irq(false); /* see tal_tx.c */
			}
		} else { /* Other tal_state than TAL_TX_... */
			/* Handle rx done interrupt. */
			handle_received_frame_irq(); /* see tal_rx.c */
		}
	}

#if (_DEBUG_ > 0)
	/* Other IRQ than TRX_END */
	if (trx_irq_cause != TRX_IRQ_TRX_END) {
		if (trx_irq_cause & TRX_IRQ_PLL_LOCK) {
			Assert("unexpected IRQ: TRX_IRQ_PLL_LOCK" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_PLL_UNLOCK) {
			Assert("unexpected IRQ: TRX_IRQ_PLL_UNLOCK" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_RX_START) {
			Assert("unexpected IRQ: TRX_IRQ_RX_START" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_4) {
			Assert("unexpected IRQ: TRX_IRQ_4" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_5) {
			Assert("unexpected IRQ: TRX_IRQ_5" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_TRX_UR) {
			Assert("unexpected IRQ: TRX_IRQ_TRX_UR" == 0);
		}

		if (trx_irq_cause & TRX_IRQ_BAT_LOW) {
			Assert("unexpected IRQ: TRX_IRQ_BAT_LOW" == 0);
		}
	}

#endif
} /* trx_irq_handler_cb() */

/* EOF */
