/**
 * \file
 *
 * \brief User Interface
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

#include "compiler.h"
#include "conf_usb.h"
#include "board.h"
#include "led.h"
#include "ui.h"

void ui_init(void)
{
	LED_On(LED0_GPIO);
	LED_Off(LED1_GPIO);
	LED_Off(LED2_GPIO);
	LED_Off(LED3_GPIO);
}

void ui_powerdown(void)
{
	LED_Off(LED0_GPIO);
	LED_Off(LED1_GPIO);
	LED_Off(LED2_GPIO);
	LED_Off(LED3_GPIO);
	led_power_off();
}

void ui_wakeup(void)
{
	LED_On(LED0_GPIO);
	led_power_on();
}

void ui_com_open(void)
{
	LED_On(LED1_GPIO);
}

void ui_com_close(void)
{
	LED_Off(LED1_GPIO);
	LED_Off(LED2_GPIO);
	LED_Off(LED3_GPIO);
}

void ui_com_rx_start(void)
{
	LED_On(LED2_GPIO);
}

void ui_com_rx_stop(void)
{
	LED_Off(LED2_GPIO);
}

void ui_com_tx_start(void)
{
	LED_On(LED3_GPIO);
}

void ui_com_tx_stop(void)
{
	LED_Off(LED3_GPIO);
}

void ui_com_error(void)
{
}

void ui_com_overflow(void)
{
}

void ui_process(uint16_t framenumber)
{
	if (0 == framenumber) {
		LED_On(LED0_GPIO);
	}
	if (1000 == framenumber) {
		LED_Off(LED0_GPIO);
	}
}

/**
 * \defgroup UI User Interface
 *
 * Human interface on XMEGA-B1 Xplained:
 * - Led 0 and the RED led close to USB connector are on
 *   when USB line is in IDLE mode, and off in SUSPEND mode
 * - Led 1 blinks when USB Host have checked and enabled CDC interface
 * - Led 2 is on during data transfer from CDC to UART
 * - Led 3 is on during data transfer from UART to CDC
 */
