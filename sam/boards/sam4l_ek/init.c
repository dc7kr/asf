/**
 * \file
 *
 * \brief SAM4L-EK Board init.
 *
 * This file contains board initialization function.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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
 *
 */
#include "compiler.h"
#include "sam4l_ek.h"
#include "conf_board.h"
#include "ioport.h"
#include "board.h"
/**
 * \addtogroup  sam4l_ek_group
 * @{
 */

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

void board_init(void)
{
	// Initialize IOPORTs
	ioport_init();

	// Put all pins to default state (input & pull-up)
	uint32_t pin;
	for (pin = PIN_PA00; pin <= PIN_PC31; pin ++) {
		// Skip output pins to configure later
		if (pin == LED0_GPIO || pin == LCD_BL_GPIO
#ifdef CONF_BOARD_RS485
		|| pin == RS485_USART_CTS_PIN
#endif
		) {
			continue;
		}
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(pin, IOPORT_MODE_PULLUP);
	}

	/* Configure the pins connected to LEDs as output and set their
	 * default initial state to high (LEDs off).
	 */
	ioport_set_pin_dir(LED0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);

#ifdef  CONF_BOARD_EIC
	// Set push button as external interrupt pin
	ioport_set_pin_peripheral_mode(GPIO_PUSH_BUTTON_EIC_PIN,
			GPIO_PUSH_BUTTON_EIC_PIN_MUX);
	ioport_set_pin_peripheral_mode(GPIO_UNIT_TEST_EIC_PIN,
			GPIO_UNIT_TEST_EIC_PIN_MUX);
#else
	// Push button as input: already done, it's the default pin state
#endif

#if (defined CONF_BOARD_BL)
	// Configure LCD backlight
	ioport_set_pin_dir(LCD_BL_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_BL_GPIO, LCD_BL_INACTIVE_LEVEL);
#endif

#if (defined CONF_BOARD_USB_PORT)
	ioport_set_pin_peripheral_mode(PIN_PA25A_USBC_DM, MUX_PA25A_USBC_DM);
	ioport_set_pin_peripheral_mode(PIN_PA26A_USBC_DP, MUX_PA26A_USBC_DP);
	ioport_set_pin_dir(GPIO_VBUS_INPUT, IOPORT_DIR_INPUT);
#endif

#if defined (CONF_BOARD_COM_PORT)
	ioport_set_pin_peripheral_mode(COM_PORT_RX_PIN, COM_PORT_RX_MUX);
	ioport_set_pin_peripheral_mode(COM_PORT_TX_PIN, COM_PORT_TX_MUX);
#endif

#if defined (CONF_BOARD_BM_USART)
	ioport_set_pin_peripheral_mode(BM_USART_RX_PIN, BM_USART_RX_MUX);
	ioport_set_pin_peripheral_mode(BM_USART_TX_PIN, BM_USART_TX_MUX);
#endif

#ifdef CONF_BOARD_SPI
	ioport_set_pin_peripheral_mode(PIN_PC04A_SPI_MISO, MUX_PC04A_SPI_MISO);
	ioport_set_pin_peripheral_mode(PIN_PC05A_SPI_MOSI, MUX_PC05A_SPI_MOSI);
	ioport_set_pin_peripheral_mode(PIN_PC06A_SPI_SCK, MUX_PC06A_SPI_SCK);

	#ifdef CONF_BOARD_SPI_NPCS0
		ioport_set_pin_peripheral_mode(PIN_PA02B_SPI_NPCS0,
				MUX_PA02B_SPI_NPCS0);
	#endif
	#ifdef CONF_BOARD_SPI_NPCS2
		ioport_set_pin_peripheral_mode(PIN_PC00A_SPI_NPCS2,
				MUX_PC00A_SPI_NPCS2);
	#endif
	#ifdef CONF_BOARD_SPI_NPCS3
		ioport_set_pin_peripheral_mode(PIN_PC01A_SPI_NPCS3,
				MUX_PC01A_SPI_NPCS3);
	#endif

#endif

#ifdef CONF_BOARD_RS485
	ioport_set_pin_peripheral_mode(RS485_USART_RX_PIN, RS485_USART_RX_MUX);
	ioport_set_pin_peripheral_mode(RS485_USART_TX_PIN, RS485_USART_TX_MUX);
	ioport_set_pin_peripheral_mode(RS485_USART_RTS_PIN,
			RS485_USART_RTS_MUX);
	ioport_set_pin_dir(RS485_USART_CTS_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(RS485_USART_CTS_PIN, IOPORT_PIN_LEVEL_LOW);
#endif

#ifdef CONF_BOARD_DACC_VOUT
	ioport_set_pin_peripheral_mode(DACC_VOUT_PIN, DACC_VOUT_MUX);
#endif
}
/**
 * @}
 */