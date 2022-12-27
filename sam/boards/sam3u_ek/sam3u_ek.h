/**
 * \file
 *
 * \brief SAM3U-EK Board Definition.
 *
 * Copyright (c) 2011 - 2012 Atmel Corporation. All rights reserved.
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

#ifndef _SAM3U_EK_H_
#define _SAM3U_EK_H_

#include "compiler.h"
#include "system_sam3u.h"
#include "exceptions.h"

/*----------------------------------------------------------------------------*/
/**
 *  \page sam3u_ek_opfreq SAM3U-EK - Operating frequencies
 *  This page lists several definition related to the board operating frequency
 *
 *  \section Definitions
 *  - \ref BOARD_FREQ_*
 *  - \ref BOARD_MCK
 */

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL		(32768U)
#define BOARD_FREQ_SLCK_BYPASS		(32768U)
#define BOARD_FREQ_MAINCK_XTAL		(12000000U)
#define BOARD_FREQ_MAINCK_BYPASS	(12000000U)

/** Master clock frequency */
#define BOARD_MCK					CHIP_FREQ_CPU_MAX

/*----------------------------------------------------------------------------*/
/**
 * \page sam3u_ek_board_info SAM3U-EK - Board informations
 * This page lists several definition related to the board description.
 *
 * \section Definitions
 * - \ref BOARD_NAME
 * - \ref sam3uek
 * - \ref sam3u
 * - \ref cortexm3
 */

/** Name of the board */
#define BOARD_NAME "SAM3U-EK"
/** Board definition */
#define sam3uek
/** Family definition (already defined) */
#define sam3u
/** Core definition */
#define cortexm3

/*----------------------------------------------------------------------------*/
/**
 * \page sam3u_ek_pio_info SAM3UE-EK - PIO definitions
 * This pages lists all the pio definitions. The constants
 * are named using the following convention: PIN_* for a constant which defines
 * a single Pin instance (but may include several PIOs sharing the same
 * controller), and PINS_* for a list of Pin instances.
 *
 * \section ADC
 * - \ref PIN_ADC0_AD0
 * - \ref PIN_ADC0_AD1
 * - \ref PIN_ADC0_AD2
 * - \ref PIN_ADC0_AD3
 * - \ref PIN_ADC0_AD4
 * - \ref PIN_ADC0_AD5
 * - \ref PIN_ADC0_AD6
 * - \ref PIN_ADC0_AD7
 * - \ref PINS_ADC
 *
 *
 * \section EBI
 * - \ref PIN_EBI_DATA_BUS
 * - \ref PIN_EBI_NCS0
 * - \ref PIN_EBI_NRD
 * - \ref PIN_EBI_NWE
 * - \ref PIN_EBI_PSRAM_ADDR_BUS
 * - \ref PIN_EBI_PSRAM_NBS
 * - \ref PIN_EBI_A1
 * - \ref PIN_EBI_NCS2
 * - \ref PIN_EBI_LCD_RS
 *
 * \section LEDs
 * - \ref PIN_LED_0
 * - \ref PIN_LED_1
 * - \ref PIN_LED_2
 * - \ref PINS_LEDS
 *
 * \section MCI
 * - \ref PINS_MCI
 * - \ref PIN_MCI_DAT0
 * - \ref PIN_MCI_CD
 *
 * \section Push buttons
 * - \ref PIN_PUSHBUTTON_1
 * - \ref PIN_PUSHBUTTON_2
 * - \ref PINS_PUSHBUTTONS
 * - \ref PUSHBUTTON_BP1
 * - \ref PUSHBUTTON_BP2
 *
 * \section PWMC
 * - \ref PIN_PWMC_PWMH0
 * - \ref PIN_PWMC_PWML0
 * - \ref PIN_PWMC_PWMH1
 * - \ref PIN_PWMC_PWML1
 * - \ref PIN_PWMC_PWMH2
 * - \ref PIN_PWMC_PWML2
 * - \ref PIN_PWM_LED0
 * - \ref PIN_PWM_LED1
 * - \ref PIN_PWM_LED2
 * - \ref CHANNEL_PWM_LED0
 * - \ref CHANNEL_PWM_LED1
 * - \ref CHANNEL_PWM_LED2
 *
 * \section SPI0
 * - \ref PIN_SPI0_MISO
 * - \ref PIN_SPI0_MOSI
 * - \ref PIN_SPI0_SPCK
 * - \ref PIN_SPI0_NPCS2_PC14
 * - \ref PINS_SPI0
 *
 * \section  SSC
 * - \ref PIN_SSC_TD
 * - \ref PIN_SSC_TK
 * - \ref PIN_SSC_TF
 * - \ref PINS_SSC_CODEC
 *
 * \section  PCK0
 * - \ref PIN_PCK0
 *
 * \section TWI
 * - \ref TWI_V3XX
 * - \ref PIN_TWI_TWD0
 * - \ref PIN_TWI_TWCK0
 * - \ref PINS_TWI0
 * - \ref PIN_TWI_TWD1
 * - \ref PIN_TWI_TWCK1
 * - \ref PINS_TWI1
 *
 * \section USART0
 * - \ref PIN_USART0_RXD
 * - \ref PIN_USART0_TXD
 * - \ref PIN_USART0_CTS
 * - \ref PIN_USART0_RTS
 * - \ref PIN_USART0_SCK
 *
 * \section USART1
 * - \ref PIN_USART1_RXD
 * - \ref PIN_USART1_TXD
 * - \ref PIN_USART1_CTS
 * - \ref PIN_USART1_RTS
 * - \ref PIN_USART1_SCK
 *
 * \section USB
 * - \ref PIN_USB_VBUS
 */
//! ADC_AD0 pin definition.
#define PIN_ADC0_AD0 {1 << 21, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD1 pin definition.
#define PIN_ADC0_AD1 {1 << 30, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD2 pin definition.
#define PIN_ADC0_AD2 {1 << 3,  PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD3 pin definition.
#define PIN_ADC0_AD3 {1 << 4, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD4 pin definition.
#define PIN_ADC0_AD4 {1 << 15, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD5 pin definition.
#define PIN_ADC0_AD5 {1 << 16, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD6 pin definition.
#define PIN_ADC0_AD6 {1 << 17, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}
//! ADC_AD7 pin definition.
#define PIN_ADC0_AD7 {1 << 18, PIOC, ID_PIOC, PIO_INPUT, PIO_DEFAULT}

//! Pins ADC
#define PINS_ADC PIN_ADC0_AD0, PIN_ADC0_AD1, PIN_ADC0_AD2, PIN_ADC0_AD3, PIN_ADC0_AD4, PIN_ADC0_AD5, PIN_ADC0_AD6, PIN_ADC0_AD7

#define PINS_ADC12B_TRIG  PIO_PA2_IDX
#define PINS_ADC12B_TRIG_FLAG  (PIO_PERIPH_B | PIO_DEFAULT)
#define PINS_ADC_TRIG  PIO_PA17_IDX
#define PINS_ADC_TRIG_FLAG  (PIO_PERIPH_B | PIO_DEFAULT)

//! UART pins (UTXD0 and URXD0) definitions, PA9,10.
#define PINS_UART		(PIO_PA11A_URXD | PIO_PA12A_UTXD)
#define PINS_UART_FLAGS	(PIO_PERIPH_A | PIO_DEFAULT)

#define PINS_UART_MASK PIO_PA11A_URXD|PIO_PA12A_UTXD
#define PINS_UART_PIO PIOA
#define PINS_UART_ID ID_PIOA
#define PINS_UART_TYPE PIO_PERIPH_A
#define PINS_UART_ATTR PIO_DEFAULT

/** EBI Data Bus pins */
#define PIN_EBI_DATA_BUS_D0        PIO_PB9_IDX
#define PIN_EBI_DATA_BUS_D1        PIO_PB10_IDX
#define PIN_EBI_DATA_BUS_D2        PIO_PB11_IDX
#define PIN_EBI_DATA_BUS_D3        PIO_PB12_IDX
#define PIN_EBI_DATA_BUS_D4        PIO_PB13_IDX
#define PIN_EBI_DATA_BUS_D5        PIO_PB14_IDX
#define PIN_EBI_DATA_BUS_D6        PIO_PB15_IDX
#define PIN_EBI_DATA_BUS_D7        PIO_PB16_IDX
#define PIN_EBI_DATA_BUS_D8        PIO_PB25_IDX
#define PIN_EBI_DATA_BUS_D9        PIO_PB26_IDX
#define PIN_EBI_DATA_BUS_D10      PIO_PB27_IDX
#define PIN_EBI_DATA_BUS_D11      PIO_PB28_IDX
#define PIN_EBI_DATA_BUS_D12      PIO_PB29_IDX
#define PIN_EBI_DATA_BUS_D13      PIO_PB30_IDX
#define PIN_EBI_DATA_BUS_D14      PIO_PB31_IDX
#define PIN_EBI_DATA_BUS_FLAG1   PIO_PERIPH_A | PIO_PULLUP
#define PIN_EBI_DATA_BUS_D15      PIO_PB6_IDX
#define PIN_EBI_DATA_BUS_FLAG2  PIO_PERIPH_B | PIO_PULLUP
/** EBI NRD pin */
#define PIN_EBI_NRD                 PIO_PB19_IDX
#define PIN_EBI_NRD_FLAGS       PIO_PERIPH_A | PIO_PULLUP
#define PIN_EBI_NRD_MASK  1 << 19
#define PIN_EBI_NRD_PIO  PIOB
#define PIN_EBI_NRD_ID  ID_PIOB
#define PIN_EBI_NRD_TYPE PIO_PERIPH_A
#define PIN_EBI_NRD_ATTR PIO_PULLUP
/** EBI NWE pin */
#define PIN_EBI_NWE                  PIO_PB23_IDX
#define PIN_EBI_NWE_FLAGS       PIO_PERIPH_A | PIO_PULLUP
#define PIN_EBI_NWE_MASK  1 << 23
#define PIN_EBI_NWE_PIO  PIOB
#define PIN_EBI_NWE_ID  ID_PIOB
#define PIN_EBI_NWE_TYPE PIO_PERIPH_A
#define PIN_EBI_NWE_ATTR PIO_PULLUP
/** EBI NCS0 pin */
#define PIN_EBI_NCS0                PIO_PB20_IDX
#define PIN_EBI_NCS0_FLAGS     PIO_PERIPH_A | PIO_PULLUP
#define PIN_EBI_NCS0_MASK  1 << 20
#define PIN_EBI_NCS0_PIO  PIOB
#define PIN_EBI_NCS0_ID  ID_PIOB
#define PIN_EBI_NCS0_TYPE PIO_PERIPH_A
#define PIN_EBI_NCS0_ATTR PIO_PULLUP
/** EBI address bus pins  */
#define PIN_EBI_ADDR_BUS_A0 PIO_PB7_IDX
#define PIN_EBI_ADDR_BUS_NBS0 PIO_PB7_IDX
#define PIN_EBI_ADDR_BUS_A1     PIO_PB8_IDX
#define PIN_EBI_ADDR_BUS_FLAG1  PIO_PERIPH_B | PIO_PULLUP
#define PIN_EBI_ADDR_BUS_NBS1 PIO_PC15_IDX
#define PIN_EBI_ADDR_BUS_A2     PIO_PC0_IDX
#define PIN_EBI_ADDR_BUS_A3     PIO_PC1_IDX
#define PIN_EBI_ADDR_BUS_A4     PIO_PC2_IDX
#define PIN_EBI_ADDR_BUS_A5     PIO_PC3_IDX
#define PIN_EBI_ADDR_BUS_A6     PIO_PC4_IDX
#define PIN_EBI_ADDR_BUS_A7     PIO_PC5_IDX
#define PIN_EBI_ADDR_BUS_A8     PIO_PC6_IDX
#define PIN_EBI_ADDR_BUS_A9     PIO_PC7_IDX
#define PIN_EBI_ADDR_BUS_A10   PIO_PC8_IDX
#define PIN_EBI_ADDR_BUS_A11   PIO_PC9_IDX
#define PIN_EBI_ADDR_BUS_A12   PIO_PC10_IDX
#define PIN_EBI_ADDR_BUS_A13   PIO_PC11_IDX
#define PIN_EBI_ADDR_BUS_A14   PIO_PC20_IDX
#define PIN_EBI_ADDR_BUS_A15   PIO_PC21_IDX
#define PIN_EBI_ADDR_BUS_A16   PIO_PC22_IDX
#define PIN_EBI_ADDR_BUS_A17   PIO_PC23_IDX
#define PIN_EBI_ADDR_BUS_A18   PIO_PC24_IDX
#define PIN_EBI_ADDR_BUS_A19   PIO_PC25_IDX
#define PIN_EBI_ADDR_BUS_A20   PIO_PC26_IDX
#define PIN_EBI_ADDR_BUS_FLAG2  PIO_PERIPH_A | PIO_PULLUP
/** EBI pin for LCD CS */
#define PIN_EBI_NCS2                 PIO_PC16_IDX
#define PIN_EBI_NCS2_FLAGS      PIO_PERIPH_A | PIO_PULLUP
#define PIN_EBI_NCS2_MASK  1 << 16
#define PIN_EBI_NCS2_PIO  PIOC
#define PIN_EBI_NCS2_ID  ID_PIOC
#define PIN_EBI_NCS2_TYPE PIO_PERIPH_A
#define PIN_EBI_NCS2_ATTR PIO_PULLUP
/** EBI pin for LCD RS */
#define PIN_EBI_LCD_RS                PIO_PB8_IDX
#define PIN_EBI_LCD_RS_FLAGS     PIO_PERIPH_B | PIO_PULLUP
#define PIN_EBI_LCD_RS_MASK  1 << 8
#define PIN_EBI_LCD_RS_PIO  PIOB
#define PIN_EBI_LCD_RS_ID  ID_PIOB
#define PIN_EBI_LCD_RS_TYPE PIO_PERIPH_B
#define PIN_EBI_LCD_RS_ATTR PIO_PULLUP

/** Backlight pin definition. */
#define BOARD_BACKLIGHT                PIO_PC19_IDX
#define BOARD_BACKLIGHT_FLAG       PIO_OUTPUT_0 | PIO_DEFAULT

/** Define HX8347A base address. */
#define BOARD_LCD_BASE              0x62000000
/** Define HX8347A register select signal. */
#define BOARD_LCD_RS                (1 << 1)

/** Define MCP980X TWI instance. */
#define BOARD_MCP980X_TWI_INSTANCE  (TWI0)
/** Define MCP980X base address. */
#define BOARD_MCP980X_ADDR          (0x48u)

//! LED #0 pin definition (BLUE).
#define LED_0_NAME    "green LED D2"
#define LED0_GPIO     (PIO_PB0_IDX)
#define LED0_FLAGS    (PIO_TYPE_PIO_OUTPUT_1 | PIO_DEFAULT)

#define PIN_LED_0   {1 << 0, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_LED_0_MASK 1 << 0
#define PIN_LED_0_PIO PIOB
#define PIN_LED_0_ID ID_PIOB
#define PIN_LED_0_TYPE PIO_OUTPUT_0
#define PIN_LED_0_ATTR PIO_DEFAULT

//! LED #1 pin definition (GREEN)
#define LED_1_NAME    "green LED D3"
#define LED1_GPIO     (PIO_PB1_IDX)
#define LED1_FLAGS    (PIO_TYPE_PIO_OUTPUT_1 | PIO_DEFAULT)

#define PIN_LED_1   {1 << 1, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_1_MASK 1 << 1
#define PIN_LED_1_PIO PIOB
#define PIN_LED_1_ID ID_PIOB
#define PIN_LED_1_TYPE PIO_OUTPUT_1
#define PIN_LED_1_ATTR PIO_DEFAULT

//! LED #2 pin definition (RED).
#define LED2_GPIO 		(PIO_PB2_IDX)
#define LED2_GPIO_FLAGS (PIO_TYPE_PIO_OUTPUT_1 | PIO_DEFAULT)

#define PIN_LED_2   {1 << 2, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
#define PIN_LED_2_MASK 1 << 2
#define PIN_LED_2_PIO PIOB
#define PIN_LED_2_ID ID_PIOB
#define PIN_LED_2_TYPE PIO_OUTPUT_1
#define PIN_LED_2_ATTR PIO_DEFAULT

//! List of all LEDs definitions.
#define PINS_LEDS   PIN_LED_0, PIN_LED_1, PIN_LED_2

//!// MCI pins definition.
#define PINS_MCI  {0x1f8, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_PULLUP}, \
                      {1 << 3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
//! MCI pin DAT0 (busy)
#define PIN_MCI_DAT0 \
    {PIO_PA5, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_PULLUP}
//! MCI pin Card Detect
#define PIN_MCI_CD \
    {PIO_PA25, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP}

//! Push button #0 definition.
#define PIN_PUSHBUTTON_1    {1 << 18, PIOA, ID_PIOA, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}
//! Push button #1 definition.
#define PIN_PUSHBUTTON_2    {1 << 19, PIOA, ID_PIOA, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}

//! Push button #0 definition.
#define PUSHBUTTON_1_NAME    "USR-LEFT"
#define GPIO_PUSH_BUTTON_1    (PIO_PA18_IDX)
#define GPIO_PUSH_BUTTON_1_FLAGS    (PIO_INPUT | PIO_PULLUP | PIO_DEGLITCH | PIO_IT_RISE_EDGE)

#define PIN_PUSHBUTTON_1    {1 << 18, PIOA, ID_PIOA, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}
#define PIN_PUSHBUTTON_1_MASK 1 << 18
#define PIN_PUSHBUTTON_1_PIO PIOA
#define PIN_PUSHBUTTON_1_ID ID_PIOA
#define PIN_PUSHBUTTON_1_TYPE PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR PIO_DEGLITCH | PIO_PULLUP | PIO_IT_RISE_EDGE

//! Push button #1 definition.
#define PUSHBUTTON_2_NAME    "USR-RIGHT"
#define GPIO_PUSH_BUTTON_2    (PIO_PA19_IDX)
#define GPIO_PUSH_BUTTON_2_FLAGS    (PIO_INPUT | PIO_PULLUP | PIO_DEGLITCH | PIO_IT_FALL_EDGE)

#define PIN_PUSHBUTTON_2    {1 << 19, PIOA, ID_PIOA, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}
#define PIN_PUSHBUTTON_2_MASK 1 << 19
#define PIN_PUSHBUTTON_2_PIO PIOA
#define PIN_PUSHBUTTON_2_ID ID_PIOA
#define PIN_PUSHBUTTON_2_TYPE PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR PIO_DEGLITCH | PIO_PULLUP | PIO_IT_FALL_EDGE
//! List of all push button definitions.
#define PINS_PUSHBUTTONS    PIN_PUSHBUTTON_1, PIN_PUSHBUTTON_2
//! Push button #1 index.
#define PUSHBUTTON_BP1   0
//! Push button #2 index.
#define PUSHBUTTON_BP2   1
//! Simulated joystick LEFT index.
#define JOYSTICK_LEFT    0
//! Simulated joystick RIGHT index.
#define JOYSTICK_RIGHT   1

#define PIN_TC0_TIOA0		(PIO_PA1_IDX)
#define PIN_TC0_TIOA0_FLAGS	(PIO_PERIPH_A | PIO_DEFAULT)

#define PIN_TC0_TIOA1		(PIO_PB5_IDX)
#define PIN_TC0_TIOA1_FLAGS	(PIO_PERIPH_A | PIO_DEFAULT)

#define PIN_TC0_TIOA1_PIO     PIOB
#define PIN_TC0_TIOA1_MASK  PIO_PB5
#define PIN_TC0_TIOA1_ID      ID_PIOB
#define PIN_TC0_TIOA1_TYPE   PIO_PERIPH_A
#define PIN_TC0_TIOA1_ATTR   PIO_DEFAULT

#define PIN_TC0_TIOA2		(PIO_PA30_IDX)
#define PIN_TC0_TIOA2_FLAGS	(PIO_INPUT | PIO_DEFAULT)

#define PIN_TC0_TIOA2_PIO     PIOA
#define PIN_TC0_TIOA2_MASK  PIO_PA30
#define PIN_TC0_TIOA2_ID      ID_PIOA
#define PIN_TC0_TIOA2_TYPE   PIO_INPUT
#define PIN_TC0_TIOA2_ATTR   PIO_DEFAULT

//! PWMC PWM0 pin definition.
#define PIN_PWMC_PWMH0  {1 << 0, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_PWMC_PWMH0_TRIG   PIO_PB0_IDX
#define PIN_PWMC_PWMH0_TRIG_FLAG   PIO_PERIPH_A | PIO_DEFAULT
#define PIN_PWMC_PWML0  {1 << 7, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
//! PWMC PWM1 pin definition.
#define PIN_PWMC_PWMH1  {1 << 1, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_PWMC_PWML1  {1 << 8, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
//! PWMC PWM2 pin definition.
#define PIN_PWMC_PWMH2  {1 << 2, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_PWMC_PWML2  {1 << 9, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
//! PWM pin definition for LED0
#define PIN_PWM_LED0 PIN_PWMC_PWMH0
//! PWM pin definition for LED1
#define PIN_PWM_LED1 PIN_PWMC_PWMH1
//! PWM pin definition for LED2
#define PIN_PWM_LED2 PIN_PWMC_PWMH2
//! PWM channel for LED0
#define CHANNEL_PWM_LED0 0
//! PWM channel for LED1
#define CHANNEL_PWM_LED1 1
//! PWM channel for LED2
#define CHANNEL_PWM_LED2 2

//! PWM LED0 pin definitions.
#define PIN_PWM_LED0_GPIO    PIO_PB0_IDX
#define PIN_PWM_LED0_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_LED0_CHANNEL 0

//! PWM LED1 pin definitions.
#define PIN_PWM_LED1_GPIO    PIO_PB1_IDX
#define PIN_PWM_LED1_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_LED1_CHANNEL 1

//! PWM LED2 pin definitions.
#define PIN_PWM_LED2_GPIO    PIO_PB2_IDX
#define PIN_PWM_LED2_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

//! SPI0 MISO pin definition.
#define PIN_SPI0_MISO  {1 << 13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
//! SPI0 MOSI pin definition.
#define PIN_SPI0_MOSI  {1 << 14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
//! SPI0 SPCK pin definition.
#define PIN_SPI0_SPCK  {1 << 15, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
//! SPI0 chip select 0 pin definition.
#define PIN_SPI0_NPCS0_PA16  {1 <<  16, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_SPI0_NPCS0  PIN_SPI0_NPCS0_PA16
//! SPI0 chip select 2 pin definition.
#define PIN_SPI0_NPCS2_PC14  {1 <<  14, PIOC, ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT}
//! List of SPI0 pin definitions (MISO, MOSI & SPCK).
#define PINS_SPI0      PIN_SPI0_MISO, PIN_SPI0_MOSI, PIN_SPI0_SPCK
/** SPI MISO pin definition. */
#define SPI_MISO_GPIO 		(PIO_PA13_IDX)
#define SPI_MISO_FLAGS (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO 		(PIO_PA14_IDX)
#define SPI_MOSI_FLAGS (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO 		(PIO_PA15_IDX)
#define SPI_SPCK_FLAGS (PIO_PERIPH_A | PIO_DEFAULT)

/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO            (PIO_PA16_IDX)
#define SPI_NPCS0_FLAGS           (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI chip select 1 pin definition. (multiple configurations are possible) */
#define SPI_NPCS1_PA0_GPIO 		  (PIO_PA0_IDX)
#define SPI_NPCS1_PA0_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS1_PC3_GPIO 	      (PIO_PC3_IDX)
#define SPI_NPCS1_PC3_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS1_PC19_GPIO 	  (PIO_PC19_IDX)
#define SPI_NPCS1_PC19_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
/** SPI chip select 2 pin definition. (multiple configurations are possible) */
#define SPI_NPCS2_PA1_GPIO 	      (PIO_PA1_IDX)
#define SPI_NPCS2_PA1_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS2_PC4_GPIO 	      (PIO_PC4_IDX)
#define SPI_NPCS2_PC4_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS2_PC14_GPIO 	  (PIO_PC14_IDX)
#define SPI_NPCS2_PC14_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
/** SPI chip select 3 pin definition. (multiple configurations are possible) */
#define SPI_NPCS3_PA19_GPIO 	  (PIO_PA19_IDX)
#define SPI_NPCS3_PA19_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS3_PC5_GPIO 		  (PIO_PC5_IDX)
#define SPI_NPCS3_PC5_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)

//! SSC pins definition.
#define PIN_SSC_TD      {0x1 << 26, PIOA,ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_SSC_TK      {0x1 << 28, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_SSC_TF      {0x1 << 30, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PINS_SSC_CODEC       PIN_SSC_TD,  PIN_SSC_TK, PIN_SSC_TF

//! PCK0
#define PIN_PCK0		(PIO_PA21_IDX)
#define PIN_PCK0_FLAGS	(PIO_PERIPH_B | PIO_DEFAULT)

#define PIN_PCK_0_MASK PIO_PA21
#define PIN_PCK_0_PIO PIOA
#define PIN_PCK_0_ID ID_PIOA
#define PIN_PCK_0_TYPE PIO_PERIPH_B
#define PIN_PCK_0_ATTR PIO_DEFAULT

/** Address for transferring command bytes to the nandflash. */
#define BOARD_NF_COMMAND_ADDR   0x60400000
/** Address for transferring address bytes to the nandflash. */
#define BOARD_NF_ADDRESS_ADDR   0x60200000
/** Address for transferring data bytes to the nandflash. */
#define BOARD_NF_DATA_ADDR      0x60000000

//! TWI pins definition.
#define TWI_V3XX
#define PIN_TWI_TWD0    {0x1 << 9, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_TWI_TWCK0    {0x1 << 10, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PINS_TWI0     PIN_TWI_TWD0, PIN_TWI_TWCK0
#define PIN_TWI_TWD1    {0x1 << 24, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_TWI_TWCK1    {0x1 << 25, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PINS_TWI1     PIN_TWI_TWD1, PIN_TWI_TWCK1

/** TWI0 pins definition */
#define TWI0_DATA_GPIO   PIO_PA9_IDX
#define TWI0_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define TWI0_CLK_GPIO    PIO_PA10_IDX
#define TWI0_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

/** TWI1 pins definition */
#define TWI1_DATA_GPIO   PIO_PA24_IDX
#define TWI1_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define TWI1_CLK_GPIO    PIO_PA25_IDX
#define TWI1_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

//! USART0
#define PIN_USART0_RXD    {0x1 << 19, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART0_RXD_IDX        (PIO_PA19_IDX)
#define PIN_USART0_RXD_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART0_TXD    {0x1 << 18, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART0_TXD_IDX        (PIO_PA18_IDX)
#define PIN_USART0_TXD_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART0_CTS    {0x1 << 8, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART0_CTS_IDX        (PIO_PA8_IDX)
#define PIN_USART0_CTS_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART0_RTS    {0x1 << 7, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART0_RTS_IDX        (PIO_PA7_IDX)
#define PIN_USART0_RTS_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART0_SCK    {0x1 << 17, PIOA,ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART0_SCK_IDX        (PIO_PA17_IDX)
#define PIN_USART0_SCK_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)

//! USART1
#define PIN_USART1_RXD    {0x1 << 21, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_RXD_IDX        (PIO_PA21_IDX)
#define PIN_USART1_RXD_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART1_TXD    {0x1 << 20, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_TXD_IDX        (PIO_PA20_IDX)
#define PIN_USART1_TXD_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART1_CTS    {0x1 << 23, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART1_CTS_IDX        (PIO_PA23_IDX)
#define PIN_USART1_CTS_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_USART1_RTS    {0x1 << 22, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART1_RTS_IDX        (PIO_PA22_IDX)
#define PIN_USART1_RTS_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_USART1_EN     {0x1 << 24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_USART1_EN_IDX         (PIO_PA24_IDX)
#define PIN_USART1_EN_FLAGS       (PIO_OUTPUT_0 | PIO_DEFAULT)
#define PIN_USART1_SCK    {0x1 << 24, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART1_SCK_IDX        (PIO_PA24_IDX)
#define PIN_USART1_SCK_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)

//! USB VBus monitoring pin definition.
#define PIN_USB_VBUS    {1 << 0, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/** USART RX pin for application */
#define BOARD_PIN_USART_RXD        PIN_USART1_RXD
/** USART TX pin for application */
#define BOARD_PIN_USART_TXD        PIN_USART1_TXD
/** USART CTS pin for application */
#define BOARD_PIN_USART_CTS        PIN_USART1_CTS
/** USART RTS pin for application */
#define BOARD_PIN_USART_RTS        PIN_USART1_RTS
/** USART Base for application */
#define BOARD_USART_BASE           USART1
/** USART ID for application */
#define BOARD_ID_USART             ID_USART1

#define CONSOLE_UART			   UART
//------------------------------------------------------------------------------
#define CONSOLE_UART_ID            ID_UART

/* RE pin. */
#define PIN_RE_IDX                 PIN_USART1_CTS_IDX
#define PIN_RE_FLAGS               (PIO_OUTPUT_0 | PIO_DEFAULT)

/* IRDA SD pin. */
#define PIN_IRDA_SD_IDX            PIN_USART1_CTS_IDX
#define PIN_IRDA_SD_FLAGS          (PIO_OUTPUT_1 | PIO_DEFAULT)

/* TXD pin configuration. */
#define PIN_USART_TXD_IDX          PIN_USART1_TXD_IDX
#define PIN_USART_TXD_FLAGS        (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_USART_TXD_IO_FLAGS     (PIO_OUTPUT_0 | PIO_DEFAULT)

/* ISO7816 example relate PIN definition. */
#define ISO7816_USART_ID           ID_USART0
#define ISO7816_USART              USART0
#define PIN_ISO7816_RST_IDX        PIO_PA15_IDX
#define PIN_ISO7816_RST_FLAG       (PIO_OUTPUT_0 | PIO_DEFAULT) 

#endif  // _SAM3U_EK_H_
