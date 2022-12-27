/**
 * \file
 *
 * \brief lcd controller HX8347A example.
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

/**
 * \mainpage lcd controller HX8347A example
 *
 * \section Purpose
 *
 * This example demonstrates how to configure lcd controller HX8347A
 * to control the LCD on the board.
 *
 * \section Requirements
 *
 * This example can be used with SAM3U and SAM3X evaluation kits now.
 *
 * \section Description
 *
 * This example first configures HX8347A for access the LCD controller,
 * then initialize the LCD, finally draw some text, image, basic shapes (line,
 * rectangle, circle) on LCD.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Some text, image and basic shapes should be displayed on the LCD.
 *
 */

#include "board.h"
#include "sysclk.h"
#include "hx8347a.h"
#include "gpio.h"
#include "pio.h"
#include "pmc.h"
#include "smc.h"
#include "aat31xx.h"

struct hx8347a_opt_t hx8347a_display_opt;

/* Convert 24-bits color to 16-bits color */
static hx8347a_color_t rgb24_to_rgb16(uint32_t dw_color)
{
	hx8347a_color_t result_color;
	result_color = (((dw_color >> 8) & 0xF800) |
			((dw_color >> 5) & 0x7E0) | ((dw_color >> 3) & 0x1F));
	return result_color;
}

/**
 * \brief Application entry point for smc_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	sysclk_init();
	board_init();

	/* Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/* Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, CONF_BOARD_HX8347A_LCD_CS, SMC_SETUP_NWE_SETUP(1)
			| SMC_SETUP_NCS_WR_SETUP(1)
			| SMC_SETUP_NRD_SETUP(9)
			| SMC_SETUP_NCS_RD_SETUP(9));
	smc_set_pulse_timing(SMC, CONF_BOARD_HX8347A_LCD_CS, SMC_PULSE_NWE_PULSE(4)
			| SMC_PULSE_NCS_WR_PULSE(4)
			| SMC_PULSE_NRD_PULSE(36)
			| SMC_PULSE_NCS_RD_PULSE(36));
	smc_set_cycle_timing(SMC, CONF_BOARD_HX8347A_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
			| SMC_CYCLE_NRD_CYCLE(45));
	smc_set_mode(SMC, CONF_BOARD_HX8347A_LCD_CS, SMC_MODE_READ_MODE
			| SMC_MODE_WRITE_MODE | SMC_MODE_DBW_BIT_16);

	/* Initialize display parameter */
	hx8347a_display_opt.dw_width = HX8347A_LCD_WIDTH;
	hx8347a_display_opt.dw_height = HX8347A_LCD_HEIGHT;
	hx8347a_display_opt.foreground_color = rgb24_to_rgb16(COLOR_BLACK);
	hx8347a_display_opt.background_color = rgb24_to_rgb16(COLOR_WHITE);

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	if(hx8347a_init(&hx8347a_display_opt)){
		puts("Read HX8347A chip ID error, please check the configuration.\r");
	}

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_WHITE));
	hx8347a_draw_filled_rectangle(0, 0, HX8347A_LCD_WIDTH - 1, HX8347A_LCD_HEIGHT - 1);

	/* Turn on LCD */
	hx8347a_display_on();

	/* Draw text, image and basic shapes on the LCD */
	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_BLACK));
	hx8347a_draw_string(10, 20, (uint8_t *)"HX8347A LCD Example");

	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_RED));
	hx8347a_draw_circle(60, 120, 40);
	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_GREEN));
	hx8347a_draw_circle(120, 120, 40);
	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_BLUE));
	hx8347a_draw_circle(180, 120, 40);

	hx8347a_set_foreground_color(rgb24_to_rgb16(COLOR_VIOLET));
	hx8347a_draw_rectangle(40, 200, 200, 260);
	hx8347a_draw_line(40, 200, 200, 260);
	hx8347a_draw_line(40, 260, 200, 200);
	hx8347a_draw_line(40, 230, 200, 230);
	hx8347a_draw_line(120, 200, 120, 260);

	while (1) {
		/* Busy-wait forever. */
	}
}

