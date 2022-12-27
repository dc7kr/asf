/**
 * \file
 *
 * \brief lcd controller ili9225 example.
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
 * \mainpage lcd controller ili9225 Example
 *
 * \section Purpose
 *
 * This example demonstrates how to configure lcd controller ili9225
 * to control the LCD on the board.
 *
 * \section Requirements
 *
 * This package can be used with SAM3N evaluation kits.
 *
 * \section Description
 *
 * This example first configure ili9225 for access the LCD controller,
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
#include "ili9225.h"
#include "aat31xx.h"

struct ili9225_opt_t ili9225_display_opt;

/**
 * \brief Override SPI handler.
 */
void SPI_Handler(void)
{
	ili9225_spi_handler();
}

/**
 * \brief Application entry point for ili9225_spi_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	sysclk_init();
	board_init();

	/* Initialize display parameter */
	ili9225_display_opt.dw_width = BOARD_LCD_WIDTH;
	ili9225_display_opt.dw_height = BOARD_LCD_HEIGHT;
	ili9225_display_opt.foreground_color = COLOR_BLACK;
	ili9225_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9225_init(&ili9225_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	/* Turn on LCD */
	ili9225_display_on();

	/* Draw filled rectangle with white color */
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT);

	/* Draw text and basic shapes on the LCD */
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10, 20, (uint8_t *)"ili9225_lcd");

	/* Draw three circle with red, green and blue color */
	ili9225_set_foreground_color(COLOR_RED);
	ili9225_draw_circle(60, 80, 30);
	ili9225_set_foreground_color(COLOR_GREEN);
	ili9225_draw_circle(60, 120, 30);
	ili9225_set_foreground_color(COLOR_BLUE);
	ili9225_draw_circle(60, 160, 30);

	/* Draw one line */
	ili9225_draw_line(0, 0, 176, 220);

	while (1) {
	}
}
