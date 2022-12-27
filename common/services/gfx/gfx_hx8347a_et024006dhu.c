/**
 * \file
 *
 * \brief Graphic service settings for the ET024006DHU panel using the HX8347A
 * display controller
 *
 * This files includes the correct header files for the grapics service
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

/**
 * \ingroup gfx_hx8347a_et024006dhu
 * @{
 */

#include "gfx.h"
#include "gfx_hx8347a_et024006dhu.h"
#include "hx8347a.h"
#include "hx8347a_regs.h"

gfx_coord_t gfx_height, gfx_width;
gfx_coord_t gfx_min_x, gfx_min_y;
gfx_coord_t gfx_max_x, gfx_max_y;

void gfx_hx8347a_set_orientation(uint8_t flags)
{
	hx8347a_set_orientation(flags);

	/* Switch width and height if XY is switched. */
	if ((flags & GFX_SWITCH_XY) != 0x00) {
		gfx_width = GFX_PANELHEIGHT;
		gfx_height = GFX_PANELWIDTH;
	} else {
		gfx_width = GFX_PANELWIDTH;
		gfx_height = GFX_PANELHEIGHT;
	}

#ifdef CONF_GFX_USE_CLIPPING
	/* Reset clipping region. */
	gfx_set_clipping(0, 0, gfx_width - 1, gfx_height - 1);
#endif
}

gfx_color_t gfx_hx8347a_color(uint8_t r, uint8_t g, uint8_t b)
{
	gfx_color_t color;
	uint16_t red = r >> 3;
	uint16_t green = g >> 2;
	uint16_t blue = b >> 3;

	/* Stuff into one 16-bit word. */
	red <<= (5 + 6);
	green <<= 5;
	color = red | green | blue;

	/* Convert to big endian, to fit the display data format. */
	color = (color >> 8) | (color << 8);

	return color;
}

gfx_color_t gfx_hx8347a_get_pixel(gfx_coord_t x, gfx_coord_t y)
{
	gfx_color_t color;

#ifdef CONF_GFX_USE_CLIPPING
	if ((x < gfx_min_x) || (x > gfx_max_x) ||
			(y < gfx_min_y) || (y > gfx_max_y)) {
		return GFX_COLOR_INVALID;
	}

#endif

	/* Set up draw area and read the three bytes of pixel data. */
	gfx_set_limits(x, y, x, y);
	color = hx8347a_read_gram();

	return color;
}

void gfx_hx8347a_draw_pixel(gfx_coord_t x, gfx_coord_t y, gfx_color_t color)
{
#ifdef CONF_GFX_USE_CLIPPING
	if ((x < gfx_min_x) || (x > gfx_max_x) ||
			(y < gfx_min_y) || (y > gfx_max_y)) {
		return;
	}

#endif

	/* Set up draw area and write the two bytes of pixel data. */
	gfx_set_limits(x, y, x, y);
	hx8347a_write_gram(color);
}

void gfx_hx8347a_draw_line_pixel(gfx_coord_t x, gfx_coord_t y,
		gfx_color_t color)
{
#ifdef CONF_GFX_USE_CLIPPING
	if ((x < gfx_min_x) || (x > gfx_max_x) ||
			(y < gfx_min_y) || (y > gfx_max_y)) {
		return;
	}

#endif

	/* Set up top left corner of area and write the two bytes of */
	/* pixel data.  Bottom left corner is already set to max_x/y. */
	gfx_set_top_left_limit(x, y);
	hx8347a_write_gram(color);
}

void gfx_hx8347a_init(void)
{
	/* initialize globals */
	gfx_width = GFX_PANELWIDTH;
	gfx_height = GFX_PANELHEIGHT;

	hx8347a_init();
	hx8347a_backlight_on();

	/* Set clipping area to whole screen initially */
	gfx_set_clipping(0, 0, GFX_PANELWIDTH, GFX_PANELHEIGHT);

	gfx_draw_filled_rect(0, 0, GFX_PANELWIDTH, GFX_PANELHEIGHT,
			GFX_COLOR(0xFF, 0xFF, 0xFF));
}

/**
 *@}
 */
