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

#include "gfx.h"

PROGMEM_DECLARE(gfx_color_t, smiley_data[400]) = {
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0x22ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x43ff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0x22ff, 0x21ff, 0xb5ff, 0x91ff,
	0xfcff, 0x46ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x43ff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x22ff,
	0x20ff, 0x6aff, 0xfeff, 0x48ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x43ff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0x22ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x43ff, 0xffff, 0xffff, 0xffff, 0xffff, 0x20ff, 0x20ff,
	0xffff, 0x7def, 0x0000, 0xc08b, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x7def, 0x7def, 0x0000, 0xc08b, 0x20ff, 0x20ff, 0xffff, 0xffff,
	0xffff, 0x22ff, 0x20ff, 0x20ff, 0xffff, 0x34a5, 0x0000, 0xa039,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x7def, 0x7def, 0x0000, 0xa039,
	0x20ff, 0x20ff, 0x43ff, 0xffff, 0xffff, 0x20ff, 0x20ff, 0x20ff,
	0x0000, 0x0000, 0x0000, 0x2073, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x0000, 0x0000, 0x0000, 0x2073, 0x20ff, 0x20ff, 0x20ff, 0xffff,
	0xffff, 0x20ff, 0x20ff, 0x20ff, 0xc08b, 0xa039, 0x2073, 0x80ee,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0xc08b, 0xa039, 0x2073, 0x80ee,
	0x20ff, 0x20ff, 0x20ff, 0xffff, 0xffff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0xffff,
	0xffff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0xffff, 0xffff, 0x20ff, 0x20ff, 0x20ff,
	0x0000, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x0000, 0x20ff, 0x20ff, 0xffff,
	0xffff, 0x20ff, 0x20ff, 0x20ff, 0x0000, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x0000,
	0x0000, 0x20ff, 0x20ff, 0xffff, 0xffff, 0x21ff, 0x20ff, 0x20ff,
	0x0000, 0x0000, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x0000, 0x20ff, 0x20ff, 0x41ff, 0xffff,
	0xffff, 0xffff, 0x20ff, 0x20ff, 0x20ff, 0x0000, 0x0000, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x0000, 0x0000,
	0x20ff, 0x20ff, 0xffff, 0xffff, 0xffff, 0xffff, 0x21ff, 0x20ff,
	0x20ff, 0x20ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x20ff, 0x20ff,
	0x0000, 0x0000, 0x0000, 0x20ff, 0x20ff, 0x41ff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0x21ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x20ff, 0x20ff, 0x20ff,
	0x41ff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0x21ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x41ff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x21ff, 0x20ff,
	0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x20ff, 0x41ff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
	0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
};
struct gfx_bitmap smiley = {
	.width = 20, .height = 20 , .type = GFX_BITMAP_PROGMEM,
	.data.progmem = (gfx_color_t PROGMEM_PTR_T)smiley_data};
