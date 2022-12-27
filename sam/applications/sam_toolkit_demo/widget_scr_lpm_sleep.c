/**
 * \file
 *
 * \brief Sleep mode widget for SAM toolkit demo application.
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
 */
 
#include "demo.h"
#include <asf.h>
#include "string.h"
#include "bitmaps.h"

/** Widget application context */
struct widget_context_lpm_sleep {
	/** Pointer to frame for application */
	struct wtk_basic_frame *frame;
	/** Frame background object */
	struct gfx_bitmap frame_bg;
	/** Pointer to frame for application */
	struct wtk_basic_frame *logo_frame;
	/** Label widget for user selection feedback */
	struct wtk_label *lbl_board_name;
	/** Label widget for user selection feedback */
	struct wtk_label *lbl_func_name;
	/** Pointer to frame for sleep frame */
	struct wtk_basic_frame *lpm_sleep_frame;
	/** Label widget for indications */
	struct wtk_label *lbl_indication[2];
};

/** Wakeup PIN */
static const uint32_t sleep_wakeup_pin = LPM_WAKEUP_INPUT_ID;

/** Statically allocated context pointer */
static struct widget_context_lpm_sleep *widget_ctx_lpm_sleep;

static char demo_func_name[] = "Low-Power Mode";

static char str_sleep_mode[] = "Sleep Mode";
static char str_sleep_ret[] = "Touch the screen\n to wake-up";

/** Sleep bitmap */
static struct gfx_bitmap win_lpm_sleep_icon = {
	.width = 64, .height = 64,
	.type = GFX_BITMAP_EXT,
	.data.custom = DEMO_BMP_SLEEP_MODE
};

/** Logo bitmap */
static struct gfx_bitmap win_root_logo_icon = {
	.width = 87, .height = 40,
	.type = GFX_BITMAP_EXT,
	.data.custom = DEMO_BMP_LOGO_ATMEL
};

/**
 * \brief Setup sleep mode widget.
 *
 * Allocates memory for the application context, and creates all widgets that
 * make up its interface. If memory allocation or widget creation fails, the
 * application exits immediately.
 */
static void app_widget_launch_lpm_sleep(void)
{
	struct win_window *parent;
	struct win_area area;

	/* Create a new context for the GUI, clear new context state */
	widget_ctx_lpm_sleep
		= membag_alloc(sizeof(struct widget_context_lpm_sleep));

	/* Set the background information for the GUI window */
	widget_ctx_lpm_sleep->frame_bg.type = GFX_BITMAP_SOLID;
	widget_ctx_lpm_sleep->frame_bg.data.color = GFX_COLOR(255, 255, 255);

	/* Set the area for the GUI window */
	area = win_get_attributes(win_get_root())->area;

	/* Create and show the main GUI frame */
	widget_ctx_lpm_sleep->frame = wtk_basic_frame_create(
			win_get_root(), &area, &widget_ctx_lpm_sleep->frame_bg, NULL,
			NULL, NULL);
	if (!widget_ctx_lpm_sleep->frame) {
		goto error_frame;
	}

	parent = wtk_basic_frame_as_child(widget_ctx_lpm_sleep->frame);
	win_show(parent);

	/* Set the area for logo frame */
	area.pos.x = 0;
	area.pos.y = 0;
	area.size.x = 87;
	area.size.y = 40;
	/* Create and show the main GUI frame */
	widget_ctx_lpm_sleep->logo_frame = wtk_basic_frame_create(
			parent, &area, &win_root_logo_icon, NULL,
			NULL, NULL);
	if (!widget_ctx_lpm_sleep->frame) {
		goto error_frame;
	}

	win_show(wtk_basic_frame_as_child(widget_ctx_lpm_sleep->logo_frame));

	/* Set area for board name */
	area.pos.x = 100;
	area.pos.y = 0;
	wtk_label_size_hint(&area.size, demo_board_name);

	/* Create label inside frame */
	widget_ctx_lpm_sleep->lbl_board_name = wtk_label_create(
			parent, &area, demo_board_name,
			GFX_COLOR(0, 0, 0), NULL, false);
	if (!widget_ctx_lpm_sleep->lbl_board_name) {
		goto error_widget;
	}

	win_show(wtk_label_as_child(widget_ctx_lpm_sleep->lbl_board_name));

	/* Set area for function name */
	area.pos.x = 100;
	area.pos.y = 20;
	wtk_label_size_hint(&area.size, demo_func_name);

	/* Create label inside frame */
	widget_ctx_lpm_sleep->lbl_func_name = wtk_label_create(
			parent, &area, demo_func_name,
			GFX_COLOR(0, 0, 0), NULL, false);
	if (!widget_ctx_lpm_sleep->lbl_func_name) {
		goto error_widget;
	}

	win_show(wtk_label_as_child(widget_ctx_lpm_sleep->lbl_func_name));

	/* Set area for sleep icon */
	area.pos.x = 91;
	area.pos.y = 66;
	area.size.x = 64;
	area.size.y = 64;
	/* Create and show the main GUI frame */
	widget_ctx_lpm_sleep->lpm_sleep_frame = wtk_basic_frame_create(
			parent, &area, &win_lpm_sleep_icon, NULL,
			NULL, NULL);
	if (!widget_ctx_lpm_sleep->lpm_sleep_frame) {
		goto error_frame;
	}

	win_show(wtk_basic_frame_as_child(widget_ctx_lpm_sleep->lpm_sleep_frame));

	/* Set area for sleep mode label */
	area.pos.x = 66;
	area.pos.y = 150;
	wtk_label_size_hint(&area.size, str_sleep_mode);

	/* Create label inside frame */
	widget_ctx_lpm_sleep->lbl_indication[0] = wtk_label_create(
			parent, &area, str_sleep_mode,
			GFX_COLOR(0, 0, 0), NULL, false);
	if (!widget_ctx_lpm_sleep->lbl_indication[0]) {
		goto error_widget;
	}

	win_show(wtk_label_as_child(widget_ctx_lpm_sleep->lbl_indication[0]));

	/* Set area for sleep return label */
	area.pos.x = 42;
	area.pos.y = 240;
	wtk_label_size_hint(&area.size, str_sleep_ret);

	/* Create label inside frame */
	widget_ctx_lpm_sleep->lbl_indication[1] = wtk_label_create(
			parent, &area, str_sleep_ret,
			GFX_COLOR(0, 0, 0), NULL, false);
	if (!widget_ctx_lpm_sleep->lbl_indication[1]) {
		goto error_widget;
	}

	win_show(wtk_label_as_child(widget_ctx_lpm_sleep->lbl_indication[1]));

	/* Disable all ITs */
	taskENTER_CRITICAL();

	/* Configure PIOA with IT priority > basepri  (lower has higher priority
	 * on Cortex M3) */
	NVIC_SetPriority(PIOA_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 16);
	/* Enable the interrupt for PIOA */
	NVIC_EnableIRQ(PIOA_IRQn);

	/* Enable the PIO for wake-up */
	supc_set_wakeup_inputs(SUPC,
			(sleep_wakeup_pin << 16) | sleep_wakeup_pin, 0);
	/* Enter into sleep Mode */
	pmc_enable_sleepmode(0);
	/* Re configure PIOA with IT priority > basepri  (lower has higher
	 * priority on Cortex M3) */
	NVIC_SetPriority(PIOA_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY + 16);

	taskEXIT_CRITICAL();

	win_destroy(wtk_basic_frame_as_child(widget_ctx_lpm_sleep->frame));

	/* Enter sleep mode handler */
	demo_set_special_mode_status(DEMO_SLEEP_MODE, 1);
	return;

	/* Error handling to clean up allocations after an error */
error_widget:
	win_destroy(wtk_basic_frame_as_child(widget_ctx_lpm_sleep->frame));
error_frame:
	membag_free(widget_ctx_lpm_sleep);
}

/**
 * \brief Enable/disable sleep mode widget.
 *
 * \param on Turn on/off sleep mode widget.
 */
void app_widget_lpm_sleep_on(uint8_t on)
{
	if (on) {
		app_widget_launch_lpm_sleep();
	} else {
		if (widget_ctx_lpm_sleep->frame) {
			win_destroy(wtk_basic_frame_as_child(
					widget_ctx_lpm_sleep->frame));
		}
	}
}
