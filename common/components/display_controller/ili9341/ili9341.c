/**
 * \file
 *
 * \brief ILI9341 display controller component driver
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
#include "conf_ili9341.h"
#include "ili9341.h"
#include "ili9341_regs.h"
#include <sysclk.h>
#include <gpio.h>
#include <delay.h>

/**
 * \internal
 * \brief Helper function to select the CS of the controller on the bus
 */
__always_inline static void ili9341_select_chip(void)
{
	gpio_set_pin_low(CONF_ILI9341_CS_PIN);
}

/**
 * \internal
 * \brief Helper function to de-select the CS of the controller on the bus
 */
__always_inline static void ili9341_deselect_chip(void)
{
	gpio_set_pin_high(CONF_ILI9341_CS_PIN);
}

/**
 * \internal
 * \brief Helper function to select command byte transmission mode
 */
__always_inline static void ili9341_select_command_mode(void)
{
	gpio_set_pin_low(CONF_ILI9341_DC_PIN);
}

/**
 * \internal
 * \brief Helper function to select data byte transmission mode
 */
__always_inline static void ili9341_select_data_mode(void)
{
	gpio_set_pin_high(CONF_ILI9341_DC_PIN);
}

/**
 * \internal
 * \brief Helper function to wait for the last send operation to complete
 */
__always_inline static void ili9341_wait_for_send_done(void)
{
#if defined(CONF_ILI9341_USART_SPI) && defined(XMEGA)
	/* Wait for TX to complete */
	while (!usart_tx_is_complete(CONF_ILI9341_USART_SPI)) {
		/* Do nothing */
	}
	/* Clear the TX complete flag */
	usart_clear_tx_complete(CONF_ILI9341_USART_SPI);
#elif defined(CONF_ILI9341_SPI)
	/* Wait for TX to complete */
	while (!spi_is_tx_ok(CONF_ILI9341_SPI)) {
		/* Do nothing */
	}
#endif
}

/**
 * \internal
 * \brief Helper function to send a byte over an arbitrary interface
 *
 * This function is used to hide what interface is used by the component
 * driver, e.g.  the component driver does not need to know if USART in SPI
 * mode is used or the native SPI module.
 *
 * \param data The byte to be transfered
 */
__always_inline static void ili9341_send_byte(uint8_t data)
{
#if defined(CONF_ILI9341_USART_SPI)
#  if defined(UC3)
	usart_spi_write_packet(CONF_ILI9341_USART_SPI, &data, 1);
#  elif defined(XMEGA)
	irqflags_t flags;

	/* Wait for data register to be empty if send is in progress */
	while (!usart_data_register_is_empty(CONF_ILI9341_USART_SPI)) {
		/* Do nothing */
	}

	/* Disable interrupts to avoid too much time between sending new byte
	 * and clearing TX complete flag.
	 */
	flags = cpu_irq_save();
	usart_spi_write_single(CONF_ILI9341_USART_SPI, data);
	/* Clear the TX complete flag */
	usart_clear_tx_complete(CONF_ILI9341_USART_SPI);
	cpu_irq_restore(flags);
#  endif
#elif defined(CONF_ILI9341_SPI)
	/* Wait for any previously running send data */
	ili9341_wait_for_send_done();

	spi_write_single(CONF_ILI9341_SPI, data);
#endif
}

/**
 * \internal
 * \brief Helper function to read a byte from an arbitrary interface
 *
 * This function is used to hide what interface is used by the component
 * driver, e.g.  the component driver does not need to know if USART in SPI
 * mode is used or the native SPI module.
 *
 * \retval uint8_t Byte of data read from the display controller
 */
__always_inline static uint8_t ili9341_read_byte(void)
{
	uint8_t data;
#if defined(CONF_ILI9341_USART_SPI)
#  if defined(UC3)

	/* This function could also be used for XMEGA but results in a very slow
	 * framerate, hence a workaround has been implemented for the XMEGA */
	usart_spi_read_packet(CONF_ILI9341_USART_SPI, &data, 1);
#  elif defined(XMEGA)
	/* Workaround for clearing the RXCIF for XMEGA */
	usart_rx_enable(CONF_ILI9341_USART_SPI);

	usart_spi_write_single(CONF_ILI9341_USART_SPI, 0xFF);

	/* Wait for RX to complete */
	while (!usart_rx_is_complete(CONF_ILI9341_USART_SPI)) {
		/* Do nothing */
	}
	usart_spi_read_single(CONF_ILI9341_USART_SPI, &data);

	/* Workaround for clearing the RXCIF for XMEGA */
	usart_rx_disable(CONF_ILI9341_USART_SPI);
#  endif
#elif defined(CONF_ILI9341_SPI)
	spi_write_single(CONF_ILI9341_SPI, 0xFF);

	/* Wait for RX to complete */
	while (!spi_is_rx_full(CONF_ILI9341_SPI)) {
		/* Do nothing */
	}

	spi_read_single(CONF_ILI9341_SPI, &data);
#endif
	return data;
}

/**
 * \internal
 * \brief Sends a command to the controller, and prepares for parameter transfer
 *
 * Helper function to use for sending a command to the controller.
 *
 * \note After the command is sent, the display remains selected.
 *
 * \param command The command to send
 */
static void ili9341_send_command(uint8_t command)
{
	ili9341_select_command_mode();
	ili9341_select_chip();
	ili9341_send_byte(command);
	ili9341_wait_for_send_done();
	ili9341_select_data_mode();
}

static ili9341_coord_t limit_start_x, limit_start_y;
static ili9341_coord_t limit_end_x, limit_end_y;

/**
 * \internal
 * \brief Helper function to send the drawing limits (boundaries) to the display
 *
 * This function is used to send the currently set upper-left and lower-right
 * drawing limits to the display, as set through the various limit functions.
 */
static void ili9341_send_draw_limits(void)
{
	ili9341_send_command(ILI9341_CMD_COLUMN_ADDRESS_SET);
	ili9341_send_byte(limit_start_x >> 8);
	ili9341_send_byte(limit_start_x & 0xFF);
	ili9341_send_byte(limit_end_x >> 8);
	ili9341_send_byte(limit_end_x & 0xFF);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_PAGE_ADDRESS_SET);
	ili9341_send_byte(limit_start_y >> 8);
	ili9341_send_byte(limit_start_y & 0xFF);
	ili9341_send_byte(limit_end_y >> 8);
	ili9341_send_byte(limit_end_y & 0xFF);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Set the display top left drawing limit
 *
 * Use this function to set the top left limit of the drawing limit box.
 *
 * \param x The x coordinate of the top left corner
 * \param y The y coordinate of the top left corner
 */
void ili9341_set_top_left_limit(ili9341_coord_t x, ili9341_coord_t y)
{
	limit_start_x = x;
	limit_start_y = y;

	ili9341_send_draw_limits();
}

/**
 * \brief Set the display bottom right drawing limit
 *
 * Use this function to set the bottom right corner of the drawing limit box.
 *
 * \param x The x coordinate of the bottom right corner
 * \param y The y coordinate of the bottom right corner
 */
void ili9341_set_bottom_right_limit(ili9341_coord_t x, ili9341_coord_t y)
{
	limit_end_x = x;
	limit_end_y = y;

	ili9341_send_draw_limits();
}

/**
 * \brief Set the full display drawing limits
 *
 * Use this function to set the full drawing limit box.
 *
 * \param start_x The x coordinate of the top left corner
 * \param start_y The y coordinate of the top left corner
 * \param end_x The x coordinate of the bottom right corner
 * \param end_y The y coordinate of the bottom right corner
 */
void ili9341_set_limits(ili9341_coord_t start_x, ili9341_coord_t start_y,
		ili9341_coord_t end_x, ili9341_coord_t end_y)
{
	limit_start_x = start_x;
	limit_start_y = start_y;
	limit_end_x = end_x;
	limit_end_y = end_y;

	ili9341_send_draw_limits();
}

/**
 * \brief Read a single color from the graphical memory
 *
 * Use this function to read a color from the graphical memory of the
 * controller.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \retval ili9341_color_t The read color pixel
 */
ili9341_color_t ili9341_read_gram(void)
{
	uint8_t red, green, blue;

	ili9341_send_command(ILI9341_CMD_MEMORY_READ);

	/* No interesting data in the first byte, hence read and discard */
	red = ili9341_read_byte();

	red = ili9341_read_byte();
	green = ili9341_read_byte();
	blue = ili9341_read_byte();

	ili9341_deselect_chip();

	return ILI9341_COLOR(red, green, blue);
}

/**
 * \brief Write the graphical memory with a single color pixel
 *
 * Use this function to write a single color pixel to the controller memory.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \param color The color pixel to write to the screen
 */
void ili9341_write_gram(ili9341_color_t color)
{
	/* Only 16-bit color supported */
	Assert(sizeof(color) == 2);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);
	ili9341_send_byte(color);
	ili9341_send_byte(color >> 8);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Copy pixels from SRAM to the screen
 *
 * Used to copy a large quantitative of data to the screen in one go.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \param pixels Pointer to the pixel data
 * \param count Number of pixels to copy to the screen
 */
void ili9341_copy_pixels_to_screen(const ili9341_color_t *pixels, uint32_t count)
{
	const ili9341_color_t *pixel = pixels;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

	while (count--) {
		ili9341_send_byte(*pixel);
		ili9341_send_byte(*pixel >> 8);

		pixel++;
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Copy pixels from progmem to the screen
 *
 * This function can be used to copy data from program memory (flash) to the
 * display.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \param pixels Pointer to the progmem data
 * \param count Number of pixels to write
 */
void ili9341_copy_progmem_pixels_to_screen(
		ili9341_color_t PROGMEM_PTR_T pixels, uint32_t count)
{
	ili9341_color_t color;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

	while (count--) {
		color = PROGMEM_READ_WORD(pixels);

		ili9341_send_byte(color);
		ili9341_send_byte(color >> 8);

		pixels++;
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Set a given number of pixels to the same color
 *
 * Use this function to write a certain number of pixels to the same color
 * within a set limit.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \param color The color to write to the display
 * \param count The number of pixels to write with this color
 */
void ili9341_duplicate_pixel(const ili9341_color_t color, uint32_t count)
{
	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

	while (count--) {
		ili9341_send_byte(color);
		ili9341_send_byte(color >> 8);
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Copy pixels from the screen to a pixel buffer
 *
 * Use this function to copy pixels from the display to an internal SRAM buffer.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
 * ili9341_set_top_left_limit(0, 0);
 * ili9341_set_bottom_right_limit(320, 240);
 * ...
 * \endcode
 *
 * \param pixels Pointer to the pixel buffer to read to
 * \param count Number of pixels to read
 */
void ili9341_copy_pixels_from_screen(ili9341_color_t *pixels, uint32_t count)
{
	uint8_t red, green, blue;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_READ);

	/* No interesting data in the first byte, hence read and discard */
	red = ili9341_read_byte();

	while (count--) {
		red = ili9341_read_byte();
		green = ili9341_read_byte();
		blue = ili9341_read_byte();

		*pixels = ILI9341_COLOR(red, green, blue);
		pixels++;
	}

	ili9341_deselect_chip();
}

/**
 * \internal
 * \brief Initialize the hardware interface to the controller
 *
 * This will initialize the module used for communication with the controller.
 * Currently supported interfaces by this component driver are the SPI
 * interface through either the SPI module in master mode or the USART in
 * Master SPI mode.  Configuration must be done in the associated
 * conf_ili9341.h file.
 */
static void ili9341_interface_init(void)
{
#if defined(CONF_ILI9341_USART_SPI) | defined(CONF_ILI9341_SPI)
	spi_flags_t spi_flags = SPI_MODE_0;
	board_spi_select_id_t spi_select_id = 0;
#else
	#error Interface for ILI9341 has not been selected or interface not\
	supported, please configure component driver using the conf_ili9341.h\
        file!
#endif

#if defined(CONF_ILI9341_USART_SPI)
	struct usart_spi_device device = {
		.id = CONF_ILI9341_CS_PIN,
	};
	usart_spi_init(CONF_ILI9341_USART_SPI);
	usart_spi_setup_device(CONF_ILI9341_USART_SPI, &device, spi_flags,
			CONF_ILI9341_CLOCK_SPEED, spi_select_id);
#elif defined(CONF_ILI9341_SPI)
	struct spi_device device = {
		.id = CONF_ILI9341_CS_PIN,
	};
	spi_master_init(CONF_ILI9341_SPI);
	spi_master_setup_device(CONF_ILI9341_SPI, &device, spi_flags,
			CONF_ILI9341_CLOCK_SPEED, spi_select_id);
	spi_enable(CONF_ILI9341_SPI);

	/* Send one dummy byte for the spi_is_tx_ok() to work as expected */
	spi_write_single(CONF_ILI9341_SPI, 0);
#endif
}

/**
 * \internal
 * \brief Initialize all the display registers
 *
 * This function will set up all the internal registers according the the
 * manufacturer's description.
 */
static void ili9341_controller_init_registers(void)
{
	ili9341_send_command(ILI9341_CMD_POWER_CONTROL_A);
	ili9341_send_byte(0x39);
	ili9341_send_byte(0x2C);
	ili9341_send_byte(0x00);
	ili9341_send_byte(0x34);
	ili9341_send_byte(0x02);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_POWER_CONTROL_B);
	ili9341_send_byte(0x00);
	ili9341_send_byte(0xAA);
	ili9341_send_byte(0XB0);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_PUMP_RATIO_CONTROL);
	ili9341_send_byte(0x30);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_POWER_CONTROL_1);
	ili9341_send_byte(0x25);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_POWER_CONTROL_2);
	ili9341_send_byte(0x11);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_VCOM_CONTROL_1);
	ili9341_send_byte(0x5C);
	ili9341_send_byte(0x4C);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_VCOM_CONTROL_2);
	ili9341_send_byte(0x94);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_DRIVER_TIMING_CONTROL_A);
	ili9341_send_byte(0x85);
	ili9341_send_byte(0x01);
	ili9341_send_byte(0x78);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_DRIVER_TIMING_CONTROL_B);
	ili9341_send_byte(0x00);
	ili9341_send_byte(0x00);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET);
	ili9341_send_byte(0x05);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_set_orientation(0);
	ili9341_set_limits(0, 0, ILI9341_DEFAULT_WIDTH,
			ILI9341_DEFAULT_HEIGHT);
}

/**
 * \internal
 * \brief Send display commands to exit standby mode
 *
 * This function is used to exit the display standby mode, which is the default
 * mode after a reset signal to the display.
 */
static void ili9341_exit_standby(void)
{
	ili9341_send_command(ILI9341_CMD_SLEEP_OUT);
	ili9341_deselect_chip();
	delay_ms(150);
	ili9341_send_command(ILI9341_CMD_DISPLAY_ON);
	ili9341_deselect_chip();
}

/**
 * \internal
 * \brief Reset the display using the digital control interface
 *
 * Controls the reset pin of the display controller to reset the display.
 */
static void ili9341_reset_display(void)
{
	gpio_set_pin_high(CONF_ILI9341_RESET_PIN);
	delay_ms(10);
	gpio_set_pin_low(CONF_ILI9341_RESET_PIN);
	delay_ms(10);
	gpio_set_pin_high(CONF_ILI9341_RESET_PIN);
	delay_ms(150);
}

/**
 * \brief Initialize the controller
 *
 * Used to initialize the ILI9341 display controller by setting up the hardware
 * interface, and setting up the controller according to the manufacturer's
 * description. It also set up the screen orientation to the default state
 * (portrait).
 */
void ili9341_init(void)
{
	/* Initialize the communication interface */
	ili9341_interface_init();

	/* Reset the display */
	ili9341_reset_display();

	/* Send commands to exit standby mode */
	ili9341_exit_standby();

	/* Write all the controller registers with correct values */
	ili9341_controller_init_registers();
}

/**
 * \brief Sets the orientation of the display data
 *
 * Configures the display for a given orientation, including mirroring and/or
 * screen rotation.
 *
 * \param flags Orientation flags to use, see \ref ILI9341_FLIP_X, \ref ILI9341_FLIP_Y
 *        and \ref ILI9341_SWITCH_XY.
 */
void ili9341_set_orientation(uint8_t flags)
{
	uint8_t madctl = 0x48;

	if (flags & ILI9341_FLIP_X) {
		madctl &= ~(1 << 6);
	}

	if (flags & ILI9341_FLIP_Y) {
		madctl |= 1 << 7;
	}

	if (flags & ILI9341_SWITCH_XY) {
		madctl |= 1 << 5;
	}

	ili9341_send_command(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	ili9341_send_byte(madctl);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}
