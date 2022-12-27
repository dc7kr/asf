/**
 * \file
 *
 * \brief USB host driver for Communication Device Class interface.
 *
 * Copyright (C) 2012 Atmel Corporation. All rights reserved.
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

#ifndef _UHI_CDC_H_
#define _UHI_CDC_H_

#include "conf_usb_host.h"
#include "usb_protocol_cdc.h"
#include "uhc.h"
#include "uhi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup uhi_cdc_group
 * \defgroup uhi_cdc_group_uhc Interface with USB Host Core (UHC)
 *
 * Structures and functions required by UHC.
 * 
 * @{
 */

//! Global struture which contains standard UHI API for UHC
#define UHI_CDC { \
	.install = uhi_cdc_install, \
	.enable = uhi_cdc_enable, \
	.uninstall = uhi_cdc_uninstall, \
	.sof_notify = uhi_cdc_sof, \
}

/**
 * \name Functions required by UHC
 * See \ref uhi_api_t for the function descriptions
 * @{
 */
uhc_enum_status_t uhi_cdc_install(uhc_device_t* dev);
void uhi_cdc_enable(uhc_device_t* dev);
void uhi_cdc_uninstall(uhc_device_t* dev);
void uhi_cdc_sof(bool b_micro);
//@}

//@}

/**
 * \ingroup uhi_group
 * \defgroup uhi_cdc_group UHI for Communication Device Class
 *
 * Common APIs used by high level application to use this USB host class.
 * These routines are used by memory to transfer its data
 * to/from USB CDC endpoint.
 *
 * @{
 */

/**
 * \brief Open a port of UHI CDC interface
 *
 * \param port          Communication port number
 * \param configuration Pointer on port configuration
 *
 * \return \c true if the port is available
 */
bool uhi_cdc_open(uint8_t port, usb_cdc_line_coding_t *configuration);

/**
 * \brief Close a port
 *
 * \param port       Communication port number
 */
void uhi_cdc_close(uint8_t port);

/**
 * \brief This function checks if a character has been received on the CDC line
 *
 * \param port       Communication port number
 *
 * \return \c true if a byte is ready to be read.
 */
bool uhi_cdc_is_rx_ready(uint8_t port);

/**
 * \brief This function returns the number of character available on the CDC line
 *
 * \param port       Communication port number
 *
 * \return the number of data received
 */
iram_size_t uhi_cdc_get_nb_received(uint8_t port);

/**
 * \brief Waits and gets a value on CDC line
 *
 * \param port       Communication port number
 *
 * \return value read on CDC line
 */
int uhi_cdc_getc(uint8_t port);

/**
 * \brief Reads a RAM buffer on CDC line
 *
 * \param port      Communication port number
 * \param buf       Values read
 * \param size      Number of value read
 *
 * \return the number of data remaining
 */
iram_size_t uhi_cdc_read_buf(uint8_t port, void* buf, iram_size_t size);

/**
 * \brief This function checks if a new character sent is possible
 * The type int is used to support scanf redirection from compiler LIB.
 *
 * \param port       Communication port number
 *
 * \return \c true if a new character can be sent
 */
bool uhi_cdc_is_tx_ready(uint8_t port);

/**
 * \brief Puts a byte on CDC line
 * The type int is used to support printf redirection from compiler LIB.
 *
 * \param port       Communication port number
 * \param value      Value to put
 *
 * \return \c true if function was successfully done, otherwise \c false.
 */
int uhi_cdc_putc(uint8_t port, int value);

/**
 * \brief Writes a RAM buffer on CDC line
 *
 * \param port       Communication port number
 * \param buf       Values to write
 * \param size      Number of value to write
 *
 * \return the number of data remaining
 */
iram_size_t uhi_cdc_write_buf(uint8_t port, const void* buf, iram_size_t size);
//@}


#ifdef __cplusplus
}
#endif
#endif // _UHI_CDC_H_
