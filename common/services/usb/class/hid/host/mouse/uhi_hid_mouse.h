/**
 * \file
 *
 * \brief USB host driver for Human Interface Device (HID) mouse interface.
 *
 * Copyright (C) 2011-2012 Atmel Corporation. All rights reserved.
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

#ifndef _UHI_HID_MOUSE_H_
#define _UHI_HID_MOUSE_H_

#include "conf_usb_host.h"
#include "usb_protocol.h"
#include "usb_protocol_hid.h"
#include "uhi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup uhi_hid_mouse_group
 * \defgroup uhi_hid_mouse_group_uhc Interface with USB Host Core (UHC)
 *
 * Structures and functions required by UHC.
 *
 * @{
 */

//! Global struture which contains standard UHI API for UHC
#define UHI_HID_MOUSE { \
	.install = uhi_hid_mouse_install, \
	.enable = uhi_hid_mouse_enable, \
	.uninstall = uhi_hid_mouse_uninstall, \
	.sof_notify = NULL, \
}

/**
 * \name Functions required by UHC
 * @{
 */
extern uhc_enum_status_t uhi_hid_mouse_install(uhc_device_t* dev);
extern void uhi_hid_mouse_enable(uhc_device_t* dev);
extern void uhi_hid_mouse_uninstall(uhc_device_t* dev);
//@}
//@}

/**
 * \ingroup uhi_group
 * \defgroup uhi_hid_mouse_group UHI for Human Interface Device Mouse Class
 *
 * Common APIs used by high level application to use this USB host class.
 * 
 * This API requires only callback definitions in conf_usb_host.h file
 * through following defines:
 * - \code  #define UHI_HID_MOUSE_CHANGE(dev,b_plug)
 * #define UHI_HID_MOUSE_EVENT_BTN_LEFT(b_state)
 * #define UHI_HID_MOUSE_EVENT_BTN_RIGHT(b_state)
 * #define UHI_HID_MOUSE_EVENT_BTN_MIDDLE(b_state)
 * #define UHI_HID_MOUSE_EVENT_MOUVE(x,y,scroll) \endcode
 *
 * @{
 */
//@}


#ifdef __cplusplus
}
#endif
#endif // _UHI_HID_MOUSE_H_
