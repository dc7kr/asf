/**
 * \file
 *
 * \brief USB host Mass Storage interface for control access module.
 *
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
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

#ifndef _UHI_MSC_MEM_H_
#define _UHI_MSC_MEM_H_

#include "ctrl_access.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup uhi_msc_group
 * \defgroup uhi_msc_mem_group USB host Mass Storage interface for control access module
 * @{
 */

/**
 * \brief Gives the number of available LUN
 * Note: A LUN can be avaliable, but with a status not present.
 * It is the case for a card reader without card.
 *
 * \return Number of avalaible LUN
 */
uint8_t uhi_msc_mem_get_lun(void);

/**
 * \brief Checks and update the status of the LUN
 *
 * \param lun       LUN number
 *
 * \return Status of the LUN
 */
Ctrl_status uhi_msc_mem_test_unit_ready(uint8_t lun);

/**
 * \brief Returns the capacity of the LUN
 *
 * \param lun           LUN number
 * \param u32_nb_sector Pointer to store the last sector address possible on this LUN
 *
 * \return Status of the LUN
 */
Ctrl_status uhi_msc_mem_read_capacity(uint8_t lun, uint32_t *u32_nb_sector);

/**
 * \brief Returns the sector size of the LUN
 *
 * \param lun           LUN number
 *
 * \return Sector size (unit 512B)
 */
uint8_t uhi_msc_mem_read_sector_size(uint8_t lun);

/**
 * \brief Checks if the LUN is write protected
 *
 * \param lun           LUN number
 *
 * \return true, if write protected
 */
bool uhi_msc_mem_wr_protect(uint8_t lun);

/**
 * \brief Checks if the device is removed
 *
 * \return Always true for USB Device
 */
bool uhi_msc_mem_removal(void);

/**
 * \brief Reads 512 bytes from the current LUN
 * The LUN is selected by uhi_msc_mem_test_unit_ready()
 * or uhi_msc_mem_read_capacity() function.
 *
 * \param addr  Disk address (unit 512B)
 * \param ram   Pointer to store the data
 *
 * \return Status of the LUN
 */
Ctrl_status uhi_msc_mem_read_10_ram(uint32_t addr, void *ram);

/**
 * \brief Writes 512 bytes to the current LUN
 * The LUN is selected by uhi_msc_mem_test_unit_ready()
 * or uhi_msc_mem_read_capacity() function.
 *
 * \param addr  Disk address (unit 512B)
 * \param ram   Pointer on the data
 *
 * \return Status of the LUN
 */
Ctrl_status uhi_msc_mem_write_10_ram(uint32_t addr, const void *ram);

//@}

#ifdef __cplusplus
}
#endif
#endif // _UHI_MSC_MEM_H_
