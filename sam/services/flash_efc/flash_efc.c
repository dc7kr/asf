/**
 * \file
 *
 * \brief Embedded Flash service for SAM.
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

#include <string.h>
#include <assert.h>
#include "flash_efc.h"
#include "sysclk.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_services_flash_efc_group Embedded Flash Service
 *
 * The Embedded Flash service provides functions for internal flash operations.
 *
 * @{
 */

#if (SAM3XA || SAM3U4)
/* Internal Flash Controller 0. */
# define EFC     EFC0
/* The max GPNVM number. */
# define GPNVM_NUM_MAX        3
/* Internal Flash 0 base address. */
# define IFLASH_ADDR     IFLASH0_ADDR
/* Internal flash page size. */
# define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE
/* Internal flash lock region size. */
# define IFLASH_LOCK_REGION_SIZE     IFLASH0_LOCK_REGION_SIZE
#elif (SAM3U)
/* There is no EFC1 for SAM3U except for SAM3U4 */
# undef EFC1
/* Internal Flash Controller 0. */
# define EFC     EFC0
/* The max GPNVM number. */
# define GPNVM_NUM_MAX        2
/* Internal Flash 0 base address. */
# define IFLASH_ADDR     IFLASH0_ADDR
/* Internal flash page size. */
# define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE
/* Internal flash lock region size. */
# define IFLASH_LOCK_REGION_SIZE     IFLASH0_LOCK_REGION_SIZE
#elif (SAM3SD8)
/* The max GPNVM number. */
# define GPNVM_NUM_MAX        3
/* Internal flash page size. */
# define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE
/* Internal flash lock region size. */
# define IFLASH_LOCK_REGION_SIZE     IFLASH0_LOCK_REGION_SIZE
#else
/* The max GPNVM number. */
# define GPNVM_NUM_MAX        2
#endif

#if SAM4S
/* User signature size */
# define FLASH_USER_SIG_SIZE   (512)
#endif

/* Flash page buffer for alignment */
static uint32_t gdw_page_buffer[IFLASH_PAGE_SIZE / sizeof(uint32_t)];

/**
 * \brief Translate the given flash address to page and offset values.
 * \note pw_page and pw_offset must not be null in order to store the corresponding values.
 *
 * \param pp_efc Pointer to an EFC pointer.
 * \param dw_addr Address to translate.
 * \param pw_page The first page accessed.
 * \param pw_offset Byte offset in the first page.
 */
static void translate_address(Efc **pp_efc, uint32_t dw_addr,
		uint16_t *pw_page, uint16_t *pw_offset)
{
	Efc *p_efc;
	uint16_t w_page;
	uint16_t w_offset;

#if (SAM3XA || SAM3U4)
	if (dw_addr >= IFLASH1_ADDR) {
		p_efc = EFC1;
		w_page = (dw_addr - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
		w_offset = (dw_addr - IFLASH1_ADDR) % IFLASH1_PAGE_SIZE;
	} else {
		p_efc = EFC0;
		w_page = (dw_addr - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
		w_offset = (dw_addr - IFLASH0_ADDR) % IFLASH0_PAGE_SIZE;
	}
#elif (SAM3SD8)
	p_efc = EFC;
	w_page = (dw_addr - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
	w_offset = (dw_addr - IFLASH0_ADDR) % IFLASH0_PAGE_SIZE;
#else
	assert(dw_addr >= IFLASH_ADDR);
	assert(dw_addr <= (IFLASH_ADDR + IFLASH_SIZE));

	p_efc = EFC;
	w_page = (dw_addr - IFLASH_ADDR) / IFLASH_PAGE_SIZE;
	w_offset = (dw_addr - IFLASH_ADDR) % IFLASH_PAGE_SIZE;
#endif

	/* Store values */
	if (pp_efc) {
		*pp_efc = p_efc;
	}

	if (pw_page) {
		*pw_page = w_page;
	}

	if (pw_offset) {
		*pw_offset = w_offset;
	}
}

/**
 * \brief Compute the address of a flash by the given page and offset.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param w_page Page number.
 * \param w_offset Byte offset inside page.
 * \param pdw_addr Computed address (optional).
 */
static void compute_address(Efc *p_efc, uint16_t w_page, uint16_t w_offset,
		uint32_t *pdw_addr)
{
	uint32_t dw_addr;

/* Dual bank flash */
#ifdef EFC1
	/* Compute address */
	dw_addr = (p_efc == EFC0) ?
			IFLASH0_ADDR + w_page * IFLASH_PAGE_SIZE + w_offset :
			IFLASH1_ADDR + w_page * IFLASH_PAGE_SIZE + w_offset;

/* One bank flash */
#else
	/* Stop warning */
	p_efc = p_efc;
	/* Compute address */
	dw_addr = IFLASH_ADDR + w_page * IFLASH_PAGE_SIZE + w_offset;
#endif

	/* Store result */
	if (pdw_addr != NULL) {
		*pdw_addr = dw_addr;
	}
}

/**
 * \brief Compute the lock range associated with the given address range.
 *
 * \param dw_start Start address of lock range.
 * \param dw_end End address of lock range.
 * \param pdw_actual_start Actual start address of lock range.
 * \param pdw_actual_end Actual end address of lock range.
 */
static void compute_lock_range(uint32_t dw_start, uint32_t dw_end,
		uint32_t *pdw_actual_start, uint32_t *pdw_actual_end)
{
	uint32_t dw_actual_start, dw_actual_end;

	dw_actual_start = dw_start - (dw_start % IFLASH_LOCK_REGION_SIZE);
	dw_actual_end = dw_end - (dw_end % IFLASH_LOCK_REGION_SIZE) +
			IFLASH_LOCK_REGION_SIZE - 1;

	if (pdw_actual_start) {
		*pdw_actual_start = dw_actual_start;
	}

	if (pdw_actual_end) {
		*pdw_actual_end = dw_actual_end;
	}
}

/**
 * \brief Initialize the flash service.
 *
 * \param dw_mode FLASH_ACCESS_MODE_128 or FLASH_ACCESS_MODE_64.
 * \param dw_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_init(uint32_t dw_mode, uint32_t dw_fws)
{
	efc_init(EFC, dw_mode, dw_fws);
#ifdef EFC1
	efc_init(EFC1, dw_mode, dw_fws);
#endif

	return FLASH_RC_OK;
}

/** 
 * \brief Set flash wait state.
 * 
 * \param dw_address Flash bank start address.
 * \param dw_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_wait_state(uint32_t dw_address, uint32_t dw_fws)
{
	Efc *p_efc;

	translate_address(&p_efc, dw_address, NULL, NULL);
	efc_set_wait_state(p_efc, dw_fws);

	return FLASH_RC_OK;
}

/** 
 * \brief Set flash wait state.
 * 
 * \param dw_address Flash bank start address.
 * \param dw_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_wait_state_adaptively(uint32_t dw_address)
{
	Efc *p_efc;
	uint32_t clock = sysclk_get_main_hz();

	translate_address(&p_efc, dw_address, NULL, NULL);

	/* Set FWS for embedded Flash access according to operating frequency */
	if (clock < CHIP_FREQ_FWS_0) {
		efc_set_wait_state(p_efc, 0);
	} else if (clock < CHIP_FREQ_FWS_1) {
		efc_set_wait_state(p_efc, 1);
	} else if (clock < CHIP_FREQ_FWS_2) {
		efc_set_wait_state(p_efc, 2);
#if (SAM3XA || SAM3U || SAM4S)
	} else if (clock < CHIP_FREQ_FWS_3) {
		efc_set_wait_state(p_efc, 3);
	} else {
		efc_set_wait_state(p_efc, 4);
	}
#else
	} else {
		efc_set_wait_state(p_efc, 3);
	}
#endif

	return FLASH_RC_OK;
}

/** 
 * \brief Get flash wait state.
 *
 * \param dw_address Flash bank start address.
 *
 * \return The number of wait states in cycle (no shift).
 */
uint32_t flash_get_wait_state(uint32_t dw_address)
{
	Efc *p_efc;

	translate_address(&p_efc, dw_address, NULL, NULL);
	return efc_get_wait_state(p_efc);
}

/**
 * \brief Get flash descriptor.
 *
 * \param dw_address Flash bank start address.
 * \param pdw_flash_descriptor Pointer to a data buffer to store flash descriptor.
 * \param dw_size Data buffer size in DWORD.
 *
 * \return The actual descriptor length.
 */
uint32_t flash_get_descriptor(uint32_t dw_address,
		uint32_t *pdw_flash_descriptor, uint32_t dw_size)
{
	Efc *p_efc;
	uint32_t dw_tmp;
	uint32_t dw_cnt;

	translate_address(&p_efc, dw_address, NULL, NULL);

	/* Command fails */
	if (FLASH_RC_OK != efc_perform_command(p_efc, EFC_FCMD_GETD, 0)) {
		return 0;
	} else {
		/* Read until no result */
		for (dw_cnt = 0;; dw_cnt++) {
			dw_tmp = efc_get_result(p_efc);
			if ((dw_size > dw_cnt) && (dw_tmp != 0)) {
				*pdw_flash_descriptor++ = dw_tmp;
			} else {
				break;
			}
		}
	}

	return dw_cnt;
}

/**
 * \brief Get flash total page count for the specified bank.
 *
 * \param pdw_flash_descriptor Pointer to a flash descriptor.
 * \note The flash descriptor must be fetched from flash_get_descriptor function first.
 *
 * \return The flash total page count.
 */
uint32_t flash_get_page_count(const uint32_t *pdw_flash_descriptor)
{
	return (pdw_flash_descriptor[1] / pdw_flash_descriptor[2]);
}

/**
 * \brief Get flash page count per region (plane) for the specified bank.
 *
 * \param pdw_flash_descriptor Pointer to a flash descriptor.
 * The flash descriptor must be fetched from flash_get_descriptor function first.
 *
 * \return The flash page count per region (plane).
 */
uint32_t flash_get_page_count_per_region(const uint32_t *pdw_flash_descriptor)
{
	return (pdw_flash_descriptor[4] / pdw_flash_descriptor[2]);
}

/**
 * \brief Get flash region (plane) count for the specified bank.
 *
 * \param pdw_flash_descriptor Pointer to a flash descriptor.
 * The flash descriptor must be fetched from flash_get_descriptor function first.
 *
 * \return The flash region (plane) count.
 */
uint32_t flash_get_region_count(const uint32_t *pdw_flash_descriptor)
{
	return (pdw_flash_descriptor[3]);
}

/**
 * \brief Erase the entire flash.
 *
 * \param dw_address  Flash bank start address.
 * \note Only the flash bank including dw_address will be erased.
 * If there are two flash banks, we need to call this function twice with
 * each bank start address.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_all(uint32_t dw_address)
{
	Efc *p_efc;

	translate_address(&p_efc, dw_address, NULL, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EA, 0)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

#if SAM3SD8
/**
 * \brief Erase the flash by plane.
 *
 * \param dw_address Flash plane start address.
 * Erase plane command needs a page number parameter which belongs to the plane to be erased.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_plane(uint32_t dw_address)
{
	Efc *p_efc;
	uint16_t w_page;

	translate_address(&p_efc, dw_address, &w_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EPL, w_page)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}
#endif

#if SAM4S
/**
 * \brief Erase the specified pages of flash.
 *
 * \param dw_address Flash bank start address.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_page(uint32_t dw_address, uint8_t uc_page_num)
{
	Efc *p_efc;
	uint16_t w_page;

	if (uc_page_num >= IFLASH_ERASE_PAGES_INVALID) {
		return FLASH_RC_INVALID;
	}

	if (dw_address & (IFLASH_PAGE_SIZE - 1)) {
		return FLASH_RC_INVALID;
	}

	translate_address(&p_efc, dw_address, &w_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EPA,
					(w_page | uc_page_num))) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Erase the flash sector.
 *
 * \param dw_address Flash sector start address.
 * \note Erase sector command needs a page number parameter which belongs to the sector to be erased.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_plane(uint32_t dw_address)
{
	Efc *p_efc;
	uint16_t w_page;

	translate_address(&p_efc, dw_address, &w_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_ES, w_page)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}
#endif

/**
 * \brief Write a data buffer on flash.
 *
 * \note This function works in polling mode, and thus only returns when the
 * data has been effectively written.
 * \note For dual bank flash, this function doesn't support cross write from
 * bank 0 to bank 1. In this case, flash_write must be called twice (ie for each bank).
 *
 * \param dw_address Write address.
 * \param p_buffer Data buffer.
 * \param dw_size Size of data buffer in bytes.
 * \param dw_erase_flag Flag to set if erase first.
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_write(uint32_t dw_address, const void *p_buffer,
		uint32_t dw_size, uint32_t dw_erase_flag)
{
	Efc *p_efc;
	uint32_t dw_fws_temp;
	uint16_t w_page;
	uint16_t w_offset;
	uint32_t writeSize;
	uint32_t dw_page_addr;
	uint16_t w_padding;
	uint32_t dw_error;
	uint32_t dw_idx;
	uint32_t *p_aligned_dest;
	uint8_t *puc_page_buffer = (uint8_t *) gdw_page_buffer;

	translate_address(&p_efc, dw_address, &w_page, &w_offset);

	/* According to the errata, set the wait state value to 6. */
	dw_fws_temp = efc_get_wait_state(p_efc);
	efc_set_wait_state(p_efc, 6);

	/* Write all pages */
	while (dw_size > 0) {
		/* Copy data in temporary buffer to avoid alignment problems. */
		writeSize = Min((uint32_t) IFLASH_PAGE_SIZE - w_offset,
				dw_size);
		compute_address(p_efc, w_page, 0, &dw_page_addr);
		w_padding = IFLASH_PAGE_SIZE - w_offset - writeSize;

		/* Pre-buffer data */
		memcpy(puc_page_buffer, (void *)dw_page_addr, w_offset);

		/* Buffer data */
		memcpy(puc_page_buffer + w_offset, p_buffer, writeSize);

		/* Post-buffer data */
		memcpy(puc_page_buffer + w_offset + writeSize,
				(void *)(dw_page_addr + w_offset + writeSize),
				w_padding);

		/* Write page.
		 * Writing 8-bit and 16-bit data is not allowed and may lead to 
		 * unpredictable data corruption.
		 */
		p_aligned_dest = (uint32_t *) dw_page_addr;
		for (dw_idx = 0; dw_idx < (IFLASH_PAGE_SIZE / sizeof(uint32_t));
				++dw_idx) {
			*p_aligned_dest++ = gdw_page_buffer[dw_idx];
		}

		if (dw_erase_flag) {
			dw_error = efc_perform_command(p_efc, EFC_FCMD_EWP,
					w_page);
		} else {
			dw_error = efc_perform_command(p_efc, EFC_FCMD_WP,
					w_page);
		}

		if (dw_error) {
			return dw_error;
		}

		/* Progression */
		p_buffer = (void *)((uint32_t) p_buffer + writeSize);
		dw_size -= writeSize;
		w_page++;
		w_offset = 0;
	}

	/* According to the errata, restore the wait state value. */
	efc_set_wait_state(p_efc, dw_fws_temp);

	return FLASH_RC_OK;
}


/**
 * \brief Lock all the regions in the given address range. The actual lock
 * range is reported through two output parameters.
 *
 * \param dw_start Start address of lock range.
 * \param dw_end End address of lock range.
 * \param pdw_actual_start Start address of the actual lock range (optional).
 * \param pdw_actual_end End address of the actual lock range (optional).
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_lock(uint32_t dw_start, uint32_t dw_end,
		uint32_t *pdw_actual_start, uint32_t *pdw_actual_end)
{
	Efc *p_efc;
	uint32_t dw_actual_start, dw_actual_end;
	uint16_t dw_start_page, dw_end_page;
	uint32_t dw_error;
	uint16_t dw_num_pages_in_region =
			IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;

	/* Compute actual lock range and store it */
	compute_lock_range(dw_start, dw_end, &dw_actual_start, &dw_actual_end);

	if (pdw_actual_start != NULL) {
		*pdw_actual_start = dw_actual_start;
	}

	if (pdw_actual_end != NULL) {
		*pdw_actual_end = dw_actual_end;
	}

	/* Compute page numbers */
	translate_address(&p_efc, dw_actual_start, &dw_start_page, 0);
	translate_address(0, dw_actual_end, &dw_end_page, 0);

	/* Lock all pages */
	while (dw_start_page < dw_end_page) {
		dw_error = efc_perform_command(p_efc, EFC_FCMD_SLB, dw_start_page);
		
		if (dw_error) {
			return dw_error;
		}
		dw_start_page += dw_num_pages_in_region;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Unlock all the regions in the given address range. The actual unlock 
 * range is reported through two output parameters.
 *
 * \param dw_start Start address of unlock range.
 * \param dw_end End address of unlock range.
 * \param pdw_actual_start Start address of the actual unlock range (optional).
 * \param pdw_actual_end End address of the actual unlock range (optional).
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_unlock(uint32_t dw_start, uint32_t dw_end,
		uint32_t *pdw_actual_start, uint32_t *pdw_actual_end)
{
	Efc *p_efc;
	uint32_t dw_actual_start, dw_actual_end;
	uint16_t dw_start_page, dw_end_page;
	uint32_t dw_error;
	uint16_t dw_num_pages_in_region =
			IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;

	/* Compute actual unlock range and store it */
	compute_lock_range(dw_start, dw_end, &dw_actual_start, &dw_actual_end);
	if (pdw_actual_start != NULL) {
		*pdw_actual_start = dw_actual_start;
	}
	if (pdw_actual_end != NULL) {
		*pdw_actual_end = dw_actual_end;
	}

	/* Compute page numbers */
	translate_address(&p_efc, dw_actual_start, &dw_start_page, 0);
	translate_address(0, dw_actual_end, &dw_end_page, 0);

	/* Unlock all pages */
	while (dw_start_page < dw_end_page) {
		dw_error = efc_perform_command(p_efc, EFC_FCMD_CLB,
				dw_start_page);
		if (dw_error) {
			return dw_error;
		}
		dw_start_page += dw_num_pages_in_region;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Get the number of locked regions inside the given address range.
 *
 * \param dw_start Start address of range
 * \param dw_end End address of range.
 *
 * \return The number of locked regions inside the given address range.
 */
uint32_t flash_is_locked(uint32_t dw_start, uint32_t dw_end)
{
	Efc *p_efc;
	uint16_t dw_start_page, dw_end_page;
	uint8_t uc_start_region, uc_end_region;
	uint32_t dw_num_pages_in_region;
	uint32_t dw_status;
	uint32_t dw_error;
	uint32_t dw_num_locked_regions = 0;
	uint32_t dw_count = 0;
	uint32_t dw_bit = 0;

	assert(dw_end >= dw_start);
	assert((dw_start >= IFLASH_ADDR)
			&& (dw_end <= IFLASH_ADDR + IFLASH_SIZE));

	/* Compute page numbers */
	translate_address(&p_efc, dw_start, &dw_start_page, 0);
	translate_address(0, dw_end, &dw_end_page, 0);

	/* Compute region numbers */
	dw_num_pages_in_region = IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;
	uc_start_region = dw_start_page / dw_num_pages_in_region;
	uc_end_region = dw_end_page / dw_num_pages_in_region;

	/* Retrieve lock status */
	dw_error = efc_perform_command(p_efc, EFC_FCMD_GLB, 0);
	assert(!dw_error);

	/* Skip unrequested regions (if necessary) */
	dw_status = efc_get_result(p_efc);
	while (!(dw_count <= uc_start_region && uc_start_region < (dw_count + 32))) {
		dw_status = efc_get_result(p_efc);
		dw_count += 32;
	}

	/* Check status of each involved region */
	dw_bit = uc_start_region - dw_count;
	
	/* Number of region to check (must be > 0) */
	dw_count = uc_end_region - uc_start_region + 1;

	while (dw_count > 0) {
		if (dw_status & (1 << (dw_bit))) {
			dw_num_locked_regions++;
		}
		
		dw_count -= 1;
		dw_bit += 1;
		if (dw_bit == 32) {
			dw_status = efc_get_result(p_efc);
			dw_bit = 0;
		}
	}

	return dw_num_locked_regions;
}

/**
 * \brief Set the given GPNVM bit.
 *
 * \param dw_gpnvm GPNVM bit index.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_gpnvm(uint32_t dw_gpnvm)
{
	if (dw_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (FLASH_RC_YES == flash_is_gpnvm_set(dw_gpnvm)) {
		return FLASH_RC_OK;
	}

	if (EFC_RC_OK == efc_perform_command(EFC, EFC_FCMD_SGPB, dw_gpnvm)) {
		return FLASH_RC_OK;
	}

	return FLASH_RC_ERROR;
}

/**
 * \brief Clear the given GPNVM bit.
 *
 * \param dw_gpnvm GPNVM bit index.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_clear_gpnvm(uint32_t dw_gpnvm)
{
	if (dw_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (FLASH_RC_NO == flash_is_gpnvm_set(dw_gpnvm)) {
		return FLASH_RC_OK;
	}

	if (EFC_RC_OK == efc_perform_command(EFC, EFC_FCMD_CGPB, dw_gpnvm)) {
		return FLASH_RC_OK;
	}

	return FLASH_RC_ERROR;
}

/**
 * \brief Check if the given GPNVM bit is set or not.
 *
 * \param dw_gpnvm GPNVM bit index.
 *
 * \retval 1 If the given GPNVM bit is currently set.
 * \retval 0 If the given GPNVM bit is currently cleared.
 */
uint32_t flash_is_gpnvm_set(uint32_t dw_gpnvm)
{
	uint32_t dw_gpnvm_bits;

	if (dw_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (EFC_RC_OK != efc_perform_command(EFC, EFC_FCMD_GGPB, 0)) {
		return FLASH_RC_ERROR;
	}

	dw_gpnvm_bits = efc_get_result(EFC);
	if (dw_gpnvm_bits & (1 << dw_gpnvm)) {
		return FLASH_RC_YES;
	}

	return FLASH_RC_NO;
}

/**
 * \brief Set security bit.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_enable_security_bit(void)
{
	return flash_set_gpnvm(0);
}

/**
 * \brief Check if the security bit is set or not.
 * 
 * \retval 1 If the security bit is currently set.
 * \retval 0 If the security bit is currently cleared.
 */
uint32_t flash_is_security_bit_enabled(void)
{
	return flash_is_gpnvm_set(0);
}

/**
 * \brief Read the flash unique ID.
 *
 * \param pdw_data Pointer to a data buffer to store 128-bit unique ID.
 * \param dw_size Data buffer size in DWORD.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_read_unique_id(uint32_t *pdw_data, uint32_t dw_size)
{
	uint32_t uid_buf[4];
	uint32_t dw_idx;

	if (FLASH_RC_OK != efc_perform_read_sequence(EFC, EFC_FCMD_STUI,
					EFC_FCMD_SPUI, uid_buf, 4)) {
		return FLASH_RC_ERROR;
	}

	if (dw_size > 4) {
		/* Only 4 dword to store unique ID */
		dw_size = 4;
	}

	for (dw_idx = 0; dw_idx < dw_size; dw_idx++) {
		pdw_data[dw_idx] = uid_buf[dw_idx];
	}

	return FLASH_RC_OK;
}

#if SAM4S
/**
 * \brief Read the flash user signature.
 *
 * \param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * \param dw_size Data buffer size.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_read_user_signature(uint32_t *p_data, uint32_t dw_size)
{
	if (dw_size > FLASH_USER_SIG_SIZE) {
		/* Only 512 byte to store unique ID */
		dw_size = FLASH_USER_SIG_SIZE;
	}

	/* Send the read user signature commands */
	if (FLASH_RC_OK != efc_perform_read_sequence(EFC, EFC_FCMD_STUS,
					EFC_FCMD_SPUS, p_data, dw_size)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Write the flash user signature.
 *
 * \param dw_address Write address. 
 * \param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * \param dw_size Data buffer size.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_write_user_signature(uint32_t dw_address, const void *p_buffer, uint32_t dw_size)
{
	/* The user signature should be no longer than 512 bytes */
	if (dw_size > FLASH_USER_SIG_SIZE) {
		return FLASH_RC_INVALID;
	}

	/* Write the full page */
	flash_write(dw_address,  p_buffer, dw_size, 0);

	/* Send the write signagure command */
	if (FLASH_RC_OK != efc_perform_command(EFC, EFC_FCMD_WUS, 0)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Erase the flash user signature.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_user_signature(void)
{
	/* Perform the erase user signature command */
	return efc_perform_command(EFC, EFC_FCMD_EUS, 0);
}
#endif

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
