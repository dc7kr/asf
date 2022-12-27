/**
 * \file
 *
 * \brief Implementation of low level disk I/O module skeleton for FatFS.
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
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#include "compiler.h"
#include "diskio.h"
#include "ctrl_access.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "rtc.h"

/** Default sector size */
#define SECTOR_SIZE_DEFAULT 512

/** Supported sector size. These values are based on the LUN function:
 * mem_sector_size(). */
#define SECTOR_SIZE_512   1
#define SECTOR_SIZE_1024 2
#define SECTOR_SIZE_2048 4
#define SECTOR_SIZE_4096 8

/* Prototype definition, avoid compile warning */
DWORD get_fattime(void);

/**
 * \brief Initialize a disk.
 *
 * \param drv Physical drive number (0..).
 *
 * \return RES_OK if a disk is found, otherwise RES_ERROR.
 */
DSTATUS disk_initialize(BYTE drv)
{
	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	if (drv > MAX_LUN) {
		/* At least one of the LUN should be defined */
		return STA_NOINIT;
	}

	/* The memory should already be initialized */
	return RES_OK;
}

/**
 * \brief  Return disk status.
 *
 * \param drv Physical drive number (0..).
 *
 * \return RES_OK if the disk is ready, otherwise RES_ERROR.
 */
DSTATUS disk_status(BYTE drv)
{
	if (mem_test_unit_ready(drv) == CTRL_GOOD) {
		return RES_OK;
	} else {
		return STA_NODISK;
	}
}

/**
 * \brief  Read sector(s).
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
#if ACCESS_MEM_TO_RAM
	uint8_t uc_sector_size = mem_sector_size(drv);
	uint32_t i;
	uint32_t ul_last_sector_num;

	if (uc_sector_size == 0) {
		return RES_ERROR;
	}

	/* Check valid address */
	mem_read_capacity(drv, &ul_last_sector_num);
	if ((sector + count * uc_sector_size) >
			(ul_last_sector_num + 1) * uc_sector_size) {
		return RES_PARERR;
	}

	/* Read the data */
	for (i = 0; i < count; i++) {
		if (memory_2_ram(drv, sector + uc_sector_size *
				SECTOR_SIZE_DEFAULT * i,
				buff +
				uc_sector_size *
				SECTOR_SIZE_DEFAULT * i) !=
				CTRL_GOOD) {
			return RES_ERROR;
		}
	}

	return RES_OK;

#else
	return RES_ERROR;
#endif
}

/**
 * \brief  Write sector(s). The FatFs module will issue multiple sector transfer
 * request (count > 1) to the disk I/O layer. The disk function should process
 *  the multiple sector transfer properly. Do not translate it into
 *  multiple sector transfers to the media, or the data read/write
 *  performance may be drastically decreased.
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
#if _READONLY == 0
DRESULT disk_write(BYTE drv, BYTE const *buff, DWORD sector, BYTE count)
{
#if ACCESS_MEM_TO_RAM
	uint8_t uc_sector_size = mem_sector_size(drv);
	uint32_t i;
	uint32_t ul_last_sector_num;

	if (uc_sector_size == 0) {
		return RES_ERROR;
	}

	/* Check valid address */
	mem_read_capacity(drv, &ul_last_sector_num);
	if ((sector + count * uc_sector_size) >
			(ul_last_sector_num + 1) * uc_sector_size) {
		return RES_PARERR;
	}

	/* Read the data */
	for (i = 0; i < count; i++) {
		if (ram_2_memory(drv, sector + uc_sector_size *
				SECTOR_SIZE_DEFAULT * i,
				buff +
				uc_sector_size *
				SECTOR_SIZE_DEFAULT * i) !=
				CTRL_GOOD) {
			return RES_ERROR;
		}
	}

	return RES_OK;

#else
	return RES_ERROR;
#endif
}

#endif /* _READONLY */

/**
 * \brief  Miscellaneous functions, which support the following commands:
 *
 * CTRL_SYNC    Make sure that the disk drive has finished pending write
 * process. When the disk I/O module has a write back cache, flush the
 * dirty sector immediately.
 * In read-only configuration, this command is not needed.
 *
 * GET_SECTOR_COUNT    Return total sectors on the drive into the DWORD variable
 * pointed by buffer.
 * This command is used only in f_mkfs function.
 *
 * GET_BLOCK_SIZE    Return erase block size of the memory array in unit
 * of sector into the DWORD variable pointed by Buffer.
 * When the erase block size is unknown or magnetic disk device, return 1.
 * This command is used only in f_mkfs function.
 *
 * GET_SECTOR_SIZE    Return sector size of the memory array.
 *
 * \param drv Physical drive number (0..).
 * \param ctrl Control code.
 * \param buff Buffer to send/receive control data.
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res = RES_PARERR;

	switch (ctrl) {
	case GET_BLOCK_SIZE:
		*(DWORD *)buff = 1;
		res = RES_OK;
		break;

	/* Get the number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT:
	{
		uint32_t ul_last_sector_num;

		/* Check valid address */
		mem_read_capacity(drv, &ul_last_sector_num);

		*(DWORD *)buff = ul_last_sector_num + 1;

		res = RES_OK;
	}
	break;

	/* Get sectors on the disk (WORD) */
	case GET_SECTOR_SIZE:
	{
		uint8_t uc_sector_size = mem_sector_size(drv);

		if ((uc_sector_size != SECTOR_SIZE_512) &&
				(uc_sector_size != SECTOR_SIZE_1024) &&
				(uc_sector_size != SECTOR_SIZE_2048) &&
				(uc_sector_size != SECTOR_SIZE_4096)) {
			/* The sector size is not supported by the FatFS */
			return RES_ERROR;
		}

		*(U8 *)buff = uc_sector_size * SECTOR_SIZE_DEFAULT;

		res = RES_OK;
	}
	break;

	/* Make sure that data has been written */
	case CTRL_SYNC:
		res = RES_OK;
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}

/**
 * \brief Current time returned is packed into a DWORD value.
 *
 * The bit field is as follows:
 *
 * bit31:25  Year from 1980 (0..127)
 *
 * bit24:21  Month (1..12)
 *
 * bit20:16  Day in month(1..31)
 *
 * bit15:11  Hour (0..23)
 *
 * bit10:5   Minute (0..59)
 *
 * bit4:0    Second (0..59)
 *
 * \return Current time.
 */
DWORD get_fattime(void)
{
	uint32_t ul_time;
	uint32_t ul_hour, ul_minute, ul_second;
	uint32_t ul_year, ul_month, ul_day, ul_week;

	/* Retrieve date and time */
	rtc_get_time(RTC, &ul_hour, &ul_minute, &ul_second);
	rtc_get_date(RTC, &ul_year, &ul_month, &ul_day, &ul_week);

	ul_time = ((ul_year - 1980) << 25)
			| (ul_month << 21)
			| (ul_day << 16)
			| (ul_hour << 11)
			| (ul_minute << 5)
			| (ul_second << 0);

	return ul_time;
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
