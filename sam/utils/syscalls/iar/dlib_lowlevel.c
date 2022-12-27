/**
 * \file
 *
 * \brief Syscalls for SAM (IAR).
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

#if defined __ICCARM__	/* IAR Ewarm 5.41+ */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <yfuns.h>
#include "board.h"
#include "uart.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

extern __weak size_t __write(int handle, const unsigned char *buf,
		size_t bufSize)
{
	size_t nChars = 0;

	/* Check for the command to flush all handles */
	if (handle == -1) {
		return 0;
	}

	/* Check for stdout and stderr (only necessary if FILE descriptors are enabled.) */
	if (handle != 1 && handle != 2) {
		/* remove warnings */
		return 0xfffffff;
	}

	for ( /* Empty */ ; bufSize > 0; --bufSize) {
		while (!uart_is_tx_ready(CONSOLE_UART));
		uart_write(CONSOLE_UART, *buf);
		++buf;
		++nChars;
	}

	return nChars;
}

extern __weak size_t __read(int handle, unsigned char *buf, size_t bufSize)
{
	size_t nChars = 0;
	uint8_t c = 0;

	/* Check for stdin (only necessary if FILE descriptors are enabled) */
	if (handle != 0) {
		/* remove warnings */
		return 0xfffffff;
	}

	for (; bufSize > 0; --bufSize) {

		while (uart_read(CONSOLE_UART, &c));

		if (c == 0) {
			break;
		}
		*buf++ = c;
		++nChars;
	}

	return nChars;
}

/**
 * \brief Outputs a character on the UART.
 *
 * \param c  Character to output.
 *
 * \return The character that was output.
 */
extern __weak signed int putchar(signed int c)
{
	while (!uart_is_tx_ready(CONSOLE_UART));
	uart_write(CONSOLE_UART, c);

	return c;
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* __ICCARM__ */
