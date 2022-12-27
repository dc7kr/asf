/**
 * \file
 *
 * \brief Enhanced Embedded Flash Controller (EEFC) driver for SAM.
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

#include "efc.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_drivers_efc_group Enhanced Embedded Flash Controller (EEFC)
 *
 * The Enhanced Embedded Flash Controller ensures the interface of the Flash block with
 * the 32-bit internal bus.
 *
 * @{
 */

/* Address definition for read operation */
#if (SAM3XA || SAM3U4)
# define READ_BUFF_ADDR0    IFLASH0_ADDR
# define READ_BUFF_ADDR1    IFLASH1_ADDR
#elif (SAM3S || SAM4S || SAM3N)
# define READ_BUFF_ADDR     IFLASH_ADDR
#elif (SAM3U)
# define READ_BUFF_ADDR     IFLASH0_ADDR
#else
# warning Only reading unique id for sam3 is implemented.
#endif

/* Flash Writing Protection Key */
#define FWP_KEY    0x5Au

/*
 * Local function declaration.
 * Because they are RAM functions, they need 'extern' declaration.
 */
extern void efc_write_fmr(Efc *p_efc, uint32_t dw_fmr);
extern uint32_t efc_perform_fcr(Efc *p_efc, uint32_t dw_fcr);

/** 
 * \brief Initialize the EFC controller.
 *
 * \param dw_access_mode 0 for 128-bit, EEFC_FMR_FAM for 64-bit. 
 * \param dw_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful.
 */
uint32_t efc_init(Efc *p_efc, uint32_t dw_access_mode, uint32_t dw_fws)
{
	efc_write_fmr(p_efc, dw_access_mode | EEFC_FMR_FWS(dw_fws));
	return EFC_RC_OK;
}

/** 
 * \brief Enable the flash ready interrupt.
 *
 * \param p_efc Pointer to an EFC instance.
 */
void efc_enable_frdy_interrupt(Efc *p_efc)
{
	uint32_t dw_fmr = p_efc->EEFC_FMR;

	efc_write_fmr(p_efc, dw_fmr | EEFC_FMR_FRDY);
}

/** 
 * \brief Disable the flash ready interrupt.
 *
 * \param p_efc Pointer to an EFC instance.
 */
void efc_disable_frdy_interrupt(Efc *p_efc)
{
	uint32_t dw_fmr = p_efc->EEFC_FMR;

	efc_write_fmr(p_efc, dw_fmr & (~EEFC_FMR_FRDY));
}

/** 
 * \brief Set flash access mode.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_mode 0 for 128-bit, EEFC_FMR_FAM for 64-bit.
 */
void efc_set_flash_access_mode(Efc *p_efc, uint32_t dw_mode)
{
	uint32_t dw_fmr = p_efc->EEFC_FMR & (~EEFC_FMR_FAM);

	efc_write_fmr(p_efc, dw_fmr | dw_mode);
}

/** 
 * \brief Get flash access mode.
 *
 * \param p_efc Pointer to an EFC instance.
 *
 * \return 0 for 128-bit or EEFC_FMR_FAM for 64-bit.
 */
uint32_t efc_get_flash_access_mode(Efc *p_efc)
{
	return (p_efc->EEFC_FMR & EEFC_FMR_FAM);
}

/** 
 * \brief Set flash wait state.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_fws The number of wait states in cycle (no shift).
 */
void efc_set_wait_state(Efc *p_efc, uint32_t dw_fws)
{
	uint32_t dw_fmr = p_efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk);

	efc_write_fmr(p_efc, dw_fmr | EEFC_FMR_FWS(dw_fws));
}

/** 
 * \brief Get flash wait state.
 *
 * \param p_efc Pointer to an EFC instance.
 *
 * \return The number of wait states in cycle (no shift).
 */
uint32_t efc_get_wait_state(Efc *p_efc)
{
	return ((p_efc->EEFC_FMR & EEFC_FMR_FWS_Msk) >> EEFC_FMR_FWS_Pos);
}

/** 
 * \brief Perform the given command and wait until its completion (or an error).
 *
 * \note Unique ID commands are not supported, use efc_read_unique_id.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_command Command to perform.
 * \param dw_argument Optional command argument.
 *
 * \note This function will automatically choose to use IAP function.
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t efc_perform_command(Efc *p_efc, uint32_t dw_command,
		uint32_t dw_argument)
{
	// Unique ID commands are not supported.
	if (dw_command == EFC_FCMD_STUI || dw_command == EFC_FCMD_SPUI) {
		return EFC_RC_NOT_SUPPORT;
	}

#if (SAM3XA || SAM3U4)
	// Use IAP function with 2 parameters in ROM.
	static uint32_t(*iap_perform_command) (uint32_t, uint32_t);
	uint32_t dw_efc_nb = (p_efc == EFC0) ? 0 : 1;

	iap_perform_command =
			(uint32_t(*)(uint32_t, uint32_t))
			*((uint32_t *) CHIP_FLASH_IAP_ADDRESS);
	iap_perform_command(dw_efc_nb,
			EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(dw_argument) |
			EEFC_FCR_FCMD(dw_command));
	return (p_efc->EEFC_FSR & (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE));
#elif (SAM3N || SAM3S || SAM4S || SAM3U)
	// Use IAP function with 2 parameter in ROM.
	static uint32_t(*iap_perform_command) (uint32_t, uint32_t);

	iap_perform_command =
			(uint32_t(*)(uint32_t, uint32_t))
			*((uint32_t *) CHIP_FLASH_IAP_ADDRESS);
	iap_perform_command(0,
			EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(dw_argument) |
			EEFC_FCR_FCMD(dw_command));
	return (p_efc->EEFC_FSR & (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE));
#else
	// Use RAM Function.
	return efc_perform_fcr(p_efc,
			EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(dw_argument) |
			EEFC_FCR_FCMD(dw_command));

#endif
}

/** 
 * \brief Get the current status of the EEFC.
 *
 * \note This function clears the value of some status bits (FLOCKE, FCMDE).
 *
 * \param p_efc Pointer to an EFC instance.
 *
 * \return The current status.
 */
uint32_t efc_get_status(Efc *p_efc)
{
	return p_efc->EEFC_FSR;
}

/** 
 * \brief Get the result of the last executed command.
 *
 * \param p_efc Pointer to an EFC instance.
 *
 * \return The result of the last executed command.
 */
uint32_t efc_get_result(Efc *p_efc)
{
	return p_efc->EEFC_FRR;
}

/** 
 * \brief Perform read sequence. Supported sequences are read Unique ID and 
 * read User Signature
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_cmd_st Start command to perform.
 * \param dw_cmd_sp Stop command to perform.
 * \param p_dw_buf Pointer to an data buffer.
 * \param dw_size Buffer size.
 *
 * \return 0 if successful, otherwise returns an error code.
 */
#ifdef __ICCARM__
__ramfunc
#else
__attribute__ ((section(".ramfunc")))
#endif
uint32_t efc_perform_read_sequence(Efc *p_efc,
		uint32_t dw_cmd_st, uint32_t dw_cmd_sp,
		uint32_t *p_dw_buf, uint32_t dw_size)
{
	volatile uint32_t dw_status;
	uint32_t dw_cnt;

#if (SAM3U4 || SAM3XA)
	uint32_t *p_dw_data =
			(uint32_t *) ((p_efc == EFC0) ?
			READ_BUFF_ADDR0 : READ_BUFF_ADDR1);
#elif (SAM3S || SAM4S || SAM3N || SAM3U)
	uint32_t *p_dw_data = (uint32_t *) READ_BUFF_ADDR;
#else
	return EFC_RC_NOT_SUPPORT;
#endif

	if (p_dw_buf == NULL) {
		return EFC_RC_INVALID;
	}

	p_efc->EEFC_FMR |= (0x1u << 16);
	
	/* Send the Start Read command */
	p_efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(0)
			| EEFC_FCR_FCMD(dw_cmd_st);

	/* Wait for the FRDY bit in the Flash Programming Status Register 
	 * (EEFC_FSR) falls.
	 */
	do {
		dw_status = p_efc->EEFC_FSR;
	} while ((dw_status & EEFC_FSR_FRDY) == EEFC_FSR_FRDY);

	/* The data is located in the first address of the Flash
	 * memory mapping.
	 */
	for (dw_cnt = 0; dw_cnt < dw_size; dw_cnt++) {
		p_dw_buf[dw_cnt] = p_dw_data[dw_cnt];
	}

	/* To stop the read mode */
	p_efc->EEFC_FCR =
			EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(0) |
			EEFC_FCR_FCMD(dw_cmd_sp);

	/* Wait for the FRDY bit in the Flash Programming Status Register (EEFC_FSR)
	 * rises.
	 */
	do {
		dw_status = p_efc->EEFC_FSR;
	} while ((dw_status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);

	p_efc->EEFC_FMR &= ~(0x1u << 16);

	return EFC_RC_OK;
}

/** 
 * \brief Set mode register.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_fmr Value of mode register
 */
#ifdef __ICCARM__
__ramfunc
#else
__attribute__ ((section(".ramfunc")))
#endif
void efc_write_fmr(Efc *p_efc, uint32_t dw_fmr)
{
	p_efc->EEFC_FMR = dw_fmr;
}

/** 
 * \brief Perform command.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param dw_fcr Flash command.
 *
 * \return The current status.
 */
#ifdef __ICCARM__
__ramfunc
#else
__attribute__ ((section(".ramfunc")))
#endif
uint32_t efc_perform_fcr(Efc *p_efc, uint32_t dw_fcr)
{
	volatile uint32_t dw_status;

	p_efc->EEFC_FCR = dw_fcr;
	do {
		dw_status = p_efc->EEFC_FSR;
	} while ((dw_status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);

	return (dw_status & (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE));
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
