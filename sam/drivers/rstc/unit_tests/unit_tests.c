/**
 * \file
 *
 * \brief Unit tests for RSTC driver.
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

#include <stdint.h>
#include <stdbool.h>
#include <board.h>
#include <sysclk.h>
#include <wdt.h>
#include <flash_efc.h>
#include <rstc.h>
#include <string.h>
#include <unit_test/suite.h>
#include <stdio_serial.h>
#include <conf_test.h>
#include <conf_board.h>

/**
 * \mainpage
 *
 * \section intro Introduction
 * This is the unit test application for the RSTC driver.
 * It consists of test cases for the following functionality:
 * - Watchdog Reset
 * - Software Reset
 *
 * \section files Main Files
 * - \ref unit_tests.c
 * - \ref conf_test.h
 * - \ref conf_board.h
 * - \ref conf_clock.h
 * - \ref conf_uart_serial.h
 *
 * \section device_info Device Info
 * All SAM devices can be used.
 * This example has been tested with the following setup:
 * - sam3n4c_sam3n_ek
 * - sam3s4c_sam3s_ek
 * - sam3sd8c_sam3s_ek2
 * - sam3u4e_sam3u_ek
 * - sam3x8h_sam3x_ek
 *
 * \section compinfo Compilation info
 * This software was written for the GNU GCC and IAR for ARM. Other compilers
 * may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit <a href="http://www.atmel.com/">Atmel</a>.\n
 * Support and FAQ: http://support.atmel.no/
 */

//! \name Unit test configuration
//@{
/**
 * \def CONF_TEST_RSTC
 * \brief Check the reset type of different reasons.
 */
//@}

/* Pointer to the module instance to use for stdio. */
#if defined(__GNUC__)
void (*ptr_get) (void volatile *, int *);
int (*ptr_put) (void volatile *, int);
volatile void *volatile stdio_base;
#endif

/** Start RSTC test flag value. */
#define START_FLAG             0xAA55AA55

/** Test step of chip reset. */
#define STEP_WDT               0
#define STEP_SOFTWARE          1

/** Reset type of chip. */
#define GENERAL_RESET          (0x00 << RSTC_SR_RSTTYP_Pos)
#define BACKUP_RESET           (0x01 << RSTC_SR_RSTTYP_Pos)
#define WDT_RESET              (0x02 << RSTC_SR_RSTTYP_Pos)
#define SOFTWARE_RESET         (0x03 << RSTC_SR_RSTTYP_Pos)
#define USER_RESET             (0x04 << RSTC_SR_RSTTYP_Pos)

/** Flash wait state number. */
#define FLASH_WAIT_STATE_NBR   6

/** WDT load value. */
#define WDT_LOAD_VALUE         26

/** RSTC test structure. */
typedef struct {
	uint32_t dw_flag;
	uint32_t dw_step;
	uint32_t dw_step0_result;
} rstc_test_t;

rstc_test_t st_unit_test;

/**
 * \brief Test Reset Controller.
 *
 * This test check the reset type of RSTC when the chip resets for different reasons.
 *
 * \param test Current test case.
 */
static void run_rstc_test(const struct test_case *test)
{
	uint32_t dw_last_page_addr = LAST_PAGE_ADDRESS;
	static uint32_t dw_reset_type;

	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, FLASH_WAIT_STATE_NBR);

	/* Unlock flash page. */
	flash_unlock(dw_last_page_addr,
			dw_last_page_addr + IFLASH_PAGE_SIZE - 1, NULL, NULL);

	/* Read the RSTC test data in the flash. */
	memcpy((uint8_t *) & st_unit_test, (uint8_t *) dw_last_page_addr,
			sizeof(rstc_test_t));

	/* Get the reset type of this time. */
	dw_reset_type = rstc_get_reset_cause(RSTC);

	switch (st_unit_test.dw_flag) {
	case START_FLAG:
		if (st_unit_test.dw_step == STEP_WDT) {
			/* Prepare transfer to Software reset test. */
			wdt_disable(WDT);
			st_unit_test.dw_flag = START_FLAG;
			st_unit_test.dw_step = STEP_SOFTWARE;
			st_unit_test.dw_step0_result = WDT_RESET;

#if SAM4S
			/* For SAM4S, the EWP command is not supported, the pages requires
			   erased first. */
			flash_erase_page(dw_last_page_addr,
					IFLASH_ERASE_PAGES_4);

			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 0);
#else
			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 1);
#endif

			rstc_start_software_reset(RSTC);
			while (1) {
			}
		} else if (st_unit_test.dw_step == STEP_SOFTWARE) {
			wdt_disable(WDT);
			/* Check first reset test result. */
			test_assert_true(test,
					st_unit_test.dw_step0_result ==
					WDT_RESET,
					"Test: unexpected RSTC reset type!");

			/* Check sencond reset test result. */
			test_assert_true(test, dw_reset_type == SOFTWARE_RESET,
					"Test: unexpected RSTC reset type!");

			/* Clear the flag for next round test. */
			memset((uint8_t *) & st_unit_test, 0,
					sizeof(rstc_test_t));


#if SAM4S
			/* For SAM4S, the EWP command is not supported, the pages requires
			   erased first. */
			flash_erase_page(dw_last_page_addr,
					IFLASH_ERASE_PAGES_4);

			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 0);
#else
			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 1);
#endif
		} else {
			/* Prepare transfer to Watchdog reset test. */
			st_unit_test.dw_flag = START_FLAG;
			st_unit_test.dw_step = STEP_WDT;
			st_unit_test.dw_step0_result = 0;

#if SAM4S
			/* For SAM4S, the EWP command is not supported, the pages requires
			   erased first. */
			flash_erase_page(dw_last_page_addr,
					IFLASH_ERASE_PAGES_4);

			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 0);
#else
			flash_write(dw_last_page_addr,
					(uint8_t *) & st_unit_test,
					sizeof(rstc_test_t), 1);
#endif
			wdt_init(WDT, WDT_MR_WDRSTEN, WDT_LOAD_VALUE,
					WDT_LOAD_VALUE);
			while (1) {
			}
		}
		break;
	default:
		/* Prepare transfer to Watchdog reset test. */
		st_unit_test.dw_flag = START_FLAG;
		st_unit_test.dw_step = STEP_WDT;
		st_unit_test.dw_step0_result = 0;

#if SAM4S
		/* For SAM4S, the EWP command is not supported, the pages requires
		   erased first. */
		flash_erase_page(dw_last_page_addr, IFLASH_ERASE_PAGES_4);

		flash_write(dw_last_page_addr, (uint8_t *) & st_unit_test,
				sizeof(rstc_test_t), 0);
#else
		flash_write(dw_last_page_addr, (uint8_t *) & st_unit_test,
				sizeof(rstc_test_t), 1);
#endif
		wdt_init(WDT, WDT_MR_WDRSTEN, WDT_LOAD_VALUE, WDT_LOAD_VALUE);
		while (1) {
		}
		break;
	}
}

/**
 * \brief Run RSTC driver unit tests.
 */
int main(void)
{
	const usart_serial_options_t usart_serial_options = {
		.baudrate = CONF_TEST_BAUDRATE,
		.paritytype = CONF_TEST_PARITY
	};

	sysclk_init();
	board_init();

	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_TEST_USART, &usart_serial_options);

#if defined(__GNUC__)
	setbuf(stdout, NULL);
#endif

	/* Define all the test cases. */
	DEFINE_TEST_CASE(rstc_test, NULL, run_rstc_test, NULL,
			"Reset Controller, check reset type");

	/* Put test case addresses in an array. */
	DEFINE_TEST_ARRAY(rstc_tests) = {
	&rstc_test,};

	/* Define the test suite */
	DEFINE_TEST_SUITE(rstc_suite, rstc_tests, "SAM RSTC driver test suite");

	/* Run all tests in the test suite. */
	test_suite_run(&rstc_suite);

	while (1) {
		/* Busy-wait forever. */
	}
}
