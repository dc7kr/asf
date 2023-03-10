/**
 * \file
 *
 * Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
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
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "bootloader.h"

/**
 * \mainpage Bootloader of Starter Kit Bootloader Demo
 *
 * \section Introduction
 *
 * Bootloader of Starter Kit Bootloader Demo is a part of Starter Kit
 * Bootloader Demo for SAM Xplained Pro. It is to facilitate firmware
 * upgrade using microSD card. The bootloader checks the trigger flag in the
 * \ref regions_info first. The \ref region_info is normally at the end of
 * the Flash which size is defined by \ref MEM_ERASE_SIZE in the
 * conf_bootloader.h. If the \ref trigger matched, it will try to find a
 * firmware file on microSD card and updates the application firmware from
 * that file.
 *
 * \section boot_op Bootloader Operation
 * The bootloader occupies the first part of the flash memory. The size is
 * defined by \ref BOOT_SIZE which is configurable in conf_bootloader.h.
 * The application has to be shifted by the same offset so that the bootloader
 * can load and execute it rightly. It need to modify the linker scripts for
 * the application project: Increase the flash origion address by
 * \ref BOOT_SIZE and decrease the flash size by \ref BOOT_SIZE and
 * \ref MEM_ERASE_SIZE.
 *
 * On startup, the bootloader will check the \ref trigger, if it was found,
 * it will start firmware update. If there was no trigger, it will validate
 * the main application and jump to it.
 *
 * Software trigger offers a method for application to start a customized
 * firmware upgrade. The application can modified the boot information block to
 * enable once trigger for update and set the update source file name. Once the
 * system is reset the bootloader will use this specified setting to upgrade.
 * Note that once the boot file name in boot information block is changed, it
 * will replace the default boot file list and the file name will not be cleared
 * after upgrade, only the trigger is cleared after upgrade.
 *
 * \section start Starter Kit Bootloader Demo Quick Start
 * This is the step-by-step instructions on how to run the Starter Kit
 * Bootloader Demo:
 * - Perform a complete chip erase.
 * - Program this bootloader.
 * - Load the firmware file into a microSD card, the file is generated by the
 * main application of Starter Kit Bootloader Demo. Please refer the main
 * application documents for details.
 * - Insert the microSD card to I/O1 extension board
 * - Connect the I/O1 extension board to EXT1 of Xplained Pro board.
 * - Connect the OLED extension board to EXT3 of Xplained Pro board.
 * - Press RESET to start the demo.
 *
 * \section Indication Bootloader status Indications
 * - The bootloader truns on the led at the begin to indicate it's running.
 * - The bootloader blinks the led to indicate the firmware update progress.
 * - The bootloader turns off the led before jump to the application.
 *
 * \section boot_cfg Bootloader Configurations
 * Bootloader configuration is in conf_bootloader.h.
 * -# Debug configurations
 *    - \ref DBG_USE_USART : Enable/Disable debug console input/outputs
 *    - \ref DBG_USE_LED : Enable/Disable debug LED output
 *    - \ref DBG_USE_INFO_EDIT : Enable/Disable boot information edit
 *      - Boot Information Edit is started by RESET with some TRIGGER pin down
 * -# Upgrade source media configurations
 *    - \ref MEDIA_USE_SD : Enable/Disable SD/MMC source support
 *      - In the demo SD/MMC source is always used, so it must be defined
 * -# Triggers configurations
 *    - \ref TRIGGER_USE_BUTTONS : Enable/Disable push button triggers
 *      - If buttons triggers are enabled, reset the board with push button
 *        down will trigger firmware upgrade
 *    - \ref TRIGGER_USE_FLAG : Enable/Disable software flag in boot
 *      information to trigger firmware update (in application the boot
 *      information could be modified, then on reset the firmware is updated
 *      and trigger cleared)
 * -# Memory configurations
 *    - \ref MEM_LOCK_BOOT_REGION : Enable/Disable boot region lock
 *    - \ref MEM_USE_FLASH : Must be defined to use internal flash
 *    - Some importent definitions for memory access
 *      - \ref MEM_BLOCK_SIZE : block size while writing
 *      - \ref MEM_ERASE_SIZE : erase block size
 *      - \ref BOOT_SIZE : reserved size for bootloader code
 *      - \ref APP_SIZE : reserved size for application with boot information
 *      - \ref INFO_SIZE : reserved size for boot information
 *        - By default information size is one erase block, information is
 *          placed at the end of application region
 *
 * \section Link Extended Link
 * The OLED Xplained Pro extension board can be found at:
 * <a href="http://www.atmel.com/tools/ATOLED1-XPRO.aspx">OLED1 Xplained Pro</a>
 *
 * The I/O1 Xplained Pro extension board can be found at:
 * <a href="http://www.atmel.com/tools/ATIO1-XPRO.aspx">I/O1 Xplained Pro</a>
 *
 * More information of the bootloader can be found in application note AT02333:
 * <a href="http://www.atmel.com/Images/Atmel-42141-SAM-AT02333-Safe-and-Secure-Bootloader-Implementation-for-SAM3-4_Application-Note.pdf">
 * Safe and Secure Bootloader Implementation for SAM3/4</a>
 */

/* Dump written data after load */
#define DUMP_ENABLE   0

/** Cortex-M Code region start */
#define CM_CODE_START      0x00000080
/** Cortex-M Code region end */
#define CM_CODE_END        0x1FFFFF80
/** Cortex-M SRAM region end */
#define CM_SRAM_START      0x20000000
/** Cortex-M SRAM region end */
#define CM_SRAM_END        0x3FFFFF80

/** Buffer to save one block of data */
static uint32_t app_mem_block_buf[MEM_BLOCK_SIZE / 4];
/** Default boot file check list */
static const char *file_list[MEDIA_FILE_LIST_LEN] = {
	MEDIA_FILE_LIST_DEFAULT
};

/**
 * Get boot region information
 * \return 0 or 1.
 */
static uint32_t _app_boot_get(void)
{
	uint32_t gpmvm_status = flash_is_gpnvm_set(2);
	if (gpmvm_status == FLASH_RC_YES) {
		return 1;
	} else {
		return 0;
	}
}

/** Turn LED on
 * \param led_pin The pin index that connected to LED
 */
#define _app_led_on(led_pin) \
	ioport_set_pin_level(led_pin, DBG_LED_PIN_ON_LEVEL)

/** Turn LED off
 * \param led_pin The pin index that connected to LED
 */
#define _app_led_off(led_pin) \
	ioport_set_pin_level(led_pin, !DBG_LED_PIN_ON_LEVEL)

/** Toggle LED
 * \param led_pin The pin index that connected to LED
 */
#define _app_led_toggle(led_pin) \
	ioport_toggle_pin_level(led_pin)

/**
 * Blink LED
 * \param delay_ms LED toggle delay time in ms
 * \param count Blink times
 */
static void _app_led_blink(uint32_t delay_ms, uint32_t count)
{
	uint32_t i;
	for (i = 0; i < count; i++) {
		_app_led_toggle(DBG_LED_PIN);
		delay_ms(delay_ms);
		_app_led_toggle(DBG_LED_PIN);
		delay_ms(delay_ms);
	}
}

/**
 * Indicate error
 * - ON for 1s
 * - Blink very fast 1s
 * - OFF
 */
static void _app_led_error(void)
{
	_app_led_on(DBG_LED_PIN);
	delay_ms(1000);
	_app_led_blink(20, 25);
	_app_led_off(DBG_LED_PIN);
}

#if DUMP_ENABLE

/**
 * Dump memory block
 *
 * \param hint additional hints to display
 * \param addr memory address
 * \param size block size in bytes
 * \param cut_off only first & last lines dummped if buffer size over 32 bytes
 */
static void _dump_data(char *hint, void *addr, uint32_t size, bool cut_off)
{
	uint32_t line = 16;
	uint8_t *pu8 = (uint8_t *)addr;
	volatile uint32_t i;
	if (hint) {
		dbg_print(hint);
	}

	dbg_print("Dump data @ %x, size %d", addr, size);
	if (size < line) {
		line = size;
	}

	/* First line */
	dbg_print("\r\n[%8x]", pu8);
	for (i = 0; i < line; i++) {
		dbg_print(" %02x", pu8[i]);
	}
	/* Data body cut off */
	if (i < size) {
		if (cut_off) {
			dbg_print("\r\n[........] ...");
			if (i + 16 < size) {
				i
					= (size % 16) ? (size &
						0xFFFFFFFF0) : (size -
						16);
			}
		}
	}

	/* Last lines */
	for (; i < size; i++) {
		if ((i % 16) == 0) {
			dbg_print("\r\n[%8x]", &pu8[i]);
		}

		dbg_print(" %02x", pu8[i]);
	}
	dbg_print("\r\n");
}

#endif

/**
 * Save memory block
 * \param addr Target address to save data
 * \param buf Source buffer
 * \param len Save size (must < MEM_BLOCK_SIZE)
 */
static uint32_t _app_save_block(void *addr, void *buf, uint32_t len)
{
	uint8_t *p_u8 = (uint8_t *)buf;
	if (len < MEM_BLOCK_SIZE) {
		/* Padding with MEM_BLANK_VALUE */
		for (; len < MEM_BLOCK_SIZE; len++) {
			p_u8[len] = MEM_BLANK_VALUE;
		}
	}

	/* Save one block */
	memory_write(addr, buf);

	/* Toggle the led when app load. */
	_app_led_toggle(DBG_LED_PIN);

	return MEM_BLOCK_SIZE;
}

/**
 * Receive new firmware
 *
 * \param info Regions information struct
 * \param no_partition Use single partition, do not split memory to app + buff
 *
 * \return received firmware size
 */
static uint32_t _app_load(struct regions_info *info, bool no_partition)
{
	void *addr, *info_addr;
	uint32_t *p_sign, *p_len;
	uint32_t rx_size;
	uint32_t target_region;
	uint32_t i = 0, flags = 0xFF;
	if (info->boot_region != 0 && info->boot_region != 1) {
		dbg_print("bl: Boot region information (%d) invalid\r\n",
				(int)info->boot_region);
		return 0;
	}

	/* Wait media connection */
	i = 0;
	while (i < MEDIA_NUM_MAX) {
		media_select((enum media_types)i);
		if (media_connect()) {
			dbg_print("bl: Source media %s is ready\r\n",
					media_get_type_str((enum media_types)i));
			break;
		} else if (flags & (1 << i)) {
			flags &= ~(1 << i);
			dbg_print("bl: Source media %s not ready\r\n",
					media_get_type_str((enum media_types)i));
		}

		i++;
	}
	target_region = no_partition ? info->boot_region : (!info->boot_region);

	addr      = (void *)APP_START(target_region);
	info_addr = (void *)INFO_ADDR(false);
	dbg_print("bl: Load to %x, info @ %x\r\n", (unsigned)addr,
			(unsigned)info_addr);

	dbg_print("bl: Unlock download buffer & info area ...\r\n");
	memory_unlock(addr, (void *)((uint32_t)addr + APP_CODE_SIZE - 1));
	memory_unlock(info_addr, (void *)((uint32_t)info_addr + INFO_SIZE - 1));
	dbg_print("bl: Unlock download buffer & info area done\r\n");

	dbg_print("bl: Erase download buffer & info area ...\r\n");
	memory_erase(     addr, APP_CODE_SIZE);
	memory_erase(info_addr, INFO_SIZE);
	dbg_print("bl: Erase download buffer & info area done\r\n");

#ifdef DBG_USE_LED
	_app_led_blink(50, 4);
	_app_led_on(DBG_LED_PIN);
#endif

	/* Clear SW force boot trigger */
#ifdef TRIGGER_USE_FLAG
	info->trigger = TRIGGER_BOOT;
#endif
	p_sign = &info->signature[target_region];
	p_len  = &info->length[target_region];
	rx_size = media_load_file(addr, APP_SIZE,
			(uint8_t *)app_mem_block_buf, MEM_BLOCK_SIZE,
			_app_save_block);
	if (rx_size) {
		*p_sign = region_signature(addr, rx_size);
		*p_len  = rx_size;
	} else {
		*p_sign = 0;
		*p_len  = 0;
	}

	/* Save region information */
	region_info_write(info_addr, info);

	dbg_print("bl: Lock download buffer & info area ...\r\n");
	memory_lock(addr, (void *)((uint32_t)addr + APP_CODE_SIZE - 1));
	memory_lock(info_addr, (void *)((uint32_t)info_addr + INFO_SIZE - 1));
	dbg_print("bl: Lock download buffer & info area done\r\n");

#ifdef DBG_USE_LED
	_app_led_off(DBG_LED_PIN);
#endif

#if DUMP_ENABLE
	_dump_data("Downloaded APP:\r\n", addr, rx_size, true);
	_dump_data("INFO:\r\n", info_addr, INFO_SIZE, true);
#endif
	return rx_size;
}

/**
 * Jump to CM vector table
 *
 * \param code_addr Application start address (vector table address)
 */
#if defined   (__CC_ARM)     /* Keil ??Vision 4 */
__asm__ void jump_to_app(void *code_addr)
{
	mov r1, r0
	ldr r0, [r1, # 4]
	ldr sp, [r1]
	blx r0
}

#elif defined (__ICCARM__)   /* IAR Ewarm 5.41+ */
void jump_to_app(void *code_addr)
{
	UNUSED(code_addr);
	__asm(
			"mov     r1, r0        \n"
			"ldr     r0, [r1, #4]  \n"
			"ldr     sp, [r1]      \n"
			"blx     r0"
			);
}

#elif defined (__GNUC__)     /* GCC CS3 2009q3-68 */
void jump_to_app(void *code_addr)
{
	__asm__(
			"mov   r1, r0        \n"
			"ldr   r0, [r1, #4]  \n"
			"ldr   sp, [r1]      \n"
			"blx   r0"
			);
}

#else /* General C, no stack reset */
void jump_to_app(void *code_addr)
{
	void (*pFct)(void) = NULL;
	/* Point on __main address located in the second word in vector table */
	pFct = (void (*)(void))(*(uint32_t *)((uint32_t)code_addr + 4));
	pFct();
}

#endif

/**
 * Execute the application binary
 *
 * \param addr Application start address.
 * \return If success, no return;
 *         1 - address alignment error;
 *         2 - address not executable.
 */
static uint8_t _app_exec(void *addr)
{
	uint32_t i;
	/* Check parameters */
	if ((uint32_t)addr & 0x7F) {
		return 1;
	}

	if ((uint32_t)addr > CM_SRAM_END) {
		return 2;
	}

	__disable_irq();
	/* Disable SysTick */
	SysTick->CTRL = 0;
	/* Disable IRQs & clear pending IRQs */
	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Switch clock to slow RC */
	osc_enable(OSC_SLCK_32K_RC);
	osc_wait_ready(OSC_SLCK_32K_RC);
	pmc_switch_mck_to_sclk(SYSCLK_PRES_1);
	/* Switch clock to fast RC */
#if SAMG55
	osc_enable(OSC_MAINCK_24M_RC);
	osc_wait_ready(OSC_MAINCK_24M_RC);
#else
	osc_enable(OSC_MAINCK_12M_RC);
	osc_wait_ready(OSC_MAINCK_12M_RC);
#endif	
	pmc_switch_mck_to_mainck(SYSCLK_PRES_1);

	/* Modify vector table location */
	__DSB();
	__ISB();
	SCB->VTOR = ((uint32_t)addr & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();
	__enable_irq();
	/* Jump to application */
	jump_to_app(addr);
	/* Never be here */
	return 0;
}

#ifdef DBG_USE_INFO_EDIT

/**
 * Show regions info
 * \param info Pointer to region information data
 */
static void _app_show_regions_info(struct regions_info *info)
{
	uint32_t boot_region = info->boot_region;
	void *app_addr = (void *)(APP_START(boot_region));
	bool app_valid = false;
	app_valid = region_check_valid(app_addr,
			info->length[ boot_region],
			info->signature[ boot_region]);

	dbg_print("==== Regions Information ====\r\n");

	dbg_print(": trigger flag: %s\r\n", trigger_modes_str[info->trigger]);
	dbg_print(": source file : %s\r\n", info->boot_file_name);

	dbg_print(":----------------------------\r\n");
	dbg_print(": region\t%8d\t%8d\r\n", 0, 1);
	dbg_print(": size  \t%8d\t%8d\r\n",
			(int)info->length[0], (int)info->length[1]);
	dbg_print(": sign  \t%8x\t%8x\r\n",
			(unsigned)info->signature[0],
			(unsigned)info->signature[1]);
	dbg_print(": boot  \t       %c\t       %c\r\n",
			boot_region ? ' ' : 'Y',
			boot_region ? 'Y' : ' ');
	dbg_print(": App @ %d, %x - data %x ...\r\n",
			(int)boot_region, (unsigned)app_addr,
			(unsigned)*(uint32_t *)app_addr);
	dbg_print("bl: App valid: %c\r\n", app_valid ? 'Y' : 'N');
}

/**
 * Save regions info
 */
static void _app_save_regions_info(struct regions_info *info, bool for_boot)
{
	UNUSED(for_boot);
	uint32_t info_addr = INFO_ADDR(for_boot);
	dbg_print("bl: Save regions info to %x\r\n", (unsigned)info_addr);

	dbg_print("bl: Unlock regions info ...\r\n");
	memory_unlock((void *)info_addr, (void *)(info_addr + INFO_SIZE - 1));
	dbg_print("bl: Unlock regions info done\r\n");

	dbg_print("bl: Erase regions info ...\r\n");
	memory_erase((void *)info_addr, INFO_SIZE);
	dbg_print("bl: Erase regions info done\r\n");

	dbg_print("bl: Update regions info\r\n");
	region_info_write((void *)info_addr, info);  /* Save for boot */
	dbg_print("bl: Update regions info done\r\n");

	dbg_print("bl: Lock regions info ...\r\n");
	memory_lock((void *)info_addr, (void *)(info_addr + INFO_SIZE - 1));
	dbg_print("bl: Lock regions info done\r\n");
}

#define ESC 0x1B
#define CR  0x0D
#define LF  0x0A
#define BS  0x08
#define DBGIN_ECHO_ON  (1u << 0) /**< Echo inputs */
#define DBGIN_NUM_ONLY (1u << 1) /**< Accept numbers only */
#define DBGIN_WANT_ESC (1u << 2) /**< Accept ESC to cancel input */
#define DBGIN_WANT_CR  (1u << 3) /**< Accept enter to terminate input */
/** console input buffer */
uint8_t input_buf[16];

/**
 * Wait for input
 * \param keys pointer to buffer to store inputs
 * \param nb_keys max number of keys
 * \param ctrl_flags flags to control inputs (DBGIN_ECHO_ON, DBGIN_NUM_ONLY,
 *                   DBGIN_WANT_ESC and DBGIN_WANT_CR can be used)
 * \return -1 if ESC, else number of input chars
 */
static int _app_dbg_input(uint8_t *keys, int nb_keys, uint32_t ctrl_flags)
{
	uint8_t key;
	int i;
	for (i = 0; i < nb_keys;) {
		if (!dbg_rx_ready()) {
			continue;
		}

		key = dbg_getchar();
		if (key == CR) {
			if (DBGIN_WANT_CR & ctrl_flags) {
				keys[i++] = 0;
				break;
			}

			/* Ignore */
			continue;
		} else if (key == ESC) {
			if (DBGIN_WANT_ESC & ctrl_flags) {
				keys[0] = 0;
				return -1;
			}
		} else {
			if (DBGIN_NUM_ONLY & ctrl_flags) {
				if (key < '0' || key > '9') {
					/* Not accepted */
					continue;
				}
			}

			if (key <= ' ' || key >= 'z') {
				/* Not accepted */
				continue;
			}
		}

		if (ctrl_flags & DBGIN_ECHO_ON) {
			dbg_putchar(key);
		}

		keys[i++] = key;
	}
	return i;
}

/**
 * Boot information edit
 * \param info Pointer to region information block
 */
static void _app_info_edit(struct regions_info *info)
{
	uint32_t i, input_size = 1, input_flags = DBGIN_ECHO_ON;
	uint8_t menu = 0;
	bool wait_input = false;
	int rc;
	ioport_set_pin_dir(DBG_INFO_EDIT_TRIGGER_PIN, IOPORT_DIR_INPUT);
	if (!DBG_INFO_EDIT_TRIGGER_PIN_ACTIVE
			== ioport_get_pin_level(DBG_INFO_EDIT_TRIGGER_PIN)) {
		return;
	}

	while (1) {
		if (wait_input) {
			rc = _app_dbg_input(input_buf, input_size, input_flags);
			switch (menu) {
			case 1: /* Select boot mode */
				if (input_buf[0] >= '0' &&
						input_buf[0] <
						('0' + TRIGGER_NUM_MAX)) {
					int new_trig = input_buf[0] - '0';
					dbg_print("\r\n");
					dbg_print(
							"- Trigger: %d, %s -> %d, %s\r\n",
							(int)info->trigger,
							trigger_get_mode_str((
								enum
								trigger_modes)
							info
							->trigger),
							new_trig,
							trigger_get_mode_str((
								enum
								trigger_modes)
							new_trig));
					info->trigger = input_buf[0] - '0';
					/* Return to default menu */
					menu = 0;
					wait_input = false;
				} else {
					/* Invalid input, wait another */
					dbg_putchar(BS);
				}

				break;

			case 2: /* Change boot file */
				dbg_print("\r\n");
				if (-1 != rc) {
					dbg_print("- Set file: %s\r\n",
							input_buf);
					strcpy(info->boot_file_name,
							(char *)input_buf);
				}

				/* Return to default menu */
				menu = 0;
				wait_input = false;
				break;

			default: /* Main menu */
				switch (input_buf[0]) {
				/* Switch to menu */
				case '1':
				case '2':
					dbg_print("\r\n");
					menu = input_buf[0] - '0';
					wait_input = false;
					break;

				/* Show regions info */
				case 'i':
				case 'I':
					dbg_print("\r\n");
					_app_show_regions_info(info);
					wait_input = false;
					break;

				/* Load regions info */
				case 'l':
				case 'L':
					region_info_read((void *)INFO_ADDR(
							true), info);
					dbg_print("\r\n- Info loaded\r\n");
					_app_show_regions_info(info);
					wait_input = false;
					break;

				/* Save regions info */
				case 's':
				case 'S':
					_app_save_regions_info(info, true);
					dbg_print("\r\n- Info saved\r\n");
					_app_show_regions_info(info);
					wait_input = false;
					break;

				/* Quit edit */
				case 'q':
				case 'Q':
					dbg_print("\r\n");
					return;

				default: /* Invalid input, wait another */
					dbg_putchar(BS);
				}
			}
		}

		switch (menu) {
		case 1: /* Select trigger mode */
			for (i = 0; i < TRIGGER_NUM_MAX; i++) {
				dbg_print("= %u: %s\r\n", (unsigned)i,
						trigger_get_mode_str((enum
						trigger_modes)i));
			}
			dbg_print("= Current boot mode: %d, %s\r\n",
					(int)info->trigger,
					trigger_get_mode_str((enum trigger_modes)
					info->trigger));
			dbg_print("> New mode:");
			input_size = 1;
			input_flags = DBGIN_ECHO_ON | DBGIN_NUM_ONLY;
			break;

		case 2: /* Change boot file */
			dbg_print("==== Available List ====\r\n");
			for (i = 0; i < MEDIA_NUM_MAX; i++) {
				dbg_print(" - In #%u, %s\r\n", (unsigned)i,
						media_get_type_str((enum
						media_types)i));
				media_select((enum media_types)i);
				media_scan_files(false);
			}
			dbg_print("= Current file: %s\r\n",
					info->boot_file_name);
			dbg_print("> New file:");
			input_size = 13;
			input_flags = DBGIN_ECHO_ON | DBGIN_WANT_ESC |
					DBGIN_WANT_CR;
			break;

		default:
			dbg_print("\r\n==== Info Edit ====\r\n");
			dbg_print(" - 1: Change boot mode\r\n");
			dbg_print(" - 2: Change boot file\r\n");
			dbg_print(" - i: Show regions information\r\n");
			dbg_print(" - l: Load/restore boot information\r\n");
			dbg_print(" - s: Save boot information\r\n");
			dbg_print(" - q: quit edit\r\n");
			input_size = 1;
			input_flags = DBGIN_ECHO_ON;
		}
		wait_input = true;
	}
}

#endif

/**
 * Bootloader main entry
 */
int main(void)
{
	uint8_t boot_region = 0; /* Real boot region at this time */
#ifdef DBG_USE_USART
	uint8_t load_region = 0; /* Real region to put loaded data */
#endif
	struct regions_info info;
	void *app_addr = NULL;
	uint32_t app_size = 0;
	enum trigger_modes trigger;
	bool app_valid = false;

	wdt_disable(WDT);
	sysclk_init();
	board_init();

	/* First turn on the led to indicate the bootloader run. */
	ioport_set_pin_dir(DBG_LED_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(DBG_LED_PIN, DBG_LED_PIN_ON_LEVEL);

	dbg_init();
	dbg_print("\r\n\n----------------------\r\n");
	dbg_print("%s Bootloader\r\n", BOARD_NAME);
	dbg_print("Boot region: %x, size %dK\r\n", BOOT0_START, BOOT_SIZE /
			1024);
	dbg_print("App  region: %x, size %dK\r\n", APP0_START,
			(int)APP_SIZE / 1024);
	dbg_print(" - Code %dK + Info %dK\r\n", (int)APP_CODE_SIZE,
			(int)INFO_SIZE);
	dbg_print("----------------------\r\n");

	/* bootloader initialize */
	dbg_print("bl: init ...\r\n");
	trigger_init();
	memory_init();
	media_init(file_list, MEDIA_FILE_LIST_LEN);
	dbg_print("bl: init done\r\n");

	boot_region = _app_boot_get();
	dbg_print("bl: current boot region %d\r\n", (int)boot_region);

#ifdef MEM_LOCK_BOOT_REGION
	dbg_print("bl: lock boot region ...\r\n");
	memory_lock((void *)BOOT0_START, (void *)BOOT0_END);
	dbg_print("bl: lock boot region done\r\n");
#endif

	/* load regions information
	 * Single flash: from last page
	 * Dual flash (remap) : from last page of last physical flash
	 * Dual flash (mirror): from last page of boot flash
	 */
	dbg_print("bl: read regions info ...\r\n");
	region_info_read((void *)INFO_ADDR(true), &info); /* Read for boot */

	app_addr = (void *)(APP_START(boot_region));
	app_valid = region_check_valid(app_addr,
			info.length[boot_region], info.signature[boot_region]);
	dbg_print("bl: read regions info done\r\n");
	dbg_print("bl: trigger flag %s\r\n", trigger_modes_str[info.trigger]);
	dbg_print("bl: region\t%8d\t%8d\r\n", 0, 1);
	dbg_print("bl: size\t%8d\t%8d\r\n", (int)info.length[0],
			(int)info.length[1]);
	dbg_print("bl: sign\t%8x\t%8x\r\n", (unsigned)info.signature[0],
			(unsigned)info.signature[1]);
	dbg_print("bl: boot\t       %c\t       %c\r\n", boot_region ? ' ' : 'Y',
			boot_region ? 'Y' : ' ');
	dbg_print("bl: App @ %d, %x - data %x ...\r\n", (int)boot_region,
			(unsigned)app_addr, (unsigned)*(uint32_t *)app_addr);
	dbg_print("bl: App valid: %c\r\n", app_valid ? 'Y' : 'N');

#ifdef DBG_USE_INFO_EDIT
	/* InfoEdit */
	_app_info_edit(&info);
#endif

	/* bootloader trigger check */
	dbg_print("bl: trigger ...\r\n");
	trigger = trigger_poll(&info);
	dbg_print("bl: trigger mode %s\r\n", trigger_modes_str[trigger]);
	if (TRIGGER_BOOT == trigger) {
		goto main_run_app_check;
	}

	/* Now any other trigger load file to update application directly */
#ifdef DBG_USE_LED
	_app_led_blink(100, 1);
#endif
	/* Update media file information */
	if (info.boot_file_name[0]) {
		dbg_print("bl: boot file assigned, set it\r\n");
		media_set_file_name(info.boot_file_name);
	}

main_load_app:
	/* load new firmware */
#ifdef DBG_USE_USART
	load_region = boot_region;
	dbg_print("bl: download @ %d ...\r\n", load_region);
#endif
	app_size = _app_load(&info, true);
	if (app_size == 0) {
		_app_led_error();
		dbg_print("bl: download fail, retry\r\n");
		goto main_load_app;
	} else {
		dbg_print("bl: download done, size %d\r\n", (int)app_size);
	}

main_run_app_check:

	/* Is application valid */
	dbg_print("bl: check app @ %d is valid\r\n", (int)info.boot_region);
	app_valid = region_check_valid(app_addr, info.length[info.boot_region],
			info.signature[info.boot_region]);
	if (!app_valid) {
		dbg_print("bl: application is not valid\r\n");
		_app_led_error();
		dbg_print("bl: reload firmware\r\n");
		goto main_load_app;
	}

	dbg_print("bl: application is valid, run\r\n");


	/* Turn off the led before jump to the app. */
	_app_led_off(DBG_LED_PIN);

	/* cleanup */
	dbg_print("bl: cleanup ...\r\n");
	media_cleanup();
	trigger_cleanup();
	memory_cleanup();
	dbg_print("bl: cleanup done\r\n");

	/* load application */
	dbg_print("bl: load application ...\r\n\n");

#ifdef DBG_USE_USART
	delay_ms(50); /* Wait USART lines idle */

	dbg_cleanup();

	delay_ms(50);
#endif

	/* run application */
	_app_exec(app_addr);

	return (int)app_addr;
}
