#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ../../../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m4

# Target part: none, sam3n4 or sam4l4aa
PART = sam4e16e

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = nand_flash_example2_flash.elf
TARGET_SRAM = nand_flash_example2_sram.elf

# List of C source files.
CSRCS = \
       common/components/memory/nand_flash/nand_flash_ebi/example2/nand_flash_ecc_sw_example.c \
       common/components/memory/nand_flash/nand_flash_ebi/ftl_lib/nand_flash_mem.c \
       common/components/memory/nand_flash/nand_flash_ebi/nand_flash_ecc_sw.c \
       common/components/memory/nand_flash/nand_flash_ebi/nand_flash_model.c \
       common/components/memory/nand_flash/nand_flash_ebi/nand_flash_raw_smc.c \
       common/components/memory/nand_flash/nand_flash_ebi/nand_flash_spare_scheme.c \
       common/services/clock/sam4e/sysclk.c               \
       common/services/delay/sam/cycle_counter.c          \
       common/services/serial/usart_serial.c              \
       common/services/storage/ctrl_access/ctrl_access.c  \
       common/services/storage/ecc_hamming/ecc-sw.c       \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common/utils/stdio/read.c                          \
       common/utils/stdio/write.c                         \
       sam/boards/sam4e_ek/init.c                         \
       sam/drivers/ebi/smc/smc.c                          \
       sam/drivers/matrix/matrix.c                        \
       sam/drivers/pio/pio.c                              \
       sam/drivers/pio/pio_handler.c                      \
       sam/drivers/pmc/pmc.c                              \
       sam/drivers/pmc/sleep.c                            \
       sam/drivers/uart/uart.c                            \
       sam/drivers/usart/usart.c                          \
       sam/utils/cmsis/sam4e/source/templates/exceptions.c \
       sam/utils/cmsis/sam4e/source/templates/gcc/startup_sam4e.c \
       sam/utils/cmsis/sam4e/source/templates/system_sam4e.c \
       sam/utils/syscalls/gcc/syscalls.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/components/memory/nand_flash/nand_flash_ebi \
       common/components/memory/nand_flash/nand_flash_ebi/example2/sam4e16e_sam4e_ek \
       common/components/memory/nand_flash/nand_flash_ebi/ftl_lib \
       common/services/clock                              \
       common/services/delay                              \
       common/services/gpio                               \
       common/services/ioport                             \
       common/services/serial                             \
       common/services/serial/sam_uart                    \
       common/services/storage/ctrl_access                \
       common/services/storage/ecc_hamming                \
       common/utils                                       \
       common/utils/stdio/stdio_serial                    \
       sam/boards                                         \
       sam/boards/sam4e_ek                                \
       sam/drivers/ebi/smc                                \
       sam/drivers/matrix                                 \
       sam/drivers/pio                                    \
       sam/drivers/pmc                                    \
       sam/drivers/uart                                   \
       sam/drivers/usart                                  \
       sam/utils                                          \
       sam/utils/cmsis/sam4e/include                      \
       sam/utils/cmsis/sam4e/source/templates             \
       sam/utils/fpu                                      \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC \
       common/components/memory/nand_flash/nand_flash_ebi/example2/sam4e16e_sam4e_ek/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       common/components/memory/nand_flash/nand_flash_ebi/ftl_lib/gcc \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       _nand_flash_cortexm4                               \
       arm_cortexM4lf_math                                \
       m                                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/sam4e/sam4e16e/gcc/flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/sam4e/sam4e16e/gcc/sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/sam4e_ek/debug_scripts/gcc/sam4e_ek_flash.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/sam4e_ek/debug_scripts/gcc/sam4e_ek_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = all

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS =  \
       -mfloat-abi=softfp                                 \
       -mfpu=vfpv4                                       

# Extra flags to use when compiling.
CFLAGS =  \
       -mfloat-abi=softfp                                 \
       -mfpu=vfpv4                                       

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ARM_MATH_CM4=true                               \
       -D BOARD=SAM4E_EK                                  \
       -D __SAM4E16E__                                    \
       -D printf=iprintf

# Extra flags to use when linking
LDFLAGS = \
