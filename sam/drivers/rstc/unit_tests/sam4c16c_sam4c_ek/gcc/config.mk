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
PRJ_PATH = ../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m4

# Target part: none, sam3n4 or sam4l4aa
PART = sam4c16c_0

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = rstc_unit_tests_flash.elf
TARGET_SRAM = rstc_unit_tests_sram.elf

# List of C source files.
CSRCS = \
       common/services/clock/sam4c/sysclk.c               \
       common/services/serial/usart_serial.c              \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common/utils/stdio/read.c                          \
       common/utils/stdio/write.c                         \
       common/utils/unit_test/suite.c                     \
       sam/boards/sam4c_ek/init.c                         \
       sam/drivers/gpbr/gpbr.c                            \
       sam/drivers/pmc/pmc.c                              \
       sam/drivers/pmc/sleep.c                            \
       sam/drivers/rstc/rstc.c                            \
       sam/drivers/rstc/unit_tests/unit_tests.c           \
       sam/drivers/uart/uart.c                            \
       sam/drivers/usart/usart.c                          \
       sam/drivers/wdt/wdt.c                              \
       sam/utils/cmsis/sam4c/source/templates/gcc/startup_sam4c.c \
       sam/utils/cmsis/sam4c/source/templates/system_sam4c.c \
       sam/utils/syscalls/gcc/syscalls.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/clock                              \
       common/services/ioport                             \
       common/services/serial                             \
       common/services/serial/sam_uart                    \
       common/utils                                       \
       common/utils/stdio/stdio_serial                    \
       sam/boards                                         \
       sam/boards/sam4c_ek                                \
       sam/drivers/gpbr                                   \
       sam/drivers/pmc                                    \
       sam/drivers/rstc                                   \
       sam/drivers/rstc/unit_tests                        \
       sam/drivers/rstc/unit_tests/sam4c16c_sam4c_ek      \
       sam/drivers/uart                                   \
       sam/drivers/usart                                  \
       sam/drivers/wdt                                    \
       sam/utils                                          \
       sam/utils/cmsis/sam4c/include                      \
       sam/utils/cmsis/sam4c/source/templates             \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC \
       sam/drivers/rstc/unit_tests/sam4c16c_sam4c_ek/gcc 

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM4l_math                                 \
       m                                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/sam4c/gcc/sam4c16c_0_flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/sam4c/gcc/sam4c16c_0_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/sam4c_ek/debug_scripts/gcc/sam4c_ek_flash_0.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/sam4c_ek/debug_scripts/gcc/sam4c_ek_sram_0.gdb

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
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

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
       -D BOARD=SAM4C_EK                                  \
       -D TEST_SUITE_DEFINE_ASSERT_MACRO                  \
       -D _ASSERT_ENABLE_                                 \
       -D __SAM4C16C_0__                                  \
       -D printf=iprintf

# Extra flags to use when linking
LDFLAGS = \
