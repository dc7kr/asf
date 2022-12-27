#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
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
PRJ_PATH = ../../../../../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m3

# Target part: none, sam3n4 or sam4l4aa
PART = sam3u4e

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = generic_example_flash.elf
TARGET_SRAM = generic_example_sram.elf

# List of C source files.
CSRCS = \
       common/services/clock/sam3u/sysclk.c               \
       common/services/sleepmgr/sam/sleepmgr.c            \
       common/services/usb/class/hid/device/generic/example/main.c \
       common/services/usb/class/hid/device/generic/example/sam3u4e_sam3u_ek/ui.c \
       common/services/usb/class/hid/device/generic/udi_hid_generic.c \
       common/services/usb/class/hid/device/generic/udi_hid_generic_desc.c \
       common/services/usb/class/hid/device/udi_hid.c     \
       common/services/usb/udc/udc.c                      \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       sam/boards/sam3u_ek/init.c                         \
       sam/drivers/pio/pio.c                              \
       sam/drivers/pio/pio_handler.c                      \
       sam/drivers/pmc/pmc.c                              \
       sam/drivers/udphs/udphs_device.c                   \
       sam/utils/cmsis/sam3u/source/templates/exceptions.c \
       sam/utils/cmsis/sam3u/source/templates/gcc/startup_sam3u.c \
       sam/utils/cmsis/sam3u/source/templates/system_sam3u.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/clock                              \
       common/services/gpio                               \
       common/services/sleepmgr                           \
       common/services/usb                                \
       common/services/usb/class/hid                      \
       common/services/usb/class/hid/device               \
       common/services/usb/class/hid/device/generic       \
       common/services/usb/class/hid/device/generic/example \
       common/services/usb/class/hid/device/generic/example/sam3u4e_sam3u_ek \
       common/services/usb/udc                            \
       common/utils                                       \
       sam/boards                                         \
       sam/boards/sam3u_ek                                \
       sam/drivers/pio                                    \
       sam/drivers/pmc                                    \
       sam/drivers/udphs                                  \
       sam/utils                                          \
       sam/utils/cmsis/sam3u/include                      \
       sam/utils/cmsis/sam3u/source/templates             \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include \
       ./common/services/usb/class/hid/device/generic/example/sam3u4e_sam3u_ek/gcc

# Additional search paths for libraries.
LIB_PATH = 

# List of libraries to use during linking.
LIBS = 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/sam3u/sam3u4/gcc/flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/sam3u/sam3u4/gcc/sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/sam3u_ek/debug_scripts/gcc/sam3u_ek_flash.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/sam3u_ek/debug_scripts/gcc/sam3u_ek_sram.gdb

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
       -D BOARD=SAM3U_EK                                  \
       -D UDD_ENABLE                                      \
       -D __SAM3U4E__

# Extra flags to use when linking
LDFLAGS = \
