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
PRJ_PATH = ../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m4

# Target part: none, sam3n4 or sam4l4aa
PART = sam4lc4c

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = sam4l_qtouch_demo_flash.elf
TARGET_SRAM = sam4l_qtouch_demo_sram.elf

# List of C source files.
CSRCS = \
       common/services/clock/sam4l/dfll.c                 \
       common/services/clock/sam4l/osc.c                  \
       common/services/clock/sam4l/pll.c                  \
       common/services/clock/sam4l/sysclk.c               \
       common/services/delay/sam/cycle_counter.c          \
       common/services/sleepmgr/sam4l/sleepmgr.c          \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common/utils/stdio/read.c                          \
       common/utils/stdio/write.c                         \
       sam/applications/sam4l_qtouch_demo/app.c           \
       sam/applications/sam4l_qtouch_demo/event.c         \
       sam/applications/sam4l_qtouch_demo/main.c          \
       sam/applications/sam4l_qtouch_demo/qtouch/touch.c  \
       sam/applications/sam4l_qtouch_demo/ui.c            \
       sam/boards/sam4l_ek/board_monitor.c                \
       sam/boards/sam4l_ek/init.c                         \
       sam/components/display/c42364a/c42364a_lcdca.c     \
       sam/drivers/ast/ast.c                              \
       sam/drivers/bpm/bpm.c                              \
       sam/drivers/eic/eic.c                              \
       sam/drivers/flashcalw/flashcalw.c                  \
       sam/drivers/lcdca/lcdca.c                          \
       sam/drivers/pdca/pdca.c                            \
       sam/drivers/usart/usart.c                          \
       sam/drivers/wdt/wdt_sam4l.c                        \
       sam/utils/cmsis/sam4l/source/templates/exceptions.c \
       sam/utils/cmsis/sam4l/source/templates/gcc/startup_sam4l.c \
       sam/utils/cmsis/sam4l/source/templates/system_sam4l.c \
       sam/utils/syscalls/gcc/syscalls.c                  \
       thirdparty/qtouch/devspecific/sam4/sam4l/common/BitBangSPI_Master.c \
       thirdparty/qtouch/qdebug/QDebugTransport.c         \
       thirdparty/qtouch/qdebug/QDebug_sam4l.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/clock                              \
       common/services/delay                              \
       common/services/ioport                             \
       common/services/sleepmgr                           \
       common/utils                                       \
       sam/applications/sam4l_qtouch_demo                 \
       sam/applications/sam4l_qtouch_demo/qtouch          \
       sam/applications/sam4l_qtouch_demo/sam4lc4c_sam4l_ek \
       sam/boards                                         \
       sam/boards/sam4l_ek                                \
       sam/components/display/c42364a                     \
       sam/drivers/ast                                    \
       sam/drivers/bpm                                    \
       sam/drivers/eic                                    \
       sam/drivers/flashcalw                              \
       sam/drivers/lcdca                                  \
       sam/drivers/pdca                                   \
       sam/drivers/usart                                  \
       sam/drivers/wdt                                    \
       sam/utils                                          \
       sam/utils/cmsis/sam4l/include                      \
       sam/utils/cmsis/sam4l/source/templates             \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/qtouch/devspecific/sam4/sam4l           \
       thirdparty/qtouch/devspecific/sam4/sam4l/common    \
       thirdparty/qtouch/devspecific/sam4/sam4l/include   \
       thirdparty/qtouch/qdebug \
       sam/applications/sam4l_qtouch_demo/sam4lc4c_sam4l_ek/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/qtouch/devspecific/sam4/sam4l/lib/gcc  

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM4l_math                                 \
       sam4l-qt-gnu                                       \
       m                                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/sam4l/sam4l4/gcc/flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/sam4l/sam4l4/gcc/sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/sam4l_ek/debug_scripts/gcc/sam4l_ek_flash.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/sam4l_ek/debug_scripts/gcc/sam4l_ek_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

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
       -D BOARD=SAM4L_EK                                  \
       -D _QTOUCH_                                        \
       -D __SAM4LC4C__                                    \
       -D printf=iprintf

# Extra flags to use when linking
LDFLAGS = \
