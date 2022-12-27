#
# Copyright (c) 2009-2010 Atmel Corporation. All rights reserved.
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

# Target CPU architecture: ap, ucr1, ucr2 or ucr3
ARCH = ucr2

# Target part: none, ap7xxx or uc3xxxxx
PART = uc3a3256

# Target device flash memory details (used by the avr32program programming
# tool: [cfi|internal]@address
FLASH = internal@0x80000000

# Clock source to use when programming; xtal, extclk or int
PROG_CLOCK = int

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = example_freertos.elf

# List of C source files.
CSRCS = \
       avr32/boards/evk1104/led.c                         \
       avr32/drivers/flashc/flashc.c                      \
       avr32/drivers/gpio/gpio.c                          \
       avr32/drivers/intc/intc.c                          \
       avr32/drivers/pm/pm.c                              \
       avr32/drivers/pm/pm_conf_clocks.c                  \
       avr32/drivers/pm/power_clocks_lib.c                \
       avr32/drivers/tc/tc.c                              \
       avr32/drivers/usart/usart.c                        \
       avr32/drivers/usbb/_asf_v1/enum/device/usb_device_task.c \
       avr32/drivers/usbb/_asf_v1/enum/device/usb_standard_request.c \
       avr32/drivers/usbb/_asf_v1/enum/host/usb_host_enum.c \
       avr32/drivers/usbb/_asf_v1/enum/host/usb_host_task.c \
       avr32/drivers/usbb/_asf_v1/enum/usb_task.c         \
       avr32/drivers/usbb/_asf_v1/usb_drv.c               \
       avr32/services/usb/_asf_v1/class/cdc/examples/cdc_example.c \
       avr32/services/usb/_asf_v1/class/cdc/examples/device_cdc_task.c \
       avr32/services/usb/_asf_v1/class/cdc/examples/enum/device/usb_descriptors.c \
       avr32/services/usb/_asf_v1/class/cdc/examples/enum/device/usb_specific_request.c \
       avr32/services/usb/_asf_v1/class/cdc/examples/host_cdc_task.c \
       avr32/services/usb/_asf_v1/class/cdc/examples/uart_usb_lib.c \
       avr32/utils/debug/print_funcs.c                    \
       common/services/fifo/fifo.c                        \
       thirdparty/freertos/source/croutine.c              \
       thirdparty/freertos/source/list.c                  \
       thirdparty/freertos/source/portable/gcc/avr32_uc3/port.c \
       thirdparty/freertos/source/portable/gcc/avr32_uc3/read.c \
       thirdparty/freertos/source/portable/gcc/avr32_uc3/write.c \
       thirdparty/freertos/source/portable/memmang/heap_3.c \
       thirdparty/freertos/source/queue.c                 \
       thirdparty/freertos/source/tasks.c                 \
       thirdparty/freertos/source/timers.c

# List of assembler source files.
ASSRCS = \
       avr32/utils/startup/trampoline_uc3.S               \
       thirdparty/freertos/source/portable/gcc/avr32_uc3/exception.S

# List of include paths.
INC_PATH = \
       avr32/boards                                       \
       avr32/boards/evk1104                               \
       avr32/components/joystick/skrhabe010               \
       avr32/drivers/cpu/cycle_counter                    \
       avr32/drivers/flashc                               \
       avr32/drivers/gpio                                 \
       avr32/drivers/intc                                 \
       avr32/drivers/pm                                   \
       avr32/drivers/tc                                   \
       avr32/drivers/usart                                \
       avr32/drivers/usbb/_asf_v1                         \
       avr32/drivers/usbb/_asf_v1/enum                    \
       avr32/drivers/usbb/_asf_v1/enum/device             \
       avr32/drivers/usbb/_asf_v1/enum/host               \
       avr32/services/usb/_asf_v1                         \
       avr32/services/usb/_asf_v1/class/cdc               \
       avr32/services/usb/_asf_v1/class/cdc/examples      \
       avr32/services/usb/_asf_v1/class/cdc/examples/conf \
       avr32/services/usb/_asf_v1/class/cdc/examples/enum \
       avr32/services/usb/_asf_v1/class/cdc/examples/enum/device \
       avr32/services/usb/_asf_v1/class/cdc/examples/prj_freertos/freertos \
       avr32/utils                                        \
       avr32/utils/debug                                  \
       avr32/utils/preprocessor                           \
       common/boards                                      \
       common/services/fifo                               \
       common/utils                                       \
       thirdparty/freertos/source/include                 \
       thirdparty/freertos/source/portable/gcc/avr32_uc3  \
       thirdparty/newlib_addons/libs/include \
       ./avr32/services/usb/_asf_v1/class/cdc/examples/prj_freertos/at32uc3a3256_evk1104/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/newlib_addons/libs/at32ucr2            

# List of libraries to use during linking.
LIBS =  \
       newlib_addons-at32ucr2-speed_opt                  

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT = avr32/utils/linker_scripts/at32uc3a3/256/gcc/link_uc3a3256.lds

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -Os

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
       -D BOARD=EVK1104                                   \
       -D FREERTOS_USED

# Extra flags to use when linking
LDFLAGS = \
        -Wl,-e,_trampoline
