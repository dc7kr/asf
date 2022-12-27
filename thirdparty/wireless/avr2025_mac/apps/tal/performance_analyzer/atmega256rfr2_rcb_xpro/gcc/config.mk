#
# Copyright (c) 2010 Atmel Corporation. All rights reserved.
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

# Microcontroller: atxmega128a1, atmega128, attiny261, etc.
MCU = atmega256rfr2

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = tal_performance_analyzer.elf

# C source files located from the top-level source directory
CSRCS = \
       common/services/clock/mega/sysclk.c                \
       common/services/serial/usart_serial.c              \
       common/utils/stdio/read.c                          \
       common/utils/stdio/write.c                         \
       mega/boards/atmega256rfr2_rcb_xpro/init.c          \
       mega/drivers/usart/usart_megarf.c                  \
       thirdparty/wireless/addons/sio2host/uart/sio2host.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/init_state.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/main.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/peer_search_initiator.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/peer_search_receptor.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/per_mode_common_utils.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/per_mode_initiator.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/per_mode_receptor.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/perf_api_serial_handler.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/range_measure.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/user_interface.c \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/src/wait_for_event.c \
       thirdparty/wireless/avr2025_mac/source/pal/common_hw_timer/mega/hw_timer.c \
       thirdparty/wireless/avr2025_mac/source/pal/common_sw_timer/common_sw_timer.c \
       thirdparty/wireless/avr2025_mac/source/pal/mega/drivers/tc/tc_megarf.c \
       thirdparty/wireless/avr2025_mac/source/pal/pal.c   \
       thirdparty/wireless/avr2025_mac/source/resources/buffer/src/bmm.c \
       thirdparty/wireless/avr2025_mac/source/resources/queue/src/qmm.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_ed.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_init.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_irq_handler.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_pib.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_pwr_mgmt.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_rx.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_rx_enable.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_slotted_csma.c \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/src/tal_tx.c \
       thirdparty/wireless/avr2025_mac/source/tal/src/tal_helper.c \
       thirdparty/wireless/avr2025_mac/source/tfa/atmegarfr2/src/tfa.c

# Assembler source files located from the top-level source directory
ASSRCS = 

# Include path located from the top-level source directory
INC_PATH = \
       common/boards                                      \
       common/services/clock                              \
       common/services/delay                              \
       common/services/gpio                               \
       common/services/ioport                             \
       common/services/serial                             \
       common/services/serial/megarf_usart                \
       common/utils                                       \
       common/utils/stdio/stdio_serial                    \
       mega/boards                                        \
       mega/boards/atmega256rfr2_rcb_xpro                 \
       mega/drivers/cpu                                   \
       mega/drivers/usart                                 \
       mega/utils                                         \
       mega/utils/preprocessor                            \
       thirdparty/wireless/addons/sio2host/uart           \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/atmega256rfr2_rcb_xpro \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/inc \
       thirdparty/wireless/avr2025_mac/include            \
       thirdparty/wireless/avr2025_mac/source/mac/inc     \
       thirdparty/wireless/avr2025_mac/source/pal         \
       thirdparty/wireless/avr2025_mac/source/pal/common_hw_timer \
       thirdparty/wireless/avr2025_mac/source/pal/common_hw_timer/mega \
       thirdparty/wireless/avr2025_mac/source/pal/common_sw_timer \
       thirdparty/wireless/avr2025_mac/source/pal/mega/drivers/tc \
       thirdparty/wireless/avr2025_mac/source/resources/buffer/inc \
       thirdparty/wireless/avr2025_mac/source/resources/queue/inc \
       thirdparty/wireless/avr2025_mac/source/tal/atmegarfr2/inc \
       thirdparty/wireless/avr2025_mac/source/tal/inc     \
       thirdparty/wireless/avr2025_mac/source/tfa/inc \
       thirdparty/wireless/avr2025_mac/apps/tal/performance_analyzer/atmega256rfr2_rcb_xpro/gcc

# Library paths from the top-level source directory
LIB_PATH =  \
       thirdparty/wireless/avr2025_mac/source/pal/common_hw_timer/mega/lib

# Libraries to link with the project
LIBS =  \
       megarfr2_lib_hw_timer                             

# Additional options for debugging. By default the common Makefile.in will
# add -gdwarf-2.
DBGFLAGS = 

# Optimization settings
OPTIMIZATION = -Os

# Extra flags used when creating an EEPROM Intel HEX file. By default the
# common Makefile.in will add -j .eeprom
# --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0.
EEPROMFLAGS = 

# Extra flags used when creating an Intel HEX file. By default the common
# Makefile.in will add -R .eeprom -R .usb_descriptor_table.
FLASHFLAGS = 

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
       -D ANTENNA_DIVERSITY=1                             \
       -D BOARD=RCB256RFR2_XPRO                           \
       -D ENABLE_TFA                                      \
       -D FFD                                             \
       -D HIGHEST_STACK_LAYER=TAL                         \
       -D HIGH_DATA_RATE_SUPPORT                          \
       -D TAL_TYPE=ATMEGARFR2                             \
       -D TRX_REG_RAW_VALUE

# Extra flags to use when linking
LDFLAGS = 