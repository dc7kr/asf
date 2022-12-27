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
PRJ_PATH = ../../../../../../../..

# Target CPU architecture: ap, ucr1, ucr2 or ucr3
ARCH = ucr1

# Target part: none, ap7xxx or uc3xxxxx
PART = uc3b0256

# Target device flash memory details (used by the avr32program programming
# tool: [cfi|internal]@address
FLASH = internal@0x80000000

# Clock source to use when programming; xtal, extclk or int
PROG_CLOCK = int

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = avr32_services_dsplib_example_dsp16_operators_evk1101.elf

# List of C source files.
CSRCS = \
       avr32/drivers/flashc/flashc.c                      \
       avr32/drivers/gpio/gpio.c                          \
       avr32/drivers/intc/intc.c                          \
       avr32/drivers/pm/pm.c                              \
       avr32/drivers/pm/pm_conf_clocks.c                  \
       avr32/drivers/pm/power_clocks_lib.c                \
       avr32/drivers/usart/usart.c                        \
       avr32/services/dsp/dsplib/at32uc/basic/operators/op_dsp16_ln_at32uc.c \
       avr32/services/dsp/dsplib/at32uc/basic/operators/op_dsp16_sqrt_at32uc.c \
       avr32/services/dsp/dsplib/examples/dsp16_operators/operators_example.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_dsp16_fix_sqrt.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_dsp16_kfix_ln.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_dsp32_fix_sqrt.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_dsp32_kfix_ln.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_asin.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_exp.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_ln.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_log10.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_log2.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_pow.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_rand.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_fix_sin.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_kfix_cos.c \
       avr32/services/dsp/dsplib/generic/basic/operators/op_kfix_sin.c \
       avr32/services/dsp/dsplib/utils/modules/debug/dsp_debug_print.c \
       avr32/services/dsp/dsplib/utils/modules/debug/dsp_debug_printf.c \
       avr32/services/dsp/dsplib/utils/modules/debug/dsp_debug_read.c \
       avr32/services/dsp/dsplib/utils/modules/debug/evk1101/usart1/dsp_debug_setup_usart1_at32uc.c

# List of assembler source files.
ASSRCS = \
       avr32/drivers/intc/exception.S                     \
       avr32/utils/startup/startup_uc3.S                  \
       avr32/utils/startup/trampoline_uc3.S

# List of include paths.
INC_PATH = \
       avr32/boards                                       \
       avr32/boards/evk1101                               \
       avr32/drivers/cpu/cycle_counter                    \
       avr32/drivers/flashc                               \
       avr32/drivers/gpio                                 \
       avr32/drivers/intc                                 \
       avr32/drivers/pm                                   \
       avr32/drivers/usart                                \
       avr32/services/dsp/dsplib/at32uc                   \
       avr32/services/dsp/dsplib/include                  \
       avr32/services/dsp/dsplib/utils/modules/debug      \
       avr32/services/dsp/dsplib/utils/programs/adpcm_encode/docsrc \
       avr32/services/dsp/dsplib/utils/programs/adpcm_streaming/docsrc \
       avr32/services/dsp/dsplib/utils/programs/data_extract/docsrc \
       avr32/services/dsp/dsplib/utils/programs/data_get/docsrc \
       avr32/services/dsp/dsplib/utils/programs/data_print/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/benchmark/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/print_complex_vect/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/print_real_vect/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/serial_scope/docsrc \
       avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator/docsrc \
       avr32/utils                                        \
       avr32/utils/preprocessor                           \
       common/boards                                      \
       common/utils \
       ./avr32/services/dsp/dsplib/examples/dsp16_operators/at32uc3b0256_evk1101/gcc

# Additional search paths for libraries.
LIB_PATH = 

# List of libraries to use during linking.
LIBS = 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT = avr32/utils/linker_scripts/at32uc3b/0256/gcc/link_uc3b0256.lds

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
       -D BOARD=EVK1101                                   \
       -D DSP_OPERATORS                                   \
       -D DSP_OPTIMIZATION=DSP_OPTI_SPEED

# Extra flags to use when linking
LDFLAGS = \
       -nostartfiles -Wl,-e,_trampoline
