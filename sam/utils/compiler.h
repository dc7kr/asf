/**
 * \file
 *
 * \brief Commonly used includes, types and macros.
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

#ifndef UTILS_COMPILER_H
#define UTILS_COMPILER_H

/**
 * \defgroup group_sam_utils Compiler abstraction layer and code utilities
 *
 * Compiler abstraction layer and code utilities for 32-bit SAM.
 * This module provides various abstraction layers and utilities to make code compatible between different compilers.
 *
 * \{
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#if (defined __ICCARM__)
#  include <intrinsics.h>
#endif

#include "parts.h"

#ifdef __ICCARM__
/*! \name Compiler Keywords
 *
 * Port of some keywords from GCC to IAR Embedded Workbench.
 */
/*! @{ */
#define __asm__              asm
#define __inline__           inline
#define __volatile__
/*! @} */
#endif

/**
 * \def barrier
 * \brief Memory barrier
 */
#if defined(__GNUC__)
#  define barrier()        asm volatile("" ::: "memory")
#elif defined(__ICCARM__)
#  define barrier()        __asm__ __volatile__ ("")
#endif

/* ____ M A C R O S ________________________________________________________ */

/**
 * \def __always_inline
 * \brief The function should always be inlined.
 *
 * This annotation instructs the compiler to ignore its inlining
 * heuristics and inline the function no matter how big it thinks it
 * becomes.
 */
#if (defined __GNUC__)
#	define __always_inline   inline __attribute__((__always_inline__))
#elif (defined __ICCARM__)
#	define __always_inline   _Pragma("inline=forced") 
#endif

/*! \brief This macro is used to test fatal errors.
 *
 * The macro tests if the expression is false. If it is, a fatal error is
 * detected and the application hangs up. If TEST_SUITE_DEFINE_ASSERT_MACRO
 * is defined, a unit test version of the macro is used, to allow execution
 * of further tests after a false expression.
 *
 * \param expr  Expression to evaluate and supposed to be nonzero.
 */
#if defined(_ASSERT_ENABLE_)
#  if defined(TEST_SUITE_DEFINE_ASSERT_MACRO)
	// Assert() is defined in unit_test/suite.h
//#    include "unit_test/suite.h"
//#  else
#undef TEST_SUITE_DEFINE_ASSERT_MACRO
#    define Assert(expr) \
	{\
		if (!(expr)) while (true);\
	}
#  endif
#else
#  define Assert(expr) ((void) 0)
#endif

/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
    #define WEAK __attribute__ ((weak))
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define NO_INIT
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
#elif defined (  __GNUC__  )
    #define NO_INIT
#endif

/*! \name Usual Types
 */
/*! @{ */
typedef unsigned char           Bool; /*!< Boolean. */
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
typedef unsigned char           bool; /*!< Boolean. */
#endif
#endif
/*! @} */

/* ____ M A C R O S ________________________________________________________ */

/*! \name Usual Constants
 */
/*! @{ */
#define DISABLE   0
#define ENABLE    1
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
#define false     0
#define true      1
#endif
#endif
#define PASS      0
#define FAIL      1
#define LOW       0
#define HIGH      1
/*! @} */

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))

#endif /* UTILS_COMPILER_H */
