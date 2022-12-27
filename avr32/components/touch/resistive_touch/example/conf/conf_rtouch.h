/**
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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
#ifndef _CONF_RTOUCH_H_
#define _CONF_RTOUCH_H_

#include "board.h"
/*============================ DEFINES =======================================*/

//! Sample count per signal line.
#define RTOUCH_OVERSAMPLING 4
//! Number of right shifts required to scale down to 10 bits after oversampling.
#define RTOUCH_SAMPLESCALE 2

//! Deepest sleep mode allowed when waiting for touch detect.
#define TOUCH_DETECT_SLEEP_MODE SLEEPMGR_SAVE
//! Deepest sleep mode allowed when sampling.
#define TOUCH_SAMPLE_SLEEP_MODE SLEEPMGR_IDLE

//! Set to 0 if the timer module should be used for delayed resampling.
#define TOUCH_USE_IMMEDIATE_RESAMPLE 1



/*============================ PLATFORM DEPENDENT DEFINES ====================*/

#define RTOUCH_ADC_INT_LEVEL	1

#if BOARD == EVK1105
#define RTOUCH_ADC_IRQ		        AVR32_ADC_IRQ

//! IO pin used for XL line.
#define RTOUCH_XH_PIN		         AVR32_ADC_AD_4_PIN //PA25
#define RTOUCH_XH_PIN_FUNCTION	 AVR32_ADC_AD_4_FUNCTION
//! ADC channel number for XL
#define RTOUCH_ADC_XH_CHANNEL	   (1 << 4)

//! IO pin used for XH line.
#define RTOUCH_XL_PIN		         AVR32_ADC_AD_6_PIN //PA27
#define RTOUCH_XL_PIN_FUNCTION	 AVR32_ADC_AD_6_FUNCTION
#define RTOUCH_ADC_XL_CHANNEL	   (1 << 6)

//! IO pin used for YL line.
#define RTOUCH_YL_PIN		         AVR32_ADC_AD_5_PIN //PA26
#define RTOUCH_YL_PIN_FUNCTION	 AVR32_ADC_AD_5_FUNCTION
#define RTOUCH_ADC_YL_CHANNEL	   (1 << 5)

//! IO pin used for YH line.
#define RTOUCH_YH_PIN		         AVR32_ADC_AD_7_PIN //PA28
#define RTOUCH_YH_PIN_FUNCTION	 AVR32_ADC_AD_7_FUNCTION
#define RTOUCH_ADC_YH_CHANNEL	   (1 << 7)

//! AVR32 ADC module used for touch panel sampling.
#define RTOUCH_ADC	 	           AVR32_ADC

#elif BOARD == UC3C_EK

#define RTOUCH_ADC_IRQ		       AVR32_ADCIFA_SEQUENCER0_IRQ

//! IO pin used for XL line.
#define RTOUCH_XH_PIN		         AVR32_ADCIN1_PIN //PA5
#define RTOUCH_XH_PIN_FUNCTION	 AVR32_ADCIN1_FUNCTION
#define RTOUCH_ADC_XH_CHANNEL    AVR32_ADCIFA_INP_ADCIN1

//! IO pin used for XH line.
#define RTOUCH_XL_PIN		         AVR32_ADCIN15_PIN //PA13
#define RTOUCH_XL_PIN_FUNCTION	 AVR32_ADCIN15_FUNCTION
#define RTOUCH_ADC_XL_CHANNEL    AVR32_ADCIFA_INN_ADCIN15

//! IO pin used for YL line.
#define RTOUCH_YL_PIN		         AVR32_ADCIN13_PIN //PA24
#define RTOUCH_YL_PIN_FUNCTION	 AVR32_ADCIN13_FUNCTION
#define RTOUCH_ADC_YL_CHANNEL    AVR32_ADCIFA_INN_ADCIN13

//! IO pin used for YH line.
#define RTOUCH_YH_PIN		         AVR32_ADCIN0_PIN //PA4
#define RTOUCH_YH_PIN_FUNCTION	 AVR32_ADCIN0_FUNCTION
#define RTOUCH_ADC_YH_CHANNEL    AVR32_ADCIFA_INP_ADCIN0

//! AVR32 ADC module used for touch panel sampling.
#define RTOUCH_ADC		          AVR32_ADCIFA

#else

#error 'Board not supported'

#endif

/*============================ MACROS ========================================*/

#endif
/* EOF */
