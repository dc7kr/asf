/* This source file is part of the ATMEL QTouch Library Release 4.4 */
/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the QTouch Libary configuration options for
 *         AVR 32-bit QTouch method Capacitive Touch acquisition.
 *
 * - Compiler:           IAR EWAVR and GNU GCC for AVR32
 * - Supported devices:  AT32UC3A0/A1 Series, AT32UC3B0/B1 Series 
 *                       AND AT32UC3C0/C1 Series 
 * - Userguide:          QTouch Library User Guide - doc8207.pdf.
 * - Support:            http://www.atmel.com/design-support/
 *
 * \author               Atmel Corporation: http://www.atmel.com
 *
 * Copyright (c) 2011-2014, Atmel Corporation All rights reserved.
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
 ******************************************************************************/


#ifndef _TOUCH_QT_CONFIG_AVR_UC3_H_
#define _TOUCH_QT_CONFIG_AVR_UC3_H_

/*
 * Default configuration for AVR UC3
 */

/*
* Number of Channels(dependent on the library used).Please refer to the user guide
* more information on selecting the number of channels.
*
* Possible values: 8,16,32.
*/
#ifndef QT_NUM_CHANNELS
    #define QT_NUM_CHANNELS 32
#endif

/*
* Delay cycles that determine the capacitance charge pulse width.
*
* Possible values: 3,4,5,10,25,50
*/
#ifndef QT_DELAY_CYCLES
    #define QT_DELAY_CYCLES 5
#endif

/*
* Enabling the _ROTOR_SLIDER_ constant will link the library need for using rotors
* and sliders.
*
* Possible values: comment/uncomment the define
*/
#ifndef _ROTOR_SLIDER_
    #define _ROTOR_SLIDER_
#endif

/*
* Define the ports to be used for SNS1 and SNSK1 pins. SNS1 and SNSK1 port pins
* can be available on the same port or on different ports
*
* Possible values: refer to the device data sheet and QTouch libraries user guide.
*/
#ifndef SNS
	#define SNS A
    #warning SNS should be defined. Now default value of Port A is taken.
#endif
#ifndef SNSK
	#define SNSK B
    #warning SNSK should be defined. Now default value of Port B is taken.
#endif

/*
* Define the ports to be used for SNS2 and SNSK2 pins. SNS2 and SNSK2 port pins
* can be available on the same port or on different ports
*
* Possible values: refer to the device data sheet and QTouch libraries user guide.
*/
/* Should be defined if required */

/**************************************************************/
/* Please do not change any part of the code below this line. */
/**************************************************************/
#if (QT_DELAY_CYCLES == 0)
#error 'QT_DELAY_CYCLES can only have values in the range 1-255.'
#endif

#define _STATIC_PORT_PIN_CONF_ 1

#ifdef _SNS2_SNSK2_SAME_PORT_
    #define QTOUCH_SNS_PORT_COUNT 2
#else
    #define QTOUCH_SNS_PORT_COUNT 1
#endif

#ifdef _SNS1_SNSK1_SAME_PORT_
    #define INTRABURST_1 1
#else
    #define INTRABURST_1 0
#endif

#ifdef _SNS2_SNSK2_SAME_PORT_
    #define INTRABURST_2 1
#else
    #define INTRABURST_2 0
#endif


#ifndef _DEBUG_INTERFACE_
    //#define _DEBUG_INTERFACE_
#endif
#ifndef _QTOUCH_
    #define _QTOUCH_
#endif

#endif //_TOUCH_QT_CONFIG_AVR_UC3_H_
