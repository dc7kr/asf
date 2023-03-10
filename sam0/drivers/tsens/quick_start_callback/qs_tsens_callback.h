/**
 * \file
 *
 * \brief SAM TSENS Quick Start
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
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
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

/**
 * \page asfdoc_sam0_tsens_basic_use_case_callback Quick Start Guide for TSENS - Callback
 *
 * In this use case, the TSENS will measure the temperature using interrupt
 * driven conversion. When the temperature value have been measured, a
 * callback will be called that signals the main application that conversion
 * is compete.
 *
 * The TSENS will be set up as follows:
 *  - GCLK generator 0 (GCLK main) clock source
 *  - Free running disabled
 *  - Run in standby
 *  - Window monitor disabled
 *  - All events (input and generation) disabled
 *  - Calibration value which read from NVM or user set
 *
 * \section asfdoc_sam0_tsens_callback_basic_use_case_callback_setup Setup
 *
 * \subsection asfdoc_sam0_tsens_callback_basic_use_case_prereq Prerequisites
 * There are no special setup requirements for this use-case.
 *
 * \subsection asfdoc_sam0_tsens_callback_basic_use_case_callback_code Code
 * Add to the main application source file, outside of any functions:
 * \snippet qs_tsens_callback.c module_inst
 * \snippet qs_tsens_callback.c result
 *
 * Callback function:
 * \snippet qs_tsens_callback.c job_complete_callback
 *
 * Copy-paste the following setup code to your user application:
 * \snippet qs_tsens_callback.c setup
 *
 * Add to user application initialization (typically the start of \c main()):
 * \snippet qs_tsens_callback.c setup_init
 *
 * \subsection asfdoc_sam0_tsens_basic_use_case_callback_workflow Workflow
 * -# Create a module software instance structure for the TSENS module to store
 *    the TSENS driver state while it is in use.
 *    \snippet qs_tsens_callback.c module_inst
 *    \note This should never go out of scope as long as the module is in use.
 *          In most cases, this should be global.
 *
 * -# Create a variable for the TSENS sample to be stored in by the driver
 *    asynchronously.
 *    \snippet qs_tsens_callback.c result
 * -# Create a callback function that will be called each time the TSENS completes
 *    an asynchronous read job.
 * \snippet qs_tsens_callback.c job_complete_callback
 * -# Configure the TSENS module.
 *  -# Create a TSENS module configuration struct, which can be filled out to
 *     adjust the configuration of a physical TSENS peripheral.
 *     \snippet qs_tsens_callback.c setup_config
 *  -# Initialize the TSENS configuration struct with the module's default values.
 *     \snippet qs_tsens_callback.c setup_config_defaults
 *     \note This should always be performed before using the configuration
 *           struct to ensure that all values are initialized to known default
 *           settings.
 *
 *  -# Set TSENS configurations.
 *     \snippet qs_tsens_callback.c setup_set_config
 *  -# Enable the TSENS module so that conversions can be made.
 *     \snippet qs_tsens_callback.c setup_enable
 * -# Register and enable the TSENS read complete callback handler.
 *  -# Register the user-provided read complete callback function with
 *     the driver, so that it will be run when an asynchronous read job
 *     completes.
 *     \snippet qs_tsens_callback.c setup_register_callback
 *  -# Enable the read complete callback so that it will generate callbacks.
 *     \snippet qs_tsens_callback.c setup_enable_callback
 *
 * \section asfdoc_sam0_tsens_basic_use_case_callback_use Use Case
 *
 * \subsection asfdoc_sam0_tsens_basic_use_case_callback_use_code Code
 * Copy-paste the following code to your user application:
 * \snippet qs_tsens_callback.c main
 *
 * \subsection asfdoc_sam0_tsens_basic_use_case_callback_use_workflow Workflow
 *  -# Enable interrupts, so that callbacks can be generated by the driver.
 *     \snippet qs_tsens_callback.c enable_interrupts
 *  -# Start an asynchronous TSENS conversion, to store TSENS sample into the
 *     variable and generate a callback when complete.
 *     \snippet qs_tsens_callback.c start_tsens_job
 *  -# Wait until the asynchronous conversion is complete.
 *     \snippet qs_tsens_callback.c job_complete_poll
 *  -# Enter an infinite loop once the conversion is complete.
 *     \snippet qs_tsens_callback.c inf_loop
 */
