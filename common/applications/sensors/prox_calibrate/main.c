/**
 * \file
 *
 * \brief Common Sensor Service Proximity Sensor Calibration Example
 *
 * \mainpage
 *
 * \section intro Introduction
 *
 * This application performs a multi-step manual calibration on a 3-channel
 * proximity sensor device.  An object (e.g. a hand) is placed in front of
 * the proximity sensor at the desired distance for proximity detection.
 * The user presses the button on the processor board to advance to the
 * next channel.  After each channel's threshold has been set, the program
 * will continuously display an indication of the current proximity
 * status for each channel.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 * Support and FAQ: http://support.atmel.no/
 *
 * Copyright (C) 2012 Atmel Corporation. All rights reserved.
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

#include <stdio.h>

#include <board.h>
#include <led.h>

#include <sensors/sensor.h>


// Proximity sensor settings

#define SET_PROX_CURRENT    (true)	// true = manually set proximity current
#define	PROX_CURRENT_mA     (150)	// current for proximity sensor LEDs (mA)

// Switch and LED definitions

#if UC3
#  define SWITCH_PRESSED  (gpio_get_pin_value (GPIO_PUSH_BUTTON_0) ==     \
				           GPIO_PUSH_BUTTON_0_PRESSED)
#elif XMEGA
#  define SWITCH_PRESSED  (ioport_get_value (GPIO_PUSH_BUTTON_0) == false)
#endif

#if (BOARD==UC3_L0_XPLAINED)
#  define NUM_BLINK_LEDS (4)
#  define ALL_LEDS       (LED1 | LED2 | LED3 | LED4)   
									// don't blink LED0 (shared GPIO)
#  define PROMPT_LED     LED4       // blink while waiting for user input
   static const uint32_t led_array[NUM_BLINK_LEDS] = {LED1, LED2, LED3, LED4};

#else
#  define NUM_BLINK_LEDS   (4)
#  define ALL_LEDS         (LED0 | LED1 | LED2 | LED3)
#  define PROMPT_LED       LED0     // blink while waiting for user input
   static const uint32_t led_array[NUM_BLINK_LEDS] = {LED0, LED1, LED2, LED3};

#endif

#define LOOP_DELAY_MSEC 100         // main loop delay value (msec)
#define BLINK_DELAY     0xF000      // prompt blink delay loop count


static const char * const prox_labels[4] = {
	// strings to display based on proximity values
	"none  ",		// PROXIMITY_NONE
	"FAR   ",		// PROXIMITY_FAR
	"MEDIUM",		// PROXIMITY_MEDIUM
	"NEAR  " 		// PROXIMITY_NEAR
};



/*! \brief User prompt routine
 *
 * This routine prompts the user to press the button and then waits
 * for him to do so.
 *
 * \return  Nothing.
 */
void prompt_user(char *prompt_string)
{
	volatile int     delay;

	// Output prompt string

	printf("%s\r\n", prompt_string);

	// Wait for user to push button before continuing

	LED_Off(ALL_LEDS);                  // turn all LED's off

	while ( ! SWITCH_PRESSED) {

		// Just blink LED until button is pushed

		LED_Toggle(PROMPT_LED);
		
		for (delay = 0; delay < BLINK_DELAY; delay++)
			{;}                         // do nothing
	}

	LED_Off(PROMPT_LED);               // leave LED off 

	while (SWITCH_PRESSED)
		{;}                             // wait until button is released

	return;
}

/*! \brief Proximity sensor threshold calibration application
 *
 * This application illustrates the use of the sensor_calibrate() function
 * to set the proximity detection thresholds in a 3-channel proximity sensor.  
 * This threshold is the level at which the sensor will report that an object
 * is near the device.
 *
 * The calibration sequence requires three steps, one for each channel.  During 
 * each step, an object is placed at the desired distance in front of the
 * sensor, and the user presses the button on the board to trigger a proximity
 * reading.
 *
 * After Step 3 is completed, the threshold values for the sensor are
 * calculated and are written to non-volatile (flash) memory on the
 * microcontroller.  These values will continue to be used for future
 * proximity readings, unless they are overwritten by an application
 * calling the sensor_set_threshold functoin for the proximity sensor
 * channel(s).
 *
 * \return  Nothing.
 */
int main(void)
{
	sensor_t        prox_dev;    // proximity sensor device
	sensor_data_t   prox_data;   // proximity data
	int             led_num = 0;


	// Initialize hardware & sensor interfaces

	sensor_platform_init();

	LED_On(ALL_LEDS);              // turn all LED's on


	// Wait for user to press button to start

	prompt_user("Press button to start");       // text might not be seen

	// Attach descriptor and initialize the proximity sensor

	sensor_attach(&prox_dev, SENSOR_TYPE_PROXIMITY, 0, 0);


#if (SET_PROX_CURRENT == true)
	// Manually set LED current value for each channel
	//   Otherwise, sensor will use default values
	sensor_set_channel(&prox_dev, SENSOR_CHANNEL_ALL);	// select all channels
	sensor_set_current(&prox_dev, PROX_CURRENT_mA);
#endif

	// Set sensor data output formats (for display after calibration complete)

	prox_data.scaled = true;		// display proximity threshold status

	// Perform calibration sequence

	// Step 1
	printf("Setting channel 1: ");
	prompt_user("Place object at desired distance and press button");
	(void) sensor_calibrate(&prox_dev, MANUAL_CALIBRATE, 1, NULL);     

	// Step 2
	printf("Setting channel 2: ");
	prompt_user("Place object at desired distance and press button");
	(void) sensor_calibrate(&prox_dev, MANUAL_CALIBRATE, 2, NULL);

	// Step 3
	printf("Setting channel 3: ");
	prompt_user("Place object at desired distance and press button");
	if ( sensor_calibrate(&prox_dev, MANUAL_CALIBRATE, 3, NULL) != true) {
		if (prox_dev.err == SENSOR_ERR_IO) {
			printf ("Calibration failure: write error\n\r");
		} else {
			printf ("Unknown error while calibrating device\n\r");
		}
		return -1;		// error exit
	}

	// Display threshold values
	//
	int16_t value;
	sensor_set_channel(&prox_dev, 1);	// select channel 1
	sensor_get_threshold(&prox_dev, SENSOR_THRESHOLD_NEAR_PROXIMITY, &value);
	printf ("Channel 1 threshold = %d\r\n", value);

	sensor_set_channel(&prox_dev, 2);	// select channel 2
	sensor_get_threshold(&prox_dev, SENSOR_THRESHOLD_NEAR_PROXIMITY, &value);
	printf ("Channel 2 threshold = %d\r\n", value);

	sensor_set_channel(&prox_dev, 3);	// select channel 3
	sensor_get_threshold(&prox_dev, SENSOR_THRESHOLD_NEAR_PROXIMITY, &value);
	printf ("Channel 3 threshold = %d\r\n", value);

		
	/* Once the calibration is complete, the proximity status is continuously
	 * captured and displayed.
	 */

	while (true) {

		// Change LED display

		LED_Toggle(led_array [led_num++]);
		if (led_num >= NUM_BLINK_LEDS)
			led_num = 0;


		// Sample proximity and display results for each channel

		sensor_get_proximity(&prox_dev,  &prox_data);

		printf("prox  = 1:%s 2:%s 3:%s\r\n",
			prox_labels[prox_data.proximity.value[0]],
			prox_labels[prox_data.proximity.value[1]],
			prox_labels[prox_data.proximity.value[2]]);
		
		mdelay(LOOP_DELAY_MSEC);
	}

	return 0;
}
