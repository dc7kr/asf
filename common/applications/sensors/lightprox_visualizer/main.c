/**
 * \file
 *
 * \brief Common Sensor Service Light and Proximity Sensor Example
 *
 * \mainpage
 *
 * \section intro Introduction
 *
 * This application obtains sensor data from a light and proximity sensor
 * device installed on an Atmel development board.
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
#include "data_visualizer.h"


// supply missing C-library constants

#if defined(XMEGA) && defined(__GNUC__)
enum {EXIT_SUCCESS, EXIT_FAILURE};
#endif


// Application configuration constants

#define SCALED_DATA (true)      // true = convert sensor data to std. units

#define	LIGHT_SAMPLE_RATE	(10)	// light sensor sample rate (Hz)
#define	PROX_SAMPLE_RATE	(100)	// proximity sensor sample rate (Hz)

#define SET_PROX_THRESHOLD  (true)	// true = manually set proximity threshold
#define	PROX_THRESHOLD		(100)	// manual prox sensor threshold for "near"

#define SET_PROX_CURRENT    (true)	// true = manually set proximity current
#define	PROX_CURRENT_mA		(50)	// prox sensor LED current value (mA)

// Data Visualizer definitions

#define LIGHT_STREAM_NUM		1  // Light Sensor data stream number
#define NUM_LIGHT_FIELDS		1  // Light Sensor data fields in data pkt
#define PROX_STREAM_NUM			2  // Prox Sensor data stream number
#define NUM_PROX_FIELDS			3  // Prox Sensor data fields in data pkt
#define PROX_THRESHOLD_STREAM_NUM 3 // Prox threshold data stream number
#define NUM_PROX_THRESHOLD_FIELDS 3 // Prox threshold data fields in data pkt

#define TIMESTAMP_RES   1000000 // timestamp resolution (1 per usec)
#define BLINK_DELAY     0xF000      // prompt blink delay loop count

// Switch and LED definitions
//
#if UC3
#  define SWITCH_PRESSED  (gpio_get_pin_value (GPIO_PUSH_BUTTON_0) ==     \
				           GPIO_PUSH_BUTTON_0_PRESSED)
#elif XMEGA
#  define SWITCH_PRESSED  (ioport_get_value (GPIO_PUSH_BUTTON_0) == false)
#endif

#if (BOARD==UC3_L0_XPLAINED)
#  define NUM_BLINK_LEDS  (LED_COUNT - 1)    // don't blink LED0 (shared gpio)
#  define ALL_LEDS        (LED1 | LED2 | LED3 | LED4)
#  define PROMPT_LED      LED4      // blink while waiting for user input
   static const uint32_t led_array[LED_COUNT - 1] = {LED1, LED2, LED3, LED4};

#elif (BOARD==UC3_A3_XPLAINED)
#  define NUM_BLINK_LEDS  (LED_COUNT)        // blink all LEDs
#  define ALL_LEDS        (LED0 | LED1 | LED2 | LED3)
#  define PROMPT_LED      LED0      // blink while waiting for user input
   static const uint32_t led_array[LED_COUNT] = {LED0, LED1, LED2, LED3};

#elif (BOARD==XMEGA_A1_XPLAINED) || (BOARD==XMEGA_B1_XPLAINED) || \
      (BOARD==XMEGA_A3BU_XPLAINED)
#  define NUM_BLINK_LEDS  (4)        // blink 4 LEDs
#  define ALL_LEDS        (LED0 | LED1 | LED2 | LED3)
#  define PROMPT_LED      LED0      // blink while waiting for user input
   static const uint32_t led_array[LED_COUNT] = {LED0, LED1, LED2, LED3};
#endif

#define BLINK_DELAY     0xF000      // prompt blink delay loop count

// Forward References
//

void  visual_stream_init(void);

// Hardware environment constants



static const char * const prox_labels[4] = {
	// strings to display based on proximity values
	"none  ",		// PROXIMITY_NONE
	"far   ",		// PROXIMITY_FAR
	"medium",		// PROXIMITY_MEDIUM
	"close "		// PROXIMITY_NEAR
};

/*! \brief Inertial sensor demo application entry
 *
 * After initializing the Xplained platform and sensor boards, this application
 * attaches descriptors to the accelerometer, gyroscope, and compass devices on
 * an Xplained inertial sensor board.  The sensor data, which is formatted and
 * printed via printf() after being read, can be viewed with a serial terminal
 * application on a machine attached to the USB interface on the Xplained
 * board.
 *
 * \return  Nothing.
 */
int main(void)
{
	
	volatile int    delay;
	sensor_t light_dev;     // light sensor device descriptor
	sensor_t prox_dev;   // proximity sensor device descriptor
	int8_t	 channel;	// channel within proximity sensor (1 to 3)


	/* Initialize the board (Xplained UC3 or XMEGA & Xplained Sensor boards)
	 * I/O pin mappings and any other configurable resources selected in
	 * the build configuration.
	 */
	sensor_platform_init();

	// Attach descriptors to the defined sensor devices.

	sensor_attach(&light_dev, SENSOR_TYPE_LIGHT,     0, 0);
	sensor_attach(&prox_dev,  SENSOR_TYPE_PROXIMITY, 0, 0);

	if (light_dev.err || prox_dev.err) {
		puts ("\rSensor initialization error.");
		mdelay (5000);
		return EXIT_FAILURE;
	}
	
	// Set sample rates for light and proximity sensors
	//
	if (sensor_set_sample_rate(&light_dev, LIGHT_SAMPLE_RATE) != true) {
		printf("Error setting light sensor sample rate.\r\n");
		}

	if (sensor_set_sample_rate(&prox_dev, PROX_SAMPLE_RATE) != true) {	
		printf("Error setting proximity sensor sample rate.\r\n");
		}	

	sensor_set_channel(&prox_dev, SENSOR_CHANNEL_ALL);	// select all channels

#if (SET_PROX_THRESHOLD == true)
	// Manually  set proximity threshold values for each channel
	//   Otherwise, sensor will use values previously stored in nvram.

	sensor_set_threshold(&prox_dev, SENSOR_THRESHOLD_NEAR_PROXIMITY, 
		PROX_THRESHOLD);
#endif

#if (SET_PROX_CURRENT == true)
	// Manually set LED current value for each channel
	//   Otherwise, sensor will use default values.

	sensor_set_current(&prox_dev, PROX_CURRENT_mA);
#endif



	// Initialize sensor data descriptors for scaled vs. raw data.

	static sensor_data_t light_data = {.scaled = SCALED_DATA};
	static sensor_data_t prox_data  = {.scaled = false};
	
	

// Wait for user to push button before continuing

	LED_Off(ALL_LEDS);                 // turn all LED's off

	while ( ! SWITCH_PRESSED) {

		// Just blink LED until button is pushed

		LED_Toggle(PROMPT_LED);
		
		for (delay = 0; delay < BLINK_DELAY; delay++) {
			;    // do nothing
		}
		
	}

	LED_Off(PROMPT_LED);                // leave LED off 

	while (SWITCH_PRESSED) {
		;    // wait until button is released
	}
	
	// Enable output streams for Atmel Data Visualizer (ADV)

	visual_stream_init();

	// Enter main loop

	while (true) {              // loop forever

		LED_Toggle(PROMPT_LED);

		// Read sensor values

		prox_data.scaled = false;
		sensor_get_light(&light_dev, &light_data);
		
		adv_data_send_1(LIGHT_STREAM_NUM, light_data.timestamp,
		light_data.light.value);	
		
		
		mdelay(15);
		sensor_get_proximity(&prox_dev,  &prox_data);
		
		adv_data_send_3(PROX_STREAM_NUM, light_data.timestamp,
		prox_data.proximity.value[0],
		prox_data.proximity.value[1],
		prox_data.proximity.value[2]);

				
		mdelay(15);	
		prox_data.scaled = true;
		sensor_get_proximity(&prox_dev,  &prox_data);
		
		adv_data_send_3(PROX_THRESHOLD_STREAM_NUM, light_data.timestamp,
		prox_data.proximity.value[0],
		prox_data.proximity.value[1],
		prox_data.proximity.value[2]);
		

	}

	return EXIT_SUCCESS;
}


void visual_stream_init(void)
{
	adv_config_start_t  start;      // config start packet
	adv_config_stream_t stream;     // stream config packet
	adv_config_field_t  field;      // data field info packet
	adv_config_end_t    end;        // config end packet


	// Configuration start packet

	start.header1     = ADV_PKT_HEADER_1;
	start.header2     = ADV_PKT_HEADER_2;
	start.length      = cpu_to_le16(sizeof(adv_config_start_t));
	start.type        = ADV_PKT_CONFIG_START;
	start.num_streams = 3;
	start.crc         = 0x00;          // not used
	start.mark        = ADV_PKT_END;

	adv_write_buf((uint8_t *) &start, sizeof(start));

	// Common field values for all stream config packets

	stream.header1  = ADV_PKT_HEADER_1;
	stream.header2  = ADV_PKT_HEADER_2;
	stream.type     = ADV_PKT_CONFIG_STREAM;
	stream.tick_res = cpu_to_le32(TIMESTAMP_RES);
	stream.length   = cpu_to_le16(sizeof(adv_config_stream_t));
	stream.mark     = ADV_PKT_END;
	stream.crc      = 0x00;                   // not used

	// Common field values for all data field descriptor config packets

	field.header1      = ADV_PKT_HEADER_1;
	field.header2      = ADV_PKT_HEADER_2;
	field.type         = ADV_PKT_CONFIG_FIELD;
	field.length       = cpu_to_le16(sizeof(adv_config_field_t));
	field.field_length = 4;            // all fields are signed 4-byte integers
	field.format       = FIELD_FORMAT_SIGNED;
	field.mark         = ADV_PKT_END;
	field.crc          = 0x00;                   // not used


	// Light stream
	// --------------------
	stream.stream_num = LIGHT_STREAM_NUM;
	stream.num_fields = NUM_LIGHT_FIELDS;
	strcpy((void *) &(stream.stream_name), "Ambient_Light");

	adv_write_buf((uint8_t *) &stream, sizeof(stream));

	// ALS data field
	field.stream_num = LIGHT_STREAM_NUM;
	field.field_num  = 1;
	field.min        = cpu_to_le32(0);
	field.max        = cpu_to_le32(2500);
	strcpy((void *) &(field.units), "Lx");
	strcpy((void *) &(field.name), "ALS");

	adv_write_buf((uint8_t *) &field, sizeof(field));  
	
	// Proximity stream
	// --------------------
	stream.stream_num = PROX_STREAM_NUM;
	stream.num_fields = NUM_PROX_FIELDS;
	strcpy((void *) &(stream.stream_name), "Proximity");

	adv_write_buf((uint8_t *) &stream, sizeof(stream));

	// Channel 1 data field
	field.stream_num = PROX_STREAM_NUM;
	field.field_num  = 1;
	field.min        = cpu_to_le32(50);
	field.max        = cpu_to_le32(150);
	strcpy((void *) &(field.units), "Counts");
	strcpy((void *) &(field.name), "IRLED 1");

	adv_write_buf((uint8_t *) &field, sizeof(field));  

	// Channel 2 data field (re-use previous values except name & field number)
	field.field_num = 2;
	strcpy((void *) &(field.name), "IRLED 2");

	adv_write_buf((uint8_t *) &field, sizeof(field));

	// Channel 3 data field (re-use previous values except name & field number)
	field.field_num = 3;
	strcpy((void *) &(field.name), "IRLED 3");

	adv_write_buf((uint8_t *) &field, sizeof(field));


	// Proximity Threshold stream
	// --------------------------
	stream.stream_num = PROX_THRESHOLD_STREAM_NUM;
	stream.num_fields = NUM_PROX_THRESHOLD_FIELDS;
	strcpy((void *) &(stream.stream_name), "Proximity Threshold");

	adv_write_buf((uint8_t *) &stream, sizeof(stream));

	// Channel 0 data field
	field.stream_num = PROX_THRESHOLD_STREAM_NUM;
	field.field_num  = 1;
	field.min        = cpu_to_le32(0);
	field.max        = cpu_to_le32(5);
	strcpy((void *) &(field.units), "Proximity");
	strcpy((void *) &(field.name), "IR LED 1");

	adv_write_buf((uint8_t *) &field, sizeof(field));  

	// Channel 1 data field (re-use previous values except name & field number)
	field.field_num = 2;
	strcpy((void *) &(field.name), "IR LED 2");

	adv_write_buf((uint8_t *) &field, sizeof(field));

	// Channel 2 data field (re-use previous values except name & field number)
	field.field_num = 3;
	strcpy((void *) &(field.name), "IR LED 3");

	adv_write_buf((uint8_t *) &field, sizeof(field));

	// Configuration end packet

	end.header1 = ADV_PKT_HEADER_1;
	end.header2 = ADV_PKT_HEADER_2;
	end.length  = cpu_to_le16(sizeof(adv_config_end_t));
	end.type    = ADV_PKT_CONFIG_END;
	end.num_streams = 1;
	end.crc     = 0x00;             // not used
	end.mark    = ADV_PKT_END;

	adv_write_buf((uint8_t *) &end, sizeof(end));

	return;
}
