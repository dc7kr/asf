/**
 * \file
 *
 * \brief Common Sensor Service Proximity Sensor Gesture Recognition Example
 *
 * \mainpage
 *
 * \section intro Introduction
 *
 * This application demonstrates the use of a multi-channel proximity sensor
 * to identify the movement direction of a physical gesture.  Typically,
 * this will be a hand or other object moving from side to side within
 * the detection range of a sensor installed on an Atmel Sensors Xplained
 * board.  The application sets up an event handler with a proximity 
 * threshold, then places the microcontroller in a low-power sleep state.  
 * When the proximity threshold is detected, the system wakes up, and
 * the channels of the proximity are repeatedly measured.  Based on the
 * sequence of the individual channels exceeding their thresholds, the
 * direction of motion of the gesture object is indicated.
 * The system then re-enters the sleep mode, and the wake/sleep cycle 
 * repeats indefinitely.
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
#include <sleepmgr.h>

#include <sensors/sensor.h>


// Configuration Constants

#define USE_PRINTF          (true)	// true = output results via serial terminal

#define PROX_SAMPLE_RATE    (100)   // proximity sensor sample rate (Hz)

#define SET_PROX_THRESHOLD  (true)	// true = manually set proximity threshold
#define	PROX_THRESHOLD		(100)	// manual prox sensor threshold for "near"

#define SET_PROX_CURRENT    (true)	// true = manually set proximity current
#define	PROX_CURRENT_mA     (150)	// current for proximity sensor LEDs (mA)

#if UC3
	// UC3 Sleep Mode to use, one of: 
	//   SLEEPMGR_ACTIVE, SLEEPMGR_IDLE, SLEEPMGR_FROZEN, SLEEPMGR_STDBY,
	//   SLEEPMGR_STOP, SLEEPMGR_DEEPSTOP, SLEEPMGR_STATIC,
	//   SLEEPMGR_SHUTDOWN (UC3L only)
#  define SLEEP_MODE     (SLEEPMGR_FROZEN)   // sleep mode to use

#elif XMEGA
	// XMega Sleep Mode to use, one of: 
	//   SLEEPMGR_ACTIVE, SLEEPMGR_IDLE, SLEEPMGR_ESTDBY, SLEEPMGR_PSAVE,
	//   SLEEPMGR_STDBY, SLEEPMGR_PDOWN
#  define SLEEP_MODE     (SLEEPMGR_IDLE)     // sleep mode to use
#endif


// LED definitions 
//	 Specify which LEDs to use to indicate gesture direction

#if (BOARD==UC3_L0_XPLAINED) || \
    (BOARD==XMEGA_B1_XPLAINED) 
#  define UP_LED     LED0
#  define DOWN_LED   LED1
#  define LEFT_LED   LED2
#  define RIGHT_LED  LED3

#elif (BOARD==UC3_A3_XPLAINED) || \
    (BOARD==XMEGA_A3BU_XPLAINED) 
#  define UP_LED     LED0
#  define DOWN_LED   LED1
#  define LEFT_LED   LED3
#  define RIGHT_LED  LED2

#elif (BOARD==XMEGA_A1_XPLAINED) 
#  define UP_LED     LED0
#  define DOWN_LED   LED3
#  define LEFT_LED   LED2
#  define RIGHT_LED  LED6

#endif

#define	ERR_BLINK_COUNT  (2)	// number of times to blink LEDs on error



// Strings to display based on gesture direction
static const char * const direction_labels[5] = {
	"Unknown",
	"Up     ",		
	"Down   ",		
	"Left   ",		
	"Right  "			
};

// Direction code values
typedef enum {
	UNK   = 0,		// unknown - unable to determine direction
	UP    = 1,		// up
	DOWN  = 2,		// down
	LEFT  = 3,		// left
	RIGHT = 4		// right
	} direction_t;

// Direction lookup table of start_channels (rows) x end_channels (columns) 
//
direction_t  dir_tbl[8][8] =	  
	//  none    1      2      1+2     3     1+3    2+3    1+2+3
	// --------------------------------------------------------
	{{  UNK,   UNK,   UNK,    UNK,   UNK,   UNK,   UNK,   UNK   },	 // none
	 {  UNK,   UNK,   RIGHT,  UNK,   DOWN,  UNK,   DOWN,  DOWN  },   // 1
	 {  UNK,   LEFT,  UNK,    UNK,   LEFT,  LEFT,  UNK,   LEFT  },   // 2
	 {  UNK,   UNK,   UNK,    UNK,   DOWN,  UNK,   DOWN,  UNK   },   // 1+2
	 {  UNK,   UP,    RIGHT,  UP,    UNK,   UNK,   RIGHT, UP    },   // 3
	 {  UNK,   UP,    RIGHT,  RIGHT, DOWN,  UNK,   RIGHT, RIGHT },   // 1+3
	 {  UNK,   UP,    RIGHT,  UP,    LEFT,  LEFT,  UNK,   UNK   },   // 2+3
	 {  UNK,   UP,    RIGHT,  UNK,   DOWN,  LEFT,  UNK,   UNK   }};  // 1+2+3


// Proximity channel bit codes - indicate which channels are above threshold
//   Used cumulatively to form index values for direction table lookup.
#define CHAN_NONE  0x00
#define CHAN_1     0x01
#define CHAN_2     0x02
#define CHAN_3     0x04


// Strings to indicate which channels detected proximity
static const char * const channel_labels[8] = {
	"none ",		// CHAN_NONE
	"1    ",		// CHAN_1
	"2    ",		// CHAN_2
	"1+2  ",		// CHAN_1 | CHAN_2
	"3    ",		// CHAN_3
	"1+3  ",		// CHAN_1 | CHAN_3
	"2+3  ",		// CHAN_2 | CHAN_3
	"1+2+3"			// CHAN_1 | CHAN_2 | CHAN_3
};


// Sensor descriptor and data storage
//
	sensor_t      prox_dev;     // proximity sensor descriptor
	sensor_data_t prox_data  = { .scaled = true };
								// sensor data - must use "scaled" prox output

	bool	 prox_event_occurred = false;		// flag to indicate event

// supply missing C-library constants

#if defined(XMEGA) && defined(__GNUC__)
	enum {EXIT_SUCCESS, EXIT_FAILURE};
#endif


/*! \brief Proximity value test 
 *
 * This routine examines the data fields in the sensor_data_t structure
 * and returns a single value summarizing which channels have a reported
 * proximity value (i.e. the value is not PROXIMITY_NONE).
 *
 * \param	data	Address of sensor_data_t struct containing prox data
 *
 * \return  int8_t	Encoded with bits set per channel over threshold
 */
static int8_t test_channels (sensor_data_t *data)	
{
	int8_t channels = CHAN_NONE;

	if (data->proximity.value[0] != PROXIMITY_NONE) {	// channel 1
		channels |= CHAN_1;
	}
	if (data->proximity.value[1] != PROXIMITY_NONE) {	// channel 2
		channels |= CHAN_2;
	}
	if (data->proximity.value[2] != PROXIMITY_NONE) {	// channel 3
		channels |= CHAN_3;
	}

	return channels;
}


/*! \brief Proximity event handler
 *
 * This routine will be called when a near-proximity event occurs.  It
 * simply copies the proximity data to the sensor_data_t structure
 * specified during the sensor_add_event function call.
 *
 * This routine executes as part of the interrupt service routine for
 * the sensor interrupt.
 *
 * \return  Nothing.
 */
static void prox_event_handler(volatile void * in) 
{
	sensor_event_desc_t * const input = (sensor_event_desc_t *) in;

	*((sensor_data_t *)(input->arg)) = input->data;      // copy sensor data

	prox_event_occurred = true;
}



/*! \brief Proximity Sensor gesture recognition demo application entry
 *
 * This application uses a 3-channel proximity sensor to recognize simple
 * gestures.  When a proximity event occurs, the routine will wake up from
 * a low-power sleep mode and begin repeatedly sampling the proximity 
 * sensor, until the proximity of the object is no longer detected.  Then
 * the beginning and ending sensor readings are compared, and the overall
 * direction of the object's movement is determined based on a lookup table.
 *
 * Once the direction is determined, it is indicate by turning on one of the
 * LEDs on the controller board and (optionally) by serial output to a
 * terminal device.  If the direction cannot be determined, all indicator
 * LEDs will be blinked rapidly.
 *
 * The application then resets by returning to a low-power sleep mode until 
 * the next proximity event is detected.
 *
 * \return  Nothing.
 */
int main(void)
{
	uint8_t	     start_channels;	// first channels detecting proximity 
	uint8_t	     current_channels;	// current channels detecting proximity 
	uint8_t	     end_channels;		// final channels detecting proximity 
	direction_t  direction;			// calculated gesture direction
	int			 i;


	// Initialize the Xplained (UC3 or XMEGA) platform & sensor boards.
	//
	sensor_platform_init();

	// Turn on LEDs while initialization completes
	//
	LED_On(UP_LED);
	LED_On(DOWN_LED);
	LED_On(LEFT_LED);
	LED_On(RIGHT_LED);

	// Initialize the MCU sleep manager API and specify a sleep mode.
	//
	sleepmgr_init();
	sleepmgr_lock_mode(SLEEP_MODE);


	// ************ Proximity Sensor Initialization ************* //
	//
	// Attach and initialize proximity sensor
	//
	sensor_attach(&prox_dev, SENSOR_TYPE_PROXIMITY, 0, 0);

	if (prox_dev.err) {
		puts("\r\nProximity sensor initialization error.");
		mdelay(5000);
		return EXIT_FAILURE;
	}
	
#if (USE_PRINTF == true)
	uint32_t     id;				// device ID
	uint8_t      version;			// device version
	sensor_device_id(&prox_dev, &id, &version);
	printf("\r\nProximity sensor: %s    ID = 0x%02x ver. 0x%02x\r\n",
			prox_dev.drv->caps.name, (unsigned) id, (unsigned) version);
#endif

	// Set sample rate
	//
	sensor_set_sample_rate(&prox_dev, PROX_SAMPLE_RATE);

	sensor_set_channel(&prox_dev, SENSOR_CHANNEL_ALL);	// select all channels

#if (SET_PROX_THRESHOLD == true)
	// Manually  set proximity threshold values for each channel
	//   Otherwise, sensor will use values previously stored in nvram.
	sensor_set_threshold(&prox_dev, SENSOR_THRESHOLD_NEAR_PROXIMITY, 
		PROX_THRESHOLD);
#endif

#if (SET_PROX_CURRENT == true)
	// Manually set LED current value for each channel
	//   Otherwise, sensor will use default values
	sensor_set_current(&prox_dev, PROX_CURRENT_mA);
#endif

	// Set up close proximity event to wakeup system
	//
	sensor_event_desc_t prox_event_desc = {
		.sensor      = &prox_dev,
		.event       = SENSOR_EVENT_NEAR_PROXIMITY,
		.data.scaled = true,
		.handler     = prox_event_handler,
		.arg         = &prox_data,
		.enabled     = false		// will be enabled in main() loop
		};

	sensor_add_event(&prox_event_desc);



	// ******************* Main Loop ************************ //
		
	while (true) {
		
		// Enable proximity event 
		sensor_enable_event(&prox_event_desc);

		// Delay before putting device to sleep
		mdelay(10);

		// Put device in low power sleep mode; wait for an interrupt to wake.
		//
		LED_Off(UP_LED);            // turn off LEDs while asleep
		LED_Off(DOWN_LED);
		LED_Off(LEFT_LED);
		LED_Off(RIGHT_LED);
		
		sleepmgr_enter_sleep();     // enter specified sleep mode


		// ** Device has woken up **
		//

		// Only do sensor processing if proximity event woke device up
		//
		if (prox_event_occurred) {

			prox_event_occurred = false;		// clear flag

			// Disable new proximity events during gesture sampling
			//
			sensor_disable_event(&prox_event_desc);
	
			// Get starting value saved by event handler routine
			//
			start_channels = test_channels(&prox_data);
			end_channels = start_channels;	// initially start and end are same
	
							
			// Loop until no longer detecting proximity
			//
			do {
				// Get new readings from sensor
				//
				sensor_get_proximity(&prox_dev, &prox_data);
	
				current_channels = test_channels(&prox_data);
	
				// Update end value if proximity is still detected
				//
				if (current_channels != CHAN_NONE) {
					end_channels = current_channels;
				}
				
			} while(current_channels != CHAN_NONE);
	
			
			// Get direction from lookup table based on start/end channel sets
			//
			direction = dir_tbl [start_channels] [end_channels];
	
#if USE_PRINTF
			// Display direction
			//
			printf("Start: %s  End: %s  Direction: %s \r\n",
				channel_labels[start_channels],
				channel_labels[end_channels],
				direction_labels[direction]);
#endif
								
			// Use LEDs to display direction 
			//
			switch  (direction) {
				case UP:
					LED_On(UP_LED);
					break;
				case DOWN:
					LED_On(DOWN_LED);
					break;
				case LEFT:
					LED_On(LEFT_LED);
					break;
				case RIGHT:
					LED_On(RIGHT_LED);
					break;
				default:			// unknown - blink all LEDs to indicate
					for (i = 0; i < (ERR_BLINK_COUNT * 2); i++) {
						LED_Toggle(UP_LED);
						LED_Toggle(DOWN_LED);
						LED_Toggle(LEFT_LED);
						LED_Toggle(RIGHT_LED);
						mdelay(40);
					}
					break;
			}

		}	// end  if (prox_event_occurred)

		mdelay(500);			// minimum active time = 500 msec
			

	}	// end of main loop


	return EXIT_SUCCESS;
}


