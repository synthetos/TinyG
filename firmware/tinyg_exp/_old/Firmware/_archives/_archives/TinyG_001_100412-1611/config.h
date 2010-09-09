/*
  config.h - eeprom and compile time configuration handling 
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  TinyG Notes
  Modified to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.
  
	- This file has been largely re-written.
	- All chip-sepecific registers have been changed
	- The step/dir bits are not bound to the same port any more
	- Added port definitions for min/max switches and encoder port
	- Missing function prototypes added

  Key: "configs" are things that can change, "constants" probably should not

*/

#ifndef config_h
#define config_h

#define TINYG_VERSION "0.01"	// make this the TinyG version (not Grbl version)


/* function prototypes */
void config_init(void);		// init configuration subsystem (load settings from EEPROM)
void reset_settings(void);	// reset the struct only (no eeprom write)
void dump_settings(void);	// print current settings
int read_settings(void);	// read settings from eeprom
void write_settings(void);	// write settings to eeprom
void store_setting(int parameter, double value);  // helper method to set new settings from command line


/* Base Constants */

#define CLK_MHZ = (F_CPU/1000000)	// clock in Mhz

#define MM_PER_ARC_SEGMENT 0.1
#define INCHES_PER_MM (1.0/25.4) 	// A conversion rate


/* Serial Configuration Settings
	Values for common baud rates at 32 Mhz clock
	BSEL	BSCALE 
	207		0			// 9600b
	103		0			// 19200b
	 34		0			// 57600b
	 33 	(-1<<4) 	// 115.2kb
	 31		(-2<<4)		// 230.4kb
	 27		(-3<<4) 	// 460.8kb
	 19		(-4<<4)		// 921.6kb
	  1 	(1<<4)		// 500kb
	  1		0			// 1Mb
*/

// Invoke terminal screen: screen /dev/tty.usbserial-A700eUQop 115200

#define USB_BAUD_RATE 115200	// these 3 must be consistent
#define USB_BSEL 33				// 115200 BAUD
#define USB_BSCALE (-1<<4) 


#define RS485_BAUD_RATE 115200	// these 3 must be consistent
#define RS485_BSEL 33
#define RS485USB_BSCALE (-1<<4)


/* Motor and Robot Settings */ 
// Default settings (used when resetting eeprom-settings)

#define X_MICROSTEPS 8					// microsteps 
#define Y_MICROSTEPS 8					// (stepper driver configuration parameter)
#define Z_MICROSTEPS 8
#define A_MICROSTEPS 8

#define X_SEEK_WHOLE_STEPS_PER_SEC 1600	// max whole steps per second for G0 motion
#define Y_SEEK_WHOLE_STEPS_PER_SEC 1600 // (motor parameter)
#define Z_SEEK_WHOLE_STEPS_PER_SEC 1600
#define A_SEEK_WHOLE_STEPS_PER_SEC 1600

#define X_SEEK_STEPS_PER_SEC (X_SEEK_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_SEEK_STEPS_PER_SEC (Y_SEEK_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_SEEK_STEPS_PER_SEC (Z_SEEK_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_SEEK_STEPS_PER_SEC (A_SEEK_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_FEED_WHOLE_STEPS_PER_SEC 1000	// max whole steps per sec for feed motion
#define Y_FEED_WHOLE_STEPS_PER_SEC 1000 // (motor parameter)
#define Z_FEED_WHOLE_STEPS_PER_SEC 1000
#define A_FEED_WHOLE_STEPS_PER_SEC 1000

#define X_FEED_STEPS_PER_SEC (X_FEED_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_FEED_STEPS_PER_SEC (Y_FEED_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_FEED_STEPS_PER_SEC (Z_FEED_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_FEED_STEPS_PER_SEC (A_FEED_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_DEGREE_PER_WHOLE_STEP	1.8		// degrees per whole step
#define Y_DEGREE_PER_WHOLE_STEP	1.8 	// (motor parameter)
#define Z_DEGREE_PER_WHOLE_STEP	1.8
#define A_DEGREE_PER_WHOLE_STEP	1.8

#define X_DEGREE_PER_STEP (X_DEGREE_PER_WHOLE_STEP / X_MICROSTEPS)
#define Y_DEGREE_PER_STEP (Y_DEGREE_PER_WHOLE_STEP / Y_MICROSTEPS)
#define Z_DEGREE_PER_STEP (Z_DEGREE_PER_WHOLE_STEP / Z_MICROSTEPS)
#define A_DEGREE_PER_STEP (A_DEGREE_PER_WHOLE_STEP / A_MICROSTEPS)

#define X_MM_PER_REVOLUTION 2.54		// typically 0.100" per revolution
#define Y_MM_PER_REVOLUTION 2.54		// (machine parameter)
#define Z_MM_PER_REVOLUTION 2.54
#define A_MM_PER_REVOLUTION 2.54

#define X_STEPS_PER_MM ((360 / X_DEGREE_PER_STEP) / X_MM_PER_REVOLUTION)
#define Y_STEPS_PER_MM ((360 / Y_DEGREE_PER_STEP) / Y_MM_PER_REVOLUTION)
#define Z_STEPS_PER_MM ((360 / Z_DEGREE_PER_STEP) / Z_MM_PER_REVOLUTION)
#define A_STEPS_PER_MM ((360 / A_DEGREE_PER_STEP) / A_MM_PER_REVOLUTION)

// in millimeters per minute...
#define RAPID_FEEDRATE ((X_SEEK_STEPS_PER_SEC / (360/X_DEGREE_PER_STEP)) * 60)
#define DEFAULT_FEEDRATE ((X_FEED_STEPS_PER_SEC / (360/X_DEGREE_PER_STEP)) * 60)


/* Port configs - motor port bits are:
	b7	(in) max limit switch  	// alt: (out) spindle direction on A axis
	b6	(in) min limit switch	// alt: (out) spindle enable on A axis
	b5	(out) output bit for encoder port
	b4	(out) microstep 1
	b3	(out) microstep 0 
	b2	(out) motor enable
	b1	(out) direction
	b0	(out) step
*/
#define X_MOTOR_PORT			PORTA		// labeled as motor #1
#define Y_MOTOR_PORT			PORTF		//					#2
#define Z_MOTOR_PORT			PORTE		//					#3
#define A_MOTOR_PORT			PORTD		//					#4

#define X_MOTOR_PORT_DIR_gm		0x3F		// direction register settings
#define Y_MOTOR_PORT_DIR_gm		0x3F
#define Z_MOTOR_PORT_DIR_gm		0x3F
#define A_MOTOR_PORT_DIR_gm		0x3F		// spindle out bits are also on b7 & b6

#define STEP_PULSE_MICROSECONDS	2			// step pulse width microseconds (delay)

/* port constants */

#define MAX_LIMIT_BIT_bp		7			// motor control port bit positions
#define MIN_LIMIT_BIT_bp		6
#define ENCODER_OUT_BIT_bp		5			// 4 encoder out bits total, one from each axis
#define MICROSTEP_BIT_1_bp		4
#define MICROSTEP_BIT_0_bp		3
#define MOTOR_ENABLE_BIT_bp 	2
#define DIRECTION_BIT_bp		1
#define STEP_BIT_bp				0

#define MAX_LIMIT_BIT_bm		(1<<MAX_LIMIT_BIT_bp)		// motor control port bit masks
#define MIN_LIMIT_BIT_bm		(1<<MIN_LIMIT_BIT_bp)
#define ENCODER_OUT_BIT_bm		(1<<ENCODER_OUT_BIT_bp)		
#define MICROSTEP_BIT_1_bm		(1<<MICROSTEP_BIT_1_bp)
#define MICROSTEP_BIT_0_bm		(1<<MICROSTEP_BIT_0_bp)
#define MOTOR_ENABLE_BIT_bm 	(1<<MOTOR_ENABLE_BIT_bp)
#define DIRECTION_BIT_bm		(1<<DIRECTION_BIT_bp)
#define STEP_BIT_bm				(1<<STEP_BIT_bp)

#define MICROSTEP_FULL_bm (~MICROSTEP_BIT_1_bm | ~MICROSTEP_BIT_0_bm)
#define MICROSTEP_HALF_bm (~MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)
#define MICROSTEP_QUARTER_bm (MICROSTEP_BIT_1_bm | ~MICROSTEP_BIT_0_bm)
#define MICROSTEP_EIGHTH_bm (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)

/* bit positions and masks used by line buffer and some other routines */

#define X_BIT_bp				0			// general purpose bit position for x axis
#define Y_BIT_bp				1
#define Z_BIT_bp				2
#define A_BIT_bp				3
#define X_DIRECTION_BIT_bp		4			// secondary bit position for x axis
#define Y_DIRECTION_BIT_bp		5
#define Z_DIRECTION_BIT_bp		6
#define A_DIRECTION_BIT_bp		7

#define X_BIT_bm				(1<<X_BIT_bp)// general purpose bit mask for x axis
#define Y_BIT_bm				(1<<Y_BIT_bp)
#define Z_BIT_bm				(1<<Z_BIT_bp)
#define A_BIT_bm				(1<<A_BIT_bp)
#define X_DIRECTION_BIT_bm		(1<<X_DIRECTION_BIT_bp)	// secondary bit mask for x axis
#define Y_DIRECTION_BIT_bm		(1<<Y_DIRECTION_BIT_bp)
#define Z_DIRECTION_BIT_bm		(1<<Z_DIRECTION_BIT_bp)
#define A_DIRECTION_BIT_bm		(1<<A_DIRECTION_BIT_bp)

/* spindle config and constants - bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	A_MOTOR_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	A_MOTOR_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction

/* timer configs */

#define X_TIMER			TCC0				// x-axis timer
#define Y_TIMER			TCD0
#define Z_TIMER			TCE0
#define A_TIMER			TCF0

#define X_TIMER_vect 	TCC0_OVF_vect		// x-axis step rate timer vector
#define Y_TIMER_vect 	TCD0_OVF_vect
#define Z_TIMER_vect 	TCE0_OVF_vect
#define A_TIMER_vect 	TCF0_OVF_vect

/* timer and rate constants */

#define TC_WGMODE		0					// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3					// high level interrupt

#define TC_CLK_OFF 		0					// turn timer off (clock = 0 Hz)
#define TC_CLK_DIV_1 	1					// timer freq =  32 Mhz
#define TC_CLK_DIV_2 	2					// timer freq =  16 Mhz (32Mhz / 2)
#define TC_CLK_DIV_4 	3					// timer freq =   8 Mhz (32Mhz / 4)
#define TC_CLK_DIV_8 	4					// timer freq =   4 Mhz (32Mhz / 8)
#define TC_CLK_DIV_64 	5					// timer freq = 500 Khz (32Mhz / 64)
#define TC_CLK_DIV_256 	6					// timer freq = 125 Khz (32Mhz / 256)

											// times below reflect 32 MHz CPU clock:
//#define DIV1_RANGE 	(2^16/(F_CPU/1000000))	// up to   2.048 mSec / pulse (~500+ pps)
//#define DIV2_RANGE 	(2^16/(F_CPU/2000000))	// up to   4.096 mSec / pulse (~250-500 pps)
//#define DIV4_RANGE 	(2^16/(F_CPU/4000000))	// up to   8.192 mSec / pulse (~125-250 pps)
//#define DIV8_RANGE 	(2^16/(F_CPU/8000000))	// up to  16.384 mSec / pulse (~ 60-125 pps)
//#define DIV64_RANGE (2^16/(F_CPU/6400000))	// up to 131.072 mSec / pulse (~  8-60
//#define DIV256_RANGE (2^16/(F_CPU/2560000))	// up to 524.288 mSec / pulse 
#define DIV1_RANGE 	(2048)
#define DIV2_RANGE 	(4096)
#define DIV4_RANGE 	(8192)
#define DIV8_RANGE 	(16384)
#define DIV64_RANGE (131072)
#define DIV256_RANGE (544288)

/* Version of the EEPROM data. 
	Used to migrate existing data from older versions during Grbl firmware upgrade
	Always stored in byte 0 of eeprom
	GrblX changed version from 1 to 100. Use 2 LS digits for minor rel#
*/
#define SETTINGS_VERSION 100 

// Current global settings (persisted in EEPROM from byte 1 onwards)
struct Settings {
	double steps_per_mm[3];
	uint8_t microsteps;
	uint8_t pulse_microseconds;
	double default_feed_rate;
	double default_seek_rate;
	uint8_t invert_mask;
	double mm_per_arc_segment;
};
struct Settings settings;


/* Gcode amd Motion Control constants */



// Use this line for default operation (step-pulses high)
#define STEPPING_INVERT_MASK 0
// Uncomment this line for inverted stepping (step-pulses low, rest high)
// #define STEPPING_INVERT_MASK (STEP_MASK)
// Uncomment this line to invert all step- and direction bits
// #define STEPPING_INVERT_MASK (STEPPING_MASK)
// Or bake your own like this adding any step-bits or directions you want to invert:
// #define STEPPING_INVERT_MASK (STEP_MASK | (1<<X_DIRECTION_BIT) | (1<<Y_DIRECTION_BIT))

// Some useful constants - from Grbl (commented out)
//#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
//#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
//#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)
//#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits



#endif
