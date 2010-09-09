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

NOTES:
--- This file has been largely re-written.
	All chip-sepecific registers have been changed
	The step/dir bits are not bound to the same port any more
	Added port definitions for min/max switches and encoder port
	Mising function prototypes added

*/

#ifndef config_h
#define config_h

/* function prototypes */
void config_init(void);		// init configuration subsystem (load settings from EEPROM)
void reset_settings(void);	// reset the struct only (no eeprom write)
void dump_settings(void);	// print current settings
int read_settings(void);	// read settings from eeprom
void write_settings(void);	// write settings to eeprom
void store_setting(int parameter, double value);  // helper method to set new settings from command line


#define GRBLX_VERSION "0.01"	// make this the GrblX version (not Grbl version)

/* Settings that can only be set at compile-time:*/

/* Serial configuration 
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

#define BAUD_RATE 57600	// these 3 must be consistent
#define USB_BSEL 34
#define USB_BSCALE 0

/* 
  Port configs - motor port bits are:
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
#define A_MOTOR_PORT_DIR_gm		0x3F		// spindle out bits are also on b7 and b6

#define MAX_LIMIT_BIT_bm		(1<<7)		// motor control port bit masks
#define MIN_LIMIT_BIT_bm		(1<<6)
#define ENCODER_OUT_BIT_bm		(1<<5)		// 4 out bits total, one from each axis
#define MICROSTEP_BIT_1_bm		(1<<4)
#define MICROSTEP_BIT_0_bm		(1<<3)
#define MOTOR_ENABLE_BIT_bm 	(1<<2)
#define DIRECTION_BIT_bm		(1<<1)
#define STEP_BIT_bm				(1<<0)

#define MAX_LIMIT_BIT_bp		7			// motor control port bit positions
#define MIN_LIMIT_BIT_bp		6
#define ENCODER_OUT_BIT_bp		5
#define MICROSTEP_BIT_1_bp		4
#define MICROSTEP_BIT_0_bp		3
#define MOTOR_ENABLE_BIT_bp 	2
#define DIRECTION_BIT_bp		1
#define STEP_BIT_bp				0

#define ENCODER_IN_3_bm			(1<<3)		// encoder input bit masks
#define ENCODER_IN_2_bm			(1<<2)
#define ENCODER_IN_1_bm			(1<<1)
#define ENCODER_IN_0_bm			(1<<0)

#define ENCODER_IN_3_bp			3			// encoder input bit positions
#define ENCODER_IN_2_bp			2
#define ENCODER_IN_1_bp			1
#define ENCODER_IN_0_bp			0

/* spindle bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	A_MOTOR_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	A_MOTOR_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction

/* timer configs */

#define TC_CLK_DIV_8 	4					// timer freq = 4 Mhz 	(32Mhz / 8)
#define TC_CLK_DIV_64 	5					// timer freq = 500 Khz (32Mhz / 64)
#define TC_CLK_DIV_256 	6					// timer freq = 125 Khz (32Mhz / 256)

#define TC_WGMODE		0					// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3					// high level interrupt

#define X_TIMER			TCC0				// x-axis timer
#define Y_TIMER			TCC1
#define Z_TIMER			TCD0
#define A_TIMER			TCD1

#define X_TIMER_vect 	TCC0_OVF_vect		// x-axis step rate timer vector
#define Y_TIMER_vect 	TCC1_OVF_vect
#define Z_TIMER_vect 	TCD0_OVF_vect
#define A_TIMER_vect 	TCD1_OVF_vect


/* REMOVE THIS WHEN THE TIME COMES
#define STEPPERS_ENABLE_DDR     DDRD
#define STEPPERS_ENABLE_PORT    PORTD
#define STEPPERS_ENABLE_BIT         2

#define STEPPING_DDR       DDRC
#define STEPPING_PORT      PORTC 
#define X_STEP_BIT           0
#define Y_STEP_BIT           1
#define Z_STEP_BIT           2
#define X_DIRECTION_BIT            3
#define Y_DIRECTION_BIT            4
#define Z_DIRECTION_BIT            5

#define LIMIT_DDR      DDRD
#define LIMIT_PORT     PORTD
#define X_LIMIT_BIT          3
#define Y_LIMIT_BIT          4
#define Z_LIMIT_BIT          5

#define SPINDLE_ENABLE_DDR DDRD
#define SPINDLE_ENABLE_PORT PORTD
#define SPINDLE_ENABLE_BIT 6

#define SPINDLE_DIRECTION_DDR DDRD
#define SPINDLE_DIRECTION_PORT PORTD
#define SPINDLE_DIRECTION_BIT 7
*/


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


// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 8
#define X_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define Y_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define STEP_PULSE_MICROSECONDS 30

#define MM_PER_ARC_SEGMENT 0.1

#define RAPID_FEEDRATE 480.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 480.0

// Use this line for default operation (step-pulses high)
#define STEPPING_INVERT_MASK 0
// Uncomment this line for inverted stepping (step-pulses low, rest high)
// #define STEPPING_INVERT_MASK (STEP_MASK)
// Uncomment this line to invert all step- and direction bits
// #define STEPPING_INVERT_MASK (STEPPING_MASK)
// Or bake your own like this adding any step-bits or directions you want to invert:
// #define STEPPING_INVERT_MASK (STEP_MASK | (1<<X_DIRECTION_BIT) | (1<<Y_DIRECTION_BIT))


// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#define INCHES_PER_MM (1.0/25.4) // A conversion rate

#endif
