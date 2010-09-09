/*
 * config.h - configuration subsystem prototypes and 
 * 			 gcode, motion_control and stepper defaults
 * 
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * ------
 * This file is somewhat different.from the original Grbl settings code
 * TinyG configurations are held in the config struct (cfg)
 *
 *	Config				example	description
 *	-------------------	-------	---------------------------------------------
 *	(non-axis configs)
 *	config_version		1.00	config version
 *	mm_arc_segment		0.01	arc drawing resolution in millimeters per segment 
 *
 *	(axis configs - one per axis - only X axis is shown)
 *	x_seek_steps_sec	1800	max seek whole steps per second for X axis
 *	x_feed_steps_sec	1200	max feed whole steps per second for X axis
 *	x_degree_per_step	1.8		degrees per whole step for X axis
 *	x_mm_per_rev		2.54	millimeters of travel per revolution of X axis
 *	x_mm_travel			406		millimeters of travel in X dimension (total)
 * 	x_microstep			8		microsteps to apply for X axis steps
 *	x_low_pwr_idle		1		1=low power idle mode, 0=full power idle mode 
 *	x_limit_enable		1		1=max limit switch enabled, 0=not enabled
 */

#ifndef config_h
#define config_h

#include "tinyg.h"

/*
 * Global Scope Functions
 */

void cfg_init(void);		// initialize config struct by reading from EEPROM
void cfg_reset(void);		// reset config values to defaults
int cfg_parse(char *text);	// parse a tag=value config string
int cfg_read(void);			// read config record from EEPROM
void cfg_write(void);		// write config record to EEPROM
void cfg_dump(void);
void cfg_test(void);		// unit tests for config routines

/*
 * Global scope config structs
 */

struct cfgStructAxis {
 // motor configuration
  	uint8_t microstep;			// microsteps to apply for each axis (ex: 8)
 	uint8_t low_pwr_idle;		// 1=low power idle mode, 0=full power idle mode
	uint8_t polarity;			// 0=normal polarity, 1=reverse motor direction
	uint16_t seek_steps_sec;	// max seek whole steps per second (ex: 1600)
	uint16_t feed_steps_sec;	// max feed whole steps per second (ex: 1200)
	double degree_per_step;		// degrees per whole step (ex: 1.8)
 // machine configuration
	double mm_per_rev;			// millimeters of travel per revolution (ex: 2.54)
	double mm_travel;			// millimeters of travel max in N dimension (ex: 400)
	double steps_per_mm;		// # steps (actually usteps)/mm of travel (COMPUTED)
	uint8_t limit_enable;		// 1=limit switches enabled, 0=not enabled
};

struct cfgStructGlobal {
	uint8_t config_version;		// config format version. starts at 100
	uint8_t status;				// interpreter status
 // model configuration
	double mm_per_arc_segment;	// arc drawing resolution in millimeters per segment
	double default_feed_rate;	// mm of trav in mm/s (was mm/min in Grbl)(COMPUTED)
	double default_seek_rate;	// mm of trav in mm/s (was mm/min in Grbl)(COMPUTED)
 // axis structs
	struct cfgStructAxis a[4];	// holds axes X,Y,Z,A
};

struct cfgStructGlobal cfg; 	// declared in the header to make it global
#define CFG(x) cfg.a[x]			// handy macro for referencing the axis values, 
								// e.g: CFG(X_AXIS).steps_per_mm

/* 
 * BASE CONFIGURATION VALUES 
 */

// Constants from Grbl
//#define MM_PER_ARC_SEGMENT 0.1
#define MM_PER_ARC_SEGMENT 0.01
#define ONE_MINUTE_OF_MICROSECONDS 60000000.0
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define INCHES_PER_MM (1.0/25.4)

/* The stepper ISRs generate step pulses approximately 1.5 microseconds duration
 * The TI DRV8811's used on the TinyG board are fine with this pulse width. 
 * Some outboarded drivers might not be. If the drivers require a longer pulse 
 * uncomment __STEPPER_DELAY and adjust the microseconds to your requirements.
 * The delay is in addition to the 1.5 uSec burned in the ISR.
 */
//#define __STEPPER_DELAY
#ifdef __STEPPER_DELAY
#define STEP_PULSE_ADDITIONAL_MICROSECONDS	2
#define STEPPER_DELAY _delay_us(STEP_PULSE_ADDITIONAL_MICROSECONDS);
#else
#define STEPPER_DELAY				// used as a no-op in the ISR
#endif


/*	
 * Version of the EEPROM data. 
 * Used to migrate existing data from older versions during firmware upgrades
 * Stored in EEPROM byte 0
 */
#define CONFIG_VERSION 100		// TinyG value

/*
 * SYSTEM SETTINGS AND CONSTANTS 
 */ 
enum cfgAxisNum {				// define axis numbers and array indexes from 0 to 3
		X_AXIS,
		Y_AXIS,
		Z_AXIS,
		A_AXIS
};

#ifdef __RILEY					// support for Rileys's blown X axis
#define X_MOTOR_PORT PORTD		// labeled as motor #1
#define Y_MOTOR_PORT PORTF		//					#2
#define Z_MOTOR_PORT PORTE		//					#3
#define A_MOTOR_PORT PORTA		//					#4
#else
#define X_MOTOR_PORT PORTA		// labeled as motor #1
#define Y_MOTOR_PORT PORTF		//					#2
#define Z_MOTOR_PORT PORTE		//					#3
#define A_MOTOR_PORT PORTD		//					#4
#endif

#define MOTOR_PORT_DIR_gm		0x3F		// direction register settings
#define X_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define Y_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define Z_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define A_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm	
								// Note: spindle out bits are on PORT_A - b7 & b6

#define X_TIMER					TCC0		// x-axis timer
#define Y_TIMER					TCD0
#define Z_TIMER					TCE0
#define A_TIMER					TCF0

#define X_TIMER_vect 			TCC0_OVF_vect // x-axis step rate timer vector
#define Y_TIMER_vect		 	TCD0_OVF_vect
#define Z_TIMER_vect		 	TCE0_OVF_vect
#define A_TIMER_vect 			TCF0_OVF_vect

#define X_ACTIVE_BIT_bm			(1<<3)		// used in Axes to detect move complete
#define Y_ACTIVE_BIT_bm			(1<<2)
#define Z_ACTIVE_BIT_bm			(1<<1)
#define A_ACTIVE_BIT_bm			(1<<0)

/* Port bit configs - motor port bits are:
 *	b7	(in) max limit switch  	// alt: (out) spindle direction on A axis
 *	b6	(in) min limit switch	// alt: (out) spindle enable on A axis
 *	b5	(out) output bit for encoder port
 *	b4	(out) microstep 1
 *	b3	(out) microstep 0 
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b0	(out) step			(SET is step,  CLR is rest)
 */

enum cfgPortBits {				// motor control port bit positions - hardwired
	STEP_BIT_bp,				// bit 0
	DIRECTION_BIT_bp,			// bit 1
	MOTOR_ENABLE_BIT_bp,		// bit 2
	MICROSTEP_BIT_0_bp,			// bit 3
	MICROSTEP_BIT_1_bp,			// bit 4
	ENCODER_OUT_BIT_bp,			// bit 5 (4 encoder bits total, one from each axis)
	MIN_LIMIT_BIT_bp,			// bit 6
	MAX_LIMIT_BIT_bp			// bit 7
};

#define STEP_BIT_bm				(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm		(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm 	(1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm		(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm		(1<<MICROSTEP_BIT_1_bp)
#define ENCODER_OUT_BIT_bm		(1<<ENCODER_OUT_BIT_bp)		
#define MIN_LIMIT_BIT_bm		(1<<MIN_LIMIT_BIT_bp)
#define MAX_LIMIT_BIT_bm		(1<<MAX_LIMIT_BIT_bp) // motor control port bit masks

#define	LIMIT_BIT_SETUP_gc		PORT_OPC_PULLUP_gc	  // totem poll pullup mode

#define MICROSTEP_FULL_bm (0)
#define MICROSTEP_HALF_bm (MICROSTEP_BIT_0_bm)
#define MICROSTEP_QUARTER_bm (MICROSTEP_BIT_1_bm)
#define MICROSTEP_EIGHTH_bm (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)

#define MICROSTEPS 8	// FOR NOW THESE VALUES MUST BE SYNCD - e.g. 8 means eighths
#define MICROSTEP_UNITS_bm MICROSTEP_EIGHTH_bm
//#define MICROSTEP_UNITS_bm MICROSTEP_QUARTER_bm
//#define MICROSTEP_UNITS_bm MICROSTEP_HALF_bm
//#define MICROSTEP_UNITS_bm MICROSTEP_FULL_bm

/* timer constants */

#define TC_WGMODE		0			// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3			// assign timer interrupt level (3=hi)
#define TC_CLK_OFF 		0			// turn timer off (clock = 0 Hz)
#define TC_CLK_ON		1			// turn timer clock on (32 Mhz)

/* spindle config and constants - bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	A_MOTOR_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	A_MOTOR_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction


/*
 * CONFIGURATION DEFAULT VALUES (used when resetting eeprom-settings)
 */

#define X_MICROSTEPS MICROSTEPS			// microsteps 
#define Y_MICROSTEPS MICROSTEPS			// (stepper driver configuration parameter)
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS

#define X_POLARITY 0					// motor direction polarity
#define Y_POLARITY 1
#define Z_POLARITY 0
#define A_POLARITY 0

#define X_SEEK_WHOLE_STEPS_PER_SEC 1500	// max whole steps per second for G0 motion
#define Y_SEEK_WHOLE_STEPS_PER_SEC 1500 // (motor parameter)
#define Z_SEEK_WHOLE_STEPS_PER_SEC 1500
#define A_SEEK_WHOLE_STEPS_PER_SEC 1500

#define X_SEEK_STEPS_PER_SEC (X_SEEK_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_SEEK_STEPS_PER_SEC (Y_SEEK_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_SEEK_STEPS_PER_SEC (Z_SEEK_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_SEEK_STEPS_PER_SEC (A_SEEK_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_FEED_WHOLE_STEPS_PER_SEC 1500	// max whole steps per sec for feed motion
#define Y_FEED_WHOLE_STEPS_PER_SEC 1500 // (motor parameter)
#define Z_FEED_WHOLE_STEPS_PER_SEC 1500
#define A_FEED_WHOLE_STEPS_PER_SEC 1500

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

/*
#define X_MM_PER_REVOLUTION 2.54		// typically 0.100" per revolution
#define Y_MM_PER_REVOLUTION 2.54		// (machine parameter)
#define Z_MM_PER_REVOLUTION 2.54
#define A_MM_PER_REVOLUTION 2.54
*/

#define X_MM_PER_REVOLUTION 1.27		// 1/4 - 20 lead screw (0.050" per rev)
#define Y_MM_PER_REVOLUTION 1.27		// (machine parameter)
#define Z_MM_PER_REVOLUTION 1.27
#define A_MM_PER_REVOLUTION 1.27

#define X_MM_TRAVEL 400					// full excursion from min to max 
#define Y_MM_TRAVEL 400					// (machine parameter)
#define Z_MM_TRAVEL 300
#define A_MM_TRAVEL -1					// -1 is no limit (typ for rotary axis)

#define X_LIMIT_ENABLE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_ENABLE TRUE				// (machine parameter)
#define Z_LIMIT_ENABLE TRUE
#define A_LIMIT_ENABLE FALSE

#define X_LOW_POWER_IDLE TRUE			// 1=low power idle enabled 
#define Y_LOW_POWER_IDLE TRUE			// (machine parameter)
#define Z_LOW_POWER_IDLE TRUE
#define A_LOW_POWER_IDLE TRUE

#endif
