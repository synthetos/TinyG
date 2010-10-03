/*
 * hardware.h - hardware configuration values 
 *			 	Reflects system hardware dependencies 
 *			 	Application (software) globals are in tinyg.h
 * 
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef hardware_h
#define hardware_h

void hw_init(void);						// master hardware init

/* CPU clock */	

#undef F_CPU							// set for delays
#define F_CPU 32000000UL				// should always precede <avr/delay.h>

// default hardwired settings in - chose one:
#define __LEADSCREW_20					// 1/4" 20 leadscrew table
//#define __MAKERBOT						// makerbot settings

// Clock Crystal Config. - chose one:
//#define __CLOCK_INTERNAL_32MHZ TRUE	// use internal oscillator
//#define __CLOCK_EXTERNAL_8MHZ	TRUE	// uses PLL to provide 32 MHz system clock
#define __CLOCK_EXTERNAL_16MHZ TRUE		// uses PLL to provide 32 MHz system clock

/* Stepper Ports \- motor port bits are:
 *	b7	(in) max limit switch  			// alt: (out) spindle direction on A axis
 *	b6	(in) min limit switch			// alt: (out) spindle enable on A axis
 *	b5	(out) output bit for encoder port
 *	b4	(out) microstep 1
 *	b3	(out) microstep 0 
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b0	(out) step			(SET is step,  CLR is rest)
 */

enum cfgPortBits {						// motor control port bit positions
	STEP_BIT_bp,						// bit 0
	DIRECTION_BIT_bp,					// bit 1
	MOTOR_ENABLE_BIT_bp,				// bit 2
	MICROSTEP_BIT_0_bp,					// bit 3
	MICROSTEP_BIT_1_bp,					// bit 4
	ENCODER_OUT_BIT_bp,					// bit 5 (4 encoder bits; 1 from each axis)
	MIN_LIMIT_BIT_bp,					// bit 6
	MAX_LIMIT_BIT_bp					// bit 7
};

#define STEP_BIT_bm				(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm		(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm 	(1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm		(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm		(1<<MICROSTEP_BIT_1_bp)
#define ENCODER_OUT_BIT_bm		(1<<ENCODER_OUT_BIT_bp)		
#define MIN_LIMIT_BIT_bm		(1<<MIN_LIMIT_BIT_bp)
#define MAX_LIMIT_BIT_bm		(1<<MAX_LIMIT_BIT_bp) // motor control port bit masks

/*
 * BASELINE HARDWARE CONFIGURATION VALUES (overridden by EEPROM values)
 */

/* Gcode defaults */
#define GCODE_PLANE	CANON_PLANE_XY
#define GCODE_UNITS 1					// mm
#define GCODE_HOMING_MODE 0				// unsupported
#define GCODE_TOOL 1
#define GCODE_FEED_RATE 200
#define GCODE_SPINDLE_SPEED 1500

/* non-axis machine defaults */

#define MM_PER_ARC_SEGMENT 0.02

#ifdef __LEADSCREW_20

/* Microstep defaults - chose a value on the top line */
#define MICROSTEPS 8		// Choose one of: 8, 4, 2, 1
#define MAX_STEPS 1400		// maximum steps per second for the selected motor

#define X_MICROSTEPS MICROSTEPS			// microsteps 
#define Y_MICROSTEPS MICROSTEPS			// (stepper driver configuration parameter)
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS

#define X_POLARITY 0					// motor direction polarity
#define Y_POLARITY 0
#define Z_POLARITY 0
#define A_POLARITY 1

#define X_SEEK_WHOLE_STEPS_PER_SEC MAX_STEPS // max whole steps per second for G0 motion
#define Y_SEEK_WHOLE_STEPS_PER_SEC MAX_STEPS // (motor parameter)
#define Z_SEEK_WHOLE_STEPS_PER_SEC MAX_STEPS
#define A_SEEK_WHOLE_STEPS_PER_SEC MAX_STEPS

#define X_FEED_WHOLE_STEPS_PER_SEC (MAX_STEPS * 0.90) // max whole steps per sec for feed motion
#define Y_FEED_WHOLE_STEPS_PER_SEC (MAX_STEPS * 0.90) // (motor parameter)
#define Z_FEED_WHOLE_STEPS_PER_SEC (MAX_STEPS * 0.90)
#define A_FEED_WHOLE_STEPS_PER_SEC (MAX_STEPS * 0.90)

#define X_DEGREE_PER_WHOLE_STEP	1.8		// degrees per whole step
#define Y_DEGREE_PER_WHOLE_STEP	1.8 	// (motor parameter)
#define Z_DEGREE_PER_WHOLE_STEP	1.8
#define A_DEGREE_PER_WHOLE_STEP	1.8

#define X_TRAVEL_PER_REVOLUTION 1.27	// 1/4 - 20 lead screw (0.050" per rev)
#define Y_TRAVEL_PER_REVOLUTION 1.27	// (robot parameter)
#define Z_TRAVEL_PER_REVOLUTION 1.27
#define A_TRAVEL_PER_REVOLUTION 1		// moves 1 degree per 360 deg motor revolution (hypothetical machine)

#define X_TRAVEL_MAX 400				// full excursion from min to max 
#define Y_TRAVEL_MAX 400				// (robot parameter)
#define Z_TRAVEL_MAX 100
#define A_TRAVEL_MAX -1					// -1 is no limit (typ for rotary axis)

#define X_LIMIT_ENABLE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_ENABLE TRUE				// (robot parameter)
#define Z_LIMIT_ENABLE TRUE
#define A_LIMIT_ENABLE FALSE

#define X_LOW_POWER_IDLE TRUE			// 1=low power idle enabled 
#define Y_LOW_POWER_IDLE TRUE			// (robot parameter)
#define Z_LOW_POWER_IDLE TRUE
#define A_LOW_POWER_IDLE TRUE

#endif	// _LEADSCREW_20

#ifdef __MAKERBOT

/* Microstep defaults - chose a value on the top line */
#define MICROSTEPS 8	// Choose one of: 8, 4, 2, 1

#define X_MICROSTEPS MICROSTEPS			// microsteps 
#define Y_MICROSTEPS MICROSTEPS			// (stepper driver configuration parameter)
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS

#define X_POLARITY 0					// motor direction polarity
#define Y_POLARITY 0
#define Z_POLARITY 1
#define A_POLARITY 0

#define X_SEEK_WHOLE_STEPS_PER_SEC 500	// max whole steps per second for G0 motion
#define Y_SEEK_WHOLE_STEPS_PER_SEC 500 // (motor parameter)
#define Z_SEEK_WHOLE_STEPS_PER_SEC 500
#define A_SEEK_WHOLE_STEPS_PER_SEC 500

#define X_FEED_WHOLE_STEPS_PER_SEC 500	// max whole steps per sec for feed motion
#define Y_FEED_WHOLE_STEPS_PER_SEC 500 // (motor parameter)
#define Z_FEED_WHOLE_STEPS_PER_SEC 500
#define A_FEED_WHOLE_STEPS_PER_SEC 500

#define X_DEGREE_PER_WHOLE_STEP	1.8		// degrees per whole step
#define Y_DEGREE_PER_WHOLE_STEP	1.8 	// (motor parameter)
#define Z_DEGREE_PER_WHOLE_STEP	1.8
#define A_DEGREE_PER_WHOLE_STEP	1.8

#define X_TRAVEL_PER_REVOLUTION 33			// Makerbot settings
#define Y_TRAVEL_PER_REVOLUTION 33			// (robot parameter)
#define Z_TRAVEL_PER_REVOLUTION 1.27
#define A_TRAVEL_PER_REVOLUTION 1.27

#define X_TRAVEL_MAX 100					// full excursion from min to max 
#define Y_TRAVEL_MAX 100					// (robot parameter)
#define Z_TRAVEL_MAX 150
#define A_TRAVEL_MAX -1					// -1 is no limit (typ for rotary axis)

#define X_LIMIT_ENABLE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_ENABLE TRUE				// (robot parameter)
#define Z_LIMIT_ENABLE TRUE
#define A_LIMIT_ENABLE FALSE

#define X_LOW_POWER_IDLE TRUE			// 1=low power idle enabled 
#define Y_LOW_POWER_IDLE TRUE			// (robot parameter)
#define Z_LOW_POWER_IDLE TRUE
#define A_LOW_POWER_IDLE TRUE

#endif	// __MAKERBOT


/* Derived Values */

#define X_SEEK_STEPS_PER_SEC (X_SEEK_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_SEEK_STEPS_PER_SEC (Y_SEEK_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_SEEK_STEPS_PER_SEC (Z_SEEK_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_SEEK_STEPS_PER_SEC (A_SEEK_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_FEED_STEPS_PER_SEC (X_FEED_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_FEED_STEPS_PER_SEC (Y_FEED_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_FEED_STEPS_PER_SEC (Z_FEED_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_FEED_STEPS_PER_SEC (A_FEED_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_DEGREE_PER_STEP (X_DEGREE_PER_WHOLE_STEP / X_MICROSTEPS)
#define Y_DEGREE_PER_STEP (Y_DEGREE_PER_WHOLE_STEP / Y_MICROSTEPS)
#define Z_DEGREE_PER_STEP (Z_DEGREE_PER_WHOLE_STEP / Z_MICROSTEPS)
#define A_DEGREE_PER_STEP (A_DEGREE_PER_WHOLE_STEP / A_MICROSTEPS)

#if (MICROSTEPS == 8)
#define MICROSTEP_BITS_bm (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)
#endif
#if (MICROSTEPS == 4)
#define MICROSTEP_BITS_bm (MICROSTEP_BIT_1_bm)
#endif
#if (MICROSTEPS == 2)
#define MICROSTEP_BITS_bm (MICROSTEP_BIT_0_bm)
#endif
#if (MICROSTEPS == 1)
#define MICROSTEP_BITS_bm (0)
#endif

// not needed but could be useful later
//#define MICROSTEP_EIGHTH_bm (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)
//#define MICROSTEP_QUARTER_bm (MICROSTEP_BIT_1_bm)
//#define MICROSTEP_HALF_bm (MICROSTEP_BIT_0_bm)
//#define MICROSTEP_FULL_bm (0)

#endif
