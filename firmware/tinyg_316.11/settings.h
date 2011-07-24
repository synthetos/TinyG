/*
 * settings.h - system untime settings
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/* Note: The values in this file are the default settings that are loaded
 * 		 into a virgin EEPROM, and can be changed using the config commands.
 *		 After initial load the EEPROM values (or changed values) are used.
 *
 *		 System and hardware settings that you shouldn't need to change 
 *		 are in system.h  Application settings that also shouldn't need 
 *		 to be changed are in tinyg.h
 */

#ifndef settings_h
#define settings_h

/*
 * SYSTEM SETTINGS - for initial load of EEPROM values
 */
/* general machine settings */

/*	Angular jerk thresholds set that ranges over which different path 
	control modes are in effect. Angular jerk of 0.0 is no jerk - i.e. 
 	a straight line. Max jerk is 1.0 for a 180 degree turn. A 90 degree 
	turn is 0.707... If the jerk is above the upper threshold the path
	control mode will be degraded to exact_stop mode. If between the upper 
	and lower if will be degraded to exact_path mode. If below the lower 
	threshold the path control mode will not be affected - i.e. can 
	operate in full continuous mode.
 */

//#define MAX_LINEAR_JERK 5000000	//   5,000,000 mm/(min^3)
//#define MAX_LINEAR_JERK 1000000	//  10,000,000 mm/(min^3)
//#define MAX_LINEAR_JERK 25000000	//  25,000,000 mm/(min^3)
#define MAX_LINEAR_JERK 50000000	//  50,000,000 mm/(min^3)
//#define MAX_LINEAR_JERK 100000000	// 100,000,000 mm/(min^3)

#define MAX_ROTARY_JERK 1000000		//  1,000,000 deg/(min^3)

#define CORNER_JERK_UPPER_THRESHOLD 0.60	// above which it's exact stop
#define CORNER_JERK_LOWER_THRESHOLD 0.20	// below which it's continuous
#define ENABLE_ACCEL 1						// 1 to enable
//#define ENABLE_ACCEL 0

//#define MAX_VELOCITY 1500			// ++++ THIS NEEDS TO BE A COMPUTED VALUE

/* Gcode power-on defaults */

#define GCODE_UNITS 21				// MM units
#define GCODE_PLANE	17				// XY plane
#define GCODE_PATH_CONTROL 64		// continuous mode
#define GCODE_DISTANCE_MODE 90		// absolute mode

/* default machine profiles - chose one: */

#define __ZENTOOLWORKS_7X12			// Zen Toolworks 7x12 table
//#define __PROBOTIX_V90			// Probotix Fireball V90

// NOTE: None of the profiles below have been updated 
//		 for all the various changes in the revisions
//#define __LUMENLABS_MICRO_V3		// Lumenlabs micRo v3
//#define __MAKERBOT_CUPCAKE_CNC	// Makerbot cupcake CNC
//#define __TEST_RIG_SMALL			// Test rig with small motors

/***********************************************************************/
/**** Zen Toolworks 7x12 profile ***************************************/
/***********************************************************************/

#ifdef __ZENTOOLWORKS_7X12

// motor values
#define M1_MOTOR_MAP X				// motor maps to axis
#define M2_MOTOR_MAP Y
#define M3_MOTOR_MAP Z
#define M4_MOTOR_MAP A

#define M1_STEP_ANGLE 1.8			// degrees per whole step
#define M2_STEP_ANGLE 1.8
#define M3_STEP_ANGLE 1.8
#define M4_STEP_ANGLE 1.8

#define M1_TRAVEL_PER_REV 1.25		// mm of travel = lead screw pitch
#define M2_TRAVEL_PER_REV 1.25
#define M3_TRAVEL_PER_REV 1.25
#define M4_TRAVEL_PER_REV 18		// degrees traveled per motor rev

#define M1_MICROSTEPS 8				// one of: 8, 4, 2, 1
#define M2_MICROSTEPS 8
#define M3_MICROSTEPS 8
#define M4_MICROSTEPS 8

#define M1_POLARITY 0				// 0=normal, 1=reversed
#define M2_POLARITY 1				// Y is inverted
#define M3_POLARITY 0
#define M4_POLARITY 0

#define M1_POWER_MODE TRUE			// TRUE=low power idle enabled 
#define M2_POWER_MODE TRUE
#define M3_POWER_MODE TRUE
#define M4_POWER_MODE TRUE

// axis values
#define X_AXIS_MODE AXIS_STANDARD
#define Y_AXIS_MODE AXIS_STANDARD
#define Z_AXIS_MODE AXIS_STANDARD
#define A_AXIS_MODE AXIS_RADIUS
#define B_AXIS_MODE AXIS_RADIUS
#define C_AXIS_MODE AXIS_RADIUS

#define X_SEEK_RATE_MAX 800 		// G0 max seek rate in mm/min
#define Y_SEEK_RATE_MAX 800
#define Z_SEEK_RATE_MAX 500			// Z axis won't move as fast

#define M4_STEPS_PER_SEC 2000 		// motor characteristic
#define A_SEEK_RATE_MAX ((M4_STEPS_PER_SEC * M4_STEP_ANGLE * 60) / M4_TRAVEL_PER_REV)
#define B_SEEK_RATE_MAX A_SEEK_RATE_MAX
#define C_SEEK_RATE_MAX A_SEEK_RATE_MAX

#define A_RADIUS 10					// radius in mm
#define B_RADIUS 10					// (XYZ values are not defined)
#define C_RADIUS 10

// G1 max feed rate in mm/min
#define FEED_RATE_FACTOR (0.9)
#define X_FEED_RATE_MAX (X_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Y_FEED_RATE_MAX (Y_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Z_FEED_RATE_MAX (Z_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define A_FEED_RATE_MAX (A_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define B_FEED_RATE_MAX (B_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define C_FEED_RATE_MAX (C_SEEK_RATE_MAX * FEED_RATE_FACTOR)

#define X_TRAVEL_HARD_LIMIT 400		// travel between switches or crashes
#define Y_TRAVEL_HARD_LIMIT 175
#define Z_TRAVEL_HARD_LIMIT 75
#define A_TRAVEL_HARD_LIMIT -1		// -1 is no limit (typ for rotary axis)
#define B_TRAVEL_HARD_LIMIT -1
#define C_TRAVEL_HARD_LIMIT -1

#define SOFT_LIMIT_FACTOR (0.95)
#define X_TRAVEL_SOFT_LIMIT (X_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Y_TRAVEL_SOFT_LIMIT (Y_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Z_TRAVEL_SOFT_LIMIT (Z_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define A_TRAVEL_SOFT_LIMIT -1
#define B_TRAVEL_SOFT_LIMIT -1
#define C_TRAVEL_SOFT_LIMIT -1

#define X_LIMIT_MODE TRUE			// 1=limit switches present and enabled
#define Y_LIMIT_MODE TRUE
#define Z_LIMIT_MODE TRUE
#define A_LIMIT_MODE TRUE
#define B_LIMIT_MODE TRUE
#define C_LIMIT_MODE TRUE

// homing settings
//#define HOMING_MODE TRUE			// global: set TRUE for power-on-homing
#define HOMING_MODE FALSE			// global: set TRUE for power-on-homing

#define X_HOMING_ENABLE 1			// 1=enabled for that axis
#define Y_HOMING_ENABLE 1
#define Z_HOMING_ENABLE 1
#define A_HOMING_ENABLE 1
#define B_HOMING_ENABLE 0
#define C_HOMING_ENABLE 0

#define X_HOMING_OFFSET -(X_TRAVEL_HARD_LIMIT/2) // offset to zero from axis minimum
#define Y_HOMING_OFFSET -(Y_TRAVEL_HARD_LIMIT/2)
#define Z_HOMING_OFFSET -(Z_TRAVEL_HARD_LIMIT/2)
#define A_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define B_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define C_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)

#define X_HOMING_SEEK_RATE X_FEED_RATE_MAX
#define Y_HOMING_SEEK_RATE Y_FEED_RATE_MAX
#define Z_HOMING_SEEK_RATE Z_FEED_RATE_MAX
#define A_HOMING_SEEK_RATE A_FEED_RATE_MAX
#define B_HOMING_SEEK_RATE B_FEED_RATE_MAX
#define C_HOMING_SEEK_RATE C_FEED_RATE_MAX

#define X_HOMING_CLOSE_RATE 10		// mm/min
#define Y_HOMING_CLOSE_RATE 10
#define Z_HOMING_CLOSE_RATE 10
#define A_HOMING_CLOSE_RATE 360		// degrees per minute
#define B_HOMING_CLOSE_RATE 360
#define C_HOMING_CLOSE_RATE 360

#define X_HOMING_BACKOFF 5			// mm
#define Y_HOMING_BACKOFF 5
#define Z_HOMING_BACKOFF 5
#define A_HOMING_BACKOFF 5			// degrees
#define B_HOMING_BACKOFF 5
#define C_HOMING_BACKOFF 5

#endif

/***********************************************************************/
/**** Probotix Fireball V90 profile ************************************/
/***********************************************************************/

#ifdef __PROBOTIX_V90

// motor values
#define M1_MOTOR_MAP X				// motor maps to axis
#define M2_MOTOR_MAP Y
#define M3_MOTOR_MAP Z
#define M4_MOTOR_MAP A

#define M1_STEP_ANGLE 1.8			// degrees per whole step
#define M2_STEP_ANGLE 1.8
#define M3_STEP_ANGLE 1.8
#define M4_STEP_ANGLE 1.8

#define M1_TRAVEL_PER_REV 5.08		// 5 TPI in mm
#define M2_TRAVEL_PER_REV 5.08
#define M3_TRAVEL_PER_REV 2.1166666	// 12 TPI
#define M4_TRAVEL_PER_REV 18		// degrees per motor rev - 20:1 gearing

#define M1_MICROSTEPS 8				// one of: 8, 4, 2, 1
#define M2_MICROSTEPS 8
#define M3_MICROSTEPS 8
#define M4_MICROSTEPS 8

#define M1_POLARITY 1  // invert X 	// 0=normal, 1=reversed
#define M2_POLARITY 0
#define M3_POLARITY 0
#define M4_POLARITY 0

#define M1_POWER_MODE TRUE			// TRUE=low power idle enabled 
#define M2_POWER_MODE TRUE
#define M3_POWER_MODE TRUE
#define M4_POWER_MODE TRUE

// axis values
#define X_AXIS_MODE AXIS_STANDARD
#define Y_AXIS_MODE AXIS_STANDARD
#define Z_AXIS_MODE AXIS_STANDARD
#define A_AXIS_MODE AXIS_RADIUS
#define B_AXIS_MODE AXIS_RADIUS
#define C_AXIS_MODE AXIS_RADIUS

#define X_SEEK_RATE_MAX 1200 		// G0 max seek rate in mm/min
#define Y_SEEK_RATE_MAX 1200
#define Z_SEEK_RATE_MAX 1200

#define M4_STEPS_PER_SEC 2000 		// motor characteristic
#define A_SEEK_RATE_MAX ((M4_STEPS_PER_SEC * M4_STEP_ANGLE * 60) / M4_TRAVEL_PER_REV)
#define B_SEEK_RATE_MAX A_SEEK_RATE_MAX
#define C_SEEK_RATE_MAX A_SEEK_RATE_MAX

#define A_RADIUS 10					// radius in mm
#define B_RADIUS 10					// (XYZ values are not defined)
#define C_RADIUS 10

// G1 max feed rate in mm/min
#define FEED_RATE_FACTOR (0.9)
#define X_FEED_RATE_MAX (X_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Y_FEED_RATE_MAX (Y_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Z_FEED_RATE_MAX (Z_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define A_FEED_RATE_MAX (A_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define B_FEED_RATE_MAX (B_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define C_FEED_RATE_MAX (C_SEEK_RATE_MAX * FEED_RATE_FACTOR)

#define X_TRAVEL_HARD_LIMIT 400		// travel between switches or crashes
#define Y_TRAVEL_HARD_LIMIT 300
#define Z_TRAVEL_HARD_LIMIT 75
#define A_TRAVEL_HARD_LIMIT -1		// -1 is no limit (typ for rotary axis)
#define B_TRAVEL_HARD_LIMIT -1
#define C_TRAVEL_HARD_LIMIT -1

#define SOFT_LIMIT_FACTOR (0.95)
#define X_TRAVEL_SOFT_LIMIT (X_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Y_TRAVEL_SOFT_LIMIT (Y_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Z_TRAVEL_SOFT_LIMIT (Z_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define A_TRAVEL_SOFT_LIMIT -1
#define B_TRAVEL_SOFT_LIMIT -1
#define C_TRAVEL_SOFT_LIMIT -1

#define X_LIMIT_MODE TRUE			// 1=limit switches present and enabled
#define Y_LIMIT_MODE TRUE
#define Z_LIMIT_MODE TRUE
#define A_LIMIT_MODE TRUE
#define B_LIMIT_MODE TRUE
#define C_LIMIT_MODE TRUE

// homing settings
//#define HOMING_MODE TRUE			// global: set TRUE for power-on-homing
#define HOMING_MODE FALSE			// global: set TRUE for power-on-homing

#define X_HOMING_ENABLE 1			// 1=enabled for that axis
#define Y_HOMING_ENABLE 1
#define Z_HOMING_ENABLE 1
#define A_HOMING_ENABLE 1
#define B_HOMING_ENABLE 0
#define C_HOMING_ENABLE 0

#define X_HOMING_OFFSET -(X_TRAVEL_HARD_LIMIT/2) // offset to zero from axis minimum
#define Y_HOMING_OFFSET -(Y_TRAVEL_HARD_LIMIT/2)
#define Z_HOMING_OFFSET -(Z_TRAVEL_HARD_LIMIT/2)
#define A_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define B_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define C_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)

#define X_HOMING_SEEK_RATE X_FEED_RATE_MAX
#define Y_HOMING_SEEK_RATE Y_FEED_RATE_MAX
#define Z_HOMING_SEEK_RATE Z_FEED_RATE_MAX
#define A_HOMING_SEEK_RATE A_FEED_RATE_MAX
#define B_HOMING_SEEK_RATE B_FEED_RATE_MAX
#define C_HOMING_SEEK_RATE C_FEED_RATE_MAX

#define X_HOMING_CLOSE_RATE 10		// mm/min
#define Y_HOMING_CLOSE_RATE 10
#define Z_HOMING_CLOSE_RATE 10
#define A_HOMING_CLOSE_RATE 360		// degrees per minute
#define B_HOMING_CLOSE_RATE 360
#define C_HOMING_CLOSE_RATE 360

#define X_HOMING_BACKOFF 5			// mm
#define Y_HOMING_BACKOFF 5
#define Z_HOMING_BACKOFF 5
#define A_HOMING_BACKOFF 5			// degrees
#define B_HOMING_BACKOFF 5
#define C_HOMING_BACKOFF 5

#endif

/***********************************************************************/
/**** Lumenlabs micRo v3 profile ***************************************/
/***********************************************************************/
/*
 * Note: A axis is mapped to X2
 */

#ifdef __LUMENLABS_MICRO_V3

// common defines
#define STEP_ANGLE 1.8					// 1.8 degrees is typical
#define MICROSTEPS 8				// Choose one of: 8, 4, 2, 1

// per-axis settings
#define X_SEEK_RATE 2500 				// G0 max whole steps per second
#define Y_SEEK_RATE 2000
#define Z_SEEK_RATE 2000
#define A_SEEK_RATE 2000

#define X_FEED_RATE 2000 				// G1 max whole steps per sec
#define Y_FEED_RATE 1600
#define Z_FEED_RATE 1600
#define A_FEED_RATE 2000

#define X_STEP_ANGLE STEP_ANGLE			// degrees per whole step
#define Y_STEP_ANGLE STEP_ANGLE
#define Z_STEP_ANGLE STEP_ANGLE
#define A_STEP_ANGLE STEP_ANGLE

#define X_TRAVEL_PER_REV (25.4 /10)		// 10 TPI lead screws
#define Y_TRAVEL_PER_REV (25.4 /10)
#define Z_TRAVEL_PER_REV (25.4 /10)
#define A_TRAVEL_PER_REV (25.4 /10)

#define X_TRAVEL_MAX_HARD 440				// total travel in X in mm (long axis)
#define Y_TRAVEL_MAX_HARD 300
#define Z_TRAVEL_MAX_HARD 75
#define A_TRAVEL_MAX_HARD -1					// -1 is no limit (typ for rotary axis)

#define X_TRAVEL_MAX_SOFT 				// total travel in X in mm (long axis)
#define Y_TRAVEL_MAX 300
#define Z_TRAVEL_MAX 75
#define A_TRAVEL_MAX -1					// -1 is no limit (typ for rotary axis)

#define X_CIRCUMFERENCE 0						// 0 is undefined
#define Y_CIRCUMFERENCE 0
#define Z_CIRCUMFERENCE 0
#define A_CIRCUMFERENCE RADIAN// makes mm/min = degrees/min on conversion

#define X_MICROSTEPS MICROSTEPS		// microstep mode 
#define Y_MICROSTEPS MICROSTEPS
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS

#define X_POLARITY 1					// motor direction polarity
#define Y_POLARITY 1
#define Z_POLARITY 0
#define A_POLARITY 1					// X2

#define X_POWER_MODE TRUE				// 1=low power idle enabled 
#define Y_POWER_MODE TRUE
#define Z_POWER_MODE TRUE
#define A_POWER_MODE TRUE

#define X_LIMIT_MODE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_MODE TRUE
#define Z_LIMIT_MODE TRUE
#define A_LIMIT_MODE TRUE

#define X_HOMING_OFFSET -(X_TRAVEL_MAX/2) // offset to zero from axis minimum
#define Y_HOMING_OFFSET -(Y_TRAVEL_MAX/2)
#define Z_HOMING_OFFSET -(Z_TRAVEL_MAX/2)
#define A_HOMING_OFFSET -(A_TRAVEL_MAX/2)

#define X_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define Y_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define Z_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define A_HOMING_SEEK_RATE HOMING_SEEK_RATE

#define X_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define Y_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define Z_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define A_HOMING_CLOSE_RATE HOMING_CLOSE_RATE

#define X_HOMING_BACKOFF HOMING_BACKOFF
#define Y_HOMING_BACKOFF HOMING_BACKOFF
#define Z_HOMING_BACKOFF HOMING_BACKOFF
#define A_HOMING_BACKOFF HOMING_BACKOFF

#endif

/***********************************************************************/
/**** Makerbot profile (hypothetical) **********************************/
/***********************************************************************/

#ifdef __MAKERBOT_CUPCAKE_CNC

// common defines
#define STEP_ANGLE 1.8					// 1.8 degrees is typical
#define MICROSTEPS 8					// Choose one of: 8, 4, 2, 1
#define TOP_SPEED 500					// whole steps per second
#define PULLEY_CIRCUMFERENCE 50			// mm

// per-axis values
#define X_SEEK_RATE TOP_SPEED 			// G0 max whole steps per second
#define Y_SEEK_RATE TOP_SPEED
#define Z_SEEK_RATE TOP_SPEED
#define A_SEEK_RATE TOP_SPEED

#define X_FEED_RATE TOP_SPEED 			// G1 max whole steps per sec
#define Y_FEED_RATE TOP_SPEED
#define Z_FEED_RATE TOP_SPEED
#define A_FEED_RATE TOP_SPEED

#define X_STEP_ANGLE STEP_ANGLE			// degrees per whole step
#define Y_STEP_ANGLE STEP_ANGLE
#define Z_STEP_ANGLE STEP_ANGLE
#define A_STEP_ANGLE STEP_ANGLE

#define X_TRAVEL_PER_REV 50				// Makebot is closer to 33
#define Y_TRAVEL_PER_REV 50
#define Z_TRAVEL_PER_REV 1.27			// mm per revolution (guesstimate)
#define A_TRAVEL_PER_REV 360			// degrees moved per motor revolution

#define X_TRAVEL_MAX 150				// total travel in X in mm (long axis)
#define Y_TRAVEL_MAX 150
#define Z_TRAVEL_MAX 100
#define A_TRAVEL_MAX -1					// -1 is no limit (typ for rotary axis)

#define X_CIRCUMFERENCE 0						// 0 is undefined
#define Y_CIRCUMFERENCE 0
#define Z_CIRCUMFERENCE 0
#define A_CIRCUMFERENCE RADIAN// makes mm/min = degrees/min on conversion

#define X_MICROSTEPS MICROSTEPS		// microstep mode 
#define Y_MICROSTEPS MICROSTEPS
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS

#define X_POLARITY 0					// motor direction polarity
#define Y_POLARITY 0
#define Z_POLARITY 1
#define A_POLARITY 1

#define X_POWER_MODE TRUE				// 1=low power idle enabled 
#define Y_POWER_MODE TRUE
#define Z_POWER_MODE TRUE
#define A_POWER_MODE TRUE

#define X_LIMIT_MODE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_MODE TRUE
#define Z_LIMIT_MODE TRUE
#define A_LIMIT_MODE TRUE

#define X_HOMING_OFFSET -(X_TRAVEL_MAX/2) // offset to zero from axis minimum
#define Y_HOMING_OFFSET -(Y_TRAVEL_MAX/2)
#define Z_HOMING_OFFSET -(Z_TRAVEL_MAX/2)
#define A_HOMING_OFFSET -(A_TRAVEL_MAX/2)

#define X_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define Y_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define Z_HOMING_SEEK_RATE HOMING_SEEK_RATE
#define A_HOMING_SEEK_RATE HOMING_SEEK_RATE

#define X_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define Y_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define Z_HOMING_CLOSE_RATE HOMING_CLOSE_RATE
#define A_HOMING_CLOSE_RATE HOMING_CLOSE_RATE

#define X_HOMING_BACKOFF HOMING_BACKOFF
#define Y_HOMING_BACKOFF HOMING_BACKOFF
#define Z_HOMING_BACKOFF HOMING_BACKOFF
#define A_HOMING_BACKOFF HOMING_BACKOFF

#endif

/***********************************************************************/
/**** SMALL MOTOR TEST RIG profile *************************************/
/***********************************************************************/

#ifdef __TEST_RIG_SMALL

// common defines
#define MICROSTEPS 8				// choose one of: 8, 4, 2, 1
#define STEP_ANGLE 1.8
#define STEPS_PER_SEC 2000 			// motor characteristic
#define HOMING_MODE FALSE			// set TRUE for power-on-homing

// per-axis settings
// per-axis values
#define X_AXIS_MODE AXIS_STANDARD
#define Y_AXIS_MODE AXIS_STANDARD
#define Z_AXIS_MODE AXIS_STANDARD
#define A_AXIS_MODE AXIS_STANDARD
#define B_AXIS_MODE AXIS_STANDARD
#define C_AXIS_MODE AXIS_STANDARD

//#define A_AXIS_MODE AXIS_RADIUS
//#define B_AXIS_MODE AXIS_RADIUS
//#define C_AXIS_MODE AXIS_RADIUS

#define X_SEEK_RATE_MAX 1500 		// G0 max seek rate in mm/min
#define Y_SEEK_RATE_MAX 1500
#define Z_SEEK_RATE_MAX 1500
#define A_SEEK_RATE_MAX 600
#define B_SEEK_RATE_MAX 600
#define C_SEEK_RATE_MAX 600
//#define A_SEEK_RATE_MAX ((STEPS_PER_SEC * STEP_ANGLE * 60) / A_TRAVEL_PER_REV)
//#define B_SEEK_RATE_MAX ((STEPS_PER_SEC * STEP_ANGLE * 60) / B_TRAVEL_PER_REV)
//#define C_SEEK_RATE_MAX ((STEPS_PER_SEC * STEP_ANGLE * 60) / C_TRAVEL_PER_REV)

// G1 max feed rate in mm/min
#define FEED_RATE_FACTOR (0.9)
#define X_FEED_RATE_MAX (X_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Y_FEED_RATE_MAX (Y_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define Z_FEED_RATE_MAX (Z_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define A_FEED_RATE_MAX (A_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define B_FEED_RATE_MAX (B_SEEK_RATE_MAX * FEED_RATE_FACTOR)
#define C_FEED_RATE_MAX (C_SEEK_RATE_MAX * FEED_RATE_FACTOR)

#define X_STEP_ANGLE STEP_ANGLE	 	// degrees per whole step
#define Y_STEP_ANGLE STEP_ANGLE
#define Z_STEP_ANGLE STEP_ANGLE
#define A_STEP_ANGLE STEP_ANGLE
#define B_STEP_ANGLE STEP_ANGLE
#define C_STEP_ANGLE STEP_ANGLE

#define X_TRAVEL_PER_REV 2.54		// arbitrary
#define Y_TRAVEL_PER_REV 2.54
#define Z_TRAVEL_PER_REV 2.54
#define A_TRAVEL_PER_REV 2.54			// degrees traveled per motor rev
#define B_TRAVEL_PER_REV 2.54			// typically gears down from 360/rev
#define C_TRAVEL_PER_REV 2.54

#define X_TRAVEL_HARD_LIMIT 400		// travel between switches or crashes
#define Y_TRAVEL_HARD_LIMIT 300
#define Z_TRAVEL_HARD_LIMIT 75
#define A_TRAVEL_HARD_LIMIT -1		// -1 is no limit (typ for rotary axis)
#define B_TRAVEL_HARD_LIMIT -1
#define C_TRAVEL_HARD_LIMIT -1

#define SOFT_LIMIT_FACTOR (0.95)
#define X_TRAVEL_SOFT_LIMIT (X_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Y_TRAVEL_SOFT_LIMIT (Y_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define Z_TRAVEL_SOFT_LIMIT (Z_TRAVEL_HARD_LIMIT * SOFT_LIMIT_FACTOR)
#define A_TRAVEL_SOFT_LIMIT -1
#define B_TRAVEL_SOFT_LIMIT -1
#define C_TRAVEL_SOFT_LIMIT -1

#define A_RADIUS 10.					// radius in mm
#define B_RADIUS 10					// (XYZ values are not defined)
#define C_RADIUS 10

#define X_MICROSTEPS MICROSTEPS		// motor microstep mode 
#define Y_MICROSTEPS MICROSTEPS
#define Z_MICROSTEPS MICROSTEPS
#define A_MICROSTEPS MICROSTEPS
#define B_MICROSTEPS MICROSTEPS
#define C_MICROSTEPS MICROSTEPS

#define X_POLARITY 0				// motor direction polarity
#define Y_POLARITY 0
#define Z_POLARITY 0
#define A_POLARITY 0
#define B_POLARITY 0
#define C_POLARITY 0

#define X_POWER_MODE TRUE			// motor power mode: 1=low power idle enabled 
#define Y_POWER_MODE TRUE
#define Z_POWER_MODE TRUE
#define A_POWER_MODE TRUE
#define B_POWER_MODE TRUE
#define C_POWER_MODE TRUE

#define X_LIMIT_MODE TRUE			// 1=limit switches present and enabled
#define Y_LIMIT_MODE TRUE
#define Z_LIMIT_MODE TRUE
#define A_LIMIT_MODE TRUE
#define B_LIMIT_MODE TRUE
#define C_LIMIT_MODE TRUE

#define X_HOMING_ENABLE 1			// 1=enabled for that axis
#define Y_HOMING_ENABLE 1
#define Z_HOMING_ENABLE 1
#define A_HOMING_ENABLE 1
#define B_HOMING_ENABLE 0
#define C_HOMING_ENABLE 0

#define X_HOMING_OFFSET -(X_TRAVEL_HARD_LIMIT/2) // offset to zero from axis minimum
#define Y_HOMING_OFFSET -(Y_TRAVEL_HARD_LIMIT/2)
#define Z_HOMING_OFFSET -(Z_TRAVEL_HARD_LIMIT/2)
#define A_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define B_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)
#define C_HOMING_OFFSET -(A_TRAVEL_HARD_LIMIT/2)

#define X_HOMING_SEEK_RATE X_FEED_RATE_MAX
#define Y_HOMING_SEEK_RATE Y_FEED_RATE_MAX
#define Z_HOMING_SEEK_RATE Z_FEED_RATE_MAX
#define A_HOMING_SEEK_RATE A_FEED_RATE_MAX
#define B_HOMING_SEEK_RATE B_FEED_RATE_MAX
#define C_HOMING_SEEK_RATE C_FEED_RATE_MAX

#define X_HOMING_CLOSE_RATE 10		// mm/min
#define Y_HOMING_CLOSE_RATE 10
#define Z_HOMING_CLOSE_RATE 10
#define A_HOMING_CLOSE_RATE 360		// degrees per minute
#define B_HOMING_CLOSE_RATE 360
#define C_HOMING_CLOSE_RATE 360

#define X_HOMING_BACKOFF 5			// mm
#define Y_HOMING_BACKOFF 5
#define Z_HOMING_BACKOFF 5
#define A_HOMING_BACKOFF 5			// degrees
#define B_HOMING_BACKOFF 5
#define C_HOMING_BACKOFF 5

#endif


#endif // settings_h header file
