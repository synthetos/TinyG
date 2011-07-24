/*
 * settings.h - default runtime settings
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

/**** GENERAL SETTINGS ************************************************/

/*	Angular jerk thresholds set that ranges over which different path 
	control modes are in effect. Angular jerk of 0.0 is no jerk - i.e. 
 	a straight line. Max jerk is 1.0 for a 180 degree turn. A 90 degree 
	turn is 0.707... If the jerk is above the upper threshold the path
	control mode will be degraded to exact_stop mode. If between the upper 
	and lower if will be degraded to exact_path mode. If below the lower 
	threshold the path control mode will not be affected - i.e. can 
	operate in full continuous mode.
 */

/*** System setup and operation ***/

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

/*** Gcode power-on defaults ***/

#define GCODE_UNITS 21				// MM units
#define GCODE_PLANE	17				// XY plane
#define GCODE_PATH_CONTROL 64		// continuous mode
#define GCODE_DISTANCE_MODE 90		// absolute mode

/*** Communications defaults ***/

#define COM_APPEND_TX_CR FALSE
#define COM_IGNORE_RX_CR FALSE
#define COM_IGNORE_RX_LF FALSE
#define COM_ENABLE_XON TRUE
#define COM_ENABLE_ECHO TRUE

/**** MACHINE PROFILES *************************************************/
// default machine profiles - chose only one:

#include "settings/settings_zen7x12.h"			// Zen Toolworks 7x12
//#include "settings/settings_probotixV90.h"		// Probotix FireballV90
//#include "settings/settings_lumenlabMicRoV3.h"	// Lumenlabs micRo v3
//#include "settings/settings_makerbotCupcake.h"	// makebot Cupcake CNC

#endif // settings_h header file

/* NOTES:

	What are the legal settings for each of these defines, and where are 
	the sources for these values?

	M1_MOTOR_MAP

	X_AXIS_MODE 	gcode.h

#define M1_MOTOR_MAP X				// motor maps to axis
#define M1_STEP_ANGLE 1.8			// degrees per whole step
#define M1_TRAVEL_PER_REV 2.54		// mm of travel = 10 TPI lead screw pitch
#define M1_MICROSTEPS 8				// one of: 8, 4, 2, 1
#define M1_POLARITY 1				// 0=normal, 1=reversed
#define M1_POWER_MODE TRUE			// TRUE=low power idle enabled 

// axis values
#define X_AXIS_MODE AXIS_STANDARD	// see gcode.h for valid values
#define X_SEEK_RATE_MAX 2500 		// G0 max seek rate in mm/min
#define A_RADIUS 10					// radius in mm
#define B_RADIUS 10					// (XYZ values are not defined)
#define C_RADIUS 10

#define X_FEED_RATE_MAX 
#define X_TRAVEL_HARD_LIMIT 400		// travel between switches or crashes
#define X_TRAVEL_SOFT_LIMIT 
#define X_LIMIT_MODE TRUE			// 1=limit switches present and enabled

// homing settings
//#define HOMING_MODE TRUE			// global: set TRUE for power-on-homing
#define HOMING_MODE FALSE			// global: set TRUE for power-on-homing

#define X_HOMING_ENABLE 1			// 1=enabled for that axis

#define X_HOMING_OFFSET -(X_TRAVEL_HARD_LIMIT/2) // offset to zero from axis minimum
#define X_HOMING_SEEK_RATE X_FEED_RATE_MAX
#define X_HOMING_CLOSE_RATE 10		// mm/min
#define X_HOMING_BACKOFF 5			// mm

*/
