/*
 * settings.h - system runtime settings
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


/***********************************************************************/
/**** Makerbot profile (hypothetical) **********************************/
/***********************************************************************/

//#define JERK 10000000				//  yes, that's "10,000,000" mm/(min^3)
//#define JERK 25000000
#define JERK 50000000
#define CORNER_ACCELERATION 100000
#define CORNER_DELTA 0.05			// default value, in mm

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

