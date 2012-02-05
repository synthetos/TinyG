/*
 * settings_zen7x12.h - Zen Toolworks 7x12 machine profile
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

/***********************************************************************/
/**** Zen Toolworks 7x12 profile ***************************************/
/***********************************************************************/

#define JERK_MAX_COMMON 	100000000	// yes, that's "50,000,000" mm/(min^3)
#define CORNER_ACCELERATION 200000
#define JUNCTION_DEVIATION	0.05		// default value, in mm

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
#define X_AXIS_MODE AXIS_STANDARD	// see gcode.h for valid values
#define Y_AXIS_MODE AXIS_STANDARD
#define Z_AXIS_MODE AXIS_STANDARD
//#define Z_AXIS_MODE AXIS_INHIBITED	// Z kill
#define A_AXIS_MODE AXIS_RADIUS
#define B_AXIS_MODE AXIS_RADIUS
#define C_AXIS_MODE AXIS_RADIUS

#define X_VELOCITY_MAX 1000 			// G0 max velocity in mm/min
#define Y_VELOCITY_MAX 1000
//#define X_VELOCITY_MAX 600 			// G0 max velocity in mm/min
//#define Y_VELOCITY_MAX 600
#define Z_VELOCITY_MAX 500			// Z axis won't move as fast
#define M4_STEPS_PER_SEC 2000 		// motor characteristic
#define A_VELOCITY_MAX ((M4_STEPS_PER_SEC * M4_STEP_ANGLE * 60) / M4_TRAVEL_PER_REV)
#define B_VELOCITY_MAX A_VELOCITY_MAX
#define C_VELOCITY_MAX A_VELOCITY_MAX

// G1 max feed rate in mm/min
#define X_FEEDRATE_MAX X_VELOCITY_MAX
#define Y_FEEDRATE_MAX Y_VELOCITY_MAX
#define Z_FEEDRATE_MAX Z_VELOCITY_MAX
#define A_FEEDRATE_MAX A_VELOCITY_MAX
#define B_FEEDRATE_MAX B_VELOCITY_MAX
#define C_FEEDRATE_MAX C_VELOCITY_MAX

#define X_TRAVEL_MAX 100		// travel between switches or crashes
#define Y_TRAVEL_MAX 100
#define Z_TRAVEL_MAX 30
#define A_TRAVEL_MAX -1		// -1 is no limit (typ for rotary axis)
#define B_TRAVEL_MAX -1
#define C_TRAVEL_MAX -1

#define X_JERK_MAX JERK_MAX_COMMON
#define Y_JERK_MAX JERK_MAX_COMMON
#define Z_JERK_MAX JERK_MAX_COMMON
#define A_JERK_MAX JERK_MAX_COMMON
#define B_JERK_MAX JERK_MAX_COMMON
#define C_JERK_MAX JERK_MAX_COMMON

#define X_JUNCTION_DEVIATION JUNCTION_DEVIATION
#define Y_JUNCTION_DEVIATION JUNCTION_DEVIATION
#define Z_JUNCTION_DEVIATION JUNCTION_DEVIATION
#define A_JUNCTION_DEVIATION JUNCTION_DEVIATION
#define B_JUNCTION_DEVIATION JUNCTION_DEVIATION
#define C_JUNCTION_DEVIATION JUNCTION_DEVIATION

#define A_RADIUS 10					// radius in mm
#define B_RADIUS 10					// (XYZ values are not defined)
#define C_RADIUS 10

#define X_SWITCH_MODE TRUE			// 1=limit switches present and enabled
#define Y_SWITCH_MODE TRUE
#define Z_SWITCH_MODE TRUE
#define A_SWITCH_MODE FALSE
#define B_SWITCH_MODE FALSE
#define C_SWITCH_MODE FALSE

// homing settings
#define X_HOMING_SEARCH_TRAVEL 100
#define Y_HOMING_SEARCH_TRAVEL 80
//#define X_HOMING_SEARCH_TRAVEL X_TRAVEL_MAX
//#define Y_HOMING_SEARCH_TRAVEL Y_TRAVEL_MAX 
#define Z_HOMING_SEARCH_TRAVEL Z_TRAVEL_MAX 
#define A_HOMING_SEARCH_TRAVEL -1
#define B_HOMING_SEARCH_TRAVEL -1
#define C_HOMING_SEARCH_TRAVEL -1

#define X_HOMING_SEARCH_VELOCITY X_VELOCITY_MAX
#define Y_HOMING_SEARCH_VELOCITY Y_VELOCITY_MAX
#define Z_HOMING_SEARCH_VELOCITY Z_VELOCITY_MAX
#define A_HOMING_SEARCH_VELOCITY A_VELOCITY_MAX
#define B_HOMING_SEARCH_VELOCITY B_VELOCITY_MAX
#define C_HOMING_SEARCH_VELOCITY C_VELOCITY_MAX

#define X_HOMING_LATCH_VELOCITY 100		// mm/min
#define Y_HOMING_LATCH_VELOCITY 100
#define Z_HOMING_LATCH_VELOCITY 100
#define A_HOMING_LATCH_VELOCITY 3600	// degrees per minute
#define B_HOMING_LATCH_VELOCITY 3600
#define C_HOMING_LATCH_VELOCITY 3600

#define X_HOMING_ZERO_OFFSET 5			// mm
#define Y_HOMING_ZERO_OFFSET 5
#define Z_HOMING_ZERO_OFFSET 5
#define A_HOMING_ZERO_OFFSET 5			// degrees
#define B_HOMING_ZERO_OFFSET 5
#define C_HOMING_ZERO_OFFSET 5

#define X_HOMING_WORK_OFFSET (X_HOMING_SEARCH_TRAVEL/2) // offset to zero from axis minimum
#define Y_HOMING_WORK_OFFSET (Y_HOMING_SEARCH_TRAVEL/2)
//#define X_HOMING_WORK_OFFSET (X_TRAVEL_MAX/2) // offset to zero from axis minimum
//#define Y_HOMING_WORK_OFFSET (Y_TRAVEL_MAX/2)
#define Z_HOMING_WORK_OFFSET (Z_TRAVEL_MAX/2)
#define A_HOMING_WORK_OFFSET (A_TRAVEL_MAX/2)
#define B_HOMING_WORK_OFFSET (B_TRAVEL_MAX/2)
#define C_HOMING_WORK_OFFSET (C_TRAVEL_MAX/2)

#define X_HOMING_JERK 1000000000		// 1,000,000,000
#define Y_HOMING_JERK X_HOMING_JERK
#define Z_HOMING_JERK X_HOMING_JERK
#define A_HOMING_JERK X_HOMING_JERK
#define B_HOMING_JERK A_HOMING_JERK
#define C_HOMING_JERK a_HOMING_JERK
