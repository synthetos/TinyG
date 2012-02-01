/*
 * gcode.h - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart, Jr.
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

#ifndef gcode_h
#define gcode_h
#include "tinyg.h"

/*
 * Global Scope Functions
 */

void gc_init(void);								// Initialize the parser
uint8_t gc_gcode_parser(char *block);

/*
 * Structures
 */

struct GCodeParser {					// gcode parser state & helper variables
	uint8_t status;						// now uses unified TG_ status codes
	char letter;						// parsed letter, eg.g. G or X or Y
	double value;						// value parsed from letter (e.g. 2 for G2)
	double fraction;					// value fraction, eg. 0.1 for 64.1
};

struct GCodeModel {						// Gcode model- meaning depends on context
	uint8_t next_action;				// handles G modal group 1 moves & non-modals
	uint8_t motion_mode;				// Group1: G0, G1, G2, G3, G38.2, G80, G81,
										// G82, G83 G84, G85, G86, G87, G88, G89 
	uint8_t program_flow;

	double position[AXES];				// XYZ, ABC - meaning depends on context
	double target[AXES]; 				// XYZ, ABC - where the move should go
	double offset[3];  					// IJK - used by arc commands

	double feed_rate; 					// F - normalized to millimeters/minute
	double seek_rate;					// seek rate in millimeters/second
	double inverse_feed_rate; 			// ignored if inverse_feed_rate not active
	uint8_t inverse_feed_rate_mode;		// TRUE = inv (G93) FALSE = normal (G94)
	uint8_t feed_override_mode;			// TRUE = feed override is active, FALSE = inactive
	double feed_override_rate;			// 1.0000 = set feed rate. Go up or down from there

	uint8_t select_plane;				// values to set plane to
	uint8_t plane_axis_0;		 		// actual axes of the selected plane
	uint8_t plane_axis_1;		 		// ...(set in gm only)
	uint8_t plane_axis_2; 

	uint8_t inches_mode;				// TRUE = inches (G20), FALSE = mm (G21)
	uint8_t absolute_mode;				// TRUE = absolute (G90), FALSE = rel.(G91)
	uint8_t absolute_override;			// TRUE = abs motion- this block only (G53)
	uint8_t set_origin_mode;			// TRUE = in set origin mode (G92)
	uint8_t	override_enable;			// TRUE = overrides enabled (M48), F=(M49)
	uint8_t path_control;				// EXACT_STOP, EXACT_PATH, CONTINUOUS

	uint8_t tool;						// T value
	uint8_t change_tool;				// M6

	uint8_t spindle_mode;				// 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	double spindle_speed;				// in RPM

	double dwell_time;					// P - dwell time in seconds
	double radius;						// R - radius value in arc radius mode

/* unimplemented gcode					// this block would follow inches_mode
	double cutter_radius;				// D - cutter radius compensation (0 is off)
	double cutter_length;				// H - cutter length compensation (0 is off)
	uint8_t coord_system;				// select coordinate system 1-9
	uint8_t path_control_mode;

   unimplemented gcode					// this block would follow spindle_speed
	uint8_t mist_coolant_on;			// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant_on;			// TRUE = flood on (M8), FALSE = off (M9)
*/
};
struct GCodeModel gm;					// gcode model
struct GCodeModel gn;					// gcode input values
struct GCodeModel gf;					// gcode input flags
struct GCodeModel gt;					// gcode model temp storage for cycles

#endif
