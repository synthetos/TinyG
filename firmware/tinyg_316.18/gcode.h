/*
 * gcode.h - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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

// Constants
#define ONE_MINUTE_OF_MICROSECONDS 60000000
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define MM_PER_INCH 25.4
#define RADIAN 57.2957795
#define INCHES_MODE 1
#define MILLIMETER_MODE 0
#define ABSOLUTE_MODE 1	
#define INCREMENTAL_MODE 0

// Settings
//#define MIN_LINE_LENGTH 0.01	// mm - smallest complete line it can handle
#define MIN_LINE_LENGTH 0.03	// mm - smallest complete line it can handle
#define MIN_SEGMENT_LENGTH 0.03	// accel/decel segments - must be <= MIN_LINE_LENGTH
#define MM_PER_ARC_SEGMENT 0.03	// set to produce ~10 ms segments
#define MIN_SEGMENT_TIME 10000	// microseconds - 10 ms. works well


struct GCodeParser {			// gcode parser state & helper variables
	uint8_t status;				// now uses unified TG_ status codes
	char letter;				// parsed letter, eg.g. G or X or Y
	double value;				// value parsed from letter (e.g. 2 for G2)
	double fraction;			// value fraction, eg. 0.1 for 92.1
};

struct GCodeModel {				// Gcode model- meaning depends on context
	uint8_t next_action;		// handles G modal group 1 moves & non-modals
	uint8_t motion_mode;		// Group1: G0, G1, G2, G3, G38.2, G80, G81,
								// G82, G83 G84, G85, G86, G87, G88, G89 
	uint8_t program_flow;		// M0, M1 - pause / resume program flow

	double position[AXES];		// X, Y, Z, A - meaning depends on context
	double target[AXES]; 		// X, Y, Z, A - where the move should go
	double offset[3];  			// I, J, K - used by arc commands

	double feed_rate; 			// F - normalized to millimeters/minute
	double seek_rate;			// seek rate in millimeters/second
	double max_feed_rate;		// max supported feed rate (mm/min)
	double max_seek_rate;		// max supported seek rate (mm/min)
	double inverse_feed_rate; 	// ignored if inverse_feed_rate not active
	uint8_t inverse_feed_rate_mode; // TRUE = inv (G93) FALSE = normal (G94)

	uint8_t set_plane;			// values to set plane to
	uint8_t plane_axis_0; 		// actual axes of the selected plane
	uint8_t plane_axis_1; 		// ...(set in gm only)
	uint8_t plane_axis_2; 

	uint8_t inches_mode;        // TRUE = inches (G20), FALSE = mm (G21)
	uint8_t absolute_mode;		// TRUE = absolute (G90), FALSE = rel.(G91)
	uint8_t absolute_override; 	// TRUE = abs motion- this block only (G53)
	uint8_t set_origin_mode;	// TRUE = in set origin mode (G92)
	uint8_t	override_enable;	// TRUE = overrides enabled (M48), F=(M49)
	uint8_t path_control_mode;	// EXACT_STOP, EXACT_PATH, CONTINUOUS

	uint8_t tool;				// T value
	uint8_t change_tool;		// M6

	uint8_t spindle_mode;		// 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	double spindle_speed;		// in RPM
	double max_spindle_speed;	// limit

	double dwell_time;			// P - dwell time in seconds
	double radius;				// R - radius value in arc radius mode

/* unimplemented gcode			// this block would follow inches_mode
	double cutter_radius;		// D - cutter radius compensation (0 is off)
	double cutter_length;		// H - cutter length compensation (0 is off)
	uint8_t coord_system;		// select coordinate system 1-9
	uint8_t path_control_mode;

   unimplemented gcode			// this block would follow spindle_speed
	uint8_t mist_coolant_on;	// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant_on;	// TRUE = flood on (M8), FALSE = off (M9)
*/
};
struct GCodeModel gm;			// gcode model
struct GCodeModel gn;			// gcode input values
struct GCodeModel gf;			// gcode input flags
struct GCodeModel gt;			// gcode model temp storage for cycles

/*
 * Global Scope Functions
 */

void gc_init(void);								// Initialize the parser
uint8_t gc_gcode_parser(char *block);
uint8_t gc_read_double(char *buf, uint8_t *i, double *double_ptr);

/* 
 * definitions used by gcode interpreter and canonical machine
 *
 * The difference between NextAction and MotionMode is that NextAction is 
 * used by the current block, and may carry non-modal commands, whereas 
 * MotionMode persists across blocks (as G modal group 1)
 */

enum gcNextAction {						// motion mode and non-modals
	NEXT_ACTION_NONE = 0,				// no moves
	NEXT_ACTION_MOTION,					// action set by MotionMode
	NEXT_ACTION_DWELL,					// G4
	NEXT_ACTION_GO_HOME,				// G28
	NEXT_ACTION_OFFSET_COORDINATES		// G92
};

enum gcMotionMode {						// G Modal Group 1
	MOTION_MODE_STRAIGHT_TRAVERSE = 0,	// G0 - seek
	MOTION_MODE_STRAIGHT_FEED,			// G1 - feed
	MOTION_MODE_CW_ARC,					// G2 - arc feed
	MOTION_MODE_CCW_ARC,				// G3 - arc feed
	MOTION_MODE_STRAIGHT_PROBE,			// G38.2
	MOTION_MODE_CANCEL_MOTION_MODE,		// G80
	MOTION_MODE_CANNED_CYCLE_81,		// G81 - drilling
	MOTION_MODE_CANNED_CYCLE_82,		// G82 - drilling with dwell
	MOTION_MODE_CANNED_CYCLE_83,		// G83 - peck drilling
	MOTION_MODE_CANNED_CYCLE_84,		// G84 - right hand tapping
	MOTION_MODE_CANNED_CYCLE_85,		// G85 - boring, no dwell, feed out
	MOTION_MODE_CANNED_CYCLE_86,		// G86 - boring, spindle stop, rapid out
	MOTION_MODE_CANNED_CYCLE_87,		// G87 - back boring
	MOTION_MODE_CANNED_CYCLE_88,		// G88 - boring, spindle stop, manual out
	MOTION_MODE_CANNED_CYCLE_89			// G89 - boring, dwell, feed out
};

enum gcUnitsMode {						// spindle settings
	UNITS_INCHES = 0,
	UNITS_MM
};

enum gcPathControlMode {				// G Modal Group 13
	PATH_EXACT_STOP = 0,				// G61
	PATH_EXACT_PATH,					// G61.1
	PATH_CONTINUOUS,					// G64 and typically the default mode
	PATH_CONTINUOUS_FROM_ARC  			// special case for trajectory planner
};

enum gcProgramFlow {
	PROGRAM_FLOW_START = 0,				// START must be zero
	PROGRAM_FLOW_STOP,
	PROGRAM_FLOW_END
};

enum gcCanonicalSpindle {				// spindle settings
	SPINDLE_OFF = 0,
	SPINDLE_CW,
	SPINDLE_CCW
};

enum gcCanonicalPlane {					// canonical plane - translates to:
										// axis_0	axis_1	axis_2
	CANON_PLANE_XY = 0,					//   X		  Y		  Z
	CANON_PLANE_XZ,						//   X		  Z		  Y
	CANON_PLANE_YZ						//	 Y		  Z		  X							
};

enum gcDirection {						// used for spindle and arc dir
	DIRECTION_CW = 0,
	DIRECTION_CCW
};

enum tgAxisMode {		// axis modes (ordered: see _cm_get_feed_time())
	AXIS_DISABLED = 0,	// kill axis
	AXIS_STANDARD,		// axis in coordinated motion w/standard behaviors
	AXIS_INHIBITED,		// axis is computed but not activated
	AXIS_RADIUS,		// rotary axis calibrated to circumference
	AXIS_SLAVE_X,		// rotary axis slaved to X axis
	AXIS_SLAVE_Y,		// rotary axis slaved to Y axis
	AXIS_SLAVE_Z,		// rotary axis slaved to Z axis
	AXIS_SLAVE_XY,		// rotary axis slaved to XY plane
	AXIS_SLAVE_XZ,		// rotary axis slaved to XZ plane
	AXIS_SLAVE_YZ,		// rotary axis slaved to YZ plane
	AXIS_SLAVE_XYZ		// rotary axis slaved to XYZ movement
};	// ordering must be preserved. See _cm_get_feed_time() and seek time()

#endif
