/*
 * gcode.c - rs274/ngc parser.
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef gcode_h
#define gcode_h

struct GCodeParser {				// gcode parser state & helper variables
	uint8_t status;					// now uses unified TG_ status codes
	char letter;					// parsed letter, eg.g. G or X or Y
	double value;					// value parsed from letter (e.g. 2 for G2)
	double fraction;				// value fraction, eg. 0.1 for 92.1
};

struct GCodeModel {					// Gcode model- meaning depends on context
	double feed_rate; 				// F - normalized to millimeters/minute
	double seek_rate;				// seek rate in millimeters/second
	double max_feed_rate;			// max supported feed rate (mm/min)
	double max_seek_rate;			// max supported seek rate (mm/min)
	double inverse_feed_rate; 		// ignored if inverse_feed_rate not active
	uint8_t inverse_feed_rate_mode; // TRUE = inv (G93) FALSE = normal (G94)

	uint8_t set_plane;				// values to set plane to
	uint8_t plane_axis_0; 			// actual axes of the selected plane
	uint8_t plane_axis_1; 			// ...(set in gm only)
	uint8_t plane_axis_2; 

	uint8_t inches_mode;         	// TRUE = inches (G20), FALSE = mm (G21)
	uint8_t absolute_mode;			// TRUE = absolute (G90), FALSE = rel.(G91)
	uint8_t absolute_override; 		// TRUE = abs motion- this block only (G53)
	uint8_t set_origin_mode;		// TRUE = in set origin mode (G92)
	uint8_t	override_enable;		// TRUE = overrides enabled (M48), F=(M49)

	uint8_t tool;					// T value
	uint8_t change_tool;			// M6

	uint8_t spindle_mode;			// 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	double spindle_speed;			// in RPM
	double max_spindle_speed;		// limit

	uint8_t next_action;			// handles G modal Grp1 moves & non-modals
	uint8_t motion_mode;			// Group1: G0, G1, G2, G3, G38.2, G80, G81,
									// G82, G83 G84, G85, G86, G87, G88, G89 
	uint8_t program_flow;			// M0, M1 - pause / resume program flow

	double dwell_time;				// P - dwell time in seconds
	double position[3];				// X, Y, Z - meaning depends on context
	double target[3]; 				// X, Y, Z - where the move should go
	double offset[3];  				// I, J, K - used by arc commands
	double radius;					// R - radius value in arc raduis mode

/* unimplemented					// this block would follow inches_mode
	double cutter_radius;			// D - cutter radius compensation (0 is off)
	double cutter_length;			// H - cutter length compensation (0 is off)
	uint8_t coord_system;			// select coordinate system 1-9
	uint8_t path_control_mode;

   unimplemented					// this block would follow spindle_speed
	uint8_t mist_coolant_on;		// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant_on;		// TRUE = flood on (M8), FALSE = off (M9)
*/
};

/*
 * Global Scope Functions
 */

void gc_init(void);								// Initialize the parser
uint8_t gc_gcode_parser(char *block);

/* 
 * definitions used by gcode interpreter 
 *
 * The difference between NextAction and MotionMode is that NextAction is 
 * used by the current block, and may carry non-modal commands, whereas 
 * MotionMode persists across blocks (as G modal group 1)
 */

enum gcNextAction {						// motion mode and non-modals
	NEXT_ACTION_NONE,					// no moves
	NEXT_ACTION_MOTION,					// action set by MotionMode
	NEXT_ACTION_DWELL,					// G4
	NEXT_ACTION_GO_HOME,				// G28
	NEXT_ACTION_OFFSET_COORDINATES		// G92
};

enum gcMotionMode {						// G Modal Group 1
	MOTION_MODE_STRAIGHT_TRAVERSE,		// G0 - seek
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

enum gcPathControlMode {				// G Modal Group 13
	PATH_CONTROL_MODE_EXACT_PATH,
	PATH_CONTROL_MODE_EXACT_STOP,
	PATH_CONTROL_MODE_CONTINOUS
};

enum gcProgramFlow {
	PROGRAM_FLOW_START,					// START must be zero
	PROGRAM_FLOW_STOP,
	PROGRAM_FLOW_END
};

enum gcCanonicalSpindle {				// spindle settings
	SPINDLE_OFF,
	SPINDLE_CW,
	SPINDLE_CCW
};

enum gcCanonicalPlane {					// canonical plane - translates to:
										// axis_0	axis_1	axis_2
	CANON_PLANE_XY,						//   X		  Y		  Z
	CANON_PLANE_XZ,						//   X		  Z		  Y
	CANON_PLANE_YZ						//	 Y		  Z		  X							
};

enum gcDirection {						// used for spindle and arc dir
	DIRECTION_CW,
	DIRECTION_CCW
};

#endif


/**** GCODE NOTES ****/

/*---- Coordinate system notes ----

  TinyG runs a reduced functionality coordinate system from full NIST.
  Commands that affect the coordinate system are:

  	G10				Coordinate system origin setting
	G54 - G59.3		Select coordinate system (group 12)
	G92	- G92.3		Coordinate system offsets
	G43				Tool offset

  There are 9 coordinate systems (P1 - P9), plus the machine coordinate system
  which also defines the machine zero. Our challenge is that we don't know the 
  machine zero unless we go through a lengthy homing cycle - which is not even 
  necessarily supported on all machines. On power up the Gcode interpreter 
  is set to zero (X,Y,Z), which makes the machine zero the current (random) 
  position of the tool.

  The solution (hack) is to define P1 as the only supported coordinate system
  and simply ignore the machine coordinate system or make it the same as the 
  P1 system. The steps to set up the machine would be:
 
  Alternate 1 - using a homing cycle:
	- The machine travels to maximum excursion in all axes then resets to a 
	  machine coordinate zero position that is defined relative to the max 
	  excursions. In practice this would be either in the middle of the X/Y 
	  plane (4 quadrant solution) or in the "upper left", which is a the 
	  traditional zero point for many machines.
 
    - From this point the P1 coordinate system is set relative to the machine
	  coordinate system - either identical to it, or some config-defined
	  offset (like turning an upper-left zero into a 4 quadrant zero.
 
  Alternate 2 - using a "touch off" dialog (similar to Linux CNC)
	- The user must position the machine and enter zero. This defines the zero
	  for the P1 coordinate system. If the machine has a machine coordinate 
	  system 
 
  and require an offset (G10) to a floating machine zero. LinuxCNC uses a 
  "touch off" dialog to set zero


*/
/*---- Notes on Starting, Stopping and program state ----
  
  NIST RS274NGC_3 defines program run state semantics as so:

 	(Program) Start Program starts when if begins receiving blocks.
  					Corresponds to pressing the "cycle start" button.
					Program preserves state from previously run program, or
					defaults to persisted state (defaults, currently) upon power-on

 	(Program) Stop {M0} Program stops running temporarily (also M1)

 	(Program) End {M2} Program ends without the ability to resume
  					Also corresponds to trailing '%' sign in a g-code file

 	Reset 		  Resets machine parameters to defaults (NIST pg 38):
					- zero is reset
					- plane is set to xy
					- distance mode is set to absolute mode
					- feed rate mode is set to units per minute
					- spindle stopped
					- current motion mode set to G1
					- (others may be added)


  We define mappings as so:

  	^c	End and Reset
	^x	End and Reset
	^s	Stop
	^q	Start (resume)
	^z	Set coordinate system P1 origin to current tool position
*/
