/*
 * gcode.h - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef gcode_h
#define gcode_h
#include "tinyg.h"

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

/* unimplemented				// this block would follow inches_mode
	double cutter_radius;		// D - cutter radius compensation (0 is off)
	double cutter_length;		// H - cutter length compensation (0 is off)
	uint8_t coord_system;		// select coordinate system 1-9
	uint8_t path_control_mode;

   unimplemented				// this block would follow spindle_speed
	uint8_t mist_coolant_on;	// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant_on;	// TRUE = flood on (M8), FALSE = off (M9)
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
	PATH_EXACT_STOP,					// G61.1
	PATH_EXACT_PATH,					// G61
	PATH_CONTINUOUS,					// G64 and typically the default mode
	PATH_CONTINUOUS_FROM_ARC  			// special case for trajectory planner
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

/*---- Notes on structure of this code ----

  The Gcode interpreter and lower layers are organized as so:

	gcode.c/.h
	canonical_machine.c/.h
	planner.c/.h
	move_buffer.c/.h
	stepper.c/.h

	gcode.c/.h is the gcode parser. It reads and executes gcode blocks. 
	Gcode is executed by calling the underlying canonical machining 
	functions in canonical_machine.c/.h. The parser is stateless and 
	starts "from scratch" for each new gcode block (some state is 
	retrieved from the canonical machine).

	canonical_machine.c/.h implements the NIST RS274NGC canonical 
	machining functions (more or less). Some functions have been added,
	some not implemented, and some of the calling conventions are 
	different. The canonical machine normalizes all coordinates and 
	parameters to internal representation, keeps the Gcode model state,
	and makes calls to the motion planning layer for actual movement.
	The canonical machine may be extended to handle canned cycles, homing
	and probe cycles, and other complex cycles using motion primitives.
	(I'm not sure if this is exactly how Kramer planned it - particularly
	when it comes to state management, but it's how it's implemented).

	planner.c/.h plans trajectories and executes motion primitives for 
	the desired robot type. Motion primitives include lines, arcs, dwells, 
	stop/start. The motion planning  layer implements the actual robot 
	kinematics. At the current time only a cartesian robot for X,Y,Z and A 
	axes is supported. Other types of robot kinematics would extend this 
	layer. The motion planning layer may be extended to manage 
	acelleration/decelleration and path control.

	move_buffer.c/.h queues axis moves from the motion planning layer for 
	the steppers (or other motor control layer). It also pre-computes the 
	queued moves for the motors so the motor interrupts can just load
	values without having computational load during line segment 
	interstitials.

	stepper.c/.h runs the stepper motors. In TinyG these are independent 
	timers with their own interrupts. Stop, start and dwell are also 
	handled here. In other configurations this is where the DDA would go.

  A note about efficiency: Having all these layers doesn't mean that there
    are an excessive number of stack operations - just that things are 
	easier to maintain and visualize. Much of the code is run as inlines
	and static scoped variables (i.e. not passed on the stack). And even 
	if there were a lot of function calls, most of the code  doesn't need
	optimization anyway (with the exception of the steppers)
*/

/*---- Supported commands ----
 	G0				Rapid linear motion
	G1				Linear motion at feed rate
	G2, G3			Clockwise / counterclockwise arc at feed rate
	G4				Dwell
	G17, G18, G19	Select plane: XY plane {G17}, XZ plane {G18}, YZ plane {G19}
	G20, G21		Length units: inches {G20}, millimeters {G21}
	G53				Move in absolute coordinates
	G80				Cancel motion mode
	G90, G91		Set distance mode; absolute {G90}, incremental {G91}
	G92				Coordinate System Offsets - limited support provided
	G93, G94		Set feed rate mode: inverse time mode {G93}, 
										units per minute mode {G94}
	M0				Program stop
	M1				Optional program stop
	M2				Program end
	M3, M4			Turn spindle clockwise / counterclockwise
	M5				Stop spindle turning
	M30				Program end (pallet shuttle and reset)
	M60				Program stop (and pallet shuttle)

  Commands omitted for the time being:
	G10	  			Coordinate system data
	G14, G15		Spiral motion
	G28, G30		Return to home (requires parameters)
	G38.2 			Straight probe
	G40, G41, G42	Cutter radius compensation
	G43, G49		Tool length offsets
	G54 - G59.3		Select coordinate system (group 12)
	G61, G61.1, G64 Set path control mode (group 13)
	G81 - G89		Canned cycles
	G92	- G92.3		Coordinate system offsets
	G98, G99		Set canned cycle return level

	M6				Tool change
	M7, M8, M9		Coolant (group8)
	M48, M49		Enable/disable feed and speed override switches (group 9)
	
  Other commands and features intentionally not supported:
	- A,B,C axes
	- Multiple coordinate systems
	- Evaluation of expressions
	- Variables (Parameters)
	- Multiple home locations
	- Probing
	- Override control

  FYI: GCode modal groups (from NIST RS274NGC_3 Table 4)

   The modal groups for G codes are:
	group 1 = {G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89} motion
	group 2 = {G17, G18, G19} plane selection 
	group 3 = {G90, G91} distance mode 
	group 5 = {G93, G94} feed rate mode
	group 6 = {G20, G21} units 
	group 7 = {G40, G41, G42} cutter radius compensation 
	group 8 = {G43, G49} tool length offset 
	group 10 = {G98, G99} return mode in canned cycles 
	group 12 = {G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} coordinate system selection 
	group 13 = {G61, G61.1, G64} path control mode

   The modal groups for M codes are:
	group 4 = {M0, M1, M2, M30, M60} stopping 
	group 6 = {M6} tool change 
	group 7 = {M3, M4, M5} spindle turning 
	group 8 = {M7, M8, M9} coolant (special case: M7 and M8 may be active at the same time) 
	group 9 = {M48, M49} enable/disable feed and speed override switches

   In addition to the above modal groups, there is a group for non-modal G codes:
	group 0 = {G4, G10, G28, G30, G53, G92, G92.1, G92.2, G92.3}	
*/
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

 	Reset 	  Resets machine parameters to defaults (NIST pg 38):
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
/*---- Notes on use of rotary axis (A) and feedrate ----

  A single rotary axis is implemented - A, which is supposed to represent 
  rotation along the X axis.

  Feedrate may be in any of mm/min, inches/min, or degrees/min.
  Feedrate is kept in the canonical machine in mm/min or degrees/min.

  If the move is linear only (XYZ) feedrate is interpreted as in/min of mm/min
  If the move is rotary only (A) feedrate is interpreted as degrees/min

  If the move is mixed the move is interpreted as a helix 


*/
