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

/*
 * Global Scope Functions
 */

void gc_init(void);							// Initialize the parser
uint8_t gc_gcode_parser(char *block);
uint8_t gc_execute_block(char *line);		// Execute one block of rs275/ngc/g-code
void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);

/* 
 * various defines used by the gcode module 
 */

enum gcNextAction {
	NEXT_ACTION_NONE,						// no moves
	NEXT_ACTION_MOTION,						// move is set by motion mode (below)
	NEXT_ACTION_DWELL,
	NEXT_ACTION_GO_HOME,
	NEXT_ACTION_SET_COORDINATES
};

enum gcMotionMode {
	MOTION_MODE_RAPID_LINEAR,		 		// G0 
	MOTION_MODE_LINEAR,				 		// G1
	MOTION_MODE_CW_ARC,						// G2
	MOTION_MODE_CCW_ARC,					// G3
	MOTION_MODE_CANCEL,						// G80
};

enum gcPathControlMode {
	PATH_CONTROL_MODE_EXACT_PATH,
	PATH_CONTROL_MODE_EXACT_STOP,
	PATH_CONTROL_MODE_CONTINOUS
};

enum gcProgramFlow {
	PROGRAM_FLOW_RUNNING,					// must be zero
	PROGRAM_FLOW_PAUSED,
	PROGRAM_FLOW_COMPLETED
};

enum gcSpindleDirection {
	SPINDLE_DIRECTION_CW,
	SPINDLE_DIRECTION_CCW
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
