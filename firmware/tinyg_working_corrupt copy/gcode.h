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
