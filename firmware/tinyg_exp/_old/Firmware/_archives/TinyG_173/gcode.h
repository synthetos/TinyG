/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef gcode_h
#define gcode_h

/*
 * Global Scope Functions
 */

void gc_init(void);							// Initialize the parser
uint8_t gc_gcode_parser(char *block);
uint8_t gc_execute_line(char *line);		// Execute one block of rs275/ngc/g-code
void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);

/* 
 * various defines used by the gcode module 
 */

#define NEXT_ACTION_DEFAULT 	0
#define NEXT_ACTION_DWELL 		1
#define NEXT_ACTION_GO_HOME 	2

#define MOTION_MODE_RAPID_LINEAR 0 		// G0 
#define MOTION_MODE_LINEAR 		1 		// G1
#define MOTION_MODE_CW_ARC 		2  		// G2
#define MOTION_MODE_CCW_ARC 	3  		// G3
#define MOTION_MODE_CANCEL 		4 		// G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS  2

#define PROGRAM_FLOW_RUNNING	0
#define PROGRAM_FLOW_PAUSED		1
#define PROGRAM_FLOW_COMPLETED	2

#define SPINDLE_DIRECTION_CW	0
#define SPINDLE_DIRECTION_CCW	1

/* The following enums are used for G-code status returns
 * They can also be overloaded (extended) by negative numbers for _FDEV_ERR returns
 */
enum gcStatus {
	GC_STATUS_OK,
	GC_STATUS_QUIT,						// encountered a Quit command
	GC_STATUS_CONTINUE,					// returned if GC command requires a continuation
	GC_STATUS_BAD_NUMBER_FORMAT,
	GC_STATUS_EXPECTED_COMMAND_LETTER,
	GC_STATUS_UNSUPPORTED_STATEMENT,
	GC_STATUS_MOTION_CONTROL_ERROR,
	GC_STATUS_FLOATING_POINT_ERROR
};
#endif
