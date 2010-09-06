/*
 * gcode.c - rs274/ngc parser.
 * Part of Grbl
 * This code is inspired by the Arduino GCode Interpreter by Mike Ellery and 
 * the NIST RS274/NGC Interpreter by Kramer, Proctor and Messina. 
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 * Modified for TinyG project by Alden S Hart, Jr.
 *
 * Grbl is free software: you can redistribute it and/or modify it under the 
 * termsof the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * Grbl is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along 
 * with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */
/* 
  Supported commands are:
 	G0				Rapid linear motion
	G1				Linear motion at feed rate
	G2, G3			Clockwise / counterclockwise arc at feed rate
	G4				Dwell
	G17, G18, G19	Select plane: XY plane {G17}, XZ plane {G18}, YZ plane {G19}
	G20, G21		Length units: inches {G20}, millimeters {G21}
	G53				Move in absolute coordinates
	G80				Cancel motion mode
	G90, G91		Set distance mode; absolute {G90}, incremental {G91}
	G93, G94		Set feed rate mode: inverse time mode {93}, 
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

#include <ctype.h>
#include <stdlib.h>
#include <string.h>					// for memset()
#include <math.h>
#include <stdio.h>
#include <avr/pgmspace.h>			// needed for exception strings

#include "tinyg.h"
#include "gcode.h"					// must precede config.h
#include "config.h"
#include "controller.h"
#include "motion_control.h"
#include "spindle.h"

/* data structures 
 *
 * - gp is a minimal structure to keep parser state
 *
 * The three GCodeModel structs look the same but have different uses:
 *
 * - gm keeps the internal state model in normalized, canonical form. All
 * 	 values are unit converted (to mm) and in the internal coordinate system.
 *	 Gm is owned by the canonical motion layer and is  accessed by the 
 *	 parser through cm_ routines (which include various setters and getters).
 *	 Gm's state persists from block to block.
 *
 * - gn records the data in the new gcode block in the formats present in
 *	 the block (pre-normalized forms). It is initialized for each block 
 *	 during which some state elements may be restored from gm.
 *
 * - gf is a flag struct which records any data that has changed in gn. 
 */
static struct GCodeParser gp;		// gcode parser variables
static struct GCodeModel gm;		// gcode model - current state
static struct GCodeModel gn;		// gcode model - current block values
static struct GCodeModel gf;		// gcode model - flags changed values

/* local helper functions and macros */
static void _gc_normalize_gcode_block(char *block);
static int _gc_parse_gcode_block(char *line);	// Parse the block into structs
static int _gc_execute_gcode_block(void);		// Execute the gcode block
static int _gc_read_double(char *buf, uint8_t *i, double *double_ptr);
static int _gc_next_statement(char *letter, double *value_ptr, double *fraction_ptr, char *line, uint8_t *i);
static int _gc_compute_radius_arc(void);
static int _gc_compute_center_arc(void);
static double _theta(double x, double y);

#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#define SET_NEXT_STATE(a,v) ({gn.a=v; gf.a=1; break;})
#define SET_NEXT_STATE_x2(a,v,b,w) ({gn.a=v; gf.a=1; gn.b=w; gf.a=1; break;})
#define SET_NEXT_ACTION_MOTION(a,v) ({gn.a=v; gf.a=1; gn.next_action=NEXT_ACTION_MOTION; gf.next_action=1; break;})

/* 
 * gc_init() 
 */

void gc_init()
{
	ZERO_MODEL_STATE(&gm);					// most vars start at zero
	ZERO_MODEL_STATE(&gn);
	ZERO_MODEL_STATE(&gf);

	cm_select_plane(CANON_PLANE_XY);		// default planes, 0, 1 and 2
	gm.seek_rate = cfg.max_seek_rate;		// in mm/minute
	gm.max_seek_rate = cfg.max_seek_rate;	// in mm/minute
	gm.max_feed_rate = cfg.max_feed_rate;	// in mm/minute
//	gm.inches_mode = TRUE;					// default to inches (G20)
	gm.absolute_mode = TRUE;				// default to absolute mode (G90)
}

/*
 * gc_gcode_parser() - parse a block (line) of gcode
 */

uint8_t gc_gcode_parser(char *block)
{
	_gc_normalize_gcode_block(block);
	if (block[0] == 0) { 					// ignore comments (stripped)
		return(TG_OK);
	}
	if (block[0] == 'Q') {					// quit gcode mode
		return(TG_QUIT);
	}
	if (_gc_parse_gcode_block(block)) {		// parse block or fail trying
		return (gp.status);
	}
	gp.status = _gc_execute_gcode_block();	// execute gcode block
	tg_print_status(gp.status, block);
	return (gp.status);
}

/*
 * _gc_normalize_gcode_block() - normalize a block (line) of gcode in place
 *
 *	Comments always terminate the block (embedded comments are not supported)
 *	Messages in comments are sent to console (stderr)
 *	Processing: split string into command and comment portions. Valid choices:
 *	  supported:	command
 *	  supported:	comment
 *	  supported:	command comment
 *	  unsupported:	command command
 *	  unsupported:	comment command
 *	  unsupported:	command comment command
 *
 *	Valid characters in a Gcode block are (see RS274NGC_3 Appendix E)
 *		digits						all digits are passed to interpreter
 *		lower case alpha			all alpha is passed
 *		upper case alpha			all alpha is passed
 *		+ - . / *	< = > 			chars passed to interpreter
 *		| % # ( ) [ ] { } 			chars passed to interpreter
 *		<sp> <tab> 					chars are legal but are not passed
 *		/  							if first, block delete char - omits the block
 *
 *	Invalid characters in a Gcode block are:
 *		control characters			chars < 0x20
 *		! $ % ,	; ; ? @ 
 *		^ _ ~ " ' <DEL>
 *
 *	MSG specifier in comment can have mixed case but cannot cannot have 
 *	embedded white spaces
 *
 *	++++ todo: Support leading and trailing spaces around the MSG specifier
 */

void _gc_normalize_gcode_block(char *block) 
{
	char c;
	char *comment=0;	// comment pointer - first char past opening paren
	uint8_t i=0; 		// index for incoming characters
	uint8_t j=0;		// index for normalized characters

	// discard deleted block
	if (block[0] == '/') {
		block[0] = 0;
		return;
	}
	// normalize the comamnd block & mark the comment(if any)
	while ((c = toupper(block[i++])) != 0) {// NUL character
		if ((isupper(c)) || (isdigit(c))) {	// capture common chars
		 	block[j++] = c; 
			continue;
		}
		if (c == '(') {						// detect & handle comments
			block[j] = 0;
			comment = &block[i]; 
			break;
		}
		if (c <= ' ') continue;				// toss controls & whitespace
		if (c == 0x7F) continue;			// toss DELETE
		if (strchr("!$%,;:?@^_~`\'\"", c))	// toss invalid punctuation
			continue;
		block[j++] = c;
	}
	block[j] = 0;							// nul terminate the command
	if (comment) {
		if ((toupper(comment[0]) == 'M') && 
			(toupper(comment[1]) == 'S') &&
			(toupper(comment[2]) == 'G')) {
			i=0;
			while ((c = comment[i++]) != 0) {// remove trailing parenthesis
				if (c == ')') {
					comment[--i] = 0;
					break;
				}
			}
			cm_message(comment+3);
//			printf_P(PSTR("%s\n"),(comment+3)); // canonical machining func
		}
	}
}

/* 
 * _theta(double x, double y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static double _theta(double x, double y)
{
	double theta = atan(x/fabs(y));

	if (y>0) {
		return(theta);
	} else {
		if (theta>0) 
	    {
			return(M_PI-theta);
    	} else {
			return(-M_PI-theta);
		}
	}
}

/* 
 * _gc_next_statement() - parse next block of Gcode
 *
 *	Parses the next statement and leaves the counter on the first character 
 *	following the statement. 
 *	Returns TRUE if there was a statement, FALSE if end of string was reached
 *	or there was an error (check gp.status).
 */

int _gc_next_statement(char *letter, double *value_ptr, double *fraction_ptr, 
					   char *buf, uint8_t *i) {
	if (buf[*i] == 0) {
		return(FALSE); // No more statements
	}
	*letter = buf[*i];
	if(!isupper(*letter)) {
		gp.status = TG_EXPECTED_COMMAND_LETTER;
		return(FALSE);
	}
	(*i)++;
	if (!_gc_read_double(buf, i, value_ptr)) {
		return(FALSE);
	};
	*fraction_ptr = (*value_ptr - trunc(*value_ptr));
	return(TRUE);
}

/* 
 * _gc_read_double() - read a double from a Gcode statement 
 *
 *	buf			string: line of RS274/NGC code being processed
 *	i			index into string array (position on the line)
 *	double_ptr	pointer to double to be read
 */

int _gc_read_double(char *buf, uint8_t *i, double *double_ptr) 
{
	char *start = buf + *i;
	char *end;
  
	*double_ptr = strtod(start, &end);
	if(end == start) { 
		gp.status = TG_BAD_NUMBER_FORMAT; 
		return(FALSE); 
	};
	*i = end - buf;
	return(TRUE);
}

/*
 * _gc_parse_gcode_block() - parses one line of NULL terminated G-Code. 
 *
 *	All the parser does is load the state values in gn (next model state),
 *	and flags in gf (model state flags). The execute routine applies them.
 *	The line is assumed to contain only uppercase characters and signed 
 *  floats (no whitespace).
 *
 *	A lot of implicit things happen when the gn struct is zeroed:
 *	  - inverse feed rate mode is cancelled - set back to units_per_minute mode
 */

int _gc_parse_gcode_block(char *buf) 
{
	uint8_t i = 0;  			// index into Gcode block buffer (buf)
  
	ZERO_MODEL_STATE(&gn);		// clear all next-state values
	ZERO_MODEL_STATE(&gf);		// clear all next-state flags

	// pull needed state from gm structure to preset next state
	for (i=X; i<=Z; i++) {
		gn.target[i] = cm_get_position(i);	// pre-set target values
		gn.position[i] = gn.target[i];		//...and current position
	}
	gn.next_action = cm_get_next_action();	// next action persists
	gn.motion_mode = cm_get_motion_mode();	// set motion mode (modal group1)

	gp.status = TG_OK;						// initialize return code

  	// extract commands and parameters
	i = 0;
	while(_gc_next_statement(&gp.letter, &gp.value, &gp.fraction, buf, &i)) {
    	switch(gp.letter) {
			case 'G':
				switch((int)gp.value) {
					case 0:  SET_NEXT_ACTION_MOTION(motion_mode, MOTION_MODE_STRAIGHT_TRAVERSE);
					case 1:  SET_NEXT_ACTION_MOTION(motion_mode, MOTION_MODE_STRAIGHT_FEED);
					case 2:  SET_NEXT_ACTION_MOTION(motion_mode, MOTION_MODE_CW_ARC);
					case 3:  SET_NEXT_ACTION_MOTION(motion_mode, MOTION_MODE_CCW_ARC);
					case 4:  SET_NEXT_STATE(next_action, NEXT_ACTION_DWELL);
					case 17: SET_NEXT_STATE(set_plane, CANON_PLANE_XY);
					case 18: SET_NEXT_STATE(set_plane, CANON_PLANE_XZ);
					case 19: SET_NEXT_STATE(set_plane, CANON_PLANE_YZ);
					case 20: SET_NEXT_STATE(inches_mode, TRUE);
					case 21: SET_NEXT_STATE(inches_mode, FALSE);
					case 28: SET_NEXT_STATE(next_action, NEXT_ACTION_GO_HOME);
					case 30: SET_NEXT_STATE(next_action, NEXT_ACTION_GO_HOME);
					case 53: SET_NEXT_STATE(absolute_override, TRUE);
					case 80: SET_NEXT_STATE(motion_mode, MOTION_MODE_CANCEL_MOTION_MODE);
					case 90: SET_NEXT_STATE(absolute_mode, TRUE);
					case 91: SET_NEXT_STATE(absolute_mode, FALSE);
					case 92: SET_NEXT_STATE(set_origin_mode, TRUE);
					case 93: SET_NEXT_STATE(inverse_feed_rate_mode, TRUE);
					case 94: SET_NEXT_STATE(inverse_feed_rate_mode, FALSE);
					case 40: break;	// ignore cancel cutter radius compensation
					case 49: break;	// ignore cancel tool length offset comp.
					case 61: break;	// ignore set exact path (it is anyway)
					default: gp.status = TG_UNSUPPORTED_STATEMENT;
				}
				break;

			case 'M':
				switch((int)gp.value) {
					case 0: case 1: 
							SET_NEXT_STATE(program_flow, PROGRAM_FLOW_STOP);
					case 2: case 30: case 60:
							SET_NEXT_STATE(program_flow, PROGRAM_FLOW_END);
					case 3: SET_NEXT_STATE(spindle_mode, SPINDLE_CW);
					case 4: SET_NEXT_STATE(spindle_mode, SPINDLE_CCW);
					case 5: SET_NEXT_STATE(spindle_mode, SPINDLE_OFF);
					case 6: SET_NEXT_STATE(change_tool, TRUE);
					case 7: break;	// ignore mist coolant on
					case 8: break;	// ignore flood coolant on
					case 9: break;	// ignore mist and flood coolant off
					case 48: break;	// enable speed and feed overrides
					case 49: break;	// disable speed and feed overrides
 					default: gp.status = TG_UNSUPPORTED_STATEMENT;
				}
				break;

			case 'T': SET_NEXT_STATE(tool, trunc(gp.value));
			case 'F': SET_NEXT_STATE(feed_rate, gp.value);
			case 'P': SET_NEXT_STATE(dwell_time, gp.value);
			case 'S': SET_NEXT_STATE(spindle_speed, gp.value); 
			case 'X': SET_NEXT_STATE(target[X], gp.value);
			case 'Y': SET_NEXT_STATE(target[Y], gp.value);
			case 'Z': SET_NEXT_STATE(target[Z], gp.value);
			case 'I': SET_NEXT_STATE(offset[0], gp.value);
			case 'J': SET_NEXT_STATE(offset[1], gp.value);
			case 'K': SET_NEXT_STATE(offset[2], gp.value);
			case 'R': SET_NEXT_STATE(radius, gp.value);
			case 'N': break;	// ignore line numbers
			default: gp.status = TG_UNSUPPORTED_STATEMENT;
		}
		if(gp.status) {
			break;
		}
	}
	return(gp.status);
}

/*
 * _gc_execute_gcode_block() - execute parsed block
 *
 *  Conditionally (based on whether a flag is set in gf) call the canonical 
 *	machining functions in order of execution as per RS274NGC_3 table 8 
 *  (below, with modifications):
 *
 *		1. comment (includes message) [handled during block normalization]
 *		2. set feed rate mode (G93, G94 - inverse time or per minute)
 *		3. set feed rate (F)
 *		4. set spindle speed (S)
 *		5. select tool (T)
 *		6. change tool (M6)
 *		7. spindle on or off (M3, M4, M5)
 *		8. coolant on or off (M7, M8, M9)
 *		9. enable or disable overrides (M48, M49)
 *		10. dwell (G4)
 *		11. set active plane (G17, G18, G19)
 *		12. set length units (G20, G21)
 *		13. cutter radius compensation on or off (G40, G41, G42)
 *		14. cutter length compensation on or off (G43, G49)
 *		15. coordinate system selection (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3)
 *		16. set path control mode (G61, G61.1, G64)
 *		17. set distance mode (G90, G91)
 *		18. set retract mode (G98, G99)
 *		19a. home (G28, G30) or
 *		19b. change coordinate system data (G10) or
 *		19c. set axis offsets (G92, G92.1, G92.2, G94)
 *		20. perform motion (G0 to G3, G80-G89) as modified (possibly) by G53
 *		21. stop (M0, M1, M2, M30, M60)
 *
 *	Values in gn are in original units and should not be unit converted prior 
 *	to calling the canonical functions (which do the unit conversions)
 */

#define CALL_CM_FUNC(f,v) if(gf.v) {if((gp.status = f(gn.v))) {return(gp.status);}}
/* Example:
	if (gf.feed_rate) {
		if ((gp.status = cm_set_feed_rate(gn.feed_rate))) {
			return(gp.status);								// error return
		}
	}
 */

int _gc_execute_gcode_block() 
{
	CALL_CM_FUNC(cm_set_inverse_feed_rate_mode, inverse_feed_rate_mode);
	CALL_CM_FUNC(cm_set_feed_rate, feed_rate);
	CALL_CM_FUNC(cm_set_spindle_speed, spindle_speed);
	CALL_CM_FUNC(cm_select_tool, tool);
	CALL_CM_FUNC(cm_change_tool, tool);

	// spindle on or off
	if (gf.spindle_mode) {
    	if (gn.spindle_mode == SPINDLE_CW) {
			cm_start_spindle_clockwise();
		} else if (gn.spindle_mode == SPINDLE_CCW) {
			cm_start_spindle_counterclockwise();
		} else {
			cm_stop_spindle_turning();	// failsafe: any error causes stop
		}
	}

 	//--> coolant on or off goes here
	//--> enable or disable overrides goes here

	// dwell
	if (gn.next_action == NEXT_ACTION_DWELL) {
		if ((gp.status = cm_dwell(gn.dwell_time))) {
			return (gp.status);
		}
	}

	CALL_CM_FUNC(cm_select_plane, set_plane);
	CALL_CM_FUNC(cm_use_length_units, inches_mode);

	//--> cutter radius compensation goes here
	//--> cutter length compensation goes here
	//--> coordinate system selection goes here
	//--> set path control mode goes here

	CALL_CM_FUNC(cm_set_distance_mode, absolute_mode);

	//--> set retract mode goes here

	// homing cycle
	if (gn.next_action == NEXT_ACTION_GO_HOME) {
		if ((gp.status = cm_return_to_home())) {
			return (gp.status);								// error return
		}
	}

	//--> change coordinate system data goes here

	// set axis offsets
	if (gn.next_action == NEXT_ACTION_OFFSET_COORDINATES) {
		if ((gp.status = cm_set_origin_offsets(
						 gn.target[X], gn.target[Y], gn.target[Z]))) {
			return (gp.status);								// error return
		}
	}

	// G0 (linear traverse motion command)
	if ((gn.next_action == NEXT_ACTION_MOTION) && 
	    (gn.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE)) {
		gp.status = cm_straight_traverse(gn.target[X], gn.target[Y], gn.target[Z]);
		return (gp.status);
	}

	// G1 (linear feed motion command)
	if ((gn.next_action == NEXT_ACTION_MOTION) && 
	    (gn.motion_mode == MOTION_MODE_STRAIGHT_FEED)) {
		gp.status = cm_straight_feed(gn.target[X], gn.target[Y], gn.target[Z]);
		return (gp.status);
	}

	// G2 or G3 (arc motion command)
	if ((gn.next_action == NEXT_ACTION_MOTION) &&
	   ((gn.motion_mode == MOTION_MODE_CW_ARC) || 
		(gn.motion_mode == MOTION_MODE_CCW_ARC))) {
		// gf.radius sets radius mode if radius was collected in gn
		gp.status = cm_arc_feed(gn.motion_mode, gf.radius);
		return (gp.status);
	}
	return(gp.status);
}


/*************************************************************************
 *
 * CANONICAL MACHINING FUNCTIONS
 *
 *	Values are passed in pre-unit_converted state
 *	All operations occur on gm (current model state)
 *
 ************************************************************************/

/*--- HELPER ROUTINES ---*/

/*
 * Helpers
 *
 * _to_millimeters()
 */

inline double _to_millimeters(double value)
{
	return(gm.inches_mode ? (value * MM_PER_INCH) : value);
}
/*
 * Getters
 *
 * cm_get_position() - return position from the gm struct into gn struct form
 * cm_get_next_action() - get next_action from the gm struct
 * cm_get_motion_mode() - get motion mode from the gm struct
 */
inline double cm_get_position(uint8_t axis) 
{
	return (gm.inches_mode ? (gm.position[axis] / MM_PER_INCH) : gm.position[axis]);
}

inline uint8_t cm_get_next_action() { return gm.next_action; }
inline uint8_t cm_get_motion_mode() { return gm.motion_mode; }


/*
 * Setters
 *
 * cm_set_position() - set XYZ position
 * cm_set_target() - set XYZ target
 * cm_set_offset() - set IJK offset
 * cm_set_radius() - set R
 * cm_set_position_to_target()
 *
 *	Input coordinates are not unit adjusted (are raw gn coords)
 */

inline void cm_set_position(double x, double y, double z) 
{ 
	gm.position[X] = _to_millimeters(x);
	gm.position[Y] = _to_millimeters(y);
	gm.position[Z] = _to_millimeters(z);
}

inline void cm_set_target(double x, double y, double z) 
{ 
	gm.target[X] = _to_millimeters(x);
	gm.target[Y] = _to_millimeters(y);
	gm.target[Z] = _to_millimeters(z);
}

inline void cm_set_offset(double i, double j, double k) 
{ 
	gm.offset[0] = _to_millimeters(i);
	gm.offset[1] = _to_millimeters(j);
	gm.offset[2] = _to_millimeters(k);
}

inline void cm_set_radius(double r) 
{ 
	gm.radius = _to_millimeters(r);
}


/*--- CANONICAL MACHINING FUNCTIONS ---*/

/*
 * cm_comment() - ignore comments (I do)
 */

uint8_t cm_comment(char *comment)
{
	return (TG_OK);		// no operation
}

/*
 * cm_message() - send message to console
 */

uint8_t cm_message(char *message)
{
	printf_P(PSTR("%s\n"), message);
	return (TG_OK);
}

/*
 * cm_straight_traverse() - linear seek (G0)
 */

uint8_t cm_straight_traverse(double x, double y, double z)
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_target(x, y, z);

	// execute the move
	gp.status = mc_line(gm.target[X], gm.target[Y], gm.target[Z], gm.seek_rate, FALSE);

	// set final position
	cm_set_position(x, y, z);
	return (gp.status);
}

/*
 * cm_straight_feed() - G1
 */

uint8_t cm_straight_feed(double x, double y, double z)
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
	cm_set_target(x, y, z);

	// execute the move
	gp.status = mc_line(gm.target[X], gm.target[Y], gm.target[Z],  
					  ((gm.inverse_feed_rate_mode) ? gm.inverse_feed_rate : gm.feed_rate),
						gm.inverse_feed_rate_mode);

	// set final position
	/* As far as the gcode engine is concerned the position is now the target.
	 * In reality, motion_control / steppers will still be processing the
	 * action and the real tool position is still close to the starting point.
	 * The endpoint position is not moved if there has been an error.
	 */
	cm_set_position(x, y, z);
	return (gp.status);
}

/*
 * cm_set_inverse_feed_rate()
 *
 *	TRUE = inverse time feed rate in effect - for this block only
 *	FALSE = units per minute feed rate in effect
 */

inline uint8_t cm_set_inverse_feed_rate_mode(uint8_t mode)
{
	gm.inverse_feed_rate_mode = mode;
	return (TG_OK);
}

/*
 * cm_set_feed_rate() - F parameter
 *
 * Sets feed rate; or sets inverse feed rate if it's active.
 * Converts all values to internal format (mm's)
 * Errs out of feed rate exceeds maximum, but doesn't compute maximum for 
 * inverse feed rate as this would require knowing the move length in advance.
 */

uint8_t cm_set_feed_rate(double rate)
{
	if (gm.inverse_feed_rate_mode) {
		gm.inverse_feed_rate = rate; // minutes per motion for this block only
	} else {
		rate = _to_millimeters(rate);
		if (rate > gm.max_feed_rate) {
			return (TG_MAX_FEED_RATE_EXCEEDED);
		} else {
			gm.feed_rate = rate; 	// as mm per minute
		}
	}
	return (TG_OK);
}

/*
 * cm_set_traverse_rate() - set seek rate
 */

uint8_t cm_set_traverse_rate(double rate)
{
	rate = _to_millimeters(rate);

	if (rate > gm.max_seek_rate) {
		return (TG_MAX_SEEK_RATE_EXCEEDED);
	} else {
		gm.seek_rate = rate; 		// mm per minute
	}
	return (TG_OK);
}

/* 
 * cm_select_plane() - select axis plane 
 *
 * Defaults to XY on erroneous specification
 */

uint8_t cm_select_plane(uint8_t plane) 
{
	if (plane == CANON_PLANE_YZ) {
		gm.plane_axis_0 = Y;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = X;
	} else if (plane == CANON_PLANE_XZ) {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = Y;
	} else {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Y;
		gm.plane_axis_2 = Z;
	}
	return (TG_OK);
}

/* 
 * cm_select_tool() - T parameter
 */

uint8_t cm_select_tool(uint8_t tool)
{
	gm.tool = tool;
	return (TG_OK);
}

/* 
 * cm_change_tool() - M6
 */

uint8_t cm_change_tool(uint8_t tool)
{
	return (TG_OK);
}

/* 
 * cm_set_spindle_speed() - S parameter
 */

uint8_t cm_set_spindle_speed(double speed)
{
//	if (speed > gm.max_spindle speed) {
//		return (TG_MAX_SPINDLE_SPEED_EXCEEDED);
//	}
	gm.spindle_speed = speed;
	return (TG_OK);
}

/* 
 * cm_start_spindle_clockwise() - M3
 */

uint8_t cm_start_spindle_clockwise(void)
{
	return (TG_OK);
}

/* 
 * cm_start_spindle_counterclockwise() - M4
 */

uint8_t cm_start_spindle_counterclockwise(void)
{
	return (TG_OK);
}

/* 
 * cm_stop_spindle_turning() - M5
 */

uint8_t cm_stop_spindle_turning(void)
{
	return (TG_OK);
}

/* 
 * cm_start() - (re)enable stepper timers
 */

uint8_t cm_start()
{
	return (TG_OK);
}

/* 
 * cm_stop() - M0, M1
 */

uint8_t cm_stop()
{
	return (TG_OK);
}

/* 
 * cm_return_to_home() - G28
 */

uint8_t cm_return_to_home()
{
	return (TG_OK);
}

/* 
 * cm_set_origin_offsets() - G92
 */

uint8_t cm_set_origin_offsets(double x, double y, double z)
{
	cm_set_position(x, y, z);
	mc_set_position(_to_millimeters(x), _to_millimeters(y), _to_millimeters(z));
	return (TG_OK);
}

/* 
 * cm_use_length_units() - G20, G21
 */

uint8_t cm_use_length_units(uint8_t inches_mode)
{
	gm.inches_mode = inches_mode;
	return (TG_OK);
}

/* 
 * cm_set_distance_mode() - G90, G91
 */

uint8_t cm_set_distance_mode(uint8_t absolute_mode)
{
	gm.absolute_mode = absolute_mode;
	return (TG_OK);
}

/* 
 * cm_dwell() - G4, P parameter (seconds)
 */

uint8_t cm_dwell(double seconds)
{
	gm.dwell_time = seconds;
	mc_dwell(seconds);
	return (TG_OK);
}

uint8_t cm_init_canon()						// init canonical machining functions
{
	return (TG_OK);
}

uint8_t cm_optional_program_stop()			// M1
{
	return (TG_OK);
}

uint8_t cm_program_stop()					// M0
{
	return (TG_OK);
}

uint8_t cm_program_end()					// M2
{
	return (TG_OK);
}


/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _gc_compute_radius_arc() - compute arc center (offset) from radius.
 * _gc_compute_center_arc() - compute arc from I and J (arc center point)
 *
 * Works completely from current state (gm)
 * Note: this is all original grbl code with little modification other
 * 		 that refactoring into smaller routines.
 */

uint8_t cm_arc_feed(uint8_t motion_mode, uint8_t radius_mode)
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = motion_mode;

	// ++++ this is a cheat. The gn values should be passed in, not read directly from gn
	// ++++ fix this
	cm_set_target(gn.target[X], gn.target[Y], gn.target[Z]);
	cm_set_offset(gn.offset[0], gn.offset[1], gn.offset[2]);
	cm_set_radius(gn.radius);

	// execute the move
	if (radius_mode) {
		if ((_gc_compute_radius_arc() != TG_OK)) {
			return (gp.status);
		}
	}
	gp.status = _gc_compute_center_arc();

	// set final position
	if ((gp.status == TG_OK) || (gp.status == TG_EAGAIN)) {
		cm_set_position(gm.target[X], gm.target[Y], gm.target[Z]);
	}
	return (gp.status);
}

/* _gc_compute_radius_arc() - compute arc center (offset) from radius. */

int _gc_compute_radius_arc()
{
	double x;
	double y;
	double h_x2_div_d;

/*  We need to calculate the center of the circle that has the designated 
	radius and passes through both the current position and the target position
		  
	This method calculates the following set of equations where:
	`  [x,y] is the vector from current to target position, 
		d == magnitude of that vector, 
		h == hypotenuse of the triangle formed by the radius of the circle, 
			 the distance to the center of the travel vector. 
		  
	A vector perpendicular to the travel vector [-y,x] is scaled to the length
	of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2]
	to form the new point [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the 
	center of our arc.
          
       d^2 == x^2 + y^2
       h^2 == r^2 - (d/2)^2
       i == x/2 - y/d*h
       j == y/2 + x/d*h
                                                          O <- [i,j]
                                            -  |
                                  r      -     |
                                      -        |
                                   -           | h
                                -              |
                  [0,0] ->  C -----------------+--------------- T  <- [x,y]
                            | <------ d/2 ---->|
                    
       C - Current position
       T - Target position
       O - center of circle that pass through both C and T
       d - distance from C to T
       r - designated radius
       h - distance from center of CT to O
          
	Expanding the equations:

      	d -> sqrt(x^2 + y^2)
        h -> sqrt(4 * r^2 - x^2 - y^2)/2
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
         
	Which can be written:
          
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          
	Which we for size and speed reasons optimize to:

       	h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
      	i = (x - (y * h_x2_div_d))/2
      	j = (y + (x * h_x2_div_d))/2  
	*/
        
	// Calculate the change in position along each selected axis
	x = gm.target[gm.plane_axis_0]-gm.position[gm.plane_axis_0];
	y = gm.target[gm.plane_axis_1]-gm.position[gm.plane_axis_1];

	clear_vector(&gm.offset);
	// == -(h * 2 / d)
	h_x2_div_d = -sqrt(4 * gm.radius*gm.radius - ((x*x) - (y*y))) / hypot(x,y);

	// If r is smaller than d the arc is now traversing the complex plane beyond
	// the reach of any real CNC, and thus - for practical reasons - we will 
	// terminate promptly (well spoken Simen!)
	if(isnan(h_x2_div_d)) { 
		gp.status = TG_FLOATING_POINT_ERROR; 
		return(gp.status); 
	}

	// Invert the sign of h_x2_div_d if circle is counter clockwise 
	// (see sketch below)
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) {
		h_x2_div_d = -h_x2_div_d;
	}

	/*	The counter clockwise circle lies to the left of the target direction. 
		When offset is positive, the left hand circle will be generated - 
		when it is negative the right hand circle is generated.

    
                                     T  <-- Target position
    
                                     ^ 
        Clockwise circles with       |     Clockwise circles with
		this center will have        |     this center will have
        > 180 deg of angular travel  |     < 180 deg of angular travel, 
                          \          |      which is a good thing!
                           \         |         /
    center of arc when  ->  x <----- | -----> x <- center of arc when 
    h_x2_div_d is positive           |             h_x2_div_d is negative
                                     |
    
                                     C  <-- Current position
	*/                

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees
	// of travel" (go figure!), even though it is advised against ever generating
	// such circles in a single line of g-code. By inverting the sign of 
	// h_x2_div_d the center of the circles is placed on the opposite side of 
	// the line of travel and thus we get the unadvisably long arcs as prescribed.
	if (gm.radius < 0) { 
		h_x2_div_d = -h_x2_div_d; 
	}        
        
	// Complete the operation by calculating the actual center of the arc
	gm.offset[gm.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	gm.offset[gm.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	return (gp.status);
} 
    
/*
 * _gc_compute_center_arc() - compute arc from I and J (arc center point)
 */

int _gc_compute_center_arc()
{
	double theta_start;
	double theta_end;
	double angular_travel;
	double radius_tmp;
	double depth;

    /*	This segment sets up an clockwise or counterclockwise arc from the current
		position to the target position around the center designated by the offset
		vector. All theta-values measured in radians of deviance from the positive 
		y-axis. 

                        | <- theta == 0
                      * * *
                    *       *
                  *           *
                  *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
                  *   /
                    C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))
 	*/

	// calculate the theta (angle) of the current point
	theta_start = _theta(-gm.offset[gm.plane_axis_0], -gm.offset[gm.plane_axis_1]);
	if(isnan(theta_start)) { 
		gp.status = TG_ARC_SPECIFICATION_ERROR;
		return(gp.status); 
	}

	// calculate the theta (angle) of the target point
	theta_end = _theta(gm.target[gm.plane_axis_0] 
					- gm.offset[gm.plane_axis_0] 
					- gm.position[gm.plane_axis_0], 
 					  gm.target[gm.plane_axis_1] 
					- gm.offset[gm.plane_axis_1] 
					- gm.position[gm.plane_axis_1]);

	if(isnan(theta_end)) { 
		gp.status = TG_ARC_SPECIFICATION_ERROR; 
		return(gp.status);
	}

	// ensure that the difference is positive so that we have clockwise travel
	if (theta_end < theta_start) {
		theta_end += 2*M_PI;
	}
	angular_travel = theta_end - theta_start;

	// Invert angular motion if the g-code wanted a counterclockwise arc
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) {
		angular_travel = angular_travel - 2*M_PI;
	}

	// Find the radius
	radius_tmp = hypot(gm.offset[gm.plane_axis_0], gm.offset[gm.plane_axis_1]);

	// Calculate the motion along the depth axis of the helix
	depth = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];

	// Trace the arc
	gp.status = mc_arc(theta_start, angular_travel, radius_tmp, depth, 
					   gm.plane_axis_0, gm.plane_axis_1, gm.plane_axis_2, 
        	   		  (gm.inverse_feed_rate_mode) ? gm.inverse_feed_rate : gm.feed_rate, 
					   gm.inverse_feed_rate_mode);

    // Finish off with a line to make sure we arrive exactly where we think we are
	//--> For this to work correctly it must be delivered ONLY after the arc generator 
	// has completed the arc. So the endpoint should be passed to the generator and
	// executed there.
//	gp.status = mc_line(gp.target[X_AXIS], gp.target[Y_AXIS], gp.target[Z_AXIS], 
//					   (gp.inverse_feed_rate_mode) ? gp.inverse_feed_rate : 
//						gp.feed_rate, gp.inverse_feed_rate_mode);
	return (gp.status);
}


