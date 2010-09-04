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

/* data structures */
static struct GCodeParser gp;		// gcode parser variables
static struct GCodeModel gm;		// gcode model - current state
static struct GCodeModel gn;		// gcode model - parameters for next state
static struct GCodeModel gf;		// gcode model - flags changed values

/* local helper functions and macros */
static void _gc_normalize_gcode_block(char *block);
static int _gc_parse_gcode_block(char *line);	// Parse the block into structs
static int _gc_execute_gcode_block(void);		// Execute the gcode block
static int _gc_read_double(char *buf, int *i, double *double_ptr);
static int _gc_next_statement(char *letter, double *value_ptr, double *fraction_ptr, char *line, int *i);
static int _gc_compute_arc(void);
static int _gc_compute_radius_arc(void);
static int _gc_compute_center_arc(void);
static double _theta(double x, double y);

#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#define SET_NEXT_STATE(a,v) ({gn.a=v; gf.a=1; break;})
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
	gm.seek_rate = cfg.max_seek_rate;		// units/minute
	gm.max_seek_rate = cfg.max_seek_rate;	// units/minute
	gm.max_feed_rate = cfg.max_feed_rate;	// units/minute
	gm.inches_mode = TRUE;					// default to inches (G20)
	gm.absolute_mode = TRUE;				// default to absolute mode (G90)
	gm.inverse_feed_rate = -1; 				// negative inverse_feed_rate means
											//...no inverse_feed_rate specified
}

/*
 * gc_send_to_parser() - send a block of gcode to the parser
 *
 *	Inject a block into parser taking gcode command processing state into account
 */
/*
void gc_send_to_parser(char *block)
{
	gc_gcode_parser(block);
	return;
}
*/

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
			printf_P(PSTR("%s\n"),(comment+3)); // canonical machining func
		}
	}
}

/*
 * to_millimeters()
 */

static inline double to_millimeters(double value) 	// inline won't compile at -O0
//float to_millimeters(double value) 
{
	return(gn.inches_mode ? (value * MM_PER_INCH) : value);
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
					   char *buf, int *i) {
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

int _gc_read_double(char *buf, int *i, double *double_ptr) 
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
 */

int _gc_parse_gcode_block(char *buf) 
{
	int i = 0;  				// index into Gcode block buffer (buf)
  
	ZERO_MODEL_STATE(&gn);		// clear all next-state values
	ZERO_MODEL_STATE(&gf);		// clear all next-state flags
	gp.status = TG_OK;

  	// extract commands and parameters
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
			case 'I': SET_NEXT_STATE(offset[0], gp.value);
			case 'J': SET_NEXT_STATE(offset[1], gp.value);
			case 'K': SET_NEXT_STATE(offset[2], gp.value);
			case 'P': SET_NEXT_STATE(dwell_time, gp.value);
			case 'R': SET_NEXT_STATE(radius, gp.value); 
					  SET_NEXT_STATE(radius_mode, TRUE); 	// +++ FIX THIS
			case 'S': SET_NEXT_STATE(spindle_speed, gp.value); 
			case 'X': SET_NEXT_STATE(target[X], gp.value);
			case 'Y': SET_NEXT_STATE(target[Y], gp.value);
			case 'Z': SET_NEXT_STATE(target[Z], gp.value);
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
 *  Conditionally (based on gf) call the canonical machining functions in 
 *	order of execution as per RS274NGC_3 table 8 (below, with modifications):
 *
 *		1. comment (includes message) [handled during block normalization]
 *		2. set feed rate mode (G93, G94 - inverse time or per minute). 
 *		3. set feed rate (F). 
 *		4. set spindle speed (S). 
 *		5. select tool (T). 
 *		6. change tool (M6). 
 *		7. spindle on or off (M3, M4, M5). 
 *		8. coolant on or off (M7, M8, M9). 
 *		9. enable or disable overrides (M48, M49). 
 *		10. dwell (G4). 
 *		11. set active plane (G17, G18, G19). 
 *		12. set length units (G20, G21). 
 *		13. cutter radius compensation on or off (G40, G41, G42) 
 *		14. cutter length compensation on or off (G43, G49) 
 *		15. coordinate system selection (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3) 
 *		16. set path control mode (G61, G61.1, G64) 
 *		17. set distance mode (G90, G91). 
 *		18. set retract mode (G98, G99). 
 *		19. home (G28, G30) or
 *				change coordinate system data (G10) or
 *				set axis offsets (G92, G92.1, G92.2, G94)
 *		20. perform motion (G0 to G3, G80-G89) as modified (possibly) by G53
 *		21. stop (M0, M1, M2, M30, M60).
 *
 *	Unit conversions should occur (as needed) on the inputs to the canonical 
 *	functions.
 */

int _gc_execute_gcode_block() 
{
	if (gf.inverse_feed_rate_mode) {
		cm_set_inverse_feed_rate_mode(gn.inverse_feed_rate_mode);
	}
	if (gf.feed_rate) {
		if ((gp.status = cm_set_feed_rate(to_millimeters(gn.feed_rate)))) {
			return (gp.status);				// error return
		}
	}
	if (gf.spindle_speed) {
		if ((gp.status = cm_set_spindle_speed(gn.spindle_speed))) {
			return (gp.status);
		}
	}
	if (gf.tool) {
		cm_select_tool(gn.tool);
	}
	if (gf.change_tool) {
		cm_change_tool();
	}
	if (gf.spindle_mode) {
    	if (gn.spindle_mode == SPINDLE_CW) {
			cm_start_spindle_clockwise();
		} else if (gn.spindle_mode == SPINDLE_CCW) {
			cm_start_spindle_counterclockwise();
		} else {
			cm_stop_spindle_turning();
		}
	}
 	// coolant goes here
	// overrides go here
	if (gn.next_action == NEXT_ACTION_DWELL) {
		gm.dwell_time = gn.dwell_time;
		gp.status = mc_dwell(gm.dwell_time); 
	}


 
/*
			case 'X': case 'Y': case 'Z': {
				if (gn.set_origin_mode) {
					gp.position[gp.letter - 'X'] = gp.unit_converted_value;
					gp.target[gp.letter - 'X'] = gp.position[gp.letter - 'X'];
//					memcpy(gp.target, gp.position, sizeof(gp.target));	// target = position
					gn.next_action = NEXT_ACTION_OFFSET_COORDINATES;
				} else if (gp.absolute_mode || gp.absolute_override) {
					gp.target[gp.letter - 'X'] = gp.unit_converted_value;
				} else {
					gp.target[gp.letter - 'X'] += gp.unit_converted_value;
				}
 				break;
			}
		}	
	}

  // Perform any physical actions
  //	memcpy(gm.target, gm.position, sizeof(gm.target)); // target = position

	switch (gp.next_action) {
		case NEXT_ACTION_NONE: {				// nothing to do here
			break;
		}

		case NEXT_ACTION_GO_HOME: { 
			gp.status = mc_home(); 
			break;
		}

		case NEXT_ACTION_OFFSET_COORDINATES: { 
			gp.status = mc_set_position(gp.position[X], gp.position[Y], gp.position[Z]); 
			break;
		}

		case NEXT_ACTION_MOTION: {
			switch (gp.motion_mode) {
				case MOTION_MODE_CANCEL_MOTION_MODE: {
					break;
				}

				case MOTION_MODE_STRAIGHT_TRAVERSE: case MOTION_MODE_STRAIGHT_FEED: {
					gp.status = mc_line(gp.target[X], gp.target[Y], gp.target[Z],
							    	   (gp.inverse_feed_rate_mode) ? gp.inverse_feed_rate : gp.feed_rate,
										gp.inverse_feed_rate_mode); 
					break;
				}

				case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC: {
					gp.status = _gc_compute_arc(); 
					break;
				}
			}
		}
	}
	if ((gp.status == TG_OK) || (gp.status == TG_EAGAIN)) {
		memcpy(gp.position, gp.target, sizeof(gp.target));
	};
*/
	return(gp.status);
}


/*****************************************************************************
 *
 * CANONICAL MACHINING FUNCTIONS
 *
 *	Arguments are assumed to be unit-converted (to millimeters) as appropriate
 *	All operations occur on gm (current model state)
 *
 ****************************************************************************/

/*
 * cm_set_inverse_feed_rate()
 */

inline uint8_t cm_set_inverse_feed_rate_mode(uint8_t mode)  // T=inv feed rate
{
	gm.inverse_feed_rate_mode = mode;
	return (TG_OK);
}

/*
 * cm_set_feed_rate() - F parameter
 */

uint8_t cm_set_feed_rate(double rate) // rate in mm / minute (unit converted)
{
	if (rate > gm.max_feed_rate) {
		return (TG_MAX_FEED_RATE_EXCEEDED);
	}
	if (gm.inverse_feed_rate_mode) {
		gm.inverse_feed_rate = rate; // minutes per motion for this motion only
	} else {
		gm.feed_rate = rate; 		// mm per minute
	}
	return (TG_OK);
}

/* 
 * cm_select_plane() - select axis plane 
 */

uint8_t cm_select_plane(uint8_t plane) 
{
	if (plane == CANON_PLANE_XY) {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Y;
		gm.plane_axis_2 = Z;
	} else if (plane == CANON_PLANE_XZ) {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = Y;
	} else {
		gm.plane_axis_0 = Y;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = X;
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

uint8_t cm_change_tool(void)
{
	return (TG_OK);
}

/* 
 * cm_set_spindle_speed() - S parameter
 */

uint8_t cm_set_spindle_speed(double speed)
{
//	if (rate > gm.max_spindle speed) {
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
uint8_t cm_init_canon()						// init canonical machining functions
uint8_t cm_set_origin_offsets(x,y,z)		// supported as limited G92 for zeroing
uint8_t cm_use_length_units(UNITS)			// G20/G21
uint8_t cm_set_traverse_rate(rate)			// (no code, get from config)
uint8_t cm_straight_traverse(x,y,z)			// G0
uint8_t cm_arc_feed()						// G2/G3
uint8_t cm_dwell(seconds)					// G4, P parameter
uint8_t cm_straight_feed()					// G1
uint8_t cm_start_spindle_clockwise()		// M3
uint8_t cm_start_spindle_counterclockwise()	// M4
uint8_t cm_stop_spindle_turning				// M5
uint8_t cm_change_tool()					// M6, T parameter
uint8_t cm_select_tool()					// T parameter
uint8_t cm_comment(char *)					// ignore comments (I do)
uint8_t cm_message(char *)					// send message to console
uint8_t cm_optional_program_stop()			// M1
uint8_t cm_program_stop()					// M0
uint8_t cm_program_end()					// M2
uint8_t cm_stop()							// used by M0,M1
uint8_t cm_start()							// (re)enables stepper timers
uint8_t cm_return_to_home()					// G28 
uint8_t cm_set_distance_mode()				// G90/G91 (absolute/incremental motion)
*/


/*
 * _gc_compute_arc() - arc computation helper routine 
 *
 * Works completely from current state (gm)
 */

int _gc_compute_arc()
{
	if (gn.radius_mode) {
		if ((_gc_compute_radius_arc() != TG_OK)) {
			return (gp.status);
		}
	}
	return (_gc_compute_center_arc());
}

/*
 * _gc_compute_radius_arc()
 *
 * Compute arc center (offset) from radius. Used to prep for computing an center arc
 */

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
 * _gc_compute_center_arc()
 *
 * Compute the arc move given I and J (arc center point - found in offset vector).
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


