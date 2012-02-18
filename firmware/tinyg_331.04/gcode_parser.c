/*
 * gcode_interpreter.c - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010-2011 Alden S. Hart, Jr.
 * Copyright (c) 2009 Simen Svale Skogsrud
 *
 * This interpreter attempts to follow the NIST RS274/NGC 
 * specification as closely as possible with regard to order 
 * of operations and other behaviors.
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>				// needed for memcpy, memset
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "help.h"
#include "xio/xio.h"				// for char definitions

/* local helper functions and macros */
static void _gc_normalize_gcode_block(char *block);
static uint8_t _gc_parse_gcode_block(char *line);	// Parse the block into structs
static uint8_t _gc_execute_gcode_block(void);		// Execute the gcode block
static uint8_t _get_next_statement(char *letter, double *value, char *buf, uint8_t *i);
static uint8_t _point(double value);

#define coord_select ((uint8_t)gn.dwell_time)		// alias for P which is shared by both dwells and G10s
#define SET_NEXT_STATE(a,v) ({gn.a=v; gf.a=1; break;})
#define SET_NON_MODAL(a,v) ({gn.a=v; gf.a=1; break;})
#define SET_MODAL(a,m,f) ({gn.a=f; gf.a=1; gn.next_action=m; gf.next_action=1; break;})
//#define SET_NEXT_ACTION(a,u,v) ({gn.a=v; gf.a=1; gn.next_action=u; gf.next_action=1; break;})
//#define SET_NEXT_ACTION_MOTION(a,v) ({gn.a=v; gf.a=1; gn.next_action=NEXT_ACTION_MOTION; gf.next_action=1; break;})

//#define SET_NEXT_ACTION_OFFSET(a,v,ofs) ({gn.a=v; gf.a=1; gn.next_action=ofs; gf.next_action=1; break;})
//#define SET_NEXT_ACTION_SET_COORD_OFFSET(a,v) ({gn.a=v; gf.a=1; gn.next_action=NEXT_ACTION_SET_COORD_OFFSET; gf.next_action=1; break;})
//#define SET_NEXT_ACTION_SET_ORIGIN_OFFSET(a,v)   ({gn.a=v; gf.a=1; gn.next_action=NEXT_ACTION_SET_ORIGIN_OFFSET; gf.next_action=1; break;})
//#define SET_NEXT_ACTION_RESET_ORIGIN_OFFSET(a,v) ({gn.a=v; gf.a=1; gn.next_action=NEXT_ACTION_RESET_ORIGIN_OFFSET; gf.next_action=1; break;})

#define CALL_CM_FUNC(f,v) if((int)gf.v != 0) { if ((status = f(gn.v)) != TG_OK) { return(status); } }
/* Derived from::
	if ((int)gf.feed_rate != 0) {		// != 0 either means true or non-zero value
		if ((status = cm_set_feed_rate(gn.feed_rate)) != TG_OK) {
			return(status);				// error return
		}
	}
 */

/* 
 * gc_init() 
 */

void gc_init()
{
	return;
}

/*
 * gc_gcode_parser() - parse a block (line) of gcode
 *
 *	Top level of gcode parser. Normalizes block and looks for special cases
 */

uint8_t gc_gcode_parser(char *block)
{
	_gc_normalize_gcode_block(block);		// get block ready for parsing
	if (block[0] == NUL) return (TG_NOOP); 	// ignore comments (stripped)
	return(_gc_parse_gcode_block(block));	// parse block & return status
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

static void _gc_normalize_gcode_block(char *block) 
{
	char c;
	char *comment=0;	// comment pointer - first char past opening paren
	uint8_t i=0; 		// index for incoming characters
	uint8_t j=0;		// index for normalized characters

	if (block[0] == '/') {					// discard deleted blocks
		block[0] = NUL;
		return;
	}
	if (block[0] == '?') {					// trap and return ? command
		return;
	}
	// normalize the command block & mark the comment(if any)
	while ((c = toupper(block[i++])) != NUL) {
		if ((isupper(c)) || (isdigit(c))) {	// capture common chars
		 	block[j++] = c; 
			continue;
		}
		if (c == '(') {						// detect & handle comments
			block[j] = NUL;
			comment = &block[i]; 
			break;
		}
		if (c <= ' ') continue;				// toss controls & whitespace
		if (c == DEL) continue;				// toss DELETE (0x7F)
		if (strchr("!$%,;:?@^_~`\'\"", c))	// toss invalid punctuation
			continue;
		block[j++] = c;
	}
	block[j] = NUL;							// terminate the command
	if (comment != 0) {
		if ((toupper(comment[0]) == 'M') && 
			(toupper(comment[1]) == 'S') &&
			(toupper(comment[2]) == 'G')) {
			i=0;
			while ((c = comment[i++]) != NUL) {// remove trailing parenthesis
				if (c == ')') {
					comment[--i] = NUL;
					break;
				}
			}
			(void)cm_message(comment+3);
		}
	}
	cm.linecount += 1;
}

/* 
 * _gc_next_statement() - parse next statement in a block of Gcode
 *
 *	Parses the next statement and leaves the index on the next character
 *	after the statement.
 */

static uint8_t _get_next_statement(char *letter, double *value, char *buf, uint8_t *i) {
	if (buf[*i] == NUL) { 		// no more statements
		return (TG_COMPLETE);
	}
	*letter = buf[*i];
	if(isupper(*letter) == false) { 
		return (TG_EXPECTED_COMMAND_LETTER);
	}
	(*i)++;
	if (read_double(buf, i, value) == false) {
		return (TG_BAD_NUMBER_FORMAT);
	}
	return (TG_OK);
}

static uint8_t _point(double value) 
{
	return((uint8_t)(value*10 - trunc(value)*10));	// isolate the decimal point as an int
}

/*
 * _gc_parse_gcode_block() - parses one line of NULL terminated G-Code. 
 *
 *	All the parser does is load the state values in gn (next model state),
 *	and flags in gf (model state flags). The execute routine applies them.
 *	The line is assumed to contain only uppercase characters and signed 
 *  floats (no whitespace).
 *
 *	A number of implicit things happen when the gn struct is zeroed:
 *	  - inverse feed rate mode is cancelled - set back to units_per_minute mode
 */

static uint8_t _gc_parse_gcode_block(char *buf) 
{
	uint8_t i=0; 	 			// persistent index into Gcode block buffer (buf)
  	char letter;				// parsed letter, eg.g. G or X or Y
	double value;				// value parsed from letter (e.g. 2 for G2)
//	uint8_t point;				// gcode decimal point value, e.g. 92.1, 92.2...
	uint8_t status = TG_OK;

	memset(&gn, 0, sizeof(gn));	// clear all next-state values
	memset(&gf, 0, sizeof(gf));	// clear all next-state flags

	// pull needed state from gm structure to preset next state
	gn.next_action = cm_get_next_action();	// next action persists
	gn.motion_mode = cm_get_motion_mode();	// motion mode (G modal group 1)
	gn.distance_mode = cm_get_distance_mode();
	cm_set_absolute_override(FALSE);		// must be set per block 

  	// extract commands and parameters
	while((status = _get_next_statement(&letter, &value, buf, &i)) == TG_OK) {
		switch(letter) {
			case 'G':
				switch((uint8_t)value) {
					case 0:  SET_MODAL(motion_mode, NEXT_ACTION_MOTION, MOTION_MODE_STRAIGHT_TRAVERSE);
					case 1:  SET_MODAL(motion_mode, NEXT_ACTION_MOTION, MOTION_MODE_STRAIGHT_FEED);
					case 2:  SET_MODAL(motion_mode, NEXT_ACTION_MOTION, MOTION_MODE_CW_ARC);
					case 3:  SET_MODAL(motion_mode, NEXT_ACTION_MOTION, MOTION_MODE_CCW_ARC);
					case 4:  SET_NEXT_STATE(next_action, NEXT_ACTION_DWELL);
					case 10: SET_NEXT_STATE(set_coord_offset, true);
					case 17: SET_NEXT_STATE(select_plane, CANON_PLANE_XY);
					case 18: SET_NEXT_STATE(select_plane, CANON_PLANE_XZ);
					case 19: SET_NEXT_STATE(select_plane, CANON_PLANE_YZ);
					case 20: SET_NEXT_STATE(units_mode, INCHES_MODE);
					case 21: SET_NEXT_STATE(units_mode, MILLIMETER_MODE);
					case 28: SET_NEXT_STATE(next_action, NEXT_ACTION_RETURN_TO_HOME);
					case 30: SET_NEXT_STATE(next_action, NEXT_ACTION_HOMING_CYCLE);
					case 40: break;	// ignore cancel cutter radius compensation
					case 49: break;	// ignore cancel tool length offset comp.
					case 53: SET_NEXT_STATE(absolute_override, true);
					case 54: SET_NEXT_STATE(coord_system, G54);
					case 55: SET_NEXT_STATE(coord_system, G55);
					case 56: SET_NEXT_STATE(coord_system, G56);
					case 57: SET_NEXT_STATE(coord_system, G57);
					case 58: SET_NEXT_STATE(coord_system, G58);
					case 59: SET_NEXT_STATE(coord_system, G59);
					case 61: {
						switch (_point(value)) {
							case 0: SET_NEXT_STATE(path_control, PATH_EXACT_STOP);
							case 1: SET_NEXT_STATE(path_control, PATH_EXACT_PATH); 
							default: status = TG_UNRECOGNIZED_COMMAND;
						}
						break;
					}
					case 64: SET_NEXT_STATE(path_control, PATH_CONTINUOUS);
					case 80: SET_NEXT_STATE(motion_mode, MOTION_MODE_CANCEL_MOTION_MODE);
					case 90: SET_NEXT_STATE(distance_mode, ABSOLUTE_MODE);
					case 91: SET_NEXT_STATE(distance_mode, INCREMENTAL_MODE);
					case 92: {
						switch (_point(value)) {
							case 0: SET_NON_MODAL(origin_offset_mode, ORIGIN_OFFSET_SET);
							case 1: SET_NON_MODAL(origin_offset_mode, ORIGIN_OFFSET_CANCEL);
							case 2: SET_NON_MODAL(origin_offset_mode, ORIGIN_OFFSET_SUSPEND);
							case 3: SET_NON_MODAL(origin_offset_mode, ORIGIN_OFFSET_RESUME); 
							default: status = TG_UNRECOGNIZED_COMMAND;
						}
						break;
					}
					case 93: SET_NEXT_STATE(inverse_feed_rate_mode, true);
					case 94: SET_NEXT_STATE(inverse_feed_rate_mode, false);
					default: status = TG_UNRECOGNIZED_COMMAND;
				}
				break;

			case 'M':
				switch((uint8_t)value) {
					case 0: case 1: 
							SET_NEXT_STATE(program_flow, PROGRAM_FLOW_PAUSED);
					case 2: case 30: case 60:
							SET_NEXT_STATE(program_flow, PROGRAM_FLOW_COMPLETED);
					case 3: SET_NEXT_STATE(spindle_mode, SPINDLE_CW);
					case 4: SET_NEXT_STATE(spindle_mode, SPINDLE_CCW);
					case 5: SET_NEXT_STATE(spindle_mode, SPINDLE_OFF);
					case 6: SET_NEXT_STATE(change_tool, true);
					case 7: break;	// ignore mist coolant on
					case 8: break;	// ignore flood coolant on
					case 9: break;	// ignore mist and flood coolant off
					case 48: break;	// enable speed and feed overrides
					case 49: break;	// disable speed and feed overrides
					default: status = TG_UNRECOGNIZED_COMMAND;
				}
				break;

			case 'T': SET_NEXT_STATE(tool, (uint8_t)trunc(value));
			case 'F': SET_NEXT_STATE(feed_rate, value);
			case 'P': SET_NEXT_STATE(dwell_time, value); 	// also used as G10 coord system select
			case 'S': SET_NEXT_STATE(spindle_speed, value); 
			case 'X': SET_NEXT_STATE(target[X], value);
			case 'Y': SET_NEXT_STATE(target[Y], value);
			case 'Z': SET_NEXT_STATE(target[Z], value);
			case 'A': SET_NEXT_STATE(target[A], value);
			case 'B': SET_NEXT_STATE(target[B], value);
			case 'C': SET_NEXT_STATE(target[C], value);
		//	case 'U': SET_NEXT_STATE(target[U], value);		// reserved
		//	case 'V': SET_NEXT_STATE(target[V], value);		// reserved
		//	case 'W': SET_NEXT_STATE(target[W], value);		// reserved
			case 'I': SET_NEXT_STATE(arc_offset[0], value);
			case 'J': SET_NEXT_STATE(arc_offset[1], value);
			case 'K': SET_NEXT_STATE(arc_offset[2], value);
			case 'R': SET_NEXT_STATE(arc_radius, value);
			case 'N': cm.linenum = (uint32_t)value; break;	// save line #
			case 'L': break;								// not used for anything
			default: status = TG_UNRECOGNIZED_COMMAND;
		}
		if(status != TG_OK) break;
	}
	// set targets correctly. fill-in any unset target if in absolute mode, 
	// otherwise leave the target values alone
	for (i=0; i<AXES; i++) {
		if (((gn.distance_mode == ABSOLUTE_MODE) || 
			 (gn.absolute_override == true)) && (gf.target[i] < EPSILON)) {		
			gn.target[i] = cm_get_model_position(i); // get target from model
		}
	}
	return (_gc_execute_gcode_block());
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
 *		15. coordinate system selection (G54, G55, G56, G57, G58, G59)
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

static uint8_t _gc_execute_gcode_block() 
{
	uint8_t status = TG_OK;

	CALL_CM_FUNC(cm_set_inverse_feed_rate_mode, inverse_feed_rate_mode);
	CALL_CM_FUNC(cm_set_feed_rate, feed_rate);
	CALL_CM_FUNC(cm_set_spindle_speed, spindle_speed);
	CALL_CM_FUNC(cm_select_tool, tool);
	CALL_CM_FUNC(cm_change_tool, tool);
	CALL_CM_FUNC(cm_spindle_control, spindle_mode); 	// spindle on or off
	//--> coolant on or off goes here
	//--> enable or disable overrides goes here

	if (gn.next_action == NEXT_ACTION_DWELL) { 			// G4 - dwell
		ritorno(cm_dwell(gn.dwell_time));
	}
	CALL_CM_FUNC(cm_select_plane, select_plane);
	CALL_CM_FUNC(cm_set_units_mode, units_mode);
	//--> cutter radius compensation goes here
	//--> cutter length compensation goes here
	CALL_CM_FUNC(cm_set_coord_system, coord_system);
	CALL_CM_FUNC(cm_set_path_control, path_control);
	CALL_CM_FUNC(cm_set_distance_mode, distance_mode);
	//--> set retract mode goes here

	if (gn.next_action == NEXT_ACTION_RETURN_TO_HOME) {	// G28 - return to zero
		return(cm_return_to_home());
	}
	if (gn.next_action == NEXT_ACTION_HOMING_CYCLE) {	// G30 - initiate a homing cycle
		return(cm_homing_cycle());
	}
	if (gn.set_coord_offset == true) {					// G10 - set coordinate offsets
		cm_set_coord_offsets(coord_select, gn.target, gf.target);
	}
	if ((uint8_t)gf.origin_offset_mode != 0) {			// G92's - set/cancel/suspend/resume axis offsets
		ritorno(cm_set_origin_offsets(gn.origin_offset_mode, gn.target, gf.target));
	}
	if (gn.next_action == NEXT_ACTION_MOTION) {
		if (gn.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) {	// G0 - traverse
			return (cm_straight_traverse(gn.target));
		}
		if (gn.motion_mode == MOTION_MODE_STRAIGHT_FEED) {		// G1 - linear feed
			return (cm_straight_feed(gn.target));
		}
		// G2 or G3 (arc motion command)
		if ((gn.motion_mode == MOTION_MODE_CW_ARC) || (gn.motion_mode == MOTION_MODE_CCW_ARC)) {
			// gf.radius sets radius mode if radius was collected in gn
			return (cm_arc_feed(gn.target, gn.arc_offset[0], gn.arc_offset[1], gn.arc_offset[2], 
								gn.arc_radius, gn.motion_mode));
		}
	}
	return (status);
}
