/*
 * gcode_interpreter.c - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010-2012 Alden S. Hart, Jr.
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
static void _normalize_gcode_block(char *block);
static uint8_t _parse_gcode_block(char *line);	// Parse the block into structs
static uint8_t _execute_gcode_block(void);		// Execute the gcode block
static uint8_t _check_gcode_block(void);		// check the block for correctness
static uint8_t _get_next_statement(char *letter, double *value, char *buf, uint8_t *i);
static uint8_t _point(double value);
static uint8_t _axis_changed(void);

#define coord_select ((uint8_t)gn.dwell_time)	// alias for P which is shared by both dwells and G10s

#define SET_PARAMETER(parm,val) ({gn.parm=val; gf.parm=1; break;})
#define SET_NEXT_ACTION(action) ({gn.next_action=action; gf.next_action=1; break;})
#define SET_MOTION_MODE(motion) ({gn.motion_mode=motion; gf.motion_mode=1; break;})
#define EXEC_FUNC(f,v) if((uint8_t)gf.v != false) { status = f(gn.v);}

//#define CALL_FUNC(f,v) if((int)gf.v != 0) { if ((status = f(gn.v)) != TG_OK) { return(status); } }
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
	_normalize_gcode_block(block);			// get block ready for parsing
	if (block[0] == NUL) return (TG_NOOP); 	// ignore comments (stripped)
	return(_parse_gcode_block(block));		// parse block & return status
}

/*
 * _normalize_gcode_block() - normalize a block (line) of gcode in place
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

static void _normalize_gcode_block(char *block) 
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
}

/*
 * _parse_gcode_block() - parses one line of NULL terminated G-Code. 
 *
 *	All the parser does is load the state values in gn (next model state),
 *	and flags in gf (model state flags). The execute routine applies them.
 *	The line is assumed to contain only uppercase characters and signed 
 *  floats (no whitespace).
 *
 *	A number of implicit things happen when the gn struct is zeroed:
 *	  - inverse feed rate mode is cancelled - set back to units_per_minute mode
 */

static uint8_t _parse_gcode_block(char *buf) 
{
	uint8_t i=0; 	 			// persistent index into Gcode block buffer (buf)
  	char letter;				// parsed letter, eg.g. G or X or Y
	double value;				// value parsed from letter (e.g. 2 for G2)
	uint8_t status = TG_OK;

	// set initial state for new move 
	memset(&gf, 0, sizeof(gf));	// clear all next-state flags
	memset(&gn, 0, sizeof(gn));	// clear all next-state values
	gn.motion_mode = cm_get_motion_mode();	// motion mode persists from previous block

  	// extract commands and parameters
	while((status = _get_next_statement(&letter, &value, buf, &i)) == TG_OK) {

		switch(letter) {
			case 'G':
			/*
				switch(value) {
					case 4: case 10: case 28: case 30: case 53: case 92: group_number = MODAL_GROUP_0; break;
					case 0: case 1: case 2: case 3: case 80: group_number = MODAL_GROUP_1; break;
					case 17: case 18: case 19: group_number = MODAL_GROUP_2; break;
					case 90: case 91: group_number = MODAL_GROUP_3; break;
					case 93: case 94: group_number = MODAL_GROUP_5; break;
					case 20: case 21: group_number = MODAL_GROUP_6; break;
					case 54: case 55: case 56: case 57: case 58: case 59: group_number = MODAL_GROUP_12; break;
				}
			*/
				switch((uint8_t)value) {
					case 0:  SET_MOTION_MODE(MOTION_MODE_STRAIGHT_TRAVERSE);
					case 1:  SET_MOTION_MODE(MOTION_MODE_STRAIGHT_FEED);
					case 2:  SET_MOTION_MODE(MOTION_MODE_CW_ARC);
					case 3:  SET_MOTION_MODE(MOTION_MODE_CCW_ARC);
					case 4:  SET_NEXT_ACTION(NEXT_ACTION_DWELL);
					case 10: SET_PARAMETER(set_coord_offset, true);
					case 17: SET_PARAMETER(select_plane, CANON_PLANE_XY);
					case 18: SET_PARAMETER(select_plane, CANON_PLANE_XZ);
					case 19: SET_PARAMETER(select_plane, CANON_PLANE_YZ);
					case 20: SET_PARAMETER(units_mode, INCHES);
					case 21: SET_PARAMETER(units_mode, MILLIMETERS);
					case 28: {
						switch (_point(value)) {
							case 0: SET_NEXT_ACTION(NEXT_ACTION_GO_HOME);
							case 1: SET_NEXT_ACTION(NEXT_ACTION_SEARCH_HOME); 
							default: status = TG_UNRECOGNIZED_COMMAND;
						}
						break;
					}
					case 40: break;	// ignore cancel cutter radius compensation
					case 49: break;	// ignore cancel tool length offset comp.
					case 53: SET_PARAMETER(absolute_override, true);
					case 54: SET_PARAMETER(coord_system, G54);
					case 55: SET_PARAMETER(coord_system, G55);
					case 56: SET_PARAMETER(coord_system, G56);
					case 57: SET_PARAMETER(coord_system, G57);
					case 58: SET_PARAMETER(coord_system, G58);
					case 59: SET_PARAMETER(coord_system, G59);
					case 61: {
						switch (_point(value)) {
							case 0: SET_PARAMETER(path_control, PATH_EXACT_PATH);
							case 1: SET_PARAMETER(path_control, PATH_EXACT_STOP); 
							default: status = TG_UNRECOGNIZED_COMMAND;
						}
						break;
					}
					case 64: SET_PARAMETER(path_control, PATH_CONTINUOUS);
					case 80: SET_MOTION_MODE(MOTION_MODE_CANCEL_MOTION_MODE);
					case 90: SET_PARAMETER(distance_mode, ABSOLUTE_MODE);
					case 91: SET_PARAMETER(distance_mode, INCREMENTAL_MODE);
					case 92: {
						switch (_point(value)) {
							case 0: SET_NEXT_ACTION(NEXT_ACTION_SET_ORIGIN_OFFSETS);
							case 1: SET_NEXT_ACTION(NEXT_ACTION_RESET_ORIGIN_OFFSETS);
							case 2: SET_NEXT_ACTION(NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS);
							case 3: SET_NEXT_ACTION(NEXT_ACTION_RESUME_ORIGIN_OFFSETS); 
							default: status = TG_UNRECOGNIZED_COMMAND;
						}
						break;
					}
					case 93: SET_PARAMETER(inverse_feed_rate_mode, true);
					case 94: SET_PARAMETER(inverse_feed_rate_mode, false);
					default: status = TG_UNRECOGNIZED_COMMAND;
				}
				break;

			case 'M':
			/*
				switch(int_value) {
					case 0: case 1: case 2: case 30: group_number = MODAL_GROUP_4; break;
					case 3: case 4: case 5: group_number = MODAL_GROUP_7; break;
				}
			*/
				switch((uint8_t)value) {
					case 0: case 1: 
							SET_PARAMETER(program_flow, PROGRAM_FLOW_PAUSED);
					case 2: case 30: case 60:
							SET_PARAMETER(program_flow, PROGRAM_FLOW_COMPLETED);
					case 3: SET_PARAMETER(spindle_mode, SPINDLE_CW);
					case 4: SET_PARAMETER(spindle_mode, SPINDLE_CCW);
					case 5: SET_PARAMETER(spindle_mode, SPINDLE_OFF);
					case 6: SET_PARAMETER(change_tool, true);
					case 7: break;	// ignore mist coolant on
					case 8: break;	// ignore flood coolant on
					case 9: break;	// ignore mist and flood coolant off
					case 48: break;	// enable speed and feed overrides
					case 49: break;	// disable speed and feed overrides
					default: status = TG_UNRECOGNIZED_COMMAND;
				}
				break;

			case 'T': SET_PARAMETER(tool, (uint8_t)trunc(value));
			case 'F': SET_PARAMETER(feed_rate, value);
			case 'P': SET_PARAMETER(dwell_time, value); 	// also used as G10 coord system select
			case 'S': SET_PARAMETER(spindle_speed, value); 
			case 'X': SET_PARAMETER(target[X], value);
			case 'Y': SET_PARAMETER(target[Y], value);
			case 'Z': SET_PARAMETER(target[Z], value);
			case 'A': SET_PARAMETER(target[A], value);
			case 'B': SET_PARAMETER(target[B], value);
			case 'C': SET_PARAMETER(target[C], value);
		//	case 'U': SET_PARAMETER(target[U], value);		// reserved
		//	case 'V': SET_PARAMETER(target[V], value);		// reserved
		//	case 'W': SET_PARAMETER(target[W], value);		// reserved
			case 'I': SET_PARAMETER(arc_offset[0], value);
			case 'J': SET_PARAMETER(arc_offset[1], value);
			case 'K': SET_PARAMETER(arc_offset[2], value);
			case 'R': SET_PARAMETER(arc_radius, value);
			case 'N': SET_PARAMETER(linenum,(uint32_t)value);// line number
			case 'L': break;								// not used for anything
			default: status = TG_UNRECOGNIZED_COMMAND;
		}
		if(status != TG_OK) break;
	}

	// set targets correctly. fill-in any unset target if in absolute mode, 
	// otherwise leave the target values alone
	for (i=0; i<AXES; i++) {
		if ((gn.distance_mode == ABSOLUTE_MODE) && (gf.target[i] < EPSILON)) {		
			gn.target[i] = cm_get_model_work_position(i); // get target from model
		}
	}
	ritorno(_check_gcode_block());			// perform error checking
	return (_execute_gcode_block());		// otherwise execute the block
}

/*
 * _execute_gcode_block() - execute parsed block
 *
 *  Conditionally (based on whether a flag is set in gf) call the canonical 
 *	machining functions in order of execution as per RS274NGC_3 table 8 
 *  (below, with modifications):
 *
 *	    0. apply the line number or auto-increment if there are none
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
 *		19c. set axis offsets (G92, G92.1, G92.2, G92.3)
 *		20. perform motion (G0 to G3, G80-G89) as modified (possibly) by G53
 *		21. stop (M0, M1, M2, M30, M60)
 *
 *	Values in gn are in original units and should not be unit converted prior 
 *	to calling the canonical functions (which do the unit conversions)
 */

static uint8_t _execute_gcode_block()
{
	uint8_t status = TG_OK;

	cm_set_linenum(gn.linenum);
	EXEC_FUNC(cm_set_inverse_feed_rate_mode, inverse_feed_rate_mode);
	EXEC_FUNC(cm_set_feed_rate, feed_rate);
	EXEC_FUNC(cm_set_spindle_speed, spindle_speed);
	EXEC_FUNC(cm_select_tool, tool);
	EXEC_FUNC(cm_change_tool, tool);
	EXEC_FUNC(cm_spindle_control, spindle_mode); 	// spindle on or off
	//--> coolant on or off goes here
	//--> enable or disable overrides goes here

	if (gn.next_action == NEXT_ACTION_DWELL) { 		// G4 - dwell
		ritorno(cm_dwell(gn.dwell_time));			// return if error, otherwise complete the block
	}
	EXEC_FUNC(cm_select_plane, select_plane);
	EXEC_FUNC(cm_set_units_mode, units_mode);
	//--> cutter radius compensation goes here
	//--> cutter length compensation goes here
	EXEC_FUNC(cm_set_coord_system, coord_system);
	EXEC_FUNC(cm_set_path_control, path_control);
	EXEC_FUNC(cm_set_distance_mode, distance_mode);
	//--> set retract mode goes here

	switch (gn.next_action) {
		case NEXT_ACTION_GO_HOME: { status = cm_return_to_home(); break;}
		case NEXT_ACTION_SEARCH_HOME: { status = cm_homing_cycle(); break;}
		case NEXT_ACTION_SET_COORD_DATA: { status = cm_set_coord_offsets(coord_select, gn.target, gf.target); break;}

		case NEXT_ACTION_SET_ORIGIN_OFFSETS: { status = cm_set_origin_offsets(gn.target, gf.target); break;}
		case NEXT_ACTION_RESET_ORIGIN_OFFSETS: { status = cm_reset_origin_offsets(); break;}
		case NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS: { status = cm_suspend_origin_offsets(); break;}
		case NEXT_ACTION_RESUME_ORIGIN_OFFSETS: { status = cm_resume_origin_offsets(); break;}

		case NEXT_ACTION_DEFAULT: { 
			if (_axis_changed() == false) break;
			cm_set_absolute_override(gn.absolute_override);	// apply override setting to gm struct
			switch (gn.motion_mode) {
				case MOTION_MODE_STRAIGHT_TRAVERSE: { status = cm_straight_traverse(gn.target, gf.target); break;}
				case MOTION_MODE_STRAIGHT_FEED: { status = cm_straight_feed(gn.target, gf.target); break;}
				case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
					// gf.radius sets radius mode if radius was collected in gn
					{ status = cm_arc_feed(gn.target, gf.target, gn.arc_offset[0], gn.arc_offset[1],
								gn.arc_offset[2], gn.arc_radius, gn.motion_mode); break;}
			}
			cm_set_absolute_override(false);		// now un-set it (for reporting purposes) 
		}
	}
	if (gf.program_flow == true) {
		// do the M stops: M0, M1, M2, M30, M60
	}
	return (status);
}

/*
 * _check_gcode_block() - return a TG_ error if an error is detected
 */

static uint8_t _check_gcode_block()
{
/*
    if (group_number) {
      if ( bit_istrue(modal_group_words,bit(group_number)) ) {
        FAIL(STATUS_MODAL_GROUP_VIOLATION);
      } else {
        bit_true(modal_group_words,bit(group_number));
      }
      group_number = MODAL_GROUP_NONE; // Reset for next command.
    }
*/
	return (TG_OK);
}

/*
 * helpers
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
	return (TG_OK);		// leave the index on the next character after the statement
}

static uint8_t _point(double value) 
{
	return((uint8_t)(value*10 - trunc(value)*10));	// isolate the decimal point as an int
}

static uint8_t _axis_changed()
{
	return (gf.target[X] + gf.target[Y] + gf.target[Z] + gf.target[A] + gf.target[B] + gf.target[C]);
}

