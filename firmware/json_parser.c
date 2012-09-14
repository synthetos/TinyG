/*
 * json_parser.c - JSON parser for rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2012 Alden S. Hart, Jr.
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
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-JSON
 */

#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>					// needed for memcpy, memset
#include <stdio.h>					// precursor for xio.h
#include <avr/pgmspace.h>			// precursor for xio.h

#include "tinyg.h"
#include "config.h"					// JSON sits on top of the config system
#include "controller.h"
#include "json_parser.h"
#include "report.h"
#include "util.h"
#include "xio/xio.h"				// for char definitions

// local scope stuff

static uint8_t _json_parser(char *str);
static uint8_t _get_nv_pair(cmdObj *cmd, char **pstr, const char *group, int8_t *depth);
static uint8_t _normalize_json_string(char *str, uint16_t size);

/****************************************************************************
 * js_init() 
 */

void js_init() 
{
	return;
}

/****************************************************************************
 * js_json_parser() - parse a JSON string
 * _json_parser()   - inner loop so any errors retrun back to the main routine
 *
 *	This is a dumbed down JSON parser to fit in limited memory with no malloc
 *	or practical way to do recursion ("depth" tracks parent/child levels).
 *
 *	This function will parse the following forms up to the JSON_MAX limits:
 *	  {"name":"value"}
 *	  {"name":12345}
 *	  {"name1":"value1", "n2":"v2", ... "nN":"vN"}
 *	  {"parent_name":{"name":"value"}}
 *	  {"parent_name":{"name1":"value1", "n2":"v2", ... "nN":"vN"}}
 *
 *	  "value" can be a string, number, true, false, or null (2 types)
 *
 *	Numbers
 *	  - number values are not quoted and can start with a digit or -. 
 *	  - numbers cannot start with + or . (period)
 *	  - exponentiated numbers are handled OK. 
 *	  - hexadecimal or other non-decimal number bases are not supported
 *
 *	The parser:
 *	  - extracts an array of one or more JSON object structs from the input string
 *	  - once the array is built it executes the object(s) in order in the array
 *	  - passes the executed array to the response handler to generate the response string
 *	  - returns the status and the JSON response string
 */

uint8_t js_json_parser(char *in_str, char *out_str)
{
	uint8_t status = _json_parser(in_str);		// parse and execute the input
	cmd_print_list(status, TEXT_INLINE_PAIRS);	// echo result as JSON
	return (status);
}

static uint8_t _json_parser(char *str)
{
	uint8_t status;
	int8_t depth = 2;							// starting body depth is 2
	cmdObj *cmd = cmd_body;						// point at first struct in the body
	char group[CMD_GROUP_LEN+1] = {""};			// group identifier - starts as NUL
	int8_t i = CMD_BODY_LEN;

	// parse the JSON command into the cmd body
	ritorno(_normalize_json_string(str, JSON_OUTPUT_STRING_MAX));	// return if error
	do {
		if (--i == 0) { return (TG_JSON_TOO_MANY_PAIRS); }			// length error
		if ((status = _get_nv_pair(cmd, &str, group, &depth)) > TG_EAGAIN) { // erred out
			return (status);
		}
		strncpy(group, cmd->group, CMD_GROUP_LEN);// propagate the group ID from previous obj
		cmd = cmd->nx;
	} while (status != TG_OK);					// breaks when parsing is complete

	// execute the command - iterate through all commands in the body
	cmd = cmd_body;
	if (cmd->type == TYPE_NULL){				// means GET the value
		ritorno(cmd_get(cmd));					// ritorno returns w/status on any errors
	} else {
		ritorno(cmd_set(cmd));					// set value or call a function (e.g. gcode)
		cmd_persist(cmd);
	}
	return (TG_OK);								// only successful commands exit through this point
}

/*
 * _normalize_json_string - normalize a JSON string in place
 *
 *	Validate string size limits, remove all whitespace and convert 
 *	to lower case, with the exception of gcode comments
 */

static uint8_t _normalize_json_string(char *str, uint16_t size)
{
	char *wr;								// write pointer
	uint8_t in_comment = false;

	if (strlen(str) > size) return (TG_INPUT_EXCEEDS_MAX_LENGTH);

	for (wr = str; *str != NUL; str++) {
		if (!in_comment) {					// normal processing
			if (*str == '(') in_comment = true;
			if ((*str <= ' ') || (*str == DEL)) continue; // toss ctrls, WS & DEL
			*wr++ = tolower(*str);
		} else {							// Gcode comment processing	
			if (*str == ')') in_comment = false;
			*wr++ = *str;
		}
	}
	*wr = NUL;
	return (TG_OK);
}

/*
 * _get_nv_pair() - get the next name-value pair
 *
 *	Parse the next statement and populate the command object (cmdObj).
 *
 *	Leaves string pointer (str) on the first character following the object.
 *	Which is the character just past the ',' separator if it's a multi-valued 
 *	object or the terminating NUL if single object or the last in a multi.
 *
 *	Keeps track of tree depth and closing braces as much as it has to.
 *	If this were to be extended to track multiple parents or more than two
 *	levels deep it would have to track closing curlies - which it does not.
 *
 *	ASSUMES INPUT STRING HAS FIRST BEEN NORMALIZED BY _normalize_json_string()
 *
 *	If a group prefix is passed in it will be pre-pended to any name parsed
 *	to form a token string. For example, if "x" is provided as a group and 
 *	"fr" is found in the name string the parser will search for "xfr"in the 
 *	cfgArray.
 */

static uint8_t _get_nv_pair(cmdObj *cmd, char **pstr, const char *group, int8_t *depth)
{
	char *tmp;
	char terminators[] = {"},"};

	cmd_clear(cmd);								// wipe the object and set the depth
//	cmd->depth = *depth;						// tree depth. 0 = root

	// process name element
	// find leading and trailing name quotes and set pointers accordingly
	// accommodate groups by looking up index by full name but stripping group from token
	if ((*pstr = strchr(*pstr, '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	if ((tmp = strchr(++(*pstr), '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	*tmp = NUL;
//	strncpy(cmd->friendly_name, *pstr, CMD_STRING_LEN);// copy name from string
	strncpy(cmd->friendly_name, group, CMD_GROUP_LEN);  // prepend the group or noop if no group
	strncpy(&cmd->friendly_name[strlen(group)], *pstr, CMD_STRING_LEN); // cat name to the group prefix
	if ((cmd->index = cmd_get_index(cmd->friendly_name)) == -1) { return (TG_UNRECOGNIZED_COMMAND);}
	cmd_get_token(cmd->index, cmd->token);
	if (group[0] != NUL) {						// strip group prefix if this is a group
		strncpy(cmd->token, &cmd->friendly_name[strlen(group)], CMD_TOKEN_LEN);
		strncpy(cmd->group, group, CMD_GROUP_LEN);// propagate group token to this child
	}
	*pstr = ++tmp;

	// process value element
	if ((*pstr = strchr(*pstr, ':')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	(*pstr)++;									// advance to start of value field
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { 
		cmd->type = TYPE_NULL;
		cmd->value = TYPE_NULL;
		if (cmd_is_group(cmd->token) == true) { 
			strncpy(cmd->group, cmd->token, CMD_GROUP_LEN);// record the group token
		}
	} else if (**pstr == 'f') { 
		cmd->type = TYPE_FALSE;
		cmd->value = false;						// (technically not necessary due to the init)
	} else if (**pstr == 't') { 
		cmd->type = TYPE_TRUE;
		cmd->value = true;
	} else if (isdigit(**pstr) || (**pstr == '-')) { // value is a number
		cmd->value = strtod(*pstr, &tmp);		// tmp is the end pointer
		if(tmp == *pstr) return (TG_BAD_NUMBER_FORMAT);
		cmd->type = TYPE_FLOAT;
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;
		if ((tmp = strchr(*pstr, '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR); // find the end of the string
		*tmp = NUL;
		if (strlen(*pstr) >= CMD_STRING_LEN) return (TG_INPUT_EXCEEDS_MAX_LENGTH);
		strncpy(cmd->string, *pstr, CMD_STRING_LEN);
		cmd->type = TYPE_STRING;
		*pstr = ++tmp;
	} else if (**pstr == '{') { cmd->type = TYPE_PARENT;
		strncpy(cmd->group, cmd->token, CMD_GROUP_LEN);// record the group token
//		*depth += 1;							// will set the next object down one level
		(*pstr)++;
		return(TG_EAGAIN);						// signal that there is more to parse
	} else {
		 return (TG_JSON_SYNTAX_ERROR);			// ill-formed JSON
	}

	// process pair transitions and end conditions
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) { // advance to terminator or err out
		return (TG_JSON_SYNTAX_ERROR);
	}
	if (**pstr == '}') { 
		*depth -= 1;							// pop up a nesting level
		(*pstr)++;								// advance to comma or whatever follows
	}
	if (**pstr == ',') { 
		return (TG_EAGAIN);						// signal that there is more to parse
	}
	(*pstr)++;
	return (TG_OK);								// signal that parsing is complete
}

/****************************************************************************
 * js_serialize_json() - make a JSON object string from JSON object array
 *
 *	*cmd is a pointer to the first element in the cmd list to serialize
 *	*str is a pointer to the output string - usually what was the input string
 *	Returns the character count of the resulting string
 */

uint16_t js_serialize_json(char *out_buf)
{
	cmdObj *cmd = cmd_header;
	char *str = out_buf;						// set workikng string pointer 
	int8_t depth = 0;

	strcpy(str++, "{"); 						// write opening curly
	do {	// serialize the current element (assumes the first element is not empty)
		str += sprintf(str, "\"%s\":", cmd->token);

		if (cmd->type == TYPE_PARENT) {
			str += sprintf(str, "{");
			cmd = cmd->nx;
			depth = cmd->depth;
			continue;
		} else if (cmd->type == TYPE_END)	 { str += sprintf(str, "\"\"");
		} else if (cmd->type == TYPE_NULL)	 { str += sprintf(str, "\"\"");
		} else if (cmd->type == TYPE_FALSE)	 { str += sprintf(str, "false");
		} else if (cmd->type == TYPE_TRUE)	 { str += sprintf(str, "true");
		} else if (cmd->type == TYPE_INTEGER){ str += sprintf(str, "%1.0f", cmd->value);
		} else if (cmd->type == TYPE_FLOAT)	 { str += sprintf(str, "%0.3f", (double)cmd->value);
		} else if (cmd->type == TYPE_STRING) { str += sprintf(str, "\"%s\"", cmd->string);
		} 
		do {  // advance to the next non-empty element
			cmd = cmd->nx;
			if (cmd->nx == NULL) break;
		} while (cmd->type == TYPE_END); // skip over empty elements

		while (depth > cmd->depth) {		// write commas or embedded closing curlies
			str += sprintf(str, "}");
			depth--;
		}
		if (cmd->nx != NULL) {
			str += sprintf(str, ",");
		}
	} while (cmd->nx != NULL);

	do { // handle closing curlies and NEWLINE
		str += sprintf(str, "}");
	} while (depth-- > 0);
	sprintf(str, "\n");
	return (str - out_buf);
}

//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_JSON

void js_unit_tests()
{
// tip: breakpoint the js_json_parser return (TG_OK) and examine the js[] array

// success cases

	// single NV pair cases
	js_json_parser("{\"config_version\":null}\n");					// simple null test
	js_json_parser("{\"config_profile\":true}\n");					// simple true test
	js_json_parser("{\"prompt\":false}\n");							// simple false test
	js_json_parser("{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");// string test w/comment
	js_json_parser("{\"x_feedrate\":1200}\n");						// numeric test
	js_json_parser("{\"y_feedrate\":-1456}\n");						// numeric test

	js_json_parser("{\"Z_velocity_maximum\":null}\n");				// axis w/null
	js_json_parser("{\"m1_microsteps\":null}\n");					// motor w/null
	js_json_parser("{\"2mi\":8}\n");								// motor token w/null
	js_json_parser("{\"no-token\":12345}\n");						// non-token w/number

	// multi-pair cases					 tabs here V
	js_json_parser("{\"firmware_version\":329.26,		\"config_version\":0.93}\n");
	js_json_parser("{\"1mi\":8, \"2mi\":8,\"3mi\":8,\"4mi\":8}\n");	// 4 elements

	// parent / child cases
	js_json_parser("{\"status_report\":{\"ln\":true, \"x_pos\":true, \"y_pos\":true, \"z_pos\":true}}\n");
	js_json_parser("{\"parent_case1\":{\"child_null\":null}}\n");	// parent w/single child
	js_json_parser("{\"parent_case2\":{\"child_num\":23456}}\n");	// parent w/single child
	js_json_parser("{\"parent_case3\":{\"child_str\":\"stringdata\"}}\n");// parent w/single child

// error cases

	js_json_parser("{\"err_1\":36000x\n}");							// illegal number 
	js_json_parser("{\"err_2\":\"text\n}");							// no string termination
	js_json_parser("{\"err_3\":\"12345\",}\n");						// bad } termination
	js_json_parser("{\"err_4\":\"12345\"\n");						// no } termination

}

#endif // __UNIT_TEST_JSON
