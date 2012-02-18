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
static uint8_t _get_nv_pair(cmdObj *cmd, char **pstr, int8_t *level, const char *grp);
static uint8_t _normalize_json_string(char *str, uint16_t size);
//static uint8_t _make_json_response(cmdObj *cmd, char *str, uint16_t size);
//static uint8_t _make_json_error_response(cmdObj *cmd, char **pstr, uint8_t status);

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
 *	This is a dumbed down JSON parser to fit in limited memory with no malloc.
 *	This function will parse the following forms up to the JSON_MAX limits:
 *
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
 *	  - exponentiated numbers ore OK. 
 *	  - hexadecimal is not supported
 *
 *	The parser:
 *	  - extracts an array of one or more JSON object structs from the input string
 *	  - executes the object(s) in the array
 *	  - passes the executed array to the response handler to generate the response string
 *	  - returns the status and the JSON response string
 */

uint8_t js_json_parser(char *in_str, char *out_str)
{
	uint8_t status = _json_parser(in_str);
	js_make_json_string(&cmd_array[0], out_str);
	return (status);
}

static uint8_t _json_parser(char *str)
{
	uint8_t i=0;								// json array index
	int8_t nesting_level = 0;					// root is starting level 
	char grp[CMD_GROUP_LEN+1] = { "" };			// group identifier
	cmdObj *cmd = &cmd_array[0];				// point at first struct in the array

	// test and normalize JSON input string
	ritorno(_normalize_json_string(str, JSON_STRING_LEN));

	// deserialize JSON input string into cmdObj array
	for (i=0; i<CMD_ARRAY_SIZE; i++) {
		ritorno(_get_nv_pair(&cmd_array[i], &str, &nesting_level, grp));
		if (cmd_array[i].nx == NULL) break;		// NULL means it's the last (or only) NV pair
		if (cmd->value_type == VALUE_TYPE_PARENT) {
			strncpy(grp, cmd->token, CMD_GROUP_LEN);
		}
	}

	// take action on the cmdObj array
	for (i=0; i<CMD_ARRAY_SIZE; i++) {
		if (cmd->value_type == VALUE_TYPE_NULL){// means GET the value
			ritorno(cmd_get(cmd->index, cmd));
		} else {								// else SET it or perform action
			ritorno(cmd_set(cmd->index, cmd));
			cmd_write_NVM_value(cmd->index, cmd);// persist the value to NVM
		}
		if ((cmd->nx == NULL) || (cmd->value_type == VALUE_TYPE_PARENT)) {
			break;
		}
		cmd = cmd->nx;
	}
	return (TG_OK);
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
 *	Parse the next statement and populate the JSON object.
 *
 *	Leaves string pointer (str) on the first character following the object.
 *	Which is the character just past the ',' separator if it's a multi-valued 
 *	object or the terminating NUL if single object or the last in a multi.
 *
 *	Keeps track of nesting levels and closing braces as much as it has to.
 *	If this were to be extended to track multiple parents or more than two
 *	levels it would have to track closing curlies - which it does not.
 *
 *	ASSUMES INPUT STRING HAS FIRST BEEN NORMALIZED BY _normalize_json_string()
 *
 *	If a group prefix is passed in it will be pre-pended to any name parsed
 *	to form a token string. For example, if "x" is provided as a group and 
 *	"fr" is found in the name string the parser will search for "xfr"in the 
 *	cfgArray.
 */

static uint8_t _get_nv_pair(cmdObj *cmd, char **pstr, int8_t *level, const char *grp) 
{
	char *tmp;
	char terminators[] = {"},"};

	cmd_new_object(cmd);						// wipe the object
	cmd->nesting_level = *level;
	cmd->value_type = VALUE_TYPE_ERROR;			//...until told otherwise

	// process name field
	// find leading and trailing name quotes and set pointers accordingly
	// accommodate groups by looking up index by full name but stripping group from token
	if ((*pstr = strchr(*pstr, '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	if ((tmp = strchr(++(*pstr), '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	*tmp = NUL;
//	strncpy(cmd->name, *pstr, CMD_NAME_LEN);	// copy name from string
	strncpy(cmd->name, grp, CMD_NAME_LEN);		// prepend the group if there is a group
	strncpy(&cmd->name[strlen(grp)], *pstr, CMD_NAME_LEN); // cat group prefix to name
	if ((cmd->index = cmd_get_index(cmd->name)) == -1) { return (TG_UNRECOGNIZED_COMMAND);}
	cmd_get_token(cmd->index, cmd->token);
	strncpy(cmd->token, &cmd->name[strlen(grp)], CMD_TOKEN_LEN+1);	// strip group if group
	*pstr = ++tmp;

	// process value field
	if ((*pstr = strchr(*pstr, ':')) == NULL) return (TG_JSON_SYNTAX_ERROR);
	(*pstr)++;									// advance to start of value field
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { 
		cmd->value_type = VALUE_TYPE_NULL;
		cmd->value = VALUE_TYPE_NULL;
	} else if (**pstr == 'f') { 
		cmd->value_type = VALUE_TYPE_FALSE;
		cmd->value = false;						// (technically not necessary due to the init)
	} else if (**pstr == 't') { 
		cmd->value_type = VALUE_TYPE_TRUE;
		cmd->value = true;
	} else if (isdigit(**pstr) || (**pstr == '-')) { // value is a number
		cmd->value = strtod(*pstr, &tmp);		// tmp is the end pointer
		if(tmp == *pstr) return (TG_BAD_NUMBER_FORMAT);
		cmd->value_type = VALUE_TYPE_FLOAT;
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;
		if ((tmp = strchr(*pstr, '\"')) == NULL) return (TG_JSON_SYNTAX_ERROR); // find the end of the string
		*tmp = NUL;
		strncpy(cmd->string, *pstr, CMD_STRING_LEN);
		cmd->value_type = VALUE_TYPE_STRING;
		*pstr = ++tmp;
	} else if (**pstr == '{') { cmd->value_type = VALUE_TYPE_PARENT;
		cmd->nx = cmd + 1;						// signal that there is more to come
		*level += 1;
		(*pstr)++;
		return(TG_OK);
	} else {
		 return (TG_JSON_SYNTAX_ERROR);			// ill-formed JSON
	}

	// process end condition
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) { 
		return (TG_JSON_SYNTAX_ERROR);
	}
	if (**pstr == ',') { 
		cmd->nx = cmd + 1;						// signal that there is more to come
	}
	(*pstr)++;
	return (TG_OK);
}

/****************************************************************************
 * js_make_json_string() - make a vanilla JSON string from JSON object array
 *
 *	*cmd is a pointer to the first element in the cmd array
 *	*str is a pointer to the output string - usually what was the input string
 */
uint8_t js_make_json_string(cmdObj *cmd, char *str)
{
	uint8_t end_curlies = 1;

	strcpy(str++, "{"); 							// write opening curly
	for (uint8_t i=0; i<CMD_ARRAY_SIZE; i++) {		// iterate cmd array
		if (cmd->index == -1) break;
		str += sprintf(str, "\"%s\":", cmd->token);
		if (cmd->value_type == VALUE_TYPE_PARENT) {
			str += sprintf(str, "{");
			cmd = cmd->nx;
			end_curlies++;
			continue;
		} else if (cmd->value_type == VALUE_TYPE_NULL)   { str += sprintf(str, "\"\"");
		} else if (cmd->value_type == VALUE_TYPE_FALSE)  { str += sprintf(str, "false");
		} else if (cmd->value_type == VALUE_TYPE_TRUE)   { str += sprintf(str, "true");
		} else if (cmd->value_type == VALUE_TYPE_INT32)  { str += sprintf(str, "%d", (int)cmd->value);
		} else if (cmd->value_type == VALUE_TYPE_FLOAT)  { str += sprintf(str, "%0.3f", cmd->value);
		} else if (cmd->value_type == VALUE_TYPE_STRING) { str += sprintf(str, "\"%s\"", cmd->string);
		} 
		if (cmd->nx == NULL) break;					// no more. You can leave now.
		str += sprintf(str, ","); 
		cmd = cmd->nx;
	}
	while (end_curlies-- != 0) {
		str += sprintf(str, "}");
	}
	sprintf(str, "\n");
	return (TG_OK);
}

/****************************************************************************
 * _make_json_response() - make a response string from JSON object array
 * _make_json_error_response() - add an error response to the string
 *
 *	*cmd is a pointer to the first element in the cmd array
 *	*str is a pointer to the output string - usually what was the input string
 *
 * 	Generates one non-nested response or multiple nested responses of form:
 *	 {"name":
 *		{"name":value,				// value form depends on value type
 *		 "stat":0,					// stat zero is TG_OK. All else are errors
 *		 "msg":"error message"		// msg only present if error
 *		}
 *	 }
 *
 *	Note: This function only handles objects made of single and multiple NV pairs.
 *	It does not handle parent/child nested objects.
 */
/*
static uint8_t _make_json_response(cmdObj *cmd, char *str, uint16_t size)
{
	uint8_t status = TG_OK;

	// write opening curly
	strcpy(str++, "{");

	// preprocess struct looking for gross errors affecting the entire structure
	if (cmd->index == -1) {	
		status = _make_json_error_response(cmd, &str, TG_UNRECOGNIZED_COMMAND);
		str += sprintf(str, "}\n");
		return (status);
	}

	// iterate over command object array to make NV responses
	for (uint8_t i=0; i<CMD_ARRAY_SIZE; i++) {

		// trap error response cases
		if (cmd->index == -1) {
			status = _make_json_error_response(cmd, &str, TG_UNRECOGNIZED_COMMAND);
		} else if (cmd->status != TG_OK) {
			status = _make_json_error_response(cmd, &str, cmd->status);
		} else if ((cmd->value_type < VALUE_TYPE_NULL) || (cmd->value_type > VALUE_TYPE_STRING)) {
			status = _make_json_error_response(cmd, &str, TG_JSON_SYNTAX_ERROR);

		// non-error response cases
		} else {
			str += sprintf(str, "\"%s\":{\"%s\":", cmd->token, cmd->token);
			if (cmd->value_type == VALUE_TYPE_NULL) {
				str += sprintf(str, "\"\"");
			} else if (cmd->value_type == VALUE_TYPE_FALSE) {
				str += sprintf(str, "false");
			} else if (cmd->value_type == VALUE_TYPE_TRUE) {
				str += sprintf(str, "true");
			} else if (cmd->value_type == VALUE_TYPE_NUMBER) {
				str += sprintf(str, "%0.3f", cmd->value);
			} else if (cmd->value_type == VALUE_TYPE_STRING) {
				str += sprintf(str, "\"%s\"", cmd->string);
			} 
			str += sprintf(str, ",\"st\":%d}", cmd->status);	// status NV pair & termination
		}
		if (strlen(str) > size) return (TG_OUTPUT_EXCEEDS_MAX_LENGTH); // that's bad
		if (cmd->nx == NULL) break;			// no more. You can leave now.
		str += sprintf(str, ","); 
		cmd = cmd->nx;
	}

	// write closing curly
	str += sprintf(str, "}\n");
	return (status);
}

static uint8_t _make_json_error_response(cmdObj *cmd, char **pstr, uint8_t status)
{
	char tbuf[STATUS_MESSAGE_LEN];

	cmd->status = status;
	*pstr += sprintf(*pstr, "\"%s\":{\"%s\":\"\"", cmd->name, cmd->name);
	*pstr += sprintf(*pstr, ",\"stat\":%d", cmd->status);
	*pstr += sprintf(*pstr, ",\"msg\":\"%s\"}", tg_get_status_message(cmd->status, tbuf));
	return (status);
}
*/
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
