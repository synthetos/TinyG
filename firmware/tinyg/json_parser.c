/*
 * json_parser.c - JSON parser for TinyG
 * This file is part of the TinyG project
 *
 * Copyright (c) 2011 - 2014 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg.h"
#include "config.h"					// JSON sits on top of the config system
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "canonical_machine.h"
#include "report.h"
#include "util.h"
#include "xio.h"					// for char definitions

#ifdef __cplusplus
extern "C"{
#endif

/**** Allocation ****/

jsSingleton_t js;

/**** local scope stuff ****/

static stat_t _json_parser_kernal(char_t *str);
//static stat_t _get_nv_pair_strict(cmdObj_t *cmd, char_t **pstr, int8_t *depth);
static stat_t _get_nv_pair_relaxed(cmdObj_t *cmd, char_t **pstr, int8_t *depth);
static stat_t _normalize_json_string(char_t *str, uint16_t size);

/****************************************************************************
 * json_parser() - exposed part of JSON parser
 * _json_parser_kernal()
 * _normalize_json_string()
 * _get_nv_pair_strict()
 *
 *	This is a dumbed down JSON parser to fit in limited memory with no malloc
 *	or practical way to do recursion ("depth" tracks parent/child levels).
 *
 *	This function will parse the following forms up to the JSON_MAX limits:
 *	  {"name":"value"}
 *	  {"name":12345}
 *	  {"name1":"value1", "n2":"v2", ... "nN":"vN"}
 *	  {"parent_name":""}
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
 *
 *	Separation of concerns
 *	  json_parser() is the only exposed part. It does parsing, display, and status reports.
 *	  _get_nv_pair() only does parsing and syntax; no semantic validation or group handling
 *	  _json_parser_kernal() does index validation and group handling and executes sets and gets
 *		in an application agnostic way. It should work for other apps than TinyG 
 */

void json_parser(char_t *str)
{
	stat_t status = _json_parser_kernal(str);
	cmd_print_list(status, TEXT_NO_PRINT, JSON_RESPONSE_FORMAT);
	sr_request_status_report(SR_IMMEDIATE_REQUEST); // generate incremental status report to show any changes
}

static stat_t _json_parser_kernal(char_t *str)
{
	stat_t status;
	int8_t depth;
	cmdObj_t *cmd = cmd_reset_list();				// get a fresh cmdObj list
	char_t group[GROUP_LEN+1] = {""};				// group identifier - starts as NUL
	int8_t i = CMD_BODY_LEN;

	ritorno(_normalize_json_string(str, JSON_OUTPUT_STRING_MAX));	// return if error

	// parse the JSON command into the cmd body
	do {
		if (--i == 0) { return (STAT_JSON_TOO_MANY_PAIRS); } // length error
//		if ((status = _get_nv_pair_strict(cmd, &str, &depth)) > STAT_EAGAIN) { // erred out
		if ((status = _get_nv_pair_relaxed(cmd, &str, &depth)) > STAT_EAGAIN) { // erred out
			return (status);
		}
		// propagate the group from previous NV pair (if relevant)
		if (group[0] != NUL) {
			strncpy(cmd->group, group, GROUP_LEN);	// copy the parent's group to this child
		}
		// validate the token and get the index
		if ((cmd->index = cmd_get_index(cmd->group, cmd->token)) == NO_MATCH) {
			return (STAT_UNRECOGNIZED_COMMAND);
		}
		if ((cmd_index_is_group(cmd->index)) && (cmd_group_is_prefixed(cmd->token))) {
			strncpy(group, cmd->token, GROUP_LEN);	// record the group ID
		}
		if ((cmd = cmd->nx) == NULL) return (STAT_JSON_TOO_MANY_PAIRS);// Not supposed to encounter a NULL
	} while (status != STAT_OK);					// breaks when parsing is complete

	// execute the command
	cmd = cmd_body;
	if (cmd->objtype == TYPE_NULL){					// means GET the value
		ritorno(cmd_get(cmd));						// ritorno returns w/status on any errors
	} else {
		if (cm.machine_state == MACHINE_ALARM) return (STAT_MACHINE_ALARMED);
		ritorno(cmd_set(cmd));						// set value or call a function (e.g. gcode)
		cmd_persist(cmd);
	}
	return (STAT_OK);								// only successful commands exit through this point
}

/*
 * _normalize_json_string - normalize a JSON string in place
 *
 *	Validate string size limits, remove all whitespace and convert 
 *	to lower case, with the exception of gcode comments
 */

static stat_t _normalize_json_string(char_t *str, uint16_t size)
{
	char_t *wr;								// write pointer
	uint8_t in_comment = false;

	if (strlen(str) > size) return (STAT_INPUT_EXCEEDS_MAX_LENGTH);

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
	return (STAT_OK);
}

/*
 * _get_nv_pair_relaxed() - get the next name-value pair w/relaxed JSON rules
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
 *	"fr" is found in the name string the parser will search for "xfr" in the 
 *	cfgArray.
 */
/*	RELAXED RULES
 * 
 *	Quotes are accepted but not needed on names 
 *	Quotes are required for string values
 *
 *	See build 406.xx or earlier for strict JSON parser - deleted in 407.03
 */

#define MAX_PAD_CHARS 8
#define MAX_NAME_CHARS 32

static stat_t _get_nv_pair_relaxed(cmdObj_t *cmd, char_t **pstr, int8_t *depth)
{
	uint8_t i;
	char_t *tmp;
	char_t leaders[] = {"{,\""};				// open curly, quote and leading comma
	char_t separators[] = {":\""};				// colon and quote
	char_t terminators[] = {"},\""};			// close curly, comma and quote
	char_t value[] = {"{\".-+"};				// open curly, quote, period, minus and plus

	cmd_reset_obj(cmd);							// wipes the object and sets the depth

	// --- Process name part ---
	// Find, terminate and set pointers for the name. Allow for leading and trailing name quotes.
	char_t * name = *pstr;
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(leaders, (int)**pstr) == NULL) { 			// find leading character of name
			name = (*pstr)++;
			break;
		}
		if (i == MAX_PAD_CHARS) return (STAT_JSON_SYNTAX_ERROR);
	}

	// Find the end of name, NUL terminate and copy token
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(separators, (int)**pstr) != NULL) {
			*(*pstr)++ = NUL;
			strncpy(cmd->token, name, TOKEN_LEN+1);			// copy the string to the token
			break;
		}
		if (i == MAX_NAME_CHARS) return (STAT_JSON_SYNTAX_ERROR);
	}

	// --- Process value part ---  (organized from most to least frequently encountered)

	// Find the start of the value part
	for (i=0; true; i++, (*pstr)++) {
		if (isalnum((int)**pstr)) break;
		if (strchr(value, (int)**pstr) != NULL) break;
		if (i == MAX_PAD_CHARS) return (STAT_JSON_SYNTAX_ERROR);
	}

	// nulls (gets)
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { // process null value
		cmd->objtype = TYPE_NULL;
		cmd->value = TYPE_NULL;
	
	// numbers
	} else if (isdigit(**pstr) || (**pstr == '-')) {// value is a number
		cmd->value = (float)strtod(*pstr, &tmp);	// tmp is the end pointer
		if(tmp == *pstr) { return (STAT_BAD_NUMBER_FORMAT);}
		cmd->objtype = TYPE_FLOAT;

	// object parent
	} else if (**pstr == '{') { 
		cmd->objtype = TYPE_PARENT;
//		*depth += 1;							// cmd_reset_obj() sets the next object's level so this is redundant
		(*pstr)++;
		return(STAT_EAGAIN);					// signal that there is more to parse

	// strings
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;
		cmd->objtype = TYPE_STRING;
		if ((tmp = strchr(*pstr, '\"')) == NULL) { return (STAT_JSON_SYNTAX_ERROR);} // find the end of the string
		*tmp = NUL;

		// if string begins with 0x it might be data, needs to be at least 3 chars long
		if( strlen(*pstr)>=3 && (*pstr)[0]=='0' && (*pstr)[1]=='x')
		{
			uint32_t *v = (uint32_t*)&cmd->value;
			*v = strtoul((const char *)*pstr, 0L, 0);
			cmd->objtype = TYPE_DATA;
		} else {
			ritorno(cmd_copy_string(cmd, *pstr));
		}
	
		*pstr = ++tmp;

	// boolean true/false
	} else if (**pstr == 't') { 
		cmd->objtype = TYPE_BOOL;
		cmd->value = true;
	} else if (**pstr == 'f') { 
		cmd->objtype = TYPE_BOOL;
		cmd->value = false;

	// arrays
	} else if (**pstr == '[') {
		cmd->objtype = TYPE_ARRAY;
		ritorno(cmd_copy_string(cmd, *pstr));	// copy array into string for error displays
		return (STAT_INPUT_VALUE_UNSUPPORTED);	// return error as the parser doesn't do input arrays yet

	// general error condition
	} else { return (STAT_JSON_SYNTAX_ERROR); }	// ill-formed JSON

	// process comma separators and end curlies
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) { // advance to terminator or err out
		return (STAT_JSON_SYNTAX_ERROR);
	}
	if (**pstr == '}') { 
		*depth -= 1;							// pop up a nesting level
		(*pstr)++;								// advance to comma or whatever follows
	}
	if (**pstr == ',') { return (STAT_EAGAIN);}	// signal that there is more to parse

	(*pstr)++;
	return (STAT_OK);							// signal that parsing is complete
}

/*
 * _get_nv_pair_strict() - get the next name-value pair w/strict JSON rules
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
 *	"fr" is found in the name string the parser will search for "xfr" in the 
 *	cfgArray.
 */
/*
static stat_t _get_nv_pair_strict(cmdObj_t *cmd, char_t **pstr, int8_t *depth)
{
	char_t *tmp;
	char_t terminators[] = {"},"};

	cmd_reset_obj(cmd);							// wipes the object and sets the depth

	// --- Process name part ---
	// find leading and trailing name quotes and set pointers.
	if ((*pstr = strchr(*pstr, '\"')) == NULL) { return (STAT_JSON_SYNTAX_ERROR);}
	if ((tmp = strchr(++(*pstr), '\"')) == NULL) { return (STAT_JSON_SYNTAX_ERROR);}
	*tmp = NUL;
	strncpy(cmd->token, *pstr, TOKEN_LEN);		// copy the string to the token

	// --- Process value part ---  (organized from most to least frequently encountered)
	*pstr = ++tmp;
	if ((*pstr = strchr(*pstr, ':')) == NULL) return (STAT_JSON_SYNTAX_ERROR);
	(*pstr)++;									// advance to start of value field

	// nulls (gets)
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { // process null value
		cmd->objtype = TYPE_NULL;
		cmd->value = TYPE_NULL;
	
	// numbers
	} else if (isdigit(**pstr) || (**pstr == '-')) {// value is a number
		cmd->value = (float)strtod(*pstr, &tmp);	// tmp is the end pointer
		if(tmp == *pstr) { return (STAT_BAD_NUMBER_FORMAT);}
		cmd->objtype = TYPE_FLOAT;

	// object parent
	} else if (**pstr == '{') { 
		cmd->objtype = TYPE_PARENT;
//		*depth += 1;							// cmd_reset_obj() sets the next object's level so this is redundant
		(*pstr)++;
		return(STAT_EAGAIN);					// signal that there is more to parse

	// strings
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;
		cmd->objtype = TYPE_STRING;
		if ((tmp = strchr(*pstr, '\"')) == NULL) { return (STAT_JSON_SYNTAX_ERROR);} // find the end of the string
		*tmp = NUL;

//		ritorno(cmd_copy_string(cmd, *pstr));
		// if string begins with 0x it might be data, needs to be at least 3 chars long
		if( strlen(*pstr)>=3 && (*pstr)[0]=='0' && (*pstr)[1]=='x')
		{
			uint32_t *v = (uint32_t*)&cmd->value;
			*v = strtoul(*pstr, 0L, 0);
			cmd->objtype = TYPE_DATA;
		} else {
			ritorno(cmd_copy_string(cmd, *pstr));
		}
	
		*pstr = ++tmp;

	// boolean true/false
	} else if (**pstr == 't') { 
		cmd->objtype = TYPE_BOOL;
		cmd->value = true;
	} else if (**pstr == 'f') { 
		cmd->objtype = TYPE_BOOL;
		cmd->value = false;

	// arrays
	} else if (**pstr == '[') {
		cmd->objtype = TYPE_ARRAY;
		ritorno(cmd_copy_string(cmd, *pstr));	// copy array into string for error displays
		return (STAT_INPUT_VALUE_UNSUPPORTED);	// return error as the parser doesn't do input arrays yet

	// general error condition
	} else { return (STAT_JSON_SYNTAX_ERROR); }	// ill-formed JSON

	// process comma separators and end curlies
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) { // advance to terminator or err out
		return (STAT_JSON_SYNTAX_ERROR);
	}
	if (**pstr == '}') { 
		*depth -= 1;							// pop up a nesting level
		(*pstr)++;								// advance to comma or whatever follows
	}
	if (**pstr == ',') { return (STAT_EAGAIN);}	// signal that there is more to parse

	(*pstr)++;
	return (STAT_OK);							// signal that parsing is complete
}
*/

/****************************************************************************
 * json_serialize() - make a JSON object string from JSON object array
 *
 *	*cmd is a pointer to the first element in the cmd list to serialize
 *	*out_buf is a pointer to the output string - usually what was the input string
 *	Returns the character count of the resulting string
 *
 * 	Operation:
 *	  - The cmdObj list is processed start to finish with no recursion
 *
 *	  - Assume the first object is depth 0 or greater (the opening curly)
 *
 *	  - Assume remaining depths have been set correctly; but might not achieve closure;
 *		e.g. list starts on 0, and ends on 3, in which case provide correct closing curlies
 *
 *	  - Assume there can be multiple, independent, non-contiguous JSON objects at a 
 *		given depth value. These are processed correctly - e.g. 0,1,1,0,1,1,0,1,1
 *
 *	  - The list must have a terminating cmdObj where cmd->nx == NULL. 
 *		The terminating object may or may not have data (empty or not empty).
 *
 *	Returns:
 *		Returns length of string
 *
 *	Desired behaviors:
 *	  - Allow self-referential elements that would otherwise cause a recursive loop
 *	  - Skip over empty objects (TYPE_EMPTY)
 *	  - If a JSON object is empty represent it as {}
 *	    --- OR ---
 *	  - If a JSON object is empty omit the object altogether (no curlies)
 */

#define BUFFER_MARGIN 8			// safety margin to avoid buffer overruns during footer checksum generation

uint16_t json_serialize(cmdObj_t *cmd, char_t *out_buf, uint16_t size)
{
#ifdef __SILENCE_JSON_RESPONSES
	return (0);
#else
	char_t *str = out_buf;
	char_t *str_max = out_buf + size - BUFFER_MARGIN;
	int8_t initial_depth = cmd->depth;
	int8_t prev_depth = 0;
	uint8_t need_a_comma = false;

	*str++ = '{'; 								// write opening curly

	while (true) {
		if (cmd->objtype != TYPE_EMPTY) {
			if (need_a_comma) { *str++ = ',';}
			need_a_comma = true;
			if (js.json_syntax == JSON_SYNTAX_RELAXED) {		// write name
				str += sprintf((char *)str, "%s:", cmd->token);
			} else {
				str += sprintf((char *)str, "\"%s\":", cmd->token);
			}

			// check for illegal float values
			if (cmd->objtype == TYPE_FLOAT) {
				if (isnan((double)cmd->value) || isinf((double)cmd->value)) { cmd->value = 0;}
			}

			// serialize output value
			if		(cmd->objtype == TYPE_NULL)		{ str += (char_t)sprintf((char *)str, "null");} // Note that that "" is NOT null.
			else if (cmd->objtype == TYPE_INTEGER)	{
				str += (char_t)sprintf((char *)str, "%1.0f", (double)cmd->value);
			}
			else if (cmd->objtype == TYPE_DATA)	{
				uint32_t *v = (uint32_t*)&cmd->value;
				str += (char_t)sprintf((char *)str, "\"0x%lx\"", (unsigned long)*v);
			}
			else if (cmd->objtype == TYPE_STRING)	{ str += (char_t)sprintf((char *)str, "\"%s\"",(char *)*cmd->stringp);}
			else if (cmd->objtype == TYPE_ARRAY)	{ str += (char_t)sprintf((char *)str, "[%s]",  (char *)*cmd->stringp);}
			else if (cmd->objtype == TYPE_FLOAT) {
				if 		(cmd->precision == 0) { str += (char_t)sprintf((char *)str, "%0.0f", (double)cmd->value);}
				else if (cmd->precision == 1) { str += (char_t)sprintf((char *)str, "%0.1f", (double)cmd->value);}
				else if (cmd->precision == 2) { str += (char_t)sprintf((char *)str, "%0.2f", (double)cmd->value);}
				else if (cmd->precision == 3) { str += (char_t)sprintf((char *)str, "%0.3f", (double)cmd->value);}
				else if (cmd->precision == 4) { str += (char_t)sprintf((char *)str, "%0.4f", (double)cmd->value);}
				else 						  { str += (char_t)sprintf((char *)str, "%f",    (double)cmd->value);}
			}
			else if (cmd->objtype == TYPE_BOOL) {
				if (fp_FALSE(cmd->value)) { str += sprintf((char *)str, "false");}
				else { str += (char_t)sprintf((char *)str, "true"); }
			}
			if (cmd->objtype == TYPE_PARENT) {
				*str++ = '{';
				need_a_comma = false;
			}
		}
		if (str >= str_max) { return (-1);}		// signal buffer overrun
		if ((cmd = cmd->nx) == NULL) { break;}	// end of the list

		while (cmd->depth < prev_depth--) {		// iterate the closing curlies
			need_a_comma = true;
			*str++ = '}';
		}
		prev_depth = cmd->depth;
	}

	// closing curlies and NEWLINE
	while (prev_depth-- > initial_depth) { *str++ = '}';}
	str += sprintf((char *)str, "}\n");	// using sprintf for this last one ensures a NUL termination
	if (str > out_buf + size) { return (-1);}
	return (str - out_buf);
#endif
}

/*
 * json_print_object() - serialize and print the cmdObj array directly (w/o header & footer)
 *
 *	Ignores JSON verbosity settings and everything else - just serializes the list & prints
 *	Useful for reports and other simple output.
 *	Object list should be terminated by cmd->nx == NULL
 */
void json_print_object(cmdObj_t *cmd)
{
#ifdef __SILENCE_JSON_RESPONSES
	return;
#endif

	json_serialize(cmd, cs.out_buf, sizeof(cs.out_buf));
	fprintf(stderr, "%s", (char *)cs.out_buf);
}

/*
 * json_print_list() - command to select and produce a JSON formatted output
 */

void json_print_list(stat_t status, uint8_t flags)
{
	switch (flags) {
		case JSON_NO_PRINT: { break; } 
		case JSON_OBJECT_FORMAT: { json_print_object(cmd_body); break; }
		case JSON_RESPONSE_FORMAT: { json_print_response(status); break; }
	}
}

/*
 * json_print_response() - JSON responses with headers, footers and observing JSON verbosity 
 *
 *	A footer is returned for every setting except $jv=0
 *
 *	JV_SILENT = 0,	// no response is provided for any command
 *	JV_FOOTER,		// responses contain  footer only; no command echo, gcode blocks or messages
 *	JV_CONFIGS,		// echo configs; gcode blocks are not echoed; messages are not echoed
 *	JV_MESSAGES,	// echo configs; gcode messages only (if present); no block echo or line numbers
 *	JV_LINENUM,		// echo configs; gcode blocks return messages and line numbers as present
 *	JV_VERBOSE		// echos all configs and gcode blocks, line numbers and messages
 *
 *	This gets a bit complicated. The first cmdObj is the header, which must be set by reset_list().
 *	The first object in the body will always have the gcode block or config command in it, 
 *	which you may or may not want to display. This is followed by zero or more displayable objects. 
 *	Then if you want a gcode line number you add that here to the end. Finally, a footer goes 
 *	on all the (non-silent) responses.
 */
#define MAX_TAIL_LEN 8

void json_print_response(uint8_t status)
{
#ifdef __SILENCE_JSON_RESPONSES
	return;
#endif

	if (js.json_verbosity == JV_SILENT) return;			// silent responses

	// Body processing
	cmdObj_t *cmd = cmd_body;
	if (status == STAT_JSON_SYNTAX_ERROR) {
		cmd_reset_list();
		cmd_add_string((const char_t *)"err", escape_string(cs.in_buf, cs.saved_buf));

	} else if (cm.machine_state != MACHINE_INITIALIZING) {	// always do full echo during startup
		uint8_t cmd_type;
		do {
			if ((cmd_type = cmd_get_type(cmd)) == CMD_TYPE_NULL) break;

			if (cmd_type == CMD_TYPE_GCODE) {
				if (js.echo_json_gcode_block == false) {	// kill command echo if not enabled
					cmd->objtype = TYPE_EMPTY;
				}

//++++		} else if (cmd_type == CMD_TYPE_CONFIG) {		// kill config echo if not enabled
//fix me		if (js.echo_json_configs == false) {
//					cmd->objtype = TYPE_EMPTY;
//				}

			} else if (cmd_type == CMD_TYPE_MESSAGE) {		// kill message echo if not enabled
				if (js.echo_json_messages == false) {
					cmd->objtype = TYPE_EMPTY;
				}

			} else if (cmd_type == CMD_TYPE_LINENUM) {		// kill line number echo if not enabled
				if ((js.echo_json_linenum == false) || (fp_ZERO(cmd->value))) { // do not report line# 0
					cmd->objtype = TYPE_EMPTY;
				}
			}
		} while ((cmd = cmd->nx) != NULL);
	}

	// Footer processing
	while(cmd->objtype != TYPE_EMPTY) {						// find a free cmdObj at end of the list...
		if ((cmd = cmd->nx) == NULL) {						//...or hit the NULL and return w/o a footer
			json_serialize(cmd_header, cs.out_buf, sizeof(cs.out_buf));
			return;
		}
	}
	char_t footer_string[CMD_FOOTER_LEN];
	sprintf((char *)footer_string, "%d,%d,%d,0", FOOTER_REVISION, status, cs.linelen);
	cs.linelen = 0;										// reset linelen so it's only reported once

	cmd_copy_string(cmd, footer_string);				// link string to cmd object
//	cmd->depth = 0;										// footer 'f' is a peer to response 'r' (hard wired to 0)
	cmd->depth = js.json_footer_depth;					// 0=footer is peer to response 'r', 1=child of response 'r'
	cmd->objtype = TYPE_ARRAY;
	strcpy(cmd->token, "f");							// terminate the list
	cmd->nx = NULL;

	// do all this to avoid having to serialize it twice
	int16_t strcount = json_serialize(cmd_header, cs.out_buf, sizeof(cs.out_buf));// make JSON string w/o checksum
	if (strcount < 0) { return;}						// encountered an overrun during serialization
	if (strcount > OUTPUT_BUFFER_LEN - MAX_TAIL_LEN) { return;}	// would overrun during checksum generation
	int16_t strcount2 = strcount;
	char tail[MAX_TAIL_LEN];

	while (cs.out_buf[strcount] != '0') { strcount--; }	// find end of checksum
	strcpy(tail, cs.out_buf + strcount + 1);			// save the json termination

	while (cs.out_buf[strcount2] != ',') { strcount2--; }// find start of checksum
	sprintf((char *)cs.out_buf + strcount2 + 1, "%d%s", compute_checksum(cs.out_buf, strcount2), tail);
	fprintf(stderr, "%s", cs.out_buf);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * json_set_jv()
 */

stat_t json_set_jv(cmdObj_t *cmd) 
{
	if (cmd->value > JV_VERBOSE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	js.json_verbosity = cmd->value;

	js.echo_json_footer = false;
	js.echo_json_messages = false;
	js.echo_json_configs = false;
	js.echo_json_linenum = false;
	js.echo_json_gcode_block = false;

	if (cmd->value >= JV_FOOTER) 	{ js.echo_json_footer = true;}
	if (cmd->value >= JV_MESSAGES)	{ js.echo_json_messages = true;}
	if (cmd->value >= JV_CONFIGS)	{ js.echo_json_configs = true;}
	if (cmd->value >= JV_LINENUM)	{ js.echo_json_linenum = true;}
	if (cmd->value >= JV_VERBOSE)	{ js.echo_json_gcode_block = true;}

	return(STAT_OK);
}


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

/*
 * js_print_ej()
 * js_print_jv()
 * js_print_j2()
 * js_print_fs()
 */

static const char fmt_ej[] PROGMEM = "[ej]  enable json mode%13d [0=text,1=JSON]\n";
static const char fmt_jv[] PROGMEM = "[jv]  json verbosity%15d [0=silent,1=footer,2=messages,3=configs,4=linenum,5=verbose]\n";
static const char fmt_js[] PROGMEM = "[js]  json serialize style%9d [0=relaxed,1=strict]\n";
static const char fmt_fs[] PROGMEM = "[fs]  footer style%17d [0=new,1=old]\n";

void js_print_ej(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_ej);}
void js_print_jv(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_jv);}
void js_print_js(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_js);}
void js_print_fs(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_fs);}

#endif // __TEXT_MODE


/****************************************************************************
 ***** Unit Tests ***********************************************************
 ****************************************************************************/

#if defined (__UNIT_TESTS) && defined (__UNIT_TEST_JSON)

static void _test_parser(void);
static void _test_serialize(void);
static cmdObj_t * _reset_array(void);
static cmdObj_t * _add_parent(cmdObj_t *cmd, char_t *token);
static cmdObj_t * _add_string(cmdObj_t *cmd, char_t *token, char_t *string);
static cmdObj_t * _add_integer(cmdObj_t *cmd, char_t *token, uint32_t integer);
static cmdObj_t * _add_empty(cmdObj_t *cmd);
static cmdObj_t * _add_array(cmdObj_t *cmd, char_t *footer);
static char_t * _clr(char_t *buf);
static void _printit(void);

#define ARRAY_LEN 8
	cmdObj_t cmd_array[ARRAY_LEN];

void js_unit_tests()
{
	_test_parser();
//	_test_serialize();
}

static void _test_serialize()
{
	cmdObj_t *cmd = cmd_array;

	// null list
	_reset_array();
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// parent with a null child
	cmd = _reset_array();
	cmd = _add_parent(cmd, (char_t *)"r");
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// single string element (message)
	cmd = _reset_array();
	cmd = _add_string(cmd, (char_t *)"msg", (char_t *)"test message");
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// string element and an integer element
	cmd = _reset_array();
	cmd = _add_string(cmd, (char_t *)"msg", (char_t *)"test message");
	cmd = _add_integer(cmd, (char_t *)"answer", 42);
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// parent with a string and an integer element
	cmd = _reset_array();
	cmd = _add_parent(cmd, (char_t *)"r");
	cmd = _add_string(cmd, (char_t *)"msg", (char_t *)"test message");
	cmd = _add_integer(cmd, (char_t *)"answer", 42);
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// parent with a null child followed by a final level 0 element (footer)
	cmd = _reset_array();
	cmd = _add_parent(cmd, (char_t *)"r");
	cmd = _add_empty(cmd);
	cmd = _add_string(cmd, (char_t *)"f", (char_t *)"[1,0,12,1234]");	// fake out a footer
	cmd->pv->depth = 0;
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// parent with a single element child followed by empties folowed by a final level 0 element
	cmd = _reset_array();
	cmd = _add_parent(cmd, (char_t *)"r");
	cmd = _add_integer(cmd, (char_t *)"answer", 42);
	cmd = _add_empty(cmd);
	cmd = _add_empty(cmd);
	cmd = _add_string(cmd, (char_t *)"f", (char_t *)"[1,0,12,1234]");	// fake out a footer
	cmd->pv->depth = 0;
	json_serialize(cmd_array, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// response object parent with no children w/footer
	cmd_reset_list();											// works with the header/body/footer list
	_add_array(cmd, (char_t *)"1,0,12,1234");					// fake out a footer
	json_serialize(cmd_header, cs.out_buf, sizeof(cs.out_buf));
	_printit();

	// response parent with one element w/footer
	cmd_reset_list();											// works with the header/body/footer list
	cmd_add_string((char_t *)"msg", (char_t *)"test message");
	_add_array(cmd, (char_t *)"1,0,12,1234");					// fake out a footer
	json_serialize(cmd_header, cs.out_buf, sizeof(cs.out_buf));
	_printit();
}

static char_t * _clr(char_t *buf)
{
	for (uint8_t i=0; i<250; i++) {
		buf[i] = 0;
	}
	return (buf);
}

static void _printit(void)
{
//	printf("%s", cs.out_buf);	
}

static cmdObj_t * _reset_array()
{
	cmdObj_t *cmd = cmd_array;
	for (uint8_t i=0; i<ARRAY_LEN; i++) {
		if (i == 0) { cmd->pv = NULL; } 
		else { cmd->pv = (cmd-1);}
		cmd->nx = (cmd+1);
		cmd->index = 0;
		cmd->token[0] = NUL;
		cmd->depth = 0;
		cmd->objtype = TYPE_EMPTY;
		cmd++;
	}
	(--cmd)->nx = NULL;				// correct last element
	return (cmd_array);
}

static cmdObj_t * _add_parent(cmdObj_t *cmd, char_t *token)
{
	strncpy(cmd->token, token, TOKEN_LEN);
	cmd->nx->depth = cmd->depth+1;
	cmd->objtype = TYPE_PARENT;
	return (cmd->nx);
}

static cmdObj_t * _add_string(cmdObj_t *cmd, char_t *token, char_t *string)
{
	strncpy(cmd->token, token, TOKEN_LEN);
	cmd_copy_string(cmd, string);
	if (cmd->depth < cmd->pv->depth) { cmd->depth = cmd->pv->depth;}
	cmd->objtype = TYPE_STRING;
	return (cmd->nx);
}

static cmdObj_t * _add_integer(cmdObj_t *cmd, char_t *token, uint32_t integer)
{
	strncpy(cmd->token, token, TOKEN_LEN);
	cmd->value = (float)integer;
	if (cmd->depth < cmd->pv->depth) { cmd->depth = cmd->pv->depth;}
	cmd->objtype = TYPE_INTEGER;
	return (cmd->nx);
}

cmdObj_t * _add_data(cmdObj_t *cmd, char *token, uint32_t integer)
{
	strncpy(cmd->token, token, TOKEN_LEN);
	uint32_t *v = (uint32_t*)&cmd->value;
	*v = integer;
	if (cmd->depth < cmd->pv->depth) { cmd->depth = cmd->pv->depth;}
	cmd->objtype = TYPE_DATA;
	return (cmd->nx);
}

static cmdObj_t * _add_empty(cmdObj_t *cmd)
{
	if (cmd->depth < cmd->pv->depth) { cmd->depth = cmd->pv->depth;}
	cmd->objtype = TYPE_EMPTY;
	return (cmd->nx);
}

static cmdObj_t * _add_array(cmdObj_t *cmd, char_t *array_string)
{
	cmd->objtype = TYPE_ARRAY;
//	strncpy(cmd->string, array_string, CMD_STRING_LEN);
	cmd_copy_string(cmd, array_string);
	return (cmd->nx);
}

static void _test_parser()
{
// tip: breakpoint the json_parser return (STAT_OK) and examine the js[] array

// success cases

	// success cases
//	json_parser((char_t *)"{\"gc\":\"g0x100\"}\n");
//	json_parser((char_t *)"{gc:\"g0x100\"}\n");
//	json_parser((char_t *)"{xvm:1000}\n");
//	json_parser((char_t *)"{xvm:n}\n");
	json_parser((char_t *)"{x:{vm:1200, fr:1100}}\n");
	json_parser((char_t *)"{uda:{1:\"0x3\",2:\"0x4\"}}\n");

/*
	json_parser((char_t *)"{\"config_version\":null}\n");		// simple null test
	json_parser((char_t *)"{\"config_profile\":true}\n");		// simple true test
	json_parser((char_t *)"{\"prompt\":false}\n");				// simple false test
	json_parser((char_t *)"{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");// string test w/comment
	json_parser((char_t *)"{\"x_feedrate\":1200}\n");			// numeric test
	json_parser((char_t *)"{\"y_feedrate\":-1456}\n");			// numeric test

	json_parser((char_t *)"{\"Z_velocity_maximum\":null}\n");	// axis w/null
	json_parser((char_t *)"{\"m1_microsteps\":null}\n");		// motor w/null
	json_parser((char_t *)"{\"2mi\":8}\n");						// motor token w/null
	json_parser((char_t *)"{\"no-token\":12345}\n");			// non-token w/number

	// multi-pair cases					 tabs here V
	json_parser((char_t *)"{\"firmware_version\":329.26,		\"config_version\":0.93}\n");
	json_parser((char_t *)"{\"1mi\":8, \"2mi\":8,\"3mi\":8,\"4mi\":8}\n");	// 4 elements

	// parent / child cases
	json_parser((char_t *)"{\"status_report\":{\"ln\":true, \"x_pos\":true, \"y_pos\":true, \"z_pos\":true}}\n");
	json_parser((char_t *)"{\"parent_case1\":{\"child_null\":null}}\n");		// parent w/single child
	json_parser((char_t *)"{\"parent_case2\":{\"child_num\":23456}}\n");		// parent w/single child
	json_parser((char_t *)"{\"parent_case3\":{\"child_str\":\"stringdata\"}}\n");// parent w/single child
*/
// error cases

	json_parser((char_t *)"{\"err_1\":36000x\n}");				// illegal number 
	json_parser((char_t *)"{\"err_2\":\"text\n}");				// no string termination
	json_parser((char_t *)"{\"err_3\":\"12345\",}\n");			// bad } termination
	json_parser((char_t *)"{\"err_4\":\"12345\"\n");			// no } termination

}

#endif // __UNIT_TEST_JSON

#ifdef __cplusplus
}
#endif // __cplusplus
