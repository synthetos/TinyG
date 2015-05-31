/*
 * json_parser.c - JSON parser for TinyG
 * This file is part of the TinyG project
 *
 * Copyright (c) 2011 - 2015 Alden S. Hart, Jr.
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
static stat_t _get_nv_pair(nvObj_t *nv, char_t **pstr, int8_t *depth);
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
	nv_print_list(status, TEXT_NO_PRINT, JSON_RESPONSE_FORMAT);
	sr_request_status_report(SR_IMMEDIATE_REQUEST); // generate incremental status report to show any changes
}

static stat_t _json_parser_kernal(char_t *str)
{
	stat_t status;
	int8_t depth;
	nvObj_t *nv = nv_reset_nv_list();				// get a fresh nvObj list
	char_t group[GROUP_LEN+1] = {""};				// group identifier - starts as NUL
	int8_t i = NV_BODY_LEN;

	ritorno(_normalize_json_string(str, JSON_OUTPUT_STRING_MAX));	// return if error

	// parse the JSON command into the nv body
	do {
		if (--i == 0)
            return (STAT_JSON_TOO_MANY_PAIRS);      // length error

        // Use relaxed parser. Will read eitehr strict or relaxed mode. To use strict-only parser refer
        // to build earlier than 407.03. Substitute _get_nv_pair_strict() for _get_nv_pair()
		if ((status = _get_nv_pair(nv, &str, &depth)) > STAT_EAGAIN) { // erred out
			return (status);
		}
		// propagate the group from previous NV pair (if relevant)
		if (group[0] != NUL) {
			strncpy(nv->group, group, GROUP_LEN);	// copy the parent's group to this child
		}
		// validate the token and get the index
		if ((nv->index = nv_get_index(nv->group, nv->token)) == NO_MATCH) {
			return (STAT_UNRECOGNIZED_NAME);
		}
		if ((nv_index_is_group(nv->index)) && (nv_group_is_prefixed(nv->token))) {
			strncpy(group, nv->token, GROUP_LEN);	// record the group ID
		}
		if ((nv = nv->nx) == NULL)
            return (STAT_JSON_TOO_MANY_PAIRS);      // Not supposed to encounter a NULL
	} while (status != STAT_OK);					// breaks when parsing is complete

	// execute the command
	nv = nv_body;
	if (nv->valuetype == TYPE_NULL){				// means GET the value
		ritorno(nv_get(nv));						// ritorno returns w/status on any errors
	} else {
		if (cm.machine_state == MACHINE_ALARM)
            return (STAT_MACHINE_ALARMED);
		ritorno(nv_set(nv));						// set value or call a function (e.g. gcode)
		nv_persist(nv);
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

	if (strlen(str) > size)
        return (STAT_INPUT_EXCEEDS_MAX_LENGTH);

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
 * _get_nv_pair() - get the next name-value pair w/relaxed JSON rules. Also parses strict JSON.
 *
 *	Parse the next statement and populate the command object (nvObj).
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

static stat_t _get_nv_pair(nvObj_t *nv, char_t **pstr, int8_t *depth)
{
	uint8_t i;
	char_t *tmp;
	char_t leaders[] = {"{,\""};				// open curly, quote and leading comma
	char_t separators[] = {":\""};				// colon and quote
	char_t terminators[] = {"},\""};			// close curly, comma and quote
	char_t value[] = {"{\".-+"};				// open curly, quote, period, minus and plus

	nv_reset_nv(nv);							// wipes the object and sets the depth

	// --- Process name part ---
	// Find, terminate and set pointers for the name. Allow for leading and trailing name quotes.
	char_t * name = *pstr;
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(leaders, (int)**pstr) == NULL) { 		// find leading character of name
			name = (*pstr)++;
			break;
		}
		if (i == MAX_PAD_CHARS)
            return (STAT_JSON_SYNTAX_ERROR);
	}

	// Find the end of name, NUL terminate and copy token
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(separators, (int)**pstr) != NULL) {
			*(*pstr)++ = NUL;
			strncpy(nv->token, name, TOKEN_LEN+1);			// copy the string to the token
			break;
		}
		if (i == MAX_NAME_CHARS)
            return (STAT_JSON_SYNTAX_ERROR);
	}

	// --- Process value part ---  (organized from most to least frequently encountered)

	// Find the start of the value part
	for (i=0; true; i++, (*pstr)++) {
		if (isalnum((int)**pstr)) break;
		if (strchr(value, (int)**pstr) != NULL) break;
		if (i == MAX_PAD_CHARS)
            return (STAT_JSON_SYNTAX_ERROR);
	}

	// nulls (gets)
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { // process null value
		nv->valuetype = TYPE_NULL;
		nv->value = TYPE_NULL;

	// numbers
	} else if (isdigit(**pstr) || (**pstr == '-')) {// value is a number
		nv->value = (float)strtod(*pstr, &tmp);	// tmp is the end pointer
		if(tmp == *pstr)
            return (STAT_BAD_NUMBER_FORMAT);
		nv->valuetype = TYPE_FLOAT;

	// object parent
	} else if (**pstr == '{') {
		nv->valuetype = TYPE_PARENT;
//		*depth += 1;							// nv_reset_nv() sets the next object's level so this is redundant
		(*pstr)++;
		return(STAT_EAGAIN);					// signal that there is more to parse

	// strings
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;
		nv->valuetype = TYPE_STRING;
		if ((tmp = strchr(*pstr, '\"')) == NULL)
            return (STAT_JSON_SYNTAX_ERROR);    // find the end of the string
		*tmp = NUL;

		// if string begins with 0x it might be data, needs to be at least 3 chars long
		if( strlen(*pstr)>=3 && (*pstr)[0]=='0' && (*pstr)[1]=='x')
		{
			uint32_t *v = (uint32_t*)&nv->value;
			*v = strtoul((const char *)*pstr, 0L, 0);
			nv->valuetype = TYPE_DATA;
		} else {
			ritorno(nv_copy_string(nv, *pstr));
		}

		*pstr = ++tmp;

	// boolean true/false
	} else if (**pstr == 't') {
		nv->valuetype = TYPE_BOOL;
		nv->value = true;
	} else if (**pstr == 'f') {
		nv->valuetype = TYPE_BOOL;
		nv->value = false;

	// arrays
	} else if (**pstr == '[') {
		nv->valuetype = TYPE_ARRAY;
		ritorno(nv_copy_string(nv, *pstr));		// copy array into string for error displays
		return (STAT_UNSUPPORTED_TYPE);	        // return error as the parser doesn't do input arrays yet

	// general error condition
	} else {
        return (STAT_JSON_SYNTAX_ERROR);	    // ill-formed JSON
    }

	// process comma separators and end curlies
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) { // advance to terminator or err out
		return (STAT_JSON_SYNTAX_ERROR);
	}
	if (**pstr == '}') {
		*depth -= 1;							// pop up a nesting level
		(*pstr)++;								// advance to comma or whatever follows
	}
	if (**pstr == ',')
        return (STAT_EAGAIN);                   // signal that there is more to parse

	(*pstr)++;
	return (STAT_OK);							// signal that parsing is complete
}

/****************************************************************************
 * json_serialize() - make a JSON object string from JSON object array
 *
 *	*nv is a pointer to the first element in the nv list to serialize
 *	*out_buf is a pointer to the output string - usually what was the input string
 *	Returns the character count of the resulting string
 *
 * 	Operation:
 *	  - The nvObj list is processed start to finish with no recursion
 *
 *	  - Assume the first object is depth 0 or greater (the opening curly)
 *
 *	  - Assume remaining depths have been set correctly; but might not achieve closure;
 *		e.g. list starts on 0, and ends on 3, in which case provide correct closing curlies
 *
 *	  - Assume there can be multiple, independent, non-contiguous JSON objects at a
 *		given depth value. These are processed correctly - e.g. 0,1,1,0,1,1,0,1,1
 *
 *	  - The list must have a terminating nvObj where nv->nx == NULL.
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

uint16_t json_serialize(nvObj_t *nv, char_t *out_buf, uint16_t size)
{
#ifdef __SILENCE_JSON_RESPONSES
	return (0);
#else
	char_t *str = out_buf;
	char_t *str_max = out_buf + size - BUFFER_MARGIN;
	int8_t initial_depth = nv->depth;
	int8_t prev_depth = 0;
	uint8_t need_a_comma = false;

	*str++ = '{'; 								// write opening curly

	while (true) {
		if (nv->valuetype != TYPE_EMPTY) {
			if (need_a_comma) { *str++ = ',';}
			need_a_comma = true;
			if (js.json_syntax == JSON_SYNTAX_RELAXED) {		// write name
				str += sprintf((char *)str, "%s:", nv->token);
			} else {
				str += sprintf((char *)str, "\"%s\":", nv->token);
			}

			// check for illegal float values
			if (nv->valuetype == TYPE_FLOAT) {
				if (isnan((double)nv->value) || isinf((double)nv->value)) { nv->value = 0;}
			}

			// serialize output value
			if		(nv->valuetype == TYPE_NULL)		{ str += (char_t)sprintf((char *)str, "null");} // Note that that "" is NOT null.
			else if (nv->valuetype == TYPE_INTEGER)	{
				str += (char_t)sprintf((char *)str, "%1.0f", (double)nv->value);
			}
			else if (nv->valuetype == TYPE_DATA)	{
				uint32_t *v = (uint32_t*)&nv->value;
				str += (char_t)sprintf((char *)str, "\"0x%lx\"", *v);
			}
			else if (nv->valuetype == TYPE_STRING)	{ str += (char_t)sprintf((char *)str, "\"%s\"",(char *)*nv->stringp);}
			else if (nv->valuetype == TYPE_ARRAY)	{ str += (char_t)sprintf((char *)str, "[%s]",  (char *)*nv->stringp);}
			else if (nv->valuetype == TYPE_FLOAT)	{ preprocess_float(nv);
//													  str += fntoa((char *)str, nv->value, nv->precision);
													  str += fntoa(str, nv->value, nv->precision);
			}
			else if (nv->valuetype == TYPE_BOOL) {
				if (fp_FALSE(nv->value)) { str += sprintf((char *)str, "false");}
				else { str += (char_t)sprintf((char *)str, "true"); }
			}
			if (nv->valuetype == TYPE_PARENT) {
				*str++ = '{';
				need_a_comma = false;
			}
		}
		if (str >= str_max) { return (-1);}		// signal buffer overrun
		if ((nv = nv->nx) == NULL) { break;}	// end of the list

		while (nv->depth < prev_depth--) {		// iterate the closing curlies
			need_a_comma = true;
			*str++ = '}';
		}
		prev_depth = nv->depth;
	}

	// closing curlies and NEWLINE
	while (prev_depth-- > initial_depth) { *str++ = '}';}
	str += sprintf((char *)str, "}\n");	// using sprintf for this last one ensures a NUL termination
	if (str > out_buf + size) { return (-1);}
	return (str - out_buf);
#endif
}

/*
 * json_print_object() - serialize and print the nvObj array directly (w/o header & footer)
 *
 *	Ignores JSON verbosity settings and everything else - just serializes the list & prints
 *	Useful for reports and other simple output.
 *	Object list should be terminated by nv->nx == NULL
 */
void json_print_object(nvObj_t *nv)
{
#ifdef __SILENCE_JSON_RESPONSES
	return;
#endif

	json_serialize(nv, cs.out_buf, sizeof(cs.out_buf));
	fprintf(stderr, "%s", (char *)cs.out_buf);
}

/*
 * json_print_list() - command to select and produce a JSON formatted output
 */

void json_print_list(stat_t status, uint8_t flags)
{
	switch (flags) {
		case JSON_NO_PRINT: { break; }
		case JSON_OBJECT_FORMAT: { json_print_object(nv_body); break; }
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
 *	This gets a bit complicated. The first nvObj is the header, which must be set by reset_nv_list().
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
	nvObj_t *nv = nv_body;
	if (status == STAT_JSON_SYNTAX_ERROR) {
		nv_reset_nv_list();
		nv_add_string((const char_t *)"err", escape_string(cs.in_buf, cs.saved_buf));

	} else if (cm.machine_state != MACHINE_INITIALIZING) {	// always do full echo during startup
		uint8_t nv_type;
		do {
			if ((nv_type = nv_get_type(nv)) == NV_TYPE_NULL) break;

			if (nv_type == NV_TYPE_GCODE) {
				if (js.echo_json_gcode_block == false) {	// kill command echo if not enabled
					nv->valuetype = TYPE_EMPTY;
				}

//++++		} else if (nv_type == NV_TYPE_CONFIG) {			// kill config echo if not enabled
//fix me		if (js.echo_json_configs == false) {
//					nv->valuetype = TYPE_EMPTY;
//				}

			} else if (nv_type == NV_TYPE_MESSAGE) {		// kill message echo if not enabled
				if (js.echo_json_messages == false) {
					nv->valuetype = TYPE_EMPTY;
				}

			} else if (nv_type == NV_TYPE_LINENUM) {		// kill line number echo if not enabled
				if ((js.echo_json_linenum == false) || (fp_ZERO(nv->value))) { // do not report line# 0
					nv->valuetype = TYPE_EMPTY;
				}
			}
		} while ((nv = nv->nx) != NULL);
	}

	// Footer processing
	while(nv->valuetype != TYPE_EMPTY) {					// find a free nvObj at end of the list...
		if ((nv = nv->nx) == NULL) {						//...or hit the NULL and return w/o a footer
			json_serialize(nv_header, cs.out_buf, sizeof(cs.out_buf));
			return;
		}
	}
	char_t footer_string[NV_FOOTER_LEN];
	sprintf((char *)footer_string, "%d,%d,%d,0", FOOTER_REVISION, status, cs.linelen);
	cs.linelen = 0;											// reset linelen so it's only reported once

	nv_copy_string(nv, footer_string);						// link string to nv object
//	nv->depth = 0;											// footer 'f' is a peer to response 'r' (hard wired to 0)
	nv->depth = js.json_footer_depth;						// 0=footer is peer to response 'r', 1=child of response 'r'
	nv->valuetype = TYPE_ARRAY;
	strcpy(nv->token, "f");									// terminate the list
	nv->nx = NULL;

	// do all this to avoid having to serialize it twice
	int16_t strcount = json_serialize(nv_header, cs.out_buf, sizeof(cs.out_buf));// make JSON string w/o checksum
	if (strcount < 0) { return;}							// encountered an overrun during serialization
	if (strcount > OUTPUT_BUFFER_LEN - MAX_TAIL_LEN) { return;}	// would overrun during checksum generation
	int16_t strcount2 = strcount;
	char tail[MAX_TAIL_LEN];

	while (cs.out_buf[strcount] != '0') { strcount--; }		// find end of checksum
	strcpy(tail, cs.out_buf + strcount + 1);				// save the json termination

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

stat_t json_set_jv(nvObj_t *nv)
{
	if (nv->value > JV_VERBOSE)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	js.json_verbosity = nv->value;

	js.echo_json_footer = false;
	js.echo_json_messages = false;
	js.echo_json_configs = false;
	js.echo_json_linenum = false;
	js.echo_json_gcode_block = false;

	if (nv->value >= JV_FOOTER) 	{ js.echo_json_footer = true;}
	if (nv->value >= JV_MESSAGES)	{ js.echo_json_messages = true;}
	if (nv->value >= JV_CONFIGS)	{ js.echo_json_configs = true;}
	if (nv->value >= JV_LINENUM)	{ js.echo_json_linenum = true;}
	if (nv->value >= JV_VERBOSE)	{ js.echo_json_gcode_block = true;}

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

void js_print_ej(nvObj_t *nv) { text_print_ui8(nv, fmt_ej);}
void js_print_jv(nvObj_t *nv) { text_print_ui8(nv, fmt_jv);}
void js_print_js(nvObj_t *nv) { text_print_ui8(nv, fmt_js);}
void js_print_fs(nvObj_t *nv) { text_print_ui8(nv, fmt_fs);}

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif // __cplusplus
