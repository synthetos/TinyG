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
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "report.h"
#include "util.h"
#include "xio.h"					// for char definitions

/**** Allocation ****/

jsSingleton_t js;

/**** local scope stuff ****/

static stat_t _json_parser_kernal(nvObj_t **nv, char *str);
static stat_t _get_nv_pair(nvObj_t *nv, char **pstr, int8_t *depth);
static void _js_run_container_as_text (nvObj_t *nv, char *str);
static stat_t _normalize_json_string(char *str, uint16_t size);

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
 *    - uses a relaxed parser, but will read either strict or relaxed JSON
 *      For strict-only parser refer to builds earlier than 407.03; Substitute _get_nv_pair_strict() for _get_nv_pair()
 *
 *	Separation of concerns
 *	  json_parser() is the only exposed part. It does parsing, display, and status reports.
 *	  _get_nv_pair() only does parsing and syntax; no semantic validation or group handling
 *	  _json_parser_kernal() does index validation and group handling and executes sets and gets
 *	  in an application agnostic way. It should work for other apps than TinyG
 */

void json_parser(char *str)
{
    nvObj_t *nv = nv_reset_nv_list(NUL);		    // get a fresh nvObj list
	stat_t status = _json_parser_kernal(&nv, str);
	nv_print_list(status, TEXT_NO_PRINT, JSON_RESPONSE_FORMAT);
	sr_request_status_report(SR_REQUEST_ASAP);      // generate incremental status report to show any changes
}

/*
 * _json_parser_execute() - execute one command
 *
 *  Return nv to next free (EMPTY) nv object in list
 */
static nvObj_t *_json_parser_execute(nvObj_t *nv, stat_t *status)
{
    *status = STAT_OK;
    if (nv->valuetype == TYPE_EMPTY) {
        return (nv);
    }
    if (nv->valuetype == TYPE_NULL) {           // GET the value or group
        if ((*status = nv_get(nv)) != STAT_OK) {
            return (NULL);
        }
    } else {
        if (cm.machine_state != MACHINE_ALARM) {// don't execute actions if in ALARM state
            // SET value or call a function (e.g. gcode), then perform conditional persist
            if ((*status = nv_set(nv)) != STAT_OK) {
                return (NULL);
            }
            if ((*status = nv_persist(nv)) != STAT_OK) {
                return (NULL);
            }
        }
	}
    while (nv->nx != NULL) {                    // find and return next free NV buffer in list
        if (nv->valuetype == TYPE_EMPTY) {
            return (nv);
        }
        nv= nv->nx;
    }
    return (NULL);
}

/* _json_parser_kernal()
 *
 *  A (one level) recursively callable "inner loop" for the parser. Will parse
 *  whatever string is provided (*str) into the nv object provided (**nv).
 *  It may consume multiple nv objects if the parse expands into a group (e.g. "x")
 *  Leaves nv pointing to the first free (EMPTY) nv object past those filled during
 *  the parse. Since nv can be manipulated at either recursion level the nv pointer
 *  is passed by reference.
 */

static stat_t _json_parser_kernal(nvObj_t **nv, char *str)
{
	stat_t status = STAT_OK;
	int8_t depth = 1;
	char group[GROUP_LEN+1] = { NUL };              // group identifier - starts as NUL
    nvObj_t *nv_exec;                               // nv pair on which to start execution

	ritorno(_normalize_json_string(str, JSON_OUTPUT_STRING_MAX)); // return if error

	//---- parse the JSON string into the nv list ----//

    do { // read the character string
        nv_exec = *nv;

        // decode the next JSON name/value pair
        // returns if error or STAT_COMPLETE
		if ((status = _get_nv_pair(*nv, &str, &depth)) > STAT_NOOP) {
			return (status);                                    // error return
        }
        if (status == STAT_NOOP) {                              // normal return
            return (STAT_OK);
        }

        // handle PARENTs
        if ((*nv)->valuetype == TYPE_PARENT) {
		    if ((nv_index_is_group((*nv)->index)) && (nv_group_is_prefixed((*nv)->token))) {
    		    strncpy(group, (*nv)->token, GROUP_LEN);        // capture the group ID
		    }
            do {
                *nv = (*nv)->nx;
                status = _get_nv_pair(*nv, &str, &depth);       // parse the child objects
                if (status == STAT_NOOP) {
                    break;
                }
		        if (*group != NUL) {
    		        strncpy((*nv)->group, group, GROUP_LEN);    // copy the parent's group to this child
		        }
            }
            while ((*nv)->nx != NULL);

        // handle all other TYPEs
        } else {

            // Skip over TIDs
            if ((*nv)->index == nvl.tid_index) {
                (*nv)->valuetype = TYPE_TID;
            }

            // test and process text container ("txt" key)
            if (nv_index_is_container((*nv)->index)) {
                if (strncmp("txt", *(*nv)->stringp, 3) == 0) { // don't allow nested txt containers
                    return (STAT_NESTED_JSON_CONTAINER);
                } else if (*(*(*nv)->stringp) == '{') {        // it's JSON passed in as a string
                    _json_parser_kernal(nv, *(*nv)->stringp);  // call JSON parser recursively
                } else {
                    _js_run_container_as_text(*nv, *(*nv)->stringp); // it's a text command
                }
                continue;
            }
        }
        *nv = _json_parser_execute(nv_exec, &status);           // execute the list from current starting point
        if (status != STAT_OK) {
            return (status);
        }

    } while ((*nv)->nx != NULL);                                // this test is a safety valve

    return (STAT_OK);
}

/*
 * _js_run_container_as_text - callout from JSON kernal to run text commands
 *
 *	For text-mode commands this starts a JSON response then runs the text command
 *  in a 'msg' element. Text lines are terminated with JSON-friendly line ends
 *  (e,g \n instead of LF). The text response string is closed, then
 *  json_continuation is set so that the JSON response is properly handled.
 *
 *  Gcode is simply wrapped in a JSON gc tag and processed.
 */

static void _js_run_container_as_text (nvObj_t *nv, char *str)
{
    // process pure text-mode commands
    if (strchr("$?Hh", *str) != NULL) {             // a match indicates text mode
        if (js.json_syntax == JSON_SYNTAX_RELAXED) {// opening JSON
            printf_P(PSTR("{r:{txt:\""));
        } else {
            printf_P(PSTR("{\"r\":{\"txt\":\""));
        }
        cs.comm_mode = JSON_MODE_TXT_OVERRIDE;      // override JSON mode for this output only
        text_parser(str);
        cs.comm_mode = JSON_MODE;                   // restore JSON mode
        printf_P(PSTR("\""));                       // close quote
        nv_reset_nv_list(NUL);                      // reset the list to start at the head
        NV_HEAD->valuetype = TYPE_TXT_CONTINUATION; // label the list as a text continuation

    // process gcode
    } else {
        strcpy(nv->token,"gc");
        text_response(gc_gcode_parser(str), cs.saved_buf);
    }
}

/*
 * _normalize_json_string - normalize a JSON string in place
 *
 *	Validate string size limits, remove all whitespace and convert
 *	to lower case, with the exception of gcode comments
 */

static stat_t _normalize_json_string(char *str, uint16_t size)
{
	char *wr;								// write pointer
	bool in_comment = false;
    bool in_front = true;                   // leading characters

	if (strlen(str) > size) {
        return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
    }
	for (wr = str; *str != NUL; str++) {
        if ((*str == 0xE2) && (*(str+1) == 0x80)) { // replace "smart" quotes in all strings
            str += 2;
            if ((*str == 0x9C) || (*str == 0x9D)) {
                *wr++ = '\"';
                continue;
            }
        }
		if (!in_comment) {					// normal processing
			if (*str == '(') {
                in_comment = true;
            }
            if (in_front) {                 // toss leading ctrls, WS & DEL
    			if ((*str <= ' ') || (*str == DEL)) {
                    continue;
                }
            }
            in_front = false;
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
 * _get_nv_pair() - get next name-value pair w/relaxed JSON rules. Also parses strict JSON.
 *
 *	Parse the next statement and populate the command object (nvObj) with:
 *    - nv->token
 *    - nv->index       - validates name token in the process)
 *    - nv->valuetype   - can be any type depending on JSON input
 *    - nv->value_int or nv->value_flt
 *
 *	Leaves string pointer (str) on the first character following the object, which
 *	is the character just past the ',' separator if it's a multi-valued object or
 *  the terminating NUL if single object or the last in a multi.
 *
 *	Keeps track of tree depth and closing braces as much as it has to. If this were
 *  to be extended to track multiple parents or more than two levels deep it would
 *  have to track closing curlies - which it does not.
 *
 *	ASSUMES INPUT STRING HAS FIRST BEEN NORMALIZED BY _normalize_json_string(),
 *  i.e. it can have no leading whitespace, and embedded spaces have been removed
 *  except for within strings.
 *
 *	If a group prefix is passed in it will be pre-pended to any name parsed to form a
 *  token string. For example, if "x" is provided as a group and "fr" is found in
 *  the name string the parser will search for "xfr" in the cfgArray.
 *
 *  RETURNS:
 *    - STAT_EAGAIN     a valid NV pair was returned and there is more to parse
 *    - STAT_OK         a valid NV pair was returned and there is no more to parse
 *    - STAT_NOOP       there was nothing to parse
 *    - STAT_xxxx       a parsing error occurred
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
#define MAX_STRING_CHARS RX_BUFFER_MIN_SIZE // inherit the size of the input buffer

static stat_t _get_nv_pair(nvObj_t *nv, char **pstr, int8_t *depth)
{
	uint8_t i;
	char *end;
	char leaders[] =     {  "{,\""    };        // open curly, comma, quote
	char separators[] =  {  ":\""     };        // colon, quote
	char terminators[] = {  ",}"      };        // comma, close curly
	char value[] =       {  "{.-+\""  };        // open curly, period, minus, plus, quote

    //--- Test for end of data - indicated by garbage or a null ---//
//	if ((**pstr == NUL) || (strchr(leaders, (int)**pstr) == NULL)) {
	if (**pstr == NUL) {
        return (STAT_NOOP);
    }
	nv_reset_nv(nv);							// wipe the object
    nv->depth = *depth;                         // set its depth

	// --- Process name part ---
	// Find, terminate and set pointers for the name. Allow for leading and trailing name quotes.
	char *name = *pstr;
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(leaders, (int)**pstr) == NULL) { // find leading character of name
			name = (*pstr)++;
			break;
		}
		if (*name == NUL) {
    		return (STAT_NOOP);
		}
		if (i == MAX_PAD_CHARS) {
            return (STAT_JSON_SYNTAX_ERROR);
        }
	}

	// Find the end of name, NUL terminate and copy token
	for (i=0; true; i++, (*pstr)++) {
		if (strchr(separators, (int)**pstr) != NULL) {
			*(*pstr)++ = NUL;
			strncpy(nv->token, name, TOKEN_LEN+1);  // copy the string to the token
			break;
		}
		if (i == MAX_NAME_CHARS) {
            return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
        }
	}

    // Validate the name token and set index
	if ((nv->index = nv_get_index((const char *)"", nv->token)) == NO_MATCH) { // get index or fail it
    	return (STAT_UNRECOGNIZED_NAME);
	}

	// --- Process value part ---  (organized from most to least frequently encountered)

	// Find the start of the value part
	for (i=0; true; i++, (*pstr)++) {
		if (isalnum((int)**pstr)) { break; }
		if (strchr(value, (int)**pstr) != NULL) { break; }
		if (i == MAX_PAD_CHARS) {
            return (STAT_INVALID_OR_MALFORMED_COMMAND);
        }
	}

	// nulls (gets)
	if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) { // process null value
		nv->valuetype = TYPE_NULL;

	// numbers
	} else if (isdigit(**pstr) || (**pstr == '-')) {    // value is a number
	    if (GET_TABLE_BYTE(flags) & F_FLOAT) {          // treat value as float
		    nv->value_flt = (float)strtod(*pstr, &end); // 'end' should get set
		    if (end == *pstr) {
    		    return (STAT_BAD_NUMBER_FORMAT);
		    }
		    nv->valuetype = TYPE_FLOAT;
        } else {                                        // treat value as integer
    	    nv->value_int = atol(*pstr);
    	    nv->valuetype = TYPE_INTEGER;
        }

	// object parent
	} else if (**pstr == '{') {
		nv->valuetype = TYPE_PARENT;
		*depth += 1;
		(*pstr)++;
		return(STAT_EAGAIN);					// signal that there is more to parse

	// strings
	} else if (**pstr == '\"') { 				// value is a string
		(*pstr)++;                              // advance past quote to first character
		nv->valuetype = TYPE_STRING;

        // find the closing quote while un-escaping embedded quotes
        char *rd = (*pstr);                     // read pointer for copy to stringp (on 1st char)
        char *wr = (*pstr);                     // write pointer for string manipulation
	    for (i=0; true; i++, (*pstr)++, wr++) {
    	    if (i == MAX_STRING_CHARS) {        // NOT REQUIRED. Assume string has a closing quote
        	    return (STAT_JSON_TOO_LONG);
    	    }
            if ((*(*pstr) == '\\') && *((*pstr)+1) == '\"') { // escaped quote
                *wr = '\"';
                (*pstr)++;
                continue;
            }
            if (*(*pstr) == '\"') {
                *wr = NUL;
                (*pstr)++;
                break;
            }
            *wr = *(*pstr);
	    }

		// if string begins with 0x it might be data, needs to be at least 3 chars long
		if (strlen(*pstr)>=3 && (*pstr)[0]=='0' && (*pstr)[1]=='x') {
			uint32_t *v = (uint32_t*)&nv->value_flt;
			*v = strtoul((const char *)*pstr, 0L, 0);
			nv->valuetype = TYPE_DATA;
		} else {
			ritorno(nv_copy_string(nv, rd));
		}

	// boolean true/false
	} else if (**pstr == 't') {
		nv->valuetype = TYPE_BOOL;
		nv->value_int = true;
	} else if (**pstr == 'f') {
		nv->valuetype = TYPE_BOOL;
		nv->value_int = false;

	// arrays (not supported on input)
	} else if (**pstr == '[') {
		nv->valuetype = TYPE_ARRAY;
		ritorno(nv_copy_string(nv, *pstr));		// copy array into string for error displays
		return (STAT_UNSUPPORTED_TYPE);	        // return error as the parser doesn't do input arrays yet

	// general error condition
	} else {
        return (STAT_JSON_SYNTAX_ERROR);	    // ill-formed JSON
    }

	// advance past any remaining chars in the value to the terminator - or err out
	if ((*pstr = strpbrk(*pstr, terminators)) == NULL) {
		return (STAT_JSON_SYNTAX_ERROR);
	}

	// process closing curlies and adjust the depth variable
    while (**pstr != NUL) {
    	if ((**pstr <= ' ') || (**pstr == DEL)) { // toss whitespace characters
            (*pstr)++;
            continue;
        }
	    if (**pstr == '}') {
		    *depth -= 1;                        // pop up a nesting level
		    (*pstr)++;
            continue;
	    }
	    if (**pstr == ',') {                    // return **pstr on the comma
            return (STAT_EAGAIN);               // signal that there is more to parse
        }
    }
    // signal last pair; parsing is complete. Return **pstr on the NUL terminator
	return (STAT_OK);
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
 *    - Objects of TYPE_EMPTY or TYPE_SKIP are passed over (ignored)
 *
 *	  - If a JSON object has no data it's represented as {}
 *
 *	  - The list must have a terminating nvObj where nv->nx == NULL.
 *		The terminating object may or may not have data (empty or not empty).
 *
 *	Returns:
 *	   -  length of string (size)
 *     -  0 if string is empty
 *     - -1 if string overflowed buffer
 */

int16_t json_serialize(nvObj_t *nv, char *out_buf, int16_t out_size)
{
    char *str = out_buf;
    char *out_max = out_buf + out_size - 16;        // 16 is a pad to allow value expansion & closing, but ...
                                                    //...does not need to account for strings which are sized separately

    if ((nv = nv_relink_nv_list()) == NULL) {       // remove TYPE_EMPTY and TYPE_SKIP pairs
        *str = NUL;
        return (0);
    }
    int8_t initial_depth = nv->depth;
    int8_t prev_depth = initial_depth - 1;          // forces open curly to write

    // JSON text continuation special handling.
    // See _json_parser_kernal() / _js_run_container_as_text()
    if (nv->valuetype == TYPE_TXT_CONTINUATION) {
        nv = nv->nx;                                // skip over head pair
        prev_depth = 1;                             // text continuations are always at depth 1
    } else {
        *str++ = '{';                               // first curly for new JSON object
    }

    // Serialize the list - Note: nv points to opening r{} or first usable object past continuation text
    do {
        // opening and closing curlies, commas between elements
        if (nv->pv->valuetype == TYPE_PARENT) {     // cases where previous NV was a parent
            if (nv->depth <= prev_depth) {
                while (nv->depth <= prev_depth--) {
                    *str++ = '}';                   // terminate parent with no children --> {}
                }
                *str++ = ',';                       // continuing comma for current NV pair
            }
        } else if (nv->depth < prev_depth) {        // terminating a parent through 1 or more levels
            while (nv->depth < prev_depth--) {
                *str++ = '}';                       // closing curly(s) for previous NV pair (might be the last)
            }
            *str++ = ',';                           // leading comma for current NV pair
        } else if (nv->depth == prev_depth) {
            *str++ = ',';                           // leading comma for current NV pair
        }

        // serialize name
		if (js.json_syntax == JSON_SYNTAX_RELAXED) { // write name
			str += sprintf_P((char *)str, PSTR("%s:"), nv->token);
		} else {
			str += sprintf_P((char *)str, PSTR("\"%s\":"), nv->token);
		}

		// serialize value
		if (nv->valuetype == TYPE_PARENT) {
            *str++ = '{';
        }
		else if (nv->valuetype == TYPE_NULL)	{
            str += sprintf_P(str, PSTR("null"));    // Note that that "" is NOT null.
        }
		else if (nv->valuetype == TYPE_INTEGER)	{
			str += sprintf_P(str, PSTR("%lu"), nv->value_int);
		}
		else if (nv->valuetype == TYPE_DATA) {
			uint32_t *v = (uint32_t*)&nv->value_flt;
			str += sprintf_P(str, PSTR("\"0x%lx\""), *v);
		}
		else if (nv->valuetype == TYPE_STRING) {
            if ((str + strlen(*nv->stringp)) >= out_max) {
                return (-1);
            }
            str += sprintf_P(str, PSTR("\"%s\""), *nv->stringp);
        }
		else if (nv->valuetype == TYPE_ARRAY) {
            str += sprintf_P(str, PSTR("[%s]"), *nv->stringp);
        }
		else if (nv->valuetype == TYPE_FLOAT) {
            preprocess_float(nv);
			str += fntoa(str, nv->value_flt, nv->precision);
		}
		else if (nv->valuetype == TYPE_BOOL) {
			if (nv->value_int == true) {
                str += sprintf_P(str, PSTR("true"));
			} else {
                str += sprintf_P(str, PSTR("false"));
            }
		}
		if (str >= out_max) { return (-1);}         // test for buffer overrun
        prev_depth = nv->depth;
	} while ((nv = nv->nx) != NULL);

    // finish up string
	if ((str + prev_depth - initial_depth + 2) > (out_buf + out_size)) { // test for space
        return (-1);
    }
    while (prev_depth-- > initial_depth) {	        // write closing curlies
        *str++ = '}';
    }
	str += sprintf(str, "}\n");	                    // sprintf ensures a NUL termination
	return (str - out_buf);
}

/*
 * json_print_list() - command to select and produce a JSON formatted output
 */

void json_print_list(stat_t status, uint8_t flags)
{
	switch (flags) {
		case JSON_NO_PRINT: { break; }
		case JSON_OBJECT_FORMAT: { json_print_object(nvl.list); break; }
		case JSON_RESPONSE_FORMAT: { json_print_response(status); break; }
	}
}

/*
 * json_print_object() - serialize and print the nvObj array directly (w/o header & footer)
 *
 *	Ignores JSON verbosity settings and everything else - just serializes the list & prints
 *	Useful for reports and other simple output.
 *	Object list must be terminated by nv->nx == NULL
 */
void json_print_object(nvObj_t *nv)
{
	if (json_serialize(nv, cs.out_buf, sizeof(cs.out_buf)) > 0) {
        printf(cs.out_buf);
    }
    // fails silently on 0 or -1 return. May want to revisit this.
}

/*
 * json_print_response() - JSON responses with headers, footers and observing JSON verbosity
 *
 *	r{} header is returned for every setting except $jv=0
 *	f{} footer is returned for every setting except $jv=0
 *
 *	JV_SILENT = 0,	// no response is provided for any command
 *	JV_FOOTER,		// responses contain footer only; no command echo, gcode blocks or messages
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
 *
 *  NV types labeled as TYPE_EMPTY signal the end of the incoming NV list
 */
#define MAX_TAIL_LEN 8

void json_print_response(uint8_t status)
{
	if (js.json_verbosity == JV_SILENT) {               // silent responses
        return;
    }
	if (status == STAT_NO_DISPLAY) {                    // upstream function has already performed output
    	return;
	}

	// Setup the response header
	nvObj_t *nv = NV_HEAD;
    if (nv->valuetype != TYPE_TXT_CONTINUATION) {       // include r{} if not a continuation
	    nv->valuetype = TYPE_PARENT;
	    strcpy(nv->token, "r");
    }

	// Body processing
	nv = NV_BODY;
	if (status == STAT_JSON_SYNTAX_ERROR) {
		nv_reset_nv_list(NUL);
	    nv->valuetype = TYPE_PARENT;
	    strcpy(nv->token, "r");
		nv_add_string((const char *)"err", escape_string(cs.bufp, cs.saved_buf));

    // Filter the NV pairs down to the JSON verbosity reporting level
	} else if (cm.machine_state != MACHINE_INITIALIZING) {	// always do full echo during startup
		nvType nv_type;
		do {
            if (nv->valuetype == TYPE_EMPTY) {		        // find end of list
                break;
            }
            if (nv->valuetype == TYPE_TID) {		        // process transaction IDs
            	cs.txn_id = nv->value_int;
            	nv->valuetype = TYPE_SKIP;
                continue;
        	}

            // filter output according to JV setting
			nv_type = nv_get_type(nv);
            if (nv_type == NV_TYPE_REPORT) {                // always display all of status, queue and exception reports
                break;
			} else if (nv_type == NV_TYPE_GCODE) {
				if (!js.echo_json_gcode_block) {	        // skip command echo if not enabled
					nv->valuetype = TYPE_SKIP;
				}
			} else if (nv_type == NV_TYPE_LINENUM) {		// skip line number echo if not enabled
				if ((js.echo_json_linenum == false) || (nv->value_int == 0)) { // do not report line# 0
					nv->valuetype = TYPE_SKIP;
				}
            } else if (nv_type == NV_TYPE_CONFIG) {			// skip config echo if not enabled
		        if (js.echo_json_configs == false) {
					nv->valuetype = TYPE_SKIP;
				}
			} else if (nv_type == NV_TYPE_MESSAGE) {		// skip message echo if not enabled
			    if (js.echo_json_messages == false) {
    			    nv->valuetype = TYPE_SKIP;
			    }
            }
		} while ((nv = nv->nx) != NULL);
	}

    // Add a transaction ID if one was present in the request
    if (cs.txn_id != 0) {
        nv = NV_TID;
        nv->value_int = cs.txn_id;
        strcpy(nv->token, "tid");
        nv->valuetype = TYPE_INTEGER;
        nv->depth = 0;                                      // always a top-level object
    }

    // Add the footer
    nv = NV_FOOT;
	char footer_string[NV_FOOTER_LEN];

    if (xio.rx_mode == RX_MODE_CHAR) {                      // character style footer
        sprintf(footer_string, "2,%d,%d", status, cs.linelen);//...character mode
        cs.linelen = 0;										// reset linelen so it's only reported once
    } else {                                                // line_mode style footer
        sprintf(footer_string, "3,%d,%d", status, xio_get_line_buffers_available());
    }

	nv_copy_string(nv, footer_string);						// link string to nv object
	nv->depth = 0;											// footer is a peer to r{} response
	nv->valuetype = TYPE_ARRAY;
	strcpy(nv->token, "f");									// terminate the list

	// serialize the JSON response and print it if there were no errors
	if (json_serialize(NV_HEAD, cs.out_buf, sizeof(cs.out_buf)) > 0) {
    	printf(cs.out_buf);
	}
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
	if (nv->value_int > JV_VERBOSE) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	js.json_verbosity = nv->value_int;

	js.echo_json_footer = false;
	js.echo_json_messages = false;
	js.echo_json_configs = false;
	js.echo_json_linenum = false;
	js.echo_json_gcode_block = false;

	if (nv->value_int >= JV_FOOTER) 	{ js.echo_json_footer = true;}
	if (nv->value_int >= JV_MESSAGES)	{ js.echo_json_messages = true;}
	if (nv->value_int >= JV_CONFIGS)	{ js.echo_json_configs = true;}
	if (nv->value_int >= JV_LINENUM)	{ js.echo_json_linenum = true;}
	if (nv->value_int >= JV_VERBOSE)	{ js.echo_json_gcode_block = true;}

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

void js_print_ej(nvObj_t *nv) { text_print(nv, fmt_ej);}
void js_print_jv(nvObj_t *nv) { text_print(nv, fmt_jv);}
void js_print_js(nvObj_t *nv) { text_print(nv, fmt_js);}
void js_print_fs(nvObj_t *nv) { text_print(nv, fmt_fs);}

#endif // __TEXT_MODE
