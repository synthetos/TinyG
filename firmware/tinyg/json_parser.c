/*
 * json_parser.c - JSON parser for TinyG
 * This file is part of the TinyG project
 *
 * Copyright (c) 2011 - 2016 Alden S. Hart, Jr.
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
#include "canonical_machine.h"
#include "report.h"
#include "util.h"
#include "xio.h"                    // for char definitions

// Notes
//  replace static sprintf's in serialize w/strcat_P
//      pre-populate 'str' value with pointer to the actual strings?
// Should we trap NAN of INF float getting to json_print_response()?
// Change text_parser number handling to be more like json_parset (strtol, error checking...)

/**** Allocation ****/

jsSingleton_t js;

/**** local scope stuff ****/

static stat_t _json_parser_kernal(nvObj_t **p_nv, char *str);
static stat_t _tokenize_json_string(char *str, uint16_t size, uint8_t pairs);
static stat_t _marshall_nv_pair(nvObj_t **nv, char **p_str, char *group, uint8_t depth, bool typesafe);

/****************************************************************************
 * json_parser()          - exposed part of JSON parser
 * _json_parser_kernal()  - inner kernal function (recursable)
 *
 *	This is a dumbed down JSON parser to fit in limited memory with no malloc
 *  and limited recursion.
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
 *    - only supports 7 bit ASCII. (Smart quotes can be sent but are replaced)
 *	  - extracts an array of one or more JSON object structs from the input string
 *	  - once the array is built it executes the object(s) in order in the array
 *	  - passes the executed array to the response handler to generate the response string
 *	  - returns the status and the JSON response string
 *    - uses a relaxed parser, but will read either strict or relaxed JSON
 *
 *	Separation of concerns
 *	  json_parser() is the only exposed part. It does parsing, display, and status reports.
 *	  _json_parser_kernal() is the json_parser inner function
 *    _tokenize_json_string() is application agnostic JSON validation, normalization & tokenization
 *	  _marshall_json_to_nv() is application-aware marshalling from the tokenized string
 */

//void json_parser(char *str, nvObj_t *nv, bool display)
void json_parser(char *str)
{
    nvObj_t *nv = nv_reset_nv_list(NUL);		    // get a fresh nvObj list
	stat_t status = _json_parser_kernal(&nv, str);
//    if (display) {
        json_print_response(status);
//    }
	sr_request_status_report(SR_REQUEST_ASAP);      // generate incremental status report to show any changes
}

/* _json_parser_kernal()
 *
 *  Recursively callable "inner function" for the parser. Will parse string (*str)
 *  into the nv object list provided (**nv). It may consume multiple nv objects
 *  if the parse expands into a group (e.g. "x").
 *
 *  Leaves nv pointing to the first free (EMPTY) nv object past those filled during
 *  the parse. Since nv can be manipulated at either recursion level the nv pointer
 *  is passed by reference.
 *
 *  "txt" keys (text containers) are handled as a special case. A TXT key will execute
 *  the string value as a text-mode command by invoking the text parser, or will
 *  recursively call the JSON parser if the wrapped value is a JSON string. The response
 *  from the text-mode command is wrapped in JSON and properly escaped so that the
 *  response is a valid JSON object.
 *
 *  TXT restrictions are:
 *    - A TXT JSON line cannot have any other elements, excepting TID
 *    - TXT commands cannot be nested
 */

static stat_t _json_parser_kernal(nvObj_t **p_nv, char *str)
{
	char group[GROUP_LEN+1] = "";   // group identifier - initial value must be provided
    nvObj_t *nv;                    // NV _on_which_to_start_execution

    //---- do an application agnostic pre-parse of JSON string ----//
	ritorno(_tokenize_json_string(str, JSON_INPUT_STRING_MAX, JSON_PAIRS_MAX)); // return if exception

	//---- parse the JSON string into the nv list one executable NV at a time ----//
    do {
        if (*str == NUL) {
            return (STAT_OK);   // normal return if no (more) tokenized JSON to process
        }
        nv = *p_nv;             // set starting NV pair for execution
        ritorno(_marshall_nv_pair(p_nv, &str, group, 0, true)); // collect NV pair from JSON string

        // Special processing for text container (TXT tag)
        if ((*p_nv)->valuetype == TYPE_TXTCON) {
            if (strncmp("txt", (*p_nv)->str, 3) == 0) {     // don't allow nested txt containers
                return (STAT_NESTED_TXT_CONTAINER);
            } else if (*(*p_nv)->str == '{') {              // it's JSON passed in as a string
                _json_parser_kernal(p_nv, (*p_nv)->str);    // call JSON parser recursively
            } else {
                controller_dispatch_txt_container((*p_nv), (*p_nv)->str); // call specialized dispatcher
            }
            continue;
        }

        // Execute starting with NV that was set earlier
        if (nv->valuetype == TYPE_NULL) {                   // GET the value or group
            ritorno(nv_get(nv));                            // return if exception
        } else {
            if (!cm_is_alarmed()) {                         // don't execute actions if in ALARM state
                ritorno(nv_set(nv));
                if (nv->valuetype != TYPE_PARENT) {         // group SETs (parents) perform persistence
                    ritorno(nv_persist(nv));                // as part of group iteration so don't need
                }                                           // to persist here.
            }                                               // See set_grp() if this makes no sense
        }
    } while ((*p_nv = nv_next_empty(*p_nv)) != NULL);       // skip past any NV's that have been filled
    return (STAT_OK);
}

/*
 * _tokenize_json_string - normalize, validate and tokenize a JSON string in place
 *
 *  Isolates fields in JSON by terminating each field with a token containing data
 *  about that field. Runs as a state machine defined as jsonTokenizationState.
 *  Tokenization does not increase the size of the string.
 *
 *  Normalization and validation functions:
 *	  - validate string size limits
 *    - force all input to 7 bit ASCII
 *    - strip quotes from keys (convert to relaxed JSON mode)
 *    - convert keys to lower case
 *    - remove whitespace except in string values
 *
 *  Tokens are single characters inserted in-place as separators.
 *    - 1000 0000        MSB = 1 indicates it's a token
 *    - 0111 0000        3 bits for JSON nesting depth. 0 is none. 1 is root nesting level
 *    - 0000 1000        1 marks key, 0 marks value
 *    - 0000 0100        1 marks array value (modal to above bit)
 *    - 0000 0011        2 bits for value type, as per:
 *      TYPE_NULL = 0    value is NULL
 *      TYPE_BOOL = 1    value is boolean
 *      TYPE_NUMBER = 2  value is number. May be integer or float
 *      TYPE_STRING = 3  value is string
 */

// Position string pointer past whitespace
static char *_skip_whitespace(char *str)
{
    do {
        if (((*str > NUL) && (*str <= SPC)) || (*str == DEL)) {
            str++;
        } else {
            return (str);
        }
    } while (*str != NUL);      // safety valve
    return (NUL);
}

// Position string pointer past quote and any trailing whitespace
static char *_skip_quote(char *str)
{
    if (*str == '"') {
        str++;
        str = _skip_whitespace(str);
    }
    return (str);
}

// Return the value type by examining the first (and maybe second) character
// Requires char under *s pointer to be lowercased first.
// Traps "" (null string) as null. Note that "  " is a valid string with spaces, not a null.
static char _value_type(char *s)
{
	if       (*s == 'n')                    { return (TYPE_NULL); }
    else if ((*s == 't') || (*s == 'f'))    { return (TYPE_BOOLEAN); }
    else if ((*s == '-') || isdigit(*s))    { return (TYPE_INTEGER); }  // provisionally INT. Might be FLOAT
    else if  (*s == '{')                    { return (TYPE_NEW_LEVEL); }
    else if  (*s == '"') {
        if (*(s+1) == '"')                  { return (TYPE_NULL_STRING); }
        else                                { return (TYPE_STRING); }
    }
	return (TYPE_TYPE_ERROR);               // none of the above
}

static stat_t _tokenize_json_string(char *str, uint16_t size, uint8_t pairs)
{
    str_asciify(str);                               // remove or replace any 8-bit chars

	if (strlen(str) > size) {
    	return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
	}
	char *wr = str;                                 // write pointer
    stat_t status = STAT_JSON_SYNTAX_ERROR;         // so pessimistic!
    jsonTokenizationState state = JTSM_START;       // state machine state
    uint8_t jtoken = 0;                             // parking place for JSON token
    uint8_t depth = 0;                              // nesting depth

    do {
        // STRING_VALUE state                       // must be first to avoid WS removal
        if (state == JTSM_STRING_VALUE) {
            if (*str == NUL) { break; }             // exit with error
            if (*str == '\\') {                     // skip over escaped quotes (unescape them later)
                if (*(str+1) == '"') { str++; }
                continue;
            }
            if (*str == '"') {                      // a lone quote is the end quote
                state = JTSM_VALUE;                 // use the VALUE state logic to exit
            }
            *wr++ = *str;
            continue;
        }

        // skip over white space for VALUE, START & KEY states
        str = _skip_whitespace(str);

        // START state
        if (state == JTSM_START) {
            if (*str == '{') {
                if (*(_skip_whitespace(str+1)) == ':') break; // trap missing key field
                depth = 1;
                state = JTSM_KEY; jtoken = JTOK_FLAG | depth << JTOK_DSHIFT | JTOK_KEY;
                continue;
            }
            else { break; }                         // any char other than { is an error
        }

        // KEY state
        else if (state == JTSM_KEY) {
            str = _skip_quote(str);                 // skip past opening or closing quotes + WS
            if (*str == '}') {                      // up a level
                depth--;
                *wr++ = jtoken;                     // termination
                state = JTSM_KEY; jtoken = NUL;     // NUL prevents multiple tokens for nested values
                continue;
            }
            else if (*str == NUL) {                  // normal or abnormal termination
                if (depth == 0) { status = STAT_OK;} // normal
                break;                               // abnormal (exit either STAT_OK or with error)
            }
            if (*str == ':') {
                *wr++ = jtoken;                     // record previous token
                str = _skip_whitespace(++str);      // advance to 1st char of value field
                *str = tolower(*str);
                valueType type = _value_type(str);  // what does 1st value character tell us?
                if (type == TYPE_NEW_LEVEL) {
                    if (*(str+1) == ':') { break; } // trap missing key field
                    if (++depth > JSON_DEPTH_MAX) {
                        return (STAT_MAX_DEPTH_EXCEEDED);
                    }
                    state = JTSM_KEY; jtoken = JTOK_FLAG | (depth << JTOK_DSHIFT) | JTOK_KEY;
                    --wr; *wr++ |= JTOK_PARENT;     // back annotate the previous token
                    continue;
                }
                else if (type == TYPE_NULL_STRING) {
                    jtoken = JTOK_FLAG | (depth << JTOK_DSHIFT) | TYPE_NULL;
                    *wr++ = 'n'; str++;             // write 'n(ull)' & advance past closing " char
                    continue;
                }
                else if (type == TYPE_STRING) {
                    state = JTSM_STRING_VALUE; jtoken = JTOK_FLAG | (depth << JTOK_DSHIFT) | type;
                }
                else if (type == TYPE_TYPE_ERROR) { break; }
                else {  // if NULL, BOOL, NUMBER. All value validation occurs during marshalling
                    state = JTSM_VALUE; jtoken = JTOK_FLAG | (depth << JTOK_DSHIFT) | type;
                }
            }
        }

        // VALUE state
        else if (state == JTSM_VALUE) {
            if (*str == ',') {                      // new key at same depth
                *wr++ = jtoken;                     // record previous token
                state = JTSM_KEY; jtoken = JTOK_FLAG | (depth << JTOK_DSHIFT) | JTOK_KEY;
                if (--pairs <= 0) { return (STAT_JSON_TOO_MANY_PAIRS); }
                continue;
            }
            else if (*str == '}') {                 // end value, up a level
                depth--;
                *wr++ = jtoken;                     // record previous token
                state = JTSM_KEY; jtoken = NUL;     // NUL prevents multiple tokens for nested values
                if (--pairs <= 0) { return (STAT_JSON_TOO_MANY_PAIRS); }
                continue;
            }
            else if (*str == NUL) { break; }        // exit with error
        }
//        else if (state == JTSM_ARRAY_VALUE) {
//            if      (*str == '"') { state = JSN_STRING_VALUE; }
//            else if (*str == ',') { state = JSN_KEY; }           // new key
//            else if (*str == '{') { state = JSN_KEY; depth++; }  // is parent
//            else if (*str == '}') { state = JSN_KEY; depth--; }  // end value, up a level
//            else if (*str == NUL) { break; }                     // exit with error
//        }
        *wr++ = tolower(*str);
    } while (str++ != NUL); // This is a safety value. Ordinarily exits on its own
    *wr++ = NUL;            // always terminate. Regardless.
    *wr = NUL;              // and your little dog, too. (used to terminate marshalling)
	return (status);
}

/*
 * _marshall_nv_pair() - recursively get next name-value pair using tokenized JSON string
 *
 *	Requires that the JSON input string has been processed by _tokenize_json_string().
 *
 *  p_str is a pointer to a pointer to a tokenized JSON input string.
 *  p_str must start at the first char of a key (or to whitespace preceding).
 *	Leaves p_str on the first character following the last nv pair (which might be a NUL).
 *
 *  p_nv is a pointer to a pointer to the first NV pair to fill. On return it points to the
 *  next available NV pair in the list - allowing this function to be called recursively.
 */

// Return pointer to next token or NULL if none found
static char *_next_token(char *str)
{
    do {
        if (*str & JTOK_FLAG) {
            return (str);
        }
    } while (str++, *str != NUL);
    return (NULL);
}

static stat_t _marshall_nv_pair(nvObj_t **p_nv, char **p_str, char *group, uint8_t parent_depth, bool typesafe)
{
    nvObj_t *nv = *p_nv;            // local pointer for convenience and easier code inspection
    char *tok = *p_str;             // local pointer to token location
    char *str = *p_str;             // local pointer to string start(s)
    uint8_t jtoken;                 // saved JSON token marker
    uint8_t depth;
    valueType cfg_type;

    //---- Collect NV pair or recursively call marshalling if parent ----//
    tok = _next_token(str);
    jtoken = (tok) ? *tok : 0;                  // collect JSON token or 0 if it hit NUL
    depth = (jtoken >> JTOK_DSHIFT) & JTOK_DMASK; // token extracted from NUL yields depth == 0
    if (depth <= parent_depth) {                // normal exit
        *p_nv = nv;                             // update external pointer prior to normal exit
        return (STAT_OK);
    }
    *tok = NUL;                                 // terminate the string (overwrite the token)

    //---- Process the name part ----//
	if (strlen(str) > TOKEN_LEN) {
    	return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
	}
	nv_reset_nv(nv);                            // clear the current NV object
	nv->depth = depth;

   	strncpy(nv->token, str, TOKEN_LEN+1);       // copy the name to the nv_token
   	strncpy(nv->group, group, GROUP_LEN+1);     // copy the group iff set by parent
//    nv->token = str;                            // bind token string to nv struct
//    nv->group = group;                          // bind group string to nv struct

    if ((nv->index = nv_get_index(nv->group, nv->token)) == NO_MATCH) {
        return (STAT_UNRECOGNIZED_NAME);        // failed to get a valid index
    }
    str = tok+1; *p_str = str;                  // set str & p_str to start of next field

    // Process parent key - recursively marshall all children
    if (jtoken & JTOK_PARENT) {
        nv->valuetype = TYPE_PARENT;
        depth = nv->depth;                        // record parent depth
        if (nv_group_is_prefixed(nv->token)) {
            strncpy(group, nv->token, GROUP_LEN); // capture the group field
        }
        bool typesafe = nv_group_is_typesafe(nv->token); // deal with typesafe exceptions

        while (true) {  // gather up the kiddies: Nothing is certain except depth and tokens
            if ((nv = nv_next(nv)) == NULL) { return (STAT_JSON_TOO_MANY_PAIRS); } // get next child or exit
            *p_nv = nv;
            ritorno(_marshall_nv_pair(p_nv, p_str, group, depth, typesafe)); // recursively parse child, exit if error
            if ((tok = _next_token(*p_str)) == NULL) { return(STAT_OK); }  // normal end-of-string exit
            if (((*tok >> JTOK_DSHIFT) & JTOK_DMASK) <= depth) { return (STAT_OK); } // normal depth exit
        }
    }

    //---- Process value part ----//
    if ((tok = _next_token(str)) == NULL) {
        return (STAT_JSON_SYNTAX_ERROR);
    }
    *p_str = tok+1;                                 // set p_str to next char past token
    jtoken = *tok; *tok = NUL;                      // collect JSON token & terminate value string
    nv->valuetype = jtoken & JTOK_TMASK;            // what type the value says it is

    if (nv->valuetype == TYPE_NULL) {               // no need to check NULL objects - exit OK
        return (STAT_OK);
    }

    // set type checking - true: type must agree with variable, false: type is set by JSON & therefore always OK
    cfg_type = (typesafe) ? cfg_is_type(nv->index) : nv->valuetype;

    if (cfg_type == TYPE_FLOAT) {
        if ((nv->valuetype == TYPE_INTEGER) || (nv->valuetype == TYPE_FLOAT)) {
            nv->valuetype = TYPE_FLOAT;             // morph the into to a float
            ritorno(str2float(str, &(nv->value_flt)));
        } else { return (STAT_VALUE_TYPE_ERROR); }
    }
    else if (cfg_type == TYPE_INTEGER) {
        if (nv->valuetype == TYPE_INTEGER) {
            ritorno(str2long(str, &(nv->value_int)));
        } else { return (STAT_VALUE_TYPE_ERROR); }
    }
    else if (cfg_type == TYPE_BOOLEAN) {            // booleans are complicated
        if (nv->valuetype == TYPE_BOOLEAN) {
            if      (*str == 'f') { nv->value_int = 0; } // handle t/f input values
            else if (*str == 't') { nv->value_int = 1; }
            else { return (STAT_JSON_SYNTAX_ERROR); }
        } else if (nv->valuetype == TYPE_INTEGER) {     // handle 0/1 input values
            nv->valuetype = TYPE_BOOLEAN;
            ritorno(str2long(str, &(nv->value_int)));
            if ((nv->value_int < 0) || (nv->value_int > 1)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
        } else { return (STAT_VALUE_TYPE_ERROR); }
    }
    else if (cfg_type == TYPE_STRING) {
        if (nv->valuetype == TYPE_STRING) {
            nv->str = str_unescape(str);            // link directly to string in input buffer
        } else {
            nv->valuetype = TYPE_STRING;
            nv_copy_string(nv, "null");
            return (STAT_VALUE_TYPE_ERROR);
        }
    }
    else if (cfg_type == TYPE_TXTCON) {
        if (nv->valuetype == TYPE_STRING) {
            nv->valuetype = TYPE_TXTCON;
            nv->str = str_unescape(str);            // link directly to string in input buffer
        } else { return (STAT_VALUE_TYPE_ERROR); }
    }
    else if (cfg_type == TYPE_DATA) {
        if (nv->valuetype == TYPE_STRING) {
            if (strlen(str)>=3 && (str)[0]=='0' && (str)[1]=='x') {
                uint32_t *v = (uint32_t*)&nv->value_flt;
                *v = strtoul((const char *)str, 0L, 0);
                nv->valuetype = TYPE_DATA;
            } else { return (STAT_BAD_NUMBER_FORMAT); }
        } else { return (STAT_VALUE_TYPE_ERROR); }
    }
    return (STAT_OK);
}

/****************************************************************************
 * json_serialize() - make a JSON object string from NV list
 *
 *	*nv is a pointer to the first element in the nv list to serialize
 *	*out_buf is a pointer to the output string - usually what was the input string
 *	Returns the character count of the resulting string
 *
 * 	Operation:
 *	  - The nvObj list is processed start to finish with no recursion
 *	  - Assume the first object is depth 0 or greater (the opening curly)
 *	  - Assume remaining depths have been set correctly; but might not achieve closure;
 *		e.g. list starts on 0, and ends on 3, in which case provide correct closing curlies
 *	  - Assume there can be multiple, independent, non-contiguous JSON objects at a
 *		given depth value. These are processed correctly - e.g. 1,2,2,1,2,2,1,2,2
 *    - Objects of TYPE_EMPTY or TYPE_SKIP are passed over (ignored)
 *	  - If a JSON object has no data it's represented as {}
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

    // JSON text container special handling.
    // See _json_parser_kernal() / controller_dispatch_txt_container()
    if (nv->valuetype == TYPE_TXTCON) {
        nv = nv_next(nv);                           // skip over head pair
        prev_depth = 1;                             // text containers are always at depth 1
    }

    // Serialize the list - Note: nv points to opening r{} or first usable object past continuation text
    while (true) {
        // write opening curlies, closing curlies, or leading comma for the current NV pair
        if (nv->depth > prev_depth) {               // one or more opening curlies
            while (nv->depth > prev_depth) {
                *str++ = '{';
                prev_depth++;
            }
        }
        else if (nv->depth < prev_depth) {
            *str++ = '}';
            prev_depth--;
            continue;                               // continue w/o advancing to next NV
        }
        else if (nv->depth == prev_depth) {
            *str++ = ',';
        }

        // serialize name
        str += sprintf_P(str, PSTR("\"%s\":"), nv->token);

		// serialize value
        switch (nv->valuetype) {
            case TYPE_NULL :    {   str = strcat_literal_P(str, PSTR("null")); break; }

            case TYPE_BOOLEAN : {   if (nv->value_int) { str = strcat_literal_P(str, PSTR("true")); }
    			                                  else { str = strcat_literal_P(str, PSTR("false")); } 
                                    break; 
                                }

            case TYPE_INTEGER : {   str = strcat_integer(str, nv->value_int); break; }

            case TYPE_STRING :  {   if ((str + strlen(nv->str)) >= out_max) { return (-1); }
                                    str = strcat_string(str, nv->str);
                                    break;
                                }

            case TYPE_FLOAT :   {   if (prep_float(nv)) { str = strcat_float(str, nv->value_flt, nv->precision); }
                                                   else { str = strcat_literal_P(str, PSTR("null")); }
                                    break;              // bad float value - does not change status codes
                                }

            case TYPE_SIGNED :  {   str = strcat_signed(str, nv->value_int); break; }

            case TYPE_DATA :    {   uint32_t *v = (uint32_t*)&nv->value_flt;
                                    str += sprintf_P(str, PSTR("\"0x%lx\""), *v); break; 
                                }

            case TYPE_ARRAY :   {   str += sprintf_P(str, PSTR("[%s]"), nv->str); break; }

            case TYPE_PARENT :  {   if (nv->nx->depth <= nv->depth) { str = strcat_literal_P(str, PSTR("{}")); }
                                    break;                     // handles parent with null child object ^
                                }

            default :           {    break; }
//            default :           { str += sprintf_P(str, PSTR("TYPE ERROR %d"), nv->valuetype); break; }
        }
		if (str >= out_max) { return (-1); }        // test for buffer overrun
        prev_depth = nv->depth;
	    if ((nv = nv_next(nv)) == NULL) { break; }
	}

    // finish up closing curlies and terminate string
	if ((str + prev_depth - initial_depth + 3) > (out_buf + out_size)) { // test for space
        return (-1);
    }
    while (prev_depth-- > initial_depth) {	        // write closing curlies
        *str++ = '}';
    }
	str += sprintf(str, "}\n");	                    // sprintf ensures a NUL termination
    json_relax(cs.out_buf);
	return (str - out_buf);
}

/*
 * json_print_response() - JSON responses with headers, footers and observing JSON verbosity
 *
 *	  r{} header is returned for every setting except $jv=0
 *	  f{} footer is returned for every setting except $jv=0
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
	nvObj_t *nv = NV_HEAD;	                            // setup the response header
    if (nv->valuetype != TYPE_TXTCON) {                 // include r{} if not a text container
	    nv->valuetype = TYPE_PARENT;
	    strcpy(nv->token, "r");
    }
	nv = NV_BODY;	                                    // start body processing
	if (status == STAT_JSON_SYNTAX_ERROR) {             // wrap syntax errors. These would blow up the host's JSON parser
    	nv_add_string((const char *)"err", str_escape(cs.bufp, cs.saved_buf));
	}

    // Filter the NV pairs down to the JSON verbosity reporting level
    // Always do full echo during initialization or if there is an error
    else if ((cm_get_machine_state() != MACHINE_INITIALIZING) || (status != STAT_OK)) {
		nvType nv_type;
		do {
            if (nv->valuetype == TYPE_EMPTY) {		    // find end of list
                break;
            }
            // filter output according to JV settings
			nv_type = nv_get_type(nv);
            if (nv_type == NV_TYPE_REPORT) {            // always display all of status, queue and exception reports
                break;
			} else if (nv_type == NV_TYPE_GCODE) {
				if (!js.echo_json_gcode_block) {	    // skip command echo if not enabled
					nv->valuetype = TYPE_SKIP;
				}
			} else if (nv_type == NV_TYPE_LINENUM) {    // skip line number echo if not enabled
				if ((js.echo_json_linenum == false) || (nv->value_int == 0)) { // do not report line# 0
					nv->valuetype = TYPE_SKIP;
				}
            } else if (nv_type == NV_TYPE_CONFIG) {     // skip config echo if not enabled
		        if (js.echo_json_configs == false) {
					nv->valuetype = TYPE_SKIP;
				}
			} else if (nv_type == NV_TYPE_MESSAGE) {    // skip message echo if not enabled
			    if (js.echo_json_messages == false) {
    			    nv->valuetype = TYPE_SKIP;
			    }
            }
		} while ((nv = nv_next(nv)) != NULL);
	}
    nv = NV_FOOT;                                       // Add the footer
	char footer_string[NV_FOOTER_LEN];

    if (xio.rx_mode == RX_MODE_CHAR) {                  // character style footer?
        sprintf(footer_string, "2,%d,%d", status, cs.linelen);
        cs.linelen = 0;									// reset linelen so it's only reported once
    } else {                                            // or line_mode style footer
        sprintf(footer_string, "3,%d,%d", status, xio_get_line_buffers_available());
    }
	nv_copy_string(nv, footer_string);					// link string to nv object
	nv->depth = 0;                                      // footer is a peer to r{} response
	nv->valuetype = TYPE_ARRAY;
	strcpy(nv->token, "f");                             // terminate the list

	// serialize the JSON response and print it if there were no errors
	if (json_serialize(NV_HEAD, cs.out_buf, sizeof(cs.out_buf)) > 0) {
    	printf(cs.out_buf);
	}
}

/*
 * json_print_list() - command to select and produce a JSON formatted output
 */

void json_print_list(stat_t status, uint8_t flags)
{
	switch (flags) {
		case JSON_NO_DISPLAY: { break; }
		case JSON_OBJECT: { json_print_object(nvl.list); break; }
		case JSON_RESPONSE: { json_print_response(status); break; }
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
 * json_relax() - convert strict syntax string to relaxed mode if in relaxed mode
 *
 * Assumes the string is well-formed JSON - so very limited error checking is performed
 *  e.g. JSON strings will never start with an opening quote
 */

char *json_relax(char *str)
{
    if (js.json_syntax == JSON_SYNTAX_STRICT) {
        return (str);
    }
    char *wr = str;  // write pointer
    char *start = str;
    bool in_string = false;

    for (; *str != 0; *wr++ = *str, str++) {
        if (in_string) {
            if ((*str == '"') && (*(str-1) != '\\')) {  // find closing quote
                in_string = false;
            }
            continue;
        } else {
            if (*str == '"') {
                if (*(str-1) == ':') {  // opening quote
                    in_string = true;
                    continue;
                }
                str++;
            }
        }
    }
    *wr = 0;
    return (start);
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
#include "text_parser.h"

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
