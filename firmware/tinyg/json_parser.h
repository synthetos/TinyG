/*
 * json_parser.h - JSON parser and JSON support for TinyG
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

#ifndef _JSON_PARSER_H_ONCE
#define _JSON_PARSER_H_ONCE

/**** Configs, Definitions and Structures ****/

/* JSON array definitions / revisions */
// for now there is only one JSON array in use - the footer
// if you add these make sure there are no collisions w/present or past numbers

#define FOOTER_REVISION 1

#define JSON_INPUT_STRING_MAX 128   // set an arbitrary max
#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)

#define JSON_DEPTH_MAX 4            // maximum nesting level
#define JSON_PAIRS_MAX NV_LIST_LEN  // maximum number of total pairs allowed

typedef enum {
	JV_SILENT = 0,					// no response is provided for any command
	JV_FOOTER,						// returns footer only (no command echo, gcode blocks or messages)
	JV_MESSAGES,					// returns footer, messages (exception and gcode messages)
	JV_CONFIGS,						// returns footer, messages, config commands
	JV_LINENUM,						// returns footer, messages, config commands, gcode line numbers if present
	JV_VERBOSE						// returns footer, messages, config commands, gcode blocks
} jsonVerbosity;

typedef enum {                      // json output display modes
	JSON_NO_DISPLAY = 0,            // don't print anything if you find yourself in JSON mode
	JSON_OBJECT,                    // print just the body as a json object
	JSON_RESPONSE                   // print the header/body/footer as a response object
} jsonDisplays;

typedef enum {
	JSON_SYNTAX_RELAXED = 0,		// Does not require quotes on names
	JSON_SYNTAX_STRICT				// requires quotes on names
} jsonSyntax;

typedef enum {                      // JSON tokenization state machine
    JTSM_START = 0,                 // normalization start
    JTSM_KEY,                       // in a key
    JTSM_VALUE,                     // in a value
    JTSM_STRING_VALUE,              // in a value string
    JTSM_ARRAY_VALUE                // in an array
} jsonTokenizationState;

#define JTOK_FLAG    0x80           // flag char as a token
#define JTOK_DEPTH   0x70           // mask to extract depth
#define JTOK_DMASK   0x07           // mask for down-shifted depth value
#define JTOK_DSHIFT  4              // number of bits to shift depth value
#define JTOK_KEY     0x08           // key flag:   1=key, 0=value
#define JTOK_PARENT  0x04           // parent flag. Key is parent
#define JTOK_ARRAY   0x04           // array flag: 1=value is array value, 0=not
#define JTOK_TMASK   0x03           // mask to extract jsonToken enum

typedef struct jsSingleton {

	/*** config values (PUBLIC) ***/
	jsonVerbosity json_verbosity;   // see enum in this file for settings
	jsonSyntax json_syntax;         // 0=relaxed syntax, 1=strict syntax

#ifdef __BITFIELDS
	bool echo_json_footer       : 1 ;   // flags for JSON responses serialization
	bool echo_json_messages     : 1 ;
	bool echo_json_configs      : 1 ;
	bool echo_json_linenum      : 1 ;
	bool echo_json_gcode_block  : 1 ;
#else
	bool echo_json_footer       ;   // flags for JSON responses serialization
	bool echo_json_messages     ;
	bool echo_json_configs      ;
	bool echo_json_linenum      ;
	bool echo_json_gcode_block  ;
#endif
	/*** runtime values (PRIVATE) ***/

} jsSingleton_t;

/**** Externs - See report.c for allocation ****/

extern jsSingleton_t js;

/**** Function Prototypes ****/

void json_parser(char *str);
//void json_parser(char *str, nvObj_t *nv, bool display);

int16_t json_serialize(nvObj_t *nv, char *out_buf, int16_t out_size);
void json_print_response(uint8_t status);
void json_print_list(stat_t status, uint8_t flags);
void json_print_object(nvObj_t *nv);
char *json_relax(char *str);

stat_t json_set_jv(nvObj_t *nv);

#ifdef __TEXT_MODE

	void js_print_ej(nvObj_t *nv);
	void js_print_jv(nvObj_t *nv);
	void js_print_js(nvObj_t *nv);
	void js_print_fs(nvObj_t *nv);

#else

	#define js_print_ej tx_print_stub
	#define js_print_jv tx_print_stub
	#define js_print_js tx_print_stub
	#define js_print_fs tx_print_stub

#endif // __TEXT_MODE

#endif // End of include guard: JSON_PARSER_H_ONCE
