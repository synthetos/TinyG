/*
 * json_parser.c - JSON parser for TinyG
 * Part of TinyG project
 *
 * Copyright (c) 2012 - 2013 Alden S. Hart, Jr.
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

#ifndef json_parser_h
#define json_parser_h

/**** Configs, Definitions and Structures ****/

/* JSON array definitions / revisions */
// for now there is only one JSON array in use - the footer
// if you add these make sure there are no collisions w/present or past numbers

#define FOOTER_REVISION 1
#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)

enum jsonVerbosity {
	JV_SILENT = 0,					// no response is provided for any command
	JV_FOOTER,						// returns footer only (no command echo, gcode blocks or messages)
	JV_MESSAGES,					// returns footer, messages (exception and gcode messages)
	JV_CONFIGS,						// returns footer, messages, config commands
	JV_LINENUM,						// returns footer, messages, config commands, gcode line numbers if present
	JV_VERBOSE						// returns footer, messages, config commands, gcode blocks
};

enum jsonFormats {					// json output print modes
	JSON_NO_PRINT = 0,				// don't print anything if you find yourself in JSON mode
	JSON_OBJECT_FORMAT,				// print just the body as a json object
	JSON_RESPONSE_FORMAT			// print the header/body/footer as a response object
};

typedef struct jsSingleton {

	/*** config values (PUBLIC) ***/
	uint8_t json_verbosity;			// see enum in this file for settings
	uint8_t json_footer_depth;		// 0=footer is peer to response 'r', 1=child of response 'r'
//	uint8_t json_footer_style;		// select footer style

	uint8_t echo_json_footer;		// flags for JSON responses serialization
	uint8_t echo_json_messages;
	uint8_t echo_json_configs;
	uint8_t echo_json_linenum;
	uint8_t echo_json_gcode_block;

	/*** runtime values (PRIVATE) ***/

} jsSingleton_t;

/**** Externs - See report.c for allocation ****/

extern jsSingleton_t js;

/**** Function Prototypes ****/

stat_t json_set_jv(cmdObj_t *cmd);

void json_parser(char_t *str);
uint16_t json_serialize(cmdObj_t *cmd, char_t *out_buf, uint16_t size);
void json_print_object(cmdObj_t *cmd);
void json_print_response(uint8_t status);
void json_print_list(stat_t status, uint8_t flags);

void js_print_ej(cmdObj_t *cmd);
void js_print_jv(cmdObj_t *cmd);
void js_print_fs(cmdObj_t *cmd);

//++++ remove +++/
extern const char_t PROGMEM fmt_ej[];
extern const char_t PROGMEM fmt_jv[];
extern const char_t PROGMEM fmt_fs[];

/* unit test setup */

//#define __UNIT_TEST_JSON				// uncomment to enable JSON unit tests
#ifdef __UNIT_TEST_JSON
void js_unit_tests(void);
#define	JSON_UNITS js_unit_tests();
#else
#define	JSON_UNITS
#endif // __UNIT_TEST_JSON

#endif
