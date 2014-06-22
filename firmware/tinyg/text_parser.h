/*
 * text_parser.h - text parser and text mode support for tinyg2
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart, Jr.
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

#ifndef TEXT_PARSER_H_ONCE
#define TEXT_PARSER_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

enum textVerbosity {
	TV_SILENT = 0,					// no response is provided
	TV_VERBOSE						// response is provided. Error responses ech message and failed commands
};

enum textFormats {					// text output print modes
	TEXT_NO_PRINT = 0,				// don't print anything if you find yourself in TEXT mode
	TEXT_INLINE_PAIRS,				// print key:value pairs as comma separated pairs
	TEXT_INLINE_VALUES,				// print values as commas separated values
	TEXT_MULTILINE_FORMATTED		// print formatted values on separate lines with formatted print per line
};

typedef struct txtSingleton {		// text mode data

	/*** config values (PUBLIC) ***/

	char_t format[NV_FORMAT_LEN+1];

	/*** runtime values (PRIVATE) ***/

	uint8_t text_verbosity;			// see enum in this file for settings

} txtSingleton_t;
extern txtSingleton_t txt;

/**** Global Scope Functions ****/

#ifdef __TEXT_MODE

	stat_t text_parser(char_t *str);
	void text_response(const stat_t status, char_t *buf);
	void text_print_list(stat_t status, uint8_t flags);
	void text_print_inline_pairs(nvObj_t *nv);
	void text_print_inline_values(nvObj_t *nv);
	void text_print_multiline_formatted(nvObj_t *nv);

	void tx_print_nul(nvObj_t *nv);
	void tx_print_str(nvObj_t *nv);
	void tx_print_ui8(nvObj_t *nv);
	void tx_print_int(nvObj_t *nv);
	void tx_print_flt(nvObj_t *nv);

	void text_print_nul(nvObj_t *nv, const char *format);
	void text_print_str(nvObj_t *nv, const char *format);
	void text_print_ui8(nvObj_t *nv, const char *format);
	void text_print_int(nvObj_t *nv, const char *format);
	void text_print_flt(nvObj_t *nv, const char *format);
	void text_print_flt_units(nvObj_t *nv, const char *format, const char *units);

	void tx_print_tv(nvObj_t *nv);

#else

	#define text_parser text_parser_stub
	#define text_response text_response_stub
	#define text_print_list text_print_list_stub
	#define tx_print_nul tx_print_stub
	#define tx_print_ui8 tx_print_stub
	#define tx_print_int tx_print_stub
	#define tx_print_flt tx_print_stub
	#define tx_print_str tx_print_stub
	#define tx_print_tv tx_print_stub

	void tx_print_stub(nvObj_t *nv);

#endif

stat_t text_parser_stub(char_t *str);
void text_response_stub(const stat_t status, char_t *buf);
void text_print_list_stub(stat_t status, uint8_t flags);

#ifdef __cplusplus
}
#endif

#endif // End of include guard: TEXT_PARSER_H_ONCE
