/*
 * text_parser.c - text parser for TinyG
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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
#include "config.h"
#include "canonical_machine.h"
#include "text_parser.h"
#include "json_parser.h"
#include "report.h"
#include "help.h"
#include "util.h"
#include "xio.h"					// for ASCII char definitions

#ifdef __cplusplus
extern "C"{
#endif

txtSingleton_t txt;					// declare the singleton for either __TEXT_MODE setting

#ifndef __TEXT_MODE

stat_t text_parser_stub(char_t *str) {return (STAT_OK);}
void text_response_stub(const stat_t status, char_t *buf) {}
void text_print_list_stub (stat_t status, uint8_t flags) {}
void tx_print_stub(nvObj_t *nv) {}

#else // __TEXT_MODE

static stat_t _text_parser_kernal(char_t *str, nvObj_t *nv);

/******************************************************************************
 * text_parser() 		 - update a config setting from a text block (text mode)
 * _text_parser_kernal() - helper for above
 *
 * Use cases handled:
 *	- $xfr=1200		set a parameter (strict separators))
 *	- $xfr 1200		set a parameter (relaxed separators)
 *	- $xfr			display a parameter
 *	- $x			display a group
 *	- ?				generate a status report (multiline format)
 */
stat_t text_parser(char_t *str)
{
	nvObj_t *nv = nv_reset_nv_list();				// returns first object in the body
	stat_t status = STAT_OK;

	// trap special displays
	if (str[0] == '?') {							// handle status report case
		sr_run_text_status_report();
		return (STAT_OK);
	}
	if (str[0] == 'H') {							// print help screens
		help_general((nvObj_t *)NULL);
		return (STAT_OK);
	}

	// pre-process the command
	if ((str[0] == '$') && (str[1] == NUL)) {		// treat a lone $ as a sys request
		strcat(str,"sys");
	}

	// parse and execute the command (only processes 1 command per line)
	ritorno(_text_parser_kernal(str, nv));			// run the parser to decode the command
	if ((nv->valuetype == TYPE_NULL) || (nv->valuetype == TYPE_PARENT)) {
		if (nv_get(nv) == STAT_COMPLETE){			// populate value, group values, or run uber-group displays
			return (STAT_OK);						// return for uber-group displays so they don't print twice
		}
	} else { 										// process SET and RUN commands
		if (cm.machine_state == MACHINE_ALARM)
            return (STAT_MACHINE_ALARMED);
		status = nv_set(nv);						// set (or run) single value
		if (status == STAT_OK) {
			nv_persist(nv);							// conditionally persist depending on flags in array
		}
	}
	nv_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT); // print the results
	return (status);
}

static stat_t _text_parser_kernal(char_t *str, nvObj_t *nv)
{
	char_t *rd, *wr;								// read and write pointers
//	char_t separators[] = {"="};					// STRICT: only separator allowed is = sign
	char_t separators[] = {" =:|\t"};				// RELAXED: any separator someone might use

	// pre-process and normalize the string
//	nv_reset_nv(nv);								// initialize config object
	nv_copy_string(nv, str);						// make a copy for eventual reporting
	if (*str == '$') str++;							// ignore leading $
	for (rd = wr = str; *rd != NUL; rd++, wr++) {
		*wr = tolower(*rd);							// convert string to lower case
		if (*rd == ',') { *wr = *(++rd);}			// skip over commas
	}
	*wr = NUL;										// terminate the string

	// parse fields into the nv struct
	nv->valuetype = TYPE_NULL;
	if ((rd = strpbrk(str, separators)) == NULL) {	// no value part
		strncpy(nv->token, str, TOKEN_LEN);
	} else {
		*rd = NUL;									// terminate at end of name
		strncpy(nv->token, str, TOKEN_LEN);
		str = ++rd;
		nv->value = strtof(str, &rd);				// rd used as end pointer
		if (rd != str) {
			nv->valuetype = TYPE_FLOAT;
		}
	}

	// validate and post-process the token
	if ((nv->index = nv_get_index((const char_t *)"", nv->token)) == NO_MATCH) { // get index or fail it
		return (STAT_UNRECOGNIZED_NAME);
	}
	strcpy_P(nv->group, cfgArray[nv->index].group);	// capture the group string if there is one

	// see if you need to strip the token
	if (nv->group[0] != NUL) {
		wr = nv->token;
		rd = nv->token + strlen(nv->group);
		while (*rd != NUL) { *(wr)++ = *(rd)++;}
		*wr = NUL;
	}
	return (STAT_OK);
}

/************************************************************************************
 * text_response() - text mode responses
 */
static const char prompt_ok[] PROGMEM = "tinyg [%s] ok> ";
static const char prompt_err[] PROGMEM = "tinyg [%s] err: %s: %s ";

void text_response(const stat_t status, char_t *buf)
{
	if (txt.text_verbosity == TV_SILENT) return;	// skip all this

	char units[] = "inch";
	if (cm_get_units_mode(MODEL) != INCHES) { strcpy(units, "mm"); }

	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP)) {
		fprintf_P(stderr, prompt_ok, units);
	} else {
		fprintf_P(stderr, prompt_err, units, get_status_message(status), buf);
	}
	nvObj_t *nv = nv_body+1;

	if (nv_get_type(nv) == NV_TYPE_MESSAGE) {
		fprintf(stderr, (char *)*nv->stringp);
	}
	fprintf(stderr, "\n");
}

/***** PRINT FUNCTIONS ********************************************************
 * json_print_list() - command to select and produce a JSON formatted output
 * text_print_inline_pairs()
 * text_print_inline_values()
 * text_print_multiline_formatted()
 */

void text_print_list(stat_t status, uint8_t flags)
{
	switch (flags) {
		case TEXT_NO_PRINT: { break; }
		case TEXT_INLINE_PAIRS: { text_print_inline_pairs(nv_body); break; }
		case TEXT_INLINE_VALUES: { text_print_inline_values(nv_body); break; }
		case TEXT_MULTILINE_FORMATTED: { text_print_multiline_formatted(nv_body);}
	}
}

void text_print_inline_pairs(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value;
	for (uint8_t i=0; i<NV_BODY_LEN-1; i++) {
		switch (nv->valuetype) {
			case TYPE_PARENT: 	{ if ((nv = nv->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ preprocess_float(nv);
								  fntoa(global_string_buf, nv->value, nv->precision);
								  fprintf_P(stderr,PSTR("%s:%s"), nv->token, global_string_buf) ; break;
								}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%s:%1.0f"), nv->token, nv->value); break;}
			case TYPE_DATA:	    { fprintf_P(stderr,PSTR("%s:%lu"), nv->token, *v); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s:%s"), nv->token, *nv->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((nv = nv->nx) == NULL) return;
		if (nv->valuetype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}
	}
}

void text_print_inline_values(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value;
	for (uint8_t i=0; i<NV_BODY_LEN-1; i++) {
		switch (nv->valuetype) {
			case TYPE_PARENT: 	{ if ((nv = nv->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ preprocess_float(nv);
								  fntoa(global_string_buf, nv->value, nv->precision);
								  fprintf_P(stderr,PSTR("%s"), global_string_buf) ; break;
								}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%1.0f"), nv->value); break;}
			case TYPE_DATA:	    { fprintf_P(stderr,PSTR("%lu"), *v); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s"), *nv->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((nv = nv->nx) == NULL) return;
		if (nv->valuetype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}
	}
}

void text_print_multiline_formatted(nvObj_t *nv)
{
	for (uint8_t i=0; i<NV_BODY_LEN-1; i++) {
		if (nv->valuetype != TYPE_PARENT) {
			preprocess_float(nv);
			nv_print(nv);
		}
		if ((nv = nv->nx) == NULL) return;
		if (nv->valuetype == TYPE_EMPTY) break;
	}
}

/*
 * Text print primitives using generic formats
 */
static const char fmt_str[] PROGMEM = "%s\n";	// generic format for string message (with no formatting)
static const char fmt_ui8[] PROGMEM = "%d\n";	// generic format for ui8s
static const char fmt_int[] PROGMEM = "%lu\n";	// generic format for ui16's and ui32s
static const char fmt_flt[] PROGMEM = "%f\n";	// generic format for floats

void tx_print_nul(nvObj_t *nv) {}
void tx_print_str(nvObj_t *nv) { text_print_str(nv, fmt_str);}
void tx_print_ui8(nvObj_t *nv) { text_print_ui8(nv, fmt_ui8);}
void tx_print_int(nvObj_t *nv) { text_print_int(nv, fmt_int);}
void tx_print_flt(nvObj_t *nv) { text_print_flt(nv, fmt_flt);}

/*
 * Text print primitives using external formats
 *
 *	NOTE: format's are passed in as flash strings (PROGMEM)
 */

void text_print_nul(nvObj_t *nv, const char *format) { fprintf_P(stderr, format);}	// just print the format string
void text_print_str(nvObj_t *nv, const char *format) { fprintf_P(stderr, format, *nv->stringp);}
void text_print_ui8(nvObj_t *nv, const char *format) { fprintf_P(stderr, format, (uint8_t)nv->value);}
void text_print_int(nvObj_t *nv, const char *format) { fprintf_P(stderr, format, (uint32_t)nv->value);}
void text_print_flt(nvObj_t *nv, const char *format) { fprintf_P(stderr, format, nv->value);}

void text_print_flt_units(nvObj_t *nv, const char *format, const char *units)
{
	fprintf_P(stderr, format, nv->value, units);
}

/*
 * Formatted print supporting the text parser
 */
static const char fmt_tv[] PROGMEM = "[tv]  text verbosity%15d [0=silent,1=verbose]\n";

void tx_print_tv(nvObj_t *nv) { text_print_ui8(nv, fmt_tv);}


#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif // __cplusplus
