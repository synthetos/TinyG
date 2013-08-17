/*
 * text_parser.c - text parser for TinyG
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 *	See config.h for a Config system overview and a bunch of details.
 */

#include "tinyg.h"
#include "config.h"
#include "report.h"
#include "controller.h"
#include "canonical_machine.h"
#include "text_parser.h"
#include "xio/xio.h"

static stat_t _text_parser_kernal(char_t *str, cmdObj_t *cmd);

/******************************************************************************
 * text_parser() - update a config setting from a text block (text mode)
 * _text_parser() 	 - helper for above
 * 
 * Use cases handled:
 *	- $xfr=1200	set a parameter
 *	- $xfr		display a parameter
 *	- $x		display a group
 *	- ?			generate a status report (multiline format)
 */
stat_t text_parser(char_t *str)
{
	cmdObj_t *cmd = cmd_reset_list();		// returns first object in the body
	stat_t status = STAT_OK;

	// pre-process the command 
	if (str[0] == '?') {					// handle status report case

//+++++ DIAGNOSTIC +++++
//		printf("Text: posX: %6.3f, posY: %6.3f\n", (double)gm.position[AXIS_X], (double)gm.target[AXIS_Y]);
		rpt_run_text_status_report();
		return (STAT_OK);
	}
	if ((str[0] == '$') && (str[1] == NUL)) { // treat a lone $ as a sys request
		strcat(str,"sys");
	}

	// parse and execute the command (only processes 1 command per line)
	ritorno(_text_parser_kernal(str, cmd));	// run the parser to decode the command
	if ((cmd->objtype == TYPE_NULL) || (cmd->objtype == TYPE_PARENT)) {
		if (cmd_get(cmd) == STAT_COMPLETE) {// populate value, group values, or run uber-group displays
			return (STAT_OK);				// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		status = cmd_set(cmd);				// set (or run) single value
		cmd_persist(cmd);					// conditionally persist depending on flags in array
	}
	cmd_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT); // print the results
	return (status);
}

static stat_t _text_parser_kernal(char_t *str, cmdObj_t *cmd)
{
	char_t *ptr_rd, *ptr_wr;				// read and write pointers
	char_t separators[] = {" =:|\t"};		// any separator someone might use

	// pre-process and normalize the string
	cmd_reset_obj(cmd);						// initialize config object
	cmd_copy_string(cmd, str);				// make a copy for eventual reporting
	if (*str == '$') str++;					// ignore leading $
	for (ptr_rd = ptr_wr = str; *ptr_rd!=NUL; ptr_rd++, ptr_wr++) {
		*ptr_wr = tolower(*ptr_rd);			// convert string to lower case
		if (*ptr_rd==',') { *ptr_wr = *(++ptr_rd); } // skip over commas
	}
	*ptr_wr = NUL;							// terminate the string

	// parse fields into the cmd struct
	cmd->objtype = TYPE_NULL;
	if ((ptr_rd = strpbrk(str, separators)) == NULL) { // no value part
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
	} else {
		*ptr_rd = NUL;						// terminate at end of name
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
		str = ++ptr_rd;
		cmd->value = strtod(str, &ptr_rd);	// ptr_rd used as end pointer
		if (ptr_rd != str) {
			cmd->objtype = TYPE_FLOAT;
		}
	}

	// validate and post-process the token
	if ((cmd->index = cmd_get_index("",cmd->token)) == NO_MATCH) { // get index or fail it
		return (STAT_UNRECOGNIZED_COMMAND);
	}
	strcpy_P(cmd->group, cfgArray[cmd->index].group);	// capture the group string if there is one

	if (strlen(cmd->group) > 0) {			// see if you need to strip the token
		ptr_wr = cmd->token;
		ptr_rd = cmd->token + strlen(cmd->group);
		while (*ptr_rd != NUL) {
			*(ptr_wr)++ = *(ptr_rd)++;
		}
		*ptr_wr = NUL;
	}
	return (STAT_OK);
}

/************************************************************************************
 * text_response() - text mode responses
 */
static const char prompt_mm[] PROGMEM = "mm";
static const char prompt_in[] PROGMEM = "inch";
static const char prompt_ok[] PROGMEM = "tinyg [%S] ok> ";
static const char prompt_err[] PROGMEM = "tinyg [%S] err: %s: %s ";

void tg_text_response(const uint8_t status, const char *buf)
{
	if (cfg.text_verbosity == TV_SILENT) return;	// skip all this

	const char *units;								// becomes pointer to progmem string
	if (cm_get_model_units_mode() != INCHES) { 
		units = (PGM_P)&prompt_mm;
	} else {
		units = (PGM_P)&prompt_in;
	}
//	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP) || (status == STAT_ZERO_LENGTH_MOVE)) {
	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP)) {
		fprintf_P(stderr, (PGM_P)&prompt_ok, units);
	} else {
		char status_message[STATUS_MESSAGE_LEN];
		fprintf_P(stderr, (PGM_P)prompt_err, units, rpt_get_status_message(status, status_message), buf);
	}
	cmdObj_t *cmd = cmd_body+1;
	if (cmd->token[0] == 'm') {
		fprintf(stderr, *cmd->stringp);
	}
	fprintf(stderr, "\n");
}

/************************************************************************************
 * text_print_inline_pairs()
 * text_print_inline_values()
 * text_print_multiline_formatted()
 */

void text_print_inline_pairs(cmdObj_t *cmd)
{
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%s:%1.3f"), cmd->token, cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%s:%1.0f"), cmd->token, cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s:%s"), cmd->token, *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}		
	}
}

void text_print_inline_values(cmdObj_t *cmd)
{
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%1.3f"), cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%1.0f"), cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s"), *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}
	}
}

void text_print_multiline_formatted(cmdObj_t *cmd)
{
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		if (cmd->objtype != TYPE_PARENT) { cmd_print(cmd);}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype == TYPE_EMPTY) break;
	}
}

