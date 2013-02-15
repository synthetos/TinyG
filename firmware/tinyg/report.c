/*
 * report.c - TinyG status report and other reporting functions.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S Hart, Jr.
 */
/* TinyG is free software: you can redistribute it and/or modify it 
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "json_parser.h"
#include "controller.h"
#include "planner.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "report.h"
#include "settings.h"
#include "xio/xio.h"
#include "xmega/xmega_rtc.h"

/**** System Messages **************************************************************
 * tg_get_status_message()
 */

/* These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 * Reference for putting display strings and string arrays in program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */
static const char msg_sc00[] PROGMEM = "OK";
static const char msg_sc01[] PROGMEM = "Error";
static const char msg_sc02[] PROGMEM = "Eagain";
static const char msg_sc03[] PROGMEM = "Noop";
static const char msg_sc04[] PROGMEM = "Complete";
static const char msg_sc05[] PROGMEM = "Terminated";
static const char msg_sc06[] PROGMEM = "Hard reset";
static const char msg_sc07[] PROGMEM = "End of line";
static const char msg_sc08[] PROGMEM = "End of file";
static const char msg_sc09[] PROGMEM = "File not open";
static const char msg_sc10[] PROGMEM = "Max file size exceeded";
static const char msg_sc11[] PROGMEM = "No such device";
static const char msg_sc12[] PROGMEM = "Buffer empty";
static const char msg_sc13[] PROGMEM = "Buffer full";
static const char msg_sc14[] PROGMEM = "Buffer full - fatal";
static const char msg_sc15[] PROGMEM = "Initializing";
static const char msg_sc16[] PROGMEM = "#16";
static const char msg_sc17[] PROGMEM = "#17";
static const char msg_sc18[] PROGMEM = "#18";
static const char msg_sc19[] PROGMEM = "#19";

static const char msg_sc20[] PROGMEM = "Internal error";
static const char msg_sc21[] PROGMEM = "Internal range error";
static const char msg_sc22[] PROGMEM = "Floating point error";
static const char msg_sc23[] PROGMEM = "Divide by zero";
static const char msg_sc24[] PROGMEM = "#24";
static const char msg_sc25[] PROGMEM = "#25";
static const char msg_sc26[] PROGMEM = "#26";
static const char msg_sc27[] PROGMEM = "#27";
static const char msg_sc28[] PROGMEM = "#28";
static const char msg_sc29[] PROGMEM = "#29";
static const char msg_sc30[] PROGMEM = "#30";
static const char msg_sc31[] PROGMEM = "#31";
static const char msg_sc32[] PROGMEM = "#32";
static const char msg_sc33[] PROGMEM = "#33";
static const char msg_sc34[] PROGMEM = "#34";
static const char msg_sc35[] PROGMEM = "#35";
static const char msg_sc36[] PROGMEM = "#36";
static const char msg_sc37[] PROGMEM = "#37";
static const char msg_sc38[] PROGMEM = "#38";
static const char msg_sc39[] PROGMEM = "#39";

static const char msg_sc40[] PROGMEM = "Unrecognized command";
static const char msg_sc41[] PROGMEM = "Expected command letter";
static const char msg_sc42[] PROGMEM = "Bad number format";
static const char msg_sc43[] PROGMEM = "Input exceeds max length";
static const char msg_sc44[] PROGMEM = "Input value too small";
static const char msg_sc45[] PROGMEM = "Input value too large";
static const char msg_sc46[] PROGMEM = "Input value range error";
static const char msg_sc47[] PROGMEM = "Input value unsupported";
static const char msg_sc48[] PROGMEM = "JSON syntax error";
static const char msg_sc49[] PROGMEM = "JSON input has too many pairs";
static const char msg_sc50[] PROGMEM = "Out of buffer space";
static const char msg_sc51[] PROGMEM = "#51";
static const char msg_sc52[] PROGMEM = "#52";
static const char msg_sc53[] PROGMEM = "#53";
static const char msg_sc54[] PROGMEM = "#54";
static const char msg_sc55[] PROGMEM = "#55";
static const char msg_sc56[] PROGMEM = "#56";
static const char msg_sc57[] PROGMEM = "#57";
static const char msg_sc58[] PROGMEM = "#58";
static const char msg_sc59[] PROGMEM = "#59";

static const char msg_sc60[] PROGMEM = "Zero length move";
static const char msg_sc61[] PROGMEM = "Gcode block skipped";
static const char msg_sc62[] PROGMEM = "Gcode input error";
static const char msg_sc63[] PROGMEM = "Gcode feedrate error";
static const char msg_sc64[] PROGMEM = "Gcode axis word missing";
static const char msg_sc65[] PROGMEM = "Gcode modal group violation";
static const char msg_sc66[] PROGMEM = "Homing cycle failed";
static const char msg_sc67[] PROGMEM = "Max travel exceeded";
static const char msg_sc68[] PROGMEM = "Max spindle speed exceeded";
static const char msg_sc69[] PROGMEM = "Arc specification error";

PGM_P const msgStatusMessage[] PROGMEM = {
	msg_sc00, msg_sc01, msg_sc02, msg_sc03, msg_sc04, msg_sc05, msg_sc06, msg_sc07, msg_sc08, msg_sc09,
	msg_sc10, msg_sc11, msg_sc12, msg_sc13, msg_sc14, msg_sc15, msg_sc16, msg_sc17, msg_sc18, msg_sc19,
	msg_sc20, msg_sc21, msg_sc22, msg_sc23, msg_sc24, msg_sc25, msg_sc26, msg_sc27, msg_sc28, msg_sc29,
	msg_sc30, msg_sc31, msg_sc32, msg_sc33, msg_sc34, msg_sc35, msg_sc36, msg_sc37, msg_sc38, msg_sc39,
	msg_sc40, msg_sc41, msg_sc42, msg_sc43, msg_sc44, msg_sc45, msg_sc46, msg_sc47, msg_sc48, msg_sc49,
	msg_sc50, msg_sc51, msg_sc52, msg_sc53, msg_sc54, msg_sc55, msg_sc56, msg_sc57, msg_sc58, msg_sc59,
	msg_sc60, msg_sc61, msg_sc62, msg_sc63, msg_sc64, msg_sc65, msg_sc66, msg_sc67, msg_sc68, msg_sc69
};

char *rpt_get_status_message(uint8_t status, char *msg) 
{
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgStatusMessage[status]), STATUS_MESSAGE_LEN);
	return (msg);
}

/**** Fatal Errors *****************************************************************
 * tg_print_message()        - print a character string passed as argument
 * tg_print_message_value()  - print a message with a value
 * tg_print_message_number() - print a canned message by number
 */

void rpt_fatal_error(uint8_t errno)
{
	printf_P(PSTR("{\"st\":100,\"msg\":\"JSON serializer buffer overrun\"}\n"));
}

/**** Message Primitives ***********************************************************
 * rpt_print_message()        - print a character string passed as argument
 * rpt_print_message_value()  - print a message with a value
 * rpt_print_message_number() - print a canned message by number
 */
void rpt_print_message(char *msg)
{
	cmd_add_string("msg", msg);
	cmd_print_list(TG_OK, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);
}
/*
void rpt_print_message_value(char *msg, double value)
{
	cmd_add_string("msg", msg);
	cmd_add_float("v", value);
	cmd_print_list(TG_OK, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);
}
*/
/*
void rpt_print_message_number(uint8_t msgnum) 
{
	char msg[APPLICATION_MESSAGE_LEN];
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgApplicationMessage[msgnum]), APPLICATION_MESSAGE_LEN);
	tg_print_message(msg);
}
*/

/**** Application Messages *********************************************************
 * rpt_print_loading_configs_message()
 * rpt_print_initializing_message()
 * rpt_print_system_ready_message()
 */
void rpt_print_loading_configs_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_reset_list();
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string_P("msg", PSTR("Loading configs from EEPROM"));
	cmd_print_list(TG_INITIALIZING, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
#endif
}

void rpt_print_initializing_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_reset_list();
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string_P("msg", PSTR(INIT_CONFIGURATION_MESSAGE)); // see settings.h & sub-headers
	cmd_print_list(TG_INITIALIZING, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
#endif
}

void rpt_print_system_ready_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_reset_list();
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string_P("msg", PSTR("SYSTEM READY"));
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
#endif
	if (cfg.comm_mode == TEXT_MODE) { tg_text_response(TG_OK, "");}// prompt
}

/*****************************************************************************
 * Status Reports
 *
 *	Status report behaviors
 *
 *	Configuration:
 *
 *		Status reports are configurable only from JSON. SRs are configured
 *		by sending a status report SET object, e.g:
 *
 *		  {"sr":{"line":true,"posx":true,"posy":true....."motm":true,"stat":true}}
 *
 *	Status report formats: The following formats exist for status reports:
 *
 *	  -	JSON format: Returns a JSON object as above, but with the values filled in. 
 *		In JSON form all values are returned as numeric values or enumerations. 
 *		E.g. "posx" is returned as 124.523 and "unit" is returned as 0 for 
 *		inches (G20) and 1 for mm (G21).
 *
 *	  - CSV format: Returns a single line of comma separated token:value pairs.
 *		Values are returned as numeric values or English text.
 *		E.g. "posx" is still returned as 124.523 but "unit" is returned as 
 *		"inch" for inches (G20) and "mm" for mm (G21).
 *
 *	  - Multi-line format: Returns a multi-line report where each value occupies 
 *		one line. Each line contains explanatory English text. Enumerated values are
 *		returned as English text as per CSV form.
 *	
 *	Status report invocation: Status reports can be invoked in the following ways:
 *
 *	  - Ad-hoc request in JSON mode. Issue {"sr":""} (or equivalent). Returns a 
 *		JSON format report (wrapped in a response header, of course).
 *
 *	  - Automatic status reports in JSON mode. Returns JSON format reports 
 *		according to "si" setting.
 *
 *	  - Ad-hoc request in text mode. Triggered by sending ?<cr>. Returns status 
 *		report in multi-line format. Additionally, a line starting with ? will put 
 *		the system into text mode.
 *
 *	  - Automatic status reports in text mode return CSV format according to si setting
 *
 *	  - grbl compatibility forms are not yet supported.
 */

/* rpt_init_status_report()
 *
 *	Call this function to completely re-initialze the status report
 *	Sets SR list to hard-coded defaults and re-initializes sr values in NVM
 */
void rpt_init_status_report(uint8_t persist_flag)
{
	cmdObj_t *cmd = cmd_reset_list();	// used for status report persistence locations
	char sr_defaults[CMD_STATUS_REPORT_LEN][CMD_TOKEN_LEN+1] = { SR_DEFAULTS };	// see settings.h
	cm.status_report_counter = (cfg.status_report_interval / RTC_PERIOD);	// RTC fires every 10 ms

	cmd->index = cmd_get_index("","se00");				// set first SR persistence index
	for (uint8_t i=0; i < CMD_STATUS_REPORT_LEN ; i++) {
		if (sr_defaults[i][0] == NUL) break;			// quit on first blank array entry
		cfg.status_report_value[i] = -1234567;			// pre-load values with an unlikely number
		cmd->value = cmd_get_index("", sr_defaults[i]);	// load the index for the SR element
		cmd_set(cmd);
		cmd_persist(cmd);
		cmd->index++;
	}
	cm.status_report_request = false;
}

/* 
 * rpt_run_text_status_report()	- generate a text mode status report in multiline format
 * rpt_request_status_report()	- request a status report to run after minimum interval
 * rpt_status_report_rtc_callback()	- real-time clock downcount for minimum reporting interval
 * rpt_status_report_callback()	- main loop callback to send a report if one is ready
 *
 *	Status reports can be request from a number of sources including:
 *	  - direct request from command line in the form of ? or {"sr:""}
 *	  - timed requests during machining cycle
 *	  - filtered request after each Gcode block
 *
 *	Status reports are generally returned with minimal delay (from the controller callback), 
 *	but will not be provided more frequently than the status report interval
 */
void rpt_run_text_status_report()
{
	rpt_populate_unfiltered_status_report();
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
}

void rpt_request_status_report()
{
	cm.status_report_request = true;
}

void rpt_status_report_rtc_callback() 		// called by 10ms real-time clock
{
	if (cm.status_report_counter != 0) { cm.status_report_counter--;} // stick at zero
}

uint8_t rpt_status_report_callback() 		// called by controller dispatcher
{
	if ((cfg.status_report_verbosity == SR_OFF) || 
		(cm.status_report_counter != 0) ||
		(cm.status_report_request == false)) {
		return (TG_NOOP);
	}
	// the following could be re-organized but -Os is actually most efficient with this code:
	if ((cfg.comm_mode == JSON_MODE) && (cfg.status_report_verbosity == SR_FILTERED)) {
		if (rpt_populate_filtered_status_report() == true) {
			cmd_print_list(TG_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
		}
	} else {
		rpt_populate_unfiltered_status_report();
		cmd_print_list(TG_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	}
	cm.status_report_counter = (cfg.status_report_interval / RTC_PERIOD);	// reset minimum interval
	cm.status_report_request = false;
	return (TG_OK);
}

/*
 * rpt_populate_unfiltered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be run as a response; i.e. have a "r" header and a footer.
 */

void rpt_populate_unfiltered_status_report()
{
	cmdObj_t *cmd = cmd_reset_list();		// sets cmd to the start of the body
	cmd->type = TYPE_PARENT; 				// setup the parent object
	strcpy(cmd->token, "sr");
//	sprintf_P(cmd->token, PSTR("sr"));		// alternate form of above: less RAM, more FLASH & cycles
	cmd = cmd->nx;

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = cfg.status_report_list[i]) == 0) { break;}
		cmd_get_cmdObj(cmd);
		cmd = cmd->nx;
	}
}

/*
 * rpt_populate_filtered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be displayed as a JSON object; i;e; no footer or header
 *	Returns 'true' if the report has new data, 'false' if there is nothing to report.
 */
uint8_t rpt_populate_filtered_status_report()
{
	uint8_t has_data = false;
	cmdObj_t *cmd = cmd_reset_list();		// sets cmd to the start of the body

	cmd->type = TYPE_PARENT; 				// setup the parent object
	strcpy(cmd->token, "sr");
//	sprintf_P(cmd->token, PSTR("sr"));		// alternate form of above: less RAM, more FLASH & cycles
	cmd = cmd->nx;
	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = cfg.status_report_list[i]) == 0) { break;}
		cmd_get_cmdObj(cmd);
		if (cfg.status_report_value[i] == cmd->value) {	// float == comparison runs the risk of overreporting. So be it
			continue;
		} else {
			cfg.status_report_value[i] = cmd->value;
			cmd = cmd->nx;
//			if (cmd == NULL) { return (false);}	// This is never supposed to happen
			has_data = true;
		}
	}
	cmd->pv->nx = NULL;						// back up one and terminate the body
	return (has_data);
}

/*****************************************************************************
 * Queue Reports
 * rpt_request_queue_report()	- request a queue report with current values
 * rpt_queue_report_callback()	- run the queue report w/stored values
 */

struct qrIndexes {				// static data for queue reports
	uint8_t request;			// set to true to request a report
	uint8_t buffers_available;	// stored value used by callback
	uint8_t prev_available;		// used to filter reports
};
static struct qrIndexes qr;

void rpt_request_queue_report() 
{ 
	if (cfg.enable_qr == QR_OFF) return;

	qr.buffers_available = mp_get_planner_buffers_available();

	// perform filtration for QR_FILTERED reports
	if (cfg.enable_qr == QR_FILTERED) {
		if (qr.buffers_available == qr.prev_available) {
			return;
		}
		if ((qr.buffers_available > cfg.qr_lo_water) && (qr.buffers_available < cfg.qr_hi_water)) {
			return;
		}
	}
	qr.prev_available = qr.buffers_available;
	qr.request = true;
}

uint8_t rpt_queue_report_callback()
{
	if (qr.request == false) { return (TG_NOOP);}
	qr.request = false;

	// cget a clean cmd object
//	cmdObj_t *cmd = cmd_reset_list();		// normally you do a list reset but the following is more time efficient
	cmdObj_t *cmd = cmd_body;
	cmd_reset_obj(cmd);
	cmd->nx = NULL;							// terminate the list

	// make a qr object and print it
	sprintf_P(cmd->token, PSTR("qr"));
	cmd->value = qr.buffers_available;
	cmd->type = TYPE_INTEGER;
	cmd_print_list(TG_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	return (TG_OK);
}

/****************************************************************************
 ***** Report Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_REPORT

void sr_unit_tests(void)
{
	sr_init();
	tg.communications_mode = TG_JSON_MODE;
	sr_run_status_report();
}

#endif
#endif
