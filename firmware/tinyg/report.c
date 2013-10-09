/*
 * report.c - TinyG status report and other reporting functions.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S Hart, Jr.
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
#include "report.h"
#include "json_parser.h"
#include "text_parser.h"
#include "planner.h"
#include "settings.h"
#include "util.h"
#include "xio/xio.h"
#include "xmega/xmega_rtc.h"

#ifdef __cplusplus
extern "C"{
#endif

/**** Allocation ****/

srSingleton_t sr;
qrSingleton_t qr;

/**** Exception Messages ************************************************************
 * rpt_exception() - generate an exception message - always in JSON format
 * rpt_er()		   - send a bogus exception report for testing purposes (it's not real)
 */
//void rpt_exception(uint8_t status, int16_t value)
void rpt_exception(uint8_t status)
{
	printf_P(PSTR("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s\"}}\n"), 
		TINYG_FIRMWARE_BUILD, status, get_status_message(status));

//	printf_P(PSTR("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s\",\"val\":%d}}\n"), 
//		TINYG_FIRMWARE_BUILD, status, get_status_message(status), value);
}

stat_t rpt_er(cmdObj_t *cmd) 
{
	rpt_exception(STAT_GENERIC_EXCEPTION_REPORT);	// bogus exception report
	return (STAT_OK);
}


/**** Application Messages *********************************************************
 * rpt_print_initializing_message()	   - initializing configs from hard-coded profile
 * rpt_print_loading_configs_message() - loading configs from EEPROM
 * rpt_print_system_ready_message()    - system ready message
 *
 *	These messages are always in JSON format to allow UIs to sync
 */

void _startup_helper(stat_t status, const char *msg)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	js.json_footer_depth = JSON_FOOTER_DEPTH;	//++++ temporary until changeover is complete
	cmd_reset_list();
	cmd_add_object((const char_t *)"fv");		// firmware version
	cmd_add_object((const char_t *)"fb");		// firmware build
	cmd_add_object((const char_t *)"hp");		// hardware platform
	cmd_add_object((const char_t *)"hv");		// hardware version
	cmd_add_object((const char_t *)"id");		// hardware ID
	cmd_add_string((const char_t *)"msg", (const char_t *)msg);	// startup message
	json_print_response(status);
#endif
}

void rpt_print_initializing_message(void)
{
	_startup_helper(STAT_INITIALIZING, (const char_t *)INIT_MESSAGE);
}

void rpt_print_loading_configs_message(void)
{
	_startup_helper(STAT_INITIALIZING, (const char_t *)"Loading configs from EEPROM");
}

void rpt_print_system_ready_message(void)
{
	_startup_helper(STAT_OK, (const char_t *)"SYSTEM READY");
	if (cfg.comm_mode == TEXT_MODE) { text_response(STAT_OK, (char_t *)"");}// prompt
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
 */

/* 
 * sr_init_status_report()
 *
 *	Call this function to completely re-initialize the status report
 *	Sets SR list to hard-coded defaults and re-initializes SR values in NVM
 */
void sr_init_status_report()
{
	cmdObj_t *cmd = cmd_reset_list();	// used for status report persistence locations
	sr.status_report_requested = false;
	char_t sr_defaults[CMD_STATUS_REPORT_LEN][CMD_TOKEN_LEN+1] = { SR_DEFAULTS };	// see settings.h
	cmd->index = cmd_get_index((const char_t *)"", (const char_t *)"se00");	// set first SR persistence index

	for (uint8_t i=0; i < CMD_STATUS_REPORT_LEN ; i++) {
		if (sr_defaults[i][0] == NUL) break;			// quit on first blank array entry
		sr.status_report_value[i] = -1234567;			// pre-load values with an unlikely number
		cmd->value = cmd_get_index((const char_t *)"", sr_defaults[i]);// load the index for the SR element
		cmd_set(cmd);
		cmd_persist(cmd);								// conditionally persist - automatic by cmd_persis()
		cmd->index++;									// increment SR NVM index
	}
}

/* 
 * sr_set_status_report() - interpret an SR setup string and return current report
 */
stat_t sr_set_status_report(cmdObj_t *cmd)
{
	uint8_t elements = 0;
	index_t status_report_list[CMD_STATUS_REPORT_LEN];
	memset(status_report_list, 0, sizeof(status_report_list));
	index_t sr_start = cmd_get_index((const char_t *)"",(const char_t *)"se00");// set first SR persistence index

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if (((cmd = cmd->nx) == NULL) || (cmd->objtype == TYPE_EMPTY)) { break;}
		if ((cmd->objtype == TYPE_BOOL) && (fp_TRUE(cmd->value))) {
			status_report_list[i] = cmd->index;
			cmd->value = cmd->index;					// persist the index as the value
			cmd->index = sr_start + i;					// index of the SR persistence location
			cmd_persist(cmd);
			elements++;
		} else {
			return (STAT_INPUT_VALUE_UNSUPPORTED);
		}
	}
	if (elements == 0) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	memcpy(sr.status_report_list, status_report_list, sizeof(status_report_list));
	sr_populate_unfiltered_status_report();			// return current values
	return (STAT_OK);
}

/* 
 * sr_request_status_report()	- request a status report to run after minimum interval
 * sr_status_report_callback()	- main loop callback to send a report if one is ready
 *
 *	Status reports can be request from a number of sources including:
 *	  - direct request from command line in the form of ? or {"sr:""}
 *	  - timed requests during machining cycle
 *	  - filtered request after each Gcode block
 *
 *	Status reports are generally returned with minimal delay (from the controller callback), 
 *	but will not be provided more frequently than the status report interval
 */
stat_t sr_request_status_report(uint8_t request_type)
{
	if (request_type == SR_IMMEDIATE_REQUEST) {
		sr.status_report_systick = SysTickTimer_getValue();
	}
	if ((request_type == SR_TIMED_REQUEST) && (sr.status_report_requested == false)) {
		sr.status_report_systick = SysTickTimer_getValue() + sr.status_report_interval;
	}
	sr.status_report_requested = true;
	return (STAT_OK);
}

stat_t sr_status_report_callback() 		// called by controller dispatcher
{
	if (sr.status_report_verbosity == SR_OFF) return (STAT_NOOP);
	if (sr.status_report_requested == false) return (STAT_NOOP);
	if (SysTickTimer_getValue() < sr.status_report_systick) return (STAT_NOOP);

	sr.status_report_requested = false;		// disable reports until requested again

	if (sr.status_report_verbosity == SR_VERBOSE) {
		sr_populate_unfiltered_status_report();
	} else {
		if (sr_populate_filtered_status_report() == false) {	// no new data
			return (STAT_OK);
		}
	}
	cmd_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	return (STAT_OK);
}

/* 
 * sr_run_text_status_report() - generate a text mode status report in multiline format
 */
stat_t sr_run_text_status_report()
{
	sr_populate_unfiltered_status_report();
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
	return (STAT_OK);
}

/*
 * sr_populate_unfiltered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be run as a response; i.e. have a "r" header and a footer.
 */

stat_t sr_populate_unfiltered_status_report()
{
	const char_t sr_str[] = "sr";
	char_t tmp[CMD_TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets *cmd to the start of the body

	cmd->objtype = TYPE_PARENT; 			// setup the parent object
	strcpy(cmd->token, sr_str);
	cmd->index = cmd_get_index((const char_t *)"", sr_str);// set the index - may be needed by calling function
	cmd = cmd->nx;							// no need to check for NULL as list has just been reset

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = sr.status_report_list[i]) == 0) { break;}
		cmd_get_cmdObj(cmd);
		strcpy(tmp, cmd->group);			// concatenate groups and tokens
		strcat(tmp, cmd->token);
		strcpy(cmd->token, tmp);
		if ((cmd = cmd->nx) == NULL) 
			return (cm_alarm(STAT_BUFFER_FULL_FATAL));	// should never be NULL unless SR length exceeds available buffer array 
	}
	return (STAT_OK);
}

/*
 * sr_populate_filtered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be displayed as a JSON object; i;e; no footer or header
 *	Returns 'true' if the report has new data, 'false' if there is nothing to report.
 *
 *	NOTE: Unlike sr_populate_unfiltered_status_report(), this function does NOT set 
 *	the SR index, which is a relatively expensive operation. In current use this 
 *	doesn't matter, but if the caller assumes its set it may lead to a side-effect (bug)
 *
 *	NOTE: Room for improvement - look up the SR index initially and cache it, use the 
 *		  cached value for all remaining reports.
 */
uint8_t sr_populate_filtered_status_report()
{
	const char_t sr_str[] = "sr";
	uint8_t has_data = false;
	char_t tmp[CMD_TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets cmd to the start of the body

	cmd->objtype = TYPE_PARENT; 			// setup the parent object
	strcpy(cmd->token, sr_str);
//	cmd->index = cmd_get_index((const char_t *)"", sr_str);// OMITTED - set the index - may be needed by calling function
	cmd = cmd->nx;							// no need to check for NULL as list has just been reset

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = sr.status_report_list[i]) == 0) { break;}

		cmd_get_cmdObj(cmd);
		if (fp_EQ(cmd->value, sr.status_report_value[i])) {
			cmd->objtype = TYPE_EMPTY;
			continue;
		} else {
			strcpy(tmp, cmd->group);		// flatten out groups
			strcat(tmp, cmd->token);
			strcpy(cmd->token, tmp);
			sr.status_report_value[i] = cmd->value;
			if ((cmd = cmd->nx) == NULL) return (false); // should never be NULL unless SR length exceeds available buffer array
			has_data = true;
		}
	}
	return (has_data);
}

/* 
 * Wrappers and Setters - for calling from cmdArray table
 *
 * sr_get()		- run status report
 * sr_set()		- set status report elements
 * sr_set_si()	- set status report interval
 */

stat_t sr_get(cmdObj_t *cmd) { return (sr_populate_unfiltered_status_report());}
stat_t sr_set(cmdObj_t *cmd) { return (sr_set_status_report(cmd));}

stat_t sr_set_si(cmdObj_t *cmd)
{
	if (cmd->value < STATUS_REPORT_MIN_MS) { cmd->value = STATUS_REPORT_MIN_MS;}
	sr.status_report_interval = (uint32_t)cmd->value;
	return(STAT_OK);
}

/*****************************************************************************
 * Queue Reports
 *
 * qr_get() 					- run a queue report (as data)
 * qr_clear_queue_report()		- wipe stored values
 * qr_request_queue_report()	- request a queue report with current values
 * qr_queue_report_callback()	- run the queue report w/stored values
 */

stat_t qr_get(cmdObj_t *cmd) 
{
	cmd->value = (float)mp_get_planner_buffers_available();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

void qr_clear_queue_report()
{
	qr.request = false;
	qr.buffers_added = 0;
	qr.buffers_removed = 0;
}

void qr_request_queue_report(int8_t buffers)
{
	if (qr.queue_report_verbosity == QR_OFF) return;

	qr.buffers_available = mp_get_planner_buffers_available();

	if (buffers > 0) {
		qr.buffers_added += buffers;
	} else {
		qr.buffers_removed -= buffers;
	}

	// perform filtration for QR_FILTERED reports
	if (qr.queue_report_verbosity == QR_FILTERED) {
		if (qr.buffers_available == qr.prev_available) {
			return;
		}
		if ((qr.buffers_available > qr.queue_report_lo_water) && 	// e.g. > 2 buffers available
			(qr.buffers_available < qr.queue_report_hi_water)) {	// e.g. < 20 buffers available
			return;
		}
	}
	qr.prev_available = qr.buffers_available;
	qr.request = true;
}

uint8_t qr_queue_report_callback()
{
	if (qr.request == false) { return (STAT_NOOP);}
	qr.request = false;

	if (cfg.comm_mode == TEXT_MODE) {
		if (qr.queue_report_verbosity == QR_VERBOSE) {
			fprintf(stderr, "qr:%d\n", qr.buffers_available);
		} else  {
			if (qr.queue_report_verbosity == QR_TRIPLE) {
				fprintf(stderr, "qr:%d,added:%d,removed:%d\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
			}
		}
	} else {
		if (qr.queue_report_verbosity == QR_VERBOSE) {
			fprintf(stderr, "{\"qr\":%d}\n", qr.buffers_available);
		} else {
			if (qr.queue_report_verbosity == QR_TRIPLE) {
				fprintf(stderr, "{\"qr\":[%d,%d,%d]}\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
				qr_clear_queue_report();
			}
		}
	}
	return (STAT_OK);
}
/* Alternate Formulation - using cmdObj list

	// get a clean cmd object
//	cmdObj_t *cmd = cmd_reset_list();		// normally you do a list reset but the following is more time efficient
	cmdObj_t *cmd = cmd_body;
	cmd_reset_obj(cmd);
	cmd->nx = NULL;							// terminate the list

	// make a qr object and print it
	sprintf_P(cmd->token, PSTR("qr"));
	cmd->value = qr.buffers_available;
	cmd->objtype = TYPE_INTEGER;
	cmd_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	return (STAT_OK);
*/

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE
/*
 * sr_print_sr() - produce SR text output
 */
static const char fmt_si[] PROGMEM = "[si]  status interval%14.0f ms\n";
static const char fmt_sv[] PROGMEM = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";

void sr_print_sr(cmdObj_t *cmd) { sr_populate_unfiltered_status_report();}
void sr_print_si(cmdObj_t *cmd) { text_print_flt(cmd, fmt_si);}
void sr_print_sv(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_sv);}

/*
 * qr_print_qr() - produce QR text output
 */
static const char fmt_qr[] PROGMEM = "qr:%d\n";
static const char fmt_qv[] PROGMEM = "[qv]  queue report verbosity%7d [0=off,1=filtered,2=verbose]\n";

void qr_print_qr(cmdObj_t *cmd) { text_print_int(cmd, fmt_qr);}
void qr_print_qv(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_qv);}

#endif // __TEXT_MODE


/****************************************************************************
 ***** Unit Tests ***********************************************************
 ****************************************************************************/

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_REPORT

void sr_unit_tests(void)
{
	sr_init();
	cs.communications_mode = STAT_JSON_MODE;
	sr_run_status_report();
}

#endif
#endif

#ifdef __cplusplus
}
#endif
