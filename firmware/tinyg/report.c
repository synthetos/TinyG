/*
 * report.c - TinyG status report and other reporting functions.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
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
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "planner.h"
#include "settings.h"
#include "util.h"
#include "xio.h"

#ifdef __cplusplus
extern "C"{
#endif

/**** Allocation ****/

srSingleton_t sr;
qrSingleton_t qr;

/**** Exception Reports ************************************************************
 * rpt_exception() - generate an exception message - always in JSON format
 * rpt_er()		   - send a bogus exception report for testing purposes (it's not real)
 *
 * WARNING: Do not call this function from MED or HI interrupts (LO is OK) or there is
 *			a potential for deadlock in the TX buffer.
 */
void rpt_exception(uint8_t status)
{
	if (js.json_syntax == JSON_SYNTAX_RELAXED) {
		printf_P(PSTR("{er:{fb:%0.2f,st:%d,msg:\"%s\"}}\n"),
			TINYG_FIRMWARE_BUILD, status, get_status_message(status));
	} else {
		printf_P(PSTR("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s\"}}\n"),
			TINYG_FIRMWARE_BUILD, status, get_status_message(status));
	}
}

stat_t rpt_er(cmdObj_t *cmd)
{
	rpt_exception(STAT_GENERIC_EXCEPTION_REPORT);	// bogus exception report for testing
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
	cmd_add_string((const char_t *)"msg", pstr2str(msg));	// startup message
	json_print_response(status);
#endif
}

void rpt_print_initializing_message(void)
{
	_startup_helper(STAT_INITIALIZING, PSTR(INIT_MESSAGE));
}

void rpt_print_loading_configs_message(void)
{
	_startup_helper(STAT_INITIALIZING, PSTR("Loading configs from EEPROM"));
}

void rpt_print_system_ready_message(void)
{
	_startup_helper(STAT_OK, PSTR("SYSTEM READY"));
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
static stat_t _populate_unfiltered_status_report(void);
static uint8_t _populate_filtered_status_report(void);

uint8_t _is_stat(cmdObj_t *cmd)
{
	char_t tok[TOKEN_LEN+1];

	GET_TOKEN_STRING(cmd->value, tok);
	if (strcmp(tok, "stat") == 0) { return (true);}
	return (false);
}

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
	char_t sr_defaults[CMD_STATUS_REPORT_LEN][TOKEN_LEN+1] = { SR_DEFAULTS };	// see settings.h
	cmd->index = cmd_get_index((const char_t *)"", (const char_t *)"se00");	// set first SR persistence index
	sr.stat_index = 0;

	for (uint8_t i=0; i < CMD_STATUS_REPORT_LEN ; i++) {
		if (sr_defaults[i][0] == NUL) break;			// quit on first blank array entry
		sr.status_report_value[i] = -1234567;			// pre-load values with an unlikely number
		cmd->value = cmd_get_index((const char_t *)"", sr_defaults[i]);// load the index for the SR element
		if (_is_stat(cmd) == true)
			sr.stat_index = cmd->value;					// identify index for 'stat' if status is in the report
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
	_populate_unfiltered_status_report();			// return current values
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
#ifdef __ARM
	if (request_type == SR_IMMEDIATE_REQUEST) {
		sr.status_report_systick = SysTickTimer.getValue();
	}
	if ((request_type == SR_TIMED_REQUEST) && (sr.status_report_requested == false)) {
		sr.status_report_systick = SysTickTimer.getValue() + sr.status_report_interval;
	}
#endif
#ifdef __AVR
	if (request_type == SR_IMMEDIATE_REQUEST) {
		sr.status_report_systick = SysTickTimer_getValue();
	}
	if ((request_type == SR_TIMED_REQUEST) && (sr.status_report_requested == false)) {
		sr.status_report_systick = SysTickTimer_getValue() + sr.status_report_interval;
	}
#endif
	sr.status_report_requested = true;
	return (STAT_OK);
}

stat_t sr_status_report_callback() 		// called by controller dispatcher
{
#ifdef __SUPPRESS_STATUS_REPORTS
	return (STAT_NOOP);
#endif

	if (sr.status_report_verbosity == SR_OFF) return (STAT_NOOP);
	if (sr.status_report_requested == false) return (STAT_NOOP);
#ifdef __ARM
	if (SysTickTimer.getValue() < sr.status_report_systick) return (STAT_NOOP);
#endif
#ifdef __AVR
	if (SysTickTimer_getValue() < sr.status_report_systick) return (STAT_NOOP);
#endif

	sr.status_report_requested = false;		// disable reports until requested again

	if (sr.status_report_verbosity == SR_VERBOSE) {
		_populate_unfiltered_status_report();
	} else {
		if (_populate_filtered_status_report() == false) {	// no new data
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
	_populate_unfiltered_status_report();
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
	return (STAT_OK);
}

/*
 * _populate_unfiltered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be run as a response; i.e. have a "r" header and a footer.
 */
static stat_t _populate_unfiltered_status_report()
{
	const char_t sr_str[] = "sr";
	char_t tmp[TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets *cmd to the start of the body

	cmd->objtype = TYPE_PARENT; 			// setup the parent object (no length checking required)
	strcpy(cmd->token, sr_str);
	cmd->index = cmd_get_index((const char_t *)"", sr_str);// set the index - may be needed by calling function
	cmd = cmd->nx;							// no need to check for NULL as list has just been reset

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = sr.status_report_list[i]) == 0) { break;}
		cmd_get_cmdObj(cmd);

		strcpy(tmp, cmd->group);			// flatten out groups - WARNING - you cannot use strncpy here...
		strcat(tmp, cmd->token);
		strcpy(cmd->token, tmp);			//...or here.

		if ((cmd = cmd->nx) == NULL) 
			return (cm_hard_alarm(STAT_BUFFER_FULL_FATAL));	// should never be NULL unless SR length exceeds available buffer array
	}
	return (STAT_OK);
}

/*
 * _populate_filtered_status_report() - populate cmdObj body with status values
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
static uint8_t _populate_filtered_status_report()
{
	const char_t sr_str[] = "sr";
	uint8_t has_data = false;
	char_t tmp[TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets cmd to the start of the body

	cmd->objtype = TYPE_PARENT; 			// setup the parent object (no need to length check the copy)
	strcpy(cmd->token, sr_str);
//	cmd->index = cmd_get_index((const char_t *)"", sr_str);// OMITTED - set the index - may be needed by calling function
	cmd = cmd->nx;							// no need to check for NULL as list has just been reset

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = sr.status_report_list[i]) == 0) { break;}

		cmd_get_cmdObj(cmd);
		// do not report values that have not changed...
		// ...except for stat=3 (STOP), which is an exception
		if (fp_EQ(cmd->value, sr.status_report_value[i])) {
//			if (cmd->index != sr.stat_index) {
//				if (fp_EQ(cmd->value, COMBINED_PROGRAM_STOP)) {
					cmd->objtype = TYPE_EMPTY;
					continue;
//				}
//			}
			// report anything that has changed
		} else {
			strcpy(tmp, cmd->group);		// flatten out groups - WARNING - you cannot use strncpy here...
			strcat(tmp, cmd->token);
			strcpy(cmd->token, tmp);		//...or here.
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
stat_t sr_get(cmdObj_t *cmd) { return (_populate_unfiltered_status_report());}
stat_t sr_set(cmdObj_t *cmd) { return (sr_set_status_report(cmd));}

stat_t sr_set_si(cmdObj_t *cmd)
{
	if (cmd->value < STATUS_REPORT_MIN_MS) { cmd->value = STATUS_REPORT_MIN_MS;}
	sr.status_report_interval = (uint32_t)cmd->value;
	return(STAT_OK);
}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_si[] PROGMEM = "[si]  status interval%14.0f ms\n";
static const char fmt_sv[] PROGMEM = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";

void sr_print_sr(cmdObj_t *cmd) { _populate_unfiltered_status_report();}
void sr_print_si(cmdObj_t *cmd) { text_print_flt(cmd, fmt_si);}
void sr_print_sv(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_sv);}

#endif // __TEXT_MODE


/*****************************************************************************
 * Queue Reports
 *
 *	Queue reports can report three values:
 *	  - qr	queue depth - # of buffers availabel in planner queue
 *	  - qi	buffers added to planner queue since las report
 *	  - qo	buffers removed from planner queue since last report
 *
 *	A QR_SINGLE report returns qr only. A QR_TRIPLE returns all 3 values
 *
 *	There are 2 ways to get queue reports:
 *
 *	 1.	Enable single or triple queue reports using the QV variable. This will
 *		return a queue report every time the buffer depth changes
 *
 *	 2.	Add qr, qi and qo (or some combination) to the status report. This will
 *		return queue report data when status reports are generated.
 */
/*
 * qr_init_queue_report() - initialize or clear queue report values
 */
void qr_init_queue_report()
{
	qr.queue_report_requested = false;
	qr.buffers_added = 0;
	qr.buffers_removed = 0;
	qr.init_tick = SysTickTimer_getValue();
}

/*
 * qr_request_queue_report() - request a queue report
 *
 *	Requests a queue report and also records the buffers added and removed
 *	since the last init (usually re-initted when a report is generated).
 */
void qr_request_queue_report(int8_t buffers)
{
	// time-throttle requests while generating arcs
	qr.motion_mode = cm_get_motion_mode(ACTIVE_MODEL);
	if ((qr.motion_mode == MOTION_MODE_CW_ARC) || (qr.motion_mode == MOTION_MODE_CCW_ARC)) {
		uint32_t tick = SysTickTimer_getValue();
		if (tick - qr.init_tick < MIN_ARC_QR_INTERVAL) {
			qr.queue_report_requested = false;
			return;
		}
	}

	// get buffer depth and added/removed count
	qr.buffers_available = mp_get_planner_buffers_available();
	if (buffers > 0) {
		qr.buffers_added += buffers;
	} else {
		qr.buffers_removed -= buffers;
	}

	// either return or request a report
	if (qr.queue_report_verbosity != QR_OFF) {
		qr.queue_report_requested = true;
	}
}

/*
 * qr_queue_report_callback() - generate a queue report if one has been requested
 */
stat_t qr_queue_report_callback() 		// called by controller dispatcher
{
#ifdef __SUPPRESS_QUEUE_REPORTS
	return (STAT_NOOP);
#endif

	if (qr.queue_report_verbosity == QR_OFF) { return (STAT_NOOP);}
	if (qr.queue_report_requested == false) { return (STAT_NOOP);}
	qr.queue_report_requested = false;

	if (cfg.comm_mode == TEXT_MODE) {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			fprintf(stderr, "qr:%d\n", qr.buffers_available);
		} else  {
			fprintf(stderr, "qr:%d, qi:%d, qo:%d\n", qr.buffers_available,qr.buffers_added,qr.buffers_removed);
		}

	} else if (js.json_syntax == JSON_SYNTAX_RELAXED) {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			fprintf(stderr, "{qr:%d}\n", qr.buffers_available);
		} else {
			fprintf(stderr, "{qr:%d,qi:%d,qo:%d}\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
		}

	} else {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			fprintf(stderr, "{\"qr\":%d}\n", qr.buffers_available);
		} else {
			fprintf(stderr, "{\"qr\":%d,\"qi\":%d,\"qo\":%d}\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
		}
	}
	qr_init_queue_report();
	return (STAT_OK);
}

/* Alternate Formulation for a Single report - using cmdObj list

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

/* 
 * Wrappers and Setters - for calling from cmdArray table
 *
 * qr_get() - run a queue report (as data)
 * qi_get() - run a queue report - buffers in
 * qo_get() - run a queue report - buffers out
 */
stat_t qr_get(cmdObj_t *cmd) 
{
	cmd->value = (float)mp_get_planner_buffers_available(); // ensure that manually requested QR count is always up to date
//	cmd->value = (float)qr.buffers_available;
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t qi_get(cmdObj_t *cmd) 
{
	cmd->value = (float)qr.buffers_added;
	cmd->objtype = TYPE_INTEGER;
	qr.buffers_added = 0;				// reset it
	return (STAT_OK);
}

stat_t qo_get(cmdObj_t *cmd) 
{
	cmd->value = (float)qr.buffers_removed;
	cmd->objtype = TYPE_INTEGER;
	qr.buffers_removed = 0;				// reset it
	return (STAT_OK);
}

/*****************************************************************************
 * JOB ID REPORTS
 *
 *	job_populate_job_report()
 *	job_set_job_report()
 *	job_report_callback()
 *	job_get()
 *	job_set()
 *	job_print_job()
 */
stat_t job_populate_job_report()
{
	const char_t job_str[] = "job";
	char_t tmp[TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets *cmd to the start of the body

	cmd->objtype = TYPE_PARENT; 			// setup the parent object
	strcpy(cmd->token, job_str);

	//cmd->index = cmd_get_index((const char_t *)"", job_str);// set the index - may be needed by calling function
	cmd = cmd->nx;							// no need to check for NULL as list has just been reset

	index_t job_start = cmd_get_index((const char_t *)"",(const char_t *)"job1");// set first job persistence index
	for (uint8_t i=0; i<4; i++) {
		
		cmd->index = job_start + i;
		cmd_get_cmdObj(cmd);

		strcpy(tmp, cmd->group);			// concatenate groups and tokens - do NOT use strncpy()
		strcat(tmp, cmd->token);
		strcpy(cmd->token, tmp);

		if ((cmd = cmd->nx) == NULL) return (STAT_OK); // should never be NULL unless SR length exceeds available buffer array 
	}
	return (STAT_OK);
}

stat_t job_set_job_report(cmdObj_t *cmd)
{
	index_t job_start = cmd_get_index((const char_t *)"",(const char_t *)"job1");// set first job persistence index

	for (uint8_t i=0; i<4; i++) {
		if (((cmd = cmd->nx) == NULL) || (cmd->objtype == TYPE_EMPTY)) { break;}
		if (cmd->objtype == TYPE_INTEGER) {
			cs.job_id[i] = cmd->value;
			cmd->index = job_start + i;					// index of the SR persistence location
			cmd_persist(cmd);
		} else {
			return (STAT_INPUT_VALUE_UNSUPPORTED);
		}
	}
	job_populate_job_report();			// return current values
	return (STAT_OK);
}

uint8_t job_report_callback()
{
	if (cfg.comm_mode == TEXT_MODE) {
		// no-op, job_ids are client app state
	} else if (js.json_syntax == JSON_SYNTAX_RELAXED) {
		fprintf(stderr, "{job:[%lu,%lu,%lu,%lu]}\n", cs.job_id[0], cs.job_id[1], cs.job_id[2], cs.job_id[3] );
	} else {
		fprintf(stderr, "{\"job\":[%lu,%lu,%lu,%lu]}\n", cs.job_id[0], cs.job_id[1], cs.job_id[2], cs.job_id[3] );
		//job_clear_report();
	}
	return (STAT_OK);
}

stat_t job_get(cmdObj_t *cmd) { return (job_populate_job_report());}
stat_t job_set(cmdObj_t *cmd) { return (job_set_job_report(cmd));}
void job_print_job(cmdObj_t *cmd) { job_populate_job_report();}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_qr[] PROGMEM = "qr:%d\n";
static const char fmt_qi[] PROGMEM = "qi:%d\n";
static const char fmt_qo[] PROGMEM = "qo:%d\n";
static const char fmt_qv[] PROGMEM = "[qv]  queue report verbosity%7d [0=off,1=single,2=triple]\n";

void qr_print_qr(cmdObj_t *cmd) { text_print_int(cmd, fmt_qr);}
void qr_print_qi(cmdObj_t *cmd) { text_print_int(cmd, fmt_qi);}
void qr_print_qo(cmdObj_t *cmd) { text_print_int(cmd, fmt_qo);}
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
#endif	// __UNIT_TESTS
#endif	// __UNIT_TESTS_REPORT

#ifdef __cplusplus
}
#endif
