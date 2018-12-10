/*
 * report.c - TinyG status report and other reporting functions.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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
/*
 * This file contains code for the following reports:
 *  - System Startup Reports
 *  - Exception Reports
 *  - Status Reports
 *  - Queue Reports
 *  - RX Reports
 *  - Job ID Reports
 */

#include "tinyg.h"
#include "config.h"
#include "report.h"
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "planner.h"
#include "settings.h"
#include "persistence.h"
#include "util.h"
#include "xio.h"

/**** Allocation ****/

srSingleton_t sr;
qrSingleton_t qr;
rxSingleton_t rx;

/***********************************************************************************/
/**** System Startup Reports *******************************************************/
/***********************************************************************************/
/* rpt_print_initializing_message()	   - initializing configs from hard-coded profile
 * rpt_print_loading_configs_message() - loading configs from EEPROM
 * rpt_print_system_ready_message()    - system ready message
 *
 *	These messages are always in strict JSON format to allow UIs to sync
 */

void _startup_helper(stat_t status, const char *msg)
{
	nv_reset_nv_list("sr");
	nv_add_object((const char *)"fv");		// firmware version
	nv_add_object((const char *)"fb");		// firmware build
	nv_add_object((const char *)"hp");		// hardware platform
	nv_add_object((const char *)"hv");		// hardware version
	nv_add_object((const char *)"id");		// hardware ID
	nv_add_string((const char *)"msg", pstr2str(msg));	// startup message
    json_print_object(NV_HEAD);
}

void rpt_print_initializing_message(void)
{
	_startup_helper(STAT_INITIALIZING, PSTR(INIT_MESSAGE));
}

void rpt_print_loading_configs_message(void)
{
    js.json_syntax = JSON_SYNTAX_STRICT;    // always do this message strict
	_startup_helper(STAT_INITIALIZING, PSTR("Loading configs from EEPROM"));
}

void rpt_print_system_ready_message(void)
{
	_startup_helper(STAT_OK, PSTR("SYSTEM READY"));
	if (cs.comm_mode == TEXT_MODE) {
        text_response(STAT_OK, (char *)"");   // prompt
    }
}

/***********************************************************************************/
/**** Exception Reports ************************************************************/
/***********************************************************************************/
/*
 * rpt_exception()   - generate an exception message - always in JSON format
 * rpt_exception_P() - generate an exception message with message from program space
 * rpt_er()	         - send a bogus exception report for testing purposes (it's not real)
 *
 * Returns incoming status value
 *
 * WARNING: Do not call these functions from MED or HI interrupts (LO is OK)
 *			or there is a potential for deadlock in the TX buffer.
 */

stat_t rpt_exception_P(stat_t status, const char *msg_P)
{
    char msg_buf[TEXT_ITEM_LEN];
    strcpy_P(msg_buf, msg_P);
    return(rpt_exception(status, msg_buf));
}

stat_t rpt_exception(stat_t status, const char *msg)
{
    if (status != STAT_OK) { // makes it possible to call exception reports w/o checking status value
        char line_msg[LINE_MSG_LEN];
        sprintf_P(line_msg, PSTR("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s - %s\"}}\n"),
            TINYG_FIRMWARE_BUILD, status, get_status_message(status), msg);
        printf(json_relax(line_msg));
    }
    return (status);			// makes it possible to inline, e.g: return(rpt_exception(status, msg));
}

stat_t rpt_er(nvObj_t *nv)      // bogus exception report for testing
{
	return(rpt_exception_P(STAT_GENERIC_EXCEPTION_REPORT, PSTR("bogus exception report")));
}

/***********************************************************************************/
/**** Status Reports ***************************************************************/
/***********************************************************************************/
/*
 *	Status reports use 2 JSON objects:
 *  {sr:n}            request status repoort
 *  {srs:{....}}      manage (set) status reports
 *
 *  {sr:null} requests a status report. Returned in an r{} with a footer,
 *  i.e. no changes from current behavior
 *
 *  {sr:{line:t, posx:t}} is the legacy setup behavior, but will be deprecated
 *  and eventually removed. When a line of this form is interpreted the previous
 *  SR list is removed and only those items on the new sr{} line are set.
 *  All items must have a value of 'true' (or 't').
 *
 *  {srs:... Keys under srs should be the "verb" or action to take on the sr.
 *  The contents of that verb/action are the parameters of the action.
 *
 *    set: {srs:{set:{line:t, posx:f}}} add/remove elements from SR reports
 *    clear: {srs:{clear:t}} remove all items from SR report. Must be "true"
 *    defa: {srs:{defa:t}} reset SR report to "factory defaults. Must be "true"
 *
 *
 *  Notes:
 *
 *	Status reports are configurable only from JSON. Text mode can only request
 *    a status report using '?' or $sr
 *
 *	Status report invocation: Status reports can be invoked in the following ways:
 *
 *	  - Ad-hoc request in JSON mode. Issue {"sr":n}. Returns a
 *		JSON format report (wrapped in a response header, of course).
 *
 *	  - Automatic status reports in JSON mode. Returns JSON format reports
 *		according to "si" setting.
 *
 *	  - Ad-hoc request in text mode. Triggered by sending ?<cr>. Returns status
 *		report in multi-line format. Additionally, a line starting with ? will put
 *		the system into text mode.
 *
 *	  - Automatic status reports in text mode return CSV format according to $si setting
 */

static stat_t _populate_unfiltered_status_report(char *key);
static bool _populate_filtered_status_report(void);
static stat_t _set_sr(nvObj_t *nv);
static stat_t _set_srs(nvObj_t *nv);

/*
 * _helpers
 */

static void _persist_status_report_list()
{
    nvObj_t nv;                                         // local working object
    nv.index = nv_get_index("","se00");                 // set first SR persistence index
    nv.valuetype = TYPE_INTEGER;
    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        nv.value_int = sr.status_report_list[i];
        nv_persist(&nv);
        nv.index++;                                     // index of the next SR persistence location
    }
}

/*
 * sr_init_status_report_P() - initialize status report from CSV string (in PROGMEM)
 *
 *  SR settings are not initialized by reading NVram during the system load process.
 *  Instead they are loaded by running this function:
 *    - If sr_csv_P is present load settings from the sr_csv list and persist to NVram
 *    - If sr_csv_P is not present load settings from NVram
 *
 *  sr_csv_P is a comma-separate-value list in program memory. Spaces are not allowed.
 *  Will fail silently if SR string exceeds available space. Fills all slots then truncates.
 */

void sr_init_status_report_P(const char *sr_csv_P)
{
    char sr_csv[ NV_STATUS_REPORT_LEN * (TOKEN_LEN+1) ]; strcpy_P(sr_csv, sr_csv_P);
    uint8_t i;
    nvObj_t nv;

    nv.index = nv_get_index("", "se00");                    // set first SR persistence index
    nv.valuetype = TYPE_INTEGER;
    sr.stat_index = nv_get_index("", "stat");               // set index of stat element
    sr.status_report_request = SR_OFF;                      // clear any current requests

    // SR CSV list is NULL, load SR from NVram
    if (*sr_csv == NUL) {
        for (i=0; i<NV_STATUS_REPORT_LEN; i++) {
            read_persistent_value(&nv);                     // read token index from NVram into nv->value_int element
            sr.status_report_list[i] = nv.value_int;        // load into the active SR list
            sr.value_flt[i] = 8675309;                      // pre-load SR values with an unlikely number
            nv.index++;                                     // increment SR NVM index
        }
    }
    else { // load the sr_csv_P list provided as an arg and persist it NVram
        char *rd = strtok(sr_csv, ",");                     // initialize strtok & get pointer for token parsing
    	for (i=0; i<NV_STATUS_REPORT_LEN; i++) {            // initialize the SR list
            if (rd == NULL) {
                nv.value_int = NO_MATCH;                    // ensures unused positions are disabled (-1)
            } else {
                if ((nv.value_int = nv_get_index("", rd)) == NO_MATCH) {
                    rpt_exception(STAT_BAD_STATUS_REPORT_SETTING, rd);  // trap mis-configured profile settings
                }
            }
            nv_set(&nv);
            nv_persist(&nv);                                // conditionally persist - automatic by nv_persist()
            sr.value_flt[i] = 8675309;                      // pre-load SR values with an unlisted number
            nv.index++;                                     // increment SR NVM index
            rd = strtok(NULL, ",");                         // next strtok() call
        }
    }
}

/*
 * _set_sr() - legancy SR setter. Called as (sr:{...}}
 */

static stat_t _set_sr(nvObj_t *nv)
{
    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if (((nv = nv_next(nv)) == NULL) || (nv->valuetype == TYPE_EMPTY)) {
            sr.status_report_list[i] = NO_MATCH;
        }
        else if ((nv->valuetype == TYPE_BOOLEAN) && (nv->value_int)) {   // key_to_set : true
            sr.status_report_list[i] = nv->index;       // save the index
            sr.value_flt[i] = 8675309;                  // reset the filter term to an unlisted number
        }
    }
    _persist_status_report_list();
    return (STAT_OK);
}

/*
 * _set_srs() - interpret an SRS setup string and return current report
 *
 *  SRS supports the following functions:
 *    {srs:{set:{....}}} add and remove items form status report list
 *    {srs:{set:n}}      query items in SR list
 *    {srs:{clear:t}}    clear SR list
 *    {srs:{defa:t}}     reset SR list to defaults
 *
 *    - Add/remove objects may have a mix of t and f pairs
 *    - List ordering is not guaranteed for mixed adds & removes
 *    - On entry nv points to the parent "srs" object
 *
 *  Error conditions:
 *    - All failures leave original SR list untouched
 *    - An attempt to add an element that exceeds list max fails with STAT_INPUT_EXCEEDS_MAX_LENGTH
 *    - A value other than 't', or 'f' fails with STAT_INPUT_VALUE_RANGE_ERROR
 *    - Malformed JSON fails upstream in the JSON parser, such as...
 *    - Unrecognized tokens (keys) are fail in the JSON parser as STAT_UNRECOGNIZED_NAME
 */

static stat_t _set_srs(nvObj_t *nv)
{
    // advance off the 'srs' parent to the SRS verb
    if ((nv = nv_next(nv)) == NULL) { return (STAT_INTERNAL_ERROR); }

    // {srs:{clear:t}}  clear SR list
    if (nv->token[0] == 'c') {
        if (nv->value_int) {
	        for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) { // clear all SR settings
    	        sr.status_report_list[i] = NO_MATCH;
	        }
	        _persist_status_report_list();
        }
        return (STAT_OK);
    }

    // {srs:{defa:t}}  Reset SR list to defaults
    if (nv->token[0] == 'd') {
        if (nv->value_int) {
            sr_init_status_report_P(SR_DEFAULTS);
        }
        return (STAT_OK);
    }

    // Add/Remove items from SR list

    // initialize the working list from the current SR list
    int8_t i, j;
    index_t working_list[SR_WORKING_LIST_LEN];
	for (i=0; i<SR_WORKING_LIST_LEN; i++) {
        if (i<NV_STATUS_REPORT_LEN) {
        	working_list[i] = sr.status_report_list[i]; // copy in the current SR list
        } else {
    	    working_list[i] = NO_MATCH;                 //...then fill the rest with -1's
        }
	}

    // process {sr:{.... process one or more SR add/deletes (this is all so much easier in Python)
	for (i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if ((nv = nv_next(nv)) == NULL) {               // advance to next element (past the "set" parent)
            return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
        }
		if (nv->valuetype == TYPE_EMPTY) { break; }     // end of items

        // type check the NV - this NV pair was not previously type-checked
		if ((nv->valuetype == TYPE_FLOAT) || (nv->valuetype == TYPE_STRING)) {
            return (STAT_VALUE_TYPE_ERROR);
        }

        if (nv->value_int) {                            // add an item from the working list
            int8_t slot = NO_MATCH;                     // index of first available slot
            bool unique = true;                         // flag if item was not found (passed uniqueness)
            for (j=0; j<SR_WORKING_LIST_LEN; j++) {
                if ((slot == NO_MATCH) && (working_list[j] == NO_MATCH)) {
                    slot = j;
                    continue;
                }
                if (working_list[j] == nv->index) {
                    unique = false;
                    break;
                }
            }
            if (unique && (slot != NO_MATCH)) {
                working_list[slot] = nv->index;
            }
        }
        else {                                          // remove an item to the working list
            for (j=0; j<SR_WORKING_LIST_LEN; j++) {
                if (working_list[j] == nv->index) {     // item exists in working list
                    working_list[j] = -2;               // flag for deletion
                    break;
                }
            }
        }
	}

    // copy the working list to the SR list; use i as read pointer, j as write pointer
    for (i=0, j=0; i<SR_WORKING_LIST_LEN; i++) {
        sr.value_flt[i] = 8675309;                      // reset all filter terms
        if (working_list[i] == -2) { continue; }        // skip deleted elements
        sr.status_report_list[j] = working_list[i];
        if (++j >= NV_STATUS_REPORT_LEN) {
            break;
        }
    }
    _persist_status_report_list();
    return (STAT_OK);
}

/*
 * sr_request_status_report()	- request a status report to run after minimum interval
 * sr_status_report_callback()	- main loop callback to send a report if one is ready
 *
 *	Status reports can be requested from a number of sources including:
 *	  - direct request from command line in the form of ? or {sr:n}
 *	  - timed requests during machining cycle
 *	  - filtered request after each Gcode block
 *
 *	Status reports are generally returned with minimal delay (from the controller callback),
 *	but will not be provided more frequently than the status report interval
 */

stat_t sr_request_status_report(uint8_t request_type)
{
	if (sr.status_report_request != SR_OFF) {       // ignore multiple requests. First one wins.
    	return (STAT_OK);
	}

	sr.status_report_systick = SysTickTimer_getValue();
	if (request_type == SR_REQUEST_ASAP) {
//    	sr.status_report_request = SR_FILTERED;		// will trigger a filtered or verbose report depending on verbosity setting
    	sr.status_report_request = sr.status_report_verbosity; // will trigger a filtered or verbose report depending on verbosity setting
    }
    else if (request_type == SR_REQUEST_ASAP_UNFILTERED) {
    	sr.status_report_request = SR_VERBOSE;		// will trigger a verbose report, regardless of verbosity setting
    }
    else if (request_type == SR_REQUEST_TIMED) {
    	sr.status_report_request = sr.status_report_verbosity;
    	sr.status_report_systick += sr.status_report_interval;
    }
    else {
    	sr.status_report_request = SR_VERBOSE;
    	sr.status_report_systick += sr.status_report_interval;
	}
	return (STAT_OK);
}

stat_t sr_status_report_callback() 		// called by controller dispatcher
{
    // conditions where autogenerated SRs will not be returned
    if ((sr.status_report_request == SR_OFF) || (sr.status_report_verbosity == SR_OFF)) {
        return (STAT_NOOP);
    }
    if (SysTickTimer_getValue() < sr.status_report_systick) {
        return (STAT_NOOP);
    }
    if (sr.status_report_request == SR_VERBOSE) {
        _populate_unfiltered_status_report("sr");
    } else {
        if (_populate_filtered_status_report() == false) {	// no new data
            return (STAT_OK);
        }
    }
    sr.status_report_request = SR_OFF;
    nv_print_list(STAT_OK, TEXT_RESPONSE, JSON_OBJECT);
    return (STAT_OK);
}

/*
 * sr_run_text_status_report() - generate a text mode status report in multiline format
 */
stat_t sr_run_text_status_report()
{
	_populate_unfiltered_status_report("sr");
    text_print_list(STAT_OK, TEXT_RESPONSE);
	return (STAT_OK);
}

/*
 * _populate_unfiltered_status_report() - populate nvObj body with status values
 *
 *  'key' should be "sr" or "srs", depending on who's calling
 */
static stat_t _populate_unfiltered_status_report(char *key)
{
	char tmp[TOKEN_LEN+1];
	nvObj_t *nv = nv_reset_nv_list(key);
 	nv = nv_next(nv);	                // set *nv to the first empty pair past the SR parent

	for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
		if ((nv->index = sr.status_report_list[i]) == NO_MATCH) { break;}
		nv_populate_nv_by_index(nv, nv->index);

		strcpy(tmp, nv->group);			// flatten out groups - WARNING - you cannot use strncpy here...
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);			//...or here.

		if ((nv = nv_next(nv)) == NULL) { // should never be NULL unless SR length exceeds available buffer array
			return (cm_panic_P(STAT_BUFFER_FULL_FATAL, PSTR("_populate_unfiltered_status_report")));
        }
	}
	return (STAT_OK);
}

/*
 * _populate_filtered_status_report() - populate nvObj body with status values
 *
 *	Designed to be displayed as a JSON object; i.e. no footer or header
 *	Returns 'true' if the report has new data, 'false' if there is nothing to report.
 *
 *	NOTE: Unlike sr_populate_unfiltered_status_report(), this function does NOT set
 *	the SR index, which is a relatively expensive operation. In current use this
 *	doesn't matter, but if the caller assumes its set it may lead to a side-effect (bug)
 *
 *	NOTE: Room for improvement - look up the SR index initially and cache it, use the
 *		  cached value for all remaining reports.
 */

static bool _populate_filtered_status_report()
{
	char tmp[TOKEN_LEN+1];
	uint8_t has_data = false;
	nvObj_t *nv = nv_reset_nv_list("sr");   // initialize nv_list as a status report
    nv = nv_next(nv);                       // set *nv to the first empty pair past the SR parent

	for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
		if ((nv->index = sr.status_report_list[i]) == NO_MATCH) { // normal list termination or error
            break;
        }
		nv_populate_nv_by_index(nv, nv->index);     // get the current value and other NV parameters

		// Special handling for stat values - always report the end conditions
        if (nv->index == sr.stat_index) {
            if ((nv->value_int == COMBINED_PROGRAM_STOP) || (nv->value_int == COMBINED_PROGRAM_END)) {
                sr.value_int[i] = nv->value_int;
                nv = nv_next(nv);
    			has_data = true;
                continue;
            }
        }

		// Only report values that have changed
        if ((nv->valuetype == TYPE_INTEGER) || (nv->valuetype == TYPE_SIGNED)) {
            if (nv->value_int == sr.value_int[i]) {
                nv->valuetype = TYPE_EMPTY;
                continue;
            }
		    sr.value_int[i] = nv->value_int;
        } else { // (nv->valuetype == TYPE_FLOAT)
            if (fabs(nv->value_flt - sr.value_flt[i]) < SR_MATCH_PRECISION) {
                nv->valuetype = TYPE_EMPTY;
                continue;
            }
            sr.value_flt[i] = nv->value_flt;
        }

        // flatten out groups - WARNING - you cannot use strncpy here...
		strcpy(tmp, nv->group);
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);		        //...or here.

		if ((nv = nv_next(nv)) == NULL) {   // should never be NULL unless SR length exceeds available buffer array
    		return (false);
		}
		has_data = true;
	}
	return (has_data);
}

/*
 * Wrappers and Setters - for calling from nvArray table
 *
 * sr_get()    - run status report {sr:n}
 * sr_set()    - set status report elements (sr:{...}}
 * srs_get()   - query status report list {srs:n}
 * srs_set()   - set status report {srs:{...:{...}}}
 * sr_set_si() - set status report interval
 */

stat_t sr_get(nvObj_t *nv)
{
    return (_populate_unfiltered_status_report("sr"));
}

stat_t sr_set(nvObj_t *nv)  // legacy semantics
{
    return (_set_sr(nv));
}

stat_t srs_get(nvObj_t *nv) // new semantics
{
    return (_populate_unfiltered_status_report("srs"));
}

stat_t srs_set(nvObj_t *nv) // new semantics
{
    return (_set_srs(nv));
}

stat_t sr_set_si(nvObj_t *nv)
{
	if (nv->value_int < STATUS_REPORT_MIN_MS) {
    	nv->value_int = STATUS_REPORT_MIN_MS;
	}
	sr.status_report_interval = nv->value_int;

	return(STAT_OK);
}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_si[] PROGMEM = "[si]  status interval%14lu ms\n";
static const char fmt_sv[] PROGMEM = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";

void sr_print_sr(nvObj_t *nv) { _populate_unfiltered_status_report("sr");}
void sr_print_si(nvObj_t *nv) { text_print(nv, fmt_si);}
void sr_print_sv(nvObj_t *nv) { text_print(nv, fmt_sv);}

#endif // __TEXT_MODE


/***********************************************************************************/
/**** Queue Reports ****************************************************************/
/***********************************************************************************/
/*
 *	Queue reports can report three values:
 *	  - qr	queue depth - # of buffers available in planner queue
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
	// get buffer depth and added/removed count
	qr.buffers_available = mp_get_planner_buffers_available();
	if (buffers > 0) {
		qr.buffers_added += buffers;
	} else {
		qr.buffers_removed -= buffers;
	}

	// time-throttle requests while generating arcs
	qr.motion_mode = cm_get_motion_mode(ACTIVE_MODEL);
	if ((qr.motion_mode == MOTION_MODE_CW_ARC) || (qr.motion_mode == MOTION_MODE_CCW_ARC)) {
		uint32_t tick = SysTickTimer_getValue();
		if (tick - qr.init_tick < MIN_ARC_QR_INTERVAL) {
			qr.queue_report_requested = false;
			return;
		}
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
	if (qr.queue_report_verbosity == QR_OFF) {
        return (STAT_NOOP);
    }
	if (qr.queue_report_requested == false) {
        return (STAT_NOOP);
    }
	qr.queue_report_requested = false;

	if (cs.comm_mode == TEXT_MODE) {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			printf_P(PSTR("qr:%d\n"), qr.buffers_available);
		} else  {
			printf_P(PSTR("qr:%d, qi:%d, qo:%d\n"), qr.buffers_available,qr.buffers_added,qr.buffers_removed);
		}
	} else {
        char line_msg[32];
		if (qr.queue_report_verbosity == QR_SINGLE) {
			sprintf_P(line_msg, PSTR("{\"qr\":%d}\n"), qr.buffers_available);
		} else {
			sprintf_P(line_msg,PSTR("{\"qr\":%d,\"qi\":%d,\"qo\":%d}\n"), qr.buffers_available, qr.buffers_added,qr.buffers_removed);
		}
        printf(json_relax(line_msg));
	}
	qr_init_queue_report();
	return (STAT_OK);
}

/*
 * Wrappers and Setters - for calling from cfgArray table
 *
 * qr_get() - run a queue report (as data)
 * qi_get() - run a queue report - buffers in
 * qo_get() - run a queue report - buffers out
 */
stat_t qr_get(nvObj_t *nv)
{
	nv->value_int = mp_get_planner_buffers_available(); // ensure that manually requested QR count is always up to date
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t qi_get(nvObj_t *nv)
{
	nv->value_int = qr.buffers_added;
	nv->valuetype = TYPE_INTEGER;
	qr.buffers_added = 0;				// reset it
	return (STAT_OK);
}

stat_t qo_get(nvObj_t *nv)
{
	nv->value_int = qr.buffers_removed;
	nv->valuetype = TYPE_INTEGER;
	qr.buffers_removed = 0;				// reset it
	return (STAT_OK);
}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_qr[] PROGMEM = "qr:%d\n";
static const char fmt_qi[] PROGMEM = "qi:%d\n";
static const char fmt_qo[] PROGMEM = "qo:%d\n";
static const char fmt_qv[] PROGMEM = "[qv]  queue report verbosity%7d [0=off,1=single,2=triple]\n";

void qr_print_qr(nvObj_t *nv) { text_print(nv, fmt_qr);}
void qr_print_qi(nvObj_t *nv) { text_print(nv, fmt_qi);}
void qr_print_qo(nvObj_t *nv) { text_print(nv, fmt_qo);}
void qr_print_qv(nvObj_t *nv) { text_print(nv, fmt_qv);}

#endif // __TEXT_MODE

/***********************************************************************************/
/**** RX Reports *******************************************************************/
/***********************************************************************************/
/*
 * rx_request_rx_report() - request an update on usb serial buffer space available
 * rx_report_callback() - send rx report if one has been requested
 */
void rx_request_rx_report(void) {
    rx.rx_report_requested = true;
    rx.space_available = xio_get_usb_rx_free();
}

stat_t rx_report_callback(void) {
    if (!rx.rx_report_requested) {
        return (STAT_NOOP);
    }
    rx.rx_report_requested = false;

    printf_P(PSTR("{\"rx\":%d}\n"), rx.space_available);
    return (STAT_OK);
}


/***********************************************************************************/
/**** Job ID Reports ***************************************************************/
/***********************************************************************************/
/*
 *	job_populate_job_report()
 *	job_set_job_report()
 *	job_report_callback()
 *	job_get()
 *	job_set()
 *	job_print_job()
 */
stat_t job_populate_job_report()
{
	const char job_str[] = "job";
	char tmp[TOKEN_LEN+1];
	nvObj_t *nv = nv_reset_nv_list("job");	// sets *nv to the start of the body

	nv->valuetype = TYPE_PARENT; 			// setup the parent object
	strcpy(nv->token, job_str);

	//nv->index = nv_get_index((const char *)"", job_str);// set the index - may be needed by calling function
	nv = nv_next(nv);						// no need to check for NULL as list has just been reset

	index_t job_start = nv_get_index("", "job1"); // set first job persistence index
	for (uint8_t i=0; i<4; i++) {

		nv_populate_nv_by_index(nv, job_start + i);

		strcpy(tmp, nv->group);				// concatenate groups and tokens - do NOT use strncpy()
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);

		if ((nv = nv_next(nv)) == NULL) {
            return (STAT_OK);               // should never be NULL unless SR length exceeds available buffer array
        }
	}
	return (STAT_OK);
}

stat_t job_set_job_report(nvObj_t *nv)
{
	index_t job_start = nv_get_index((const char *)"",(const char *)"job1");// set first job persistence index

	for (uint8_t i=0; i<4; i++) {
		if (((nv = nv_next(nv)) == NULL) || (nv->valuetype == TYPE_EMPTY)) { break;}
		if (nv->valuetype == TYPE_INTEGER) {
			cs.job_id[i] = nv->value_int;
			nv->index = job_start + i;		// index of the job persistence location
			nv_persist(nv);
		} else {
			return (STAT_UNSUPPORTED_TYPE);
		}
	}
	job_populate_job_report();				// return current values
	return (STAT_OK);
}

uint8_t job_report_callback()
{
	if (cs.comm_mode == JSON_MODE) {		// only JSON mode; job_ids are client app state
        char local_msg[64];
		sprintf_P(local_msg, PSTR("{\"job\":[%lu,%lu,%lu,%lu]}\n"), cs.job_id[0], cs.job_id[1], cs.job_id[2], cs.job_id[3] );
        printf(json_relax(local_msg));
    }
	return (STAT_OK);
}

stat_t job_get(nvObj_t *nv) { return (job_populate_job_report());}
stat_t job_set(nvObj_t *nv) { return (job_set_job_report(nv));}
void job_print_job(nvObj_t *nv) { job_populate_job_report();}

