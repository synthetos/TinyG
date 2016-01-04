/*
 * report.c - TinyG status report and other reporting functions.
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

#define SR_WORKING_LIST_LEN (2*(NV_STATUS_REPORT_LEN+1)) // supports full replacements

/**** Exception Reports ************************************************************
 * rpt_exception() - generate an exception message - always in JSON format
 *
 * Returns incoming status value
 *
 * WARNING: Do not call this function from MED or HI interrupts (LO is OK)
 *			or there is a potential for deadlock in the TX buffer.
 */

stat_t rpt_exception(stat_t status, const char *msg)
{
    if (status != STAT_OK) { // makes it possible to call exception reports w/o checking status value

        if (js.json_syntax == JSON_SYNTAX_RELAXED) {
            printf_P(PSTR("{er:{fb:%0.2f,st:%d,msg:\"%s - %s\"}}\n"),
                TINYG_FIRMWARE_BUILD, status, get_status_message(status), msg);

        } else {
            printf_P(PSTR("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s - %s\"}}\n"),
                TINYG_FIRMWARE_BUILD, status, get_status_message(status), msg);
        }
    }
    return (status);			// makes it possible to inline, e.g: return(rpt_exception(status, msg));
}

/*
 * rpt_er()	- send a bogus exception report for testing purposes (it's not real)
 */
stat_t rpt_er(nvObj_t *nv)
{
    char msg[sizeof("bogus exception report")];
    sprintf_P(msg, PSTR("bogus exception report"));
	return(rpt_exception(STAT_GENERIC_EXCEPTION_REPORT, msg)); // bogus exception report for testing
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

/*
 * sr_init_status_report_P() - initialize status report from CSV string (in PROGMEM)
 *
 *  SR settings are not initialized by reading NVram during the system load process.
 *  Instead they are loaded by running this function:
 *    - If sr_list_P is present load the settings from sr_list and persist to NVram
 *    - If sr_list_P is not present load settings in NVram
 *
 *  sr_list_P is a comma-separate-value list in program memory. Spaces are not allowed
 */

void sr_init_status_report_P(const char *sr_list_P)
{
    char sr_list[ NV_STATUS_REPORT_LEN * (TOKEN_LEN+1) ]; strcpy_P(sr_list, sr_list_P);
    char *wr = sr_list;     // pointer to write NULL terminations over commas
    char *rd = sr_list;     // pointer to pass token
    uint8_t i=0;

    nvObj_t *nv = nv_reset_nv_list(NUL);	// used for status report persistence locations
    sr.stat_index = nv_get_index("", "stat");               // set index of stat element
    nv->index = nv_get_index("", "se00");                   // set first SR persistence index
    nv->valuetype = TYPE_INTEGER;

    // No SR list, load SR from NVram
    if (*sr_list == NUL) {
        for (; i<NV_STATUS_REPORT_LEN; i++) {
            read_persistent_value(nv);                      // read token index from NVram into nv->value_int element
            sr.status_report_list[i] = nv->value_int;       // load into the active SR list
            sr.status_report_value[i] = 8675309;            // pre-load SR values with an unlikely number
            nv->index++;                                    // increment SR NVM index
        }
    // load the sr_list and persist it NVram
    } else {
        for (; i<NV_STATUS_REPORT_LEN; i++) {
            while (true) {                                  // find the token to display
                if ((*wr == ',') || (*wr == NUL)) {
                    *wr = NUL;
                    break;
                }
                if (++wr > (sr_list + (NV_STATUS_REPORT_LEN * (TOKEN_LEN+1)))) { // if string was not terminated properly
                    return;
                }
            }
            if ((nv->value_int = nv_get_index("", rd)) == NO_MATCH) {
                strcpy_P(sr_list, sr_list_P);                           // reset the SR list RAM string
                rpt_exception(STAT_BAD_STATUS_REPORT_SETTING, sr_list); // trap mis-configured profile settings
                return;
            }
            nv_set(nv);
            nv_persist(nv);                                 // conditionally persist - automatic by nv_persist()
            sr.status_report_value[i] = 8675309;			// pre-load SR values with an unlikely number
            nv->index++;                                    // increment SR NVM index
            rd = (++wr);                                    // set up to read next SR token
        }
    }
    sr.status_report_requested = false;
}

/*
 * sr_init_status_report()
 *
 *  SR settings are not initialized by reading NVram during the system load process. Instead they are
 *  loaded by running this function which reads the SR settings from NVram. If 'use_defaults' is true
 *  then SR settings will be reset from the profile and persisted back to NVram.
 */

void sr_init_status_report(bool use_defaults)
{
    nvObj_t *nv = nv_reset_nv_list(NUL);	// used for status report persistence locations
    char sr_defaults[NV_STATUS_REPORT_LEN][TOKEN_LEN+1] = { STATUS_REPORT_DEFAULTS };	// see settings.h
    sr.stat_index = nv_get_index("", "stat");               // set index of stat element
    nv->index = nv_get_index("", "se00");                   // set first SR persistence index
    nv->valuetype = TYPE_INTEGER;

    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if (!use_defaults) {
            read_persistent_value(nv);                      // read NVram into nv->value_int element
            sr.status_report_list[i] = nv->value_int;       // pre-load the stored SR list
        } else {
            if (sr_defaults[i][0] == NUL) {                 // load the index for the SR element
                nv->value_int = NO_MATCH;                   // label as a blank spot
            } else {                                        // set and persist the default value
                if ((nv->value_int = nv_get_index((const char *)"", sr_defaults[i])) == NO_MATCH) {
                    char msg[sizeof("mis-configured status report settings")];
                    sprintf_P(msg, PSTR("mis-configured status report settings"));
                    rpt_exception(STAT_BAD_STATUS_REPORT_SETTING, msg); // trap mis-configured profile settings
                    return;
                }
            }
            nv_set(nv);
            nv_persist(nv);                                 // conditionally persist - automatic by nv_persist()
        }
        sr.status_report_value[i] = 8675309;			    // pre-load SR values with an unlikely number
        nv->index++;                                        // increment SR NVM index
    }
    sr.status_report_requested = false;
}

/*
 * sr_set_status_report() - interpret an SR setup string and return current report
 *
 * Behaviors:
 *    {sr:f} removes all status reports (clears)
 *    {sr:t} restores status reports to default
 *    {sr:{<key1>:t,...<keyN>:t}} adds <key1> through <keyN> to the status report list
 *    {sr:{<key1>:f,...<keyN>:t}} removes <key1> through <keyN> from the status report list
 *
 *    - Lines may have a mix of t and f pairs
 *    - On entry nv points to the parent "sr" element on entry
 *    - List ordering is not guaranteed in the case of mixed removes and adds in the same command
 *
 *  Error conditions:
 *    - All failures leave original SR list untouched
 *    - An attempt to add an element that exceeds list max fails with STAT_INPUT_EXCEEDS_MAX_LENGTH
 *    - A token that is not recognized fails with STAT_UNRECOGNIZED_NAME
 *    - A value other than 't', or 'f' fails with STAT_INPUT_VALUE_RANGE_ERROR
 *    - Malformed JSON fails as usual before this point
 */

static void _persist_status_report_list(nvObj_t *nv)
{
    nv->index = nv_get_index("","se00");            // set first SR persistence index
    nv->valuetype = TYPE_INTEGER;
    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        nv->value_int = sr.status_report_list[i];
        nv_persist(nv);
        nv->index++;                                // index of the next SR persistence location
    }
    nv->valuetype = TYPE_BOOL;
}

stat_t sr_set_status_report(nvObj_t *nv)
{
    int8_t i;
    int8_t j;
    nvObj_t *nv_first_sr_child = nv->nx;            // save for later

	index_t working_list[SR_WORKING_LIST_LEN];      // init working list from the current SR list
	for (i=0; i<SR_WORKING_LIST_LEN; i++) {         // first fill with -1's
    	working_list[i] = NO_MATCH;
	}

    // process {sr:f}    clear all SR settings
    if ((nv->valuetype == TYPE_BOOL) && (nv->value_int == false)) {
	    for (i=0; i<NV_STATUS_REPORT_LEN; i++) {
            sr.status_report_list[i] = NO_MATCH;
        }
        _persist_status_report_list(nv);
        return (STAT_OK);
    }

    // process {sr:t}    restore SR settings to defaults
    if ((nv->valuetype == TYPE_BOOL) && (nv->value_int == true)) {
        sr_init_status_report_P(SR_DEFAULTS);
        return (STAT_OK);
    }

    // process {sr:{.... process one or more SR drop/adds
	for (i=0; i<NV_STATUS_REPORT_LEN; i++) {        // read in the current SR list
        working_list[i] = sr.status_report_list[i];
    }

    // iterate the items in the nvlist
    index_t item;                                   // status report item being worked on
	for (i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if ((nv = nv->nx) == NULL) {                // advance to next element (past the "sr" parent)
            return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
        }
		if (nv->valuetype == TYPE_EMPTY) {          // end of items
            break;
        }
		if (nv->valuetype != TYPE_BOOL) {           // unsupported type in request
    		return (STAT_INPUT_VALUE_RANGE_ERROR);
		}
	    if ((item = nv_get_index(nv->group,nv->token)) == NO_MATCH) {
    	    return(STAT_UNRECOGNIZED_NAME);         // trap non-existent tags
	    }
        if (nv->value_int == false) {               // remove an item from the working list
            for (j=0; j<SR_WORKING_LIST_LEN; j++) {
                if (working_list[j] == item) {      // item exists in working list
                    working_list[j] = NO_MATCH;
                    break;
                }
            }
        } else {                                    // add an item to the working list
	        for (j=0; j<SR_WORKING_LIST_LEN; j++) {
    	        if (working_list[j] == item) {      // item already exists in working list
                    break;
    	        }
    	        if (working_list[j] == NO_MATCH) {  // add the item to working list
        	        working_list[j] = item;
                    break;
    	        }
            }
        }
	}
    // pack the list and copy it to the SR list
    // i is the read pointer, j is the write pointer
    for (i=0, j=0; i<SR_WORKING_LIST_LEN; i++, j++) {
        if (working_list[i] == NO_MATCH) { i++; }
        working_list[j] = working_list[i];
    }
    working_list[j] = NO_MATCH;                     // terminate the list

    for (i=0; i<SR_WORKING_LIST_LEN; i++) {         // test for overflow
        if (working_list[i] == NO_MATCH) {
            break;
        }
    }
    if (i > NV_STATUS_REPORT_LEN) {
        return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
    }
	memcpy(sr.status_report_list, working_list, sizeof(sr.status_report_list));
    _persist_status_report_list(nv_first_sr_child);
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
	if (sr.status_report_verbosity == SR_OFF) {
        return (STAT_NOOP);
    }
	if (sr.status_report_requested == false) {
        return (STAT_NOOP);
    }
	if (SysTickTimer_getValue() < sr.status_report_systick) {
        return (STAT_NOOP);
    }
	sr.status_report_requested = false;		// disable reports until requested again

	if (sr.status_report_verbosity == SR_VERBOSE) {
		_populate_unfiltered_status_report();
	} else {
		if (_populate_filtered_status_report() == false) {	// no new data
			return (STAT_OK);
		}
	}
	nv_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	return (STAT_OK);
}

/*
 * sr_run_text_status_report() - generate a text mode status report in multiline format
 */
stat_t sr_run_text_status_report()
{
	_populate_unfiltered_status_report();
	nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
	return (STAT_OK);
}

/*
 * _populate_unfiltered_status_report() - populate nvObj body with status values
 *
 *	Set r_header true if you want the report to be a response.
 *  Set r_header false if it's to be used as an autogenerated status report
 */
static stat_t _populate_unfiltered_status_report()
{
	char tmp[TOKEN_LEN+1];
	nvObj_t *nv = nv_reset_nv_list("sr");
 	nv = nv->nx;	                    // set *nv to the first empty pair past the SR parent

	for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
		if ((nv->index = sr.status_report_list[i]) == NO_MATCH) { break;}
		nv_populate_nvObj_by_index(nv);

		strcpy(tmp, nv->group);			// flatten out groups - WARNING - you cannot use strncpy here...
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);			//...or here.

		if ((nv = nv->nx) == NULL) {
			return (cm_hard_alarm_P(STAT_BUFFER_FULL_FATAL, PSTR("_populate_unfiltered_status_report")));	// should never be NULL unless SR length exceeds available buffer array
        }
	}
	return (STAT_OK);
}

/*
 * _populate_filtered_status_report() - populate nvObj body with status values
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
	char tmp[TOKEN_LEN+1];
	uint8_t has_data = false;
	nvObj_t *nv = nv_reset_nv_list("sr");   // initialize nv_list as a status report
	nv = nv->nx;	                        // set *nv to the first empty pair past the SR parent

	for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
		if ((nv->index = sr.status_report_list[i]) == NO_MATCH) { // normal list termination or error
            break;
        }
		nv_populate_nvObj_by_index(nv);     // get the current value and other NV parameters

		// Special handling for stat values - always report the end conditions
        if (nv->index == sr.stat_index) {
            if ((nv->value_int == COMBINED_PROGRAM_STOP) || (nv->value_int == COMBINED_PROGRAM_END)) {
    			sr.status_report_value[i] = nv->value_int;
                nv = nv->nx;
    			has_data = true;
                continue;
            }
        }

		// Only report values that have changed
        if (nv->valuetype == TYPE_INTEGER) {
            if (nv->value_int == sr.status_report_value[i]) {
                nv->valuetype = TYPE_EMPTY;
                continue;
            }
        } else { // (nv->valuetype == TYPE_FLOAT)
            if (fabs(nv->value_flt - sr.status_report_value[i]) < 0.0001) {
                nv->valuetype = TYPE_EMPTY;
                continue;
            }
        }

        // flatten out groups - WARNING - you cannot use strncpy here...
		strcpy(tmp, nv->group);
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);		        //...or here.

		sr.status_report_value[i] = nv->value_int;  // works for either int or float
		if ((nv = nv->nx) == NULL) {        // should never be NULL unless SR length exceeds available buffer array
    		return (false);
		}
		has_data = true;
	}
	return (has_data);
}

/*
 * Wrappers and Setters - for calling from nvArray table
 *
 * sr_get()		- run status report
 * sr_set()		- set status report elements
 * sr_set_si()	- set status report interval
 */
stat_t sr_get(nvObj_t *nv)
{
    return (_populate_unfiltered_status_report());
}

stat_t sr_set(nvObj_t *nv)
{
    return (sr_set_status_report(nv));
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

void sr_print_sr(nvObj_t *nv) { _populate_unfiltered_status_report();}
void sr_print_si(nvObj_t *nv) { text_print(nv, fmt_si);}
void sr_print_sv(nvObj_t *nv) { text_print(nv, fmt_sv);}

#endif // __TEXT_MODE


/*****************************************************************************
 * Queue Reports
 *
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
	if (qr.queue_report_verbosity == QR_OFF)
        return (STAT_NOOP);

	if (qr.queue_report_requested == false)
        return (STAT_NOOP);

	qr.queue_report_requested = false;

	if (cs.comm_mode == TEXT_MODE) {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			printf_P(PSTR("qr:%d\n"), qr.buffers_available);
		} else  {
			printf_P(PSTR("qr:%d, qi:%d, qo:%d\n"), qr.buffers_available,qr.buffers_added,qr.buffers_removed);
		}

	} else if (js.json_syntax == JSON_SYNTAX_RELAXED) {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			printf_P(PSTR("{qr:%d}\n"), qr.buffers_available);
		} else {
			printf_P(PSTR("{qr:%d,qi:%d,qo:%d}\n"), qr.buffers_available, qr.buffers_added,qr.buffers_removed);
		}

	} else {
		if (qr.queue_report_verbosity == QR_SINGLE) {
			printf_P(PSTR("{\"qr\":%d}\n"), qr.buffers_available);
		} else {
			printf_P(PSTR("{\"qr\":%d,\"qi\":%d,\"qo\":%d}\n"), qr.buffers_available, qr.buffers_added,qr.buffers_removed);
		}
	}
	qr_init_queue_report();
	return (STAT_OK);
}

/*
 * rx_request_rx_report() - request an update on usb serial buffer space available
 */
void rx_request_rx_report(void) {
    rx.rx_report_requested = true;
    rx.space_available = xio_get_usb_rx_free();
}

/*
 * rx_report_callback() - send rx report if one has been requested
 */
stat_t rx_report_callback(void) {
    if (!rx.rx_report_requested) {
        return (STAT_NOOP);
    }
    rx.rx_report_requested = false;

    printf_P(PSTR("{\"rx\":%d}\n"), rx.space_available);
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
	const char job_str[] = "job";
	char tmp[TOKEN_LEN+1];
	nvObj_t *nv = nv_reset_nv_list("job");	// sets *nv to the start of the body

	nv->valuetype = TYPE_PARENT; 			// setup the parent object
	strcpy(nv->token, job_str);

	//nv->index = nv_get_index((const char *)"", job_str);// set the index - may be needed by calling function
	nv = nv->nx;							// no need to check for NULL as list has just been reset

	index_t job_start = nv_get_index("", "job1"); // set first job persistence index
	for (uint8_t i=0; i<4; i++) {

		nv->index = job_start + i;
		nv_populate_nvObj_by_index(nv);

		strcpy(tmp, nv->group);				// concatenate groups and tokens - do NOT use strncpy()
		strcat(tmp, nv->token);
		strcpy(nv->token, tmp);

		if ((nv = nv->nx) == NULL)
            return (STAT_OK);               // should never be NULL unless SR length exceeds available buffer array
	}
	return (STAT_OK);
}

stat_t job_set_job_report(nvObj_t *nv)
{
	index_t job_start = nv_get_index((const char *)"",(const char *)"job1");// set first job persistence index

	for (uint8_t i=0; i<4; i++) {
		if (((nv = nv->nx) == NULL) || (nv->valuetype == TYPE_EMPTY)) { break;}
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
	if (cs.comm_mode == TEXT_MODE) {
		// no-op, job_ids are client app state
	} else if (js.json_syntax == JSON_SYNTAX_RELAXED) {
		printf_P(PSTR("{job:[%lu,%lu,%lu,%lu]}\n"), cs.job_id[0], cs.job_id[1], cs.job_id[2], cs.job_id[3] );
	} else {
		printf_P(PSTR("{\"job\":[%lu,%lu,%lu,%lu]}\n"), cs.job_id[0], cs.job_id[1], cs.job_id[2], cs.job_id[3] );
	}
	return (STAT_OK);
}

stat_t job_get(nvObj_t *nv) { return (job_populate_job_report());}
stat_t job_set(nvObj_t *nv) { return (job_set_job_report(nv));}
void job_print_job(nvObj_t *nv) { job_populate_job_report();}

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
