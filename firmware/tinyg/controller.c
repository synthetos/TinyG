/*
 * controller.c - tinyg controller and top level parser
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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

#include "tinyg.h"				// #1
#include "config.h"				// #2
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"

#include "encoder.h"
#include "hardware.h"
#include "switch.h"
#include "gpio.h"
#include "report.h"
#include "testhooks.h"
#include "help.h"
#include "util.h"
#include "xio.h"
#include "runtime.h"

#ifdef __ARM
#include "Reset.h"
#endif

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS *********************************************************
 ***********************************************************************************/

controller_t cs;		// controller state structure

enum { SYNC_OFF, SYNC_REQUESTED, SYNC_WAITING };
enum { SYNC_SILENT, SYNC_TEXT, SYNC_JSON };
static struct {
    uint8_t state;
    uint8_t reply;
    bool char_mode;
    uint16_t linelen;
    uint32_t tid;
    bool tid_present;
} motion_sync;

/***********************************************************************************
 **** STATICS AND LOCALS ***********************************************************
 ***********************************************************************************/

static void _controller_HSM(void);
static stat_t _shutdown_handler(void);
static stat_t _normal_idler(void);
static stat_t _enquiry_handler(void);
static stat_t _limit_switch_handler(void);

static void _init_assertions(void);
static stat_t _test_system_assertions(void);

static stat_t _sync_to_planner(void);
static stat_t _sync_to_motion(void);
static stat_t _sync_to_tx_buffer(void);

static stat_t _controller_state(void);
static stat_t _dispatch_command(void);
static stat_t _dispatch_control(void);
static void _dispatch_kernel(void);

// prep for export to other modules:
stat_t hardware_hard_reset_handler(void);
stat_t hardware_bootloader_handler(void);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err)
{
	memset(&cs, 0, sizeof(controller_t));			// clear all values, job_id's, pointers and status
	_init_assertions();
	memset(&motion_sync, 0, sizeof(motion_sync));

	cs.fw_build = TINYG_FIRMWARE_BUILD;
	cs.fw_version = TINYG_FIRMWARE_VERSION;
	cs.hw_platform = TINYG_HARDWARE_PLATFORM;		// NB: HW version is set from EEPROM
	cs.controller_state = CONTROLLER_STARTUP;		// ready to run startup lines

#ifdef __AVR
	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	xio.default_src = std_in;
	controller_set_primary_source(xio.default_src);
#endif

#ifdef __ARM
	cs.controller_state = CONTROLLER_NOT_CONNECTED;	// find USB next
	IndicatorLed.setFrequency(100000);
#endif
}

/*
 * controller_run() - MAIN LOOP - top-level controller
 *
 * The order of the dispatched tasks is very important.
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon.
 *
 * Tasks must be written as continuations as they will be called repeatedly,
 * and are called even if they are not currently active.
 *
 * The DISPATCH macro calls the function and returns to the controller parent
 * if not finished (STAT_EAGAIN), preventing later routines from running
 * (they remain blocked). Any other condition - OK or ERR - drops through
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return STAT_NOOP
 */

void controller_run()
{
	while (true) {
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return;
static void _controller_HSM()
{
//----- Interrupt Service Routines are the highest priority controller functions ----//
//      See hardware.h for a list of ISRs and their priorities.
//
//----- kernel level ISR handlers ----(flags are set in ISRs)------------------------//
												// Order is important:
	DISPATCH(hw_hard_reset_handler());			// handle hard reset requests
	DISPATCH(hw_bootloader_handler());			// handle requests to enter bootloader

#ifdef TINYG_TEST_HOOKS
	DISPATCH(testhooks_callback());				// HIL test hooks (test builds only)
#endif
	DISPATCH(_enquiry_handler());				// answer ENQ from the main loop
	DISPATCH(_normal_idler());					// blink LEDs slowly to show everything is OK
    DISPATCH(_shutdown_handler());              // invoke shutdown (++++ INCOMPLETE)
// 	DISPATCH(_interlock_handler());             // invoke / remove safety interlock
 	DISPATCH(_limit_switch_handler());          // invoke limit switch
    DISPATCH(_controller_state());              // controller state management
    config_report_load_failure();              // non-fatal, after startup enables TX
	DISPATCH(_test_system_assertions());		// system integrity assertions
    DISPATCH(mp_startup_callback());           // release idle startup before planner backpressure
	DISPATCH(_dispatch_control());				// read any control messages prior to executing cycles

//----- planner hierarchy for gcode and cycles ---------------------------------------//

	DISPATCH(st_motor_power_callback());		// stepper motor power sequencing
	DISPATCH(sr_status_report_callback());		// conditionally send status report
	DISPATCH(qr_queue_report_callback());		// conditionally send queue report
	DISPATCH(rx_report_callback());             // conditionally send rx report
	DISPATCH(rt_heartbeat_callback());			// idle heartbeat, when enabled ({"rt":{"hb":n}})

	DISPATCH(cm_feedhold_sequencing_callback());// feedhold state machine runner
	DISPATCH(cm_arc_callback());				// arc generation runs behind lines
	DISPATCH(cm_homing_callback());				// G28.2 continuation
	DISPATCH(cm_probe_callback());				// G38.2 continuation
	DISPATCH(cm_jogging_callback());			// jog function
	DISPATCH(cm_deferred_write_callback());		// persist G10 changes when not in machining cycle

//----- command readers and parsers --------------------------------------------------//

	DISPATCH(_sync_to_motion());				// complete M400 before accepting more data
	DISPATCH(_sync_to_planner());				// ensure there is at least one free buffer in planning queue
	DISPATCH(_sync_to_tx_buffer());				// sync with TX buffer (pseudo-blocking)
	DISPATCH(set_baud_callback());				// (AVR only) perform baud rate update (must be after TX sync)
	DISPATCH(_dispatch_command());				// read and execute next command
}

/*****************************************************************************************
 * _controller_state() - manage conrtroller connection, startup, and other state changes
 */
static stat_t _controller_state()
{
#ifdef __AVR
	if (cs.controller_state <= CONTROLLER_STARTUP) {		// first time through after reset
		cs.controller_state = CONTROLLER_READY;
		cm_request_queue_flush();
		rpt_print_system_ready_message();
	}
	return (STAT_OK);
#endif // __AVR

#ifdef __ARM
	// detect USB connection and transition to disconnected state if it disconnected
	//	if (SerialUSB.isConnected() == false) cs.state = CONTROLLER_NOT_CONNECTED;
	return (xio_callback());					// manages state changes in the XIO system
#endif // __ARM
}

/*****************************************************************************
 * command dispatchers
 * _dispatch_command - entry point for control and data dispatches
 * _dispatch_control - entry point for control-0nly dispatches
 * _dispatch_kernel - core dispatch routines
 * controller_dispatch_text_container() - specialized dispatcher for 'txt' elements
 *
 *	Reads next command line and dispatches to relevant parser or action
 */

#ifdef __AVR
/*
 * _reject_overlong_line() - answer a truncated line-mode line with one error response (E087)
 *
 *	The line is not executed: running the front of a cut line (or its remainder) is unsafe.
 *	No alarm - the input was bad, the machine is fine.
 */
static void _reject_overlong_line()
{
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);
	if (cs.comm_mode == TEXT_MODE) {
		text_response(STAT_INPUT_EXCEEDS_MAX_LENGTH, cs.saved_buf);
	} else {
		nv_reset_nv_list("");
		nv_print_list(STAT_INPUT_EXCEEDS_MAX_LENGTH, TEXT_RESPONSE, JSON_RESPONSE);
	}
}

/*
 * _reject_damaged_line() - a line lost bytes in an RX ring overrun: it never runs (E162)
 *
 *	The first one raises ALARM 13. Each one also gets its own error response, so none vanishes
 *	silently while the machine is alarmed - including a host line that follows lost bytes.
 */
static void _reject_damaged_line()
{
	cm_alarm(STAT_BUFFER_FULL, "input lost");		// no-op if already alarmed
	cs.saved_buf[0] = NUL;
	if (cs.comm_mode == TEXT_MODE) {
		text_response(STAT_BUFFER_FULL, cs.saved_buf);
	} else {
		nv_reset_nv_list("");
		nv_print_list(STAT_BUFFER_FULL, TEXT_RESPONSE, JSON_RESPONSE);
	}
}

/*
 * _count_line() - add a line read to the bytes the next character-mode footer reports (E164)
 *
 *	A line that gets no reply (a blank line, the second byte of a CRLF) still took ring space: its
 *	bytes ride on the next reply, so a host counting bytes never drifts. Line mode and text mode
 *	report no byte count, so there it is just the line read.
 */
static void _count_line(uint16_t size)
{
	if ((xio.rx_mode == RX_MODE_CHAR) && (cs.comm_mode != TEXT_MODE)) {
		cs.linelen += size;
	} else {
		cs.linelen = size;
	}
}

static stat_t _dispatch_command()
{
	if (cm.queue_flush_input || cm.queue_flush_input_due) {
		return (STAT_OK);						// a host '%' is pending: buffered lines came before it (E146)
	}
	devflags_t flags = DEV_IS_BOTH;
	uint16_t size = 0;
	cs.bufp = readline(&flags, &size);
	_count_line(size);
	if (cm.queue_flush_input || cm.queue_flush_input_due) {
		return (STAT_OK);						// a '%' accepted while this line was read: it came before it (E191)
	}
    if (cs.bufp == (char *)_FDEV_ERR) {     // the line lost input (E162)
        _reject_damaged_line();
        return (STAT_OK);
    }
    if ((cs.bufp != (char *)NULL) && (flags & DEV_IS_OVERFLOW)) {
        _reject_overlong_line();
    } else if (cs.bufp != (char *)NULL) {   // process the command
        _dispatch_kernel();
    }
	return (STAT_OK);
}

static stat_t _dispatch_control()
{
	if (cm.queue_flush_input || cm.queue_flush_input_due) {
		return (STAT_OK);						// a host '%' is pending: buffered lines came before it (E146)
	}
	devflags_t flags = DEV_IS_CTRL;
	uint16_t size = 0;
	cs.bufp = readline(&flags, &size);
	_count_line(size);
	if (cm.queue_flush_input || cm.queue_flush_input_due) {
		return (STAT_OK);						// a '%' accepted while this line was read: it came before it (E191)
	}
	if (cs.bufp == (char *)_FDEV_ERR) {     // the line lost input (E162)
		_reject_damaged_line();
		return (STAT_OK);
	}
	if ((cs.bufp != (char *)NULL) && (flags & DEV_IS_OVERFLOW)) {
		_reject_overlong_line();
	} else if (cs.bufp != (char *)NULL) {   // process the command
    	_dispatch_kernel();
	}
	return (STAT_OK);
}
#endif // __AVR

#ifdef __ARM
static stat_t _dispatch_command()
{
    devflags_t flags = DEV_IS_BOTH;
	cs.bufp = readline(flags, cs.linelen);
    if (cs.bufp != (char *)NULL) {          // process the command
        _dispatch_kernel();
    }
    return (STAT_OK);
}
static stat_t _dispatch_control()
{
    devflags_t flags = DEV_IS_CTRL;
    if ((cs.bufp = readline(flags, cs.linelen)) != NULL) {
        _dispatch_kernel();
    }
    return (STAT_OK);
}
#endif // __ARM

static void _dispatch_kernel()
{
    // skip leading whitespace & quotes
	while ((*cs.bufp == SPC) || (*cs.bufp == TAB) || (*cs.bufp == '"')) {
        cs.bufp++;
    }
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);		// save input buffer for reporting
    js.motion_sync_allowed = true;
	js.tid_present = false;									// only json_parser() sets it, for this line (E049)

	// A stray leading '?' (a status request sent just before the line) must not swallow the line:
	// strip it before dispatching, so the rest is parsed and answered (E053)
	if ((*cs.bufp == '?') && (cs.bufp[1] != NUL)) {
		cs.bufp++;
	}

	if (*cs.bufp == NUL) {									// blank line - just a CR or the 2nd termination in a CRLF
		if (cs.comm_mode == TEXT_MODE) {
			nv_reset_nv_list("");							// no message left over from the line before (E167)
			text_response(STAT_OK, cs.saved_buf);
		}
    }
	// included for AVR diagnostics and ARM serial (which does not trap these characters immediately on RX)
	else if (*cs.bufp == '!') { cm_request_feedhold(); }
	else if (*cs.bufp == '~') { cm_request_end_hold(); }
	else if (*cs.bufp == '%') { cm_request_queue_flush(); }
	else if (*cs.bufp == ENQ) { controller_request_enquiry(); }


	else if (*cs.bufp == '{') {							    // process as JSON mode
		cs.comm_mode = JSON_MODE;							// switch to JSON mode
//		json_parser(cs.bufp, NV_BODY, true);
		json_parser(cs.bufp);
    }
#ifdef __TEXT_MODE
	else if (strchr("$?Hh", *cs.bufp) != NULL) {			// process as text mode
		cs.comm_mode = TEXT_MODE;							// switch to text mode
		text_response(text_parser(cs.bufp), cs.saved_buf);
    }
	else if (cs.comm_mode == TEXT_MODE) {					// anything else must be Gcode
		// A fresh list, laid out as for JSON Gcode (the block first), so a (MSG ...) lands at
		// NV_BODY+1 where text_response() prints it, whatever the previous line left behind (E167)
		nv_reset_nv_list("gc");
		text_response(gc_gcode_parser(cs.bufp), cs.saved_buf);
    }
#endif
	else {  // anything else is interpreted as Gcode

    	// this optimization bypasses the standard JSON parser and does what it needs directly
    	nvObj_t *nv = nv_reset_nv_list(NUL);                // get a fresh nvObj list
    	strcpy(nv->token, "gc");                            // label is as a Gcode block (do not get an index - not necessary)
    	if (nv_copy_string(nv, cs.bufp) == STAT_OK) {       // copy the Gcode line (for the echo)
    	    nv->valuetype = TYPE_STRING;
    	} else {
    	    nv->valuetype = TYPE_EMPTY;                     // too long to echo: report the status without it
    	}
    	float status = gc_gcode_parser(cs.bufp);
    	nv_print_list(status, TEXT_NO_DISPLAY, JSON_RESPONSE);
    	sr_request_status_report(SR_REQUEST_TIMED);         // generate incremental status report to show any changes
	}
    if (motion_sync.state == SYNC_REQUESTED) {
        motion_sync.state = SYNC_WAITING;
    }
}

/*
 * controller_dispatch_txt_container - callout from JSON parser kernal to run text commands
 *
 *	For text-mode commands this starts a JSON response then runs the text command
 *  in a 'msg' element. Text lines are escaped with JSON-friendly line ends
 *  (e,g \n instead of LF). The text response string is closed, then
 *  json_continuation is set so that the JSON response is properly completed.
 *
 *  Gcode is run as if it had been sent as {"gc":"..."}: no text-mode prompt goes into the JSON
 *  stream, and the Gcode status is returned for the JSON footer (E057).
 */

stat_t controller_dispatch_txt_container (nvObj_t *nv, char *str)
{
    // process pure text-mode commands
    if (strchr("$?Hh", *str) != NULL) {             // a match indicates text mode
        if (js.json_syntax == JSON_SYNTAX_RELAXED) {// opening JSON
            printf_P(PSTR("{r:{txt:\""));
        } else {
            printf_P(PSTR("{\"r\":{\"txt\":\""));
        }
        cs.comm_mode = JSON_MODE_TXT_OVERRIDE;      // override JSON mode for this output only
        xio.json_escape = true;                     // the text goes inside the JSON string (E137)
        text_parser(str);
        xio.json_escape = false;
        cs.comm_mode = JSON_MODE;                   // restore JSON mode
        printf_P(PSTR("\""));                       // close quote
        nv_reset_nv_list(NUL);                      // reset the list to start at the head
        NV_HEAD->valuetype = TYPE_TXTCON;           // label the list as a text container
        return (STAT_OK);
    }
    // process gcode
    strcpy(nv->token,"gc");
    nv->valuetype = TYPE_STRING;                    // echo like a gc pair, not a text container
    return (gc_gcode_parser(str));
}

/**** Local Utilities ********************************************************/
/*
 * _shutdown_handler() - put system into shutdown state
 * _limit_switch_handler() - alarm system if limit switch fired
 * _normal_idler() - blink Indicator LED slowly to show everything is OK
 *
 *	Shutdown idler flashes indicator LED rapidly to show everything is not OK.
 *	Shutdown idler returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) or bootloader request can be processed.
 */

static stat_t _shutdown_handler()
{
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) { return (STAT_OK);}

	// No indicator blink here: INDICATOR_LED is the spindle direction output (hardware.h), and
	// toggling it would flip the direction line of a spindle drive that is still wired up.
	// Let control lines run so {clear:n} can exit SHUTDOWN without a reset (E045);
	// cm_is_alarmed() still rejects G-code and settings while shut down.
	return (STAT_OK);
}

/*
 * _limit_switch_handler() - shut down system if limit switch fired
 */

void controller_assert_limit_condition(uint8_t input)
{
    cs.limit_switch_asserted = input;
}

static stat_t _limit_switch_handler(void)
{
    if ((cm.machine_state == MACHINE_ALARM) ||
        (cm.machine_state == MACHINE_PANIC) ||
        (cm.machine_state == MACHINE_SHUTDOWN)) {
        return (STAT_NOOP);                             // don't test limits if already in an alarm state
    }

    if (cs.limit_switch_asserted == 0) {
        return(STAT_NOOP);
    }
    char msg[12];
    sprintf_P(msg, PSTR("input %d"), cs.limit_switch_asserted);
    cm_alarm(STAT_LIMIT_SWITCH_HIT, msg);
    cs.limit_switch_asserted = 0;
    for (uint8_t axis=0; axis<AXES; axis++) {        // a limit hit means the machine was not where it
        cm.homed[axis] = false;                     // thought it was: positions are kept, homing is not
    }
    cm.homing_state = HOMING_NOT_HOMED;

    return (STAT_OK);
}

static stat_t _normal_idler()
{
/*
	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + LED_NORMAL_TIMER;
//		IndicatorLed_toggle();
	}
*/
	return (STAT_OK);
}

/*
 * controller_reset_source() 		 - reset source to default input device (see note)
 * controller_set_primary_source() 	 - set current primary input source
 * controller_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 * be expanded to also set the stdout/stderr console device so the prompt
 * and other messages are sent to the active device.
 */

void controller_reset_source() { controller_set_primary_source(xio.default_src);}
void controller_set_primary_source(uint8_t dev) { xio.primary_src = dev;}
void controller_set_secondary_source(uint8_t dev) { xio.secondary_src = dev;}
void controller_request_enquiry() { cs.enquiry_requested = true; }	// called from the RX ISR: no printing there (E052)

static stat_t _enquiry_handler()
{
	if (!cs.enquiry_requested) { return (STAT_NOOP); }
	cs.enquiry_requested = false;
	printf_P(PSTR("{\"ack\":true}\n"));
	return (STAT_OK);
}

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

/* Capture only completion metadata; control queries reuse the shared NV/output buffers. */
stat_t controller_request_motion_sync(void)
{
    if (motion_sync.state != SYNC_OFF) { return (STAT_COMMAND_NOT_ACCEPTED); }
    if (!js.motion_sync_allowed) { return (STAT_COMMAND_NOT_ACCEPTED); }
    motion_sync.tid = js.tid;
    motion_sync.tid_present = js.tid_present;
    motion_sync.state = SYNC_REQUESTED;
    motion_sync.reply = SYNC_SILENT;
    motion_sync.char_mode = (xio.rx_mode == RX_MODE_CHAR);
    motion_sync.linelen = cs.linelen;
    return (STAT_OK);
}

bool controller_defer_motion_response(const stat_t status, bool json)
{
    if (motion_sync.state != SYNC_REQUESTED) { return (false); }
    if (status != STAT_OK) {
        motion_sync.state = SYNC_OFF;
        return (false);
    }
    motion_sync.reply = json ? SYNC_JSON : SYNC_TEXT;
    return (true);
}

static stat_t _sync_to_motion(void)
{
    if (motion_sync.state == SYNC_OFF) { return (STAT_NOOP); }
    // Include the prepared segment: its planner buffer may already be freed.
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    stat_t status = cm_is_alarmed();
    bool busy = mp_has_runnable_buffer() || !mp_runtime_is_idle() ||
                (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER);
#ifdef __AVR
    SREG = saved_sreg;
#endif
    if (status == STAT_OK && busy) { return (STAT_EAGAIN); }
    motion_sync.state = SYNC_OFF;
    if (motion_sync.reply == SYNC_TEXT) {
        char command[5];
        strcpy_P(command, PSTR("M400"));
        nv_reset_nv_list(NUL);
        text_response(status, command);
    } else if (motion_sync.reply == SYNC_JSON) {
        fputs_P(PSTR("{\"r\":{\"gc\":\"M400\"}"), stdout);
        if (motion_sync.tid_present) {
            fprintf_P(stdout, PSTR(",\"tid\":%lu"), (unsigned long)motion_sync.tid);
        }
        fprintf_P(stdout, PSTR(",\"f\":[%u,%u,%u]}\n"),
                  (unsigned int)(motion_sync.char_mode ? 2 : 3), (unsigned int)status,
                  (unsigned int)(motion_sync.char_mode ? motion_sync.linelen : xio_get_line_buffers_available()));
    }
    return (STAT_OK);
}

static stat_t _sync_to_planner()
{
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}


/*
 * _init_assertions() - initialize controller memory integrity assertions
 * _test_assertions() - check controller memory integrity assertions
 * _test_system_assertions() - check assertions for entire system
 */

static void _init_assertions()
{
	cs.magic_start = MAGICNUM;
	cs.magic_end = MAGICNUM;
}

static stat_t _test_assertions()
{
	if ((cs.magic_start != MAGICNUM) || (cs.magic_end != MAGICNUM)) {
        return(cm_panic_P(STAT_CONTROLLER_ASSERTION_FAILURE, PSTR("controller_test_assertions()")));
    }
	return (STAT_OK);
}

stat_t _test_system_assertions()
{
    // these functions will panic if an assertion fails
	_test_assertions();                     // controller assertions (local)
	config_test_assertions();
	canonical_machine_test_assertions();
	planner_test_assertions();
	stepper_test_assertions();
	encoder_test_assertions();
	xio_test_assertions();
	return (STAT_OK);
}
