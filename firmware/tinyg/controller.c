/*
 * controller.c - tinyg controller and top level parser
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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
#include "gpio.h"
#include "report.h"
#include "help.h"
#include "util.h"
#include "xio.h"
//#include "settings.h"

#ifdef __ARM
#include "Reset.h"
#endif

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS *********************************************************
 ***********************************************************************************/

controller_t cs;		// controller state structure

/***********************************************************************************
 **** STATICS AND LOCALS ***********************************************************
 ***********************************************************************************/

static void _controller_HSM(void);
static stat_t _led_indicator(void);             // twiddle the LED indicator
static stat_t _shutdown_handler(void);          // new (replaces _interlock_estop_handler)
static stat_t _interlock_handler(void);         // new (replaces _interlock_estop_handler)
static stat_t _limit_switch_handler(void);      // revised for new GPIO code

static void _init_assertions(void);
static stat_t _test_system_assertions(void);

static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _dispatch_command(void);
//static stat_t _dispatch_control(void);
//static void _dispatch_kernel(void);
static stat_t _controller_state(void);          // manage controller state transitions


/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err)
{
    // preserve settable parameters that may have already been set up
//    uint8_t comm_mode = cs.comm_mode;

	memset(&cs, 0, sizeof(controller_t));           // clear all values, job_id's, pointers and status
	_init_assertions();

	cs.fw_build = TINYG_FIRMWARE_BUILD;
	cs.fw_version = TINYG_FIRMWARE_VERSION;
	cs.hw_platform = TINYG_HARDWARE_PLATFORM;       // NB: HW version is set from EEPROM
	cs.controller_state = CONTROLLER_STARTUP;       // ready to run startup lines

#ifdef __AVR
	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	cs.default_src = std_in;
	cs_set_primary_source(cs.default_src);
#endif

#ifdef __ARM
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

    DISPATCH(_led_indicator());				    // blink LEDs at the current rate
    DISPATCH(_shutdown_handler());              // invoke shutdown
    DISPATCH(_interlock_handler());             // invoke / remove safety interlock
    DISPATCH(_limit_switch_handler());          // invoke limit switch
    DISPATCH(_controller_state());              // controller state management
    DISPATCH(_test_system_assertions());        // system integrity assertions
//    DISPATCH(_dispatch_control());              // read any control messages prior to executing cycles

//----- planner hierarchy for gcode and cycles ---------------------------------------//

    DISPATCH(st_motor_power_callback());        // stepper motor power sequencing
    DISPATCH(sr_status_report_callback());      // conditionally send status report
    DISPATCH(qr_queue_report_callback());       // conditionally send queue report
    DISPATCH(rx_report_callback());             // conditionally send rx report

    DISPATCH(cm_feedhold_sequencing_callback());// feedhold state machine runner
//    DISPATCH(mp_planner_callback());		    // motion planner
    DISPATCH(cm_arc_callback());                // arc generation runs as a cycle above lines
    DISPATCH(cm_homing_cycle_callback());       // homing cycle operation (G28.2)
    DISPATCH(cm_probing_cycle_callback());      // probing cycle operation (G38.2)
    DISPATCH(cm_jogging_cycle_callback());      // jog cycle operation
    DISPATCH(cm_deferred_write_callback());     // persist G10 changes when not in machining cycle

//----- command readers and parsers --------------------------------------------------//

    DISPATCH(_sync_to_planner());               // ensure there is at least one free buffer in planning queue
    DISPATCH(_sync_to_tx_buffer());             // sync with TX buffer (pseudo-blocking)
#ifdef __AVR
    DISPATCH(set_baud_callback());              // perform baud rate update (must be after TX sync)
#endif
    DISPATCH(_dispatch_command());              // MUST BE LAST - read and execute next command
}

/*****************************************************************************
 * _dispatch_command() - dispatch line received from active input device
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control
 */

static stat_t _dispatch_command()
{
#ifdef __AVR
	stat_t status;

	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	while (true) {
		if ((status = xio_gets(cs.primary_src, cs.in_buf, sizeof(cs.in_buf))) == STAT_OK) {
			cs.bufp = cs.in_buf;
			break;
		}
		// handle end-of-file from file devices
		if (status == STAT_EOF) {						// EOF can come from file devices only
			if (cs.comm_mode == TEXT_MODE) {
				fprintf_P(stderr, PSTR("End of command file\n"));
			} else {
				rpt_exception(STAT_EOF, "EOF");			// not really an exception
			}
			cs_reset_source();							// reset to default source
		}
		return (status);								// Note: STAT_EAGAIN, errors, etc. will drop through
	}
#endif // __AVR
#ifdef __ARM
	// detect USB connection and transition to disconnected state if it disconnected
	if (SerialUSB.isConnected() == false) cs.state = CONTROLLER_NOT_CONNECTED;

	// read input line and return if not a completed line
	if (cs.state == CONTROLLER_READY) {
		if (read_line(cs.in_buf, &cs.read_index, sizeof(cs.in_buf)) != STAT_OK) {
			cs.bufp = cs.in_buf;
			return (STAT_OK);	// This is an exception: returns OK for anything NOT OK, so the idler always runs
		}
	} else if (cs.state == CONTROLLER_NOT_CONNECTED) {
		if (SerialUSB.isConnected() == false) return (STAT_OK);
		cm_request_queue_flush();
		rpt_print_system_ready_message();
		cs.state = CONTROLLER_STARTUP;

	} else if (cs.state == CONTROLLER_STARTUP) {		// run startup code
		cs.state = CONTROLLER_READY;

	} else {
		return (STAT_OK);
	}
	cs.read_index = 0;
#endif // __ARM

	// set up the buffers

	cs.linelen = strlen(cs.in_buf)+1;					// linelen only tracks primary input
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(*cs.bufp)) {						// first char

		case '!': { cm_request_feedhold(); break; }		// include for AVR diagnostics and ARM serial
		case '%': { cm_request_queue_flush(); break; }
		case '~': { cm_request_end_hold(); break; }

		case NUL: { 									// blank line (just a CR)
			if (cs.comm_mode != JSON_MODE) {
				text_response(STAT_OK, cs.saved_buf);
			}
			break;
		}
		case '$': case '?': case 'H': { 				// text mode input
			cs.comm_mode = TEXT_MODE;
			text_response(text_parser(cs.bufp), cs.saved_buf);
			break;
		}
		case '{': { 									// JSON input
			cs.comm_mode = JSON_MODE;
			json_parser(cs.bufp);
			break;
		}
		default: {										// anything else must be Gcode
			if (cs.comm_mode == JSON_MODE) {			// run it as JSON...
				strncpy(cs.out_buf, cs.bufp, INPUT_BUFFER_LEN -8);					// use out_buf as temp
				sprintf((char *)cs.bufp,"{\"gc\":\"%s\"}\n", (char *)cs.out_buf);	// '-8' is used for JSON chars
				json_parser(cs.bufp);
			} else {									//...or run it as text
				text_response(gc_gcode_parser(cs.bufp), cs.saved_buf);
			}
		}
	}
	return (STAT_OK);
}

/**** Local Functions ********************************************************/
/* CONTROLLER STATE MANAGEMENT
 * _controller_state() - manage controller connection, startup, and other state changes
 */

static stat_t _controller_state()
{
	if (cs.controller_state == CONTROLLER_CONNECTED) {		// first time through after reset
		cs.controller_state = CONTROLLER_READY;
        // Oops, we just skipped CONTROLLER_STARTUP. Do we still need it? -r
		rpt_print_system_ready_message();
	}
	return (STAT_OK);
}

/*
 * controller_set_connected(bool) - hook for xio to tell the controller that we
 * have/don't have a connection.
 */
/*
void controller_set_connected(bool is_connected) {
    if (is_connected) {
        cs.controller_state = CONTROLLER_CONNECTED; // we JUST connected
    } else {  // we just disconnected from the last device, we'll expect a banner again
        cs.controller_state = CONTROLLER_NOT_CONNECTED;
    }
}
*/
/*
 * controller_parse_control() - return true if command is a control (versus data)
 * Note: parsing for control is somewhat naiive. This will need to get better
 */
/*
bool controller_parse_control(char *p) {
    if (strchr("{$?!~%Hh", *p) != NULL) {		    // a match indicates control line
        return (true);
    }
    return (false);
}
*/

/*
 * _led_indicator() - blink an LED to show it we are normal, alarmed, or shut down
 */

static stat_t _led_indicator()
{
    uint32_t blink_rate;
    if (cm_get_machine_state() == MACHINE_ALARM) {
        blink_rate = LED_ALARM_BLINK_RATE;
    } else if (cm_get_machine_state() == MACHINE_SHUTDOWN) {
        blink_rate = LED_SHUTDOWN_BLINK_RATE;
    } else if (cm_get_machine_state() == MACHINE_PANIC) {
        blink_rate = LED_PANIC_BLINK_RATE;
    } else {
        blink_rate = LED_NORMAL_BLINK_RATE;
    }

    if (blink_rate != cs.led_blink_rate) {
        cs.led_blink_rate =  blink_rate;
        cs.led_timer = 0;
    }
	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + cs.led_blink_rate;
		IndicatorLed_toggle();
	}
	return (STAT_OK);
}

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 * _sync_to_time() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

static stat_t _sync_to_planner()
{
	if (cm.buffers_drain_state == DRAIN_REQUESTED) {
		if (mp_get_planner_buffers_available() < PLANNER_BUFFER_POOL_SIZE) { // need to drain it
			return (STAT_EAGAIN);
		}
		else {
			cm.buffers_drain_state = DRAIN_OFF;
			return (STAT_COMPLETE);
		}
	}
	else {
		if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
			return (STAT_EAGAIN);
		}
	}
	return (STAT_OK);
}

/*
 * cs_reset_source() 		 - reset source to default input device (see note)
 * cs_set_primary_source() 	 - set current primary input source
 * cs_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 * be expanded to also set the stdout/stderr console device so the prompt
 * and other messages are sent to the active device.
 */

void cs_reset_source() { cs_set_primary_source(cs.default_src);}
void cs_set_primary_source(uint8_t dev) { cs.primary_src = dev;}
void cs_set_secondary_source(uint8_t dev) { cs.secondary_src = dev;}

/* ALARM STATE HANDLERS
 *
 * _shutdown_handler() - put system into shutdown state
 * _limit_switch_handler() - shut down system if limit switch fired
 * _interlock_handler() - feedhold and resume depending on edge
 *
 *	Some handlers return EAGAIN causing the control loop to never advance beyond that point.
 *
 * _interlock_handler() reacts the follwing ways:
 *   - safety_interlock_requested == INPUT_EDGE_NONE is normal operation (no interlock)
 *   - safety_interlock_requested == INPUT_EDGE_LEADING is interlock onset
 *   - safety_interlock_requested == INPUT_EDGE_TRAILING is interlock offset
 */
static stat_t _shutdown_handler(void)
{
    if (cm.shutdown_requested != 0) {  // request may contain the (non-zero) input number
	    char msg[10];
	    sprintf_P(msg, PSTR("input %d"), (int)cm.shutdown_requested);
	    cm.shutdown_requested = false; // clear limit request used here ^
        cm_shutdown(STAT_SHUTDOWN, msg);
    }
    return(STAT_OK);
}

static stat_t _limit_switch_handler(void)
{
    if (cm.limit_enable && (cm.limit_requested != 0)) {
	    char msg[10];
	    sprintf_P(msg, PSTR("input %d"), (int)cm.limit_requested);
        cm.limit_requested = false; // clear limit request used here ^
        cm_alarm(STAT_LIMIT_SWITCH_HIT, msg);
    }
    return (STAT_OK);
}

static stat_t _interlock_handler(void)
{
    if (cm.safety_interlock_enable) {
    // interlock broken
        if (cm.safety_interlock_disengaged != 0) {
            cm.safety_interlock_disengaged = 0;
            cm.safety_interlock_state = SAFETY_INTERLOCK_DISENGAGED;
            cm_request_feedhold();                                  // may have already requested STOP as INPUT_ACTION
            // feedhold was initiated by input action in gpio
            // pause spindle
            // pause coolant
        }

        // interlock restored
        if ((cm.safety_interlock_reengaged != 0) && (mp_runtime_is_idle())) {
            cm.safety_interlock_reengaged = 0;
            cm.safety_interlock_state = SAFETY_INTERLOCK_ENGAGED;   // interlock restored
            // restart spindle with dwell
            cm_request_end_hold();                                // use cm_request_end_hold() instead of just ending
            // restart coolant
        }
    }
    return(STAT_OK);
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
        return(cm_panic(STAT_CONTROLLER_ASSERTION_FAILURE, "controller_test_assertions()"));
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
