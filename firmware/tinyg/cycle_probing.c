/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S Hart, Jr.
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
#include "json_parser.h"
#include "text_parser.h"
#include "canonical_machine.h"
#include "spindle.h"
#include "report.h"
#include "switch.h"
#include "util.h"
#include "planner.h"

/**** Probe singleton structure ****/

#define MINIMUM_PROBE_TRAVEL 0.254

struct pbProbingSingleton {						// persistent probing runtime variables
	stat_t (*func)();							// binding for callback function state machine

	// switch configuration
	uint8_t probe_switch;						// which switch should we check?

	// state saved from gcode model
	uint8_t saved_distance_mode;				// G90,G91 global setting
	float saved_jerk[AXES];						// saved and restored for each axis

	// probe destination
	float start_position[AXES];
	float target[AXES];
	bool flags[AXES];
};
static struct pbProbingSingleton pb;

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_init();
static stat_t _probing_start();
static stat_t _probing_finish();
static stat_t _probing_finalize_exit();
static stat_t _probing_error_exit(int8_t axis);


/**** HELPERS ***************************************************************************
 * _set_pb_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_pb_func(uint8_t (*func)())
{
	pb.func = func;
	return (STAT_EAGAIN);
}

/****************************************************************************************
 * cm_probing_cycle_start()	- G38.2 homing cycle using limit switches
 * cm_probing_callback() 	- main loop callback for running the homing cycle
 *
 *	--- Some further details ---
 *
 *	All cm_probe_cycle_start does is prevent any new commands from queueing to the
 *	planner so that the planner can move to a sop and report MACHINE_PROGRAM_STOP.
 *	OK, it also queues the function that's called once motion has stopped.
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued move per
 *	entry into the continuation, then you must exit.
 *
 *	Another Note: When coding a cycle (like this one) you must wait until
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done. Otherwise there is a nasty race condition in the
 *	tg_controller() that will accept the next command before the position of
 *	the final move has been recorded in the Gcode model. That's what the call
 *	to cm_get_runtime_busy() is about.
 */

uint8_t cm_straight_probe(const float target[], const bool flags[])
{
	// trap feed rate errors
    if (!cm.gf.feed_rate) {
        return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    }
	if ((cm.gm.feed_rate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feed_rate))) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}

	// trap missing XYZ axes
    if (!(flags[AXIS_X] || flags[AXIS_Y] || flags[AXIS_Z])) {
		return (STAT_GCODE_AXIS_IS_MISSING);
    }

	// trap ABC axes
    if (flags[AXIS_A] || flags[AXIS_B] || flags[AXIS_C]) {
        return(STAT_GCODE_AXIS_CANNOT_BE_PRESENT);
    }

    // locate the probe switch
    pb.probe_switch = find_probe_switch();
    if (pb.probe_switch == -1) {
        return(STAT_NO_PROBE_SWITCH_CONFIGURED);
    }
    if (pb.probe_switch == -2) {
        return(STAT_MULTIPLE_PROBE_SWITCHES_CONFIGURED);
    }

	// set probe move endpoint
	copy_vector(pb.target, target);		// set probe move endpoint
	copy_vector(pb.flags, flags);		// set axes involved on the move
	clear_vector(cm.probe_results);		// clear the old probe position.
										// NOTE: relying on probe_result will not detect a probe to 0,0,0.

	cm.probe_state = PROBE_WAITING;		// wait until planner queue empties before completing initialization
	pb.func = _probing_init; 			// bind probing initialization function
	return (STAT_OK);
}

uint8_t cm_probe_callback(void)
{
	if ((cm.cycle_state != CYCLE_PROBE) && (cm.probe_state != PROBE_WAITING)) {
		return (STAT_NOOP);				// exit if not in a probe cycle or waiting for one
	}
	if (cm_get_runtime_busy()) {        // sync to planner move ends
        return (STAT_EAGAIN);
    }
	return (pb.func());                 // execute the current homing move
}

/*
 * _probing_init()	- G38.2 homing cycle using limit switches
 *
 *	These initializations are required before starting the probing cycle.
 *	They must be done after the planner has exhausted all current CYCLE moves as
 *	they affect the runtime (specifically the switch modes). Side effects would
 *	include limit switches initiating probe actions instead of just killing movement
 */

static uint8_t _probing_init()
{
	// so optimistic... ;)
	// NOTE: it is *not* an error condition for the probe not to trigger.
	// it is an error for the limit or homing switches to fire, or for some other configuration error.
	cm.probe_state = PROBE_FAILED;
	cm.cycle_state = CYCLE_PROBE;

	// initialize the axes - save the jerk settings & switch to the jerk_homing settings
	for( uint8_t axis=0; axis<AXES; axis++ ) {
		pb.saved_jerk[axis] = cm_get_axis_jerk(axis);	// save the max jerk value
		cm_set_axis_jerk(axis, cm.a[axis].jerk_homing);	// use the homing jerk for probe
		pb.start_position[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
	}

	// error if the probe target is too close to the current position
	if (get_axis_vector_length(pb.start_position, pb.target) < MINIMUM_PROBE_TRAVEL) {
		_probing_error_exit(-2);
    }

	// probe in workspace coordinate system, absolute distance
	pb.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);   //cm.gm.distance_mode;
	cm_set_distance_mode(ABSOLUTE_MODE);
	cm_spindle_control(SPINDLE_OFF);
	return (_set_pb_func(_probing_start));							// start the move
}

/*
 * _probing_start()
 */

static stat_t _probing_start()
{
	// initial probe state, don't probe if we're already contacted!
    if (read_switch(pb.probe_switch) == SW_INACTIVE) {              // reads and sets initial switch state
        ritorno(cm_straight_feed(pb.target, pb.flags));
    }
	return (_set_pb_func(_probing_finish));
}

/*
 * _probing_finish()
 */

static stat_t _probing_finish()
{
	int8_t probe = sw.s[pb.probe_switch].state;
	cm.probe_state = (probe == SW_ACTIVE) ? PROBE_SUCCEEDED : PROBE_FAILED;

	for( uint8_t axis=0; axis<AXES; axis++ ) {
		// if we got here because of a feed hold we need to keep the model position correct
		cm_set_position(axis, mp_get_runtime_work_position(axis));

		// store the probe results
		cm.probe_results[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
	}
	json_parser("{\"prb\":null}");
	return (_set_pb_func(_probing_finalize_exit));
}

/*
 * _probe_restore_settings()
 * _probing_finalize_exit()
 * _probing_error_exit()
 */

static void _probe_restore_settings()
{
	mp_flush_planner(); 						// we should be stopped now, but in case of switch closure
//    cm_queue_flush();       // flush queue, set position, end hold

	// restore axis jerk
	for( uint8_t axis=0; axis<AXES; axis++ ) {
		cm_set_axis_jerk(axis, pb.saved_jerk[axis]);
    }
	cm_set_distance_mode(pb.saved_distance_mode);

	// update the model with actual position
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL);
	cm_cycle_end();
	cm.cycle_state = CYCLE_OFF;
}

static stat_t _probing_finalize_exit()
{
	_probe_restore_settings();
	return (STAT_OK);
}

static stat_t _probing_error_exit(int8_t axis)
{
	// Generate the warning message. Since the error exit returns via the probing callback
	// - and not the main controller - it requires its own display processing
	nv_reset_nv_list("");
	if (axis == -2) {
		nv_add_conditional_message_P(PSTR("Probing error - invalid probe destination"));
	} else {
		char msg[NV_MESSAGE_LEN];
		sprintf_P(msg, PSTR("Probing error - %c axis cannot move during probing"), cm_get_axis_char(axis));
		nv_add_conditional_message(msg);
	}
	nv_print_list(STAT_PROBE_CYCLE_FAILED, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);

	// clean up and exit
	_probe_restore_settings();
	return (STAT_PROBE_CYCLE_FAILED);
}
