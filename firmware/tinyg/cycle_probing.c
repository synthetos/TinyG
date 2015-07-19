/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S Hart, Jr.
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
#ifndef __NEW_SWITCHES
	uint8_t probe_switch;						// which switch should we check?
	uint8_t saved_switch_type;					// saved switch type NO/NC
	uint8_t saved_switch_mode;	                // save the probe switch's original settings
#else
	uint8_t probe_switch_axis;					// which axis should we check?
	uint8_t probe_switch_position;				//...and position
	uint8_t saved_switch_type;					// saved switch type NO/NC
	uint8_t saved_switch_mode;					// save the probe switch's original settings
#endif

	// state saved from gcode model
	uint8_t saved_distance_mode;				// G90,G91 global setting
	uint8_t saved_coord_system;					// G54 - G59 setting
	float saved_jerk[AXES];						// saved and restored for each axis

	// probe destination
	float start_position[AXES];
	float target[AXES];
	float flags[AXES];
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

uint8_t cm_straight_probe(float target[], float flags[])
{
	// trap zero feed rate condition
	if ((cm.gm.feed_rate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feed_rate))) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}

	// trap no axes specified
	if (fp_NOT_ZERO(flags[AXIS_X]) && fp_NOT_ZERO(flags[AXIS_Y]) && fp_NOT_ZERO(flags[AXIS_Z]))
		return (STAT_GCODE_AXIS_IS_MISSING);

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
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN);}	// sync to planner move ends
	return (pb.func());                                         // execute the current homing move
}

/*
 * _probing_init()	- G38.2 homing cycle using limit switches
 *
 *	These initializations are required before starting the probing cycle.
 *	They must be done after the planner has exhasted all current CYCLE moves as
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
	if (get_axis_vector_length(pb.start_position, pb.target) < MINIMUM_PROBE_TRAVEL)
		_probing_error_exit(-2);

	// error if the probe target requires a move along the A/B/C axes
	for ( uint8_t axis=AXIS_A; axis<AXES; axis++ ) {
		if (fp_NE(pb.start_position[axis], pb.target[axis]))
			_probing_error_exit(axis);
	}

	// initialize the probe switch

    // switch the switch type mode for the probe
    // FIXME: we should be able to use the homing switch at this point too,
	// Can't because switch mode is global and our probe is NO, not NC.

#ifndef __NEW_SWITCHES	// old style switch code:
	pb.probe_switch = SW_MIN_Z;										// FIXME: hardcoded...
	pb.saved_switch_mode = sw.mode[pb.probe_switch];

	sw.mode[pb.probe_switch] = SW_MODE_HOMING;
	pb.saved_switch_type = sw.switch_type;							// save the switch type for recovery later.
	sw.switch_type = SW_TYPE_NORMALLY_OPEN;							// contact probes are NO switches... usually
	switch_init();													// re-init to pick up new switch settings
#else // new style switch code:
	pb.probe_switch_axis = AXIS_Z;									// FIXME: hardcoded...
	pb.probe_switch_position = SW_MIN;								// FIXME: hardcoded...

	pb.saved_switch_mode = sw.s[pb.probe_switch_axis][pb.probe_switch_position].mode;
	sw.s[pb.probe_switch_axis][pb.probe_switch_position].mode = SW_MODE_HOMING;

	pb.saved_switch_type = sw.s[pb.probe_switch_axis][pb.probe_switch_position].type;
	sw.s[pb.probe_switch_axis][pb.probe_switch_position].type = SW_TYPE_NORMALLY_OPEN; // contact probes are NO switches... usually.
	switch_init();													// re-init to pick up new switch settings
#endif

	// probe in absolute machine coords
	pb.saved_coord_system = cm_get_coord_system(ACTIVE_MODEL);     //cm.gm.coord_system;
	pb.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);   //cm.gm.distance_mode;
	cm_set_distance_mode(ABSOLUTE_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);

	cm_spindle_control(SPINDLE_OFF);
	return (_set_pb_func(_probing_start));							// start the move
}

/*
 * _probing_start()
 */

static stat_t _probing_start()
{
	// initial probe state, don't probe if we're already contacted!
#ifndef __NEW_SWITCHES
	int8_t probe = sw.state[pb.probe_switch];
#else
	int8_t probe = read_switch(pb.probe_switch_axis, pb.probe_switch_position);
#endif

    if( probe==SW_OPEN ) {
        ritorno(cm_straight_feed(pb.target, pb.flags));
    }
	return (_set_pb_func(_probing_finish));
}

/*
 * _probing_finish()
 */

static stat_t _probing_finish()
{
#ifndef __NEW_SWITCHES
	int8_t probe = sw.state[pb.probe_switch];
#else
	int8_t probe = read_switch(pb.probe_switch_axis, pb.probe_switch_position);
#endif
	cm.probe_state = (probe==SW_CLOSED) ? PROBE_SUCCEEDED : PROBE_FAILED;

	for( uint8_t axis=0; axis<AXES; axis++ ) {
		// if we got here because of a feed hold we need to keep the model position correct
		cm_set_position(axis, mp_get_runtime_work_position(axis));

		// store the probe results
		cm.probe_results[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
	}

	json_parser("{\"prb\":null}"); // TODO: verify that this is OK to do...
	// printf_P(PSTR("{\"prb\":{\"e\":%i"), (int)cm.probe_state);
	// if (pb.flags[AXIS_X]) printf_P(PSTR(",\"x\":%0.3f"), cm.probe_results[AXIS_X]);
	// if (pb.flags[AXIS_Y]) printf_P(PSTR(",\"y\":%0.3f"), cm.probe_results[AXIS_Y]);
	// if (pb.flags[AXIS_Z]) printf_P(PSTR(",\"z\":%0.3f"), cm.probe_results[AXIS_Z]);
	// if (pb.flags[AXIS_A]) printf_P(PSTR(",\"a\":%0.3f"), cm.probe_results[AXIS_A]);
	// if (pb.flags[AXIS_B]) printf_P(PSTR(",\"b\":%0.3f"), cm.probe_results[AXIS_B]);
	// if (pb.flags[AXIS_C]) printf_P(PSTR(",\"c\":%0.3f"), cm.probe_results[AXIS_C]);
	// printf_P(PSTR("}}\n"));

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

#ifndef __NEW_SWITCHES // restore switch settings (old style)
	sw.switch_type = pb.saved_switch_type;
	sw.mode[pb.probe_switch] = pb.saved_switch_mode;
	switch_init();								// re-init to pick up changes
#else // restore switch settings (new style)
	sw.s[pb.probe_switch_axis][pb.probe_switch_position].mode = pb.saved_switch_mode;
	sw.s[pb.probe_switch_axis][pb.probe_switch_position].type = pb.saved_switch_type;
	switch_init();								// re-init to pick up changes
#endif

	// restore axis jerk
	for( uint8_t axis=0; axis<AXES; axis++ )
		cm_set_axis_jerk(axis, pb.saved_jerk[axis]);

	// restore coordinate system and distance mode
	cm_set_coord_system(pb.saved_coord_system);
	cm_set_distance_mode(pb.saved_distance_mode);

	// update the model with actual position
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
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
	nv_reset_nv_list();
	if (axis == -2) {
		nv_add_conditional_message((const char_t *)"Probing error - invalid probe destination");
	} else {
		char message[NV_MESSAGE_LEN];
		sprintf_P(message, PSTR("Probing error - %c axis cannot move during probing"), cm_get_axis_char(axis));
		nv_add_conditional_message((char_t *)message);
	}
	nv_print_list(STAT_PROBE_CYCLE_FAILED, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);

	// clean up and exit
	_probe_restore_settings();
	return (STAT_PROBE_CYCLE_FAILED);
}
