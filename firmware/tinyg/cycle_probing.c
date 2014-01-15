/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
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
#include "util.h"
#include "config.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "switch.h"

/**** Probe singleton structure ****/

struct pbProbingSingleton {		// persistent homing runtime variables
    stat_t (*func)();// binding for callback function state machine
    
	// state saved from gcode model
    uint8_t saved_switch_type;  // saved switch type NC/NO
    uint8_t saved_switch_mode[NUM_SWITCHES];
    uint8_t probe_switch;       // what switch should we check?
    
    // probe destination
    float target[AXES];
    float flags[AXES];
    
	float saved_jerk[AXES];		// saved and restored for each axis
};
static struct pbProbingSingleton pb;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_start();
static stat_t _probing_finish();
//static stat_t _probing_error_exit();

static stat_t _set_pb_func(uint8_t (*func)());


/*****************************************************************************
 * cm_probing_cycle_start()	- G38.2 homing cycle using limit switches
 * cm_probing_callback() 	- main loop callback for running the homing cycle
 *
 */
/*	--- Some further details ---
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued 
 *	move per entry into the continuation, then you must exit. 
 *
 *	Another Note: When coding a cycle (like this one) you must wait until 
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done. Otherwise there is a nasty race condition in the 
 *	tg_controller() that will accept the next command before the position of 
 *	the final move has been recorded in the Gcode model. That's what the call
 *	to cm_isbusy() is about.
 *
 *  ESTEE: is this still true???
 */

uint8_t cm_probe_cycle_start( float target[], float flags[] )
{
    // so optimistic... ;)
    cm.probe_state = PROBE_FAILED;
    
    // axis init
    for( int axis=0; axis<AXES; axis++ )
    {
        // clear the old probe position, note that realying on probe_result will not detect a probe to 0,0,0.
        cm.probe_results[axis] = 0.0;
        
        // save the jerk settings & switch to the jerk_homing settings
        pb.saved_jerk[axis] = cm.a[axis].jerk_max;         // save the max jerk value
        cm.a[axis].jerk_max = cm.a[axis].jerk_homing;	// use the homing jerk for probe
        
        // set endpoint
        pb.target[axis] = target[axis];
        pb.flags[axis] = flags[axis];
    }

    // switch the switch type mode for the probe
    // FIXME: we should be able to use the homing switch at this point too, can't because switch mode is global
    // and our probe is NO, not NC.
    pb.probe_switch = SW_MIN_Z;         // FIXME: hardcoded...
    for( int i=0; i<NUM_SWITCHES; i++ )
        pb.saved_switch_mode[i] = sw.mode[i];
    
    sw.mode[pb.probe_switch] = SW_MODE_HOMING;
    pb.saved_switch_type = sw.switch_type;  // save the switch type for recovery later.
    sw.switch_type = SW_TYPE_NORMALLY_OPEN; // contact probes are NO switches... usually.
    
    // re-init to pick up new switch settings
    switch_init();
    
    
	pb.func = _probing_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_PROBE;
	st_energize_motors();				// enable motors if not already enabled
	return (STAT_OK);
}

// called when exiting on success or error
void _probe_restore_settings()
{
    mp_flush_planner(); 						// should be stopped, but in case of switch closure
    
    // restore switch settings
    sw.switch_type = pb.saved_switch_type;
    for( int i=0; i<NUM_SWITCHES; i++ )
        sw.mode[i] = pb.saved_switch_mode[i];
    switch_init();
    
    // restore axis jerk
    for( int axis=0; axis<AXES; axis++ )
        cm.a[axis].jerk_max = pb.saved_jerk[axis];
    
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
    cm.cycle_state = CYCLE_OFF;
	cm_cycle_end();
}

uint8_t cm_probe_callback(void)
{
	if (cm.cycle_state != CYCLE_PROBE) { return (STAT_NOOP);}	// exit if not in a homing cycle
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN);}	// sync to planner move ends
	return (pb.func());                                         // execute the current homing move
}

static stat_t _probing_finalize_exit()	// third part of return to home
{
	_probe_restore_settings();
	return (STAT_OK);
}

/* 
 * _probing_error_exit()
 */

//static stat_t _probing_error_exit(int8_t axis)
//{
//	// Generate the warning message. Since the error exit returns via the homing callback 
//	// - and not the main controller - it requires its own display processing 
//	cmd_reset_list();
//	if (axis == -2) {
//		cmd_add_conditional_message((const char_t *)"*** WARNING *** Probing error: Specified axis(es) cannot use probe");
//	} else {
//		char message[CMD_MESSAGE_LEN];
//		sprintf_P(message, PSTR("*** WARNING *** Probing error: %c axis settings misconfigured"), cm_get_axis_char(axis));
//		cmd_add_conditional_message((const char_t *)message);
//	}
//	cmd_print_list(STAT_PROBING_CYCLE_FAILED, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);
//
//	// clean up and exit
//	_probe_restore_settings();
//	return (STAT_PROBING_CYCLE_FAILED);
//}

static stat_t _probing_start()
{
    cm_request_queue_flush();
	cm_request_cycle_start();
    
	ritorno(cm_straight_feed(pb.target, pb.flags));
    
	return (_set_pb_func(_probing_finish));				// start the clear
}

static stat_t _probing_finish()
{
    int8_t probe = read_switch(pb.probe_switch);
    cm.probe_state = (probe==SW_CLOSED) ? PROBE_SUCCEDED : PROBE_FAILED;
    
    for( int i=0; i<AXES; i++ )
        cm.probe_results[i] = cm_get_absolute_position(ACTIVE_MODEL, i);
    
    printf_P(PSTR("{\"prb\":{\"e\":%i,\"x\":%g,\"y\":%g,\"z\":%g}}\n"),
             (int)cm.probe_state, cm.probe_results[AXIS_X], cm.probe_results[AXIS_Y], cm.probe_results[AXIS_Z]);
    
    return (_set_pb_func(_probing_finalize_exit));
}

//// Handle an initial switch closure by backing off switches
//// NOTE: Relies on independent switches per axis (not shared)
//static stat_t _probing_axis_clear(int8_t axis)				// first clear move
//{
//	int8_t homing = read_switch(pb.homing_switch);
//	int8_t limit = read_switch(pb.limit_switch);
//
//	if ((homing == SW_OPEN) && (limit != SW_CLOSED)) {
// 		return (_set_pb_func(_probing_axis_search));		// OK to start the search
//	}
//	if (homing == SW_CLOSED) {
//		_probing_axis_move(axis, pb.latch_backoff, pb.search_velocity);
// 		return (_set_pb_func(_probing_axis_backoff_home));	// will backoff homing switch some more
//	}
//	_probing_axis_move(axis, -pb.latch_backoff, pb.search_velocity);
// 	return (_set_pb_func(_probing_axis_backoff_limit));		// will backoff limit switch some more
//}

//static stat_t _probing_axis_backoff_home(int8_t axis)		// back off cleared homing switch
//{
//	_probing_axis_move(axis, pb.latch_backoff, pb.search_velocity);
//    return (_set_pb_func(_probing_axis_search));
//}

//static stat_t _probing_axis_backoff_limit(int8_t axis)		// back off cleared limit switch
//{
//	_probing_axis_move(axis, -pb.latch_backoff, pb.search_velocity);
//    return (_set_pb_func(_probing_axis_search));
//}
//
//static stat_t _probing_axis_search(int8_t axis)				// start the search
//{
//	cm.a[axis].jerk_max = cm.a[axis].jerk_homing;	// use the homing jerk for search onward
//	_probing_axis_move(axis, pb.search_travel, pb.search_velocity);
//    return (_set_pb_func(_probing_axis_latch));
//}
//
//static stat_t _probing_axis_latch(int8_t axis)				// latch to switch open
//{
//	_probing_axis_move(axis, pb.latch_backoff, pb.latch_velocity);    
//	return (_set_pb_func(_probing_axis_zero_backoff)); 
//}
//
//static stat_t _probing_axis_zero_backoff(int8_t axis)		// backoff to zero position
//{
//	_probing_axis_move(axis, pb.zero_backoff, pb.search_velocity);
//	return (_set_pb_func(_probing_axis_set_zero));
//}
//
//static stat_t _probing_axis_set_zero(int8_t axis)			// set zero and finish up
//{
//	cm.a[axis].jerk_max = pb.saved_jerk;					// restore the max jerk value
//	//cm.homed[axis] = true;
//	return (_set_pb_func(_probing_axis_start));
//}
//
//static stat_t _probing_axis_move(int8_t axis, float target, float velocity)
//{
//	float flags[] = {1,1,1,1,1,1};
//	set_vector_by_axis(target, axis);
//	cm_set_feed_rate(velocity);
//	cm_request_queue_flush();
//	cm_request_cycle_start();
//	ritorno(cm_straight_feed(vector, flags));
//	return (STAT_EAGAIN);
//}

/* _run_homing_dual_axis() - kernal routine for running homing on a dual axis */
//static stat_t _run_homing_dual_axis(int8_t axis) { return (STAT_OK);}

/**** HELPERS ****************************************************************/
/*
 * _set_hm_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_pb_func(uint8_t (*func)())
{
	pb.func = func;
	return (STAT_EAGAIN);
}

///*
// * _get_next_axis() - return next axis in sequence based on axis in arg
// *
// *	Accepts "axis" arg as the current axis; or -1 to retrieve the first axis
// *	Returns next axis based on "axis" argument and if that axis is flagged for homing in the gf struct
// *	Returns -1 when all axes have been processed
// *	Returns -2 if no axes are specified (Gcode calling error)
// *	Homes Z first, then the rest in sequence
// *
// *	Isolating this function facilitates implementing more complex and 
// *	user-specified axis homing orders
// */
//
//int8_t _get_next_axis(int8_t axis)
//{
//	if (axis == -1) {	// Only one axis allowed for probing.  The order below is the priority 
//		if (cm.gf.target[AXIS_Z] == true) return (AXIS_Z);
//		if (cm.gf.target[AXIS_X] == true) return (AXIS_X);
//		if (cm.gf.target[AXIS_Y] == true) return (AXIS_Y);
//		if (cm.gf.target[AXIS_A] == true) return (AXIS_A);
//		return (-2);	// error
//	}
//	return (-1);	// done
//}

