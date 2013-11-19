/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2013 Rob Grzyck 
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
	// controls for homing cycle
	int8_t axis;				// axis currently being homed
	uint8_t min_mode;			// mode for min switch fo this axis
	uint8_t max_mode;			// mode for max switch fo this axis
	int8_t homing_switch;		// homing switch for current axis (index into switch flag table)
	int8_t limit_switch;		// limit switch for current axis, or -1 if none
	uint8_t homing_closed;		// 0=open, 1=closed
	uint8_t limit_closed;		// 0=open, 1=closed
	uint8_t (*func)(int8_t axis);// binding for callback function state machine
	float open_position;
	
	// per-axis parameters
	float direction;			// set to 1 for positive (max), -1 for negative (to min);
	float search_travel;		// signed distance to travel in search
	float search_velocity;		// search speed as positive number
	float latch_velocity;		// latch speed as positive number
	float latch_backoff;		// max distance to back off switch during latch phase 
	float zero_backoff;			// distance to back off switch before setting zero
	float max_clear_backoff;	// maximum distance of switch clearing backoffs before erring out

	// state saved from gcode model
	float saved_feed_rate;		// F setting
	uint8_t saved_units_mode;	// G20,G21 global setting
	uint8_t saved_coord_system;	// G54 - G59 setting
	uint8_t saved_distance_mode;// G90,G91 global setting
	float saved_jerk;			// saved and restored for each axis homed	
};
static struct pbProbingSingleton pb;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_axis_start(int8_t axis);
static stat_t _probing_axis_clear(int8_t axis);
static stat_t _probing_axis_backoff_home(int8_t axis);
static stat_t _probing_axis_backoff_limit(int8_t axis);
static stat_t _probing_axis_search(int8_t axis);
static stat_t _probing_axis_latch(int8_t axis);
static stat_t _probing_axis_zero_backoff(int8_t axis);
static stat_t _probing_axis_set_zero(int8_t axis);
static stat_t _probing_axis_move(int8_t axis, float target, float velocity);
static stat_t _probing_finalize_exit(int8_t axis);
static stat_t _probing_error_exit(int8_t axis);

static stat_t _set_pb_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);


/*****************************************************************************
 * cm_probing_cycle_start()	- G28.1 homing cycle using limit switches
 * cm_probing_callback() 	- main loop callback for running the homing cycle
 *
 * Homing works from a G28.1 according to the following writeup: 
 *	https://github.com/synthetos/TinyG/wiki/TinyG-Homing-(version-0.95-and-above)
 *
 *	--- How does this work? ---
 *
 *	Homing is invoked using a G28.1 command with 1 or more axes specified in the
 *	command: e.g. g28.1 x0 y0 z0     (FYI: the number after each axis is irrelevant)
 *
 *	Homing is always run in the following order - for each enabeled axis:
 *	  Z,X,Y,A			Note: B and C cannot be homed
 *
 *	At the start of a homing cycle those switches configured for homing 
 *	(or for homing and limits) are treated as homing switches (they are modal).
 *
 *	After initialization the following sequence is run for each axis to be homed:
 *
 *	  0. If a homing or limit switch is closed on invocation, clear off the switch
 *	  1. Drive towards the homing switch at search velocity until switch is hit
 *	  2. Drive away from the homing switch at latch velocity until switch opens
 *	  3. Back off switch by the zero backoff distance and set zero for that axis
 *
 *	Homing works as a state machine that is driven by registering a callback 
 *	function at hm.func() for the next state to be run. Once the axis is 
 *	initialized each callback basically does two things (1) start the move 
 *	for the current function, and (2) register the next state with hm.func(). 
 *	When a move is started it will either be interrupted if the homing switch 
 *	changes state, This will cause the move to stop with a feedhold. The other 
 *	thing that can happen is the move will run to its full length if no switch 
 *	change is detected (hit or open),
 *
 *	Once all moves for an axis are complete the next axis in the sequence is homed
 *
 *	When a homing cycle is intiated the homing state is set to HOMING_NOT_HOMED
 *	When homing comples successfully this si set to HOMING_HOMED, otherwise it
 *	remains HOMING_NOT_HOMED.	
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
 */

uint8_t cm_probe_cycle_start(void)
{
	// save relevant non-axis parameters from Gcode model
//	pb.saved_units_mode = cm.gm.units_mode;
//	pb.saved_coord_system = cm.gm.coord_system;
//	pb.saved_distance_mode = cm.gm.distance_mode;
//	pb.saved_feed_rate = cm.gm.feed_rate;
	
	pb.saved_units_mode = cm_get_units_mode(ACTIVE_MODEL);			//cm.gm.units_mode;
	pb.saved_coord_system = cm_get_coord_system(ACTIVE_MODEL);		//cm.gm.coord_system;
	pb.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);	//cm.gm.distance_mode;
	pb.saved_feed_rate = cm_get_distance_mode(ACTIVE_MODEL);		//cm.gm.feed_rate;

	pb.open_position = 0;
	
	// set working values
	cm_set_units_mode(MILLIMETERS);
	cm_set_distance_mode(INCREMENTAL_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);	// homing is done in machine coordinates

	pb.axis = -1;							// set to retrieve initial axis
	pb.func = _probing_axis_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_PROBE;
	st_energize_motors();					// enable motors if not already enabled
	return (STAT_OK);
}

uint8_t cm_probe_callback(void)
{
	if (cm.cycle_state != CYCLE_PROBE) { return (STAT_NOOP);}	// exit if not in a homing cycle
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN);}	// sync to planner move ends
	return (pb.func(pb.axis));									// execute the current homing move
}

int8_t cm_probe_get_axis(void)
{
	return pb.axis;
}

void cm_probe_set_position(float p)
{
	pb.open_position = p;
}


static stat_t _probing_finalize_exit(int8_t axis)	// third part of return to home
{
	mp_flush_planner(); 						// should be stopped, but in case of switch closure
	cm_set_coord_system(pb.saved_coord_system);	// restore to work coordinate system
	cm_set_units_mode(pb.saved_units_mode);
	cm_set_distance_mode(pb.saved_distance_mode);
	cm_set_feed_rate(pb.saved_feed_rate);
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
//	cm.cycle_state = CYCLE_MACHINING;
	cm.cycle_state = CYCLE_OFF;
	cm_cycle_end();
	return (STAT_OK);
}


/* 
 * _probing_error_exit()
 */

static stat_t _probing_error_exit(int8_t axis)
{
	// Generate the warning message. Since the error exit returns via the homing callback 
	// - and not the main controller - it requires its own display processing 
	cmd_reset_list();
	if (axis == -2) {
		cmd_conditional_message((const char_t *)"*** WARNING *** Probing error: Specified axis(es) cannot use probe");
	} else {
		char message[CMD_MESSAGE_LEN];
		sprintf_P(message, PSTR("*** WARNING *** Probing error: %c axis settings misconfigured"), cm_get_axis_char(axis));
		cmd_conditional_message((const char_t *)message);
	}
	cmd_print_list(STAT_PROBING_CYCLE_FAILED, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);

	// clean up and exit
	mp_flush_planner(); 						// should be stopped, but in case of switch closure
												// don't use cm_request_queue_flush() here

	cm_set_coord_system(pb.saved_coord_system);	// restore to work coordinate system
	cm_set_units_mode(pb.saved_units_mode);
	cm_set_distance_mode(pb.saved_distance_mode);
	cm_set_feed_rate(pb.saved_feed_rate);
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
	cm.cycle_state = CYCLE_OFF;
	cm_cycle_end();
	return (STAT_PROBING_CYCLE_FAILED);
}

/* Homing axis moves - these execute in sequence for each axis
 *	_probing_axis_start()		- get next axis, initialize variables, call the clear
 *	_probing_axis_clear()		- initiate a clear to move off a switch that is thrown at the start
 *	_probing_axis_backoff_home()	- back off the cleared home switch
 *	_probing_axis_backoff_limit()- back off the cleared limit switch
 *	_probing_axis_search()		- fast search for switch, closes switch
 *	_probing_axis_latch()		- slow reverse until switch opens again
 *	_probing_axis_final()		- backoff from latch location to zero position 
 *	_probing_axis_move()			- helper that actually executes the above moves
 */

static stat_t _probing_axis_start(int8_t axis)
{
	// get the first or next axis
	if ((axis = _get_next_axis(axis)) < 0) { 				// axes are done or error
		if (axis == -1) {									// -1 is done
			return (_set_pb_func(_probing_finalize_exit));
		} else if (axis == -2) { 							// -2 is error
			cm_set_units_mode(pb.saved_units_mode);
			cm_set_distance_mode(pb.saved_distance_mode);
			cm.cycle_state = CYCLE_OFF;
			cm_cycle_end();
			return (_probing_error_exit(-2));
		}
	}
	// trap gross mis-configurations
	if ((cm.a[axis].search_velocity == 0) || (cm.a[axis].latch_velocity == 0)) {
		return (_probing_error_exit(axis));
	}
	if ((cm.a[axis].travel_max <= 0) || (cm.a[axis].latch_backoff <= 0)) {
		return (_probing_error_exit(axis));
	}

	// determine the switch setup and that config is OK
	pb.min_mode = get_switch_mode(MIN_SWITCH(axis));
	pb.max_mode = get_switch_mode(MAX_SWITCH(axis));

	if ( ((pb.min_mode & SW_HOMING_BIT) ^ (pb.max_mode & SW_HOMING_BIT)) == 0) {// one or the other must be homing
		return (_probing_error_exit(axis));					// axis cannot be homed
	}
	pb.axis = axis;											// persist the axis
	pb.search_velocity = fabs(cm.a[axis].search_velocity);	// search velocity is always positive
	pb.latch_velocity = fabs(cm.a[axis].latch_velocity);	// latch velocity is always positive

	// setup parameters homing to the minimum switch
	if (pb.min_mode & SW_HOMING_BIT) {
		pb.homing_switch = MIN_SWITCH(axis);				// the min is the homing switch
		pb.limit_switch = MAX_SWITCH(axis);					// the max would be the limit switch
		pb.search_travel = -cm.a[axis].travel_max;			// search travels in negative direction
		pb.latch_backoff = cm.a[axis].latch_backoff;		// latch travels in positive direction
		pb.zero_backoff = cm.a[axis].zero_backoff;

	// setup parameters for positive travel (homing to the maximum switch)
	} else {
		pb.homing_switch = MAX_SWITCH(axis);				// the max is the homing switch
		pb.limit_switch = MIN_SWITCH(axis);					// the min would be the limit switch
		pb.search_travel = cm.a[axis].travel_max;			// search travels in positive direction
		pb.latch_backoff = -cm.a[axis].latch_backoff;		// latch travels in negative direction
		pb.zero_backoff = -cm.a[axis].zero_backoff;
	}
    // if homing is disabled for the axis then skip to the next axis
	uint8_t sw_mode = get_switch_mode(pb.homing_switch);
	if ((sw_mode != SW_MODE_HOMING) && (sw_mode != SW_MODE_HOMING_LIMIT)) {
		return (_set_pb_func(_probing_axis_start));
	}
	// disable the limit switch parameter if there is no limit switch
	if (get_switch_mode(pb.limit_switch) == SW_MODE_DISABLED) {
		pb.limit_switch = -1;
	}
	pb.saved_jerk = cm.a[axis].jerk_max;					// save the max jerk value
	return (_set_pb_func(_probing_axis_clear));				// start the clear
}

// Handle an initial switch closure by backing off switches
// NOTE: Relies on independent switches per axis (not shared)
static stat_t _probing_axis_clear(int8_t axis)				// first clear move
{
	int8_t homing = read_switch(pb.homing_switch);
	int8_t limit = read_switch(pb.limit_switch);

	if ((homing == SW_OPEN) && (limit != SW_CLOSED)) {
 		return (_set_pb_func(_probing_axis_search));		// OK to start the search
	}
	if (homing == SW_CLOSED) {
		_probing_axis_move(axis, pb.latch_backoff, pb.search_velocity);
 		return (_set_pb_func(_probing_axis_backoff_home));	// will backoff homing switch some more
	}
	_probing_axis_move(axis, -pb.latch_backoff, pb.search_velocity);
 	return (_set_pb_func(_probing_axis_backoff_limit));		// will backoff limit switch some more
}

static stat_t _probing_axis_backoff_home(int8_t axis)		// back off cleared homing switch
{
	_probing_axis_move(axis, pb.latch_backoff, pb.search_velocity);
    return (_set_pb_func(_probing_axis_search));
}

static stat_t _probing_axis_backoff_limit(int8_t axis)		// back off cleared limit switch
{
	_probing_axis_move(axis, -pb.latch_backoff, pb.search_velocity);
    return (_set_pb_func(_probing_axis_search));
}

static stat_t _probing_axis_search(int8_t axis)				// start the search
{
	cm.a[axis].jerk_max = cm.a[axis].jerk_homing;	// use the homing jerk for search onward
	_probing_axis_move(axis, pb.search_travel, pb.search_velocity);
    return (_set_pb_func(_probing_axis_latch));
}

static stat_t _probing_axis_latch(int8_t axis)				// latch to switch open
{
	_probing_axis_move(axis, pb.latch_backoff, pb.latch_velocity);    
	return (_set_pb_func(_probing_axis_zero_backoff)); 
}

static stat_t _probing_axis_zero_backoff(int8_t axis)		// backoff to zero position
{
	_probing_axis_move(axis, pb.zero_backoff, pb.search_velocity);
	return (_set_pb_func(_probing_axis_set_zero));
}

static stat_t _probing_axis_set_zero(int8_t axis)			// set zero and finish up
{
	cm.a[axis].jerk_max = pb.saved_jerk;					// restore the max jerk value
	//cm.homed[axis] = true;
	return (_set_pb_func(_probing_axis_start));
}

static stat_t _probing_axis_move(int8_t axis, float target, float velocity)
{
	float flags[] = {1,1,1,1,1,1};
	set_vector_by_axis(target, axis);
	cm_set_feed_rate(velocity);
	cm_request_queue_flush();
	cm_request_cycle_start();
	ritorno(cm_straight_feed(vector, flags));
	return (STAT_EAGAIN);
}

/* _run_homing_dual_axis() - kernal routine for running homing on a dual axis */
//static stat_t _run_homing_dual_axis(int8_t axis) { return (STAT_OK);}

/**** HELPERS ****************************************************************/
/*
 * _set_hm_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_pb_func(uint8_t (*func)(int8_t axis))
{
	pb.func = func;
	return (STAT_EAGAIN);
}

/*
 * _get_next_axis() - return next axis in sequence based on axis in arg
 *
 *	Accepts "axis" arg as the current axis; or -1 to retrieve the first axis
 *	Returns next axis based on "axis" argument and if that axis is flagged for homing in the gf struct
 *	Returns -1 when all axes have been processed
 *	Returns -2 if no axes are specified (Gcode calling error)
 *	Homes Z first, then the rest in sequence
 *
 *	Isolating this function facilitates implementing more complex and 
 *	user-specified axis homing orders
 */

int8_t _get_next_axis(int8_t axis)
{
	if (axis == -1) {	// Only one axis allowed for probing.  The order below is the priority 
		if (cm.gf.target[AXIS_Z] == true) return (AXIS_Z);
		if (cm.gf.target[AXIS_X] == true) return (AXIS_X);
		if (cm.gf.target[AXIS_Y] == true) return (AXIS_Y);
		if (cm.gf.target[AXIS_A] == true) return (AXIS_A);
		return (-2);	// error
	}
	return (-1);	// done
}

