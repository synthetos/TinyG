/*
 * cycle_homing - homing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "gpio.h"

/**** Homing singleton structure ****/

struct hmHomingSingleton {		// persistent homing runtime variables
	// controls for homing cycle
	int8_t axis;				// axis currently being homed
//	int8_t axis2;				// second axis if dual axis
	uint8_t min_mode;			// mode for min switch fo this axis
	uint8_t max_mode;			// mode for max switch fo this axis
	int8_t homing_switch;		// homing switch for current axis (index into switch flag table)
	int8_t limit_switch;		// limit switch for current axis, or -1 if none
	uint8_t homing_closed;		// 0=open, 1=closed
	uint8_t limit_closed;		// 0=open, 1=closed
	uint8_t (*func)(int8_t axis);// binding for callback function state machine
	uint8_t set_coordinates;	// G28.4 flag. true = set coords to zero at the end of homing cycle

	// per-axis parameters
	float direction;			// set to 1 for positive (max), -1 for negative (to min);
	float search_travel;		// signed distance to travel in search
	float search_velocity;		// search speed as positive number
	float latch_velocity;		// latch speed as positive number
	float latch_backoff;		// max distance to back off switch during latch phase 
	float zero_backoff;		// distance to back off switch before setting zero
	float max_clear_backoff;	// maximum distance of switch clearing backoffs before erring out

	// state saved from gcode model
	float saved_feed_rate;		// F setting
	uint8_t saved_units_mode;	// G20,G21 global setting
	uint8_t saved_coord_system;	// G54 - G59 setting
	uint8_t saved_distance_mode;// G90,G91 global setting
	float saved_jerk;			// saved and restored for each axis homed
};
static struct hmHomingSingleton hm;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _homing_axis_start(int8_t axis);
static stat_t _homing_axis_clear(int8_t axis);
static stat_t _homing_axis_backoff_home(int8_t axis);
static stat_t _homing_axis_backoff_limit(int8_t axis);
static stat_t _homing_axis_search(int8_t axis);
static stat_t _homing_axis_latch(int8_t axis);
static stat_t _homing_axis_zero_backoff(int8_t axis);
static stat_t _homing_axis_set_zero(int8_t axis);
static stat_t _homing_axis_move(int8_t axis, float target, float velocity);
static stat_t _homing_finalize_exit(int8_t axis);
static stat_t _homing_error_exit(int8_t axis);

static stat_t _set_homing_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);
//static int8_t _get_next_axes(int8_t axis);


/*****************************************************************************
 * cm_homing_cycle_start()	- G28.1 homing cycle using limit switches
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

uint8_t cm_homing_cycle_start(void)
{
	// save relevant non-axis parameters from Gcode model
	hm.saved_units_mode = gm.units_mode;
	hm.saved_coord_system = gm.coord_system;
	hm.saved_distance_mode = gm.distance_mode;
	hm.saved_feed_rate = gm.feed_rate;

	// set working values
	cm_set_units_mode(MILLIMETERS);
	cm_set_distance_mode(INCREMENTAL_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);	// homing is done in machine coordinates
	hm.set_coordinates = true;

	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _homing_axis_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_HOMING;
	cm.homing_state = HOMING_NOT_HOMED;
	st_enable_motors();						// enable motors if not already enabled
	return (STAT_OK);
}

uint8_t cm_homing_cycle_start_no_set(void)
{
	// save relevant non-axis parameters from Gcode model
	hm.saved_units_mode = gm.units_mode;
	hm.saved_coord_system = gm.coord_system;
	hm.saved_distance_mode = gm.distance_mode;
	hm.saved_feed_rate = gm.feed_rate;

	// set working values
	cm_set_units_mode(MILLIMETERS);
	cm_set_distance_mode(INCREMENTAL_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);	// homing is done in machine coordinates
	hm.set_coordinates = false;				// set flag to not update position variables at the end of the cycle

	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _homing_axis_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_HOMING;
	cm.homing_state = HOMING_NOT_HOMED;	    // this cycle should not change the homing state but at this point I think the homing state is the only way to see if the cycle is complete.
	st_enable_motors();						// enable motors if not already enabled
	return (STAT_OK);
}

static stat_t _homing_finalize_exit(int8_t axis)	// third part of return to home
{
	mp_flush_planner(); 							// should be stopped, but in case of switch closure
	cm_set_coord_system(hm.saved_coord_system);		// restore to work coordinate system
	cm_set_units_mode(hm.saved_units_mode);
	cm_set_distance_mode(hm.saved_distance_mode);
	cm_set_feed_rate(hm.saved_feed_rate);
	cm_set_motion_mode(MOTION_MODE_CANCEL_MOTION_MODE);
	cm.homing_state = HOMING_HOMED;
	cm.cycle_state = CYCLE_OFF;						// required
	cm_cycle_end();
//+++++ DIAGNOSTIC +++++
//	printf("Homed: posX: %6.3f, posY: %6.3f\n", (double)gm.position[AXIS_X], (double)gm.target[AXIS_Y]);
	return (STAT_OK);
}

/* _homing_error_exit()
 *
 * Since the error exit returns via the homing callback - and not the main controller - 
 * it requires its own diplay processing 
 */
static const char msg_axis0[] PROGMEM = "X";
static const char msg_axis1[] PROGMEM = "Y";
static const char msg_axis2[] PROGMEM = "Z";
static const char msg_axis3[] PROGMEM = "A";
static PGM_P const msg_axis[] PROGMEM = { msg_axis0, msg_axis1, msg_axis2, msg_axis3};

static stat_t _homing_error_exit(int8_t axis)
{
	cmd_reset_list();
	char message[CMD_MESSAGE_LEN]; 
	if (axis == -2) {
		sprintf_P(message, PSTR("*** WARNING *** Homing error: Specified axis(es) cannot be homed"));
	} else {
		sprintf_P(message, PSTR("*** WARNING *** Homing error: %S axis settings misconfigured"), (PGM_P)pgm_read_word(&msg_axis[axis]));
	}
//	cmd_add_string("msg",message);
	cmd_add_message(message);
	cmd_print_list(STAT_HOMING_CYCLE_FAILED, TEXT_INLINE_PAIRS, JSON_RESPONSE_FORMAT);

	mp_flush_planner(); 						// should be stopped, but in case of switch closure
	cm_set_coord_system(hm.saved_coord_system);	// restore to work coordinate system
	cm_set_units_mode(hm.saved_units_mode);
	cm_set_distance_mode(hm.saved_distance_mode);
	cm_set_feed_rate(hm.saved_feed_rate);
	cm_set_motion_mode(MOTION_MODE_CANCEL_MOTION_MODE);
	cm.cycle_state = CYCLE_OFF;
	cm_cycle_end();
	return (STAT_HOMING_CYCLE_FAILED);			// homing state remains HOMING_NOT_HOMED
}

/* Homing axis moves - these execute in sequence for each axis
 * cm_homing_callback() 		- main loop callback for running the homing cycle
 *	_homing_axis_start()		- get next axis, initialize variables, call the clear
 *	_homing_axis_clear()		- initiate a clear to move off a switch that is thrown at the start
 *	_homing_axis_backoff_home()	- back off the cleared home switch
 *	_homing_axis_backoff_limit()- back off the cleared limit switch
 *	_homing_axis_search()		- fast search for switch, closes switch
 *	_homing_axis_latch()		- slow reverse until switch opens again
 *	_homing_axis_final()		- backoff from latch location to zero position 
 *	_homing_axis_move()			- helper that actually executes the above moves
 */

uint8_t cm_homing_callback(void)
{
	if (cm.cycle_state != CYCLE_HOMING) { return (STAT_NOOP);} 	// exit if not in a homing cycle
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN);}	// sync to planner move ends
	return (hm.func(hm.axis));						// execute the current homing move
}

static stat_t _homing_axis_start(int8_t axis)
{
	// get the first or next axis
	if ((axis = _get_next_axis(axis)) < 0) { 				// axes are done or error
		if (axis == -1) {									// -1 is done
			return (_set_homing_func(_homing_finalize_exit));
		} else if (axis == -2) { 							// -2 is error
			cm_set_units_mode(hm.saved_units_mode);
			cm_set_distance_mode(hm.saved_distance_mode);
			cm.cycle_state = CYCLE_OFF;
			cm_cycle_end();
			return (_homing_error_exit(-2));
		}
	}
	// trap gross mis-configurations
	if ((cfg.a[axis].search_velocity == 0) || (cfg.a[axis].latch_velocity == 0)) {
		return (_homing_error_exit(axis));
	}
	if ((cfg.a[axis].travel_max <= 0) || (cfg.a[axis].latch_backoff <= 0)) {
		return (_homing_error_exit(axis));
	}

	// determine the switch setup and that config is OK
	hm.min_mode = gpio_get_switch_mode(MIN_SWITCH(axis));
	hm.max_mode = gpio_get_switch_mode(MAX_SWITCH(axis));

	if ( ((hm.min_mode & SW_HOMING) ^ (hm.max_mode & SW_HOMING)) == 0) {// one or the other must be homing
		return (_homing_error_exit(axis));					// axis cannot be homed
	}
	hm.axis = axis;											// persist the axis
	hm.search_velocity = fabs(cfg.a[axis].search_velocity); // search velocity is always positive
	hm.latch_velocity = fabs(cfg.a[axis].latch_velocity); 	// latch velocity is always positive

	// setup parameters homing to the minimum switch
	if (hm.min_mode & SW_HOMING) {
		hm.homing_switch = MIN_SWITCH(axis);				// the min is the homing switch
		hm.limit_switch = MAX_SWITCH(axis);					// the max would be the limit switch
		hm.search_travel = -cfg.a[axis].travel_max;			// search travels in negative direction
		hm.latch_backoff = cfg.a[axis].latch_backoff;		// latch travels in positive direction
		hm.zero_backoff = cfg.a[axis].zero_backoff;

	// setup parameters for positive travel (homing to the maximum switch)
	} else {
		hm.homing_switch = MAX_SWITCH(axis);				// the max is the homing switch
		hm.limit_switch = MIN_SWITCH(axis);					// the min would be the limit switch
		hm.search_travel = cfg.a[axis].travel_max;			// search travels in positive direction
		hm.latch_backoff = -cfg.a[axis].latch_backoff;		// latch travels in negative direction
		hm.zero_backoff = -cfg.a[axis].zero_backoff;
	}
    // if homing is disabled for the axis then skip to the next axis
	uint8_t sw_mode = gpio_get_switch_mode(hm.homing_switch);
	if ((sw_mode != SW_MODE_HOMING) && (sw_mode != SW_MODE_HOMING_LIMIT)) {
		return (_set_homing_func(_homing_axis_start));
	}
	// disable the limit switch parameter if there is no limit switch
	if (gpio_get_switch_mode(hm.limit_switch) == SW_MODE_DISABLED) {
		hm.limit_switch = -1;
	}
	hm.saved_jerk = cfg.a[axis].jerk_max;					// save the max jerk value
	return (_set_homing_func(_homing_axis_clear));			// start the clear
}

// Handle an initial switch closure by backing off switches
// NOTE: Relies on independent switches per axis (not shared)
static stat_t _homing_axis_clear(int8_t axis)				// first clear move
{
	int8_t homing = gpio_read_switch(hm.homing_switch);
	int8_t limit = gpio_read_switch(hm.limit_switch);

	if ((homing == SW_OPEN) && (limit != SW_CLOSED)) {
 		return (_set_homing_func(_homing_axis_search));		// OK to start the search
	}
	if (homing == SW_CLOSED) {
		_homing_axis_move(axis, hm.latch_backoff, hm.search_velocity);
 		return (_set_homing_func(_homing_axis_backoff_home));// will backoff homing switch some more
	}
	_homing_axis_move(axis, -hm.latch_backoff, hm.search_velocity);
 	return (_set_homing_func(_homing_axis_backoff_limit));	// will backoff limit switch some more
}

static stat_t _homing_axis_backoff_home(int8_t axis)		// back off cleared homing switch
{
	_homing_axis_move(axis, hm.latch_backoff, hm.search_velocity);
    return (_set_homing_func(_homing_axis_search));
}

static stat_t _homing_axis_backoff_limit(int8_t axis)		// back off cleared limit switch
{
	_homing_axis_move(axis, -hm.latch_backoff, hm.search_velocity);
    return (_set_homing_func(_homing_axis_search));
}

static stat_t _homing_axis_search(int8_t axis)				// start the search
{
	cfg.a[axis].jerk_max = cfg.a[axis].jerk_homing;			// use the homing jerk for search onward
	_homing_axis_move(axis, hm.search_travel, hm.search_velocity);
    return (_set_homing_func(_homing_axis_latch));
}

static stat_t _homing_axis_latch(int8_t axis)				// latch to switch open
{
	_homing_axis_move(axis, hm.latch_backoff, hm.latch_velocity);    
	return (_set_homing_func(_homing_axis_zero_backoff)); 
}

static stat_t _homing_axis_zero_backoff(int8_t axis)		// backoff to zero position
{
	_homing_axis_move(axis, hm.zero_backoff, hm.search_velocity);
	return (_set_homing_func(_homing_axis_set_zero));
}

static stat_t _homing_axis_set_zero(int8_t axis)			// set zero and finish up
{
	if (hm.set_coordinates != false) {						// do not set axis if in G28.4 cycle
		cm_set_axis_origin(axis, 0);
		mp_set_runtime_position(axis, 0);
	} else {
		cm_set_axis_origin(axis, cm_get_runtime_work_position(AXIS_X));
	}
	cfg.a[axis].jerk_max = hm.saved_jerk;					// restore the max jerk value
	cm.homed[axis] = true;
	return (_set_homing_func(_homing_axis_start));
}

static stat_t _homing_axis_move(int8_t axis, float target, float velocity)
{
	cm_set_feed_rate(velocity);
	mp_flush_planner();
	cm_request_cycle_start();

	float flags[] = {true, true, true, true, true, true};
	set_vector_by_axis(target, axis);
	ritorno(cm_straight_feed(vector, flags));
	return (STAT_EAGAIN);
}

/* _run_homing_dual_axis() - kernal routine for running homing on a dual axis */
//static stat_t _run_homing_dual_axis(int8_t axis) { return (STAT_OK);}

/**** HELPERS ****************************************************************/
/*
 * _set_homing_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_homing_func(uint8_t (*func)(int8_t axis))
{
	hm.func = func;
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
	if (axis == -1) {	// inelegant brute force solution
		if (gf.target[AXIS_Z] == true) return (AXIS_Z);
		if (gf.target[AXIS_X] == true) return (AXIS_X);
		if (gf.target[AXIS_Y] == true) return (AXIS_Y);
		if (gf.target[AXIS_A] == true) return (AXIS_A);
//		if (gf.target[AXIS_B] == true) return (AXIS_B);
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
		return (-2);	// error
	} else if (axis == AXIS_Z) {
		if (gf.target[AXIS_X] == true) return (AXIS_X);
		if (gf.target[AXIS_Y] == true) return (AXIS_Y);
		if (gf.target[AXIS_A] == true) return (AXIS_A);
//		if (gf.target[AXIS_B] == true) return (AXIS_B);
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
	} else if (axis == AXIS_X) {
		if (gf.target[AXIS_Y] == true) return (AXIS_Y);
		if (gf.target[AXIS_A] == true) return (AXIS_A);
//		if (gf.target[AXIS_B] == true) return (AXIS_B);
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
	} else if (axis == AXIS_Y) {
		if (gf.target[AXIS_A] == true) return (AXIS_A);
//		if (gf.target[AXIS_B] == true) return (AXIS_B);
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
//	} else if (axis == AXIS_A) {
//		if (gf.target[AXIS_B] == true) return (AXIS_B);
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
//	} else if (axis == AXIS_B) {
//		if (gf.target[AXIS_C] == true) return (AXIS_C);
	}
	return (-1);	// done
}

/*
 * _get_next_axes() - return next axis in sequence based on axis in arg
 *
 *	Accepts "axis" arg as the current axis; or -1 to retrieve the first axis
 *	Returns next axis based on "axis" argument
 *	Returns -1 when all axes have been processed
 *	Returns -2 if no axes are specified (Gcode calling error)
 *
 *	hm.axis2 is set to the secondary axis if axis is a dual axis
 *	hm.axis2 is set to -1 otherwise
 *
 *	Isolating this function facilitates implementing more complex and 
 *	user-specified axis homing orders
 *
 *	Note: the logic to test for disabled or inhibited axes will allow the 
 *	following condition to occur: A single axis is specified but it is
 *	disabled or inhibited - homing will say that it was successfully homed.
 */
/*
int8_t _get_next_axes(int8_t axis)
{
	int8_t next_axis;
	hm.axis2 = -1;

	// Scan target vector for case where no valid axes are specified
	for (next_axis = 0; next_axis < AXES; next_axis++) {	
		if ((gf.target[next_axis] == true) &&
			(cfg.a[next_axis].axis_mode != AXIS_INHIBITED) &&
			(cfg.a[next_axis].axis_mode != AXIS_DISABLED)) {	
			break;
		}
	}
	if (next_axis == AXES) {
//		fprintf_P(stderr, PSTR("***** Homing failed: none or disabled/inhibited axes specified\n"));
		return (-2);	// didn't find any axes to process
	}

	// Scan target vector from the current axis to find next axis or the end
	for (next_axis = ++axis; next_axis < AXES; next_axis++) {
		if (gf.target[next_axis] == true) { 
			if ((cfg.a[next_axis].axis_mode == AXIS_INHIBITED) || 	
				(cfg.a[next_axis].axis_mode == AXIS_DISABLED)) {	// Skip if axis disabled or inhibited
				continue;
			}
			break;		// got a good one
		}
		return (-1);	// you are done
	}

	// Got a valid axis. Find out if it's a dual
	return (STAT_OK);
}
*/
