/*
 * cycle_homing - homing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S Hart, Jr.
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "report.h"
#include "gpio.h"

/**** Homing singleton structure ****/

struct hmHomingSingleton {		// persistent homing runtime variables
	// controls for homing cycle
	int8_t axis;				// axis currently being homed
//	int8_t axis2;				// second axis if dual axis
	int8_t homing_switch;		// homing switch for current axis (index into switch flag table)
	int8_t limit_switch;		// limit switch for current axis, or -1 if none
	uint8_t (*func)(int8_t axis);// binding for current processing function

	// per-axis parameters
	double direction;			// set to 1 for positive (max), -1 for negative (to min);
	double search_travel;
	double search_velocity;
	double latch_velocity;
	double latch_backoff;
	double zero_backoff;

	// state saved from gcode model
	double saved_feed_rate;		// F setting
	uint8_t saved_units_mode;	// G20,G21 global setting
	uint8_t saved_coord_system;	// G54 - G59 setting
	uint8_t saved_distance_mode;// G90,G91 global setting
//	uint8_t saved_path_control;	// G61,G61.1,G64 global setting
//	double saved_jerk;			// saved and restored for each axis homed
};
static struct hmHomingSingleton hm;

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static uint8_t _homing_axis_start(int8_t axis);
static uint8_t _homing_axis_clear(int8_t axis);
static uint8_t _homing_axis_search(int8_t axis);
static uint8_t _homing_axis_latch_backoff(int8_t axis);
static uint8_t _homing_axis_latch(int8_t axis);
static uint8_t _homing_axis_zero_backoff(int8_t axis);
static uint8_t _homing_axis_set_zero(int8_t axis);
static uint8_t _homing_axis_move(int8_t axis, double target, double velocity);
static uint8_t _homing_finalize(int8_t axis);

static uint8_t _set_hm_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);
//static int8_t _get_next_axes(int8_t axis);

/*****************************************************************************
 * cm_return_to_home() - G28 cycle
 * cm_return_to_home_callback() - main loop callback for the above
 */

uint8_t cm_return_to_home(void)
{
	double zero[] = {0,0,0,0,0,0};
	double flags[] = {1,1,1,1,1,1};
	ritorno(cm_straight_traverse(zero, flags));
	return (TG_OK);
}

/*****************************************************************************
 * cm_homing_cycle_start()	- G28.1 homing cycle using limit switches
 * cm_homing_callback() 	- main loop callback for running the homing cycle
 *
 * Homing works from a G28.1 according to the following writeup: 
 *  https://github.com/synthetos/TinyG/wiki/TinyG-Homing-planned-for-version-0.95
 *
 *	--- How does this work? ---
 *
 *	When a homing cycle is intiated the cycle state is set to CYCLE_HOMING.
 *	At the start of a homing cycle those switches configured for homing 
 *	(or for loming andf limits) are treated as homing switches (they are modal).
 *
 *	After some initialization and backing off any closed switches, a series of 
 *	search and latch moves are run for each axis specified in the G28.1 command.
 *	The cm_homing_callback() function is a dispatcher that vectors to the homing
 *	move currently active. Each move must clear the planner and any previous 
 *	feedhold state before it can be run.
 *
 *	Each move runs until either it is done or a switch is hit. The switch interrupt 
 *	causes a feedhold to be executed and the hold state to become HOLD. This then 
 *	causes the machine to become "not busy" so cm_isbusy() returns false, allowing 
 *	the next move to be run. 
 *
 *	The act of finishing the last axis resets the machine to machine zero. 
 *
 *	--- Some further details ---
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued 
 *	move per entry into the continuation, then you must exit. 
 *
 *	Another Note: When coding a cycle (like this one) you must wait until 
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done. Otherwise there is a nasty race condition in the 
 *	tg_controller() that will accept the next command before the position of 
 *	the final move has been recorded in the Gcode model.
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

	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _homing_axis_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_HOMING;
	cm.homing_state = HOMING_NOT_HOMED;
	return (TG_OK);
}

uint8_t cm_homing_callback(void)
{
	if (cm.cycle_state != CYCLE_HOMING) { return (TG_NOOP);} // exit if not in a homing cycle
	if (cm_isbusy() == true) { return (TG_EAGAIN);}	 // sync to planner move ends
	return (hm.func(hm.axis));				// execute the current homing move
}

static uint8_t _homing_finalize(int8_t axis)	// third part of return to home
{
	mp_flush_planner(); 						// should be stopped, but in case of switch closure
	cm_set_coord_system(hm.saved_coord_system);	// restore to work coordinate system
	cm_set_units_mode(hm.saved_units_mode);
	cm_set_distance_mode(hm.saved_distance_mode);
	cm_set_feed_rate(hm.saved_feed_rate);
	cm.homing_state = HOMING_HOMED;
	cm.cycle_state = CYCLE_STARTED;
	cm_exec_cycle_end();
	return (TG_OK);
}

/* Homing axis moves - these execute in sequence for each axis
 *	_homing_axis_start()	- get next axis, initialize variables, call the clear
 *	_homing_axis_clear()	- clear off any switches that are thrown at the start
 *	_homing_axis_search()	- initial search for switch
 *	_homing_axis_backoff()	- backoff when switch is hit
 *	_homing_axis_latch()	- slow search for switch
 *	_homing_axis_final()	- backoff from latch 
 *	_homing_axis_move()		- helper that actually executes the above moves
 */

static uint8_t _homing_axis_start(int8_t axis)
{
	// get first or next axis
	if ((axis = _get_next_axis(axis)) < 0) { 		// axes are done or error
		if (axis == -1) {							// -1 is done
			return (_set_hm_func(_homing_finalize));
		} else if (axis == -2) { 					// -2 is error
			cm_set_units_mode(hm.saved_units_mode);
			cm_set_distance_mode(hm.saved_distance_mode);
			cm.cycle_state = CYCLE_STARTED;
			cm_exec_cycle_end();
			return (TG_HOMING_CYCLE_FAILED);		// homing state remains HOMING_NOT_HOMED
		}
	}
	if ((cfg.a[axis].search_velocity == 0) || (cfg.a[axis].travel_max == 0)) {
		return (TG_GCODE_INPUT_ERROR);				// requested axis can't be homed
	}

	hm.axis = axis;											// persist the axis
//	hm.saved_jerk = cfg.a[axis].jerk_max;					// per-axis save
	hm.search_velocity = fabs(cfg.a[axis].search_velocity); // search velocity is always positive
	hm.latch_velocity = fabs(cfg.a[axis].latch_velocity); 	// ...and so is latch velocity

	// setup parameters for negative travel (homing to the minimum switch)
	if (cfg.a[axis].search_velocity < 0) {
		hm.homing_switch = MIN_SWITCH(axis);				// the min is the homing switch
		hm.limit_switch = MAX_SWITCH(axis);					// the max would be the limit switch
		hm.search_travel = -cfg.a[axis].travel_max;			// search travels in negative direction
		hm.latch_backoff = cfg.a[axis].latch_backoff;		// backoffs travel in positive direction
		hm.zero_backoff = cfg.a[axis].zero_backoff;

	// setup parameters for positive travel (homing to the maximum switch)
	} else {
		hm.homing_switch = MAX_SWITCH(axis);				// the max is the homing switch
		hm.limit_switch = MIN_SWITCH(axis);					// the min would be the limit switch
		hm.search_travel = cfg.a[axis].travel_max;			// search travels in positive direction
		hm.latch_backoff = -cfg.a[axis].latch_backoff;		// backoffs travel in negative direction
		hm.zero_backoff = -cfg.a[axis].zero_backoff;
	}
    // if homing is disabled for an axis just set the axis position to zero
	uint8_t sw_mode = gpio_get_switch_mode(hm.homing_switch);
	if ((sw_mode != SW_MODE_HOMING) && (sw_mode != SW_MODE_HOMING_LIMIT)) {
		return (_set_hm_func(_homing_axis_set_zero));
	}
	// disable the limit switch parameter if there is no limit switch
	if (gpio_get_switch_mode(hm.limit_switch) != SW_MODE_LIMIT) {
		hm.limit_switch = -1;
	}
	return (_set_hm_func(_homing_axis_clear));
}

static uint8_t _homing_axis_clear(int8_t axis)
{
	// Handle an initial switch closure by backing off switches
	// NOTE: Relies on independent switches per axis (not shared)

	if (gpio_read_switch(hm.homing_switch) == true) {	// test if homing switch is thrown
 	   	_homing_axis_move(axis, hm.latch_backoff, hm.search_velocity); // latch backoff at search velocity
		return (_set_hm_func(_homing_axis_clear));		// repeat until no longer on switch
	}
	if (hm.limit_switch != -1) {						// -1 means thre is no limit configured
		if (gpio_read_switch(hm.limit_switch) == true) {// test if limit switch is thrown
	 	   	_homing_axis_move(axis, -hm.latch_backoff, hm.search_velocity);
			return (_set_hm_func(_homing_axis_clear));	// repeat until no longer on switch
		}
	}
	return (_set_hm_func(_homing_axis_search));			// we are clear. Ready to start the search
}

static uint8_t _homing_axis_search(int8_t axis)
{
	_homing_axis_move(axis, hm.search_travel, hm.search_velocity);
    return (_set_hm_func(_homing_axis_latch_backoff));
}

static uint8_t _homing_axis_latch_backoff(int8_t axis)
{
	_homing_axis_move(axis, hm.latch_backoff, hm.search_velocity);
	return (_set_hm_func(_homing_axis_latch));
}

static uint8_t _homing_axis_latch(int8_t axis)
{
	_homing_axis_move(axis, -2*hm.latch_backoff, hm.latch_velocity);    
	return (_set_hm_func(_homing_axis_zero_backoff)); 
}

static uint8_t _homing_axis_zero_backoff(int8_t axis)
{
	_homing_axis_move(axis, hm.zero_backoff, hm.search_velocity);
	return (_set_hm_func(_homing_axis_set_zero));
}

static uint8_t _homing_axis_set_zero(int8_t axis)
{
	cm_set_machine_axis_position(axis, 0);
	return (_set_hm_func(_homing_axis_start));
}

static uint8_t _homing_axis_move(int8_t axis, double target, double velocity)
{
	double flags[] = {1,1,1,1,1,1};
	set_vector_by_axis(target, axis);
	cm_set_feed_rate(velocity);
	mp_flush_planner();
	ritorno(cm_straight_feed(vector, flags));
	return (TG_EAGAIN);
}

/* _run_homing_dual_axis() - kernal routine for running homing on a dual axis */
//static uint8_t _run_homing_dual_axis(int8_t axis) { return (TG_OK);}

/**** HELPERS ****************************************************************/
/*
 * _set_hm_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_hm_func(uint8_t (*func)(int8_t axis))
{
	hm.func = func;
	return (TG_EAGAIN);
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
		if (gf.target[Z] == true) return (Z);
		if (gf.target[X] == true) return (X);
		if (gf.target[Y] == true) return (Y);
		if (gf.target[A] == true) return (A);
		if (gf.target[B] == true) return (B);
		if (gf.target[C] == true) return (C);
		return (-2);	// error
	} else if (axis == Z) {
		if (gf.target[X] == true) return (X);
		if (gf.target[Y] == true) return (Y);
		if (gf.target[A] == true) return (A);
		if (gf.target[B] == true) return (B);
		if (gf.target[C] == true) return (C);
	} else if (axis == X) {
		if (gf.target[Y] == true) return (Y);
		if (gf.target[A] == true) return (A);
		if (gf.target[B] == true) return (B);
		if (gf.target[C] == true) return (C);
	} else if (axis == Y) {
		if (gf.target[A] == true) return (A);
		if (gf.target[B] == true) return (B);
		if (gf.target[C] == true) return (C);
	} else if (axis == A) {
		if (gf.target[B] == true) return (B);
		if (gf.target[C] == true) return (C);
	} else if (axis == B) {
		if (gf.target[C] == true) return (C);
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
	return (TG_OK);
}
*/
