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

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/
static uint8_t _homing_axis_start(int8_t axis);
static uint8_t _homing_axis_initial_backoff(int8_t axis);
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

struct hmHomingSingleton {		// persistent G28 and G30 runtime variables
	// controls for homing cycle
	int8_t axis;				// axis currently being homed
	int8_t axis2;				// second axis if dual axis
	uint8_t (*func)(int8_t axis);// binding for current processing function

	// convenience copies of config parameters - somewhat wasteful, but makes coding simpler 
	double search_travel;
	double search_velocity;
	double latch_velocity;
	double latch_backoff;
	double zero_backoff;

	// saved state from gcode model
	double feed_rate_saved;		// F setting
	uint8_t units_mode_saved;	// G20,G21 global setting
	uint8_t coord_system_saved;	// G54 - G59 setting
	uint8_t distance_mode_saved;// G90,G91 global setting
//	uint8_t path_control_saved;	// G61,G61.1,G64 global setting
//	double jerk_saved;			// saved and restored for each axis homed
};
static struct hmHomingSingleton hm;

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
 * cm_homing_cycle() 	- G28.1 homing cycle using limit switches
 * cm_homing_callback() - main loop callback for running the homing cycle
 *
 * Homing works from a G28.1 according to the following writeup: 
 * 	http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing
 *
 *	--- How does this work? ---
 *
 *	When a homing cycle is intiated the cycle state is set to CYCLE_HOMING.
 *	At the start of a homing cycle the limit switches in gpio.c are treated 
 *	as homing switches (they become modal).
 *
 *	After some initialization and backing off any closed switches a series of 
 *	search and latch moves are run for each axis specified in the G28.1 command.
 *	The cm_homing_callback() function is a dispatcher that vectors to the homing 
 *	currently running homing move. Each move must clear the planner and any 
 *	previous hold state before it can be run.
 *
 *	Each move runs until either it is done or a switch is hit. The action of
 *	hitting a switch causes a feedhold to be executed and the hold state to 
 *	become HOLD. This then causes the machine to become "not busy" so cm_isbusy() 
 *	in the callback returns false, allowing the next move to be run. 
 *
 *	NOTE: Currently any switch will stop motion. They will need to be isolated 
 *	in order to support dual-gantry homing.
 *
 *	The act of finishing the per-axis homing resets the machine to machine zero. 
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

uint8_t cm_homing_cycle(void)
{
	// save relevant parameters from Gcode model
	hm.units_mode_saved = gm.units_mode;
	hm.coord_system_saved = gm.coord_system;
	hm.distance_mode_saved = gm.distance_mode;
	hm.feed_rate_saved = gm.feed_rate;

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
	if (cm_isbusy() == true) { return (TG_EAGAIN);}			 // sync to planner move ends
	return (hm.func(hm.axis));				// execute the current homing move
}

/* Homing axis moves - these execute in sequence:
 *	_homing_axis_start()	- get next axis, initialize variables, start search
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
			cm_set_units_mode(hm.units_mode_saved);
			cm_set_distance_mode(hm.distance_mode_saved);
			cm.cycle_state = CYCLE_STARTED;
			cm_exec_cycle_end();
			return (TG_HOMING_CYCLE_FAILED);
		}
	}
	if ((cfg.a[axis].search_velocity == 0) || (cfg.a[axis].travel_max == 0)) {
		return (TG_GCODE_INPUT_ERROR);				// requested axis can't be homed
	}

	hm.axis = axis;
	hm.search_velocity = fabs(cfg.a[axis].search_velocity); // search velocity is always positive
	hm.latch_velocity = fabs(cfg.a[axis].latch_velocity); 	// and so is latch velocity
//	hm.jerk_saved = cfg.a[axis].jerk_max;					// per-axis save

	// if moving to a MIN switch...
    int axis_switch = axis;
	if (cfg.a[axis].search_velocity < 0) {					// search velocity is negative
		hm.search_travel = -cfg.a[axis].travel_max;			// make search travel negative
		hm.latch_backoff = cfg.a[axis].latch_backoff;		// backoffs move opposite of search
		hm.zero_backoff = cfg.a[axis].zero_backoff;
	} else { // if moving to a MAX switch...				// search velocity is positive
		hm.search_travel = cfg.a[axis].travel_max;			// make search travel positive
		hm.latch_backoff = -cfg.a[axis].latch_backoff;		// backoffs move opposite of search
		hm.zero_backoff = -cfg.a[axis].zero_backoff;
        axis_switch = axis + SW_OFFSET;                     // check the max switch for backoffs
	}

    // if homing is disabled for an axis (switch mode == 0) just set the axis position to zero
    if( cfg.a[axis].switch_mode == SW_MODE_DISABLED )
        return (_set_hm_func(_homing_axis_set_zero));

	// ---> For now all axes are single - no dual axis detection or invocation
	// This is where you have to detect and handle dual axes.

	// Handle an initial switch closure by backing off the switch
	// (NOTE: this gets more complicated if switch pins are shared)
    gpio_clear_switches();
    sw.lockout_count = 0;
    
	gpio_read_switches();				// sets gp.sw_flags
	if (gpio_get_switch(axis_switch) == true) // test if the MIN/MAX switch for the axis is thrown
		return (_set_hm_func(_homing_axis_initial_backoff));
    else    
        return (_set_hm_func(_homing_axis_search));
}

static uint8_t _homing_axis_initial_backoff(int8_t axis)
{
    _homing_axis_move(axis, hm.latch_backoff, hm.latch_velocity);
    return (_set_hm_func(_homing_axis_search));
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

static uint8_t _homing_finalize(int8_t axis)	// third part of return to home
{
	mp_flush_planner(); 						// should be stopped, but in case of switch closure
	cm_set_coord_system(hm.coord_system_saved);	// restore to work coordinate system
	cm_set_units_mode(hm.units_mode_saved);
	cm_set_distance_mode(hm.distance_mode_saved);
	cm_set_feed_rate(hm.feed_rate_saved);
	cm.homing_state = HOMING_HOMED;
	cm.cycle_state = CYCLE_STARTED;
	cm_exec_cycle_end();
	return (TG_OK);
}

/*
 * _run_homing_dual_axis() - kernal routine for running homing on a dual axis
 *
 */
/*
static uint8_t _run_homing_dual_axis(int8_t axis)
{
	return (TG_OK);
}
*/

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
 *	hm.axis2 is set to the secondary axis if axis is a dual axis
 *	hm.axis2 is set to -1 otherwise
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
