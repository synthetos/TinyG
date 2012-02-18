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
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "config.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "report.h"
#include "util.h"
#include "gpio.h"

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static uint8_t _homing_axis_start(int8_t axis);
static uint8_t _homing_axis_search(int8_t axis);
static uint8_t _homing_axis_backoff(int8_t axis);
static uint8_t _homing_axis_latch(int8_t axis);
static uint8_t _homing_axis_final(int8_t axis);
static uint8_t _homing_axis_move(int8_t axis, double target, double velocity);
static uint8_t _homing_finalize_1(int8_t axis);
static uint8_t _homing_finalize_2(int8_t axis);
static uint8_t _homing_finalize_3(int8_t axis);

static uint8_t _set_hm_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);
static int8_t _get_next_axes(int8_t axis);

struct hmHomingSingleton {		// persistent G28 and G30 runtime variables
	// controls for homing cycle
	int8_t axis;				// axis currently being homed
	int8_t axis2;				// second axis if dual axis
	uint8_t (*func)(int8_t axis);// binding for current processing function

	// convenience copies of config parameters
	double zero_offset;			// somewhat wasteful, but makes the coding simpler
	double coord_offset;		//   ""
	double search_travel;		//	 ""
	double search_velocity;		//   ""
	double latch_velocity;		//   ""

	// saved state from gcode model
	double feed_rate_saved;		// F setting
	uint8_t units_mode_saved;	// G20,G21 global setting
	uint8_t coord_system_saved;	// G54 - G59 setting
	uint8_t distance_mode_saved;// G90,G91 global setting
//	uint8_t path_control_saved;	// G61,G61.1,G64 global setting
//	double jerk_saved;			// saved and restored for each axis homed
};
static struct hmHomingSingleton hm;

/* 
 * cm_homing_cycle() 	- homing cycle using limit switches
 * cm_homing_callback() - wrapper routine for running the homing cycle
 *
 * Homing works from a G30 according to the following writeup: 
 * 		http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing
 *
 *	--- How does this work? ---
 *
 *	When a G30 homing cycle is intiated machine state is set to RUN and homing
 *	state is set to HOMING_IN_CYCLE. At the start of a homing cycle the limit 
 *	switches in gpio.c are treated as homing switches (they become modal).
 *
 *	After some initialization and backing off any closed switches a series of 
 *	search and latch moves are run for each axis specified in the G30 command.
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
 *	Once this occurs a return-to-zero move is performed that sends the machine
 *	to the zero of the selected coordinate system via the way-point specified 
 *	in the G30 request. 
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
	cm_set_units_mode(MILLIMETER_MODE);
	cm_set_distance_mode(INCREMENTAL_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);	// homing is done in machine coordinates

	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _homing_axis_start; 			// bind initial processing function
	cm.machine_state = MACHINE_RUN;
	cm.homing_state = HOMING_IN_CYCLE;
	return (TG_OK);
}

uint8_t cm_homing_callback(void)
{
	if (cm.homing_state != HOMING_IN_CYCLE) return (TG_NOOP); // exit if not in a homing cycle
	if (cm_isbusy() == true) return (TG_EAGAIN);			// sync to planner move ends
	return (hm.func(hm.axis));
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
			return (_set_hm_func(_homing_finalize_1));
		} else if (axis == -2) { 					// -2 is error
			cm.homing_state = HOMING_NOT_HOMED;
			cm.machine_state = MACHINE_STOP;
			cm_set_units_mode(hm.units_mode_saved);
			cm_set_distance_mode(hm.distance_mode_saved);
			return (TG_HOMING_CYCLE_FAILED);
		}
	}
	hm.axis = axis;									// make convenience copies
	hm.coord_offset = cm_get_coord_offset(axis);	// offset for this axis of active coord system
	hm.search_travel = cfg.a[axis].travel_max;
	hm.zero_offset = cfg.a[axis].zero_offset;
	hm.search_velocity = cfg.a[axis].search_velocity;
	hm.latch_velocity = cfg.a[axis].latch_velocity;
//	hm.jerk_saved = cfg.a[axis].jerk_max;			// per-axis save

	if ((hm.search_velocity == 0) || (hm.search_travel == 0)) {
		return (TG_GCODE_INPUT_ERROR);	// requested axis that can't be homed
	}
	// Note: the is-the-switch-enabled? test is left out for now

	// ---> For now all axes are single - no dual axis detection or invocation
	// This is where you have to detect and handle dual axes.

	// Handle an initial switch closure by backing off the switch
	// (NOTE: this gets more complicated if switch pins are shared)
	gpio_read_switches();				// sets gp.sw_flags
	if (gpio_get_switch(axis) == true) {// test if the MIN switch for the axis is thrown
		_homing_axis_move(axis, hm.zero_offset, hm.latch_velocity);
	}
	gpio_clear_switches();
	return (_set_hm_func(_homing_axis_search));
}

static uint8_t _homing_axis_search(int8_t axis)
{
	_homing_axis_move(axis, -hm.search_travel, hm.search_velocity);
	return (_set_hm_func(_homing_axis_backoff));
}

static uint8_t _homing_axis_backoff(int8_t axis)
{
	_homing_axis_move(axis, hm.zero_offset, hm.search_velocity);
	return (_set_hm_func(_homing_axis_latch));
}

static uint8_t _homing_axis_latch(int8_t axis)
{
	_homing_axis_move(axis, -2*hm.zero_offset, hm.latch_velocity);
	return (_set_hm_func(_homing_axis_final)); 
}

static uint8_t _homing_axis_final(int8_t axis)
{
	_homing_axis_move(axis, hm.zero_offset, hm.search_velocity);
//	cfg.a[axis].jerk_max = hm.jerk_saved;		// per-axis restore
	return (_set_hm_func(_homing_axis_start));
}

static uint8_t _homing_axis_move(int8_t axis, double target, double velocity)
{
//	INFO3(PSTR("Homing move: [%d] %f, %f"), axis, target, velocity);
	set_vector_by_axis(target, axis);
	cm_set_feed_rate(velocity);
	mp_flush_planner();
	ritorno(cm_straight_feed(vector));
	return (TG_EAGAIN);
}

/* Homing finalization moves:
 *	_homing_finalize_1() - move to way point specified in G30 command
 *	_homing_finalize_2() - move to work coordinate system zero
 *	_homing_finalize_3() - wait for finalize_2 move to complete and restore Gcode model
 */
static uint8_t _homing_finalize_1(int8_t axis)	// move to way point in return to home
{
	cm_set_machine_coords(set_vector(0,0,0,0,0,0));
	mp_flush_planner(); // should be stopped, but just in case of switch closure

	cm_set_coord_system(hm.coord_system_saved);	// restore to work coordinate system
	cm_set_distance_mode(ABSOLUTE_MODE); 		// needs to work in absolute coordinates for now
	cm_straight_traverse(gn.target);			// only axes with gf flags set will move
	return (_set_hm_func(_homing_finalize_2));
}

static uint8_t _homing_finalize_2(int8_t axis) // move to zero in selected coordinate system
{
	cm_straight_traverse(set_vector(0,0,0,0,0,0));// only axes with gf flags will move
	return (_set_hm_func(_homing_finalize_3));
}

static uint8_t _homing_finalize_3(int8_t axis)	// third part of return to home
{
	cm_set_units_mode(hm.units_mode_saved);
	cm_set_distance_mode(hm.distance_mode_saved);
	cm_set_feed_rate(hm.feed_rate_saved);
	cm.homing_state = HOMING_HOMED;
	cm.machine_state = MACHINE_STOP;
	rpt_run_status_report();
	return (TG_OK);
}

/*
 * _run_homing_dual_axis() - kernal routine for running homing on a dual axis
 *
 */

static uint8_t _run_homing_dual_axis(int8_t axis)
{
	return (TG_OK);
}

/***************************************************************************** 
 * cm_return_to_home() - G28 cycle
 * cm_return_to_home_callback() - continuation for the above
 */

uint8_t cm_return_to_home(void)
{
	return (TG_OK);
}

uint8_t cm_return_to_home_callback(void)
{
	return (TG_NOOP);
}


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
 *	Returns next axis based on "axis" argument
 *	Returns -1 when all axes have been processed
 *	Returns -2 if no axes are specified (Gcode calling error)
 *
 *	hm.axis2 is set to the secondary axis if axis is a dual axis
 *	hm.axis2 is set to -1 otherwise
 *
 *	Isolating this function facilitates implementing more complex and 
 *	user-specified axis homing orders
 */

int8_t _get_next_axis(int8_t axis)
{
	int8_t next_axis;

	// test for next axis or break if no more
	for (next_axis = ++axis; next_axis < AXES; next_axis++) {
		if (gf.target[next_axis] == true) {
			return (next_axis);
		}
	}
	// test if there are axes to process
	for (next_axis = 0; next_axis < AXES; next_axis++) {	
		if (gf.target[next_axis] == true) {
			return (-1);
		}
	}
	return (-2);
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
		INFO(PSTR("Homing failed because no axes or disabled/inhibited axes were specified"));
		return (-2);	// didn't find any axes to process
	}

	// Scan target vector from the current axis to find next axis or the end
	for (next_axis = ++axis; next_axis < AXES; next_axis++) {
		if (gf.target[next_axis] == true) { 
			if ((cfg.a[next_axis].axis_mode == AXIS_INHIBITED) || 	
				(cfg.a[next_axis].axis_mode == AXIS_DISABLED)) {	// Skip if axis disabled or inhibited
//				INFO1(PSTR("Requested to home disabled or inhibited axis %s"), PSTR_axis(axis));
				continue;
			}
			break;		// got a good one
		}
		return (-1);	// you are done
	}

	// Got a valid axis. Find out if it's a dual
/*
	for (int8_t i=0; i<AXES; i++) {
		if (cfg.a[i].axis_mode == AXIS_INHIBITED) {
			joint[i] = 0;
		}
		for (j=0; j<MOTORS; j++) {
			if (cfg.m[j].motor_map == i) {
				steps[j] = joint[i] * cfg.m[j].steps_per_unit;
			}
		}
	}
*/
	return (TG_OK);
}

