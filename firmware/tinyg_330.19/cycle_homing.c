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
static uint8_t _homing_axis_search_backoff(int8_t axis);
static uint8_t _homing_axis_latch(int8_t axis);
static uint8_t _homing_axis_latch_backoff(int8_t axis);
static uint8_t _homing_axis_move(int8_t axis, double target, double velocity);
static uint8_t _homing_go_to_way_point(int8_t axis);
static uint8_t _homing_go_to_work_zero(int8_t axis);
static uint8_t _homing_finalize(int8_t axis);
//uint8_t _homing_return_to_zero(void);

static uint8_t _set_hm_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);
static int8_t _get_next_axes(int8_t axis);

struct hmHomingSingleton {		// persistent G28 and G30 runtime variables
	int8_t axis;				// axis currently being homed
	int8_t axis2;				// second axis if dual axis
	uint8_t (*func)(int8_t axis);// binding for current processing function
	double search_travel;		// wasteful, but makes the code much easier to read
	double search_velocity;		//   ""
	double latch_velocity;		//   ""
	double zero_offset;			//   ""
	double work_offset;			//   ""
};
static struct hmHomingSingleton hm;
#define AXIS_STATE hm.axis_state[hm.axis]	// shorthand

/*
// Axis characters in program memory and an accessor macro (for INFO statements)
char msg_X[] PROGMEM = "X";	
char msg_Y[] PROGMEM = "Y";
char msg_Z[] PROGMEM = "Z";
char msg_A[] PROGMEM = "A";
char msg_B[] PROGMEM = "B";
char msg_C[] PROGMEM = "C";
PGM_P msg_Axes[] PROGMEM = { msg_X, msg_Y, msg_Z, msg_A, msg_B, msg_C }; 
#define PSTR_axis(a) (PGM_P)pgm_read_word(&msg_Axes[a])
*/

/* 
 * cm_homing_cycle() 	- homing cycle using limit switches
 * cm_homing_callback() - wrapper routine for running the homing cycle
 * _homing_axis_start()	- start a new axis
 * _homing_axis_search()
 * _homing_axis_search_backoff()
 * _homing_axis_latch()
 * _homing_axis_latch_backoff()
 * _homing_axis_move()
 * _homing_cycle_finalize()
 * _homing_go_to_way_point()
 * _homing_go_to_work_zero()
 * _homing_finalize()
 */
/* Homing works from a G30 according to the following writeup: 
 * 		http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing
 *
 *	--- How does this work? ---
 *
 *	When a G30 homing cycle is intiated machine state is set to RUN and homing
 *	state is set to HOMING_IN_CYCLE. At the start of a homing cycle the limit 
 *	switches in gpio.c are treated as homing switches (modal).
 *
 *	After some initialization and backing off any closed switches a series of 
 *	search and latch moves are run for each affected axis, in turn. The 
 *	cm_homing_callback() function is a dispatcher that vectors to the homing 
 *	currently running. Each move must clear the planner and any previous hold 
 *	state before it can be run.
 *
 *	Each move runs until either it is done or a switch is hit. The action of
 *	hitting a switch causes a feedhold to be executed and the hold state to 
 *	become HOLD. This then causes the machine to become "not busy" so cm_isbusy() 
 *	in the callback returns false, allowing the next move to be run.
 *
 *	At the end a homing offset move is performed that sends the machine to the 
 *	work coordinate zero via the way-point specified in the G30 request. 
 *	The coordinate systems are initialized to zero this point.
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
	cm.machine_state = MACHINE_RUN;
	cm.homing_state = HOMING_IN_CYCLE;
	cm_save_gcode_model();					// save current state for later
	cm_set_inches_mode(MILLIMETER_MODE);
	cm_set_absolute_mode(INCREMENTAL_MODE);
	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _homing_axis_start; 			// bind initial processing function
	return (TG_OK);
}

uint8_t cm_homing_callback(void)
{
	if (cm.homing_state != HOMING_IN_CYCLE) return (TG_NOOP); // exit if not in a homing cycle
	if (cm_isbusy() == true) return (TG_EAGAIN);			// sync to the canonical machine
	return (hm.func(hm.axis));
}

static uint8_t _homing_axis_start(int8_t axis)
{
	// get first or next axis
	if ((axis = _get_next_axis(axis)) < 0) { 		// axes are done or error
		if (axis == -1) {							// -1 is done
			return (_set_hm_func(_homing_go_to_way_point));
		} else if (axis == -2) { 					// -2 is error
			cm.homing_state = HOMING_NOT_HOMED;
			cm.machine_state = MACHINE_STOP;
			return (TG_HOMING_CYCLE_FAILED);
		}
	}
	hm.axis = axis;
	hm.search_travel = cfg.a[axis].homing_travel;
	hm.search_velocity = cfg.a[axis].homing_search_velocity;
	hm.latch_velocity = cfg.a[axis].homing_latch_velocity;
	hm.zero_offset = cfg.a[axis].homing_zero_offset;
	hm.work_offset = cfg.a[axis].homing_work_offset;

	if ((hm.search_velocity == 0) || (hm.search_travel == 0)) {
		return (TG_GCODE_INPUT_ERROR);		// requested axis that can't be homed
	}
	// Note: the is-the-switch-enabled? test is left out for now

	// ---> For now all axes are single - no dual axis detection or invocation
	// This is where you have to detect and handle dual axes.

	// Handle an initial switch closure by backing off the switch
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
	return (_set_hm_func(_homing_axis_search_backoff));
}

static uint8_t _homing_axis_search_backoff(int8_t axis)
{
	_homing_axis_move(axis, hm.zero_offset, hm.search_velocity);
	return (_set_hm_func(_homing_axis_latch));
}

static uint8_t _homing_axis_latch(int8_t axis)
{
	_homing_axis_move(axis, -2*hm.zero_offset, hm.latch_velocity);
	return (_set_hm_func(_homing_axis_latch_backoff)); 
}

static uint8_t _homing_axis_latch_backoff(int8_t axis)
{
	_homing_axis_move(axis, hm.zero_offset, hm.search_velocity);
	return (_set_hm_func(_homing_axis_start));
}

static uint8_t _homing_axis_move(int8_t axis, double target, double velocity)
{
//	INFO3(PSTR("%S homing move: %f, %f"), PSTR_axis(axis), target, velocity);
	set_vector_by_axis(target, axis);
	(void)cm_set_feed_rate(velocity);
	(void)mp_flush_planner();
	ritorno(cm_straight_feed(vector));
	return (TG_EAGAIN);
}

static uint8_t _homing_go_to_way_point(int8_t axis)	// move to way point in return to home
{
	double way_point[AXES];

	cm_restore_gcode_model();
	cm_set_origin_offsets(set_vector(0,0,0,0,0,0));
	(void)mp_set_axis_position(gt.position);		// MP layer must agree with gt position
	(void)mp_flush_planner();

	for (uint8_t i=0; i<AXES; i++) { // takes advantage of unspecified axis target[] values == 0
		way_point[i] = gn.target[i]; // note to self: now it's back working in absolute coordinates
	}
	(void)cm_straight_traverse(way_point);
	return (_set_hm_func(_homing_go_to_work_zero));
}

static uint8_t _homing_go_to_work_zero(int8_t axis)	// move in return to home
{
	double zero_point[AXES];

	for (uint8_t i=0; i<AXES; i++) {
		if (gf.target[i] == true) {
			zero_point[i] = cfg.a[i].homing_work_offset; // absolute coordinates
		}
	}
	(void)cm_straight_traverse(zero_point);
	return (_set_hm_func(_homing_finalize));
}

static uint8_t _homing_finalize(int8_t axis)	// third part of return to home
{
	cm_set_origin_offsets(set_vector(0,0,0,0,0,0));
	(void)mp_set_axis_position(gt.position);	// MP layer must agree with gt position
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

