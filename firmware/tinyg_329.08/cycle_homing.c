 /*
 * cycle_homing - homing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S Hart, Jr.
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
#include "util.h"
#include "gpio.h"

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static uint8_t _cm_set_hm_func(uint8_t (*func)(int8_t axis));
static int8_t _get_next_axis(int8_t axis);
static int8_t _get_next_axes(int8_t axis);
static uint8_t _cm_homing_cycle_start(int8_t axis);
static uint8_t _cm_homing_cycle_finalize(int8_t axis);
static uint8_t _cm_homing_axis_start(int8_t axis);
static uint8_t _cm_homing_axis_search(int8_t axis);
static uint8_t _cm_homing_axis_search_backoff(int8_t axis);
static uint8_t _cm_homing_axis_latch(int8_t axis);
static uint8_t _cm_homing_axis_latch_backoff(int8_t axis);
static uint8_t _cm_homing_axis_move(double target, double velocity, int8_t axis);
static uint8_t _cm_homing_return_to_home_1(int8_t axis);
static uint8_t _cm_homing_return_to_home_2(int8_t axis);
static uint8_t _cm_homing_return_to_home_3(int8_t axis);

//uint8_t _cm_homing_return_to_zero(void);

struct hmHomingSingleton {		// persistent G28 and G30 runtime variables
	int8_t axis;				// axis currently being homed
	int8_t axis2;				// second axis if dual axis
	uint8_t (*func)(int8_t axis); // binding for current processing function

	double search_velocity;		// wasteful, but makes the code much easier to read
	double latch_velocity;		//   ""
	double zero_offset;			//   ""
	double work_offset;			//   ""
	double travel_hard_limit;	//   ""
};
static struct hmHomingSingleton hm;
#define AXIS_STATE hm.axis_state[hm.axis]	// shorthand

// Axis characters in program memory and an accessor macro (for INFO statements)
char strX[] PROGMEM = "X";	
char strY[] PROGMEM = "Y";
char strZ[] PROGMEM = "Z";
char strA[] PROGMEM = "A";
char strB[] PROGMEM = "B";
char strC[] PROGMEM = "C";
PGM_P strAxes[] PROGMEM = { strX, strY, strZ, strA, strB, strC }; 
#define PSTR_axis(a) (PGM_P)pgm_read_word(&strAxes[a])

/* 
 * cm_homing_cycle() 	- homing cycle using limit switches
 * cm_homing_callback() - wrapper routine for running the homing cycle
 *
 *	Homing works from a G30 according to the following 
 * 		http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing
 *
 *	The cm_homing_callback() function is a dispatcher that intelligently cycles 
 *	through the homing moves needed to complete the operation. Child routines
 *	can return these states:
 *
 *		TG_OK		- operation was OK. The homing cycle is still running
 *		TG_ERR		- operation had error. End homing cycle without achieving HOMED
 *		TG_COMPLETE	- Homing cycle is complete
 *
 *	The continuation (cm_homing_callback()) is coded as an outer wrapper 
 *	routine and a dispatch through hm.func. The wrapper handles trivial noop 
 *	cases, synchronizes to move endings and feedholds (switch closures) 
 *	and translates the return codes from the lower routines so the continuation 
 *	sends well-behaved return codes back to the controller.
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
	cm.homing_state = HOMING_CYCLE;
	cm_save_gcode_model();					// save current state for later
	cm_use_length_units(MILLIMETER_MODE);
	cm_set_distance_mode(INCREMENTAL_MODE);
	hm.axis = -1;							// set to retrieve initial axis
	hm.func = _cm_homing_cycle_start; 		// bind initial processing function
	return (TG_OK);
}

uint8_t cm_homing_callback(void)
{
	if (cm.homing_state != HOMING_CYCLE) {	// exit if not in a homing cycle
		return (TG_NOOP);
	}
	if (cm_isbusy() == TRUE) {   			// sync to the canonical machine
		return (TG_EAGAIN); 
	}
	if (hm.func(hm.axis) == TG_COMPLETE) {	// run the current function
		return (TG_OK);
	}
	return (TG_EAGAIN);
}

/*
 * _cm_set_hm_fuunc() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _cm_set_hm_func(uint8_t (*func)(int8_t axis))
{
	hm.func = func;
	return (TG_OK);
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
				INFO1(PSTR("Requested to home disabled or inhibited axis %s"), PSTR_axis(axis));
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
	return (TG_OK);
*/
}

/***** Homing movement functions *****
 * _cm_homing_cycle_start()	- initial call to homing goes here.
 * _cm_homing_axis_start() - start a new axis
 * _cm_homing_axis_search()
 * _cm_homing_axis_search_backoff()
 * _cm_homing_axis_latch()
 * _cm_homing_axis_latch_backoff()
 * _cm_homing_axis_move()
 */

uint8_t _cm_homing_cycle_start(int8_t axis)
{
//	INFO(PSTR("Homing cycle start"));
	return (_cm_set_hm_func(_cm_homing_axis_start));
}

uint8_t _cm_homing_cycle_finalize(int8_t axis)
{
	if (axis == -1) {							// -1 is done OK
		return (_cm_set_hm_func(_cm_homing_return_to_home_1));
	} else if (axis == -2) { 					// -2 is error
		cm.homing_state = HOMING_NOT_HOMED;
		cm.machine_state = MACHINE_STOP;
		return (TG_COMPLETE);
	}
	INFO1(PSTR("Homing cycle finalize error: axis = %s"), PSTR_axis(axis));
	return (TG_ERR);
}

uint8_t _cm_homing_axis_start(int8_t axis)
{
	// get first or next axis
	if ((axis = _get_next_axis(axis)) < 0) {
		return (_cm_homing_cycle_finalize(axis));
	}
	hm.axis = axis;
	hm.search_velocity = cfg.a[axis].homing_search_velocity;
	hm.latch_velocity = cfg.a[axis].homing_latch_velocity;
	hm.zero_offset = cfg.a[axis].homing_zero_offset;
	hm.work_offset = cfg.a[axis].homing_work_offset;
	hm.travel_hard_limit = cfg.a[axis].travel_hard_limit;

	if (hm.search_velocity == 0) { 			// validate the axis can be homed
		INFO1(PSTR("%S axis search velocity is zero"), PSTR_axis(axis));
		return (TG_COMPLETE);
	}
	if (hm.travel_hard_limit == 0) {
		INFO1(PSTR("%S axis travel hard limit is zero"), PSTR_axis(axis));
		return (TG_COMPLETE);
	}
	// Note: the is-the-switch-enabled? test is left out for now

	// ---> For now all axes are single - no dual axis detection or invocation
	// This is where you have to detect and handle dual axes.

	// Handle an initial switch closure by backing off the switch
	gp_read_switches();					// sets gp.sw_flags
	if (gp.sw_flags[axis]) {			// test if the MIN switch for the axis is thrown
//		INFO3(PSTR("%S axis move: %f, %f"), PSTR_axis(axis), hm.zero_offset, hm.latch_velocity);
		_cm_homing_axis_move(hm.zero_offset, hm.latch_velocity, axis);
	}
	gp_clear_switches();
	return (_cm_set_hm_func(_cm_homing_axis_search));
}

uint8_t _cm_homing_axis_search(int8_t axis)
{
//	INFO3(PSTR("%S axis search: %f, %f"), PSTR_axis(axis), -hm.travel_hard_limit, hm.search_velocity);
	_cm_homing_axis_move(-hm.travel_hard_limit, hm.search_velocity, axis);
	return (_cm_set_hm_func(_cm_homing_axis_search_backoff));
}

uint8_t _cm_homing_axis_search_backoff(int8_t axis)
{
//	INFO3(PSTR("%S axis search backoff: %f, %f"), PSTR_axis(axis), hm.zero_offset, hm.search_velocity);
	_cm_homing_axis_move(hm.zero_offset, hm.search_velocity, axis);
	return (_cm_set_hm_func(_cm_homing_axis_latch));
}

uint8_t _cm_homing_axis_latch(int8_t axis)
{
//	INFO3(PSTR("%S axis latch: %f, %f"), PSTR_axis(axis), -2*hm.zero_offset, hm.latch_velocity);
	_cm_homing_axis_move(-2*hm.zero_offset, hm.latch_velocity, axis);
	return (_cm_set_hm_func(_cm_homing_axis_latch_backoff)); 
}

uint8_t _cm_homing_axis_latch_backoff(int8_t axis)
{
//	INFO3(PSTR("%S axis latch backoff: %f, %f"), PSTR_axis(axis), hm.zero_offset, hm.latch_velocity);
	_cm_homing_axis_move(hm.zero_offset, hm.latch_velocity, axis);
	return (_cm_set_hm_func(_cm_homing_axis_start));
}

uint8_t _cm_homing_axis_move(double target, double velocity, int8_t axis)
{
	set_vector_by_axis(target, axis);
	(void)cm_set_feed_rate(velocity);
	(void)mp_flush_planner();
	return(cm_straight_feed(vector));
}

uint8_t _cm_homing_return_to_home_1(int8_t axis)	// first move in return to home
{
//	double way_point[AXES] = {0,0,0,0,0,0};
//	for (uint8_t i=0; i<AXES; i++) {
//		if (gf.target[i] == true) {
//			way_point[i] = gm.target[i]; // note to self: it's still working in incremental coordinates
//		}
//	}

	// simplified - takes advantage of unspecified axis target[] values == 0
	double way_point[AXES];

	for (uint8_t i=0; i<AXES; i++) {
		way_point[i] = gm.target[i]; // note to self: it's still working in incremental coordinates
	}
	(void)mp_flush_planner();
	(void)cm_straight_traverse(way_point);
	return (_cm_set_hm_func(_cm_homing_return_to_home_2));
}

uint8_t _cm_homing_return_to_home_2(int8_t axis)	// second move in return to home
{
	double end_point[AXES] = {0,0,0,0,0,0};

	for (uint8_t i=0; i<AXES; i++) {
		if (gf.target[i] == true) {
			end_point[i] = cfg.a[i].homing_work_offset;	// note to self: it's working in incremental coordinates
		}
	}
//	(void)mp_flush_planner();
	(void)cm_straight_traverse(vector);
	return (_cm_set_hm_func(_cm_homing_return_to_home_3));
}

uint8_t _cm_homing_return_to_home_3(int8_t axis)	// third part of return to home
{
	// STILL NEED TO SET ZERO HERE.
	cm_restore_gcode_model();
	cm.homing_state = HOMING_HOMED;
	cm.machine_state = MACHINE_STOP;
	return (TG_COMPLETE);
}

/*
	// Return to zero and reset the models
	cm_restore_gcode_model();
	(void)mp_set_axis_position(gt.position);	// MP layer must agree with gt position
	(void)cm_set_distance_mode(ABSOLUTE_MODE);
	(void)cm_set_feed_rate(HOMING_ZERO_RATE);
	cm_set_vector(0,0,0,0,0,0);
	return(HOMING_ZERO_MOVE(vector));

	// wait until return to zero is complete before releasing the cycle
	if (cm.homing_state == HOMING_RTZ_WAIT) {
		cm.homing_state = HOMING_HOMED;			// declare victory
		cm.machine_state = MACHINE_STOP;		// done homing
		return (TG_COMPLETE);
	}
	return (TG_OK);
}
*/

/*
 * _cm_run_homing_dual_axis() - kernal routine for running homing on a dual axis
 *
 */

uint8_t _cm_run_homing_dual_axis(int8_t axis)
{
	return (TG_OK);
}

/* 
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



