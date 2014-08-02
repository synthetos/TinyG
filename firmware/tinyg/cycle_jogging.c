/*
 * cycle_jogging.c - jogging cycle extension to canonical_machine.c
 *
 * by Mike Estee - Other Machine Company
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
#include "planner.h"
#include "util.h"

#ifdef __cplusplus
extern "C"{
#endif

/**** Jogging singleton structure ****/

struct jmJoggingSingleton {			// persistent jogging runtime variables
	// controls for jogging cycle
	int8_t axis;					// axis currently being jogged
	float dest_pos;					// distance relative to start position to travel
	float start_pos;
	float velocity_start;			// initial jog feed
	float velocity_max;

	uint8_t (*func)(int8_t axis);	// binding for callback function state machine

	// state saved from gcode model
	float saved_feed_rate;			// F setting
	uint8_t saved_units_mode;		// G20,G21 global setting
	uint8_t saved_coord_system;		// G54 - G59 setting
	uint8_t saved_distance_mode;	// G90,G91 global setting
	uint8_t saved_feed_rate_mode;   // G93,G94 global setting
	float saved_jerk;				// saved and restored for each axis homed
};
static struct jmJoggingSingleton jog;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _set_jogging_func(uint8_t (*func)(int8_t axis));
static stat_t _jogging_axis_start(int8_t axis);
static stat_t _jogging_axis_jog(int8_t axis);
static stat_t _jogging_finalize_exit(int8_t axis);

/*****************************************************************************
 * cm_jogging_cycle_start()	- jogging cycle using soft limits
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
 */

static stat_t _set_jogging_func(uint8_t (*func)(int8_t axis));
static stat_t _jogging_axis_start(int8_t axis);
static stat_t _jogging_axis_jog(int8_t axis);
static stat_t _jogging_finalize_exit(int8_t axis);

stat_t cm_jogging_cycle_start(uint8_t axis)
{
	// save relevant non-axis parameters from Gcode model
	jog.saved_units_mode = cm_get_units_mode(ACTIVE_MODEL);
	jog.saved_coord_system = cm_get_coord_system(ACTIVE_MODEL);
	jog.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);
	jog.saved_feed_rate_mode = cm_get_feed_rate_mode(ACTIVE_MODEL);
	jog.saved_feed_rate = cm_get_feed_rate(ACTIVE_MODEL);
	jog.saved_jerk = cm_get_axis_jerk(axis);

	// set working values
	cm_set_units_mode(MILLIMETERS);
	cm_set_distance_mode(ABSOLUTE_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);			// jogging is done in machine coordinates
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);

	jog.velocity_start = JOGGING_START_VELOCITY;	// see canonical_machine.h for #define
	jog.velocity_max = cm.a[axis].velocity_max;

	jog.start_pos = cm_get_absolute_position(RUNTIME, axis);
	jog.dest_pos = cm_get_jogging_dest();

	jog.axis = axis;
	jog.func = _jogging_axis_start; 				// bind initial processing function

	cm.cycle_state = CYCLE_JOG;
	return (STAT_OK);
}


/* Jogging axis moves - these execute in sequence for each axis
 * cm_jogging_callback() 		- main loop callback for running the jogging cycle
 *	_set_jogging_func()			- a convenience for setting the next dispatch vector and exiting
 *	_jogging_axis_start()		- setup and start
 *	_jogging_axis_jog()			- ramp the jog
 *	_jogging_axis_move()		- move
 *	_jogging_finalize_exit()	- back off the cleared limit switch
 */

stat_t cm_jogging_callback(void)
{
	if (cm.cycle_state != CYCLE_JOG) { return (STAT_NOOP); } 		// exit if not in a jogging cycle
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN); }	// sync to planner move ends
	return (jog.func(jog.axis));									// execute the current homing move
}

static stat_t _set_jogging_func(stat_t (*func)(int8_t axis))
{
	jog.func = func;
	return (STAT_EAGAIN);
}

static stat_t _jogging_axis_start(int8_t axis)
{
	return (_set_jogging_func(_jogging_axis_jog));		// register the callback for the jog move
}

static stat_t _jogging_axis_jog(int8_t axis)			// run the jog move
{
	float vect[] = {0,0,0,0,0,0};
	float flags[] = {false, false, false, false, false, false};
	flags[axis] = true;

	float velocity = jog.velocity_start;
	float direction = jog.start_pos <= jog.dest_pos ? 1. : -1.;
	float delta = abs(jog.dest_pos - jog.start_pos);

	cm.gm.feed_rate = velocity;
	mp_flush_planner();									// don't use cm_request_queue_flush() here
	cm_request_cycle_start();

#if 1
	float ramp_dist = 2.0;
	float steps = 0.0;
	float max_steps = 25;
	float offset = 0.01;
	while( delta>ramp_dist && offset < delta && steps < max_steps )
	{
		vect[axis] = jog.start_pos + offset * direction;
		cm.gm.feed_rate = velocity;
		ritorno(cm_straight_feed(vect, flags));

		steps++;
		float scale = pow(10.0, steps/max_steps) / 10.0;
		velocity = jog.velocity_start + (jog.velocity_max - jog.velocity_start) * scale;
		offset += ramp_dist * steps/max_steps;
	}
#else
    // use a really slow jerk so we ramp up speed
    // FIXME: need asymmetric accel/deaccel jerk for this to work...
//	cm.a[axis].jerk_max = 25;
	cm_set_axis_jerk(axis, 25);
    //cm.a[axis].jerk_accel = 10;
    //cm.a[axis].jerk_deaccel = 900;
#endif

	// final move
	cm.gm.feed_rate = jog.velocity_max;
	vect[axis] = jog.dest_pos;
	ritorno(cm_straight_feed(vect, flags));
    return (_set_jogging_func(_jogging_finalize_exit));
}


static stat_t _jogging_finalize_exit(int8_t axis)	// finish a jog
{
	mp_flush_planner(); 							// FIXME: not sure what to do on exit
	cm_set_axis_jerk(axis, jog.saved_jerk);
	cm_set_coord_system(jog.saved_coord_system);	// restore to work coordinate system
	cm_set_units_mode(jog.saved_units_mode);
	cm_set_distance_mode(jog.saved_distance_mode);
	cm_set_feed_rate_mode(jog.saved_feed_rate_mode);
	cm.gm.feed_rate = jog.saved_feed_rate;
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
	cm_cycle_end();
	cm.cycle_state = CYCLE_OFF;

	printf("{\"jog\":0}\n");
	return (STAT_OK);
}

/*
static stat_t _jogging_error_exit(int8_t axis)
{
	// Generate the warning message. Since the error exit returns via the jogging callback
	// - and not the main controller - it requires its own display processing
//	nv_reset_nv_list();
	_jogging_finalize_exit(axis);					// clean up
	return (STAT_JOGGING_CYCLE_FAILED);				// jogging state
}
*/

#ifdef __cplusplus
}
#endif
