/*
 * motion_control.c - cartesian robot controller.
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/* --- TinyG Notes ----
 *
 *  Modified to support Xmega family processors
 *	This layer has been re-written to work with the canonical machine.
 *	It is now responsible only for the cartesian coordinates and motions
 *	The calls to the routines are simpler and do not need to know about
 *	the state of the gcode model. A rudimentary multitasking capability 
 *	is implemented for lines, arcs, dwells, and program control. Routines 
 *	are coded as non-blocking continuations - which are simple state 
 *	machines that are re-entered multiple times until a particular 
 *	operation is complete (like queuing an arc).
 */

#include "xmega_init.h"				// put before <util/delay.h>
#include <util/delay.h>				// needed for dwell
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>

#include "tinyg.h"
#include "config.h"
#include "motion_control.h"
#include "move_queue.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

enum mcGeneratorState {
	MC_STATE_OFF,					// generator is OFF
	MC_STATE_NEW,					// initial call to generator
	MC_STATE_RUNNING,				// in process, needs re-entry (continuation)
	MC_STATE_MAX
};

struct MotionControlState {			// robot pos & vars used by lines and arcs
	uint8_t move_type;				// type of move. See mvType
	volatile uint8_t line_continue_state;	// line generator state
	volatile uint8_t dwell_continue_state;	// dwell generator state
	volatile uint8_t stop_continue_state;	// start/stop state
	volatile uint8_t arc_continue_state;	// arc generator state
	int32_t position[AXES];    		// current position of tool in abs steps
	int32_t target[AXES];			// target position of tool in abs steps
	int32_t steps[AXES];			// target line in relative steps
	uint32_t microseconds;			// target move microseconds
	double mm_of_travel;			// different than ma.mm_of_travel
};
static struct MotionControlState mc;

struct MotionControlArc {			// additional values used by arc
	uint8_t axis_1;					// arc plane axis
	uint8_t axis_2;					// arc plan axis
	uint8_t axis_linear;			// transverse axis (helical)
	uint16_t segments;				// number of segments in arc
	uint16_t segment_counter;		// number of segments queued so far

	double theta;					// total angle specified by arc
	double radius;					// computed via offsets
	double center_x;				// center of this circle
	double center_y;				// center of this circle
	double dtarget[AXES];			// target position in floating point
	double theta_per_segment;		// angular motion per segment
	double linear_per_segment;		// linear motion per segment
};
struct MotionControlArc ma;

/* 
 * mc_init() 
 */

void mc_init()
{
	clear_vector(mc.position);				// zero robot position
	mc.line_continue_state = MC_STATE_OFF;	// turn off the generators
	mc.dwell_continue_state = MC_STATE_OFF;
	mc.stop_continue_state = MC_STATE_OFF;
	mc.arc_continue_state = MC_STATE_OFF;
}

/* 
 * mc_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mc_isbusy()
{
	if (st_isbusy()) {
		return (TRUE);
	}
	if (mc.line_continue_state || mc.dwell_continue_state ||
		mc.stop_continue_state || mc.arc_continue_state) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

/* 
 * mc_set_position() - set current position (support for G92)
 */

int mc_set_position(double x, double y, double z, double a)
{
	mc.position[X] = lround(x * CFG(X).steps_per_unit);
	mc.position[Y] = lround(y * CFG(Y).steps_per_unit);
	mc.position[Z] = lround(z * CFG(Z).steps_per_unit); 
	mc.position[A] = lround(z * CFG(Z).steps_per_unit); 
	return (TG_OK);
}

/* 
 * mc_async_stop() - stop current motion immediately
 * mc_async_start() - (re)start motion
 * mc_async_end() - stop current motion immediately
 */

int mc_async_stop()
{
	st_stop();						// stop the steppers
	return (TG_OK);
}

int mc_async_start()
{
	st_start();						// start the stoppers
	return (TG_OK);
}

int mc_async_end()
{
	st_end();								// stop the motion
	mc.line_continue_state = MC_STATE_OFF;	// turn off the generators
	mc.dwell_continue_state = MC_STATE_OFF;
	mc.stop_continue_state = MC_STATE_OFF;
	mc.arc_continue_state = MC_STATE_OFF;
//	st_init();						// reset the motors
	cfg.kill = TRUE;				// set kill flag
	return (TG_OK);
}

/* 
 * mc_queued_stop() - queue a motor stop
 * mc_queued_start() - queue a motor start
 * mc_queued_end() - end current motion and program
 * mc_queued_start_stop_continue() - start and stop continuation
 *
 *	End should do all the following things (from NIST RS274NG_3)
 * 	Those we don't care about are in [brackets]
 *
 *	- Stop all motion once current block is complete 
 *		(as opposed to kill, which stops immediately)
 *	- Axes is set to zero (like G92)
 * 	- Selected plane is set to CANON_PLANE_XY (like G17).
 *	- Distance mode is set to MODE_ABSOLUTE (like G90).
 *	- Feed rate mode is set to UNITS_PER_MINUTE (like G94).
 * 	- [Feed and speed overrides are set to ON (like M48).
 *	- [Cutter compensation is turned off (like G40).
 *	- The spindle is stopped (like M5).
 *	- The current motion mode is set to G1
 *	- Coolant is turned off (like M9).
 */

int mc_queued_stop() 
{
	mc.move_type = MOVE_TYPE_STOP;
	mc.stop_continue_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_start() 
{
	mc.move_type = MOVE_TYPE_START;
	mc.stop_continue_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	mc.move_type = MOVE_TYPE_END;
	mc.stop_continue_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_start_stop_continue() 
{
	if (mc.stop_continue_state == MC_STATE_OFF) {
		return (TG_NOOP);
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);			  //...but instead you return	
	} else {
		mv_queue_start_stop(mc.move_type);
		mc.stop_continue_state = MC_STATE_OFF;
		return (TG_OK);
	}
}

/* 
 * mc_line() - queue a line move; non-blocking version
 * mc_line_continue() - continuation to generate and load a linear move
 *
 * Compute and queue a line segment to the move buffer.
 * Execute linear motion in absolute millimeter coordinates. 
 * Feed rate has already been converted to minutes
 *
 * Zero length lines are skipped at this level. 
 * The mv_queue doesn't check line length and queues anything.
 * 
 * The line generator (continuation) can be called multiple times until it
 * can successfully load the line into the move buffer.
 */

int mc_line(double x, double y, double z, double a, double minutes)
{
	mc.target[X] = lround(x * CFG(X).steps_per_unit);
	mc.target[Y] = lround(y * CFG(Y).steps_per_unit);
	mc.target[Z] = lround(z * CFG(Z).steps_per_unit); 
	mc.target[A] = lround(a * CFG(A).steps_per_unit); 

	mc.steps[X] = mc.target[X] - mc.position[X];
	mc.steps[Y] = mc.target[Y] - mc.position[Y];
	mc.steps[Z] = mc.target[Z] - mc.position[Z];
	mc.steps[A] = mc.target[A] - mc.position[A];

	// skip zero length moves
	if (!mc.steps[X] && !mc.steps[Y] && !mc.steps[Z] && !mc.steps[A]) {
		return (TG_ZERO_LENGTH_MOVE);
	}

	mc.microseconds = lround(minutes * ONE_MINUTE_OF_MICROSECONDS);
	mc.move_type = MOVE_TYPE_LINE;
	mc.line_continue_state = MC_STATE_NEW;
	memcpy(mc.position, mc.target, sizeof(mc.target)); 	// record new position
	return (mc_line_continue());
}

int mc_line_continue() 
{
	if (mc.line_continue_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // return NULL for non-started line
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);
	} else {
		mv_queue_line(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.steps[A], mc.microseconds);
		mc.line_continue_state = MC_STATE_OFF;
		return (TG_OK);
	}
}

/* 
 * mc_dwell() - queue a dwell (non-blocking behavior)
 * mc_dwell_continue() - dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers. When 
 * the stepper driver sees a dwell it times the move but does not send any 
 * pulses. Only the X axis is used to time the dwell - the otehrs are idle.
 */

int mc_dwell(double seconds) 
{
	mc.microseconds = trunc(seconds*1000000);
	mc.move_type = MOVE_TYPE_DWELL;
	mc.dwell_continue_state = MC_STATE_NEW;
	return (mc_dwell_continue());
}

int mc_dwell_continue() 
{
	if (mc.dwell_continue_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // returns NOOP for non-started dwell
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);			  //...but instead you return	
	} else {
		mv_queue_dwell(mc.microseconds); 
		mc.dwell_continue_state = MC_STATE_OFF;
		return (TG_OK);
	}
}

/* 
 * mc_arc() - execute an arc; non-blocking version
 * mc_arc_continue() - continuation inner loop to generate an arc
 *
 * Generates the line segments in an arc and queues them to the move buffer.
 * The arc is approximated by generating a huge number of tiny, linear
 * segments. The length of each segment is configured in config.h by 
 * setting MM_PER_ARC_SEGMENT.  
 *
 * Continue operation: 
 *	Continue is called initially by mc_arc. Continue will will either run to 
 *  arc completion (unlikely) or until the move buffer queue is full (likely).
 * 	It can then be re-entered to generate and queue the next segment(s) of arc.
 * 	Calling this function when there is no arc in process has no effect (NOOP).
 *
 * Note on mv_test_move_buffer_full()
 *	The move buffer is tested and sometime later it's queued (via mc_line())
 *	This only works because no ISRs queue this buffer, and this continuation 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */

int mc_arc(double theta, 			// starting angle
		   double radius, 			// radius of the circle in millimeters.
		   double angular_travel, 	// radians to go along arc (+ CW, - CCW)
		   double linear_travel, 
		   uint8_t axis_1, 			// select circle plane in tool space
		   uint8_t axis_2,  		// select circle plane in tool space
		   uint8_t axis_linear, 	// linear travel if helical motion
		   double minutes)			// time to complete the move
{
	double mm_of_travel = hypot(angular_travel * radius, labs(linear_travel));

	// load the arc persistence struct
	ma.theta = theta;
	ma.radius = radius;
	ma.axis_1 = axis_1;
	ma.axis_2 = axis_2;
	ma.axis_linear = axis_linear;
	
	if (mm_of_travel < cfg.mm_per_arc_segment) { // too short to draw
		return (TG_ZERO_LENGTH_MOVE);
	}
	ma.segments = ceil(mm_of_travel / cfg.mm_per_arc_segment);
 	mc.microseconds = lround((minutes / ma.segments) * ONE_MINUTE_OF_MICROSECONDS);
	ma.theta_per_segment = angular_travel / ma.segments;
	ma.linear_per_segment = linear_travel / ma.segments;
	ma.center_x = (mc.position[ma.axis_1] / CFG(ma.axis_1).steps_per_unit)
				 	- sin(ma.theta) * ma.radius;
	ma.center_y = (mc.position[ma.axis_2] / CFG(ma.axis_2).steps_per_unit) 
					- cos(ma.theta) * ma.radius;

  	// 	dtarget tracks the end point of each segment. Initialize linear axis
	ma.dtarget[ma.axis_linear] = mc.position[ma.axis_linear] / 
								 CFG(ma.axis_linear).steps_per_unit;

	// init settings for arc computation iteration and load
	ma.segment_counter=0;
	mc.move_type = MOVE_TYPE_LINE;
	mc.arc_continue_state = MC_STATE_RUNNING;
	return (mc_arc_continue());
}

int mc_arc_continue() 
{
	if (mc.arc_continue_state == MC_STATE_OFF) {
		return (TG_NOOP);				// return NULL for non-started arc
	} 
	while (ma.segment_counter <= ma.segments) {
		if (mv_test_move_buffer_full()) { // this is where you would block
			return (TG_EAGAIN);
		}
		// compute the line segment
		ma.segment_counter++;
		ma.theta += ma.theta_per_segment;
		ma.dtarget[ma.axis_1] = ma.center_x + sin(ma.theta) * ma.radius;
		ma.dtarget[ma.axis_2] = ma.center_y + cos(ma.theta) * ma.radius;
		ma.dtarget[ma.axis_linear] += ma.linear_per_segment;

		// setup and queue the line segment
		for (uint8_t i = X; i < AXES; i++) {
			mc.target[i] = lround(ma.dtarget[i] * CFG(i).steps_per_unit);
			mc.steps[i] = mc.target[i] - mc.position[i];
		}
		memcpy(mc.position, mc.target, sizeof(mc.target)); // record endpoint
		mv_queue_line(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.steps[A], 
					  mc.microseconds);
	}
	mc.arc_continue_state = MC_STATE_OFF; // arc is done. turn generator off
	return (TG_OK);
}
