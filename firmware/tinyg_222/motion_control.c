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
 * mc_line_accel() - queue line move with acceleration / deceleration
 * mc_line_accel_continue() - continuation 
 *
 * Algorithm at coarse grain: 
 *
 *	This code uses a cubic spline solution to generate acceleration and 
 *	deceleration ramps that obey maximum jerk parameters. The approach 
 *	and the motion equations were taken or derived from Ed Red's BYU  
 *	robotics course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *
 *	A line is divided into 3 regions:
 *	  - line head	initial acceleration/deceleration to target velocity
 *	  - line body	bulk of line at target speed (absent in come cases)
 *	  - line tail	ending acceleration/deceleration to exit velocity
 *
 *	The line head is computed from the exit velocity of the previous line
 *	or from zero velocity if fresh start. The head is queued as a set of 
 *	constant-time segments that implement the transition ramp. The time 
 *	constant is chosen to allow sufficient time between ramp segments for 
 *	the computation of the next segment and other processor tasks. 
 *
 *	The remainder of the line after the head is divided into a body and 
 *	a tail. The tail length is computed as the worst-case length required 
 *	to decelerate the line to zero velocity. 
 *
 *	Once the body and tail are computed the body is queued. The tail region
 *	is held back until the body is complete. 
 *
 *	The shape of the tail is set by the path control mode in effect:
 *
 *	  -	Exact Stop Mode: The line runs to zero velocity before the next 
 *		line is started. The entire tail region is used.
 *
 *	  - Exact Path Mode: The line is spliced to the next line with an 
 *		attempt to keep the path as accurate as possible. The splice 
 *		will compute a maximum jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerate the
 *		line to a computed "safe" velocity before accelerating into the
 *		next line. For 180 degree turns the line will stop before reversing.
 *
 *	  - Continuous Mode: The splice will attempt to run at the maximum 
 *		theoretical rate, accelerating or decelerating between lines. The
 *		velodity at the join point will always equal half the difference 
 *		between the two velocities (plus the smaller). In the future this
 *		mode should also spline the lines to round the corners - which it
 *		does not currently do.
 *
 *	The following cases apply if the line is joining to an arc:
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc. The tail is reserved, as usual.
 *
 *	  - Line is follwed by an arc: The tail is used to accelerate or
 *		decelerate to match the arc feed rate. (see arc routines)
 *
 *	  - Arc to arc splining: is not supported. A velocity step may occur
 *
 * Notes on algorithm at fine grain:
 *
 *	The main routine / continuation sequencing is weird:
 *
 *	The main routine is called whenever there is a new line. It looks at
 *	the path control mode and sets parameters for the previous tail, the 
 *	head, and the body. It also reserves etime / distance for the tail and 
 *	computes an Exact Stop Mode tail
 *
 *	The continuation executes the (previous) tail, the head, then the body,
 *	Then returns OK - permitting the next line to be received. If a new
 *	line is received it may overwrite the tail; otherwise the Exact Stop
 *	tail will be computed and run to a stop.
 *
 *	Special cases:
 *	  - If no previous tail exists, one is not executed - it's skipped, and 
 *		the new line (head) starts from zero velocity.
 *
 *	Comments on the routine:
 *	  - On entry, the following values reflect the previous line, or inits:
 *		- ml.velocity_initial	- target or actual velocity of previous line
 *		- mc.position[]			- position at end of previous line (or arc) 
 */

/*
 * Local Scope Data and Functions
 */

enum mcGeneratorState {
	MC_STATE_OFF,				// generator is OFF
	MC_STATE_NEW,				// initial call to generator
	MC_STATE_RUNNING,			// in process, needs re-entry (continuation)
	MC_STATE_RUNNING_HEAD,		// states for acceleration / deceleration
	MC_STATE_RUNNING_BODY,
	MC_STATE_RUNNING_TAIL,
	MC_STATE_MAX
};

struct MotionControlCommon {		// robot pos & vars used by lines and arcs
	uint8_t move_type;				// type of move. See mvType
	volatile uint8_t line_state;	// line generator state
	volatile uint8_t dwell_state;	// dwell generator state
	volatile uint8_t stop_state;	// start/stop state
	volatile uint8_t arc_state;		// arc generator state

	// geometry in floating point
	double position[AXES];			// previous position in floating point
	double target[AXES];			// target position in floating point
	double mm_of_travel;			// total travel of linear move in mm

	// geometry in stepper steps
	int32_t steps_position[AXES];   // current position of tool in abs steps
	int32_t steps_target[AXES];		// target position of tool in abs steps
	int32_t steps[AXES];			// target line in relative steps
	uint32_t microseconds;			// target move microseconds
};

struct MotionControlArc {			// continuation values used by arc
	uint8_t axis_1;					// arc plane axis
	uint8_t axis_2;					// arc plan axis
	uint8_t axis_linear;			// transverse axis (helical)
	uint16_t segments;				// number of segments in arc
	uint16_t segment_counter;		// number of segments queued so far

	double theta;					// total angle specified by arc
	double radius;					// computed via offsets
	double center_x;				// center of this circle
	double center_y;				// center of this circle
	double theta_per_segment;		// angular motion per segment
	double linear_per_segment;		// linear motion per segment
};

struct MotionControlLine {			// continuationvalues used by accel lines
	double velocity_initial;		// velocity at end of previous line
	double velocity_target;			// target velocity for this line
	double velocity_delta;			// change in velocity

	double line_length;				// total line length in mm
	double line_time;

	double head_length;				// head length in mm
	double head_time;				// head time in minutes
	uint16_t head_segments;			// number of segments in head
	uint16_t head_seg_time;			// time per segment in microseconds

	double body_length;				// body length in mm
	double body_time;				// body time in minutes

	double tail_length;				// tail length in mm
	double tail_time;				// tail time in minutes
	uint16_t tail_segments;			// number of segments in head
	uint16_t tail_seg_time;			// time per segment in microseconds

	double unit_vector[AXES];		// unit vector for scaling distance to axes
};

static struct MotionControlCommon mc;
static struct MotionControlArc ma;
static struct MotionControlLine ml;

int mc_line_accel(double x, double y, double z, double a, double minutes)
{
//	PATH_CONTROL_MODE_EXACT_STOP
//	PATH_CONTROL_MODE_EXACT_PATH
//	PATH_CONTROL_MODE_CONTINUOUS

	// compute basic values
	mc.target[X] = x;				// target in floating point 
	mc.target[Y] = y;
	mc.target[Z] = z;
	mc.target[A] = a;
	ml.line_time = minutes;

	ml.line_length = sqrt(square(mc.target[X] - mc.position[X]) +
						  square(mc.target[Y] - mc.position[Y]) +
						  square(mc.target[Z] - mc.position[Z]));
	
	for (uint8_t i=0; i < AXES; i++) {
		ml.unit_vector[i] = (mc.target[i] - mc.position[i]) / ml.line_length;
	}
	// compute the velocities
	ml.velocity_target = ml.line_length / minutes;
	ml.velocity_delta = fabs(ml.velocity_target - ml.velocity_initial);

	// optimal time in the head is given by the jerk equation (5.x - derived)
	ml.head_time = 2 * sqrt(ml.velocity_delta / cfg.max_linear_jerk);

	// position at end of head from classic position equation (2)
	ml.head_length = 0.5 * ml.velocity_delta * ml.head_time;

	// special case where line is not long enough to reach full velocity
	if ((2 * ml.head_length) > ml.line_length) {
		ml.velocity_target *= (ml.line_time / (2 * ml.head_time)); // scale V down
		ml.head_length = ml.line_length / 2;	// best you can do...
		ml.head_time = ml.line_time / 2;		//...under the circumstances
	}
	ml.tail_length = ml.head_length;
	ml.tail_time = ml.head_time;
	ml.body_length = ml.line_length - ml.tail_length - ml.head_length;
	ml.body_time = ml.line_time * (ml.body_length / ml.line_length);

	ml.head_segments = round(uSec(ml.head_time) / cfg.min_segment_time);
	ml.head_seg_time = round(uSec(ml.head_time / ml.head_segments));
	ml.tail_segments = round(uSec(ml.tail_time) / cfg.min_segment_time);
	ml.tail_seg_time = round(uSec(ml.tail_time / ml.tail_segments));

	mc.move_type = MOVE_TYPE_LINE;
	mc.line_state = MC_STATE_RUNNING_TAIL;
	memcpy(mc.steps_position, mc.steps_target, sizeof(mc.steps_target));// new step position
	return (mc_line_accel_continue());
}

int mc_line_accel_continue()
{
	if (mc.line_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // return NULL for non-started line
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);
	}

	// run the previous tail
		


	mv_queue_line(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.steps[A], mc.microseconds);
	mc.line_state = MC_STATE_OFF;
	return (TG_OK);
}









/* 
 * mc_init() 
 */

void mc_init()
{
	clear_vector(mc.steps_position);			// zero robot position
	clear_vector(mc.position);			// zero robot position

	mc.line_state = MC_STATE_OFF;	// turn off the generators
	mc.dwell_state = MC_STATE_OFF;
	mc.stop_state = MC_STATE_OFF;
	mc.arc_state = MC_STATE_OFF;
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
	if (mc.line_state || mc.dwell_state || mc.stop_state || mc.arc_state) {
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
	mc.position[X] = x;
	mc.position[Y] = y;
	mc.position[Z] = z; 
	mc.position[A] = a; 

	mc.steps_position[X] = round(x * CFG(X).steps_per_unit);
	mc.steps_position[Y] = round(y * CFG(Y).steps_per_unit);
	mc.steps_position[Z] = round(z * CFG(Z).steps_per_unit); 
	mc.steps_position[A] = round(z * CFG(Z).steps_per_unit); 

	return (TG_OK);
}

/* 
 * mc_async_stop() - stop current motion immediately
 * mc_async_start() - (re)start motion
 * mc_async_end() - stop current motion immediately
 *
 *	These routines must be safe to call from ISRs
 *	Mind the volatiles.
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
	mc.line_state = MC_STATE_OFF;	// kill the generators
	mc.dwell_state = MC_STATE_OFF;
	mc.stop_state = MC_STATE_OFF;
	mc.arc_state = MC_STATE_OFF;
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
	mc.stop_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_start() 
{
	mc.move_type = MOVE_TYPE_START;
	mc.stop_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	mc.move_type = MOVE_TYPE_END;
	mc.stop_state = MC_STATE_NEW;
	return (mc_queued_start_stop_continue());
}

int mc_queued_start_stop_continue() 
{
	if (mc.stop_state == MC_STATE_OFF) {
		return (TG_NOOP);
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);			  //...but instead you return	
	} else {
		mv_queue_start_stop(mc.move_type);
		mc.stop_state = MC_STATE_OFF;
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
	mc.steps_target[X] = round(x * CFG(X).steps_per_unit);
	mc.steps_target[Y] = round(y * CFG(Y).steps_per_unit);
	mc.steps_target[Z] = round(z * CFG(Z).steps_per_unit); 
	mc.steps_target[A] = round(a * CFG(A).steps_per_unit); 

	mc.steps[X] = mc.steps_target[X] - mc.steps_position[X];
	mc.steps[Y] = mc.steps_target[Y] - mc.steps_position[Y];
	mc.steps[Z] = mc.steps_target[Z] - mc.steps_position[Z];
	mc.steps[A] = mc.steps_target[A] - mc.steps_position[A];

	// skip zero length moves
	if (!mc.steps[X] && !mc.steps[Y] && !mc.steps[Z] && !mc.steps[A]) {
		return (TG_ZERO_LENGTH_MOVE);
	}

	mc.microseconds = round(minutes * ONE_MINUTE_OF_MICROSECONDS);
	mc.move_type = MOVE_TYPE_LINE;
	mc.line_state = MC_STATE_NEW;
	memcpy(mc.steps_position, mc.steps_target, sizeof(mc.steps_target)); 	// record new position
	return (mc_line_continue());
}

int mc_line_continue() 
{
	if (mc.line_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // return NULL for non-started line
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);
	} else {
		mv_queue_line(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.steps[A], mc.microseconds);
		mc.line_state = MC_STATE_OFF;
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
	mc.dwell_state = MC_STATE_NEW;
	return (mc_dwell_continue());
}

int mc_dwell_continue() 
{
	if (mc.dwell_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // returns NOOP for non-started dwell
	}
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);			  //...but instead you return	
	} else {
		mv_queue_dwell(mc.microseconds); 
		mc.dwell_state = MC_STATE_OFF;
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
 	mc.microseconds = round((minutes / ma.segments) * ONE_MINUTE_OF_MICROSECONDS);
	ma.theta_per_segment = angular_travel / ma.segments;
	ma.linear_per_segment = linear_travel / ma.segments;
	ma.center_x = (mc.steps_position[ma.axis_1] / CFG(ma.axis_1).steps_per_unit)
				 	- sin(ma.theta) * ma.radius;
	ma.center_y = (mc.steps_position[ma.axis_2] / CFG(ma.axis_2).steps_per_unit) 
					- cos(ma.theta) * ma.radius;

  	// 	target tracks the end point of each segment. Initialize linear axis
	mc.target[ma.axis_linear] = mc.steps_position[ma.axis_linear] / 
								 CFG(ma.axis_linear).steps_per_unit;

	// init settings for arc computation iteration and load
	ma.segment_counter=0;
	mc.move_type = MOVE_TYPE_LINE;
	mc.arc_state = MC_STATE_RUNNING;
	return (mc_arc_continue());
}

int mc_arc_continue() 
{
	if (mc.arc_state == MC_STATE_OFF) {
		return (TG_NOOP);				// return NULL for non-started arc
	} 
	while (ma.segment_counter <= ma.segments) {
		if (mv_test_move_buffer_full()) { // this is where you would block
			return (TG_EAGAIN);
		}
		// compute the line segment
		ma.segment_counter++;
		ma.theta += ma.theta_per_segment;
		mc.target[ma.axis_1] = ma.center_x + sin(ma.theta) * ma.radius;
		mc.target[ma.axis_2] = ma.center_y + cos(ma.theta) * ma.radius;
		mc.target[ma.axis_linear] += ma.linear_per_segment;

		// setup and queue the line segment
		for (uint8_t i = X; i < AXES; i++) {
			mc.steps_target[i] = round(mc.target[i] * CFG(i).steps_per_unit);
			mc.steps[i] = mc.steps_target[i] - mc.steps_position[i];
		}
		memcpy(mc.steps_position, mc.steps_target, sizeof(mc.steps_target)); // record endpoint
		mv_queue_line(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.steps[A], 
					  mc.microseconds);
	}
	mc.arc_state = MC_STATE_OFF; // arc is done. turn generator off
	return (TG_OK);
}
