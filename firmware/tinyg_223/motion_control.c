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
#include "motor_queue.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

uint8_t _mc_run_line(struct mcMotionControl *m);
uint8_t _mc_run_aline(struct mcMotionControl *m);
uint8_t _mc_run_start_stop(struct mcMotionControl *m);
uint8_t _mc_run_dwell(struct mcMotionControl *m);
uint8_t _mc_run_arc(struct mcMotionControl *m);
void _mc_set_endpoint_position(struct mcMotionControl *m);

#define MC_BUFFER_SIZE 3		// queued moves (limited to 255)
#define mc_bump(a) ((a<MC_BUFFER_SIZE-1)?(a+1):0)	// buffer incr & wrap
#define return_if_motor_buffer_full(a)	if (!mq_test_motor_buffer()) { return (a); }

// There are 2 distinct state machines that must be kept straight

enum mcBufferState {		// state of motion control struct
	MOVE_BUFFER_EMPTY = 0,	// struct is available for use
	MOVE_BUFFER_LOADING,	// being written
	MOVE_BUFFER_WAITING,	// in queue
	MOVE_BUFFER_RUNNING		// current running move
};

enum mcMoveState {			// applicable to any type of queued move
	MOVE_STATE_OFF = 0,		// move is OFF for whatever reason (don't run it)
	MOVE_STATE_NEW,			// <-- value on initial call to run routine
	MOVE_STATE_RUNNING,
	// states to process an aline:
	MOVE_STATE_HEAD_A1,		// concave acceleration period in head
	MOVE_STATE_HEAD_A2,		// convex acceleration period in head
	MOVE_STATE_HEAD_D1,		// convex deceleration period in head  
	MOVE_STATE_HEAD_D2,		// concave deceleration period in head
	MOVE_STATE_BLEND_A1,	// concave acceleration period in blend
	MOVE_STATE_BLEND_A2,	// convex acceleration period in blend
	MOVE_STATE_BLEND_D1,	// convex deceleration period in blend  
	MOVE_STATE_BLEND_D2,	// concave deceleration period in blend
	MOVE_STATE_BODY,		// cruise period - may not exist
	MOVE_STATE_TAIL,		// prep for tail or invoke a blend
	MOVE_STATE_TAIL_D1,		// convex deceleration period in tail  
	MOVE_STATE_TAIL_D2		// concave deceleration period in tail
};

struct mcMotionControl {		// robot pos & vars used by lines and arcs
	// buffer and flow control
	uint8_t buffer_state;		// mcBufferState
	uint8_t move_type;			// mvType
	uint8_t move_state;			// mcMoveState
	struct mcMotionControl *next; // static pointer to next buffer in ring

	// common variables
	double dtarget[AXES];		// target position in floating point
	int32_t target[AXES];		// target position in absolute steps
	int32_t steps[AXES];		// target position in relative steps
	double unit_vector[AXES];	// unit vector for scaling axes
	double prev_vector[AXES];	// unit vector for previous line

	double angular_jerk;
	double linear_jerk_div2;	// saves a few cycles even w/full optimization
	uint32_t microseconds;		// uSec of target move (a convenience)

	double move_length;			// line or helix length in mm
	double move_time;			// line or helix or dwell time in minutes

	// blend variables (acceleration / deceleration)
	uint32_t segments;			// number of segments in arc or blend
	uint32_t segment_count;		// number of segments queued (run) so far
	double segment_time;		// constant time per aline segment
	double elapsed_time;		// running time for each aline region
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment

	double velocity_initial;	// velocity at end of previous line
	double velocity_midpoint;	// velocity at the transition midpoint
	double velocity_target;		// target velocity for this line
	double velocity_delta;		// change in velocity
	double acceleration_midpoint; // acceleration at the half

	double head_length;			// head region for aline
	double head_time;
	double body_length;			// cruise region for aline
	double body_time;
	double tail_length;			// tail region for aline
	double tail_time;
	double prev_tail_length;	// previous line's tail length in mm
	double prev_tail_time;
	uint8_t prev_move_end_state; // used for blending (or not)

	// arc variables (that are not already captured above)
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double center_x;			// center of circle
	double center_y;			// center of circle
	double theta_per_segment;	// angular motion per segment
	double linear_per_segment;	// linear motion per segment
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mcMotionControlMaster {
	uint8_t (*run_move)(struct mcMotionControl *m);	// currently running move
	uint8_t run_flag;							// status of controller

	// persistent position info
	double dposition[AXES];		// current position in floating point
	int32_t position[AXES];   	// current position in steps

	// ring buffer for queueing and processing moves
	struct mcMotionControl *r;	// pointer to running move
	struct mcMotionControl *w;	// pointer to move being loaded
	struct mcMotionControl b[MC_BUFFER_SIZE];// buffer storage
};
static struct mcMotionControlMaster mm;

/* 
 * mc_init()
 *
 * The memset does:
 *	- clears all values
 *	- sets all buffer states to AVAILABLE
 *	- sets all move states to OFF
 */

void mc_init()
{
	memset(&mm, 0, sizeof(mm));		// clear all values, pointers and status
	mm.w = &mm.b[0];				// init write and read buffer pointers
	mm.r = &mm.b[0];
	for (uint8_t i=0; i < MC_BUFFER_SIZE; i++) {	// set the ring pointers
		mm.b[i].next = &mm.b[mc_bump(i)];
	}
}

/* 
 * mc_move_controller() - routine for dequeuing and executing moves
 *
 *	Dequeues the buffer queue and runs the individual move continuations.
 *	Manages run buffers and other details.
 *	Runs as a continuation itself; called from tg_controller()
 */

uint8_t mc_move_controller() 
{
	uint8_t status;

	if (mc_get_run_buffer() == NULL) {	// NULL means nothing's running
		return (TG_NOOP);
	}
	if (mm.r->move_state == MOVE_STATE_NEW) {	// first time in?
		mm.run_flag = TRUE;						// it's useful to have a flag
		switch (mm.r->move_type) { 				// setup the dispatch vector
			case MOVE_TYPE_LINE:  { mm.run_move = _mc_run_line; break; }
			case MOVE_TYPE_ALINE: { mm.run_move = _mc_run_aline; break; }
			case MOVE_TYPE_ARC:   { mm.run_move = _mc_run_arc; break; }
			case MOVE_TYPE_DWELL: { mm.run_move = _mc_run_dwell; break; }
			case MOVE_TYPE_STOP:  { mm.run_move = _mc_run_start_stop; break; }
		}
	}
	if ((status = mm.run_move(mm.r)) == TG_EAGAIN) { // run current run buffer
		return (status);
	}
	mm.run_flag = FALSE;				// finalize and return
	mc_end_run_buffer();
	return (status);
}

/**** MOVE QUEUE ROUTINES ****
 * mc_test_write_buffer() 	- test if a write buffer is available
 * mc_get_write_buffer() 	- get pointer to available write buffer
 * mc_commit_write_buffer() - commit the write buffer to the queue
 * mc_get_run_buffer()		- get the current run buffer
 * mc_end_run_buffer()		- release and return the run buffer
 *
 * These 5 routines control the move queue. The sequence is:
 *	0 - test if you can get a write buffer (this is optional).
 *	1 - get a free write buffer
 *	2 - commit the write buffer to the queue
 *	3 - get a run buffer to execute the move. You can call this as 
 *		many times as you need to service the move. (but mm.r is persistent)
 *	4 - end the run buffer when the move is complete (free the buffer)
 *
 *	The pointers only move forward on commit and end calls (not test & get)
 * 	The get's return a NULL pointer if the request can't be satisfied.
 *
 *	Write commits require a move type arg as this is always needed.
 *	Do not commit a failed get_write, and do not end a failed run buffer.
 *
 * 	The routines also set mm.w and mm.r so you can just use mm.w and mm.r 
 *	if you want, but you still have to check for a NULL on return. 
 *	mm.w and mm.r are never nulled.
 *
 *	You must remember to commit write buffers and end run buffers of this 
 *	all fails. Usually this is done at the end of the routine that gets the
 *	buffer.
 */

uint8_t mc_test_write_buffer() 
{
	if (mm.w->buffer_state == MOVE_BUFFER_EMPTY) {
		return (TRUE);
	}
	return (FALSE);
}

struct mcMotionControl * mc_get_write_buffer() 
{
	if (mm.w->buffer_state == MOVE_BUFFER_EMPTY) {
		mm.w->buffer_state = MOVE_BUFFER_LOADING;
		return (mm.w);
	}
	return (NULL);
}

uint8_t mc_commit_write_buffer(uint8_t move_type)
{
	mm.w->move_type = move_type;
	mm.w->move_state = MOVE_STATE_NEW;
	mm.w->buffer_state = MOVE_BUFFER_WAITING;
	mm.w = mm.w->next;
	return (TG_OK);		// this is a convenience for calling routines
}

struct mcMotionControl * mc_get_run_buffer() 
{
	// condition: buffer was ended, buffer becomes running if it's waiting
	if (mm.r->buffer_state == MOVE_BUFFER_WAITING) {
		mm.r->buffer_state = MOVE_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mm.r->buffer_state == MOVE_BUFFER_RUNNING) { // return the same buffer
		return (mm.r);
	}
	// condition: no waiting buffers. fail it.
	return (NULL);
}

uint8_t mc_end_run_buffer()
{
	mm.r->buffer_state = MOVE_BUFFER_EMPTY;
	mm.r = mm.r->next;						// always advance
	return (TG_OK);		// this is a convenience for calling routines
}

/* 
 * mc_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mc_isbusy()
{
	if (st_isbusy() || mm.run_flag) {
		return (TRUE);
	}
	return (FALSE);
}

/* 
 * mc_set_position() - set current position (support for G92)
 *
 *	Note: Position values are global, not in any given move buffer
 */

uint8_t mc_set_position(double x, double y, double z, double a)
{
	mm.dposition[X] = x;
	mm.dposition[Y] = y;
	mm.dposition[Z] = z; 
	mm.dposition[A] = a; 

	mm.position[X] = round(x * CFG(X).steps_per_unit);
	mm.position[Y] = round(y * CFG(Y).steps_per_unit);
	mm.position[Z] = round(z * CFG(Z).steps_per_unit); 
	mm.position[A] = round(z * CFG(A).steps_per_unit); 

	return (TG_OK);
}

/* 
 * _mc_set_endpoint_position() - copy target to position in both systems
 *
 * Note: As far as the motion control layer is concerned the final position 
 *	is achieved as soon at the move is executed and the position is now 
 *	the target. In reality, motion_control / steppers will still be 
 *	processing the action and the real tool position is still close to 
 *	the starting point. 
 */

void _mc_set_endpoint_position(struct mcMotionControl *m) 
{ 
	memcpy(mm.dposition, m->dtarget, sizeof(mm.dposition));	// floats
	memcpy(mm.position, m->target, sizeof(mm.position)); 	// steps
}

/* 
 * mc_async_stop() - stop current motion immediately
 * mc_async_start() - (re)start motion
 * mc_async_end() - stop current motion immediately
 *
 *	These routines must be safe to call from ISRs
 *	Mind the volatiles.
 */

uint8_t mc_async_stop()
{
	st_stop();						// stop the steppers
	return (TG_OK);
}

uint8_t mc_async_start()
{
	st_start();						// start the stoppers
	return (TG_OK);
}

uint8_t mc_async_end()
{
	st_end();						// stop the motion
//	mm.line_generator_state = MC_GENERATOR_OFF;	// kill the generators
//	mm.dwell_generator_state = MC_GENERATOR_OFF;
//	mm.stop_generator_state = MC_GENERATOR_OFF;
//	mm.arc_generator_state = MC_GENERATOR_OFF;
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

uint8_t mc_queued_stop() 
{
	if (mc_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mc_commit_write_buffer(MOVE_TYPE_STOP));
}

uint8_t mc_queued_start() 
{
	if (mc_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mc_commit_write_buffer(MOVE_TYPE_START));
}

uint8_t mc_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	if (mc_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mc_commit_write_buffer(MOVE_TYPE_END));
}

uint8_t _mc_run_start_stop(struct mcMotionControl *m) 
{
	return_if_motor_buffer_full(TG_EAGAIN);
	mq_queue_start_stop(m->move_type);
	return (TG_OK);
}

/* 
 * mc_dwell() - queue a dwell (non-blocking behavior)
 * mc_dwell_continue() - dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers. When 
 * the stepper driver sees a dwell it times the move but does not send any 
 * pulses. Only the X axis is used to time the dwell - the otehrs are idle.
 */

uint8_t mc_dwell(double seconds) 
{
	struct mcMotionControl *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	m->move_time = seconds / 60;				// convert to minutes
	return (mc_commit_write_buffer(MOVE_TYPE_DWELL));
}

uint8_t _mc_run_dwell(struct mcMotionControl *m)
{
	return_if_motor_buffer_full(TG_EAGAIN);
	mq_queue_dwell(uSec(m->move_time)); 
	return (TG_OK);
}

/* 
 * mc_line() 	  - queue a linear move (simple version - no accel/decel)
 * _mc_run_line() - run a line to generate and load a linear move
 *
 * Compute and queue a line segment to the move buffer.
 * Executes linear motion in absolute millimeter coordinates. 
 * Feed rate has already been converted to time (minutes).
 * Zero length lines are skipped at this level. 
 * The mq_queue doesn't check line length and queues anything.
 * 
 * The run_line routine is a continuation and can be called multiple times 
 * until it can successfully load the line into the move buffer.
 */

uint8_t mc_line(double x, double y, double z, double a, double minutes)
{
	struct mcMotionControl *m; 
	uint8_t zed_test=0;

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	m->dtarget[X] = x;
	m->dtarget[Y] = y;
	m->dtarget[Z] = z;
	m->dtarget[A] = a;
	m->move_time = minutes;

	for (uint8_t i=0; i < AXES; i++) {
		m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
		m->steps[i] = m->target[i] - mm.position[i];
		zed_test += m->steps[i];
	}
	if (zed_test == 0) {			// skip zero length moves
		return (TG_ZERO_LENGTH_MOVE);
	}
	return(mc_commit_write_buffer(MOVE_TYPE_LINE));
}

uint8_t _mc_run_line(struct mcMotionControl *m) 
{
	m->move_state = MOVE_STATE_RUNNING;

	return_if_motor_buffer_full(TG_EAGAIN);
	mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], 
				  uSec(m->move_time));
	_mc_set_endpoint_position(m);
	return (TG_OK);
}

/* 
 * mc_aline() 		- queue line move with acceleration / deceleration
 * _mc_run_aline()	- run accel move 
 *
 * Algorithm at coarse grain: 
 *
 *	This code uses a cubic spline solution to generate acceleration and 
 *	deceleration ramps that obey maximum jerk parameters. The approach 
 *	and the motion equations were taken or derived from Ed Red's BYU  
 *	robotics course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *
 *	A line (or arc) is divided into 3 regions:
 *	  - head	initial acceleration/deceleration to target velocity
 *	  - body	bulk of move at target speed (absent in come cases)
 *	  - tail	ending acceleration/deceleration to exit velocity
 *
 *	The head is computed from the exit velocity of the previous move
 *	or from zero velocity if fresh start. The head is queued as a set of 
 *	constant-time segments that implement the transition ramp. The time 
 *	constant is chosen to allow sufficient time between ramp segments for 
 *	the computation of the next segment and for other processor tasks. 
 *
 *	The remainder of the move after the head is divided into a body and 
 *	a tail. The tail length is a reserved region that allows for the 
 *	worst-case length required to decelerate the line to zero velocity. 
 *
 *	Once the body and tail are computed the body is queued. The tail region
 *	is held back until the body is complete, and may be used by the current 
 *	running line as a stop, or by the next move as a blend region.
 *
 *	The shape of the tail is set by the path control mode in effect:
 *
 *	  -	Exact Stop Mode: The move runs to zero velocity before the next 
 *		move is started. The entire tail region is used.
 *
 *	  - Exact Path Mode: The move is spliced to the next move with an 
 *		attempt to keep the path as accurate as possible. The splice 
 *		computes a maximum jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerates the
 *		line to a computed "safe" velocity before accelerating into the
 *		next line. For 180 degree turns the line will stop before reversing.
 *
 *	  - Continuous Mode: The splice will attempt to run at the maximum 
 *		theoretical rate, accelerating or decelerating between lines. The
 *		velocity at the join point will always equal half the difference 
 *		between the two velocities (plus the smaller). In the future this
 *		mode should also spline the lines to round the corners - which it
 *		does not currently do.
 *
 *	The following cases apply if the line is joining to an arc:
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc. The tail is reserved, as usual.
 *
 *	  - Line is followed by an arc: The tail is used to accelerate or
 *		decelerate to match the arc feed rate.
 *
 *	  - Arc to arc splining: is not currently supported. 
 *		A velocity step may occur between arcs of different speeds. This
 *		will also occur if an arc is started or stopped from vero velocity.
 *
 * Notes on algorithm at fine grain:
 *
 *	The main routine is called whenever there is a new line. It gathers 
 *	or computes all initial parameters and queues the move. It also 
 *	writes a bunch of stuff into the next move buffer to setup for blends.
 *
 *	The move waits in queue and is run in turn. The run performs a blend 
 *	to the previous line or a start from zero if there is no move to blend.
 *	It then computes a body and a tail region. The tail reserves a 
 *	worst-case time for the move to decelerate to zero (exact stop).
 *
 *	The run executes the head region first - performing the blend
 *	with the previous move (which wrote all the parameters needed for 
 *	the blend into the run struct). If there is no move to blend it 
 *	does a start from zero velocity.
 *	
 *	It then queues the body (cruise region). 
 *
 *	It then looks to see if there is another line queued behind it. 
 *	If there is if copies its parameters into the "previous tail" vars
 *	of the next line and exits.
 *
 *	If there is not another line queued it executes the tail by 
 *	decelerating to zero (exact stop).
 *
 *	Note: Ideally the run should wait for the body to almost complete 
 *	before looking for the next line. Might do this later.
 *
 *	Comments on the functions
 *	- All math is done in floating point and minutes until the very end,
 *	  when it's converted to steps and microseconds for queueing motor moves
 *
 *	- On entry the following values reflect the previous line:
 *		mm.dposition[]			- position at end of previous line (or arc)
 *		m->velocity_initial		- target or actual velocity of previous line
 *		m->prev_vector[] 		- unit vector of previous line
 *		m->prev_tail_length		- tail length of previous line
 *		m->prev_tail_time		- tail time of previous line
 */

uint8_t mc_aline(double x, double y, double z, double a, double minutes)
{
//	PATH_CONTROL_MODE_EXACT_STOP
//	PATH_CONTROL_MODE_EXACT_PATH
//	PATH_CONTROL_MODE_CONTINUOUS

	struct mcMotionControl *m; 					// write buffer pointer
	struct mcMotionControl *nx; 				// pointer to next buffer

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	nx = m->next;

	// capture the args
	m->dtarget[X] = x;
	m->dtarget[Y] = y;
	m->dtarget[Z] = z;
	m->dtarget[A] = a;
	m->move_time = minutes;

	// compute initial values
	m->move_length = sqrt(square(m->dtarget[X] - mm.dposition[X]) +
						  square(m->dtarget[Y] - mm.dposition[Y]) +
						  square(m->dtarget[Z] - mm.dposition[Z]));

	// create a unit vector for scaling segments and finding angular jerk
	for (uint8_t i=0; i < AXES; i++) {
		m->unit_vector[i] = (m->dtarget[i] - mm.dposition[i]) / m->move_length;
		nx->prev_vector[i] = m->unit_vector[i];	// (used by next line)
	}
	// compute the velocities
	m->velocity_target = m->move_length / m->move_time;
	m->velocity_delta = fabs(m->velocity_target - m->velocity_initial);
	m->velocity_midpoint =  (m->velocity_target + m->velocity_initial) / 2;
	nx->velocity_initial = m->velocity_target;	// (for the next line)

	// optimal time in the head is given by the jerk equation (5.x - derived)
	m->head_time = 2 * sqrt(m->velocity_delta / cfg.max_linear_jerk);
	m->head_length = m->velocity_delta * m->head_time / 2; // classic mechanics
	m->linear_jerk_div2 = cfg.max_linear_jerk / 2;	// saves cycles even w/Os
	m->acceleration_midpoint = m->linear_jerk_div2 * m->head_time;	// useful

	// special case where line is not long enough to reach cruise velocity
	if ((2 * m->head_length) > m->move_length) {
		m->velocity_target *= (m->move_time / (2 * m->head_time)); // scale V down
		m->head_length = m->move_length / 2;	// best you can do...
		m->head_time = m->move_time / 2;		//...under the circumstances
	}

	// compute the rest of the variables needed for running the line
	m->tail_length = m->head_length;
	m->tail_time   = m->head_time;
	m->body_length = m->move_length - m->tail_length - m->head_length;
	m->body_time   = m->move_time * (m->body_length / m->move_length);
	nx->prev_tail_length = m->tail_length;
	nx->prev_tail_time = m->tail_time;

	return(mc_commit_write_buffer(MOVE_TYPE_ALINE)); // also sets ...STATE_NEW
}

uint8_t _mc_run_aline(struct mcMotionControl *m)
{
	// initialize for head acceleration
	if (m->move_state == MOVE_STATE_NEW) {
		if (m->velocity_target > m->velocity_initial) {
			m->move_state = MOVE_STATE_HEAD_A1;	// run a simple head
			m->elapsed_time = 0;
			m->segments = uSec(m->head_time / cfg.min_segment_time);
			m->segment_time = m->head_time / m->segments;
			m->segment_count = m->segments / 2;
			m->microseconds = uSec(m->segment_time);
		} else {
			// initial deceleration is not implemented yet
		}
	}
	// first half of a head acceleration
	if (m->move_state == MOVE_STATE_HEAD_A1) {	// concave portion of curve
		while (m->segment_count) {
//			if (!mq_test_motor_buffer()) { 
//				return (TG_EAGAIN); 
//			}
			return_if_motor_buffer_full(TG_EAGAIN);
			m->segment_count--;
			// compute the acceleration segment and setup and queue the line segment
			m->elapsed_time += m->segment_time;
			m->segment_velocity = m->velocity_initial + 
								 (m->linear_jerk_div2 * square(m->elapsed_time));
			for (uint8_t i=0; i < AXES; i++) {
				m->dtarget[i] = mm.dposition[i] + m->unit_vector[i] * 
							    m->segment_velocity * m->segment_time;
				m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
				m->steps[i] = m->target[i] - mm.position[i];
			}
			mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
			_mc_set_endpoint_position(m);
		}
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->move_state = MOVE_STATE_HEAD_A2;
	}
	// second half of a head acceleration
	if (m->move_state == MOVE_STATE_HEAD_A2) {	// convex portion of curve
		while (m->segment_count) {
			return_if_motor_buffer_full(TG_EAGAIN);
			m->segment_count--;
			m->elapsed_time += m->segment_time;
			m->segment_velocity = m->velocity_midpoint + 
								 (m->elapsed_time * m->acceleration_midpoint) -
								 (m->linear_jerk_div2 * square(m->elapsed_time));
			for (uint8_t i=0; i < AXES; i++) {
				m->dtarget[i] = mm.dposition[i] + m->unit_vector[i] * 
							    m->segment_velocity * m->segment_time;
				m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
				m->steps[i] = m->target[i] - mm.position[i];
			}
			mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
			_mc_set_endpoint_position(m);
		}
		m->move_state = MOVE_STATE_BODY;
	}
	// body region (cruise region)
	if (m->move_state == MOVE_STATE_BODY) {
		return_if_motor_buffer_full(TG_EAGAIN);
		for (uint8_t i=0; i < AXES; i++) {
			m->dtarget[i] = mm.dposition[i] + m->unit_vector[i] * m->body_length;
			m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
			m->steps[i] = m->target[i] - mm.position[i];
		}
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], 
					  uSec(m->body_time));
		_mc_set_endpoint_position(m);
		m->move_state = MOVE_STATE_TAIL;	
	}

	// initialize for tail deceleration
	if (m->move_state == MOVE_STATE_TAIL) {
		m->move_state = MOVE_STATE_TAIL_D1;
		m->next->prev_move_end_state = MOVE_STATE_TAIL; // needed for blend

		m->velocity_initial = m->velocity_target;
		m->velocity_target = 0;
		m->elapsed_time = 0;

		m->segments = uSec(m->tail_time / cfg.min_segment_time);
		m->segment_time = m->tail_time / m->segments;
		m->segment_count = m->segments / 2;
		m->microseconds = uSec(m->segment_time);
	}

	// first half of a tail deceleration
	if (m->move_state == MOVE_STATE_TAIL_D1) {
		while (m->segment_count) {
			return_if_motor_buffer_full(TG_EAGAIN);
			m->segment_count--;
			m->elapsed_time += m->segment_time;
			m->segment_velocity = m->velocity_initial - 
								 (m->linear_jerk_div2 * square(m->elapsed_time));
			for (uint8_t i=0; i < AXES; i++) {
				m->dtarget[i] = mm.dposition[i] + m->unit_vector[i] * 
							    m->segment_velocity * m->segment_time;
				m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
				m->steps[i] = m->target[i] - mm.position[i];
			}
			mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
			_mc_set_endpoint_position(m);
		}
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->move_state = MOVE_STATE_TAIL_D2;
	}
	// second half of a tail deceleration
	if (m->move_state == MOVE_STATE_TAIL_D2) {	// convex portion of curve
		while (m->segment_count) {
			return_if_motor_buffer_full(TG_EAGAIN);
			m->segment_count--;
			m->elapsed_time += m->segment_time;
			m->segment_velocity = m->velocity_midpoint - 
								 (m->elapsed_time * m->acceleration_midpoint) +
								 (m->linear_jerk_div2 * square(m->elapsed_time));
			for (uint8_t i=0; i < AXES; i++) {
				m->dtarget[i] = mm.dposition[i] + m->unit_vector[i] * 
							    m->segment_velocity * m->segment_time;
				m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
				m->steps[i] = m->target[i] - mm.position[i];
			}
			mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
			_mc_set_endpoint_position(m);
		}
		m->move_state = MOVE_STATE_OFF;
		m->next->prev_move_end_state = MOVE_STATE_OFF;
	}
	return (TG_OK);
}

/* 
 * mc_arc() 	- setup and queue an arc move
 * mc_run_arc() - generate an arc
 *
 * Generates an arc by queueing line segments to the move buffer.
 * The arc is approximated by generating a huge number of tiny, linear
 * segments. The length of each segment is configured in config.h by 
 * setting MM_PER_ARC_SEGMENT.  
 *
 * mc_arc()
 *	Loads a move buffer with calling args and initialzation values
 *
 * mc_run_arc() 
 *	run_arc() is structured as a continuation called by the mc_controller.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 * Note on mq_test_move_buffer_full()
 *	The move buffer is tested and sometime later it's queued (via mc_line())
 *	This only works because no ISRs queue this buffer, and the arc run 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */

uint8_t mc_arc(double theta, 			// starting angle
		   double radius, 			// radius of the circle in millimeters.
		   double angular_travel, 	// radians to go along arc (+ CW, - CCW)
		   double linear_travel, 
		   uint8_t axis_1, 			// select circle plane in tool space
		   uint8_t axis_2,  		// select circle plane in tool space
		   uint8_t axis_linear, 	// linear travel if helical motion
		   double minutes)			// time to complete the move
{
	struct mcMotionControl *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	m->move_length = hypot(angular_travel * radius, labs(linear_travel));	
	if (m->move_length < cfg.mm_per_arc_segment) { // too short to draw
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the arc move struct
	m->theta = theta;
	m->radius = radius;
	m->axis_1 = axis_1;
	m->axis_2 = axis_2;
	m->axis_linear = axis_linear;

	m->segments = ceil(m->move_length / cfg.mm_per_arc_segment);
	m->segment_count=0;
 	m->microseconds = round((minutes / m->segments) * ONE_MINUTE_OF_MICROSECONDS);
	m->theta_per_segment = angular_travel / m->segments;
	m->linear_per_segment = linear_travel / m->segments;
	m->center_x = (mm.position[m->axis_1] / CFG(m->axis_1).steps_per_unit) 
					- sin(m->theta) * m->radius;
	m->center_y = (mm.position[m->axis_2] / CFG(m->axis_2).steps_per_unit) 
					- cos(m->theta) * m->radius;

  	// 	target tracks the end point of each segment. Initialize linear axis
	m->dtarget[m->axis_linear] = mm.position[m->axis_linear] / 
								 CFG(m->axis_linear).steps_per_unit;

	return(mc_commit_write_buffer(MOVE_TYPE_ARC));
}

uint8_t _mc_run_arc(struct mcMotionControl *m) 
{
	while (m->segment_count <= m->segments) {
		return_if_motor_buffer_full(TG_EAGAIN);
		// compute the arc segment
		m->segment_count++;
		m->theta += m->theta_per_segment;
		m->dtarget[m->axis_1] = m->center_x + sin(m->theta) * m->radius;
		m->dtarget[m->axis_2] = m->center_y + cos(m->theta) * m->radius;
		m->dtarget[m->axis_linear] += m->linear_per_segment;

		// setup and queue the arc segment
		for (uint8_t i = X; i < AXES; i++) {
			m->target[i] = round(m->dtarget[i] * CFG(i).steps_per_unit);
			m->steps[i] = m->target[i] - mm.position[i];
		}
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_endpoint_position(m);
	}
	return (TG_OK);
}

//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _mc_test_buffers(void);

void mc_unit_tests()
{
	_mc_test_buffers();
}

void _mc_test_buffers()
{
	mc_get_write_buffer();		// open a write buffer [0]
	mc_get_run_buffer();		// attempt to get run buf - should fail (NULL)
	mc_commit_write_buffer();	// commit the write buffer
	mc_get_run_buffer();		// attempt to get run buf - should succeed
	mc_get_write_buffer();		// open the next write buffer [1]
	mc_commit_write_buffer();	// commit it
	mc_get_write_buffer();		// open the next write buffer [2]
	mc_commit_write_buffer();	// commit it
	mc_get_write_buffer();		// attempt write buffer - should fail (NULL)
	mc_end_run_buffer();
	mc_get_write_buffer();		// now if should succeed

}

#endif
