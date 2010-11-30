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
#include "gcode.h"
#include "config.h"
#include "motion_control.h"
#include "motor_queue.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

uint8_t _mc_run_line(struct mcBuffer *m);
uint8_t _mc_run_aline(struct mcBuffer *m);
uint8_t _mc_run_accel(struct mcBuffer *m);
uint8_t _mc_run_decel(struct mcBuffer *m);
uint8_t _mc_run_linear_decel(struct mcBuffer *m);
uint8_t _mc_run_cruise(struct mcBuffer *m);
uint8_t _mc_run_start_stop(struct mcBuffer *m);
uint8_t _mc_run_dwell(struct mcBuffer *m);
uint8_t _mc_run_arc(struct mcBuffer *m);

void _mc_set_final_position(struct mcBuffer *m);
void _mc_set_intermediate_position(struct mcBuffer *m);

uint8_t _mc_make_line_buffer(double Vi, double Vt, double len, uint8_t type);
uint8_t _mc_queue_head_body_tail(void);
uint8_t _mc_queue_head_tail(void);
uint8_t _mc_queue_body_tail(void);
uint8_t _mc_recompute_previous_tail(struct mcBuffer *p);
double _mc_reduce_target_velocity(void); 
double _mc_estimate_angular_jerk(struct mcBuffer *p);
uint8_t _mc_line_to_arc(struct mcBuffer *p);

#define MC_BUFFER_SIZE 8		// sub-move buffer pool (255 max)
#define ANGULAR_JERK_UPPER_THRESHOLD 0.8	// above which its continuous mode
#define ANGULAR_JERK_LOWER_THRESHOLD 0.1	// below which its exact stop mode

#define _mc_bump(a) ((a<MC_BUFFER_SIZE-1)?(a+1):0)	// buffer incr & wrap
#define _steps(x,a) round(a * CFG(x).steps_per_unit)

// All the enums that equal zero must be zero. Don't change this

enum mcBufferState {			// m->buffer_state values 
	MC_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE ZERO)
	MC_BUFFER_LOADING,			// being written ("checked out")
	MC_BUFFER_QUEUED,			// in queue
	MC_BUFFER_RUNNING			// current running buffer
};

enum mcMoveType {				// m->move_type values 
	MC_TYPE_NONE = 0,			// no move specified (MUST BE ZERO)
	MC_TYPE_LINE,				// simple line
	MC_TYPE_CRUISE,				// cruise at fixed velocity
	MC_TYPE_ACCEL,				// acceleration region
	MC_TYPE_DECEL,				// deceleration region
	MC_TYPE_LIN_D,				// linear deceleration for pathological cases
	MC_TYPE_ARC,
	MC_TYPE_DWELL,
	MC_TYPE_START,
	MC_TYPE_STOP,
	MC_TYPE_END
};

enum mcMoveState {				// m->move_state values
	MC_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MC_STATE_RUNNING_1,			// first half of move or sub-move
	MC_STATE_RUNNING_2			// second half of move or sub-move
};
#define MC_STATE_RUNNING MC_STATE_RUNNING_1	// a convenience for above

struct mcBuffer {				// move/sub-move motion control structure
	// buffer management		// Required initial values are marked as INIT
	struct mcBuffer *nx;		// INIT: static pointer to next buffer (ring)
	struct mcBuffer *pv;		// INIT: static pointer to previous buffer
	uint8_t buffer_state;		// INIT: mcBufferState - manages queues

	// move control variables
	uint8_t move_type;			// INIT: used to dispatch to run routine
	uint8_t move_state;			// INIT: state machine sequence

	// common variables
	double unit_vector[AXES];	// INIT: for axis scaling & jerk computation
	double target[AXES];		// target position in floating point
	int32_t steps[AXES];		// target position in relative steps

	double length;				// INIT: line or helix length in mm
	double time;				// line, helix or dwell time in minutes
	uint32_t microseconds;		// uSec of target move (a convenience)

	double initial_velocity;	// INIT: starting velocity of a region
	double target_velocity;		// INIT: target velocity of a region
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double midpoint_acceleration;//acceleration at the midpoint
	double linear_jerk_div2;	// max linear jerk divided by 2

	// acceleration / deceleration working variables
	uint32_t segments;			// number of segments in arc or blend
	uint32_t segment_count;		// number of segments queued (run) so far
	double segment_time;		// constant time per aline segment
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double elapsed_time;		// running time for sub-move

	// arc variables (that are not already captured above)
	double theta;				// INIT: total angle specified by arc
	double radius;				// INIT: computed via offsets
	double center_1;			// INIT: center of circle at axis 1 (typically X)
	double center_2;			// INIT: center of circle at axis 2 (typically Y)
	double segment_theta;		// angular motion per segment
	uint8_t axis_1;				// INIT: arc plane axis
	uint8_t axis_2;				// INIT: arc plane axis
	uint8_t axis_linear;		// INIT: transverse axis (helical)
};

struct mcMotionMaster {	
	uint8_t (*run_move)(struct mcBuffer *m);	// currently running move
	uint8_t run_flag;							// move status
	uint8_t path_mode;			// Gcode path control mode w/adjustments

	// persistent position info
	double position[AXES];		// final move position
	double position_inter[AXES];// sub-move position (intermediate position)

	// common data used and passed around by the trajectory planner
	double target[AXES];		// move target
	double unit_vector[AXES];	// for axis scaling and jerk computation

	double time;				// total time of move in minutes
	double length;				// length of line or helix in mm
	double initial_velocity;	// initial velocity of the move
	double target_velocity;		// target velocity for the move
	double delta_velocity;		// difference between initial and target
	double previous_velocity;	// maximum V of previous move (not tail exit)
	double angular_jerk;		// angular jerk factor: 0=none, 1=max

	double head_length;			// needed for trajectory planning
	double tail_length;

	// ring buffer for queueing and processing moves
	struct mcBuffer *w;			// get_write_buffer pointer
	struct mcBuffer *q;			// queue_write_buffer pointer
	struct mcBuffer *r;			// get/end_run_buffer pointer
	struct mcBuffer b[MC_BUFFER_SIZE];// buffer storage
};
static struct mcMotionMaster mm;

// p.s. I tried listing variables both ways: target_velocity or Vt,
//		initial_velocity or Vi, etc. and found the first way easier to read

/* 
 * mc_init()
 *
 * The memset does:
 *	- clears all values
 *	- sets all buffer states to MC_EMPTY
 *	- sets all other states to MC_OFF
 */

void mc_init()
{
	struct mcBuffer *pv;

	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	mm.w = &mm.b[0];			// init write and read buffer pointers
	mm.q = &mm.b[0];
	mm.r = &mm.b[0];
	pv = &mm.b[MC_BUFFER_SIZE-1];
	for (uint8_t i=0; i < MC_BUFFER_SIZE; i++) {  // setup ring pointers
		mm.b[i].nx = &mm.b[_mc_bump(i)];
		mm.b[i].pv = pv;
		pv = &mm.b[i];
	}
}

/* 
 * mc_move_dispatcher() - routine for dequeuing and executing moves
 *
 *	Dequeues the buffer queue and runs the individual move continuations.
 *	Manages run buffers and other details.
 *	Runs as a continuation itself; called from tg_controller()
 */

uint8_t mc_move_dispatcher() 
{
	uint8_t status;

	if (mc_get_run_buffer() == NULL) {	// NULL means nothing's running
		return (TG_NOOP);
	}
	if (mm.r->move_state == MC_STATE_NEW) {	// first time in?
		mm.run_flag = TRUE;					// it's useful to have a flag
		switch (mm.r->move_type) { 			// setup the dispatch vector
			case MC_TYPE_LINE:  { mm.run_move = _mc_run_line; break; }
			case MC_TYPE_CRUISE:{ mm.run_move = _mc_run_cruise; break; }
			case MC_TYPE_ACCEL: { mm.run_move = _mc_run_accel; break; }
			case MC_TYPE_DECEL: { mm.run_move = _mc_run_decel; break; }
			case MC_TYPE_LIN_D: { mm.run_move = _mc_run_linear_decel; break; }
			case MC_TYPE_ARC:   { mm.run_move = _mc_run_arc; break; }
			case MC_TYPE_DWELL: { mm.run_move = _mc_run_dwell; break; }
			case MC_TYPE_STOP:  { mm.run_move = _mc_run_start_stop; break; }
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
 *
 * mc_test_write_buffer()  Return TRUE if N write buffers are available
 *
 * mc_get_write_buffer()   Get pointer to next available write buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Multiple write buffers may be open at once
 *
 * mc_queue_write_buffer() Commit the next write buffer to the queue
 *						   Write buffers must be queued in order to be run,
 *						   and will queue in the order they were gotten
 *						   Advances write pointer & changes buffer state
 *
 * mc_get_run_buffer()	   Get the next or current run buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Returns new run buffer if previous buf was ended
 *						   Returns same buf if called again before end_run
 *						   This supports continuations (iteration)
 *
 * mc_end_run_buffer()	   Release the run buffer & return to buffer pool
 *						   End_run causes get_run to return the next buffer
 *
 * mc_get_previous_buffer() Return a pointer to the buffer immediately 
 *							before the next available write buffer. From
 *							there preceding buffers can be read using 
 *							the backwards pointers. This buffer cannot be 
 *							queued and should not be end_run'd.
 *
 * A typical usage sequence is:
 *	1 - test if you can get 4 write buffers (worst case needed for aline)
 *	2 - aline first gets previous_buffer to look back at the previous move
 *	3 - aline then gets write buffers as they are needed
 *	4 - aline queues the write buffers - one queue_write call per buffer
 *	5 - run_aline gets a new run buffer and starts to execute the sub-move
 *	6 - run_aline gets the same buffer as it iterates through the sub-move
 *	7 - run_aline ends the run buffer when the sub-move is complete
 *	8 - run_aline gets a run buffer - which now returns a new one
 *
 * Further notes:
 *	The pointers only move forward on commit and end calls (not test & get)
 *	Do not commit a failed get_write, and do not end a failed run buffer.
 *	You must queue write buffers and end run buffers or this all fails. 
 *	Usually this is done at the end of the routine that gets the buffer.
 */

uint8_t mc_test_write_buffer(uint8_t count) 
{
	struct mcBuffer *w = mm.w;	// temp write buffer pointer

	for (uint8_t i=0; i < count; i++) {
		if (w->buffer_state != MC_BUFFER_EMPTY) {
			return (FALSE);
		}
		w = w->nx;
	}
	return (TRUE);
}

struct mcBuffer * mc_get_write_buffer() 
{
	if (mm.w->buffer_state == MC_BUFFER_EMPTY) {
		struct mcBuffer *w = mm.w;
		w->buffer_state = MC_BUFFER_LOADING;
		mm.w = w->nx;
		return (w);
	}
	return (NULL);
}

uint8_t mc_queue_write_buffer(uint8_t move_type)
{
	mm.q->move_type = move_type;
	mm.q->move_state = MC_STATE_NEW;
	mm.q->buffer_state = MC_BUFFER_QUEUED;
	mm.q = mm.q->nx;
	return (TG_OK);			// convenience for calling routines
}

struct mcBuffer * mc_get_run_buffer() 
{
	// condition: fresh buffer; buffer becomes running if it's queued
	if (mm.r->buffer_state == MC_BUFFER_QUEUED) {
		mm.r->buffer_state = MC_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mm.r->buffer_state == MC_BUFFER_RUNNING) { // return the same buffer
		return (mm.r);
	}
	// condition: no queued buffers. fail it.
	return (NULL);
}

uint8_t mc_end_run_buffer()	// zero-out current run buffer & advance to next
{
	struct mcBuffer *nx = mm.r->nx;	// save the next and previous pointers
	struct mcBuffer *pv = mm.r->pv;

	memset(mm.r, 0, sizeof(struct mcBuffer)); // sets status to BUFFER_EMPTY
	mm.r->nx = nx;			// restore pointers
	mm.r->pv = pv;
	mm.r = mm.r->nx;		// advance to next run buffer
	return (TG_OK);			// convenience for calling routines
}

struct mcBuffer * mc_get_previous_buffer()
{
	return (mm.w->pv);
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

/**** POSITION SETTING ROUTINES ****
 *
 * mc_set_position() 		  	   - set current MC position (support G92)
 * _mc_set_final_position()	  	   - copy move final position
 * _mc_set_intermediate_position() - copy sub-move position
 *
 * Keeping track of position is complicated by the fact that moves that 
 *	have sub-moves (e.g. aline) require multiple reference frames. A final 
 *	position is needed to compute the next incoming move, wheras an interim
 *	position is needed to support the execution of sub-moves.
 *
 * Bear in mind that the positions are set immediately when they are 
 *	computed and are not an accurate representation of the tool position.
 *	In reality the motors will still be processing the action and the 
 *	real tool position is still close to the starting point. 
 *
 * Note: Position values are global, not in any given move buffer
 */

uint8_t mc_set_position(double x, double y, double z, double a)
{
	mm.position[X] = x;
	mm.position[Y] = y;
	mm.position[Z] = z; 
	mm.position[A] = a; 
	return (TG_OK);
}

inline void _mc_set_final_position(struct mcBuffer *m) 
{ 
	memcpy(mm.position, m->target, sizeof(mm.position));
}

inline void _mc_set_intermediate_position(struct mcBuffer *m) 
{ 
	memcpy(mm.position_inter, m->target, sizeof(mm.position_inter));
}
/**** STOP START AND END ROUTINES ****
 *
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
	return (mc_queue_write_buffer(MC_TYPE_STOP));
}

uint8_t mc_queued_start() 
{
	if (mc_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mc_queue_write_buffer(MC_TYPE_START));
}

uint8_t mc_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	if (mc_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mc_queue_write_buffer(MC_TYPE_END));
}

uint8_t _mc_run_start_stop(struct mcBuffer *m) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_start_stop(m->move_type);
	return (TG_OK);
}

/**** MOVES ****
 *
 * mc_dwell() - queue a dwell (non-blocking behavior)
 * mc_dwell_continue() - dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers. When 
 * the stepper driver sees a dwell it times the move but does not send any 
 * pulses. Only the X axis is used to time the dwell - the otehrs are idle.
 */

uint8_t mc_dwell(double seconds) 
{
	struct mcBuffer *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	m->time = seconds / 60;						// convert to minutes
	return (mc_queue_write_buffer(MC_TYPE_DWELL));
}

uint8_t _mc_run_dwell(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_dwell(uSec(m->time)); 
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
	struct mcBuffer *m; 
	uint8_t zed_test=0;

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	m->target[X] = x;
	m->target[Y] = y;
	m->target[Z] = z;
	m->target[A] = a;
	m->time = minutes;

	for (uint8_t i=0; i < AXES; i++) {
		m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position[i]);
		zed_test += m->steps[i];
	}
	if (zed_test == 0) {			// skip zero length moves
		return (TG_ZERO_LENGTH_MOVE);
	}
	return(mc_queue_write_buffer(MC_TYPE_LINE));
}

uint8_t _mc_run_line(struct mcBuffer *m) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], uSec(m->time));
	_mc_set_final_position(m);
	return (TG_OK);
}

/* 
 * mc_aline() 		- queue line move with acceleration / deceleration
 * _mc_run_aline()	- run accel move 
 *
 *	This code uses a cubic spline solution to generate acceleration and 
 *	deceleration ramps that obey maximum jerk parameters. The approach 
 *	and the motion equations were taken or derived from Ed Red's BYU  
 *	robotics course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *
 *	A typical move (line or arc) is divided into 3 regions (sub-moves):
 *	  - head	initial acceleration/deceleration to target velocity
 *	  - body	bulk of move at target speed (cruise region)
 *	  - tail	ending deceleration to exit velocity
 *
 *	The head is computed from the exit velocity of the previous move,
 *	from zero velocity, or somewhere in between. The body is the "cruise 
 *	region" where the line is running at its target velocity (Vt). 
 *	The tail (usually) decelerates to the initial velocity (Vi) of 
 *	the next line. See special cases where much of the above is not true.
 *
 *	The tail length is reserved to allow a worst-case deceleration from
 *	the target velocity to zero. In many cases the tail is recomputed
 *	to blend with the next move.
 *
 *	The computation of the regions is set by path control mode in effect:
 *
 *	  - CONTINUOUS_MODE (G64): The moves will attempt to run at the maximum 
 *		theoretical rate, accelerating or decelerating at line junctions 
 *		to match speeds and maintain maximum velocity. 
 *
 *	  - EXACT_PATH_MODE (G61.1): The move is spliced to the next move with 
 *		an attempt to keep the path as accurate as possible. The splice 
 *		computes a maximum jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerates the
 *		line to a computed "safe" velocity before accelerating into the
 *		next line. For 180 degree turns the line will stop before reversing.
 *
 *	  -	EXACT_STOP_MODE (G61): The move runs to zero velocity before the 
 *		next move is started. The entire reserved tail region is used.
 */
/* 
 *	Aline() is separated into a trajectory planner and a set of trajectory
 *	execution routines (run routines) that run as continuations.
 *
 * Trajectory planner:
 *
 *	The aline() main routine is the trajectory planner. It is called to 
 *	compute and queue a new line. It computes all initial parameters, 
 *	examines the transition cases, computes and queues the sub-moves  
 *	as as set of move structs (typically a head, body and tail, but 
 *	not always).
 * 
 *	The tail is computed as an exact stop tail - i.e. to decelerate to 
 *	zero velocity. If a following line arrives before the tail is executed
 *	the queued tail may be re-computed to blend with the next line. 
 *
 *	Various blending cases are supported depending on the path control mode
 *	selected, the velocity differences of the lines, the angle the lines 
 *	connect, and whether lines are connecting to other lines or to arcs.
 *	
 *	The cases for joining lines to lines are:
 *
 *	  - CONTINUOUS MODE is the default mode. If the angle between two lines
 *		is too sharp (angular jerk is too high) the mode will be downgraded
 *		to exact path mode (for that line only; which may in turn get 
 *		downgraded to exact stop mode) In the future continuous mode should
 *		also spline the lines to round the corners. 
 *		Continuous mode line-to-line cases are: 
 *
 *		- Accelerating:	The body of the previous line is extended to the
 *						start of the new line. The new line performs 
 *						acceleration to Vt.
 *
 *		- Decelerating:	The tail of the previous line decelerates to the 
 *						initial velocity of the new line. The new line 
 *						begins with a cruise.
 *
 *		- Cruising:		The body of the previous line is extended to the
 *						start of the new line. The new line 
 *						begins with a cruise.
 *
 *	  - EXACT_PATH_MODE is similar to continuous mode except that the
 *		previous line will decelerate (if needed) to a "safe" speed at 
 *		the join. The join speed is computed based on the estimated 
 *		angular jerk between the two lines. The new line accelerates from
 *		the join speed. If the angular jerk is too extreme (join angle is 
 *		too sharp) exact path mode will be downgraded for that line to
 *		exact stop mode.
 *
 *	  - EXACT_STOP_MODE: is the same as exact path mode except the join
 *		speed is zero. Exact stop is always used for 180 degree joins.
 *
 *	The following cases apply for joining lines to arcs and arcs to arcs.
 *	At the current time only continuous mode is supported (no acceleration
 *	or deceleration is supported within an arc).
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc.
 *
 *	  - Line is followed by an arc: The line tail is used to accelerate or
 *		decelerate to match the arc feed rate.
 *
 *	  - Arc to arc blending: is not currently supported... 
 *		...so a velocity step may occur between arcs of different speeds. 
 *		A discontinuous step will also occur if an arc is started from 
 *		zero velocity or stopped to zero velocity.(for now, until its fixed)
 *
 *  Special Cases:
 *
 *	  -	In some cases the line is too short to reach cruise velocity. 
 *		The target velocity is scaled down to a maximum achievable 
 *		velocity that still supports a maximum jerk acceleration and
 *		deceleration region. The head and tail join directly at 
 *		that maximum velocity. There is no body.
 *
 *	  - In still other cases the line is even too short to get to zero
 *		velocity from the initial velocity. A linear deceleration is
 *		performed that may exceed maximum jerk; which is still better 
 *		than doing a dead stop.
 *
 * Trajectory execution:
 *
 *	The aline continuation routines (run routines) execute the trajectory.
 *	They read the queued sub-moves and execute them in sequence.
 *
 *	Head and tail acceleration / deceleration sub-moves are run as a set
 *	of constant-time segments that implement the transition. The segment 
 *	time constant is chosen (10 ms) to allow sufficiently fine accel/decel 
 *	resolution and enough steps to occur in a segment so that low velocity
 *	moves are not jerky. (FYI: a seg takes ~150 uSec to compute @ 32 Mhz)
 */
/*
 * Definitions
 *
 *	alines are divided into 3 types of regions (sub-moves)
 *		HEAD	- starting portion of a line
 *		BODY	- cruise - may be absent if can't reach cruise velocity
 *		TAIL	- ending portion of a line
 *
 *	There are 3 types of path modes as defined by RS274
 *		CONTINUOUS	- sacrifice path accuracy for speed
 *		EXACT_PATH	- control start and end to maintain path accuracy
 *		EXACT_STOP	- start from stop and end at stop
 *
 *	There are 4 types of velocity contours
 *		CRUISE	- move at fixed velocity (Vi = Vt)
 *		ACCEL	- accelerate from Vi to Vt. Vi may be zero or non-zero
 *		DECEL	- decelerate from Vi to Vt. Vt may be zero or non-zero
 *		LINEAR_DECEL - decelerate to zero by linear steps. This move may
 *					   exceed maximum jerk values
 */
/*
 * Notes:
 *	(1) See the excel sheet that has the simulations and equations.
 *
 *	(2)	All math is done in double precision floating point and minutes 
 *		until the very end, when it's converted to steps and microseconds 
 *		for queueing motor moves
 *
 *	(3)	An aline() will need between 2 and 4 write buffers. Before calling
 *		aline() you should test that MAX_BUFFERS_NEEDED buffers are 
 *		available or aline() could fail fatally.
 *
 *	(4)	You will notice that initialized line buffers use Vi, Vt and length
 *		but do not require time. Time is derived from velocities, length
 *		and jerk during move setup by the following equation:
 *
 *		  length = delta_velocity * sqrt(delta_velocity / max_linear_jerk);
 *
 *				where delta_velocity is abs(Vt-Vi)
 *
 *`	    The length equation is a combination of these two equations:
 *
 *			time = 2 * sqrt(delta_velocity / cfg.max_linear_jerk);	// 5.x
 *			length = delta_velocity * time / 2;						// [2]
 */

uint8_t mc_aline(double x, double y, double z, double a, double minutes)
{
	uint8_t i;
	struct mcBuffer *p; 		// previous tail buffer pointer

	// capture the function args and compute line length
	mm.target[X] = x;
	mm.target[Y] = y;
	mm.target[Z] = z;
	mm.target[A] = a;
	mm.time = minutes;

	mm.length = sqrt(square(mm.target[X] - mm.position[X]) +
					 square(mm.target[Y] - mm.position[Y]) +
					 square(mm.target[Z] - mm.position[Z]));

// ------- Compute data needed for classifying the move -------

	// Set path control mode and initial velocities.
	// The target velocity Vp of the previous move (mm.previous_velocity)
	// will = zero if the tail is already running or the move doesn't exist
	p = mc_get_previous_buffer();
	if (p->move_type == MC_TYPE_ARC) {				// an active arc
		mm.path_mode = PATH_CONTINUOUS_FROM_ARC;	// force all to arc case
		mm.previous_velocity = p->target_velocity;
		mm.initial_velocity = p->target_velocity;
	} else {										// line cases
		mm.path_mode = cfg.gcode_path_control;		// requested path mode
		mm.previous_velocity = p->initial_velocity;
		mm.initial_velocity = mm.previous_velocity;	// preset Vi to Vp
	}

	// estimate the angular jerk to set up for path control downgrades
	mm.target_velocity = mm.length / mm.time;		// Vt before downgrades
	for (i=0; i < AXES; i++) {						// generate unit vector
		mm.unit_vector[i] = (mm.target[i] - mm.position[i]) / mm.length;
	}
	mm.angular_jerk = _mc_estimate_angular_jerk(p);	// uses unit vector

	// downgrade path modes and adjust velocities
	if ((mm.path_mode == PATH_CONTINUOUS) && (mm.angular_jerk > ANGULAR_JERK_LOWER_THRESHOLD)) {
		mm.path_mode = PATH_EXACT_PATH;				// downgrade path mode
		mm.initial_velocity *= mm.angular_jerk; 	// scale Vi down
	} 
	if ((mm.path_mode == PATH_EXACT_PATH) && (mm.angular_jerk > ANGULAR_JERK_UPPER_THRESHOLD)) {
		mm.path_mode = PATH_EXACT_STOP;				// downgrade path mode
		mm.initial_velocity = 0;					// and start from zero
	}
	if (p->buffer_state != MC_BUFFER_QUEUED) {		// is EMPTY or RUNNING
		mm.path_mode = PATH_EXACT_STOP;		  		// then start from zero
//+++++++		mm.initial_velocity = 0;
		mm.initial_velocity = 150;	//+++++test value
		mm.previous_velocity = 0;
	}

	// compute optimal head and tail lengths. See header note (4),
	mm.tail_length = mm.target_velocity * sqrt(mm.target_velocity / cfg.max_linear_jerk);
	mm.delta_velocity = fabs(mm.target_velocity - mm.initial_velocity);
	mm.head_length = mm.delta_velocity * sqrt(mm.delta_velocity / cfg.max_linear_jerk);

// ------ Now sort out the cases and call the subroutines -------

	// By this point the path modes have already been normalized to 
	// accommodate the angular jerk. This makes the sorting much simpler.
	if (mm.path_mode == PATH_CONTINUOUS_FROM_ARC) {
		_mc_line_to_arc(p);
	} else {
		// separate out acceleration, deceleration and cruise cases

		// ACCELERATION CASES
		if (mm.previous_velocity < mm.target_velocity) {

			if (mm.length > (mm.head_length + mm.tail_length)) {

			} else if (mm.length > mm.tail_length ) {
//				_mc_line_to_line_accelerating(m, p, MC_CASE_NO_BODY);
			}
		// DECELERATION CASES
		} else 	if (mm.previous_velocity > mm.target_velocity) {
//			_mc_line_to_line_decelerating(m,p);

		// CRUISE CASES
		} else {
//			_mc_line_to_line_cruising(m,p);
		}
	}
	// final position for the move (not sub-move)
	mc_set_position(mm.target[X], mm.target[Y], mm.target[Z], mm.target[A]);
	return (TG_OK);
}

/**** ALINE HELPERS ****
 *
 * _mc_make_line_buffer()  		- helper helper for making line buffers
 * _mc_queue_head_body_tail()	- queue a 3 part move
 * _mc_queue_head_tail()		- queue a 2 part move with no body
 * _mc_queue_body_tail()		- queue a 2 part move with no head
 * _mc_recompute_previous_tail()- match previous move to initial velocity
 * _mc_reduce_target_velocity()	- adjust Vt for 2 part no-body move
 */

uint8_t _mc_make_line_buffer(double Vi, double Vt, double len, uint8_t type)
{
	struct mcBuffer *m;

	if ((m = mc_get_write_buffer()) == NULL) { 	// get a buffer or die trying
		return (TG_BUFFER_FULL_FATAL); 
	} 
	m->initial_velocity = Vi;					// set initial velocity
	m->target_velocity = Vt;					// set target velocity
	m->length = len;							// set move length
	for (uint8_t i=0; i < AXES; i++) { 			// copy the unit vector from mm
		m->unit_vector[i] = mm.unit_vector[i]; 
	}
	return(mc_queue_write_buffer(type));		// queue the buffer and return
}

uint8_t _mc_queue_head_body_tail()
{
	uint8_t s;	// status code

	s = _mc_make_line_buffer(mm.initial_velocity, mm.target_velocity, 
 					 		 mm.head_length, MC_TYPE_ACCEL);
	if (s != TG_OK) { return (s); }

	s = _mc_make_line_buffer(mm.target_velocity, mm.target_velocity, 
							(mm.length - mm.head_length - mm.tail_length), 
						 	 MC_TYPE_CRUISE);
	if (s != TG_OK) { return (s); }

	return (_mc_make_line_buffer(mm.target_velocity, 0, 
						 		 mm.tail_length, MC_TYPE_DECEL));
}

uint8_t _mc_queue_head_tail()
{
	uint8_t s;	// status code

	s = _mc_make_line_buffer(mm.initial_velocity, mm.target_velocity, 
							 mm.head_length, MC_TYPE_ACCEL);
	if (s != TG_OK) { return (s); }

	return (_mc_make_line_buffer(mm.target_velocity, 0, 
								 mm.tail_length, MC_TYPE_DECEL));
}

uint8_t _mc_queue_body_tail()
{
	uint8_t s;	// status code

	s = _mc_make_line_buffer(mm.target_velocity, mm.target_velocity, 
							(mm.length - mm.head_length - mm.tail_length), 
							 MC_TYPE_CRUISE);
	if (s != TG_OK) { return (s); }

	return (_mc_make_line_buffer(mm.target_velocity, 0, 
								 mm.tail_length, MC_TYPE_DECEL));
}

/*
 * _mc_recompute_previous_tail()
 *
 *	Match the previous tail to the initial velocity of the current line
 */

uint8_t _mc_recompute_previous_tail(struct mcBuffer *p)
{
	struct mcBuffer *m;
	double length;		// tail length

	if (p->buffer_state != MC_BUFFER_QUEUED) {	// it's a no-op
		return (TG_OK);
	}
	if (mm.initial_velocity == 0) {				// another no-op
		return (TG_OK);
	}
	// reminder: initial velocity of the tail is target velocity of the move
	if (mm.initial_velocity == mm.previous_velocity) {
		p->move_type = MC_TYPE_CRUISE;			// change tail to a cruise
		p->target_velocity = mm.previous_velocity;
		p->time = p->length / p->target_velocity;
		return (TG_OK);						// no need to update unit_vector
	} else {	
		// We need a deceleration. Recompute the previous tail into a
		// body extension and make a new tail
		length = (mm.previous_velocity - mm.initial_velocity) * 
				  sqrt((mm.previous_velocity - mm.initial_velocity) / 
				  cfg.max_linear_jerk);
		p->move_type = MC_TYPE_CRUISE;
		p->target_velocity = mm.previous_velocity;
		p->length -= length; 
		p->time = p->length / p->target_velocity;

		// make a new tail
		if ((m = mc_get_write_buffer()) == NULL) {
			return (TG_BUFFER_FULL_FATAL); 
		} 
		m->initial_velocity = p->target_velocity;
		m->target_velocity = mm.initial_velocity;
		m->length = length;
		for (uint8_t i=0; i < AXES; i++) {
			m->unit_vector[i] = p->unit_vector[i];
		}
		mc_queue_write_buffer(MC_TYPE_DECEL);
	}
	return (TG_OK);
}

uint8_t _mc_line_to_arc(struct mcBuffer *p)
{
	return (TG_OK);
}
/*
 * _mc_reduce_target_velocity()
 *
 *	This function handles the case where the line length and velocities
 *	cannot support a full-speed cruise region. The target velocity must
 *	be reduced to a point where the head and tail can be joined directly
 *	(with no interventing body).
 *
 *	The equations to directly compute the new target velocity are not 
 *	practical to solve on this tiny little computer :(   [see spreadsheet]
 *	Instead we perform an iterative linear approximation to converge on
 *	the reduced velocity while preserving the correct total length.
 */

double _mc_reduce_target_velocity() 
{
	uint8_t i=0;

	if (mm.length > (mm.head_length + mm.tail_length)) {
		return (-1);	// line is too long and doesn't need reduction
	} else if (mm.length < mm.tail_length ) {
		return (-2);	// line is too short and can't be reduced
	}

	while (fabs(mm.head_length + mm.tail_length - mm.length) > 0.002) { // mm
		mm.target_velocity *= mm.length / (mm.head_length + mm.tail_length);
		mm.delta_velocity = fabs(mm.target_velocity - mm.initial_velocity);
		mm.tail_length = mm.target_velocity * sqrt(mm.target_velocity / cfg.max_linear_jerk);
		mm.head_length = mm.delta_velocity * sqrt(mm.delta_velocity / cfg.max_linear_jerk);
		if (i++ > 50) {
			break; // usually converges in < 20 - this is for safety
		}
	}
	return (mm.target_velocity);
}

/*	
 * _mc_estimate_angular_jerk() - estimate the angular jerk
 *
 * The following is borrowed from Simen Svale Skogsrud's Twister project:
 *
 *  Estimate the power of the jerk at the intersection of two motions.
 *	For our application jerk is half the phytagorean magnitude of the 
 *	difference between the unit vector of the two motions which gives 
 *	us a value between 0 and 1.0 where 0 represents no change of 
 *	direction and 1.0 is a full U-turn
 */

double _mc_estimate_angular_jerk(struct mcBuffer *p)
{
	return (sqrt(square(mm.unit_vector[X] - p->unit_vector[X]) +
				 square(mm.unit_vector[Y] - p->unit_vector[Y]) +
				 square(mm.unit_vector[Z] - p->unit_vector[Z]))/2.0);
}

/*	
 * aline RUN routines
 *	_mc_run_cruise()
 *	_mc_run_accel()
 *	_mc_run_decel()
 *	_mc_run_linear_decel()
 */

uint8_t _mc_run_cruise(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	m->time = 2 * m->length / m->target_velocity;	// must derive time from length
	for (uint8_t i=0; i < AXES; i++) {
		m->target[i] = mm.position_inter[i] + m->unit_vector[i] * m->length;
		m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position_inter[i]);
	}
	mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], uSec(m->time));
	_mc_set_intermediate_position(m);
	return (TG_OK);
}

uint8_t _mc_run_accel(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for acceleration
	if (m->move_state == MC_STATE_NEW) {
		m->move_state = MC_STATE_RUNNING_1;
		m->time = 2 * m->length / abs(m->target_velocity - m->initial_velocity);
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_count = m->segments / 2;
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->microseconds = uSec(m->segment_time);
		m->midpoint_velocity = (m->target_velocity + m->initial_velocity) / 2;
		m->midpoint_acceleration = m->linear_jerk_div2 * m->time;
		m->linear_jerk_div2 = cfg.max_linear_jerk / 2;
	}
	// first half of acceleration - concave portion of curve
	if (m->move_state == MC_STATE_RUNNING_1) {
		m->elapsed_time += m->segment_time;
		m->segment_velocity = m->initial_velocity + 
							 (m->linear_jerk_div2 * square(m->elapsed_time));
		// multiply the computed position by the unit vector to get the 
		// contribution for each axis. Set the target position in absolute
		// steps, and compute the relative steps for this move.
		for (uint8_t i=0; i < AXES; i++) {
			m->target[i] = mm.position_inter[i] + m->unit_vector[i] * 
						    m->segment_velocity * m->segment_time;
			m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position_inter[i]);
		}
		// queue the line and adjust the variables for the next iteration
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_intermediate_position(m);
		if (--m->segment_count) {
			return (TG_EAGAIN);
		}
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->move_state = MC_STATE_RUNNING_2;
	}
	// second half of acceleration - convex portion of curve
	if (m->move_state == MC_STATE_RUNNING_2) {
		m->elapsed_time += m->segment_time;
		m->segment_velocity = m->midpoint_velocity + 
							 (m->elapsed_time * m->midpoint_acceleration) -
							 (m->linear_jerk_div2 * square(m->elapsed_time));
		for (uint8_t i=0; i < AXES; i++) {
			m->target[i] = mm.position_inter[i] + m->unit_vector[i] * 
						    m->segment_velocity * m->segment_time;
			m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position_inter[i]);
		}
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_intermediate_position(m);
		if (--m->segment_count) {
			return (TG_EAGAIN);
		}
	}
	return (TG_OK);
}

uint8_t _mc_run_decel(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for deceleration
	if (m->move_state == MC_STATE_NEW) {
		m->move_state = MC_STATE_RUNNING_1;
		m->time = 2 * m->length / abs(m->target_velocity - m->initial_velocity);
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->microseconds = uSec(m->segment_time);
		m->midpoint_velocity = (m->target_velocity + m->initial_velocity) / 2;
		m->midpoint_acceleration = m->linear_jerk_div2 * m->time;
		m->linear_jerk_div2 = cfg.max_linear_jerk / 2;
	}
	// first half of deceleration
	if (m->move_state == MC_STATE_RUNNING_1) {	// concave portion of curve
		m->segment_velocity = m->initial_velocity - 
							 (m->linear_jerk_div2 * square(m->elapsed_time));
		for (uint8_t i=0; i < AXES; i++) {
			m->target[i] = mm.position_inter[i] + m->unit_vector[i] * 
						    m->segment_velocity * m->segment_time;
			m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position_inter[i]);
		}
		// queue the line and adjust the variables for the next iteration
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_intermediate_position(m);
		m->elapsed_time += m->segment_time;
		if (--m->segment_count) {
			return (TG_EAGAIN);
		}
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->move_state = MC_STATE_RUNNING_2;
	}
	// second half of deceleration
	if (m->move_state == MC_STATE_RUNNING_2) {	// convex portion of curve
		m->segment_velocity = m->midpoint_velocity - 
							 (m->elapsed_time * m->midpoint_acceleration) +
							 (m->linear_jerk_div2 * square(m->elapsed_time));
		for (uint8_t i=0; i < AXES; i++) {
			m->target[i] = mm.position_inter[i] + m->unit_vector[i] * 
						    m->segment_velocity * m->segment_time;
			m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position_inter[i]);
		}
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_intermediate_position(m);
		m->elapsed_time += m->segment_time;
		if (--m->segment_count) {
			return (TG_EAGAIN);
		}
	}
	return (TG_OK);
}

uint8_t _mc_run_linear_decel(struct mcBuffer *m)
{
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
 *	run_arc() is structured as a continuation called by mc_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 * Note on mq_test_MC_BUFFER_full()
 *	The move buffer is tested and sometime later it's queued (via mc_line())
 *	This only works because no ISRs queue this buffer, and the arc run 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */

uint8_t mc_arc(double theta, 		// starting angle
		   double radius, 			// radius of the circle in millimeters.
		   double angular_travel, 	// radians to go along arc (+ CW, - CCW)
		   double linear_travel, 
		   uint8_t axis_1, 			// select circle plane in tool space
		   uint8_t axis_2,  		// select circle plane in tool space
		   uint8_t axis_linear, 	// linear travel if helical motion
		   double minutes)			// time to complete the move
{
	struct mcBuffer *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get a write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	m->length = hypot(angular_travel * radius, labs(linear_travel));	
	if (m->length < cfg.mm_per_arc_segment) { // too short to draw
		return (TG_ZERO_LENGTH_MOVE);
	}
	m->target_velocity = m->length / minutes; // used for trajectory planning

	// load the move struct
	m->theta = theta;
	m->radius = radius;
	m->axis_1 = axis_1;
	m->axis_2 = axis_2;
	m->axis_linear = axis_linear;

	m->segment_count=0;
	m->segments = ceil(m->length / cfg.mm_per_arc_segment);
 	m->microseconds = uSec(minutes / m->segments);
	m->segment_theta = angular_travel / m->segments;
	m->segment_length = linear_travel / m->segments;
	m->center_1 = mm.position[m->axis_1] - sin(m->theta) * m->radius;
	m->center_2 = mm.position[m->axis_2] - cos(m->theta) * m->radius;
	m->target[m->axis_linear] = mm.position[m->axis_linear];
	return(mc_queue_write_buffer(MC_TYPE_ARC));
}

uint8_t _mc_run_arc(struct mcBuffer *m) 
{
	while (m->segment_count <= m->segments) {
		if (!mq_test_motor_buffer()) { 
			return (TG_EAGAIN); 
		}
		// compute the arc segment
		m->segment_count++;
		m->theta += m->segment_theta;
		m->target[m->axis_1] = m->center_1 + sin(m->theta) * m->radius;
		m->target[m->axis_2] = m->center_2 + cos(m->theta) * m->radius;
		m->target[m->axis_linear] += m->segment_length;

		// setup and queue the arc segment
		for (uint8_t i = X; i < AXES; i++) {
			m->steps[i] = _steps(i, m->target[i]) - _steps(i, mm.position[i]);
		}
		mq_queue_line(m->steps[X], m->steps[Y], m->steps[Z], m->steps[A], m->microseconds);
		_mc_set_final_position(m);
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
	mc_test_write_buffer(MAX_BUFFERS_NEEDED); // test for enough free buffers

	mc_get_write_buffer();		// open a write buffer [0]
	mc_get_write_buffer();		// open a write buffer [1]
	mc_get_write_buffer();		// open a write buffer [2]

	mc_get_run_buffer();		// attempt to get run buf - should fail (NULL)

	mc_queue_write_buffer(MOVE_TYPE_ALINE);	// queue the write buffer [0]
	mc_queue_write_buffer(MOVE_TYPE_ALINE);	// queue the write buffer [1]
	mc_queue_write_buffer(MOVE_TYPE_ALINE);	// queue the write buffer [2]

	mc_get_run_buffer();		// attempt to get run buf - should succeed

/*
	mc_get_write_buffer();		// open the next write buffer [1]
	mc_queue_write_buffer(MOVE_TYPE_ALINE);	// commit it
	mc_get_write_buffer();		// open the next write buffer [2]
	mc_queue_write_buffer(MOVE_TYPE_ALINE);	// commit it
	mc_get_write_buffer();		// attempt write buffer - should fail (NULL)
	mc_end_run_buffer();
	mc_get_write_buffer();		// now if should succeed
*/
}

#endif
