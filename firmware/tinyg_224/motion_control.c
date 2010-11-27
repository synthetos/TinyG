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
uint8_t _mc_run_cruise(struct mcBuffer *m);
uint8_t _mc_run_start_stop(struct mcBuffer *m);
uint8_t _mc_run_dwell(struct mcBuffer *m);
uint8_t _mc_run_arc(struct mcBuffer *m);

void _mc_set_final_position(struct mcBuffer *m);
void _mc_set_intermediate_position(struct mcBuffer *m);
double _mc_estimate_angular_jerk(struct mcBuffer *m, struct mcBuffer *p);

#define MC_BUFFER_SIZE 8		// sub-move buffer pool (255 max)

#define _mc_bump(a) ((a<MC_BUFFER_SIZE-1)?(a+1):0)	// buffer incr & wrap
#define _steps(x,a) round(a * CFG(x).steps_per_unit)

// All the enums that equal zero must be zero. Don't change this

enum mcBufferState {			// state of motion control struct
	MC_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE ZERO)
	MC_BUFFER_LOADING,			// being written ("checked out")
	MC_BUFFER_QUEUED,			// in queue
	MC_BUFFER_RUNNING			// current running buffer
};

enum mcMoveType {				// used to dispatch run routines
	MC_TYPE_LINE = 0,			// simple line
	MC_TYPE_ACCEL,				// run as an acceleration
	MC_TYPE_DECEL,				// run as a deceleration
	MC_TYPE_CRUISE,				// run as a cruise
	MC_TYPE_ARC,
	MC_TYPE_DWELL,
	MC_TYPE_START,
	MC_TYPE_STOP,
	MC_TYPE_END
};

enum mcMoveState {				// states applicable to moves and sub-moves
	MC_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MC_STATE_RUNNING_1,			// first half of move or sub-move
	MC_STATE_RUNNING_2			// second half of move or sub-move
};
#define MC_STATE_RUNNING MC_STATE_RUNNING_1	// a convenience

struct mcBuffer {				// move/sub-move motion control structure
	// buffer management
	struct mcBuffer *nx;		// static pointer to next buffer in ring
	struct mcBuffer *pv;		// static pointer to previous buffer in ring
	uint8_t buffer_state;		// mcBufferState - manages queues

	// move control variables
	uint8_t move_type;			// mvType - used to dispatch to run routine
	uint8_t move_state;			// sequence state

	// common variables
	double target[AXES];		// target position in floating point
	int32_t steps[AXES];		// target position in relative steps
	double unit_vector[AXES];	// unit vector for scaling axes

	double time;				// line or helix or dwell time in minutes
	double length;				// line or helix length in mm
	uint32_t microseconds;		// uSec of target move (a convenience)

	double initial_velocity;	// velocity at end of previous line
	double midpoint_velocity;	// velocity at the transition midpoint
	double target_velocity;		// target velocity for this line
	double linear_jerk_div2;	// saves a few cycles even w/full optimization
	double acceleration_midpoint; // acceleration at the half

	// acceleration / deceleration variables
	uint32_t segments;			// number of segments in arc or blend
	uint32_t segment_count;		// number of segments queued (run) so far
	double segment_time;		// constant time per aline segment
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double elapsed_time;		// running time for sub-move

	// arc variables (that are not already captured above)
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
	double segment_theta;		// angular motion per segment
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mcMotionMaster {
	uint8_t (*run_move)(struct mcBuffer *m);	// currently running move
	uint8_t run_flag;							// move status

	// persistent position info
	double position[AXES];		// final move position in floating point
	double position_inter[AXES];// sub-move position in floating point

	// ring buffer for queueing and processing moves
	struct mcBuffer *w;			// get_write_buffer pointer
	struct mcBuffer *q;			// queue_write_buffer pointer
	struct mcBuffer *r;			// get/end_run_buffer pointer
	struct mcBuffer b[MC_BUFFER_SIZE];// buffer storage
};
static struct mcMotionMaster mm;

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

	memset(&mm, 0, sizeof(mm));		// clear all values, pointers and status
	mm.w = &mm.b[0];				// init write and read buffer pointers
	mm.q = &mm.b[0];
	mm.r = &mm.b[0];
	pv = &mm.b[MC_BUFFER_SIZE-1];
	for (uint8_t i=0; i < MC_BUFFER_SIZE; i++) {	// set the ring pointers
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
			case MC_TYPE_ACCEL: { mm.run_move = _mc_run_accel; break; }
			case MC_TYPE_DECEL: { mm.run_move = _mc_run_decel; break; }
			case MC_TYPE_CRUISE:{ mm.run_move = _mc_run_cruise; break; }
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
 *						   Write buffers must be queued in order to be run
 *						   Advances write pointer & changes buffer state
 *
 * mc_get_run_buffer()	   Get the next or current run buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Returns a new run buffer if previous buf was ended
 *						   Returns same buffer if called again before end_run
 *						   This behaviour supports continuations (iteration)
 *
 * mc_end_run_buffer()	   Release the run buffer & return it to buffer pool
 *						   End-run causes get_run to return the next buffer
 *
 * A typical usage sequence is:
 *	1 - test if you can get 3 write buffers (head, body, tail)
 *	2 - call aline and get 3 write buffers (or perhaps only 2 if no body)
 *	3 - aline queues the write buffers - one queue_write call per buffer
 *	4 - run_aline gets a new run buffer and starts to execute the sub-move
 *	5 - run_aline gets the same buffer as it iterates through the sub-move
 *	6 - run_aline ends the run buffer when the sub-move is complete
 *	7 - run_aline gets a run buffer - which now returns a new one
 *
 * Further notes:
 *	The pointers only move forward on commit and end calls (not test & get).
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

	memset(mm.r, 0, sizeof(struct mcBuffer)); // BUFFER_EMPTY
	mm.r->nx = nx;			// restore pointers
	mm.r->pv = pv;
	mm.r = mm.r->nx;		// advance to next run buffer
	return (TG_OK);			// convenience for calling routines
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
 * mc_set_position() 		  	   - set current MC position (support for G92)
 * _mc_set_final_position()	  	   - copy move final position in float and steps
 * _mc_set_intermediate_position() - copy sub-move position in float and steps
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
 * Algorithm at coarse grain: 
 *
 *	This code uses a cubic spline solution to generate acceleration and 
 *	deceleration ramps that obey maximum jerk parameters. The approach 
 *	and the motion equations were taken or derived from Ed Red's BYU  
 *	robotics course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *
 *	A move (line or arc) is divided into 3 regions (sub-moves):
 *	  - head	initial acceleration/deceleration to target velocity
 *	  - body	bulk of move at target speed (absent in come cases)
 *	  - tail	ending deceleration to exit velocity
 *
 *	The head is computed from the exit velocity of the previous move
 *	or from zero velocity in many cases. In blend cases the head region  
 *	overlaps two lines, with the midpoint of the transition region at 
 *	the point the lines join. 
 *
 *	The body is the "cruise region" where the line is running at its
 *	target velocity. In some cases there is no body as the move is too 
 *	short to achieve target cruise velocity. The target velocity is 
 *	scaled down and the head and tail join directly at that maximum 
 *	achievable velocity.
 *
 *	The tail length is reserved to allow a worst-case deceleration from
 *	target velocity to zero. In many cases the tail is recomputed to blend 
 *	with the next move.
 *
 *	The computation of the regions is set by the path control mode in effect:
 *
 *	  -	EXACT_STOP_MODE: The move runs to zero velocity before the next 
 *		move is started. The entire tail region is used.
 *
 *	  - EXACT_PATH_MODE: The move is spliced to the next move with an 
 *		attempt to keep the path as accurate as possible. The splice 
 *		computes a maximum jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerates the
 *		line to a computed "safe" velocity before accelerating into the
 *		next line. For 180 degree turns the line will stop before reversing.
 *
 *	  - CONTINUOUS_MODE: The splice will attempt to run at the maximum 
 *		theoretical rate, accelerating or decelerating between lines. The
 *		velocity at the join point will always equal half the difference 
 *		between the two velocities (plus the smaller). In the future this
 *		mode should also spline the lines to round the corners - which it
 *		does not currently do.
 *
 *	The following cases apply if the line is joining to an arc:
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc.
 *
 *	  - Line is followed by an arc: The tail is used to accelerate or
 *		decelerate to match the arc feed rate.
 *
 *	  - Arc to arc splining: is not currently supported. 
 *		A velocity step may occur between arcs of different speeds. This
 *		will also occur if an arc is started or stopped from vero velocity.
 *
 *	Head and tail acceleration / deceleration sub-moves are queued as a set 
 *	of constant-time segments that implement the transition. The segment 
 *	time constant is chosen to allow sufficiently fine accel/decel 
 *	resolution and enough time between segments for the computation of the
 *	next segment and for other processor tasks (a segment takes ~150 uSec 
 *	to cycle @ 32 Mhz)
 *
 * Notes on algorithm at fine grain:
 *
 *	The aline() main routine is the trajectory planner. It is called to 
 *	compute and queue a new line. It computes all initial parameters and 
 *	queues the move as three move structs. 
 *
 *	The tail is computed as an exact stop mode tail - i.e. to decelerate to 
 *	zero velocity. If a following line arrives before the tail is executed
 *	the queued tail may be re-computed:
 *
 *	  - CONTINUOUS_MODE (blend case): Previously queued tail is re-computed.
 *		The tail deceleration is replaced with a cruise - extending the 
 *		body sub-move of the move. This will join to a blend which is queued 
 *		as the the head of the new line.
 *
 *	  - CONTINUOUS_MODE dip case: If a continuous mode move cannot be run 
 *		as a blend because it violates angular jerk maximum it is degraded
 *		to an exact path (dip case) move; see below. In some extreme cases
 *		(180 degree turns) it may be degraded to exact stop mode.
 *
 *	  - EXACT_PATH_MODE (dip case): Previously queued tail is recomputed 
 *		to decelerate to a non-zero exit velocity. The body is extended 
 *		with a cruise that replaces the queued tail. A new tail is computed 
 *		and queued bringing the move to the new target exit velocity.
 *
 *	  - EXACT_STOP_MODE: Tail decelerates to zero velocity as planned.
 *
 *	The aline() continuation (run) is the trajectory executor. It reads 
 *	queued sub-moves and executes them in sequence. It just keep dequeing
 *	unless there's nothing to run.
 *
 *	Further notes: 
 *	  - See the excel sheet that has the simulations and equations.
 *
 *	  - All math is done in double precision floating point and minutes 
 *		until the very end, when it's converted to steps and microseconds 
 *		for queueing motor moves
 */
/*
 * Definitions
 *
 *	There are 3 types of path modes as defined by RS274
 *		EXACT_STOP	- start from stop and end at stop
 *		EXACT_PATH	- control start and end to maintain path accuracy
 *		CONTINUOUS	- sacrifice path accuracy for speed
 *
 *	alines are divided into 3 types of regions (sub-moves)
 *		HEAD	- starting portion of a line
 *		BODY	- cruise - may be absent if can't reach cruise velocity
 *		TAIL	- ending portion of a line
 *
 *	There are 3 types of velocity contours
 *		ACCEL	- accelerate from Vi to Vt. Vi may be zero or non-zero
 *		DECEL	- decelerate from Vi to Vt. Vt may be zero or non-zero
 *		CRUISE	- move at fixed velocity (Vi = Vt)
 *
 *	There are 2 types of path contours
 *		EXACT	- follow the exact path
 *		SPLINE	- spline between points A and B to reduce angular jerk
 *
 *	Some interesting velocities are
 *		Vi	- initial velocity of a region (Vi = 0 for a standing start)
 *		Vt 	- target velocity of a region  (Vt = cruise velocity)
 *		Vm	- velocity at the midpoint between Vi and Vj
 *		Vj	- velocity at the point 2 moves join (joint velocity)
 *
 * TRAJECTORY PLANNING (main routine pseudocode)

	First determine your achievable target velocity
		Compute a head and see if you get a cruise region
		Otherwise figure the maximum velocity you can achieve

	Compute timing for an exact_path head, body and tail.

	Perform this conditional based on the path control mode:

	  CONTINUOUS MODE
		If no previous tail, process this line as an EXACT_STOP
		Compute the angular jerk factor
		If angular jerk > threshold, process as EXACT_PATH mode
		Else create a blend in the span region ensuring that Vj = Vm
			Extend the cruise of the previous line to join the blend
			Re-compute the cruise of the current line from the end of the blend

	  EXACT PATH MODE
		If no previous tail, process this line as an EXACT_STOP
		Compute the angular jerk factor
		Compute Vj where:
			Vj = 0 if the turn is 180 degrees (maximum jerk)
			0 < Vj < Vm for any angle between 180 degrees and 0 degrees
		Compute the previous tail to the join (Vt = Vj). 
			Extend cruise as needed
		Compute the head in the current line (Vi = Vj)

	  EXACT STOP MODE
		If there is a previous tail compute amnd run an exact_stop tail
		Compute an exact_stop head for the current line (Vi = 0)

	How to set up the tail transitions (this gets confusing)
	  Preamble: ending a a run buffer zeros out all state information
	  Do the following in mc_aline()
		Compute and write the tail parameters into the current move buffer
		Enable the current buffer tail by setting mc->tail = MC_DECEL
		Write these same tail params into the next buffer as prev_tail
		Queue the tail in the next buffer by setting mc->prev = MC_QUEUED
		Enable the prev tail settings in the *current* buffer by setting 
			m->prev to MC_ACCEL, MC_DECEL, or MC_CRUISE (depending)

		When the trajectory execution reaches the tail region it should
			run the tail in the current buffer if the next buffer tail is
			is MC_QUEUED (i.e. it has not been processed into a subsequent
			write buffer) Otherhwise it should end the move and let the
			next buffer run the tail as its prev_tail. Sufficiently clear?

  TRAJECTORY EXECUTION (continuation routine)

	Execution is controlled by 
	Execution is controlled by the region variables: m->prev, head, body, tail
		and procedes in that order.

	A region can be set to any of: MC_OFF, MC_CRUISE, MC_ACCEL, MC_DECEL
		which set the subroutine to use to execute that region.

	The state in any region can be: MC_NEW, MC_RUN_1, MC_RUN_2, MC_DONE

	At the end of a region it 
 
  	Execute the tail of the previous move if so indicated
	Execute the head of the current line
	Execute the body of the current line
	Determine if the next line has been queued
		If so, do not run the tail, but exit instead
		If not, run the tail

	Sort the moves out like so:
		MC - where the move is right now (PREV, HEAD, BODY, TAIL)
		region control - set velocity contour (OFF, CRUISE, ACCEL, DECEL)
		region_state - where the region is right now 

*/

uint8_t mc_aline(double x, double y, double z, double a, double move_time)
{
	struct mcBuffer *h; 		// head buffer pointer
	struct mcBuffer *b; 		// body buffer pointer
	struct mcBuffer *t; 		// tail buffer pointer
	struct mcBuffer *p; 		// previous tail buffer pointer

	uint8_t i;
	double move_length;			// line or helix length in mm
	double velocity_delta;
//	double angular_jerk;		// jerk factor: 0=none, 1=max
//	uint8_t path_control;		// Gcode path control mode w/adjustments
	uint8_t I_aint_got_no_body = FALSE;	// Marty Feldman variable

	// This routine will need to as many as 4 write buffers.
	// Their availability should have tested beore calling this routine
	// They must be gotten in the order they will be run
	if ((h = mc_get_write_buffer()) == NULL) { 
		return (TG_BUFFER_FULL_FATAL); 
	}
	p = h->pv;

	// capture the args and compute line length
	h->target[X] = x;
	h->target[Y] = y;
	h->target[Z] = z;
	h->target[A] = a;

	move_length = sqrt(square(h->target[X] - mm.position[X]) +
					   square(h->target[Y] - mm.position[Y]) +
					   square(h->target[Z] - mm.position[Z]));

	// generate a unit vector for scaling segments and finding angular jerk
	for (i=0; i < AXES; i++) {
		h->unit_vector[i] = (h->target[i] - mm.position[i]) / move_length;
	}
	// precompute the move as if it's EXACT_STOP mode
	h->initial_velocity = 0;					// may need to be reset later
	h->target_velocity = move_length / move_time;
	velocity_delta = fabs(h->target_velocity - h->initial_velocity);

	// optimal time in the head is given by the jerk equation (5.x - derived)
	h->time = 2 * sqrt(velocity_delta / cfg.max_linear_jerk);
	h->length = velocity_delta * h->time / 2; 	// classic mechanics

	// compute some other needed variables
	h->midpoint_velocity = (h->target_velocity + h->initial_velocity) / 2;
	h->linear_jerk_div2 = cfg.max_linear_jerk / 2;	// saves cycles even w/-Os
	h->acceleration_midpoint = h->linear_jerk_div2 * h->time;	// useful

	// handle the case where the line is not long enough to reach cruise velocity
	if ((2 * h->length) > move_length) {
		h->target_velocity *= (move_time / (2 * h->time)); // scale Vt
		h->length = move_length / 2;				// best you can do...
		h->time = move_time / 2;					//...given the situation
		I_aint_got_no_body = TRUE;
	} else {
	// allocate and load the body sub-move
		if ((b = mc_get_write_buffer()) == NULL) {
			return (TG_BUFFER_FULL_FATAL);
		} 
		b->length = move_length - (2* h->length);
		b->time = b->length / h->target_velocity;
		b->target_velocity = h->target_velocity;
		for (i=0; i < AXES; i++) {
			b->unit_vector[i] = h->unit_vector[i];
		}
	}

	// allocate and load the tail sub-move
	if ((t = mc_get_write_buffer()) == NULL) {
		return (TG_BUFFER_FULL_FATAL); 
	}
//	t->length = h->length;		// not needed
	t->time = h->time;
	t->target_velocity = 0;
	t->initial_velocity = h->target_velocity;
	t->midpoint_velocity = (t->target_velocity + t->initial_velocity) / 2;
	t->linear_jerk_div2 = h->linear_jerk_div2;
	t->acceleration_midpoint = t->linear_jerk_div2 * t->time;
	for (i=0; i < AXES; i++) {
		t->unit_vector[i] = h->unit_vector[i];
	}

	// Determine what mode to use to transition the moves
	// 	regardless of what mode is set, no previous tail = EXACT_STOP mode
/*
	if (h->prev == MC_QUEUED) { // decel if q'd but not run
		h->path_control = PATH_CONTROL_MODE_EXACT_STOP;	
	} else {
		h->path_control = cfg.gcode_path_control;
	}
	h->angular_jerk = _mc_estimate_angular_jerk(h,p);

	//
	// more cases here...
	//

	//	PATH_CONTROL_MODE_CONTINUOUS - blend the 2 lines

	if (h->path_control == PATH_CONTROL_MODE_CONTINUOUS) {
		// make the head become the blend sub-move
		velocity_delta = fabs(h->target_velocity - h->prev_velocity);
		h->head_time = 2 * sqrt(velocity_delta / cfg.max_linear_jerk);
		h->head_length = velocity_delta * h->head_time / 2;
		if (h->target_velocity > h->prev_velocity) {
			h->head = MC_ACCEL;
		} else {
			h->head = MC_DECEL;
		}
		// make the previous tail continue the cruise of the previous line
		h->prev = MC_CRUISE; // make the tail continue the cruise
		h->prev_tail_length -= h->head_length / 2;
		h->prev_tail_time = h->prev_tail_length / h->prev_velocity;
		// lengthen the body to accommodate the shift of the head
		h->body = MC_CRUISE;
		h->body_length += h->tail_length - h->head_length / 2; // tail = old head
		h->body_time = h->body_length / h->target_velocity;
	}

	if (path_control == PATH_CONTROL_MODE_EXACT_PATH) {
		
	}

	if (path_control == PATH_CONTROL_MODE_EXACT_STOP) {
		
	}
*/
	_mc_set_final_position(h); // final position for the move (not sub-move)

	mc_queue_write_buffer(MC_TYPE_ACCEL);		// queue head buffer
	if (!I_aint_got_no_body) {
		mc_queue_write_buffer(MC_TYPE_CRUISE);	// queue body buffer
	}
	mc_queue_write_buffer(MC_TYPE_DECEL); 		// queue tail buffer
	return (TG_OK);
}

uint8_t _mc_run_cruise(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
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
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_count = m->segments / 2;
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->microseconds = uSec(m->segment_time);
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
							 (m->elapsed_time * m->acceleration_midpoint) -
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
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->microseconds = uSec(m->segment_time);
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
							 (m->elapsed_time * m->acceleration_midpoint) +
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

/*	
 * _mc_estimate_angular_jerk() - estiamte the angular jerk
 *
 * The following is borrowed from Simen Svale Skogsrud's Twister project:
 *
 *  Estimate the power of the jerk at the intersection of two motions.
 *	For our application jerk is half the phytagorean magnitude of the 
 *	difference between the unit vector of the two motions which gives 
 *	us a value between 0 and 1.0 where 0 represents no change of 
 *	direction and 1.0 is a full U-turn
 */

double _mc_estimate_angular_jerk(struct mcBuffer *m, struct mcBuffer *p)
{
	return (sqrt(square(m->unit_vector[X] - p->unit_vector[X]) +
				 square(m->unit_vector[Y] - p->unit_vector[Y]) +
				 square(m->unit_vector[Z] - p->unit_vector[Z]))/2.0);

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
