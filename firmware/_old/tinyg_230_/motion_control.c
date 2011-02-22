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
 *
 *	Now supports acceleration and deceleration
 */

#include "xmega_init.h"				// put before <util/delay.h>
#include <util/delay.h>				// needed for dwell
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>

#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "settings.h"
#include "motion_control.h"
#include "motor_queue.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

static uint8_t _mc_run_accel(struct mcBuffer *m);
static uint8_t _mc_run_cruise(struct mcBuffer *m);
static uint8_t _mc_run_decel(struct mcBuffer *m);
static uint8_t _mc_run_line(struct mcBuffer *m);
static uint8_t _mc_run_arc(struct mcBuffer *m);
static uint8_t _mc_run_dwell(struct mcBuffer *m);
static uint8_t _mc_run_stops(struct mcBuffer *m);
static uint8_t _mc_aline_run_segment(struct mcBuffer *m);
static uint8_t _mc_aline_run_finalize(struct mcBuffer *m);

static void _mc_set_move_position(double target[AXES]) ;
static void _mc_set_run_position(double target[AXES]);

//static uint8_t _mc_line_to_arc(struct mcBuffer *p);
static uint8_t _mc_queue_head(void);
static uint8_t _mc_queue_body(void);
static uint8_t _mc_queue_tail(void);
static uint8_t _mc_queue_line_buffer(double Vi, double Vt, double len, uint8_t type);
static uint8_t _mc_compute_regions(void); 
static uint8_t _mc_recompute_previous(struct mcBuffer *p);
static double _mc_get_length(double Vi, double Vt);
static double _mc_estimate_angular_jerk(const struct mcBuffer *p);

// All the enums that equal zero must be zero. Don't change this

enum mcBufferState {			// m->buffer_state values 
	MC_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MC_BUFFER_LOADING,			// being written ("checked out")
	MC_BUFFER_QUEUED,			// in queue
	MC_BUFFER_RUNNING			// current running buffer
};

enum mcMoveType {				// m->move_type values 
	MC_TYPE_NONE = 0,			// no move specified (MUST BE ZERO)
	MC_TYPE_ACCEL,				// controlled jerk acceleration region
	MC_TYPE_CRUISE,				// cruise at fixed velocity
	MC_TYPE_DECEL,				// controlled jerk deceleration region
	MC_TYPE_LINE,				// simple line
	MC_TYPE_ARC,				// arc feed
	MC_TYPE_DWELL,				// delay with no movement
	MC_TYPE_START,				// restart motors
	MC_TYPE_STOP,				// stop motors
	MC_TYPE_END					// stop motors and end program
};

enum mcMoveState {				// m->move_state values
	MC_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MC_STATE_RUNNING_1,			// first half of move or sub-move
	MC_STATE_RUNNING_2,			// second half of move or sub-move
	MC_STATE_FINALIZE,			// finalize the move or sub-move
	MC_STATE_END				// force the move to end (kill)
};
#define MC_STATE_RUNNING MC_STATE_RUNNING_1	// a convenience for above

struct mcBuffer {				// move/sub-move motion control structure
	struct mcBuffer *nx;		// static pointer to next buffer
	struct mcBuffer *pv;		// static pointer to previous buffer
	uint8_t buffer_state;		// mcBufferState - manages queues
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// state machine sequence

	double target[AXES];		// target position in floating point
	double unit_vec[AXES];		// axis scaling & jerk computation

	double length;				// line or helix length in mm
	double time;				// line, helix or dwell time in minutes
	double starting_velocity;	// starting velocity of a region
	double ending_velocity;		// ending velocity of a region

	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mcBufferPool {			// ring buffer for sub-moves
	struct mcBuffer *w;			// get_write_buffer pointer
	struct mcBuffer *q;			// queue_write_buffer pointer
	struct mcBuffer *r;			// get/end_run_buffer pointer
	struct mcBuffer b[MC_BUFFER_SIZE];// buffer storage
};

struct mcMotionMaster {	
	double position[AXES];		// final move position
	double target[AXES];		// target move position
	double unit_vec[AXES];		// for axis scaling and jerk computation

	double length;				// length of line or helix in mm
	double head_length;			// computed for trajectory planning
	double body_length;			// redundant, but useful
	double tail_length;			// computed for trajectory planning

	double previous_velocity;	// Vt of prev move (not its exit velocity)
	double initial_velocity;	// move initial velocity
	double target_velocity;		// move target velocity
	double final_velocity;		// move exit velocity
};

struct mcMotionRuntime {		// persistent runtime variables
	uint8_t run_flag;			// move status
	uint8_t (*run_move)(struct mcBuffer *m); // currently running move

	double position[AXES];		// final move position
	double target[AXES];		// target move position

	double length;				// length of line or helix in mm
	double time;				// total running time (derived)
	uint32_t microseconds;		// line or segment time in microseconds
	double elapsed_time;		// current running time (increments)
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double midpoint_acceleration;//acceleration at the midpoint
	double linear_jerk_div2;	// max linear jerk divided by 2

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_time;		// constant time per aline segment
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double segment_theta;		// angular motion per segment
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
};

static struct mcMotionMaster mm;
static struct mcMotionRuntime mr;
static struct mcBufferPool mb;

// p.s. I tried listing variables both ways: target_velocity or Vt,
//		initial_velocity or Vi, etc. and found the first way easier to 
//		read in spite of the wrapped lines.

/* 
 * mc_init()
 *
 * The memset does:
 *	- clears all values
 *	- sets buffer states to MC_EMPTY
 *	- sets other states to their zero values - which is typically OFF
 */

void mc_init()
{
	struct mcBuffer *pv;
	uint8_t i=0;

	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	memset(&mb, 0, sizeof(mb));	// clear all values, pointers and status

	mb.w = &mb.b[0];			// init write and read buffer pointers
	mb.q = &mb.b[0];
	mb.r = &mb.b[0];
	pv = &mb.b[MC_BUFFER_SIZE-1];
	for (i=0; i < MC_BUFFER_SIZE; i++) {  // setup ring pointers
		mb.b[i].nx = &mb.b[_mc_bump(i)];
		mb.b[i].pv = pv;
		pv = &mb.b[i];
	}
}

/* 
 * mc_move_dispatcher() - routine for dequeuing and executing moves
 *
 *	Dequeues the buffer queue and executes the move run continuations.
 *	Manages run buffers and other details.
 *	Responsible for freeing the completed run buffers
 *	Runs as a continuation itself; called from tg_controller()
 */

uint8_t mc_move_dispatcher(uint8_t kill) 
{
	uint8_t status;
	struct mcBuffer *m;

	if ((m = mc_get_run_buffer()) == NULL) {	// NULL means nothing's running
		return (TG_NOOP);
	}
	if (kill) {
		m->move_state = MC_STATE_END;
		mr.run_flag = FALSE;				// finalize and return
		mc_end_run_buffer();
		return (TG_OK);
	}
	if (m->move_state == MC_STATE_NEW) {	// first time in?
		mr.run_flag = TRUE;					// it's useful to have a flag
		switch (m->move_type) { 			// setup the dispatch vector
			case MC_TYPE_ACCEL:	{ mr.run_move = _mc_run_accel; break; }
			case MC_TYPE_CRUISE:{ mr.run_move = _mc_run_cruise; break; }
			case MC_TYPE_DECEL:	{ mr.run_move = _mc_run_decel; break; }
			case MC_TYPE_LINE:	{ mr.run_move = _mc_run_line; break; }
			case MC_TYPE_ARC:	{ mr.run_move = _mc_run_arc; break; }
			case MC_TYPE_DWELL:	{ mr.run_move = _mc_run_dwell; break; }
			case MC_TYPE_START:	{ mr.run_move = _mc_run_stops; break;}
			case MC_TYPE_STOP:	{ mr.run_move = _mc_run_stops; break;}
			case MC_TYPE_END: 	{ mr.run_move = _mc_run_stops; break;}
		}
	}
	if ((status = mr.run_move(m)) == TG_EAGAIN) { // run current run buf
		return (TG_EAGAIN);
	}
	mr.run_flag = FALSE;				// finalize and return
	mc_end_run_buffer();
	return (status);
}

/**** MOVE QUEUE ROUTINES ************************************************
 * mc_test_write_buffer()  Return TRUE if N write buffers are available
 *
 * mc_get_write_buffer()   Get pointer to next available write buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Multiple write buffers may be open at once
 *
 * mc_queue_write_buffer() Commit the next write buffer to the queue
 *						   Write buffers will queue in order gotten,
 *						   and will run in the order queued.
 *						   Advances write pointer & changes buffer state
 *
 * mc_unget_write_buffer() Free write buffer if you decide not to queue it
 *						   Only works on most recently gotten write buffer
 *						   You could work your way back in a set or buffers
 *						   Use this one carefully.
 *
 * mc_get_run_buffer()	   Get pointer to the next or current run buffer
 *						   Returns a new run buffer if prev buf was ENDed
 *						   Returns same buf if called again before ENDing
 *						   Returns NULL if no buffer available
 *						   The behavior supports continuations (iteration)
 *
 * mc_end_run_buffer()	   Release the run buffer & return to buffer pool
 *						   End_run causes get_run to return the next buffer
 *
 * mc_get_prev_buffer()	   Return a pointer to the buffer immediately 
 *						   before the next available write buffer. From
 *						   there earlier buffers can be read using the 
 *						   backwards pointers. This buffer cannot be 
 *						   queued and should not be ENDed.
 *
 * A typical usage sequence is:
 *	1 - test if you can get 4 write buffers (worst case needed for aline)
 *	2 - aline first gets prev_buffer to look back at the previous tail
 *	3 - aline then gets write buffers as they are needed
 *  3a- sometimes aline ungets a write buffer an exception case is detected
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

uint8_t mc_test_write_buffer(const uint8_t count) 
{
	struct mcBuffer *w = mb.w;	// temp write buffer pointer

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
	if (mb.w->buffer_state == MC_BUFFER_EMPTY) {
		struct mcBuffer *w = mb.w;
		struct mcBuffer *nx = mb.w->nx;	// save pointers
		struct mcBuffer *pv = mb.w->pv;
		memset(mb.w, 0, sizeof(struct mcBuffer));
		w->nx = nx;			// restore pointers
		w->pv = pv;
		w->buffer_state = MC_BUFFER_LOADING;
		mb.w = w->nx;
		return (w);
	}
	return (NULL);
}

uint8_t mc_queue_write_buffer(const uint8_t move_type)
{
	mb.q->move_type = move_type;
	mb.q->move_state = MC_STATE_NEW;
	mb.q->buffer_state = MC_BUFFER_QUEUED;
	mb.q = mb.q->nx;		// advance the queued buffer pointer
	return (TG_OK);			// convenience for calling routines
}

void mc_unget_write_buffer()
{
	mb.w->buffer_state = MC_BUFFER_EMPTY;
	mb.w = mb.w->pv;
}

struct mcBuffer * mc_get_run_buffer() 
{
	// condition: fresh buffer; buffer becomes running if it's queued
	if (mb.r->buffer_state == MC_BUFFER_QUEUED) {
		mb.r->buffer_state = MC_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mb.r->buffer_state == MC_BUFFER_RUNNING) { // return same buffer
		return (mb.r);
	}
	return (NULL);		// condition: no queued buffers. fail it.
}

uint8_t mc_end_run_buffer()	// EMPTY current run buf & advance to next
{
	mb.r->buffer_state = MC_BUFFER_EMPTY;
	mb.r = mb.r->nx;		// advance to next run buffer
	return (TG_OK);			// convenience for calling routines
}

struct mcBuffer * mc_get_prev_buffer()
{
	return (mb.w->pv);
}

/* 
 * mc_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mc_isbusy()
{
	if (st_isbusy() || mr.run_flag) {
		return (TRUE);
	}
	return (FALSE);
}

/**** POSITION SETTING ROUTINES ******************************************
 * mc_set_position() 		  - set current MC position (support for G92)
 * _mc_set_move_position()	  - copy move final position
 * _mc_set_run_position() 	  - copy sub-move position
 *
 * 	Keeping track of position is complicated by the fact that moves can
 *	have sub-moves (e.g. aline) which require multiple reference frames.
 *	The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for trajectory planning
 *	 - mm.target	- target position for trajectory planning
 *	 - mr.position	- current position of sub-move (runtime endpoint)
 *	 - mr.target	- target position of submove (runtime final target)
 *	 - m->target	- target position of submove (runtime working target)
 *					  also used to carry final target from mm to mr
 *
 * Bear in mind that the positions are set immediately when they are 
 *	computed and are not an accurate representation of the tool position.
 *	In reality the motors will still be processing the action and the 
 *	real tool position is still close to the starting point. 
 */

// used by external callers such as G92
uint8_t mc_set_position(const double x, const double y, const double z, const double a)
{
	mm.position[X] = x;
	mm.position[Y] = y;
	mm.position[Z] = z; 
	mm.position[A] = a; 
	_mc_set_run_position(mm.position);
	return (TG_OK);
}

// copy both levels to keep runtime level sync'd with move level
inline static void _mc_set_move_position(double target[AXES]) 
{ 
	mm.position[X] = target[X];
	mm.position[Y] = target[Y];
	mm.position[Z] = target[Z];
	mm.position[A] = target[A];
	_mc_set_run_position(target);
}

// copy only runtime position
inline static void _mc_set_run_position(double target[AXES]) 
{ 
	mr.position[X] = target[X];
	mr.position[Y] = target[Y];
	mr.position[Z] = target[Z];
	mr.position[A] = target[A];
}

/**** STOP START AND END ROUTINES ****************************************
 * mc_async_stop() 	- stop current motion immediately
 * mc_async_start() - (re)start motion
 * mc_async_end() 	- stop current motion immediately
 *
 *	These routines must be safe to call from ISRs. Mind the volatiles.
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
	mc_move_dispatcher(TRUE);		// kill the current move
	return (TG_OK);
}

/* 
 * mc_queued_stop() 	- queue a motor stop
 * mc_queued_start()	- queue a motor start
 * mc_queued_end()		- end current motion and program
 * _mc_run_start_stop()	- start and stop continuation
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
 * 	- [Feed and speed overrides are set to ON (like M48)].
 *	- [Cutter compensation is turned off (like G40)].
 *	- The spindle is stopped (like M5).
 *	- The current motion mode is set to G1
 *	- [Coolant is turned off (like M9)].
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

static uint8_t _mc_run_stops(struct mcBuffer *m) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_stops(m->move_type);
	return (TG_OK);
}

/*************************************************************************
 * mc_dwell() 		- queue a dwell
 * _mc_run_dwell()	- dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the move but does not 
 * send any pulses. Only the Z axis is used to time the dwell - 
 * the others are idle.
 */

uint8_t mc_dwell(double seconds) 
{
	struct mcBuffer *m; 

	if ((m = mc_get_write_buffer()) == NULL) { // get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);		   // (not supposed to fail)
	}
	m->time = seconds;						   // in seconds, not minutes
	return (mc_queue_write_buffer(MC_TYPE_DWELL));
}

static uint8_t _mc_run_dwell(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_dwell(m->time * 1000000);		   // convert seconds to uSec
	return (TG_OK);
}

/*************************************************************************
 * mc_line() 	  - queue a linear move (simple version - no accel/decel)
 * _mc_run_line() - run a line to generate and load a linear move
 *
 * Compute and queue a line segment to the move buffer.
 * Executes linear motion in absolute millimeter coordinates. 
 * Feed rate has already been converted to time (minutes).
 * Zero length lines are skipped at this level. 
 * 
 * The run_line routine is a continuation and can be called multiple times 
 * until it can successfully load the line into the move buffer.
 */

uint8_t mc_line(double x, double y, double z, double a, double minutes)
{
	struct mcBuffer *m;

	if ((m = mc_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// capture the function args and compute line length
	m->target[X] = x;		// targets for the move
	m->target[Y] = y;
	m->target[Z] = z;
	m->target[A] = a;

	if ((m->time = minutes) == 0) {
		mc_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	m->length = sqrt(square(m->target[X] - mr.position[X]) +
					 square(m->target[Y] - mr.position[Y]) +
					 square(m->target[Z] - mr.position[Z]) +
					 square(m->target[A] - mr.position[A]));

	if (m->length < MIN_LINE_LENGTH) {			// trap zero-length lines
		mc_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	ritorno(mc_queue_write_buffer(MC_TYPE_LINE));
	return(TG_OK);
}

static uint8_t _mc_run_line(struct mcBuffer *m) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}

	uint32_t steps[AXES];	// this form is more memory efficient
	for (uint8_t i=0; i < AXES; i++) {
		steps[i] = _steps(i, m->target[i]) - _steps(i, mr.position[i]);
	}
	mr.microseconds = uSec(m->time);
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);

//	mq_queue_line((_steps(X, m->target[X]) - _steps(X, mr.position[X])),
//				  (_steps(Y, m->target[Y]) - _steps(Y, mr.position[Y])),
//				  (_steps(Z, m->target[Z]) - _steps(Z, mr.position[Z])),
//				  (_steps(A, m->target[A]) - _steps(A, mr.position[A])),
//				   mr.microseconds);

	_mc_set_move_position(m->target);
	return (TG_OK);
}

/************************************************************************* 
 * mc_arc() 	 - setup and queue an arc move
 * _mc_run_arc() - generate an arc
 *
 * Generates an arc by queueing line segments to the move buffer.
 * The arc is approximated by generating a large number of tiny, linear
 * segments. The length of the segments is configured in motion_control.h
 * as MM_PER_ARC_SEGMENT.
 *
 * mc_arc()
 *	Loads a move buffer with calling args and initialization values
 *
 * _mc_run_arc() 
 *	run_arc() is structured as a continuation called by mc_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 * Note on mq_test_MC_BUFFER_full()
 *	The move buffer is tested and sometime later its queued (via mc_line())
 *	This only works because no ISRs queue this buffer, and the arc run 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */

uint8_t mc_arc( double x, double y, double z, double a, 
				double i, double j, double k, 
				double theta, 		// starting angle
				double radius, 		// radius of the circle in mm
				double angular_travel, // radians along arc (+CW, -CCW)
				double linear_travel, 
				uint8_t axis_1, 	// select circle plane in tool space
				uint8_t axis_2,  	// select circle plane in tool space
				uint8_t axis_linear,// linear travel if helical motion
				double minutes)		// time to complete the move
{
	struct mcBuffer *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	m->length = hypot(angular_travel * radius, labs(linear_travel));	
	if (m->length < cfg.mm_per_arc_segment) { // too short to draw
		mc_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the move struct for an arc
	m->target[X] = x;
	m->target[Y] = y;
	m->target[Z] = z;
	m->target[A] = a;
	m->time = minutes;
	m->theta = theta;
	m->radius = radius;
	m->axis_1 = axis_1;
	m->axis_2 = axis_2;
	m->axis_linear = axis_linear;
	m->angular_travel = angular_travel;
	m->linear_travel = linear_travel;
	m->starting_velocity = m->length / m->time; // for trajectory planning

	double length;
	double offset[3] = {i, j, k};
	length = sqrt(square(m->target[axis_1] - i) +
				  square(m->target[axis_2] - j) +
				  square(m->target[axis_linear] - k));

	//	I think you take the normal of the vector between the 
	//	center point (i,j) and the target (x,y) and divide by the 
	//	length of (i,j) to (x,y). Must also account for plane-axes
	//	and the linear axis.
	for (uint8_t i=0; i < 3; i++) {
		m->unit_vec[i] = (m->target[i] - offset[i]) / length;
	}

	return(mc_queue_write_buffer(MC_TYPE_ARC));
}

static uint8_t _mc_run_arc(struct mcBuffer *m) 
{
	uint8_t i;

	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize arc variables
	if (m->move_state == MC_STATE_NEW) {
		mr.segments = ceil(m->length / cfg.mm_per_arc_segment);
		mr.segment_count = mr.segments;
		mr.segment_theta = m->angular_travel / mr.segments;
		mr.segment_length = m->linear_travel / mr.segments;
 		mr.microseconds = uSec(m->time / mr.segments);
		mr.center_1 = mr.position[m->axis_1] - sin(m->theta) * m->radius;
		mr.center_2 = mr.position[m->axis_2] - cos(m->theta) * m->radius;
		m->target[m->axis_linear] = mr.position[m->axis_linear];

		for (i=0; i < AXES; i++) {
			mr.target[i] = m->target[i];	// mr.target saves endpoint
		}
		m->move_state = MC_STATE_RUNNING;
	}
	// compute an arc segment and exit
	if (m->move_state == MC_STATE_RUNNING) {
		m->theta += mr.segment_theta;
		m->target[m->axis_1] = mr.center_1 + sin(m->theta) * m->radius;
		m->target[m->axis_2] = mr.center_2 + cos(m->theta) * m->radius;
		m->target[m->axis_linear] += mr.segment_length;

		uint32_t steps[AXES];
		for (i=0; i < AXES; i++) {
			steps[i] = _steps(i, m->target[i]) - _steps(i, mr.position[i]);
		}
		mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);

//		mq_queue_line((_steps(X, m->target[X]) - _steps(X, mr.position[X])),
//					  (_steps(Y, m->target[Y]) - _steps(Y, mr.position[Y])),
//					  (_steps(Z, m->target[Z]) - _steps(Z, mr.position[Z])),
//					  (_steps(A, m->target[A]) - _steps(A, mr.position[A])),
//					   mr.microseconds);

		_mc_set_run_position(m->target);
		if (--mr.segment_count) {
			return (TG_EAGAIN);
		}
	}
	_mc_set_move_position(m->target);
	return (TG_OK);
}

/*************************************************************************
 * mc_aline() 		- queue line move with acceleration / deceleration
 * _mc_run_aline()	- run accel/decel move 
 *
 *	This module uses the third order position equations to generate 
 *	acceleration and deceleration ramps that obey maximum jerk parameters.
 *	The jerk is the rate of change of acceleration (derivative), which is
 *	the third derivative of position. The jerk is a measure of impact that 
 *	a machine can take, and is therefore the most logical way to limit the
 *	velocity of a move. If the rate of acceleration is controlled at the 
 *	start and end of a move the acceleration or deceleration of the move
 *	can be much faster in the middle of the transition than the machine
 *	could sustain at either end, and therefore allows the move to 
 *	transition to the target velocity much faster. This path makes an S 
 *	curve in velocity.
 *
 *	For more background and the motion equations see Ed Red's BYU robotics
 *	course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf. There is also
 *	an accompanying Excel spreadsheet deriving the remaining equations
 *	and running a simulation.
 *
 *	A typical move (line or arc) is divided into 3 regions (sub-moves):
 *	  - head	acceleration to target velocity (acceleration region)
 *	  - body	bulk of move at target speed 	(cruise region)
 *	  - tail	deceleration to exit velocity 	(deceleration region)
 *
 *	The initial velocity of the head (Vi) head is computed from the exit
 *	velocity of the previous move, from zero, or somewhere in between. 
 *	The body is the "cruise region" where the line is running at its 
 *	target velocity (Vt). The tail is pre-computed to decelerate to zero. 
 *	(See "Special Cases" for exceptions to the above.)
 *
 *	As mentioned above, sufficient length is reserved in the tail to 
 *	allow a worst-case deceleration from Vt to zero - which will occur
 *	if there is no following move or the following move has a Vi = 0 
 *	(such as in EXACT_STOP mode). If the following move has a non-zero Vi
 *	the tail region (of the previous move) is re-computed to meet the Vi.
 */
/*	The computation of the regions is set by path control mode in effect:
 *
 *	  - CONTINUOUS_MODE (G64): The moves will attempt to run at their 
 *		maximum theoretical speeds, accelerating or decelerating at line 
 *		junctions to match speeds and maintain maximum velocity. 
 *
 *	  - EXACT_PATH_MODE (G61.1): The move is spliced to the next move with 
 *		an attempt to keep the path as accurate as possible. The splice 
 *		computes an estimated jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerates the
 *		current line to a computed "safe" velocity before accelerating 
 *		into the next line. For 180 degree turns the line will stop 
 *		completely before reversing.
 *
 *	  -	EXACT_STOP_MODE (G61): The move runs to zero velocity before the 
 *		next move is started. The entire reserved tail region is used.
 */
/*	Aline() is separated into a trajectory planner and a set of trajectory
 *	execution routines (run routines) that execute as continuations called 
 *	by mc_move_dispatcher()
 *
 * Trajectory planner:
 *
 *	The aline() main routine is the trajectory planner. It is called to 
 *	compute and queue a new line. It computes all initial parameters, 
 *	examines the transition cases, computes and queues the sub-moves  
 *	as a set of move buffers (typically a head, body and tail, but 
 *	not always).
 * 
 *	The tail is always pre-computed as an exact stop tail - i.e. to 
 *	decelerate to zero velocity in the event that no new line arrives. 
 *	If a following line arrives before the tail is executed the queued 
 *	tail may be re-computed to blend with the next line. 
 *
 *	Various blending cases are supported depending on the path control mode
 *	in effect, velocity differences between the lines, the angle the lines
 *	connect, and whether lines are connecting to other lines or to arcs.
 *	
 *	The cases for joining lines to lines are:
 *
 *	  - CONTINUOUS MODE (G64) is the default mode. If the angle between two 
 *		lines is too sharp (angular jerk is too high) the move will be 
 *		downgraded to exact path mode for that line only (which may in turn 
 *		get downgraded to exact stop mode). In the future continuous mode 
 *		should also spline the lines to round the corners. Continuous mode 
 *		line-to-line cases are: 
 *
 *		- CRUISING:		The body of the previous line is extended to the
 *						start of the new line. The new line typically 
 *						begins with a cruise body.
 *
 *		- DECELERATING:	The tail of the previous line decelerates to the 
 *						initial velocity of the new line. The new line 
 *						typically begins with a cruise body.
 *
 *		- ACCELERATING:	The body and tail of the previous line are 
 *						extended at Vp to the join with the new line. 
 *						The new line performs acceleration to Vt.
 *
 *	  - EXACT_PATH_MODE (G61.1) is similar to continuous mode except that
 *		the previous line will decelerate if needed ("dip") to a safe 
 *		speed at the join. The new line accelerates from the join speed. 
 *		The join speed is computed based on the estimated angular jerk 
 *		between the two lines and the velocity of the previous line.  
 *		If the jerk is too extreme (join angle is too sharp & fast) 
 *		exact path mode will be further downgraded to exact stop mode 
 *		(again, for that line only).
 *
 *	  - EXACT_STOP_MODE: (G61) is the same as exact path mode except the 
 *		join speed is zero. Exact stop is always used for 180 degree turns
 *
 *  Combined Cases - By the time you combine all these you get a series of 
 *	combined curves, best illustrated by drawing out the velocity 
 *	relationships and short-line morph cases below      (--> morphs into:)
 *	  	[AC] Accel-Continuous	Vp = Vi < Vt	Vi != 0	 --> DC, CC
 *		[AD] Accel-Dip			Vi < Vp < Vt	Vi != 0	 --> DD, DC, CD 
 *		[AS] Accel-Stop			Vi < Vp < Vt	Vi = 0	 --> <isolated>
 *	  	[DC] Decel-Continuous	Vp = Vi < Vp	Vi != 0	 --> <no morph>
 *		[DD] Decel-Dip			Vi < Vt < Vp	Vi != 0	 --> <no morph>
 *		[DS] Decel-Stop			Vi < Vt < Vp	Vi = 0	 --> <no morph>
 *	  	[DC] Cruise-Continuous	Vi = Vp = Vt	Vi != 0	 --> <no morph>
 *		[DD] Cruise-Dip			Vi < Vp = Vt	Vi != 0	 --> <no morph>
 *		[DS] Cruise-Stop		Vi < Vp = Vt	Vi = 0	 --> <no morph>
 *
 *  Special Cases - All of the above cases have sub-cases that are invoked
 *	if the new line is too short to support a deceleration to zero - and 
 *	therefore cannot have a full tail pre-computed. These short line 
 *	cases cause the above cases to morph into other cases - all of which 
 *	are captured above.
 *
 *	  -	In some cases the new line is too short to reach Vt (cruise 
 *		velocity). The target velocity is scaled down to a maximum 
 *		achievable velocity that still supports maximum jerk acceleration 
 *		and deceleration curves. The head and tail join directly at 
 *		that new maximum velocity. There is no body. 
 *
 *	  - In still other cases the line is even too short to get to zero
 *		velocity from the initial velocity. In this case the initial 
 *		velocity is re-computed to support a clean deceleration and the
 *		previous tail is decelerated even more severely to meet this Vi.
 */
/*	Joining to Arcs - The following cases apply for joining lines to arcs 
 *	and arcs to arcs. At the current time only continuous mode is supported 
 *	(no acceleration or deceleration is supported within an arc).
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc - or there is no head if the and speed and 
 *		the line speed are the same. Angular jerk is not taken into account
 *
 *	  - Line is followed by an arc: The line tail is used to accelerate or
 *		decelerate to match the arc feed rate. (Not implemented).
 *
 *	  - Arc to arc blending: is not currently supported... 
 *		...so a velocity step may occur between arcs of different speeds. 
 *		A discontinuous step will also occur if an arc is started from 
 *		zero velocity or stopped to zero velocity.(for now, until fixed)
 */
/* Trajectory Execution:
 *
 *	The aline continuation routines (run routines) execute the trajectory.
 *	They read the queued sub-moves and execute them in sequence.
 *
 *	Head and tail acceleration / deceleration sub-moves are run as a set
 *	of constant-time segments that implement the transition. The segment 
 *	time constant is chosen (~10 ms) to allow sufficiently fine accel/decel 
 *	resolution and enough steps to occur in a segment so that low velocity
 *	moves are not jerky. (FYI: a seg takes ~150 uSec to compute @ 32 Mhz)
 */
/*
 * Notes:
 *	(1)	An aline() will need between 1 and 4 write buffers to compute. 
 *		Before calling aline() you MUST test that MAX_BUFFERS_NEEDED (4)
 *		buffers are available or aline() could fail fatally.
 *
 *	(2)	All math is done in absolute coordinates using double precision 
 *		floating point and in douvble float minutes until the very end, 
 *		when it's converted to steps and microseconds for queueing the 
 *		motor moves.
 *
 *	(3)	You may notice that initialized line buffers use Vi, Vt and Length
 *		but do not require Time. Time is derived from Vi, Vt & L.
 */

uint8_t mc_aline(double x, double y, double z, double a, double minutes)
{
	uint8_t i;
	uint8_t path_mode;
	struct mcBuffer *p; 	// previous tail buffer pointer
	double angular_jerk;

	// capture the function args and compute line length
	mm.target[X] = x;
	mm.target[Y] = y;
	mm.target[Z] = z;
	mm.target[A] = a;
	
	// setup initial values
	mm.length = sqrt(square(mm.target[X] - mm.position[X]) +
					 square(mm.target[Y] - mm.position[Y]) +
					 square(mm.target[Z] - mm.position[Z]) +
					 square(mm.target[A] - mm.position[A]));

	if (mm.length < MIN_LINE_LENGTH) {			// trap zero-length lines
		return (TG_ZERO_LENGTH_MOVE);
	}
	mm.target_velocity = mm.length / minutes;		// Vt requested
	mm.initial_velocity = 0;						// Vi starting value
	path_mode = cfg.gcode_path_control;				// starting path mode
	for (i=0; i < AXES; i++) {						// compute unit vector
		mm.unit_vec[i] = (mm.target[i] - mm.position[i]) / mm.length;
	}
	mr.linear_jerk_div2 = cfg.max_linear_jerk / 2;

	// setup initial conditions from the previous move
	p = mc_get_prev_buffer();
	if ((p->move_type == MC_TYPE_ARC) && 
		(p->buffer_state != MC_BUFFER_EMPTY)) {		// q'd or running arc
//		return(_mc_line_to_arc(p));					// separate out arcs
		mm.initial_velocity = mm.previous_velocity;	// +++ test various arc join speed changes up and down
		ritorno(_mc_compute_regions());	// compute region lengths & Vt
		ritorno(_mc_queue_body());		// queue a body if body_length != 0
		ritorno(_mc_queue_tail());		// queue a tail if tail_length != 0
		return (TG_OK);

	} else if (p->buffer_state == MC_BUFFER_QUEUED){// q'd but not running
		mm.previous_velocity = p->starting_velocity;// Vt of previous move
	} else {
		mm.previous_velocity = 0;
		path_mode = PATH_EXACT_STOP;				// downgrade path mode
	}
	// getting angular jerk requires unit vectors and mm.previous_velocity
	angular_jerk = _mc_estimate_angular_jerk(p);	// for path downgrades

	// setup initial velocity and do path downgrades
	if (path_mode == PATH_CONTINUOUS) {
		if (angular_jerk > cfg.angular_jerk_lower) {
			path_mode = PATH_EXACT_PATH; 			// downgrade path
		} else if (mm.target_velocity > mm.previous_velocity) {// accels
			mm.initial_velocity = mm.previous_velocity;
		} else { 									// decels and cruises
			mm.initial_velocity = 
							min(mm.previous_velocity, mm.target_velocity);
		}
	} 
	if (path_mode == PATH_EXACT_PATH) {
		if (angular_jerk > cfg.angular_jerk_upper) {	// downgrade
			path_mode = PATH_EXACT_STOP;	
			mm.initial_velocity = 0;
		} else {
			mm.initial_velocity = mm.previous_velocity * 
								  (1 - angular_jerk);	// dip adjustment
		}
	}

	// do the actual work
	ritorno(_mc_compute_regions());	   // compute region lengths & Vt
	ritorno(_mc_recompute_previous(p));// recompute previous tail
	ritorno(_mc_queue_head());		// queue a head if head_length != 0
	ritorno(_mc_queue_body());		// queue a body if body_length != 0
	ritorno(_mc_queue_tail());		// queue a tail if tail_length != 0
	return (TG_OK);
}

/**** ALINE HELPERS ****
 * _mc_get_length()				- get length given Vi and Vt
 * _mc_estimate_angular_jerk()	- factor of 0 to 1 where 1 = max jerk
 * _mc_queue_head()				- queue a 3 part move
 * _mc_queue_body()				- queue a 2 part move with no body
 * _mc_queue_tail()				- queue a 2 part move with no head
 * _mc_queue_line_buffer()  	- helper helper for making line buffers
 * _mc_line_to_arc()			- handle arc cases
 * _mc_compute_regions() 		- compute region lengths and velocities
 * _mc_recompute_previous()		- join previous tail to Vi
 */

/*	
 * _mc_estimate_angular_jerk()
 *
 * The following is borrowed from Simen Svale Skogsrud's Twister project:
 *
 *  Estimate the power of the jerk at the intersection of two motions.
 *	For our application jerk is half the phytagorean magnitude of the 
 *	difference between the unit vector of the two motions which gives 
 *	us a value between 0 and 1.0 where 0 represents no change of 
 *	direction and 1.0 is a full U-turn
 */

static double _mc_estimate_angular_jerk(const struct mcBuffer *p)
{
	double j = (sqrt(square(mm.unit_vec[X] - p->unit_vec[X]) +
				 	 square(mm.unit_vec[Y] - p->unit_vec[Y]) +
				 	 square(mm.unit_vec[Z] - p->unit_vec[Z]))/2.0);
	j *= min(1,(mm.previous_velocity / MAX_VELOCITY)); // +++ remove to test
	return (j);
}

/*	
 * _mc_get_length()
 *
 * 	A convenient expression for determinting the length of a line 
 *	given the starting and ending velocities and the jerk is:
 *
 *		length = abs(end-start) * sqrt(abs(end-start) / max_linear_jerk)
 *
 *	which is derived from these two equations:
 *
 *		time = 2 * sqrt(abs(end-start) / cfg.max_linear_jerk);	// 5.x
 *		length = abs(end-start) * time / 2;						// [2]
 *
 *	Let the compiler optimize out the Vi=0 constant case
 */

inline static double _mc_get_length(double start, double end)
{
	double delta = fabs(start - end);
	return (delta * sqrt(delta / cfg.max_linear_jerk));
}

/*	
 * queue buffers if lengths are non-zero
 */

inline static uint8_t _mc_queue_head() {
	return(_mc_queue_line_buffer(mm.initial_velocity, mm.target_velocity,
								 mm.head_length, MC_TYPE_ACCEL));
}

inline static uint8_t _mc_queue_body() {
	return(_mc_queue_line_buffer(mm.target_velocity, mm.target_velocity,
		  						 mm.body_length, MC_TYPE_CRUISE));
}

inline static uint8_t _mc_queue_tail() {
	return(_mc_queue_line_buffer(mm.target_velocity, mm.final_velocity, 
								 mm.tail_length, MC_TYPE_DECEL));
}

static uint8_t _mc_queue_line_buffer(double Vi, double Vt, double len, uint8_t type)
{
	struct mcBuffer *m;

	if (len < MIN_LINE_LENGTH) { 
		return (TG_OK); 
	}
	if ((m = mc_get_write_buffer()) == NULL) { 	// get a buffer or die trying
		return (TG_BUFFER_FULL_FATAL); 
	} 
	m->starting_velocity = Vi;
	m->ending_velocity = Vt;
	m->length = len;
	for (uint8_t i=0; i < AXES; i++) { 			// copy the unit vector from mm
		m->unit_vec[i] = mm.unit_vec[i]; 
		mm.position[i] += len * m->unit_vec[i];
		m->target[i] = mm.position[i]; 
	}
	return(mc_queue_write_buffer(type));		// queue the buffer and return
}

/*
static uint8_t _mc_line_to_arc(struct mcBuffer *p)
{
	mm.initial_velocity = mm.previous_velocity;	// +++ test various arc join speed changes up and down
	ritorno(_mc_compute_regions());	// compute region lengths & Vt
	ritorno(_mc_queue_body());		// queue a body if body_length != 0
	ritorno(_mc_queue_tail());		// queue a tail if tail_length != 0
	return (TG_OK);
}
*/

/*
 * _mc_compute_regions()
 *
 *	This function computes the region lengths and Vt. It first attempts 
 *	to generate an optimal 3-region line (head, body, tail) - which it can
 *	if sufficient length exists for a head, body and tail at the requested
 *	Vt and the prevailing max jerk.
 *
 *	If it cannot support a full-speed move it adjusts Vt so that the 
 *	acceleration and deceleration regions will obey maximum jerk. This 
 *	means reducing the Vt, omitting the body, and possibly the head. The 
 *	is always computed. In some very short cases the Vi will also be 
 *	reduced to accommodate a tail deceleration to zero.
 *
 *	This function should be called before adjusting the previous tail to 
 *	properly fit the previous tail to the ultimate Vi of the new line.
 *
 *	Cases:
 *
 *	3 regions:	The line supports a head, body and tail. No Vt adjustment
 *				made. Returns 3 regions.
 *
 *	2 regions:	The line can't achieve cruise velocity. Vt is reduced to 
 *				a value where the head and tail can be joined directly
 *				with no intervening body. 
 *				Returns 2 regions w/body_length = 0
 *
 *	1 region:	The line is too short for either of the above. Vt is 
 *				reduced to permit a tail deceleration region only.
 *				Returns tail_region only, head_length = 0 and 
 *				body_length = 0.
 *
 *	0 regions:	Pathological case where the routine was passed a line
 *				below the minimum length. Returns regions & lengths = 0.
 *
 *	The equations to directly compute the new target velocity are not 
 *	practical to solve on this tiny little computer :(   [see spreadsheet]
 *	Instead we perform an iterative linear approximation to converge on
 *	the reduced velocity while preserving the correct total length. 
 */

static uint8_t _mc_compute_regions() 
{
	double Vt = mm.target_velocity;
	double Vt_ = Vt;						// previous pass Vt

	if (mm.length < MIN_LINE_LENGTH) {		// line is too short or zero
		mm.head_length = 0;
		mm.body_length = 0;
		mm.tail_length = 0;
		return (TG_OK);
	}

	// compute optimal head and tail lengths
	mm.tail_length = _mc_get_length(Vt, 0);
	mm.head_length = _mc_get_length(Vt, mm.initial_velocity);
	if (mm.head_length < ROUNDING_ERROR) {
		mm.head_length = 0;
	}
	mm.body_length = mm.length - mm.head_length - mm.tail_length;
	if (mm.body_length > 0) {	// exit if no reduction required
		return (TG_OK);			// 3 region return
	}

	// ----- recompute Vt and lengths for various cases -----

	uint8_t i=0;

	// 2 region case (head and tail)
	if (mm.length > mm.tail_length) {
		while (fabs(mm.body_length) > ROUNDING_ERROR) {
			Vt_ = Vt;		// previous pass value - speeds convergence
			Vt *= mm.length / (mm.head_length + mm.tail_length);
			Vt = (Vt + Vt_)/2;
			mm.tail_length = _mc_get_length(Vt, 0);
			mm.head_length = _mc_get_length(Vt, mm.initial_velocity);
			mm.body_length = mm.length - mm.head_length - mm.tail_length;
			if (i++ > 20) { // chose value with lots of experimentation
#ifdef __UNFORGIVING		// usually converges in ~2 - but not always
				return (TG_FAILED_TO_CONVERGE);
#elif
				break;
#endif
			}
		}
		mm.target_velocity = Vt;
		mm.final_velocity = 0;
		mm.body_length = 0;
		if (mm.head_length > MIN_LINE_LENGTH) {
			return (TG_OK);	// 2 region return
		}
	}
	// In some cases above the new Vt will have become less than the 
	// initial velocity, reducing the 2 region case to a tail-only 
	// case. So you must run it again, below

	// 1 region case (tail-only case)
	if (mm.length <= mm.tail_length) {	// ++++ add in the low Vt case
		i=0;
		while (fabs(mm.length - mm.tail_length) > ROUNDING_ERROR) {
			Vt_ = Vt;
			Vt *= mm.length / mm.tail_length;
			Vt = (Vt + Vt_)/2;
			mm.tail_length = _mc_get_length(Vt, 0);
			if (i++ > 20) { 	// usually converges in ~5 - but not always
#ifdef __UNFORGIVING
				return (TG_FAILED_TO_CONVERGE);
#elif
				break;
#endif
			}
		}
		mm.initial_velocity = Vt;
		mm.target_velocity = Vt; 				
		mm.final_velocity = 0;
		mm.tail_length = mm.length;
		mm.head_length = 0;
		mm.body_length = 0;
		return (TG_OK);	// 1 region return
	}
	return (TG_ERR);	// never should happen. Keep compiler happy
}

/*
 * _mc_recompute_previous()
 *
 *	Recompute the previous move (P) so that its exit velocity (Vpf) 
 *	matches the initial velocity of the current move (Vi). Vpf could be
 *	greater or less than the max velocity of the previous move (Vpt). 
 *	The previous move could be a 1, 2, or 3 region move, so this must 
 *	also be taken into account.
 *
 *	This routine assumes the original tail is in place and the starting 
 *	value for Vpf is therefore zero.
 *
 *	There is a pathological case where the P move needs to be accelerated
 *	to join an arc but can't reach the arc's Vi in the given tail region. 
 *	In this case do the best you can towards the final velocity (and live 
 *	with the velocity step going into the arc).
 */

static uint8_t _mc_recompute_previous(struct mcBuffer *p)
{
	uint8_t i;
	double length;				// desired tail length
	struct mcBuffer *m;			// pointer to active buffer

	// exit if the buffer anything but queued and idle. Leaves Ve=0
	if (p->buffer_state != MC_BUFFER_QUEUED) {
		return (TG_OK);
	}

	// exit if Vi = 0 as no adjustment is required (EXACT STOP mode)
	if (mm.initial_velocity < ROUNDING_ERROR) {
		return (TG_OK);
	}

	// handle the trivial case where Vpe = Vpt = Vi
	if (fabs(mm.initial_velocity - mm.previous_velocity) < ROUNDING_ERROR) {
		p->ending_velocity = mm.previous_velocity;
		p->move_type = MC_TYPE_CRUISE;	// change tail to a cruise
		return (TG_OK);	// no need to update unit_vector or target
	} 

	// compute the new tail length
	length = (_mc_get_length(mm.previous_velocity, mm.initial_velocity));

	// case where new tail length is too short to bother - leave old tail
	if (fabs(length) < MIN_LINE_LENGTH) {
//		p->starting_velocity = p->ending_velocity;
//		p->ending_velocity = mm.initial_velocity;
//		p->length = length;
		return (TG_OK);	
	}

	// convert the old tail into a cruise
	p->ending_velocity = mm.previous_velocity;
	p->move_type = MC_TYPE_CRUISE;
	p->length -= length;
	for (i=0; i < AXES; i++) { 
		p->target[i] -= length * p->unit_vec[i];
	}

	// make a new tail (normal cases)
	if ((m = mc_get_write_buffer()) == NULL) {
		return (TG_BUFFER_FULL_FATAL); 
	} 
	p = m->pv;
	m->starting_velocity = p->ending_velocity;
	m->ending_velocity = mm.initial_velocity;
	m->length = length;
	for (uint8_t i=0; i < AXES; i++) {
		m->unit_vec[i] = p->unit_vec[i];
		m->target[i] = mm.position[i]; // use end of prev move as target
	}
	mc_queue_write_buffer(MC_TYPE_DECEL);
	return (TG_OK);
}

/**** ALINE RUN ROUTINES ****
 *	_mc_run_cruise()
 *	_mc_run_accel()
 *	_mc_run_decel()
 *	_mc_aline_run_segment()	- helper code for running a segment
 *	_mc_aline_run_finalize() - helper code for running last segment
 *
 *	Note to self: Returning TG_OK from these routines ends the aline
 *	Returning TG_EAGAIN (or any other non-zero value) continues iteration 
 */

static uint8_t _mc_run_cruise(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	if (m->length < MIN_LINE_LENGTH) {			// toss the line
		return (TG_OK);
	}
	m->time = m->length / m->ending_velocity;	// get time from length
	mr.microseconds = uSec(m->time);

	uint32_t steps[AXES];
	for (uint8_t i=0; i < AXES; i++) {
		mr.target[i] = m->target[i];
		m->target[i] = mr.position[i] + m->unit_vec[i] * m->length; //++++ remove this line for test
		steps[i] = _steps(i, m->target[i]) - _steps(i, mr.position[i]);
	}
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mc_set_run_position(m->target);
	return (TG_OK);
}

static uint8_t _mc_run_accel(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for acceleration
	if (m->move_state == MC_STATE_NEW) {
		if (m->length < MIN_LINE_LENGTH) { return (TG_OK); }	// toss
		mr.midpoint_velocity = (m->starting_velocity + m->ending_velocity) / 2;
		mr.time = m->length / mr.midpoint_velocity;
		mr.midpoint_acceleration = mr.time * mr.linear_jerk_div2;
		for (uint8_t i=0; i < AXES; i++) {
			mr.target[i] = m->target[i];	// transfer target to mr
		}
		// number of segments in *each half*
		mr.segments = round(round(uSec(mr.time / cfg.min_segment_time)) / 2);
		if (isinf(mr.segment_time = mr.time / (2 * mr.segments))) {
			return (TG_OK);					// cancel the move if too small
		}
		mr.microseconds = uSec(mr.segment_time);
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time /2; //compute pos'n from midpoint
		m->move_state = MC_STATE_RUNNING_1;
	}
	// first half of acceleration - concave portion of curve
	if (m->move_state == MC_STATE_RUNNING_1) {
		mr.segment_velocity = m->starting_velocity + 
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno (_mc_aline_run_segment(m));
		// setup for second half
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time /2;
		m->move_state = MC_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a buffer
	}
	// second half of acceleration - convex portion of curve
	if (m->move_state == MC_STATE_RUNNING_2) {
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity + 
						(mr.elapsed_time * mr.midpoint_acceleration) -
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mc_aline_run_segment(m));
		} else {
			return(_mc_aline_run_finalize(m));	// for accuracy
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mc_run_decel(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for deceleration
	if (m->move_state == MC_STATE_NEW) {
		if (m->length < MIN_LINE_LENGTH) { return (TG_OK); } // toss
		mr.midpoint_velocity = (m->starting_velocity + m->ending_velocity) / 2;
		mr.time = m->length / mr.midpoint_velocity;
		mr.midpoint_acceleration = mr.time * mr.linear_jerk_div2;
		for (uint8_t i=0; i < AXES; i++) {
			mr.target[i] = m->target[i];	// transfer target
		}
		// number of segments in *each half*
		mr.segments = round(round(uSec(mr.time / cfg.min_segment_time)) / 2);
		if (isinf(mr.segment_time = mr.time / (2 * mr.segments))) {
			return (TG_OK);					// cancel the move if too small
		}
		mr.microseconds = uSec(mr.segment_time);
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time / 2;
		m->move_state = MC_STATE_RUNNING_1;
	}
	// first half of deceleration
	if (m->move_state == MC_STATE_RUNNING_1) {	// concave part of curve
		mr.segment_velocity = m->starting_velocity - 
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno(_mc_aline_run_segment(m));
		// setup for second half
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time / 2;
		m->move_state = MC_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a buffer
	}
	// second half of deceleration
	if (m->move_state == MC_STATE_RUNNING_2) {	// convex part of curve
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity - 
						(mr.elapsed_time * mr.midpoint_acceleration) +
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mc_aline_run_segment(m));
		} else {
			return(_mc_aline_run_finalize(m));	// for accuracy
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mc_aline_run_segment(struct mcBuffer *m)
{
	uint32_t steps[AXES];

	/* Multiply the computed position by the unit vector to get the 
	 * contribution for each axis. Set the target in absolute coords
	 * (floating point) and compute the relative steps.
	 */
	for (uint8_t i=0; i < AXES; i++) {
		m->target[i] = mr.position[i] + (m->unit_vec[i] * 
					   mr.segment_velocity * mr.segment_time);
		steps[i] = _steps(i, m->target[i]) - _steps(i, mr.position[i]);
	}
	// queue the line and adjust the variables for the next iteration
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	mr.elapsed_time += mr.segment_time;
	_mc_set_run_position(m->target);
	if (--mr.segment_count) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

static uint8_t _mc_aline_run_finalize(struct mcBuffer *m)
{
	// finalize - do the last segment to maintain position accuracy
	mr.length = sqrt(square(mr.target[X] - mr.position[X]) +
					 square(mr.target[Y] - mr.position[Y]) +
					 square(mr.target[Z] - mr.position[Z]) +
					 square(mr.target[A] - mr.position[A]));

	if (mr.length < MIN_LINE_LENGTH) {		// trap zero-length case
		return (TG_OK);
	}
	mr.time = mr.length / m->ending_velocity; // get time from length
	mr.microseconds = uSec(mr.time);

	uint32_t steps[AXES];
	for (uint8_t i=0; i < AXES; i++) {
		steps[i] = _steps(i, mr.target[i]) - _steps(i, mr.position[i]);
	}
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mc_set_run_position(mr.target);
	return (TG_OK);
}


//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _mc_test_buffers(void);
void _mc_test_recompute_vt(void);
void _mc_call_recompute_vt(double l, double Vp, double Vi, double Vt);


void mc_unit_tests()
{
//	_mc_test_buffers();
	_mc_test_recompute_vt();
}

void _mc_call_recompute_vt(double l, double Vp, double Vi, double Vt) 
{
	mm.length = l;
	mm.previous_velocity = Vp;
	mm.initial_velocity = Vi;
	mm.target_velocity = Vt;
	mm.head_length = _mc_get_length(mm.target_velocity, mm.initial_velocity);
	mm.tail_length = _mc_get_length(mm.target_velocity, 0);
	_mc_recompute_velocity();
}

void _mc_test_recompute_vt()
{
//						  Len	Vp	 Vi	  Vt
	_mc_call_recompute_vt( 3.0, 250, 100, 400);	// 3 regions - fits
	_mc_call_recompute_vt( 2.0, 250, 100, 400);	// 2 regions - simple reduction
	_mc_call_recompute_vt( 1.0, 250, 100, 400);	// 1 region - more extreme reduction
	_mc_call_recompute_vt( 0.5, 250, 100, 400);	// 1 region - Vi reduces below Vp
												// 1 region - zero legnth line
	_mc_call_recompute_vt( MIN_LINE_LENGTH/2, 250, 100, 400);	
}

void _mc_test_buffers()
{
	mc_test_write_buffer(MC_BUFFERS_NEEDED); // test for enough free buffers

	mc_get_write_buffer();		// open a write buffer [0]
	mc_get_write_buffer();		// open a write buffer [1]
	mc_get_write_buffer();		// open a write buffer [2]

	mc_get_run_buffer();		// attempt to get run buf - should fail (NULL)

	mc_queue_write_buffer(MC_TYPE_ACCEL);	// queue the write buffer [0]
	mc_queue_write_buffer(MC_TYPE_CRUISE);	// queue the write buffer [1]
	mc_queue_write_buffer(MC_TYPE_DECEL);	// queue the write buffer [2]

	mc_get_run_buffer();		// attempt to get run buf - should succeed

/*
	mc_get_write_buffer();		// open the next write buffer [1]
	mc_queue_write_buffer(MC_TYPE_ACCEL);	// commit it
	mc_get_write_buffer();		// open the next write buffer [2]
	mc_queue_write_buffer(MC_TYPE_ACCEL);	// commit it
	mc_get_write_buffer();		// attempt write buffer - should fail (NULL)
	mc_end_run_buffer();
	mc_get_write_buffer();		// now if should succeed
*/
}

#endif
