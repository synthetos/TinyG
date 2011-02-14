/*
 * planner.c - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */
/* --- TinyG Notes ----
 *
 *	This layer works below the canonical machine.and above the motor
 *	mapping and queues. It is responsible only for cartesian motions.
 *	The calls to the routines are simple and do not need to know about
 *	the state of the gcode model. A rudimentary multitasking capability 
 *	is implemented for lines, arcs, dwells, and program control. 
 *
 *	Routines are coded as non-blocking continuations - which are simple 
 *	state machines that are re-entered multiple times until a particular 
 *	operation is complete (like queuing an arc).
 */

#include "xmega_init.h"				// put before <util/delay.h>
#include <util/delay.h>				// needed for dwell
#include <stdlib.h>
#include <math.h>
//#include <avr/io.h>

#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "settings.h"
#include "planner.h"
#include "motor_queue.h"
#include "stepper.h"

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// b->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_RUNNING			// current running buffer
};

enum mcMoveType {				// b->move_type values 
	MP_TYPE_NULL = 0,			// null move
	MP_TYPE_ACCEL,				// controlled jerk acceleration region
	MP_TYPE_CRUISE,				// cruise at fixed velocity
	MP_TYPE_DECEL,				// controlled jerk deceleration region
	MP_TYPE_LINE,				// simple line
	MP_TYPE_ARC,				// arc feed
	MP_TYPE_DWELL,				// delay with no movement
	MP_TYPE_START,				// restart motors
	MP_TYPE_STOP,				// stop motors
	MP_TYPE_END					// stop motors and end program
};

enum mcMoveState {				// b->move_state values
	MP_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MP_STATE_RUNNING_1,			// first half of move or sub-move
	MP_STATE_RUNNING_2,			// second half of move or sub-move
	MP_STATE_FINALIZE,			// finalize the move or sub-move
	MP_STATE_END				// force the move to end (kill)
};
#define MP_STATE_RUNNING MP_STATE_RUNNING_1	// a convenience for above

enum mpPlanState {				// descibes the state in M, P and B structs
	MP_PLAN_NULL = 0,			// zero-length move
	MP_PLAN_1_REGION,			// move meets final target only
	MP_PLAN_2_REGION,			// move meets initial and final targets
	MP_PLAN_3_REGION,			// move meets all targets
};

struct mpBuffer {				// move/sub-move motion control structure
	struct mpBuffer *nx;		// static pointer to next buffer
	struct mpBuffer *pv;		// static pointer to previous buffer
	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// state machine sequence

	double target[AXES];		// target position in floating point
	double unit_vec[AXES];		// axis scaling & jerk computation

	double time;				// line, helix or dwell time in minutes
	double length;				// line or helix length in mm
	double start_velocity;		// actual starting velocity of a region
	double end_velocity;		// actual ending velocity of a region
	double request_velocity;	// requested initial, target, or end velocity
								// for head, body, or tail, respectively

	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mpBufferPool {			// ring buffer for sub-moves
	struct mpBuffer *w;			// get_write_buffer pointer
	struct mpBuffer *q;			// queue_write_buffer pointer
	struct mpBuffer *r;			// get/end_run_buffer pointer
	struct mpBuffer b[MP_BUFFER_SIZE];// buffer storage
};

struct mpMotionMaster {	
	double position[AXES];		// final move position
	double target[AXES];		// target move position
	double unit_vec[AXES];		// for axis scaling and jerk computation
};

struct mpMovePlanner {			// used to compute or recompute regions
	uint8_t	plan_state;			// state of move plan
								// buffer pointers
	struct mpBuffer *prev;		// pointer to tail of previous move
	struct mpBuffer *head;		// pointer to head of current move
	struct mpBuffer *body;		// pointer to body of current move
	struct mpBuffer *tail;		// pointer to tail of current move
	struct mpBuffer *next;		// pointer to head of next move

	double length;				// length of line or helix in mm
	double head_length;			// computed for trajectory planning
	double body_length;			// redundant, but useful
	double tail_length;			// computed for trajectory planning

	double initial_velocity_req;// requested initial velocity
	double initial_velocity;	// actual initial velocity
	double target_velocity; 	// requested target velocity 
	double cruise_velocity;		// actual achieved velocity
	double final_velocity;		// actual exit velocity
};

struct mpMoveRuntime {		// persistent runtime variables
	uint8_t run_flag;			// move status
	uint8_t (*run_move)(struct mpBuffer *m); // currently running move

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

static struct mpMotionMaster mm;
static struct mpMovePlanner mp[2];	// current and fwd or bkwd neighbor
static struct mpMoveRuntime mr;
static struct mpBufferPool mb;

/*
 * Local Scope Data and Functions
 */

static uint8_t _mp_run_null(struct mpBuffer *b);
static uint8_t _mp_run_accel(struct mpBuffer *b);
static uint8_t _mp_run_cruise(struct mpBuffer *b);
static uint8_t _mp_run_decel(struct mpBuffer *b);
static uint8_t _mp_run_line(struct mpBuffer *b);
static uint8_t _mp_run_arc(struct mpBuffer *b);
static uint8_t _mp_run_dwell(struct mpBuffer *b);
static uint8_t _mp_run_stops(struct mpBuffer *b);
static uint8_t _mp_aline_run_segment(struct mpBuffer *b);
static uint8_t _mp_aline_run_finalize(struct mpBuffer *b);

static void _mp_set_mm_position(double target[AXES]) ;
static void _mp_set_mr_position(double target[AXES]);

static uint8_t _mp_queue_move(struct mpMovePlanner *m);
static uint8_t _mp_update_move(struct mpMovePlanner *m);
static struct mpBuffer *_mp_queue_buffer(double Vi, double Vt, double Vr, double len, uint8_t type);
static double _mp_estimate_angular_jerk(const struct mpBuffer *p, double previous_velocity);
static double _mp_get_length(double Vi, double Vt);

static uint8_t _mp_compute_regions(double Vi, double Vt, double Vf, struct mpMovePlanner *m);
static uint8_t _mp_construct_prev_move(struct mpMovePlanner *p, struct mpMovePlanner *m);
static uint8_t _mp_recompute_backwards(struct mpMovePlanner *m);
static uint8_t _mp_recompute_forwards(struct mpMovePlanner *p, struct mpMovePlanner *m);


// p.s. I tried listing variables both ways: target_velocity or Vt,
//		initial_velocity or Vi, etc. and found the first way easier to 
//		read in spite of the wrapped lines.

/* 
 * mp_init()
 *
 * The memset does:
 *	- clears all values
 *	- sets buffer states to MP_EMPTY
 *	- sets other states to their zero values - which is typically OFF
 */

void mp_init()
{
	struct mpBuffer *pv;
	uint8_t i=0;

	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	memset(&mb, 0, sizeof(mb));	// clear all values, pointers and status

	mb.w = &mb.b[0];			// init write and read buffer pointers
	mb.q = &mb.b[0];
	mb.r = &mb.b[0];
	pv = &mb.b[MP_BUFFER_SIZE-1];
	for (i=0; i < MP_BUFFER_SIZE; i++) {  // setup ring pointers
		mb.b[i].nx = &mb.b[_mp_bump(i)];
		mb.b[i].pv = pv;
		pv = &mb.b[i];
	}
}

/* 
 * mp_move_dispatcher() - routine for dequeuing and executing moves
 *
 *	Dequeues the buffer queue and executes the move run continuations.
 *	Manages run buffers and other details.
 *	Responsible for freeing the completed run buffers
 *	Runs as a continuation itself; called from tg_controller()
 */

uint8_t mp_move_dispatcher(uint8_t kill) 
{
	uint8_t status;
	struct mpBuffer *b;

	if ((b = mp_get_run_buffer()) == NULL) {	// NULL means nothing's running
		return (TG_NOOP);
	}
	if (kill) {
		b->move_state = MP_STATE_END;
		mr.run_flag = FALSE;				// finalize and return
		mp_end_run_buffer();
		return (TG_OK);
	}
	if (b->move_state == MP_STATE_NEW) {	// first time in?
		mr.run_flag = TRUE;					// it's useful to have a flag
		switch (b->move_type) { 			// setup the dispatch vector
			case MP_TYPE_NULL:	{ mr.run_move = _mp_run_null; break; }
			case MP_TYPE_ACCEL:	{ mr.run_move = _mp_run_accel; break; }
			case MP_TYPE_CRUISE:{ mr.run_move = _mp_run_cruise; break; }
			case MP_TYPE_DECEL:	{ mr.run_move = _mp_run_decel; break; }
			case MP_TYPE_LINE:	{ mr.run_move = _mp_run_line; break; }
			case MP_TYPE_ARC:	{ mr.run_move = _mp_run_arc; break; }
			case MP_TYPE_DWELL:	{ mr.run_move = _mp_run_dwell; break; }
			case MP_TYPE_START:	{ mr.run_move = _mp_run_stops; break;}
			case MP_TYPE_STOP:	{ mr.run_move = _mp_run_stops; break;}
			case MP_TYPE_END: 	{ mr.run_move = _mp_run_stops; break;}
		}
	}
	if ((status = mr.run_move(b)) == TG_EAGAIN) { // run current run buf
		return (TG_EAGAIN);
	}
	mr.run_flag = FALSE;				// finalize and return
	mp_end_run_buffer();
	return (status);
}

/**** MOVE QUEUE ROUTINES ************************************************
 * mp_test_write_buffer()  Return TRUE if N write buffers are available
 *
 * mp_get_write_buffer()   Get pointer to next available write buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Multiple write buffers may be open at once
 *
 * mp_queue_write_buffer() Commit the next write buffer to the queue
 *						   Write buffers will queue in order gotten,
 *						   and will run in the order queued.
 *						   Advances write pointer & changes buffer state
 *
 * mp_unget_write_buffer() Free write buffer if you decide not to queue it
 *						   Only works on most recently gotten write buffer
 *						   You could work your way back in a set or buffers
 *						   Use this one carefully.
 *
 * mp_get_run_buffer()	   Get pointer to the next or current run buffer
 *						   Returns a new run buffer if prev buf was ENDed
 *						   Returns same buf if called again before ENDing
 *						   Returns NULL if no buffer available
 *						   The behavior supports continuations (iteration)
 *
 * mp_end_run_buffer()	   Release the run buffer & return to buffer pool
 *						   End_run causes get_run to return the next buffer
 *
 * mp_get_prev_buffer_implicit() Return pointer to the buffer immediately 
 *						   before the next available write buffer. From
 *						   there earlier buffers can be read using the 
 *						   backwards pointers. This buffer cannot be 
 *						   queued and should not be ENDed.
 *
 * mp_get_prev_buffer()	   Return pointer to prev buffer in linked list
 * mp_get_next_buffer()	   Return pointer to next buffer in linked list 
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

uint8_t mp_test_write_buffer(const uint8_t count) 
{
	struct mpBuffer *w = mb.w;	// temp write buffer pointer

	for (uint8_t i=0; i < count; i++) {
		if (w->buffer_state != MP_BUFFER_EMPTY) {
			return (FALSE);
		}
		w = w->nx;
	}
	return (TRUE);
}

struct mpBuffer * mp_get_write_buffer() 
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
		struct mpBuffer *w = mb.w;
		struct mpBuffer *nx = mb.w->nx;	// save pointers
		struct mpBuffer *pv = mb.w->pv;
		memset(mb.w, 0, sizeof(struct mpBuffer));
		w->nx = nx;			// restore pointers
		w->pv = pv;
		w->buffer_state = MP_BUFFER_LOADING;
		mb.w = w->nx;
		return (w);
	}
	return (NULL);
}

uint8_t mp_queue_write_buffer(const uint8_t move_type)
{
	mb.q->move_type = move_type;
	mb.q->move_state = MP_STATE_NEW;
	mb.q->buffer_state = MP_BUFFER_QUEUED;
	mb.q = mb.q->nx;		// advance the queued buffer pointer
	return (TG_OK);			// convenience for calling routines
}

void mp_unget_write_buffer()
{
	mb.w->buffer_state = MP_BUFFER_EMPTY;
	mb.w = mb.w->pv;
}

struct mpBuffer * mp_get_run_buffer() 
{
	// condition: fresh buffer; buffer becomes running if it's queued
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {
		mb.r->buffer_state = MP_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mb.r->buffer_state == MP_BUFFER_RUNNING) { // return same buffer
		return (mb.r);
	}
	return (NULL);		// condition: no queued buffers. fail it.
}

uint8_t mp_end_run_buffer()	// EMPTY current run buf & advance to next
{
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;		// advance to next run buffer
	return (TG_OK);			// convenience for calling routines
}

struct mpBuffer * mp_get_prev_buffer_implicit()
{
	return (mb.w->pv);
}

struct mpBuffer * mp_get_prev_buffer(struct mpBuffer *b)
{
	return (b->pv);
}

struct mpBuffer * mp_get_next_buffer(struct mpBuffer *b)
{
	return (b->nx);
}

/* 
 * mp_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mp_isbusy()
{
	if (st_isbusy() || mr.run_flag) {
		return (TRUE);
	}
	return (FALSE);
}

/**** POSITION SETTING ROUTINES ******************************************
 * mp_set_position() 		  - set current MC position (support for G92)
 * _mp_set_mm_position()	  - set move final position for traj planning
 * _mp_set_mr_position() 	  - set move/sub-move position for runtime
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
uint8_t mp_set_position(const double x, const double y, const double z, const double a)
{
	mm.position[X] = x;
	mm.position[Y] = y;
	mm.position[Z] = z; 
	mm.position[A] = a; 
	_mp_set_mr_position(mm.position);
	return (TG_OK);
}

// set move final position for trajectory planning
inline static void _mp_set_mm_position(double target[AXES]) 
{ 
	mm.position[X] = target[X];
	mm.position[Y] = target[Y];
	mm.position[Z] = target[Z];
	mm.position[A] = target[A];
}

// set move/sub-move runtime position
inline static void _mp_set_mr_position(double target[AXES]) 
{ 
	mr.position[X] = target[X];
	mr.position[Y] = target[Y];
	mr.position[Z] = target[Z];
	mr.position[A] = target[A];
}

/*************************************************************************
 * _mp_run_null() - null move
 *
 * Removes a null beffer from the queue
 */

static uint8_t _mp_run_null(struct mpBuffer *b)
{
	return (TG_OK);		// will free the buffer on return
}


/**** STOP START AND END ROUTINES ****************************************
 * mp_async_stop() 	- stop current motion immediately
 * mp_async_start() - (re)start motion
 * mp_async_end() 	- stop current motion immediately
 *
 *	These routines must be safe to call from ISRs. Mind the volatiles.
 */

uint8_t mp_async_stop()
{
	st_stop();						// stop the steppers
	return (TG_OK);
}

uint8_t mp_async_start()
{
	st_start();						// start the stoppers
	return (TG_OK);
}

uint8_t mp_async_end()
{
	st_end();						// stop the motion
	mp_move_dispatcher(TRUE);		// kill the current move
	return (TG_OK);
}

/* 
 * mp_queued_stop() 	- queue a motor stop
 * mp_queued_start()	- queue a motor start
 * mp_queued_end()		- end current motion and program
 * _mp_run_start_stop()	- start and stop continuation
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

uint8_t mp_queued_stop() 
{
	if (mp_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mp_queue_write_buffer(MP_TYPE_STOP));
}

uint8_t mp_queued_start() 
{
	if (mp_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mp_queue_write_buffer(MP_TYPE_START));
}

uint8_t mp_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	if (mp_get_write_buffer() == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (mp_queue_write_buffer(MP_TYPE_END));
}

static uint8_t _mp_run_stops(struct mpBuffer *b) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_stops(b->move_type);
	return (TG_OK);
}

/*************************************************************************
 * mp_dwell() 		- queue a dwell
 * _mp_run_dwell()	- dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the move but does not 
 * send any pulses. Only the Z axis is used to time the dwell - 
 * the others are idle.
 */

uint8_t mp_dwell(double seconds) 
{
	struct mpBuffer *b; 

	if ((b = mp_get_write_buffer()) == NULL) { // get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);		   // (not supposed to fail)
	}
	b->time = seconds;						   // in seconds, not minutes
	return (mp_queue_write_buffer(MP_TYPE_DWELL));
}

static uint8_t _mp_run_dwell(struct mpBuffer *b)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	mq_queue_dwell(b->time * 1000000);		   // convert seconds to uSec
	return (TG_OK);
}

/*************************************************************************
 * mp_line() 	  - queue a linear move (simple version - no accel/decel)
 * _mp_run_line() - run a line to generate and load a linear move
 *
 * Compute and queue a line segment to the move buffer.
 * Executes linear motion in absolute millimeter coordinates. 
 * Feed rate has already been converted to time (minutes).
 * Zero length lines are skipped at this level. 
 * 
 * The run_line routine is a continuation and can be called multiple times 
 * until it can successfully load the line into the move buffer.
 */

uint8_t mp_line(double x, double y, double z, double a, double minutes)
{
	struct mpBuffer *b;

	if ((b = mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// capture the function args and compute line length
	b->target[X] = x;		// targets for the move
	b->target[Y] = y;
	b->target[Z] = z;
	b->target[A] = a;

	if ((b->time = minutes) == 0) {
		mp_unget_write_buffer();		// early exit requires free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	b->length = sqrt(square(b->target[X] - mr.position[X]) +
					 square(b->target[Y] - mr.position[Y]) +
					 square(b->target[Z] - mr.position[Z]) +
					 square(b->target[A] - mr.position[A]));

	if (b->length < MIN_LINE_LENGTH) {	// trap zero-length lines
		mp_unget_write_buffer();		// early exit requires free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	ritorno(mp_queue_write_buffer(MP_TYPE_LINE));
	_mp_set_mm_position(b->target);		// set mm position for planning
	return(TG_OK);
}

static uint8_t _mp_run_line(struct mpBuffer *b) 
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}

	uint32_t steps[AXES];
	for (uint8_t i=0; i < AXES; i++) {
		steps[i] = _steps(i, b->target[i]) - _steps(i, mr.position[i]);
	}
	mr.microseconds = uSec(b->time);
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mp_set_mr_position(b->target);		// set mr position for runtime
	return (TG_OK);
}

/************************************************************************* 
 * mp_arc() 	 - setup and queue an arc move
 * _mp_run_arc() - generate an arc
 *
 * Generates an arc by queueing line segments to the move buffer.
 * The arc is approximated by generating a large number of tiny, linear
 * segments. The length of the segments is configured in motion_control.h
 * as MM_PER_ARC_SEGMENT.
 *
 * mp_arc()
 *	Loads a move buffer with calling args and initialization values
 *
 * _mp_run_arc() 
 *	run_arc() is structured as a continuation called by mp_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 * Note on mq_test_MP_BUFFER_full()
 *	The move buffer is tested and sometime later its queued (via mp_line())
 *	This only works because no ISRs queue this buffer, and the arc run 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */

uint8_t mp_arc( double x, double y, double z, double a, 
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
	struct mpBuffer *b; 

	if ((b = mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	b->length = hypot(angular_travel * radius, labs(linear_travel));	
	if (b->length < cfg.mm_per_arc_segment) { // too short to draw
		mp_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the move struct for an arc
	// note: m->target is for debugging convenience and not actually used
	b->target[X] = x;
	b->target[Y] = y;
	b->target[Z] = z;
	b->target[A] = a;
	b->time = minutes;
	b->theta = theta;
	b->radius = radius;
	b->axis_1 = axis_1;
	b->axis_2 = axis_2;
	b->axis_linear = axis_linear;
	b->angular_travel = angular_travel;
	b->linear_travel = linear_travel;
	b->start_velocity = b->length / b->time;	// for trajectory planning
	b->end_velocity = b->start_velocity;	 	// for consistency

	double length;
	length = sqrt(square(b->target[axis_1] - i) +
				  square(b->target[axis_2] - j) +
				  square(b->target[axis_linear] - k));

	//	Compute unit vector
	// I think you can take the normal of the vector between the 
	//	center point (i,j) and the target (x,y) and divide by the 
	//	length of (i,j) to (x,y). Must also account for plane-axes
	//	and the linear axis.
/*
	double offset[3] = {i, j, k};
	for (uint8_t i=0; i < 3; i++) {
		b->unit_vec[i] = (b->target[i] - offset[i]) / length;
	}
*/
	_mp_set_mm_position(b->target);		// set mm position for planning
	return(mp_queue_write_buffer(MP_TYPE_ARC));
}

static uint8_t _mp_run_arc(struct mpBuffer *b) 
{
	uint8_t i;

	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize arc variables
	if (b->move_state == MP_STATE_NEW) {
		mr.segments = ceil(b->length / cfg.mm_per_arc_segment);
		mr.segment_count = mr.segments;
		mr.segment_theta = b->angular_travel / mr.segments;
		mr.segment_length = b->linear_travel / mr.segments;
 		mr.microseconds = uSec(b->time / mr.segments);
		mr.center_1 = mr.position[b->axis_1] - sin(b->theta) * b->radius;
		mr.center_2 = mr.position[b->axis_2] - cos(b->theta) * b->radius;
		mr.target[b->axis_linear] = mr.position[b->axis_linear];
		b->move_state = MP_STATE_RUNNING;
	}
	// compute an arc segment and exit
	if (b->move_state == MP_STATE_RUNNING) {
		b->theta += mr.segment_theta;
		mr.target[b->axis_1] = mr.center_1 + sin(b->theta) * b->radius;
		mr.target[b->axis_2] = mr.center_2 + cos(b->theta) * b->radius;
		mr.target[b->axis_linear] += mr.segment_length;

		uint32_t steps[AXES];
		for (i=0; i < AXES; i++) {
			steps[i] = _steps(i, mr.target[i]) - _steps(i, mr.position[i]);
		}
		mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
		_mp_set_mr_position(mr.target);
		if (--mr.segment_count) {
			return (TG_EAGAIN);
		}
	}
	return (TG_OK);
}

/*************************************************************************
 * mp_aline() 		- queue line move with acceleration / deceleration
 * _mp_run_aline()	- run accel/decel move 
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
 *	by mp_move_dispatcher()
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

uint8_t mp_aline(double x, double y, double z, double a, double minutes)
{
	uint8_t i;
	uint8_t path_mode;
	double angular_jerk;
	double previous_velocity;
	struct mpBuffer *p; 		// previous tail buffer pointer
	struct mpMovePlanner *m = &mp[0];
	
	// capture the function args and compute line length
	mm.target[X] = x;
	mm.target[Y] = y;
	mm.target[Z] = z;
	mm.target[A] = a;
	
	// setup initial values
	m->length = sqrt(square(mm.target[X] - mm.position[X]) +
					 square(mm.target[Y] - mm.position[Y]) +
					 square(mm.target[Z] - mm.position[Z]) +
					 square(mm.target[A] - mm.position[A]));

	if (m->length < MIN_LINE_LENGTH) {			// trap zero-length lines
		return (TG_ZERO_LENGTH_MOVE);
	}
	m->target_velocity = m->length / minutes;		// Vt requested
	m->initial_velocity_req = 0;					// Vi requested starting value
	path_mode = cfg.gcode_path_control;				// starting path mode
	for (i=0; i < AXES; i++) {						// compute unit vector
		mm.unit_vec[i] = (mm.target[i] - mm.position[i]) / m->length;
	}
	mr.linear_jerk_div2 = cfg.max_linear_jerk / 2;	// init linear jerk term

	// setup initial conditions from the previous move
	p = mp_get_prev_buffer_implicit();
	if ((p->move_type == MP_TYPE_ARC) && 
		(p->buffer_state != MP_BUFFER_EMPTY)) {		// q'd or running arc
		previous_velocity = p->end_velocity;
		m->initial_velocity_req = previous_velocity;// +++ test various arc join speed changes up and down

		// compute region lengths & Vt, w/Vf=0
		ritorno(_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m));
		ritorno(_mp_queue_move(m));
		return (TG_OK);

	} else if (p->buffer_state == MP_BUFFER_QUEUED){// q'd but not running
		previous_velocity = p->start_velocity;		// Vt of previous move
	} else {
		previous_velocity = 0;
		path_mode = PATH_EXACT_STOP;				// downgrade path mode
	}
	// getting angular jerk requires unit vectors and mm.previous_velocity
	angular_jerk = _mp_estimate_angular_jerk(p, previous_velocity);	// for path downgrades

	// setup initial velocity and do path downgrades
	if (path_mode == PATH_CONTINUOUS) {
		if (angular_jerk > cfg.angular_jerk_lower) {
			path_mode = PATH_EXACT_PATH; 			// downgrade path
		} else if (m->target_velocity > previous_velocity) {// accels
			m->initial_velocity_req = previous_velocity;
		} else { 									// decels and cruises
			m->initial_velocity_req = min(previous_velocity, m->target_velocity);
		}
	} 
	if (path_mode == PATH_EXACT_PATH) {
		if (angular_jerk > cfg.angular_jerk_upper) {	// downgrade
			path_mode = PATH_EXACT_STOP;	
			m->initial_velocity_req = 0;
		} else {
			m->initial_velocity_req = previous_velocity * (1 - angular_jerk);	// dip adjustment
		}
	}

	// do the actual work
	ritorno(_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m));
	ritorno(_mp_queue_move(m));
	ritorno(_mp_recompute_backwards(m));
//	ritorno(_mp_recompute_forwards(m));
	return (TG_OK);
}

/**** ALINE HELPERS ****
 * _mp_compute_regions() 		- compute region lengths and velocities
 * _mp_recompute_backwards()	- recompute moves backwards from latest move
 * _mp_recompute_forwards()		- recompute moves forwards from running move
 * _mp_get_length()				- get length given Vi and Vt
 * _mp_estimate_angular_jerk()	- factor of 0 to 1 where 1 = max jerk
 * _mp_queue_move()				- queue 3 regions of a move
 * _mp_queue_buffer() 		 	- helper helper for making line buffers
 * _mp_update_move()			- update a move after a replan
 * _mp_line_to_arc()			- handle arc cases
 */

/*
 * _mp_compute_regions()
 *
 *	This function computes the region lengths and Vt. It first attempts 
 *	to generate an optimal 3-region line (head, body, tail) - which it can
 *	if sufficient length exists for a head, body and tail at the requested
 *	Vt and the prevailing max jerk.
 *
 *	If it cannot support a full-speed move it adjusts Vt so that the 
 *	acceleration and deceleration regions will obey maximum jerk. This 
 *	means reducing the Vt, omitting the body, and possibly the head. The 
 *	tail is always computed. In some very short cases the Vi will also be
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
 *	2 regions:	The line cruises at Vi then decelerates to final velocity 
 *				Vt is reduced to Vi. 
 *				Returns 2 regions w/head_length = 0
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
 *
 *  Inputs:
 * 		Vi = initial velocity requested
 *		Vt = target velocity requested (cruise velocity)
 *		Vf = final velocity requested
 *
 */
static uint8_t _mp_compute_regions(double Vi, double Vt, double Vf, struct mpMovePlanner *m) 
{
	double Vt_ = Vt;					// previous iteration's Vt
	double temp_tail;

	// ----- 0 region case - line is too short of zero length -----

	if (m->length < MIN_LINE_LENGTH) {	// line is too short or zero
		m->head_length = 0;
		m->body_length = 0;
		m->tail_length = 0;
		m->plan_state = MP_PLAN_NULL;	// indicates zero length move
		return (TG_OK);
	}
	// setup M struct
	m->initial_velocity_req = Vi;		// requested value
	m->initial_velocity = Vi;			// value achieved
	m->target_velocity = Vt;			// requested value
	m->cruise_velocity = Vt;			// value achieved
	m->final_velocity = Vf;				// this one never changes

	// compute optimal head and tail lengths
	m->tail_length = _mp_get_length(Vt, Vf);
	m->head_length = _mp_get_length(Vt, Vi);
	if (m->head_length < ROUNDING_ERROR) {
		m->head_length = 0;
	}

	// ----- 3 region case - no need to recompute Vt and lengths -----

	m->body_length = m->length - m->head_length - m->tail_length;
	if (m->body_length > 0) {	// exit if no reduction required
		m->plan_state = MP_PLAN_3_REGION;
		return (TG_OK);			// 3 region return
	}

	// ----- 2 region case (body and tail, where Vi > Vf) ----- 
	temp_tail = _mp_get_length(Vi, Vf);
	if (m->length > temp_tail) {
		m->head_length = 0;
		m->tail_length = temp_tail;
		m->body_length = m->length - m->tail_length;
		m->cruise_velocity = Vi;
		m->final_velocity = Vf;
		m->plan_state = MP_PLAN_2_REGION;
		return (TG_OK);	// 2 region return
	}

	uint8_t i=0;

	// ----- 2 region case (head and tail) -----
	if (m->length > m->tail_length) {
		while (fabs(m->body_length) > ROUNDING_ERROR) {
			Vt_ = Vt;		// previous pass value - speeds convergence
			Vt *= m->length / (m->head_length + m->tail_length);
			Vt = (Vt + Vt_)/2;
			m->tail_length = _mp_get_length(Vt, Vf);
			m->head_length = _mp_get_length(Vt, Vi);
			m->body_length = m->length - m->head_length - m->tail_length;
			if (i++ > 20) { // chose value with lots of experimentation
#ifdef __UNFORGIVING		// usually converges in ~2 - but not always
				return (TG_FAILED_TO_CONVERGE);
#elif
				break;
#endif
			}
		}
		m->cruise_velocity = Vt;
		m->final_velocity = Vf;
		m->body_length = 0;
		if (m->head_length > MIN_LINE_LENGTH) {
			m->plan_state = MP_PLAN_2_REGION;
			return (TG_OK);	// 2 region return
		}
	}

	// In some cases computed above the new Vt will have become less 
	// than Vi, reducing the 2 region case to a tail-only case. 
	// So you must run it again, below

	// ----- 1 region case (tail-only case) -----
	if (m->length <= m->tail_length) {	// ++++ add in the low Vt case
		i=0;
		while (fabs(m->length - m->tail_length) > ROUNDING_ERROR) {
			Vt_ = Vt;
			Vt *= m->length / m->tail_length;
			Vt = (Vt + Vt_)/2;
			m->tail_length = _mp_get_length(Vt, Vf);
			if (i++ > 20) { 	// usually converges in ~5 - but not always
#ifdef __UNFORGIVING
				return (TG_FAILED_TO_CONVERGE);
#elif
				break;
#endif
			}
		}
		m->initial_velocity = Vt;
		m->cruise_velocity = Vt; 				
		m->tail_length = m->length;
		m->head_length = 0;
		m->body_length = 0;
		m->plan_state = MP_PLAN_1_REGION;
		return (TG_OK);	// 1 region return
	}
	return (TG_ERR);	// should only happen if isNAN or other math error
}


/*
 * _mp_recompute_backwards()
 * _mp_recompute_forwards()
 *
 *	Recompute the previous moves to optimize target velocities.
 *
 *	Recompute_backwards starts at the latest queued move and works back
 *	in the queue until a "pinned" velocity is encountered - one of:
 *
 *	  (a) requested target velocity (Vt) is achieved,
 *	  (b) a way point between moves was set to a velocity by path control
 *		  (exact stop (G61) or exact stop (G61.1) mode in effect). 
 *	  (c) the region being examined is already executing (running) 
 *
 *	Recompute_backwards assumes that the requested Vt of the latest move 
 *	can be achieved, and recomputes previous moves based on that assumption.
 *	The computation stops looking backwards once it gets to a pinned 
 *	velocity - either it (a) achieves Vt, or (b) or (c) condition is found.
 *
 *	If the requested Vt is achieved - either Vt or a way-point velocity - 
 *	then the computation is done. It not, recompute_forward is called
 *	from the pinned velocity, which optimized forwards from the pinned 
 *	velocity.
 */

static uint8_t _mp_recompute_backwards(struct mpMovePlanner *m)
{
	struct mpMovePlanner *p = &mp[1];// pointer to a move in bkwds chain
	struct mpMovePlanner *tmp;

	// setup struct for previous move
	while (_mp_construct_prev_move(p,m) != TG_COMPLETE) {	
		_mp_compute_regions(p->initial_velocity_req, p->target_velocity, m->initial_velocity, p);
		_mp_update_move(p); 
		tmp=m; m=p; p=tmp; 	// shuffle buffers to walk backwards
	}
	return (TG_OK);
}

static uint8_t _mp_recompute_forwards(struct mpMovePlanner *p, struct mpMovePlanner *m)
{
	return (TG_OK);
}

/*
 * construct the M struct for previous move (P) based on current move (M) 
 * Assumes M has valid buffer popinters for head, body, tail (but not prev)
 * Assumes P has no valid buffer pointers 
 */
static uint8_t _mp_construct_prev_move(struct mpMovePlanner *p, struct mpMovePlanner *m)
{
	// setup buffer linkages (the compiler will optimize these get calls down)
	p->next = m->head;						// link to later move as next
	p->tail = mp_get_prev_buffer(m->head);	// set previous tail
	p->body = mp_get_prev_buffer(p->tail);	// etc.
	p->head = mp_get_prev_buffer(p->body);

	// Detect buffer state end conditions - move must be queued & idle 
	// Only test that the body is free. It's OK to recompute a line 
	// where the head is running. This creates a potential race condition,
	// but the alternative of not recomputing the tail is worse.
	if (p->body->buffer_state != MP_BUFFER_QUEUED) {
		return (TG_COMPLETE);
	}
	// populate the move velocities and lengths from underlying buffers
	p->initial_velocity_req = p->head->request_velocity;// requested start v
	p->initial_velocity = p->head->start_velocity;	// actual initial vel
	p->target_velocity = p->body->request_velocity;	// requested cruise vel
	p->cruise_velocity = p->body->start_velocity;	// actual cruise vel
	p->final_velocity = p->tail->end_velocity;		// actual final vel

	p->head_length = p->head->length;
	p->body_length = p->body->length;
	p->tail_length = p->tail->length;
	p->length = p->head_length + p->body_length + p->tail_length;

	// detect end condition - if move is already optimal
/*
	if ((p->initial_velocity == p->initial_velocity_req) && 
		(p->cruise_velocity == p->target_velocity) &&
		(p->final_velocity == m->initial_velocity)) {
		return (TG_COMPLETE);
	}
*/
	if ((p->initial_velocity == p->initial_velocity_req) && 
		(p->cruise_velocity == p->target_velocity)) {
		return (TG_COMPLETE);
	}
	return (TG_OK);
}


/*	
 * _mp_get_length()
 *
 * 	A convenient expression for determinting the length of a line 
 *	given the starting and ending velocities and the jerk is:
 *
 *	  length = abs(end-start) * sqrt(abs(end-start) / max_linear_jerk)
 *
 *	which is derived from these two equations:
 *
 *	  time = 2 * sqrt(abs(end-start) / max_linear_jerk);	// 5.x
 *	  length = abs(end-start) * time / 2;					// [2]
 *
 *	Let the compiler optimize out the Vi=0 constant case
 */

inline static double _mp_get_length(double start, double end)
{
	double delta = fabs(start - end);
	return (delta * sqrt(delta / cfg.max_linear_jerk));
}

/*	
 * _mp_estimate_angular_jerk()
 *
 * The following is borrowed from Simen Svale Skogsrud's Twister project:
 *
 *  Estimate the power of the jerk at the intersection of two motions.
 *	For our application jerk is half the phytagorean magnitude of the 
 *	difference between the unit vector of the two motions which gives 
 *	us a value between 0 and 1.0 where 0 represents no change of 
 *	direction and 1.0 is a full U-turn
 */

static double _mp_estimate_angular_jerk(const struct mpBuffer *p, double previous_velocity)
{
	double j = (sqrt(square(mm.unit_vec[X] - p->unit_vec[X]) +
				 	 square(mm.unit_vec[Y] - p->unit_vec[Y]) +
				 	 square(mm.unit_vec[Z] - p->unit_vec[Z]))/2.0);
	j *= min(1,(previous_velocity / MAX_VELOCITY)); // +++ remove to test
	return (j);
}

/*	
 * _mp_queue_move() - write an M structure to buffers
 */

static uint8_t _mp_queue_move(struct mpMovePlanner *m) 
{
	if ((m->head = _mp_queue_buffer(
								m->initial_velocity, 
								m->cruise_velocity, 
								m->initial_velocity_req,
								m->head_length, MP_TYPE_ACCEL)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	if ((m->body = _mp_queue_buffer(
								m->cruise_velocity, 
								m->cruise_velocity, 
								m->target_velocity, 
								m->body_length, MP_TYPE_CRUISE)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	if ((m->tail = _mp_queue_buffer(
								m->cruise_velocity, 
								m->final_velocity, 0, 
								m->tail_length, MP_TYPE_DECEL)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (TG_OK);
}

static struct mpBuffer *_mp_queue_buffer(double Vi, double Vt, double Vr, double len, uint8_t type)
{
	struct mpBuffer *b;

	if ((b = mp_get_write_buffer()) == NULL) { 	// get buffer or die trying
		return (NULL); 
	}
	b->start_velocity = Vi;
	b->end_velocity = Vt;
	b->request_velocity = Vr;
	b->length = len;
	for (uint8_t i=0; i < AXES; i++) { 			// copy unit vector from mm
		b->unit_vec[i] = mm.unit_vec[i]; 
		mm.position[i] += len * b->unit_vec[i];	// set mm position
		b->target[i] = mm.position[i]; 
	}
	if (len < MIN_LINE_LENGTH) { 
		mp_queue_write_buffer(MP_TYPE_NULL);	// make it a null buffer
	} else {
		mp_queue_write_buffer(type);
	}
	return(b);									// return pointer
}

/*	
 * _mp_update_move() - update buffers according to M struct
 */

static uint8_t _mp_update_move(struct mpMovePlanner *m) 
{
	m->head->start_velocity = m->initial_velocity;
	m->head->end_velocity = m->cruise_velocity;
	m->head->request_velocity = m->initial_velocity_req;
	m->head->length = m->head_length;

	m->body->start_velocity = m->cruise_velocity;
	m->body->end_velocity = m->cruise_velocity;
	m->body->request_velocity = m->target_velocity;
	m->body->length = m->body_length;

	m->tail->start_velocity = m->cruise_velocity;
	m->tail->end_velocity = m->final_velocity;
	m->tail->request_velocity = m->final_velocity;
	m->tail->length = m->tail_length;

	return (TG_OK);
}

/**** ALINE RUN ROUTINES ****
 *	_mp_run_cruise()
 *	_mp_run_accel()
 *	_mp_run_decel()
 *	_mp_aline_run_segment()	- helper code for running a segment
 *	_mp_aline_run_finalize() - helper code for running last segment
 *
 *	Note to self: Returning TG_OK from these routines ends the aline
 *	Returning TG_EAGAIN (or any other non-zero value) continues iteration 
 *
 * 	Solving equation 5.7 for Time for acceleration 1st half if you know: 
 *	length (S), jerk (J), initial velocity (V)
 *
 *	T = (sqrt((8*V^3+9*J*S^2)/J)/J+3*S/J)^(1/3)- 2*V/(J*
 *		(sqrt((8*V^3+9*J*S^2)/J)/J+3*S/J)^(1/3))
 *
 * 	Solving equation 5.11' for Time for acceleration 2nd half if you know:
 *	length (S), jerk (J), position at the half (H), accel at the half (A)
 *
 *	T = (sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+
 *			(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3)+ A^2/(J^2*
 *		(sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+
 *			(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3))+ A/J
 *
 */

static uint8_t _mp_run_cruise(struct mpBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	if (m->length < MIN_LINE_LENGTH) {			// toss the line
		return (TG_OK);
	}
	m->time = m->length / m->end_velocity;		// get time from length
	mr.microseconds = uSec(m->time);

	uint32_t steps[AXES];
	for (uint8_t i=0; i < AXES; i++) {
		mr.target[i] = m->target[i];
		m->target[i] = mr.position[i] + m->unit_vec[i] * m->length; //++++ remove this line for test
		steps[i] = _steps(i, m->target[i]) - _steps(i, mr.position[i]);
	}
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mp_set_mr_position(m->target);
	return (TG_OK);
}

static uint8_t _mp_run_accel(struct mpBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for acceleration
	if (m->move_state == MP_STATE_NEW) {
		if (m->length < MIN_LINE_LENGTH) { return (TG_OK); }	// toss
		mr.midpoint_velocity = (m->start_velocity + m->end_velocity) / 2;
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
		m->move_state = MP_STATE_RUNNING_1;
	}
	// first half of acceleration - concave portion of curve
	if (m->move_state == MP_STATE_RUNNING_1) {
		mr.segment_velocity = m->start_velocity + 
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno (_mp_aline_run_segment(m));	 // return is OK, not an error
		// setup for second half
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time /2;
		m->move_state = MP_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a buffer
	}
	// second half of acceleration - convex portion of curve
	if (m->move_state == MP_STATE_RUNNING_2) {
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity + 
						(mr.elapsed_time * mr.midpoint_acceleration) -
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mp_aline_run_segment(m));
		} else {
			return(_mp_aline_run_finalize(m));	// for accuracy
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mp_run_decel(struct mpBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	// initialize for deceleration
	if (m->move_state == MP_STATE_NEW) {
		if (m->length < MIN_LINE_LENGTH) { return (TG_OK); } // toss
		mr.midpoint_velocity = (m->start_velocity + m->end_velocity) / 2;
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
		m->move_state = MP_STATE_RUNNING_1;
	}
	// first half of deceleration
	if (m->move_state == MP_STATE_RUNNING_1) {	// concave part of curve
		mr.segment_velocity = m->start_velocity - 
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno(_mp_aline_run_segment(m));
		// setup for second half
		mr.segment_count = mr.segments;
		mr.elapsed_time = mr.segment_time / 2;
		m->move_state = MP_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a buffer
	}
	// second half of deceleration
	if (m->move_state == MP_STATE_RUNNING_2) {	// convex part of curve
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity - 
						(mr.elapsed_time * mr.midpoint_acceleration) +
						(mr.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mp_aline_run_segment(m));
		} else {
			return(_mp_aline_run_finalize(m));	// for accuracy
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mp_aline_run_segment(struct mpBuffer *m)
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
	_mp_set_mr_position(m->target);
	if (--mr.segment_count) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

static uint8_t _mp_aline_run_finalize(struct mpBuffer *m)
{
	// finalize - do the last segment to maintain position accuracy
	mr.length = sqrt(square(mr.target[X] - mr.position[X]) +
					 square(mr.target[Y] - mr.position[Y]) +
					 square(mr.target[Z] - mr.position[Z]) +
					 square(mr.target[A] - mr.position[A]));

	if (mr.length < MIN_LINE_LENGTH) {		// trap zero-length case
		return (TG_OK);
	}
	mr.time = mr.length / m->end_velocity; // get time from length
	mr.microseconds = uSec(mr.time);

	uint32_t steps[AXES];
	for (uint8_t i=0; i < AXES; i++) {
		steps[i] = _steps(i, mr.target[i]) - _steps(i, mr.position[i]);
	}
	mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mp_set_mr_position(mr.target);
	return (TG_OK);
}


//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _mp_test_buffers(void);
void _mp_test_recompute_vt(void);
void _mp_call_recompute_vt(double l, double Vp, double Vi, double Vt);


void mp_unit_tests()
{
//	_mp_test_buffers();
	_mp_test_recompute_vt();
}

void _mp_call_recompute_vt(double l, double Vp, double Vi, double Vt) 
{
	mm.length = l;
	mm.previous_velocity = Vp;
	mm.initial_velocity = Vi;
	mm.target_velocity = Vt;
	mm.head_length = _mp_get_length(mm.target_velocity, mm.initial_velocity);
	mm.tail_length = _mp_get_length(mm.target_velocity, 0);
	_mp_recompute_velocity();
}

void _mp_test_recompute_vt()
{
//						  Len	Vp	 Vi	  Vt
	_mp_call_recompute_vt( 3.0, 250, 100, 400);	// 3 regions - fits
	_mp_call_recompute_vt( 2.0, 250, 100, 400);	// 2 regions - simple reduction
	_mp_call_recompute_vt( 1.0, 250, 100, 400);	// 1 region - more extreme reduction
	_mp_call_recompute_vt( 0.5, 250, 100, 400);	// 1 region - Vi reduces below Vp
												// 1 region - zero legnth line
	_mp_call_recompute_vt( MIN_LINE_LENGTH/2, 250, 100, 400);	
}

void _mp_test_buffers()
{
	mp_test_write_buffer(MP_BUFFERS_NEEDED); // test for enough free buffers

	mp_get_write_buffer();		// open a write buffer [0]
	mp_get_write_buffer();		// open a write buffer [1]
	mp_get_write_buffer();		// open a write buffer [2]

	mp_get_run_buffer();		// attempt to get run buf - should fail (NULL)

	mp_queue_write_buffer(MP_TYPE_ACCEL);	// queue the write buffer [0]
	mp_queue_write_buffer(MP_TYPE_CRUISE);	// queue the write buffer [1]
	mp_queue_write_buffer(MP_TYPE_DECEL);	// queue the write buffer [2]

	mp_get_run_buffer();		// attempt to get run buf - should succeed

/*
	mp_get_write_buffer();		// open the next write buffer [1]
	mp_queue_write_buffer(MP_TYPE_ACCEL);	// commit it
	mp_get_write_buffer();		// open the next write buffer [2]
	mp_queue_write_buffer(MP_TYPE_ACCEL);	// commit it
	mp_get_write_buffer();		// attempt write buffer - should fail (NULL)
	mp_end_run_buffer();
	mp_get_write_buffer();		// now if should succeed
*/
}

#endif
