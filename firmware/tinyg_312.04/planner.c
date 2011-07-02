/*
 * planner.c - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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

#include <stdlib.h>
#include <math.h>

#include "xio.h"				// supports trap and debug statements
#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "settings.h"
#include "planner.h"
#include "motor_queue.h"
#include "canonical_machine.h"
#include "stepper.h"

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// b->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_PENDING,			// marked as the next buffer to run
	MP_BUFFER_RUNNING			// current running buffer
};

enum mpMoveType {				// b->move_type values 
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

enum mpMoveState {				// b->move_state values
	MP_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MP_STATE_RUNNING_1,			// first half of move or sub-move
	MP_STATE_RUNNING_2,			// second half of move or sub-move
	MP_STATE_FINALIZE,			// finalize the move or sub-move
	MP_STATE_END				// force the move to end (kill)
};
#define MP_STATE_RUNNING MP_STATE_RUNNING_1	// a convenience for above


struct mpBufferArc {			// arc variables for move/sub-move buffers
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mpBuffer {				// move/sub-move motion control structure
	struct mpBuffer *nx;		// static pointer to next buffer
	struct mpBuffer *pv;		// static pointer to previous buffer

	double target[AXES];		// target position in floating point
	double unit_vec[AXES];		// axis scaling & jerk computation
	struct mpBufferArc a;		// arc variables

	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// state machine sequence
	uint8_t replannable;		// TRUE if region can be replanned

	double time;				// line, helix or dwell time in minutes
	double length;				// line or helix length in mm
	double start_velocity;		// actual starting velocity of a region
	double end_velocity;		// actual ending velocity of a region
	double request_velocity;	// requested initial, target, or end velocity
								// for head, body, or tail, respectively
};

struct mpBufferPool {			// ring buffer for sub-moves
	struct mpBuffer *w;			// get_write_buffer pointer
	struct mpBuffer *q;			// queue_write_buffer pointer
	struct mpBuffer *r;			// get/end_run_buffer pointer
	struct mpBuffer b[MP_BUFFER_SIZE];// buffer storage
};

struct mpMoveMasterSingleton {	// common variables for planning (move master)
	double position[AXES];		// final move position 
	double target[AXES];		// target move position
	double unit_vec[AXES];		// for axis scaling and jerk computation
	double ang_jerk_vec[AXES];	// for angular jerk time accumulation
	double linear_jerk_div2;	// max linear jerk divided by 2
	double linear_jerk_rad3;	// cube root of max linear jerk
};

struct mpMovePlanner {			// used to compute or recompute regions
//	uint8_t regions;			// # of regions set by recompute_regions
	uint8_t path_mode;			// path control mode 

								// buffer pointers
	struct mpBuffer *head;		// pointer to head of current move
	struct mpBuffer *body;		// pointer to body of current move
	struct mpBuffer *tail;		// pointer to tail of current move

	double length;				// length of line or helix in mm
	double head_length;			// computed for trajectory planning
	double body_length;			// redundant, but useful
	double tail_length;			// computed for trajectory planning

	double initial_velocity_req;// requested initial velocity
	double initial_velocity;	// actual initial velocity
	double target_velocity; 	// requested target velocity 
	double cruise_velocity;		// actual achieved velocity
	double final_velocity;		// actual exit velocity
	double angular_jerk;		// estimate of angular jerk
};

struct mpMoveRuntimeSingleton {	// persistent runtime variables
	uint8_t run_flag;			// move status
	uint8_t (*run_move)(struct mpBuffer *m); // currently running move

	double position[AXES];		// final move position
	double target[AXES];		// target move position

	double length;				// length of line or helix in mm
	double time;				// total running time (derived)
	double microseconds;		// line or segment time in microseconds
	double elapsed_time;		// current running time (increments)
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double midpoint_acceleration;//acceleration at the midpoint

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_time;		// constant time per aline segment
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double segment_theta;		// angular motion per segment
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
};

static struct mpMoveMasterSingleton mm;
static struct mpMovePlanner mp[2];// current move and fwd or bkwd neighbor
static struct mpMoveRuntimeSingleton mr;
static struct mpBufferPool mb;

// p.s. I tried listing variables both ways: target_velocity or Vt,
//		initial_velocity or Vi, etc. and found the first way easier to 
//		read in spite of the wrapped lines.

/*
 * Local Scope Data and Functions
 */

//static void _mp_kill_dispatcher(void);	// unused
static void _mp_init_buffers(void);
static void _mp_unget_write_buffer(void);
static void _mp_clear_buffer(struct mpBuffer *b); 
static void _mp_queue_write_buffer(const uint8_t move_type);
static void _mp_finalize_run_buffer(void);
static struct mpBuffer * _mp_get_write_buffer(void); 
static struct mpBuffer * _mp_get_run_buffer(void);
static struct mpBuffer * _mp_get_prev_buffer_implicit(void);
static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *b);
//static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *b); // unused

static uint8_t _mp_run_null(struct mpBuffer *b);
static uint8_t _mp_run_accel(struct mpBuffer *b);
static uint8_t _mp_run_cruise(struct mpBuffer *b);
static uint8_t _mp_run_decel(struct mpBuffer *b);
static uint8_t _mp_run_line(struct mpBuffer *b);
static uint8_t _mp_run_arc(struct mpBuffer *b);
static uint8_t _mp_run_dwell(struct mpBuffer *b);
static uint8_t _mp_run_stops(struct mpBuffer *b);
static uint8_t _mp_aline_run_segment(struct mpBuffer *b);
static void _mp_aline_run_finalize(struct mpBuffer *b);

static void _mp_set_mm_position(const double target[]) ;
static void _mp_set_mr_position(const double target[]);
static void _mp_clear_planner(struct mpMovePlanner *m);

static uint8_t _mp_compute_regions(const double Vir, const double Vt, const double Vf, struct mpMovePlanner *m);
static void _mp_backplan(struct mpMovePlanner *m);
static void _mp_set_braking_velocity(struct mpMovePlanner *p, struct mpMovePlanner *m);
static uint8_t _mp_make_previous_move(struct mpMovePlanner *p, const struct mpMovePlanner *m);
static uint8_t _mp_queue_move(struct mpMovePlanner *m);
static void _mp_update_move(const struct mpMovePlanner *p, const struct mpMovePlanner *m);
static uint8_t _mp_get_move_type(const struct mpBuffer *b);
static struct mpBuffer *_mp_queue_buffer(const double Vs, const double Ve, 
									     const double Vr, const double len);

static double _mp_get_length(const double Vi, const double Vt);
static double _mp_get_velocity(const double Vi, const double len);
static double _mp_estimate_angular_jerk(const struct mpBuffer *p);

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
	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	_mp_init_buffers();
}

/* 
 * mp_move_dispatcher() - routine for dequeuing and executing moves
 *
 *	Dequeues the buffer queue and executes the move run continuations.
 *	Manages run buffers and other details.
 *	Responsible for freeing the completed run buffers
 *	Runs as a continuation itself; called from tg_controller()
 */

uint8_t mp_move_dispatcher() 
{
	uint8_t status;
	struct mpBuffer *b;

	if ((b = _mp_get_run_buffer()) == NULL) {// NULL means nothing's running
		return (TG_NOOP);
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
			case MP_TYPE_START:	{ mr.run_move = _mp_run_stops; break; }
			case MP_TYPE_STOP:	{ mr.run_move = _mp_run_stops; break; }
			case MP_TYPE_END: 	{ mr.run_move = _mp_run_stops; break; }
		}
	}
	if ((status = mr.run_move(b)) == TG_EAGAIN) { // run current run buf
		return (TG_EAGAIN);
	}
	mr.run_flag = FALSE;				// finalize and return
	_mp_finalize_run_buffer();
	return (status);
}

/* 
 * _mp_kill_dispatcher() - kill current move and reset dispatcher
 */
/* UNUSED
void _mp_kill_dispatcher() 
{
	struct mpBuffer *b;

	if ((b = _mp_get_run_buffer()) != NULL) {// NULL means nothing's running
		b->move_state = MP_STATE_END;
		mr.run_flag = FALSE;
		_mp_finalize_run_buffer();
	}
}
*/

/**** MOVE QUEUE ROUTINES ************************************************
 * mp_check_for_write_buffers(N) Return TRUE if N write buffers are avail
 *
 * _mp_init_buffers()	   Initializes or restes buffers
 *
 * _mp_get_write_buffer()  Get pointer to next available write buffer
 *						   Returns pointer or NULL if no buffer available
 *						   Multiple write buffers may be open at once
 *
 * _mp_unget_write_buffer() Free write buffer if you decide not to queue it
 *						   Only works on most recently gotten write buffer
 *						   You could work your way back in a set or buffers
 *						   Use this one carefully.
 *
 * _mp_queue_write_buffer() Commit the next write buffer to the queue
 *						   Write buffers will queue in order gotten,
 *						   and will run in the order queued.
 *						   Advances write pointer & changes buffer state
 *
 * _mp_get_run_buffer()	   Get pointer to the next or current run buffer
 *						   Returns a new run buffer if prev buf was ENDed
 *						   Returns same buf if called again before ENDing
 *						   Returns NULL if no buffer available
 *						   The behavior supports continuations (iteration)
 *
 * _mp_finalize_run_buffer() Release the run buffer & return to buffer pool
 *						   End_run causes get_run to return the next buffer
 *
 * _mp_get_prev_buffer_implicit() Return pointer to the buffer immediately
 *						   before the next available write buffer. From
 *						   there earlier buffers can be read using the 
 *						   backwards pointers. This buffer cannot be 
 *						   queued and should not be ENDed.
 *
 * _mp_get_prev_buffer_implicit() Return previous buffer w/o knowing b
 * _mp_get_prev_buffer(b)  Return pointer to prev buffer in linked list
 * _mp_get_next_buffer(b)  Return pointer to next buffer in linked list 
 * _mp_clear_buffer(b)	   Zero the contents of the buffer
 *
 * A typical usage sequence is:
 *	1 - test if you can get 3 write buffers - for an aline()
 *	2 - aline first gets prev_buffer_implicit to look back at previous Vt
 *	3 - aline then gets write buffers as they are needed
 *  3a- sometimes aline ungets a write buffer an exception case is detected
 *	4 - aline queues the write buffers - one queue_write call per buffer
 *	5 - run_aline gets a new run buffer and starts to execute the sub-move
 *	6 - run_aline gets the same buffer as it iterates through the sub-move
 *	7 - run_aline finalizes the run buffer when the sub-move is complete
 *	8 - run_aline gets a run buffer - which now returns a new one
 *
 * Further notes:
 *	The write buffer pointer only moves forward on queue_write, and 
 *	the read buffer pointer only moves forward on finalize_read calls.
 *	(check, get and unget have no effect)
 *	Do not queue a failed get_write, and do not finalize a failed run buffer
 *	The program must be sure to queue write buffers and to finalize run 
 *	buffers or this app-level memory management all fails. Usually this is 
 *	done at the end of the routine that gets the buffer.
 */

static void _mp_init_buffers()
{
	struct mpBuffer *pv;
	uint8_t i;

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

uint8_t mp_check_for_write_buffers(const uint8_t count) 
{
	uint8_t i;
	struct mpBuffer *w = mb.w;	// temp write buffer pointer

	for (i=0; i < count; i++) {
		if (w->buffer_state != MP_BUFFER_EMPTY) {
			return (FALSE);
		}
		w = w->nx;
	}
	return (TRUE);
}

static struct mpBuffer * _mp_get_write_buffer() 
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

static void _mp_unget_write_buffer()
{
	mb.w = mb.w->pv;						// queued --> write
	mb.w->buffer_state = MP_BUFFER_EMPTY; 	// not loading anymore
}

static void _mp_queue_write_buffer(const uint8_t move_type)
{
	mb.q->move_type = move_type;
	mb.q->move_state = MP_STATE_NEW;
	mb.q->buffer_state = MP_BUFFER_QUEUED;
	mb.q = mb.q->nx;		// advance the queued buffer pointer
}

static struct mpBuffer * _mp_get_run_buffer() 
{
	// condition: fresh buffer; becomes running if queued or pending
	if ((mb.r->buffer_state == MP_BUFFER_QUEUED) || 
		(mb.r->buffer_state == MP_BUFFER_PENDING)) {
		mb.r->buffer_state = MP_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mb.r->buffer_state == MP_BUFFER_RUNNING) { // return same buffer
		return (mb.r);
	}
	return (NULL);		// condition: no queued buffers. fail it.
}

static void _mp_finalize_run_buffer()	// EMPTY current run buf & advance to next
{
	_mp_clear_buffer(mb.r);	// clear it out (& reset replannable)
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;		// advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) { // only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING;   // pend next buffer
	}
}

static struct mpBuffer * _mp_get_prev_buffer_implicit()
{
	return (mb.w->pv);
}

static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *b)
{
	return (b->pv);
}
/* UNUSED
static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *b)
{
	return (b->nx);
}
*/
static void _mp_clear_buffer(struct mpBuffer *b) 
{
	struct mpBuffer *nx = b->nx;	// save pointers
	struct mpBuffer *pv = b->pv;
	memset(b, 0, sizeof(struct mpBuffer));
	b->nx = nx;						// restore pointers
	b->pv = pv;
}

/* 
 * mp_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mp_isbusy()
{
	if ((st_isbusy() == TRUE) || (mr.run_flag == TRUE)) {
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
 *	 - b->target	- target position of submove (runtime working target)
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
static void _mp_set_mm_position(const double target[]) 
{ 
	mm.position[X] = target[X];
	mm.position[Y] = target[Y];
	mm.position[Z] = target[Z];
	mm.position[A] = target[A];
}

// set move/sub-move runtime position
static void _mp_set_mr_position(const double target[]) 
{ 
	mr.position[X] = target[X];
	mr.position[Y] = target[Y];
	mr.position[Z] = target[Z];
	mr.position[A] = target[A];
}

// initialize a planner struct (isolates the dangerous memset function)
//static struct mpMovePlanner *_mp_clear_planner(struct mpMovePlanner *m) 
static void _mp_clear_planner(struct mpMovePlanner *m) 
{
	memset(m, 0, sizeof(struct mpMovePlanner));
//	return (m);
}

/*************************************************************************
 * _mp_run_null() - null move
 *
 * Removes a null buffer from the queue
 */

static uint8_t _mp_run_null(struct mpBuffer *b)
{
	b->replannable = FALSE;	// stop replanning
	return (TG_OK);		// dispatcher will free the buffer after return
}


/**** STOP START AND END ROUTINES ****************************************
 * mp_async_stop() 	- stop current motion immediately
 * mp_async_start() - (re)start motion
 * mp_async_end() 	- stop current motion immediately
 *
 *	These routines must be safe to call from ISRs. Mind the volatiles.
 */

void mp_async_stop()
{
	st_stop();						// stop the steppers
}

void mp_async_start()
{
	st_start();						// start the stoppers
}

void mp_async_end()
{
	tg_application_init();			// re-init EVERYTHING
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

void mp_queued_stop() 
{
	if (_mp_get_write_buffer() == NULL) {
		TRAP(PSTR("Failed to get buffer in mp_queued_stop()"));
		return;
	}
	_mp_queue_write_buffer(MP_TYPE_STOP);
}

void mp_queued_start() 
{
	if (_mp_get_write_buffer() == NULL) {
		TRAP(PSTR("Failed to get buffer in mp_queued_start()"));
		return;
	}
	_mp_queue_write_buffer(MP_TYPE_START);
}

void mp_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	if (_mp_get_write_buffer() == NULL) {
		TRAP(PSTR("Failed to get buffer in mp_queued_end()"));
		return;
	}
	_mp_queue_write_buffer(MP_TYPE_END);
}

static uint8_t _mp_run_stops(struct mpBuffer *b) 
{
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	(void)mq_queue_stops(b->move_type);
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

	if ((b = _mp_get_write_buffer()) == NULL) { // get write buffer or fail
		TRAP(PSTR("Failed to get buffer in mp_dwell()"));
		return (TG_BUFFER_FULL_FATAL);		   // (not supposed to fail)
	}
	b->time = seconds;						   // in seconds, not minutes
	_mp_queue_write_buffer(MP_TYPE_DWELL);
	return (TG_OK);
}

static uint8_t _mp_run_dwell(struct mpBuffer *b)
{
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	(void)mq_queue_dwell((uint32_t)(b->time * 1000000));	// convert seconds to uSec
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

	if ((b = _mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		TRAP(PSTR("Failed to get buffer in mp_line()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// capture the function args and compute line length
	b->target[X] = x;		// targets for the move
	b->target[Y] = y;
	b->target[Z] = z;
	b->target[A] = a;

	if ((b->time = minutes) < EPSILON) {
		_mp_unget_write_buffer();		// early exit requires free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	b->length = sqrt(square(b->target[X] - mr.position[X]) +
					 square(b->target[Y] - mr.position[Y]) +
					 square(b->target[Z] - mr.position[Z]) +
					 square(b->target[A] - mr.position[A]));

	if (b->length < MIN_LINE_LENGTH) {	// trap zero-length lines
		_mp_unget_write_buffer();		// early exit requires free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}
	b->request_velocity = b->length / b->time;	// for yuks
	_mp_queue_write_buffer(MP_TYPE_LINE);
	_mp_set_mm_position(b->target);		// set mm position for planning
	return(TG_OK);
}

static uint8_t _mp_run_line(struct mpBuffer *b) 
{
	uint8_t i;
	double steps[AXES];
	
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}

	for (i=0; i < AXES; i++) {
		steps[i] = _steps(i, b->target[i]) - _steps(i, mr.position[i]);
	}
	mr.microseconds = uSec(b->time);
	(void)mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
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
	double length;

	if ((b = _mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		TRAP(PSTR("Failed to get buffer in mp_arc()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	b->length = hypot(angular_travel * radius, fabs(linear_travel));	
	if (b->length < cfg.min_segment_len) { // too short to draw
		_mp_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the move struct for an arc
	// note: b->target is for debugging convenience and not actually used
	b->target[X] = x;
	b->target[Y] = y;
	b->target[Z] = z;
	b->target[A] = a;
	b->time = minutes;
	b->a.theta = theta;
	b->a.radius = radius;
	b->a.axis_1 = axis_1;
	b->a.axis_2 = axis_2;
	b->a.axis_linear = axis_linear;
	b->a.angular_travel = angular_travel;
	b->a.linear_travel = linear_travel;
	b->start_velocity = b->length / b->time;	// for trajectory planning
	b->end_velocity = b->start_velocity;	 	// for consistency

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
	_mp_queue_write_buffer(MP_TYPE_ARC);
	return (TG_OK);
}

static uint8_t _mp_run_arc(struct mpBuffer *b) 
{
	uint8_t i;
	double steps[AXES];

	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	// initialize arc variables
	if (b->move_state == MP_STATE_NEW) {
		mr.segments = ceil(b->length / cfg.min_segment_len);
		mr.segment_count = (uint32_t)mr.segments;
		mr.segment_theta = b->a.angular_travel / mr.segments;
		mr.segment_length = b->a.linear_travel / mr.segments;
 		mr.microseconds = uSec(b->time / mr.segments);
		mr.center_1 = mr.position[b->a.axis_1] - sin(b->a.theta) * b->a.radius;
		mr.center_2 = mr.position[b->a.axis_2] - cos(b->a.theta) * b->a.radius;
		mr.target[b->a.axis_linear] = mr.position[b->a.axis_linear];
		b->move_state = MP_STATE_RUNNING;
	}
	// compute an arc segment and exit
	if (b->move_state == MP_STATE_RUNNING) {
		b->a.theta += mr.segment_theta;
		mr.target[b->a.axis_1] = mr.center_1 + sin(b->a.theta) * b->a.radius;
		mr.target[b->a.axis_2] = mr.center_2 + cos(b->a.theta) * b->a.radius;
		mr.target[b->a.axis_linear] += mr.segment_length;

		for (i=0; i < AXES; i++) {
			steps[i] = _steps(i, mr.target[i]) - _steps(i, mr.position[i]);
		}
		(void)mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
		_mp_set_mr_position(mr.target);
		if (--mr.segment_count > 0) {
			return (TG_EAGAIN);
		}
	}
	return (TG_OK);
}

/*************************************************************************
 * mp_aline() 		- queue line move with acceleration / deceleration
 * _mp_run_aline()	- run accel/decel move 
 *
 *	This module uses the third order jerk motion equations to generate 
 *	acceleration and deceleration curves that obey maximum jerk parameters.
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
 *	A move (line or arc) is divided into 3 regions (sub-moves):
 *	  - head	acceleration to target velocity (acceleration region)
 *	  - body	bulk of move at target speed 	(cruise region)
 *	  - tail	deceleration to exit velocity 	(deceleration region)
 *
 *	The initial velocity of the head (Vi) head is dependent on the path
 *	control mode in effect. Vi is zero for EXACT STOP mode. For EXACT PATH
 *	and CONTINUOUS modes it's dependent on the velocity of the previous 
 *	move. The body is the "cruise region" where the line is running at its
 *	target velocity (Vt). The tail is pre-computed to decelerate to zero. 
 *	(See "Special Cases" for exceptions to the above.)
 *
 *	As mentioned above, sufficient length is reserved in the tail to 
 *	allow a worst-case deceleration from Vt to zero - which will occur
 *	if there is no following move or the following move has a Vi = 0 
 *	(such as in EXACT_STOP mode). If the following move has a non-zero Vi
 *	The previous moves are recomputed to attain the maximum velocity while
 *	still supporting a deceleration to zero.
 */
/*	The computation of the regions is set by path control mode in effect:
 *
 *	  - CONTINUOUS_MODE (G64): The moves will attempt to run at their 
 *		maximum requested speed, accelerating or decelerating at way points
 *		(line junctions) to match speeds and maintain maximum velocity. 
 *
 *	  - EXACT_PATH_MODE (G61.1): The move is joined to the next move at
 *		reduced velocity in an attempt to keep the path as accurate as 
 *		possible. The join computes an estimated jerk based on the change 
 *		in velocity and direction (vector) between the two lines, then 
 *		decelerates the leading line to a computed "safe" velocity before 
 *		accelerating into the following line. For 180 degree turns the line
 *		will stop completely before reversing.
 *
 *	  -	EXACT_STOP_MODE (G61): The move runs to zero velocity before the 
 *		next move is started.
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
 *	as a set of move buffers (buffers for head, body and tail, but 
 *	sometimes these are NULL buffers).
 * 
 *	The tail is always pre-computed as an exact stop tail - i.e. to 
 *	decelerate to zero velocity in the event that no new line arrives. 
 *	If a following line arrives before the tail is executed the moves 
 *	prior to the new move are recomputed (backplanned) to blend with the 
 *	new line. In this way optimal velocities can be achieved while still 
 *	allowing for braking at the end of a chain of moves.
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
 *		- CRUISING:		No reduction in velocity between lines
 *
 *		- DECELERATING:	The previous line decelerates to the initial 
 *						velocity of the new line. 
 *
 *		- ACCELERATING:	The previous line cruises to the way point of the 
 *						new line, which accelerates to its cruise velocity
 *
 *	  - EXACT_PATH_MODE (G61.1) is similar to continuous mode except that
 *		the previous line will decelerate if needed ("dip") to a safe 
 *		speed at the way point The new line accelerates from the join speed
 *		The join speed is computed based on the estimated angular jerk 
 *		between the two lines. If the jerk is too extreme (join angle is 
 *		too sharp & fast) the line will be further downgraded to exact stop 
 *		mode (for that line only).
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
 *	(1)	An aline() requires between 3 write buffers to compute. 
 *		Before calling aline() you MUST test that MAX_BUFFERS_NEEDED (3)
 *		buffers are available or aline() could fail fatally.
 *
 *	(2)	All math is done in absolute coordinates using double precision 
 *		floating point and in double float minutes until the very end, 
 *		when it's converted to steps and microseconds for queuing the 
 *		motor moves.
 *
 *	(3)	You may notice that initialized line buffers use Vi, Vt and Length
 *		but do not require Time. Time is derived from Vi, Vt & L.
 */

uint8_t mp_aline(double x, double y, double z, double a, double minutes)
{
	uint8_t i;
	struct mpBuffer *t; 		// previous tail buffer pointer
	struct mpMovePlanner *m = (&mp[0]);

#ifdef __dbALINE_CALLED
		fprintf_P(stderr, PSTR("Aline called %1.4f, %1.4f, %1.4f, %1.4f    %1.4f\n"),
				  x,y,z,a, minutes);
#endif

	// setup initial move values
	_mp_clear_planner(m);		// set all V's = 0
	mm.target[X] = x;
	mm.target[Y] = y;
	mm.target[Z] = z;
	mm.target[A] = a;	
	m->length = sqrt(square(mm.target[X] - mm.position[X]) +
					 square(mm.target[Y] - mm.position[Y]) +
					 square(mm.target[Z] - mm.position[Z]) +
					 square(mm.target[A] - mm.position[A]));

	if (m->length < MIN_LINE_LENGTH) {			// trap zero-length lines
		return (TG_ZERO_LENGTH_MOVE);
	}
	m->target_velocity = m->length / minutes;	// Vt requested

	for (i=0; i < AXES; i++) {					// compute unit vector
		mm.unit_vec[i] = (mm.target[i] - mm.position[i]) / m->length;
	}

	// some other random inits
	mm.linear_jerk_div2 = cfg.max_linear_jerk/2;// init linear jerk terms
	mm.linear_jerk_rad3 = pow(cfg.max_linear_jerk, 0.33333333333334);
	t = _mp_get_prev_buffer_implicit();			// get previous tail

	// handle case where previous move is a queued or running arc
	if ((t->move_type == MP_TYPE_ARC) && (t->buffer_state != MP_BUFFER_EMPTY)) {
		m->initial_velocity_req = t->end_velocity;
		(void)_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m);
		ritorno(_mp_queue_move(m));
		return (TG_OK);	// don't bother to backplan an arc. Just return.
	} 

	// handle straight line (non-arc) cases
	m->path_mode = cm_get_path_control_mode();	// requested path mode
	if (t->buffer_state != MP_BUFFER_QUEUED) {
		m->path_mode = PATH_EXACT_STOP;			// downgrade path & Vir=0
	} else { 
		m->initial_velocity_req = t->request_velocity;// use prev Vt
		m->angular_jerk = _mp_estimate_angular_jerk(t);
	}
	// setup initial velocity and do path downgrades
	if (m->path_mode == PATH_CONTINUOUS) {
		if (m->angular_jerk > cfg.angular_jerk_lower) {
			m->path_mode = PATH_EXACT_PATH; 	// downgrade path mode
		} else { // this next line ensures decels happen in previous line
			m->initial_velocity_req = min(m->initial_velocity_req, m->target_velocity);
		}
	}
	if (m->path_mode == PATH_EXACT_PATH) {
		if (m->angular_jerk > cfg.angular_jerk_upper) {
			m->path_mode = PATH_EXACT_STOP;		// downgrade path mode
			m->initial_velocity_req = 0;
		} else { // adjust way-point velocity to reduce angular jerk
			m->initial_velocity_req *= (1 - (m->angular_jerk - cfg.angular_jerk_lower));
		}
	}
	// do the actual work
//	if (!_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m)) {;
	if (_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m) == 0) {;
		return (TG_OK);	// returned 0 regions, exit 'cause line's too-short
	}
	ritorno(_mp_queue_move(m));
	_mp_backplan(m);
	return (TG_OK);
}

/**** ALINE HELPERS ****
 * _mp_backplan()  			  - recompute moves backwards from latest move
 * _mp_lookback()	 		  - scan backwards for velocities and length
 * _mp_make_previous_move()	  - reconstruct a planning struct from move buffers
 * _mp_compute_regions() 	  - compute region lengths and velocity contours
 * _mp_queue_move()			  - queue 3 regions of a move
 * _mp_queue_buffer() 		  - helper helper for making line buffers
 * _mp_get_move_type()		  - writes the move type based on velocities
 * _mp_update_move()		  - update a move after a replan
 * _mp_get_length()			  - get length given Vi and Vt
 * _mp_estimate_angular_jerk()- factor of 0 to 1 where 1 = max jerk
 */

/*
 * _mp_backplan()
 *
 *	Recompute the velocities of the previous moves to fit the acceleration
 *	and distance constraints &  optimize target velocities. Backplanning 
 *	starts at the current move and works back through the moves in the 
 *	queue until a "non-replannable" move is found. Moves become 
 *	non-replannable when:
 *
 *	  (a) A move becomes optimized, i.e. hits all it's target velocities:
 *			Vi=Vir, Vc=Vt, and Vf=Vir_of_the_next_move 
 *	  (b) A way point between moves was fixed to a velocity by path control
 *		  	(i.e. exact path (G61.1) or exact stop (G61) modes). 
 *	  (c) The move is already executing. It's OK if the head is running,
 *			but not if the body or tail is running.
 *
 *	The first backwards pass fixes the starting velocity to allow braking.
 *	The second pass uses these limits to recompute the velocities and 
 *	region lengths for each of the constituent moves. If a move becomes 
 *	optimized it's set non-replannable, reducing the length of the chain.
 */

static void _mp_backplan(struct mpMovePlanner *m)
{
	uint8_t i=0;
	struct mpMovePlanner *tmp;
	struct mpMovePlanner *p = &mp[1];	// pointer to a move in bkwds chain

	// set previous move non-replannable if current move is exact stop 
	if (m->path_mode == PATH_EXACT_STOP) {
		(void)_mp_make_previous_move(p,m);
		p->head->replannable = FALSE;
		p->body->replannable = FALSE;
		p->tail->replannable = FALSE;
		return;
	}

	// do backplanning passes
	_mp_set_braking_velocity(p,m);	// set first Vir to achieve full braking
	while (_mp_make_previous_move(p,m) != TG_COMPLETE) {
		(void)_mp_compute_regions(p->initial_velocity_req, p->target_velocity, m->initial_velocity, p);
		_mp_update_move(p,m); 
		tmp=m; m=p; p=tmp; 	// shuffle buffers to walk backwards
		if (++i > 20) {
			TRAP1(PSTR("Backplanning error: %f"), m->length);
			break;
		}
	}
	return;
}

/*
 * _mp_set_braking_velocity()
 *
 *	This function looks back in the move chain until it hits a move that 
 *	can't be replanned ("non-replannable"). It accululates the total 
 *	length of the chain then calculates the maximum starting velocity
 *	that can still brake to zero velocity by the end of the chain. If the 
 *	max braking velocity is less than the requested initial velocity of 
 *	the chain (Vir), then Vir is set to the computed max braking velocity.
 */
static void _mp_set_braking_velocity(struct mpMovePlanner *p, struct mpMovePlanner *m)
{
	uint8_t i=0;

	// set P as current move (M) and initialize needed values
	_mp_clear_planner(p);
	p->head = m->head;
	p->body = m->body;
	p->tail = m->tail;
	p->length = m->length;
	p->initial_velocity_req = m->initial_velocity_req;

	do { // move back to prev move; accumulate length
		p->tail = p->head->pv;
		p->body = p->tail->pv;
		p->head = p->body->pv;
		p->length += p->head->length + p->body->length + p->tail->length;
		if (++i > 20) {			// #### Batman, it's trap! ####
			TRAP1(PSTR("Lookback error: %f"), m->length);
			break;
		}
	} while (p->head->pv->replannable == TRUE);

	//	compute and conditionally apply max braking velocity
	p->initial_velocity_req = min(_mp_get_velocity(0, m->length), p->initial_velocity_req);
	return;
}

/*
 * _mp_make_previous_move() - reconstruct M struct from buffers
 *
 * Construct the M struct for previous move (P) based on current move (M) 
 * Assumes M has a valid buffer pointer for the head
 * Returns TG_COMPLETE if prev move is empty, done, or running
 *	(note - it's OK if the head is running, just not the body or tail)
 */
static uint8_t _mp_make_previous_move(struct mpMovePlanner *p, const struct mpMovePlanner *m)
{
	_mp_clear_planner(p);					// zero it out

	// setup buffer linkages (the compiler will optimize these get calls down)
	p->tail = _mp_get_prev_buffer(m->head);	// set previous tail
	p->body = _mp_get_prev_buffer(p->tail);	// etc.
	p->head = _mp_get_prev_buffer(p->body);

	// return if the move is not replannable
	if ((p->tail->replannable == FALSE) || (p->body->replannable == FALSE)) {
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
	return (TG_OK);
}

/*
 * _mp_compute_regions()
 *
 *	This function computes the region lengths and the velocities:
 *
 *  Inputs:
 * 		Vir = initial velocity requested
 *		Vt = target velocity requested
 *		Vf = final velocity requested
 *		length = total length of line
 *
 *	Computes:
 *		Vi = actual initial velocity, which may be Vir or less
 *		Vc = cruise velocity, which may be Vt or less
 *		head_length
 *		body_length
 *		tail_length
 *
 *	Returns: 
 *		number of regions - 0-3
 *
 *	Handles these line cases:
 *	  HBT	Line length and speeds support an optimally computed 
 *			head, body and tail. 	Vi=Vir, Vc=Vt.
 *	  HT	Line has head and tail	Vi=Vir  Vc<Vt.
 *	  BT	Line has body and tail	Vi=Vir  Vc=Vir.
 *	  T		Line has tail only		Vi<=Vir Vc=Vi (but has no body)
 *	  HB	Line has head and body	Vi=Vir	Vc=Vf
 *	  H		Line has head only		Vi=Vir	Vc=Vf (but has no body)	
 *	  B		Line has body only		Vi=Vir=Vc=Vt=Vf
 *	  0		No line returned - uncomputable
 */
static uint8_t _mp_compute_regions(const double Vir, const double Vt, const double Vf, struct mpMovePlanner *m) 
{
	double deltaVh;		// HT case values
	double _tmp_length;	// previous body length, or saved tail
	uint8_t i=0;

	// ----- setup M struct with initial values -----
	m->initial_velocity_req = Vir;	// requested initial velocity 
	m->initial_velocity = Vir;		// achieved initial velocity
	m->target_velocity = Vt;		// requested target velocity
	m->cruise_velocity = Vt;		// achieved cruise velocity
	m->final_velocity = Vf;			// this one never changes
	m->head_length = 0;
	m->body_length = 0;
	m->tail_length = 0;

	// ----- 0 case - line is too short or can't span -----
	if (m->length < MIN_LINE_LENGTH) {	// line is too short or zero
		TRAP1(PSTR("Line too short: %f"), m->length);
		return (0);
	}

	// ----- HBT case -----  // compute optimal head and tail lengths
	m->head_length = _mp_get_length(Vir,Vt);
	m->tail_length = _mp_get_length(Vt,Vf);
	m->body_length = m->length - m->head_length - m->tail_length;
	if (m->body_length > 0) {		// exit if no reduction required
		// add sub-minimum heads and tails to body length
		if (m->head_length < MIN_LINE_LENGTH) {
			m->body_length += m->head_length;
			m->head_length = 0;
		}
		if (m->tail_length < MIN_LINE_LENGTH) {
			m->body_length += m->tail_length;
			m->tail_length = 0;
		}
		return (3);
	}

	// ----- H, B & T single region cases -----
	m->body_length = 0;
	if ((Vf < Vir) && (m->length < m->tail_length)) { // T case
		m->head_length = 0;
		m->tail_length = m->length;
		m->initial_velocity = _mp_get_velocity(Vf, m->tail_length);
		m->cruise_velocity = m->initial_velocity;
		return (1);
	}
	if ((Vf > Vir) && (m->length < m->head_length)) { // H case
		m->head_length = m->length;
		m->tail_length = 0;
		m->initial_velocity = m->initial_velocity_req;
		m->cruise_velocity = _mp_get_velocity(Vir, m->head_length);
		m->final_velocity = m->cruise_velocity;
		return (1);
	}
	if ((fabs(Vf-Vir) < EPSILON) && (fabs(Vf-Vt) < EPSILON)) {	// B case	
		m->head_length = 0;
		m->tail_length = 0;
		m->body_length = m->length;
		return (1);
	}

	// ----- HT case -----
	do { // iterate head and tail adjustments to remove body & set Vc
		deltaVh	= fabs(m->initial_velocity - m->cruise_velocity);
		m->head_length = m->length * (deltaVh / (deltaVh + 
			fabs(m->cruise_velocity - m->final_velocity)));
		m->cruise_velocity = _mp_get_velocity(m->initial_velocity, m->head_length);
		m->head_length = _mp_get_length(m->cruise_velocity, m->initial_velocity);
		m->tail_length = _mp_get_length(m->cruise_velocity, m->final_velocity);
		_tmp_length = m->body_length;
		m->body_length = m->length - m->head_length - m->tail_length;
		if (++i > 100) {
			TRAP1(PSTR("Iteration error: %f"), m->body_length);
			break;
		}
	} while (fabs(_tmp_length - m->body_length) > EPSILON);

	if (m->body_length > 0.01) { 
		TRAP1(PSTR("Region error: %f"), m->body_length);
	}
	m->body_length = 0;
	if (m->head_length < EPSILON) {	// clean it up
		m->head_length = 0;
	}
	if (m->tail_length < EPSILON) {	// clean it up
		m->tail_length = 0;
	}
	return (2);		// 2 region return
	// NOTE:
	// If the line splits into 2 lines that are too short to process
	// try it as a 1 segment line - even though this is not optimal,
	// as it will ignore the exact-stop condition. and attempt to join
	// to the previous line at velocity. This is usually OK as the Vi
	// will be very slow due to the shortness of the line - but will 
	// violate the exact-stop condition.
}

/*	
 * _mp_get_length()
 *
 * 	A convenient expression for determining the length of a line given the 
 *	initial velocity (Vi), final velocity (Vf) and the max jerk (Jm):
 *
 *	  length = |Vf-Vi| * sqrt(|Vf-Vi| / Jm)
 *
 *	which is derived from these two equations:
 *
 *	  time = 2 * sqrt(abs(Vf-Vi) / max_linear_jerk);	// 5.x
 *	  length = abs(Vf-Vi) * time / 2;					// [2]
 *
 *	Let the compiler optimize out the Vi=0 & Vf=0 constant cases
 */

inline static double _mp_get_length(const double Vi, const double Vf)
{
	double deltaV = fabs(Vf - Vi);
	return (deltaV * sqrt(deltaV / cfg.max_linear_jerk));
}

/*	
 * _mp_get_velocity()
 *
 * 	Solve this equation to get cruise velocity (Vc) given the initial or 
 *	final velocity (V) and max jerk (Jm). V must be less than Vc. 
 *
 *	  length = (Vc-V) * sqrt((Vc-V) / Jm)
 *
 *	Solves to:
 *
 *	  Vc = Jm^(1/3) * length^(2/3) + V
 *
 *  http://www.wolframalpha.com/input/?i=L%3D%28X-V%29*sqrt%28%28X-V%29%2FJ%29
 */

inline static double _mp_get_velocity(const double V, const double L)
{
	return (mm.linear_jerk_rad3 * pow(L, 0.6666667) + V);
//	return (pow(cfg.max_linear_jerk, 0.33333334) * pow(L, 0.666666667) + V);
}

/*	
 * _mp_queue_move() - write an M structure to buffers
 */

static uint8_t _mp_queue_move(struct mpMovePlanner *m) 
{
	if ((m->head = _mp_queue_buffer(m->initial_velocity, 
									m->cruise_velocity, 
									m->initial_velocity_req,
									m->head_length)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	if ((m->body = _mp_queue_buffer(m->cruise_velocity, 
									m->cruise_velocity, 
									m->target_velocity, 
									m->body_length)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	if ((m->tail = _mp_queue_buffer(m->cruise_velocity, 
									m->final_velocity, 
									m->target_velocity, 
									m->tail_length)) == NULL) {
		return (TG_BUFFER_FULL_FATAL);
	}
	return (TG_OK);
}

static struct mpBuffer *_mp_queue_buffer(const double Vs, const double Ve,	
										 const double Vr, const double len)
{
	uint8_t i;
	struct mpBuffer *b;

	if ((b = _mp_get_write_buffer()) == NULL) { 	// get buffer or die trying
		return (NULL); 
	}
	b->start_velocity = Vs;
	b->end_velocity = Ve;
	b->request_velocity = Vr;
	b->length = len;
	for (i=0; i < AXES; i++) { 			// copy unit vector from mm
		b->unit_vec[i] = mm.unit_vec[i]; 
		mm.position[i] += len * b->unit_vec[i];	// set mm position
		b->target[i] = mm.position[i]; 
	}
	b->replannable = TRUE;
	_mp_queue_write_buffer(_mp_get_move_type(b));
	return(b);									// return pointer
}


/*	
 * _mp_update_move() - update buffers according to M structs
 *
 *	p is move to be updated
 *	m is next move in the chain (not updated)
 */

static void _mp_update_move(const struct mpMovePlanner *p, const struct mpMovePlanner *m) 
{
	// update region buffers from planning structure
	p->head->start_velocity = p->initial_velocity;
	p->head->end_velocity = p->cruise_velocity;
	p->head->request_velocity = p->initial_velocity_req;
	p->head->length = p->head_length;
	p->head->move_type = _mp_get_move_type(p->head);

	p->body->start_velocity = p->cruise_velocity;
	p->body->end_velocity = p->cruise_velocity;
	p->body->request_velocity = p->target_velocity;
	p->body->length = p->body_length;
	p->body->move_type = _mp_get_move_type(p->body);

	p->tail->start_velocity = p->cruise_velocity;
	p->tail->end_velocity = p->final_velocity;
	p->tail->request_velocity = p->final_velocity;
	p->tail->length = p->tail_length;
	p->tail->move_type = _mp_get_move_type(p->tail);

	// set to non-replannable if the move is now optimally planned
	if (((fabs(p->head->start_velocity - p->initial_velocity_req)) < EPSILON)  &&
		((fabs(p->body->start_velocity - p->target_velocity)) < EPSILON) &&
		((fabs(p->tail->end_velocity - m->initial_velocity_req)) < EPSILON)) {
		p->head->replannable = FALSE;
		p->body->replannable = FALSE;
		p->tail->replannable = FALSE;
	}
	return;
}

/*
 * _mp_get_move_type() - based on conditions in the buffer
 */

static uint8_t _mp_get_move_type(const struct mpBuffer *b)
{
	if (b->length < MIN_LINE_LENGTH) {
		return (MP_TYPE_NULL);
	} else if ((fabs(b->start_velocity - b->end_velocity)) < EPSILON) {
		return (MP_TYPE_CRUISE);
	} else if (b->start_velocity < b->end_velocity) {
		return (MP_TYPE_ACCEL);
	} else {
		return (MP_TYPE_DECEL);
	}
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

//static double _mp_estimate_angular_jerk(const struct mpBuffer *p, double previous_velocity)
static double _mp_estimate_angular_jerk(const struct mpBuffer *p)
{
	double j = (sqrt(square(mm.unit_vec[X] - p->unit_vec[X]) +
				 	 square(mm.unit_vec[Y] - p->unit_vec[Y]) +
				 	 square(mm.unit_vec[Z] - p->unit_vec[Z]))/2.0);
//	j *= min(1,(previous_velocity / MAX_VELOCITY)); // +++ remove to test
	return (j);
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

static uint8_t _mp_run_cruise(struct mpBuffer *b)
{
	uint8_t i;
	double steps[AXES];

	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	b->replannable = FALSE;						// stop replanning
	if (b->length < MIN_LINE_LENGTH) {			// toss the line
		return (TG_OK);
	}
	b->time = b->length / b->end_velocity;		// get time from length
	TRAP_IF_TRUE((b->time == 0), PSTR("Time: %f"), b->time)
	mr.microseconds = uSec(b->time);

	for (i=0; i < AXES; i++) {
		mr.target[i] = b->target[i];
		b->target[i] = mr.position[i] + b->unit_vec[i] * b->length; //++++ remove this line for test
		steps[i] = _steps(i, b->target[i]) - _steps(i, mr.position[i]);
	}
	(void)mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mp_set_mr_position(b->target);
	return (TG_OK);
}

static uint8_t _mp_run_accel(struct mpBuffer *b)
{
	uint8_t i;
	
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	// initialize for acceleration
	if (b->move_state == MP_STATE_NEW) {
		b->replannable = FALSE;				// stop replanning
		if (b->length < MIN_LINE_LENGTH) { 	// toss
			return (TG_OK); 
		}
		mr.midpoint_velocity = (b->start_velocity + b->end_velocity) / 2;
		TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Accel Midpoint Velocity: %f"), mr.midpoint_velocity)	
		mr.time = b->length / mr.midpoint_velocity;
		mr.midpoint_acceleration = mr.time * mm.linear_jerk_div2;
		for (i=0; i < AXES; i++) {
			mr.target[i] = b->target[i];	// transfer target to mr
		}
		// number of segments in *each half*
		mr.segments = round(round(ONE_MINUTE_OF_MICROSECONDS * (mr.time / cfg.min_segment_time)) / 2);
		if ((uint16_t)mr.segments == 0) {		
			TRAP1(PSTR("Acceleration Segments: %f"), mr.segments)
			return (TG_OK);					// cancel the move if too small			
		}
		mr.segment_time = mr.time / (2 * mr.segments);
		mr.elapsed_time = mr.segment_time / 2; //compute pos'n from midpoint
		mr.microseconds = uSec(mr.segment_time);
		mr.segment_count = (uint32_t)mr.segments;
		b->move_state = MP_STATE_RUNNING_1;
	}
	// first half of acceleration - concave portion of curve
	if (b->move_state == MP_STATE_RUNNING_1) {
		mr.segment_velocity = b->start_velocity + 
						(mm.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno (_mp_aline_run_segment(b));	 // return is OK, not an error
		// setup for second half
		mr.segment_count = (uint32_t)mr.segments;
		mr.elapsed_time = mr.segment_time / 2;
		b->move_state = MP_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a motor buffer
	}
	// second half of acceleration - convex portion of curve
	if (b->move_state == MP_STATE_RUNNING_2) {
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity + 
						(mr.elapsed_time * mr.midpoint_acceleration) -
						(mm.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mp_aline_run_segment(b));
		} else {
			_mp_aline_run_finalize(b);	// for accuracy
			return(TG_OK);	
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mp_run_decel(struct mpBuffer *b)
{
	uint8_t i;
	
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	// initialize for deceleration
	if (b->move_state == MP_STATE_NEW) {
		b->replannable = FALSE;				// stop replanning
		if (b->length < MIN_LINE_LENGTH) {  // toss
			return (TG_OK); 
		}
		mr.midpoint_velocity = (b->start_velocity + b->end_velocity) / 2;
		TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Decel Midpoint Velocity: %f"), mr.midpoint_velocity)	
		mr.time = b->length / mr.midpoint_velocity;
		mr.midpoint_acceleration = mr.time * mm.linear_jerk_div2;
		for (i=0; i < AXES; i++) {
			mr.target[i] = b->target[i];	// transfer target
		}
		// number of segments in *each half*
		mr.segments = round(round(ONE_MINUTE_OF_MICROSECONDS * (mr.time / cfg.min_segment_time)) / 2);
		if ((uint16_t)mr.segments == 0) {		// more efficient than comparing to < EPSILON
			TRAP1(PSTR("Deceleration Segments: %f"),mr.segments)
			return (TG_OK);					// cancel the move if too small			
		}
		mr.segment_time = mr.time / (2 * mr.segments);
		mr.elapsed_time = mr.segment_time / 2; //compute pos'n from midpoint	
		mr.microseconds = uSec(mr.segment_time);
		mr.segment_count = (uint32_t)mr.segments;
		b->move_state = MP_STATE_RUNNING_1;
	}
	// first half of deceleration
	if (b->move_state == MP_STATE_RUNNING_1) {	// convex part of curve
		mr.segment_velocity = b->start_velocity - 
						(mm.linear_jerk_div2 * square(mr.elapsed_time));
		ritorno(_mp_aline_run_segment(b));	// return is OK, not an error
		// setup for second half
		mr.segment_count = (uint32_t)mr.segments;
		mr.elapsed_time = mr.segment_time / 2;
		b->move_state = MP_STATE_RUNNING_2;
		return (TG_EAGAIN); // no guarantee you can get a motor buffer
	}
	// second half of deceleration
	if (b->move_state == MP_STATE_RUNNING_2) {	// concave part of curve
		if (mr.segment_count > 1) {
			mr.segment_velocity = mr.midpoint_velocity - 
						(mr.elapsed_time * mr.midpoint_acceleration) +
						(mm.linear_jerk_div2 * square(mr.elapsed_time));
			return(_mp_aline_run_segment(b));
		} else {
			_mp_aline_run_finalize(b);	// for accuracy
			return(TG_OK);
		}
	}
	return (TG_ERR);			// shouldn't happen
}

static uint8_t _mp_aline_run_segment(struct mpBuffer *b)
{
	uint8_t i;
	double steps[AXES];

	/* Multiply the computed position by the unit vector to get the 
	 * contribution for each axis. Set the target in absolute coords
	 * (floating point) and compute the relative steps.
	 */
	for (i=0; i < AXES; i++) {
		b->target[i] = mr.position[i] + (b->unit_vec[i] * 
					   mr.segment_velocity * mr.segment_time);
		steps[i] = _steps(i, b->target[i]) - _steps(i, mr.position[i]);
	}
	// queue the line and adjust the variables for the next iteration
	(void)mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	mr.elapsed_time += mr.segment_time;
	_mp_set_mr_position(b->target);
	if (--mr.segment_count > 0) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

static void _mp_aline_run_finalize(struct mpBuffer *b)
{
	uint8_t i;
	double steps[AXES];
	
	// finalize - do the last segment to maintain position accuracy
	mr.length = sqrt(square(mr.target[X] - mr.position[X]) +
					 square(mr.target[Y] - mr.position[Y]) +
					 square(mr.target[Z] - mr.position[Z]) +
					 square(mr.target[A] - mr.position[A]));

	if (mr.length < MIN_LINE_LENGTH) {		// trap zero-length case
		return;
	}
	mr.time = mr.length / b->end_velocity; // ++++ 1#INF problem here (get time from length
	mr.microseconds = uSec(mr.time);

	for (i=0; i < AXES; i++) {
		steps[i] = _steps(i, mr.target[i]) - _steps(i, mr.position[i]);
	}
	(void)mq_queue_line(steps[X], steps[Y], steps[Z], steps[A], mr.microseconds);
	_mp_set_mr_position(mr.target);
	return;
}


//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _mp_test_buffers(void);
void _mp_test_recompute_vt(void);
void _mp_call_recompute_vt(double l, double Vp, double Vi, double Vt);

/*++++ These tests are broken and need re-work

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
	_mp_compute_regions(m->initial_velocity_req, m->target_velocity, 0, m);
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

}
*/

#endif
