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
#include "settings.h"
#include "motion_control.h"
#include "motor_queue.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

uint8_t _mc_run_line(struct mcBuffer *m);
uint8_t _mc_run_aline(struct mcBuffer *m);
uint8_t _mc_run_cruise(struct mcBuffer *m);
uint8_t _mc_run_accel(struct mcBuffer *m);
uint8_t _mc_run_decel(struct mcBuffer *m);
uint8_t _mc_run_start_stop(struct mcBuffer *m);
uint8_t _mc_run_dwell(struct mcBuffer *m);
uint8_t _mc_run_arc(struct mcBuffer *m);

void _mc_set_final_position(struct mcBuffer *m);
void _mc_set_intermediate_position(struct mcBuffer *m);

uint8_t _mc_make_line_buffer(double Vi, double Vt, double len, uint8_t type);
uint8_t _mc_queue_head(void);
uint8_t _mc_queue_body(void);
uint8_t _mc_queue_tail(void);
uint8_t _mc_recompute_previous_tail(struct mcBuffer *p);
uint8_t _mc_recompute_target_velocity(void); 
double _mc_estimate_angular_jerk(struct mcBuffer *p);
uint8_t _mc_line_to_arc(struct mcBuffer *p);

// All the enums that equal zero must be zero. Don't change this

enum mcBufferState {			// m->buffer_state values 
	MC_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MC_BUFFER_LOADING,			// being written ("checked out")
	MC_BUFFER_QUEUED,			// in queue
	MC_BUFFER_RUNNING			// current running buffer
};

enum mcMoveType {				// m->move_type values 
	MC_TYPE_NONE = 0,			// no move specified (MUST BE ZERO)
	MC_TYPE_LINE,				// simple line
	MC_TYPE_CRUISE,				// cruise at fixed velocity
	MC_TYPE_ACCEL,				// max jerk acceleration region
	MC_TYPE_DECEL,				// max jerk deceleration region
	MC_TYPE_ARC,				// arc feed
	MC_TYPE_DWELL,				// delay with no movement
	MC_TYPE_START,				// restart motors
	MC_TYPE_STOP,				// stop motors
	MC_TYPE_END					// stop motors and end program
};

enum mcMoveState {				// m->move_state values
	MC_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MC_STATE_RUNNING_1,			// first half of move or sub-move
	MC_STATE_RUNNING_2			// second half of move or sub-move
};
#define MC_STATE_RUNNING MC_STATE_RUNNING_1	// a convenience for above

struct mcBuffer {				// move/sub-move motion control structure
	// buffer management		// INIT - marks required initial values
	struct mcBuffer *nx;		// INIT: static pointer to next buffer
	struct mcBuffer *pv;		// INIT: static pointer to previous buffer
	uint8_t buffer_state;		// INIT: mcBufferState - manages queues

	// move control variables
	uint8_t move_type;			// INIT: used to dispatch to run routine
	uint8_t move_state;			// INIT: state machine sequence

	// common variables
	double unit_vector[AXES];	// INIT: axis scaling & jerk computation
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
	double center_1;			// INIT: center of circle at axis 1 (typ X)
	double center_2;			// INIT: center of circle at axis 2 (typ Y)
	double segment_theta;		// angular motion per segment
	uint8_t axis_1;				// INIT: arc plane axis
	uint8_t axis_2;				// INIT: arc plane axis
	uint8_t axis_linear;		// INIT: transverse axis (helical)
};

struct mcMotionMaster {	
	uint8_t (*run_move)(struct mcBuffer *m);	// currently running move
	uint8_t run_flag;							// move status
	uint8_t path_mode;			// Gcode path control mode w/adjustments
	uint8_t regions;			// number of regions in resulting move

	// persistent position info
	double position[AXES];		// final move position
	double position_inter[AXES];// sub-move position (intermediate pos'n)

	// common data used and passed around by the trajectory planner
	double target[AXES];		// move target
	double unit_vector[AXES];	// for axis scaling and jerk computation

	double length;				// length of line or helix in mm
	double time;				// total time of move in minutes
	double initial_velocity;	// initial velocity of the move
	double target_velocity;		// target velocity for the move
	double delta_velocity;		// difference between initial and target
	double previous_velocity;	// Vt of previous move (not exit velocity)
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

	cfg.mm_per_arc_segment = MM_PER_ARC_SEGMENT;
	cfg.min_segment_time = MIN_SEGMENT_TIME;

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
			case MC_TYPE_LINE:	{ mm.run_move = _mc_run_line; break; }
			case MC_TYPE_CRUISE:{ mm.run_move = _mc_run_cruise; break; }
			case MC_TYPE_ACCEL:	{ mm.run_move = _mc_run_accel; break; }
			case MC_TYPE_DECEL:	{ mm.run_move = _mc_run_decel; break; }
			case MC_TYPE_ARC:	{ mm.run_move = _mc_run_arc; break; }
			case MC_TYPE_DWELL:	{ mm.run_move = _mc_run_dwell; break; }
			case MC_TYPE_START:	{ mm.run_move = _mc_run_start_stop; break;}
			case MC_TYPE_STOP:	{ mm.run_move = _mc_run_start_stop; break;}
			case MC_TYPE_END: 	{ mm.run_move = _mc_run_start_stop; break;}
		}
	}
	if ((status = mm.run_move(mm.r)) == TG_EAGAIN) { // run current run buf
		return (status);
	}
	mm.run_flag = FALSE;				// finalize and return
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
 *							there earlier buffers can be read using the 
 *							backwards pointers. This buffer cannot be 
 *							queued and should not be end_run'd.
 *
 * A typical usage sequence is:
 *	1 - test if you can get 4 write buffers (worst case needed for aline)
 *	2 - aline first gets previous_buffer to look back at the previous tail
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
		struct mcBuffer *nx = mm.w->nx;	// save pointers
		struct mcBuffer *pv = mm.w->pv;
		memset(mm.w, 0, sizeof(struct mcBuffer));
		w->nx = nx;			// restore pointers
		w->pv = pv;
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
	mm.q = mm.q->nx;		// advance the queued buffer pointer
	return (TG_OK);			// convenience for calling routines
}

struct mcBuffer * mc_get_run_buffer() 
{
	// condition: fresh buffer; buffer becomes running if it's queued
	if (mm.r->buffer_state == MC_BUFFER_QUEUED) {
		mm.r->buffer_state = MC_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mm.r->buffer_state == MC_BUFFER_RUNNING) { // return same buffer
		return (mm.r);
	}
	// condition: no queued buffers. fail it.
	return (NULL);
}

uint8_t mc_end_run_buffer()	// EMPTY current run buf & advance to next
{
	mm.r->buffer_state = MC_BUFFER_EMPTY;
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

/**** POSITION SETTING ROUTINES ******************************************
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

void _mc_set_final_position(struct mcBuffer *m) 
{ 
	memcpy(mm.position, m->target, sizeof(mm.position));
}

void _mc_set_intermediate_position(struct mcBuffer *m) 
{ 
	memcpy(mm.position_inter, m->target, sizeof(mm.position_inter));
}

/**** STOP START AND END ROUTINES ****************************************
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

/*************************************************************************
 * mc_dwell() - queue a dwell (non-blocking behavior)
 * mc_dwell_continue() - dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the move but does not 
 * send any pulses. Only the X axis is used to time the dwell - 
 * the others are idle.
 */

uint8_t mc_dwell(double seconds) 
{
	struct mcBuffer *m; 

	if ((m = mc_get_write_buffer()) == NULL) {	// get write buffer or fail
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

/*************************************************************************
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

	if ((m = mc_get_write_buffer()) == NULL) {	// get write buffer or fail
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
		mc_end_run_buffer();		// early exit requires you free buffer
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

/************************************************************************* 
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
 *	The move buffer is tested and sometime later its queued (via mc_line())
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
		mc_end_run_buffer();		// early exit requires you free buffer
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

/*************************************************************************
 * mc_aline() 		- queue line move with acceleration / deceleration
 * _mc_run_aline()	- run accel move 
 *
 *	This module uses a cubic spline solution to generate acceleration and 
 *	deceleration ramps that obey maximum jerk parameters. The motion 
 *	equations were taken or derived from Ed Red's BYU robotics course: 
 *	  http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *
 *	A typical move (line or arc) is divided into 3 regions (sub-moves):
 *	  - head	initial acceleration to target velocity
 *	  - body	bulk of move at target speed (cruise region)
 *	  - tail	ending deceleration to exit velocity
 *
 *	The head is computed from the exit velocity of the previous move,
 *	from zero velocity, or somewhere in between. The body is the "cruise 
 *	region" where the line is running at its target velocity (Vt). 
 *	The tail (usually) decelerates to the initial velocity (Vi) of 
 *	the next line. See special cases where some of the above is not true.
 *
 *	The tail length is reserved to allow a worst-case deceleration from
 *	the target velocity to zero. The tail is also used as a deceleration
 *	region to enter the next move at its Vt, so the tail is often 
 *	re-computed as the next move is being computed and queued.
 *
 *	The computation of the regions is set by path control mode in effect:
 *
 *	  - CONTINUOUS_MODE (G64): The moves will attempt to run at their 
 *		maximum theoretical speeds, accelerating or decelerating at line 
 *		junctions to match speeds and maintain maximum velocity. 
 *
 *	  - EXACT_PATH_MODE (G61.1): The move is spliced to the next move with 
 *		an attempt to keep the path as accurate as possible. The splice 
 *		computes a maximum jerk based on the change in velocity and
 *		direction (vector) between the two lines, then decelerates the
 *		current line to a computed "safe" velocity before accelerating 
 *		into the next line. For 180 degree turns the line will stop 
 *		completely before reversing.
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
 *	  - CONTINUOUS MODE is the default mode. If the angle between two lines
 *		is too sharp (angular jerk is too high) the mode will be downgraded
 *		to exact path mode for that line only (which may in turn get 
 *		downgraded to exact stop mode). In the future continuous mode 
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
 *		- ACCELERATING:	The body and tail of the previous line are extended
 *						to the join with the new line. The new line 
 *						performs acceleration to Vt.
 *
 *	  - EXACT_PATH_MODE is similar to continuous mode except that the
 *		previous line will decelerate (if needed) to a "safe" speed at 
 *		the join. The join speed is computed based on the estimated 
 *		angular jerk between the two lines. The new line accelerates from
 *		the join speed. If the angular jerk is too extreme (join angle is 
 *		too sharp) exact path mode will be further downgraded to exact 
 *		stop mode (again, for that line only).
 *
 *	  - EXACT_STOP_MODE: is the same as exact path mode except the join
 *		speed is zero. Exact stop is always used for 180 degree joins.
 *
 *  Combined Cases - By the time you combine all these you get a series of 
 *	combined cases, velocity relationships, and --> short-line morph cases
 *	(which really should be drawn to better visualize)
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
 *	if the new line is too short to support a deceleration to zero (and 
 *	therefore cannot have a full tail pre-computed). These short line 
 *	cases cause the above cases to morph into other cases - all of which 
 *	are captured above.
 *
 *	  -	In some cases the line is too short to reach Vt (cruise velocity). 
 *		The target velocity is scaled down to a maximum achievable 
 *		velocity that still supports maximum jerk acceleration and
 *		deceleration curves. The head and tail join directly at 
 *		that maximum velocity. There is no body. 
 *
 *	  - In still other cases the line is even too short to get to zero
 *		velocity from the initial velocity. In this case the initial 
 *		velocity is re-computed to support a clean deceleration and the
 *		previous tail is decelerated even more severely to meet this Vi.
 *
 *	The following cases apply for joining lines to arcs and arcs to arcs.
 *	At the current time only continuous mode is supported (no acceleration
 *	or deceleration is supported within an arc).
 *
 *	  - Line follows an arc: The head accelerates or decelerates from the 
 *		exit velocity of the arc - or there is no head if the and speed and 
 *		the line speed are the same. 
 *
 *	  - Line is followed by an arc: The line tail is used to accelerate or
 *		decelerate to match the arc feed rate.
 *
 *	  - Arc to arc blending: is not currently supported... 
 *		...so a velocity step may occur between arcs of different speeds. 
 *		A discontinuous step will also occur if an arc is started from 
 *		zero velocity or stopped to zero velocity.(for now, until fixed)
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
 * Notes:
 *	(1) See the excel sheet that has the equations and simulations. 
 *
 *	(2)	All math is done in double precision floating point and minutes 
 *		until the very end, when it's converted to steps and microseconds 
 *		for queueing motor moves
 *
 *	(3)	An aline() will need between 2 and 4 write buffers. Before calling
 *		aline() you MUST test that MAX_BUFFERS_NEEDED buffers are avialable
 *		or aline() could fail fatally.
 *
 *	(4)	You may notice that initialized line buffers use Vi, Vt and Length
 *		but do not require Time. Time is derived from velocities, length
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
	struct mcBuffer *p; 	// previous tail buffer pointer

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
	if (p->move_type == MC_TYPE_ARC) {				// a queued arc
		mm.path_mode = PATH_CONTINUOUS_FROM_ARC;	// force all to arc case
		mm.previous_velocity = p->target_velocity;
	} else {										// line cases
		mm.path_mode = cfg.gcode_path_control;		// requested path mode
		mm.previous_velocity = p->initial_velocity;
	}

	// estimate the angular jerk and downgrade path control modes
	for (i=0; i < AXES; i++) {						// generate unit vector
		mm.unit_vector[i] = (mm.target[i] - mm.position[i]) / mm.length;
	}
	mm.angular_jerk = _mc_estimate_angular_jerk(p);	// uses unit vector
	if (p->buffer_state != MC_BUFFER_QUEUED) {// prev tail EMPTY or RUNNING
		mm.path_mode = PATH_EXACT_STOP;
	} else {
		if ((mm.path_mode == PATH_CONTINUOUS) && 
			(mm.angular_jerk > cfg.angular_jerk_lower)) {
			mm.path_mode = PATH_EXACT_PATH;
		} 
		if ((mm.path_mode == PATH_EXACT_PATH) && 
			(mm.angular_jerk > cfg.angular_jerk_upper)) {
			mm.path_mode = PATH_EXACT_STOP;	
		}
	}

// ------ Compute all the velocities, lengths, and region counts -------

	// set preliminary velocities based on path control mode
	mm.target_velocity = mm.length / mm.time;		// Vt before reduction
	if (mm.path_mode == PATH_CONTINUOUS_FROM_ARC) {
		mm.initial_velocity = p->target_velocity;
	} else if (mm.path_mode == PATH_EXACT_STOP) {
		mm.initial_velocity = 0;
	} else if (mm.target_velocity > mm.previous_velocity) {// accel cases
		mm.initial_velocity = mm.previous_velocity;
	} else { 										// decel and cruise
		mm.initial_velocity = mm.target_velocity;
	}
	if (mm.path_mode == PATH_EXACT_PATH) {		// adjust for exact path
		mm.initial_velocity *= mm.angular_jerk;
	}
	// precompute optimal head and tail lengths. See header note (4),
	mm.tail_length = mm.target_velocity * sqrt(mm.target_velocity / cfg.max_linear_jerk);
	mm.delta_velocity = fabs(mm.target_velocity - mm.initial_velocity);
	mm.head_length = mm.delta_velocity * sqrt(mm.delta_velocity / cfg.max_linear_jerk);

	// adjust Vt for short line cases and set regions
	mm.regions = 3;
	if (mm.length <= (mm.tail_length + mm.head_length)) {
		if (_mc_recompute_target_velocity() == TG_OK) {
			mm.regions = 2;						// no body
		} else {
			mm.regions = 1;						// tail only (super short)
		}
	}

// ------ Sort out the cases and call the subroutines -------

	if (mm.path_mode == PATH_CONTINUOUS_FROM_ARC) {	// dispense with arcs
		ritorno(_mc_line_to_arc(p));
		return(mc_set_position(mm.target[X], mm.target[Y], 
							   mm.target[Z], mm.target[A]));
	}
	// line to line cases
	ritorno(_mc_recompute_previous_tail(p));
	// CRUISE and DECELERATION cases
	if (mm.target_velocity <= mm.initial_velocity) {
		if (mm.regions == 1) {				
			ritorno(_mc_queue_tail());			// tail only case
		} else {
			ritorno(_mc_queue_body());			// normal body case
			ritorno(_mc_queue_tail());
		}
	// ACCELERATION cases
	} else if (mm.target_velocity > mm.initial_velocity) {
		if (mm.regions == 1) {
			ritorno(_mc_queue_tail());			// tail only case
		} else if (mm.regions == 2) {
			ritorno(_mc_queue_head());			// no body case
			ritorno(_mc_queue_tail());
		} else {
			ritorno(_mc_queue_head());			// normal body case
			ritorno(_mc_queue_body());
			ritorno(_mc_queue_tail());
		}
	}
	// final position for the move (not sub-move)
	return(mc_set_position(mm.target[X], mm.target[Y], 
						   mm.target[Z], mm.target[A]));
}

/**** ALINE HELPERS ****
 * _mc_make_line_buffer()  		- helper helper for making line buffers
 * _mc_queue_head()				- queue a 3 part move
 * _mc_queue_body()				- queue a 2 part move with no body
 * _mc_queue_tail()				- queue a 2 part move with no head
 * _mc_line_to_arc()
 * _mc_recompute_target_velocity() - adjust Vt to Vt achievable in length
 * _mc_recompute_previous_tail()   - join previous tail to Vi
 * _mc_estimate_angular_jerk()	   - factor of 0 to 1 where 1 = max jerk
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

uint8_t _mc_queue_head() {
	return(_mc_make_line_buffer(mm.initial_velocity, mm.target_velocity,
 					 		 	mm.head_length, MC_TYPE_ACCEL));
}

uint8_t _mc_queue_body() {
	return(_mc_make_line_buffer(mm.target_velocity, mm.target_velocity,
		  (mm.length - mm.head_length - mm.tail_length), MC_TYPE_CRUISE));
}

uint8_t _mc_queue_tail() {
	return(_mc_make_line_buffer(mm.target_velocity, 0, 
								mm.tail_length, MC_TYPE_DECEL));
}

uint8_t _mc_line_to_arc(struct mcBuffer *p)
{
	return(TG_OK);
}
/*
 * _mc_recompute_target_velocity()
 *
 *	This function handles the case where the line length and velocities
 *	cannot support a full-speed cruise region. The target velocity must
 *	be reduced to a point where the head and tail can be joined directly
 *	with no intervening body.
 *
 *	This function shoiuld be called before an acceleration to be able to 
 *	properly fit the previous tail to the ultimate Vi of the new line.
 *	There are three exit conditions:
 *
 *	  -	Line will fit into a normal accel/decel profile. 
 *		Vt and Vi are unaffected.
 *
 *	  -	Vt falls above Vi
 *		Vt is reduced to accommodate max-jerk head and tail regions.
 *		Vi remains the same.
 *
 *	  -	Vt falls below Vi
 *		Vt is reduced.
 *		Vi is set to the new Vt to allow the previous tail to meet it.
 *
 *	The equations to directly compute the new target velocity are not 
 *	practical to solve on this tiny little computer :(   [see spreadsheet]
 *	Instead we perform an iterative linear approximation to converge on
 *	the reduced velocity while preserving the correct total length. The 
 *	tail length and head length equations are the length equation 
 *	described in aline() header note (4).
 */

uint8_t _mc_recompute_target_velocity() 
{
	uint8_t i=0;
	double target_velocity = mm.target_velocity;	// temps
	double delta_velocity = mm.delta_velocity;
	double length;

	// test if line fits std. accel/decel curves and doesn't need reduction
	if (mm.length > (mm.head_length + mm.tail_length)) {
		return (TG_PARAMETER_OVER_RANGE);
	}
	// iterate on the optimal Vt for reduced accel/decel regions
	while (fabs(mm.length - mm.head_length - mm.tail_length) > 0.002) { // mm
		target_velocity *= mm.length / (mm.head_length + mm.tail_length);
		delta_velocity = fabs(target_velocity - mm.initial_velocity);
		mm.tail_length = target_velocity * sqrt(target_velocity / cfg.max_linear_jerk);
		mm.head_length = delta_velocity * sqrt(delta_velocity / cfg.max_linear_jerk);
		if (i++ > 40) {	// chose this value with lots of experimentation
			break; 		// usually converges in < 20 - this is for safety
		}
	}
	// In some cases above the new Vt is less than the initial velocity,
	// so conditionally compute best achievable Vt given the Vi and length
	if (target_velocity < mm.initial_velocity) {
		i=0;
		target_velocity = mm.target_velocity;	// reset
		length = mm.head_length;				// OK starting value
		while (fabs(mm.length - length) > 0.002) {// mm
			target_velocity *= mm.length / length;
			length = target_velocity * sqrt(target_velocity / cfg.max_linear_jerk);
			if (i++ > 40) {
				break;
			}
		}
		mm.target_velocity = target_velocity;
		mm.initial_velocity = target_velocity; 	// reset Vi for Vt < Vi case
		mm.delta_velocity = 0;				   	// by definition
		// recompute tail_length. head_length will be ignored 
		mm.tail_length = target_velocity * sqrt(target_velocity / cfg.max_linear_jerk);
		return (TG_PARAMETER_UNDER_RANGE);
	}
	mm.target_velocity = target_velocity;
	mm.delta_velocity = delta_velocity;
	return (TG_OK);
}

/*
 * _mc_recompute_previous_tail()
 *
 *	Recompute the previous move (P) to end so it's exit velocity (Ve) is 
 *	the same as the initial velocity of the current move (Vi). Ve could be
 *	greater or less than the max velocity of the previous move (Vtp). 
 *	The previous move could be a 1, 2, or 3 buffer move, so this must be 
 *	taken into account.
 *
 *	This routine assumes the original tail is in place and the starting 
 *	value for Ve is therefore zero.
 *
 *	There is a pathological case where the P move needs to be accelerated
 *	to join an arc but can't reach the arc's Vi in the given tail region. 
 *	In this case do the best you can towards the final velocity (and live 
 *	with the velocity step going into the arc).
 */

uint8_t _mc_recompute_previous_tail(struct mcBuffer *p)
{
	struct mcBuffer *m;			// pointer to active buffer
	double length;				// desired tail length

	// exit if the buffer anything but queued and idle. Leaves Ve=0
	if (p->buffer_state != MC_BUFFER_QUEUED) {
		return (TG_OK);
	}

	// exit if Vi = 0 as no adjustment is required
	if (mm.initial_velocity == 0) {
		return (TG_OK);
	}

	// handle the trivial case where Ve = Vtp = Vi
	if (mm.initial_velocity == mm.previous_velocity) {
		p->move_type = MC_TYPE_CRUISE;		// change tail to a cruise
		p->target_velocity = mm.previous_velocity;
		return (TG_OK);					// no need to update unit_vector
	} 

	// compute the new tail length - regardless of accel or decel case	
	length = fabs(mm.previous_velocity - mm.initial_velocity) * 
			 sqrt((mm.previous_velocity - mm.initial_velocity) / 
			 cfg.max_linear_jerk);

	p->move_type = MC_TYPE_CRUISE;
	p->target_velocity = mm.previous_velocity;
	p->length -= length;

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
	return (TG_OK);
}

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

double _mc_estimate_angular_jerk(struct mcBuffer *p)
{
	return (sqrt(square(mm.unit_vector[X] - p->unit_vector[X]) +
				 square(mm.unit_vector[Y] - p->unit_vector[Y]) +
				 square(mm.unit_vector[Z] - p->unit_vector[Z]))/2.0);
}

/**** ALINE RUN ROUTINES ****
 *	_mc_run_cruise()
 *	_mc_run_accel()
 *	_mc_run_decel()
 */

uint8_t _mc_run_cruise(struct mcBuffer *m)
{
	if (!mq_test_motor_buffer()) { 
		return (TG_EAGAIN); 
	}
	m->time = m->length / m->target_velocity;	// must derive time from length
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
		m->midpoint_velocity = (m->target_velocity + m->initial_velocity) / 2;
		m->time = m->length / m->midpoint_velocity;
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_count = m->segments / 2;
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->microseconds = uSec(m->segment_time);
		m->linear_jerk_div2 = cfg.max_linear_jerk / 2;
		m->midpoint_acceleration = m->time * m->linear_jerk_div2;
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
		// setup for second half
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
		m->midpoint_velocity = (m->target_velocity + m->initial_velocity) / 2;
		m->time = m->length / m->midpoint_velocity;
		m->segments = uSec(m->time / cfg.min_segment_time);
		m->segment_time = m->time / m->segments;
		m->elapsed_time = 0;
		m->segment_count = m->segments / 2;
		m->microseconds = uSec(m->segment_time);
		m->linear_jerk_div2 = cfg.max_linear_jerk / 2;
		m->midpoint_acceleration = m->time * m->linear_jerk_div2;
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
		// setup for second half
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
