/*
 * planner.c - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
 *	operation is complete.
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include <string.h>

#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "util.h"
#include "xio.h"				// supports trap and debug statements

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// bf->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_PENDING,			// marked as the next buffer to run
	MP_BUFFER_RUNNING			// current running buffer
};

struct mpBuffer {				// See Planning Velocity Notes for variable usage
	double linenum;				// line number; or block count if not numbered
	struct mpBuffer *pv;		// static pointer to previous buffer
	struct mpBuffer *nx;		// static pointer to next buffer
	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// move state machine sequence
	uint8_t replannable;		// TRUE if move can be replanned

	double target[AXES];		// target position in floating point
	double unit[AXES];			// unit vector for axis scaling & planning

	double time;				// line, helix or dwell time in minutes
	double length;				// total length of line or helix in mm
	double head_length;
	double body_length;
	double tail_length;
								// *** SEE NOTES ON THESE VARIABLES, in aline() ***
	double entry_velocity;		// entry velocity requested for the move
	double cruise_velocity;		// cruise velocity requested & achieved
	double exit_velocity;		// exit velocity requested for the move

	double entry_vmax;			// max junction velocity at entry of this move
	double cruise_vmax;			// max cruise velocity requested for move
	double exit_vmax;			// max exit velocity possible (redundant)
	double delta_vmax;			// max velocity difference for this move
	double braking_velocity;	// current value for braking velocity

	double jerk;				// linear jerk term for this move
	double recip_jerk;			// 1/Jm				compute-once term
	double cubert_jerk;			// pow(Jm,(1/3))	compute-once term
};

struct mpBufferPool {			// ring buffer for sub-moves
	struct mpBuffer *w;			// get_write_buffer pointer
	struct mpBuffer *q;			// queue_write_buffer pointer
	struct mpBuffer *r;			// get/end_run_buffer pointer
	struct mpBuffer bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage
};

struct mpMoveMasterSingleton {	// common variables for planning (move master)
	double position[AXES];		// final move position
#ifdef __UNIT_TESTS
	double test_case;
	double test_velocity;
	double a_unit[AXES];
	double b_unit[AXES];
#endif
};

struct mpMoveRuntimeSingleton {	// persistent runtime variables
	uint8_t (*run_move)(struct mpBuffer *m); // currently running move
	uint8_t status;				// return status
	uint8_t move_state;			// state machine value
	uint8_t sub_state;			// state machine value within a move section
	double linenum;				// line/block number of BF being processed

	double position[AXES];		// final move position
	double target[AXES];		// target move position
	double section_target[AXES];// target for current section

	double length;				// length of line or helix in mm
	double move_time;			// total running time (derived)
	double accel_time;			// total pseudo-time for acceleration calculation
	double elapsed_accel_time;	// current running time for accel calculation
	double endpoint_velocity;	// exit velocity for section
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double midpoint_acceleration;//acceleration at the midpoint
	double jerk_div2;			// max linear jerk divided by 2

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_move_time;	// actual time increment per aline segment
	double segment_accel_time;	// time increment for accel computation purposes
	double microseconds;		// line or segment time in microseconds
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
};

static struct mpBufferPool mb;			// move buffer queue
static struct mpMoveMasterSingleton mm;	// static context for planning
static struct mpMoveRuntimeSingleton mr;// static context for runtime

/*
 * Local Scope Data and Functions
 */

// execute routines
static uint8_t _mp_exec_line(struct mpBuffer *bf);
static uint8_t _mp_exec_dwell(struct mpBuffer *bf);
static uint8_t _mp_exec_stops(struct mpBuffer *bf);
static uint8_t _mp_exec_aline(struct mpBuffer *bf);
static uint8_t _mp_exec_aline_head(struct mpBuffer *bf);
static uint8_t _mp_exec_aline_body(struct mpBuffer *bf);
static uint8_t _mp_exec_aline_tail(struct mpBuffer *bf);
static uint8_t _mp_exec_aline_segment(struct mpBuffer *bf);
//static void _mp_finalize_aline_segment(struct mpBuffer *bf);

//UNUSED static void _mp_kill_dispatcher(void);

// planner buffer management routines
static void _mp_init_buffers(void);
static void _mp_unget_write_buffer(void);
static void _mp_clear_buffer(struct mpBuffer *bf); 
static void _mp_queue_write_buffer(const uint8_t move_type);
static void _mp_finalize_run_buffer(void);
static struct mpBuffer * _mp_get_write_buffer(void); 
static struct mpBuffer * _mp_get_run_buffer(void);
static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *bf);
static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *bf);

// aline planner routines
static uint8_t _mp_calculate_trapezoid(struct mpBuffer *bf);
static uint8_t _mp_calculate_trapezoid_finalize(struct mpBuffer *bf);
static double _mp_get_target_length(const double Vi, const double Vt, const struct mpBuffer *bf);
static double _mp_get_target_velocity(const double Vi, const double L, const struct mpBuffer *bf);
static double _mp_get_junction_vmax(const double a_unit[], const double b_unit[]);
static double _mp_get_corner_delta(const double a_unit[], const double b_unit[]);
static void _mp_stop_replanning(struct mpBuffer *bf);

#ifdef __DEBUG
static uint8_t _mp_get_buffer_index(struct mpBuffer *bf); 
static void _mp_dump_plan_buffer(struct mpBuffer *bf);
#endif

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
 * mp_exec_move() - execute runtime routine to pre move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 *	Responsible for freeing the completed run buffers
 */
uint8_t mp_exec_move() 
{
	uint8_t status;
	struct mpBuffer *bf;

	if ((bf = _mp_get_run_buffer()) == NULL) {// NULL means nothing's running
		return (TG_NOOP);
	}
	// first time setup
	if (bf->move_state == MOVE_STATE_NEW) {	// first time in?
		switch (bf->move_type) { 			// setup the dispatch vector
			case MOVE_TYPE_LINE:	{ mr.run_move = _mp_exec_line; break; }
			case MOVE_TYPE_ALINE:	{ mr.run_move = _mp_exec_aline; break; }
			case MOVE_TYPE_DWELL:	{ mr.run_move = _mp_exec_dwell; break; }
			case MOVE_TYPE_START:	{ mr.run_move = _mp_exec_stops; break; }
			case MOVE_TYPE_STOP:	{ mr.run_move = _mp_exec_stops; break; }
			case MOVE_TYPE_END: 	{ mr.run_move = _mp_exec_stops; break; }
			default: INFO1(PSTR("Bad move type %d in mp_dispatcher()"), bf->move_type);
		}
	}
	// call the move
	if ((status = mr.run_move(bf)) != TG_EAGAIN) { // run current run buffer
		_mp_finalize_run_buffer();
	}
	return (status);
}

/* 
 * _mp_kill_dispatcher() - kill current move and reset dispatcher
 */
/* UNUSED
void _mp_kill_dispatcher() 
{
	struct mpBuffer *bf;

	if ((b = _mp_get_run_buffer()) != NULL) {// NULL means nothing's running
		bf->move_state = MOVE_STATE_END;
		mr.run_flag = FALSE;
		_mp_finalize_run_buffer();
	}
}
*/

/**** PLANNER BUFFER ROUTINES *********************************************
 *
 * mp_test_write_buffer()	Returns TRUE if a write buffer is available
 *
 * _mp_init_buffers()		Initializes or resets buffers
 *
 * _mp_get_write_buffer()	Get pointer to next available write buffer
 *							Returns pointer or NULL if no buffer available.
 *
 * _mp_unget_write_buffer() Free write buffer if you decide not to queue it.
 *
 * _mp_queue_write_buffer() Commit the next write buffer to the queue
 *							Advances write pointer & changes buffer state
 *
 * _mp_get_run_buffer()		Get pointer to the next or current run buffer
 *							Returns a new run buffer if prev buf was ENDed
 *							Returns same buf if called again before ENDing
 *							Returns NULL if no buffer available
 *							The behavior supports continuations (iteration)
 *
 * _mp_finalize_run_buffer() Release the run buffer & return to buffer pool.
 *
 * _mp_request_finalize_run_buffer() Request that a finalize be run before 
 *							the next planning pass. This allows the exec routine
 *							to free the buffer w/o stomping on the main loop.
 *							
 * mp_invoke_finalize_run_buffer() Execute the finalize request. This is how 
 *							the main loop completes a request. 
 *
 * _mp_get_prev_buffer(bf) Return pointer to prev buffer in linked list
 * _mp_get_next_buffer(bf) Return pointer to next buffer in linked list 
 * _mp_clear_buffer(bf)	   Zero the contents of the buffer
 *
 * Notes:
 *	The write buffer pointer only moves forward on queue_write, and 
 *	the read buffer pointer only moves forward on finalize_read calls.
 *	(test, get and unget have no effect)
 */

uint8_t mp_test_write_buffer()
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
		return (TRUE);
	}
	return (FALSE);
}

static void _mp_init_buffers()
{
	struct mpBuffer *pv;
	uint8_t i;

	memset(&mb, 0, sizeof(mb));		// clear all values, pointers and status
	mb.w = &mb.bf[0];				// init write and read buffer pointers
	mb.q = &mb.bf[0];
	mb.r = &mb.bf[0];
	pv = &mb.bf[PLANNER_BUFFER_POOL_SIZE-1];
	for (i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {  // setup ring pointers
		mb.bf[i].nx = &mb.bf[_mp_bump(i)];
		mb.bf[i].pv = pv;
		pv = &mb.bf[i];
	}
}

static struct mpBuffer * _mp_get_write_buffer() // get & clear a buffer
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
		struct mpBuffer *w = mb.w;
		struct mpBuffer *nx = mb.w->nx;			// save pointers
		struct mpBuffer *pv = mb.w->pv;
		memset(mb.w, 0, sizeof(struct mpBuffer));
		w->nx = nx;								// restore pointers
		w->pv = pv;
		w->buffer_state = MP_BUFFER_LOADING;
		mb.w = w->nx;
		return (w);
	}
	return (NULL);
}

static void _mp_unget_write_buffer()
{
	mb.w = mb.w->pv;							// queued --> write
	mb.w->buffer_state = MP_BUFFER_EMPTY; 		// not loading anymore
}

static void _mp_queue_write_buffer(const uint8_t move_type)
{
	mb.q->move_type = move_type;
	mb.q->move_state = MOVE_STATE_NEW;
	mb.q->buffer_state = MP_BUFFER_QUEUED;
	mb.q = mb.q->nx;							// advance the queued buffer pointer
	st_request_exec_move();						// request a move exec if not busy
}

static struct mpBuffer * _mp_get_run_buffer() 
{
	// condition: fresh buffer; becomes running if queued or pending
	if ((mb.r->buffer_state == MP_BUFFER_QUEUED) || 
		(mb.r->buffer_state == MP_BUFFER_PENDING)) {
		mb.r->buffer_state = MP_BUFFER_RUNNING;
	}
	// condition: asking for the same run buffer for the Nth time
	if (mb.r->buffer_state == MP_BUFFER_RUNNING) {	// return same buffer
		return (mb.r);
	}
	return (NULL);								// condition: no queued buffers. fail it.
}

static void _mp_finalize_run_buffer()			// EMPTY current run buf & adv to next
{
	_mp_clear_buffer(mb.r);						// clear it out (& reset replannable)
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;								// advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {	// only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING; 	// pend next buffer
	}
}

static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *bf)
{
	return (bf->pv);
}

static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *bf)
{
	return (bf->nx);
}

static void _mp_clear_buffer(struct mpBuffer *bf) 
{
	struct mpBuffer *nx = bf->nx;	// save pointers
	struct mpBuffer *pv = bf->pv;
	memset(bf, 0, sizeof(struct mpBuffer));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

#ifdef __DEBUG	// currently this routine is only used by debug routines
static uint8_t _mp_get_buffer_index(struct mpBuffer *bf) 
{
	struct mpBuffer *b = bf;	// temp buffer pointer

	for (uint8_t i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
		if (b->pv > b) {
			return (i);
		}
		b = b->pv;
	}
	return (PLANNER_BUFFER_POOL_SIZE);	// should never happen
}
#endif

/* 
 * mp_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */
uint8_t mp_isbusy()
{
	return (mp_isbusy());
	if ((st_isbusy() == TRUE) || (mr.move_state > MOVE_STATE_NEW)) {
		return (TRUE);
	}
	return (FALSE);
}

/*
 * mp_set_plan_position() - sets planning position (for G92)
 * mp_get_plan_position() - returns planning position
 * mp_set_axis_position() - sets both planning and runtime positions (for G2/G3)
 * mp_get_runtime_position() - returns current running position
 *
 * 	Keeping track of position is complicated by the fact that moves can
 *	require multiple reference frames. The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for line planning
 *	 - mr.position	- current position of runtime segment
 *	 - mr.target	- target position of runtime segment
 *	 - bf->target	- final target position of runtime segment
 *
 *	Note that the positions are set immediately when they are computed and 
 *	are not an accurate representation of the tool position. In reality 
 *	the motors will still be processing the action and the real tool 
 *	position is still close to the starting point.
 */

uint8_t mp_set_plan_position(const double position[])
{
	copy_axis_vector(mm.position, position);
	return (TG_OK);
}

double *mp_get_plan_position(double position[])
{
	copy_axis_vector(position, mm.position);	
	return (position);
}

uint8_t mp_set_axis_position(const double position[])
{
	copy_axis_vector(mm.position, position);
	copy_axis_vector(mr.position, position);
	return (TG_OK);
}

double *mp_get_runtime_position(double vector[])
{
	copy_axis_vector(vector, mr.position);
	return (vector);
}

double mp_get_runtime_velocity(void)
{
	return (mr.segment_velocity);
}

double mp_get_runtime_linenum(void)
{
	return (mr.linenum);
}

/**** STOP START AND END ROUTINES ****************************************
 * mp_async_stop() 		- stop current motion immediately
 * mp_async_start()		- (re)start motion
 * mp_async_end()		- stop current motion immediately
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
 *
 *	The async routines must be safe to call from ISRs. Mind the volatiles.
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

void mp_queued_stop() 
{
	if (_mp_get_write_buffer() == NULL) {
		INFO(PSTR("Failed to get buffer in mp_queued_stop()"));
		return;
	}
	_mp_queue_write_buffer(MOVE_TYPE_STOP);
}

void mp_queued_start() 
{
	if (_mp_get_write_buffer() == NULL) {
		INFO(PSTR("Failed to get buffer in mp_queued_start()"));
		return;
	}
	_mp_queue_write_buffer(MOVE_TYPE_START);
}

void mp_queued_end() // +++ fix this. not right yet. resets must also be queued
{
	if (_mp_get_write_buffer() == NULL) {
		INFO(PSTR("Failed to get buffer in mp_queued_end()"));
		return;
	}
	_mp_queue_write_buffer(MOVE_TYPE_END);
}

static uint8_t _mp_exec_stops(struct mpBuffer *bf) 	// NEW
{
	(void)st_prep_stops(bf->move_type);
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
	struct mpBuffer *bf; 

	if ((bf = _mp_get_write_buffer()) == NULL) {// get write buffer or fail
		INFO(PSTR("Failed to get buffer in mp_dwell()"));
		return (TG_BUFFER_FULL_FATAL);		   // (not supposed to fail)
	}
	bf->time = seconds;						   // in seconds, not minutes
	_mp_queue_write_buffer(MOVE_TYPE_DWELL);
	return (TG_OK);
}

static uint8_t _mp_exec_dwell(struct mpBuffer *bf)	// NEW
{
	(void)st_prep_dwell((uint32_t)(bf->time * 1000000));	// convert seconds to uSec
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

uint8_t mp_line(const double target[], const double minutes)
{
	struct mpBuffer *bf;

	if (minutes < EPSILON) {
		return (TG_ZERO_LENGTH_MOVE);
	}
	if ((bf = _mp_get_write_buffer()) == NULL) {// get write buffer or fail
		INFO(PSTR("Failed to get buffer in mp_line()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	bf->time = minutes;
	copy_axis_vector(bf->target, target);		// target to bf_target
	bf->length = get_axis_vector_length(target, mr.position);
	if (bf->length < MIN_SEGMENT_LENGTH) {
		_mp_unget_write_buffer();				// free buffer if early exit
		return (TG_ZERO_LENGTH_MOVE);
	}
	bf->cruise_vmax = bf->length / bf->time;	// for yuks
	_mp_queue_write_buffer(MOVE_TYPE_LINE);
	copy_axis_vector(mm.position, bf->target);	// update planning position
	return(TG_OK);
}

static uint8_t _mp_exec_line(struct mpBuffer *bf) 
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	for (i=0; i < AXES; i++) {
		travel[i] = bf->target[i] - mr.position[i];
	}
	mr.microseconds = uSec(bf->time);
	(void)ik_kinematics(travel, steps, mr.microseconds);
	if (st_prep_line(steps, mr.microseconds) == TG_OK) {
		copy_axis_vector(mr.position, bf->target);	// update runtime position
	}
	return (TG_OK);
}

/**************************************************************************
 * mp_aline() - plan a line with acceleration / deceleration
 *
 *	This function uses constant jerk motion equations to plan acceleration 
 *	and deceleration. The jerk is the rate of change of acceleration; it's
 *	the 1st derivative of acceleration, and the 3rd derivative of position. 
 *	Jerk is a measure of impact to the machine can take. Controlling jerk 
 *	smoothes transitions between moves and allows for faster feeds while 
 *	controlling machine oscillations and other undesirable side-effects.
 *
 *	A detailed explanation of how this module works can be found in the 
 *	tinyg_docs_developers.txt file and on the wiki at:
 *  http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info:#Acceleration_Planning
 *
 * 	Note: All math is done in absolute coordinates using "double precision" 
 *	floating point (even though AVRgcc does this as single precision)
 */

uint8_t mp_aline(const double target[], const double minutes)
{
	struct mpBuffer *bf, *bp; 	// current move pointer and working pointer
	double exact_stop, junction_velocity;
	double length = get_axis_vector_length(target, mm.position);

	// trap error conditions
	if (minutes < EPSILON) { return (TG_ZERO_LENGTH_MOVE);}
	if (length < MIN_LINE_LENGTH) { return (TG_ZERO_LENGTH_MOVE);}

	// get a cleared buffer and setup move variables
	if ((bf = _mp_get_write_buffer()) == NULL) {// get buffer or die trying
		INFO(PSTR("Failed to get buffer in mp_aline()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	if (tg.linenum > EPSILON) bf->linenum = tg.linenum; else bf->linenum = tg.linecount;

	bf->time = minutes;
	bf->length = length;
	copy_axis_vector(bf->target, target); 	// set target for runtime
	set_unit_vector(bf->unit, bf->target, mm.position);

	// initialize jerk terms - these are needed shortly therafter
	bf->jerk = sqrt(square(bf->unit[X] * cfg.a[X].jerk) +
					square(bf->unit[Y] * cfg.a[Y].jerk) +
					square(bf->unit[Z] * cfg.a[Z].jerk) +
					square(bf->unit[A] * cfg.a[A].jerk) +
					square(bf->unit[B] * cfg.a[B].jerk) +
					square(bf->unit[C] * cfg.a[C].jerk));
	bf->recip_jerk = 1/bf->jerk;			// compute-once jerk terms
	bf->cubert_jerk = pow(bf->jerk, 0.3333333);

	// finish up the current block variables
	if (cm_get_path_control_mode() == PATH_EXACT_STOP) {	
		_mp_stop_replanning(bf); 	// you cannot replan an exact stop
		exact_stop = 0;
	} else {
		bf->replannable = TRUE;
		exact_stop = 12345678;		// an arbitrarily large number
	}
	bf->cruise_vmax = bf->length / minutes;	// target velocity requested
	junction_velocity = _mp_get_junction_vmax(bf->pv->unit, bf->unit);
	bf->entry_vmax = min3(bf->cruise_vmax, junction_velocity, exact_stop);
	bf->delta_vmax = _mp_get_target_velocity(0, bf->length, bf);
	bf->exit_vmax = min3(bf->cruise_vmax, (bf->entry_vmax + bf->delta_vmax), exact_stop);
	bf->braking_velocity = bf->delta_vmax;

	// backward planning pass. Finds beginning of the replanning chain and 
	// updates the braking velocities. At the end *bp points to the first 
	// buffer before the replanning chain.
	bp = bf;
	while ((bp = _mp_get_prev_buffer(bp)) != bf) {
		if (bp->replannable == FALSE) { break; }
		bp->braking_velocity = min(bp->nx->entry_vmax, bp->nx->braking_velocity) + bp->delta_vmax;
	}
	// forward planning pass - recomputes trapezoids in the chain.
	while ((bp = _mp_get_next_buffer(bp)) != bf) {
		bp->entry_velocity = bp->pv->exit_velocity;
		bp->cruise_velocity = bp->cruise_vmax;
		bp->exit_velocity = min4(bp->exit_vmax, bp->nx->braking_velocity, bp->nx->entry_vmax,
								(bp->entry_velocity + bp->delta_vmax)); 
		_mp_calculate_trapezoid(bp);
		if ((bf->cruise_velocity - bf->entry_velocity) > PLANNER_VELOCITY_TOLERANCE) {
			INFO2(PSTR("aline() Ve=%f > Vc=%f"), bp->entry_velocity, bp->cruise_velocity);
		}
		// test for optimally planned trapezoids
		if (bp->exit_velocity == bp->exit_vmax) {	// only need to check the exit
			_mp_stop_replanning(bp);
		}
	}
	// finish up with the current move
	bf->entry_velocity = bf->pv->exit_velocity;
	bf->cruise_velocity = bf->cruise_vmax;
	bf->exit_velocity = 0;
	_mp_calculate_trapezoid(bf);
	copy_axis_vector(mm.position, bf->target);	// update planning position
	_mp_queue_write_buffer(MOVE_TYPE_ALINE);
	return (TG_OK);
}

/***** ALINE HELPERS *****
 * _mp_stop_replanning()
 * _mp_calculate_trapezoid()
 * _mp_get_target_length()
 * _mp_get_target_velocity()
 * _mp_get_junction_vmax()
 * _mp_get_corner_delta()
 */

/*	
 * _mp_stop_replanning() - a couple of things that you need to do to stop replanning
 */
static void _mp_stop_replanning(struct mpBuffer *bf)
{
	bf->replannable = FALSE;
	bf->exit_vmax = bf->exit_velocity;
}

/*
 * _mp_calculate_trapezoid()			- calculate trapezoid parameters
 * _mp_calculate_trapezoid_finalize()
 *
 *	This rather brute-force function sets section lengths and velocities based 
 *	on the line length and velocities requested. Target velocities are specified
 *	using bf->entry_velocity, bf->cruise_velocity, and bf->exit_velocity. 
 *	Target length is specified using bf->length. 
 *	Note: The following condition must be met on entry: Ve <= Vt >= Vx 
 *
 *	It modifies the buffer and returns accurate head_length, body_length and tail_length,
 *	and accurate or reasonably approximate velocities. We care about accuracy on lengths, 
 *	less so for velocity (as long as velocity err's on the side of too slow). 
 *	We need the velocities to be set even for zero-length sections so we can compute entry
 *	and exits for adjacent sections.
 *
 *	Various cases handled;
 *	  ZERO	zero sections - the line is too short to plan
 *	  HBT	Ve<Vt>Vx	3 section trapezoid
 *	  HB	Ve<Vt=Vx	head accelerates to cruise - exits at full speed
 *	  BT	Ve=Vt>Vx	enter at full speed and decelerate
 *	  HT	Ve & Vx		does not achieve cruise velocity
 *	  H		Ve<Vx		head accelerates to exit velocity (perfect fit)
 *	  H'	Ve<Vx'		line too short to meet Jm - Ve is met but Vx is degraded
 *	  B		Ve=Vt=Vx	entire line it run at Vt. No head or tail processed
 *	  T		Ve>Vx		tail decelerates to exit velocity (perfect fit)
 *	  T'	Ve'<Vx		line too short to meet Jm - Ve is degraded but Vx is met
 *
 *	The HT cases (2 section cases) deserve special attention There are 2 sub-cases: 
 *		- symmetric case where Ve = Vx,
 *		- asymmetic case where Ve != Vx. 
 *
 *	The order of the cases/tests in the code is pretty important
 */
static uint8_t _mp_calculate_trapezoid(struct mpBuffer *bf) 
{
	// initialize lengths
	bf->head_length = 0;
	bf->body_length = 0;
	bf->tail_length = 0;

	// ZERO case - the line is too short to plan
	if (bf->length < MIN_LINE_LENGTH) {
		INFO(PSTR("Zero-length line found in _mp_calculate_trapezoid()"));
		bf->length = 0;
		return (TG_ZERO_LENGTH_MOVE);
	}

	// B case - only has a body because all velocities are equal
	if (((bf->cruise_velocity - bf->entry_velocity) < PLANNER_VELOCITY_TOLERANCE) && 
		((bf->cruise_velocity - bf->exit_velocity) < PLANNER_VELOCITY_TOLERANCE)) {
		bf->body_length = bf->length;
		return (TG_OK);
	}

	// HBT case - trapezoid has a cruise region
	if ((bf->head_length = _mp_get_target_length(bf->entry_velocity, bf->cruise_velocity, bf)) < bf->length) { 
		 bf->tail_length = _mp_get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
		 bf->body_length = bf->length - bf->head_length - bf->tail_length;
		if (bf->body_length > EPSILON) {
			return (_mp_calculate_trapezoid_finalize(bf));
		}
	}

	// HT symmetric case - Ve=Vx. Vt is set accordingly. 
	// Velocity tolerance allows fitting around FP rounding errors
	if (fabs(bf->entry_velocity - bf->exit_velocity) < PLANNER_VELOCITY_TOLERANCE) {
		bf->body_length = 0;
		bf->head_length = bf->length/2;
		bf->tail_length = bf->head_length;
		bf->cruise_velocity = _mp_get_target_velocity(bf->entry_velocity, bf->head_length, bf);
		return (TG_OK);
	}

	// H' and T' degraded cases - these only happen if line is too short to fit the required accel/decel
	double minimum_length = _mp_get_target_length(bf->entry_velocity, bf->exit_velocity, bf);
	if (bf->length < (minimum_length - PLANNER_LENGTH_TOLERANCE)) {
		if (bf->entry_velocity < bf->exit_velocity)	{ // degrade entry velocity to meet line constraints
			bf->head_length = bf->length;
			bf->tail_length = 0;
			bf->exit_velocity = _mp_get_target_velocity(bf->entry_velocity, bf->length, bf);
		} else {							// degrade exit velocity to meet line constraints
			bf->head_length = 0;
			bf->tail_length = bf->length;
			bf->entry_velocity = _mp_get_target_velocity(bf->exit_velocity, bf->length, bf);
		}
		bf->body_length = 0;
		INFO4(PSTR("Degraded line in _mp_calculate_trapezoid() %f  %f  %f  %f"), bf->length, bf->entry_velocity, bf->cruise_velocity, bf->exit_velocity);
		return (TG_OK);
	}

	// H, T, HB and BT cases - Vt=Vx, or close enough. Vt is set to match exit velocity.
	if (bf->length < (minimum_length * PLANNER_LENGTH_FACTOR)) {
		if (bf->entry_velocity < bf->exit_velocity)	{ // make an acceleration section (head)
			bf->cruise_velocity = bf->exit_velocity;
			bf->head_length = _mp_get_target_length(bf->entry_velocity, bf->exit_velocity, bf);
			bf->body_length = bf->length - bf->head_length;
			bf->tail_length = 0;
		} else {							// make a deceleration section (tail)
			bf->cruise_velocity = bf->entry_velocity;
			bf->tail_length = _mp_get_target_length(bf->entry_velocity, bf->exit_velocity, bf);
			bf->body_length = bf->length - bf->tail_length;
			bf->head_length = 0;
		}
		return (_mp_calculate_trapezoid_finalize(bf));
	}

	// HT asymmetric case - this is relatively expensive but it's not called very often
	uint8_t i=0;
	double computed_velocity = bf->cruise_vmax;
	do {
		bf->cruise_velocity = computed_velocity;	// initialize from previous iteration 
		bf->head_length = _mp_get_target_length(bf->entry_velocity, bf->cruise_velocity, bf);
		bf->tail_length = _mp_get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
		if (bf->head_length > bf->tail_length) {
			bf->head_length = (bf->head_length / (bf->head_length + bf->tail_length)) * bf->length;
			computed_velocity = _mp_get_target_velocity(bf->entry_velocity, bf->head_length, bf);
		} else {
			bf->tail_length = (bf->tail_length / (bf->head_length + bf->tail_length)) * bf->length;
			computed_velocity = _mp_get_target_velocity(bf->exit_velocity, bf->tail_length, bf);
		}
		if (++i > PLANNER_ITERATION_MAX) { 
			INFO(PSTR("_mp_calculate_trapezoid() failed to converge")); 
		}
	} while ((fabs(bf->cruise_velocity - computed_velocity) / computed_velocity) > PLANNER_ITERATION_ERROR_PERCENT);
	bf->cruise_velocity = computed_velocity;
	bf->head_length = _mp_get_target_length(bf->entry_velocity, bf->cruise_velocity, bf);
	bf->tail_length = _mp_get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
	bf->body_length = 0;
	return (_mp_calculate_trapezoid_finalize(bf));
}

/*	Handle various edge cases where sectoins are less than min line lengths. 
 * 	Also ensure that lengths are accurate, as this is what we care about most.
 */
static uint8_t _mp_calculate_trapezoid_finalize(struct mpBuffer *bf)
{
	if (bf->head_length < MIN_LINE_LENGTH) {
		bf->head_length = 0;
		bf->body_length = bf->length - bf->tail_length;
	}
	if (bf->body_length < MIN_LINE_LENGTH) {
		bf->body_length = 0;
		bf->tail_length = bf->length - bf->head_length;
	}
	if (bf->tail_length < MIN_LINE_LENGTH) {
		bf->tail_length = 0;
		if (bf->head_length > bf->body_length) {
			bf->head_length = bf->length - bf->body_length;	
		} else {
			bf->body_length = bf->length - bf->head_length;	
		}
	}
	return(TG_OK);
}

/*	
 * _mp_get_target_length()
 * _mp_get_target_velocity()
 *
 *	This pair of functions returns the fourth thing knowing the other three.
 *	
 *	_mp_get_target_length() is a convenient expression for determining 
 *	the optimal_length (L) of a line given the inital velocity (Vi), 
 *	target velocity (Vt) and maximum jerk (Jm).
 *
 *	The length (position) equation is derived from: 
 *
 *	 a)	L = (Vt-Vi) * T - (Ar*T^2)/2	... which becomes b) with substitutions for Ar and T
 *	 b) L = (Vt-Vi) * 2*sqrt((Vt-Vi)/Jm) - (2*sqrt((Vt-Vi)/Jm) * (Vt-Vi))/2
 *	 c)	L = (Vt-Vi)^(3/2) / sqrt(Jm)	...is an alternate form of b) (see Wolfram Alpha)
 *	 c')L = (Vt-Vi) * sqrt((Vt-Vi)/Jm) ... second alternate form; requires Vt >= Vi
 *
 *	 Notes: Ar = (Jm*T)/4					Ar is ramp acceleration
 *			T  = 2*sqrt((Vt-Vi)/Jm)			T is time
 *			Assumes Vt, Vi and L are positive or zero
 *			Cannot assume Vt>=Vi due to rounding errors and use of PLANNER_VELOCITY_TOLERANCE
 *			  necessitating the introduction of fabs()
 *
 * 	_mp_get_target_velocity() is a convenient expression for determining target 
 *	velocity for a given the initial velocity (Vi), length (L), and maximum jerk (Jm).
 *	Equation d) is b) solved for Vt. Equation e) is c) solved for Vt. Use e) (obviously)
 *
 *	 d)	Vt = (sqrt(L)*(L/sqrt(1/Jm))^(1/6)+(1/Jm)^(1/4)*Vi)/(1/Jm)^(1/4)
 *	 e)	Vt = L^(2/3) * Jm^(1/3) + Vi
 */
static double _mp_get_target_length(const double Vi, const double Vt, const struct mpBuffer *bf)
{
	return (fabs(Vi-Vt) * sqrt(fabs(Vi-Vt) * bf->recip_jerk));
}

static double _mp_get_target_velocity(const double Vi, const double L, const struct mpBuffer *bf)
{
	return (pow(L, 0.66666666) * bf->cubert_jerk + Vi);
}

/*
 * _mp_get_junction_vmax() - Chamnit's algorithm - simple
 *
 *  Computes the maximum allowable junction speed by finding the velocity
 *	that will yield the centripetal acceleration in the corner_acceleration 
 *	value. The value of delta sets the effective radius of curvature.
 *	Here's Chamnit's (Sungeun K. Jeon's) explanation of what's going on:
 *
 *	"First let's assume that at a junction we only look a centripetal 
 *	acceleration to simply things. At a junction of two lines, let's place 
 *	a circle such that both lines are tangent to the circle. The circular 
 *	segment joining the lines represents the path for constant centripetal 
 *	acceleration. This creates a deviation from the path (let's call this 
 *	delta), which is the distance from the junction to the edge of the 
 *	circular segment. Delta needs to be defined, so let's replace the 
 *	term max_jerk with max_junction_deviation( or delta). This indirectly 
 *	sets the radius of the circle, and hence limits the velocity by the 
 *	centripetal acceleration. Think of the this as widening the race track.
 *	If a race car is driving on a track only as wide as a car, it'll have 
 *	to slow down a lot to turn corners. If we widen the track a bit, the 
 *	car can start to use the track to go into the turn. The wider it is, 
 *	the faster through the corner it can go.
 *
 *	If you do the geometry in terms of the known variables, you get:
 *	sin(theta/2) = R/(R+delta)  Re-arranging in terms of circle radius (R)
 *	R = delta*sin(theta/2)/(1-sin(theta/2). Theta is the angle between 
 *	line segments given by: cos(theta) = dot(a,b)/(norm(a)*norm(b)). 
 *	Most of these calculations are already done in the planner. To remove 
 *	the acos() and sin() computations, use the trig half angle identity: 
 *	sin(theta/2) = +/- sqrt((1-cos(theta))/2). For our applications, this 
 *	should always be positive. Now just plug and chug the equations into 
 *	the centripetal acceleration equation: v_c = sqrt(a_max*R). You'll see
 *	that there are only two sqrt computations and no sine/cosines."
 *
 *	How to compute the radius using brute-force trig:
 *	double theta = acos(costheta);
 *	double radius = delta * sin(theta/2)/(1-sin(theta/2));
 */

static double _mp_get_junction_vmax(const double a_unit[], const double b_unit[])
{
	double costheta = - a_unit[X] * b_unit[X]
					  - a_unit[Y] * b_unit[Y]
					  - a_unit[Z] * b_unit[Z]
					  - a_unit[A] * b_unit[A]
					  - a_unit[B] * b_unit[B]
					  - a_unit[C] * b_unit[C];

	if (costheta < -0.99) { return (10000000); } 		// straight line cases
	if (costheta > 0.99)  { return (0); } 				// reversal cases
	double delta = _mp_get_corner_delta(a_unit, b_unit);// with axis compensation
//	double delta = cfg.a[X].corner_delta;				// without axis compensation
	double sintheta_over2 = sqrt((1 - costheta)/2);
	double radius = delta * sintheta_over2 / (1-sintheta_over2);
	return(sqrt(radius * cfg.jerk_corner_acceleration));
}

/*	
 * _mp_get_corner_delta() - Compute delta for Chamnit's algorithm (Sonny J)
 *
 *  This helper function extends Chamnit's algorithm by computing a value
 *	for delta that takes the contributions of the individual axes in the 
 *	move into account. It allows the radius of curvature to vary by axis.
 *	This is necessary to support axes that have different dynamics; such 
 *	as a Z axis that doesn't move as fast as X and Y (such as a screw driven 
 *	Z axis on machine with a belt driven XY - like a makerbot), or rotary 
 *	axes ABC that have completely different dynamics than their linear 
 *	counterparts.
 *
 *	The function takes the absolute values of the sum of the unit vector
 *	components as a measure of contribution to the move, then scales the 
 *	delta values from the non-zero axes into a composite delta to be used
 *	for the move. Shown for an XY vector:
 *
 *	 U[i]	Unit sum of i'th axis	fabs(unit_a[i]) + fabs(unit_b[i])
 *	 Usum	Length of sums			Ux + Uy
 *	 d		Delta of sums			(Dx*Ux+DY*UY)/Usum
 */

static double _mp_get_corner_delta(const double a_unit[], const double b_unit[])
{
	double a_delta = 0;
	double b_delta = 0;

	for (uint8_t i=0; i<AXES; i++) {
		a_delta += square(a_unit[i] * cfg.a[i].corner_delta);
		b_delta += square(b_unit[i] * cfg.a[i].corner_delta);
	}
	double d = (sqrt(a_delta) + sqrt(b_delta))/2;
	return (d);
}

/*************************************************************************/
/**** ALINE EXEC ROUTINE *************************************************/
/*************************************************************************
 * ---> Everything here fires from LO interrupt and must be interrupt safe
 *
 *  _mp_exec_aline()			- acceleration line main routine
 *	_mp_exec_aline_head()		- helper for acceleration section
 *	_mp_exec_aline_body()		- helper for cruise section
 *	_mp_exec_aline_tail()		- helper for deceleration section
 *	_mp_exec_aline_segment()	- helper for running a segment
 *	_mp_exec_aline_finalize()	- helper for running a finalize segment
 *
 *	Returns:
 *	 TG_NOOP	no operation occurred
 *	 TG_AGAIN	move is not finished - continue iteration
 *	 TG_OK		move is done. Returning TG_OK makes caller free the bf buffer
 *	 TG_xxxxx	move finished with error. Free buffer
 *	
 *	This routine is called from the (LO) interrupt level. It must either 
 *	execute and prepare a single line segment or return TG_OK if done.
 *
 *	Aline generates jerk-controlled S-curves as per Ed Red's course notes:
 *	http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *	http://www.scribd.com/doc/63521608/Ed-Red-Ch5-537-Jerk-Equations
 *
 *	A full trapezoid is divided into 5 periods Periods 1 and 2 are the 
 *	first and second halves of the acceleration ramp (the concave and convex 
 *	parts of the S curve in the "head"). Periods 3 and 4 are the first 
 *	and second parts of the deceleration ramp (the tail). There is also 
 *	a period for the constant-velocity plateau of the trapezoid (the body).
 *	There are various degraded trapezoids possible, including 2 section 
 *	combinations (head and tail; head and body; body and tail), and single 
 *	sections - any one of the three.
 *
 *	The equations that govern the acceleration and deceleration ramps are:
 *
 *	  Period 1	  V = Vi + Jm*(T^2)/2
 *	  Period 2	  V = Vh + As*T - Jm*(T^2)/2
 *	  Period 3	  V = Vi - Jm*(T^2)/2
 *	  Period 4	  V = Vh + As*T + Jm*(T^2)/2
 *
 * 	These routines play some games with the acceleration and move timing 
 *	to make sure this actually all works out.
 *
 * --- State transitions - hierarchical state machine ---
 *
 *	bf->move_state transitions:
 *	 from _NEW to _RUN on first call (sub_state set to _OFF)
 *	 from _RUN to _OFF on final call
 * 	 or just remains _OFF
 *
 *	mr.move_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *	Within each section sub_state may be 
 *	 _NEW - trigger initialization
 *	 _RUN1 - run the first part
 *	 _RUN2 - run the second part
 *
 * Note 1: Returning TG_OK ends the move and frees the bf buffer. 
 *		   Returning TG_OK at this point does NOT advance position meaning any
 *		   position error will be compensated by the next move.
 *
 * Note 2: Solves a potential race condition where the current move ends but the 
 * 		   new move has not started because the previous move is still being run 
 *		   by the steppers. Planning can overwrite the new move.
 */
static uint8_t _mp_exec_aline(struct mpBuffer *bf)
{
	if (bf->move_state == MOVE_STATE_OFF) { return (TG_NOOP);} 
	if (bf->move_state == MOVE_STATE_NEW) {
		_mp_stop_replanning(bf);
		if (bf->length < MIN_LINE_LENGTH) { return (TG_OK);}  // (Note 1)
		mr.linenum = bf->linenum;
		mr.jerk_div2 = bf->jerk/2;
		mr.move_state = MOVE_STATE_HEAD;
		mr.sub_state = MOVE_STATE_NEW;
		bf->move_state = MOVE_STATE_RUN;
	} 
	if (bf->move_state == MOVE_STATE_RUN) {
		switch (mr.move_state) {
			case (MOVE_STATE_HEAD): { mr.status = _mp_exec_aline_head(bf); break;}
			case (MOVE_STATE_BODY): { mr.status = _mp_exec_aline_body(bf); break;}
			case (MOVE_STATE_TAIL): { mr.status = _mp_exec_aline_tail(bf); break;}
		}
		if (mr.status == TG_OK) {
			_mp_stop_replanning(_mp_get_next_buffer(bf)); // prevent overplanning (Note 2)
			bf->move_state = MOVE_STATE_OFF;
			mr.move_state = MOVE_STATE_OFF;
			mr.sub_state = MOVE_STATE_OFF;
			cm_force_status_report();					  // send final status report
		} else {
			cm_decr_status_report();					  // decr status report down counter
		}
	}
	return (mr.status);
}

/*
 * _mp_exec_aline_head()
 */
static uint8_t _mp_exec_aline_head(struct mpBuffer *bf) 
{
	if (mr.sub_state == MOVE_STATE_NEW) {
		if (bf->head_length < MIN_LINE_LENGTH) { 
			mr.move_state = MOVE_STATE_BODY;
			return(_mp_exec_aline_body(bf));		// skip ahead
		}
		mr.midpoint_velocity = (bf->entry_velocity + bf->cruise_velocity) / 2;
		mr.move_time = bf->head_length / mr.midpoint_velocity;	// time for entire accel
		mr.accel_time = sqrt((bf->cruise_velocity - bf->entry_velocity) / mr.jerk_div2);
		mr.midpoint_acceleration = 2 * (bf->cruise_velocity - bf->entry_velocity) / mr.accel_time;
		mr.segments = ceil(uSec(mr.move_time) / (2 * cfg.estd_segment_usec)); // number of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);
		mr.segment_accel_time = mr.accel_time / (2 * mr.segments);// time to advance for each segment
		mr.elapsed_accel_time = mr.segment_accel_time / 2; // elapsed time starting point (offset)
		mr.microseconds = uSec(mr.segment_move_time);
		mr.segment_count = (uint32_t)mr.segments;
		mr.sub_state = MOVE_STATE_RUN1;
	}
	if (mr.sub_state == MOVE_STATE_RUN1) {
		mr.segment_velocity = bf->entry_velocity + (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment(bf) == TG_COMPLETE) { 	  	// set up for second half
			mr.elapsed_accel_time = mr.segment_accel_time / 2;	// start time from midpoint of segment
			mr.segment_count = (uint32_t)mr.segments;
			mr.sub_state = MOVE_STATE_RUN2;
		}
		return(TG_EAGAIN);
	}
	if (mr.sub_state == MOVE_STATE_RUN2) {
		mr.segment_velocity = mr.midpoint_velocity + 
							 (mr.elapsed_accel_time * mr.midpoint_acceleration) -
							 (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment(bf) == TG_COMPLETE) {
			if ((bf->body_length < MIN_LINE_LENGTH) && 
				(bf->tail_length < MIN_LINE_LENGTH)) { // end the move
				return(TG_OK);
			}
			mr.move_state = MOVE_STATE_BODY;
			mr.sub_state = MOVE_STATE_NEW;
		}
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_body()
 */
static uint8_t _mp_exec_aline_body(struct mpBuffer *bf) 
{
	if (mr.sub_state == MOVE_STATE_NEW) {
		// look for various premature end conditions
		if (bf->body_length < MIN_LINE_LENGTH) {
			mr.move_state = MOVE_STATE_TAIL;
			return(_mp_exec_aline_tail(bf));	// skip ahead
		}
		mr.move_time = bf->body_length / bf->cruise_velocity;
		mr.segments = ceil(uSec(mr.move_time) / cfg.estd_segment_usec);
//		if ((uint16_t)mr.segments == 0) {		// no segments - shouldn't have got this far
//			mr.move_state = MOVE_STATE_TAIL;
//			return (_mp_exec_aline_tail(bf));	// skip ahead
//		}
		mr.segment_move_time = mr.move_time / mr.segments;
		mr.microseconds = uSec(mr.segment_move_time);
		mr.segment_count = (uint32_t)mr.segments;
		mr.segment_velocity = bf->cruise_velocity;
		mr.sub_state = MOVE_STATE_RUN;
	}
	if (mr.sub_state == MOVE_STATE_RUN) {
		if (_mp_exec_aline_segment(bf) == TG_COMPLETE) {
			if (bf->tail_length < MIN_LINE_LENGTH) { // end the move
				return(TG_OK);
			} 
			mr.move_state = MOVE_STATE_TAIL;
			mr.sub_state = MOVE_STATE_NEW;
		}
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_tail()
 */
static uint8_t _mp_exec_aline_tail(struct mpBuffer *bf) 
{
	if (mr.sub_state == MOVE_STATE_NEW) {
		if (bf->tail_length < MIN_LINE_LENGTH) { 
			return(TG_OK);						// end the move
		}
		mr.midpoint_velocity = (bf->cruise_velocity + bf->exit_velocity) / 2;
		mr.move_time = bf->tail_length / mr.midpoint_velocity;
		mr.accel_time = 2 * sqrt((bf->cruise_velocity - bf->exit_velocity) / bf->jerk);
		mr.midpoint_acceleration = 2 * (bf->cruise_velocity - bf->exit_velocity) / mr.accel_time;
		mr.segments = ceil(uSec(mr.move_time) / (2 * cfg.estd_segment_usec));// number of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);// time to advance for each segment
		mr.segment_accel_time = mr.accel_time / (2 * mr.segments);// time to advance for each segment
		mr.elapsed_accel_time = mr.segment_accel_time / 2; //compute time from midpoint of segment
		mr.microseconds = uSec(mr.segment_move_time);
		mr.segment_count = (uint32_t)mr.segments;
		mr.sub_state = MOVE_STATE_RUN1;
	}
	if (mr.sub_state == MOVE_STATE_RUN1) {
		mr.segment_velocity = bf->cruise_velocity - (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment(bf) == TG_COMPLETE) { 	  	// set up for second half
			mr.elapsed_accel_time = mr.segment_accel_time / 2;	// start time from midpoint of segment
			mr.segment_count = (uint32_t)mr.segments;
			mr.sub_state = MOVE_STATE_RUN2;
		}
		return(TG_EAGAIN);
	}
	if (mr.sub_state == MOVE_STATE_RUN2) {
		mr.segment_velocity = mr.midpoint_velocity - 
							 (mr.elapsed_accel_time * mr.midpoint_acceleration) +
							 (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment(bf) == TG_COMPLETE) {
			return (TG_OK);
		}
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_segment() - segment runner helper
 */
static uint8_t _mp_exec_aline_segment(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	// Multiply computed length by the unit vector to get the contribution for
	// each axis. Set the target in absolute coords and compute relative steps.
	for (i=0; i < AXES; i++) {
		mr.target[i] = mr.position[i] + (bf->unit[i] * mr.segment_velocity * mr.segment_move_time);
		travel[i] = mr.target[i] - mr.position[i];
	}
	// queue the line and adjust the variables for the next iteration
	(void)ik_kinematics(travel, steps, mr.microseconds);
	SEGMENT_LOGGER				// DEBUG statement
	if (st_prep_line(steps, mr.microseconds) == TG_OK) {
		copy_axis_vector(mr.position, mr.target); // update runtime position	
	}
	mr.elapsed_accel_time += mr.segment_accel_time; // ignored if running a cruise
	if (--mr.segment_count != 0) {
		return (TG_EAGAIN);			// this section still has more segments to run
	} else {
		return (TG_COMPLETE);		// this section has run all its segments
	}
}

/*
 * _mp_finalize_aline_segment() - last segment runner helper
 *
 *	Do the last segment specially to maintain position accuracy
 */
/*
static void _mp_finalize_aline_segment(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];
	
	mr.length = get_axis_vector_length(mr.position, mr.section_target);
	if ((mr.length < MIN_LINE_LENGTH) || (mr.endpoint_velocity < EPSILON)) { return;}
	mr.move_time = mr.length / mr.endpoint_velocity;
	mr.microseconds = uSec(mr.move_time);

	for (i=0; i < AXES; i++) {
		travel[i] = mr.section_target[i] - mr.position[i];
	}
	(void)ik_kinematics(travel, steps, mr.microseconds);
	SEGMENT_LOGGER	// DEBUG statement
	if (st_prep_line(steps, mr.microseconds) == TG_OK) {	
		copy_axis_vector(mr.position, mr.section_target);	// update runtime position
	}
	return;
}
*/

/****** DEBUG Code ******	(see beginning of file for static function prototypes) */

#ifdef __DEBUG
void mp_dump_running_plan_buffer() { _mp_dump_plan_buffer(mb.r);}
void mp_dump_plan_buffer_by_index(uint8_t index) { _mp_dump_plan_buffer(&mb.bf[index]);	}

static void _mp_dump_plan_buffer(struct mpBuffer *bf)
{
	fprintf_P(stderr, PSTR("***Runtime Buffer[%d] bstate:%d  mtype:%d  mstate:%d  replan:%d\n"),
			_mp_get_buffer_index(bf),
			bf->buffer_state,
			bf->move_type,
			bf->move_state,
			bf->replannable);

	print_scalar(PSTR("line number:     "), bf->linenum);
	print_vector(PSTR("position:        "), mm.position, AXES);
	print_vector(PSTR("target:          "), bf->target, AXES);
	print_vector(PSTR("unit:            "), bf->unit, AXES);
	print_scalar(PSTR("jerk:            "), bf->jerk);
	print_scalar(PSTR("time:            "), bf->time);
	print_scalar(PSTR("length:          "), bf->length);
	print_scalar(PSTR("head_length:     "), bf->head_length);
	print_scalar(PSTR("body_length:     "), bf->body_length);
	print_scalar(PSTR("tail_length:     "), bf->tail_length);
	print_scalar(PSTR("entry_velocity:  "), bf->entry_velocity);
	print_scalar(PSTR("cruise_velocity: "), bf->cruise_velocity);
	print_scalar(PSTR("exit_velocity:   "), bf->exit_velocity);
	print_scalar(PSTR("exit_vmax:       "), bf->exit_vmax);
	print_scalar(PSTR("entry_vmax:      "), bf->entry_vmax);
	print_scalar(PSTR("cruise_vmax:     "), bf->cruise_vmax);
	print_scalar(PSTR("delta_vmax:      "), bf->delta_vmax);
	print_scalar(PSTR("braking_velocity:"), bf->braking_velocity);
}

void mp_dump_runtime_state(void)
{
	fprintf_P(stderr, PSTR("***Runtime Singleton (mr)\n"));
	print_scalar(PSTR("line number:       "), mr.linenum);
	print_vector(PSTR("position:          "), mr.position, AXES);
	print_vector(PSTR("target:            "), mr.target, AXES);
	print_scalar(PSTR("length:            "), mr.length);

	print_scalar(PSTR("move_time:         "), mr.move_time);
	print_scalar(PSTR("accel_time;        "), mr.accel_time);
	print_scalar(PSTR("elapsed_accel_time:"), mr.elapsed_accel_time);
	print_scalar(PSTR("midpoint_velocity: "), mr.midpoint_velocity);
	print_scalar(PSTR("midpoint_accel:    "), mr.midpoint_acceleration);
	print_scalar(PSTR("jerk_div2:         "), mr.jerk_div2);

	print_scalar(PSTR("segments:          "), mr.segments);
	print_scalar(PSTR("segment_count:     "), mr.segment_count);
	print_scalar(PSTR("segment_move_time: "), mr.segment_move_time);
	print_scalar(PSTR("segment_accel_time:"), mr.segment_accel_time);
	print_scalar(PSTR("microseconds:      "), mr.microseconds);
	print_scalar(PSTR("segment_length:	  "), mr.segment_length);
	print_scalar(PSTR("segment_velocity:  "), mr.segment_velocity);
}
#endif

//####################################################################################
//############## UNIT TESTS ##########################################################
//####################################################################################

#define JERK_TEST_VALUE (double)50000000

#ifdef __UNIT_TESTS

void _mp_test_calculate_trapezoid(void);
void _mp_test_get_junction_vmax(void);
void _mp_make_unit_vector(double unit[], double x, double y, double z, 
						  double a, double b, double c);
void _mp_test_calculate_trapezoid(void);
void _mp_test_trapezoid(double entry_velocity, double cruise_velocity, double exit_velocity, double length, struct mpBuffer *bf);


void mp_unit_tests()
{
	_mp_test_calculate_trapezoid();
//	_mp_test_get_junction_vmax();
}

void _mp_test_trapezoid(double entry_velocity, double cruise_velocity, double exit_velocity, double length, struct mpBuffer *bf)
{
	bf->jerk = JERK_TEST_VALUE;
	bf->entry_vmax = entry_velocity;
	bf->cruise_vmax = cruise_velocity;
	bf->exit_vmax = exit_velocity;
	_mp_calculate_trapezoid(entry_velocity, cruise_velocity, exit_velocity, length, bf);
}

void _mp_test_calculate_trapezoid()
{
	struct mpBuffer *bf = _mp_get_write_buffer();

// these tests are calibrated the following parameters:
//	jerk_max 				50 000 000		(all axes)
//	jerk_corner_offset		   		 0.1	(all exes)
//	jerk_corner_acceleration   200 000		(global)

	_mp_test_trapezoid(0,	400, 	400,	0.8, bf);

// test cases drawn from braid_600mm					 		// expected results
//				   	   Ve  		Vt		Vx			L
	_mp_test_trapezoid(000.000,	600,	000.000,	0.327, bf); // Ve=0 	   	Vc=110.155
	_mp_test_trapezoid(000.000,	600,	174.538,	0.327, bf); // Ve=0, 	   	Vc=174.744	Vx=174.537
	_mp_test_trapezoid(174.873,	600,	173.867,	0.327, bf); // Ve=174.873	Vc=185.356	Vx=173.867
	_mp_test_trapezoid(173.593,	600,	000.000,	0.327, bf); // Ve=174.873	Vc=185.356	Vx=173.867
	_mp_test_trapezoid(347.082,	600,	173.214,	0.327, bf); // Ve=174.873	Vc=185.356	Vx=173.867


// ZERO section cases: line below minimum velocity or length
//				   	   Ve  	Vt		Vx		L
	_mp_test_trapezoid(0,	0.001,	0,		1.0, bf);
	_mp_test_trapezoid(0,	100,	0,		0.0, bf);

// 1 section cases: line shorter than minimum transition length cases
//				   	   Ve  	Vt		Vx		L
	_mp_test_trapezoid(400,	400, 	0, 		0.8, bf);
	_mp_test_trapezoid(600,	600, 	200,	0.8, bf);
	_mp_test_trapezoid(0,	400, 	400,	0.8, bf);
	_mp_test_trapezoid(200,	600, 	600,	0.8, bf);

// HBT - 3 section cases
//				   	   Ve  	Vt		Vx		L
	_mp_test_trapezoid(0,	190, 	0, 		0.8, bf);
	_mp_test_trapezoid(200,	400, 	0, 		2.0, bf);

// 2 section cases (HT)
//				   	   Ve  	Vt		Vx		L
	_mp_test_trapezoid(0,	200, 	0, 		0.8, bf);
	_mp_test_trapezoid(0,	400, 	0, 		0.8, bf);
	_mp_test_trapezoid(200,	400, 	0, 		0.8, bf);
	_mp_test_trapezoid(400,	400, 	0, 		2.0, bf);
	_mp_test_trapezoid(0,	400, 	200,	0.8, bf);

// 1 section cases (H,B and T)
//				   	   Ve  	Vt		Vx		L
	_mp_test_trapezoid(800,	800, 	800, 	1.0, bf);

	_mp_test_trapezoid(0,	400, 	0, 		0.8, bf);
	_mp_test_trapezoid(200,	400, 	0, 		0.8, bf);
	_mp_test_trapezoid(400,	400, 	0, 		2.0, bf);
	_mp_test_trapezoid(0,	400, 	200,	0.8, bf);
}


void _mp_make_unit_vector(double unit[], double x, double y, double z, 
					      double a, double b, double c)
{
	double length = sqrt(x*x + y*y + z*z + a*a + b*b + c*c);
	unit[X] = x/length;
	unit[Y] = y/length;
	unit[Z] = z/length;
	unit[A] = a/length;
	unit[B] = b/length;
	unit[C] = c/length;
}

void _mp_test_get_junction_vmax()
{
	// See "Chamnit's" tab in acceleration spreadsheet for description
	//	of the following cases. 

//	cfg.a[X].jerk_max = JERK_TEST_VALUE;
//	cfg.a[Y].jerk_max = JERK_TEST_VALUE;
//	cfg.a[Z].jerk_max = JERK_TEST_VALUE;
//	cfg.a[A].jerk_max = JERK_TEST_VALUE;
//	cfg.a[B].jerk_max = JERK_TEST_VALUE;
//	cfg.a[C].jerk_max = JERK_TEST_VALUE;
//	mm.jerk_transition_size = 0.5;
//	mm.jerk_limit_max = 184.2;

	mm.test_case = 1;				// straight line along X axis
	_mp_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 2;				// angled straight line
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.7071,	0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 3;				// 5 degree bend
	_mp_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.9962,	0.0872, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 4;				// 30 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.8660,	0.5000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 5;				// 45 degrees
	_mp_make_unit_vector(mm.a_unit, 0.8660,	0.5000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.2588,	0.9659, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 6;				// 60 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 7;				// 90 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.0000,	1.0000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 8;				// 90 degrees rotated 45 degrees
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.7071,	0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 9;				// 120 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 10;				// 150 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.8660,	0.5000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 11;				// 180 degrees
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.7071,-0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_junction_vmax(mm.a_unit, mm.b_unit);
}

#endif
