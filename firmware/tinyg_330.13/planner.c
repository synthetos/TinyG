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
#include "plan_arc.h"
#include "kinematics.h"
#include "stepper.h"
#include "report.h"
#include "util.h"
#include "xio/xio.h"			// supports trap and debug statements

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// bf->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_PENDING,			// marked as the next buffer to run
	MP_BUFFER_RUNNING			// current running buffer
};

struct mpBuffer {				// See Planning Velocity Notes for variable usage
	uint32_t linenum;			// line number; or block count if not numbered
	struct mpBuffer *pv;		// static pointer to previous buffer
	struct mpBuffer *nx;		// static pointer to next buffer
	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// move state machine sequence
	uint8_t replannable;		// TRUE if move can be replanned
	uint8_t hold_point;			// marks the first buffer after a feedhold

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
	double position[AXES];		// final move position for planning purposes
#ifdef __UNIT_TEST_PLANNER
	double test_case;
	double test_velocity;
	double a_unit[AXES];
	double b_unit[AXES];
#endif
};

struct mpMoveRuntimeSingleton {	// persistent runtime variables
	uint32_t linenum;			// line/block number of BF being processed
	uint8_t (*run_move)(struct mpBuffer *m); // currently running move
	uint8_t move_state;			// state of the overall move
	uint8_t section_state;		// state within a move section

	double position[AXES];		// final move position
	double target[AXES];		// target move position
	double unit[AXES];			// unit vector for axis scaling & planning

	double head_length;			// copies of bf variables of same name
	double body_length;
	double tail_length;
	double entry_velocity;
	double cruise_velocity;
	double exit_velocity;

	double length;				// length of line or helix in mm
	double move_time;			// total running time (derived)
	double accel_time;			// total pseudo-time for acceleration calculation
	double elapsed_accel_time;	// current running time for accel calculation
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double midpoint_acceleration;//acceleration at the midpoint
	double jerk;				// max linear jerk
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

// planning buffer routines
static void _mp_init_buffers(void);
static void _mp_unget_write_buffer(void);
static void _mp_clear_buffer(struct mpBuffer *bf); 
static void _mp_copy_buffer(struct mpBuffer *bf, const struct mpBuffer *bp);
static void _mp_queue_write_buffer(const uint8_t move_type);
static void _mp_free_run_buffer(void);
static struct mpBuffer * _mp_get_write_buffer(void); 
static struct mpBuffer * _mp_get_run_buffer(void);
static struct mpBuffer * _mp_get_first_buffer(void);
static struct mpBuffer * _mp_get_last_buffer(void);

//static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *bf);
//static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *bf);
#define _mp_get_prev_buffer(b) ((struct mpBuffer *)(b->pv))
#define _mp_get_next_buffer(b) ((struct mpBuffer *)(b->nx))

// aline planner routines / feedhold planning
static void _mp_plan_block_list(struct mpBuffer *bf);
static void _mp_calculate_trapezoid(struct mpBuffer *bf);
static void _mp_calculate_trapezoid_finalize(struct mpBuffer *bf);
static double _mp_get_target_length(const double Vi, const double Vt, const struct mpBuffer *bf);
static double _mp_get_target_velocity(const double Vi, const double L, const struct mpBuffer *bf);
static double _mp_get_junction_vmax(const double a_unit[], const double b_unit[]);
static double _mp_get_junction_deviation(const double a_unit[], const double b_unit[]);
static void _mp_reset_replannable_list(void);

// execute routines
static uint8_t _mp_exec_line(struct mpBuffer *bf);
static uint8_t _mp_exec_dwell(struct mpBuffer *bf);
static uint8_t _mp_exec_stop(struct mpBuffer *bf);
static uint8_t _mp_exec_end(struct mpBuffer *bf);
static uint8_t _mp_exec_aline(struct mpBuffer *bf);
static uint8_t _mp_exec_aline_head(void);
static uint8_t _mp_exec_aline_body(void);
static uint8_t _mp_exec_aline_tail(void);
static uint8_t _mp_exec_aline_segment(void);

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

/**** PLANNER BUFFER ROUTINES *********************************************
 *
 * mp_test_write_buffer()	Returns TRUE if a write buffer is available
 * _mp_init_buffers()		Initializes or resets buffers
 * _mp_get_write_buffer()	Get pointer to next available write buffer
 *							Returns pointer or NULL if no buffer available.
 *
 * _mp_unget_write_buffer() Free write buffer if you decide not to queue it.
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
 * _mp_request_finalize_run_buffer() Request that a finalize be run before 
 *							the next planning pass. This allows the exec routine
 *							to free the buffer w/o stomping on the main loop.
 *							
 * mp_invoke_finalize_run_buffer() Execute the finalize request. This is how 
 *							the main loop completes a request. 
 *
 * _mp_test_buffer_queue_empty() Returns TRUE if buffer queue is empty
 *
 * _mp_get_prev_buffer(bf)	Returns pointer to prev buffer in linked list
 * _mp_get_next_buffer(bf)	Returns pointer to next buffer in linked list 
 * _mp_get_first_buffer(bf)	Returns pointer to first buffer, i.e. the running block
 * _mp_get_last_buffer(bf)	Returns pointer to last buffer, i.e. last block (zero)
 * _mp_clear_buffer(bf)		Zeroes the contents of the buffer
 * _mp_copy_buffer(bf,bp)	Copies the contents of bp into bf - preserves links
 *
 * Notes:
 *	The write buffer pointer only moves forward on queue_write, and 
 *	the read buffer pointer only moves forward on finalize_read calls.
 *	(test, get and unget have no effect)
 */

uint8_t mp_test_write_buffer()
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) { return (TRUE); }
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

static void _mp_free_run_buffer()				// EMPTY current run buf & adv to next
{
	_mp_clear_buffer(mb.r);						// clear it out (& reset replannable)
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;								// advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {	// only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING; 	// pend next buffer
	}

	// do an auto-stop when the queue emptpies
	if (mb.w == mb.r) { cm_exec_stop(); }
}

static struct mpBuffer * _mp_get_first_buffer(void)
{
	return(_mp_get_run_buffer());	// returns buffer or NULL if nothing's running
}

static struct mpBuffer * _mp_get_last_buffer(void)
{
	struct mpBuffer *bf = _mp_get_run_buffer();
	struct mpBuffer *bp = bf;

	if (bf == NULL) { return(NULL);}

	do {
		if ((bp->nx->move_state == MOVE_STATE_OFF) || (bp->nx == bf)) { 
			return (bp); 
		}
	} while ((bp = _mp_get_next_buffer(bp)) != bf);
	return (bp);
}

// Use the macro instead
//static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *bf) { return (bf->pv);}
//static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *bf) { return (bf->nx);}

static void _mp_clear_buffer(struct mpBuffer *bf) 
{
	struct mpBuffer *nx = bf->nx;	// save pointers
	struct mpBuffer *pv = bf->pv;
	memset(bf, 0, sizeof(struct mpBuffer));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

static void _mp_copy_buffer(struct mpBuffer *bf, const struct mpBuffer *bp)
{
	struct mpBuffer *nx = bf->nx;	// save pointers
	struct mpBuffer *pv = bf->pv;
 	memcpy(bf, bp, sizeof(struct mpBuffer));
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
	if ((st_isbusy() == TRUE) || (mr.move_state > MOVE_STATE_NEW)) {
		return (TRUE);
	}
	return (FALSE);
}

/* 
 * mp_flush_planner() - flush all moves in the planner and all arcs
 *
 *	Does not affect the move currently running in mr.
 *	This function is designed to be called during a hold to reset the planner
 */

void mp_flush_planner()
{
	ar_abort_arc();
	_mp_init_buffers();
	(void)cm_exec_stop();
}

/*
 * mp_set_plan_position() 		- sets planning position (for G92)
 * mp_get_plan_position() 		- returns planning position
 * mp_set_axis_position() 		- sets both planning and runtime positions (for G2/G3)
 *
 * mp_get_current_position_vector()	- returns current position of queried axis
 * mp_get_current_position()	- returns current position of queried axis
 * mp_get_current_velocity()	- returns current velocity (aggregate)
 * mp_get_current_linenum()		- returns currently executing line number
 *
 * 	Keeping track of position is complicated by the fact that moves can
 *	require multiple reference frames. The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for planning
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

double *mp_get_current_position_vector(double vector[])
{
	copy_axis_vector(vector, mr.position);
	return (vector);
}

double mp_get_machine_position(uint8_t axis) { return (mr.position[axis]);}
double mp_get_work_position(uint8_t axis) { return (mr.position[axis]);}
double mp_get_current_velocity(void) { return (mr.segment_velocity);}
double mp_get_current_linenum(void) { return (mr.linenum);}

/*************************************************************************/
/* mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 */

uint8_t mp_exec_move() 
{
	struct mpBuffer *bf;

	// NULL means nothing's running
	if ((bf = _mp_get_run_buffer()) == NULL) { return (TG_NOOP);}

	// automatic cycle-start (transition to running state)
	if ((cm.machine_state == MACHINE_RESET) || (cm.machine_state == MACHINE_STOP)) { 
		cm_cycle_start();
	}

	// dispatch the move
	switch (bf->move_type) {
		case MOVE_TYPE_LINE: { return (_mp_exec_line(bf));}
		case MOVE_TYPE_ALINE: { return (_mp_exec_aline(bf));}
		case MOVE_TYPE_DWELL: { return (_mp_exec_dwell(bf));}
		case MOVE_TYPE_STOP: { return (_mp_exec_stop(bf));}
		case MOVE_TYPE_END: { return (_mp_exec_end(bf));}
	}
	// never supposed to get here
	INFO1(PSTR("Bad move type %d in mp_exec_move()"), bf->move_type);
	return (TG_UNRECOGNIZED_COMMAND);
}

/**** STOP & END *********************************************************
 * mp_queue_program_stop()
 * mp_queue_program_end()
 * _mp_run_stop()
 * _mp_run_end()
 */

void mp_queue_program_stop() 
{
	if (_mp_get_write_buffer() == NULL) {
		INFO(PSTR("Failed to get buffer in _mp_queue_program_stop()"));
		return;
	}
	_mp_queue_write_buffer(MOVE_TYPE_STOP);
}
static uint8_t _mp_exec_stop(struct mpBuffer *bf)
{
	_mp_free_run_buffer();
	return (cm_exec_stop());
}

void mp_queue_program_end()
{
	if (_mp_get_write_buffer() == NULL) {
		INFO(PSTR("Failed to get buffer in _mp_queue_program_end()"));
		return;
	}
	_mp_queue_write_buffer(MOVE_TYPE_END);
}
static uint8_t _mp_exec_end(struct mpBuffer *bf)
{
	_mp_free_run_buffer();
	return (cm_exec_end());
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
	st_prep_dwell((uint32_t)(bf->time * 1000000));	// convert seconds to uSec
	_mp_free_run_buffer();
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
	if (bf->length < EPSILON) {
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
	_mp_free_run_buffer();
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
	struct mpBuffer *bf; 						// current move pointer
	double exact_stop = 0;
	double junction_velocity;
	double length = get_axis_vector_length(target, mm.position);

	// trap error conditions
	if (minutes < EPSILON) { return (TG_ZERO_LENGTH_MOVE);}
	if (length < EPSILON) { return (TG_ZERO_LENGTH_MOVE);}

	// get a cleared buffer and setup move variables
	if ((bf = _mp_get_write_buffer()) == NULL) {// get buffer or die trying
		INFO(PSTR("Failed to get buffer in mp_aline()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	if (cm.linenum == 0) bf->linenum = cm.linecount; else bf->linenum = cm.linenum;

	bf->time = minutes;
	bf->length = length;
	copy_axis_vector(bf->target, target); 	// set target for runtime
	set_unit_vector(bf->unit, bf->target, mm.position);

	// initialize jerk terms (these are in sequence)
	bf->jerk = sqrt(square(bf->unit[X] * cfg.a[X].jerk_max) +
					square(bf->unit[Y] * cfg.a[Y].jerk_max) +
					square(bf->unit[Z] * cfg.a[Z].jerk_max) +
					square(bf->unit[A] * cfg.a[A].jerk_max) +
					square(bf->unit[B] * cfg.a[B].jerk_max) +
					square(bf->unit[C] * cfg.a[C].jerk_max));
	bf->recip_jerk = 1/bf->jerk;			  // compute-once term used in planning
	bf->cubert_jerk = pow(bf->jerk, 0.333333);// compute-once term used in planning

	// finish up the current block variables
	if (cm_get_path_control() != PATH_EXACT_STOP) { // exact stop cases already zeroed
		bf->replannable = TRUE;
		exact_stop = 12345678;					// an arbitrarily large number
	}
	bf->cruise_vmax = bf->length / bf->time;	// target velocity requested
	junction_velocity = _mp_get_junction_vmax(bf->pv->unit, bf->unit);
	bf->entry_vmax = min3(bf->cruise_vmax, junction_velocity, exact_stop);
	bf->delta_vmax = _mp_get_target_velocity(0, bf->length, bf);
	bf->exit_vmax = min3(bf->cruise_vmax, (bf->entry_vmax + bf->delta_vmax), exact_stop);
	bf->braking_velocity = bf->delta_vmax;

	// replan the block list and commit the current block
	_mp_plan_block_list(bf);
	copy_axis_vector(mm.position, bf->target);	// update planning position
	_mp_queue_write_buffer(MOVE_TYPE_ALINE);
	return (TG_OK);
}

/***** ALINE HELPERS *****
 * _mp_plan_block_list()
 * _mp_calculate_trapezoid()
 * _mp_get_target_length()
 * _mp_get_target_velocity()
 * _mp_get_junction_vmax()
 * _mp_get_junction_deviation()
 * _mp_reset_replannable_list()
 */

/*
 *	_mp_plan_block_list() - plans the entire block list
 *
 *	Plans all blocks between and including the first and block provided (bf).
 *	Sets entry, exit and cruise v's from vmax's then calls trapezoid generation. 
 *
 *	Variables that must be provided in the mpBuffers that will be processed:
 *
 *	  bf (function arg)		- end of block list (last block in time)
 *	  bf->replannable		- start of block list set by last FALSE value [Note 1]
 *	  bf->move_type			- Must be ALINE. Other mode types will fail.
 *								TODO: handle DWELL, START, STOP, END blocks in list
 *
 *	  bf->length			- provides block length
 *	  bf->entry_vmax		- used during forward planning to set entry velocity
 *	  bf->cruise_vmax		- used during forward planning to set cruise velocity
 *	  bf->exit_vmax			- used during forward planning to set exit velocity
 *	  bf->delta_vmax		- used during forward planning to set exit velocity
 *
 *	  bf->recip_jerk		- used during trapezoid generation
 *	  bf->cubert_jerk		- used during trapezoid generation
 *
 *	Variables that will be set during processing:
 *
 *	  bf->replannable		- set if the block becomes optimally planned
 *
 *	  bf->braking_velocity	- set during backward planning
 *	  bf->entry_velocity	- set during forward planning
 *	  bf->cruise_velocity	- set during forward planning
 *	  bf->exit_velocity		- set during forward planning
 *
 *	  bf->head_length		- set during trapezoid generation
 *	  bf->body_length		- set during trapezoid generation
 *	  bf->tail_length		- set during trapezoid generation
 *
 *	Variables that are ignored but here's what you would expect them to be:
 *	  bf->move_state		- NEW for all blocks but the earliest
 *	  bf->target[]			- block target position
 *	  bf->unit[]			- block unit vector
 *	  bf->time				- gets set later
 *	  bf->jerk				- source of the other jerk variables. Used in mr.
 *
 * Notes:
 *	[1]	Whether or not a block is planned is controlled by the bf->replannable 
 *		setting (set TRUE if it should be). Replan flags are checked during the 
 *		backwards pass and prune the replan list to include only the the latest 
 *		blocks that require planning.
 *
 *		In normal operation the first block (currently running vlock) is not 
 *		replanned, but may be for feedholds and feed overrides. In these cases 
 *		the prep routines modify the contents of the mr buffer and re-shuffle 
 *		the block list, re-enlisting the current bf buffer with new parameters.
 *		These routines also set all blocks in the list to be replannable so the 
 *		list can be recomputed regardless of exact stops and previous replanning 
 *		optimizations.
 */
void _mp_plan_block_list(struct mpBuffer *bf)
{
	struct mpBuffer *bp = bf;

	// Backward planning pass. Find beginning of the list and update the braking velocities.
	// At the end *bp points to the first buffer before the list.
	while ((bp = _mp_get_prev_buffer(bp)) != bf) {
		if (bp->replannable == FALSE) { break; }
		bp->braking_velocity = min(bp->nx->entry_vmax, bp->nx->braking_velocity) + bp->delta_vmax;
	}

	// forward planning pass - recomputes trapezoids in the list.
	while ((bp = _mp_get_next_buffer(bp)) != bf) {
		if (bp->pv == bf) {	
			bp->entry_velocity = bp->entry_vmax;		// first block in the list
		} else {
			bp->entry_velocity = bp->pv->exit_velocity;	// other blocks in the list
		}
		bp->cruise_velocity = bp->cruise_vmax;
		bp->exit_velocity = min4(bp->exit_vmax, bp->nx->braking_velocity, bp->nx->entry_vmax,
								(bp->entry_velocity + bp->delta_vmax));
		_mp_calculate_trapezoid(bp);
		// test for optimally planned trapezoids - only need to check the exit
		if (bp->exit_velocity == bp->exit_vmax) { bp->replannable = FALSE;}
	}
	// finish up the last block move
	bp->entry_velocity = bp->pv->exit_velocity;
	bp->cruise_velocity = bp->cruise_vmax;
	bp->exit_velocity = 0;
	_mp_calculate_trapezoid(bp);
}

/*
 * _mp_calculate_trapezoid() - calculate trapezoid parameters
 *
 *	This rather brute-force function sets section lengths and velocities based 
 *	on the line length and velocities requested. Target velocities are specified
 *	using bf->entry_velocity, bf->cruise_velocity, and bf->exit_velocity. 
 *	Target length is specified using bf->length. 
 *	Note: The following condition must be met on entry: Ve <= Vt >= Vx 
 *
 *	Modifies the buffer and returns accurate head_length, body_length and tail_length,
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
static void _mp_calculate_trapezoid(struct mpBuffer *bf) 
{
	// initialize lengths
	bf->head_length = 0;
	bf->body_length = 0;
	bf->tail_length = 0;

	// ZERO case - the line is too short to plan
	if (bf->length < EPSILON) {
		INFO(PSTR("Zero-length line found in _mp_calculate_trapezoid()"));
		bf->length = 0;
		return;
	}

	// B case - only has a body because all velocities are equal
	if (((bf->cruise_velocity - bf->entry_velocity) < PLANNER_VELOCITY_TOLERANCE) && 
		((bf->cruise_velocity - bf->exit_velocity) < PLANNER_VELOCITY_TOLERANCE)) {
		bf->body_length = bf->length;
		return;
	}

	// HBT case - trapezoid has a cruise region
	if ((bf->head_length = _mp_get_target_length(bf->entry_velocity, bf->cruise_velocity, bf)) < bf->length) { 
		 bf->tail_length = _mp_get_target_length(bf->exit_velocity, bf->cruise_velocity, bf);
		 bf->body_length = bf->length - bf->head_length - bf->tail_length;
		if (bf->body_length > EPSILON) {
			_mp_calculate_trapezoid_finalize(bf);
			return;
		}
	}

	// HT symmetric case - Ve=Vx. Vt is set accordingly. 
	// Velocity tolerance allows fitting around FP rounding errors
	if (fabs(bf->entry_velocity - bf->exit_velocity) < PLANNER_VELOCITY_TOLERANCE) {
		bf->body_length = 0;
		bf->head_length = bf->length/2;
		bf->tail_length = bf->head_length;
		bf->cruise_velocity = _mp_get_target_velocity(bf->entry_velocity, bf->head_length, bf);
		return;
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
		return;
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
		_mp_calculate_trapezoid_finalize(bf);
		return;
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
	_mp_calculate_trapezoid_finalize(bf);
	return;
}

/*	Handle various edge cases where sectoins are less than min line lengths. 
 * 	Also ensure that lengths are accurate, as this is what we care about most.
 */
static void _mp_calculate_trapezoid_finalize(struct mpBuffer *bf)
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
}

/*	
 * _mp_get_target_length()
 * _mp_get_target_velocity()
 *
 *	This pair of functions returns the fourth thing knowing the other three.
 *	
 *	_mp_get_target_length() is a convenient function for determining the 
 *	optimal_length (L) of a line given the inital velocity (Vi), 
 *	target velocity (Vt) and maximum jerk (Jm).
 *
 *	The length (distance) equation is derived from: 
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
 * 	_mp_get_target_velocity() is a convenient function for determining Vt target 
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
	double delta = _mp_get_junction_deviation(a_unit, b_unit);// with axis compensation
//	double delta = cfg.a[X].junction_deviation;				// without axis compensation
	double sintheta_over2 = sqrt((1 - costheta)/2);
	double radius = delta * sintheta_over2 / (1-sintheta_over2);
	return(sqrt(radius * cfg.corner_acceleration));
}

/*	
 * _mp_get_junction_deviation() - Compute delta for Chamnit's algorithm (Sonny J)
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

static double _mp_get_junction_deviation(const double a_unit[], const double b_unit[])
{
	double a_delta = 0;
	double b_delta = 0;

	for (uint8_t i=0; i<AXES; i++) {
		a_delta += square(a_unit[i] * cfg.a[i].junction_dev);
		b_delta += square(b_unit[i] * cfg.a[i].junction_dev);
	}
	double d = (sqrt(a_delta) + sqrt(b_delta))/2;
	return (d);
}

/*
 *	_mp_reset_replannable_list() - resets all blocks in the planning list to be replannable
 */	
void _mp_reset_replannable_list()
{
	struct mpBuffer *bf = _mp_get_first_buffer();
	if (bf == NULL) { return;}
	struct mpBuffer *bp = bf;
	do {
		bp->replannable = TRUE;
	} while (((bp = _mp_get_next_buffer(bp)) != bf) && (bp->move_state != MOVE_STATE_OFF));
}

/*************************************************************************
 * feedholds - functions for performing holds
 *
 * mp_plan_hold()	 - replan block list to execute hold
 * mp_release_hold() - remove the hold and restart block list
 *
 *	Feedhold is executed as cm.hold_state transitions executed as aline exec 
 *	post-processing and main loop callbacks to mp_plan_hold() and mp_end_hold().
 */
/*	Holds run as follows:
 * 
 * 	- Hold is asserted by calling mp_start_hold()
 *		If hold_state is OFF and machine_state is RUNning it sets hold_state
 *		to SYNC.
 *
 *	- Hold state == SYNC tells the aline exec routine to execute the next aline 
 *		segment then set hold_state to PLAN. This gives the planner sufficient 
 *		time to replan the block list for the hold before the next aline 
 *		segment needs to be processed.
 *
 *	- Hold state == PLAN tells the planner to replan the mr buffer, the current
 *		run buffer (bf), and subsequent bf buffers as necessary to execute a 
 *		hold. Hold planning plans down to zero and then back up from zero. The 
 *		buffer that releases the hold (i.e. the buffer following the one that 
 *		decelerates to zero) is marked as the hold_point. Hold state is set to  
 *		DECEL when planning is complete.
 *
 *	- Hold state == DECEL persists until the aline execution gets to the hold
 *		point (decel to zero), at which point hold state transitions to HOLD
 *		and machine_state is set to HOLD.
 *
 *	- Hold state == HOLD persists until the cycle is restarted, which is when
 *		cm_cycle_start() is called and machine_state transitions to END_HOLD.
 *		Typically cycle start will be re-asserted after motion stops, but it 
 *		may have been re-asserted during any phase of feedhold execution.
 */

/*
 * mp_plan_hold()
 */

uint8_t mp_plan_hold()
{
	struct mpBuffer *bf; 	// first bf (the one associated with mr buffer)
	struct mpBuffer *bp; 	// working pointer

	if (cm.hold_state != FEEDHOLD_PLAN) { return (TG_NOOP);}
	if ((bf = _mp_get_run_buffer()) == NULL) { return (TG_NOOP);} // nothing's running
	bp = bf;

	// examine and process mr buffer
	double braking_velocity = mr.segment_velocity;			// velocity to shed
	double braking_length = _mp_get_target_length(braking_velocity, 0, bp);
	double remaining_length = get_axis_vector_length(bf->target, mr.position);

	// Case 1: feedhold deceleration fits in remaining distance in mr buffer
	if (braking_length < remaining_length) {
		// replan mr to zero exit velocity
		mr.move_state = MOVE_STATE_TAIL;		
		mr.section_state = MOVE_STATE_NEW;
		mr.tail_length = braking_length;
		mr.cruise_velocity = braking_velocity;
		mr.exit_velocity = 0;

		// replan the current bf to be the feed release point
		bp->length = remaining_length - braking_length;
		bp->delta_vmax = _mp_get_target_velocity(0, bp->length, bp); // recompute vmax
		bp->entry_vmax = 0;
		bp->move_state = MOVE_STATE_NEW;
		bp->hold_point = true;
		_mp_reset_replannable_list();
		_mp_plan_block_list(_mp_get_last_buffer());

	} else { // Case 2: feedhold deceleration exceeds remaining distance in mr buffer
		// replan mr to minimum (but non-zero) exit velocity
		mr.move_state = MOVE_STATE_TAIL;		
		mr.section_state = MOVE_STATE_NEW;
		mr.tail_length = remaining_length;
		mr.cruise_velocity = braking_velocity;
		mr.exit_velocity = braking_velocity - _mp_get_target_velocity(0, remaining_length, bp);	

		// replan next buffer into current bf (while advancing bp)
		braking_velocity = mr.exit_velocity;

		do {
			_mp_copy_buffer(bp, bp->nx);
			braking_length = _mp_get_target_length(braking_velocity, 0, bp);
			remaining_length = bp->length - braking_length;
			bp->entry_vmax = braking_velocity;
			if (braking_length > bp->length) {	// feedhold decel does not fit in bp buffer
				bp->exit_vmax = braking_velocity - _mp_get_target_velocity(0, bp->length, bp);	
				braking_velocity = bp->exit_vmax;
				bp = _mp_get_next_buffer(bp);
			} else {	// feedhold deceleration fits in the bp buffer
				bp->length = braking_length;
				bp->exit_vmax = 0;
				bp = _mp_get_next_buffer(bp);
				break;
			}
		} while (bp != bf);				// cutout if for some reason it wraps

		// setup the feed release point and replan the list
		bp->entry_vmax = 0;
		bp->length = remaining_length - braking_length;
		bp->delta_vmax = _mp_get_target_velocity(0, bp->length, bp);
		bp->hold_point = true;
		_mp_reset_replannable_list();
		_mp_plan_block_list(_mp_get_last_buffer());
	}
	cm.hold_state = FEEDHOLD_DECEL;		// set state to decelerate and exit
	return (TG_OK);
}

/* 
 * mp_end_hold() - end a feedhold
 *
 * 	This function is a callback that is called from the controller. To end a 
 *	hold do not call mp_end_feedhold() directly, instead call cm_cycle_start().
 */

uint8_t mp_end_hold()
{
	struct mpBuffer *bf;

	if (cm.machine_state != MACHINE_END_HOLD) { return (TG_NOOP); }
	cm.hold_state = FEEDHOLD_OFF;
	if ((bf = _mp_get_run_buffer()) == NULL) {	// NULL means nothing's running
//		INFO(PSTR("End Feedhold - STOPPED"));	// +++++ test line
		cm_exec_stop();
		return (TG_NOOP);
	}
//	INFO(PSTR("End Feedhold - RESUMED"));		// +++++ test line
	cm.machine_state = MACHINE_RUN;
	bf->hold_point = false;						// allows the move to be executed
	st_request_exec_move();						// restart the steppers
	return (TG_OK);
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
 *
 *	Returns:
 *	 TG_OK		move is done
 *	 TG_EAGAIN	move is not finished - has more segments to run
 *	 TG_NOOP	cause no operation from the steppers - do not load the move
 *	 TG_xxxxx	fatal error. Ends the move and frees the bf buffer
 *	
 *	This routine is called from the (LO) interrupt level. The interrupt 
 *	sequencing relies on the behaviors of the routines being exactly correct.
 *	Each call to _mp_exec_aline() must execute and prep *one and only one* 
 *	segment. If the segment is the not the last segment in the bf buffer the 
 *	_aline() must return TG_EAGAIN. If it's the last segment it must return 
 *	TG_OK. If it encounters a fatal error that would terminate the move it 
 *	should return a valid error code. Failure to obey this will introduce 
 *	subtle and very difficult to diagnose bugs (trust me on this).
 *
 *	Note 1 Returning TG_OK ends the move and frees the bf buffer. 
 *		   Returning TG_OK at this point does NOT advance position meaning any
 *		   position error will be compensated by the next move.
 *
 *	Note 2 Solves a potential race condition where the current move ends but the 
 * 		   new move has not started because the previous move is still being run 
 *		   by the steppers. Planning can overwrite the new move.
 */
/* OPERATION:
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
 */
/* --- State transitions - hierarchical state machine ---
 *
 *	bf->move_state transitions:
 *	 from _NEW to _RUN on first call (sub_state set to _OFF)
 *	 from _RUN to _OFF on final call
 * 	 or just remains _OFF
 *
 *	mr.move_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *	Within each section state may be 
 *	 _NEW - trigger initialization
 *	 _RUN1 - run the first part
 *	 _RUN2 - run the second part 
 */
static uint8_t _mp_exec_aline(struct mpBuffer *bf)
{
	uint8_t status;

	if (bf->move_state == MOVE_STATE_OFF) { 
		return (TG_NOOP);
	} 
	if (mr.move_state == MOVE_STATE_OFF) {
		if (bf->hold_point == true) { 		// implements the feedhold
			return (TG_NOOP);
		}

		// initialization to process the new incoming bf buffer
		bf->replannable = FALSE;
		if (bf->length < EPSILON) { return (TG_NOOP);}
		bf->move_state = MOVE_STATE_RUN;
		mr.move_state = MOVE_STATE_HEAD;
		mr.section_state = MOVE_STATE_NEW;
		mr.linenum = bf->linenum;
		mr.jerk = bf->jerk;
		mr.jerk_div2 = bf->jerk/2;
		mr.head_length = bf->head_length;
		mr.body_length = bf->body_length;
		mr.tail_length = bf->tail_length;
		mr.entry_velocity = bf->entry_velocity;
		mr.cruise_velocity = bf->cruise_velocity;
		mr.exit_velocity = bf->exit_velocity;
		copy_axis_vector(mr.unit, bf->unit);
	}
	// from this point on the contents of the bf buffer do not affect execution

	//**** main dispatcher to process segments ***
	switch (mr.move_state) {
		case (MOVE_STATE_HEAD): { status = _mp_exec_aline_head(); break;}
		case (MOVE_STATE_BODY): { status = _mp_exec_aline_body(); break;}
		case (MOVE_STATE_TAIL): { status = _mp_exec_aline_tail(); break;}
	}

	// feed hold post-processing
	if (cm.hold_state == FEEDHOLD_SYNC) { 
		cm.hold_state = FEEDHOLD_PLAN;
	}
	if ((cm.hold_state == FEEDHOLD_DECEL) && (status == TG_OK) && (bf->hold_point == true)) {
		cm.machine_state = MACHINE_HOLD;
		cm.hold_state = FEEDHOLD_HOLD;			// we are now holding
	}

	// There are 3 things that can happen here depending on return conditions:
	//	  status	 bf->move_state	 Description
	//    ---------	 --------------	 ----------------------------------------
	//	  TG_EAGAIN	 <don't care>	 mr buffer has more segments to run
	//	  TG_OK		 MOVE_STATE_RUN	 mr and bf buffers are done
	//	  TG_OK		 MOVE_STATE_NEW	 mr done; bf must be run again (it's been reused)

	if (status == TG_EAGAIN) { 
		sr_decr_status_report(); 				// continue running mr buffer
	} else {
		mr.move_state = MOVE_STATE_OFF;			// reset mr buffer
		mr.section_state = MOVE_STATE_OFF;
		bf->nx->replannable = FALSE;			// prevent overplanning (Note 2)
		if (bf->move_state == MOVE_STATE_RUN) {
			_mp_free_run_buffer();				// free bf if it's actually done
		}
		sr_force_status_report();				// send final status report
	}
	return (status);
}

/*
 * _mp_exec_aline_head()
 */
static uint8_t _mp_exec_aline_head()
{
	if (mr.section_state == MOVE_STATE_NEW) {
		if (mr.head_length < EPSILON) { 
			mr.move_state = MOVE_STATE_BODY;
			return(_mp_exec_aline_body());			// skip ahead
		}
		mr.midpoint_velocity = (mr.entry_velocity + mr.cruise_velocity) / 2;
		mr.move_time = mr.head_length / mr.midpoint_velocity;	// time for entire accel
		mr.accel_time = 2 * sqrt((mr.cruise_velocity - mr.entry_velocity) / mr.jerk);
		mr.midpoint_acceleration = 2 * (mr.cruise_velocity - mr.entry_velocity) / mr.accel_time;
		mr.segments = ceil(uSec(mr.move_time) / (2 * cfg.estd_segment_usec)); // number of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);
		mr.segment_accel_time = mr.accel_time / (2 * mr.segments);// time to advance for each segment
		mr.elapsed_accel_time = mr.segment_accel_time / 2; // elapsed time starting point (offset)
		mr.segment_count = (uint32_t)mr.segments;
		mr.microseconds = uSec(mr.segment_move_time);
		mr.section_state = MOVE_STATE_RUN1;
	}
	if (mr.section_state == MOVE_STATE_RUN1) {
		mr.segment_velocity = mr.entry_velocity + (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment() == TG_COMPLETE) { 	  		// set up for second half
			mr.elapsed_accel_time = mr.segment_accel_time / 2;	// start time from midpoint of segment
			mr.segment_count = (uint32_t)mr.segments;
			mr.section_state = MOVE_STATE_RUN2;
		}
		return(TG_EAGAIN);
	}
	if (mr.section_state == MOVE_STATE_RUN2) {
		mr.segment_velocity = mr.midpoint_velocity + (mr.elapsed_accel_time * mr.midpoint_acceleration) -
							 (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment() == TG_COMPLETE) {
			if ((mr.body_length < MIN_LINE_LENGTH) && 
				(mr.tail_length < MIN_LINE_LENGTH)) { return(TG_OK);}	// end the move
			mr.move_state = MOVE_STATE_BODY;
			mr.section_state = MOVE_STATE_NEW;
		}
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_body()
 */
static uint8_t _mp_exec_aline_body() 
{
	if (mr.section_state == MOVE_STATE_NEW) {
		if (mr.body_length < EPSILON) {
			mr.move_state = MOVE_STATE_TAIL;
			return(_mp_exec_aline_tail());			// skip ahead
		}
		mr.move_time = mr.body_length / mr.cruise_velocity;
		mr.segments = ceil(uSec(mr.move_time) / cfg.estd_segment_usec);
		mr.segment_move_time = mr.move_time / mr.segments;
		mr.segment_velocity = mr.cruise_velocity;
		mr.segment_count = (uint32_t)mr.segments;
		mr.microseconds = uSec(mr.segment_move_time);
		mr.section_state = MOVE_STATE_RUN;
	}
	if (mr.section_state == MOVE_STATE_RUN) {
		if (_mp_exec_aline_segment() == TG_COMPLETE) {
			if (mr.tail_length < MIN_LINE_LENGTH) { return(TG_OK);}	// end the move
			mr.move_state = MOVE_STATE_TAIL;
			mr.section_state = MOVE_STATE_NEW;
		}
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_tail()
 */
static uint8_t _mp_exec_aline_tail() 
{
	if (mr.section_state == MOVE_STATE_NEW) {
		if (mr.tail_length < EPSILON) { return(TG_OK);}		// end the move
		mr.midpoint_velocity = (mr.cruise_velocity + mr.exit_velocity) / 2;
		mr.move_time = mr.tail_length / mr.midpoint_velocity;
		mr.accel_time = 2 * sqrt((mr.cruise_velocity - mr.exit_velocity) / mr.jerk);
		mr.midpoint_acceleration = 2 * (mr.cruise_velocity - mr.exit_velocity) / mr.accel_time;
		mr.segments = ceil(uSec(mr.move_time) / (2 * cfg.estd_segment_usec));// number of segments in *each half*
		mr.segment_move_time = mr.move_time / (2 * mr.segments);// time to advance for each segment
		mr.segment_accel_time = mr.accel_time / (2 * mr.segments);// time to advance for each segment
		mr.elapsed_accel_time = mr.segment_accel_time / 2; //compute time from midpoint of segment
		mr.segment_count = (uint32_t)mr.segments;
		mr.microseconds = uSec(mr.segment_move_time);
		mr.section_state = MOVE_STATE_RUN1;
	}
	if (mr.section_state == MOVE_STATE_RUN1) {
		mr.segment_velocity = mr.cruise_velocity - (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment() == TG_COMPLETE) { 	  		// set up for second half
			mr.elapsed_accel_time = mr.segment_accel_time / 2;	// start time from midpoint of segment
			mr.segment_count = (uint32_t)mr.segments;
			mr.section_state = MOVE_STATE_RUN2;
		}
		return(TG_EAGAIN);
	}
	if (mr.section_state == MOVE_STATE_RUN2) {
		mr.segment_velocity = mr.midpoint_velocity - 
							 (mr.elapsed_accel_time * mr.midpoint_acceleration) +
							 (square(mr.elapsed_accel_time) * mr.jerk_div2);
		if (_mp_exec_aline_segment() == TG_COMPLETE) { return (TG_OK);}	// end the move
	}
	return(TG_EAGAIN);
}

/*
 * _mp_exec_aline_segment() - segment runner helper
 */
static uint8_t _mp_exec_aline_segment()
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	// Multiply computed length by the unit vector to get the contribution for
	// each axis. Set the target in absolute coords and compute relative steps.
	for (i=0; i < AXES; i++) {
		mr.target[i] = mr.position[i] + (mr.unit[i] * mr.segment_velocity * mr.segment_move_time);
		travel[i] = mr.target[i] - mr.position[i];
	}
	// prep the segment for the steppers and adjust the variables for the next iteration
	(void)ik_kinematics(travel, steps, mr.microseconds);
	SEGMENT_LOGGER				// conditional DEBUG statement
	if (st_prep_line(steps, mr.microseconds) == TG_OK) {
		copy_axis_vector(mr.position, mr.target); 	// update runtime position	
	}
	mr.elapsed_accel_time += mr.segment_accel_time; // NB: ignored if running the body
	if (--mr.segment_count == 0) {
		return (TG_COMPLETE);		// this section has run all its segments
	}
	return (TG_EAGAIN);			// this section still has more segments to run
}

/*
 * _mp_finalize_aline_segment() - last segment runner helper
 *
 *	Do the last segment specially to maintain position accuracy
 *
 *  TURNS OUT THIS IS NOT NECESSARY AS THE MATH IS ACCURATE ENOUGH WITHOUT IT
 */
/*
static void _mp_finalize_aline_segment()
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
	SEGMENT_LOGGER	// conditional DEBUG statement
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

#ifdef __UNIT_TEST_PLANNER

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
	_mp_calculate_trapezoid(bf);
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
