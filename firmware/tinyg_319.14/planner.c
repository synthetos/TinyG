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
 *	operation is complete (like queuing an arc).
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "xio.h"				// supports trap and debug statements
#include "tinyg.h"
#include "util.h"

#include "gcode.h"
#include "config.h"
#include "settings.h"
#include "planner.h"
#include "kinematics.h"
#include "motor_queue.h"
#include "canonical_machine.h"
#include "stepper.h"
#include "controller.h"

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// bf->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_PENDING,			// marked as the next buffer to run
	MP_BUFFER_RUNNING			// current running buffer
};

enum mpMoveType {				// bf->move_type values 
	MP_TYPE_NULL = 0,			// null move - nothing should be NULL
	MP_TYPE_LINE,				// simple line
	MP_TYPE_ALINE,				// acceleration planned line
	MP_TYPE_DWELL,				// delay with no movement
	MP_TYPE_START,				// restart motors
	MP_TYPE_STOP,				// stop motors
	MP_TYPE_ARC,				// arc feed
	MP_TYPE_END					// stop motors and end program
};

enum mpMoveState {				// bf->move_state values
	MP_STATE_NEW = 0,			// value on initial call (MUST BE ZERO)
	MP_STATE_ACCEL_1,			// acceleration segment 1
	MP_STATE_ACCEL_2,			// acceleration segment 2
	MP_STATE_CRUISE,			// cruise segment
	MP_STATE_DECEL_0,			// deceleration initialization
	MP_STATE_DECEL_1,			// deceleration segment 1
	MP_STATE_DECEL_2,			// deceleration segment 2
	MP_STATE_RUNNING,			// running (for non-acceleration moves) 
	MP_STATE_END				// force the move to end (kill)
};

struct mpBufferArc {			// arc variables for move/sub-move buffers
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)
};

struct mpBuffer {	// See Planning Velocity Notes for variable usage
	struct mpBuffer *nx;		// static pointer to next buffer
	struct mpBuffer *pv;		// static pointer to previous buffer

	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_state;			// move state machine sequence
	uint8_t replannable;		// TRUE if move can be replanned

	double target[AXES];		// target position in floating point
	double unit[AXES];			// unit vector for axis scaling & planning
	struct mpBufferArc a;		// arc variables

	double time;				// line, helix or dwell time in minutes
	double length;				// total length of line or helix in mm
	double head_length;
	double body_length;
	double tail_length;
								// *** SEE NOTES ON THESE VARIABLES, BELOW
	double entry_velocity;		// entry velocity requested for the move
	double cruise_velocity;		// cruise velocity requested & achieved
	double exit_velocity;		// exit velocity requested for the move
	double join_velocity_limit;	// max junction velocity at entry of this move
	double cruise_velocity_set;	// cruise velocity requested for move
	double exit_velocity_limit;	// max exit velocity possible (redundant)
	double difference_velocity;	// max velocity difference for this move
	double difference_to_stop;	// cumulative diff req'd to achieve full stop
	double difference_to_setV;	// cumulative diff req'd to achieve cruise velocity
};

/* Notes on all those planning velocities (mpBuffer variables)
 *
 *	entry_velocity, cruise_velocity and exit_velocity are the requested 
 *	velocities to the segment planner (_mp_get_segments()). These are 
 *	selected from a variety of choices so the move meets constraints.
 *	the segment planner may change the cruise_velocity, but will not 
 *	change the entry or exit velocities except in pathological cases.
 *
 *	join_velocity_limit is the maximum veleocity the move can be entered 
 *	based on path control mode (e.g. EXACT STOP) or the maximum allowable
 *	junction velocity from the previous move that meets jerk control. 
 *	This value is computed once and used repeatedly in backplanning.
 *
 *	cruise_velocity_set is the requested velocity of the move. It is 
 *	provided as an input to the move and is not changed.
 *
 *	exit_velocity_limit is the maximum velocity that the move can exit,
 *	without regard to any following move (i.e. it does not reflect the 
 *	junction velcoty limit of the next move). This is usually the same as 
 *	cruise_velocity_set and is probably redundant (it's aliased).
 *
 *	difference_velocity is the maximum change in velocity possible in 
 *	the move based on the length of the move and the max jerk. This is 
 *	applied in both the forward and backwards directions to compute the 
 *	difference_to_stop and difference_to_setV values.
 *
 *	difference_to_stop is the cumulative difference from the final exit
 *	velocity of zero (full braking) to the current head of the chain.
 *	It is recorded at the entry of each move.
 *
 *	difference_to_setV is the cumulative difference from the entry of the 
 *	first non-replannable move to the entry of the last move.
 */

struct mpBufferPool {			// ring buffer for sub-moves
	struct mpBuffer *w;			// get_write_buffer pointer
	struct mpBuffer *q;			// queue_write_buffer pointer
	struct mpBuffer *r;			// get/end_run_buffer pointer
	struct mpBuffer bf[MP_BUFFER_SIZE];// buffer storage
};

struct mpMoveMasterSingleton {	// common variables for planning (move master)
	double position[AXES];		// final move position 
	double jerk_max;			// jerk value to use for planning this move
	double jerk_max_cubert;		// cube root of jerk for planning
#ifdef __UNIT_TESTS
	double test_case;
	double test_velocity;
	double a_unit[AXES];
	double b_unit[AXES];
	double jerk_size;
	double jerk_limit_max;
#endif
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
	double jerk_max_div2;		// max linear jerk divided by 2

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_time;		// constant time per aline segment
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double segment_theta;		// angular motion per segment
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
};

static struct mpBufferPool mb;			// move buffer queue
static struct mpMoveMasterSingleton mm;	// static context for planning
static struct mpMoveRuntimeSingleton mr;// static context for runtime
//static struct mpMovePlanner mp[2];// current move and fwd or bkwd neighbor

/*
 * Local Scope Data and Functions
 */

// runtime dispatcher routines
static uint8_t _mp_run_aline(struct mpBuffer *bf);
static uint8_t _mp_run_line(struct mpBuffer *bf);
static uint8_t _mp_run_dwell(struct mpBuffer *bf);
static uint8_t _mp_run_arc(struct mpBuffer *bf);
static uint8_t _mp_run_stops(struct mpBuffer *bf);
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
static void _mp_backplan(struct mpBuffer *bf);
static uint8_t _mp_get_segments(double entry_velocity, 
								double cruise_velocity, 
								double exit_velocity, 
								double length, 
								struct mpBuffer *bf);
static uint8_t _mp_get_segments_head_cases(struct mpBuffer *bf);
static uint8_t _mp_get_segments_tail_cases(struct mpBuffer *bf);
static double _mp_get_optimal_length(const double V1, const double V2, const double Jm);
static double _mp_get_difference_velocity(const double Ve, const double L, const double Jm);
static double _mp_get_join_velocity(const double a_unit[], const double b_unit[], 
									const double a_velocity, const double b_velocity);
static double _mp_get_corner_delta(const double a_unit[], const double b_unit[]);

// aline runtime routines
static uint8_t _mp_run_accel_0(struct mpBuffer *bf);
static uint8_t _mp_run_accel_1(struct mpBuffer *bf);
static uint8_t _mp_run_accel_2(struct mpBuffer *bf);
static uint8_t _mp_run_cruise(struct mpBuffer *bf);
static uint8_t _mp_run_decel_0(struct mpBuffer *bf);
static uint8_t _mp_run_decel_1(struct mpBuffer *bf);
static uint8_t _mp_run_decel_2(struct mpBuffer *bf);
static uint8_t _mp_run_segment(struct mpBuffer *bf);
static void _mp_run_finalize(struct mpBuffer *bf);

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
	struct mpBuffer *bf;

	if ((bf = _mp_get_run_buffer()) == NULL) {// NULL means nothing's running
		return (TG_NOOP);
	}
	if (bf->move_state == MP_STATE_NEW) {	// first time in?
		mr.run_flag = TRUE;					// it's useful to have a flag
		switch (bf->move_type) { 			// setup the dispatch vector
			case MP_TYPE_LINE:	{ mr.run_move = _mp_run_line; break; }
			case MP_TYPE_ALINE:	{ mr.run_move = _mp_run_aline; break; }
			case MP_TYPE_DWELL:	{ mr.run_move = _mp_run_dwell; break; }
			case MP_TYPE_ARC:	{ mr.run_move = _mp_run_arc; break; }
			case MP_TYPE_START:	{ mr.run_move = _mp_run_stops; break; }
			case MP_TYPE_STOP:	{ mr.run_move = _mp_run_stops; break; }
			case MP_TYPE_END: 	{ mr.run_move = _mp_run_stops; break; }
		}
	}
	if ((status = mr.run_move(bf)) == TG_EAGAIN) { // run current run buf
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
	struct mpBuffer *bf;

	if ((b = _mp_get_run_buffer()) != NULL) {// NULL means nothing's running
		bf->move_state = MP_STATE_END;
		mr.run_flag = FALSE;
		_mp_finalize_run_buffer();
	}
}
*/

/**** PLANNER BUFFER ROUTINES *********************************************
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
 * _mp_get_prev_buffer(bf) Return pointer to prev buffer in linked list
 * _mp_get_next_buffer(bf) Return pointer to next buffer in linked list 
 * _mp_clear_buffer(bf)	   Zero the contents of the buffer
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
	mb.w = &mb.bf[0];			// init write and read buffer pointers
	mb.q = &mb.bf[0];
	mb.r = &mb.bf[0];
	pv = &mb.bf[MP_BUFFER_SIZE-1];
	for (i=0; i < MP_BUFFER_SIZE; i++) {  // setup ring pointers
		mb.bf[i].nx = &mb.bf[_mp_bump(i)];
		mb.bf[i].pv = pv;
		pv = &mb.bf[i];
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

static struct mpBuffer * _mp_get_write_buffer() // gets a cleared buffer
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
	return (NULL);				// condition: no queued buffers. fail it.
}

static void _mp_finalize_run_buffer()// EMPTY current run buf & adv to next
{
	_mp_clear_buffer(mb.r);			// clear it out (& reset replannable)
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;					// advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) { // only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING;   // pend next buffer
	}
}
/* UNUSED
static struct mpBuffer * _mp_get_prev_buffer_implicit()
{
	return (mb.w->pv);
}
*/
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

/**** VECTOR AND POSITION HELPERS ****************************************
 * mp_copy_vector()				- copy vector of arbitrary length
 * mp_set_unit_vector()			- populate a unit vector by pos. & target
 * mp_get_axis_vector_length()	- return the length of an axis vector
 * mp_set_axis_position()		- set MM & MR positions (supports G92)
 *
 * 	Keeping track of position is complicated by the fact that moves can
 *	have segments which require multiple reference frames. The scheme to 
 *	keep this straight is:
 *
 *	 - mm.position	- start and end position for trajectory planning
 *	 - mm.target	- target position for trajectory planning
 *	 - mr.position	- current position of segment (runtime endpoint)
 *	 - mr.target	- target position of segment (runtime final target)
 *	 - bf->target	- target position of segment (runtime working target)
 *					  also used to carry final target from mm to mr
 *
 *	Note that the positions are set immediately when they are computed and 
 *	are not an accurate representation of the tool position. In reality 
 *	the motors will still be processing the action and the real tool 
 *	position is still close to the starting point. 
 */

// copy vector
void mp_copy_vector(double dest[], const double src[], uint8_t length) 
{
	for (uint8_t i=0; i<length; i++) {
		dest[i] = src[i];
	}
}

// return the length of an axes vector
// should eventually take disabled axes and slave modes into account
double mp_get_axis_vector_length(const double a[], const double b[]) 
{
	double length = 0;
	for (uint8_t i=0; i<AXES; i++) {
		length += square(a[i] - b[i]);
	}
	return (sqrt(length));
}

// compute and set the values in a unit vector
void mp_set_unit_vector(double unit[], double target[], double position[])
{
	double length = mp_get_axis_vector_length(target, position);
	for (uint8_t i=0; i < AXES; i++) {
		unit[i] = (target[i] - position[i]) / length;
	}
}

// used by external callers such as G92
uint8_t mp_set_axis_position(const double position[])
{
	for (uint8_t i=0; i<AXES; i++) {
		mm.position[i] = position[i];
	}
	mp_copy_vector(mr.position, mm.position, AXES);
	return (TG_OK);
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

static uint8_t _mp_run_stops(struct mpBuffer *bf) 
{
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	(void)mq_queue_stops(bf->move_type);
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

	if ((bf = _mp_get_write_buffer()) == NULL) { // get write buffer or fail
		TRAP(PSTR("Failed to get buffer in mp_dwell()"));
		return (TG_BUFFER_FULL_FATAL);		   // (not supposed to fail)
	}
	bf->time = seconds;						   // in seconds, not minutes
	_mp_queue_write_buffer(MP_TYPE_DWELL);
	return (TG_OK);
}

static uint8_t _mp_run_dwell(struct mpBuffer *bf)
{
	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	(void)mq_queue_dwell((uint32_t)(bf->time * 1000000));	// convert seconds to uSec
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
		TRAP(PSTR("Failed to get buffer in mp_line()"));
		return (TG_BUFFER_FULL_FATAL);		// (not supposed to fail)
	}
	bf->time = minutes;
	mp_copy_vector(bf->target, target, AXES);// target to bf_target
	bf->length = mp_get_axis_vector_length(target, mr.position);
	if (bf->length < MIN_LINE_LENGTH) {
		_mp_unget_write_buffer();			// free buffer if early exit
		return (TG_ZERO_LENGTH_MOVE);
	}
	bf->cruise_velocity_set = bf->length / bf->time;// for yuks
	_mp_queue_write_buffer(MP_TYPE_LINE);
	mp_copy_vector(mm.position, bf->target, AXES);	// update planning position
	return(TG_OK);
}

static uint8_t _mp_run_line(struct mpBuffer *bf) 
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}

	for (i=0; i < AXES; i++) {
		travel[i] = bf->target[i] - mr.position[i];
	}
	mr.microseconds = uSec(bf->time);
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);
	mp_copy_vector(mr.position, bf->target, AXES);	// update runtime position
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
 *
 * This routine was originally sourced from the grbl project.
 */

uint8_t mp_arc( const double target[], 
				const double i, const double j, const double k, 
				const double theta, 		// starting angle
				const double radius, 		// radius of the circle in mm
				const double angular_travel, // radians along arc (+CW, -CCW)
				const double linear_travel, 
				const uint8_t axis_1, 	// select circle plane in tool space
				const uint8_t axis_2,  	// select circle plane in tool space
				const uint8_t axis_linear,// linear travel if helical motion
				const double minutes)		// time to complete the move
{
	struct mpBuffer *bf; 

	if ((bf = _mp_get_write_buffer()) == NULL) {// get write buffer or fail
		TRAP(PSTR("Failed to get buffer in mp_arc()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}

	// "move_length" is the total mm of travel of the helix (or just arc)
	bf->length = hypot(angular_travel * radius, fabs(linear_travel));	
	if (bf->length < cfg.min_segment_len) { // too short to draw
		_mp_unget_write_buffer();	// early exit requires you free buffer
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the move struct for an arc
	// note: bf->target is for debugging convenience and not actually used
	mp_copy_vector(bf->target, target, AXES);
	bf->time = minutes;
	bf->a.theta = theta;
	bf->a.radius = radius;
	bf->a.axis_1 = axis_1;
	bf->a.axis_2 = axis_2;
	bf->a.axis_linear = axis_linear;
	bf->a.angular_travel = angular_travel;
	bf->a.linear_travel = linear_travel;
	bf->entry_velocity = bf->length / bf->time;	// for trajectory planning
	bf->exit_velocity = bf->entry_velocity;	 	// for consistency

	// Compute unit vector
	// I think you can take the normal of the vector between the 
	// center point (i,j) and the target (x,y) and divide by the 
	// length of (i,j) to (x,y). Must also account for plane-axes
	// and the linear axis.
/*
	double length = sqrt(square(bf->target[axis_1] - i) +
						 square(bf->target[axis_2] - j) +
						 square(bf->target[axis_linear] - k));

	double offset[3] = {i, j, k};
	for (uint8_t i=0; i < 3; i++) {
		bf->unit[i] = (bf->target[i] - offset[i]) / length;
	}
*/
	mp_copy_vector(mm.position, bf->target, AXES);	// update planning position
	_mp_queue_write_buffer(MP_TYPE_ARC);
	return (TG_OK);
}

static uint8_t _mp_run_arc(struct mpBuffer *bf) 
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	if (mq_test_motor_buffer() == FALSE) { 
		return (TG_EAGAIN); 
	}
	// initialize arc variables
	if (bf->move_state == MP_STATE_NEW) {
		mr.segments = ceil(bf->length / cfg.min_segment_len);
		mr.segment_count = (uint32_t)mr.segments;
		mr.segment_theta = bf->a.angular_travel / mr.segments;
		mr.segment_length = bf->a.linear_travel / mr.segments;
 		mr.microseconds = uSec(bf->time / mr.segments);
		mr.center_1 = mr.position[bf->a.axis_1] - sin(bf->a.theta) * bf->a.radius;
		mr.center_2 = mr.position[bf->a.axis_2] - cos(bf->a.theta) * bf->a.radius;
		mr.target[bf->a.axis_linear] = mr.position[bf->a.axis_linear];
		bf->move_state = MP_STATE_RUNNING;
	}
	// compute an arc segment and exit
	if (bf->move_state == MP_STATE_RUNNING) {
		bf->a.theta += mr.segment_theta;
		mr.target[bf->a.axis_1] = mr.center_1 + sin(bf->a.theta) * bf->a.radius;
		mr.target[bf->a.axis_2] = mr.center_2 + cos(bf->a.theta) * bf->a.radius;
		mr.target[bf->a.axis_linear] += mr.segment_length;

		for (i=0; i < AXES; i++) {
			travel[i] = mr.target[i] - mr.position[i];
		}
		(void)ik_kinematics(travel, steps, mr.microseconds);
		(void)mq_queue_line(steps, mr.microseconds);
		mp_copy_vector(mr.position, mr.target, AXES);	// update runtime position
		if (--mr.segment_count > 0) {
			return (TG_EAGAIN);
		}
	}
	return (TG_OK);
}


/**************************************************************************
 * mp_aline() 		- queue line move with acceleration / deceleration
 * _mp_run_aline()	- run accel/decel move 
 *
 *	This module uses jerk motion equations to plan acceleration and 
 *	deceleration segments that obey maximum jerk parameters. The jerk is 
 *	the rate of change of acceleration (derivative), which is the third 
 *	derivative of position. Jerk is a measure of impact that a machine
 *	can take, and is a good way to limit the kinematics of a machine.
 *	Controlling jerk makes for smooth motion transitions between moves
 *	and allows for faster feeds while controlling machine oscillations
 *	and other undesirable side-effects. 
 *
 *	The S curve move is known as a 5 segment move, as the accel and decel 
 *	segments are divided in to concave and convex halves. A 5 segment move 
 *	has two acceleration segments, followed by a cruise segment, followed 
 *	by two deceleration segments. 
 *
 *	A 5 segment S curve move takes exactly the same time to execute as a 
 *	simpler constant acceleration trapezoidal move - it's just the endpoint
 *	transitions are smoother. The time lost in smoothing the endpoint 
 *	transitions is made up by a higher midpoint acceleration.
 *
 *	It is possible to achieve further time optimization by inserting a 
 *	constant acceleration segment in between the two accel/decel segments.
 *	(7 seg move) For more background and the motion equations see Ed Red's 
 *	BYU robotics course: http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf.
 */
/*	Aline() plans a linear move as 3 regions:
 *	  - head	acceleration to target velocity (2 acceleration segments)
 *	  - body	bulk of move at target speed 	(1 cruise segment)
 *	  - tail	deceleration to exit velocity 	(2 deceleration segments)
 *
 *	The initial velocity of the head is dependent on the path control 
 *	mode in effect and the transition jerk. Vi is always zero for
 *	EXACT STOP mode. For EXACT PATH and CONTINUOUS modes Vi is computed 
 *	based on the requested velocity and the magnitude of the linear and 
 *	tangential (cornering) jerk.
 *
 *	The body is the cruise segment where the line is running at its set 
 *	feed rate - or lower if this cannot be achieved. The tail of each line 
 *	is always intially planned to decelerate to zero. This may change to a 
 *	non-zero value as new lines are added and the move chain is re-planned.
 *
 *	As mentioned above, sufficient length is reserved in the tail to allow 
 *	deceleration from the cruise velocity to zero (braking). If the next 
 *	move has a non-zero entry velocity the previous moves are 
 *	recomputed (backplanned) to attain the maximum velocity while still 
 *	supporting braking to zero. 
 */
/*	Aline() is separated into a line planner routine and a runtime 
 *	execution routine that is executed as a continuation by 
 *	mp_move_dispatcher()
 *
 * Line planner:
 *
 *	The aline() trajectory planner main routine is called to compute and 
 *	queue a new line. It computes all initial parameters, examines the 
 *	transition cases, computes and queues the segments (trapezoid parts)
 *	as a move buffer.
 * 
 *	The tail segment is always pre-computed as an exact stop tail - 
 *	i.e. to decelerate to zero velocity in the event that no new line 
 *	arrives. If a following line arrives before the tail is executed the 
 *	moves prior to the new move are recomputed (backplanned) to blend with 
 *	the new line. In this way optimal velocities can be achieved while still 
 *	allowing for braking at the end of a chain of moves.
 */	
/*	The cases for joining lines to lines are:
 *
 *	  - CONTINUOUS MODE (G64) is the default mode. The moves will attempt 
 *		to run at their maximum requested speed, accelerating or 
 *		decelerating at way points (line junctions) to match speeds and 
 *		maintain maximum velocity. If the angle between two lines is too 
 *		sharp (tangential jerk is too high) the velocity at the join is 
 *		reduced to keep the jerk at or below maximum.
 *
 *	  - EXACT_STOP_MODE: (G61) the join speed is set to zero.
 */
/*  Segment Generation - The following cases exist for the segments of 
 *	the trapezoids:
 *
 *	  -	3 segment case: The line is long enough to support distinct head, 
 *		body and tail segments for the given entry, cruise and exit 
 *		velocities and the max jerk value.
 *
 *	  - 3 segment case with degraded head / tail: Thise case exists if
 *		the head and/or tail are less than the MIN_LINE_LENGTH
 *
 *	  - 2 segment cases: (a) No body - there is a head and tail, but no 
 *		body.(Marty Feldman case - "I ain't got no body"). Other cases are
 *		(b) no head case, and (c) no tail case.
 *
 *	  - 1 segment normal cases: Head only/ tail only -w/full accel or decel
 *	  
 *	  - 1 segment degraded case: Line is longer than MIN_LINE_LENGTH, but 
 *		the line is to short to sustain the entire accel or decel required.
 *
 *	  - 0 segment case: The line is too short to run
 */
/* Line Execution:
 *
 *	The aline continuation routine (run routine) executes the planned
 *	line. Head and tail acceleration / deceleration segments are run as a 
 *	set of constant-time very-short-line-segments that implement the 
 *	transition (stepwise-linear). The segment time constant is chosen 
 *	(~10 ms) to allow sufficiently fine accel/decel resolution and enough 
 *	steps to occur in a segment so that low velocity moves are not jerky. 
 */
/*
 * Notes:
 *	(1)	All math is done in absolute coordinates using double precision 
 *		floating point and in double float minutes.
 */

uint8_t mp_aline(const double target[], const double minutes)
{
	struct mpBuffer *bp, *bf; 		// pointers to previous & current move

#ifdef __dbALINE_CALLED
		fprintf_P(stderr, PSTR("Aline called %1.4f, %1.4f, %1.4f, %1.4f    %1.4f\n"),
				  x,y,z,a, minutes);
#endif

	if (minutes < EPSILON) {					// trap zero time moves
		return (TG_ZERO_LENGTH_MOVE);
	}
	// get a cleared buffer for current move and setup initial variables
	if ((bf = _mp_get_write_buffer()) == NULL) {// get buffer or die trying
		TRAP(PSTR("Failed to get buffer in mp_aline()"));
		return (TG_BUFFER_FULL_FATAL);			// (not supposed to fail)
	}
	bp = _mp_get_prev_buffer(bf);				// get previous move
	mp_copy_vector(bf->target, target, AXES); 	// set target for runtime
	bf->time = minutes;
	bf->length = mp_get_axis_vector_length(bf->target, mm.position);
	if (bf->length < MIN_LINE_LENGTH) {			// trap zero-length lines
		return (TG_ZERO_LENGTH_MOVE);
	}
	bf->cruise_velocity_set = bf->length / minutes;	// Vt requested
	bf->exit_velocity_limit = bf->cruise_velocity_set;
	mp_set_unit_vector(bf->unit, bf->target, mm.position);

	// initialize jerk terms - pick the smallest jerk of the active axes
	mm.jerk_max = cfg.a[X].jerk_max;			//...or X term
	for (uint8_t i=Y; i<AXES; i++) {
		if (bf->unit[i] > EPSILON) {
			mm.jerk_max = min(mm.jerk_max, cfg.a[i].jerk_max);
		}
	}
	mm.jerk_max_cubert = cubert(mm.jerk_max);	// used by planning
	mr.jerk_max_div2 = mm.jerk_max/2; 			// used by runtime

	// handle case where previous move is a queued or running arc
	if ((bp->move_type == MP_TYPE_ARC) && (bp->buffer_state != MP_BUFFER_EMPTY)) {
		bf->join_velocity_limit = bp->exit_velocity;
		(void)_mp_get_segments(bp->exit_velocity, bf->cruise_velocity_set, 0, 
							   bf->length, bf);
		bf->difference_velocity = _mp_get_difference_velocity(bf->entry_velocity, bf->length, mm.jerk_max);
		bf->difference_to_stop = bf->difference_velocity;
		bf->replannable = TRUE;				// you cannot replan an exact stop
		mp_copy_vector(mm.position, bf->target, AXES);	// update planning position
		_mp_queue_write_buffer(MP_TYPE_ALINE);
		return (TG_OK);	// don't bother to backplan after an arc. Just return.
	}

	// handle straight line cases (non-arc)
	if ((bp->buffer_state < MP_BUFFER_QUEUED) || // if this is the first buffer
		(cm_get_path_control_mode() == PATH_EXACT_STOP)) {	
		bf->join_velocity_limit = 0;
		bf->exit_velocity_limit = 0;
		bf->replannable = FALSE;		// you cannot replan an exact stop
	} else { 
		bf->join_velocity_limit = _mp_get_join_velocity(bp->unit, bf->unit, 
									bp->exit_velocity_limit, bf->cruise_velocity_set);
		bf->difference_velocity = _mp_get_difference_velocity(bf->entry_velocity, bf->length, mm.jerk_max);
		bf->difference_to_stop = bf->difference_velocity;
		bf->difference_to_setV = 0;
		bf->replannable = TRUE;
	}

//	if (tg.linenumber == 2530) {
//		tg.linenumber += 1;
//	}
//	if (tg.blockcount == 50) {
//		tg.linenumber = tg.blockcount;
//	}

	// do the actual work
	if (_mp_get_segments(min4(bp->exit_velocity_limit,
							  bf->join_velocity_limit, 
							  bf->cruise_velocity_set,
							  bf->difference_velocity),
						 bf->cruise_velocity_set, 
						 0, 
						 bf->length, bf) == 0) {;
		return (TG_OK);	// returned 0 segments, exit 'cause line's too-short
	}
	_mp_backplan(bf);
	mp_copy_vector(mm.position, bf->target, AXES);	// update planning position
	_mp_queue_write_buffer(MP_TYPE_ALINE);
	return (TG_OK);
}

/**** ALINE HELPERS ****
 * _mp_backplan()  			- recompute moves backwards from latest move
 * _mp_get_segments() 	 	- compute segment lengths and velocity contours
 * _mp_get_length()			- get length given Vi and Vt
 * _mp_get_velocity()		- get change in velocity given Vi, L and Jm
 * _mp_get_join_velocity()	- optimal vecloty for join
 */

/*
 * _mp_backplan()
 *
 *	Recompute segments and velocities of previous moves to fit the 
 *	acceleration and distance constraints & optimize target velocities.
 *
 *	Backplanning occurs as a two-pass operation. The first pass is a 
 *	backwards pass that starts at the head of the move currently being 
 *	planned and continues back to the tail of the latest move that cannot 
 *	be replanned (first non-replannable move). 
 *
 *	Moves become non-replannable when:
 *
 *	  (a) It's an exact stop move (always runs to zero)
 *
 *	  (b) A move becomes optimized, i.e. hits all it's limit velocities
 *
 *	  (c) The move is already executing. It's OK if the head is running,
 *			but not if the body or tail is running.
 *
 *	The backwards pass computes the maximum entry velocity for each move 
 *	that still satisfies the braking requirement for the various moves in
 *	the chain. The accumulated braking velocity as you work backwards in 
 *	the chain is recorded in each move as the cumulative velocity.
 *
 *	The forward pass then begins from the tail of the non-replnannable 
 *	move and continues until it optimally joins with the current move.
 *	It uses the braking velocities and the various limits to recompute 
 *	the segments and velocities for each of the constituent moves. If a 
 *	move becomes optimized it's set non-replannable, reducing the length 
 *	of the chain.
 */

static void _mp_backplan(struct mpBuffer *bf)
{
	uint8_t i;
	struct mpBuffer *current_buffer = bf;

	// Backward planning pass - compute cumulative braking velocities
	for (i=0; i<MP_BUFFER_SIZE; i++) {
		bf = _mp_get_prev_buffer(bf);// back up to previous buffer
		if (bf->replannable == FALSE) {
			break;
		}
		bf->difference_to_stop = bf->difference_velocity + 
								  bf->nx->difference_to_stop;
	}

	// Forward planning pass - recompute all the segments. At this point 
	// bf points to the first buffer before the replanning chain.
	while ((bf = _mp_get_next_buffer(bf)) != current_buffer) {
		bf->difference_to_setV = bf->difference_velocity + 
								 bf->pv->difference_to_setV;
		(void)_mp_get_segments(bf->pv->exit_velocity,
							   bf->cruise_velocity_set,
							   min4(bf->difference_to_setV,
							   		bf->nx->join_velocity_limit,
								 	bf->nx->cruise_velocity_set,
									bf->nx->difference_to_stop), 
							   bf->length, bf);

		if ((bf->entry_velocity == bf->join_velocity_limit) &&
			(bf->cruise_velocity == bf->cruise_velocity_set) &&
			(bf->exit_velocity == bf->exit_velocity_limit)) {
			bf->replannable = FALSE;
		}
	}
	bf->difference_to_setV = bf->difference_velocity + 
								 bf->pv->difference_to_setV;
	// do the current move last
	(void)_mp_get_segments(	bf->pv->exit_velocity,
							bf->cruise_velocity_set,
							0, bf->length, bf);
}

/*
 * _mp_get_segments()
 * _mp_get_segments_head_cases()
 * _mp_get_segments_tail_cases()
 *
 *	This function (and its helpers) sets the segment lengths and velocities:
 *		entry_velocity, cruise velocity, exit_velocity, 
 *		head_length, body_length, tail_length
 *		Returns: number of segments - 0-3
 *
 *	Various cases are handled; see inline comments
 */
static uint8_t _mp_get_segments(double entry_velocity, 
								double cruise_velocity,
								double exit_velocity, 
								double length,
								struct mpBuffer *bf) 
{
	uint8_t i;
	double ramp_acceleration, computed_velocity;	// used by 2 line head/tail case

	// 0 segment case - line is too short to create segments
	if (length < MIN_LINE_LENGTH) {
		TRAP(PSTR("Zero-length line found in _mp_get_segments()"));
		bf->length = 0;
		bf->head_length = 0;
		bf->body_length = 0;
		bf->tail_length = 0;
		return (0);
	}

	// initialize velocites
	bf->length = length;
	bf->entry_velocity = entry_velocity;
	bf->cruise_velocity = cruise_velocity;
	bf->exit_velocity = exit_velocity;


	// 1 segment body-only case
	if ((fabs(exit_velocity - entry_velocity) < EPSILON) &&  // Ve=Vc=Vx
		(fabs(exit_velocity - cruise_velocity) < EPSILON)) {
		bf->head_length = 0;
		bf->tail_length = 0;
		bf->body_length = bf->length;
		return (1);
	}

	// now get some optimal head and tail lengths
	bf->head_length = _mp_get_optimal_length(cruise_velocity, entry_velocity, mm.jerk_max);
	bf->tail_length = _mp_get_optimal_length(cruise_velocity, exit_velocity, mm.jerk_max);
	bf->body_length = bf->length - bf->head_length - bf->tail_length;

	// 3 segment case (full trapezoid)
	if (bf->body_length > 0) {
		return (3);
	}

	// 1 and 2 segment tail cases
	if (fabs(entry_velocity - cruise_velocity) < EPSILON) {	// Ve=Vc
		return (_mp_get_segments_tail_cases(bf));
	}

	// 1 and 2 segment head cases
	if (fabs(exit_velocity - cruise_velocity) < EPSILON) {	// Vx=Vc
		return (_mp_get_segments_head_cases(bf));
	}

	// 2 segment head and tail case (trapezoid with no plateau)
	//	A succesive approximation is used to compute a target velocity that
	//	adheres to the jerk maximum. The computed_velocity term holds the 
	//	target velocity based on the ramp acceleration of the previous best
	//	guestimate of the velocity. It's done this way because the expression 
	//	for computing the velocity directly based on the jerk is basically 
	//	uncomputable (on this chip). If the traget velocity drops below the 
	//	entry or exit velocity it's treated as a head or tail case.
	computed_velocity = cruise_velocity;
	for (i=0; i<MAX_PLANNER_ITERATIONS; i++) {
		bf->cruise_velocity = computed_velocity;
		ramp_acceleration = mm.jerk_max * sqrt(fabs(computed_velocity - entry_velocity)/mm.jerk_max)/2;
		computed_velocity = sqrt(square(entry_velocity) + square(exit_velocity) +
								 2*ramp_acceleration*bf->length) / sqrt(2);
		if (computed_velocity < bf->entry_velocity) {
			bf->cruise_velocity = entry_velocity;
			return (_mp_get_segments_tail_cases(bf));
		}
		if (computed_velocity < bf->exit_velocity) {
			bf->cruise_velocity = exit_velocity;
			return (_mp_get_segments_head_cases(bf));
		}
		if ((fabs(bf->cruise_velocity - computed_velocity) / bf->cruise_velocity) < PLANNER_ERROR_PERCENT) {
			break;
		}
	}
	if (i == MAX_PLANNER_ITERATIONS) {
		TRAP(PSTR("_mp_get_segments() failed to converge"));
	}
	bf->cruise_velocity = computed_velocity;
	bf->head_length = _mp_get_optimal_length(bf->cruise_velocity, entry_velocity, mm.jerk_max);
	bf->tail_length = _mp_get_optimal_length(bf->cruise_velocity, exit_velocity, mm.jerk_max);
	bf->body_length = 0;
	return (2);
}

static uint8_t _mp_get_segments_head_cases(struct mpBuffer *bf)
{
	bf->head_length = _mp_get_optimal_length(bf->cruise_velocity, bf->entry_velocity, mm.jerk_max);
	bf->tail_length = 0;
	// head is a perfect fit - this happens when the line is pre-computed to be optimal 
	if (fabs(bf->head_length - bf->length) < EPSILON) {	// Lh=L
		bf->body_length = 0;
		return (1);
	// head and cruise case
	} else if (bf->head_length < bf->length) {
		bf->body_length = bf->length - bf->head_length;
		return (2);
	// head region is too short - reduce entry velocity (pathological case)
	} else {
		bf->body_length = 0;
		bf->entry_velocity = _mp_get_optimal_length(bf->entry_velocity, bf->exit_velocity, mm.jerk_max);
		return (1);
	}
}

static uint8_t _mp_get_segments_tail_cases(struct mpBuffer *bf)
{
	bf->tail_length = _mp_get_optimal_length(bf->cruise_velocity, bf->exit_velocity, mm.jerk_max);
	bf->head_length = 0;
	// tail is a perfect fit - this happens when the line is pre-computed to be optimal 
	if (fabs(bf->tail_length - bf->length) < EPSILON) {	// Lt=L
		bf->body_length = 0;
		return (1);
	// cruise and tail case
	} else if (bf->tail_length < bf->length) {
		bf->body_length = bf->length - bf->tail_length;
		return (2);
	// tail region is too short - reduce entry velocity (pathological case)
	} else {
		bf->body_length = 0;
		bf->entry_velocity = _mp_get_optimal_length(bf->entry_velocity, bf->exit_velocity, mm.jerk_max);
		return (1);
	}
}

/*	
 * _mp_get_optimal_length()
 *
 * 	A convenient expression for determining the optimal_length (L) of 
 *	a line given the entry velocity (Ve), exit velocity (Vx) and the 
 *	max jerk (Jm), which is derived from these two equations:
 *
 *		T = 2 * sqrt(abs(V1-V2) / Jm);		// equation 5.x
 *		L = (V1+V2)/2 * T;					// equation [2]
 *
 */
static double _mp_get_optimal_length(const double V1, const double V2, const double Jm)
{
	return ((V1+V2) * sqrt(fabs(V1-V2) / Jm));
}

/*	
 * _mp_get_difference_velocity()
 *
 * 	A convenient expression for returning the change in velocity possible 
 *	for a given entry velocity (Ve), length (L), and maximum jerk (Jm).
 *	Ve must be less than V. Use sparingly as this's quite expensive.
 *	Solve the following for V: 
 *
 *	  length = (V+Vi) * sqrt((V-Ve)/Jm)
 *
 *  http://www.wolframalpha.com/input/?i=L%3D%28V%2BU%29*sqrt%28%28V-U%29%2FJ%29
 *	See real solution for V
 */

static double _mp_get_difference_velocity(const double Ve, const double L, const double Jm)
{
//	return (cubert(Jm) * pow(L,0.6666667) + Ve);

	double k1 = 3*sqrt(3);
	double k2 = cubert(2);
	double a = sqrt(27*square(Jm)*pow(L,4) + 32*Jm*square(L)*pow(Ve,3));
	double b = 27*Jm*square(L);
	double c = 16*pow(Ve,3);
	double d = cubert(k1*a+b+c);
	return (((d/k2)+(4*k2*square(Ve))/d - Ve)/3);

/*
	double k1 = 3*sqrt(3);
	double k2 = cubert(2);
	double a = sqrt(27*pow(Jm,2)*pow(L,4) + 32*Jm*pow(L,2)*pow(Ve,3));
	double b = 27*Jm*pow(L,2);
	double c = 16*pow(Ve,3);
	double d = cubert(k1*a+b+c);
	return (((d/k2)+(4*k2*pow(Ve,2))/d - Ve)/3);
*/
}

/*
 * _mp_get_join_velocity() - Chamnit's algorithm - simple
 *
 *  Computes the maximum allowable junction speed by finding the velocity
 *	that will yield the centripetal acceleration in the corner_acceleration 
 *	value. The value of delta sets the effective radius of curvature.
 *	Here's Chamnit's (Sonny J's) explanation of what's going on:
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

static double _mp_get_join_velocity(const double a_unit[], const double b_unit[], 
									const double a_velocity, const double b_velocity)
{
	double costheta = -((a_unit[X] * b_unit[X]) +
						(a_unit[Y] * b_unit[Y]) +
						(a_unit[Z] * b_unit[Z]) +
						(a_unit[A] * b_unit[A]) +
						(a_unit[B] * b_unit[B]) +
						(a_unit[C] * b_unit[C]));

	// 0 degree straight line case causes radius to blow up
	if (fabs(costheta + 1) < EPSILON) { // trap costheta = -1
		return (7035367115);			// a really large arbitrary number
	}
	// 180 degree reversal case causes sintheta2 to blow up
	if (fabs(costheta - 1) < EPSILON) { // trap costheta = 1
		return (0);
	}

//	double delta = _mp_get_corner_delta(a_unit, b_unit);
	double delta = cfg.a[X].jerk_corner_offset;
	double accel_max = cfg.jerk_corner_acceleration;
	double sintheta_over2 = sqrt((1 - costheta)/2);
	double radius = delta * sintheta_over2 / (1-sintheta_over2);	
	return(sqrt(accel_max * radius));
}

/*	
 * _mp_get_corner_delta() - Compute delta for Chamnit's algorithm (Sonny J)
 *
 *  This helper function extends Chamnit's algorithm by computing a value
 *	for delta that takes the contributions of the individual axes in the 
 *	move into account. It allows the radous of curvature to vary by axis.
 *	This is necessary to support axes that have different dynamics; such 
 *	as a Z axis that doesn't move as fast as X and Y (in the limit a screw 
 *	Z on a belt drive XY machine like a makerbot), or rotary axes ABC that 
 *	have completely different dynamics than their linear counterparts.
 *
 *	The function takes the absolute values of the sum of the unit vector
 *	components as a measure of contribution ot the move, then scales the 
 *	delta values from the non-zero axes into a composite delta to be used
 *	for the move. Shown for an XY vector:
 *
 *	U[i]	Unit sum of i'th axis	abs(unit_a[i]) + abs(unit_b[i])
 *	Usum	Length of sums			Ux + Uy
 *	d		Delta of sums			(Dx*Ux+DY*UY)/Usum
 */


static double _mp_get_corner_delta(const double a_unit[], const double b_unit[])
{
	double delta = 0;
	double unit_sum = 0;
//	double a,b,c,d;

	for (uint8_t i=0; i<AXES; i++) {
//		a = a_unit[i];
//		b = b_unit[i];
//		c = cfg.a[i].jerk_corner_offset;
//		d = (abs(a_unit[i]) + abs(b_unit[i])) * cfg.a[i].jerk_corner_offset;
//		delta = delta + d;
		delta += (fabs(a_unit[i]) + fabs(b_unit[i])) * cfg.a[i].jerk_corner_offset;
		unit_sum += (fabs(a_unit[i]) + fabs(b_unit[i]));
	}
	return(delta / unit_sum);
}

/*****************************/
/**** ALINE RUN ROUTINES *****/
/*****************************
 *  _mp_run_aline()		- entry point for runtime acceleration lines
 *	_mp_run_accel_0()	- initialize acceleration phase
 *	_mp_run_accel_1()	- run 1st half of acceleration
 *	_mp_run_accel_2()	- run 2nd half of acceleration
 *	_mp_run_cruise()	- run cruise phase
 *	_mp_run_decel_0()	- initialize deceleration phase
 *	_mp_run_decel_1()	- run 1st half of deceleration
 *	_mp_run_decel_2()	- run 2nd half of deceleration
 *	_mp_run_segment()	- helper for running a segment
 *	_mp_run_finalize()	- helper for running last segment
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
 *	T = (sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3)+A^2/
 * (J^2*(sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3))+A/J
 *
 *  Note: A cruise is supposed to be guaranteed to have a non-zero end 
 *		  velocity, otherwise the time spent in the cruise is infinite. 
 *		  Zero velocity cruises are detected and rejected.
 *
 *	Positions and targets: 
 *	  - mr.position is the current segment position
 *	  - mr.target is the current segment target
 *	  - bf->target carries the ndpoint of the move and is used during
 *		finalization to correct any accumulated position errors.
 */

static uint8_t _mp_run_aline(struct mpBuffer *bf)
{
	// preliminary tests on all lines 
	if (mq_test_motor_buffer() == FALSE) {
		return (TG_EAGAIN); 	// block if no motor buffer available
	}
	// run the move
	switch (bf->move_state) {
		case MP_STATE_NEW:	   { return(_mp_run_accel_0(bf)); }
		case MP_STATE_ACCEL_1: { return(_mp_run_accel_1(bf)); }
		case MP_STATE_ACCEL_2: { return(_mp_run_accel_2(bf)); }
		case MP_STATE_CRUISE:  { return(_mp_run_cruise(bf)); }
		case MP_STATE_DECEL_0: { return(_mp_run_decel_0(bf)); }
		case MP_STATE_DECEL_1: { return(_mp_run_decel_1(bf)); }
		case MP_STATE_DECEL_2: { return(_mp_run_decel_2(bf)); }
	}
	TRAP(PSTR("Abnormal exit from _mp_run_aline()"));
	return (TG_ERR);	// thisis not supposed to get here.
}

static uint8_t _mp_run_accel_0(struct mpBuffer *bf) 
{
	// general inits for the move
	bf->replannable = FALSE;					// stop replanning

	if (bf->length < MIN_LINE_LENGTH) {
		// Note this does NOT advance the position which means that 
		// any position error will be compensated by the next move.
		return (TG_OK);	// TG_OK tosses move & frees buffer
	}
	// inits specific to acceleration
	if (bf->head_length < MIN_LINE_LENGTH) {
		bf->move_state = MP_STATE_CRUISE;
		return (_mp_run_cruise(bf));
	}
	mr.midpoint_velocity = (bf->entry_velocity + bf->cruise_velocity) / 2;
	TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Zero midpoint velocity in _mp_run_accel_0(): %f"), mr.midpoint_velocity)	
	mr.time = bf->head_length / mr.midpoint_velocity;
	mr.midpoint_acceleration = mr.time * mr.jerk_max_div2;

	// number of segments in *each half*
	mr.segments = round(round(ONE_MINUTE_OF_MICROSECONDS * (mr.time / cfg.min_segment_time)) / 2);
	if ((uint16_t)mr.segments == 0) {		
		TRAP(PSTR("No acceleration segments in _mp_run_accel_0()"));
		return (TG_OK);					// cancel the move if too small			
	}
	mr.segment_time = mr.time / (2 * mr.segments);
	mr.elapsed_time = mr.segment_time / 2; //compute pos'n from midpoint
	mr.microseconds = uSec(mr.segment_time);
	mr.segment_count = (uint32_t)mr.segments;
	bf->move_state = MP_STATE_ACCEL_1;
	return (_mp_run_accel_1(bf));			// first time through
}

static uint8_t _mp_run_accel_1(struct mpBuffer *bf) 
{
	mr.segment_velocity = bf->entry_velocity + (mr.jerk_max_div2 * square(mr.elapsed_time));
	ritorno (_mp_run_segment(bf)); // returns if not done (this is not an error)
	// setup for second half
	mr.segment_count = (uint32_t)mr.segments;
	mr.elapsed_time = mr.segment_time / 2;
	bf->move_state = MP_STATE_ACCEL_2;
	return(TG_EAGAIN);			// not done yet
}

static uint8_t _mp_run_accel_2(struct mpBuffer *bf) 
{
	mr.segment_velocity = mr.midpoint_velocity + 
						 (mr.elapsed_time * mr.midpoint_acceleration) -
						 (mr.jerk_max_div2 * square(mr.elapsed_time));
	ritorno(_mp_run_segment(bf)); // returns if not done (this is not an error)
	bf->move_state = MP_STATE_CRUISE;
	return(TG_EAGAIN);			// not done yet
}

static uint8_t _mp_run_cruise(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	bf->move_state = MP_STATE_DECEL_0;	// you are going into decel after this, regardless

	// look for various premature end conditions
	if ((bf->body_length < MIN_LINE_LENGTH) || 	(bf->cruise_velocity < EPSILON)) {
		return (_mp_run_decel_0(bf));
	}

	// do the cruise for real
	bf->time = bf->body_length / bf->cruise_velocity;
	TRAP_IF_TRUE((bf->time == 0), PSTR("Zero time in _mp_run_cruise(): %f"), bf->time)
	mr.microseconds = uSec(bf->time);

	for (i=0; i < AXES; i++) {
		mr.target[i] = mr.position[i] + bf->unit[i] * bf->body_length;
		travel[i] = mr.target[i] - mr.position[i];
	}
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	mp_copy_vector(mr.position, mr.target, AXES);	// update runtime position
	return (TG_EAGAIN);
}

static uint8_t _mp_run_decel_0(struct mpBuffer *bf)
{
	if (bf->tail_length < MIN_LINE_LENGTH) {
		return (TG_OK);					// end the move & free the buffer
	}
	mr.midpoint_velocity = (bf->cruise_velocity + bf->exit_velocity) / 2;
	TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Zero midpoint velocity in _mp_run_decel_0(): %f"), mr.midpoint_velocity)	
	mr.time = bf->tail_length / mr.midpoint_velocity;
	mr.midpoint_acceleration = mr.time * mr.jerk_max_div2;

	// number of segments in *each half*
	mr.segments = round(round(ONE_MINUTE_OF_MICROSECONDS * (mr.time / cfg.min_segment_time)) / 2);
	if ((uint16_t)mr.segments == 0) {		// more efficient than comparing to < EPSILON
		TRAP(PSTR("No deceleration segments in _mp_run_decel_0"));
		return (TG_OK);					// cancel the move if too small	
	}
	mr.segment_time = mr.time / (2 * mr.segments);
	mr.elapsed_time = mr.segment_time / 2; //compute pos'n from midpoint	
	mr.microseconds = uSec(mr.segment_time);
	mr.segment_count = (uint32_t)mr.segments;
	bf->move_state = MP_STATE_DECEL_1;
	return (TG_EAGAIN);
}

static uint8_t _mp_run_decel_1(struct mpBuffer *bf) 
{
	mr.segment_velocity = bf->cruise_velocity - (mr.jerk_max_div2 * square(mr.elapsed_time));
	ritorno(_mp_run_segment(bf));	// return is OK, not an error
	// setup for second half
	mr.segment_count = (uint32_t)mr.segments;
	mr.elapsed_time = mr.segment_time / 2;
	bf->move_state = MP_STATE_DECEL_2;
	return (TG_EAGAIN);
}

static uint8_t _mp_run_decel_2(struct mpBuffer *bf) 
{
	if (mr.segment_count > 1) {
		mr.segment_velocity = mr.midpoint_velocity - 
							 (mr.elapsed_time * mr.midpoint_acceleration) +
							 (mr.jerk_max_div2 * square(mr.elapsed_time));
		return(_mp_run_segment(bf));
	} else {
		_mp_run_finalize(bf);	// for accuracy
		return(TG_OK);			// TG_OK finishes the line
	}
	TRAP(PSTR("Abnormal exit from _mp_run_decel_2()"));
	return (TG_ERR);
}

/*
 * _mp_run_segment() - segment runner helper
 */
static uint8_t _mp_run_segment(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	/* Multiply the computed position by the unit vector to get the 
	 * contribution for each axis. Set the target in absolute coords
	 * (floating point) and compute the relative steps.
	 */
	for (i=0; i < AXES; i++) {
		mr.target[i] = mr.position[i] + (bf->unit[i] * 
					   mr.segment_velocity * mr.segment_time);
		travel[i] = mr.target[i] - mr.position[i];
	}
	// queue the line and adjust the variables for the next iteration
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	mr.elapsed_time += mr.segment_time;
	mp_copy_vector(mr.position, mr.target, AXES);	// update runtime position from next target
	if (--mr.segment_count > 0) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

/*
 * _mp_run_finalize() - last segment runner helper
 */
static void _mp_run_finalize(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];
	
	// finalize - do the last segment to maintain position accuracy
	mr.length = mp_get_axis_vector_length(bf->target, mr.position);
	if ((mr.length < MIN_LINE_LENGTH) || (bf->exit_velocity < EPSILON)) {
		return; 			// trap zero-length cases
	}
	mr.time = mr.length / bf->exit_velocity;
	mr.microseconds = uSec(mr.time);

	for (i=0; i < AXES; i++) {
		travel[i] = bf->target[i] - mr.position[i];
	}
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	mp_copy_vector(mr.position, bf->target, AXES);	// update runtime position from initial target
	return;
}


//##########################################
//############## UNIT TESTS ################
//##########################################

#ifdef __UNIT_TESTS

void _mp_test_get_segments(void);
void _mp_test_buffers(void);
void _mp_test_get_jerk(void);
void _mp_test_get_join_velocity(void);
void _mp_make_unit_vector(double unit[], double x, double y, double z, 
						  double a, double b, double c);
void _mp_test_get_segments(void);
void _mp_setup_jerk(double stall_velocity, double Jm);
void _mp_test_segments(double Ve, double Vc, double Vx, double length, struct mpBuffer *bf);

void mp_unit_tests()
{
//	_mp_test_buffers();
//	_mp_test_get_jerk();
	_mp_test_get_join_velocity();
//	_mp_test_get_segments();
}

void _mp_setup_jerk(double stall_velocity, double Jm)
{
	for (uint8_t i=0; i<AXES; i++) {
		cfg.a[X].jerk_max = Jm;
		cfg.a[X].jerk_transition_size = mp_get_jerk_transition_size(stall_velocity, Jm);	
	}
	mm.jerk_max = Jm;  // just use the X value for the tests
	mm.jerk_max_cubert = cubert(mm.jerk_max);
	mr.jerk_max_div2 = mm.jerk_max/2; 
}

void _mp_test_segments(double Ve, double Vc, double Vx, double length, struct mpBuffer *bf)
{
	bf->join_velocity_limit = Ve;
	bf->cruise_velocity_set = Vc;
	bf->exit_velocity_limit = Vx;
	_mp_get_segments(Ve, Vc, Vx, length, bf);
}

void _mp_test_get_segments()
{
	struct mpBuffer *bf = _mp_get_write_buffer();

	// these tests are calibrated to indicated stall_velocity & Jm
	_mp_setup_jerk(200, 50000000);	    // set stall velocity & Jm

// 0 segment cases: line below minimum velocity or length
//				   Ventry  Vcruise Vexit   Length
	_mp_test_segments(0,	0.001,	0,		1.0, bf);
	_mp_test_segments(0,	100,	0,		0.0, bf);

// 1 segment cases: line shorter than minimum transition length cases
//				   Ventry  Vcruise Vexit   Length
//	_mp_test_segments(400,	400, 	0, 		0.8, bf);
//	_mp_test_segments(600,	600, 	200,	0.8, bf);
//	_mp_test_segments(0,	400, 	400,	0.8, bf);
//	_mp_test_segments(200,	600, 	600,	0.8, bf);

// 3 segment cases (HBT)
//				   Ventry  Vcruise Vexit   Length
//	_mp_test_segments(0,	190, 	0, 		0.8, bf);
//	_mp_test_segments(200,	400, 	0, 		2.0, bf);

// 2 segment cases (HT)
//				   Ventry  Vcruise Vexit   Length
//	_mp_test_segments(0,	200, 	0, 		0.8, bf);
//	_mp_test_segments(0,	400, 	0, 		0.8, bf);
//	_mp_test_segments(200,	400, 	0, 		0.8, bf);
//	_mp_test_segments(400,	400, 	0, 		2.0, bf);
//	_mp_test_segments(0,	400, 	200,	0.8, bf);

// 1 segment cases (H,B and T)
//				   Ventry  Vcruise Vexit   Length
	_mp_test_segments(800,	800, 	800, 	1.0, bf);

//	_mp_test_segments(0,	400, 	0, 		0.8, bf);
//	_mp_test_segments(200,	400, 	0, 		0.8, bf);
//	_mp_test_segments(400,	400, 	0, 		2.0, bf);
//	_mp_test_segments(0,	400, 	200,	0.8, bf);


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

#define JERK_TEST_VALUE (50000000)

void _mp_test_get_join_velocity()
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
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 2;				// angled straight line
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.7071,	0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 3;				// 5 degree bend
	_mp_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.9962,	0.0872, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 4;				// 30 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.8660,	0.5000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 5;				// 45 degrees
	_mp_make_unit_vector(mm.a_unit, 0.8660,	0.5000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.2588,	0.9659, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 6;				// 60 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 7;				// 90 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit, 0.0000,	1.0000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 8;				// 90 degrees rotated 45 degrees
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.7071,	0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 9;				// 120 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 10;				// 150 degrees
	_mp_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.8660,	0.5000, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);

	mm.test_case = 11;				// 180 degrees
	_mp_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.b_unit,-0.7071,-0.7071, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.b_unit, 200, 200);
}


/*

// Case 2: Y reverses direction in Line B
	mm.test_case = 2;
	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 150, 150);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

// Case 3: Both X and Y reverse direction
	mm.test_case = 3;
	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 100, 100);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -8.666,-5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

// Case 3.1: Both X and Y reverse direction - reverse direction from 3.0
	mm.test_case = 3.1;
	_mp_make_unit_vector(mm.a_unit,-8.666,-5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit,-8.666,-5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 100, 100);

	_mp_make_unit_vector(mm.a_unit,-8.666,-5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

// Case 4: Right angle along X and Y axes
	mm.test_case = 4;
	_mp_make_unit_vector(mm.a_unit, 10, 0, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,    0,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit, 10, 0, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,    0,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 100, 100);

	_mp_make_unit_vector(mm.a_unit, 10, 0, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,    0,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

// Case 5: Right angle rotated WRT X and Y axes
	mm.test_case = 5;
	_mp_make_unit_vector(mm.a_unit, 10,10, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -10,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit, 10,10, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -10,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 100, 100);

	_mp_make_unit_vector(mm.a_unit, 10,10, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,  -10,10, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

// Case 6: Y diverges slightly from X at high speed
	mm.test_case = 6;
	_mp_make_unit_vector(mm.a_unit, 10, 0, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   10, 1, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 1400, 1400);

	_mp_make_unit_vector(mm.a_unit, 10, 0, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   10, 1, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 800, 800);
}
*/

void _mp_test_get_jerk()
{
/*
	double dVmax;
	dVmax = _mp_get_jerk_limit(   0, 0.5, cfg.a[X].jerk_max);
	dVmax = _mp_get_jerk_limit( 400, 0.5, cfg.a[X].jerk_max);
	dVmax = _mp_get_jerk_limit(1000, 0.5, cfg.a[X].jerk_max);
	dVmax -=1;
	return;	
*/
}

void _mp_test_buffers()
{
	mp_check_for_write_buffers(MP_BUFFERS_NEEDED); // test for enough free buffers

	_mp_get_write_buffer();		// open a write buffer [0]
	_mp_get_write_buffer();		// open a write buffer [1]
	_mp_get_write_buffer();		// open a write buffer [2]

	_mp_get_run_buffer();		// attempt to get run buf - should fail (NULL)

	_mp_queue_write_buffer(MP_TYPE_ALINE);	// queue the write buffer [0]
	_mp_queue_write_buffer(MP_TYPE_LINE);	// queue the write buffer [1]
	_mp_queue_write_buffer(MP_TYPE_DWELL);	// queue the write buffer [2]

	_mp_get_run_buffer();		// attempt to get run buf - should succeed

}
#endif
