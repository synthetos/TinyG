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
 *
 *	317.10 - changed out jerk terms for minimum jerk axis
 *	317.11 - config now driven by stall velocity - transition size is derived
 *	317.17 - compile sizes (-O0)
 	Program:  100538 bytes (37.2% Full) (.text + .data + .bootloader)
	Data:      11162 bytes (68.1% Full) (.data + .bss + .noinit)
 */
/* Planning Velocity Notes (applies to mpBuffer variables)
 *
 * limit_velocity means:
 *	head - This is the maximum velocity allowable at the junction of the
 *			current move and the previous move. It is the smallest of:
 *			- requested velocity for the move
 *			- maximum junction velocity allowed by tangential jerk
 *			  of the join with the previous move
 *			- velocity achievable by the backplanning chain to the head 
 *			  junction. This is exit velocity of the last non-replannable 
 *			  move plus the the sum of the max_dV's of the intervening
 *			  moves in the backplanning chain. 
 *	body - This is always the requested velocity (requested cruise vel.)
 *	head - This is set to the requested velocity when the move is new 
 *			(last move in the chain), and reset to the limit of the 
 *			head of the next move when the next move arrives.
 *
 *	max_dV is the maximum change in velocity that can be accommodated by
 *	the move. It is initially computed to the theoretical maximum delta
 *	for that length and jerk using _mp_get_velocity() (at a cost of about
 *	10,000 cycles!). It may be replaced with smaller values as the entry 
 *	and exit limits become known. max_dV is stored in the body buffer for 
 *	the entire move. It is not used in the head or tail buffers. 
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
//	uint8_t path_mode;			// path control mode 

	double target[AXES];		// target position in floating point
	double unit[AXES];			// unit vector for axis scaling & planning
	struct mpBufferArc a;		// arc variables

	double time;				// line, helix or dwell time in minutes
	double length;				// total length of line or helix in mm
	double head_length;
	double body_length;
	double tail_length;

	double entry_velocity;		// actual entry velocity of the move
	double cruise_velocity;		// actual achieved cruise velocity
	double exit_velocity;		// actual exit velocity

	double entry_velocity_limit;// maximum entry velocity possible
	double cruise_velocity_set;	// cruise velocity requested for move
	double exit_velocity_limit;	// maximum exit velocity possible
								// Q: Is this always the cruise velocity set?
	double braking_velocity;	// braking velocity for this move
	double cumulative_braking;	// braking velocity for move chain
};

struct mpBufferPool {			// ring buffer for sub-moves
	struct mpBuffer *w;			// get_write_buffer pointer
	struct mpBuffer *q;			// queue_write_buffer pointer
	struct mpBuffer *r;			// get/end_run_buffer pointer
	struct mpBuffer bf[MP_BUFFER_SIZE];// buffer storage
};

struct mpMoveMasterSingleton {	// common variables for planning (move master)
	double position[AXES];		// final move position 
	double target[AXES];		// target move position
//	double a_unit[AXES];		// previous unit vector (debug purposes only)
//	double unit[AXES];			// for axis scaling and jerk computation
	double jerk_max;			// jerk value to use for planning this move
	double jerk_max_cubert;		// cube root of jerk for planning

#ifdef __UNIT_TESTS
	double jerk_size;
	double jerk_limit_max;
	double test_case;
	double test_velocity;
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

static void _mp_init_buffers(void);
static void _mp_unget_write_buffer(void);
static void _mp_clear_buffer(struct mpBuffer *bf); 
static void _mp_queue_write_buffer(const uint8_t move_type);
static void _mp_finalize_run_buffer(void);
static struct mpBuffer * _mp_get_write_buffer(void); 
static struct mpBuffer * _mp_get_run_buffer(void);
static struct mpBuffer * _mp_get_prev_buffer(const struct mpBuffer *bf);
static struct mpBuffer * _mp_get_next_buffer(const struct mpBuffer *bf);

static void _mp_set_mm_position(const double target[]) ;
static void _mp_set_mr_position(const double target[]);
static void _mp_set_unit_vector(double unit[], double target[], double position[]);

static uint8_t _mp_run_aline(struct mpBuffer *bf);
static uint8_t _mp_run_line(struct mpBuffer *bf);
static uint8_t _mp_run_dwell(struct mpBuffer *bf);
static uint8_t _mp_run_arc(struct mpBuffer *bf);
static uint8_t _mp_run_stops(struct mpBuffer *bf);
//UNUSED static void _mp_kill_dispatcher(void);

static void _mp_backplan(struct mpBuffer *bf);
static uint8_t _mp_get_segments(double entry_velocity, double cruise_velocity,
								double exit_velocity, double length,
								struct mpBuffer *bf);
static double _mp_get_length(const double Vi, const double Vt, const double Jm);
static double _mp_get_ht_length(const double Ve, const double Vc, const double Vx, const double Jm);
static double _mp_get_velocity(const double Vi, const double L, const double Jm);
static double _mp_get_join_velocity(const double a_unit[], const double b_unit[], 
									const double a_velocity, const double b_velocity);

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

/**** SIMPLE HELPERS ******************************************************
 * mp_set_axis_position()		- set MM & MR positions (supports G92)
 * mp_get_axis_vector_length()	- return the length of an axis vector
 * mp_copy_vector()				- copy vector of arbitrary length
 * _mp_set_unit_vector()		- populate a unit vector by pos. & target
 * _mp_set_mm_position()		- set move final position for traj planning
 * _mp_set_mr_position()		- set move/sub-move position for runtime
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

// used by external callers such as G92
uint8_t mp_set_axis_position(const double position[])
{
	for (uint8_t i=0; i<AXES; i++) {
		mm.position[i] = position[i];
	}
	_mp_set_mr_position(mm.position);
	return (TG_OK);
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

// copy vector
void mp_copy_vector(double dest[], const double src[], uint8_t length) 
{
	for (uint8_t i=0; i<length; i++) {
		dest[i] = src[i];
	}
}

// compute unit vector
static void _mp_set_unit_vector(double unit[], double target[], double position[])
{
	double length = mp_get_axis_vector_length(target, position);
	for (uint8_t i=0; i < AXES; i++) {
		unit[i] = (target[i] - position[i]) / length;
	}
}

// set move final position for trajectory planning
static void _mp_set_mm_position(const double target[]) 
{ 
	for (uint8_t i=0; i<AXES; i++) {
		mm.position[i] = target[i];
	}
}

// set move/sub-move runtime position
static void _mp_set_mr_position(const double target[]) 
{ 
	for (uint8_t i=0; i<AXES; i++) {
		mr.position[i] = target[i];
	}
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
	_mp_set_mm_position(bf->target);		// set mm position for planning
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
	_mp_set_mr_position(bf->target);		// set mr position for runtime
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
	double length;

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

	length = sqrt(square(bf->target[axis_1] - i) +
				  square(bf->target[axis_2] - j) +
				  square(bf->target[axis_linear] - k));

	//	Compute unit vector
	// I think you can take the normal of the vector between the 
	//	center point (i,j) and the target (x,y) and divide by the 
	//	length of (i,j) to (x,y). Must also account for plane-axes
	//	and the linear axis.
/*
	double offset[3] = {i, j, k};
	for (uint8_t i=0; i < 3; i++) {
		bf->unit[i] = (bf->target[i] - offset[i]) / length;
	}
*/
	_mp_set_mm_position(bf->target);		// set mm position for planning
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
		_mp_set_mr_position(mr.target);
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
 *	This module uses maximum jerk motion equations to plan acceleration
 *	and deceleration segments that obey maximum jerk parameters. The jerk
 *	is the rate of change of acceleration (derivative), which is the third 
 *	derivative of position. The jerk is a measure of impact that a machine
 *	can take, and is therefore the most logical way to limit the velocity
 *	of a move.
 *
 *	If the rate of acceleration is controlled at the start and end of a 
 *	move - where the jerk is highest - the acceleration or deceleration 
 *	during the move can be much faster in the middle of the transition 
 *	than the machine could sustain at either end, and therefore allow 
 *	the move to transition to the target velocity much faster. This path 
 *	makes an S curve in velocity
 *
 *	The S curve move is known as a 5 segment move, as the accel and decel 
 *	segments are divided in to concave and convex halves. A 5 segment move 
 *	has two acceleration segments, followed by a cruise segment, followed 
 *	by two deceleration segments. 
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
 *	The 5 segment moves can be reduced to 3 segment, constant acceleration
 *	trapezoidal moves for planning purposes as the time taken in the 
 *	transition segment is the same for the constant acceleration and the 
 *	S-curve cases. (See Ed Red's course notes).
 *
 *	The initial velocity of the head is dependent on the path control 
 *	mode in effect and the transition jerk. Vi is always zero for
 *	EXACT STOP mode. For EXACT PATH and CONTINUOUS modes Vi is computed 
 *	based on the requested velocity and the magnitude of the linear and 
 *	tangential (cornering) jerk.
 *
 *	The body is the cruise segment where the line is running at its
 *	cruide velocity. The tail of each line is always intially computed to 
 *	decelerate to zero. This may change to a non-zero value as new lines
 *	are added and the move chain is re-planned.
 *
 *	As mentioned above, sufficient length is reserved in the tail to allow 
 *	deceleration from the cruise velocity to zero (braking). If the next 
 *	move has a non-zero entry velocity the previous moves are 
 *	recomputed (backplanned) to attain the maximum velocity while still 
 *	supporting braking to zero. 
 */
/*	Aline() is separated into a trajectory planner and a set of runtime
 *	execution routines (run routines) that execute as continuations called 
 *	by mp_move_dispatcher()
 *
 * Trajectory planner:
 *
 *	The aline() trajectory planner main routine is called to compute and 
 *	queue a new line. It computes all initial parameters, examines the 
 *	transition cases, computes and queues the sub-moves (trapezoid parts)
 *	as a set of move buffers. There is a buffer for each trapezoid part
 *	(head, body and tail) but sometimes these are NULL buffers.
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
 */	
/*	The cases for joining lines to lines are:
 *
 *	  - CONTINUOUS MODE (G64) is the default mode. The moves will attempt 
 *		to run at their maximum requested speed, accelerating or 
 *		decelerating at way points (line junctions) to match speeds and 
 *		maintain maximum velocity. If the angle between two lines is too 
 *		sharp (angular jerk is too high) the move will be downgraded to 
 *		exact path mode for that line only (which may in turn get 
 *		downgraded to exact stop mode). Continuous mode cases are: 
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
 *		speed at the way point. The new line accelerates from the join 
 *		speed. The join speed is computed based on the estimated angular 
 *		jerk between the two lines. If the jerk is too extreme (join angle 
 *		is too sharp & fast) the line will be further downgraded to exact 
 *		stop mode (for that line only).
 *
 *	  - EXACT_STOP_MODE: (G61) is the same as exact path mode except the 
 *		join speed is zero. Exact stop is always used for 180 degree turns
 */
/*  Combined Cases - By the time you combine all these you get a series of 
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
 */
/*  Special Cases - All of the above cases have sub-cases that are invoked
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
/*	Joining to Arcs - Note that at the current time only continuous mode 
 *	is supported when joining a line to an arc. These cases apply
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
 *		floating point and in double float minutes.
 *
 *	(3)	You may notice that initialized line buffers use Vi, Vt and Length
 *		but do not require Time. Time is derived from Vi, Vt & L.
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
	mp_copy_vector(mm.target, target, AXES);	// set mm.target
	bf->time = minutes;
	bf->length = mp_get_axis_vector_length(mm.target, mm.position);
	if (bf->length < MIN_LINE_LENGTH) {			// trap zero-length lines
		return (TG_ZERO_LENGTH_MOVE);
	}
	bf->cruise_velocity_set = bf->length / minutes;	// Vt requested
	bf->exit_velocity_limit = bf->cruise_velocity_set;
	_mp_set_unit_vector(bf->unit, mm.target, mm.position);

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
		bf->entry_velocity_limit = bp->exit_velocity;
		(void)_mp_get_segments(bp->exit_velocity, bf->cruise_velocity_set, 0, 
							   bf->length, bf);
		bf->replannable = TRUE;				// you cannot replan an exact stop
		_mp_queue_write_buffer(MP_TYPE_ALINE);
		return (TG_OK);	// don't bother to backplan an arc. Just return.
	}

	// handle straight line cases (non-arc)
	if ((bp->buffer_state < MP_BUFFER_QUEUED) || // if this is the first buffer
		(cm_get_path_control_mode() == PATH_EXACT_STOP)) {	
		bf->entry_velocity_limit = 0;
		bf->exit_velocity_limit = 0;
		bf->replannable = FALSE;		// you cannot replan an exact stop
	} else { 
		bf->entry_velocity_limit = _mp_get_join_velocity(bp->unit, bf->unit, 
									bp->exit_velocity_limit, 
									bf->cruise_velocity_set);
		bf->braking_velocity = _mp_get_velocity(bf->exit_velocity, bf->length, mm.jerk_max);
		bf->cumulative_braking = bf->braking_velocity;
		bf->replannable = TRUE;
	}

	// do the actual work
	if (_mp_get_segments(min4(bp->exit_velocity_limit,
							  bf->entry_velocity_limit, 
							  bf->cruise_velocity_set,
							  bf->braking_velocity),
						 bf->cruise_velocity_set, 0, 
						 bf->length, bf) == 0) {;
		return (TG_OK);	// returned 0 segments, exit 'cause line's too-short
	}
	_mp_backplan(bf);
	_mp_queue_write_buffer(MP_TYPE_ALINE);
	return (TG_OK);
}

/**** ALINE HELPERS ****
 * _mp_backplan()  			- recompute moves backwards from latest move
 * _mp_get_segments() 	 	- compute segment lengths and velocity contours
 * _mp_get_length()			- get length given Vi and Vt
 * _mp_get_velocity()		- get change in velocity given Vi, L and Jm
 * _mp_get_join_velocity()	- optimal vecloty for join
 * mp_get_jerk_limit()	  	- helper for _mp_get_join_velocity()
 * mp_get_jerk_transition_size() - helper for _mp_get_join_velocity()
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
	// backward planning pass - compute cumulative braking velocities
	uint8_t i = MP_BUFFER_SIZE;					// put a limit on this to be safe
	while (--i > 0) {
		bf = _mp_get_prev_buffer(bf);// back up to previous buffer
		if (bf->replannable == FALSE) {
			break;
		}
		bf->cumulative_braking = bf->braking_velocity + 
								 bf->nx->cumulative_braking;
	}
	if (i==0) { 						// this shouldn't happen
		TRAP(PSTR("Lookback error in _mp_backplan"));
		return;
	}
	// At this point bf is pointing one past the first usable buffer in 
	// the chain (i.e. pointing to the first non-replannable buffer).

	// forward planning pass - recompute all the segments
	i = MP_BUFFER_SIZE;					// put a limit on this to be safe
	while (--i > 0) {
		bf = _mp_get_next_buffer(bf);	// move forward to next buffer
		if (bf->replannable == FALSE) { // the move past the current move 
			break;						// ...will be non-replannable
		}
		(void)_mp_get_segments(bf->pv->exit_velocity,
							   bf->cruise_velocity_set,
							   min4(bf->cruise_velocity,
							   		bf->nx->entry_velocity_limit,
								 	bf->nx->cruise_velocity_set,
									bf->nx->cumulative_braking), 
							   bf->length, bf);
	}
}

/*
 * _mp_get_segments()
 *
 *	This function computes the segment lengths and the velocities:
 *
 *	Computes:
 *		entry_velocity
 *		cruise velocity
 *		head_length
 *		body_length
 *		tail_length
 *
 *	Returns: 
 *		number of segments - 0-3
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
static uint8_t _mp_get_segments(double entry_velocity, 
								double cruise_velocity,
								double exit_velocity, 
								double length,
								struct mpBuffer *bf) 
{
	// ----- setup buffer struct with initial values -----
	bf->length = length;
	bf->entry_velocity = entry_velocity;		// achieved initial velocity
	bf->cruise_velocity = cruise_velocity;		// achieved cruise velocity
	bf->exit_velocity = exit_velocity;			// this one never changes

	// compute optimal head and tail lengths
	bf->head_length = _mp_get_length(entry_velocity, cruise_velocity, mm.jerk_max);
	bf->tail_length = _mp_get_length(cruise_velocity, exit_velocity, mm.jerk_max);
	bf->body_length = bf->length - bf->head_length - bf->tail_length;

	// ----- 0 segment case - line is too short -----
	if (bf->length < MIN_LINE_LENGTH) {	// line is too short or zero
		TRAP1(PSTR("Line too short in _mp_get_segments() len = %f"), bf->length);
		return (0);
	}

	// ----- 3 segment case (HBT) ---- line is long enough that no reduction is required
	if (bf->body_length > 0) {
		// add sub-minimum heads and tails to body length
		if (bf->head_length < MIN_LINE_LENGTH) {
			bf->body_length += bf->head_length;
			bf->head_length = 0;
		}
		if (bf->tail_length < MIN_LINE_LENGTH) {
			bf->body_length += bf->tail_length;
			bf->tail_length = 0;
		}
		return (3);
	}

	// ----- 1 segment less-than-minimum-length cases -----
	// Line length will not support the difference in Ventry & Vexit 
	// Adjust exit velocity to best possible under the circumstances
	double minimum_length = _mp_get_length(entry_velocity, exit_velocity, mm.jerk_max);
	if (length < minimum_length) {
		if (entry_velocity > exit_velocity) {
			bf->head_length = 0;
			bf->body_length = 0;
			bf->tail_length = length;
			bf->exit_velocity = entry_velocity - 
								(length / minimum_length) * 
								(entry_velocity - exit_velocity);
		} else {
			bf->head_length = length;
			bf->body_length = 0;
			bf->tail_length = 0;
			bf->exit_velocity = entry_velocity + 
								(length / minimum_length) *
								(exit_velocity - entry_velocity);
		}
		return (1);		// 1 segment return
	}

	// ----- 1 segment normal cases: H, B & T cases -----
	// B case: all velocities are equal - just a body is required
	if ((fabs(exit_velocity - entry_velocity) < EPSILON) && 
		(fabs(exit_velocity - cruise_velocity) < EPSILON)) {
		bf->head_length = 0;
		bf->tail_length = 0;
		bf->body_length = bf->length;
		return (1);
	}
	// H case: line accelerates but only long enough for a head
	// ++++ Does this case need to do cruise padding similar to the T case? 
//	if ((exit_velocity > entry_velocity) && (bf->length < bf->head_length)) {
	if ((entry_velocity < exit_velocity) && (bf->length <= minimum_length)) {
		bf->head_length = bf->length;
		bf->body_length = 0;
		bf->tail_length = 0;
		bf->cruise_velocity = _mp_get_velocity(bf->entry_velocity_limit, bf->length, mm.jerk_max);
		bf->exit_velocity = bf->cruise_velocity;
		return (1);
	}
	// T case: line decelerates but only long enough for a tail
	// This block computes a tail based on the entry velocity then pads 
	// it with a cruise if necessary to fill out the length of the line.
//	if ((exit_velocity < entry_velocity) && (bf->length < bf->tail_length)) {
	if ((entry_velocity > exit_velocity) && (bf->length <= minimum_length)) {
		bf->head_length = 0;
		bf->tail_length = _mp_get_length(entry_velocity, exit_velocity, mm.jerk_max);
		if ((bf->body_length = bf->length - bf->tail_length) < EPSILON) {
			bf->body_length = 0;
		}
		bf->cruise_velocity = bf->entry_velocity;
		return (1);
	}

	// ----- 2 segment case (HT) -----
	// Successive approximation to find Vc that satisfies length. 
	// Profiles (-Os) for typical convergence conditions are:
	//	length accuracy	iterations	velocity error	~cycles	~uSec
	//		0.01		  9			  0.33% (over)	 40,000	 1250uSec
	//		0.001		  14		  0.04% (over)	 60,000	 1900uSec 
	//		EPSILON		  20		  0.004% (over)	 85,000	 2700uSec 
	uint8_t i=0;
	// break it into 2 cases
	if (entry_velocity > exit_velocity) {
		double adjusted_length = _mp_get_ht_length(entry_velocity, 
										 		   bf->cruise_velocity, 
										 		   exit_velocity, mm.jerk_max);
		do {
	 		bf->cruise_velocity = entry_velocity + (length / adjusted_length) *
								 (bf->cruise_velocity - entry_velocity); 

			adjusted_length = _mp_get_ht_length(entry_velocity, 
												bf->cruise_velocity, 
												exit_velocity, mm.jerk_max);
			if (++i > 50) {
				TRAP(PSTR("_mp_get_segments() approximation failed to converge"));
				break;
			} 
		} while (fabs(bf->length - adjusted_length) > 0.001); // or EPSILON
	} else {
		double adjusted_length = _mp_get_ht_length(entry_velocity, 
										 		   bf->cruise_velocity, 
										 		   exit_velocity, mm.jerk_max);
		do {
	 		bf->cruise_velocity = exit_velocity + (length / adjusted_length) *
								 (bf->cruise_velocity - exit_velocity); 

			adjusted_length = _mp_get_ht_length(entry_velocity, 
												bf->cruise_velocity, 
												exit_velocity, mm.jerk_max);
			if (++i > 50) {
				TRAP(PSTR("_mp_get_segments() approximation failed to converge"));
				break;
			} 
		} while (fabs(bf->length - adjusted_length) > 0.001); // or EPSILON
	}

	if (entry_velocity > exit_velocity) { // which line runs at jerk?
		bf->tail_length = _mp_get_length(exit_velocity, bf->cruise_velocity, mm.jerk_max);
		bf->head_length = bf->length - bf->tail_length;
	} else {
		bf->head_length = _mp_get_length(entry_velocity, bf->cruise_velocity, mm.jerk_max);
		bf->tail_length = bf->length - bf->head_length;
	}
	bf->body_length = 0;
	return (2);		// 2 segment return
}

/*	
 * _mp_get_ht_length()
 *
 *	Get length of a 2 line "trapezoid" given the following parameters:
 *		Ve	- entry velocity		
 *		Vc	- cruise velocity
 *		Vx	- exit velocity
 *		Jm	- maximum  jerk
 *
 *	The following must apply: Ve <= Vc >= Vx  (i.e. it must be a trapezoid)
 */

static double _mp_get_ht_length(const double Ve, const double Vc, const double Vx, const double Jm)
{
	return (((Vc+Ve) * sqrt(fabs(Vc-Ve)/Jm)) + ((Vx+Vc) * sqrt(fabs(Vx-Vc)/Jm)));
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
 *	  time = 2 * sqrt(abs(Vf-Vi) / jerk_max);	// equation 5.x
 *	  length = abs(Vf-Vi) * time / 2;			// equation [2]
 *
 *	Let the compiler optimize out the Vi=0 & Vf=0 constant cases
 */

static double _mp_get_length(const double Vi, const double Vf, const double Jm)
{
//	double deltaV = fabs(Vf-Vi);		// is this really an optimization?
//	return (deltaV * sqrt(deltaV / Jm));
	return (fabs(Vf-Vi) * sqrt(fabs(Vf-Vi) / Jm));
}

/*	
 * _mp_get_velocity()
 *
 * 	A convenient expression for returning the change in velocity possible 
 *	for a given entry velocity (Vi), length (L), and maximum jerk (Jm).
 *	Vi must be less than V. 
 *
 *	  length = (V-Vi) * sqrt((V-Vi)/Jm)
 *
 *	Solved for V:
 *
 *	  V = Jm^(1/3) * length^(2/3) + Vi
 *
 *  http://www.wolframalpha.com/input/?i=L%3D%28X-V%29*sqrt%28%28X-V%29%2FJ%29
 */

static double _mp_get_velocity(const double Vi, const double L, const double Jm)
{
	return (mm.jerk_max_cubert * pow(L,0.6666667) + Vi);
//	return (cubert(Jm) * pow(L,0.6666667) + Vi);
}

/*	
 * _mp_get_join_velocity()
 *
 *  Computes the allowable junction speed by comparing the change in 
 *	velocity for each axis in the move, and comparing that to the maximum
 *	jerk that axis is allowed to sustain given it's jerk limit. 
 *	Returns the "safe" velocity derived from this computation.
 */

static double _mp_get_join_velocity(const double a_unit[], const double b_unit[], 
									const double a_velocity, const double b_velocity)
{
	double Va, Vb;					// A and B velocities along axis
//	double Vm;						// intermediate velocity
	double dV;						// change in velocity - deltaV
	double dVlimit;					// maximum delta V for the velocity 
	double b_vel = b_velocity;		// actual b velocity used in function
	double Vfactor = 1;				// velocity adjustment factor

	if (a_velocity < b_velocity) { 	// reduce b_velocity if b > a
		b_vel = a_velocity; 		// acceleration happens in B line
	}
	for (uint8_t i=0; i<AXES; i++) {
		if ((fabs(a_unit[i]) < EPSILON) && (fabs(b_unit[i]) < EPSILON)) {
			continue;				// skip idle axes
		}
		// remember: everything below are vector values - i.e. per-axis
		Va = a_unit[i] * a_velocity;// proposed velocity entering the turn
		Vb = b_unit[i] * b_vel;		// proposed velocity leaving the turn
		dV = fabs(Va - Vb);			// mangnitude of velocity difference
		Va = fabs(Va);				// we just want magnitude from here on
		if (dV < EPSILON) {			// no reduction is required
			continue;
		}
		if (dV > cfg.a[i].jerk_stall_velocity) {// full reduction to zero is required
			return (0);				// so return now with zero
		}
		dVlimit = mp_get_jerk_limit(Va, cfg.a[i].jerk_transition_size, cfg.a[i].jerk_max);
 		if (dV < dVlimit) {
			continue;				// no reduction required
		}
//		Vm = Va - dV + dVlimit;
//		Vfactor = min(Vfactor, Vm/Va);
		Vfactor = min(Vfactor, (Va - dV + dVlimit)/Va);
	}
	return (a_velocity * Vfactor);
}

/*	
 * mp_get_jerk_limit()
 *
 *	This function will tell you how much delta V you can have in a 
 *	junction while still staying below the jerk limit (for the axis).
 *	based on the following parameters:
 *
 *	  St - transition region for an axis
 *	  Jm - maximum jerk for an axis
 *	  Vi - velocity entering the transition region
 *
 *	The above parameters can be scalar (applying to all axes) or vector
 *	(applying to a single axis), honey badger don't care. For best results
 *	this function should be called for each axis (i.e. vectorized).
 *
 *	The transition region St is the diameter of an empirical circle in 
 *	which the velocity changes. In a cornering case this means that this 
 *	is the circle in which the direction change occurs and axes either 
 *	shed velocity or pick up velocity according to the difference in their 
 *	unit vector terms. The circle accounts for the fact that real machines 
 *	do not instantaneously change direction. If they did the jerk would be 
 *	infinite. This circle can be used to determine the diameter of the 
 *	splining region if you want to reduce the jerk by rounding corners.
 *
 *	The velocity returned when Vi=0 is the maximum jerk limit. Entering 
 *	with any velocity > 0 will return a smaller limit. 
 *
 *	The equations below are real solutions to the equation of motion:
 *
 *		V = Vi + Jm(T^2)/2 where (S/V) is substituted for T 
 *
 *		V = (St*sqrt(Jm*(8*Vi^3+27*Jm*St^2))/(4*3^(3/2))+(4*Vi^3+27*Jm*St^2)/108)^(1/3)+Vi^2/
 *		 (9*(St*sqrt(Jm*(8*Vi^3+27*Jm*St^2))/(4*3^(3/2))+(4*Vi^3+27*Jm*St^2)/108)^(1/3))+Vi/3
 * 		http://www.quickmath.com/webMathematica3/quickmath/page.jsp?s1=equations&s2=solve&s3=basic#v1=V%3DU%2BJ*((S%2FV)%5E2)%2F2&v2=V
 *		http://www.wolframalpha.com/input/?i=V%3DU%2BJ*%28%28S%2FV%29%5E2%29%2F2
 *
 * 	Profiled: What's ~3000 cycles (~100 uSec) between friends? (-Os)
 */
/* Notes: When setting the size of the circle the max limit should 
 *	never exceed the instantaneous acceleration sustainable by that axis.
 *	Otherwise the motor will stall when trying to accelerate from a zero
 *	velocity case. You can find the max acceleration limit for an axis 
 *	experimentally by turning off acceleration and trying to get motor to 
 *	start at various speeds (e.g. G1 F300 x10) until it won't start. 
 *	Then reduce the size of the transition region until it yields a value
 *	lower than the stall velocity for the given jerk value for that axis.
 *
 *	The other condition to test is the maximum velocity at which an un-
 *	accelerated 180 degree reversal can take place.
 */

#define kJVL  (20.7846096908265)
#define kJVL2 (5.19615242270663)

double mp_get_jerk_limit(const double Vi, const double St, const double Jm)
{
//	return (cfg.a[X].jerk_stall_velocity);		/// TEST VALUE

	// jerk limit solved for V(exit) to size the region
	double V = fabs(Vi);
	double a = 27*Jm*square(St);
	double b = 4*cube(V);
	double c = St*sqrt(Jm*(2*b+a));
	double d = cubert((c/kJVL)+(b+a)/108);
	return (d + square(V)/(9*d)+V/3) - V; 

/*
	// jerk limit solved for average of Vi and V(exit) to size the region
	double V = fabs(Vi);
	double a = 27*Jm*square(St);
	double b = 8*cube(V);
	double c = St*sqrt(Jm*(2*b+a));
	double d = cubert((c/kJVL2)+(b+a)/27);
	return (d + 4*square(V)/(9*d)-V/3) - V; 
*/
}

/*	
 * mp_get_jerk_transition_size()
 *
 *	Uses the stall velocity and the jerk_max setting to return the 
 *	effective size of the jerk transition region.
 *
 *		V = Vi + Jm*(T^2)/2				Original equation of motion
 *		V = Jm*(T^2)/2					Vi is zero for this case	
 *		V = Jm*((S/V)^2)/2				Substitute S/V for T	
 *		S = sqrt(2)*V*sqrt(V/Jm)		Solution from Quickmath	
 */

double mp_get_jerk_transition_size(const double stall_velocity, const double Jm)
{
	return (RADICAL2 * stall_velocity * sqrt(stall_velocity/Jm));
}

/*****************************/
/**** ALINE RUN ROUTINES *****/
/*****************************
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
 *	T = (sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3)+A^2/
 * (J^2*(sqrt(3)*sqrt(3*J^2*S^2+(-6*H*J^2-2*A^3)*S+3*H^2*J^2+2*A^3*H)/J^2+(-3*J^2*S+3*H*J^2+A^3)/J^3)^(1/3))+A/J
 *
 *  Note: A cruise is supposed to be guaranteed to have a non-zero end 
 *		  velocity, otherwise the time spent in the cruise is infinite. 
 *		  Zero velocity cruises are detected and rejected.
 */

static uint8_t _mp_run_aline(struct mpBuffer *bf)
{
	// preliminary tests on all lines 
	if (mq_test_motor_buffer() == FALSE) { 		// block on motor buffer queue
		return (TG_EAGAIN); 
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

// initialize acceleration phase
static uint8_t _mp_run_accel_0(struct mpBuffer *bf) 
{
	// general inits for the move
	bf->replannable = FALSE;					// stop replanning
	if (bf->length < MIN_LINE_LENGTH) {
		return (TG_OK);							// toss move & free buffer
	}
	// inits specific to acceleration
	if (bf->head_length < MIN_LINE_LENGTH) {
		bf->move_state = MP_STATE_CRUISE;
		return (_mp_run_cruise(bf));
	}
	mr.midpoint_velocity = (bf->entry_velocity + bf->cruise_velocity) / 2;
	TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Accel Midpoint Velocity: %f"), mr.midpoint_velocity)	
	mr.time = bf->head_length / mr.midpoint_velocity;
	mr.midpoint_acceleration = mr.time * mr.jerk_max_div2;
	for (uint8_t i=0; i < AXES; i++) {
		mr.target[i] = bf->target[i];	// transfer target to mr (WHY???)
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
	bf->move_state = MP_STATE_ACCEL_1;
	return (_mp_run_accel_1(bf));			// first time through
}

// first half of acceleration - concave portion of curve
static uint8_t _mp_run_accel_1(struct mpBuffer *bf) 
{
	mr.segment_velocity = bf->entry_velocity + (mr.jerk_max_div2 * square(mr.elapsed_time));
	ritorno (_mp_run_segment(bf)); // returns if not done (this is not an error)
	// setup for second half
	mr.segment_count = (uint32_t)mr.segments;
	mr.elapsed_time = mr.segment_time / 2;
	bf->move_state = MP_STATE_ACCEL_2;
	return (_mp_run_accel_2(bf));
}

// second half of acceleration - convex portion of curve
static uint8_t _mp_run_accel_2(struct mpBuffer *bf) 
{
	if (mr.segment_count > 1) {
		mr.segment_velocity = mr.midpoint_velocity + 
							 (mr.elapsed_time * mr.midpoint_acceleration) -
							 (mr.jerk_max_div2 * square(mr.elapsed_time));
		return(_mp_run_segment(bf));
	} else {
		bf->move_state = MP_STATE_CRUISE;
		_mp_run_finalize(bf);		// for accuracy
		return(TG_EAGAIN);			// not done yet
	}
	TRAP(PSTR("Abnormal exit from _mp_run_accel_2()"));
	return (TG_ERR);			// shouldn't happen
}

// initialize and run the cruise
static uint8_t _mp_run_cruise(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];

	if ((bf->body_length < MIN_LINE_LENGTH) || 	// various end condidions
		(bf->cruise_velocity < EPSILON)) {
		bf->move_state = MP_STATE_DECEL_0;
		return (_mp_run_decel_0(bf));
	}
	bf->time = bf->body_length / bf->cruise_velocity;
	TRAP_IF_TRUE((bf->time == 0), PSTR("Time: %f"), bf->time)
	mr.microseconds = uSec(bf->time);

	for (i=0; i < AXES; i++) {
		mr.target[i] = bf->target[i];
		bf->target[i] = mr.position[i] + bf->unit[i] * bf->length; //++++ remove this line for test
		travel[i] = bf->target[i] - mr.position[i];
	}
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	_mp_set_mr_position(bf->target);
	return (TG_EAGAIN);
}

// initialize deceleration phase
static uint8_t _mp_run_decel_0(struct mpBuffer *bf)
{
	if (bf->tail_length < MIN_LINE_LENGTH) {
		return (TG_OK);					// end the move & free the buffer
	}
	mr.midpoint_velocity = (bf->cruise_velocity + bf->exit_velocity) / 2;
	TRAP_IF_TRUE((mr.midpoint_velocity == 0), PSTR("Decel Midpoint Velocity: %f"), mr.midpoint_velocity)	
	mr.time = bf->tail_length / mr.midpoint_velocity;
	mr.midpoint_acceleration = mr.time * mr.jerk_max_div2;
	for (uint8_t i=0; i < AXES; i++) {
		mr.target[i] = bf->target[i];	// transfer target
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
	bf->move_state = MP_STATE_DECEL_1;
	return (_mp_run_decel_1(bf));
}

// first half of deceleration - convex portion of curve
static uint8_t _mp_run_decel_1(struct mpBuffer *bf) 
{
	mr.segment_velocity = bf->entry_velocity - 
						 (mr.jerk_max_div2 * square(mr.elapsed_time));
	ritorno(_mp_run_segment(bf));	// return is OK, not an error
	// setup for second half
	mr.segment_count = (uint32_t)mr.segments;
	mr.elapsed_time = mr.segment_time / 2;
	bf->move_state = MP_STATE_DECEL_2;
	return (_mp_run_decel_2(bf));
}

// second half of deceleration - concave portion of curve
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

// segment runner helper
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
		bf->target[i] = mr.position[i] + (bf->unit[i] * 
					   mr.segment_velocity * mr.segment_time);
		travel[i] = bf->target[i] - mr.position[i];
	}
	// queue the line and adjust the variables for the next iteration
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	mr.elapsed_time += mr.segment_time;
	_mp_set_mr_position(bf->target);
	if (--mr.segment_count > 0) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

// last segment runner helper
static void _mp_run_finalize(struct mpBuffer *bf)
{
	uint8_t i;
	double travel[AXES];
	double steps[MOTORS];
	
	// finalize - do the last segment to maintain position accuracy
	mr.length = mp_get_axis_vector_length(mr.target, mr.position);
	if ((mr.length < MIN_LINE_LENGTH) || (bf->exit_velocity < EPSILON)) {
		return; 			// trap zero-length cases
	}
	mr.time = mr.length / bf->exit_velocity;
	mr.microseconds = uSec(mr.time);

	for (i=0; i < AXES; i++) {
		travel[i] = mr.target[i] - mr.position[i];
	}
	(void)ik_kinematics(travel, steps, mr.microseconds);
	(void)mq_queue_line(steps, mr.microseconds);	
	_mp_set_mr_position(mr.target);
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
//	_mp_test_get_join_velocity();
	_mp_test_get_segments();
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
	bf->entry_velocity_limit = Ve;
	bf->cruise_velocity_set = Vc;
	bf->exit_velocity_limit = Vx;
	_mp_get_segments(Ve, Vc, Vx, length, bf);
}

void _mp_test_get_segments()
{
	struct mpBuffer *bf = _mp_get_write_buffer();

	// these tests are calibrated to indicated stall_velocity & Jm
	_mp_setup_jerk(200, 50000000);	    // set stall velocity & Jm
//				   Ventry  Vcruise Vexit   Length
//	_mp_test_segments(0,	1,		0,		1, 	bf);	// line < min length
//	_mp_test_segments(0, 	400,	0,		1, 	bf);	// HT line

	// line < minimum cases
//	_mp_test_segments(400,	400, 	0, 		0.8, bf);
//	_mp_test_segments(600,	600, 	200,	0.8, bf);
//	_mp_test_segments(0,	400, 	400,	0.8, bf);
//	_mp_test_segments(200,	600, 	600,	0.8, bf);

	// 2 line cases (HT)
//	_mp_test_segments(0,	400, 	0, 		0.8, bf);
//	_mp_test_segments(200,	400, 	0, 		0.8, bf);
	_mp_test_segments(0,	400, 	200,	0.8, bf);


	_mp_test_segments(0,	400, 	0, 		2.0, bf);
	_mp_test_segments(200,	400, 	0, 		2.0, bf);
	_mp_test_segments(400,	400, 	0, 		2.0, bf);

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
	// See "Angles" tab in tangential JERK spreadsheet for description
	//	of the following cases. Three tests are done for each case:
	//		a) velocity exceeds max velocity
	//		b) velocity below max velocity but above limit velocity
	//		c) velocity below limit velocity
/*
	cfg.a[X].jerk_max = JERK_TEST_VALUE;
	cfg.a[Y].jerk_max = JERK_TEST_VALUE;
	cfg.a[Z].jerk_max = JERK_TEST_VALUE;
	cfg.a[A].jerk_max = JERK_TEST_VALUE;
	cfg.a[B].jerk_max = JERK_TEST_VALUE;
	cfg.a[C].jerk_max = JERK_TEST_VALUE;
//	mm.jerk_transition_size = 0.5;
//	mm.jerk_limit_max = 184.2;

// Case 1: Lines A and B make a straight, oblique line
	mm.test_case = 1;
	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 200, 200);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 150, 150);

	_mp_make_unit_vector(mm.a_unit, 8.666, 5, 0, 0, 0, 0);
	_mp_make_unit_vector(mm.unit,   8.666, 5, 0, 0, 0, 0);
	mm.test_velocity = _mp_get_join_velocity(mm.a_unit, mm.unit, 80, 80);

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
*/
}

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
