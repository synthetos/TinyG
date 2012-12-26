/*
 * planner.c - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
/* --- Planner Notes ----
 *
 *	The planner works below the canonical machine and above the motor mapping 
 *	and stepper execution layers. A rudimentary multitasking capability is 
 *	implemented for long-running commands such as lines, arcs, and dwells. 
 *	These functions are coded as non-blocking continuations - which are simple 
 *	state machines that are re-entered multiple times until a particular 
 *	operation is complete. These functions have 2 parts - the initial call, 
 *	which sets up the local context, and callbacks (continuations) that are 
 *	called from the main loop (in controller.c).
 *
 *	One important concept is isolation of the three layers of the data model - 
 *	the Gcode model (gm), planner model (bf queue & mm), and runtime model (mr).
 *	These are designated as "model", "planner" and "runtime" in function names.
 *
 *	The Gcode model is owned by the canonical machine and should only be accessed
 *	by cm_xxxx() functions. Data from the Gcode model is transferred to the planner
 *	by the mp_xxx() functions called by the canonical machine. 
 *
 *	The planner should only use data in the planner model. When a move (block) 
 *	is ready for execution the planner data is transferred to the runtime model, 
 *	which should also be isolated.
 *
 *	Lower-level models should never use data from upper-level models as the data 
 *	may have changed and lead to unpredictable results.
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
#include "plan_arc.h"
#include "plan_line.h"
#include "planner.h"
#include "spindle.h"
#include "kinematics.h"
#include "stepper.h"
#include "report.h"
#include "util.h"
#include "test.h"
#include "xio/xio.h"			// supports trap and debug statements

/*
 * Local Scope Data and Functions
 */
#define _bump(a) ((a<PLANNER_BUFFER_POOL_SIZE-1)?(a+1):0) // buffer incr & wrap
#define spindle_speed time		// local alias for spindle_speed to the time variable
#define int_val move_code		// local alias for uint8_t to the move_code
#define dbl_val time			// local alias for double to the time variable

// execution routines (NB: These are all called from the LO interrupt)
static uint8_t _exec_dwell(mpBuf *bf);
static uint8_t _exec_command(mpBuf *bf);

#ifdef __DEBUG
static uint8_t _get_buffer_index(mpBuf *bf); 
static void _dump_plan_buffer(mpBuf *bf);
#endif

/* 
 * mp_init()
 */

void mp_init()
{
// You can assume all memory has been zeroed by a hard reset. If not, use this code:
//	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
//	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status
	mp_init_buffers();
}

/* 
 * mp_flush_planner() - flush all moves in the planner and all arcs
 *
 *	Does not affect the move currently running in mr.
 *	This function is designed to be called during a hold to reset the planner
 *	and is also useful for jogs and other console-driven commands
 */

void mp_flush_planner()
{
	ar_abort_arc();
	mp_init_buffers();
	cm.motion_state = MOTION_STOP;
//	cm_exec_program_stop();
}

/*
 * mp_set_plan_position() 	- sets planning position (for G92)
 * mp_get_plan_position() 	- returns planning position
 * mp_set_axis_position() 	- sets both planning and runtime positions (for G2/G3)
 *
 *
 * 	Keeping track of position is complicated by the fact that moves can
 *	require multiple reference frames. The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for planning
 *	 - mr.position	- current position of runtime segment
 *	 - mr.target	- target position of runtime segment
 *	 - mr.endpoint	- final target position of runtime segment
 *
 *	Note that the positions are set immediately when they are computed and 
 *	are not an accurate representation of the tool position. In reality 
 *	the motors will still be processing the action and the real tool 
 *	position is still close to the starting point.
 */

double *mp_get_plan_position(double position[])
{
	copy_axis_vector(position, mm.position);	
	return (position);
}

void mp_set_plan_position(const double position[])
{
	copy_axis_vector(mm.position, position);
}

void mp_set_axes_position(const double position[])
{
	copy_axis_vector(mm.position, position);
	copy_axis_vector(mr.position, position);
}

void mp_set_axis_position(uint8_t axis, const double position)
{
	mm.position[axis] = position;
	mr.position[axis] = position;
}

/*************************************************************************/
/* mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 */

uint8_t mp_exec_move() 
{
	mpBuf *bf;

	if ((bf = mp_get_run_buffer()) == NULL) return (TG_NOOP);	// NULL means nothing's running

	// Manage cycle and motion state transitions
	// cycle auto-start for lines only. Add other move types as appropriate.
	if (bf->move_type == MOVE_TYPE_ALINE) {
		if (cm.cycle_state == CYCLE_OFF) cm_cycle_start();
	}
	if ((cm.motion_state == MOTION_STOP) && (bf->move_type == MOVE_TYPE_ALINE)) {
		cm.motion_state = MOTION_RUN;
	}

	// run the move callback in the buffer
	if (bf->bf_func != NULL) {
		return (bf->bf_func(bf));
	}
	return (TG_INTERNAL_ERROR);		// never supposed to get here
}

/************************************************************************************
 * mp_queue_command() - queue a synchronous Mcode, program control, or other command
 *
 *	How this works:
 *	  - The command is called by the Gcode interpreter (cm_<command>, e.g. an M code)
 *	  - cm_ function calls mp_queue_command which puts it in the planning queue.
 *		This involves setting some parameters and registering a callback to the 
 *		execution function in the canonical machine
 *	  - the planning queue gets to the function and calls _exec_command()
 *	  - ...which passes the saved parameters to the callback function
 *	  - To finish up _exec_command() needs to run a null pre and free the planner buffer
 *
 *	Doing it this way instead of synchronizing on queue empty simplifies the
 *	handling of feedholds, feed overrides, buffer flushes, and thread blocking,
 *	and makes keeping the queue full much easier - therefore avoiding Q starvation
 */

void mp_queue_command(void(*cm_exec)(uint8_t, double), uint8_t i, double f)
{
	mpBuf *bf;

	// this error is not reported as buffer availability was checked upstream in the controller
	if ((bf = mp_get_write_buffer()) == NULL) return;

	bf->move_type = MOVE_TYPE_COMMAND;
	bf->bf_func = _exec_command;		// callback to planner queue exec function
	bf->cm_func = cm_exec;				// callback to canonical machine exec function
	bf->int_val = i;
	bf->dbl_val = f;
	mp_queue_write_buffer(MOVE_TYPE_COMMAND);
	return;
}

static uint8_t _exec_command(mpBuf *bf)
{
	bf->cm_func(bf->int_val, bf->dbl_val);
	st_prep_null();			// Must call a null prep to keep the loader happy. 
	mp_free_run_buffer();
	return (TG_OK);
}

/*************************************************************************
 * mp_dwell() 	 - queue a dwell
 * _exec_dwell() - dwell continuation
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the swell on a separate 
 * timer than the stepper pulse timer.
 */

uint8_t mp_dwell(double seconds) 
{
	mpBuf *bf; 

	if ((bf = mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (TG_BUFFER_FULL_FATAL);		  	// (not supposed to fail)
	}
	bf->bf_func = _exec_dwell;					// register the callback to the exec function
	bf->time = seconds;						  	// in seconds, not minutes
	mp_queue_write_buffer(MOVE_TYPE_DWELL);
	return (TG_OK);
}

static uint8_t _exec_dwell(mpBuf *bf)
{
	st_prep_dwell((uint32_t)(bf->time * 1000000));// convert seconds to uSec
	mp_free_run_buffer();
	return (TG_OK);
}

/**** PLANNER BUFFERS *****************************************************
 *
 * Planner buffers are used to queue and operate on Gcode blocks. Each buffer 
 * contains one Gcode block which may be a move, and M code, or other command 
 * that must be executed synchronously with movement.
 *
 * Buffers are in a circularly linked list managed by a WRITE pointer and a RUN pointer.
 * New blocks are populated by (1) getting a write buffer, (2) populating the buffer,
 * then (3) placing it in the queue (queue write buffer). If an exception occurs
 * during population you can unget the write buffer before queuing it, which returns
 * it to the pool of available buffers.
 *
 * The RUN buffer is the buffer currently executing. It may be retrieved once for 
 * simple commands, or multiple times for long-running commands like moves. When 
 * the command is complete the run buffer is returned to the pool by freeing it.
 * 
 * Notes:
 *	The write buffer pointer only moves forward on _queue_write_buffer, and
 *	the read buffer pointer only moves forward on free_read calls.
 *	(test, get and unget have no effect)
 * 
 * mp_get_planner_buffers_available()   Returns # of available planner buffers
 *
 * mp_init_buffers()		Initializes or resets buffers
 *
 * mp_get_write_buffer()	Get pointer to next available write buffer
 *							Returns pointer or NULL if no buffer available.
 *
 * mp_unget_write_buffer()	Free write buffer if you decide not to queue it.
 *
 * mp_queue_write_buffer()	Commit the next write buffer to the queue
 *							Advances write pointer & changes buffer state
 *
 * mp_get_run_buffer()		Get pointer to the next or current run buffer
 *							Returns a new run buffer if prev buf was ENDed
 *							Returns same buf if called again before ENDing
 *							Returns NULL if no buffer available
 *							The behavior supports continuations (iteration)
 *
 * mp_free_run_buffer()		Release the run buffer & return to buffer pool.
 *
 * mp_get_prev_buffer(bf)	Returns pointer to prev buffer in linked list
 * mp_get_next_buffer(bf)	Returns pointer to next buffer in linked list 
 * mp_get_first_buffer(bf)	Returns pointer to first buffer, i.e. the running block
 * mp_get_last_buffer(bf)	Returns pointer to last buffer, i.e. last block (zero)
 * mp_clear_buffer(bf)		Zeroes the contents of the buffer
 * mp_copy_buffer(bf,bp)	Copies the contents of bp into bf - preserves links
 */

uint8_t mp_get_planner_buffers_available(void) { return (mb.buffers_available);}

void mp_init_buffers(void)
{
	mpBuf *pv;
	uint8_t i;

	memset(&mb, 0, sizeof(mb));		// clear all values, pointers and status
	mb.w = &mb.bf[0];				// init write and read buffer pointers
	mb.q = &mb.bf[0];
	mb.r = &mb.bf[0];
	pv = &mb.bf[PLANNER_BUFFER_POOL_SIZE-1];
	for (i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) { // setup ring pointers
		mb.bf[i].nx = &mb.bf[_bump(i)];
		mb.bf[i].pv = pv;
		pv = &mb.bf[i];
	}
	mb.buffers_available = PLANNER_BUFFER_POOL_SIZE;
}

mpBuf * mp_get_write_buffer() 				// get & clear a buffer
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
		mpBuf *w = mb.w;
		mpBuf *nx = mb.w->nx;					// save pointers
		mpBuf *pv = mb.w->pv;
		memset(mb.w, 0, sizeof(mpBuf));
		w->nx = nx;								// restore pointers
		w->pv = pv;
		w->buffer_state = MP_BUFFER_LOADING;
		mb.buffers_available--;
		mb.w = w->nx;
		return (w);
	}
	return (NULL);
}
/* NOT USED
void mp_unget_write_buffer()
{
	mb.w = mb.w->pv;							// queued --> write
	mb.w->buffer_state = MP_BUFFER_EMPTY; 		// not loading anymore
	mb.buffers_available++;
}
*/
void mp_queue_write_buffer(const uint8_t move_type)
{
	mb.q->move_type = move_type;
	mb.q->move_state = MOVE_STATE_NEW;
	mb.q->buffer_state = MP_BUFFER_QUEUED;
	mb.q = mb.q->nx;							// advance the queued buffer pointer
	st_request_exec_move();						// request a move exec if not busy
}

mpBuf * mp_get_run_buffer() 
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

void mp_free_run_buffer()					// EMPTY current run buf & adv to next
{
	mp_clear_buffer(mb.r);						// clear it out (& reset replannable)
	mb.r->buffer_state = MP_BUFFER_EMPTY;
	mb.r = mb.r->nx;							 // advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {// only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING;  // pend next buffer
	}
	if (mb.w == mb.r) cm_cycle_end();			// end the cycle if the queue empties
	mb.buffers_available++;
	rpt_request_queue_report();
}

mpBuf * mp_get_first_buffer(void)
{
	return(mp_get_run_buffer());	// returns buffer or NULL if nothing's running
}

mpBuf * mp_get_last_buffer(void)
{
	mpBuf *bf = mp_get_run_buffer();
	mpBuf *bp = bf;

	if (bf == NULL) { return(NULL);}

	do {
		if ((bp->nx->move_state == MOVE_STATE_OFF) || (bp->nx == bf)) { 
			return (bp); 
		}
	} while ((bp = mp_get_next_buffer(bp)) != bf);
	return (bp);
}

// Use the macro instead
//mpBuf * mp_get_prev_buffer(const mpBuf *bf) { return (bf->pv);}
//mpBuf * mp_get_next_buffer(const mpBuf *bf) { return (bf->nx);}

void mp_clear_buffer(mpBuf *bf) 
{
	mpBuf *nx = bf->nx;	// save pointers
	mpBuf *pv = bf->pv;
	memset(bf, 0, sizeof(mpBuf));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

void mp_copy_buffer(mpBuf *bf, const mpBuf *bp)
{
	mpBuf *nx = bf->nx;	// save pointers
	mpBuf *pv = bf->pv;
 	memcpy(bf, bp, sizeof(mpBuf));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

#ifdef __DEBUG	// currently this routine is only used by debug routines
uint8_t mp_get_buffer_index(mpBuf *bf) 
{
	mpBuf *b = bf;		// temp buffer pointer

	for (uint8_t i=0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
		if (b->pv > b) {
			return (i);
		}
		b = b->pv;
	}
	return (PLANNER_BUFFER_POOL_SIZE);	// should never happen
}
#endif

//####################################################################################
//##### UNIT TESTS AND DEBUG CODE ####################################################
//####################################################################################

/****** DEBUG Code ******	(see beginning of file for static function prototypes) */

#ifdef __DEBUG
void mp_dump_running_plan_buffer() { _dump_plan_buffer(mb.r);}
void mp_dump_plan_buffer_by_index(uint8_t index) { _dump_plan_buffer(&mb.bf[index]);	}

static void _dump_plan_buffer(mpBuf *bf)
{
	fprintf_P(stderr, PSTR("***Runtime Buffer[%d] bstate:%d  mtype:%d  mstate:%d  replan:%d\n"),
			_get_buffer_index(bf),
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
//	print_scalar(PSTR("accel_time;        "), mr.accel_time);
//	print_scalar(PSTR("elapsed_accel_time:"), mr.elapsed_accel_time);
	print_scalar(PSTR("midpoint_velocity: "), mr.midpoint_velocity);
//	print_scalar(PSTR("midpoint_accel:    "), mr.midpoint_acceleration);
//	print_scalar(PSTR("jerk_div2:         "), mr.jerk_div2);

	print_scalar(PSTR("segments:          "), mr.segments);
	print_scalar(PSTR("segment_count:     "), mr.segment_count);
	print_scalar(PSTR("segment_move_time: "), mr.segment_move_time);
//	print_scalar(PSTR("segment_accel_time:"), mr.segment_accel_time);
	print_scalar(PSTR("microseconds:      "), mr.microseconds);
	print_scalar(PSTR("segment_length:	  "), mr.segment_length);
	print_scalar(PSTR("segment_velocity:  "), mr.segment_velocity);
}
#endif // __DEBUG

/****** UNIT TESTS ******/

#ifdef __UNIT_TEST_PLANNER

//#define JERK_TEST_VALUE (double)50000000	// set this to the value in the profile you are running
#define JERK_TEST_VALUE (double)100000000	// set this to the value in the profile you are running

static void _test_calculate_trapezoid(void);
static void _test_get_junction_vmax(void);
static void _test_trapezoid(double length, double Ve, double Vt, double Vx, mpBuf *bf);
static void _make_unit_vector(double unit[], double x, double y, double z, double a, double b, double c);
//static void _set_jerk(const double jerk, mpBuf *bf);

void mp_unit_tests()
{
	_test_calculate_trapezoid();
//	_test_get_junction_vmax();
}

static void _test_trapezoid(double length, double Ve, double Vt, double Vx, mpBuf *bf)
{
	bf->length = length;
	bf->entry_velocity = Ve;
	bf->cruise_velocity = Vt;
	bf->exit_velocity = Vx;
	bf->cruise_vmax = Vt;
	bf->jerk = JERK_TEST_VALUE;
#ifdef __PLAN_R2
	TODO_make_this_work();
#else
	bf->recip_jerk = 1/bf->jerk;
	bf->cbrt_jerk = cbrt(bf->jerk);
#endif
	_calculate_trapezoid(bf);
}

static void _test_calculate_trapezoid()
{
	mpBuf *bf = _get_write_buffer();

// these tests are calibrated the following parameters:
//	jerk_max 				50 000 000		(all axes)
//	jerk_corner_offset		   		 0.1	(all exes)
//	jerk_corner_acceleration   200 000		(global)

/*
// no-fit cases: line below minimum velocity or length
//				   	L	 Ve  	Vt		Vx
	_test_trapezoid(1.0, 0,		0.001,	0,	bf);
	_test_trapezoid(0.0, 0,		100,	0,	bf);
	_test_trapezoid(0.01, 0,	100,	0,	bf);

// requested-fit cases
//				   	L  	 Ve  	Vt		Vx
	_test_trapezoid(0.8, 400,	400, 	0, 	 bf);
	_test_trapezoid(0.8, 600,	600, 	200, bf);
	_test_trapezoid(0.8, 0,		400, 	400, bf);
	_test_trapezoid(0.8, 200,	600, 	600, bf);

// HBT - 3 section cases
//				   	L    Ve  	Vt		Vx
	_test_trapezoid(0.8, 0,		190, 	0, bf);
	_test_trapezoid(2.0, 200,	400, 	0, bf);

// 2 section cases (HT)
//				   	L   Ve  	Vt		Vx
	_test_trapezoid(0.8, 0,		200, 	0, bf);		// requested fit HT case (exact fit)
	_test_trapezoid(0.8, 0,		400, 	0, bf);		// symmetric rate-limited HT case
	_test_trapezoid(0.8, 200,	400, 	0, bf);		// asymmetric rate-limited HT case
	_test_trapezoid(2.0, 400,	400, 	0, bf);
	_test_trapezoid(0.8, 0,		400, 	200,bf);

// 1 section cases (H,B and T)
//				   	L	 Ve  	Vt		Vx
	_test_trapezoid(1.0, 800,	800, 	800,bf);	// B case
	_test_trapezoid(0.8, 0,		400, 	0, bf);		// B case
	_test_trapezoid(0.8, 200,	400, 	0, bf);
	_test_trapezoid(2.0, 400,	400, 	0, bf);
	_test_trapezoid(0.8, 0,		400, 	200,bf);
*/
// test cases drawn from Mudflap
//				   	L		Ve  	  Vt		Vx
//	_test_trapezoid(0.6604, 000.000,  800.000,  000.000, bf);	// line 50
//	_test_trapezoid(0.8443, 000.000,  805.855,  000.000, bf);	// line 55
	_test_trapezoid(0.8443, 000.000,  805.855,  393.806, bf);	// line 55'
	_test_trapezoid(0.7890, 393.805,  955.829,  000.000, bf);	// line 60
	_test_trapezoid(0.7890, 393.806,  955.829,  390.294, bf);	// line 60'
	_test_trapezoid(0.9002, 390.294,  833.884,  000.000, bf);	// line 65

	_test_trapezoid(0.9002, 390.294,  833.884,  455.925, bf);	// line 65'
	_test_trapezoid(0.9002, 390.294,  833.884,  806.895, bf);	// line 65"
	_test_trapezoid(0.9735, 455.925,  806.895,  000.000, bf);	// line 70
	_test_trapezoid(0.9735, 455.925,  806.895,  462.101, bf);	// line 70'

	_test_trapezoid(0.9735, 806.895,  806.895,  802.363, bf);	// line 70"

	_test_trapezoid(0.9935, 462.101,  802.363,  000.000, bf);	// line 75
	_test_trapezoid(0.9935, 462.101,  802.363,  000.000, bf);	// line 75'
	_test_trapezoid(0.9935, 802.363,  802.363,  477.729, bf);	// line 75"
	_test_trapezoid(0.9935, 802.363,  802.363,  802.363, bf);	// line 75"
	_test_trapezoid(1.0441, 477.729,  843.274,  000.000, bf);	// line 80
	_test_trapezoid(1.0441, 802.363,  843.274,  388.515, bf);	// line 80'
	_test_trapezoid(1.0441, 802.363,  843.274,  803.990, bf);	// line 80"
	_test_trapezoid(0.7658, 388.515,  803.990,  000.000, bf);	// line 85
	_test_trapezoid(0.7658, 803.990,  803.990,  733.618, bf);	// line 85'
	_test_trapezoid(0.7658, 803.990,  803.990,  802.363, bf);	// line 85"
	_test_trapezoid(1.9870, 733.618,  802.363,  000.000, bf);	// line 90
	_test_trapezoid(1.9870, 802.363,  802.363,  727.371, bf);	// line 90'
	_test_trapezoid(1.9870, 802.363,  802.363,  802.363, bf);	// line 90'
	_test_trapezoid(1.9617, 727.371,  802.425,  000.000, bf);	// line 95
	_test_trapezoid(1.9617, 727.371,  802.425,  000.000, bf);	// line 95'
	_test_trapezoid(1.9617, 802.363,  802.425,  641.920, bf);	// line 95"
	_test_trapezoid(1.9617, 802.363,  802.425,  802.425, bf);	// line 95"'
	_test_trapezoid(1.6264, 641.920,  826.209,  000.000, bf);	// line 100
	_test_trapezoid(1.6264, 802.425,  826.209,  266.384, bf);	// line 100'
	_test_trapezoid(1.6264, 802.425,  826.209,  658.149, bf);	// line 100"
	_test_trapezoid(1.6264, 802.425,  826.209,  679.360, bf);	// line 100"'
	_test_trapezoid(0.4348, 266.384,  805.517,  000.000, bf);	// line 105
	_test_trapezoid(0.4348, 658.149,  805.517,  391.765, bf);	// line 105'
	_test_trapezoid(0.4348, 679.360,  805.517,  412.976, bf);	// line 105"
	_test_trapezoid(0.7754, 391.765,  939.343,  000.000, bf);	// line 110
	_test_trapezoid(0.7754, 412.976,  939.343,  376.765, bf);	// line 110'
	_test_trapezoid(0.7754, 802.425,  826.209,  679.360, bf);	// line 110"
	_test_trapezoid(0.7754, 412.976,  939.343,  804.740, bf);	// line 110"'
	_test_trapezoid(0.7313, 376.765,  853.107,  000.000, bf);	// line 115
	_test_trapezoid(0.7313, 804.740,  853.107,  437.724, bf);	// line 115'
	_test_trapezoid(0.7313, 804.740,  853.107,  683.099, bf);	// line 115"
	_test_trapezoid(0.7313, 804.740,  853.107,  801.234, bf);	// line 115"'
	_test_trapezoid(0.9158, 437.724,  801.233,  000.000, bf);	// line 120
	_test_trapezoid(0.9158, 683.099,  801.233,  245.375, bf);	// line 120'
	_test_trapezoid(0.9158, 801.233,  801.233,  617.229, bf);	// line 120"
	_test_trapezoid(0.3843, 245.375,  807.080,  000.000, bf);	// line 125
	_test_trapezoid(0.3843, 617.229,  807.080,  371.854, bf);	// line 125'  6,382,804 cycles



	_test_trapezoid(0.8, 0,	400, 400, bf);


// test cases drawn from braid_600mm					 		// expected results
//				   	L   	Ve  		Vt		Vx
	_test_trapezoid(0.327,	000.000,	600,	000.000, bf); // Ve=0 	   	Vc=110.155
	_test_trapezoid(0.327,	000.000,	600,	174.538, bf); // Ve=0, 	   	Vc=174.744	Vx=174.537
	_test_trapezoid(0.327,	174.873,	600,	173.867, bf); // Ve=174.873	Vc=185.356	Vx=173.867
	_test_trapezoid(0.327,	173.593,	600,	000.000, bf); // Ve=174.873	Vc=185.356	Vx=173.867
	_test_trapezoid(0.327,	347.082,	600,	173.214, bf); // Ve=174.873	Vc=185.356	Vx=173.867

}

static void _make_unit_vector(double unit[], double x, double y, double z, double a, double b, double c)
{
	double length = sqrt(x*x + y*y + z*z + a*a + b*b + c*c);
	unit[X] = x/length;
	unit[Y] = y/length;
	unit[Z] = z/length;
	unit[A] = a/length;
	unit[B] = b/length;
	unit[C] = c/length;
}

static void _test_get_junction_vmax()
{
//	cfg.a[X].jerk_max = JERK_TEST_VALUE;
//	cfg.a[Y].jerk_max = JERK_TEST_VALUE;
//	cfg.a[Z].jerk_max = JERK_TEST_VALUE;
//	cfg.a[A].jerk_max = JERK_TEST_VALUE;
//	cfg.a[B].jerk_max = JERK_TEST_VALUE;
//	cfg.a[C].jerk_max = JERK_TEST_VALUE;
//	mm.jerk_transition_size = 0.5;
//	mm.jerk_limit_max = 184.2;

	mm.test_case = 1;				// straight line along X axis
	_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 2;				// angled straight line
	_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 3;				// 5 degree bend
	_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.9962, 0.0872, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 4;				// 30 degrees
	_make_unit_vector(mm.a_unit, 1.0000, 0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.8660, 0.5000, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 5;				// 45 degrees
	_make_unit_vector(mm.a_unit, 0.8660,	0.5000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.2588,	0.9659, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 6;				// 60 degrees
	_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 7;				// 90 degrees
	_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit, 0.0000,	1.0000, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 8;				// 90 degrees rotated 45 degrees
	_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit,-0.7071, 0.7071, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 9;				// 120 degrees
	_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit,-0.5000,	0.8660, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 10;				// 150 degrees
	_make_unit_vector(mm.a_unit, 1.0000,	0.0000, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit,-0.8660,	0.5000, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);

	mm.test_case = 11;				// 180 degrees
	_make_unit_vector(mm.a_unit, 0.7071, 0.7071, 0, 0, 0, 0);
	_make_unit_vector(mm.b_unit,-0.7071,-0.7071, 0, 0, 0, 0);
	mm.test_velocity = _get_junction_vmax(mm.a_unit, mm.b_unit);
}

#endif // __UNIT_TEST_PLANNER
