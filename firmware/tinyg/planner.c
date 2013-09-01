/*
 * planner.c - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 * Copyright (c) 2012 - 2013 Rob Giseburt
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

#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "plan_line.h"
#include "planner.h"
#include "spindle.h"
#include "stepper.h"
#include "report.h"
#include "util.h"
//#include "xio/xio.h"			// uncomment for debugging

#ifdef __cplusplus
extern "C"{
#endif

// Allocate global scope structs
mpBufferPool_t mb;				// move buffer queue
mpMoveMasterSingleton_t mm;		// context for line planning
mpMoveRuntimeSingleton_t mr;	// context for line runtime

/*
 * Local Scope Data and Functions
 */
#define _bump(a) ((a<PLANNER_BUFFER_POOL_SIZE-1)?(a+1):0) // buffer incr & wrap
#define spindle_speed time		// local alias for spindle_speed to the time variable
#define value_vector target		// alias for vector of values
#define flag_vector unit		// alias for vector of flags

// execution routines (NB: These are all called from the LO interrupt)
static stat_t _exec_dwell(mpBuf_t *bf);
static stat_t _exec_command(mpBuf_t *bf);

#ifdef __DEBUG
static uint8_t _get_buffer_index(mpBuf_t *bf); 
static void _dump_plan_buffer(mpBuf_t *bf);
#endif

/* 
 * planner_init()
 */

void planner_init()
{
// If you can can assume all memory has been zeroed by a hard reset you don;t need these next 2 lines
	memset(&mr, 0, sizeof(mr));	// clear all values, pointers and status
	memset(&mm, 0, sizeof(mm));	// clear all values, pointers and status

	mr.magic_start = MAGICNUM;
	mr.magic_end = MAGICNUM;
	ar.magic_start = MAGICNUM;
	ar.magic_end = MAGICNUM;
	mp_init_buffers();
}

/* 
 * mp_flush_planner() - flush all moves in the planner and all arcs
 *
 *	Does not affect the move currently running in mr.
 *	Does not affect mm or gm model positions
 *	This function is designed to be called during a hold to reset the planner
 *	This function should not generally be called; call cm_queue_flush() instead
 */
void mp_flush_planner()
{
	ar_abort_arc();
	mp_init_buffers();
	cm.motion_state = MOTION_STOP;
}

/*
 * mp_set_planner_position() - sets both planner position by axis (mm struct)
 * mp_set_runtime_position() - sets both runtime position by axis (mr struct)
 *
 * 	Keeping track of position is complicated by the fact that moves exist in 
 *	several reference frames. The scheme to keep this straight is:
 *
 *	 - mm.position	- start and end position for planning
 *	 - mr.position	- current position of runtime segment
 *	 - mr.target	- target position of runtime segment
 *	 - mr.endpoint	- final target position of runtime segment
 *
 *	Note that position is set immediately when called and may not be not an accurate 
 *	representation of the tool position. The motors are still processing the 
 *	action and the real tool position is still close to the starting point.
 */

void mp_set_planner_position(uint8_t axis, const float position)
{
	mm.position[axis] = position;
}

void mp_set_runtime_position(uint8_t axis, const float position)
{
	mr.position[axis] = position;
}

/*************************************************************************
 * mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 */

stat_t mp_exec_move()
{
	mpBuf_t *bf;

	if ((bf = mp_get_run_buffer()) == NULL) return (STAT_NOOP);	// NULL means nothing's running

	// Manage cycle and motion state transitions. 
	// Cycle auto-start for lines only. 
	if (bf->move_type == MOVE_TYPE_ALINE) {
		if (cm.cycle_state == CYCLE_OFF) cm_cycle_start();
		if (cm.motion_state == MOTION_STOP) cm.motion_state = MOTION_RUN;
	}
	if (bf->bf_func != NULL) { return (bf->bf_func(bf));} 	// run the move callback in the planner buffer
	return (STAT_INTERNAL_ERROR);		// never supposed to get here
}

/************************************************************************************
 * mp_queue_command() - queue a synchronous Mcode, program control, or other command
 * _exec_command() 	  - callback to execute command
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

void mp_queue_command(void(*cm_exec)(float[], float[]), float *value, float *flag)
{
	mpBuf_t *bf;

	// this error is not reported as buffer availability was checked upstream in the controller
	if ((bf = mp_get_write_buffer()) == NULL) return;

	bf->move_type = MOVE_TYPE_COMMAND;
	bf->bf_func = _exec_command;		// callback to planner queue exec function
	bf->cm_func = cm_exec;				// callback to canonical machine exec function

	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		bf->value_vector[axis] = value[axis];
		bf->flag_vector[axis] = flag[axis];
	}
	mp_queue_write_buffer(MOVE_TYPE_COMMAND);
}

static stat_t _exec_command(mpBuf_t *bf)
{
	bf->cm_func(bf->value_vector, bf->flag_vector);	// 2 vectors used by callbacks
	st_prep_null();									// Must call a null prep to keep the loader happy. 
	mp_free_run_buffer();
	return (STAT_OK);
}

/*************************************************************************
 * mp_dwell() 	  - queue a dwell
 * _exec_dwell() - dwell execution
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * When the stepper driver sees a dwell it times the dwell on a separate 
 * timer than the stepper pulse timer.
 */

stat_t mp_dwell(float seconds) 
{
	mpBuf_t *bf; 

	if ((bf = mp_get_write_buffer()) == NULL) {	// get write buffer or fail
		return (STAT_BUFFER_FULL_FATAL);		// (not ever supposed to fail)
	}
	bf->bf_func = _exec_dwell;					// register callback to dwell start
	bf->time = seconds;						  	// in seconds, not minutes
	bf->move_state = MOVE_STATE_NEW;
	mp_queue_write_buffer(MOVE_TYPE_DWELL); 
	return (STAT_OK);
}

static stat_t _exec_dwell(mpBuf_t *bf)
{
	st_prep_dwell((uint32_t)(bf->time * 1000000));// convert seconds to uSec
	mp_free_run_buffer();
	return (STAT_OK);
/*
	if (bf->move_state == MOVE_STATE_NEW) {
		st_prep_dwell((uint32_t)(bf->time * 1000000));// convert seconds to uSec
		bf->move_state = MOVE_STATE_RUN;
	}
	return (STAT_OK);
*/
}
/*
void mp_end_dwell()								// all's well that ends dwell
{
	mp_free_run_buffer();						// Note: this is called from an interrupt
}
*/

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
	mpBuf_t *pv;
	uint8_t i;

	memset(&mb, 0, sizeof(mb));		// clear all values, pointers and status
	mb.magic_start = MAGICNUM;
	mb.magic_end = MAGICNUM;

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

mpBuf_t * mp_get_write_buffer() 				// get & clear a buffer
{
	if (mb.w->buffer_state == MP_BUFFER_EMPTY) {
		mpBuf_t *w = mb.w;
		mpBuf_t *nx = mb.w->nx;					// save pointers
		mpBuf_t *pv = mb.w->pv;
		memset(mb.w, 0, sizeof(mpBuf_t));
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
	rpt_request_queue_report(+1);				// add to the "added buffers" count
}

mpBuf_t * mp_get_run_buffer() 
{
	// CASE: fresh buffer; becomes running if queued or pending
	if ((mb.r->buffer_state == MP_BUFFER_QUEUED) || 
		(mb.r->buffer_state == MP_BUFFER_PENDING)) {
		 mb.r->buffer_state = MP_BUFFER_RUNNING;
	}
	// CASE: asking for the same run buffer for the Nth time
	if (mb.r->buffer_state == MP_BUFFER_RUNNING) {	// return same buffer
		return (mb.r);
	}
	return (NULL);								// CASE: no queued buffers. fail it.
}

void mp_free_run_buffer()						// EMPTY current run buf & adv to next
{
	mp_clear_buffer(mb.r);						 // clear it out (& reset replannable)
//	mb.r->buffer_state = MP_BUFFER_EMPTY;		 // redundant after the clear, above
	mb.r = mb.r->nx;							 // advance to next run buffer
	if (mb.r->buffer_state == MP_BUFFER_QUEUED) {// only if queued...
		mb.r->buffer_state = MP_BUFFER_PENDING;  // pend next buffer
	}
	if (mb.w == mb.r) cm_cycle_end();			// end the cycle if the queue empties
	mb.buffers_available++;
	rpt_request_queue_report(-1);				// add to the "removed buffers" count
}

mpBuf_t * mp_get_first_buffer(void)
{
	return(mp_get_run_buffer());	// returns buffer or NULL if nothing's running
}

mpBuf_t * mp_get_last_buffer(void)
{
	mpBuf_t *bf = mp_get_run_buffer();
	mpBuf_t *bp = bf;

	if (bf == NULL) { return(NULL);}

	do {
		if ((bp->nx->move_state == MOVE_STATE_OFF) || (bp->nx == bf)) { 
			return (bp); 
		}
	} while ((bp = mp_get_next_buffer(bp)) != bf);
	return (bp);
}

// Use the macro instead
//mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf) { return (bf->pv);}
//mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf) { return (bf->nx);}

void mp_clear_buffer(mpBuf_t *bf) 
{
	mpBuf_t *nx = bf->nx;			// save pointers
	mpBuf_t *pv = bf->pv;
	memset(bf, 0, sizeof(mpBuf_t));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp)
{
	mpBuf_t *nx = bf->nx;			// save pointers
	mpBuf_t *pv = bf->pv;
 	memcpy(bf, bp, sizeof(mpBuf_t));
	bf->nx = nx;					// restore pointers
	bf->pv = pv;
}

#ifdef __DEBUG	// currently this routine is only used by debug routines
uint8_t mp_get_buffer_index(mpBuf_t *bf) 
{
	mpBuf_t *b = bf;		// temp buffer pointer

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

static void _dump_plan_buffer(mpBuf_t *bf)
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

#ifdef __cplusplus
}
#endif
