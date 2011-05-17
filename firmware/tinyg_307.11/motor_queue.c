/*
 * motor_queue.c - routines for managing motor moves
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
 */
/* ------
 * This module buffers pre-computed moves including line segments, dwells, 
 * stop and start commands. It manages the buffers that are consumed by
 * the stepper routines.
 */

#include <stdlib.h>
#include <string.h>					// for memset();
#include <math.h>
#include <avr/interrupt.h>

#include "xio.h"					// supports TRAP and print statements
#include "tinyg.h"
#include "motor_queue.h"
#include "stepper.h"				// for 	st_execute_move()
#include "config.h"

#define MQ_BUFFER_SIZE 3 // limited to 255 unless heads & tails --> uint16_t

struct mqMotorBuffer {
	volatile uint8_t head;			// motor queue index (for writes)
	volatile uint8_t tail;			// motor dequeue index (for reads)
	struct mqMove *p;				// motor buffer pointer
	struct mqMove move_buffer[MQ_BUFFER_SIZE];// motor buffer storage
};
static struct mqMotorBuffer mq;

// local functions
static uint32_t _mq_convert_steps(const double steps);
static int8_t _mq_set_direction(const double steps);

/* 
 * mq_init() - initialize move buffers
 */

void mq_init()
{
	mq.head = 0;
	mq.tail = MQ_BUFFER_SIZE-1;
}

/**** MOTOR QUEUE ROUTINES ****
 * mq_test_motor_buffer() 	 - test if motor buffer is available for write
 * mq_queue_motor_buffer() 	 - get and queue write buffer
 * mq_dequeue_motor_buffer() - dequeue read buffer
 * mq_flush_motor_buffer() 	 - remove all buffered moves (reset queue) 
 * 
 * Move buffer circular buffer operation
 *	mq.head is the array index to which the move will be queued (written)
 *	mq.head is post-incremented (after queuing the move)
 *  mq.tail is the array index from which the previous move was dequeued
 *	mq.tail is pre-incremented (before dequeuing the move)
 *
 *	Buffer empty:	move_buffer_head == move_buffer_tail
 *	Buffer full:	move_buffer_head+1 == move_buffer_tail 
 *
 * Note: The method of testing for buffer full then writing the buffer 
 *		 as a separate, non-atomic operation works as long as there is 
 *		 no pre-emption that could invalidate the full/not-full result. 
 *		 As this project is currently coded there is no pre-emption 
 *		 possible in this critical region - i.e. there's no way somebody 
 *		 else can get in there and write to the move buffer between the 
 *		 not-full result and the subsequent write. 
 */

uint8_t mq_test_motor_buffer() 
{
	if (mq.head == mq.tail) { 
		return FALSE; 				// buffer full
	}
	return (TRUE);
}

struct mqMove * mq_queue_motor_buffer() 
{
	if (mq.tail == mq.head) { 
		return (NULL); // buffer full
	} 
	mq.p = &mq.move_buffer[mq.head];
	if (++mq.head >= MQ_BUFFER_SIZE) { 
		mq.head = 0; 				// advance head
	}
	return (mq.p);
}

struct mqMove * mq_dequeue_motor_buffer()
{
	uint8_t next_tail = mq.tail;
	if (++next_tail >= MQ_BUFFER_SIZE) { 
		next_tail = 0; 				// incr with wrap
	}
	if (next_tail == mq.head) { 
		return (NULL); 				// buffer empty
	}
	mq.tail = next_tail;
	return (mq.p = &mq.move_buffer[mq.tail]);
}

void mq_flush_motor_buffer()
{
	cli();
	mq.tail = mq.head;
	mq.p = NULL;
	sei();
}

/*
 * _mq_convert_steps() - convert (double)steps to internal format
 * _mq_set_direction() - set direction based on (double)steps
 */

static uint32_t _mq_convert_steps(const double steps)
{
	return (uint32_t)fabs(steps);
}

static int8_t _mq_set_direction(const double steps)
{
	if (steps < 0) {
		return (1);	//  CW = 0 (negative)
	} else {
		return (0);	// CCW = 1 (positive)
	}
}

/*
 * mq_queue_line() - Add a new linear movement to the move buffer
 *
 * This function queues a line segment to the motor buffer. It deals with 
 * all the DDA optimizations and timer setups *here* so that the dequeuing
 * operation can be as rapid as possible. All args are provided as doubles
 * and converted to their appropriate integer types during queuing.
 *
 * Args:
 *	steps_x ... steps_a are signed relative motion in steps
 *	Microseconds specify how many microseconds the move should take to perform
 *
 * Blocking behavior:
 *	This routine returns BUFFER_FULL if there is no space in the buffer.
 *  A blocking version should wrap this code with blocking semantics
 */

uint8_t mq_queue_line(double steps_x, double steps_y,
					  double steps_z, double steps_a, 
					  double microseconds)
{
	uint8_t i,j;
	double maxsteps = 0;
	double steps[AXES] = { steps_x, steps_y, steps_z, steps_a };

#ifdef __dbSHOW_QUEUED_LINE
	fprintf_P(stderr, PSTR("Queue line %6.1f %6.1f %6.1f %6.1f - %6.0f\n"), 
			  steps_x, steps_y, steps_z, steps_a, microseconds);
#endif

	if (microseconds < ROUNDING_ERROR) {		// zero time move
		return (TG_ZERO_LENGTH_MOVE);
	}
	if ((fabs(steps_x) < ROUNDING_ERROR) && 
		(fabs(steps_y) < ROUNDING_ERROR) && 
		(fabs(steps_z) < ROUNDING_ERROR) && 
		(fabs(steps_a) < ROUNDING_ERROR)) {
		return (TG_ZERO_LENGTH_MOVE);
	}
	if ((mq.p = mq_queue_motor_buffer()) == NULL) {	// shouldn't ever fail
		return (TG_BUFFER_FULL_NON_FATAL);			//...but just in case
	}
	// map axes to motors and setup axis parameters
	for (i=0; i<AXES; i++) {
		for (j=0; j<MOTORS; j++) {
			if (cfg.motor_map[j] == i) {
				mq.p->a[j].steps = _mq_convert_steps(steps[i]);
				mq.p->a[j].dir = _mq_set_direction(steps[i]);
				if (maxsteps < steps[i]) { maxsteps = steps[i]; }
			}
		}
	}
	mq.p->mq_type = MQ_LINE;
	mq.p->timer_period = DDA_PERIOD;
	mq.p->timer_ticks = (uint32_t)(microseconds * DDA_MHZ);
 	TRAP_GT(maxsteps, mq.p->timer_ticks, PSTR("Steps exceeds DDA frequency: %f"))
	st_request_load();
	return (TG_OK);
}

/*
 * mq_queue_dwell() - Add a dwell to the move buffer
 */

uint8_t mq_queue_dwell(double microseconds)
{
	if ((mq.p = mq_queue_motor_buffer()) == NULL) {	// should always get a buffer
		return (TG_BUFFER_FULL_NON_FATAL);
	}
	mq.p->mq_type = MQ_DWELL;
	mq.p->timer_period = DWELL_PERIOD;
	mq.p->timer_ticks = (uint32_t)(microseconds * DWELL_MHZ);
	st_request_load();
	return (TG_OK);
}

/*
 * mq_queue_stops() - Add a start, stop or end to the move buffer
 */

uint8_t mq_queue_stops(uint8_t mq_type)
{
	if ((mq.p = mq_queue_motor_buffer()) == NULL) {	// should always get a buffer
		return (TG_BUFFER_FULL_NON_FATAL);
	}
	mq.p->mq_type = mq_type;
	st_request_load();
	return (TG_OK);
}

/*
 * mq_print_motor_queue()
 */
static char mqs_mbuf[] PROGMEM = "MotorBuffer head %d,  tail %d\n";
static char mqs_move[] PROGMEM = "  [%d]: move_type:%d  timer_period:%d  timer_ticks:%d\n";
static char mqs_motr[] PROGMEM = "     [%d]: dir:%d  steps:%d\n";

void mq_print_motor_queue()
{
	uint8_t i,j;

	fprintf_P(stderr, (PGM_P)mqs_mbuf, mq.head, mq.tail);
	for (i=0; i<MQ_BUFFER_SIZE; i++) {
		fprintf_P(stderr, (PGM_P)mqs_move, i,
			mq.move_buffer[i].mq_type, 
			mq.move_buffer[i].timer_period, 
			mq.move_buffer[i].timer_ticks);
		for (j=0; j<MOTORS; j++) {
			fprintf_P(stderr, (PGM_P)mqs_motr, j, 
				mq.move_buffer[i].a[j].dir,
				mq.move_buffer[i].a[j].steps);
		}
	}
	st_print_stepper_state();
}
