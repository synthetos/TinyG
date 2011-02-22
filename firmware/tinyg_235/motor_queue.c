/*
 * motor_queue.c - routines for managing motor moves
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
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
 * ------
 * This code buffers pre-computed moves to optimize dequeuing / loading time
 *
 *	Instead of queueing the move as:
 *		- steps_x
 *		- steps_y
 *		- steps_z
 *		- microseconds (length of move),
 *
 *	move is pre-computed and carried as the values needed by the stepper ISRs:
 *	  ...for each axis:
 *		- steps
 *		- timer period
 *		- timer postscaler value
 *		- direction 
 *
 *	This moves an expensive 64 bit division operation (~3800 cycles X 3) 
 *	to this phase and keeps it out of the high-priority stepper ISRs. 
 *	This makes for smooth movement (changes between lines) at high speeds, 
 *	and and helps to run the RS-485 network at high speeds (med priority ISRs).  
 *	See build 184 for the non-optimized version.
 */

#include <stdlib.h>
#include <string.h>								// for memset();
#include <math.h>
#include <avr/interrupt.h>
//#include <avr/sleep.h>
//#include <avr/io.h>

#include "motor_queue.h"
#include "stepper.h"							// for 	st_execute_move()
#include "config.h"
#include "tinyg.h"


//#define MQ_BUFFER_SIZE 150
#define MQ_BUFFER_SIZE 3 // limited to 255 unless heads & tails --> uint16_t

struct mqMotorBuffer {
	uint64_t ticks;					// see note below
	uint64_t ticks_per_step;
	uint64_t microseconds;
	volatile uint8_t head;			// motor queue index (for writes)
	volatile uint8_t tail;			// motor dequeue index (for reads)
	struct mqMove *p;				// motor buffer pointer
	struct mqMove move_buffer[MQ_BUFFER_SIZE];// motor buffer storage
};
static struct mqMotorBuffer mq;

/*	Note: 64 bit fixed point arithmetic is used to compute ticks, steps and 
	durations (seconds) while queuing moves. A 2 minute move overflows 32 bits. 
	Using 64 bits is expensive. The division goes from ~640 cycles at 32 bits
	to ~3800 cycles using 64 bits. Can't use doubles as you need to manipulate 
	the bits to load the timers.
*/

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
		return (NULL); 				// buffer full
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
 * mq_queue_line() - Add a new linear movement to the move buffer
 *
 * Arguments:
 *	steps_x, steps_y and steps_z are signed relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform
 *
 * Blocking behavior:
 *	This routine returns BUFFER_FULL if there is no space in the buffer.
 *	A version that sleeps (blocks) if there is no space in the move buffer is
 *  commented out. If you want to run non-blocking, first call 
 *  mq_test_move_buffer_full() to test the queue. Or package the two functions 
 *	in a non-blocking wrapper.
 */

uint8_t mq_queue_line(int32_t steps_x, int32_t steps_y, 
					  int32_t steps_z, int32_t steps_a, 
					  uint32_t microseconds)
{
	if (microseconds == 0) {						// zero time move
		return (TG_ZERO_LENGTH_MOVE);
	}
	if ((steps_x == 0) && (steps_y == 0) && (steps_z == 0) && (steps_a == 0)) {
//		return (mq_queue_dwell(microseconds));			// queue it as a dwell
		return (TG_ZERO_LENGTH_MOVE);
	}
	if ((mq.p = mq_queue_motor_buffer()) == NULL) {	// should always get a buffer
		return (TG_BUFFER_FULL_NON_FATAL);
	}
	// setup the move struct and ticks value
	memset(mq.p, 0, sizeof(struct mqMove));
	mq.p->a[X].steps = steps_x;
	mq.p->a[Y].steps = steps_y;
	mq.p->a[Z].steps = steps_z;
	mq.p->a[A].steps = steps_a;
	mq.microseconds = (uint64_t)microseconds;	// cast to larger base
	mq.ticks = mq.microseconds * TICKS_PER_MICROSECOND;

	for (uint8_t i = 0; i < AXES; i++) {
		if (mq.p->a[i].steps) { 				// skip axes w/ zero steps
			// set direction: (polarity is corrected during execute move)
			(mq.p->a[i].steps < 0) ? 
			(mq.p->a[i].direction = 1): 		// CCW = 1 
			(mq.p->a[i].direction = 0);			// CW = 0

			// set steps to absolute value
			mq.p->a[i].steps = labs(mq.p->a[i].steps);

			// Normalize ticks_per_step by right shifting until the MSword = 0
			// Accumulate LSBs shifted out of ticks_per_step into postscale
			mq.p->a[i].postscale = 1;
			mq.ticks_per_step = (uint64_t)(mq.ticks / mq.p->a[i].steps);// expensive!
			while (mq.ticks_per_step & 0xFFFFFFFFFFFF0000) {
				mq.ticks_per_step >>= 1;
				mq.p->a[i].postscale <<= 1;
			}
			mq.p->a[i].period = (uint16_t)(mq.ticks_per_step & 0x0000FFFF);
		}
	}
	mq.p->mq_type = MQ_LINE;
	st_execute_move();			// kick the stepper drivers
	return (TG_OK);
}

/*
 * mq_queue_dwell() - Add a dwell to the move buffer
 *
 * Queue a dwell on the Z axis
 */

uint8_t mq_queue_dwell(uint32_t microseconds)
{
	if ((mq.p = mq_queue_motor_buffer()) == NULL) {	// should always get a buffer
		return (TG_BUFFER_FULL_NON_FATAL);
	}
	// setup the move struct and ticks value
	memset(mq.p, 0, sizeof(struct mqMove));
	mq.microseconds = (uint64_t)microseconds;			// cast to larger base
	mq.ticks = mq.microseconds * TICKS_PER_MICROSECOND;
	mq.p->a[Z].steps = (((mq.ticks & 0xFFFF0000)>>32)+1);	// compute steps
	mq.p->a[Z].postscale = 1;
	mq.ticks_per_step = (uint64_t)(mq.ticks / mq.p->a[Z].steps); // expensive!
	while (mq.ticks_per_step & 0xFFFFFFFFFFFF0000) {
		mq.ticks_per_step >>= 1;
		mq.p->a[Z].postscale <<= 1;
	}
	mq.p->a[Z].period = (uint16_t)(mq.ticks_per_step & 0x0000FFFF);
	mq.p->mq_type = MQ_DWELL;
	st_execute_move();
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
	st_execute_move();
	return (TG_OK);
}
