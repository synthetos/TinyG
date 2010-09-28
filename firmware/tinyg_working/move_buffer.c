/*
 * move_buffer.c - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under the 
 * terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License along 
 * with TinyG  If not, see <http://www.gnu.org/licenses/>.
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
#include <avr/io.h>
#include <string.h>								// for memset();
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "move_buffer.h"
#include "stepper.h"							// for 	st_execute_move()
#include "config.h"
#include "tinyg.h"

/* 
 * Local Scope Data and Functions
 */

#define MOVE_BUFFER_SIZE 3						// limited to 255 unless you change:
												// move_buffer_head (to uint16_t)
												// move_buffer_tail (to uint16_t)
												// next_buffer_head (multiple places)
struct mvMoveBuffer {
	uint64_t ticks;								// see note.
	uint64_t ticks_per_step;
	uint64_t microseconds;
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct mvMove *p;							// move buffer pointer
	struct mvMove move_buffer[MOVE_BUFFER_SIZE];// move buffer storage
};
static struct mvMoveBuffer mv;

/*	Note: 64 bit fixed point arithmetic is used to compute ticks, steps and 
	durations (seconds) while queuing moves. A 2 minute move overflows 32 bits. 
	Using 64 bits is expensive. The division goes from ~640 cycles at 32 bits
	to ~3800 cycles using 64 bits. Can't use doubles as you need to manipulate 
	the bits to load the timers.
*/

/* 
 * mv_init() - initialize move buffers
 */

void mv_init()
{
	mv.move_buffer_head = 0;
	mv.move_buffer_tail = 0;
}

/*
 * mv_queue_line() - Add a new linear movement to the move buffer
 *
 * Arguments:
 *	steps_x, steps_y and steps_z are signed relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform
 *
 * Blocking behavior:
 *	This routine returns BUFFER _FULL if there is no space in the buffer.
 *	A version that sleeps (blocks) if there is no space in the move buffer is
 *  commented out. If you want to run non-blocking, first call 
 *  mv_test_move_buffer_full() to test the queue. Or package the two functions 
 *	in a non-blocking wrapper.
 *
 * Move buffer circular buffer operation
 *  move_buffer_tail is the array index from which the previous line was read.
 *	move_buffer_tail is always incremented before reading the line.
 *
 *	move_buffer_head is the array index to which the line will be written.
 *	move_buffer_head is always incremented after writing the line.
 *
 *	Buffer full:	move_buffer_head == move_buffer_tail
 *	Buffer empty:	move_buffer_head == move_buffer_tail+1 
 */

uint8_t mv_queue_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds)
{
	uint8_t next_buffer_head;
	uint8_t i;

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}

	// Return with error if the buffer is full 
	if (mv.move_buffer_tail == next_buffer_head) {
		return (TG_BUFFER_FULL_NON_FATAL);
//		sleep_mode();	// USE INSTEAD IF YOU WANT BLOCKING BEHAVIOR
	}

	// setup the move struct and ticks value
	mv.p = &mv.move_buffer[mv.move_buffer_head];
	memset(mv.p, 0, sizeof(struct mvMove));
	mv.p->a[X].steps = steps_x;
	mv.p->a[Y].steps = steps_y;
	mv.p->a[Z].steps = steps_z;
	mv.microseconds = (uint64_t)microseconds;	// cast to larger base
	mv.ticks = mv.microseconds * TICKS_PER_MICROSECOND;

	for (i = X; i <= Z; i++) {
		if (mv.p->a[i].steps) { 				// skip axes w/ zero steps
			// set direction: (polarity is corrected during execute move)
			(mv.p->a[i].steps < 0) ? 
			(mv.p->a[i].direction = 1): 		// CCW = 1 
			(mv.p->a[i].direction = 0);			// CW = 0

			// set steps to absolute value
			mv.p->a[i].steps = labs(mv.p->a[i].steps);

			// Normalize ticks_per_step by right shifting until the MSword = 0
			// Accumulate LSBs shifted out of ticks_per_step into postscale
			mv.p->a[i].postscale = 1;
			mv.ticks_per_step = (uint64_t)(mv.ticks / mv.p->a[i].steps);// expensive!
			while (mv.ticks_per_step & 0xFFFFFFFFFFFF0000) {
				mv.ticks_per_step >>= 1;
				mv.p->a[i].postscale <<= 1;
			}
			mv.p->a[i].period = (uint16_t)(mv.ticks_per_step & 0x0000FFFF);
		}
	}
	mv.p->move_type = MV_TYPE_LINE;
	mv.move_buffer_head = next_buffer_head;
	st_execute_move();			// kick the stepper drivers
	return (TG_OK);
}

/*
 * mv_queue_dwell() - Add a dwell to the move buffer
 *
 * Queue a dwell on the Z axis
 */

uint8_t mv_queue_dwell(uint32_t microseconds)
{
	uint8_t next_buffer_head;

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}

	// Return with error if the buffer is full 
	if (mv.move_buffer_tail == next_buffer_head) {
		return (TG_BUFFER_FULL_NON_FATAL);
//		sleep_mode();	// USE INSTEAD IF YOU WANT BLOCKING BEHAVIOR
	}

	// setup the move struct and ticks value
	mv.p = &mv.move_buffer[mv.move_buffer_head];
	memset(mv.p, 0, sizeof(struct mvMove));
	mv.microseconds = (uint64_t)microseconds;			// cast to larger base
	mv.ticks = mv.microseconds * TICKS_PER_MICROSECOND;
	mv.p->a[Z].steps = (((mv.ticks & 0xFFFF0000)>>32)+1);	// compute steps
	mv.p->a[Z].postscale = 1;
	mv.ticks_per_step = (uint64_t)(mv.ticks / mv.p->a[Z].steps); // expensive!
	while (mv.ticks_per_step & 0xFFFFFFFFFFFF0000) {
		mv.ticks_per_step >>= 1;
		mv.p->a[Z].postscale <<= 1;
	}
	mv.p->a[Z].period = (uint16_t)(mv.ticks_per_step & 0x0000FFFF);
	mv.p->move_type = MV_TYPE_DWELL;
	mv.move_buffer_head = next_buffer_head;
	st_execute_move();
	return (TG_OK);
}

/*
 * mv_queue_start_stop() - Add a start or stop to the move buffer
 */

uint8_t mv_queue_start_stop(uint8_t move_type)
{
	uint8_t next_buffer_head;

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}
	if (mv.move_buffer_tail == next_buffer_head) {
		return (TG_BUFFER_FULL_NON_FATAL);
//		sleep_mode();	// USE INSTEAD IF YOU WANT BLOCKING BEHAVIOR
	}
	mv.p->move_type = move_type;
	mv.move_buffer_head = next_buffer_head;
	st_execute_move();
	return (TG_OK);
}

/* 
 * mv_dequeue_move_buffer() - Return next move from buffer & advance buffer tail
 */

struct mvMove *mv_dequeue_move_buffer()
{
	if (mv.move_buffer_head == mv.move_buffer_tail) {	// buffer empty
		return (NULL);
	}
	mv.p = &mv.move_buffer[mv.move_buffer_tail];	// get and save current index
	if (++mv.move_buffer_tail >= MOVE_BUFFER_SIZE) { // increment and wrap (no OBOE)
		mv.move_buffer_tail = 0;
	}
	return (mv.p); 
}

/*
 * mv_test_move_buffer_full() - Test if the move buffer is full
 *
 *	Returns:	1 (TRUE) 	the buffer is full
 *  			0 (FALSE) 	the buffer is not full
 *
 *  Note: The method of testing for buffer full then writing the buffer as a
 *		  separate, non-atomic operation works as long as there is no pre-emption
 *		  that could invalidate the full/not-full result. As this project is
 *		  currently coded there is no pre-emption possible in this critical region -
 *		  i.e. there's no way somebody else can get in there and write to the line 
 *		  buffer between the not-full result and the subsequent write. 
 *		  Be careful about changing this condition.
 */

uint8_t mv_test_move_buffer_full() 
{
	uint8_t next_buffer_head;

	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					  // wrap condition
	}
	if (mv.move_buffer_tail == next_buffer_head) { 	// test if full
		return (TRUE);
	};
	return (FALSE);
}

/* 
 * mv_flush() - remove all buffered moves (reset queue) 
 */

void mv_flush()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;
	mv.p = NULL;
	sei();
}
