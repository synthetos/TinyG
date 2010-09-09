/*
 * move_buffer.c - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * This version uses a pre-computed move buffer to optimize dequeuing / loading time
 *
 *	Instead of queueing the move as:
 *		- steps_x
 *		- steps_y
 *		- steps_z
 *		- microseconds (length of move),
 *
 *	the move is pre-computed and carried as the values needed by the stepper ISRs:
 *		- steps
 *		- timer period
 *		- timer postscaler value
 *		- direction ...for each axis
 *
 *	This moves a very expensive 64 bit division operation (~3800 cycles X 3) 
 *	to this phase and keeps it out of the high and priority stepper ISRs. 
 *	This makes for smooth movement (changes beetween lines) at high speeds, 
 *	and and helps to run the RS-485 network at high speeds (med priority ISRs).  
 *	See build 184 for the non-optimized version.
 */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "move_buffer.h"
#include "stepper.h"							// for 	st_execute_move()
#include "config.h"
#include "tinyg.h"

/* 
 * Local Scope Data and Functions
 */

#define MOVE_BUFFER_SIZE 4						// number of moves (lines) buffered

struct mvMoveBuffer {
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct mvMove *p;							// move buffer pointer
	struct mvMove move_buffer[MOVE_BUFFER_SIZE];// move buffer storage
};
static struct mvMoveBuffer mv;

/* 
 * mv_init() - initialize move buffers
 */

void mv_init()
{
	mv.move_buffer_head = 0;
	mv.move_buffer_tail = 0;
}

/*
 * mv_queue_move_buffer() - Add a new linear movement to the move buffer
 *
 * Arguments:
 *	steps_x, steps_y and steps_z are signed, relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform.
 *
 * Blocking behavior:
 *	This routine returns EAGAIN if there is no space in the buffer.
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

uint8_t mv_queue_move_buffer(int32_t steps_x, 
							 int32_t steps_y, 
							 int32_t steps_z, 
							 uint32_t microseconds)
{
	uint8_t next_buffer_head;
	uint8_t i;
	uint64_t ticks;	// Timer ticks in the move. A 2 minute move overflows 32 bits
					// Using 64 bits is expensive! The division goes from ~640 
					// cycles at 32 bits to ~3800 cycles using 64 bits
	uint32_t ticks_per_step; // temp variable

	// Bail on a zero length line (perhaps test for abs val < min line length)
	if ((steps_x == 0) && (steps_y == 0) && (steps_z) == 0) {
		return (TG_ZERO_LENGTH_LINE);
	}

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}

	// Return with error if the buffer is full 
	if (mv.move_buffer_tail == next_buffer_head) {
		return (TG_BUFFER_FULL);
	}
	/* Version that sleeps if buffer is full
	 * while(mv.move_buffer_tail == next_buffer_head) {// tail advances, breaking loop
	 *	sleep_mode(); // non-blocking queuing routines shouldn;t enter this routine 
	 *				  // unless there is room in the queue. So this never gets hit.
	 * } 
	 */

	// setup
	mv.p = &mv.move_buffer[mv.move_buffer_head];
	mv.p->a[X_AXIS].steps = steps_x;
	mv.p->a[Y_AXIS].steps = steps_y;
	mv.p->a[Z_AXIS].steps = steps_z;

	ticks = microseconds * TICKS_PER_MICROSECOND;

	// load axis values
	for (i = X_AXIS; i <= Z_AXIS; i++) {

		if (mv.p->a[i].steps) { 				// skip axes with zero steps

			// set direction: (polarity is corrected during execute move)
			(mv.p->a[i].steps < 0) ? 
			(mv.p->a[i].direction = 1): 		// CCW = 1 
			(mv.p->a[i].direction = 0);			// CW = 0

			// set steps to absolute value
			mv.p->a[i].steps = labs(mv.p->a[i].steps);

			// Normalize ticks_per_step by right shifting until the MSword = 0
			// Accumulate LSBs shifted out of ticks_per_step into postscale
			mv.p->a[i].postscale = 1;
			ticks_per_step = (uint32_t)(ticks / mv.p->a[i].steps);// expensive!
			while (ticks_per_step & 0xFFFF0000) {
				ticks_per_step >>= 1;
				mv.p->a[i].postscale <<= 1;
			}
			mv.p->a[i].period = (uint16_t)(ticks_per_step & 0x0000FFFF);
		}
	}
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
	int next_buffer_head;

	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					  // wrap condition
	}
	if (mv.move_buffer_tail == next_buffer_head) { 	// test if full
		return (TRUE);
	};
	return (FALSE);
}

/* 
 * mv_synchronize() - block until all buffered steps are executed 
 */

void mv_synchronize()
{
	while(mv.move_buffer_tail != mv.move_buffer_head) {
		sleep_mode();
	}    
}

/* 
 * mv_flush() - cancel all buffered steps 
 */

void mv_flush()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;
	mv.p = NULL;
	sei();
}
