/*
 * move_buffer.c - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 */

//#include <math.h>
#include <stdlib.h>
#include <string.h>					// for memset()
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "move_buffer.h"
#include "stepper.h"
#include "config.h"
#include "tinyg.h"

/* 
 * Local Scope Data and Functions
 */

#define MOVE_BUFFER_SIZE 4			// number of moves (lines) buffered

struct mvMoveBuffer {
	volatile uint8_t move_busy; 				// MUTEX for st_execute_move()
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct mvMove *buf;							// buffer pointer
	struct mvMove move_buffer[MOVE_BUFFER_SIZE];// buffer storage
};
static struct mvMoveBuffer mv;

struct mvMoveBuffer2 {
	volatile uint8_t move_busy; 				// MUTEX for st_execute_move()
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct mvMove2 *buf;						// buffer pointer
	struct mvMove2 move_buffer[MOVE_BUFFER_SIZE];// buffer storage
};
static struct mvMoveBuffer2 mv2;


/* 
 * mv_init() - initialize move buffers
 */

void mv_init()
{
	mv.move_buffer_head = 0;
	mv.move_buffer_tail = 0;
	mv.move_busy = FALSE;
}


/*
 * mv_queue_move_buffer2() - Add a new linear movement to the move buffer
 *
 * Pre-computed to optimize dequeuing / loading time
 *
 * Arguments:
 *	steps_x, steps_y and steps_z are the signed, relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform.
 *
 * Blocking behavior:
 *	This routine returns 
 *
 *	This routine sleeps (blocks) if there is no space in the move buffer. 
 *	If you want to run non-blocking, first call mv_test_move_buffer_full()
 *	to test the queue. Or package the two functions in a non-blocking wrapper.
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

uint8_t mv_queue_move_buffer2(int32_t steps_x, 
							  int32_t steps_y, 
							  int32_t steps_z, 
							  uint32_t microseconds)
{
	uint8_t next_buffer_head;
	uint8_t i;
	int32_t steps[3];// temp storage for steps
	uint64_t ticks;	// Timer ticks in the move. A 2 minute move overflows 32 bits
					// Using 64 bits is expensive! The division goes from ~640 
					// cycles at 32 bits to ~3800 cycles using 64 bits
	uint32_t ticks_per_step; // temp for intermediate computed term

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

	// If the buffer is full sleep until there is room in the buffer.
//	while(mv.move_buffer_tail == next_buffer_head) {// tail advances, breaking loop
//		sleep_mode(); // non-blocking queuing routines don't enter this routine 
//					  // unless there is room in the queue. So this never gets hit.
//	}

	// setup
	steps[X_AXIS] = steps_x;
	steps[Y_AXIS] = steps_y;
	steps[Z_AXIS] = steps_z;
	ticks = microseconds * TICKS_PER_MICROSECOND;
	memset(&mv2, 0, sizeof(mv2));
	mv2.buf = &mv2.move_buffer[mv2.move_buffer_head];

	// load axis values
	for (i = X_AXIS; i <= Z_AXIS; i++) {

		if (steps[i] < 0) {			// set direction: CW = 0, CCW = 1
			mv2.buf->direction[i] = 1;	// (polarity correction done during dequeueing)
		}

		if (labs(steps[i]) != 0) {
			mv2.buf->active_axes = (mv2.buf->active_axes << 1);	// shift and set...
			mv2.buf->active_axes |= 0x01;						//...axis active bit
			mv2.buf->steps[i] = steps[i];

			// Normalize ticks_per_step by right shifting until the MSword is zero
			// Accumulate LSBs shifted out of ticks_per_step into postscale_value

			mv2.buf->postscale[i] = 1;
			ticks_per_step = (uint32_t)(ticks / labs(steps[i])); // expensive divide
			while (ticks_per_step & 0xFFFF0000) {
				ticks_per_step >>= 1;
				mv2.buf->postscale[i] <<= 1;
			}
			mv2.buf->period[i] = (uint16_t)(ticks_per_step & 0x0000FFFF);
		}
		mv2.buf->active_axes = (mv2.buf->active_axes << 1);	// compensate for no A axis
	}
	mv2.move_buffer_head = next_buffer_head;
	return (TG_OK);
}




/*
 * mv_queue_move_buffer() - Add a new linear movement to the move buffer
 *	
 * Inputs:
 *	steps_x, steps_y and steps_z are the signed, relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform.
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

void mv_queue_move_buffer(int32_t steps_x, 
						  int32_t steps_y, 
						  int32_t steps_z, 
						  uint32_t microseconds) 
{
	int next_buffer_head;

	// Bail on a zero length line (perhaps test for abs val < min line length)
	if ((steps_x == 0) && (steps_y == 0) && (steps_z) == 0) {
		return;
	};

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}

	// If the buffer is full sleep until there is room in the buffer.
	while(mv.move_buffer_tail == next_buffer_head) {// tail advances, breaking loop
		sleep_mode(); // non-blocking queuing routines don't enter this routine 
					  // unless there is room in the queue. So this never gets hit.
	}

	mv.buf = &mv.move_buffer[mv.move_buffer_head];
	mv.buf->steps_x = steps_x; 			// write the line record to the buffer head
	mv.buf->steps_y = steps_y;
	mv.buf->steps_z = steps_z;  
	mv.buf->microseconds = microseconds;

	mv.move_buffer_head = next_buffer_head;
	st_execute_move();						// run this line
}

/* 
 * mv_dequeue_move_buffer() - return the next line from the move buffer & advance buffer tail
 */ 

struct mvMove *mv_dequeue_move_buffer()
{
	if (mv.move_buffer_head == mv.move_buffer_tail) {	// buffer empty
		return (NULL);
	}
	mv.buf = &mv.move_buffer[mv.move_buffer_tail];	// get and save current index
	if (++mv.move_buffer_tail >= MOVE_BUFFER_SIZE) { // increment and wrap (no OBOE)
		mv.move_buffer_tail = 0;
	}
	return (mv.buf); 
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
	mv.buf = NULL;
	sei();
}


