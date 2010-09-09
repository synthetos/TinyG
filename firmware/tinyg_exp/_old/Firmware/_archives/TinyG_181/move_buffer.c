/*
 * move_buffer.c - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 */

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_init.h"
#include <util/delay.h>
#include "stepper.h"
#include "config.h"
#include "tinyg.h"

#ifdef __DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"
#endif


/* 
 * Local Scope Data and Functions
 */

#define MOVE_BUFFER_SIZE 4			// number of moves (lines) buffered

struct stMove {						// Linear moves are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

struct stMoves {
	volatile uint8_t move_busy; 				// MUTEX for st_execute_move()
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct stMove *buf;							// buffer pointer
	struct stMove move_buffer[MOVE_BUFFER_SIZE];// buffer storage
};
static struct stMoves mv;


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

uint8_t st_test_move_buffer_full() 
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
 * _st_dequeue_move_buffer() - return the next line from the move buffer & advance buffer tail
 */ 

struct stMove *_st_dequeue_move_buffer()
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
 * st_synchronize() - block until all buffered steps are executed 
 */

void st_synchronize()
{
	while(mv.move_buffer_tail != mv.move_buffer_head) {
		sleep_mode();
	}    
}

/* 
 * st_flush() - cancel all buffered steps 
 */

void st_flush()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;
	mv.buf = NULL;
	sei();
}

/* 
 * st_kill() - STOP. NOW. UNCONDITIONALLY
 */

void st_kill()
{
	cli();										// stop intewrrupts
	ax.x.timer->CTRLA = TC_CLK_OFF;				// stop the clocks
	ax.y.timer->CTRLA = TC_CLK_OFF;
	ax.z.timer->CTRLA = TC_CLK_OFF;
	ax.a.timer->CTRLA = TC_CLK_OFF;
	
	mv.move_buffer_tail = mv.move_buffer_head;	// clear the move buffer
	ax.active_axes = 0;							// clear all the active bits
	sei();
}

/* 
 * st_terminate() - stop moves after the current move
 */

void st_terminate()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;	// clear the line buffer
	sei();
}

/* 
 * st_go_home() - perform the homing cycle 
 */

void st_go_home()
{
  // Todo: Perform the homing cycle
}


#ifdef __DEBUG
void _st_print_exec_line(struct stMove mv, uint8_t active)
{
	printf_P(PSTR("Exec X=%d Y=%d Z=%d\n"), mv.steps_x, 
											mv.steps_y, 
											mv.steps_z);
/*
	printf_P(PSTR("Exec X=%d Y=%d Z=%d uS=%d Active=%d\n"), mv.steps_x, 
															mv.steps_y, 
															mv.steps_z, 
															mv.microseconds,
															active);}
*/
#endif

/*
void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint32_t us, uint8_t active)
{
	printf_P(PSTR("Exec X=%d Y=%d Z=%d uS=%d Active=%d\n"), x, y, z, us, active);
}
*/
