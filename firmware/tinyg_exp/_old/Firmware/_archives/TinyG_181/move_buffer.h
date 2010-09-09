/*
 * move_buffer.h - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef move_buffer_h
#define move_buffer_h 

//#include <avr/io.h>
//#include <avr/sleep.h>

/*
 * Global Scope Functions
 */

void mv_init(void);					// Initialize and start stepper motor subsystem
void st_queue_move_buffer(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);
struct stMove *st_dequeue_move_buffer(void);
uint8_t st_test_move_buffer_full(void);	// test if the line buffer is full
void st_flush(void);				// Cancel all pending steps
void st_synchronize(void);			// Block until all buffered steps are executed

#endif

