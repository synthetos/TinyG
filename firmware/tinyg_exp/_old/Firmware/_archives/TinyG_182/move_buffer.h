/*
 * move_buffer.h - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef move_buffer_h
#define move_buffer_h 


struct mvMove {						// Linear moves are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

/*
 * Global Scope Functions
 */

void mv_init(void);					// Initialize and start stepper motor subsystem
void mv_queue_move_buffer(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);
struct mvMove *mv_dequeue_move_buffer(void);
uint8_t mv_test_move_buffer_full(void);	// test if the line buffer is full
void mv_flush(void);				// Cancel all pending steps
void mv_synchronize(void);			// Block until all buffered steps are executed

#endif

