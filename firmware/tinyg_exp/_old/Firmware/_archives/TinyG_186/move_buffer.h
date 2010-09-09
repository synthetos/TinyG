/*
 * move_buffer.h - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef move_buffer_h
#define move_buffer_h 

/*
 * simple move buffer entry structure
 */

struct mvMove {						// Linear moves are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

/*
 * pre-computed move buffer entry structure
 */

struct mvMoveAxis {
 	int32_t steps; 					// total steps in each direction
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (software counter)
	uint8_t direction;				// direction (in LSB)
};

struct mvMove2 {					// Linear moves are queued stepper movements
//	uint8_t active_axes; 			// see config.h for bit positions
	struct mvMoveAxis a[3];			// axis structs
};


/*
 * Global Scope Functions
 */

void mv_init(void);					// Initialize and start stepper motor subsystem
void mv_queue_move_buffer(int32_t steps_x, int32_t steps_y, int32_t steps_z, 
						  uint32_t rate);

uint8_t mv_queue_move_buffer2(int32_t steps_x, int32_t steps_y, int32_t steps_z, 
							  uint32_t microseconds);

struct mvMove *mv_dequeue_move_buffer(void);
struct mvMove2 *mv_dequeue_move_buffer2(void);

uint8_t mv_test_move_buffer_full(void);	// test if the line buffer is full
uint8_t mv_test_move_buffer_full2(void);

void mv_flush(void);				// Cancel all pending steps
void mv_synchronize(void);			// Block until all buffered steps are executed



#endif

