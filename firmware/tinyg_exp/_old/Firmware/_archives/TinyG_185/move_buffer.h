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

struct mvMoveSub {
 	uint32_t steps; 				// total steps in each direction (abs value)
	uint8_t direction;				// direction (in LSB)
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (software counter)
};

struct mvMove2 {					// Linear moves are queued stepper movements
	uint8_t active_axes; 			// see config.h for bit positions
	struct mvMoveSub a[3];
// 	uint32_t steps[3]; 				// total steps in each direction (abs value)
//	uint8_t direction[3];			// direction (in LSB)
//	uint16_t period[3];				// timer period value
//	uint16_t postscale[3];			// timer postscaler value (software counter)
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

uint8_t mv_queue_move_buffer2(int32_t steps_x, 
							  int32_t steps_y, 
							  int32_t steps_z, 
							  uint32_t microseconds);

#endif

