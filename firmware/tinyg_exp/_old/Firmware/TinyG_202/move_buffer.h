/*
 * move_buffer.h - routines for managing motor moves
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef move_buffer_h
#define move_buffer_h 

/*
 * pre-computed move buffer entry structure
 */

struct mvMoveAxis {
 	int32_t steps; 					// total steps in each direction
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (software counter)
	uint8_t direction;				// direction (in LSB)
};

struct mvMove {						// Linear moves are queued as stepper ISR parameters
	struct mvMoveAxis a[3];			// axis structs
};

/*
 * Global Scope Functions
 */

void mv_init(void);
uint8_t mv_queue_move_buffer(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds);
struct mvMove *mv_dequeue_move_buffer(void);
uint8_t mv_test_move_buffer_full(void);
void mv_flush(void);				// Cancel all pending steps
void mv_synchronize(void);			// Block until all buffered steps are executed

#endif

