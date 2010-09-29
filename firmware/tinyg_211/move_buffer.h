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
 * types of moves that can be put in the move queue 
 */

enum mvType {
	MV_TYPE_LINE,
	MV_TYPE_DWELL,
	MV_TYPE_START,
	MV_TYPE_STOP,
	MV_TYPE_END
};

/*
 * pre-computed move buffer entry structure
 */

struct mvMoveAxis {
	int8_t direction;				// b0 = direction
 	int32_t steps; 					// total steps in each direction
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (sw counter)
};

struct mvMove {						// moves are queued as stepper ISR parms
	uint8_t move_type;				// move type
	struct mvMoveAxis a[4];			// axis structs (XYZA)
};

/*
 * Global Scope Functions
 */

void mv_init(void);
uint8_t mv_queue_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds);
uint8_t mv_queue_dwell(uint32_t microseconds);
uint8_t mv_queue_start_stop(uint8_t move_type);	// queue stops and starts
struct mvMove *mv_dequeue_move_buffer(void);
uint8_t mv_test_move_buffer_full(void);
void mv_flush(void);				// Cancel all pending steps

#endif

