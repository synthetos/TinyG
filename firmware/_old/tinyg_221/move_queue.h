/*
 * move_queue.h - routines for managing motor moves
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef move_queue_h
#define move_queue_h 
#include "tinyg.h"

/*
 * pre-computed move buffer entry structure
 *
 * A move buffere entry can either carry a line sgement or a dwell timing.
 * A dwell command is indicated by b1 set in the direction.
 * The move will the ISRs run as normal, but no pulses will be issued.
 */

enum mvType {
	MOVE_TYPE_LINE,
	MOVE_TYPE_DWELL,
	MOVE_TYPE_START,
	MOVE_TYPE_STOP,
	MOVE_TYPE_END
};

struct mvMoveAxis {
	int8_t direction;				// b0 = direction
 	int32_t steps; 					// total steps in each direction
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (sw counter)
};

struct mvMove {						// moves are queued as stepper ISR parms
	uint8_t move_type;				// move type
	struct mvMoveAxis a[AXES];		// axis structs
};

/*
 * Global Scope Functions
 */

void mv_init(void);
uint8_t mv_queue_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, int32_t steps_a, uint32_t microseconds);
uint8_t mv_queue_dwell(uint32_t microseconds);
uint8_t mv_queue_start_stop(uint8_t move_type);
struct mvMove *mv_dequeue_move_buffer(void);
uint8_t mv_test_move_buffer_full(void);
void mv_flush(void);				// Cancel all pending steps

#endif

