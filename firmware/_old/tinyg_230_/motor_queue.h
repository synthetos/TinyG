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
/*
 * At some point this whole thing ought to be renamed as the line buffer,
 * segment buffer, motor buffer, mtor command buffer, or something that's
 * more descriptive and not in conflict wjt the upper-level move buffer.
 */

#ifndef motor_queue_h
#define motor_queue_h 
#include "tinyg.h"

/*
 * pre-computed move buffer entry structure
 */

enum mqType {
	MQ_NONE,
	MQ_LINE,
	MQ_DWELL,
	MQ_START,
	MQ_STOP,
	MQ_END
};

struct mqMoveAxis {
	int8_t direction;				// b0 = direction
 	int32_t steps; 					// total steps in each direction
	uint16_t period;				// timer period value
	uint16_t postscale;				// timer postscaler value (sw counter)
};

struct mqMove {						// moves are queued as stepper ISR parms
	uint8_t mq_type;				// motor move type
	struct mqMoveAxis a[AXES];		// axis structs
};

/*
 * Global Scope Functions
 */

void mq_init(void);
uint8_t mq_queue_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, 
					  int32_t steps_a, uint32_t microseconds);
uint8_t mq_queue_dwell(uint32_t microseconds);
uint8_t mq_queue_stops(uint8_t move_type);

uint8_t mq_test_motor_buffer(void);
struct mqMove *mq_queue_motor_buffer(void);
struct mqMove *mq_dequeue_motor_buffer(void);
void mq_flush_motor_buffer(void);

#endif

