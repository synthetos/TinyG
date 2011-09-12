/*
 * move_queue.h - routines for managing motor moves 
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
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
 * segment buffer, motor buffer, motor command buffer, joint buffer, or 
 * something that's more descriptive and not in conflict with the 
 * upper-level move buffer used by the planner.
 */

#ifndef motor_queue_h
#define motor_queue_h 

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

enum mqState {
	MQ_BUFFER_READY = 0,			// buffer available for use (MUST BE 0)
	MQ_BUFFER_LOADING,				// buffer being written (pending)
};

struct mqMoveMotor {
	int8_t dir;						// b0 = direction
 	uint32_t steps; 				// total steps in each direction
};

struct mqMove {						// moves queued as stepper ISR parms
	uint8_t mq_type;				// motor move type
	volatile uint8_t mq_state;		// need a mutex for queuing operation
	uint8_t counter_reset_flag;		// set TRUE if counter should be reset
	uint16_t timer_period;			// DDA or dwell clock period setting
	uint32_t timer_ticks;			// DDA or dwell ticks for the move
	uint32_t timer_ticks_X_substeps;// DDA ticks scaled by substep factor
	struct mqMoveMotor a[MOTORS];	// per-motor structs
};

/*
 * Global Scope Functions
 */

void mq_init(void);
uint8_t mq_queue_line(double steps[], double microseconds);
uint8_t mq_queue_dwell(double microseconds);
uint8_t mq_queue_stops(uint8_t move_type);

uint8_t mq_test_motor_buffer(void);
struct mqMove *mq_queue_motor_buffer(void);
struct mqMove *mq_dequeue_motor_buffer(void);
void mq_flush_motor_buffer(void);
void mq_print_motor_queue(void);
void mq_unit_tests(void);

#endif

