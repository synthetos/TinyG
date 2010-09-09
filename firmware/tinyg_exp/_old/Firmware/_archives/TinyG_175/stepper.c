/*
  stepper.c - stepper motor interface
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud
  Modified for TinyG project by Alden Hart, 2010

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 
---- TinyG Notes ----

  --- Line drawing, flow control and synchronization ---

  This code works differently than the Grbl (Reprap) Bresenham implementation.
  Coordinated motion (line drawing) is performed by dedicating a timer to each axis
  and stepping each motor at a computed rate (timer period value) for a specified 
  number of pulses (counter value). Each timeout fires a high-priority interrupt 
  which generates a step and decrements the counter by one.

  The main_loop routines (motion_control.c non-ISR) put lines into the move buffer.
  The timer ISRs read moves from the buffer.

  Any axis that is part of the move has its ACTIVE bit set in ax.active.
  When the axis move is complete this bit is cleared.
  When all active bits are cleared st_execute_move() is called to load the next move
  into the timers.

  But you need some way to start the timers if they are not already running,
  so st_execute_move() must also be called from st_buffer_move() to start line
  execution of the timers are not already running.
  st_execute_move() therefore has a busy flag to prevent ISR and non-ISR invocation
  from stepping on each other,

  st_buffer_move() will sleep if the buffer is full, waiting for a line completion,
  allowing the motion control routines to wake up and generate the next line segment.
  fill up the line buffer then sleep (idle) as the lines from the buffer are executed 
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
#include "debug.h"								// can be deleted for production
#endif


/* 
 * Local Scope Data and Functions
 */

struct stMove {						// Linear moves are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	uint32_t step_counter;			// counts steps down to 0 (end of line)
	uint16_t postscale_value;		// timer post-scale value
	uint16_t postscale_count;		// timer post-scale count
	uint16_t timer_period;

	/* register bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

/* axes main structure
	Holds the individual axis structs
	Active_axes has bit set if axis is active. If thery are all clear the robot is idle
	Pattern is: X_BIT || Y_BIT || Z_BIT || A_BIT  (see config.h)
*/

struct Axes {						// All axes grouped in 1 struct + some extra stuff
	uint8_t active_axes;			// bits are set if axis is active. 0 = robot is idle
	struct Axis x;
	struct Axis y;
	struct Axis z;
	struct Axis a;
};
static struct Axes ax;							// master axes structure

//struct stMoveCtl {
//	unit8_t move_busy;				// TRUE if st_execute_move() is busy
//}

#define MOVE_BUFFER_SIZE 4						// number of lines buffered
static struct stMove move_buffer[MOVE_BUFFER_SIZE];// buffer for line instructions
static struct stMove *mv;
static volatile int move_buffer_head = 0;		// persistent index of written lines
static volatile int move_buffer_tail = 0;		// persistent index of read lines
static volatile int move_busy; 					// TRUE when st_execute_line running 
												// Used to avoid retriggering

void _st_load_timer(struct Axis *a, uint32_t steps, uint32_t ticks_per_step); 
struct stMove *_st_get_next_move(void);			// Return pointer to next move struct

/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	ax.x.step_counter = 0x00001000;
	ax.x.timer->PER = 0x1000;					// step rate (period)
	ax.x.timer->CTRLA = TC_CLK_ON;				// start clock

	ax.y.step_counter = 0x00000800;
	ax.y.timer->PER = 0x2000;
	ax.y.timer->CTRLA = TC_CLK_ON;

	ax.z.step_counter = 0x00000600;
	ax.z.timer->PER = 0x3000;
	ax.z.timer->CTRLA = TC_CLK_ON;

	ax.a.step_counter = 0x00000400;
	ax.a.timer->PER = 0x4000;
	ax.a.timer->CTRLA = TC_CLK_ON;

	ax.active_axes |= (X_ACTIVE_BIT_bm | Y_ACTIVE_BIT_bm | Z_ACTIVE_BIT_bm | A_ACTIVE_BIT_bm);
}

/* st_init() - initialize and start the stepper motor subsystem 
 *
 *	State at completion of initialization is:
 *	- each axis has a structure with an initialized port and a timer bound to it
 *	- ports:
 *		- input and output directions set
 *	- each axis is enabled 
 *	
 *  Note: high level interrupts must be enabled in main()
 *
 */

void st_init()
{
	ax.active_axes = 0;							// clear all active bits
	move_busy = FALSE;								// clear the busy flag

// initialize X axis
	// motor control port
	ax.x.port = &X_MOTOR_PORT;					// bind PORT to structure
	ax.x.port->DIR = X_MOTOR_PORT_DIR_gm;		// set inputs and outputs
	ax.x.port->OUT = 0x00;							// set port bits to zero initially
	ax.x.port->OUT |= MICROSTEP_UNITS_bm;		// set microstep bits
	ax.x.port->OUTSET = MOTOR_ENABLE_BIT_bm; 	// disable the motor

	// motor control timer
	ax.x.timer = &X_TIMER;						// bind TIMER to structure
	ax.x.timer->CTRLA = TC_CLK_OFF;				// turn motor off
	ax.x.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.x.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode

// initialize Y axis
	ax.y.port = &Y_MOTOR_PORT;
	ax.y.port->DIR = Y_MOTOR_PORT_DIR_gm;
	ax.y.port->OUT = 0x00;
	ax.y.port->OUT |= MICROSTEP_UNITS_bm;
	ax.y.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.y.timer = &Y_TIMER;
	ax.y.timer->CTRLA = TC_CLK_OFF;
	ax.y.timer->CTRLB = TC_WGMODE;
	ax.y.timer->INTCTRLA = TC_OVFINTLVL;

// initialize Z axis
	ax.z.port = &Z_MOTOR_PORT;
	ax.z.port->DIR = Z_MOTOR_PORT_DIR_gm;
	ax.z.port->OUT = 0x00;
	ax.z.port->OUT |= MICROSTEP_UNITS_bm;
	ax.z.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.z.timer = &Z_TIMER;
	ax.z.timer->CTRLA = TC_CLK_OFF;
	ax.z.timer->CTRLB = TC_WGMODE;
	ax.z.timer->INTCTRLA = TC_OVFINTLVL;

// initialize A axis
	ax.a.port = &A_MOTOR_PORT;
	ax.a.port->DIR = A_MOTOR_PORT_DIR_gm;
	ax.a.port->OUT = 0x00;
	ax.a.port->OUT |= MICROSTEP_UNITS_bm;
	ax.a.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.a.timer = &A_TIMER;
	ax.a.timer->CTRLA = TC_CLK_OFF;				
	ax.a.timer->CTRLB = TC_WGMODE;
	ax.a.timer->INTCTRLA = TC_OVFINTLVL;

//	st_motor_test();							// run the startup motor test
}


/*
 * ISRs - Motor timer interrupt service routines - service a tick from the axis timer
 *
 *	Use direct struct addresses and literal values for hardware devices becuase it's 
 *  faster than using the timer and port pointers in the axis structs
 */

ISR(X_TIMER_vect)
{
	if (--ax.x.postscale_count != 0) {
		return;
	}
	X_MOTOR_PORT.OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.x.step_counter == 0) {
		X_TIMER.CTRLA = TC_CLK_OFF;				// stop the clock
		X_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		ax.active_axes &= ~X_ACTIVE_BIT_bm;		// clear the X active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_execute_move();					// ...run next line
		}
	}
	ax.x.postscale_count = ax.x.postscale_value;// reset post-scaler counter
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);			// delay for correct pulse width
#endif
	X_MOTOR_PORT.OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_vect)
{
	if (--ax.y.postscale_count != 0) {
		return;
	}
	Y_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.y.step_counter == 0) {
		Y_TIMER.CTRLA = TC_CLK_OFF;
		Y_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Y_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.y.postscale_count = ax.y.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	Y_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(Z_TIMER_vect)
{
	if (--ax.z.postscale_count != 0) {
		return;
	}
	Z_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.z.step_counter == 0) {
		Z_TIMER.CTRLA = TC_CLK_OFF;	
		Z_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Z_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.z.postscale_count = ax.z.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	Z_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(A_TIMER_vect)
{
	if (--ax.a.postscale_count != 0) {
		return;
	}
	A_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a.step_counter == 0) {
		A_TIMER.CTRLA = TC_CLK_OFF;
		A_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~A_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a.postscale_count = ax.a.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	A_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

/*
 * st_execute_move() - dequeue move and load into stepper motors (if possible)
 *
 *	Load next linear move into timers and set direction bits
 *	If the motors are currently active it will load the line
 *	This routine can be called from ISR or non-ISR levels - mediated by "busy"
 *
 *	Busy race condition - there is a brief race condition in the busy test that 
 * 	should not actually cause any problems. If the routine were invoked by 
 *	st_buffer_line() (i.e. non-ISR invocation) an ISR call could occur during 
 *	the busy test; which the ISR *could* find the routine is not f - even 
 *	though it was previously invoked by the non-ISR caller. The interrupt would 
 *	run, loading the next line (or not), then return control to the non-ISR 
 *	invocation. The non-ISR invocation would then find that the axes were active 
 *	(ax.active_axes test), and exit. Alternately, it might find that the axes 
 *	were not active, but exit on the buffer empty test - because this is the 
 *	reason they are not active - the ISR found nothing to load. So please don't 
 *	mess with the ordering of this code region.
 */

int st_execute_move()
{
	uint64_t ticks;	// Timer ticks in the move. 2 minute move would overflow 32 bits
					// Using 64 bits is expensive! The division goes from ~640 
					// cycles at 32 buts to ~3800 cycles using 64 bits/

	// don't re-order this code region - from here...
	if (move_busy) { 	// prevents ISR from clobbering non-ISR invocation
		return (0);
	}
	move_busy = TRUE;
	if (ax.active_axes != 0) {	// exit if any axis is still busy (any bit set)
		move_busy = FALSE;	
		return (0);
	}
	if ((mv = _st_get_next_move()) == NULL) {	// NULL is empty buffer condition
		move_busy = FALSE;
		return (0);
  	}  //...to here. See race condition note.

	ax.active_axes = 0; 						// clear active axes

	// set direction bits (I almost never use the ternary operator, but here goes)
	(mv->steps_x < 0) ? (ax.x.port->OUTSET = DIRECTION_BIT_bm) : 	// CCW
						(ax.x.port->OUTCLR = DIRECTION_BIT_bm);		// CW
	(mv->steps_y < 0) ? (ax.y.port->OUTSET = DIRECTION_BIT_bm) : 
						(ax.y.port->OUTCLR = DIRECTION_BIT_bm);
	(mv->steps_z < 0) ? (ax.z.port->OUTSET = DIRECTION_BIT_bm) : 
						(ax.z.port->OUTCLR = DIRECTION_BIT_bm);
	// load timers
	ticks = mv->microseconds * TICKS_PER_MICROSECOND;

	if (labs(mv->steps_x) != 0) { 								// X axis
		_st_load_timer(&ax.x, labs(mv->steps_x), (uint32_t)(ticks / labs(mv->steps_x)));
		ax.active_axes |= (X_ACTIVE_BIT_bm);					// set X active
	}
	if (labs(mv->steps_y) != 0) {  								// Y axis
		_st_load_timer(&ax.y, labs(mv->steps_y), (uint32_t)(ticks / labs(mv->steps_y)));
		ax.active_axes |= (Y_ACTIVE_BIT_bm);
	}
	if (labs(mv->steps_z) != 0) { 								 // Z axis
		_st_load_timer(&ax.z, labs(mv->steps_z), (uint32_t)(ticks / labs(mv->steps_z)));
		ax.active_axes |= (Z_ACTIVE_BIT_bm);
	}

	// enable the all at the same time (roughly). Better for motor sync.
	if (ax.active_axes || X_ACTIVE_BIT_bm) { ax.x.timer->CTRLA = TC_CLK_ON; }
	if (ax.active_axes || Y_ACTIVE_BIT_bm) { ax.y.timer->CTRLA = TC_CLK_ON; }
	if (ax.active_axes || Z_ACTIVE_BIT_bm) { ax.z.timer->CTRLA = TC_CLK_ON; }

#ifdef __DEBUG
	st_print_exec_line(*ln, ax.active_axes);
#endif

	move_busy = FALSE;
	return (0);
}

/* _st_load_timer() - helper routine for st_execute_move() 
 *
 *	The ISR should also be modified to end each move on a whole-step boundary 
 *	for power management reasons, and possibly revert the microsteps to whole
 *	if necessary to do this.
 */

void _st_load_timer(struct Axis *A, uint32_t steps, uint32_t ticks_per_step) 
{
	A->timer->CTRLA = TC_CLK_OFF;				// turn clock off, just to be sure
	A->port->OUTCLR = MOTOR_ENABLE_BIT_bm;		// enable motor
	A->step_counter = steps;

	// Normalize ticks_per_step by right shifting until the MSword is zero.
	// Accumulate LSBs shifted out of ticks_per_step into postscale_value.
	A->postscale_value = 1;
	while (ticks_per_step & 0xFFFF0000) {
		ticks_per_step >>= 1;					//(ticks_per_step >> 1);
		A->postscale_value <<= 1;
	}
	A->postscale_count = A->postscale_value;
	A->timer_period = (uint16_t)(ticks_per_step & 0x0000FFFF);
	A->timer->PER = (uint16_t)(ticks_per_step & 0x0000FFFF); // timer period
}

/*
 * st_buffer_move() - Add a new linear movement to the move buffer
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

void st_buffer_move(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds) 
{
	int next_buffer_head_tmp;

	// Bail on a zero length line (perhaps test for abs val < min line length)
	if ((steps_x == 0) && (steps_y == 0) && (steps_z) == 0) {
		return;
	};

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head_tmp = move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head_tmp = 0;					 // wrap condition
	}

	// If the buffer is full sleep until there is room in the buffer.
	while(move_buffer_tail == next_buffer_head_tmp) { // tail will advance, breaking this loop
		sleep_mode(); 
	}

	mv = &move_buffer[move_buffer_head];
	mv->steps_x = steps_x; 						// write the line record to the buffer head
	mv->steps_y = steps_y;
	mv->steps_z = steps_z;  
	mv->microseconds = microseconds;

	move_buffer_head = next_buffer_head_tmp;
	st_execute_move();							// run this line
}

/*
 * st_buffer_full() - Test if the move buffer is full
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

uint8_t st_buffer_full() 
{
	int next_buffer_head_tmp;

	if ((next_buffer_head_tmp = move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head_tmp = 0;					  // wrap condition
	}
	if (move_buffer_tail == next_buffer_head_tmp) { 	// test if full
		return (TRUE);
	};
	return (FALSE);
}
/* 
 * _st_get_next_move() - return the next line from the move buffer & advance buffer tail
 */ 

struct stMove *_st_get_next_move()
{
	if (move_buffer_head == move_buffer_tail) {	// buffer empty
		return (NULL);
	}
	mv = &move_buffer[move_buffer_tail];		// get and save current index
	if (++move_buffer_tail >= MOVE_BUFFER_SIZE) { // increment and wrap (no OBOE)
		move_buffer_tail = 0;
	}
	return (mv); 
}

/* 
 * st_synchronize() - block until all buffered steps are executed 
 */

void st_synchronize()
{
	while(move_buffer_tail != move_buffer_head) {
		sleep_mode();
	}    
}

/* 
 * st_flush() - cancel all buffered steps 
 */

void st_flush()
{
	cli();
	move_buffer_tail = move_buffer_head;
//	ln = NULL;
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
	
	move_buffer_tail = move_buffer_head;		// clear the move buffer
	ax.active_axes = 0;							// clear all the active bits
	sei();
}

/* 
 * st_terminate() - stop moves after the current move
 */

void st_terminate()
{
	cli();
	move_buffer_tail = move_buffer_head;		// clear the line buffer
	sei();
}

/* 
 * st_go_home() - perform the homing cycle 
 */

void st_go_home()
{
  // Todo: Perform the homing cycle
}

