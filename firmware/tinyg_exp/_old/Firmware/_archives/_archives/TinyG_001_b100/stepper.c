/*
  stepper.c - stepper motor interface
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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
*/
/* 
  TinyG Notes
  Modified to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

  --- Line drawing, flow control and synchronization ---

  This code works differently than the Reprap or Grbl Bresenham implementations.
  Coordinated motion (line drawing) is performed by dedicating a timer to each axis
  and stepping each motor at a computed rate (timer period value) for a specified 
  number of pulses (counter value).Each timeout fires a high-priority interrupt 
  which generates a step and decrements the counter by one.

  The main_loop routines (motion_control.c non-ISR) put lines into the line buffer.
  using st_buffer_line().
  The timer ISRs read lines from the buffer (via st_execute_line()), as so:

  Any axis that is part of the move has it's ACTIVE bit set in ax.active.
  When the axis move is complete this bit is cleared.
  When all active bits are cleared st_execute_line() is called to run the next line.

  But you need some way to start the timers if they are not already running.
  So st_execute_line() must also be called from st_buffer_line() to start line
  execution of the timers are not already running.
  st_execute_line() therefore has a busy flag to prevent ISR and non-ISR invocation
  from stepping on each other,

  st_buffer_line() will sleep if the buffer is full, waiting for a line completion,
  allowing the motion control routines to wake up and generate the next line segment.
  fill up the line buffer then sleep (idle) as the lines from the buffer are executed 

*/

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_support.h"
#include <util/delay.h>
#include "stepper.h"
#include "config.h"
#include "nuts_bolts.h"
#include "debug.h"								// this can be deleted for production

struct Axes ax;									// master axes structure

#define LINE_BUFFER_SIZE 2						// number of lines buffered
struct Line line_buffer[LINE_BUFFER_SIZE];		// buffer for line instructions
struct Line *ln = NULL;							// pointer to current line
volatile int line_buffer_head = 0;				// persistent index of written lines
volatile int line_buffer_tail = 0;				// persistent index of read lines
volatile int busy; // TRUE when st_execute_line is running. Used to avoid retriggering


/* st_motor_test() - test motor subsystem */

void st_motor_test() {
	ax.x.counter = 0x00001000;					// number of steps
	ax.x.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.x.timer->PERH = 0x10;					// step rate (period) high
	ax.x.timer->PERL = 0x00;					// step rate (period) low

	ax.y.counter = 0x00000800;					// number of steps
	ax.y.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.y.timer->PERH = 0x20;					// step rate (period) high
	ax.y.timer->PERL = 0x00;					// step rate (period) low

	ax.z.counter = 0x00000600;					// number of steps
	ax.z.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.z.timer->PERH = 0x30;					// step rate (period) high
	ax.z.timer->PERL = 0x00;					// step rate (period) low

	ax.a.counter = 0x00000400;					// number of steps
	ax.a.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.a.timer->PERH = 0x40;					// step rate (period) high
	ax.a.timer->PERL = 0x00;					// step rate (period) low

	ax.active_axes |= (X_BIT_bm | Y_BIT_bm | Z_BIT_bm | A_BIT_bm);
}


/* st_init() - initialize and start the stepper motor subsystem 

   State at completion of initialization is:
	- each axis has a structure with an initialized port and a timer bound to it
	- ports:
		- input and output directions set
	- each axis is enabled 
	
   Note: high level interrupts must be enabled in main()

*/

void st_init()
{
	ax.active_axes = 0;							// clear all active bits
	busy = FALSE;								// clear the busy flag

 /* initialize X axis */
 	// operating variables
	ax.x.counter = 0;							// down counts timer steps 

	// configuration variables
	ax.x.microsteps = X_MICROSTEPS;
	ax.x.max_seek_rate = X_SEEK_WHOLE_STEPS_PER_SEC;
	ax.x.max_seek_steps = X_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.x.max_feed_rate = DEFAULT_FEEDRATE;
	ax.x.max_feed_steps = X_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.x.steps_per_mm = X_STEPS_PER_MM;

	// motor control port bound to structure 
	ax.x.port = &X_MOTOR_PORT;
	ax.x.port->DIR = X_MOTOR_PORT_DIR_gm;		// set inputs and outputs
	ax.x.port->OUT = 0;							// set port bits to zero initially
	ax.x.port->OUT |= MICROSTEP_EIGHTH_bm;		// set microstep bits to eighth

	// motor control timer channel bound to structure
	ax.x.timer = &X_TIMER;
	ax.x.timer->CTRLA = TC_CLK_OFF;				// turn motor off
	ax.x.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.x.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	ax.x.timer->PERH = 0x00;					// period high
	ax.x.timer->PERL = 0x00;					// period low

 /* initialize Y axis */
	ax.y.counter = 0;

	ax.y.microsteps = Y_MICROSTEPS;
	ax.y.max_seek_rate = Y_SEEK_WHOLE_STEPS_PER_SEC;
	ax.y.max_seek_steps = Y_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.y.max_feed_rate = DEFAULT_FEEDRATE;
	ax.y.max_feed_steps = Y_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.y.steps_per_mm = Y_STEPS_PER_MM;

	ax.y.port = &Y_MOTOR_PORT;					// bind port to structure 
	ax.y.port->DIR = Y_MOTOR_PORT_DIR_gm;
	ax.y.port->OUT = 0;							// set port bits to zero initially
	ax.y.port->OUT |= MICROSTEP_EIGHTH_bm;		// set microstep bits to eighth

	ax.y.timer = &Y_TIMER;
	ax.y.timer->CTRLA = TC_CLK_OFF;				// default division ratio
	ax.y.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.y.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	ax.y.timer->PERH = 0x00;					// period high
	ax.y.timer->PERL = 0x00;					// period low

/* initialize Z axis */
	ax.z.counter = 0;

	ax.z.microsteps = Z_MICROSTEPS;
	ax.z.max_seek_rate = Z_SEEK_WHOLE_STEPS_PER_SEC;
	ax.z.max_seek_steps = Z_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.z.max_feed_rate = DEFAULT_FEEDRATE;
	ax.z.max_feed_steps = Z_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.z.steps_per_mm = Z_STEPS_PER_MM;

	ax.z.port = &Z_MOTOR_PORT;
	ax.z.port->DIR = Z_MOTOR_PORT_DIR_gm;
	ax.z.port->OUT = 0;							// set port bits to zero initially
	ax.z.port->OUT |= MICROSTEP_EIGHTH_bm;		// set microstep bits to eighth

	ax.z.timer = &Z_TIMER;
	ax.z.timer->CTRLA = TC_CLK_OFF;				// timer clock control or division
	ax.z.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.z.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	ax.z.timer->PERH = 0x00;					// period high
	ax.z.timer->PERL = 0x00;					// period low

/* initialize A axis */
	ax.a.counter = 0;

	ax.a.microsteps = A_MICROSTEPS;
	ax.a.max_seek_rate = A_SEEK_WHOLE_STEPS_PER_SEC;
	ax.a.max_seek_steps = A_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.a.max_feed_rate = DEFAULT_FEEDRATE;
	ax.a.max_feed_steps = A_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.a.steps_per_mm = A_STEPS_PER_MM;

	ax.a.port = &A_MOTOR_PORT;
	ax.a.port->DIR = A_MOTOR_PORT_DIR_gm;
	ax.a.port->OUT = 0;							// set port bits to zero initially
	ax.a.port->OUT |= MICROSTEP_EIGHTH_bm;		// set microstep bits to eighth

	ax.a.timer = &A_TIMER;
	ax.a.timer->CTRLA = TC_CLK_OFF;				
	ax.a.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.a.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	ax.a.timer->PERH = 0x00;					// period high
	ax.a.timer->PERL = 0x00;					// period low

	st_motor_test();							// run the startup motor test
}


/*******************************************************************************
 Motor timer interrupt service routines - service a tick from the axis timer
*******************************************************************************/

ISR(X_TIMER_vect)
{
	ax.x.port->OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.x.counter == 0) {
		ax.x.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~X_BIT_bm;			// clear the X active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_print_done_line("X");			// ++++ DEBUG STATEMENT ++++
			st_execute_line();					// ...run next line
		}
	}
//	_delay_us(STEP_PULSE_MICROSECONDS);			// delay for correct pulse width
	ax.x.port->OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_vect)
{
	ax.y.port->OUTSET = STEP_BIT_bm;
	Y_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.y.counter == 0) {
		ax.y.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~Y_BIT_bm;			// clear the Y active bit
		if (ax.active_axes == 0) {
			st_print_done_line("Y");			// ++++ DEBUG STATEMENT ++++
			st_execute_line();					// run next line if all axes are done
		}
	}
//	_delay_us(STEP_PULSE_MICROSECONDS);
	Y_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

ISR(Z_TIMER_vect)
{
	Z_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.z.counter == 0) {
		ax.z.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~Z_BIT_bm;			// clear the Z active bit
		if (ax.active_axes == 0) {
			st_print_done_line("Z");			// ++++ DEBUG STATEMENT ++++
			st_execute_line();					// run next line if all axes are done
		}
	}
//	_delay_us(STEP_PULSE_MICROSECONDS);
	Z_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

ISR(A_TIMER_vect)
{
	A_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.a.counter == 0) {
		ax.a.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~A_BIT_bm;			// clear the A active bit
		if (ax.active_axes == 0) {
			st_print_done_line("A");			// ++++ DEBUG STATEMENT ++++
			st_execute_line();					// run next line if all axes are done
		}
	}
//	_delay_us(STEP_PULSE_MICROSECONDS);
	A_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}


/***********************************************************************************
  st_execute_line() - run next line if warranted

  Load next line into timers and set direction bits
  If the line is currently active it will not do the load
  If the routine is busy it will not do the load
  Busy flag is needed as the routine may be called by either an ISR or non-ISR,
  and we don't want it to execute over itself.

  Variables in ln mean:
	ln->steps_x = steps to take in X 
	ln->steps_y = steps to take in Y
	ln->steps_z = steps to take in Z
	ln->rate = total microseconds the move should take - calculates step rate

	step_rate is computed as steps_ / microseconds

  Busy race condition - there is a brief race condition in the busy test that should 
  not actually cause any problems. If the routine were invoked by st_buffer_line() 
  (i.e. non-ISR invocation) an ISR call could occur during the busy test; which
  the ISR *could* find the routine is not busy - even though it was previously
  invoked by the non-ISR caller. The interrupt would run, loading the next line 
  (or not), then return control to the non-ISR invocation. The non-ISR invocation 
  would then find that the axes were active (ax.active_axes test), and exit.
  Alternately, it might find that the axes were not active, but exit on the buffer 
  empty test - because this is the reason they are not active - the ISR found nothing
  to load. So don't mess with the order of this code region.

***********************************************************************************/

void st_execute_line()
{
	struct Line *ln;

	// don't re-order this code region - from here...
	if (busy) { 				// prevents ISR from clobbering non-ISR invocation
		return;
	}
	busy = TRUE;
	if (ax.active_axes != 0) {	// exit if any axis is still busy (any bit set)
		busy = FALSE;	
		return;
	}
	if ((ln = st_get_next_line()) == NULL) {	// empty buffer condition
		busy = FALSE;	
		return;
  	} 
	//...to here. See race condition note.

	// set direction bits
	(ln->steps_x < 0) ? (ax.x.port->OUTSET=DIRECTION_BIT_bm) : 	// CCW
						(ax.x.port->OUTCLR=DIRECTION_BIT_bm);	// CW
	(ln->steps_y < 0) ? (ax.y.port->OUTSET=DIRECTION_BIT_bm) : 
						(ax.y.port->OUTCLR=DIRECTION_BIT_bm);
	(ln->steps_z < 0) ? (ax.z.port->OUTSET=DIRECTION_BIT_bm) : 
						(ax.z.port->OUTCLR=DIRECTION_BIT_bm);

	// load timers: step rate = microseconds / absolute value of step count
	ax.active_axes = 0;											// clear active axes

	if (labs(ln->steps_x) != 0) { 								// X axis
		_st_load_timer(&ax.x, (ln->microseconds / labs(ln->steps_x)), ln->microseconds);
		ax.active_axes |= (X_BIT_bm);							// set X active
	}
	if (labs(ln->steps_y) != 0) {  // Y axis
		_st_load_timer(&ax.y, (ln->microseconds / labs(ln->steps_y)), ln->microseconds);
		ax.active_axes |= (Y_BIT_bm);
	}
	if (labs(ln->steps_z) != 0) {  // Z axis
		_st_load_timer(&ax.z, (ln->microseconds / labs(ln->steps_z)), ln->microseconds);
		ax.active_axes |= (Z_BIT_bm);
	}
		
	st_print_exec_line(*ln, ax.active_axes);	// ++++ DEBUG STATEMENT ++++

	busy = FALSE;
	return;
}

/* _st_load_timer() - helper routine for st_execute line 

  Note: this routine and the ISRs should be modified to always use the highest 
  clock rate and to drop pulses in the ISR (post scaling instead of prescaling)
  This will preserve clock accuracy at very low step rates - which is something 
  of a problem right now.
  
  The ISR should also be modified to end each move on a whole-step boundary 
  for power management reasons, and possibly revert the microsteps to whole
  if necessary to do this.  
*/

void _st_load_timer(struct Axis *A, uint32_t step_rate, uint32_t microseconds) 
{
//	st_print_four_ints( a->counter, step_rate, a->timer->CTRLA, microseconds);

	if (step_rate < DIV1_RANGE) {				// short timer - up to 2000 uSec
		A->timer->CTRLA = TC_CLK_DIV_1;			// set clock divisor
		A->counter = (microseconds/step_rate);	// # of steps to make at this rate
		step_rate = (step_rate * 32);			// normalize step rate to timer clock
	} else if (step_rate < DIV2_RANGE) {
		A->timer->CTRLA = TC_CLK_DIV_2;	
		A->counter = (microseconds/step_rate);
		step_rate = (step_rate * 16);
	} else if (step_rate < DIV4_RANGE) {
		A->timer->CTRLA = TC_CLK_DIV_4;	
		A->counter = (microseconds/step_rate);
		step_rate = (step_rate * 8);
	} else if (step_rate < DIV8_RANGE) {
		A->timer->CTRLA = TC_CLK_DIV_8;	
		A->counter = (microseconds/step_rate);
		step_rate = (step_rate * 4);
	} else if (step_rate < DIV64_RANGE) {
		A->timer->CTRLA = TC_CLK_DIV_64;	
		A->counter = (microseconds/step_rate);
		step_rate = (step_rate / 2);
	} else if (step_rate < DIV256_RANGE) {
		A->timer->CTRLA = TC_CLK_DIV_256;	
		A->counter = (microseconds/step_rate);
		step_rate = (step_rate / 8);
	}
	A->timer->PERH = (uint8_t)((step_rate >> 8) & 0x000000FF);	// period high
	A->timer->PERL = (uint8_t)(step_rate & 0x000000FF);			// period low
}

/* st_get_next_line() - return the next line from the line buffer & advance buffer tail*/ 

struct Line *st_get_next_line()
{
	struct Line *ln;
	 
	if (line_buffer_head == line_buffer_tail) {	// buffer empty
		return (NULL);
	}
	ln = &line_buffer[line_buffer_tail];		 // get and save the current pointer
	if (++line_buffer_tail > LINE_BUFFER_SIZE) { // increment and wrap
		line_buffer_tail = 0;
	}
	return (ln); 
}

/***********************************************************************************
  st_buffer_line()

	Add a new linear movement to the buffer.
	steps_x, _y and _z is the signed, relative motion in steps. 
	Microseconds specify how many microseconds the move should take to perform.

  Line buffer circular buffer operation
    line_buffer_tail is the array index from which the previous line was read.
	line_buffer_tail is always incremented before reading the line.

	line_buffer_head is the array index to which the line will be written.
	line_buffer_head is always incremented after writing the line.

	Buffer full:	line_buffer_head == line_buffer_tail
	Buffer empty:	line_buffer_head == line_buffer_tail+1 

***********************************************************************************/

void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds) 
{
	struct Line *ln;
	int next_buffer_head;

	// bail on a zero length line (perhaps test for abs val < min line length)
	if ((steps_x == 0) && (steps_y == 0) && (steps_z) == 0) {
		return;
	};

	// Calculate the buffer head needed to store this line
	if ((next_buffer_head = line_buffer_head + 1) > LINE_BUFFER_SIZE) {
		next_buffer_head = 0;					  // wrap condition
	}
	// If the buffer is full sleep until there is room in the buffer.
	while(line_buffer_tail == next_buffer_head) { // tail will advance, breaking this loop
		sleep_mode(); 
//		_delay_us(10);
	};

	ln = &line_buffer[line_buffer_head];  		// write the line record to the buffer head
	ln->steps_x = steps_x;
	ln->steps_y = steps_y;
	ln->steps_z = steps_z;  
	ln->microseconds = microseconds;

	line_buffer_head = next_buffer_head;

	st_print_line(*ln);							// ++++ DEBUG STATEMENT ++++
	st_execute_line();							// attempt to run this line
}

/* st_synchronize() - block until all buffered steps are executed */

void st_synchronize()
{
	while(line_buffer_tail != line_buffer_head) {
		sleep_mode();
	}    
}

/* st_flush() - cancel all buffered steps */

void st_flush()
{
	cli();
	line_buffer_tail = line_buffer_head;
	ln = NULL;
	sei();
}

/* st_go_home() - perform the homing cycle */

void st_go_home()
{
  // Todo: Perform the homing cycle
}


