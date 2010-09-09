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

  This code works differently than the Reprap or GRBL bresenham implementations.
  Each axis gets a dedicated timer running at very high frequency (e.g. 4 Mhz).
  Moves are made by using the dedicated timers to set the step rates for each axis.
  All 3 (or 4) moves are run independently, but started and ended simultaneously.
  Moves are executed using optimal setting based on the following variables:
	- timer prescale value that provides best time resolution for feed speed
   	- optimal step rate range for the motor - balancing smoothness and torque 
		(e.g. 200 - 1200 steps/sec)
    - finest microstepping setting that puts the step rate in the optimal range

 Todo:
 --- fix line buffer length HW dependency
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

#include "wiring_serial.h"	// ++++ NEEDED FOR DEBUG ONLY - OTHERWISE NO PRINTING
#include <avr/pgmspace.h>

struct Line {
 	uint32_t steps_x; 
	uint32_t steps_y; 
	uint32_t steps_z;
 	int32_t maximum_steps;
 	uint8_t direction_bits;
 	uint32_t rate;
};

#define LINE_BUFFER_SIZE 40					// number of lines buffered
struct Line line_buffer[LINE_BUFFER_SIZE];	// buffer for line instructions
struct Line *ln = NULL;						// pointer to current line
volatile int line_buffer_head = 0;
volatile int line_buffer_tail = 0;

struct axis { 						// one instance per axis
	/* operating  variables */
	int32_t counter;				// counts steps down to 0 (end of line)
//	uint8_t enable;					// set TRUE to enable axis
//	uint8_t dir;					// NEED SOME NOTE AS TO WHAT SENSE THE BIT IS
	/* configuration variables */
	double max_feed_rate;			// maximum speed under load in mm per minute
	double max_feed_steps;			// maximum steps under load in steps per second
	double max_seek_rate;			// maximum speed under no load in mm per minute
	double max_seek_steps;			// maximum steps under no load in steps / second
	double steps_per_mm;			// steps per mm traveled for this axis
	/* register bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

/* axes main structure
	Holds the individual axis structs
	Active_axes has bit set if axis is active. If thery are all clear the robot is idle
	Pattern is: X_BIT || Y_BIT || Z_BIT || A_BIT  (see config.h)
*/

struct axes {						// collect them all up and make easy access
	struct axis x;
	struct axis y;
	struct axis z;
	struct axis a;
	uint8_t active_axes;			// bit set if axis is active. 0 = robot is idle
};
struct axes ax;


/* st_motor_test() - test motor subsystem */

void st_motor_test() {
	ax.x.counter = 0x00004000;					// number of steps
	ax.x.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.x.timer->PERH = 0x10;					// step rate (period) high
	ax.x.timer->PERL = 0x00;					// step rate (period) low

	ax.y.counter = 0x00004000;					// number of steps
	ax.y.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.y.timer->PERH = 0x20;					// step rate (period) high
	ax.y.timer->PERL = 0x00;					// step rate (period) low

	ax.z.counter = 0x00004000;					// number of steps
	ax.z.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.z.timer->PERH = 0x30;					// step rate (period) high
	ax.z.timer->PERL = 0x00;					// step rate (period) low

	ax.a.counter = 0x00004000;					// number of steps
	ax.a.timer->CTRLA = TC_CLK_DIV_1;			// clock division ratio
	ax.a.timer->PERH = 0x40;					// step rate (period) high
	ax.a.timer->PERL = 0x00;					// step rate (period) low

	ax.active_axes |= X_BIT || Y_BIT || Z_BIT || A_BIT;

}


/* st_init() - initialize and start the stepper motor subsystem 

   State at completion of initialization is:
	- each axis has a structure wth an initialized port and a timer bound to it
	- ports:
		- input and output directions set
	- each axis is enabled (

*/

void st_init()
{
	ax.active_axes = 0;							// clear all active bits

 /* initialize X axis */
 	// operating variables
	ax.x.counter = 0;							// down counts timer steps 
//	ax.x.enable = TRUE;							// driver enable - electrical level
//	ax.x.dir = 0;								// direction value

	// configuration variables
	ax.x.max_feed_rate = DEFAULT_FEEDRATE;
	ax.x.max_feed_steps = X_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.x.max_seek_rate = RAPID_FEEDRATE;
	ax.x.max_seek_rate = X_STEPS_PER_MM / DEFAULT_FEEDRATE;
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
//	ax.y.enable = TRUE;
//	ax.y.dir = 1;

	ax.y.max_feed_rate = DEFAULT_FEEDRATE;
	ax.y.max_feed_steps = Y_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.y.max_seek_rate = RAPID_FEEDRATE;
	ax.y.max_seek_rate = Y_STEPS_PER_MM / DEFAULT_FEEDRATE;
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
//	ax.z.enable = TRUE;
//	ax.z.dir = 0;

	ax.z.max_feed_rate = DEFAULT_FEEDRATE;
	ax.z.max_feed_steps = Z_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.z.max_seek_rate = RAPID_FEEDRATE;
	ax.z.max_seek_rate = Z_STEPS_PER_MM / DEFAULT_FEEDRATE;
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
//	ax.a.enable = TRUE;
//	ax.a.dir = 0;

	ax.a.max_feed_rate = DEFAULT_FEEDRATE;
	ax.a.max_feed_steps = A_STEPS_PER_MM / DEFAULT_FEEDRATE;
	ax.a.max_seek_rate = RAPID_FEEDRATE;
	ax.a.max_seek_rate = A_STEPS_PER_MM / DEFAULT_FEEDRATE;
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

	// high level interrupts must be enabled in main()

	st_motor_test();							// run the motor test
}

/* Motor timer interrupt service routines - service a tick from the axis timer */

ISR(X_TIMER_vect)
{
	ax.x.port->OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.x.counter == 0) {
		ax.x.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~X_BIT;				// clear the X active bit
	}
//	_delay_us(STEP_PULSE_TIME);					// delay for correct pulse width
	ax.x.port->OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_vect)
{
	ax.y.port->OUTSET = STEP_BIT_bm;
	Y_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.y.counter == 0) {
		ax.y.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~Y_BIT;				// clear the Y active bit
	}
//	_delay_us(STEP_PULSE_TIME);
	Y_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

ISR(Z_TIMER_vect)
{
	Z_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.z.counter == 0) {
		ax.z.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~Z_BIT;				// clear the Z active bit
	}
//	_delay_us(STEP_PULSE_TIME);
	Z_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

ISR(A_TIMER_vect)
{
	A_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (--ax.a.counter == 0) {
		ax.a.timer->CTRLA = TC_CLK_OFF;			// stop the clock
		ax.active_axes &= ~A_BIT;				// clear the A active bit
	}
//	_delay_us(STEP_PULSE_TIME);
	A_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

/*******************************************************************************/
/* st_run_next_line() - run next line if warranted

   Function:
	The line can be in one of 2 states - active or idle
	If active - return
		- Exit if they are not all zero (could check for runaways here)
	If idle
		- load next line helper routine 
		- get a line from the buffer or return if empty
		- compute all relevant variables and start the line
*/
/*******************************************************************************/

void st_run_next_line()
{
	if (ax.active_axes != 0) {
		return;								// if any bit is set the robot is active
	}
	printPgmString(PSTR("IDLE "));		// ++++ DEBUG CODE
	printHex(ax.active_axes);
	printPgmString(PSTR("\r\n"));		// ++++ DEBUG CODE

	if ((ln = st_get_next_line()) == NULL) {
		return;
  	} 
	st_print_line(*ln);						// ++++ DEBUG CODE
}

/* st_get_next_line() - return the next line from the line buffer */ 

struct Line *st_get_next_line()
{
	if (line_buffer_head == line_buffer_tail) {	// buffer empty
		return (NULL);
	}
	if (++line_buffer_tail > LINE_BUFFER_SIZE) { // increment and wrap
		line_buffer_tail = 0;
	}
	return (&line_buffer[line_buffer_tail]); 
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

/* st_buffer_line()

	Add a new linear movement to the buffer. 
	steps_x, _y and _z is the signed, relative motion in steps. 
	Microseconds specify how many microseconds the move should take to perform.
*/

void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds) 
{
	// Calculate the buffer head after we push this byte
	int next_buffer_head = (line_buffer_head + 1) % LINE_BUFFER_SIZE;	

	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Nap until there is room in the buffer.
	while(line_buffer_tail == next_buffer_head) { 
		sleep_mode(); 
	};

	struct Line *line = &line_buffer[line_buffer_head];  	// Setup line record
	line->steps_x = labs(steps_x);
	line->steps_y = labs(steps_y);
	line->steps_z = labs(steps_z);  
	line->maximum_steps = max(line->steps_x, max(line->steps_y, line->steps_z));

  	if (line->maximum_steps == 0) { 		// Bail if this is a zero-length line
		return;
	};

	line->rate = microseconds/line->maximum_steps;
	uint8_t direction_bits = 0;
	if (steps_x < 0) { 
		direction_bits |= (1<<X_DIRECTION_BIT); 
	}
	if (steps_y < 0) { 
		direction_bits |= (1<<Y_DIRECTION_BIT); 
	}
	if (steps_z < 0) { 
		direction_bits |= (1<<Z_DIRECTION_BIT); 
	}
	line->direction_bits = direction_bits;
	st_print_line(*line);						// ++++ DEBUG CODE
	line_buffer_head = next_buffer_head;		// Move buffer head
}

/* st_go_home() - perform the homing cycle */

void st_go_home()
{
  // Todo: Perform the homing cycle
}




void st_print_line(struct Line line) {
	printPgmString(PSTR("Line X="));		// ++++ DEBUG CODE
	printInteger(line.steps_x);
	printPgmString(PSTR(", Y="));
	printInteger(line.steps_y);
	printPgmString(PSTR(", Z="));
	printInteger(line.steps_z);
	printPgmString(PSTR("\r\n"));
}
