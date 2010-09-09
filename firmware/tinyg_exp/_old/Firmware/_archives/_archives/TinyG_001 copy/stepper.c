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
  Each axis gets a dedicated timer running at very high frequency (e.g. 2 Mhz).
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
#include "wiring_serial.h"

#define LINE_BUFFER_SIZE 64		// number of lines buffered in steppers.c

struct Line {
 	uint32_t steps_x, steps_y, steps_z;
 	int32_t maximum_steps;
 	uint8_t direction_bits;
 	uint32_t rate;
};

struct Line line_buffer[LINE_BUFFER_SIZE];	// A buffer for step instructions
volatile int line_buffer_head = 0;
volatile int line_buffer_tail = 0;

unsigned long x_move_counter;				// counts steps down to 0 (end of move)
unsigned long y_move_counter;
unsigned long z_move_counter;
unsigned long a_move_counter;


/* Variables used by SIG_OUTPUT_COMPARE1A */

uint8_t out_bits; 			// The next stepping-bits to be output
struct Line *current_line;	// A pointer to the line currently being traced
uint32_t iterations; 		// The number of iterations left to complete the current_line
volatile int busy;			// TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
volatile int32_t counter_x, counter_y, counter_z; // counter variables for the bresenham line tracer


/* st_init() - initialize and start the stepper motor subsystem
	Also initializes the encoder out bits as outputs
*/
void st_init()
{
	X_MOTOR_PORT.DIR = X_MOTOR_PORT_DIR_gm;		// config interface pin directions
	Y_MOTOR_PORT.DIR = Y_MOTOR_PORT_DIR_gm;
	Z_MOTOR_PORT.DIR = Z_MOTOR_PORT_DIR_gm;
	A_MOTOR_PORT.DIR = A_MOTOR_PORT_DIR_gm;

	X_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;	// enable motors
	Y_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
	Z_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
	A_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;	// you might not want this one enabled

	/* setup timers and interrupts */

	X_TIMER.CTRLA = TC_CLK_DIV_8;				// default division ratio
	X_TIMER.CTRLB = TC_WGMODE;					// waveform generation mode
	X_TIMER.INTCTRLA = TC_OVFINTLVL;			// interrupt mode
	X_TIMER.PERL = 0xFF;						// period low
	X_TIMER.PERH = 0xFF;						// period high

	Y_TIMER.CTRLA = TC_CLK_DIV_8;				// default division ratio
	Y_TIMER.CTRLB = TC_WGMODE;					// waveform generation mode
	Y_TIMER.INTCTRLA = TC_OVFINTLVL;			// interrupt mode
	Y_TIMER.PERL = 0xFF;						// period low
	Y_TIMER.PERH = 0xFF;						// period high

	Z_TIMER.CTRLA = TC_CLK_DIV_8;				// default division ratio
	Z_TIMER.CTRLB = TC_WGMODE;					// waveform generation mode
	Z_TIMER.INTCTRLA = TC_OVFINTLVL;			// interrupt mode
	Z_TIMER.PERL = 0xFF;						// period low
	Z_TIMER.PERH = 0xFF;						// period high

	A_TIMER.CTRLA = TC_CLK_DIV_8;				// default division ratio
	A_TIMER.CTRLB = TC_WGMODE;					// waveform generation mode
	A_TIMER.INTCTRLA = TC_OVFINTLVL;			// interrupt mode
	A_TIMER.PERL = 0xFF;						// period low
	A_TIMER.PERH = 0xFF;						// period high

	// high level interrupts must be enabled in main()
}

/* X axis interrupt - service a tick from the X axis timer */

ISR(X_TIMER_vect)
{
	X_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	if (!--x_move_counter) {
		Z_TIMER.CTRLA = TC_CLK_OFF;				// stop the clock
	}
//	_delay_us(STEP_DELAY_TIME);				// only use if you need more time after the countdown
	X_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

/* Y axis interrupt - service a tick from the Y axis timer */

ISR(Y_TIMER_vect)
{
	Y_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	_delay_us(STEP_DELAY_TIME);
	Y_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

/* Z axis interrupt - service a tick from the Z axis timer */

ISR(Z_TIMER_vect)
{
	Z_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	_delay_us(STEP_DELAY_TIME);
	Z_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
}

/* A axis interrupt - service a tick from the A axis timer */

ISR(A_TIMER_vect)
{
	A_MOTOR_PORT.OUTSET	= STEP_BIT_bm;
	_delay_us(STEP_DELAY_TIME);
	A_MOTOR_PORT.OUTCLR	= STEP_BIT_bm;
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
	current_line = NULL;
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
//	if (steps_x < 0) { direction_bits |= (1<<X_DIRECTION_BIT); }
//	if (steps_y < 0) { direction_bits |= (1<<Y_DIRECTION_BIT); }
//	if (steps_z < 0) { direction_bits |= (1<<Z_DIRECTION_BIT); }
	line->direction_bits = direction_bits;

	line_buffer_head = next_buffer_head;		// Move buffer head

	// enable stepper interrupt
//	TIMSK1 |= (1<<OCIE1A);		// ********* taking out the interrupts
  
}

/****** taking out the interrupt routine ******
	This timer interrupt is executed at the rate set with config_step_timer. 
	It pops one instruction from the line_buffer, executes it. 
	Then it starts timer2 in order to reset the motor port after
	five microseconds.

ISR(TCC0_CCA_vect)
{
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt
  
  PORTD |= (1<<3);
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
  // Reset step pulse reset timer so that SIG_OVERFLOW2 can reset the signal after
  // exactly settings.pulse_microseconds microseconds.
  TCNT2 = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND)/8);

  busy = TRUE;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
  // We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
  // at exactly the right time even if we occasionally spend a lot of time inside this handler.
    
  // If there is no current line, attempt to pop one from the buffer
  if (current_line == NULL) {
    PORTD &= ~(1<<4);
    // Anything in the buffer?
    if (line_buffer_head != line_buffer_tail) {
      PORTD ^= (1<<5);
      // Retrieve a new line and get ready to step it
      current_line = &line_buffer[line_buffer_tail]; 
      config_step_timer(current_line->rate);
      counter_x = -(current_line->maximum_steps >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      iterations = current_line->maximum_steps;
    } else {
      // disable this interrupt until there is something to handle
    	TIMSK1 &= ~(1<<OCIE1A);
      PORTD |= (1<<4);          
    }    
  } 

  if (current_line != NULL) {
    out_bits = current_line->direction_bits;
    counter_x += current_line->steps_x;
    if (counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      counter_x -= current_line->maximum_steps;
    }
    counter_y += current_line->steps_y;
    if (counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      counter_y -= current_line->maximum_steps;
    }
    counter_z += current_line->steps_z;
    if (counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      counter_z -= current_line->maximum_steps;
    }
    // If current line is finished, reset pointer 
    iterations -= 1;
    if (iterations <= 0) {
      current_line = NULL;
      // move the line buffer tail to the next instruction
      line_buffer_tail = (line_buffer_tail + 1) % LINE_BUFFER_SIZE;      
    }
  } else {
    out_bits = 0;
  }
  out_bits ^= settings.invert_mask;
  busy=FALSE;
  PORTD &= ~(1<<3);  
}

// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
#ifdef TIMER2_OVF_vect
SIGNAL(TIMER2_OVF_vect)
#else
SIGNAL(SIG_OVERFLOW2)
#endif
{
  // reset stepping pins (leave the direction pins)
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
}


****** end of interrupt routines ********/






/* st_go_home() - perform the homing cycle */

void st_go_home()
{
  // Todo: Perform the homing cycle
}
