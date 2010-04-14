/*
  stepper_grbl.c - GRBL stepper motor ISR
  Part of Grbl
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

#define LINE_BUFFER_SIZE 40		// number of lines buffered in steppers.c

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


/****** interrupt routine ******
	This timer interrupt is executed at the rate set with config_step_timer. 
	It pops one instruction from the line_buffer, executes it. 
	Then it starts timer2 in order to reset the motor port after
	five microseconds.
*/

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

