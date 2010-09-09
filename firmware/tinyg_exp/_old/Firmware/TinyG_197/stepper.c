/*
 * stepper.c - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * Some function names have been preserved from Grbl, but the code has been 
 * rewritten to take advantage of the xmega facilities and works differently 
 * than the Grbl (Reprap) Bresenham line approximation implementation.
 *
 * Coordinated motion (line drawing) is performed by dedicating a timer to each 
 * axis and stepping each motor at a computed rate (timer period value) for a 
 * specified number of pulses (counter value). Each timeout fires a high-priority 
 * interrupt which generates a step and decrements the counter by one. Timer 
 * counters are post-scaled in software to extend the range to 32 bits.
 *
 * Moves are dequeued from the move buffer (move_buffer.c) and loaded into the 
 * stepper controllers (ISRs). Any axis that is part of the move has its ACTIVE
 * bit set in ax.active. When the axis move is complete this bit is cleared. 
 * When all active bits are cleared st_execute_move() is called to load the next
 * move into the timers.
 *
 * But you need some way to start the timers if they are not already running,
 * so st_execute_move() is called from mv_queue_move_buffer() to start move
 * execution if the timers are not already running.  st_execute_move() therefore 
 * has a busy flag to prevent ISR and non-ISR calls from stepping on each other.
 */

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_init.h"				// defines CPU speed...
#include <util/delay.h>				//...used here for optional step pulse delay
#include "move_buffer.h"
#include "stepper.h"
#include "config.h"
#include "tinyg.h"

#ifdef __DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"
#endif

/*  
 *	Stepper axis structures
 */

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	uint32_t step_counter;			// counts steps down to 0 (end of line)
	uint16_t timer_period;			// value loaded into timers
	uint16_t postscale_value;		// timer post-scale value (reloads)
	uint16_t postscale_counter;		// timer post-scale counter
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity

	/* hardware device bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

struct Axes {						// All axes grouped in 1 struct + some extra stuff
	uint8_t exec_mutex;				// mutual exclusion flag for dequeuing moves
	uint8_t active_axes;			// bits are set if axis is active. 0 = robot is idle
	struct mvMove *p;				// pointer to dequeued move structure
	struct Axis a[4];				// 4 axis structures, X, Y, Z, A
};
static struct Axes ax;				// master axes structure

void _st_fake_move(void);
//void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint32_t us, uint8_t active);
void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint8_t active);

/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	ax.a[X_AXIS].step_counter = 0x00001000;
	ax.a[X_AXIS].timer->PER = 0x1000;					// step rate (period)
	ax.a[X_AXIS].timer->CTRLA = TC_CLK_ON;				// start clock

	ax.a[Y_AXIS].step_counter = 0x00000800;
	ax.a[Y_AXIS].timer->PER = 0x2000;
	ax.a[Y_AXIS].timer->CTRLA = TC_CLK_ON;

	ax.a[Z_AXIS].step_counter = 0x00000600;
	ax.a[Z_AXIS].timer->PER = 0x3000;
	ax.a[Z_AXIS].timer->CTRLA = TC_CLK_ON;

	ax.a[A_AXIS].step_counter = 0x00000400;
	ax.a[A_AXIS].timer->PER = 0x4000;
	ax.a[A_AXIS].timer->CTRLA = TC_CLK_ON;

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
	ax.active_axes = 0;								// clear all active bits
	ax.exec_mutex = FALSE;

	// initialize X axis
	ax.a[X_AXIS].polarity = cfg.a[X_AXIS].polarity;
														// motor control port
	ax.a[X_AXIS].port = &X_MOTOR_PORT;					// bind PORT to structure
	ax.a[X_AXIS].port->DIR = X_MOTOR_PORT_DIR_gm;		// set inputs and outputs
	ax.a[X_AXIS].port->OUT = 0x00;						// set port bits to zero initially
	ax.a[X_AXIS].port->OUT |= MICROSTEP_UNITS_bm;		// set microstep bits
	ax.a[X_AXIS].port->OUTSET = MOTOR_ENABLE_BIT_bm; 	// disable the motor

														// motor control timer
	ax.a[X_AXIS].timer = &X_TIMER;						// bind TIMER to structure
	ax.a[X_AXIS].timer->CTRLA = TC_CLK_OFF;				// turn motor off
	ax.a[X_AXIS].timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.a[X_AXIS].timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode

	// initialize Y axis
	ax.a[Y_AXIS].polarity = cfg.a[Y_AXIS].polarity;

	ax.a[Y_AXIS].port = &Y_MOTOR_PORT;
	ax.a[Y_AXIS].port->DIR = Y_MOTOR_PORT_DIR_gm;
	ax.a[Y_AXIS].port->OUT = 0x00;
	ax.a[Y_AXIS].port->OUT |= MICROSTEP_UNITS_bm;
	ax.a[Y_AXIS].port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.a[Y_AXIS].timer = &Y_TIMER;
	ax.a[Y_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[Y_AXIS].timer->CTRLB = TC_WGMODE;
	ax.a[Y_AXIS].timer->INTCTRLA = TC_OVFINTLVL;

	// initialize Z axis
	ax.a[Z_AXIS].polarity = cfg.a[Z_AXIS].polarity;

	ax.a[Z_AXIS].port = &Z_MOTOR_PORT;
	ax.a[Z_AXIS].port->DIR = Z_MOTOR_PORT_DIR_gm;
	ax.a[Z_AXIS].port->OUT = 0x00;
	ax.a[Z_AXIS].port->OUT |= MICROSTEP_UNITS_bm;
	ax.a[Z_AXIS].port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.a[Z_AXIS].timer = &Z_TIMER;
	ax.a[Z_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[Z_AXIS].timer->CTRLB = TC_WGMODE;
	ax.a[Z_AXIS].timer->INTCTRLA = TC_OVFINTLVL;

	// initialize A axis
	ax.a[A_AXIS].polarity = cfg.a[A_AXIS].polarity;

	ax.a[A_AXIS].port = &A_MOTOR_PORT;
	ax.a[A_AXIS].port->DIR = A_MOTOR_PORT_DIR_gm;
	ax.a[A_AXIS].port->OUT = 0x00;
	ax.a[A_AXIS].port->OUT |= MICROSTEP_UNITS_bm;
	ax.a[A_AXIS].port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.a[A_AXIS].timer = &A_TIMER;
	ax.a[A_AXIS].timer->CTRLA = TC_CLK_OFF;				
	ax.a[A_AXIS].timer->CTRLB = TC_WGMODE;
	ax.a[A_AXIS].timer->INTCTRLA = TC_OVFINTLVL;

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
	if (--ax.a[X_AXIS].postscale_counter != 0) {
		return;
	}
	X_MOTOR_PORT.OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.a[X_AXIS].step_counter == 0) {		// end-of-move processing
		X_TIMER.CTRLA = TC_CLK_OFF;				// stop the clock
		X_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		ax.active_axes &= ~X_ACTIVE_BIT_bm;		// clear the X active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_execute_move();					// ...run the next move
		}
	}
	ax.a[X_AXIS].postscale_counter = ax.a[X_AXIS].postscale_value;// reset post-scaler counter
	STEPPER_DELAY								// optional stepper pulse delay
	X_MOTOR_PORT.OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_vect)
{
	if (--ax.a[Y_AXIS].postscale_counter != 0) {
		return;
	}
	Y_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[Y_AXIS].step_counter == 0) {
		Y_TIMER.CTRLA = TC_CLK_OFF;
		Y_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Y_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[Y_AXIS].postscale_counter = ax.a[Y_AXIS].postscale_value;
	STEPPER_DELAY
	Y_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(Z_TIMER_vect)
{
	if (--ax.a[Z_AXIS].postscale_counter != 0) {
		return;
	}
	Z_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[Z_AXIS].step_counter == 0) {
		Z_TIMER.CTRLA = TC_CLK_OFF;	
		Z_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Z_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[Z_AXIS].postscale_counter = ax.a[Z_AXIS].postscale_value;
	STEPPER_DELAY
	Z_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(A_TIMER_vect)
{
	if (--ax.a[A_AXIS].postscale_counter != 0) {
		return;
	}
	A_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[A_AXIS].step_counter == 0) {
		A_TIMER.CTRLA = TC_CLK_OFF;
		A_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~A_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[A_AXIS].postscale_counter = ax.a[A_AXIS].postscale_value;
	STEPPER_DELAY
	A_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}


/*
 * st_execute_move() - Dequeue move and load into stepper motors (if possible)
 *
 *	This routine can be called from ISR or non-ISR levels - mediated by mutex
 *
 *	Mutex race condition - there is a brief race condition in the mutex test that 
 * 	should not actually cause any problems. If the routine were invoked by 
 *	mv_queue_move_buffer() (i.e. non-ISR invocation) an ISR call could occur during 
 *	the mutex test; which the ISR *could* find the routine is not occupied - even 
 *	though it was previously invoked by the non-ISR caller. The interrupt would 
 *	run, loading the next line (or not), then return control to the non-ISR 
 *	invocation. The non-ISR invocation would then find that the axes were active 
 *	(ax.active_axes test), and exit. Alternately, it might find that the axes 
 *	were not active, but exit on the buffer empty test - because this is the 
 *	reason they are not active - the ISR found nothing to load. So please don't 
 *	mess with the ordering of this code region.
 */

void st_execute_move()
{
	uint8_t i;

	// ******** don't re-order this code region - from here... ********
	if (ax.exec_mutex) { 	// prevents ISR from clobbering non-ISR invocation
		return;
	}
	ax.exec_mutex = TRUE;
	if (ax.active_axes != 0) {	// exit if any axis is still busy (any bit set)
		ax.exec_mutex = FALSE;	
		return;
	}
	if ((ax.p = mv_dequeue_move_buffer()) == NULL) {// NULL is empty buffer condition
		ax.exec_mutex = FALSE;
		return;
  	} 
	//********...to here. See Mutex race condition header note. ********

#ifdef __FAKE_STEPPERS	// bypasses the ISR load for fast debugging in simulation
	_st_fake_move();
	return;
#endif

	for (i = X_AXIS; i <= Z_AXIS; i++) {
		ax.a[i].timer->CTRLA = TC_CLK_OFF;		// turn clock off, to be sure
		if (ax.p->a[i].steps == 0) {			// skip axis if zero steps
			continue;
		}
		// set direction bit and compensate for polarity
		(ax.p->a[i].direction ^ ax.a[i].polarity) ?
		   (ax.a[i].port->OUTSET = DIRECTION_BIT_bm):	// CCW
		   (ax.a[i].port->OUTCLR = DIRECTION_BIT_bm);	// CW

		// load timers and other stepper ISR values
		ax.a[i].step_counter = ax.p->a[i].steps;
		ax.a[i].postscale_value = ax.p->a[i].postscale;
		ax.a[i].postscale_counter = ax.p->a[i].postscale;
		ax.a[i].timer_period = ax.p->a[i].period;		// not used for anything
		ax.a[i].timer->PER = ax.p->a[i].period;
		ax.a[i].port->OUTCLR = MOTOR_ENABLE_BIT_bm;		// enable motor
	}

	// enable all the axes at the same time (roughly). Better for motor sync.
	ax.active_axes = 0;
	if (ax.a[X_AXIS].step_counter) { 
		ax.a[X_AXIS].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= X_ACTIVE_BIT_bm;
	}
	if (ax.a[Y_AXIS].step_counter) {
		ax.a[Y_AXIS].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= Y_ACTIVE_BIT_bm;
	}
	if (ax.a[Z_AXIS].step_counter) {
		ax.a[Z_AXIS].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= Z_ACTIVE_BIT_bm;
	}

#ifdef __DEBUG
//	_st_print_exec_line(ax.a[X_AXIS].step_counter,
//						ax.a[Y_AXIS].step_counter,
//						ax.a[Z_AXIS].step_counter, ax.active_axes);
#endif
	ax.exec_mutex = FALSE;
}

/* 
 * _st_fake_move() - Debugging tool
 */

void _st_fake_move()
{
	ax.exec_mutex = FALSE;
	st_execute_move();		// recursively empty the move queue
	return;
}


/* 
 * st_kill() - STOP. NOW. UNCONDITIONALLY
 */


void st_set_polarity(uint8_t axis, uint8_t polarity)
{
	ax.a[axis].polarity = polarity;
}

void st_kill()
{
	cli();										// stop intewrrupts
	ax.a[X_AXIS].timer->CTRLA = TC_CLK_OFF;				// stop the clocks
	ax.a[Y_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[Z_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[A_AXIS].timer->CTRLA = TC_CLK_OFF;
	
	mv_flush();									// flush the move buffer
	ax.active_axes = 0;							// clear all the active bits
	sei();
}

/* 
 * st_terminate() - stop moves after the current move
 */

void st_terminate()
{
	cli();
	mv_flush();									// flush the move buffer
	sei();
}

#ifdef __DEBUG

/* 
 * _st_print_exec_line()
 */
/*
void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint8_t active)
{
	printf_P(PSTR("Exec X=%d Y=%d Z=%d Active=%d\n"), x, y, z, active);
}
*/
#endif
