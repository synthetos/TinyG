/*
 * stepper.c - stepper motor interface
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
 *
 * -----
 * Some function names have been preserved from Grbl, but the code is different
 * to take advantage of the xmega facilities and does not use a digital 
 * differential analyser (DDA, Bresenham line approximation).
 *
 * Coordinated motion (line drawing) is performed by dedicating a timer to each 
 * axis and stepping each motor at a computed rate (timer period value) for a 
 * specified number of pulses (counter value). Each timeout fires a high-priority 
 * interrupt which generates a step and decrements the counter by one. Timer 
 * counters are post-scaled in software to extend the HW timer range to 32 bits.
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

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "move_buffer.h"
#include "hardware.h"
#include <util/delay.h>				//...used here for optional step pulse delay

#ifdef __DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"
#endif


/*
 * Local stepper functions
 */

void _st_fake_move(void);
void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint8_t active);

/* 
 * st_init() - initialize and start the stepper motor subsystem 
 *
 *	State at completion of initialization is:
 *	- each axis has a structure with an initialized port and a timer bound to it
 *	- ports: input and output directions set
 *	- each axis is enabled 
 *
 *	Note: high level interrupts must be enabled in main()
 *	Note: limit switches and other use of the ports may extend this init.
 */

void st_init()
{
	ax.active_axes = 0;								// clear all active bits
	ax.exec_mutex = FALSE;

	ax.a[X].port = &X_MOTOR_PORT;				// bind PORTs to structs
	ax.a[Y].port = &Y_MOTOR_PORT;
	ax.a[Z].port = &Z_MOTOR_PORT;
	ax.a[A].port = &A_MOTOR_PORT;

	ax.a[X].timer = &X_TIMER;					// bind TIMERs to structs
	ax.a[Y].timer = &Y_TIMER;
	ax.a[Z].timer = &Z_TIMER;
	ax.a[A].timer = &A_TIMER;

	for (uint8_t i=X; i<=A; i++) {
		ax.a[i].polarity = cfg.a[i].polarity;

		ax.a[i].port->DIR = MOTOR_PORT_DIR_gm;		// set inputs and outputs
		ax.a[i].port->OUT = 0x00;					// set port bits to zero
		ax.a[i].port->OUT |= MICROSTEP_BITS_bm;		// set microstep bits
		ax.a[i].port->OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		// limit switch setup in ls_init()

		ax.a[i].timer->CTRLA = TC_CLK_OFF;			// turn motor off
		ax.a[i].timer->CTRLB = TC_WGMODE;			// waveform generation mode
		ax.a[i].timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	}
	// if you need to anything special for A_AXIS (e.g. spindle), do it here

//	st_motor_test();							// run the startup motor test
}

/*
 * ISRs - Motor timer interrupt service routines - service a tick from the axis timer
 *
 *	Uses direct struct addresses and literal values for hardware devices because it's 
 *  faster than using the timer and port pointers in the axis structs
 */

ISR(X_TIMER_ISR_vect)
{
	if (--ax.a[X].postscale_counter != 0) {		// get out fast, if you need to
		return;
	}
	if (!(ax.a[X].flags && DWELL_FLAG_bm)) {	// issue a pulse if not a dwell
		X_MOTOR_PORT.OUTSET = STEP_BIT_bm;		// turn X step bit on
	}
	if (--ax.a[X].step_counter == 0) {			// end-of-move processing
		X_TIMER.CTRLA = TC_CLK_OFF;				// stop the clock
		X_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		ax.active_axes &= ~X_ACTIVE_BIT_bm;		// clear the X active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_execute_move();					// ...run the next move
		}
	}
	ax.a[X].postscale_counter = ax.a[X].postscale_value;// reset post-scaler counter
	STEPPER_DELAY								// optional stepper pulse delay
	X_MOTOR_PORT.OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_ISR_vect)
{
	if (--ax.a[Y].postscale_counter != 0) {
		return;
	}
	Y_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[Y].step_counter == 0) {
		Y_TIMER.CTRLA = TC_CLK_OFF;
		Y_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Y_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[Y].postscale_counter = ax.a[Y].postscale_value;
	STEPPER_DELAY
	Y_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(Z_TIMER_ISR_vect)
{
	if (--ax.a[Z].postscale_counter != 0) {
		return;
	}
	Z_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[Z].step_counter == 0) {
		Z_TIMER.CTRLA = TC_CLK_OFF;	
		Z_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Z_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[Z].postscale_counter = ax.a[Z].postscale_value;
	STEPPER_DELAY
	Z_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(A_TIMER_ISR_vect)
{
	if (--ax.a[A].postscale_counter != 0) {
		return;
	}
	A_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[A].step_counter == 0) {
		A_TIMER.CTRLA = TC_CLK_OFF;
		A_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~A_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[A].postscale_counter = ax.a[A].postscale_value;
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

	for (i=X; i<=Z; i++) {
		ax.a[i].timer->CTRLA = TC_CLK_OFF;		// turn clock off, to be sure
		if (ax.p->a[i].steps == 0) {			// skip axis if zero steps
			continue;
		}

		ax.a[i].flags = ax.p->a[i].flags; 		// import flags from queued move

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
	if (ax.a[X].step_counter) { 
		ax.a[X].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= X_ACTIVE_BIT_bm;
	}
	if (ax.a[Y].step_counter) {
		ax.a[Y].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= Y_ACTIVE_BIT_bm;
	}
	if (ax.a[Z].step_counter) {
		ax.a[Z].timer->CTRLA = TC_CLK_ON;
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
 * st_set_polarity() - setter needed by the config system
 */

void st_set_polarity(uint8_t axis, uint8_t polarity)
{
	ax.a[axis].polarity = polarity;
}

/* 
 * st_kill() - STOP. NOW. UNCONDITIONALLY
 */

void st_stop_steppers()
{
	cli();										// stop interrupts
	for (uint8_t i=X; i<=A; i++) {
		ax.a[i].timer->CTRLA = TC_CLK_OFF;		// stop the clocks
	}	
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

/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	ax.a[X].step_counter = 0x00001000;
	ax.a[X].timer->PER = 0x1000;			// step rate (period)
	ax.a[X].timer->CTRLA = TC_CLK_ON;		// start clock

	ax.a[Y].step_counter = 0x00000800;
	ax.a[Y].timer->PER = 0x2000;
	ax.a[Y].timer->CTRLA = TC_CLK_ON;

	ax.a[Z].step_counter = 0x00000600;
	ax.a[Z].timer->PER = 0x3000;
	ax.a[Z].timer->CTRLA = TC_CLK_ON;

	ax.a[A].step_counter = 0x00000400;
	ax.a[A].timer->PER = 0x4000;
	ax.a[A].timer->CTRLA = TC_CLK_ON;

	ax.active_axes |= (X_ACTIVE_BIT_bm | Y_ACTIVE_BIT_bm | Z_ACTIVE_BIT_bm | A_ACTIVE_BIT_bm);
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
