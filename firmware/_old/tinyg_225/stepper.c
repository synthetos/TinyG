/*
 * stepper.c - stepper motor interface
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
 *
 * -----
 * 	Some function names have been preserved from Grbl, but most of the 
 *	code is quite different. This is (1) to take advantage of the xmega 
 *	and (2) because the code does not use a digital differential analyser
 *  (DDA, Bresenham line approximation).
 *
 *	Coordinated motion (line drawing) is performed by dedicating a timer 
 *	to each axis and stepping each motor at a computed rate (timer period) 
 *	for a specified number of pulses (step counter). Each timeout fires a 
 *	high-priority interrupt. The ISR will decrement a post-scaler; and may 
 *	or may not generate a step and decrement the step counter. Timer counters
 *	are post-scaled in software to extend the HW timer range to 32 bits.
 *
 *	Moves are dequeued from the move buffer (move_buffer.c) and loaded into 
 *	the stepper controllers (ISRs). Any axis that is part of the move has 
 *	its ACTIVE bit set in ax.active. When the axis move is complete this bit 
 *	is cleared. When all active bits are cleared st_execute_move() is called 
 *	to load the next move into the timers.
 *
 *	But you need some way to start the timers if they are not already 
 *	running, so st_execute_move() is called from mv_queue_move_buffer() to 
 *	start move execution if the timers are not already running.  
 *	st_execute_move() therefore has a busy flag to prevent ISR and non-ISR 
 *	calls from stepping on each other.
 */

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "tinyg.h"
#include "system.h"
#include "config.h"
#include "stepper.h"
#include "motor_queue.h"
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
	ax.mutex = FALSE;
	ax.stopped = FALSE;

	ax.a[MOTOR_1].port = &MOTOR_1_PORT;				// bind PORTs to structs
	ax.a[MOTOR_2].port = &MOTOR_2_PORT;
	ax.a[MOTOR_3].port = &MOTOR_3_PORT;
	ax.a[MOTOR_4].port = &MOTOR_4_PORT;

	ax.a[MOTOR_1].timer = &MOTOR_1_TIMER;			// bind TIMERs to structs
	ax.a[MOTOR_2].timer = &MOTOR_2_TIMER;
	ax.a[MOTOR_3].timer = &MOTOR_3_TIMER;
	ax.a[MOTOR_4].timer = &MOTOR_4_TIMER;

	for (uint8_t i = MOTOR_1; i <= MOTOR_4; i++) {
		// setup port. Do this first or st_set_microsteps() can fail
		ax.a[i].port->DIR = MOTOR_PORT_DIR_gm;		// set inputs and outputs
		ax.a[i].port->OUT = 0x00;					// set port bits to zero
		ax.a[i].port->OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor

		st_set_polarity(i, cfg.a[i].polarity);		// motor polarity
		st_set_microsteps(i, cfg.a[i].microstep_mode);
		// NOTE: limit switch port bits and interrupts are setup in ls_init()

		ax.a[i].timer->CTRLA = TC_CLK_OFF;			// turn motor off
		ax.a[i].timer->CTRLB = TC_WGMODE;			// waveform generation mode
		ax.a[i].timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	}
	// if you need to anything special for AXIS modes (e.g. spindle), do it here
}

/*
 * st_reset() - reset steppers but not entire init
 * st_start() - start steppers
 * st_stop() - stop steppers
 * st_end() - STOP. NOW. UNCONDITIONALLY
 *
 *	These routines must be safe to call from ISRs
 *	Mind the volatiles.
 */

void st_reset()
{
	for (uint8_t i = MOTOR_1; i <= MOTOR_4; i++) {
		ax.a[i].timer->CTRLA = TC_CLK_OFF;		// stop the clocks
	}
	ax.active_axes = 0;							// clear all active bits
	ax.mutex = FALSE;
	ax.stopped = FALSE;
}

void st_stop()
{
	ax.stopped = TRUE;
}

void st_start()
{
	ax.stopped = FALSE;
}

void st_end()
{
	st_init();
//	st_reset();						// reset the motors
	mq_flush_motor_buffer();
}


/*
 * st_isbusy() - return TRUE if motors are running
 */
inline uint8_t st_isbusy()
{
	if (ax.active_axes) {
		return (TRUE);
	} 
	return (FALSE);
}

/* 
 * st_set_polarity() - setter needed by the config system
 */

void st_set_polarity(const uint8_t motor, const uint8_t polarity)
{
	ax.a[motor].polarity = polarity;
}

/* 
 * st_set_microsteps() - set microsteps in hardware
 *
 *	For now the microstep_mode is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */

void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode)
{
	if (microstep_mode == 1) {
		ax.a[motor].port->OUT |= (0);
	} else if (microstep_mode == 2) {
		ax.a[motor].port->OUT |= (MICROSTEP_BIT_0_bm);
	} else if (microstep_mode == 4) {
		ax.a[motor].port->OUT |= (MICROSTEP_BIT_1_bm);
	} else if (microstep_mode == 8) {
		ax.a[motor].port->OUT |= (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm);
	}
}

/*
 * ISRs - Motor timer interrupt routines - service ticks from the axis timers
 *
 *	Uses direct struct addresses and literal values for hardware devices -
 *	it's faster than using the timer and port pointers in the axis structs
 * 
 * Note that the Z axis is also used to time out dwells
 */

ISR(MOTOR_1_TIMER_ISR_vect)
{
	if (ax.stopped) {
		return;
	}
	if (--ax.a[MOTOR_1].postscale_counter) { // get out fast, if you need to
		return;
	}
	MOTOR_1_PORT.OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.a[MOTOR_1].step_counter == 0) {	// end-of-move processing
		MOTOR_1_TIMER.CTRLA = TC_CLK_OFF;		// stop the clock
		MOTOR_1_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		ax.active_axes &= ~MOTOR_1_ACTIVE_BIT_bm;// clear motor 1 active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_execute_move();					// ...run the next move
		}
	}
	// reset post-scaler counter
	ax.a[MOTOR_1].postscale_counter = ax.a[MOTOR_1].postscale_value;
	STEPPER_DELAY						// optional stepper pulse delay
	MOTOR_1_PORT.OUTCLR = STEP_BIT_bm;	// turn motor 1 step bit off
}

ISR(MOTOR_2_TIMER_ISR_vect)
{
	if (ax.stopped) {
		return;
	}
	if (--ax.a[MOTOR_2].postscale_counter != 0) {
		return;
	}
	MOTOR_2_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[MOTOR_2].step_counter == 0) {
		MOTOR_2_TIMER.CTRLA = TC_CLK_OFF;
		MOTOR_2_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~MOTOR_2_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[MOTOR_2].postscale_counter = ax.a[MOTOR_2].postscale_value;
	STEPPER_DELAY
	MOTOR_2_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(MOTOR_3_TIMER_ISR_vect)		// this one also counts out dwells
{
	if (ax.stopped) {
		return;
	}
	if (--ax.a[MOTOR_3].postscale_counter != 0) {
		return;
	}
	if (ax.line_mode) {							// issue a pulse if not a dwell
		MOTOR_3_PORT.OUTSET = STEP_BIT_bm;		// turn Z step bit on
	}
	if (--ax.a[MOTOR_3].step_counter == 0) {
		MOTOR_3_TIMER.CTRLA = TC_CLK_OFF;	
		MOTOR_3_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~MOTOR_3_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[MOTOR_3].postscale_counter = ax.a[MOTOR_3].postscale_value;
	STEPPER_DELAY
	MOTOR_3_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(MOTOR_4_TIMER_ISR_vect)
{
	if (ax.stopped) {
		return;
	}
	if (--ax.a[MOTOR_4].postscale_counter != 0) {
		return;
	}
	MOTOR_4_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a[MOTOR_4].step_counter == 0) {
		MOTOR_4_TIMER.CTRLA = TC_CLK_OFF;
		MOTOR_4_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~MOTOR_4_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a[MOTOR_4].postscale_counter = ax.a[MOTOR_4].postscale_value;
	STEPPER_DELAY
	MOTOR_4_PORT.OUTCLR = STEP_BIT_bm;
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

uint8_t st_execute_move()
{
	uint8_t i;

	// ******** don't re-order this code region - from here... ********
	if (ax.mutex) { 		// prevents ISR from clobbering non-ISR invocation
		return (0);
	}
	ax.mutex = TRUE;
	if (ax.active_axes != 0) { // exit if any axis is still busy (any bit set)
		ax.mutex = FALSE;	
		return (0);
	}
	if ((ax.p = mq_dequeue_motor_buffer()) == NULL) {// NULL is empty buffer
		ax.mutex = FALSE;
		return (0);
  	} 
	//********...to here. See Mutex race condition header note. ********

#ifdef __SIMULATION_MODE	// bypasses the ISR load for faster simulations
	ax.mutex = FALSE;
	return (0);
#endif

	if (ax.p->mq_type == MQ_STOP) {
		ax.stopped = TRUE;
		ax.mutex = FALSE;
		return (0);
	}
	if (ax.p->mq_type == MQ_START) {
		ax.stopped = FALSE;
		ax.mutex = FALSE;
		return (0);
	}
	if (ax.p->mq_type == MQ_DWELL) {
		ax.line_mode = FALSE;
	} else {
		ax.line_mode = TRUE;
	}

	for (i = 0; i < MOTORS; i++) {
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
	if (ax.a[MOTOR_1].step_counter) { 
		ax.a[MOTOR_1].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= MOTOR_1_ACTIVE_BIT_bm;
	}
	if (ax.a[MOTOR_2].step_counter) {
		ax.a[MOTOR_2].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= MOTOR_2_ACTIVE_BIT_bm;
	}
	if (ax.a[MOTOR_3].step_counter) {
		ax.a[MOTOR_3].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= MOTOR_3_ACTIVE_BIT_bm;
	}
	if (ax.a[MOTOR_4].step_counter) {
		ax.a[MOTOR_4].timer->CTRLA = TC_CLK_ON;
		ax.active_axes |= MOTOR_4_ACTIVE_BIT_bm;
	}
	ax.mutex = FALSE;
	return (0);
}


/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	ax.a[MOTOR_1].step_counter = 0x00001000;
	ax.a[MOTOR_1].timer->PER = 0x1000;			// step rate (period)
	ax.a[MOTOR_1].timer->CTRLA = TC_CLK_ON;		// start clock

	ax.a[MOTOR_2].step_counter = 0x00000800;
	ax.a[MOTOR_2].timer->PER = 0x2000;
	ax.a[MOTOR_2].timer->CTRLA = TC_CLK_ON;

	ax.a[MOTOR_3].step_counter = 0x00000600;
	ax.a[MOTOR_3].timer->PER = 0x3000;
	ax.a[MOTOR_3].timer->CTRLA = TC_CLK_ON;

	ax.a[MOTOR_4].step_counter = 0x00000400;
	ax.a[MOTOR_4].timer->PER = 0x4000;
	ax.a[MOTOR_4].timer->CTRLA = TC_CLK_ON;

	ax.active_axes |= (MOTOR_1_ACTIVE_BIT_bm | 
					   MOTOR_2_ACTIVE_BIT_bm | 
					   MOTOR_3_ACTIVE_BIT_bm | 
					   MOTOR_4_ACTIVE_BIT_bm);
}
