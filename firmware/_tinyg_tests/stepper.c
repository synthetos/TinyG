/*
 * stepper.c - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * THIS FILE CUT DOWN FOR USE IN TEST. 
 * SEE LATEST TINYG BUILD FOR FULL FUNCTIONALITY
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
//#include "xmega_init.h"				// defines CPU speed...

#include <util/delay.h>				//...used here for optional step pulse delay

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

	ax.a[X_AXIS].port = &X_MOTOR_PORT;				// bind PORTs to structs
	ax.a[Y_AXIS].port = &Y_MOTOR_PORT;
	ax.a[Z_AXIS].port = &Z_MOTOR_PORT;
	ax.a[A_AXIS].port = &A_MOTOR_PORT;

	ax.a[X_AXIS].timer = &X_TIMER;					// bind TIMERs to structs
	ax.a[Y_AXIS].timer = &Y_TIMER;
	ax.a[Z_AXIS].timer = &Z_TIMER;
	ax.a[A_AXIS].timer = &A_TIMER;

	for (uint8_t i=0; i <= A_AXIS; i++) {
		ax.a[i].polarity = cfg.a[i].polarity;

		ax.a[i].port->DIR = MOTOR_PORT_DIR_gm;		// set inputs and outputs
		ax.a[i].port->OUT = 0x00;					// set port bits to zero
		ax.a[i].port->OUT |= MICROSTEP_UNITS_bm;	// set microstep bits
		ax.a[i].port->OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		// limit switch setup in ls_init()

		ax.a[i].timer->CTRLA = TC_CLK_OFF;			// turn motor off
		ax.a[i].timer->CTRLB = TC_WGMODE;			// waveform generation mode
		ax.a[i].timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode
	}
	// if you need to anything special for A_AXIS (e.g. spindle), do it here

	st_motor_test();							// run the startup motor test
}


/*
 * ISRs - Motor timer interrupt service routines - service a tick from the axis timer
 *
 *	Uses direct struct addresses and literal values for hardware devices because it's 
 *  faster than using the timer and port pointers in the axis structs
 */

ISR(X_TIMER_ISR_vect)
{
	if (--ax.a[X_AXIS].postscale_counter != 0) {// get out fast, if you need to
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

ISR(Y_TIMER_ISR_vect)
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

ISR(Z_TIMER_ISR_vect)
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

ISR(A_TIMER_ISR_vect)
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
	ax.a[X_AXIS].timer->CTRLA = TC_CLK_OFF;		// stop the clocks
	ax.a[Y_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[Z_AXIS].timer->CTRLA = TC_CLK_OFF;
	ax.a[A_AXIS].timer->CTRLA = TC_CLK_OFF;
	
//	mv_flush();									// flush the move buffer
	ax.active_axes = 0;							// clear all the active bits
	sei();
}

/* 
 * st_terminate() - stop moves after the current move
 */

void st_terminate()
{
	cli();
//	mv_flush();									// flush the move buffer
	sei();
}

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

