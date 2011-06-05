/*
 * stepper.c - stepper motor controls
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>				// needed for memset in st_init()

#include "xio.h"				// supports TRAP and debug statements
#include "tinyg.h"
#include "system.h"
#include "config.h"
#include "stepper.h"
#include "motor_queue.h"

static void _st_end_move(void);
static void _st_load_move(void);

/*
 * Stepper structures
 *
 *	Care has been taken that no volatiles are required in these structures.
 *	This allows the compiler to optimizer the stepper inner-loops better.
 */

struct stMotor { 					// one per controlled motor
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
	int32_t steps;					// total steps in axis
	int32_t counter;				// DDA counter for axis
};

struct stSingleton {				// Stepper static values and axis parameters
	int32_t timer_ticks_downcount;	// tick down-counter (unscaled)
	int32_t timer_ticks_X_substeps;	// ticks multiplied by scaling factor
	struct stMotor m[MOTORS];		// 4 motor structures
	volatile struct mqMove *p;		// pointer to dequeued move structure
};
struct stSingleton st;

/* 
 * st_init() - initialize and start the stepper motor subsystem 
 *
 *	Notes: 
 *	  - High level interrupts must be enabled in main()
 *	  - ls_init() in limit_switches.c is dependent on st_init() as they 
 *		use the same ports. encoder.c also uses the same ports.
 */

void st_init()
{
	uint8_t i;

	memset(&st, 0, sizeof(st));	// clear all values, pointers and status

	device.port[MOTOR_1] = &DEVICE_PORT_MOTOR_1;// bind PORTs to struct
	device.port[MOTOR_2] = &DEVICE_PORT_MOTOR_2;
	device.port[MOTOR_3] = &DEVICE_PORT_MOTOR_3;
	device.port[MOTOR_4] = &DEVICE_PORT_MOTOR_4;

	for (i=0; i<MOTORS; i++) {
		// setup port. Do this first or st_set_microsteps() can fail
		device.port[i]->DIR = MOTOR_PORT_DIR_gm;	// set inputs & outputs
		device.port[i]->OUT = 0x00;				// zero port bits
		device.port[i]->OUTSET = MOTOR_ENABLE_BIT_bm; // disable motor

//		st_set_polarity(i, cfg.a[i].polarity);	// motor polarity
		st_set_microsteps(i, cfg.a[i].microsteps);
		// NOTE: limit switch ports and interrupts are setup in ls_init()
	}
	// setup DDA timer
	DEVICE_TIMER_DDA.CTRLA = TIMER_DISABLE;		// turn timer off
	DEVICE_TIMER_DDA.CTRLB = TIMER_WGMODE;		// waveform mode
	DEVICE_TIMER_DDA.INTCTRLA = TIMER_OVFINTLVL;// interrupt mode

	// setup DWELL timer
	DEVICE_TIMER_DWELL.CTRLA = TIMER_DISABLE;	// turn timer off
	DEVICE_TIMER_DWELL.CTRLB = TIMER_WGMODE;	// waveform mode
	DEVICE_TIMER_DWELL.INTCTRLA = TIMER_OVFINTLVL;// interrupt mode

	// setup software interrupt timer
	DEVICE_TIMER_SWI.CTRLA = TIMER_DISABLE;		// turn timer off
	DEVICE_TIMER_SWI.CTRLB = TIMER_WGMODE;		// waveform mode
	DEVICE_TIMER_SWI.INTCTRLA = TIMER_OVFINTLVL;// interrupt mode
	DEVICE_TIMER_SWI.PER = SWI_PERIOD;			// set period

	// setup motor mapping
	cfg.motor_map[MOTOR_1] = X;
	cfg.motor_map[MOTOR_2] = Y;
	cfg.motor_map[MOTOR_3] = Z;
	cfg.motor_map[MOTOR_4] = A;
}

/*
 * _st_load_move() - Dequeue move and load into stepper struct
 *
 *	This routine can only be called be called from an ISR at the same or 
 *	higher level as the DDA or dwell ISR. A software interrupt has been 
 *	provided to allow a non-ISR to request a load (see st_request_load())
 */

void _st_load_move()
{
	uint8_t i;

#ifdef __SIMULATION_MODE	// bypasses the load for fast simulations
	return;					// ...of the upper layers
#endif

	if (st.timer_ticks_downcount != 0) { // exit if it's still busy
		return;
	}
	if ((st.p = mq_dequeue_motor_buffer()) == NULL) {// NULL is empty buffer
		return;
  	} 

//	if (st.p->mq_type == MQ_STOP) {
//	if (st.p->mq_type == MQ_START) {

	if (st.p->mq_type == MQ_DWELL) {
		st.timer_ticks_downcount = st.p->timer_ticks;
		DEVICE_TIMER_DWELL.PER = st.p->timer_period;//load dwell timer period
 		DEVICE_TIMER_DWELL.CTRLA = TIMER_ENABLE;	// enable the dwell timer
		return;
	}

	st.timer_ticks_downcount = st.p->timer_ticks;
	st.timer_ticks_X_substeps = st.p->timer_ticks_X_substeps;
	DEVICE_TIMER_DDA.PER = st.p->timer_period;
 
	for (i=0; i < MOTORS; i++) {
		if (st.p->a[i].steps == 0) {			// skip axis if zero steps
			continue;
		}
		if (st.p->counter_reset_flag == TRUE) {			// pulse phasing
			st.m[i].counter = -(st.timer_ticks_downcount);
		}
		st.m[i].steps = st.p->a[i].steps;
		device.port[i]->OUTCLR = MOTOR_ENABLE_BIT_bm;	// enable motor
		if (st.p->a[i].dir == 0) {						// set direction
			device.port[i]->OUTCLR = DIRECTION_BIT_bm;	// CW motion
		} else {
			device.port[i]->OUTSET = DIRECTION_BIT_bm;	// CCW motion
		}
	}
	DEVICE_TIMER_DDA.CTRLA = TIMER_ENABLE;
}

/*
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	The step bit pulse width is ~1 uSec, which is OK for the TI DRV8811's.
 *	If you need to stretch the pulse I recommend moving the port OUTCLRs
 *	to the end of the routine. If you need more time than that use a 
 *	pulse OFF timer like grbl does so as not to spend any more time in 
 *	the ISR, which would limit the upper range of the DDA frequency.
 *
 *	Uses direct struct addresses and literal values for hardware devices -
 *	it's faster than using indexed timer and port accesses. I checked.
 *	Even when -0s or -03 is used.
 */

ISR(DEVICE_TIMER_DDA_ISR_vect)
{
	if ((st.m[MOTOR_1].counter += st.m[MOTOR_1].steps) > 0) {
		DEVICE_PORT_MOTOR_1.OUTSET = STEP_BIT_bm;	// turn step bit on
 		st.m[MOTOR_1].counter -= st.timer_ticks_X_substeps;
		DEVICE_PORT_MOTOR_1.OUTCLR = STEP_BIT_bm;	// turn step bit off in ~1 uSec
	}
	if ((st.m[MOTOR_2].counter += st.m[MOTOR_2].steps) > 0) {
		DEVICE_PORT_MOTOR_2.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_2].counter -= st.timer_ticks_X_substeps;
		DEVICE_PORT_MOTOR_2.OUTCLR = STEP_BIT_bm;
	}
	if ((st.m[MOTOR_3].counter += st.m[MOTOR_3].steps) > 0) {
		DEVICE_PORT_MOTOR_3.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_3].counter -= st.timer_ticks_X_substeps;
		DEVICE_PORT_MOTOR_3.OUTCLR = STEP_BIT_bm;
	}
	if ((st.m[MOTOR_4].counter += st.m[MOTOR_4].steps) > 0) {
		DEVICE_PORT_MOTOR_4.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_4].counter -= st.timer_ticks_X_substeps;
		DEVICE_PORT_MOTOR_4.OUTCLR = STEP_BIT_bm;
	}
	if (--st.timer_ticks_downcount == 0) {
 		DEVICE_TIMER_DDA.CTRLA = TIMER_DISABLE;// disable DDA timer
		_st_end_move();						// end the current move
		_st_load_move();					// load the next move
	}
}

/*
 * _st_end_move() - end current stepper move - called from ISR
 */

static void _st_end_move()
{
	// power-down motors if this feature is enabled
	if (CFG(MOTOR_1).power_mode == TRUE) {
		DEVICE_PORT_MOTOR_1.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_2).power_mode == TRUE) {
		DEVICE_PORT_MOTOR_2.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_3).power_mode == TRUE) {
		DEVICE_PORT_MOTOR_3.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_4).power_mode == TRUE) {
		DEVICE_PORT_MOTOR_4.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
}

/*
 * ISR - DWELL timer interrupt routine - service ticks from DWELL timer
 */

ISR(DEVICE_TIMER_DWELL_ISR_vect)
{
	if (--st.timer_ticks_downcount == 0) {
 		DEVICE_TIMER_DWELL.CTRLA = TIMER_DISABLE;// disable DWELL timer
		_st_load_move();
	}
}

/*
 * ISR - Software Interrupt - to request a move load into steppers
 */

ISR(DEVICE_TIMER_SWI_ISR_vect)
{
 	DEVICE_TIMER_SWI.CTRLA = TIMER_DISABLE;// disable SW interrupt timer
	_st_load_move();
}

/*
 * st_request_load() - Fire an interrupt to request a move be loaded
 */

void st_request_load()
{
	if (st.timer_ticks_downcount == 0) {		// bother interrupting
		DEVICE_TIMER_SWI.PER = SWI_PERIOD;
		DEVICE_TIMER_SWI.CTRLA = TIMER_ENABLE;
	} 	// else don't bother to interrupt. You'll just trigger an 
		// interrupt and find out the load routine is not ready for you
}

/*
 * st_start() - start steppers
 * st_stop() - stop steppers
 * st_end() - STOP. NOW. UNCONDITIONALLY
 *
 *	These routines must be safe to call from ISRs
 *	Mind the volatiles.
 */
void st_stop()
{
}

void st_start()
{
}

void st_end()
{
	st_init();
}

/*
 * st_isbusy() - return TRUE if motors are running
 */
inline uint8_t st_isbusy()
{
	if (st.timer_ticks_downcount == 0) {
		return (FALSE);
	} 
	return (TRUE);
}

/* 
 * st_set_polarity() - setter needed by the config system
 */

void st_set_polarity(const uint8_t motor, const uint8_t polarity)
{
	st.m[motor].polarity = polarity;
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
		device.port[motor]->OUT |= (0);
	} else if (microstep_mode == 2) {
		device.port[motor]->OUT |= (MICROSTEP_BIT_0_bm);
	} else if (microstep_mode == 4) {
		device.port[motor]->OUT |= (MICROSTEP_BIT_1_bm);
	} else if (microstep_mode == 8) {
		device.port[motor]->OUT |= (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm);
	}
}

/*
 * st_print_stepper_state()
 */

static char sts_sing[] PROGMEM = "stSingleton timer_ticks (remaining):%d\n";
static char sts_timr[] PROGMEM = "  timer %s  enabled:%d  period:%d\n";
static char sts_motr[] PROGMEM = "  motor[%d] pol:%d  steps:%d  counter:%d\n";

void st_print_stepper_state()
{
	uint8_t i;

	fprintf_P(stderr, (PGM_P)sts_sing, st.timer_ticks_downcount);

	fprintf_P(stderr, (PGM_P)sts_timr, "dda", DEVICE_TIMER_DDA.CTRLA, DEVICE_TIMER_DDA.PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "dwl", DEVICE_TIMER_DWELL.CTRLA, DEVICE_TIMER_DWELL.PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "swi", DEVICE_TIMER_SWI.CTRLA, DEVICE_TIMER_SWI.PER);

	for (i=0; i<MOTORS; i++) {
		fprintf_P(stderr, (PGM_P)sts_motr, i, 
			st.m[i].polarity,
			st.m[i].steps,
			st.m[i].counter);
	}
}
