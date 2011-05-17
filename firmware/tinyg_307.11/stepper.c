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
/* 
 *	Coordinated motion (line drawing) is performed using a classic 
 *	Bresenham DDA as per reprap and grbl. There are some differences:
 *
 *	- The DDA always runs at it's maximum rate (typ 50Khz) becuase there
 *	  are enough cycles to support this. Accel/decle is managed by 
 *	  passing stepwise linear accel/decel segments to the DDA as 
 *	  independent lines. The DDA is not slowed to accomplish accel/decel
 *
 *	- The state of the previous move (counter) is preserved as this 
 *	  supports smoother interpolatin between moves.
 *
 *	- Steps can be passed to the DDA in fractions. The DDA counters 
 *	  implement 28.4 fixed point math. 
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

int32_t counter_motor1, counter_motor2, counter_motor3, counter_motor4; 

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

	st.m[MOTOR_1].port = &MOTOR_1_PORT;			// bind PORTs to structs
	st.m[MOTOR_2].port = &MOTOR_2_PORT;
	st.m[MOTOR_3].port = &MOTOR_3_PORT;
	st.m[MOTOR_4].port = &MOTOR_4_PORT;

	for (i=0; i<MOTORS; i++) {
		// setup port. Do this first or st_set_microsteps() can fail
		st.m[i].port->DIR = MOTOR_PORT_DIR_gm;	// set inputs & outputs
		st.m[i].port->OUT = 0x00;				// zero port bits
		st.m[i].port->OUTSET = MOTOR_ENABLE_BIT_bm; // disable motor

		st_set_polarity(i, cfg.a[i].polarity);	// motor polarity
		st_set_microsteps(i, cfg.a[i].microsteps);
		// NOTE: limit switch ports and interrupts are setup in ls_init()
	}
	// setup DDA timer
	st.dda_timer = &DDA_TIMER;					// bind timer to struct
	st.dda_timer->CTRLA = TIMER_DISABLE;		// turn timer off
	st.dda_timer->CTRLB = TIMER_WGMODE;			// waveform mode
	st.dda_timer->INTCTRLA = TIMER_OVFINTLVL;	// interrupt mode
	st.dda_timer->PER = DDA_PERIOD;				// load but don't enable

	// setup DWELL timer
	st.dwell_timer = &DWELL_TIMER;				// bind timer to struct
	st.dwell_timer->CTRLA = TIMER_DISABLE;		// turn timer off
	st.dwell_timer->CTRLB = TIMER_WGMODE;		// waveform mode
	st.dwell_timer->INTCTRLA = TIMER_OVFINTLVL;	// interrupt mode
	st.dwell_timer->PER = DWELL_PERIOD;			// load but don't enable

	// setup software interrupt timer
	st.swi_timer = &SWI_TIMER;					// bind timer to struct
	st.swi_timer->CTRLA = TIMER_DISABLE;		// turn timer off
	st.swi_timer->CTRLB = TIMER_WGMODE;			// waveform mode
	st.swi_timer->INTCTRLA = TIMER_OVFINTLVL;	// interrupt mode
	st.swi_timer->PER = SWI_PERIOD;				// set period

	// setup motor mapping
	cfg.motor_map[MOTOR_1] = X;
	cfg.motor_map[MOTOR_2] = X;
	cfg.motor_map[MOTOR_3] = Y;
	cfg.motor_map[MOTOR_4] = Z;
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

	if (st.timer_ticks_left != 0) { // exit if it's still busy
		return;
	}
	if ((st.p = mq_dequeue_motor_buffer()) == NULL) {// NULL is empty buffer
		return;
  	} 

//	if (st.p->mq_type == MQ_STOP) {
//	if (st.p->mq_type == MQ_START) {
//	if (st.p->mq_type == MQ_DWELL) {

	for (i=0; i < MOTORS; i++) {
		if (st.p->a[i].steps == 0) {		// skip axis if zero steps
			continue;
		}
// commenting next line has the effect of retaining previous phase angle
//		st.m[i].counter = -(st.p->timer_ticks >> 1);
		st.m[i].steps = st.p->a[i].steps;
		st.m[i].port->OUTCLR = MOTOR_ENABLE_BIT_bm;	// enable motor
		if ((st.p->a[i].dir ^ st.m[i].polarity) == 0) {
		   st.m[i].port->OUTCLR = DIRECTION_BIT_bm;// CW motion
		} else {
		   st.m[i].port->OUTSET = DIRECTION_BIT_bm;// CCW motion
		}
	}
	st.timer_ticks = st.p->timer_ticks;
	st.timer_ticks_left = st.timer_ticks;
	st.dda_timer->PER = st.p->timer_period;		// load timer period
 	st.dda_timer->CTRLA = TIMER_ENABLE;
}

/*
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	Uses direct struct addresses and literal values for hardware devices -
 *	it's faster than using indexed timer and port accesses. I checked.
 *	Even when -0s or -03 is used.
 *
 *	The step bit pulse width is ~1 uSec, which is OK for the TI DRV8811's.
 *	If you need to stretch the pulse I recommend using a pulse OFF timer
 *	like grbl does so as not to spend any more time in the ISR, which would
 *	limit the upper range of the DDA frequency.
 */

ISR(DDA_TIMER_ISR_vect)
{
	counter_motor1 += st.m[MOTOR_1].steps;
	if (counter_motor1 > 0) {
		MOTOR_1_PORT.OUTSET = STEP_BIT_bm;		// turn step bit on.
 		counter_motor1 -= st.timer_ticks;
		MOTOR_1_PORT.OUTCLR = STEP_BIT_bm;		// turn off ~1 uSec later
	}
	counter_motor2 += st.m[MOTOR_2].steps;
	if (counter_motor2 > 0) {
		MOTOR_2_PORT.OUTSET = STEP_BIT_bm;
 		counter_motor2 -= st.timer_ticks;
		MOTOR_2_PORT.OUTCLR = STEP_BIT_bm;
	}
	counter_motor3 += st.m[MOTOR_3].steps;
	if (counter_motor3 > 0) {
		MOTOR_3_PORT.OUTSET = STEP_BIT_bm;
 		counter_motor3 -= st.timer_ticks;
		MOTOR_3_PORT.OUTCLR = STEP_BIT_bm;
	}
	counter_motor4 += st.m[MOTOR_4].steps;
	if (counter_motor4 > 0) {
		MOTOR_4_PORT.OUTSET = STEP_BIT_bm;
 		counter_motor4 -= st.timer_ticks;
		MOTOR_4_PORT.OUTCLR = STEP_BIT_bm;
	}
/*
	st.m[MOTOR_1].counter += st.m[MOTOR_1].steps;
	if (st.m[MOTOR_1].counter > 0) {
		MOTOR_1_PORT.OUTSET = STEP_BIT_bm;		// turn step bit on.
 		st.m[MOTOR_1].counter -= st.timer_ticks;
		MOTOR_1_PORT.OUTCLR = STEP_BIT_bm;		// turn off ~1 uSec later
	}
	st.m[MOTOR_2].counter += st.m[MOTOR_2].steps;
	if (st.m[MOTOR_2].counter > 0) {
		MOTOR_2_PORT.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_2].counter -= st.timer_ticks;
		MOTOR_2_PORT.OUTCLR = STEP_BIT_bm;
	}
	st.m[MOTOR_3].counter += st.m[MOTOR_3].steps;
	if (st.m[MOTOR_3].counter > 0) {
		MOTOR_3_PORT.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_3].counter -= st.timer_ticks;
		MOTOR_3_PORT.OUTCLR = STEP_BIT_bm;
	}
	st.m[MOTOR_4].counter += st.m[MOTOR_4].steps;
	if (st.m[MOTOR_4].counter > 0) {
		MOTOR_4_PORT.OUTSET = STEP_BIT_bm;
 		st.m[MOTOR_4].counter -= st.timer_ticks;
		MOTOR_4_PORT.OUTCLR = STEP_BIT_bm;
	}
*/
	if (--st.timer_ticks_left == 0) {
 		st.dda_timer->CTRLA = TIMER_DISABLE;// disable DDA timer
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
		MOTOR_1_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_2).power_mode == TRUE) {
		MOTOR_2_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_3).power_mode == TRUE) {
		MOTOR_3_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
	if (CFG(MOTOR_4).power_mode == TRUE) {
		MOTOR_4_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; 
	}
}

/*
 * ISR - DWELL timer interrupt routine - service ticks from DWELL timer
 */

ISR(DWELL_TIMER_ISR_vect)
{
	if (--st.timer_ticks_left == 0) {
 		st.dwell_timer->CTRLA = TIMER_DISABLE;	// disable DWELL timer
		_st_load_move();
	}
}

/*
 * ISR - Software Interrupt - to request a move load into steppers
 */

ISR(SWI_TIMER_ISR_vect)
{
 	st.swi_timer->CTRLA = TIMER_DISABLE;// disable SW interrupt timer
	_st_load_move();
}

/*
 * st_request_load() - Fire an interrupt to request a move be loaded
 */

void st_request_load()
{
	if (st.timer_ticks_left == 0) {		// bother interrupting
		st.swi_timer->PER = SWI_PERIOD;
		st.swi_timer->CTRLA = TIMER_ENABLE;
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
	if (st.timer_ticks_left == 0) {
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
		st.m[motor].port->OUT |= (0);
	} else if (microstep_mode == 2) {
		st.m[motor].port->OUT |= (MICROSTEP_BIT_0_bm);
	} else if (microstep_mode == 4) {
		st.m[motor].port->OUT |= (MICROSTEP_BIT_1_bm);
	} else if (microstep_mode == 8) {
		st.m[motor].port->OUT |= (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm);
	}
}

/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	return;
}

/*
 * st_print_stepper_state()
 */
/*
struct stSingleton {				// Stepper static values and axis parameters
	int32_t timer_ticks;			// total DDA or dwell ticks in this move
	int32_t timer_ticks_left;		// down-counter for above
	struct TC0_struct *dda_timer;	// timer/counter (type 0)
	struct TC0_struct *dwell_timer;	
	struct TC0_struct *swi_timer;
	struct stMotor m[MOTORS];		// 4 motor axis structures
	volatile struct mqMove *p;		// pointer to dequeued move structure
};

struct stMotor { 					// one per controlled motor
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
	int32_t steps;					// total steps in axis
	int32_t counter;				// DDA counter for axis
	struct PORT_struct *port;		// motor control port
};

*/

static char sts_sing[] PROGMEM = "stSingleton time_ticks:%d,  left:%d\n";
static char sts_timr[] PROGMEM = "  timer %s  enabled:%d  period:%d\n";
static char sts_motr[] PROGMEM = "  motor[%d] pol:%d  steps:%d  counter:%d\n";

void st_print_stepper_state()
{
	uint8_t i;

	fprintf_P(stderr, (PGM_P)sts_sing, st.timer_ticks, st.timer_ticks_left);

	fprintf_P(stderr, (PGM_P)sts_timr, "dda", st.dda_timer->CTRLA, st.dda_timer->PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "dwl", st.dwell_timer->CTRLA, st.dwell_timer->PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "swi", st.swi_timer->CTRLA, st.swi_timer->PER);

	for (i=0; i<MOTORS; i++) {
		fprintf_P(stderr, (PGM_P)sts_motr, i, 
			st.m[i].polarity,
			st.m[i].steps,
			st.m[i].counter);
	}
}
