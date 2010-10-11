/*
 * stepper.h - stepper motor interface
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
 */

#ifndef stepper_h
#define stepper_h

/*
 * Stepper configs and constants
 */

/* The stepper ISRs generate step pulses approximately 1.5 microseconds duration
 * The TI DRV8811's used on the TinyG board are fine with this pulse width. 
 * Some outboarded drivers might not be. If the drivers require a longer pulse 
 * uncomment __STEPPER_DELAY and adjust the microseconds to your requirements.
 * The delay is in addition to the 1.5 uSec burned in the ISR.
 */
//#define __STEPPER_DELAY
#ifdef __STEPPER_DELAY
#define STEP_PULSE_ADDITIONAL_MICROSECONDS	2
#define STEPPER_DELAY _delay_us(STEP_PULSE_ADDITIONAL_MICROSECONDS);
#else
#define STEPPER_DELAY			// used as a no-op in the ISR
#endif

/* Motor channel setup */

#define MOTORS 					4			// number of motors

#define MOTOR_1					0			// array index, #1 must be first
#define MOTOR_2					1
#define MOTOR_3					2
#define MOTOR_4					3			// #4 must be last

#define MOTOR_1_PORT			PORTA		// Typically the X axis
#define MOTOR_2_PORT 			PORTF
#define MOTOR_3_PORT 			PORTE
#define MOTOR_4_PORT 			PORTD

#define MOTOR_PORT_DIR_gm		0x3F		// direction register settings
#define MOTOR_1_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_2_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_3_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_4_PORT_DIR_gm		MOTOR_PORT_DIR_gm	
							// Note: spindle out bits are on PORT_A - b7 & b6

#define MOTOR_1_TIMER			TCC0		// x-axis timer
#define MOTOR_2_TIMER			TCD0
#define MOTOR_3_TIMER			TCE0
#define MOTOR_4_TIMER			TCF0

#define MOTOR_1_TIMER_ISR_vect 	TCC0_OVF_vect // x-axis step rate timer vector
#define MOTOR_2_TIMER_ISR_vect	TCD0_OVF_vect
#define MOTOR_3_TIMER_ISR_vect	TCE0_OVF_vect
#define MOTOR_4_TIMER_ISR_vect 	TCF0_OVF_vect

#define MOTOR_1_ACTIVE_BIT_bm	(1<<3)	// used in Axes to detect move complete
#define MOTOR_2_ACTIVE_BIT_bm	(1<<2)
#define MOTOR_3_ACTIVE_BIT_bm	(1<<1)
#define MOTOR_4_ACTIVE_BIT_bm	(1<<0)

/* timer constants */

#define TC_WGMODE		0			// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3			// assign timer interrupt level (3=hi)
#define TC_CLK_OFF 		0			// turn timer off (clock = 0 Hz)
#define TC_CLK_ON		1			// turn timer clock on (32 Mhz)

/* spindle config and constants - bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	MOTOR_4_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	MOTOR_4_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction

/*  
 *	Stepper axis structures
 */

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	uint32_t step_counter;			// counts steps down to 0 (end of line)
	uint16_t timer_period;			// value loaded into timers
	uint_fast16_t postscale_value;	// timer post-scale value (reloads)
	uint_fast16_t postscale_counter;// timer post-scale counter
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity

	/* hardware device bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

struct Axes {						// All axes + some extra stuff
	uint8_t mutex;					// mutual exclusion flag for dequeuing moves
	uint_fast8_t stopped;			// set TRUE if STOP (FALSE for STARTed)
	uint_fast8_t line_mode;			// set TRUE if LINE command (FALSE for DWELLs)
	uint8_t active_axes;			// bits are set if axis is active. 0 = robot is idle
	struct mvMove *p;				// pointer to dequeued move structure
	struct Axis a[MOTORS];			// 4 motor axis structures, X, Y, Z, A
};
struct Axes ax;						// master axes structure
#define AXIS(x) ax.a[x]				// handy macro for referencing the axis values, 
									// e.g: AXIS(X).port->OUTSET or 
									//		AXIS(i).port->PIN6CTRL or
									//		AXIS(Y).step_counter
/*
 * Global Scope Functions
 */

void st_init(void);					// initialize and start stepper motor subsystem
uint8_t st_isbusy(void);			// returns TRUE is any axis is running, FALSE if idle
void st_set_polarity(const uint8_t motor, const uint8_t polarity);
void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode);
void st_motor_test(void);			// Test stepper motor subsystem
void st_execute_move(void);			// Dequeue and start next linear move in the move buffer
void st_start(void);				// start steppers
void st_stop(void);					// stop steppers
void st_end(void);					// stop steppers and empty all queues

#endif
