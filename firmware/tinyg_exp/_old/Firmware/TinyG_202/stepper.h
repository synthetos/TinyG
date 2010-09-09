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
#define STEPPER_DELAY				// used as a no-op in the ISR
#endif


#ifdef __RILEY					// support for Rileys's blown X axis
#define X_MOTOR_PORT PORTD		// labeled as motor #1
#define Y_MOTOR_PORT PORTF		//					#2
#define Z_MOTOR_PORT PORTE		//					#3
#define A_MOTOR_PORT PORTA		//					#4
#else
#define X_MOTOR_PORT PORTA		// labeled as motor #1
#define Y_MOTOR_PORT PORTF		//					#2
#define Z_MOTOR_PORT PORTE		//					#3
#define A_MOTOR_PORT PORTD		//					#4
#endif

#define MOTOR_PORT_DIR_gm		0x3F		// direction register settings
#define X_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define Y_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define Z_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define A_MOTOR_PORT_DIR_gm		MOTOR_PORT_DIR_gm	
								// Note: spindle out bits are on PORT_A - b7 & b6

#define X_TIMER					TCC0		// x-axis timer
#define Y_TIMER					TCD0
#define Z_TIMER					TCE0
#define A_TIMER					TCF0

#define X_TIMER_ISR_vect 		TCC0_OVF_vect // x-axis step rate timer vector
#define Y_TIMER_ISR_vect		TCD0_OVF_vect
#define Z_TIMER_ISR_vect		TCE0_OVF_vect
#define A_TIMER_ISR_vect 		TCF0_OVF_vect

#define X_ACTIVE_BIT_bm			(1<<3)		// used in Axes to detect move complete
#define Y_ACTIVE_BIT_bm			(1<<2)
#define Z_ACTIVE_BIT_bm			(1<<1)
#define A_ACTIVE_BIT_bm			(1<<0)

/* timer constants */

#define TC_WGMODE		0			// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3			// assign timer interrupt level (3=hi)
#define TC_CLK_OFF 		0			// turn timer off (clock = 0 Hz)
#define TC_CLK_ON		1			// turn timer clock on (32 Mhz)

/* spindle config and constants - bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	A_MOTOR_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	A_MOTOR_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction


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
struct Axes ax;						// master axes structure
#define AXIS(x) ax.a[x]				// handy macro for referencing the axis values, 
									// e.g: AXIS(X).port->OUTSET or 
									//		AXIS(i).port->PIN6CTRL or
									//		AXIS(Y).step_counter
/*
 * Global Scope Functions
 */

void st_init(void);					// Initialize and start stepper motor subsystem
void st_motor_test(void);			// Test stepper motor subsystem
void st_execute_move(void);			// Dequeue and start next linear move in the move buffer
void st_set_polarity(uint8_t axis, uint8_t polarity);
void st_stop_steppers(void);		// Kill current move
void st_terminate(void);			// Terminate moves after the current move

#endif
