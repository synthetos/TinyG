/* stepper.h - stepper motor interface
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef stepper_h
#define stepper_h

void st_init(void);				// initialize and start stepper subsystem
void st_reset(void);
uint8_t st_isbusy(void);		// return TRUE is any axis is running (F=idle)
void st_set_polarity(const uint8_t motor, const uint8_t polarity);
void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode);
void st_motor_test(void);		// Test stepper motor subsystem
uint8_t st_execute_move(void);	// Dequeue and start next move in move buffer
void st_start(void);			// start steppers
void st_stop(void);				// stop steppers
void st_end(void);				// stop steppers and empty all queues

/*
 * Stepper configs and constants
 */

/* The stepper ISRs generate step pulses approximately 1.5 microseconds 
 * duration. The TI DRV8811's used on the TinyG board are fine with this 
 * pulse width. Some outboarded drivers might not be. If the drivers 
 * require a longer pulse uncomment __STEPPER_DELAY and adjust the 
 * microseconds to your requirements. The delay is in addition to the 
 * 1.5 uSec burned in the ISR.
 */
//#define __STEPPER_DELAY
#ifdef __STEPPER_DELAY
#define STEP_PULSE_ADDITIONAL_MICROSECONDS	2
#define STEPPER_DELAY _delay_us(STEP_PULSE_ADDITIONAL_MICROSECONDS);
#else
#define STEPPER_DELAY			// used as a no-op in the ISR
#endif

/* Motor channel setup */

#define MOTORS 					4		// number of motors

#define MOTOR_1					0		// array index, #1 must be first
#define MOTOR_2					1
#define MOTOR_3					2
#define MOTOR_4					3		// motor #4 must be last

#define MOTOR_1_PORT			PORTA	// Typically the X axis
#define MOTOR_2_PORT 			PORTF
#define MOTOR_3_PORT 			PORTE
#define MOTOR_4_PORT 			PORTD

#define MOTOR_PORT_DIR_gm		0x3F	// direction register settings
#define MOTOR_1_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_2_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_3_PORT_DIR_gm		MOTOR_PORT_DIR_gm
#define MOTOR_4_PORT_DIR_gm		MOTOR_PORT_DIR_gm	

#define MOTOR_1_TIMER			TCC0	// x-axis timer
#define MOTOR_2_TIMER			TCD0
#define MOTOR_3_TIMER			TCE0
#define MOTOR_4_TIMER			TCF0

#define MOTOR_1_TIMER_ISR_vect 	TCC0_OVF_vect // x-axis step rate timer vector
#define MOTOR_2_TIMER_ISR_vect	TCD0_OVF_vect
#define MOTOR_3_TIMER_ISR_vect	TCE0_OVF_vect
#define MOTOR_4_TIMER_ISR_vect 	TCF0_OVF_vect

#define MOTOR_1_ACTIVE_BIT_bm	(1<<0)	// used detect move complete
#define MOTOR_2_ACTIVE_BIT_bm	(1<<1)
#define MOTOR_3_ACTIVE_BIT_bm	(1<<2)
#define MOTOR_4_ACTIVE_BIT_bm	(1<<3)

/* timer constants */

#define TC_WGMODE		0		// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3		// assign timer interrupt level (3=hi)
#define TC_CLK_OFF 		0		// turn timer off (clock = 0 Hz)
#define TC_CLK_ON		1		// turn timer clock on (32 Mhz)

/* spindle config and constants
 * spindle uses the min/max bits from the A axis as outputs (A6/A7)
 */
#define SPINDLE_ENABLE_PORT 	MOTOR_4_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)	// also used to set port I/O direction
#define SPINDLE_DIRECTION_PORT 	MOTOR_4_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)	// also used to set port I/O direction

/*  
 *	Stepper axis structures
 */

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	uint32_t step_counter;			// counts steps down to 0 (end of line)
	uint_fast16_t postscale_value;	// timer post-scale value (reloads)
	uint_fast16_t postscale_counter;// timer post-scale counter
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
//	uint16_t timer_period;			// value loaded into timers (UNUSED)

	/* hardware device bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

struct Axes {						// All axes + some extra stuff
	volatile uint8_t mutex;			// mutual exclusion flag for dequeuing moves
	volatile uint8_t stopped;		// set TRUE if STOP (FALSE for STARTed)
	volatile uint8_t active_axes;	// bits are set if axis is active. 0 = robot is idle
	volatile uint8_t line_mode;		// set TRUE if LINE command (FALSE for DWELLs)
	volatile struct mqMove *p;		// pointer to dequeued move structure
	struct Axis a[MOTORS];			// 4 motor axis structures, X, Y, Z, A
};
struct Axes ax;						// master axes structure
#define AXIS(x) ax.a[x]				// handy macro for referencing the axis values, 
									// e.g: AXIS(MOTOR_1).port->OUTSET 
									//		AXIS(i).port->PIN6CTRL
									//		AXIS(MOTOR_2).step_counter
#endif
