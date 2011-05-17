/* stepper.h - stepper motor interface
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

#ifndef stepper_h
#define stepper_h

void st_init(void);			// initialize and start stepper subsystem
uint8_t st_isbusy(void);	// return TRUE is any axis is running (F=idle)
void st_set_polarity(const uint8_t motor, const uint8_t polarity);
void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode);
void st_motor_test(void);	// Test stepper motor subsystem
void st_request_load(void);	// Trigger software interrupt to request load
void st_stop(void);			// stop steppers
void st_start(void);		// start steppers
void st_end(void);			// stop steppers and empty all queues
void st_print_stepper_state(void);

/*
 * Stepper configs and constants
 */

/* The stepper ISRs generate step pulses approximately 1 microseconds 
 * duration. The TI DRV8811's used on the TinyG board are fine with this 
 * pulse width. Some outboarded drivers might not be. If the drivers 
 * require a longer pulse uncomment __STEPPER_DELAY and adjust the 
 * microseconds to your requirements. The delay is in addition to the 
 * 1 uSec burned in the ISR.
 */
//#define __STEPPER_DELAY
#ifdef __STEPPER_DELAY
#define STEP_PULSE_ADDITIONAL_MICROSECONDS	2
#define STEPPER_DELAY _delay_us(STEP_PULSE_ADDITIONAL_MICROSECONDS);
#else
#define STEPPER_DELAY			// used as a no-op in the ISR
#endif

/* common timer values */
#define TIMER_DISABLE 	0		// turn timer off (clock = 0 Hz)
#define TIMER_ENABLE	1		// turn timer clock on (F_CPU = 32 Mhz)
#define TIMER_WGMODE	0		// normal mode (count to TOP and rollover)
#define TIMER_OVFINTLVL	3		// assign timer interrupt level (3=hi)

/* DDA timer values */
#define F_DDA 			(double)50000		// 50 Khz
#define DDA_PERIOD		(uint16_t)((double)F_CPU / F_DDA) // assumes timer runs at full F_CPU
#define DDA_MHZ 		(double)(F_DDA/1000000)
#define DDA_TIMER		TCC0
#define DDA_TIMER_ISR_vect TCC0_OVF_vect

/* DWELL timer values */
#define F_DWELL			(double)10000	// 10 Khz
#define DWELL_PERIOD	(uint16_t)((double)F_CPU / F_DWELL) // assumes timer runs at full F_CPU
#define DWELL_MHZ		(double)(F_DWELL/1000000)
#define DWELL_TIMER		TCD0
#define DWELL_TIMER_ISR_vect TCD0_OVF_vect

/* Software interrupt timer settings */
#define SWI_PERIOD 		2000	// cycles you have to shut it off
#define SWI_TIMER		TCE0
#define SWI_TIMER_ISR_vect TCE0_OVF_vect
								// before it fires again

/* Motor channel setup */
								// NOTE: MOTORS is defined in tinyg.h
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

/* spindle config and constants
 * spindle uses the min/max bits from the A axis as outputs (A6/A7)
 */
#define SPINDLE_ENABLE_PORT 	MOTOR_4_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)	// also used to set port I/O direction
#define SPINDLE_DIRECTION_PORT 	MOTOR_4_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)	// also used to set port I/O direction

/*
 *	Stepper structures
 */

struct stMotor { 					// one per controlled motor
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
	int32_t steps;					// total steps in axis
	int32_t counter;				// DDA counter for axis
	struct PORT_struct *port;		// motor control port
};

struct stSingleton {				// Stepper static values and axis parameters
	int32_t timer_ticks;			// total DDA or dwell ticks in this move
	int32_t timer_ticks_left;		// down-counter for above
	struct TC0_struct *dda_timer;	// timer/counter (type 0)
	struct TC0_struct *dwell_timer;	
	struct TC0_struct *swi_timer;
	struct stMotor m[MOTORS];		// 4 motor axis structures
	volatile struct mqMove *p;		// pointer to dequeued move structure
};
struct stSingleton st;				// master axes structure
#define MOTOR(a) st.m[a]			// handy macro for referencing the motor structs, 
									// e.g: MOTOR(MOTOR_1).port->OUTSET 
									//		MOTOR(i).port->PIN6CTRL
									//		MOTOR(MOTOR_2).step_counter
#endif
