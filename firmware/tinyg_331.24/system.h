/*
 * system.h - system configuration values 
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
 * INTERRUPT USAGE - TinyG uses a lot of them all over the place
 *
 *	HI	Stepper DDA pulse generation		(set in stepper.h)
 *	HI	Stepper load routine SW interrupt	(set in stepper.h)
 *	HI	Dwell timer counter 				(set in stepper.h)
 *  LO	Segment execution SW interrupt		(set in stepper.h) 
 *	MED	GPIO1 switch port					(set in gpio.h)
 *  MED	Serial RX for USB & RS-485			(set in xio_usart.h)
 *  MED	Serial TX for USB & RS-485			(set in xio_usart.h)
 *	LO	Real time clock interrupt			(set in xmega_rtc.h)
 */
#ifndef system_h
#define system_h

void sys_init(void);					// master hardware init
//uint8_t sys_read_signature(uint8_t index);

/* CPU clock */	

#undef F_CPU							// set for delays
#define F_CPU 32000000UL				// should always precede <avr/delay.h>

// Clock Crystal Config. Pick one:
//#define __CLOCK_INTERNAL_32MHZ TRUE	// use internal oscillator
//#define __CLOCK_EXTERNAL_8MHZ	TRUE	// uses PLL to provide 32 MHz system clock
#define __CLOCK_EXTERNAL_16MHZ TRUE		// uses PLL to provide 32 MHz system clock

/* Timers and interrupt vectors */
#define DEVICE_TIMER_DDA			TCC0			// DDA timer
#define DEVICE_TIMER_DDA_ISR_vect	TCC0_OVF_vect
#define DEVICE_TIMER_DWELL	 		TCD0			// Dwell timer
#define DEVICE_TIMER_DWELL_ISR_vect TCD0_OVF_vect
#define DEVICE_TIMER_LOAD			TCE0			// Loader timer (SW interrupt)
#define DEVICE_TIMER_LOAD_ISR_vect	TCE0_OVF_vect
#define DEVICE_TIMER_EXEC			TCF0			// Exec timer (SW interrupt)
#define DEVICE_TIMER_EXEC_ISR_vect	TCF0_OVF_vect

/* Stepper / Switch Ports:
 *	b0	(out) step			(SET is step,  CLR is rest)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b3	(out) microstep 0 
 *	b4	(out) microstep 1
 *	b5	(out) output bit for GPIO port1
 *	b6	(in) min limit switch on GPIO 2
 *	b7	(in) max limit switch on GPIO 2
 */
#define MOTOR_PORT_DIR_gm 0x3F	// dir settings: lower 6 out, upper 2 in

enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	GPIO2_MIN_BIT_bp,		// bit 6 (4 gpio2 input bits for switch closures)
	GPIO2_MAX_BIT_bp		// bit 7 (4 gpio2 input bits for switch closures)
};

#define STEP_BIT_bm			(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm	(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm	(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm	(1<<MICROSTEP_BIT_1_bp)
#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)
#define GPIO2_MIN_BIT_bm	(1<<GPIO2_MIN_BIT_bp)
#define GPIO2_MAX_BIT_bm	(1<<GPIO2_MAX_BIT_bp) // motor control port bit masks

/* Motor & switch port assignments */

#define DEVICE_PORT_MOTOR_1		PORTA
#define DEVICE_PORT_MOTOR_2 	PORTF
#define DEVICE_PORT_MOTOR_3		PORTE
#define DEVICE_PORT_MOTOR_4		PORTD
#define DEVICE_PORT_GPIO2_IN	PORTB

enum gpio1Inputs {
	GPIO1_IN_BIT_0_bp = 0,	// gpio1 input bit 0
	GPIO1_IN_BIT_1_bp,		// gpio1 input bit 1
	GPIO1_IN_BIT_2_bp,		// gpio1 input bit 2
	GPIO1_IN_BIT_3_bp		// gpio1 input bit 3
};
#define GPIO1_IN_BIT_0_bm	(1<<GPIO1_IN_BIT_0_bp)
#define GPIO1_IN_BIT_1_bm	(1<<GPIO1_IN_BIT_1_bp)
#define GPIO1_IN_BIT_2_bm	(1<<GPIO1_IN_BIT_2_bp)
#define GPIO1_IN_BIT_3_bm	(1<<GPIO1_IN_BIT_3_bp)

/* Bit assignments for GPIO1_OUTs for spindle, PWM and coolant */

#define SPINDLE_BIT	0x01	// spindle on/off
#define SPINDLE_DIR	0x02	// spindle direction, 1=CW, 0=CCW
#define SPINDLE_PWM	0x04	// spindle PWN port
#define MIST_COOLANT_BIT	0x08	// coolant on/off - these are the same due to limited ports
#define FLOOD_COOLANT_BIT	0x08	// coolant on/off

#endif
