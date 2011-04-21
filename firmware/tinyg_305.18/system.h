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

#ifndef system_h
#define system_h

void hw_init(void);						// master hardware init

/* CPU clock */	

#undef F_CPU							// set for delays
#define F_CPU 32000000UL				// should always precede <avr/delay.h>

// Clock Crystal Config. Pick one:
//#define __CLOCK_INTERNAL_32MHZ TRUE	// use internal oscillator
//#define __CLOCK_EXTERNAL_8MHZ	TRUE	// uses PLL to provide 32 MHz system clock
#define __CLOCK_EXTERNAL_16MHZ TRUE		// uses PLL to provide 32 MHz system clock

/* Stepper Ports \- motor port bits are:
 *	b7	(in) max limit switch  			// alt: (out) spindle direction on A axis
 *	b6	(in) min limit switch			// alt: (out) spindle enable on A axis
 *	b5	(out) output bit for encoder port
 *	b4	(out) microstep 1
 *	b3	(out) microstep 0 
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b0	(out) step			(SET is step,  CLR is rest)
 */

enum cfgPortBits {						// motor control port bit positions
	STEP_BIT_bp,						// bit 0
	DIRECTION_BIT_bp,					// bit 1
	MOTOR_ENABLE_BIT_bp,				// bit 2
	MICROSTEP_BIT_0_bp,					// bit 3
	MICROSTEP_BIT_1_bp,					// bit 4
	ENCODER_OUT_BIT_bp,					// bit 5 (4 encoder bits; 1 from each axis)
	MIN_LIMIT_BIT_bp,					// bit 6
	MAX_LIMIT_BIT_bp					// bit 7
};

#define STEP_BIT_bm				(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm		(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm 	(1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm		(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm		(1<<MICROSTEP_BIT_1_bp)
#define ENCODER_OUT_BIT_bm		(1<<ENCODER_OUT_BIT_bp)		
#define MIN_LIMIT_BIT_bm		(1<<MIN_LIMIT_BIT_bp)
#define MAX_LIMIT_BIT_bm		(1<<MAX_LIMIT_BIT_bp) // motor control port bit masks

#endif
