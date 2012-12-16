/*
 * system.h - system hardware device configuration values 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
 *  LO	Serial TX for USB & RS-485			(set in xio_usart.h)
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

/* Motor & switch port assignments */

#define PORT_MOTOR_1		PORTA		// Note: motor and GPIO2 mappings are not the same
#define PORT_MOTOR_2 		PORTF
#define PORT_MOTOR_3		PORTE
#define PORT_MOTOR_4		PORTD

#define GPIO2_X_MIN_MAX		PORTA		// lines up with ISR vector assignments in gpio.h
#define GPIO2_Y_MIN_MAX		PORTD
#define GPIO2_Z_MIN_MAX		PORTE
#define GPIO2_A_MIN_MAX		PORTF

#define SW_PORT_X 0						// port mapping looked at the other way
#define SW_PORT_Y 3
#define SW_PORT_Z 2
#define SW_PORT_A 1

// These next four must be changed when the PORT_MOTOR_* definitions change!
#define PORTCFG_VP0MAP_PORT_MOTOR_1_gc PORTCFG_VP0MAP_PORTA_gc
#define PORTCFG_VP1MAP_PORT_MOTOR_2_gc PORTCFG_VP1MAP_PORTF_gc
#define PORTCFG_VP2MAP_PORT_MOTOR_3_gc PORTCFG_VP2MAP_PORTE_gc
#define PORTCFG_VP3MAP_PORT_MOTOR_4_gc PORTCFG_VP3MAP_PORTD_gc

#define PORT_MOTOR_1_VPORT	VPORT0
#define PORT_MOTOR_2_VPORT	VPORT1
#define PORT_MOTOR_3_VPORT	VPORT2
#define PORT_MOTOR_4_VPORT	VPORT3

/*
 * Port setup - Stepper / Switch Ports:
 *	b0	(out) step			(SET is step,  CLR is rest)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b3	(out) microstep 0 
 *	b4	(out) microstep 1
 *	b5	(out) output bit for GPIO port1
 *	b6	(in) min limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 *	b7	(in) max limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 */
#define MOTOR_PORT_DIR_gm 0x3F	// dir settings: lower 6 out, upper 2 in

enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	SW_MIN_BIT_bp,			// bit 6 (4 input bits for switch closures)
	SW_MAX_BIT_bp			// bit 7 (4 input bits for switch closures)
};

#define STEP_BIT_bm			(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm	(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm	(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm	(1<<MICROSTEP_BIT_1_bp)
#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)
#define SW_MIN_BIT_bm		(1<<SW_MIN_BIT_bp)
#define SW_MAX_BIT_bm		(1<<SW_MAX_BIT_bp) // motor control port bit masks

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

#define SPINDLE_BIT			0x08		// spindle on/off
#define SPINDLE_DIR			0x04		// spindle direction, 1=CW, 0=CCW
#define SPINDLE_PWM			0x02		// spindle PWN port
#define MIST_COOLANT_BIT	0x01		// coolant on/off - these are the same due to limited ports
#define FLOOD_COOLANT_BIT	0x01		// coolant on/off
#define INDICATOR_LED		1			// can use the spindle direction as an indicator LED

/* Timer assignments - see specific modules for details) */

#define TIMER_DDA			TCC0		// DDA timer 	(see stepper.h)
#define TIMER_DWELL	 		TCD0		// Dwell timer	(see stepper.h)
#define TIMER_LOAD			TCE0		// Loader timer	(see stepper.h)
#define TIMER_EXEC			TCF0		// Exec timer	(see stepper.h)
#define TIMER_5				TCC1		// unallocated timer
#define TIMER_PWM1			TCD1		// PWM timer #1 (see pwm.c)
#define TIMER_PWM2			TCE1		// PWM timer #2	(see pwm.c)


/**** Device singleton - global structure to allow iteration through similar devices ****/
// Ports are shared between steppers and GPIO so we need a global struct

struct deviceSingleton {
	PORT_t *port[MOTORS];	// motor control ports
};
struct deviceSingleton device;

#endif
