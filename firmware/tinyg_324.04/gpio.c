/*
 * gpio.c - geberal purpose IO bits - including limit switches, inputs, outputs
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
 *	This GPIO file is where all parallel port bits are managed that are 
 *	not already taken up by steppers, serial ports, JTAG or PDI programming
 *
 *	There are 2 GPIO ports:
 *
 *	  gpio1	  Located on 8x2 header next to the RS485 plugs (RJ45s)
 *			  Four (4) output bits capable of driving 3.3v or 5v logic
 *			  Four (4) level converted input bits capable of being driven
 *				by 3.3v or 5v logic
 *
 *	  gpio2	  Located on 9x2 header on "bottom" of board
 *			  Eight (8) non-level converted input bits
 *			  Eight (8) ground pins
 *			  Two   (2) 3.3v power pins
 *			  Inputs can be used as switch contact inputs or 
 *				3.3v input bits depending on port configuration
 *				**** These bits CANNOT be used as 5v inputs ****
 */
/*
 * 	The limit switches trigger an interrupt on the leading edge (falling)
 *	and lockout subsequent interrupts for the defined lockout period. 
 *	This beats doing debouncing as an integration as it fires immediately.
 *
 *	Note: This mocule assumes the switches are normally open (and active LO).
 *	At some point it should support NC switches by configuration option.
 */

#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "system.h"			// gpio port bits are mapped here
#include "config.h"
//#include "stepper.h"		// st_init() must run before sw_init()
#include "controller.h"
#include "gpio.h"
#include "canonical_machine.h"

#ifdef __dbSHOW_LIMIT_SWITCH
#include <stdio.h>			// precursor to xio.h
#include <avr/pgmspace.h>	// precursor to xio.h
#include "xio.h"			// for debug statements
#endif

/*
 * settings and locals
 */

#define	SW_OPC_gc PORT_OPC_PULLUP_gc		// totem poll pullup mode
//#define SW_ISC_gc PORT_ISC_RISING_gc		// ISRs on *trailing* edge
#define SW_ISC_gc PORT_ISC_FALLING_gc		// ISRs on *leading* edge
#define SW_LOCKOUT_TICKS 25					// ticks are ~10ms each


/*
 * Interrupt vectors - these are hard-wired to ports in the xmega
 * if you change axis port assignments all these need to change, too
 */

#define X_MIN_ISR_vect	PORTA_INT0_vect
#define X_MAX_ISR_vect	PORTA_INT1_vect
#define Y_MIN_ISR_vect	PORTF_INT0_vect
#define Y_MAX_ISR_vect	PORTF_INT1_vect
#define Z_MIN_ISR_vect	PORTE_INT0_vect
#define Z_MAX_ISR_vect	PORTE_INT1_vect
#define A_MIN_ISR_vect	PORTD_INT0_vect
#define A_MAX_ISR_vect	PORTD_INT1_vect


static uint8_t port_value;


/*
 * sw_init() - initialize limit switches
 *
 * This function assumes st_init() has been run previously.
 */

void sw_init(void) 
{
	uint8_t i;

	for (i=0; i<MOTORS; i++) {
		// set initial port bit state to OFF
		device.port[i]->DIRSET = MIN_LIMIT_BIT_bm;	// set min to output
		device.port[i]->OUTSET = MIN_LIMIT_BIT_bm;	// min bit off
		device.port[i]->DIRSET = MAX_LIMIT_BIT_bm;	// set max to output
		device.port[i]->OUTSET = MAX_LIMIT_BIT_bm;	// max bit off

		// setup ports bits as inputs
		device.port[i]->DIRCLR = MIN_LIMIT_BIT_bm;		 // set min input
		device.port[i]->PIN6CTRL = (SW_OPC_gc | SW_ISC_gc);// pin modes
		device.port[i]->INT0MASK = MIN_LIMIT_BIT_bm;	 // min on INT0

		device.port[i]->DIRCLR = MAX_LIMIT_BIT_bm;		 // set max input
		device.port[i]->PIN7CTRL = (SW_OPC_gc | SW_ISC_gc);// pin modes
		device.port[i]->INT1MASK = MAX_LIMIT_BIT_bm;	 // max on INT1

		// set interrupt levels. Interrupts must be enabled in main()
		device.port[i]->INTCTRL = (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc);
	}
	sw_clear_limit_switches();
	sw.count = 0;
}

/*
 * _sw_show_limit_switch() - simple display routine
 */

#ifdef __dbSHOW_LIMIT_SWITCH
static void _sw_show_limit_switch(void);
static void _sw_show_limit_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown %d %d %d %d   %d %d %d %d\n"), 
		sw.flag[SW_X_MIN], sw.flag[SW_X_MAX], 
		sw.flag[SW_Y_MIN], sw.flag[SW_Y_MAX], 
		sw.flag[SW_Z_MIN], sw.flag[SW_Z_MAX], 
		sw.flag[SW_A_MIN], sw.flag[SW_A_MAX]);
}
#endif

/*
 * ISRs - Limit switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ sw_isr_helper(SW_X_MIN);}
ISR(X_MAX_ISR_vect)	{ sw_isr_helper(SW_X_MAX);}
ISR(Y_MIN_ISR_vect)	{ sw_isr_helper(SW_Y_MIN);}
ISR(Y_MAX_ISR_vect)	{ sw_isr_helper(SW_Y_MAX);}
ISR(Z_MIN_ISR_vect)	{ sw_isr_helper(SW_Z_MIN);}
ISR(Z_MAX_ISR_vect)	{ sw_isr_helper(SW_Z_MAX);}
ISR(A_MIN_ISR_vect)	{ sw_isr_helper(SW_A_MIN);}
ISR(A_MAX_ISR_vect)	{ sw_isr_helper(SW_A_MAX);}

void sw_isr_helper(uint8_t flag)
{
	if (!sw.count) {
		cm_async_end();			// stop all motion immediately
		sw.thrown = TRUE;		// triggers the sw_handler tasks
		sw.flag[flag] = TRUE;
		sw.count = SW_LOCKOUT_TICKS;
	}
}

/*
 * sw_clear_limit_switches() - clear all limit switches but not lockout count
 *
 * Note: can't use memset on the flags because they are Volatile
 */

void sw_clear_limit_switches() 
{
	sw.thrown = FALSE;
	for (uint8_t i=0; i < SW_FLAG_SIZE; i++) {
		sw.flag[i] = FALSE; 
	}
}

/*
 * sw_read_limit_switches() - read the switches & set flags
 *
 *	As configured, switches are active LO
 */

void sw_read_limit_switches() 
{
#ifdef __DISABLE_LIMITS			// simulation mode
	sw_clear_limit_switches();		// clear flags and thrown
	return;
#endif

	uint8_t i;
	uint8_t flag = 0;				// used to index flag array

	sw_clear_limit_switches();		// clear flags and thrown

	for (i=0; i<MOTORS; i++) {
		if (!(device.port[i]->IN & MIN_LIMIT_BIT_bm)) { 	// min
			sw.flag[flag] = TRUE;
			sw.thrown = TRUE;
		}
		flag++;
		if (!(device.port[i]->IN & MAX_LIMIT_BIT_bm)) { 	// max
			sw.flag[flag] = TRUE;
			sw.thrown = TRUE;
		}
		flag++;
	}
#ifdef __dbSHOW_LIMIT_SWITCH
	_sw_show_limit_switch();
#endif
}

/*
 * getters - return TRUE if switch is thrown
 */

uint8_t sw_any_thrown(void)	{ return (sw.thrown); }
uint8_t sw_xmin_thrown(void) { return (sw.flag[SW_X_MIN]); }
uint8_t sw_xmax_thrown(void) { return (sw.flag[SW_X_MAX]); }
uint8_t sw_ymin_thrown(void) { return (sw.flag[SW_Y_MIN]); }
uint8_t sw_ymax_thrown(void) { return (sw.flag[SW_Y_MAX]); }
uint8_t sw_zmin_thrown(void) { return (sw.flag[SW_Z_MIN]); }
uint8_t sw_zmax_thrown(void) { return (sw.flag[SW_Z_MAX]); }
uint8_t sw_amin_thrown(void) { return (sw.flag[SW_A_MIN]); }
uint8_t sw_amax_thrown(void) { return (sw.flag[SW_A_MAX]); }

/*
 * sw_timer_callback() - call from timer for each clock tick.
 *
 * Also, once the lockout expires (gets to zero) it reads the switches
 * sets sw.thrown to picked up by sw_handler if the switch was thrown and
 * remained thrown (as can happen in some homing recovery cases)
 */

inline void sw_rtc_callback(void)
{
	if (sw.count) {
		--sw.count;
	}
}

/*
 * sw_handler() - main limit switch handler; called from controller loop
 */

uint8_t sw_handler(void)
{
	if (!sw.thrown) {		// leave if no switches are thrown
		return (TG_NOOP);
	}
#ifdef __dbSHOW_LIMIT_SWITCH
	_sw_show_limit_switch();
#endif
	if (cfg.homing_state == HOMING_COMPLETE) { 
//	if (cfg.homing_state != HOMING_IN_PROCESS) {
		tg_application_startup();			// initiate homing cycle
		return (TG_OK);
	}
	sw_clear_limit_switches();				// do this last, not before
	return (TG_OK);
}

void en_init(void) 
{
	return;
}

/*
 * en_bit_on() - turn bit on
 * en_bit_off() - turn bit on
 */

void en_bit_on(uint8_t b)
{
	if (b & 0x01) {
		DEVICE_PORT_MOTOR_4.OUTSET = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x02) {
		DEVICE_PORT_MOTOR_3.OUTSET = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x04) {
		DEVICE_PORT_MOTOR_2.OUTSET = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x08) {
		DEVICE_PORT_MOTOR_1.OUTSET = GPIO1_OUT_BIT_bm;
	}
}

void en_bit_off(uint8_t b)
{
	if (b & 0x01) {
		DEVICE_PORT_MOTOR_4.OUTCLR = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x02) {
		DEVICE_PORT_MOTOR_3.OUTCLR = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x04) {
		DEVICE_PORT_MOTOR_2.OUTCLR = GPIO1_OUT_BIT_bm;
	}
	if (b & 0x08) {
		DEVICE_PORT_MOTOR_1.OUTCLR = GPIO1_OUT_BIT_bm;
	}
}

/*
 * en_write() - write lowest 4 bits of a byte to GPIO 1 output port
 *
 * This is a hack to hide the fact that we've scattered the encode output
 * bits all over the place becuase we have no more contiguous ports left. 
 */

void en_write(uint8_t b)
{
	port_value = b;

	if (b & 0x01) { // b0 is on MOTOR_4 (A axis)
		DEVICE_PORT_MOTOR_4.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_4.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x02) { // b1 is on MOTOR_3 (Z axis)
		DEVICE_PORT_MOTOR_3.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_3.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x04) { // b2 is on MOTOR_2 (Y axis)
		DEVICE_PORT_MOTOR_2.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_2.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x08) { // b3 is on MOTOR_1 (X axis)
		DEVICE_PORT_MOTOR_1.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_1.OUTCLR = GPIO1_OUT_BIT_bm;
	}
}

/*
 * en_toggle() - toggle lowest 4 bits of a byte to output port
 *
 *	Note: doesn't take transitions form bit_on / bit_off into account
 */

void en_toggle(uint8_t b)
{
	port_value ^= b;	// xor the stored port value with b
	en_write(port_value);
}

