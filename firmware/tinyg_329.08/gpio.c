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
 *	Switch Modes
 *
 *	The switches are considered to be homing switches when machine_state is
 *	MACHINE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to abort and go into reset
 *
 * 	The switches trigger an interrupt on the leading edge (falling) and lockout 
 *	subsequent interrupts for the defined lockout period. This approach beats 
 *	doing debouncing as an integration as the switches fire immediately.
 *
 *	Note: This module assumes the switches are normally open (and active LO).
 *	At some point it should support NC switches and optos by config option
 */

#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "system.h"							// gpio port bits are mapped here
#include "config.h"
#include "controller.h"
#include "gpio.h"
#include "canonical_machine.h"

//#ifdef __dbSHOW_LIMIT_SWITCH
#include <stdio.h>							// precursor to xio.h
#include <avr/pgmspace.h>					// precursor to xio.h
#include "xio.h"							// for debug statements
//#endif

/*
 * variables and settings 
 */

static uint8_t gp_port_value;				// global for synthetic port read value

#define	SW_OPC_gc PORT_OPC_PULLUP_gc		// totem poll pullup mode
//#define SW_ISC_gc PORT_ISC_RISING_gc		// ISRs on *trailing* edge
#define SW_ISC_gc PORT_ISC_FALLING_gc		// ISRs on *leading* edge
#define SW_LOCKOUT_TICKS 10					// ticks are ~10ms each


/*
 * sw_init() - initialize limit switches
 *
 * This function assumes st_init() has been run previously.
 */

void gp_init(void) 
{
	uint8_t i;

	// GPIO1 - switch port
	for (i=0; i<MOTORS; i++) {
		// set initial port bit state to OFF
		device.port[i]->DIRSET = GPIO2_MIN_BIT_bm;	// set min to output
		device.port[i]->OUTSET = GPIO2_MIN_BIT_bm;	// min bit off
		device.port[i]->DIRSET = GPIO2_MAX_BIT_bm;	// set max to output
		device.port[i]->OUTSET = GPIO2_MAX_BIT_bm;	// max bit off

		// setup ports bits as inputs

		device.port[i]->DIRCLR = GPIO2_MIN_BIT_bm;		 // set min input
		device.port[i]->PIN6CTRL = (SW_OPC_gc | SW_ISC_gc);// pin modes
		device.port[i]->INT0MASK = GPIO2_MIN_BIT_bm;	 // min on INT0

		device.port[i]->DIRCLR = GPIO2_MAX_BIT_bm;		 // set max input
		device.port[i]->PIN7CTRL = (SW_OPC_gc | SW_ISC_gc);// pin modes
		device.port[i]->INT1MASK = GPIO2_MAX_BIT_bm;	 // max on INT1

		// set interrupt levels. Interrupts must be enabled in main()
		device.port[i]->INTCTRL = GPIO1_INTLVL;			// see gpio.h for setting
//		device.port[i]->INTCTRL = (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc);
//		device.port[i]->INTCTRL |= (PORT_INT0LVL_LO_gc|PORT_INT1LVL_LO_gc);
	}
	gp_clear_switches();
	gp.sw_count = 0;

	// GPIO2 - inputs and outputs port
	// (nothing here yet)
}

/*
 * ISRs - Switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ gp_switch_isr_helper(SW_MIN_X);}
ISR(Y_MIN_ISR_vect)	{ gp_switch_isr_helper(SW_MIN_Y);}
ISR(Z_MIN_ISR_vect)	{ gp_switch_isr_helper(SW_MIN_Z);}
ISR(A_MIN_ISR_vect)	{ gp_switch_isr_helper(SW_MIN_A);}

ISR(X_MAX_ISR_vect)	{ gp_switch_isr_helper(SW_MAX_X);}
ISR(Y_MAX_ISR_vect)	{ gp_switch_isr_helper(SW_MAX_Y);}
ISR(Z_MAX_ISR_vect)	{ gp_switch_isr_helper(SW_MAX_Z);}
ISR(A_MAX_ISR_vect)	{ gp_switch_isr_helper(SW_MAX_A);}

void gp_switch_isr_helper(uint8_t sw_flag)
{
	if (gp.sw_count == 0) {						// true if not in a debounce lockout 
		if (cm.homing_state == HOMING_CYCLE) { 	// true if currently-in-a-homing-cycle
//			cm_feedhold();						// invoke a feedhold
			sig.sig_feedhold = true;			// invoke a feedhold
			gp.sw_thrown = true;				// triggers the switch handler tasks
			gp.sw_flags[sw_flag] = true;
			gp.sw_count = SW_LOCKOUT_TICKS;		// start the debounce lockout timer
		} else {
			sig.sig_abort = true;				// you can't do an abort from an interrupt
		}
	}
}

/*
 * gp_switch_timer_callback() - called from RTC for each RTC tick.
 *
 *	Also, once the lockout expires (gets to zero) it reads the switches
 *	sets sw_thrown to picked up by switch_handler if the switch was thrown 
 *	and remained thrown (as can happen in some homing recovery cases)
 */

inline void gp_switch_timer_callback(void)
{
	if (gp.sw_count) { --gp.sw_count;}	// counts down to zero and sticks on zero
}

/*
 * gp_clear_switches() - clear all limit switches but not lockout count
 *
 * Note: Can't use memset on the flags because they are Volatile
 */

void gp_clear_switches() 
{
	gp.sw_thrown = false;
	for (uint8_t i=0; i < SW_SIZE; i++) {
		gp.sw_flags[i] = false; 
	}
}

/*
 * gp_read_switches() - read the switches into the switch flag array
 *
 *	As configured switches are active LO
 *
 *	This routine relies on SW_FLAG array being in order of 
 *	MIN_X, MIN_Y, MIN_Z, MIN_A, MAX_X, MAX_Y, MAX_Z, MAX_A
 *	and there being 2 groups of 4 flags.
 */

void gp_read_switches()
{
	gp_clear_switches();				// clear flags and thrown

	for (uint8_t i=0; i<4; i++) {
		if (!(device.port[i]->IN & GPIO2_MIN_BIT_bm)) { 	// min
			gp.sw_flags[i] = true;
			gp.sw_thrown = true;
		}
		if (!(device.port[i]->IN & GPIO2_MAX_BIT_bm)) { 	// max
			gp.sw_flags[i + SW_OFFSET_TO_MAX] = true;
			gp.sw_thrown = true;
		}
	}
}

/*
 * gp_get_switch() - return TRUE if switch is thrown
 * gp_set_switch() - diagnostic function for emulating a switch closure
 */

uint8_t gp_get_switch(uint8_t sw_flag) { return (gp.sw_flags[sw_flag]);}

void gp_set_switch(uint8_t sw_flag)
{
	gp.sw_thrown = true;
	gp.sw_flags[sw_flag] = true;
}

/*
 * sw_switch_handler() - main limit switch handler; called from controller loop
 */

uint8_t gp_switch_handler(void)
{
	if (gp.sw_thrown == false) {			// leave if no switches are thrown
		return (TG_NOOP);
	}
	gp_clear_switches();					// reset the switches last, not before
	return (TG_OK);
}

/*
 * gp_set_bit_on() - turn bit on
 * gp_set_bit_off() - turn bit on
 */

void gp_set_bit_on(uint8_t b)
{
	if (b & 0x01) { DEVICE_PORT_MOTOR_4.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { DEVICE_PORT_MOTOR_3.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { DEVICE_PORT_MOTOR_2.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x08) { DEVICE_PORT_MOTOR_1.OUTSET = GPIO1_OUT_BIT_bm;}
}

void gp_set_bit_off(uint8_t b)
{
	if (b & 0x01) { DEVICE_PORT_MOTOR_4.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { DEVICE_PORT_MOTOR_3.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { DEVICE_PORT_MOTOR_2.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x08) { DEVICE_PORT_MOTOR_1.OUTCLR = GPIO1_OUT_BIT_bm;}
}

/*
 * gp_write_port() - write lowest 4 bits of a byte to GPIO 1 output port
 *
 * This is a hack to hide the fact that we've scattered the encode output
 * bits all over the place becuase we have no more contiguous ports left. 
 */

void gp_write_port(uint8_t b)
{
	gp_port_value = b;

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
 * gp_toggle_port() - toggle lowest 4 bits of a byte to output port
 *
 *	Note: doesn't take transitions form bit_on / bit_off into account
 */

void gp_toggle_port(uint8_t b)
{
	gp_port_value ^= b;	// xor the stored port value with b
	gp_write_port(gp_port_value);
}

/*
 * _gp_show_switch() - simple display routine
 */

#ifdef __dbSHOW_LIMIT_SWITCH
static void _gp_show_switch(void);
static void _gp_show_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown %d %d %d %d   %d %d %d %d\n"), 
		sw.flag[SW_MIN_X], sw.flag[SW_MAX_X], 
		sw.flag[SW_MIN_Y], sw.flag[SW_MAX_Y], 
		sw.flag[SW_MIN_Z], sw.flag[SW_MAX_Z], 
		sw.flag[SW_MIN_A], sw.flag[SW_MAX_A]);
}
#endif

