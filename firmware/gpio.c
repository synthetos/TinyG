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
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge 
 *	and lockout subsequent interrupts for the defined lockout period. This approach 
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge 
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "system.h"							// gpio port bits are mapped here
#include "gpio.h"
#include "canonical_machine.h"

//#ifdef __dbSHOW_LIMIT_SWITCH
#include <stdio.h>							// precursor to xio.h
#include <avr/pgmspace.h>					// precursor to xio.h
#include "xio/xio.h"						// for debug statements
//#endif

/*
 * variables and settings 
 */
/*
struct gpioControls {						// GPIO controls for various modes
 	uint8_t pin_mode; 						// pin mode for GPIO port
	uint8_t int_mode;						// interrupt mode for GPIO port
}; 
static struct gpioControls gctl;
*/

#define SW_LOCKOUT_TICKS 10					// ticks are ~10ms each

static void _switch_isr_helper(uint8_t sw_flag);
static uint8_t gpio_port_value;				// global for synthetic port read value

/*
 * gpio_init() - initialize homing/limit switches
 *
 *	This function assumes st_init() has been run previously.
 *	The device structure singleton is defined in system.h
 *	These inits assume stepper.c, st_init() has run previously
 */

void gpio_init(void) 
{
	uint8_t int_mode;							// interrupt mode
	uint8_t pin_mode = PORT_OPC_PULLUP_gc;		// pin mode. see iox192a3.h for details

	// GPIO1 - switch port
	for (uint8_t i=0; i<MOTORS; i++) {

		// set initial port bit state to OFF
		device.port[i]->DIRSET = GPIO2_MIN_BIT_bm;			// set min to output
		device.port[i]->OUTSET = GPIO2_MIN_BIT_bm;			// min bit off
		device.port[i]->DIRSET = GPIO2_MAX_BIT_bm;			// set max to output
		device.port[i]->OUTSET = GPIO2_MAX_BIT_bm;			// max bit off

		// set interrupt mode for NO or NC
		if ((cfg.a[i].switch_mode == SW_MODE_HOMING_NO) || 
			(cfg.a[i].switch_mode == SW_MODE_ENABLED_NO)) {
			int_mode = PORT_ISC_FALLING_gc;
		} else {
			int_mode = PORT_ISC_RISING_gc;
		}

		// setup ports input bits (previously set to inputs by st_init())
		device.port[i]->DIRCLR = GPIO2_MIN_BIT_bm;		 	// set min input
		device.port[i]->PIN6CTRL = (pin_mode | int_mode);	// see 13.14.14
		device.port[i]->INT0MASK = GPIO2_MIN_BIT_bm;	 	// min on INT0

		device.port[i]->DIRCLR = GPIO2_MAX_BIT_bm;		 	// set max input
		device.port[i]->PIN7CTRL = (pin_mode | int_mode);	// 13.14.14
		device.port[i]->INT1MASK = GPIO2_MAX_BIT_bm;		// max on INT1

		// set interrupt levels. Interrupts must be enabled in main()
//		device.port[i]->INTCTRL = GPIO1_INTLVL;				// see gpio.h for setting
		device.port[i]->INTCTRL = (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc);
//		device.port[i]->INTCTRL |= (PORT_INT0LVL_LO_gc|PORT_INT1LVL_LO_gc);

	}
	gpio_clear_switches();
	gpio.sw_count = 0;

	// GPIO2 - inputs and outputs port
	// (nothing here yet)

}

/*
 * ISRs - Switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_X);}
ISR(Y_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Y);}
ISR(Z_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Z);}
ISR(A_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_A);}

ISR(X_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_X);}
ISR(Y_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Y);}
ISR(Z_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Z);}
ISR(A_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_A);}

static void _switch_isr_helper(uint8_t sw_flag)
{
	if (gpio.sw_count == 0) {					// true if not in a debounce lockout
		uint8_t axis = sw_flag;					// find out what axis this is
		if (axis >= SW_OFFSET_TO_MAX) {
			axis -= SW_OFFSET_TO_MAX;
		} 
		if (cfg.a[axis].switch_mode == SW_MODE_DISABLED) {
			return;
		}
		gpio.sw_thrown = true;					// triggers the switch handler tasks
		gpio.sw_flags[sw_flag] = true;
		gpio.sw_count = SW_LOCKOUT_TICKS;		// start the debounce lockout timer

		if (cm.cycle_state == CYCLE_HOMING) {
			sig_feedhold();
		} else {
			if ((cfg.a[axis].switch_mode == SW_MODE_ENABLED_NO) || 
				(cfg.a[axis].switch_mode == SW_MODE_ENABLED_NC)) { // only fire abort if fully enabled
				sig_abort();
			}
		}
	}
}

/*
 * gpio_switch_timer_callback() - called from RTC for each RTC tick.
 *
 *	Also, once the lockout expires (gets to zero) it reads the switches
 *	sets sw_thrown to picked up by switch_handler if the switch was thrown 
 *	and remained thrown (as can happen in some homing recovery cases)
 */

inline void gpio_switch_timer_callback(void)
{
	if (gpio.sw_count) { --gpio.sw_count;}	// counts down to zero and sticks on zero
}

/*
 * gp_clear_switches() - clear all limit switches but not lockout count
 *
 * Note: Can't use memset on the flags because they are Volatile
 */

void gpio_clear_switches() 
{
	gpio.sw_thrown = false;
	for (uint8_t i=0; i < SW_SIZE; i++) {
		gpio.sw_flags[i] = false; 
	}
}

/*
 * gpio_read_switches() - read the switches into the switch flag array
 *
 *	As configured switches are active LO
 *
 *	This routine relies on SW_FLAG array being in order of 
 *	MIN_X, MIN_Y, MIN_Z, MIN_A, MAX_X, MAX_Y, MAX_Z, MAX_A
 *	and there being 2 groups of 4 flags.
 */

void gpio_read_switches()
{
	gpio_clear_switches();				// clear flags and thrown

	for (uint8_t i=0; i<4; i++) {
		if (!(device.port[i]->IN & GPIO2_MIN_BIT_bm)) { 	// min
			gpio.sw_flags[i] = true;
			gpio.sw_thrown = true;
		}
		if (!(device.port[i]->IN & GPIO2_MAX_BIT_bm)) { 	// max
			gpio.sw_flags[i + SW_OFFSET_TO_MAX] = true;
			gpio.sw_thrown = true;
		}
	}
}

/*
 * gpio_get_switch() - return TRUE if switch is thrown
 * gpio_set_switch() - diagnostic function for emulating a switch closure
 */

uint8_t gpio_get_switch(uint8_t sw_flag) { return (gpio.sw_flags[sw_flag]);}

void gpio_set_switch(uint8_t sw_flag)
{
	gpio.sw_thrown = true;
	gpio.sw_flags[sw_flag] = true;
}

/*
 * gpio_switch_handler() - main limit switch handler; called from controller loop
 */

uint8_t gpio_switch_handler(void)
{
	if (gpio.sw_thrown == false) {			// leave if no switches are thrown
		return (TG_NOOP);
	}
	gpio_clear_switches();					// reset the switches last, not before
	return (TG_OK);
}

/*
 * gpio_set_bit_on() - turn bit on
 * gpio_set_bit_off() - turn bit on
 */

void gpio_set_bit_on(uint8_t b)
{
	if (b & 0x01) { PORT_MOTOR_4.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { PORT_MOTOR_3.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { PORT_MOTOR_2.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x08) { PORT_MOTOR_1.OUTSET = GPIO1_OUT_BIT_bm;}
}

void gpio_set_bit_off(uint8_t b)
{
	if (b & 0x01) { PORT_MOTOR_4.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { PORT_MOTOR_3.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { PORT_MOTOR_2.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x08) { PORT_MOTOR_1.OUTCLR = GPIO1_OUT_BIT_bm;}
}

/*
 * gpio_write_port() - write lowest 4 bits of a byte to GPIO 1 output port
 *
 * This is a hack to hide the fact that we've scattered the encode output
 * bits all over the place becuase we have no more contiguous ports left. 
 */

void gpio_write_port(uint8_t b)
{
	gpio_port_value = b;

	if (b & 0x01) { // b0 is on MOTOR_4 (A axis)
		PORT_MOTOR_4.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		PORT_MOTOR_4.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x02) { // b1 is on MOTOR_3 (Z axis)
		PORT_MOTOR_3.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		PORT_MOTOR_3.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x04) { // b2 is on MOTOR_2 (Y axis)
		PORT_MOTOR_2.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		PORT_MOTOR_2.OUTCLR = GPIO1_OUT_BIT_bm;
	}

	if (b & 0x08) { // b3 is on MOTOR_1 (X axis)
		PORT_MOTOR_1.OUTSET = GPIO1_OUT_BIT_bm;
	} else {
		PORT_MOTOR_1.OUTCLR = GPIO1_OUT_BIT_bm;
	}
}

/*
 * gpio_toggle_port() - toggle lowest 4 bits of a byte to output port
 *
 *	Note: doesn't take transitions form bit_on / bit_off into account
 */

void gpio_toggle_port(uint8_t b)
{
	gpio_port_value ^= b;	// xor the stored port value with b
	gpio_write_port(gpio_port_value);
}

/*
 * _show_switch() - simple display routine
 */

#ifdef __dbSHOW_LIMIT_SWITCH
static void _show_switch(void);
static void _show_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown %d %d %d %d   %d %d %d %d\n"), 
		sw.flag[SW_MIN_X], sw.flag[SW_MAX_X], 
		sw.flag[SW_MIN_Y], sw.flag[SW_MAX_Y], 
		sw.flag[SW_MIN_Z], sw.flag[SW_MAX_Z], 
		sw.flag[SW_MIN_A], sw.flag[SW_MAX_A]);
}
#endif

