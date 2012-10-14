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
 *	not already taken up by steppers, serial ports, SPI or PDI programming
 *
 *	There are 2 GPIO ports:
 *
 *	  gpio1	  Located on 5x2 header next to the PDI programming plugs (on v7's)
 *			  Four (4) output bits capable of driving 3.3v or 5v logic
 *
 *			Note: On v6 and earlier boards there are also 4 inputs:
 *			  Four (4) level converted input bits capable of being driven 
 *				by 3.3v or 5v logic - connected to B0 - B3 
 *
 *	  gpio2	  Located on 9x2 header on "bottom" of board
 *			  Eight (8) non-level converted input bits
 *			  Eight (8) ground pins
 *			  Two   (2) 3.3v power pins
 *			  Inputs can be used as switch contact inputs or 
 *				3.3v input bits depending on port configuration
 *				**** These bits CANNOT be used as 5v inputs ****
 */
/* Switch Modes
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

//#include <stdio.h>						// precursor to xio.h
//#include <avr/pgmspace.h>					// precursor to xio.h
#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "system.h"							// gpio port bits are mapped here
#include "gpio.h"
#include "canonical_machine.h"
#include "xio/xio.h"						// signals

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

#define SW_LOCKOUT_TICKS 10					// the RTC ticks are ~10ms each

static void _switch_isr_helper(uint8_t sw_flag, uint8_t axis);
static uint8_t gpio_port_value;				// global for synthetic port read value

/*
 * gpio_init() - initialize homing/limit switches
 *
 *	This function assumes st_init() has been run previously to bind the ports
 *	to the device structure and to set GPIO1 and GPIO2 bit IO directions 
 *	(see system.h for all of this)
 *
 *	Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm). 
 *	v6 and earlier use internal upllups only. Internal pullups are set 
 *	regardless of board type but are extraneous for v7 boards.
 */

void gpio_init(void) 
{
//	cli();
	uint8_t j = 0;
	uint8_t int_mode;							// interrupt mode
	uint8_t pin_mode = PORT_OPC_PULLUP_gc;		// pin mode. see iox192a3.h for details
//	uint8_t pin_mode = PORT_OPC_TOTEM_gc;		// alternate pin mode for v7 boards

	// GPIO1 - output port (nothing here yet)

	// GPIO2 - switch port

	for (uint8_t i=0; i<SW_PAIRS; i++) {
		j=i + SW_OFFSET;

		// detect initial state of all switches.
		sw.detect[i] = device.port[i]->IN & SW_MIN_BIT_bm;	// evaluates to true or false
		sw.detect[j] = device.port[i]->IN & SW_MAX_BIT_bm;	// evaluates to true or false
	
		// set the switch senses and interrupt modes
		if (cfg.a[i].switch_mode == SW_MODE_DISABLED) {
			sw.sense[i] = SW_SENSE_DISABLED;
			sw.sense[j] = SW_SENSE_DISABLED;

		} else if ((cfg.a[i].switch_mode == SW_MODE_ENABLED_NO) || (cfg.a[i].switch_mode == SW_MODE_HOMING_NO)) {
			sw.sense[i] = SW_SENSE_NO;
			sw.sense[j] = SW_SENSE_NO;
			int_mode = PORT_ISC_FALLING_gc;

		} else if ((cfg.a[i].switch_mode == SW_MODE_ENABLED_NC) || (cfg.a[i].switch_mode == SW_MODE_HOMING_NC)) {
			sw.sense[i] = SW_SENSE_NC;
			sw.sense[j] = SW_SENSE_NC;
			int_mode = PORT_ISC_RISING_gc;
		}

		// setup ports input bits (previously set to inputs by st_init())
		device.port[i]->DIRCLR = SW_MIN_BIT_bm;		 	// set min input
		device.port[i]->PIN6CTRL = (pin_mode | int_mode);	// see 13.14.14
		device.port[i]->INT0MASK = SW_MIN_BIT_bm;	 	// min on INT0

		device.port[i]->DIRCLR = SW_MAX_BIT_bm;		 	// set max input
		device.port[i]->PIN7CTRL = (pin_mode | int_mode);	// 13.14.14
		device.port[i]->INT1MASK = SW_MAX_BIT_bm;		// max on INT1

		// set interrupt levels. Interrupts must be enabled in main()
		device.port[i]->INTCTRL = GPIO1_INTLVL;				// see gpio.h for setting
	}
	sw.lockout_count = 0;
	gpio_clear_switches();
//	sei();
}

/*
 * ISRs - Switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_X, X);}
ISR(Y_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Y, Y);}
ISR(Z_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Z, Z);}
ISR(A_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_A, A);}

ISR(X_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_X, X);}
ISR(Y_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Y, Y);}
ISR(Z_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Z, Z);}
ISR(A_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_A, A);}

static void _switch_isr_helper(uint8_t sw_flag, uint8_t axis)
{
	if (sw.lockout_count != 0) return;		// exit if you are in a debounce lockout
	if (cfg.a[axis].switch_mode == SW_MODE_DISABLED) return;

	gpio_read_switches();					// now read the switches for real
	if (sw.thrown == false) return;			// false alarm
	sw.lockout_count = SW_LOCKOUT_TICKS;	// start the debounce lockout timer

	if (sw.flags[sw_flag] == true) {		// process switch closure
		if (cm.cycle_state == CYCLE_HOMING) {
			sig_feedhold();					// do not reset the switch flag array
		} else {
			if ((cfg.a[axis].switch_mode == SW_MODE_ENABLED_NO) || 
				(cfg.a[axis].switch_mode == SW_MODE_ENABLED_NC)) { // only fire abort if fully enabled
				sig_abort();				// do not reset the switch flag array
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
	if (sw.lockout_count) { --sw.lockout_count;}	// counts down to zero and sticks on zero
}

/*
 * gp_clear_switches() - clear all limit switches but not lockout count
 *
 * Note: Can't use memset on the flags because they are Volatile
 */

void gpio_clear_switches() 
{
	sw.thrown = false;
	for (uint8_t i=0; i < SW_ARRAY_SIZE; i++) {
		sw.flags[i] = false; 
	}
}

/*
 * gpio_read_switches() - read the switches into the switch flag array
 *
 *	Read the switch states into the switch array as an array of true/false bytes
 *	Read Normally Open switches as active LO, Normally Closed as active LO.
 *	Ignore switches that are disabled.
 *	This function does not care about cycle (e.g. homing), it just reads the switches
 *
 *	This routine relies on SW_FLAG array being in order of:
 *		MIN_X, MIN_Y, MIN_Z, MIN_A, MAX_X, MAX_Y, MAX_Z, MAX_A
 *	and there being 2 groups of 4 flags.
 */

void gpio_read_switches()
{
	gpio_clear_switches();				// clear flags and thrown bit

	for (uint8_t i=0; i<4; i++) {
		if ((cfg.a[i].switch_mode == SW_MODE_ENABLED_NO) || (cfg.a[i].switch_mode == SW_MODE_HOMING_NO)) {
			if ((device.port[i]->IN & SW_MIN_BIT_bm) == 0) { 	// NO switch MIN bit
				sw.flags[i] = true;
				sw.thrown = true;
			}
			if ((device.port[i]->IN & SW_MAX_BIT_bm) == 0) { 	// NO switch MAX bit
				sw.flags[i+SW_OFFSET] = true;
				sw.thrown = true;
			}			
		}
		if ((cfg.a[i].switch_mode == SW_MODE_ENABLED_NC) || (cfg.a[i].switch_mode == SW_MODE_HOMING_NC)) {
			if ((device.port[i]->IN & SW_MIN_BIT_bm) == 1) { 	// NC switch MIN bit
				sw.flags[i] = true;
				sw.thrown = true;
			}
			if ((device.port[i]->IN & SW_MAX_BIT_bm) == 1) { 	// NC switch MAX bit
				sw.flags[i+SW_OFFSET] = true;
				sw.thrown = true;
			}			
		}
	}
}

/*
 * gpio_get_switch() - return TRUE if switch is thrown
 * gpio_set_switch() - diagnostic function for emulating a switch closure
 */

uint8_t gpio_get_switch(uint8_t sw_flag) 
{
	return (sw.flags[sw_flag]);
}

void gpio_set_switch(uint8_t sw_flag)
{
	sw.thrown = true;
	sw.flags[sw_flag] = true;
}

/*
 * gpio_switch_handler() - main limit switch handler; called from controller loop
 */

uint8_t gpio_switch_handler(void)
{
	if (sw.thrown == false) {			// leave if no switches are thrown
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


//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_GPIO

void gpio_unit_tests()
{
	_switch_isr_helper(SW_MIN_X, X);
}

#endif // __UNIT_TEST_GPIO

