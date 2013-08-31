/*
 * gpio.c - general purpose IO bits - including limit switches, inputs, outputs
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 *	This GPIO file is where all parallel port bits are managed that are 
 *	not already taken up by steppers, serial ports, SPI or PDI programming
 *
 *	There are 2 GPIO ports:
 *
 *	  gpio1	  Located on 5x2 header next to the PDI programming plugs (on v7's)
 *				Four (4) output bits capable of driving 3.3v or 5v logic
 *
 *			  Note: On v6 and earlier boards there are also 4 inputs:
 *				Four (4) level converted input bits capable of being driven 
 *				by 3.3v or 5v logic - connected to B0 - B3 (now used for SPI)
 *
 *	  gpio2	  Located on 9x2 header on "bottom" edge of the board
 *				Eight (8) non-level converted input bits
 *				Eight (8) ground pins - one each "under" each input pin
 *				Two   (2) 3.3v power pins (on left edge of connector)
 *				Inputs can be used as switch contact inputs or 
 *					3.3v input bits depending on port configuration
 *					**** These bits CANNOT be used as 5v inputs ****
 */
/* Switch Modes
 *
 *	The switches are considered to be homing switches when machine_state is
 *	MACHINE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
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
#include "xio/xio.h"						// signals

/*
 * variables and settings 
 */
											// timer for debouncing switches
#define SW_LOCKOUT_TICKS 25					// 25=250ms. RTC ticks are ~10ms each
#define SW_DEGLITCH_TICKS 3					// 3=30ms

static void _isr_helper(uint8_t sw_num);

/*
 * gpio_init() - initialize homing/limit switches
 *
 *	This function assumes sys_init() and st_init() have been run previously to 
 *	bind the ports and set bit IO directions, repsectively. See system.h for details
 */
/* Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm). 
 *	v6 and earlier use internal pullups only. Internal pullups are set 
 *	regardless of board type but are extraneous for v7 boards.
 */
#define PIN_MODE PORT_OPC_PULLUP_gc				// pin mode. see iox192a3.h for details
//#define PIN_MODE PORT_OPC_TOTEM_gc			// alternate pin mode for v7 boards

void gpio_init(void)
{
	for (uint8_t i=0; i<NUM_SWITCH_PAIRS; i++) {
		// old code from when switches fired on one edge or the other:
		//	uint8_t int_mode = (sw.switch_type == SW_TYPE_NORMALLY_OPEN) ? PORT_ISC_FALLING_gc : PORT_ISC_RISING_gc;

		// setup input bits and interrupts (previously set to inputs by st_init())
		if (sw.mode[MIN_SWITCH(i)] != SW_MODE_DISABLED) {
			device.sw_port[i]->DIRCLR = SW_MIN_BIT_bm;		 	// set min input - see 13.14.14
			device.sw_port[i]->PIN6CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
			device.sw_port[i]->INT0MASK = SW_MIN_BIT_bm;	 	// interrupt on min switch
		} else {
			device.sw_port[i]->INT0MASK = 0;	 				// disable interrupt
		}
		if (sw.mode[MAX_SWITCH(i)] != SW_MODE_DISABLED) {
			device.sw_port[i]->DIRCLR = SW_MAX_BIT_bm;		 	// set max input - see 13.14.14
			device.sw_port[i]->PIN7CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
			device.sw_port[i]->INT1MASK = SW_MAX_BIT_bm;		// max on INT1
		} else {
			device.sw_port[i]->INT1MASK = 0;
		}
		// set interrupt levels. Interrupts must be enabled in main()
		device.sw_port[i]->INTCTRL = GPIO1_INTLVL;				// see gpio.h for setting
	}
	gpio_reset_switches();
}

/*
 * Switch closure processing routines
 *
 * ISRs 				- switch interrupt handler vectors
 * _isr_helper()		- common code for all switch ISRs
 * gpio_rtc_callback()	- called from RTC for each RTC tick.
 *
 *	These functions interact with each other to process switch closures and firing.
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */

ISR(X_MIN_ISR_vect)	{ _isr_helper(SW_MIN_X);}
ISR(Y_MIN_ISR_vect)	{ _isr_helper(SW_MIN_Y);}
ISR(Z_MIN_ISR_vect)	{ _isr_helper(SW_MIN_Z);}
ISR(A_MIN_ISR_vect)	{ _isr_helper(SW_MIN_A);}
ISR(X_MAX_ISR_vect)	{ _isr_helper(SW_MAX_X);}
ISR(Y_MAX_ISR_vect)	{ _isr_helper(SW_MAX_Y);}
ISR(Z_MAX_ISR_vect)	{ _isr_helper(SW_MAX_Z);}
ISR(A_MAX_ISR_vect)	{ _isr_helper(SW_MAX_A);}

static void _isr_helper(uint8_t sw_num)
{
	if (sw.mode[sw_num] == SW_MODE_DISABLED) return;	// this is never supposed to happen
	if (sw.state[sw_num] == SW_LOCKOUT) return;			// exit if switch is in lockout
	sw.state[sw_num] = SW_DEGLITCHING;					// either transitions state from IDLE or overwrites it
	sw.count[sw_num] = -SW_DEGLITCH_TICKS;				// reset deglitch count regardless of entry state
}

void gpio_rtc_callback(void)
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) { 
		if (sw.state[i] == SW_IDLE) continue;
		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.state[i] = SW_IDLE; continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.state[i] = SW_LOCKOUT;
//			sw_show_switch();							// only called if __DEBUG enabled
			if (cm.cycle_state == CYCLE_HOMING) {		// regardless of switch type
				cm_request_feedhold();
			} else if (sw.mode[i] & SW_LIMIT) {			// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
}

/*
 * gpio_get_switch_mode() 	- return switch mode setting
 * gpio_get_limit_thrown()  - return true if a limit was tripped
 * gpio_get_sw_num()  		- return switch number most recently thrown
 */

uint8_t gpio_get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
uint8_t gpio_get_limit_thrown(void) { return(sw.limit_flag);}
uint8_t gpio_get_sw_thrown(void) { return(sw.sw_num_thrown);}

/*
 * gpio_reset_switches() - reset all switches and reset limit flag
 */

void gpio_reset_switches() 
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		sw.state[i] = SW_IDLE;
//		sw.count[i] = -SW_DEGLITCH_TICKS;
	}
	sw.limit_flag = false;
}

/*
 * gpio_read_switch() - read a switch directly with no interrupts or deglitching
 */
uint8_t gpio_read_switch(uint8_t sw_num)
{
	if ((sw_num < 0) || (sw_num >= NUM_SWITCHES)) return (SW_DISABLED);

	uint8_t read = 0;
	switch (sw_num) {
		case SW_MIN_X: { read = device.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_X: { read = device.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Y: { read = device.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Y: { read = device.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Z: { read = device.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Z: { read = device.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_A: { read = device.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_A: { read = device.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm; break;}
	}
	if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
		return ((read == 0) ? SW_CLOSED : SW_OPEN);		// confusing. An NO switch drives the pin LO when thrown
	} else {
		return ((read != 0) ? SW_CLOSED : SW_OPEN);
	}
}

/*
 * gpio_led_on() - turn led on - assumes TinyG LED mapping
 * gpio_led_off() - turn led on - assumes TinyG LED mapping
 * gpio_led_toggle()
 */

void gpio_led_on(uint8_t led)
{
//	if (led == 0) return (gpio_set_bit_on(0x08));
//	if (led == 1) return (gpio_set_bit_on(0x04));
//	if (led == 2) return (gpio_set_bit_on(0x02));
//	if (led == 3) return (gpio_set_bit_on(0x01));

	if (led == 0) gpio_set_bit_on(0x08); else 
	if (led == 1) gpio_set_bit_on(0x04); else 
	if (led == 2) gpio_set_bit_on(0x02); else 
	if (led == 3) gpio_set_bit_on(0x01);
}

void gpio_led_off(uint8_t led)
{
//	if (led == 0) return (gpio_set_bit_off(0x08));
//	if (led == 1) return (gpio_set_bit_off(0x04));
//	if (led == 2) return (gpio_set_bit_off(0x02));
//	if (led == 3) return (gpio_set_bit_off(0x01));

	if (led == 0) gpio_set_bit_off(0x08); else 
	if (led == 1) gpio_set_bit_off(0x04); else 
	if (led == 2) gpio_set_bit_off(0x02); else 
	if (led == 3) gpio_set_bit_off(0x01);
}

void gpio_led_toggle(uint8_t led)
{
	if (led == 0) {
		if (gpio_read_bit(0x08)) {
			gpio_set_bit_off(0x08);
		} else {
			gpio_set_bit_on(0x08);
		}
	} else if (led == 1) {
		if (gpio_read_bit(0x04)) {
			gpio_set_bit_off(0x04);
		} else {
			gpio_set_bit_on(0x04);
		}
	} else if (led == 2) {
		if (gpio_read_bit(0x02)) {
			gpio_set_bit_off(0x02);
		} else {
			gpio_set_bit_on(0x02);
		}
	} else if (led == 3) {
		if (gpio_read_bit(0x08)) {
			gpio_set_bit_off(0x08);
		} else {
			gpio_set_bit_on(0x08);
		}
	}
}

/*
 * gpio_read_bit() - return true if bit is on, false if off
 * gpio_set_bit_on() - turn bit on
 * gpio_set_bit_off() - turn bit on
 *
 *	These functions have an inner remap depending on what hardware is running
 */

uint8_t gpio_read_bit(uint8_t b)
{
	if (b & 0x08) { return (device.out_port[0]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x04) { return (device.out_port[1]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x02) { return (device.out_port[2]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x01) { return (device.out_port[3]->IN & GPIO1_OUT_BIT_bm); }
	return (0);
}

void gpio_set_bit_on(uint8_t b)
{
	if (b & 0x08) { device.out_port[0]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { device.out_port[1]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { device.out_port[2]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { device.out_port[3]->OUTSET = GPIO1_OUT_BIT_bm; }
}

void gpio_set_bit_off(uint8_t b)
{
	if (b & 0x08) { device.out_port[0]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { device.out_port[1]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { device.out_port[2]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { device.out_port[3]->OUTCLR = GPIO1_OUT_BIT_bm; }
}

// DEPRECATED CODE THAT MIGHT STILL BE USEFUL

/*
 * gpio_write_port() - write lowest 4 bits of a byte to GPIO 1 output port
 *
 * This is a hack to hide the fact that we've scattered the output bits all
 * over the place because we have no more contiguous ports left!
 */
/*
!!!!! This needs a complete rewrite to use out_port bindings if it's to be used again !!!!
void gpio_write_port(uint8_t b)
{
	gpio_port_value = b;

	// b0 is on OUT_4 (A axis)
	if (b & 0x01)
		PORT_OUT_A.OUTSET = GPIO1_OUT_BIT_bm;
	else
		PORT_OUT_A.OUTCLR = GPIO1_OUT_BIT_bm;

	// b1 is on OUT_3 (Z axis)
	if (b & 0x02)
		PORT_OUT_Z.OUTSET = GPIO1_OUT_BIT_bm;
	else
		PORT_OUT_Z.OUTCLR = GPIO1_OUT_BIT_bm;

	// b2 is on OUT_2 (Y axis)
	if (b & 0x04)
		PORT_OUT_Y.OUTSET = GPIO1_OUT_BIT_bm;
	else
		PORT_OUT_Y.OUTCLR = GPIO1_OUT_BIT_bm;

	// b3 is on OUT_1 (X axis)
	if (b & 0x08)
		PORT_OUT_X.OUTSET = GPIO1_OUT_BIT_bm;
	else
		PORT_OUT_X.OUTCLR = GPIO1_OUT_BIT_bm;
}
*/
/*
 * gpio_toggle_port() - toggle lowest 4 bits of a byte to output port
 *
 *	Note: doesn't take transitions from bit_on / bit_off into account
 */
/*
void gpio_toggle_port(uint8_t b)
{
	gpio_port_value ^= b;	// xor the stored port value with b
	gpio_write_port(gpio_port_value);
}
*/
/*
 * _show_switch() - simple display routine
 */
#ifdef __DEBUG
void sw_show_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown Xmin %d Xmax %d  Ymin %d Ymax %d  \
		Zmin %d Zmax %d Amin %d Amax %d\n"), 
		sw.state[SW_MIN_X], sw.state[SW_MAX_X],
		sw.state[SW_MIN_Y], sw.state[SW_MAX_Y],
		sw.state[SW_MIN_Z], sw.state[SW_MAX_Z],
		sw.state[SW_MIN_A], sw.state[SW_MAX_A]);
}
#else
void sw_show_switch(void) {}
#endif

//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_GPIO

void gpio_unit_tests()
{
//	_isr_helper(SW_MIN_X, X);
	while (true) {
		gpio_led_toggle(1);
	}
}

#endif // __UNIT_TEST_GPIO
#endif
