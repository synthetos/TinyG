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

//#include <stdio.h>						// precursor to xio.h
#include <avr/pgmspace.h>					// precursor to xio.h
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

static uint8_t gpio_port_value;				// global for synthetic port read value
static void _init_helper(uint8_t swit, uint8_t port);
static void _isr_helper(uint8_t sw_num);

/*
 * gpio_init() - initialize homing/limit switches
 *
 *	This function assumes st_init() has been run previously to bind the ports
 *	to the device structure and to set GPIO1 and GPIO2 bit IO directions 
 *	(see system.h for the mappings and stepper.c for the inits)
 *
 *	Note: the reason this is not a simple for() loop is that the motor port 
 *	and switch port assignments do not line up. Damn.
 */
/* Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm). 
 *	v6 and earlier use internal upllups only. Internal pullups are set 
 *	regardless of board type but are extraneous for v7 boards.
 */
#define PIN_MODE PORT_OPC_PULLUP_gc				// pin mode. see iox192a3.h for details
//#define PIN_MODE PORT_OPC_TOTEM_gc			// alternate pin mode for v7 boards

void gpio_init(void)
{
	// Assumes all port directions previously set to 0x3F in st_init()
	_init_helper(SWITCH_X, MOTOR_1);
	_init_helper(SWITCH_Y, MOTOR_2);
	_init_helper(SWITCH_Z, MOTOR_3);
	_init_helper(SWITCH_A, MOTOR_4);
	gpio_clear_switches();
	gpio_reset_lockout();
}

static void _init_helper(uint8_t swit, uint8_t port)
{
// old code from when switches fired on one edge or the other:
//	uint8_t int_mode = (sw.switch_type == SW_TYPE_NORMALLY_OPEN) ? PORT_ISC_FALLING_gc : PORT_ISC_RISING_gc;

	// setup port input bits and interrupts (previously set to inputs by st_init())
	if (sw.mode[MIN_SWITCH(swit)] != SW_MODE_DISABLED) {
		device.port[port]->DIRCLR = SW_MIN_BIT_bm;		 	// set min input - see 13.14.14
		device.port[port]->PIN6CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
		device.port[port]->INT0MASK = SW_MIN_BIT_bm;	 	// interrupt on min switch
	} else {
		device.port[port]->INT0MASK = 0;	 				// disable interrupt
	}
	if (sw.mode[MAX_SWITCH(swit)] != SW_MODE_DISABLED) {
		device.port[port]->DIRCLR = SW_MAX_BIT_bm;		 	// set max input - see 13.14.14
		device.port[port]->PIN7CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
		device.port[port]->INT1MASK = SW_MAX_BIT_bm;		// max on INT1
	} else {
		device.port[port]->INT1MASK = 0;
	}
	// set interrupt levels. Interrupts must be enabled in main()
	device.port[port]->INTCTRL = GPIO1_INTLVL;				// see gpio.h for setting
}

/*
 * ISRs - Switch interrupt handler routine and vectors
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
	if (sw.lockout_count != 0) return;					// exit if you are in a debounce lockout

	if (sw.mode[sw_num] == SW_MODE_DISABLED) return;	// this is not supposed to happen

	sw.lockout_count = SW_LOCKOUT_TICKS;				// start the debounce lockout down-counter
	sw.flag[sw_num] = true;								// set the flag for this switch
	sw.thrown = true;									// triggers the switch handler tasks

	if (cm.cycle_state == CYCLE_HOMING) { 				// initiate a feedhold if in homing cycle...
		sig_feedhold(); 								// ...regardless of homing or limit switch type
		return;											
	}
	// must be a limit switch, so fire it.
	sw.limit_thrown = true;								// triggers an emergency shutdown
}

/*
 * gpio_switch_timer_callback() - called from RTC for each RTC tick.
 * gpio_reset_lockout() 		- end any debounce lockout that may be in progress 
 */
void gpio_reset_lockout() 
{
    sw.lockout_count = 0;
}

void gpio_switch_timer_callback(void)
{
	if (sw.lockout_count != 0) { --sw.lockout_count;}	// counts down to zero and sticks on zero
}

/*
 * gpio_get_switch() 	  - read the flag array and return TRUE if indicated switch is thrown
 * gpio_get_switch_mode() - return switch setting for sw_num
 * gpio_set_switch() 	  - diagnostic function for emulating a switch closure
 * gpio_read_switch()	  - read a switch directly with no interrupts or debouncing
 * gpio_clear_switches() 		- clear all limit switches but not lockout count
 */
uint8_t gpio_get_switch(uint8_t sw_num) { return (sw.flag[sw_num]);}
uint8_t gpio_get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
void gpio_set_switch(uint8_t sw_num) { sw.thrown = true; sw.flag[sw_num] = true;}

uint8_t gpio_read_switch(uint8_t sw_num)
{
	if ((sw_num < 0) || (sw_num >= NUM_SWITCHES)) return (SW_DISABLED);

	uint8_t read = 0;
	switch (sw_num) {
		case SW_MIN_X: { read = device.port[SWITCH_X]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_X: { read = device.port[SWITCH_X]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Y: { read = device.port[SWITCH_Y]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Y: { read = device.port[SWITCH_Y]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Z: { read = device.port[SWITCH_Z]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Z: { read = device.port[SWITCH_Z]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_A: { read = device.port[SWITCH_A]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_A: { read = device.port[SWITCH_A]->IN & SW_MAX_BIT_bm; break;}
	}
	if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
		return ((read == 0) ? SW_CLOSED : SW_OPEN);		// confusing. An NO switch drives the pin LO when thrown
	} else {
		return ((read != 0) ? SW_CLOSED : SW_OPEN);
	}
}

void gpio_clear_switches() 
{
	sw.thrown = false;	//  Can't use memset on the flag array because it is Volatile
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		sw.flag[i] = false; 
	}
}

/*
 * gpio_read_switches() - read the switches into the switch flag array
 *
 *	Read the switch states into the switch array as an array of true/false bytes
 *	Read Normally Open switches as active LO, Normally Closed as active HI.
 *	Ignore switches that are disabled.
 *	This function does not care about cycles (e.g. homing), it just reads the switches.
 *	This function is made ugly by the fact that the motor ports and the switch ports 
 *	don't line up. Otherwise it would be possible to run a simple for() loop
 */
/*
void gpio_read_switches()
{
	uint8_t fix[NUM_SWITCHES];
	fix[0] = 0x01 & (device.port[SWITCH_X]->IN >> SW_MIN_BIT_bp);
	fix[1] = 0x01 & (device.port[SWITCH_X]->IN >> SW_MAX_BIT_bp);
	fix[2] = 0x01 & (device.port[SWITCH_Y]->IN >> SW_MIN_BIT_bp);
	fix[3] = 0x01 & (device.port[SWITCH_Y]->IN >> SW_MAX_BIT_bp);
	fix[4] = 0x01 & (device.port[SWITCH_Z]->IN >> SW_MIN_BIT_bp);
	fix[5] = 0x01 & (device.port[SWITCH_Z]->IN >> SW_MAX_BIT_bp);
	fix[6] = 0x01 & (device.port[SWITCH_A]->IN >> SW_MIN_BIT_bp);
	fix[7] = 0x01 & (device.port[SWITCH_A]->IN >> SW_MAX_BIT_bp);

	// interpret them as NO or NC closures
	gpio_clear_switches();						// clear flags and thrown bit
	for (uint8_t i=0; i<NUM_SWITCHES; i++) {
		if (sw.mode[i] == SW_MODE_DISABLED) { continue;}
		if (((sw.switch_type == SW_TYPE_NORMALLY_OPEN) && (fix[i] == 0)) ||
			((sw.switch_type == SW_TYPE_NORMALLY_CLOSED) && (fix[i] == 1))) {
			sw.flag[i] = true;
			sw.thrown = true;
		}
	}
}
*/

/*
 * gpio_led_on() - turn led on - assumes TinyG LED mapping
 * gpio_led_off() - turn led on - assumes TinyG LED mapping
 */

void gpio_led_on(uint8_t led)
{
	if (led == 0) return (gpio_set_bit_on(0x08));
	if (led == 1) return (gpio_set_bit_on(0x04));
	if (led == 2) return (gpio_set_bit_on(0x02));
	if (led == 3) return (gpio_set_bit_on(0x01));
}

void gpio_led_off(uint8_t led)
{
	if (led == 0) return (gpio_set_bit_off(0x08));
	if (led == 1) return (gpio_set_bit_off(0x04));
	if (led == 2) return (gpio_set_bit_off(0x02));
	if (led == 3) return (gpio_set_bit_off(0x01));
}

/*
 * gpio_set_bit_on() - turn bit on
 * gpio_set_bit_off() - turn bit on
 */

void gpio_set_bit_on(uint8_t b)
{
	if (b & 0x08) { PORT_OUT_X.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { PORT_OUT_Y.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { PORT_OUT_Z.OUTSET = GPIO1_OUT_BIT_bm;}
	if (b & 0x01) { PORT_OUT_A.OUTSET = GPIO1_OUT_BIT_bm;}
}

void gpio_set_bit_off(uint8_t b)
{
	if (b & 0x08) { PORT_OUT_X.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x04) { PORT_OUT_Y.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x02) { PORT_OUT_Z.OUTCLR = GPIO1_OUT_BIT_bm;}
	if (b & 0x01) { PORT_OUT_A.OUTCLR = GPIO1_OUT_BIT_bm;}
}

/*
 * gpio_write_port() - write lowest 4 bits of a byte to GPIO 1 output port
 *
 * This is a hack to hide the fact that we've scattered the output bits all
 * over the place because we have no more contiguous ports left!
 */
/*
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
	fprintf_P(stderr, PSTR("Limit Switch Thrown %d %d  %d %d   %d %d  %d %d\n"), 
		sw.flag[MIN_SWITCH(X)], sw.flag[MAX_SWITCH(X)], 
		sw.flag[MIN_SWITCH(Y)], sw.flag[MAX_SWITCH(Y)], 
		sw.flag[MIN_SWITCH(Z)], sw.flag[MAX_SWITCH(Z)], 
		sw.flag[MIN_SWITCH(A)], sw.flag[MAX_SWITCH(A)]);
}
#endif

//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_GPIO

void gpio_unit_tests()
{
	_isr_helper(SW_MIN_X, X);
}

#endif // __UNIT_TEST_GPIO

