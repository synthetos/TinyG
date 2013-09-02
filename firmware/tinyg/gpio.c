/*
 * gpio.c - general purpose IO bits
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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

#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "system.h"							// gpio port bits are mapped here
//#include "switch.h"
#include "gpio.h"
#include "canonical_machine.h"
#include "xio/xio.h"						// signals


//======================== Parallel IO Functions ===============================

/*
 * IndicatorLed_set() 	- fake out for IndicatorLed.set() until we get Motate running
 * IndicatorLed_clear() - fake out for IndicatorLed.clear() until we get Motate running
 * IndicatorLed_toggle()- fake out for IndicatorLed.toggle() until we get Motate running
 */

void IndicatorLed_set()
{
	gpio_led_on(INDICATOR_LED);
	cs.led_state = 1;
}

void IndicatorLed_clear()
{
	gpio_led_off(INDICATOR_LED);
	cs.led_state = 0;
}

void IndicatorLed_toggle()
{
	if (cs.led_state == 0) {
		gpio_led_on(INDICATOR_LED);
		cs.led_state = 1;
	} else {
		gpio_led_off(INDICATOR_LED);
		cs.led_state = 0;
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
