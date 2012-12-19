/*
 * gpio.c - geberal purpose IO bits - including limit switches, inputs, outputs
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2012 Alden S. Hart Jr.
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

#ifndef gpio_h
#define gpio_h

/*
 * Interrupt levels and vectors - The vectors are hard-wired to xmega ports
 * If you change axis port assignments you need to chanage these, too.
 */
// Interrupt level: pick one:
//#define GPIO1_INTLVL (PORT_INT0LVL_HI_gc|PORT_INT1LVL_HI_gc)	// can't be hi
#define GPIO1_INTLVL (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc)
//#define GPIO1_INTLVL (PORT_INT0LVL_LO_gc|PORT_INT1LVL_LO_gc)	// shouldn;t be low

// port assignments for vectors
#define X_MIN_ISR_vect PORTA_INT0_vect	// these must line up with the SWITCH assignments in system.h
#define Y_MIN_ISR_vect PORTD_INT0_vect
#define Z_MIN_ISR_vect PORTE_INT0_vect
#define A_MIN_ISR_vect PORTF_INT0_vect
#define X_MAX_ISR_vect PORTA_INT1_vect
#define Y_MAX_ISR_vect PORTD_INT1_vect
#define Z_MAX_ISR_vect PORTE_INT1_vect
#define A_MAX_ISR_vect PORTF_INT1_vect

// macros for finding the index into the switch table give the axis number
#define MIN_SWITCH(axis) (axis*2)
#define MAX_SWITCH(axis) (axis*2+1)

/*
 * Global Scope Definitions, Functions and Data
 */

enum swNums {	 			// indexes into switch arrays
	SW_MIN_X = 0,
	SW_MAX_X,
	SW_MIN_Y,
	SW_MAX_Y,
	SW_MIN_Z, 
	SW_MAX_Z,
	SW_MIN_A,
	SW_MAX_A,
	NUM_SWITCHES 			// must be last one. Used for array sizing and for loops
};
#define SW_OFFSET SW_MAX_X	// offset between MIN and MAX switches
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)

#define SW_DISABLED -1
#define SW_OPEN 	 0
#define SW_CLOSED	 1

// switch mode settings
#define SW_HOMING 0x01
#define SW_LIMIT 0x02
#define SW_MODE_DISABLED 0			// disabled for all operations
#define SW_MODE_HOMING SW_HOMING	// enable switch for homing only
#define SW_MODE_LIMIT SW_LIMIT		// enable switch for limits only
#define SW_MODE_HOMING_LIMIT (SW_HOMING | SW_LIMIT)	// homing and limits
#define SW_MODE_MAX_VALUE SW_MODE_LIMIT

enum swType {
	SW_TYPE_NORMALLY_OPEN = 0,
	SW_TYPE_NORMALLY_CLOSED
};

struct swStruct {						// switch state
	uint8_t switch_type;				// 0=NO, 1=NC - applies to all switches
	volatile uint8_t thrown;			// 1=thrown (Note 1)
	volatile uint8_t limit_thrown;		// 1= limit switch thrown - do a lockout
	volatile uint8_t lockout_count;		// switch lockout counter (debouncing)
	volatile uint8_t flag[NUM_SWITCHES];// switch flag array
	volatile uint8_t mode[NUM_SWITCHES];// 0=disabled, 1=homing, 2=homing+limit, 3=limit
};
struct swStruct sw;

// Note 1: The term "thrown" is used because switches could be normally-open 
//		   or normally-closed. "Thrown" means activated or hit.

void gpio_init(void);
void gpio_out_map(double hw_version);
void gpio_switch_timer_callback(void);
void gpio_clear_switches(void);
void gpio_reset_lockout(void);
void gpio_set_switch(uint8_t sw_num);
uint8_t gpio_read_switch(uint8_t sw_num);
uint8_t gpio_get_switch(uint8_t sw_num);
uint8_t gpio_get_switch_mode(uint8_t sw_num);
//void gpio_read_switches(void);
//uint8_t gpio_switch_callback(void);

void gpio_led_on(uint8_t led);
void gpio_led_off(uint8_t led);
void gpio_set_bit_on(uint8_t b);
void gpio_set_bit_off(uint8_t b);
//void gpio_write_port(uint8_t b);
//void gpio_toggle_port(uint8_t b);

//void sw_show_switch(void);

/* unit test setup */

//#define __UNIT_TEST_GPIO				// uncomment to enable GPIO unit tests
#ifdef __UNIT_TEST_GPIO
void gpio_unit_tests(void);
#define	GPIO_UNITS gpio_unit_tests();
#else
#define	GPIO_UNITS
#endif // __UNIT_TEST_GPIO

#endif
