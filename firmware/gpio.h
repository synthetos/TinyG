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
//#define GPIO1_INTLVL (PORT_INT0LVL_HI_gc|PORT_INT1LVL_HI_gc)
#define GPIO1_INTLVL (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc)
//#define GPIO1_INTLVL (PORT_INT0LVL_LO_gc|PORT_INT1LVL_LO_gc)

// port assignments for vectors
#define X_MIN_ISR_vect PORTA_INT0_vect
#define Y_MIN_ISR_vect PORTF_INT0_vect
#define Z_MIN_ISR_vect PORTE_INT0_vect
#define A_MIN_ISR_vect PORTD_INT0_vect

#define X_MAX_ISR_vect PORTA_INT1_vect
#define Y_MAX_ISR_vect PORTF_INT1_vect
#define Z_MAX_ISR_vect PORTE_INT1_vect
#define A_MAX_ISR_vect PORTD_INT1_vect

/*
 * Global Scope Definitions, Functions and Data
 */

enum swFlags {	 			// indexes into sw_flag array
	SW_MIN_X = 0,			// this group corresponds to XYZA values
	SW_MIN_Y, 
	SW_MIN_Z, 
	SW_MIN_A, 

	SW_MAX_X,				// this group corresponds to XYZA + SW_MAX_OFFSET
	SW_MAX_Y,
	SW_MAX_Z,
	SW_MAX_A,
	SW_SIZE 				// last one. Used for array sizing and for loops
};
#define SW_OFFSET_TO_MAX 4

enum swMode {				// limit switch operation modes
	SW_MODE_DISABLED = 0,	// disabled for all operations
	SW_MODE_HOMING_NO,		// enable NO switch for homing only
	SW_MODE_ENABLED_NO,		// enable NO switch for homing and limits
	SW_MODE_HOMING_NC,		// enable NC switch for homing only
	SW_MODE_ENABLED_NC		// enable NC switch for homing and limits
};

struct gpioStruct {							// switch state
	volatile uint8_t sw_thrown;				// 1=thrown (Note 1)
	volatile uint8_t sw_count;				// lockout counter (debouncing)
	volatile uint8_t sw_flags[SW_SIZE];		// switch flag array
};
struct gpioStruct gpio;

// Note 1: The term "thrown" is used becuase switches could be normally-open 
//		   or normally-closed. "Thrown" means activated or hit.

void gpio_init(void);
void gpio_clear_switches(void);
void gpio_read_switches(void);
void gpio_set_switch(uint8_t sw_flag);
uint8_t gpio_get_switch(uint8_t sw_flag);		// test specific switch by arg
uint8_t gpio_switch_handler(void);
void gpio_switch_timer_callback(void);

void gpio_set_bit_on(uint8_t b);
void gpio_set_bit_off(uint8_t b);
void gpio_write_port(uint8_t b);
void gpio_toggle_port(uint8_t b);

#endif
