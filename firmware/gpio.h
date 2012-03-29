/*
 * gpio.c - geberal purpose IO bits - including limit switches, inputs, outputs
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
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
 */

#ifndef gpio_h
#define gpio_h

enum swFlags {	 // indexes into the limit switch flag array
	SW_X_MIN, SW_X_MAX,
	SW_Y_MIN, SW_Y_MAX,
	SW_Z_MIN, SW_Z_MAX,
	SW_A_MIN, SW_A_MAX,
	SW_FLAG_SIZE // last one. Used for array sizing and for loops
};

/*
 * Global Scope Functions and Data
 */

struct swStruct {						// limit and homing switch state
	volatile uint8_t thrown;			// 0=idle, 1=switch thrown
	volatile uint8_t count;				// lockout counter (debouncing)
	volatile uint8_t flag[SW_FLAG_SIZE];// min/max flag array
};
struct swStruct sw;

void sw_init(void);
uint8_t sw_handler(void);
void sw_rtc_callback(void);
void sw_clear_limit_switches(void);
void sw_read_limit_switches(void);
void sw_isr_helper(uint8_t flag);	// brought out for simulation purposes

uint8_t sw_any_thrown(void);		// return TRUE if any switch is thrown
uint8_t sw_xmin_thrown(void);		// return TRUE is X min is thrown
uint8_t sw_xmax_thrown(void);
uint8_t sw_ymin_thrown(void);
uint8_t sw_ymax_thrown(void);
uint8_t sw_zmin_thrown(void);
uint8_t sw_zmax_thrown(void);
uint8_t sw_amin_thrown(void);
uint8_t sw_amax_thrown(void);

void en_init(void);
void en_bit_on(uint8_t b);
void en_bit_off(uint8_t b);
void en_write(uint8_t b);
void en_toggle(uint8_t b);

#endif
