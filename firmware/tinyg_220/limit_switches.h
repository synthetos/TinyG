/*
 * limit_switches.t - limit switch interfaces
 * Part of TinyG project
 * Copyright (c) 2010 Alden S Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef limit_switches_h
#define limit_switches_h

/*
 * Global Scope Functions and Data
 */

struct lsStruct {
//	volatile uint8_t state;
	volatile uint8_t count;
	volatile uint8_t x_min;	// 0=open, 1=hit
	volatile uint8_t x_max;
	volatile uint8_t y_min;
	volatile uint8_t y_max;
	volatile uint8_t z_min;
	volatile uint8_t z_max;
	volatile uint8_t a_min;
	volatile uint8_t a_max;
};
struct lsStruct ls;

void ls_init(void);
void ls_clear_limit_switches(void);

#endif
