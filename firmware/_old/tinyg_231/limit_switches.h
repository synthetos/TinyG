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

enum lsFlags {	 // indexes into the limit switch flag array
	LS_X_MIN, LS_X_MAX,
	LS_Y_MIN, LS_Y_MAX,
	LS_Z_MIN, LS_Z_MAX,
	LS_A_MIN, LS_A_MAX,
	LS_FLAG_SIZE // last one. Used for array sizing and for loops
};

/*
 * Global Scope Functions and Data
 */

struct lsStruct {
	volatile uint8_t thrown;			// 0=idle, 1=switch thrown
	volatile uint8_t count;				// lockout counter (debouncing)
	volatile uint8_t flag[LS_FLAG_SIZE];// min/max flag array
};
struct lsStruct ls;

void ls_init(void);
uint8_t ls_handler(void);
void ls_rtc_callback(void);
void ls_clear_limit_switches(void);
void ls_read_limit_switches(void);
void ls_isr_helper(uint8_t flag);	// brought out for simulation purposes

uint8_t ls_any_thrown(void);		// return TRUE if any switch is thrown
uint8_t ls_xmin_thrown(void);		// return TRUE is X min is thrown
uint8_t ls_xmax_thrown(void);
uint8_t ls_ymin_thrown(void);
uint8_t ls_ymax_thrown(void);
uint8_t ls_zmin_thrown(void);
uint8_t ls_zmax_thrown(void);
uint8_t ls_amin_thrown(void);
uint8_t ls_amax_thrown(void);

#endif
