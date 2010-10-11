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

struct lsState {
	uint8_t	min[AXES];	// 0=open, 1=hit
	uint8_t	max[AXES];
};

struct lsState ls;

void ls_init(void);
void ls_clear_limit_switches(void);

#endif
