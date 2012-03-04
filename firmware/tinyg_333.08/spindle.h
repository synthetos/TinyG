/*
 * spindle.h - spindle driver
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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

#ifndef spindle_h
#define spindle_h 

//#include <avr/io.h>

/*
 * Global Scope Functions
 */

/* Note: See hardware.h for spindle port assignments and bit positions */

void cm_spindle_init();
uint8_t cm_spindle_control(uint8_t spindle_mode);
void cm_exec_spindle_control(uint8_t spindle_mode);
uint8_t cm_set_spindle_speed(double speed);
void cm_exec_spindle_speed(double speed);

#endif
