/*
 * spindle.h - spindle driver
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef spindle_h
#define spindle_h 

#include <avr/io.h>

/*
 * Global Scope Functions
 */

/* Note: See hardware.h for spindle port assignments and bit positions */

void sp_init();
void sp_spindle_run(int direction, uint32_t rpm);
void sp_spindle_stop();

#endif
