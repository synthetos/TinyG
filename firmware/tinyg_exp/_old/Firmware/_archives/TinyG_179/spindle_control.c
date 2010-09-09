/*
 * spindle_control.c - spindle control methods
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 * Modified to support Xmega family processors May 2010 Alden S. Hart, Jr.
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
 *
 */

#include <avr/io.h>
#include "spindle_control.h"
#include "config.h"

/* spindle_init()
 *	Spindle init takes over the 2 MSBs of the A axis port for spindle control
 *	These should have been initially set as A axis max/min limit inputs
 *	See config.h for settings
 */

void spindle_init()
{
	SPINDLE_ENABLE_PORT.DIRSET = SPINDLE_ENABLE_BIT_bm;
	SPINDLE_DIRECTION_PORT.DIRSET = SPINDLE_DIRECTION_BIT_bm;
}

void spindle_run(int direction, uint32_t rpm) 
{
	if(direction >= 0) {
    	SPINDLE_DIRECTION_PORT.OUTSET = SPINDLE_DIRECTION_BIT_bm;
	} else {
    	SPINDLE_DIRECTION_PORT.OUTCLR = SPINDLE_DIRECTION_BIT_bm;
	}
	SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_BIT_bm;
}

void spindle_stop()
{
	SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_BIT_bm;
}
