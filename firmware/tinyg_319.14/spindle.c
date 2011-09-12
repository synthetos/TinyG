/*
 * spindle_control.c - spindle control driver
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include "tinyg.h"
#include "stepper.h"
#include "spindle.h"
#include "gcode.h"
#include "system.h"

/* 
 * sp_init()
 *
 *	Spindle init takes over the 2 MSBs of the A axis port for spindle control
 *	These should have been initially set as A axis max/min limit inputs
 *	See hardware.h for settings
 */

void sp_init()
{
	SPINDLE_ENABLE_PORT.DIRSET = SPINDLE_ENABLE_BIT_bm;
	SPINDLE_DIRECTION_PORT.DIRSET = SPINDLE_DIRECTION_BIT_bm;
}

/*
 * sp_spindle_run() - spindle controls
 *
 * Failsafe: if invalid setting (mode) is passed in spindle will stop
 * Speed is a no-op for now.
 */

void sp_spindle_run(uint8_t mode, double speed)
{
	if (mode == SPINDLE_CW) {
    	SPINDLE_DIRECTION_PORT.OUTSET = SPINDLE_DIRECTION_BIT_bm;
		SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_BIT_bm;
	} else if (mode == SPINDLE_CCW) {
    	SPINDLE_DIRECTION_PORT.OUTCLR = SPINDLE_DIRECTION_BIT_bm;
		SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_BIT_bm;
	} else {
		SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_BIT_bm;
	}
}

void sp_spindle_stop()
{
	SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_BIT_bm;
}
