/*
 * spindle_control.c - spindle control driver
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <avr/io.h>
#include "stepper.h"
#include "spindle.h"
#include "gcode.h"

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
