/*
 * spindle.h - spindle driver
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

#ifndef spindle_h
#define spindle_h 

#include <avr/io.h>

/*
 * Global Scope Functions
 */

/* Note: See hardware.h for spindle port assignments and bit positions */

void sp_init();
void sp_spindle_run(uint8_t mode, double speed);

#endif
