/*
 * gcode.h - rs274/ngc parser.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef gcode_h
#define gcode_h
#include "tinyg.h"

/*
 * Global Scope Functions
 */

void gc_init(void);
uint8_t gc_gcode_parser(char *block);

#endif
