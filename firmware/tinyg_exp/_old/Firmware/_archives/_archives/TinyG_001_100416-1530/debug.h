/*
  debug.h - various debugging routines
  Part of TinyG

  Copyright (c) 2010 Alden S Hart, Jr.

*/

#ifndef debug_h
#define debug_h

#include "wiring_serial.h"	// ++++ NEEDED FOR DEBUG ONLY - OTHERWISE NO PRINTING
#include <avr/pgmspace.h>

void st_print_four_ints(long x, long y, long z, long u);
void st_print_active();
void st_print_line(struct Line line);

#endif
