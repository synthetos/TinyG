/*
 * lint.h - let GCC check the format strings of the PROGMEM stdio calls (make lint only)
 *
 * avr-libc declares printf_P() and friends without a format attribute, so a PSTR() format is
 * never checked. Force-included ahead of each source by 'make lint', this maps them to the plain
 * functions and PSTR(s) to s. The objects it produces are thrown away; they are not firmware.
 */
#ifndef LINT_H_ONCE
#define LINT_H_ONCE

#include <avr/pgmspace.h>   // define PSTR and the _P prototypes first, so the macros below stick
#include <stdio.h>

#undef  PSTR
#define PSTR(s) (s)
#define printf_P   printf
#define sprintf_P  sprintf
#define snprintf_P snprintf
#define fprintf_P  fprintf

#endif
