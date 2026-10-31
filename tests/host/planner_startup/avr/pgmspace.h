/*
 * include/avr/pgmspace.h - host stand-in for avr-libc's pgmspace.h
 *
 * tinyg.h unconditionally defines __AVR and includes <avr/pgmspace.h>. On the host
 * PROGMEM data is ordinary RAM, so the *_P functions map to their RAM equivalents.
 */
#ifndef HOST_PGMSPACE_H_ONCE
#define HOST_PGMSPACE_H_ONCE

#include <stdint.h>
#include <stdlib.h>		// must precede tinyg.h's "#define strtof strtod"
#include <string.h>
#include <stdio.h>

#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(a)  (*(const uint8_t *)(a))
#define pgm_read_word(a)  (*(const uint16_t *)(a))
#define pgm_read_float(a) (*(const float *)(a))
#define strcpy_P   strcpy
#define strncpy_P  strncpy
#define strcmp_P   strcmp
#define strlen_P   strlen
#define printf_P   printf
#define sprintf_P  sprintf
#define fprintf_P  fprintf

#endif
