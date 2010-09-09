/*
  xio_pgm.h - device driver for program memory "files"
    		- works with avr-gcc stdio library

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xio_pgm_h
#define xio_pgm_h

#include <avr/pgmspace.h>

/* 
 * Function prototypes and aliases
 */

void xio_pgm_init(uint16_t addr);						// init program memory device
void xio_pgm_open(uint16_t addr);						// provida a string address
//void xio_pgm_open(PSTR(addr));

int8_t xio_pgm_control(uint16_t control, int16_t arg);	// set flags
int xio_pgm_putc(char c, FILE *stream);					// always returns ERROR
int xio_pgm_getc(FILE *stream);							// get a character

#endif
