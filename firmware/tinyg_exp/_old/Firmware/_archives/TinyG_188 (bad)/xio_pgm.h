/*
  xio_pgm.h - device driver for program memory "files"
    		- works with avr-gcc stdio library

  Copyright (c) 2010 Alden S. Hart, Jr.

---- How to setup and use program memory "files" ----

  Setup a memory file (OK, it's really just a string)
  should be declared as so:

	const char PROGMEM g0_test1[] = "\
	g0 x10 y20 z30\n\
	g0 x0 y21 z-34.2";

	Line 1 is the initial declaration of the array (string) in program memory
	Line 2 is a continuation line. 
		- Must end with a newline and a continuation backslash
		- (alternately can use a semicolon instead of \n is XIO_SEMICOLONS is set)
		- Each line will be read as a single line of text using fgets()
	Line 3 is the terminating line. Note the closing quote and semicolon.


  Initialize: xio_pgm_init() must be called first. See the routine for options.

  Open the file: xio_pgm_open() is called, like so:
	xio_pgm_open(PGMFILE(&g0_test1));	// simple linear motion test

	PGMFILE does the right cast. If someone more familiar with all this can explain 
	why PSTR doesn't work I'd be grateful.

  Read a line of text. Example from parsers.c
	if (fgets(textbuf, BUF_LEN-1, srcin) == NULL) {	
		printf_P(PSTR("\r\nEnd of file encountered\r\n"));
		clearerr(srcin);
		srcin = stdin;
		tg_prompt();
		return;
	}
*/

#ifndef xio_pgm_h
#define xio_pgm_h

#include <avr/pgmspace.h>

/*
 * Global Scope Functions
 */

void xio_pgm_init(uint16_t control);			// init program memory device
FILE * xio_pgm_open(const prog_char * addr);	// open a memory string for read
int8_t xio_pgm_control(uint16_t control, int16_t arg); // set flags
int xio_pgm_putc(char c, FILE *stream);			// always returns ERROR
int xio_pgm_getc(FILE *stream);					// get a character
int xio_pgm_readln(void);						// read line from memory and dispatch

extern FILE dev_pgm;							// used to return a pointer on open()

#define PGMFILE (const PROGMEM char *)			// extends pgmspace.h

#endif
