/*
 * xio_file.h	- device driver for file-type devices
 *   			- works with avr-gcc stdio library
 *
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.

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
int xio_pgm_readln(char *buf, uint8_t len);		// read line from program memory

extern FILE dev_pgm;							// used to return a pointer on open()

#define PGMFILE (const PROGMEM char *)			// extends pgmspace.h

// control flags for inits
#define PGM_CONTROL_bm (XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_BAUD_115200)


// file-type device control struct
struct xioFILE {
	uint16_t len;						// index into file
	char * pgmbase_P;					// base location in memory
};

#endif
