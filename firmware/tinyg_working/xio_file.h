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

//#include <avr/pgmspace.h>		// included in parent xio.h

/* 
 * FILE DEVICE CONFIGS 
 */

// PGM device configuration
#define PGM_INIT_bm (XIO_RD | XIO_BLOCK | XIO_ECHO | XIO_CRLF | XIO_LINEMODE)


/* 
 * USART DEVICE CONSTANTS AND PARAMETERS
 */

#define PGMFILE (const PROGMEM char *)	// extends pgmspace.h
//extern FILE dev_pgm;					// used to return a pointer on open()
// control flags for inits
//#define PGM_CONTROL_bm (XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_BAUD_115200)

/* 
 * FILE device extended control structure 
 * Note: As defined this struct won't do files larger than 65,535 chars
 */

// file-type device control struct
struct xioFILE {
	uint16_t fflags;					// file sub-system flags
	uint16_t len;						// index into file
	char * pgmbase_P;					// base location in memory
};

/* 
 * FILE DEVICE FUNCTION PROTOTYPES
 */

// functions common to all FILE devices


// PGM functions
struct __file * xio_open_pgm(const prog_char * addr);// open a memory string readonly
int xio_setflags_pgm(const uint16_t control);	// valaidate & set dev control flags
void xio_init_pgm(uint16_t control);			// init program memory device
int xio_putc_pgm(char c, FILE *stream);			// always returns ERROR
int xio_getc_pgm(FILE *stream);					// get a character
int xio_readln_pgm(char *buf, uint8_t len);		// read line from program memory

// EEPROM functions

// SD Card functions


#endif
