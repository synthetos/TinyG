/*
  xio_pgm.h - device driver for program memory "files"
  			- works with avr-gcc stdio library

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#include "xmega_support.h"		// should be first

#include <stdio.h>
#include <avr/pgmspace.h>

#include "xio.h"
#include "xio_pgm.h"

#define XIO_PGM_DEFAULT_gm (XIO_FLAG_RD_bm | XIO_ECHO | XIO_CRLF | XIO_LINEMODE)

struct xioPGM {					// control struct for program memory "files"
	uint16_t flags;				// control flags
	char * pgmbase_P;			// base location in memory
	uint16_t idx;				// index into file
};

static struct xioPGM f;			// create local, low-level "file" control struct
								// *** must be static or collisions occur ***
/* 
 *	xio_pgm_init() - initialize and set controls for program memory device 
 *
 *	The control word is used as a pointer to the starting memory location
 *  Control parameters are defaulted and may be set using xio_pgm_control()
 */

void xio_pgm_init(uint16_t addr)
{
	f.flags = XIO_PGM_DEFAULT_gm;
	f.pgmbase_P = (PROGMEM char *)addr;
	f.idx = 0;
}

/*	
 *	xio_usb_open() - provide a (new) string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for it's moral equivalency
 *	It's really more like a re-init, only without the flag settings
 */

void xio_pgm_open(uint16_t addr)
{
	f.pgmbase_P = (PROGMEM char *)addr;
	f.idx = 0;
}

/*	
 *	xio_usb_control() - set controls for program memory device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_RD		  <null>	Y	Enable device for reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 */

int8_t xio_pgm_control(uint16_t control, int16_t arg)
{
	// transfer control flags to internal flag bits
	f.flags = XIO_FLAG_DEFAULT_gm;			// set flags to defaults & initial state
	if (control & XIO_RD) {
		f.flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_ECHO) {
		f.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		f.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		f.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		f.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		f.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		f.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		f.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		f.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}
	return (0);
}


/* 
 *	xio_pgm_putc() - char writer for program memory device
 */

int xio_pgm_putc(char c, FILE *stream)
{
	return -1;			// always returns an error. Big surprise.
}


/*
 *  xio_pgm_getc() - char reader for program memory device
 *
 *  Get next character from program memory file.
 *
 *  LINEMODE and SEMICOLONS behaviors
 *		- consider <cr> and <lf> to be EOL chars (not just <lf>)
 *		- also consider semicolons (';') to be EOL chars if SEMICOLONS enabled
 *		- convert any EOL char to <lf> to signal end-of-string (e.g. to fgets())
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc should expand newlines to <cr><lf>
 */

int xio_pgm_getc(FILE *stream)
{
	char c = pgm_read_byte(&f.pgmbase_P[f.idx++]);

	if (!LINEMODE(f.flags)) {			// processing is simple if not LINEMODE
		if (ECHO(f.flags)) {
			putchar(c);
		}
		return (c);
	}
	// now do the LINEMODE stuff
	if (c == '\r') {					// perform newline substitutions
		c = '\n';
	} else if ((SEMICOLONS(f.flags)) && (c == ';')) {
		c = '\n';
	}
	if (ECHO(f.flags)) {
		putchar(c);
	}
	return (c);
}

