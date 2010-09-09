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
#include "tinyg.h"

#define XIO_PGM_DEFAULT_gm (XIO_FLAG_RD_bm | XIO_FLAG_ECHO_bm | XIO_FLAG_CRLF_bm | XIO_FLAG_LINEMODE_bm)

struct xioPGM {					// control struct for program memory "files"
	uint16_t flags;				// control flags
	char * pgmbase_P;			// base location in memory
	uint16_t idx;				// index into file
};

static struct xioPGM fpgm;			// create local, low-level "file" control struct
								// *** must be static or collisions occur ***
/* 
 *	xio_pgm_init() - initialize and set controls for program memory device 
 *
 *	The control word is used as a pointer to the starting memory location
 *  Control parameters are defaulted and may be set using xio_pgm_control()
 */

void xio_pgm_init(uint16_t addr)
{
	fpgm.flags = XIO_PGM_DEFAULT_gm;
	fpgm.pgmbase_P = (PROGMEM char *)addr;
	fpgm.idx = 0;
}

/*	
 *	xio_usb_open() - provide a (new) string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for it's moral equivalency
 *	It's really more like a re-init, only without the flag settings
 */

void xio_pgm_open(uint16_t addr)
//void xio_pgm_open(const PROGMEM char *addr)
{
	fpgm.flags &= XIO_FLAG_RESET_gm;			// reset the signal bits
	fpgm.pgmbase_P = (PROGMEM char *)addr;
	fpgm.idx = 0;
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
	fpgm.flags = XIO_FLAG_DEFAULT_gm;			// set flags to defaults & initial state
	if (control & XIO_RD) {
		fpgm.flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_ECHO) {
		fpgm.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		fpgm.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		fpgm.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		fpgm.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		fpgm.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		fpgm.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		fpgm.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		fpgm.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}
	return (0);
}


/* 
 *	xio_pgm_putc() - write character to to program memory device
 *
 *  Always returns error. You cannot write to program memory
 */

int xio_pgm_putc(char c, FILE *stream)
{
	return -1;			// always returns an error. Big surprise.
}


/*
 *  xio_pgm_getc() - read a character from program memory device
 *
 *  Get next character from program memory file.
 *
 *  END OF FILE (EOF)
 *		- the first time you encounter NUL, return ETX
 *		- all subsequent times rreturn NUL
 *	    This allows the higherlevel stdio routines to return a line that 
 *	    terminates with a NUL, but reads from the end of file will return errors.
 *	  	
 *		- return ETX (as returning NUL is indistinguishable from an error)
 *		- return NUL (this is NOT EOF, wich is -1 and signifies and error)
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
	char c;

	if (fpgm.flags & XIO_FLAG_EOF_bm) {
		return (_FDEV_EOF);
	}
	if ((c = pgm_read_byte(&fpgm.pgmbase_P[fpgm.idx])) == NUL) {
		fpgm.flags |= XIO_FLAG_EOF_bm;
//		return (NUL);
	}
	++fpgm.idx;				// make it stick on NUL and for compiler optimization
	if (!LINEMODE(fpgm.flags)) {		// processing is simple if not LINEMODE
		if (ECHO(fpgm.flags)) {
			putchar(c);
		}
		return (c);
	}
//	if 
	// now do the LINEMODE stuff
	if (c == 0) {					// perform newline substitutions
		c = '\n';
	} else if (c == '\r') {
		c = '\n';
	} else if ((SEMICOLONS(fpgm.flags)) && (c == ';')) {
		c = '\n';
	}
	if (ECHO(fpgm.flags)) {
		putchar(c);
	}
	return (c);
}

/*
 *  xio_pgm_getc_status() - return status for program memory character reader
 *
 *  Doesn't really do much, but the stdio caller is required to clear the error 
 *	conditions in the FILE struct, so this takes care of that.
 *
 *  Returns:
 *	   0	no error
 *	  -1	_FDEV_ERR	read error occurred
 *    -2	_FDEV_EOF	hit end-of-file condition. 
 *						The pgm device must be re-opened to be read again.
 *
 *  Just in case we weren;t sufficiently confused, stdio.h defines EOF as -1,
 *	so please use _FDEV_ERR and _FDEV_EOF instead of EOF to avoid problems.
 *
 */
int xio_pgm_getc_status(FILE *stream)
{
	if (stream->flags & __SERR) {
		stream->flags &= ~__SERR;		// clear it
		return (_FDEV_ERR);
	}
	if (stream->flags & __SEOF) {
		stream->flags &= ~__SEOF;		// clear it
		return (_FDEV_EOF);
	}
	return (0);
}

