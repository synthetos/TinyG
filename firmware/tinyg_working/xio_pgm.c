/*
 *  xio_pgm.c	- device driver for program memory "files"
 * 				- works with avr-gcc stdio library
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
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "xio.h"
#include "xio_file.h"			// general .h for all file-type devices
#include "tinyg.h"				// needed for TG_ return codes, or provide your own

// necessary structures
extern struct xioDEVICE ds[XIO_DEV_COUNT];		// ref top-level device structs
extern struct xioFILE fs[XIO_DEV_FILE_COUNT];	// ref FILE extended IO structs
#define PGM ds[XIO_DEV_PGM]						// device struct accessor
#define PGMf fs[XIO_DEV_PGM_OFFSET]				// file extended struct accessor

/* 
 *	xio_init_pgm() - initialize and set controls for program memory device 
 */

void xio_init_pgm(const uint16_t control)
{
	// might be useful to sanity check the control bits before calling set flags
	//	- RD and BLOCK are mandatory
	// 	- WR and NOBLOCK are restricted
	xio_setflags_pgm(control);
}

/*	
 *	xio_open_pgm() - provide a string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for it's moral equivalency
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */

struct __file * xio_open_pgm(const prog_char *addr)
{
	PGM.flags &= XIO_FLAG_RESET_gm;			// reset flag signaling bits
	PGM.sig = 0;							// reset signal
	PGMf.pgmbase_P = (PROGMEM char *)addr;	// might want to range check this
	PGMf.len = 0;							// initialize buffer pointer
	return(PGM.fdev);							// return pointer to the fdev stream
}

/*
 *	xio_setflags_pgm() - check and set control flags for device
 */

int xio_setflags_pgm(const uint16_t control)
{
	xio_setflags(XIO_DEV_PGM, control);
	return (TG_OK);									// for now it's always OK
}

/* 
 *	xio_putc_pgm() - write character to to program memory device
 *
 *  Always returns error. You cannot write to program memory
 */

int xio_putc_pgm(const char c, FILE *stream)
{
	return -1;			// always returns an error. Big surprise.
}

/*
 *  xio_getc_pgm() - read a character from program memory device
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

int xio_getc_pgm(FILE *stream)
{

	if (PGM.flags & XIO_FLAG_EOF_bm) {
		PGM.sig = XIO_SIG_EOF;
		return (_FDEV_EOF);
	}
	if ((PGM.c = pgm_read_byte(&PGMf.pgmbase_P[PGMf.len])) == NUL) {
		PGM.flags |= XIO_FLAG_EOF_bm;
	}
	++PGMf.len;
	if (!LINEMODE(PGM.flags)) {			// processing is simple if not LINEMODE
		if (ECHO(PGM.flags)) {
			putchar(PGM.c);
		}
		return (PGM.c);
	}
	// now do the LINEMODE stuff
	if (PGM.c == NUL) {					// perform newline substitutions
		PGM.c = '\n';
	} else if (PGM.c == '\r') {
		PGM.c = '\n';
	} else if ((SEMICOLONS(PGM.flags)) && (PGM.c == ';')) {
		PGM.c = '\n';
	}
	if (ECHO(PGM.flags)) {
		putchar(PGM.c);
	}
	return (PGM.c);
}

/* 
 *	xio_readln_pgm() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_readln_pgm(char *buf, uint8_t len)
{
	if (!(PGMf.pgmbase_P)) {					// return error if no file is open
		return (TG_FILE_NOT_OPEN);
	}
	PGM.sig = XIO_SIG_OK;						// initialize signal
	if (fgets(buf, len, PGM.fdev) == NULL) {
		PGMf.pgmbase_P = NULL;
		clearerr(PGM.fdev);
		return (TG_EOF);
	}
	return (TG_OK);
}
