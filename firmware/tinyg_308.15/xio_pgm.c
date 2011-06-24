/*
 *  xio_pgm.c	- device driver for program memory "files"
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <stdio.h>
#include "xio.h"						// includes for all devices are in here

#define PGM ds[XIO_DEV_PGM]				// device struct accessor
#define PGMf fs[XIO_DEV_PGM_OFFSET]		// file extended struct accessor

/* 
 *	xio_init_pgm() - initialize and set controls for program memory device 
 */
void xio_init_pgm()
{
	// Program memory file device setup
	xio_init_dev(XIO_DEV_PGM, xio_open_pgm, xio_cntrl_pgm, xio_putc_pgm, xio_getc_pgm, xio_gets_pgm);
	xio_init_file(XIO_DEV_PGM, XIO_DEV_PGM_OFFSET, PGM_INIT_bm);
}

/*	
 *	xio_open_pgm() - provide a string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for its moral equivalent
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */

struct __file * xio_open_pgm(const prog_char *addr)
{
	PGM.flags &= XIO_FLAG_RESET_gm;	// reset flag signaling bits
	PGM.signal = 0;					// reset signal
	PGMf.filebase_P = (PROGMEM char *)addr;	// might want to range check this
	PGMf.rd_offset = 0;				// initialize read buffer pointer
	PGMf.wr_offset = 0;				// initialize write buffer pointer
	PGMf.max_offset = PGM_ADDR_MAX;
	return(PGM.fdev);				// return pointer to the fdev stream
}

/*
 *	xio_cntrl_pgm() - check and set control flags for device
 */

int xio_cntrl_pgm(const uint16_t control)
{
	xio_cntrl(XIO_DEV_PGM, control);
	return (XIO_OK);						// for now it's always OK
}

/* 
 *	xio_putc_pgm() - write character to to program memory device
 *
 *  Always returns error. You cannot write to program memory
 */

int xio_putc_pgm(const char c, struct __file *stream)
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
 *	    terminates with a NUL, but reads from the end of file will return 
 *		errors.
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

int xio_getc_pgm(struct __file *stream)
{
	if ((PGM.flags & XIO_FLAG_EOF_bm) != 0) {
		PGM.signal = XIO_SIG_EOF;
		return (_FDEV_EOF);
	}
	if ((PGM.c = pgm_read_byte(&PGMf.filebase_P[PGMf.rd_offset])) == NUL) {
		PGM.flags |= XIO_FLAG_EOF_bm;
	}
	++PGMf.rd_offset;
	if (LINEMODE(PGM.flags) == 0) {	// processing is simple if not LINEMODE
		if (ECHO(PGM.flags) != 0) {
			putchar(PGM.c);
		}
		return (PGM.c);
	}
	// now do the LINEMODE stuff
	if (PGM.c == NUL) {				// perform newline substitutions
		PGM.c = '\n';
	} else if (PGM.c == '\r') {
		PGM.c = '\n';
//	} else if ((SEMICOLONS(PGM.flags) != 0) && (PGM.c == ';')) {
//		PGM.c = '\n';
	}
	if (ECHO(PGM.flags) != 0) {
		putchar(PGM.c);
	}
	return (PGM.c);
}

/* 
 *	xio_gets_pgm() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_gets_pgm(char *buf, const int size)
{
	if ((PGMf.filebase_P) == 0) {		// return error if no file is open
		return (XIO_FILE_NOT_OPEN);
	}
	PGM.signal = XIO_SIG_OK;			// initialize signal
	if (fgets(buf, size, PGM.fdev) == NULL) {
		PGMf.filebase_P = NULL;
		clearerr(PGM.fdev);
		return (XIO_EOF);
	}
	return (XIO_OK);
}
