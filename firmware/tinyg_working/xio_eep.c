/*
 *  xio_eep.c	- device driver for program memory "files" in eeprom
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the 
 * terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License along 
 * with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"						// includes for all devices are in here

#define EEP ds[XIO_DEV_EEP]				// device struct accessor
#define EEPf fs[XIO_DEV_EEP_OFFSET]		// file extended struct accessor

/* 
 *	xio_init_eep() - initialize and set controls for program memory device 
 */
void xio_init_eep()
{
	// Program memory file device setup
	xio_init_dev(XIO_DEV_EEP, xio_open_eep, xio_setflags_eep, xio_putc_eep, xio_getc_eep, xio_readln_eep);
	xio_init_file(XIO_DEV_EEP, XIO_DEV_EEP_OFFSET, EEP_INIT_bm);
}

/* 
 *	xio_init_file() - generic init for file devices
 */
/*
void xio_init_file(const uint8_t dev, const uint8_t offset, const uint16_t control)
{
	// bind file struct to extended device parameters
	ds[dev].x = &fs[offset];		// bind pgm FILE struct
	// might be useful to sanity check the control bits before calling set flags
	//	- RD and BLOCK are mandatory
	// 	- WR and NOBLOCK are restricted
	xio_setflags_eep(control);
}
*/

/*	
 *	xio_open_eep() - provide a string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for it's moral equivalency
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */

struct __file * xio_open_eep(const prog_char *addr)
{
	EEP.flags &= XIO_FLAG_RESET_gm;			// reset flag signaling bits
	EEP.sig = 0;							// reset signal
	EEPf.filebase_P = (PROGMEM char *)addr;	// might want to range check this
	EEPf.len = 0;							// initialize buffer pointer
	return(EEP.fdev);							// return pointer to the fdev stream
}

/*
 *	xio_setflags_eep() - check and set control flags for device
 */

int xio_setflags_eep(const uint16_t control)
{
	xio_setflags(XIO_DEV_EEP, control);
	return (XIO_OK);									// for now it's always OK
}

/* 
 *	xio_putc_eep() - write character to to program memory device
 *
 *  Always returns error. You cannot write to program memory
 */

int xio_putc_eep(const char c, struct __file *stream)
{
	return -1;			// always returns an error. Big surprise.
}

/*
 *  xio_getc_eep() - read a character from program memory device
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

int xio_getc_eep(struct __file *stream)
{

	if (EEP.flags & XIO_FLAG_EOF_bm) {
		EEP.sig = XIO_SIG_EOF;
		return (_FDEV_EOF);
	}
	if ((EEP.c = pgm_read_byte(&EEPf.filebase_P[EEPf.len])) == NUL) {
		EEP.flags |= XIO_FLAG_EOF_bm;
	}
	++EEPf.len;
	if (!LINEMODE(EEP.flags)) {			// processing is simple if not LINEMODE
		if (ECHO(EEP.flags)) {
			putchar(EEP.c);
		}
		return (EEP.c);
	}
	// now do the LINEMODE stuff
	if (EEP.c == NUL) {					// perform newline substitutions
		EEP.c = '\n';
	} else if (EEP.c == '\r') {
		EEP.c = '\n';
	} else if ((SEMICOLONS(EEP.flags)) && (EEP.c == ';')) {
		EEP.c = '\n';
	}
	if (ECHO(EEP.flags)) {
		putchar(EEP.c);
	}
	return (EEP.c);
}

/* 
 *	xio_readln_eep() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_readln_eep(char *buf, const uint8_t size)
{
	if (!(EEPf.filebase_P)) {					// return error if no file is open
		return (XIO_FILE_NOT_OPEN);
	}
	EEP.sig = XIO_SIG_OK;						// initialize signal
	if (fgets(buf, size, EEP.fdev) == NULL) {
		EEPf.filebase_P = NULL;
		clearerr(EEP.fdev);
		return (XIO_EOF);
	}
	return (XIO_OK);
}
