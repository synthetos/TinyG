/*
 *  xio_pgm.c	- device driver for program memory "files"
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2015 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include "../xio.h"						// includes for all devices are in here

// Fast accessors (cheating)
#define PGM ds[XIO_DEV_PGM]				// device struct accessor
#define PGMf fs[XIO_DEV_PGM - XIO_DEV_FILE_OFFSET]	// file extended struct accessor

/*
 *	xio_gets_pgm() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_gets_pgm(xioDev_t *d, char *buf, const int size)
{
	if ((PGMf.filebase_P) == 0) {		// return error if no file is open
		return (XIO_FILE_NOT_OPEN);
	}
	PGM.signal = XIO_SIG_OK;			// initialize signal
	if (fgets(buf, size, &PGM.file) == NULL) {
		PGMf.filebase_P = NULL;
		clearerr(&PGM.file);
		return (XIO_EOF);
	}
	return (XIO_OK);
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

int xio_getc_pgm(FILE *stream)
{
	char c;

	if (PGM.flag_eof ) {
		PGM.signal = XIO_SIG_EOF;
		return (_FDEV_EOF);
	}
	if ((c = pgm_read_byte(&PGMf.filebase_P[PGMf.rd_offset])) == NUL) {
		PGM.flag_eof = true;
	}
	++PGMf.rd_offset;

	// processing is simple if not in LINEMODE
	if (PGM.flag_linemode == false) {
		if (PGM.flag_echo) putchar(c);		// conditional echo
		return (c);
	}

	// now do the LINEMODE stuff
	if (c == NUL) {							// perform newline substitutions
		c = '\n';
	} else if (c == '\r') {
		c = '\n';
	}
	if (PGM.flag_echo) putchar(c);			// conditional echo
	return (c);
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


