/*
 *  xio_pgm.c	- device driver for program memory "files"
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include "xio.h"						// includes for all devices are in here

#define PGM ds[XIO_DEV_PGM]				// device struct accessor
#define PGMf fs[XIO_DEV_PGM - XIO_DEV_FILE_OFFSET]	// file extended struct accessor

/* 
 *	xio_init_pgm() - initialize and set controls for program memory device 
 */
void xio_init_pgm()
{
	// Program memory file device setup
	xio_init_dev(XIO_DEV_PGM, xio_open_pgm, xio_ctrl, xio_gets_pgm, xio_getc_pgm, xio_putc_pgm, xio_fc_null);
	xio_init_file(XIO_DEV_PGM, PGM_INIT_bm);
}

/*	
 *	xio_open_pgm() - provide a string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for its moral equivalent
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */

FILE * xio_open_pgm(const uint8_t dev, const char *addr)
{
	PGM.flag_in_line = false;
	PGM.flag_eol = false;
	PGM.flag_eof = false;
	PGM.signal = 0;									// reset signal
	PGMf.filebase_P = (PROGMEM const char *)addr;	// might want to range check this
	PGMf.rd_offset = 0;								// initialize read buffer pointer
	PGMf.wr_offset = 0;								// initialize write buffer pointer
	PGMf.max_offset = PGM_ADDR_MAX;
	return(&PGM.file);								// return pointer to the FILE stream
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
 *	xio_gets_pgm() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_gets_pgm(const uint8_t dev, char *buf, const int size)
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
