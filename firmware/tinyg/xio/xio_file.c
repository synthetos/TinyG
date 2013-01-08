/*
 *  xio_file.c	- device driver for program memory "files"
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

#include <stdio.h>			// precursor for xio.h
#include <stdbool.h>		// true and false
#include <avr/pgmspace.h>	// precursor for xio.h
#include "xio.h"			// includes for all devices are in here

/******************************************************************************
 * FILE CONFIGURATION RECORDS
 ******************************************************************************/

struct cfgFILE {
	x_open x_open;			// see xio.h for typedefs
	x_ctrl x_ctrl;
	x_gets x_gets;
	x_getc x_getc;
	x_putc x_putc;
	fc_func fc_func;
};

static struct cfgFILE const cfgFile[] PROGMEM = {
{	// PGM config
	xio_open_file,			// open function
	xio_ctrl_generic, 		// ctrl function
	xio_gets_pgm,			// get string function
	xio_getc_pgm,			// stdio getc function
	xio_putc_pgm,			// stdio putc function
	xio_fc_null,			// flow control callback
}
};
/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/

/* 
 *	xio_init_file() - initialize and set controls for file IO
 *
 *	Need to setup the open function or a cold open will fail
 */

void xio_init_file()
{
	uint8_t idx = (XIO_DEV_PGM - XIO_DEV_FILE_OFFSET);
	xio_open_generic(XIO_DEV_PGM,
					(x_open)pgm_read_word(&cfgFile[idx].x_open),
					(x_ctrl)pgm_read_word(&cfgFile[idx].x_ctrl),
					(x_gets)pgm_read_word(&cfgFile[idx].x_gets),
					(x_getc)pgm_read_word(&cfgFile[idx].x_getc),
					(x_putc)pgm_read_word(&cfgFile[idx].x_putc),
					(fc_func)pgm_read_word(&cfgFile[idx].fc_func));

	ds[XIO_DEV_PGM].x = &fs[idx];					// bind extended struct to device
	return;
}

/*	
 *	xio_open_file() - open the program memory device to a specific string address
 *
 *	OK, so this is not really a UNIX open() except for its moral equivalent
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */
FILE * xio_open_file(const uint8_t dev, const char *addr, const CONTROL_T flags)
{
	xioDev *d = (xioDev *)&ds[dev];
	xioFile *dx = (xioFile *)d->x;

	xio_ctrl_generic(d, flags);

	d->flag_in_line = false;
	d->flag_eol = false;
	d->flag_eof = false;
	d->signal = 0;									// reset signal
	dx->filebase_P = (PROGMEM const char *)addr;	// might want to range check this
	dx->rd_offset = 0;								// initialize read buffer pointer
	dx->wr_offset = 0;								// initialize write buffer pointer
	dx->max_offset = PGM_ADDR_MAX;
	return(&(d->file));								// return pointer to the FILE stream
}
