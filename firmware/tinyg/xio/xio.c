/*
 * xio.c - Xmega IO devices - common code file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License as published by the Free Software Foundation, either version 3 
 * of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for details. You should have received a copy of the GNU General 
 * Public License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* ----- XIO - Xmega Device System ----
 *
 * XIO provides common access to native and derived xmega devices 
 * (see table below) XIO devices are compatible with avr-gcc stdio 
 * and also provide some special functions that extend stdio.
 *
 * Stdio support:
 *	- http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 * 	- Stdio compatible putc() and getc() functions provided for each device
 *	- This enables fgets, printf, scanf, and other stdio functions
 * 	- Full support for formatted printing is provided (including floats)
 * 	- Assignment of a default device to stdin, stdout & stderr is provided 
 *	- printf() and printf_P() send to stdout, so use fprintf() to stderr
 *		for things that should't go over RS485 in SLAVE mode 
 *
 * Facilities provided beyond stdio:
 *	- Devices are managed as an enumerated array of derived devices
 *	- Supported devices include:
 *		- USB (derived from USART)
 *		- RS485 (derived from USART)
 *		- Arduino connection (derived from USART)
 *		- Program memory "files" (read only)
 *		- EEPROM "files" (limited read/write capabilities)
 *		- (other devices will be added as needed)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Additional functions provided include:
 *		- open file (initialize address and other parameters)
 *		- gets (non-blocking input line reader - extends fgets)
 *		- control (ioctl-like knockoff for setting device parameters)
 *		- signal handling: interrupt on: feedhold, cycle_start, ctrl-x software reset
 *		- interrupt buffered RX and TX functions 
 *		- XON/XOFF software flow control
 */

#include <string.h>					// for memset()
#include <stdio.h>					// precursor for xio.h
#include <avr/pgmspace.h>			// precursor for xio.h

#include "xio.h"					// all device includes are nested here
#include "../tinyg.h"				// needed by init() for default source
#include "../config.h"				// needed by init() for default source
#include "../controller.h"			// needed by init() for default source

/*
 * xio_init() - initialize entire xio sub-system
 */
void xio_init()
{
	xio_init_usb();					// setup USB usart device
	xio_init_rs485();				// setup RS485 usart device
	xio_init_spi_devices();			// setup all SPI devices
	xio_init_pgm();					// setup file reader
}

/*
 * xio_init_dev() - generic (and partial) initialization for any device
 *
 *	Requires device specific init to be run afterward.
 *	Could technically do controls (flags) here, but controls are set in 
 *	device-specific init so validation can be performed.
 */
void xio_init_dev(uint8_t dev, 									// device number
	FILE *(*x_open)(const uint8_t dev, const char *addr), 		// device open routine
	int (*x_ctrl)(const uint8_t dev, const CONTROL control),	// set device control flags
//	int (*x_rctl)(const uint8_t dev, CONTROL *control),			// get device control flags
	int (*x_gets)(const uint8_t dev, char *buf, int size),		// specialized line reader
	int (*x_getc)(FILE *),										// read char (stdio compat)
	int (*x_putc)(char, FILE *),								// write char (stdio compat)
	void (*fc_func)(xioDevice *)								// flow control callback function
	)
{
	xioDevice *d = &ds[dev];

	// clear device struct
	memset (d, 0, sizeof(xioDevice));		// (Note: 'x' binding is set by device-specific init)

	d->dev = dev;
	d->fdev = &ss[dev];						// bind pre-allocated stdio stream struct
	fdev_setup_stream(d->fdev, x_putc, x_getc, _FDEV_SETUP_RW);
	fdev_set_udata(d->fdev, d);				// reference self for udata 

	// bind functions to device structure
	d->x_open = x_open;			// bind the open function to the PGM struct
	d->x_ctrl = x_ctrl;
//	d->x_rctl = x_rctl;
	d->x_gets = x_gets;
	d->x_getc = x_getc;		// you don't need to bind these unless you are going to use them directly
	d->x_putc = x_putc;		// they are bound into the fdev stream struct
	d->fc_func = fc_func;	// flow control function or null FC function
}

/*
 *	xio_init_file() - generic init for file devices
 *
 *	This should really go in xio_file.c but it seemed excessive to create 
 *	that file just for this one function - so it's here.
 */

void xio_init_file(const uint8_t dev, const CONTROL control)
{
	ds[dev].x = &fs[dev - XIO_DEV_FILE_OFFSET];	// bind pgm FILE struct
	(void)xio_ctrl(dev, control);
}

/*
 * xio_open() - Generic open function 
 */
FILE *xio_open(uint8_t dev, const char *addr)
{
	if (dev == XIO_DEV_PGM) {
		return (xio_open_pgm (dev, addr));	// take some action
	}
	return(ds[dev].fdev);					// otherwise just parrot the fdev
}

/*
 * xio_cntl() - Note: this is not ioctl() Calling conventions differ.
 */

#define SETFLAG(t,f) if ((control & t) != 0) { d->f = true; }
#define CLRFLAG(t,f) if ((control & t) != 0) { d->f = false; }

int xio_ctrl(const uint8_t dev, CONTROL control)
{
	xioDevice *d = &ds[dev];
	SETFLAG(XIO_BLOCK,		flag_block);
	CLRFLAG(XIO_NOBLOCK,	flag_block);
	SETFLAG(XIO_XOFF,		flag_xoff);
	CLRFLAG(XIO_NOXOFF,		flag_xoff);
	SETFLAG(XIO_ECHO,		flag_echo);
	CLRFLAG(XIO_NOECHO,		flag_echo);
	SETFLAG(XIO_CRLF,		flag_crlf);
	CLRFLAG(XIO_NOCRLF,		flag_crlf);
	SETFLAG(XIO_IGNORECR,	flag_ignorecr);
	CLRFLAG(XIO_NOIGNORECR,	flag_ignorecr);
	SETFLAG(XIO_IGNORELF,	flag_ignorelf);
	CLRFLAG(XIO_NOIGNORELF,	flag_ignorelf);
	SETFLAG(XIO_LINEMODE,	flag_linemode);
	CLRFLAG(XIO_NOLINEMODE,	flag_linemode);
	return (XIO_OK);
}

/*
 * xio_rctl() - get control flags
 *
 *	Note: this is not ioctl() Calling conventions differ.
 */
/*
int xio_rctl(const uint8_t dev, CONTROL *control)
{
	xioDevice *d = &ds[dev];
	control = d->flags;
	return (XIO_OK);
}
*/

/*
 * xio_fc_null() - flow control null function
 */
void xio_fc_null(xioDevice *d)
{
	return;
}

/*
 * xio_set_stdin()  - set stdin from device number
 * xio_set_stdout() - set stdout from device number
 * xio_set_stderr() - set stderr from device number
 */

void xio_set_stdin(const uint8_t dev)  { stdin  = ds[dev].fdev; }
void xio_set_stdout(const uint8_t dev) { stdout = ds[dev].fdev; }
void xio_set_stderr(const uint8_t dev) { stderr = ds[dev].fdev; }

/*
 * xio_putc() - common entry point for putc
 * xio_getc() - common entry point for getc
 * xio_gets() - common entry point for non-blocking get line function
 *
 * It would be prudent to run an assertion such as below, but we trust the callers:
 * 	if (dev < XIO_DEV_COUNT) {		
 *		return ds[dev].x_putc(c, ds[dev].fdev);
 *	} else {
 *		return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
 *	}
 */
int xio_putc(const uint8_t dev, const char c)
{
	return ds[dev].x_putc(c, ds[dev].fdev); 
}

int xio_getc(const uint8_t dev) 
{ 
	return ds[dev].x_getc(ds[dev].fdev); 
}

int xio_gets(const uint8_t dev, char *buf, const int size) 
{
	return ds[dev].x_gets(dev, buf, size); 
}

//########################################################################

#ifdef __UNIT_TEST_XIO

/*
 * xio_tests() - a collection of tests for xio
 */

void xio_unit_tests()
{
	FILE * fdev;

	fdev = xio_open(XIO_DEV_SPI1, 0);
//	while (1) {
		xio_putc_spi(0x55, fdev);
//	}
	
//	fdev = xio_open(XIO_DEV_USB, 0);
//	xio_getc_usart(fdev);
	
/*
	fdev = xio_open(XIO_DEV_PGM, 0);
//	xio_puts_pgm("ABCDEFGHIJKLMNOP\n", fdev);
	xio_putc_pgm('A', fdev);
	xio_putc_pgm('B', fdev);
	xio_putc_pgm('C', fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
*/
}

#endif
