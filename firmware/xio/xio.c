/*
 * xio.c - Xmega IO devices - common code file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
 *		- GPIO ports
 *		- (other devices will be added as needed)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Additional functions provided include:
 *		- open file (initialize address and other parameters)
 *		- gets (non-blocking input line reader - extends fgets)
 *		- ctl (ioctl-like knockoff for setting device parameters)
 *		- signal handling - captures ^c, pause, resume, etc. as interrupts
 *		- interrupt buffered RX and TX functions 
 *		- XON/XOFF software flow control implemented
 *
 *	For further notes see the end of xio.h
 */

#include <string.h>					// for memset()
#include <stdio.h>					// precursor for xio.h
#include <avr/pgmspace.h>			// precursor for xio.h

#include "xio.h"					// all device includes are nested here
#include "../tinyg.h"				// needed by init() for default source
#include "../config.h"				// needed by init() for default source
#include "../controller.h"			// needed by init() for default source

/*
 * Static allocations for XIO 
 * See xio.h for device struct definition
 * See xio_usart.h for usart device struct definition
 * See xio_file.h for file device struct definition
 * See xio_signal.h for signal flag struct definition
 */
struct xioDEVICE ds[XIO_DEV_COUNT];		// allocate top-level dev structs
struct xioUSART us[XIO_DEV_USART_COUNT];// ...USART extended IO structs
struct xioFILE fs[XIO_DEV_FILE_COUNT];	// ...FILE extended IO structs
struct __file ss[XIO_DEV_COUNT];		// ...stdio stream for each dev
struct xioSIGNALS sig;					// ...signal flags
extern struct controllerSingleton tg;	// needed by init() for default source

void xio_init()
{	
	// call device inits
	xio_init_rs485();
	xio_init_usb();
	xio_init_pgm();						// program memory file device
//	xio_init_eep();						// EEPROM memory file device
//	xio_init_ram();						// RAM memory file device
}

/*
 * xio_init_dev() - generic (and partial) initialization for device
 *
 *	Requires device specific init to be run afterward.
 *	Could technically do controls (flags) here, but controls are set in 
 *	device-specific init so validation can be performed.
 */

void xio_init_dev(uint8_t dev, 					// device number
	FILE *(*x_open)(const prog_char *addr),		// device open routine
	int (*x_cntl)(const uint32_t control),		// set device control flags
//	int (*x_rctl)(uint32_t *control),			// get device control flags
	int (*x_putc)(char, struct __file *),		// write char (stdio compat)
	int (*x_getc)(struct __file *),				// read char (stdio compat)
	int (*x_gets)(char *buf, int size)			// specialized line reader
	) 
{
	// clear device struct
	memset (&ds[dev], 0, sizeof(struct xioDEVICE));	

	// bind functions to device structure
	ds[dev].x_open = x_open;	
	ds[dev].x_cntl = x_cntl;
//	ds[dev].x_rctl = x_rctl;
	ds[dev].x_putc = x_putc;
	ds[dev].x_getc = x_getc;
	ds[dev].x_gets = x_gets;

	// bind and setup stdio struct
	ds[dev].fdev = &ss[dev];					
	fdev_setup_stream(ds[dev].fdev, x_putc, x_getc, _FDEV_SETUP_RW);
}

/* 
 *	xio_init_file() - generic init for file devices
 */

void xio_init_file(const uint8_t dev, const uint8_t offset, const uint32_t control)
{
	// bind file struct to extended device parameters
	ds[dev].x = &fs[offset];		// bind pgm FILE struct
	// might be useful to sanity check the control bits before calling set flags
	//	- RD and BLOCK are mandatory
	// 	- WR and NOBLOCK are restricted
	(void)xio_cntl(dev, control);
}

/*
 * xio_cntl() - Note: this is not ioctl() Calling conventions differ.
 */

#define SETFLAG(t,f) if ((control & t) != 0) { d->flags |= f; }
#define CLRFLAG(t,f) if ((control & t) != 0) { d->flags &= ~f; }

int xio_cntl(const uint8_t dev, uint32_t control)
{
	struct xioDEVICE *d = &ds[dev];

	SETFLAG(XIO_RD, XIO_FLAG_RD_bm);
	SETFLAG(XIO_WR, XIO_FLAG_WR_bm);
	SETFLAG(XIO_BLOCK, XIO_FLAG_BLOCK_bm);
	CLRFLAG(XIO_NOBLOCK, XIO_FLAG_BLOCK_bm);
	SETFLAG(XIO_XOFF, XIO_FLAG_XOFF_bm);
	CLRFLAG(XIO_NOXOFF, XIO_FLAG_XOFF_bm);
	SETFLAG(XIO_ECHO, XIO_FLAG_ECHO_bm);
	CLRFLAG(XIO_NOECHO, XIO_FLAG_ECHO_bm);
	SETFLAG(XIO_CRLF, XIO_FLAG_CRLF_bm);
	CLRFLAG(XIO_NOCRLF, XIO_FLAG_CRLF_bm);
	SETFLAG(XIO_IGNORECR, XIO_FLAG_IGNORECR_bm);
	CLRFLAG(XIO_NOIGNORECR, XIO_FLAG_IGNORECR_bm);
	SETFLAG(XIO_IGNORELF, XIO_FLAG_IGNORELF_bm);
	CLRFLAG(XIO_NOIGNORELF, XIO_FLAG_IGNORELF_bm);
	SETFLAG(XIO_LINEMODE, XIO_FLAG_LINEMODE_bm);
	CLRFLAG(XIO_NOLINEMODE, XIO_FLAG_LINEMODE_bm);
	return (XIO_OK);
}

/*
 * xio_rctl() - Note: this is not ioctl() Calling conventions differ.
 */
/*
int xio_rctl(const uint8_t dev, uint32_t *control)
{
	struct xioDEVICE *d = &ds[dev];
	control = d->flags;
	return (XIO_OK);
}
*/

/*
 * xio_set_stdin()  - set stdin from device number
 * xio_set_stdout() - set stdout from device number
 * xio_set_stderr() - set stderr from device number
 */

void xio_set_stdin(const uint8_t dev)
{
	stdin = ds[dev].fdev;
}

void xio_set_stdout(const uint8_t dev)
{
	stdout = ds[dev].fdev;
}

void xio_set_stderr(const uint8_t dev)
{
	stderr = ds[dev].fdev;
}

/*
 * xio_putc() - common entry point for putc
 */

int xio_putc(const uint8_t dev, const char c)
{
	if (dev < XIO_DEV_COUNT) {
		return ds[dev].x_putc(c, ds[dev].fdev);
	} else {
		return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
	}
}

/*
 * xio_getc() - common entry point for getc
 */

int xio_getc(const uint8_t dev)
{
	if (dev < XIO_DEV_COUNT) {
		return ds[dev].x_getc(ds[dev].fdev);
	} else {
		return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
	}		
}

/*
 * xio_gets() - common entry point for non-blocking receive line functions
 *
 * Arguments
 *	dev		XIO device enumeration
 *	buf		text buffer to read into
 *	size	size of text buffer in 1 offset form: e.g. use 80 instead of 79
 */

int xio_gets(const uint8_t dev, char *buf, const int size)
{
	if (dev < XIO_DEV_COUNT) {
		return ds[dev].x_gets(buf, size);
	} else {
		return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
	}		
}

//########################################################################

#ifdef __UNIT_TEST_XIO

/*
 * xio_tests() - a collection of tests for xio
 */

void xio_unit_tests()
{
	FILE * fdev;

	fdev = xio_open_pgm(0);
//	xio_puts_pgm("ABCDEFGHIJKLMNOP\n", fdev);
	xio_putc_pgm('A', fdev);
	xio_putc_pgm('B', fdev);
	xio_putc_pgm('C', fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
}

#endif
