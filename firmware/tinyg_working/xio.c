/*
 * xio.c - Xmega IO devices - common header file
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
/* ----- XIO - Xmega Device System ----
 *
 * XIO provides common access to native and derived xmega devices (see table below)
 * XIO devices are compatible with avr-gcc stdio and also provide some special 
 * functions that extend stdio.
 *
 * Stdio support:
 * 	- Stdio compatible putc() and getc() functions are provided for each device.
 *	- This enables fgets, printf, scanf, and other stdio functions.
 * 	- Full support for formatted printing is provided (including floats).
 * 	- Assignment of a default device to stdin, stdout, and stderr is provided. 
 *
 * Facilities provided beyond stdio:
 *	- Devices are managed as an enumerated array of derived devices
 *	- Supported devices include:
 *		- USB (derived from USART)
 *		- RS485 (derived from USART)
 *		- Arduino connection (derived from USART)
 *		- Program memory "files" (read only)
 *		- EEPROM "files" (limited read/write capabilities)
 *		- encoder port
 *		- limit switch port
 *		- (other devices will be added as needed)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Additional functions provided include:
 *		- open file (initialize address and other parameters)
 *		- gets (non-blocking inout line reader - extends fgets functionality)
 *		- setflags (ioctl-like knockoff for setting device parameters)
 *		- signal handling - captures ^c, pause, resume, etc. as interrupts
 *		- interrupt buffered RX and TX functions 
 *
 *	For further notes see the end of xio.h
 */

#include <stdio.h>
#include <string.h>						// for memset()
#include <avr/pgmspace.h>
#include "xio.h"						// all device includes are nested here
#include "controller.h"					// needed by init() for default source

/*
 * structs, static memory allocation, and accessors
 */

struct xioDEVICE ds[XIO_DEV_COUNT];		// allocate top-level device structs
struct xioUSART us[XIO_DEV_USART_COUNT];// allocate USART extended IO structs
struct xioFILE fs[XIO_DEV_FILE_COUNT];	// allocate FILE extended IO structs
struct __file ss[XIO_DEV_COUNT];		// allocate stdio stream for each dev

extern struct tgController tg;			// needed by init() for default source

void xio_init()
{	
	// call device inits
	xio_init_rs485();
	xio_init_usb();
	xio_init_pgm();						// program memory file device
	xio_init_eep();						// EEPROM memory file device
//	xio_init_ram();						// RAM memory file device
}

/*
 *	xio_init_stdio() - initialize stdio devices
 *
 *	Requires xio_init and tg_init to have been run previously
 */

void xio_init_stdio()
{
	// setup stdio bindings to default source device
	xio_set_stdin(tg.default_src);
	xio_set_stdout(tg.default_src);
	xio_set_stderr(tg.default_src);

#ifdef __SLAVE_MODE
	xio_set_stderr(XIO_DEV_USB);		// +++ debug
#endif

	// tell the world we are ready!
	printf_P(PSTR("\n\n**** Xmega IO subsystem initialized ****\n"));
}


/*
 * xio_init_dev() - generic (partial) initialization for device
 *
 *	Requires device specific init to be run afterward.
 *	Could technically do controls (flags) here, but controls are set in 
 *	device-specific init so validation can be performed.
 */

void xio_init_dev(uint8_t dev, 					// device number
	FILE *(*x_open)(const prog_char *addr),		// device open routine
	int (*x_cntrl)(const uint16_t control),		// device control flags
	int (*x_putc)(char, struct __file *),		// write char (stdio compat)
	int (*x_getc)(struct __file *),				// read char (stdio compat)
	int (*x_gets)(char *buf, uint8_t size)		// specialized line reader
	) 
{
	// clear device struct
	memset (&ds[dev], 0, sizeof(struct xioDEVICE));	

	// bind functions
	ds[dev].x_open = x_open;	
	ds[dev].x_cntrl = x_cntrl;
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

void xio_init_file(const uint8_t dev, const uint8_t offset, const uint16_t control)
{
	// bind file struct to extended device parameters
	ds[dev].x = &fs[offset];		// bind pgm FILE struct
	// might be useful to sanity check the control bits before calling set flags
	//	- RD and BLOCK are mandatory
	// 	- WR and NOBLOCK are restricted
	xio_cntrl(dev, control);
}

/*
 * xio_cntrl()
 */

int xio_cntrl(const uint8_t dev, const uint16_t control)
{
	struct xioDEVICE *d = &ds[dev];

	if (control & XIO_RD) {
		d->flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_WR) {
		d->flags |= XIO_FLAG_WR_bm;
	}
	if (control & XIO_BLOCK) {
		d->flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		d->flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		d->flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		d->flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		d->flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		d->flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		d->flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		d->flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		d->flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		d->flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}
	return (XIO_OK);
}

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
		return (XIO_NO_SUCH_DEVICE);
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
		return (XIO_NO_SUCH_DEVICE);
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

int xio_gets(const uint8_t dev, char *buf, const uint8_t size)
{
	if (dev < XIO_DEV_COUNT) {
		return ds[dev].x_gets(buf, size);
	} else {
		return (XIO_NO_SUCH_DEVICE);
	}		
}

#ifdef __UNIT_TESTS

/*
 * xio_tests() - a collection of tests for xio
 */

void xio_tests()
{
	FILE * fdev;

	fdev = xio_open_eep(0);
//	xio_puts_eep("ABCDEFGHIJKLMNOP\n", fdev);
	xio_putc_eep('A', fdev);
	xio_putc_eep('B', fdev);
	xio_putc_eep('C', fdev);
	xio_getc_eep(fdev);
	xio_getc_eep(fdev);
	xio_getc_eep(fdev);
}

#endif
