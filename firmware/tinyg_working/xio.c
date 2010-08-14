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
 *		- readln (non-blocking inout line reader - extends fgets functionality)
 *		- setflags (ioctl-like knockoff for setting device parameters)
 *		- signal handling - captures ^c, pause, resume, etc. as interrupts
 *		- interrupt buffered RX and TX functions 
 *
 * ---- To add a device ----
 *
 * Provide a xio_DEVICE.c and xio_DEVICE.h file (see xio_usb.c /.h for a model)
 *
 *	Include the following lines in this xio.c module:
 *		- add the include file, e.g.:#include "xio_usb.h"
 *		- add its init to xio_init(), e.g. xio_usb_init(XIO_BAUD_115200);
 *		- add the new device to the device enum in xio.h
 *		- add the new device to xio_control() and xio_fget_ln() funcs in this file
 *
 *	For further notes see the end of xio.h
 *
 */

#include <stdio.h>
#include <string.h>				// for memset()
#include <avr/pgmspace.h>
#include "xio.h"				// includes for all devices are in here

/*
 * structs, static memory allocation, and accessors
 */

struct xioDEVICE ds[XIO_DEV_COUNT];		// allocate top-level device structs
struct xioUSART us[XIO_DEV_USART_COUNT];// allocate USART extended IO structs
struct xioFILE fs[XIO_DEV_FILE_COUNT];	// allocate FILE extended IO structs
struct __file ss[XIO_DEV_COUNT];		// allocate stdio stream for each dev

/*
 *	xio_init() - initialize all active XIO devices
 */

void xio_init(void)
{	
	xio_init_rs485();
	xio_init_usb();

	// Program memory file device setup
	xio_init_dev(XIO_DEV_PGM, xio_open_pgm, xio_setflags_pgm, xio_putc_pgm, xio_getc_pgm, xio_readln_pgm);
	xio_init_pgm(XIO_DEV_PGM, XIO_DEV_PGM_OFFSET, PGM_INIT_bm);

	// setup stdio bindings to default IO device
	xio_set_stdin(XIO_DEV_USB);
	xio_set_stdout(XIO_DEV_USB);
	xio_set_stderr(XIO_DEV_USB);

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
	FILE *(*dev_open)(const prog_char *addr),	// device open routine
	int (*dev_setflags)(const uint16_t control),// set device control flags
	int (*dev_putc)(char, struct __file *),		// write char (stdio compatible)
	int (*dev_getc)(struct __file *),			// read char (stdio compatible)
	int (*dev_readln)(char *buf, uint8_t size)	// specialized line reader
	) 
{
	// clear device struct
	memset (&ds[dev], 0, sizeof(struct xioDEVICE));	

	// bind functions
	ds[dev].dev_open = dev_open;						
	ds[dev].dev_setflags = dev_setflags;
	ds[dev].dev_putc = dev_putc;
	ds[dev].dev_getc = dev_getc;
	ds[dev].dev_readln = dev_readln;

	// bind and setup stdio struct
	ds[dev].fdev = &ss[dev];					
	fdev_setup_stream(ds[dev].fdev, dev_putc, dev_getc, _FDEV_SETUP_RW);
}

/*
 * xio_setflags()
 */

int xio_setflags(const uint8_t dev, const uint16_t control)
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
 * xio_set_std___() - functions to set standard IO devices from device numbers
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
		return ds[dev].dev_putc(c, ds[dev].fdev);
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
		return ds[dev].dev_getc(ds[dev].fdev);
	} else {
		return (XIO_NO_SUCH_DEVICE);
	}		
}

/*
 * xio_readln() - common entry point for non-blocking receive line functions
 *
 * Arguments
 *	dev		XIO device enumeration
 *	buf		text buffer to read into
 *	size	size of text buffer in 1 offset form: e.g. use 80 instead of 79
 */

int xio_readln(const uint8_t dev, char *buf, const uint8_t size)
{
	if (dev < XIO_DEV_COUNT) {
		return ds[dev].dev_readln(buf, size);
	} else {
		return (XIO_NO_SUCH_DEVICE);
	}		
}
