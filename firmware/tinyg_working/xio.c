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

#include "xio.h"				// put includes for all devices here
#include "xio_usart.h"			// common declarations for USART devices
#include "xio_file.h"			// common declarations for file-type devices
#include "tinyg.h"				// needed for TG_ return codes, or provide your own


/*
 * structs, static memory allocation, and accessors
 */

struct xioDEVICE ds[XIO_DEV_COUNT];		// allocate top-level device structs
struct xioUSART us[XIO_DEV_USART_COUNT];// allocate USART extended IO structs
struct xioFILE fs[XIO_DEV_FILE_COUNT];	// allocate FILE extended IO structs
struct __file ss[XIO_DEV_COUNT];		// allocate stdio stream for each dev

// aliases for stdio devices (pointers)
#define fdev_rs485 (ds[XIO_DEV_RS485].fdev)// RS485 device for stdio functions
#define fdev_usb (ds[XIO_DEV_USB].fdev)	// USB device for stdio functions
#define fdev_pgm (ds[XIO_DEV_PGM].fdev)	// Program memory device

/*
 *	xio_init() - initialize all active XIO devices
 */

void xio_init(void)
{	
	// RS485 device setup (brute force!)
	memset (&ds[XIO_DEV_RS485], 0, sizeof(struct xioDEVICE));	// clear dev struct
	ds[XIO_DEV_RS485].x = &us[XIO_DEV_RS485_OFFSET];			// bind USART struct
	ds[XIO_DEV_RS485].fdev = &ss[XIO_DEV_RS485];				// bind stdio struct
	xio_init_usart(XIO_DEV_RS485, RS485_INIT_bm, &RS485_USART, &RS485_PORT, 
		RS485_DIRCLR_bm,RS485_DIRSET_bm,RS485_OUTCLR_bm,RS485_OUTSET_bm);
	fdev_setup_stream(ds[XIO_DEV_RS485].fdev, 
		xio_putc_rs485, xio_getc_rs485, _FDEV_SETUP_RW);
	ds[XIO_DEV_RS485].dev_open = &xio_open_rs485;				// function bindings
	ds[XIO_DEV_RS485].dev_setflags = &xio_setflags_rs485;
	ds[XIO_DEV_RS485].dev_putc = &xio_putc_rs485;
	ds[XIO_DEV_RS485].dev_getc = &xio_getc_rs485;
	ds[XIO_DEV_RS485].dev_readln = &xio_readln_rs485;

	// USB device setup
	memset (&ds[XIO_DEV_USB], 0, sizeof(struct xioDEVICE));		// clear dev struct
	ds[XIO_DEV_USB].x = &us[XIO_DEV_USB_OFFSET];				// bind USART struct
	ds[XIO_DEV_USB].fdev = &ss[XIO_DEV_USB];					// bind stdio struct
	xio_init_usart(XIO_DEV_USB, USB_INIT_bm, &USB_USART, &USB_PORT, 
		USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
	fdev_setup_stream(ds[XIO_DEV_USB].fdev, 
		xio_putc_usb, xio_getc_usb, _FDEV_SETUP_RW);
	ds[XIO_DEV_USB].dev_open = &xio_open_usb;
	ds[XIO_DEV_USB].dev_setflags = &xio_setflags_usb;
	ds[XIO_DEV_USB].dev_putc = &xio_putc_usb;
	ds[XIO_DEV_USB].dev_getc = &xio_getc_usb;
	ds[XIO_DEV_USB].dev_readln = &xio_readln_usb;

	// Program memory file device setup
	memset (&ds[XIO_DEV_PGM], 0, sizeof(struct xioDEVICE));		// clear dev struct
	ds[XIO_DEV_PGM].x = &fs[XIO_DEV_PGM_OFFSET];				// bind FILE struct
	ds[XIO_DEV_PGM].fdev = &ss[XIO_DEV_PGM];					// bind stdio struct
	xio_init_pgm(PGM_INIT_bm);
	fdev_setup_stream(ds[XIO_DEV_PGM].fdev, 
		xio_putc_pgm, xio_getc_pgm, _FDEV_SETUP_RW);
	ds[XIO_DEV_PGM].dev_open = &xio_open_pgm;
	ds[XIO_DEV_PGM].dev_setflags = &xio_setflags_pgm;
	ds[XIO_DEV_PGM].dev_putc = &xio_putc_pgm;
	ds[XIO_DEV_PGM].dev_getc = &xio_getc_pgm;
	ds[XIO_DEV_PGM].dev_readln = &xio_readln_pgm;

	// setup stdio bindings to default IO device
	stdin = fdev_usb;				// define the console device
	stdout = fdev_usb;				// ...
	stderr = fdev_usb;				// ...

	// tell the world we are ready!
	printf_P(PSTR("\n\n**** Xmega IO subsystem initialized ****\n"));
}

/*
 * xio_set_control_flags()
 */

void xio_setflags(const uint8_t dev, const uint16_t control)
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
}

/*
 * xio_readln() - common entry point for non-blocking receive line functions
 *
 * Arguments
 *	buf		text buffer to read into
 *	len		length of text buffer in 1 offset form: e.g. use 80 instead of 79
 *	dev		XIO device enumeration
 */

int xio_readln(uint8_t dev, char *buf, uint8_t len)
{

	switch (dev) {
		case (XIO_DEV_RS485): return (xio_readln_rs485(buf, len));
		case (XIO_DEV_USB): return (xio_readln_usb(buf, len));
//		case (XIO_DEV_TTL): return (xio_readln_aux(buf, len));
		case (XIO_DEV_PGM): return (xio_readln_pgm(buf, len));
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_ERROR);		// never should hit this
}

