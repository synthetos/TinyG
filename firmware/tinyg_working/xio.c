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
 *
 * -----
 * Xmega IO devices made compatible with avr-gcc stdio
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

struct xioDEVICE ds[XIO_DEV_CNT];		// allocate top-level device structs
struct xioUSART us[XIO_DEV_USART_CNT];	// allocate USART extended IO structs
struct xioFILE fs[XIO_DEV_FILE_CNT];	// allocate FILE extended IO structs

#define dev_rs485 (ds[XIO_DEV_RS485].fs)// RS485 device for stdio functions
#define dev_usb (ds[XIO_DEV_USB].fs)	// USB device for stdio functions
#define dev_pgm (ds[XIO_DEV_PGM].fs)	// Program memory device

/*
 *	xio_init() - initialize all active XIO devices
 */

void xio_init(void)
{	
	uint8_t i=0;

	// RS485 setup
	i = XIO_DEV_RS485;
	memset (&ds[i], 0, sizeof(struct xioDEVICE));
	ds[i].xio = &us[i];							// bind USART extended struct
//	ds[i].size = sizeof(ds[i].buf);
	xio_init_usart(i, RS485_INIT_bm, &RS485_USART, &RS485_PORT, 
		RS485_DIRCLR_bm,RS485_DIRSET_bm,RS485_OUTCLR_bm,RS485_OUTSET_bm);
	ds[i].dev_putc = &xio_putc_rs485;
	ds[i].dev_getc = &xio_getc_rs485;
	ds[i].dev_readln = &xio_readln_rs485;
	fdev_setup_stream(&(ds[i].fs), xio_putc_rs485, xio_getc_rs485, _FDEV_SETUP_RW);

	// USB setup
	i = XIO_DEV_USB;
	memset (&ds[i], 0, sizeof(struct xioDEVICE));
	ds[i].xio = &us[i];							// bind USART extended struct
//	ds[i].size = sizeof(ds[i].buf);
	xio_init_usart(i, USB_INIT_bm, &USB_USART, &USB_PORT, 
		USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
	ds[i].dev_putc = &xio_putc_usb;
	ds[i].dev_getc = &xio_getc_usb;
	ds[i].dev_readln = &xio_readln_usb;
	fdev_setup_stream(&(ds[i].fs), xio_putc_usb, xio_getc_usb, _FDEV_SETUP_RW);

	// Program Memory File device setup
	i = XIO_DEV_PGM;
	memset (&ds[i], 0, sizeof(struct xioDEVICE));
	ds[i].xio = &fs[XIO_DEV_FILE_LO];			// bind lowest FILE extended struct
//	ds[i].size = sizeof(ds[i].buf);
	xio_init_pgm(PGM_INIT_bm);
	ds[i].dev_putc = &xio_putc_pgm;
	ds[i].dev_getc = &xio_getc_pgm;
	ds[i].dev_readln = &xio_readln_usb;
	fdev_setup_stream(&(ds[i].fs), xio_putc_usb, xio_getc_usb, _FDEV_SETUP_RW);



	// do stdio bindings
	stddev = &dev_usb;				// stddev is a convenience
	stdin = &dev_usb;				// define the console device
	stdout = &dev_usb;				// ...
	stderr = &dev_usb;				// ...

	printf_P(PSTR("\n\n**** Xmega IO subsystem initialized ****\n"));
}

/*
 * xio_dev_init() - common entry point for device init functions
 */
/*
int8_t xio_dev_init(uint8_t dev, const int16_t arg)
{
	switch (dev) {
		case (XIO_DEV_RS485): xio_rs485_init(arg); return (TG_OK);
		case (XIO_DEV_USB): xio_usb_init(arg); return (TG_OK);
//		case (XIO_DEV_AUX): xio_aux_init(arg); return (TG_OK);
		case (XIO_DEV_PGM): xio_pgm_init(arg); return (TG_OK);
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_ERROR);		// never should hit this
}
*/

/*
 * xio_set_control_flags()
 */

void xio_set_control_flags(const uint8_t dev, const uint16_t control)
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
//		case (XIO_DEV_AUX): return (xio_readln_aux(buf, len));
//		case (XIO_DEV_PGM): return (xio_readln_pgm(buf, len));
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_ERROR);		// never should hit this
}
