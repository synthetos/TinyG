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
//#include "xio_file.h"			// common declarations for file-type devices
#include "tinyg.h"				// needed for TG_ return codes, or provide your own


/*
 * structs and memory allocation
 */

struct xioDEVICE ds[XIO_DEV_MAX];		// allocate top-level device structs
struct xioUSART us[XIO_DEV_USART_MAX];	// allocate USART extended IO structs
//struct xioFILE fs[XIO_DEV_FILE_MAX];	// allocate FILE extended IO structs
struct __file __files[XIO_DEV_MAX];		// allocate a stdio __file for each device

//#define DEV ds[XIO_DEV_USB]				// device struct accessoor
//#define USx us[XIO_DEV_USB]				// usart extended struct accessor
//#define FSx fs

/*
 *	xio_init() - initialize all active XIO devices
 */

void xio_init(void)
{	
	uint8_t i=0;

	// RS485
	i = XIO_DEV_RS485;
	memset (&ds[i], 0, sizeof(struct xioDEVICE));
	ds[i].xio = &us[i];								// bind USART extended struct
	ds[i].size = sizeof(ds[i].buf);
	xio_init_usart(i, RS485_INIT_bm, &RS485_USART, &RS485_PORT, 
				   RS485_DIRCLR_bm,RS485_DIRSET_bm,RS485_OUTCLR_bm,RS485_OUTSET_bm);
//	ds[i].dev_flags = &xio_set_control_flags_rs485;
	ds[i].dev_putc = &xio_putc_rs485;
//	void (*dev_flags)(uint16_t control);// set device control flags
//	int (*dev_putc)(char, struct __file *);	// write char (stdio compatible)
//	int (*dev_getc)(struct __file *);	// read char (stdio compatible)
//	int (*dev_readln)();				// specialized line reader


//	FILE __files[i] = FDEV_SETUP_STREAM(xio_putc_rs485, xio_getc_rs485, _FDEV_SETUP_RW);

	// USB
	i = XIO_DEV_USB;
	memset (&ds[i], 0, sizeof(struct xioDEVICE));
	ds[i].xio = &us[i];								// bind USART extended struct
	ds[i].size = sizeof(ds[i].buf);
	xio_init_usart(i, USB_INIT_bm, &USB_USART, &USB_PORT, 
				   USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
//	ds[i].dev_control = &xio_control_usart;
	
//	FILE __files[i] = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);
//FILE dev_usb = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);

	
	// character signals and error returns
//	d->sig = 0;
//	dev_usb.udata = &(u->sig);				// bind sig register to FILE struct



	// setup stdio resources
//	for (uint8_t i=0; i < XIO_DEV_MAX; i++) {
//		FILE __files[i] = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);


	// brute force the pointer initializations
//	for (uint8_t i=0; i < XIO_DEV_USART_MAX; i++) {
//		ds[i] = &us[i];
//		xio_init_usart(i, ds[i], pgm_read_word(&ucontrols[i]));
//	}
//	ds[XIO_DEV_PGM] = (struct xioUSART *)(&fpgm);
//	xio_pgm_init(PGM_INIT_bm);

	// do stdio bindings
//	stddev = &dev_usb;				// stddev is a convenience
//	stdin = &dev_usb;				// define the console device
//	stdout = &dev_usb;				// ...
//	stderr = &dev_usb;				// ...

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
 * xio_fget_ln() - common entry point for non-blocking receive line functions
 *
 * Arguments
 *	buf		text buffer to read into
 *	len		length of text buffer in 1 offset form: e.g. use 80 instead of 79
 *	dev		XIO device enumeration
 */

int xio_fget_ln(uint8_t dev, char *buf, uint8_t len)
{
/*
	switch (dev) {
		case (XIO_DEV_RS485): return (xio_rs485_readln(buf, len));
		case (XIO_DEV_USB): return (xio_usb_readln(buf, len));
//		case (XIO_DEV_AUX): return (xio_aux_readln(buf, len));
		case (XIO_DEV_PGM): return (xio_pgm_readln(buf, len));
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
*/
	return (TG_ERROR);		// never should hit this
}
