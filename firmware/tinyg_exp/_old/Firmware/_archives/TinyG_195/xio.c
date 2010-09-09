/*
 * xio.c - Xmega IO devices - common file
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
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
#include <avr/pgmspace.h>

#include "xio.h"				// put includes for all devices here
#include "xio_usb.h"
#include "xio_pgm.h"
#include "tinyg.h"				// needed for TG_ return codes, or provide your own

/*
 * Common stuff - used across multiple xio modules
 */

/* USART table lookups */
const uint8_t bsel[] PROGMEM = 				// baud rates. See xio.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

/*
 * Common functions 
 *
 *	xio_init()
 */

void xio_init(void)
{	
	// USB port defaults are:	XIO_RDWR | XIO_ECHO | XIO_CRLF - open additionally:
	xio_usb_init(XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200);

	// PGM file defaults are:	XIO_RD | XIO_BLOCK
//	xio_pgm_init(XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS);
	xio_pgm_init(XIO_LINEMODE);

	stddev = &dev_usb;				// stddev is a convenience
	stdin = &dev_usb;				// define the console device
	stdout = &dev_usb;				// ...
	stderr = &dev_usb;				// ...

	printf_P(PSTR("\n\n**** Xmega IO subsystem initialized ****\n"));
}

/*
 * xio_dev_init() - common entry point for device init functions
 */

int8_t xio_dev_init(uint8_t dev, const int16_t arg)
{
	switch (dev) {
		case (XIO_DEV_NULL): return (TG_OK);
		case (XIO_DEV_USB): xio_usb_init(arg); return (TG_OK);
//		case (XIO_DEV_AUX): xio_aux_init(arg); return (TG_OK);
//		case (XIO_DEV_RS485): xio_rs485_init(arg); return (TG_OK);
		case (XIO_DEV_PGM): xio_pgm_init(arg); return (TG_OK);
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_GENERIC_ERROR);		// never should hit this
}


/*
 * xio_control() - common entry point for device control functions
 */

int8_t xio_control(uint8_t dev, const uint16_t control, const int16_t arg)
{
	switch (dev) {
		case (XIO_DEV_NULL): return (TG_OK);
		case (XIO_DEV_USB): return (xio_usb_control(control, arg));
//		case (XIO_DEV_AUX): return (xio_aux_control(control, arg));
//		case (XIO_DEV_RS485): return (xio_rs485_control(control, arg));
		case (XIO_DEV_PGM): return (xio_pgm_control(control, arg));
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_GENERIC_ERROR);		// never should hit this
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
	switch (dev) {
		case (XIO_DEV_NULL): return (TG_OK);
		case (XIO_DEV_USB): return (xio_usb_readln(buf, len));
//		case (XIO_DEV_AUX): return (xio_aux_readln(buf, len));
//		case (XIO_DEV_RS485): return (xio_rs485_readln(buf, len));
		case (XIO_DEV_PGM): return (xio_pgm_readln(buf, len));
		default: return (TG_UNRECOGNIZED_DEVICE);
	}
	return (TG_GENERIC_ERROR);		// never should hit this
}

/*
 * xio_null_signal() - signal handler with no effect
 */

int xio_null_signal(uint8_t sig)
{
	return (TG_CONTINUE);
}

/*
 * xio_null_line() - null line handler with no effect
 *
 *	Useful for keeping an input device open for receiving signals (but not lines)
 *	Relies on the readln to reset the line buffer
 * 	Always returns a continue because the line is never "done"
 */

int xio_null_line(char * buf)
{
	return (TG_CONTINUE);
}

