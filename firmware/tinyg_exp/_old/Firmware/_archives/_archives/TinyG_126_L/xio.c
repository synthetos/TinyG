/*
  xio.c - Xmega IO devices - common file
  Copyright (c) 2010 Alden S. Hart, Jr.

  To add a device:
  
  - Provide a xio_DEVICE.c and xio_DEVICE.h file - see xio_sub.c /.h as a model

  - Include the following lines in this xio.c module:

	- add the include file, e.g.:
		#include "xio_usb.h"

  	- declare and set up the device, e.g.:
		FILE usb_str = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);

  	- intialization in xio_init(), e.g.:
		xio_usb_init(XIO_BAUD_115200);		// or whatever controls you want

*/

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"
#include "xio_usb.h"				// put includes for all devices here

/*
 * stdio FILE setups - include FILE setups for all devices here
 */

FILE usb_str = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);

/*
 * Combined initializations 
 */

void xio_init(void)
{
	xio_usb_init(XIO_BAUD_115200);
}

/*
 * Common stuff - used across multiple xio modules
 */

// USART table lookups

const uint8_t bsel[] PROGMEM = 				// baud rates. See xio.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

