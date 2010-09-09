/*
  xio.c - Xmega IO devices - common file
  Copyright (c) 2010 Alden S. Hart, Jr.

  To add a device:
  
  - Provide a xio_DEVICE.c and xio_DEVICE.h file (see xio_usb.c /.h for a model)

  - Include the following lines in this xio.c module:
	- add the include file, e.g.:#include "xio_usb.h"
  	- add its init to xio_init(), e.g. xio_usb_init(XIO_BAUD_115200);
*/

#include <stdio.h>
#include <avr/pgmspace.h>

#include "xio.h"				// put includes for all devices here
#include "xio_usb.h"
#include "xio_pgm.h"

uint16_t xio_signals;			// global for control char signal flags

/*
 * Combined initializations 
 */

void xio_init(void)
{	
	// USB port defaults are:	XIO_RDWR | XIO_BLOCK | XIO_ECHO | XIO_CRLF 
	xio_usb_init(XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200); // set in addition to defaults

	// PGM file defaults are:	XIO_RD | XIO_BLOCK
	xio_pgm_init(XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS);

	stdin = &dev_usb;				// define the console device
	stdout = &dev_usb;
	stderr = &dev_usb;

	stdin->udata = &xio_signals;	// bind xio_signals register to stdin user data

	fprintf(stdout, "\r\n\r\n**** Xmega IO subsystem initialized ****\r\n");
}

/*
 * Common stuff - used across multiple xio modules
 */

// USART table lookups

const uint8_t bsel[] PROGMEM = 				// baud rates. See xio.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

/*
 * fgets2() - A lame attempt to understand WTF is going on in fgets()
 */
/*
char * fgets2(char *b, int size, FILE *stream) {
	
	int i = 0;

	while ((b[i++] = fgetc(stream)) != '\n') {
		if (i == size) {
			return NULL;
		}
	}
	return (b);
}
*/
