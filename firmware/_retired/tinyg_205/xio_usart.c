/*
 * xio_usb.c	- FTDI USB device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
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
 * If not, see <http://www.gnu->org/licenses/>.
 *
 *------
 *
 *	This version implements signal capture at the ISR level
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "xio.h"				// includes for all devices are in here

// necessary structs
extern struct xioDEVICE ds[XIO_DEV_COUNT];		// allocate top-level device structs
extern struct xioUSART us[XIO_DEV_USART_COUNT];	// allocate USART extended IO structs
#define USX ((struct xioUSART *)(ds[dev].x))	// USART extended struct accessor

// baud rate lookup tables - indexed by enum xioBAUDRATES (see xio_usart.h)
const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

/* 
 *	xio_init_usart() - general purpose USART initialization (shared)
 */

void xio_init_usart(const uint8_t dev, 			// index into device array (ds)
					const uint8_t offset,		// index into USART array (us)
					const uint16_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
//	struct xioUSART *u = (struct xioUSART *)ds[dev].x; // example of a cast for FILEs
//	struct xioUSART *u = ds[dev].x;

	// bind USARt extended struct to device struct
	ds[dev].x = &us[offset];

	// bind USART and PORT structures - do this first
	USX->usart = (struct USART_struct *)usart_addr;
	USX->port = (struct PORT_struct *)port_addr;

	// set flags
	xio_setflags(dev, control);			// generic version. does not validate flags

	// setup internal RX/TX buffers
	USX->rx_buf_head = 1;					// can't use location 0
	USX->rx_buf_tail = 1;
	USX->tx_buf_head = 1;
	USX->tx_buf_tail = 1;

	// baud rate and USART setup
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);
	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
	xio_set_baud_usart(dev, baud);					// usart must be bound first

	USX->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm;// enable tx and rx on USART
	USX->usart->CTRLA = CTRLA_RXON_TXON;				// enable tx and rx interrupts

	USX->port->DIRCLR = dirclr;
	USX->port->DIRSET = dirset;
	USX->port->OUTCLR = outclr;
	USX->port->OUTSET = outset;
}

void xio_set_baud_usart(const uint8_t dev, const uint8_t baud)
{
	USX->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	USX->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
}
