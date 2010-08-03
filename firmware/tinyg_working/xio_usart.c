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

#include "xio.h"
#include "xio_usart.h"
//#include "xio_usb.h"
//#include "xmega_interrupts.h"
//#include "tinyg.h"				// needed for TG_ return codes, or provide your own
//#include "signals.h"

void xio_init_usart(uint8_t d, struct xioUSART *u, const uint16_t control);


/* 
 * USART config tables
 */

/* device settings. These must be aligned with enum xioDevice
 *							 [ct0, ct1 pin usage]
 *	0 = null device
 *	1 = rs485 device		 [ct0 = RE (Receive Enable) pin - active lo]
 *							 [ct1 = DE (Data Enable) pin (TX enable) - active hi]
 *	2 = USB device			 [ct0 = CTS pin, ct1 = RTS pin]
 *	3 = aux device (Arduino) [not used]
 */
const struct USART_struct *usart_addr[] PROGMEM = { 0, &USARTC1, &USARTC0, &USARTC0 };
const struct PORT_struct *port_addr[] PROGMEM = { 0, &PORTC, &PORTC, &PORTC};
const uint8_t pin_rx_bm[] PROGMEM = { (1<<6), (1<<2), (1<<2) };
const uint8_t pin_tx_bm[] PROGMEM = { (1<<7), (1<<3), (1<<3) };
const uint8_t pin_ct0_bm[] PROGMEM = { (1<<4), (1<<0), (1<<0) };
const uint8_t pin_ct1_bm[] PROGMEM = { (1<<5), (1<<1), (1<<1) };

// baud rate data, indexed by enum xioBAUDRATES (see xio_usart.h)
const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };


// Crazy code. Must be aligned with the structure definition, obviously

struct xioUSART us[XIO_DEV_USART_MAX] = {
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, &USARTC1, &PORTC},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, &USARTC0, &PORTC},
	{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, &USARTC0, &PORTC}
};

/* 
 *	xio_init_usarts() - initialize all USART devices
 */

void xio_init_usarts() 
{
//	xio_init_usart(2, *u, const uint16_t control)
	return;
}

/* 
 *	xio_init_usart() - general purpose USART initialization (shared)
 */

void xio_init_usart(uint8_t dev, struct xioUSART *u, const uint16_t control)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// transfer control flags to internal flag bits
//	u->flags = XIO_FLAG_USB_DEFS_gm;		// set flags to defaults & initial state
	if (control & XIO_RD) {
		u->flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_WR) {
		u->flags |= XIO_FLAG_WR_bm;
	}
	if (control & XIO_BLOCK) {
		u->flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		u->flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		u->flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		u->flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		u->flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		u->flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		u->flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		u->flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		u->flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		u->flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	// character signals and error returns
	u->sig = 0;
//	dev_usb.udata = &(u->sig);				// bind sig register to FILE struct

	// setup internal RX/TX buffers
	u->rx_buf_head = 1;						// can't use location 0
	u->rx_buf_tail = 1;
	u->tx_buf_head = 1;
	u->tx_buf_tail = 1;
	u->len = sizeof(u->buf);				// offset to zero

//	u->usart = &USB_USART;					// bind USART structure
//	u->port = &USB_PORT;					// bind PORT structure

	// bind USART and PORT structures to xioUSART struct
	u->usart = (struct USART_struct *)pgm_read_word(&usart_addr[dev]);
	u->port = (struct PORT_struct *)pgm_read_word(&port_addr[dev]);

	// baud rate and USART setup
	if (baud == XIO_BAUD_UNSPECIFIED) {
		baud = XIO_BAUD_DEFAULT;
	}
	u->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	u->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	u->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
/*
	u->usart->CTRLA = USARTC0_CTRLA_RXON_TXON;		// enable tx and rx interrupts

	u->port->DIRCLR = USB_RX_bm;	 			// clr RX pin as input
	u->port->DIRSET = USB_TX_bm; 			// set TX pin as output
	u->port->OUTSET = USB_TX_bm;				// set TX HI as initial state
	u->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
	u->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
	u->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
//	u->port->OUTCLR = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
*/
}
