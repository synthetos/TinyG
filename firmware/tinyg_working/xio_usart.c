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
//#include "xmega_interrupts.h"
//#include "tinyg.h"				// needed for TG_ return codes, or provide your own
//#include "signals.h"

void xio_init_usart(uint8_t d, struct xioUSART *u, const uint16_t control);


// these tables are derived from above - shouldn't need to change it
const struct USART_struct *usart_addr[] PROGMEM = { &RS4_USART, &USB_USART, &TTL_USART };
const struct PORT_struct  *port_addr[] PROGMEM = { &RS4_PORT, &USB_PORT, &TTL_PORT };
const uint8_t dirclr_bm [] PROGMEM = { RS4_DIRCLR_bm, USB_DIRCLR_bm, TTL_DIRCLR_bm };
const uint8_t dirset_bm [] PROGMEM = { RS4_DIRSET_bm, USB_DIRSET_bm, TTL_DIRSET_bm };
const uint8_t outclr_bm [] PROGMEM = { RS4_OUTCLR_bm, USB_OUTCLR_bm, TTL_OUTCLR_bm };
const uint8_t outset_bm [] PROGMEM = { RS4_OUTSET_bm, USB_OUTSET_bm, TTL_OUTSET_bm };

// change these control word initializations as you need to
const uint16_t controls[] PROGMEM = {
	(XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_BAUD_115200),	// RS485
	(XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200),
	(XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200)
};

// baud rate lookup tables - indexed by enum xioBAUDRATES (see xio_usart.h)
const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

// allocate an array of usart device control structs
struct xioUSART us[XIO_DEV_USART_MAX];

/* 
 *	xio_init_usart() - general purpose USART initialization (shared)
 */

void xio_init_usart(uint8_t dev, struct xioUSART *u, const uint16_t control)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// transfer control flags to internal flag bits
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
//	u->len = sizeof(u->buf);				// offset to zero THIS IS WRONG. NO BUFFER YET

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
	u->usart->CTRLA = CTRLA_RXON_TXON;				// enable tx and rx interrupts

	u->port->DIRCLR = (uint8_t)pgm_read_byte(&dirclr_bm[dev]);
	u->port->DIRSET = (uint8_t)pgm_read_byte(&dirset_bm[dev]);
	u->port->OUTCLR = (uint8_t)pgm_read_byte(&outclr_bm[dev]);
	u->port->OUTSET = (uint8_t)pgm_read_byte(&outset_bm[dev]);
}
