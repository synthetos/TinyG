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

// devices
extern struct xioDEVICE ds[XIO_DEV_MAX];	 // allocate top-level device structs
extern struct xioUSART us[XIO_DEV_USART_MAX];// allocate USART extended IO structs
#define DEV ds[dev]							 // device struct accessor
#define DEVx us[dev]						 // usart extenced struct accessor

// baud rate lookup tables - indexed by enum xioBAUDRATES (see xio_usart.h)
const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

// local functions
void _xio_set_baud_usart(const uint8_t dev, const uint8_t baud);
void _xio_set_flags(const uint8_t dev, const uint16_t control);

/* 
 *	xio_init_usart() - general purpose USART initialization (shared)
 */

void xio_init_usart(const uint8_t dev, 
					const uint16_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
//	struct xioDEVICE *d = &ds[dev];
//	struct xioUSART *u = (struct xioUSART *)ds[dev].xio;
	struct xioUSART *u = ds[dev].xio;

	// bind USART and PORT structures - do this first
	u->usart = (struct USART_struct *)usart_addr;
	u->port = (struct PORT_struct *)port_addr;

	// set flags
	_xio_set_flags(dev, control);			// usart must be bound first

	// character signals and error returns
//	d->sig = 0;
//	dev_usb.udata = &(u->sig);				// bind sig register to FILE struct

	// setup internal RX/TX buffers
	u->rx_buf_head = 1;						// can't use location 0
	u->rx_buf_tail = 1;
	u->tx_buf_head = 1;
	u->tx_buf_tail = 1;

	// baud rate and USART setup
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);
	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
	_xio_set_baud_usart(dev, baud);

	u->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	u->usart->CTRLA = CTRLA_RXON_TXON;				// enable tx and rx interrupts

	u->port->DIRCLR = dirclr;
	u->port->DIRSET = dirset;
	u->port->OUTCLR = outclr;
	u->port->OUTSET = outset;
}

/*	
 *	xio_control_usart() - set controls for USART devices
 */

void xio_control_usart(const uint8_t dev, const uint16_t control, const int16_t arg)
{
//	struct xioDEVICE *d = &ds[dev];
//	struct xioUSART *u = ds[dev].xio;

	// set baud rate
	if ((control & XIO_BAUD_gm) != XIO_BAUD_UNSPECIFIED) {
		_xio_set_baud_usart(dev, (control & XIO_BAUD_gm));
	}
	// set flags
	_xio_set_flags(dev, control);			// usart must be bound first
}

void _xio_set_baud_usart(const uint8_t dev, const uint8_t baud)
{
	struct xioUSART *u = ds[dev].xio;

	u->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	u->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
}

void _xio_set_flags(const uint8_t dev, const uint16_t control)
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

