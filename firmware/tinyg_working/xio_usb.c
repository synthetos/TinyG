/*
 * xio_usb.c	- FTDI USB device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the 
 * terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for details
 *
 * You should have received a copy of the GNU General Public License along with 
 * TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 *------
 *
 *	This version implements signal capture at the ISR level
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>				// needed for blocking character reads & writes

#include "xio.h"					// includes for all devices are in here
#include "xmega_interrupts.h"
#include "signals.h"

#define USB ds[XIO_DEV_USB]			// device struct accessoor
#define USBu us[XIO_DEV_USB_OFFSET]	// usart extended struct accessor

/* USB Device specific entry points to USART routines */
FILE * xio_open_usb() {return(USB.fdev);}
int xio_cntrl_usb(const uint16_t control) {return xio_cntrl(XIO_DEV_USB, control);} // SEE NOTE
int xio_putc_usb(const char c, FILE *stream) {return xio_putc_usart(XIO_DEV_USB, c, stream);}
int xio_getc_usb(FILE *stream) {return xio_getc_usart(XIO_DEV_USB, stream);}
int xio_gets_usb(char *buf, const uint8_t size) {return xio_gets_usart(XIO_DEV_USB, buf, size);}
void xio_queue_RX_char_usb(const char c) {xio_queue_RX_char_usart(XIO_DEV_USB, c);}
void xio_queue_RX_string_usb(const char *buf) {xio_queue_RX_string_usart(XIO_DEV_USB, buf);}

void xio_init_usb()	// USB inits
{
//	xio_init_dev(XIO_DEV_USB, xio_open_usb, xio_setflags_usb, xio_putc_usb, xio_getc_usb, xio_gets_usb);
	xio_init_dev(XIO_DEV_USB, xio_open_usb, xio_cntrl_usb, xio_putc_usb, xio_getc_usb, xio_gets_usb);
	xio_init_usart(XIO_DEV_USB, XIO_DEV_USB_OFFSET, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
}

// NOTE: Might later expand setflags() to validate control bits and return errors


/* 
 * USB_TX_ISR - USB transmitter interrupt (TX)
 *
 * The TX interrupt dilemma: TX interrupts occur when the USART DATA register is 
 * empty (and the ISR must disable interrupts when nothing's left to read, or they 
 * keep firing). If the TX buffer is completely empty (TXCIF is set) then enabling
 * interrupts does no good. The USART won't interrupt and the TX circular buffer 
 * never empties.
 *
 * So we define a dequeue function that can be called from either the ISR or be 
 * called from the putc() if it detects TXCIF. Care should be taken to make sure 
 * these two callers don't collide (like only enabling interrupts in putc() AFTER
 * the dequeue has occurred).
 */

ISR(USB_TX_ISR_vect)	//ISR(USARTC0_DRE_vect)	// USARTC0 data register empty
{
	if (USBu.tx_buf_head == USBu.tx_buf_tail) {	// buffer empty - disable ints
		USBu.usart->CTRLA = CTRLA_RXON_TXOFF;	// won't work if you just &= it
//		PMIC_DisableLowLevel(); 				// disable USART TX interrupts
		return;
	}
	if (!TX_MUTEX(USB.flags)) {
		if (--(USBu.tx_buf_tail) == 0) {		// advance tail and wrap 
			USBu.tx_buf_tail = TX_BUFFER_SIZE-1;// -1 avoids off-by-one err (OBOE)
		}
		USBu.usart->DATA = USBu.tx_buf[USBu.tx_buf_tail]; // write to TX DATA reg
	}
}

/* 
 * USB_RX_ISR - USB receiver interrupt (RX)
 *
 * RX buffer states can be one of:
 *	- buffer has space	(CTS should be asserted)
 *	- buffer is full 	(CTS should be not_asserted)
 *	- buffer becomes full with this character (write char and assert CTS)
 *
 * Signals:
 *	- Signals are captured at the ISR level and either dispatched or flag-set
 *	- As RX ISR is a critical code region signal handling is stupid and fast
 *	- signal characters are not put in the RX buffer
 *
 * Flow Control:
 *	- Flow control is not implemented. Need to work RTS line.
 *	- Flow control should cut off at high water mark, re-enable at low water mark
 *	- High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
 *	- Low water mark about 50% full
 *
 * 	See end notes in xio.h for a discussion of how the circular buffers work
 */

ISR(USB_RX_ISR_vect)	//ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	uint8_t c = USBu.usart->DATA;				// can only read DATA once

	// trap signals - do not insert character into RX queue
	if (c == SIG_KILL_CHAR) {	 				// trap Kill signal
		USB.sig = XIO_SIG_KILL;					// set signal value
		sig_kill();								// call app-specific sig handler
		return;
	}
	if (c == SIG_TERM_CHAR) {					// trap Terminate signal
		USB.sig = XIO_SIG_KILL;
		sig_term();
		return;
	}
	if (c == SIG_PAUSE_CHAR) {					// trap Pause signal
		USB.sig = XIO_SIG_PAUSE;
		sig_pause();
		return;
	}
	if (c == SIG_RESUME_CHAR) {					// trap Resume signal
		USB.sig = XIO_SIG_RESUME;
		sig_resume();
		return;
	}

	// normal character path
	if ((--USBu.rx_buf_head) == 0) { 			// advance buffer head with wrap
		USBu.rx_buf_head = RX_BUFFER_SIZE-1;	// -1 avoids the off-by-one error
	}
	if (USBu.rx_buf_head != USBu.rx_buf_tail) {	// write char unless buffer full
		USBu.rx_buf[USBu.rx_buf_head] = c;
		return;
	}
	// buffer-full handling
	if ((++USBu.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		USBu.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}

