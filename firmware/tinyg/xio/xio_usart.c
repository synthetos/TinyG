/*
 * xio_usart.c	- General purpose USART device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <string.h>						// for memset
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking character writes

#include "../xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

#include "../tinyg.h"					// needed for AXES definition
#include "../gpio.h"					// needed for XON/XOFF LED indicator
#include "../util.h"					// needed to pick up __debug defines
#include "../config.h"					// needed to write back usb baud rate

/******************************************************************************
 * USART CONFIGURATION RECORDS
 ******************************************************************************/
// See xio_usart.h for baud rate configuration settings
static const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
static const uint8_t bscale[] PROGMEM = { 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

struct cfgUSART {
		x_open_t x_open;
		x_ctrl_t x_ctrl;
		x_gets_t x_gets;
		x_getc_t x_getc;
		x_putc_t x_putc;
		x_flow_t x_flow;
		USART_t *usart;			// usart binding
		PORT_t *port;			// port binding
		uint8_t baud;
		uint8_t inbits;
		uint8_t outbits;
		uint8_t outclr;
		uint8_t outset;
};

static struct cfgUSART const cfgUsart[] PROGMEM = {
	{
		xio_open_usart,			// USB config record
		xio_ctrl_generic,
		xio_gets_usart,
		xio_getc_usart,
		xio_putc_usb,
		xio_fc_usart,
		&USB_USART,
		&USB_PORT,
		USB_BAUD,
		USB_INBITS_bm,
		USB_OUTBITS_bm,
		USB_OUTCLR_bm,
		USB_OUTSET_bm
	},
	{
		xio_open_usart,			// RS485 config record
		xio_ctrl_generic,
		xio_gets_usart,
		xio_getc_usart,
		xio_putc_rs485,
		xio_fc_null,
		&RS485_USART,
		&RS485_PORT,
		RS485_BAUD,
		RS485_INBITS_bm,
		RS485_OUTBITS_bm,
		RS485_OUTCLR_bm,
		RS485_OUTSET_bm
	}
};

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/

static int _gets_helper(xioDev_t *d, xioUsart_t *dx);

/*
 *	xio_init_usart() - general purpose USART initialization (shared)
 */
void xio_init_usart(void)
{
	for (uint8_t i=0; i<XIO_DEV_USART_COUNT; i++) {
		xio_open_generic(XIO_DEV_USART_OFFSET + i,
						(x_open_t)pgm_read_word(&cfgUsart[i].x_open),
						(x_ctrl_t)pgm_read_word(&cfgUsart[i].x_ctrl),
						(x_gets_t)pgm_read_word(&cfgUsart[i].x_gets),
						(x_getc_t)pgm_read_word(&cfgUsart[i].x_getc),
						(x_putc_t)pgm_read_word(&cfgUsart[i].x_putc),
						(x_flow_t)pgm_read_word(&cfgUsart[i].x_flow));
	}
}

/*
 *	xio_open_usart() - general purpose USART open (shared)
 *	xio_set_baud_usart() - baud rate setting routine
 */
FILE *xio_open_usart(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = &ds[dev];							// setup device struct pointer
	uint8_t idx = dev - XIO_DEV_USART_OFFSET;
	d->x = &us[idx];								// bind extended struct to device
	xioUsart_t *dx = (xioUsart_t *)d->x;

	memset (dx, 0, sizeof(xioUsart_t));				// clear all values
	xio_reset_working_flags(d);
	xio_ctrl_generic(d, flags);						// setup control flags
	if (d->flag_xoff) {								// initialize flow control settings
		dx->fc_state_rx = FC_IN_XON;
		dx->fc_state_tx = FC_IN_XON;
	}

	// setup internal RX/TX control buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;
	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;

	// baud rate and USART setup (do this last)
	dx->usart = (USART_t *)pgm_read_word(&cfgUsart[idx].usart);
	dx->port = (PORT_t *)pgm_read_word(&cfgUsart[idx].port);
	uint8_t baud = (uint8_t)pgm_read_byte(&cfgUsart[idx].baud);
	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
	xio_set_baud_usart(dx, baud);						// usart must be bound first
	dx->port->DIRCLR = (uint8_t)pgm_read_byte(&cfgUsart[idx].inbits);
	dx->port->DIRSET = (uint8_t)pgm_read_byte(&cfgUsart[idx].outbits);
	dx->port->OUTCLR = (uint8_t)pgm_read_byte(&cfgUsart[idx].outclr);
	dx->port->OUTSET = (uint8_t)pgm_read_byte(&cfgUsart[idx].outset);
	dx->usart->CTRLB = (USART_TXEN_bm | USART_RXEN_bm);	// enable tx and rx
	dx->usart->CTRLA = CTRLA_RXON_TXON;					// enable tx and rx IRQs

	dx->port->USB_CTS_PINCTRL = PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	dx->port->INTCTRL = USB_CTS_INTLVL;		// see xio_usart.h for setting
	dx->port->USB_CTS_INTMSK = USB_CTS_bm;

	return (&d->file);		// return FILE reference

	// here's a bag for the RS485 device
	if (dev == XIO_DEV_RS485) {
		xio_enable_rs485_rx();							// sets RS-485 to RX mode initially
	}
}

void xio_set_baud_usart(xioUsart_t *dx, const uint8_t baud)
{
	dx->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	dx->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	cfg.usb_baud_rate = baud;
}

/*
 * USART flow control functions and helpers
 *
 * xio_xoff_usart() - send XOFF flow control for USART devices
 * xio_xon_usart()  - send XON flow control for USART devices
 * xio_fc_usart() - Usart device flow control callback
 * xio_get_tx_bufcount_usart() - returns number of chars in TX buffer
 * xio_get_rx_bufcount_usart() - returns number of chars in RX buffer
 *
 *	Reminder: tx/rx queues fill from top to bottom, w/0 being the wrap location
 */

void xio_xoff_usart(xioUsart_t *dx)
{
	if (dx->fc_state_rx == FC_IN_XON) {
		dx->fc_state_rx = FC_IN_XOFF;

		// If using XON/XOFF flow control
		if (cfg.enable_flow_control == FLOW_CONTROL_XON) {
			dx->fc_char_rx = XOFF;
			dx->usart->CTRLA = CTRLA_RXON_TXON;		// force a TX interrupt
		}

		// If using hardware flow control. The CTS pin on the *FTDI* is our RTS.
		// Logic 1 means we're NOT ready for more data.
		if (cfg.enable_flow_control == FLOW_CONTROL_RTS) {
			dx->port->OUTSET = USB_RTS_bm;
		}
	}
}

void xio_xon_usart(xioUsart_t *dx)
{
	if (dx->fc_state_rx == FC_IN_XOFF) {
		dx->fc_state_rx = FC_IN_XON;

		// If using XON/XOFF flow control
		if (cfg.enable_flow_control == FLOW_CONTROL_XON) {
			dx->fc_char_rx = XON;
			dx->usart->CTRLA = CTRLA_RXON_TXON;		// force a TX interrupt
		}

		// If using hardware flow control. The CTS pin on the *FTDI* is our RTS.
		// Logic 0 means we're ready for more data.
		if (cfg.enable_flow_control == FLOW_CONTROL_RTS) {
			dx->port->OUTCLR = USB_RTS_bm;
		}
	}
}

void xio_fc_usart(xioDev_t *d)		// callback from the usart handlers
{
	xioUsart_t *dx = d->x;
	if (xio_get_rx_bufcount_usart(dx) < XOFF_RX_LO_WATER_MARK) {
		xio_xon_usart(dx);
	}
}

buffer_t xio_get_tx_bufcount_usart(const xioUsart_t *dx)
{
	if (dx->tx_buf_head <= dx->tx_buf_tail) {
		return (dx->tx_buf_tail - dx->tx_buf_head);
	} else {
		return (TX_BUFFER_SIZE - (dx->tx_buf_head - dx->tx_buf_tail));
	}
}

buffer_t xio_get_rx_bufcount_usart(const xioUsart_t *dx)
{
//	return (dx->rx_buf_count);
	if (dx->rx_buf_head <= dx->rx_buf_tail) {
		return (dx->rx_buf_tail - dx->rx_buf_head);
	} else {
		return (RX_BUFFER_SIZE - (dx->rx_buf_head - dx->rx_buf_tail));
	}
}

/*
 *	xio_gets_usart() - read a complete line from the usart device
 * _gets_helper() 	 - non-blocking character getter for gets
 *
 *	Retains line context across calls - so it can be called multiple times.
 *	Reads as many characters as it can until any of the following is true:
 *
 *	  - RX buffer is empty on entry (return XIO_EAGAIN)
 *	  - no more chars to read from RX buffer (return XIO_EAGAIN)
 *	  - read would cause output buffer overflow (return XIO_BUFFER_FULL)
 *	  - read returns complete line (returns XIO_OK)
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: This function assumes ignore CR and ignore LF handled upstream before the RX buffer
 */
int xio_gets_usart(xioDev_t *d, char *buf, const int size)
{
	xioUsart_t *dx = d->x;						// USART pointer

	if (d->flag_in_line == false) {				// first time thru initializations
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->len = 0;								// zero buffer
		d->buf = buf;
		d->size = size;
		d->signal = XIO_SIG_OK;					// reset signal register
	}
	while (true) {
		switch (_gets_helper(d,dx)) {
			case (XIO_BUFFER_EMPTY): return (XIO_EAGAIN); // empty condition
			case (XIO_BUFFER_FULL): return (XIO_BUFFER_FULL);// overrun err
			case (XIO_EOL): return (XIO_OK);			  // got complete line
			case (XIO_EAGAIN): break;					  // loop for next character
		}
	}
	return (XIO_OK);
}

static int _gets_helper(xioDev_t *d, xioUsart_t *dx)
{
	char c = NUL;

	if (dx->rx_buf_head == dx->rx_buf_tail) {	// RX ISR buffer empty
		dx->rx_buf_count = 0;					// reset count for good measure
		return(XIO_BUFFER_EMPTY);				// stop reading
	}
	advance_buffer(dx->rx_buf_tail, RX_BUFFER_SIZE);
	dx->rx_buf_count--;
	d->x_flow(d);								// run flow control
//	c = dx->rx_buf[dx->rx_buf_tail];			// get char from RX Q
	c = (dx->rx_buf[dx->rx_buf_tail] & 0x007F);	// get char from RX Q & mask MSB
	if (d->flag_echo) d->x_putc(c, stdout);		// conditional echo regardless of character

	if (d->len >= d->size) {					// handle buffer overruns
		d->buf[d->size] = NUL;					// terminate line (d->size is zero based)
		d->signal = XIO_SIG_EOL;
		return (XIO_BUFFER_FULL);
	}
	if ((c == CR) || (c == LF)) {				// handle CR, LF termination
		d->buf[(d->len)++] = NUL;
		d->signal = XIO_SIG_EOL;
		d->flag_in_line = false;				// clear in-line state (reset)
		return (XIO_EOL);						// return for end-of-line
	}
	d->buf[(d->len)++] = c;						// write character to buffer
	return (XIO_EAGAIN);
}

/*
 *  xio_getc_usart() - generic char reader for USART devices
 *
 *	Compatible with stdio system - may be bound to a FILE handle
 *
 *  Get next character from RX buffer.
 *  See https://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Module-Details#Notes_on_Circular_Buffers
 *  for a discussion of how the circular buffers work
 *
 *	This routine returns a single character from the RX buffer to the caller.
 *	It's typically called by fgets() and is useful for single-threaded IO cases.
 *	Cases with multiple concurrent IO streams may want to use the gets() function
 *	which is incompatible with the stdio system.
 *
 *  Flags that affect behavior:
 *
 *  BLOCKING behaviors
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 & XIO_SIG_WOULDBLOCK if non-blocking
 *		- return character or sleep() if blocking
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc is responsible for expanding newlines to <cr><lf> if needed
 */
int xio_getc_usart(FILE *stream)
{
	// these convenience pointers optimize faster than resolving the references each time
	xioDev_t *d = (xioDev_t *)stream->udata;
	xioUsart_t *dx = d->x;
	char c;

	while (dx->rx_buf_head == dx->rx_buf_tail) {	// RX ISR buffer empty
		dx->rx_buf_count = 0;						// reset count for good measure
		if (d->flag_block) {
			sleep_mode();
		} else {
			d->signal = XIO_SIG_EAGAIN;
			return(_FDEV_ERR);
		}
	}
	advance_buffer(dx->rx_buf_tail, RX_BUFFER_SIZE);
	dx->rx_buf_count--;
	d->x_flow(d);									// flow control callback
	c = (dx->rx_buf[dx->rx_buf_tail] & 0x007F);		// get char from RX buf & mask MSB

	// Triage the input character for handling. This code does not handle deletes
	if (d->flag_echo) d->x_putc(c, stdout);			// conditional echo regardless of character
	if (c > CR) return(c); 							// fast cutout for majority cases
	if ((c == CR) || (c == LF)) {
		if (d->flag_linemode) return('\n');
	}
	return(c);
}

/*
 * xio_putc_usart() - stdio compatible char writer for usart devices
 *	This routine is not needed at the class level.
 *	See xio_putc_usb() and xio_putc_rs485()
 */
int xio_putc_usart(const char c, FILE *stream)
{
	return (XIO_OK);
}

/* Fakeout routines
 *
 *	xio_queue_RX_string_usart() - fake ISR to put a string in the RX buffer
 *	xio_queue_RX_char_usart() - fake ISR to put a char in the RX buffer
 *
 *	String must be NUL terminated but doesn't require a CR or LF
 *	Also has wrappers for USB and RS485
 */
//void xio_queue_RX_char_usb(const char c) { xio_queue_RX_char_usart(XIO_DEV_USB, c); }
void xio_queue_RX_string_usb(const char *buf) { xio_queue_RX_string_usart(XIO_DEV_USB, buf); }
//void xio_queue_RX_char_rs485(const char c) { xio_queue_RX_char_usart(XIO_DEV_RS485, c); }
//void xio_queue_RX_string_rs485(const char *buf) { xio_queue_RX_string_usart(XIO_DEV_RS485, buf); }

void xio_queue_RX_string_usart(const uint8_t dev, const char *buf)
{
	uint8_t i=0;
	while (buf[i] != NUL) {
		xio_queue_RX_char_usart(dev, buf[i++]);
	}
}

void xio_queue_RX_char_usart(const uint8_t dev, const char c)
{
	xioDev_t *d = &ds[dev];
	xioUsart_t *dx = d->x;

	// normal path
	advance_buffer(dx->rx_buf_head, RX_BUFFER_SIZE);
	if (dx->rx_buf_head != dx->rx_buf_tail) {	// write char unless buffer full
		dx->rx_buf[dx->rx_buf_head] = c;		// FAKE INPUT DATA
		dx->rx_buf_count++;
		return;
	}
	// buffer-full handling
	if ((++dx->rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		dx->rx_buf_count = RX_BUFFER_SIZE-1;
		dx->rx_buf_head = 1;
	}
}

