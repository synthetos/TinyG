/*
 * xio_usart.c	- General purpose USART device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <string.h>						// for memset
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking character writes

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

#include "../tinyg.h"					// needed for AXES definition
#include "../gpio.h"					// needed for XON/XOFF LED indicator
#include "../util.h"					// needed to pick up __debug defines

/******************************************************************************
 * USART CONFIGURATION RECORDS
 ******************************************************************************/
// See xio_usart.h for baud rate configuration settings
static const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
static const uint8_t bscale[] PROGMEM = { 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

struct cfgUSART {
		x_open x_open;			// see xio.h for typedefs
		x_ctrl x_ctrl;
		x_gets x_gets;
		x_getc x_getc;
		x_putc x_putc;
		fc_func fc_func;
		USART_t *usart;
		PORT_t *port;
		uint8_t baud; 
		uint8_t inbits; 
		uint8_t outbits; 
		uint8_t outclr;
		uint8_t outset; 
};

static struct cfgUSART const cfgUsart[] PROGMEM = {
	{	// USB config
		xio_open_usart,			// open function
		xio_ctrl_generic, 		// ctrl function
		xio_gets_usart,			// get string function
		xio_getc_usart,			// stdio getc function
		xio_putc_usb,			// stdio putc function
		xio_fc_usart,			// flow control callback
		&USB_USART,				// usart structure
		&USB_PORT,				// usart port
		USB_BAUD,
		USB_INBITS_bm,
		USB_OUTBITS_bm,
		USB_OUTCLR_bm,
		USB_OUTSET_bm
	},
	{	// RS485 config 
		xio_open_spi,			// open function
		xio_ctrl_generic, 		// ctrl function
		xio_gets_usart,			// get string function
		xio_getc_usart,			// stdio getc function
		xio_putc_rs485,			// stdio putc function
		xio_fc_null,			// flow control callback
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

static int _gets_helper(xioDev *d, xioUsart *dx);

/*
 *	xio_init_usart() - general purpose USART initialization (shared)
 */
void xio_init_usart(void)
{
	for (uint8_t i=0; i<XIO_DEV_USART_COUNT; i++) {
		xio_open_generic(XIO_DEV_USART_OFFSET + i,
						(x_open)pgm_read_word(&cfgUsart[i].x_open),
						(x_ctrl)pgm_read_word(&cfgUsart[i].x_ctrl),
						(x_gets)pgm_read_word(&cfgUsart[i].x_gets),
						(x_getc)pgm_read_word(&cfgUsart[i].x_getc),
						(x_putc)pgm_read_word(&cfgUsart[i].x_putc),
						(fc_func)pgm_read_word(&cfgUsart[i].fc_func));
	}
}

/*
 *	xio_open_usart() - general purpose USART open (shared)
 *	xio_set_baud_usart() - baud rate setting routine
 */
FILE *xio_open_usart(const uint8_t dev, const char *addr, const CONTROL_T flags)
{
	xioDev *d = &ds[dev];							// setup device struct pointer
	uint8_t idx = dev - XIO_DEV_USART_OFFSET;
	d->x = &us[idx];								// bind extended struct to device
	xioUsart *dx = (xioUsart *)d->x;

	memset (dx, 0, sizeof(xioUsart));				// clear all values
	xio_ctrl_generic(d, flags);						// setup control flags	
	if (d->flag_xoff) dx->fc_state = FC_IN_XON;		// transfer flow control setting 

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
	return (&d->file);		// return FILE reference
}

void xio_set_baud_usart(xioUsart *dx, const uint8_t baud)
{
	dx->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	dx->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
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

void xio_xoff_usart(xioUsart *dx)
{
	if (dx->fc_state == FC_IN_XON) {
		dx->fc_char = XOFF; 
		dx->fc_state = FC_IN_XOFF;
		dx->usart->CTRLA = CTRLA_RXON_TXON;		// force a TX interrupt
	}
}

void xio_xon_usart(xioUsart *dx)
{
	if (dx->fc_state == FC_IN_XOFF) {
		dx->fc_char = XON; 
		dx->fc_state = FC_IN_XON;
		dx->usart->CTRLA = CTRLA_RXON_TXON;		// force a TX interrupt
	}
}

void xio_fc_usart(xioDev *d)		// callback from the usart handlers
{
	xioUsart *dx = d->x;
	if (xio_get_rx_bufcount_usart(dx) < XOFF_RX_LO_WATER_MARK) {
		xio_xon_usart(dx);
	}
}

BUFFER_T xio_get_tx_bufcount_usart(const xioUsart *dx)
{
	if (dx->tx_buf_head <= dx->tx_buf_tail) {
		return (dx->tx_buf_tail - dx->tx_buf_head);
	} else {
		return (TX_BUFFER_SIZE - (dx->tx_buf_head - dx->tx_buf_tail));
	}
}

BUFFER_T xio_get_rx_bufcount_usart(const xioUsart *dx)
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
int xio_gets_usart(xioDev *d, char *buf, const int size)
{
	xioUsart *dx = d->x;						// USART pointer

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
			case (XIO_BUFFER_FULL_NON_FATAL): return (XIO_BUFFER_FULL_NON_FATAL);// overrun err
			case (XIO_EOL): return (XIO_OK);			  // got complete line
			case (XIO_EAGAIN): break;					  // loop for next character
		}
	}
	return (XIO_OK);
}

static int _gets_helper(xioDev *d, xioUsart *dx)
{
	char c = NUL;

	if (dx->rx_buf_head == dx->rx_buf_tail) {	// RX ISR buffer empty
		dx->rx_buf_count = 0;					// reset count for good measure
		return(XIO_BUFFER_EMPTY);				// stop reading
	}
	advance(dx->rx_buf_tail, RX_BUFFER_SIZE);
//	if (--(dx->rx_buf_tail) == 0) {				// advance RX tail (RX q read ptr)
//		dx->rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one (OBOE)
//	}
	dx->rx_buf_count--;
	d->fc_func(d);								// run the flow control callback
//	c = dx->rx_buf[dx->rx_buf_tail];			// get char from RX Q
	c = (dx->rx_buf[dx->rx_buf_tail] & 0x007F);	// get char from RX Q & mask MSB
	if (d->flag_echo) d->x_putc(c, stdout);		// conditional echo regardless of character

	if (d->len >= d->size) {					// handle buffer overruns
		d->buf[d->size] = NUL;					// terminate line (d->size is zero based)
		d->signal = XIO_SIG_EOL;
		return (XIO_BUFFER_FULL_NON_FATAL);
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
	xioDev *d = (xioDev *)stream->udata;			// get device struct pointer
	xioUsart *dx = d->x;							// get USART pointer
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
	advance(dx->rx_buf_tail, RX_BUFFER_SIZE);
//	if (--(dx->rx_buf_tail) == 0) {					// advance RX tail (RXQ read ptr)
//		dx->rx_buf_tail = RX_BUFFER_SIZE-1;			// -1 avoids off-by-one error (OBOE)
//	}
	dx->rx_buf_count--;
	d->fc_func(d);									// run the flow control function (USB only)
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
	xioDev *d = &ds[dev];						// init device struct pointer
	xioUsart *dx = d->x;

	// trap signals - do not insert into RX queue
	if (c == CHAR_RESET) {	 					// trap Kill signal
		d->signal = XIO_SIG_RESET;				// set signal value
		sig_reset();							// call app-specific sig handler
		return;
	}
	if (c == CHAR_FEEDHOLD) {					// trap feedhold signal
		d->signal = XIO_SIG_FEEDHOLD;
		sig_feedhold();
		return;
	}
	if (c == CHAR_CYCLE_START) {				// trap cycle start signal
		d->signal = XIO_SIG_CYCLE_START;
		sig_cycle_start();
		return;
	}
	// normal path
	advance(dx->rx_buf_head, RX_BUFFER_SIZE);
//	if ((--dx->rx_buf_head) == 0) { 			// wrap condition
//		dx->rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids the off-by-one err
//	}
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

