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
 *
 * ---- Efficiency ----
 *
 *	Originally structures were accessed using "floating" accessor macros - e.g:
 *
 *    #define DEV (ds[dev])						// device struct accessor
 *    #define DEVx ((xioUsart *)(ds[dev].x))	// USART extended struct accessor
 *
 *	It turned out to be more efficient to initialize pointers in each routine and 
 *	use them instead - e.g:
 *
 *	  xioDevice *d = &ds[dev];					// setup device struct ptr
 *    xioUsart *dx = (xioUsart *)ds[dev].x; 	// setup USART struct ptr
 *
 *  What's most efficient is to use a static reference to the structures, 
 *	but this only works if the routine is not general (i.e. only handles one device)
 *
 *	  #define USB ds[XIO_DEV_USB]				// device struct accessor
 *	  #define USBu us[XIO_DEV_USB_OFFSET]		// usart extended struct accessor
 *
 *	There are other examples of this approach as well (e.g. xio_set_baud_usart())
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking character writes

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

#include "../tinyg.h"					// needed for AXES definition
#include "../gpio.h"					// needed for XON/XOFF LED indicator
#include "../util.h"					// needed to pick up __debug defines


// baud rate lookup tables - indexed by enum xioBAUDRATES (line up with values in xio_usart.h)
// Assumes CTRLB CLK2X bit (0x04) is not enabled
static const uint8_t bsel[] PROGMEM = { 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };
static const uint8_t bscale[] PROGMEM = { 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

// local function prototypes
static int _gets_helper(xioDevice *d, xioUsart *dx);

/*
 *	xio_init_usart() - general purpose USART initialization (shared)
 */
void xio_init_usart(const uint8_t dev, 			// index into device array (ds)
					uint8_t baud,
					const CONTROL_T control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t inbits, 
					const uint8_t outbits, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
	// do all the bindings first (and in this order)
	xioDevice *d = &ds[dev];						// setup device struct pointer
	d->x = &us[dev - XIO_DEV_USART_OFFSET];			// bind USART struct to device extended struct
	xioUsart *dx = (xioUsart *)d->x;				// setup USART struct pointer
	dx->usart = (struct USART_struct *)usart_addr;	// bind USART 
	dx->port = (struct PORT_struct *)port_addr;		// bind PORT

	(void)xio_ctrl(dev, control);					// set flags - doesn't validate flags
	if (d->flag_xoff) dx->fc_state = FC_IN_XON;		// transfer flow control setting 

	// setup internal RX/TX buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;
	dx->rx_buf_count = 0;

	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;
	dx->tx_buf_count = 0;

	// baud rate and USART setup
	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
	xio_set_baud_usart(dx, baud);						// usart must be bound first

	dx->usart->CTRLB = (USART_TXEN_bm | USART_RXEN_bm);	// enable tx and rx
	dx->usart->CTRLA = CTRLA_RXON_TXON;					// enable tx and rx IRQs

	dx->port->DIRCLR = inbits;
	dx->port->DIRSET = outbits;
	dx->port->OUTCLR = outclr;
	dx->port->OUTSET = outset;
}

void xio_set_baud_usart(xioUsart *dx, const uint8_t baud)
{
	dx->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	dx->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
}

/*
 * xio_fc_usart() - Usart device flow control function
 *
 * 	Called as a callback from the usart handlers
 */

void xio_fc_usart(xioDevice *d)
{
	xioUsart *dx = d->x;
	if (xio_get_rx_bufcount_usart(dx) < XOFF_RX_LO_WATER_MARK) {
		xio_xon_usart(dx);
	}
}

/* 
 * xio_xoff_usart() - send XOFF flow control for USART devices
 * xio_xon_usart()  - send XON flow control for USART devices
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

/*
 * xio_get_tx_bufcount_usart() - returns number of chars in TX buffer
 * xio_get_rx_bufcount_usart() - returns number of chars in RX buffer
 * xio_get_usb_rx_free() - returns free space in the USB RX buffer
 *
 *	Remember: The queues fill from top to bottom, w/0 being the wrap location
 */
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

BUFFER_T xio_get_usb_rx_free(void)
{
	return (RX_BUFFER_SIZE - xio_get_rx_bufcount_usart(&USBu));
}

/* 
 * xio_putc_usart() - stdio compatible char writer for usart devices
 *  NULL routine. See xio_putc_usb() for an example
 */
int xio_putc_usart(const char c, FILE *stream)
{
	return (XIO_OK);
}

/*
 * xio_queue_RX_char_usart() - fake ISR to put a char in the RX buffer
 */
void xio_queue_RX_char_usart(const uint8_t dev, const char c)
{
	xioDevice *d = &ds[dev];			// init device struct pointer
	xioUsart *dx = d->x;

	// trap signals - do not insert into RX queue
	if (c == CHAR_RESET) {	 				// trap Kill signal
		d->signal = XIO_SIG_RESET;			// set signal value
		sig_reset();						// call app-specific sig handler
		return;
	}
	if (c == CHAR_FEEDHOLD) {				// trap feedhold signal
		d->signal = XIO_SIG_FEEDHOLD;
		sig_feedhold();
		return;
	}
	if (c == CHAR_CYCLE_START) {			// trap cycle start signal
		d->signal = XIO_SIG_CYCLE_START;
		sig_cycle_start();
		return;
	}
	// normal path
	if ((--dx->rx_buf_head) == 0) { 		// wrap condition
		dx->rx_buf_head = RX_BUFFER_SIZE-1;	// -1 avoids the off-by-one err
	}
	if (dx->rx_buf_head != dx->rx_buf_tail) {// write char unless buffer full
		dx->rx_buf[dx->rx_buf_head] = c;	// FAKE INPUT DATA
		dx->rx_buf_count++;
		return;
	}
	// buffer-full handling
	if ((++dx->rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		dx->rx_buf_count = RX_BUFFER_SIZE-1;
		dx->rx_buf_head = 1;
	}
}

/*
 * xio_queue_RX_string_usart() - fake ISR to put a string in the RX buffer
 *
 *	String must be NUL terminated but doesn't require a CR or LF
 */
void xio_queue_RX_string_usart(const uint8_t dev, const char *buf)
{
	uint8_t i=0;
	while (buf[i] != NUL) {
		xio_queue_RX_char_usart(dev, buf[i++]);
	}
}

/* USB device wrappers for generic USART routines */
void xio_queue_RX_char_usb(const char c) { xio_queue_RX_char_usart(XIO_DEV_USB, c); }
void xio_queue_RX_string_usb(const char *buf) { xio_queue_RX_string_usart(XIO_DEV_USB, buf); }

// RS485 device wrappers for generic USART routines
void xio_queue_RX_char_rs485(const char c) { xio_queue_RX_char_usart(XIO_DEV_RS485, c); }
void xio_queue_RX_string_rs485(const char *buf) { xio_queue_RX_string_usart(XIO_DEV_RS485, buf); }


/* 
 *	xio_gets_usart() - read a complete line from the usart device
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
int xio_gets_usart(const uint8_t dev, char *buf, const int size)
{
	xioDevice *d = &ds[dev];					// device struct pointer
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

/*
 * _gets_helper() - non-blocking character getter for gets
 */
static int _gets_helper(xioDevice *d, xioUsart *dx)
{
	char c = NUL;

	if (dx->rx_buf_head == dx->rx_buf_tail) {	// RX ISR buffer empty
		dx->rx_buf_count = 0;					// reset count for good measure
		return(XIO_BUFFER_EMPTY);				// stop reading
	}
	if (--(dx->rx_buf_tail) == 0) {				// advance RX tail (RX q read ptr)
		dx->rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one (OBOE)
	}
	dx->rx_buf_count--;
	d->fc_func(d);								// run the flow control function (USB only)
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
	xioDevice *d = (xioDevice *)stream->udata;		// get device struct pointer
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
	if (--(dx->rx_buf_tail) == 0) {					// advance RX tail (RXQ read ptr)
		dx->rx_buf_tail = RX_BUFFER_SIZE-1;			// -1 avoids off-by-one error (OBOE)
	}
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
void xio_dump_RX_queue_usart(void)
{
	char c;
	uint8_t dev = XIO_DEV_USB;
	xioDevice *d = &ds[dev];			// init device struct pointer
	xioUsart *dx = ((xioUsart *)(ds[dev].x));// init USART pointer

	uint8_t i = dx->rx_buf_tail;
	for (uint8_t j=RX_BUFFER_SIZE; j>0; j--) {

		c = dx->rx_buf[i];
		if (c == NUL) { c = '_';}
		if (c == TAB) { c = '|';}
		if (c == CR)  { c = '/';}
		if (c == LF)  { c = '\\';}
		
		d->x_putc(c, stdout);
		if (--(i) == 0) {
			i = RX_BUFFER_SIZE-1;
		}
	}
}
*/
