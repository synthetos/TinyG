/*
 * xio_spi.c	- General purpose SPI device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
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
/* ---- SPI Protocol ----
 *
 * The SPI master/slave protocol is designed to be as simple as possible. 
 * In short, the master transmits whenever it wants to and the slave returns
 * the next character in its output buffer whenever there's an SPI transfer.
 * No flow control is needed as the master initiates and drives all transfers.
 *
 * More details:
 *
 *	- A "message" is a line of text. Examples of messages are requests from the 
 *		master to a slave, responses to these requests, and asynchronous messages 
 *		(from a slave) that are not tied to a request.
 *
 *		Messages are terminated with a newline (aka NL, LF, line-feed). The 
 *		terminating NL is considered part of the message and should be transmitted.
 *
 *		If multiple NLs are transmitted each trailing NL is interpreted as a blank
 *		message. This is generally not good practice - so watch it.
 *
 *		Carriage return (CR) is not recognized as a newline. A CR in a message is
 *		treated as any other non-special ASCII character.
 *
 *		NULs (0x00) are not transmitted in either direction (e.g. string terminations).
 *		Depending on the master or slave internals, it may convert NULs to NLs. 
 *
 *	- A slave is always in RX state - it must always be able to receive message data (MOSI).
 *
 *	- All SPI transmissions are initiated by the master and are 8 bits long. As the 
 *		slave is receiving the byte on MOSI it should be returning the next character 
 *		in its output buffer on MISO. Note that there is no inherent correlation between
 *		the char (or message) being received from the master and transmitted from the
 *		slave. It's just IO.
 *
 *		If the slave has no data to send it should return ETX (0x03) on MISO. This is 
 *		useful to distinghuish between an "empty" slave and an unpopulated or non-
 *		responsive SPI slave - which would return NULs or possibly 0xFF.
 *
 *	- The master may poll for more message data from the slave by sending STX chars to
 *		the slave. The slave discards all STXs and simply returns output data on these
 *		transfers. Presumably the master would stop polling once it receives an ETX 
 *		from the slave.
 */
/* ---- Low level SPI stuff ----
 *
 *	Uses Mode3, MSB first. See Atmel Xmega A 8077.doc, page 231
 */
#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <string.h>						// for memset
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking TX

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"
#include "../tinyg.h"					// needed for AXES definition

// statics
static int _gets_helper(xioDev *d, xioSpi *dx);
static char _xfer_char(xioSpi *dx, char c_out);

/******************************************************************************
 * SPI CONFIGURATION RECORDS
 ******************************************************************************/

struct cfgSPI {
	x_open x_open;			// see xio.h for typedefs
	x_ctrl x_ctrl;
	x_gets x_gets;
	x_getc x_getc;
	x_putc x_putc;
	fc_func fc_func;
	USART_t *usart;
	PORT_t *comm_port;		// port for SCK, MISO and MOSI
	PORT_t *ssel_port;		// port for slave select line
	uint8_t ssbit;			// slave select bit on ssel_port
	uint8_t inbits; 
	uint8_t outbits; 
	uint8_t outclr;
	uint8_t outset; 
};

static struct cfgSPI const cfgSpi[] PROGMEM = {
{	// SPI#1 config
	xio_open_spi,			// open function
	xio_ctrl_generic, 		// ctrl function
	xio_gets_spi,			// get string function
	xio_getc_spi,			// stdio getc function
	xio_putc_spi,			// stdio putc function
	xio_fc_null,			// flow control callback
	BIT_BANG,				// usart structure or BIT_BANG if none
	&SPI_DATA_PORT,			// SPI comm port
	&SPI_SS1_PORT,			// SPI slave select port
	SPI_SS1_bm,				// slave select bit bitmask
	SPI_INBITS_bm,
	SPI_OUTBITS_bm,
	SPI_OUTCLR_bm,
	SPI_OUTSET_bm,
},
{	// SPI#2 config
	xio_open_spi,			// open function
	xio_ctrl_generic, 		// ctrl function
	xio_gets_spi,			// get string function
	xio_getc_spi,			// stdio getc function
	xio_putc_spi,			// stdio putc function
	xio_fc_null,			// flow control callback
	BIT_BANG,				// usart structure
	&SPI_DATA_PORT,			// SPI comm port
	&SPI_SS2_PORT,			// SPI slave select port
	SPI_SS2_bm,				// slave select bit bitmask
	SPI_INBITS_bm,
	SPI_OUTBITS_bm,
	SPI_OUTCLR_bm,
	SPI_OUTSET_bm,
}
};

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/

/*
 *	xio_init_spi() - init entire SPI system
 */
void xio_init_spi(void)
{
	for (uint8_t i=0; i<XIO_DEV_SPI_COUNT; i++) {
		xio_open_generic(XIO_DEV_SPI_OFFSET + i,
						(x_open)pgm_read_word(&cfgSpi[i].x_open),
						(x_ctrl)pgm_read_word(&cfgSpi[i].x_ctrl),
						(x_gets)pgm_read_word(&cfgSpi[i].x_gets),
						(x_getc)pgm_read_word(&cfgSpi[i].x_getc),
						(x_putc)pgm_read_word(&cfgSpi[i].x_putc),
						(fc_func)pgm_read_word(&cfgSpi[i].fc_func));
	}
}

/*
 *	xio_open_spi() - open a specific SPI device
 */
FILE *xio_open_spi(const uint8_t dev, const char *addr, const CONTROL_T flags)
{
	xioDev *d = &ds[dev];						// setup device struct pointer
	uint8_t idx = dev - XIO_DEV_SPI_OFFSET;
	d->x = &sp[idx];							// setup extended struct pointer
	xioSpi *dx = (xioSpi *)d->x;

	memset (dx, 0, sizeof(xioSpi));				// clear all values
	xio_ctrl_generic(d, flags);					// setup flags

	// setup internal RX/TX control buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;

	// structure and device bindings and setup
	dx->usart = (USART_t *)pgm_read_word(&cfgSpi[idx].usart); 
	dx->data_port = (PORT_t *)pgm_read_word(&cfgSpi[idx].comm_port);
	dx->ssel_port = (PORT_t *)pgm_read_word(&cfgSpi[idx].ssel_port);

	dx->ssbit = (uint8_t)pgm_read_byte(&cfgSpi[idx].ssbit);
	dx->data_port->DIRCLR = (uint8_t)pgm_read_byte(&cfgSpi[idx].inbits);
	dx->data_port->DIRSET = (uint8_t)pgm_read_byte(&cfgSpi[idx].outbits);
	dx->data_port->OUTCLR = (uint8_t)pgm_read_byte(&cfgSpi[idx].outclr);
	dx->data_port->OUTSET = (uint8_t)pgm_read_byte(&cfgSpi[idx].outset);
	return (&d->file);							// return FILE reference
}

/* 
 *	xio_gets_spi() - read a complete line from an SPI device
 * _gets_helper() 	 - non-blocking character getter for gets
 *
 *	Retains line context across calls - so it can be called multiple times.
 *	Reads as many characters as it can until any of the following is true:
 *
 *	  - SPI read returns newline indicating a complete line (returns XIO_OK)
 *	  - SPI read returns ETX indicating there is no data to xfer (return XIO_EAGAIN)
 *	  - read would cause output buffer overflow (return XIO_BUFFER_FULL)
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: CRs are not recognized as NL chars - this function assumes slaves never send CRs. 
 */
int xio_gets_spi(xioDev *d, char *buf, const int size)
{
	xioSpi *dx = (xioSpi *)d->x;				// get SPI device struct pointer

	if (d->flag_in_line == false) {				// first time thru initializations
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->len = 0;								// zero buffer
		d->buf = buf;
		d->size = size;
		d->signal = XIO_SIG_OK;					// reset signal register
	}
	while (true) {
		switch (_gets_helper(d,dx)) {
//			case (XIO_BUFFER_EMPTY): return (XIO_EAGAIN); // empty condition
			case (XIO_BUFFER_FULL_NON_FATAL): return (XIO_BUFFER_FULL_NON_FATAL);// overrun err
			case (XIO_EOL): return (XIO_OK);			  // got complete line
			case (XIO_EAGAIN): break;					  // loop for next character
		}
	}
	return (XIO_OK);
}

static int _gets_helper(xioDev *d, xioSpi *dx)
{
	char c = NUL;

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
 * xio_getc_spi() - stdio compatible character RX routine
 *
 *	This fucntion attempts to poll a character from the slave then 
 *	returns the latest character in the RX buffer.
 *
 *	It's always non-blocking or it would create a deadlock.
 */
int xio_getc_spi(FILE *stream)
{
	xioDev *d = (xioDev *)stream->udata;		// get device struct pointer
	xioSpi *dx = (xioSpi *)d->x;				// get SPI device struct pointer
	char c_in;

	xio_putc_spi(STX, stream);					// poll a char into the RX buffer

	while (dx->rx_buf_head == dx->rx_buf_tail) {// RX buffer empty
		d->signal = XIO_SIG_EAGAIN;
		return(_FDEV_ERR);
	}
	if (--(dx->rx_buf_tail) == 0) {				// advance RX tail (RXQ read ptr)
		dx->rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one error (OBOE)
	}
	c_in = dx->rx_buf[dx->rx_buf_tail];			// get char from RX buf
	return(c_in);
}

/*
 * xio_putc_spi() - stdio compatible character TX routine
 * _xfer_char() - helper written as inline bit-banger
 *
 *	This sends a char to the slave and receives a byte on MISO in return.
 *	This routine clocks out to about 600 Kbps bi-directional transfer rates
 */
int xio_putc_spi(const char c_out, FILE *stream)
{
	xioSpi *dx = ((xioDev *)stream->udata)->x;	// cache SPI device struct pointer
	char c_in = _xfer_char(dx, c_out);

	// trap special characters - do not insert character into RX queue
	if (c_in == ETX) {	 						// no data received
		((xioDev *)stream->udata)->signal = XIO_SIG_EOF;
		return (XIO_OK);
	}
//	if (c_in == NUL) {	 						// slave is not present or unresponsive
//		return (XIO_OK);
//	}

	// write c_in into the RX buffer (or not)
	if ((--(dx->rx_buf_head)) == 0) { 			// adv buffer head with wrap
		dx->rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one error
	}
	if (dx->rx_buf_head != dx->rx_buf_tail) {	// buffer is not full
		dx->rx_buf[dx->rx_buf_head] = c_in;		// write char to buffer
	} else { 									// buffer-full - toss the incoming character
		if ((++(dx->rx_buf_head)) > RX_BUFFER_SIZE-1) {	// reset the head
			dx->rx_buf_head = 1;
		}
		((xioDev *)stream->udata)->signal = XIO_SIG_OVERRUN; // signal a buffer overflow
	}
	return (XIO_OK);
}

#define xfer_bit(mask, c_out, c_in) \
	dx->data_port->OUTCLR = SPI_SCK_bm; \
	if ((c_out & mask) == 0) { dx->data_port->OUTCLR = SPI_MOSI_bm; } \
	else { dx->data_port->OUTSET = SPI_MOSI_bm; } \
	if (dx->data_port->IN & SPI_MISO_bm) c_in |= (mask); \
	dx->data_port->OUTSET = SPI_SCK_bm;	

static char _xfer_char(xioSpi *dx, char c_out)
{
	char c_in = 0;
	dx->ssel_port->OUTCLR = dx->ssbit;			// drive slave select lo (active)
	xfer_bit(0x80, c_out, c_in);
	xfer_bit(0x40, c_out, c_in);
	xfer_bit(0x20, c_out, c_in);
	xfer_bit(0x10, c_out, c_in);
	xfer_bit(0x08, c_out, c_in);
	xfer_bit(0x04, c_out, c_in);
	xfer_bit(0x02, c_out, c_in);
	xfer_bit(0x01, c_out, c_in);
	dx->ssel_port->OUTSET = dx->ssbit;
	return (c_in);
}


// Looping version - about 18 uS per byte (-Os)

/*
	char miso_data = 0;								// for incoming data from MISO
	for (int8_t i=7; i>=0; i--) {
		dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
		if (c & (1<<i))  dx->data_port->OUTSET = SPI_MOSI_bm; 	// set data bit hi 
					else dx->data_port->OUTCLR = SPI_MOSI_bm; 	// set data bit lo
		dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi (take data / read data)
//		if (dx->data_port->IN & SPI_MISO_bm) miso_data |= (1<<i);// how to collect incoming data bits 
	} 
*/

/* Ultimate speed test with no branches or masks - 8.25 uSec ber byte
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;			// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;			// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;			// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;			// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
*/

/* ---- SPI slave protocol ----
 *
 *	- A "message" is a line of text. Examples of messages are requests from the 
 *		master to a slave, responses to these requests, and asynchronous messages 
 *		(from a slave) that are not tied to a request.
 *
 *		Messages are terminated with a newline (aka LF, line-feed). The terminating 
 *		LF is considered part of the message and is transmitted in each direction.
 *
 *	- A slave is always in RX state - it must always be able to receive MOSI data.
 *
 *	- A slave may be in TXON or TXOFF state -
 *
 *	  While in TXON the slave responds to any MISO by sending the next character 
 *	    in its currently queued message on the MISO line synchronous with receiving 
 *		the incoming MOSI data.
 *
 *	  While in TXOFF the slave will return either ETX or STX on the MISO line for 
 *		any incoming MOSI characters. STX indicates that it has a message to send,
 *		ETX indicates that it does not.
 *
 *	  Receiving STX on MOSI sets the slave to TXON. If the slave has no data or 
 *		does not have a complete message it immediately transitions to TXOFF.
 *		(NOTE: Incomplete message behavior may be revisited). Also, once a 
 *		slave has transmitted its las message if it reverts to TXOFF.
 *
 *	  Receiving ETX on MOSI sets the slave to TXOFF - regardless of whether it
 *		has data to transmit or not.
 */
/* ---- SPI master protocol to send / receive data to / from a slave ----
 *
 *	- To send data to a slave the master simply sends the ASCII message terminated
 *		with a LF. What comes back on MISO is dependent on the slave TX state, but
 *		in most cases the slave will have been set to TXOFF and will return a series
 *		of ETX's.
 *
 *	- The master initiates a read from the slave by first sending STX on MOSI. 
 *		This puts the slave in TXON mode, assuming that it has data to send. 
 *		The synchronous MISO response to this STX may be an ETX or STX depending 
 *		on the previous state of the slave.
 *
 *	- The master then drives transfer by sending a series of STX's on MOSI (Note 1).
 *		The slave responds with character data in MISO. The master continues to
 *		send STX's to retrieve the entire message The master will typically stop
 *		polling once a LF has been received or it receives an ETX. Optionally, the
 *		master can continue to read from the slave until all messages are transferred.
 *
 *	- The above slave read operation can be done for one char only by using  getc(), 
 *		or for an entire message by using gets(). Gets() is much more efficient.
 * *
 * Note (1): Really any character other than an ETX can be used, but STX is safest.
 */
