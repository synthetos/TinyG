/*
 * xio_spi.c	- General purpose SPI device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart Jr.
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

#include "../xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"
#include "../tinyg.h"					// needed for AXES definition

// statics
static char _read_rx_buffer(xioSpi_t *dx);
static char _write_rx_buffer(xioSpi_t *dx, char c);
static char _read_tx_buffer(xioSpi_t *dx);
//static char _write_tx_buffer(xioSpi_t *dx, char c);
static char _xfer_spi_char(xioSpi_t *dx, char c_out);
static char _read_spi_char(xioSpi_t *dx);

/******************************************************************************
 * SPI CONFIGURATION RECORDS
 ******************************************************************************/

typedef struct cfgSPI {
		x_open_t x_open;		// see xio.h for typedefs
		x_ctrl_t x_ctrl;
		x_gets_t x_gets;
		x_getc_t x_getc;
		x_putc_t x_putc;
		x_flow_t x_flow;
		USART_t *usart;			// usart binding or BIT_BANG if no usart used
		PORT_t *comm_port;		// port for SCK, MISO and MOSI
		PORT_t *ssel_port;		// port for slave select line
		uint8_t ssbit;			// slave select bit on ssel_port
		uint8_t inbits; 		// bits to set as inputs
		uint8_t outbits; 		// bits to set as outputs
		uint8_t outclr;			// output bits to initialize as CLRd
		uint8_t outset; 		// output bits to initialize as SET
} cfgSpi_t;

static cfgSpi_t const cfgSpi[] PROGMEM = {
	{
		xio_open_spi,			// SPI #1 configs
		xio_ctrl_generic,
		xio_gets_spi,
		xio_getc_spi,
		xio_putc_spi,
		xio_fc_null,
		BIT_BANG,
		&SPI_DATA_PORT,
		&SPI_SS1_PORT,
		SPI_SS1_bm,
		SPI_INBITS_bm,
		SPI_OUTBITS_bm,
		SPI_OUTCLR_bm,
		SPI_OUTSET_bm,
	},
	{
		xio_open_spi,			// SPI #2 configs
		xio_ctrl_generic,
		xio_gets_spi,
		xio_getc_spi,
		xio_putc_spi,
		xio_fc_null,
		BIT_BANG,
		&SPI_DATA_PORT,
		&SPI_SS2_PORT,
		SPI_SS2_bm,
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
 *	xio_init_spi() - top-level init for SPI sub-system
 */
void xio_init_spi(void)
{
	for (uint8_t i=0; i<XIO_DEV_SPI_COUNT; i++) {
		xio_open_generic(XIO_DEV_SPI_OFFSET + i,
						(x_open_t)pgm_read_word(&cfgSpi[i].x_open),
						(x_ctrl_t)pgm_read_word(&cfgSpi[i].x_ctrl),
						(x_gets_t)pgm_read_word(&cfgSpi[i].x_gets),
						(x_getc_t)pgm_read_word(&cfgSpi[i].x_getc),
						(x_putc_t)pgm_read_word(&cfgSpi[i].x_putc),
						(x_flow_t)pgm_read_word(&cfgSpi[i].x_flow));
	}
}

/*
 *	xio_open_spi() - open a specific SPI device
 */
FILE *xio_open_spi(const uint8_t dev, const char *addr, const flags_t flags)
{
	xioDev_t *d = &ds[dev];						// setup device struct pointer
	uint8_t idx = dev - XIO_DEV_SPI_OFFSET;
	d->x = &spi[idx];							// setup extended struct pointer
	xioSpi_t *dx = (xioSpi_t *)d->x;

	memset (dx, 0, sizeof(xioSpi_t));
	xio_reset_working_flags(d);
	xio_ctrl_generic(d, flags);

	// setup internal RX/TX control buffers
	dx->rx_buf_head = 1;
	dx->rx_buf_tail = 1;
	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;

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
 *	xio_gets_spi() - read a complete line (message) from an SPI device
 *
 *	Reads from the local RX buffer until it's empty, then reads from the
 *	slave until the line is complete or the slave is exhausted. Retains line
 *	context across calls - so it can be called multiple times. Reads as many
 *	characters as it can until any of the following is true:
 *
 *	  - Encounters newline indicating a complete line. Terminate the buffer
 *		but do not write the newlinw into the buffer. Reset line flag. Return XIO_OK.
 *
 *	  - Encounters an empty buffer and no more data in the slave. Leave in_line
 *		Return XIO_EAGAIN.
 *
 *	  - A successful read would cause output buffer overflow. Return XIO_BUFFER_FULL
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: CRs are not recognized as NL chars - slaves must send LF to terminate a line
 */
int xio_gets_spi(xioDev_t *d, char *buf, const int size)
{
	xioSpi_t *dx = (xioSpi_t *)d->x;			// get SPI device struct pointer
	char c_out;

	// first time thru initializations
	if (d->flag_in_line == false) {
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->buf = buf;							// bind the output buffer
		d->len = 0;								// zero the buffer count
		d->size = size;							// set the max size of the message
//		d->signal = XIO_SIG_OK;					// reset signal register
	}
	while (true) {
		if (d->len >= (d->size)-1) {			// size is total count - aka 'num' in fgets()
			d->buf[d->size] = NUL;				// string termination preserves latest char
			return (XIO_BUFFER_FULL);
		}
		if ((c_out = _read_rx_buffer(dx)) == Q_EMPTY) {
			if ((c_out = _read_spi_char(dx)) == ETX) { // get a char from slave
				return (XIO_EAGAIN);
			}
		}
		if (c_out == LF) {
			d->buf[(d->len)++] = NUL;
			d->flag_in_line = false;			// clear in-line state (reset)
			return (XIO_OK);					// return for end-of-line
		}
		d->buf[(d->len)++] = c_out;				// write character to buffer
	}
}

/*
 * xio_getc_spi() - stdio compatible character RX routine
 *
 *	This function first tries to return a character from the master's RX queue
 *	and if that fails it tries to get the next character from the slave.
 *
 *	This function is always non-blocking or it would create a deadlock as the
 *	bit-banger SPI transmitter is not interrupt driven
 *
 *	This function is not optimized for transfer rate, as it returns a single
 *	character and has no state information about the slave. gets() is much more
 *	efficient.
 */
int xio_getc_spi(FILE *stream)
{
	xioDev_t *d = (xioDev_t *)stream->udata;	// get device struct pointer
	xioSpi_t *dx = (xioSpi_t *)d->x;			// get SPI device struct pointer
	char c;

	if ((c = _read_rx_buffer(dx)) == Q_EMPTY) {
		if ((c = _read_spi_char(dx)) == ETX) {
			d->signal = XIO_SIG_EOL;
			return(_FDEV_ERR);
		}
	}
	return (c);
}

//void _xio_tx_spi_dx(xioSpi_t *dx);

/*
 * xio_putc_spi() - stdio compatible character TX routine
 *
 *	Putc is split in 2: xio_putc_spi() puts the char in the TX buffer, while
 *	xio_tx_spi() transmits a char from the TX buffer to the slave. This is not
 *	necessary for a pure main-loop bitbanger, but is needed for interrupts or
 *	other asynchronous IO processing.
 */
int xio_putc_spi(const char c, FILE *stream)
{
	// Native SPI device version
	// write to TX queue  - char TX occurs via SPI interrupt
//	return ((int)_write_tx_buffer(((xioDev_t *)stream->udata)->x,c));

	// bit banger version - unbuffered IO
	xioSpi_t *dx = ((xioDev_t *)stream->udata)->x;
	char c_in;

	if ((c_in = _xfer_spi_char(dx,c)) != ETX) {
		if ((c_in == 0x00) || (c_in == 0xFF)) {
			return (XIO_NO_SUCH_DEVICE);
		}
		_write_rx_buffer(dx, c_in);
	}
	return (XIO_OK);
}
/*
	int status = _write_tx_buffer(dx,c);
	if (status != XIO_OK) { return (status);}
	char c_out, c_in;
	if ((c_out = _read_tx_buffer(dx)) == Q_EMPTY) { return ();}
	if ((c_in = _xfer_spi_char(dx, c_out)) != ETX) {
		_write_rx_buffer(dx, c_in);
	}
	return (XIO_OK);
}
*/

/*
 * xio_tx_spi() - send a character trom the TX buffer to the slave
 *
 *	Send a char to the slave while receiving a char from the slave on MISO.
 *	Load received char into the RX buffer if it's a legitimate.character.
 */
void xio_tx_spi(uint8_t dev)
{
	xioDev_t *d = &ds[dev];
	xioSpi_t *dx = (xioSpi_t *)d->x;
	char c_out, c_in;

	if ((c_out = _read_tx_buffer(dx)) == Q_EMPTY) { return;}
	if ((c_in = _xfer_spi_char(dx, c_out)) != ETX) {
		_write_rx_buffer(dx, c_in);
	}
}
/*
void _xio_tx_spi_dx(xioSpi_t *dx)
{
	char c_out, c_in;
	if ((c_out = _read_tx_buffer(dx)) == Q_EMPTY) { return;}
	if ((c_in = _xfer_spi_char(dx, c_out)) != ETX) {
		_write_rx_buffer(dx, c_in);
	}
}
*/
/*
 * Buffer read and write helpers
 *
 * READ: Read from the tail. Read sequence is:
 *	- test buffer and return Q_empty if empty
 *	- read char from buffer
 *	- advance the tail (post-advance)
 *	- return C with tail left pointing to next char to be read (or no data)
 *
 * WRITES: Write to the head. Write sequence is:
 *	- advance a temporary head (pre-advance)
 *	- test buffer and return Q_empty if empty
 *	- commit head advance to structure
 *	- return status with head left pointing to latest char written
 *
 *	You can make these blocking routines by calling them in an infinite
 *	while() waiting for something other than Q_EMPTY to be returned.
 */

static char _read_rx_buffer(xioSpi_t *dx)
{
	if (dx->rx_buf_head == dx->rx_buf_tail) { return (Q_EMPTY);}
	char c = dx->rx_buf[dx->rx_buf_tail];
	if ((--(dx->rx_buf_tail)) == 0) { dx->rx_buf_tail = SPI_RX_BUFFER_SIZE-1;}
	return (c);
}

static char _write_rx_buffer(xioSpi_t *dx, char c)
{
	spibuf_t next_buf_head = dx->rx_buf_head-1;
	if (next_buf_head == 0) { next_buf_head = SPI_RX_BUFFER_SIZE-1;}
	if (next_buf_head == dx->rx_buf_tail) { return (Q_EMPTY);}
	dx->rx_buf[next_buf_head] = c;
	dx->rx_buf_head = next_buf_head;
	return (XIO_OK);
}

static char _read_tx_buffer(xioSpi_t *dx)
{
	if (dx->tx_buf_head == dx->tx_buf_tail) { return (Q_EMPTY);}
	char c = dx->tx_buf[dx->tx_buf_tail];
	if ((--(dx->tx_buf_tail)) == 0) { dx->tx_buf_tail = SPI_TX_BUFFER_SIZE-1;}
	return (c);
}
/*
static char _write_tx_buffer(xioSpi_t *dx, char c)
{
	spibuf_t next_buf_head = dx->tx_buf_head-1;
	if (next_buf_head == 0) { next_buf_head = SPI_TX_BUFFER_SIZE-1;}
	if (next_buf_head == dx->tx_buf_tail) { return (Q_EMPTY);}
	dx->tx_buf[next_buf_head] = c;
	dx->tx_buf_head = next_buf_head;
	return (XIO_OK);
}
*/
/*
 * Bitbangers used by the SPI routines
 * _xfer_spi_char() - send a character on MOSI and receive incoming char on MISO
 * _read_spi_char() - send an STX on MOSI and receive incoming char on MISO
 */

#define xfer_bit(mask, c_out, c_in) \
	dx->data_port->OUTCLR = SPI_SCK_bm; \
	if ((c_out & mask) == 0) { dx->data_port->OUTCLR = SPI_MOSI_bm; } \
	else { dx->data_port->OUTSET = SPI_MOSI_bm; } \
	if (dx->data_port->IN & SPI_MISO_bm) c_in |= (mask); \
	dx->data_port->OUTSET = SPI_SCK_bm;

#define read_bit_clr(mask, c_in) \
	dx->data_port->OUTCLR = SPI_SCK_bm; \
	dx->data_port->OUTCLR = SPI_MOSI_bm; \
	if (dx->data_port->IN & SPI_MISO_bm) c_in |= (mask); \
	dx->data_port->OUTSET = SPI_SCK_bm;

#define read_bit_set(mask, c_in) \
	dx->data_port->OUTCLR = SPI_SCK_bm; \
	dx->data_port->OUTSET = SPI_MOSI_bm; \
	if (dx->data_port->IN & SPI_MISO_bm) c_in |= (mask); \
	dx->data_port->OUTSET = SPI_SCK_bm;

static char _xfer_spi_char(xioSpi_t *dx, char c_out)
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

static char _read_spi_char(xioSpi_t *dx)
{
	char c_in = 0;
	dx->ssel_port->OUTCLR = dx->ssbit;			// drive slave select lo (active)
	read_bit_clr(0x80, c_in);					// hard coded to send STX
	read_bit_clr(0x40, c_in);
	read_bit_clr(0x20, c_in);
	read_bit_clr(0x10, c_in);
	read_bit_clr(0x08, c_in);
	read_bit_clr(0x04, c_in);
	read_bit_set(0x02, c_in);
	read_bit_clr(0x01, c_in);
	dx->ssel_port->OUTSET = dx->ssbit;
	return (c_in);
}
