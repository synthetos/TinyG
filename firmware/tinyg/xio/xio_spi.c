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
static char _read_rx_buffer(xioSpi_t *dx);
static char _write_rx_buffer(xioSpi_t *dx, char c);
static char _read_tx_buffer(xioSpi_t *dx);
static char _write_tx_buffer(xioSpi_t *dx, char c);
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
 *	xio_init_spi() - init entire SPI system
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
	xio_ctrl_generic(d, flags);

	// setup internal RX/TX control buffers
	dx->rx_buf_head = SPI_RX_BUFFER_SIZE-1;
	dx->rx_buf_tail = SPI_RX_BUFFER_SIZE-1;
	dx->tx_buf_head = SPI_TX_BUFFER_SIZE-1;
	dx->tx_buf_tail = SPI_TX_BUFFER_SIZE-1;

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
 *
 *	Retains line context across calls - so it can be called multiple times.
 *	Reads as many characters as it can until any of the following is true:
 *
 *	  - Encounters newline indicating a complete line (returns XIO_OK)
 *	  - Encounters and empty buffer and no more data in the salve (return XIO_EAGAIN)
 *	  - Read would cause output buffer overflow (return XIO_BUFFER_FULL)
 *
 *	gets() runs an optimized buffer transfer relative to getc() as it can keep 
 *	slave state. It reads from the local RX buffer and replenishes it with chars 
 *	read from the slave as it reads. Once the local RX buffer empties it reads 
 *	directly from the slave until the line is complete or the slave is exhausted.
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 *	Note: CRs are not recognized as NL chars - slaves must send LF to terminate a line
 */
int xio_gets_spi(xioDev_t *d, char *buf, const int size)
{
	xioSpi_t *dx = (xioSpi_t *)d->x;			// get SPI device struct pointer
	uint8_t read_spi_flag = true;				// read from slave until no chars available
	char c_out, c_spi;

	// first time thru initializations
	if (d->flag_in_line == false) {
		d->flag_in_line = true;					// yes, we are busy getting a line
		d->buf = buf;							// bind the output buffer
		d->len = 0;								// zero the buffer count
		d->size = size;							// set the max size of the message
//		d->signal = XIO_SIG_OK;					// reset signal register
	}

	// process chars until done or blocked
	while (true) {
		// test for string output buffer overrun
		if (d->len >= (d->size)-1) {			// size is total count - aka 'num' in fgets()
			d->buf[d->size] = NUL;				// string termination preserves latest char
			d->signal = XIO_SIG_EOL;
			return (XIO_BUFFER_FULL);
		}
		// if RX buffer has data read the RX buffer and & replenish from slave, if possible
		if ((c_out = _read_rx_buffer(dx)) != Q_EMPTY) {
			if (read_spi_flag == true) {
				if ((c_spi = _read_spi_char(dx)) == ETX) { // get a char from slave
					read_spi_flag = false;
				} else {
					_write_rx_buffer(dx, c_spi);
				}
			}
		} else {	// RX buffer is empty - try reading from the slave directly
			if ((read_spi_flag == false) || ((c_out = _read_spi_char(dx)) == ETX)) {
				return (XIO_EAGAIN);			// return with an incomplete line
			} 
		}
		// test for end-of-line
		if (c_out == LF) {
			d->buf[(d->len)++] = NUL;
//			d->signal = XIO_SIG_EOL;
			d->flag_in_line = false;			// clear in-line state (reset)
			return (XIO_OK);					// return for end-of-line
		}
		d->buf[(d->len)++] = c_out;				// write character to buffer
	}
}

/*
 * xio_getc_spi() - stdio compatible character RX routine
 *
 *	There are actually 2 queues involved in this function - the local RX queue
 *	on the master, and the slave's TX queue. This function is not optimized for 
 *	transfer, as it returns a single character and has no state information
 *	about the slave.
 *
 *	If there are no characters in the local RX buffer it will poll the slave for
 *	the next character.
 *
 *	This function is always non-blocking or it would create a deadlock.
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
	xioSpi_t *dx = ((xioDev_t *)stream->udata)->x;	// cache SPI device struct pointer
	char status = _write_tx_buffer(dx, c); 
	return ((int)status);
}

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

static char _read_rx_buffer(xioSpi_t *dx) {
	if (dx->rx_buf_head == dx->rx_buf_tail) { return (Q_EMPTY);}
	char c = dx->rx_buf[dx->rx_buf_tail];
	if ((--(dx->rx_buf_tail)) == 0) { dx->rx_buf_tail = SPI_RX_BUFFER_SIZE-1;}
	return (c);
}

static char _write_rx_buffer(xioSpi_t *dx, char c) {
	spibuf_t next_buf_head = --(dx->rx_buf_head);
	if (next_buf_head == 0) { dx->rx_buf_head = SPI_RX_BUFFER_SIZE-1;}
	if (next_buf_head == dx->rx_buf_tail) { return (Q_EMPTY);}
	dx->rx_buf[next_buf_head] = c;
	dx->rx_buf_head = next_buf_head;
	return (XIO_OK);
}

static char _read_tx_buffer(xioSpi_t *dx) {
	if (dx->tx_buf_head == dx->tx_buf_tail) { return (Q_EMPTY);}
	char c = dx->tx_buf[dx->tx_buf_tail];
	if ((--(dx->tx_buf_tail)) == 0) { dx->tx_buf_tail = SPI_TX_BUFFER_SIZE-1;}
	return (c);
}

static char _write_tx_buffer(xioSpi_t *dx, char c) {
	spibuf_t next_buf_head = --(dx->tx_buf_head);
	if (next_buf_head == 0) { dx->tx_buf_head = SPI_TX_BUFFER_SIZE-1;}
	if (next_buf_head == dx->tx_buf_tail) { return (Q_EMPTY);}
	dx->tx_buf[next_buf_head] = c;
	dx->tx_buf_head = next_buf_head;
	return (XIO_OK);
}

/*
 * Bitbangers used by the SPI routines
 * _xfer_char() - send a character on MOSI and receive incoming char on MISO
 * _read_char() - send an STX on MOSI and receive incoming char on MISO
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
