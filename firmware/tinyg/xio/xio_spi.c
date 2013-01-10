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

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <string.h>						// for memset
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking TX

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"
#include "../tinyg.h"					// needed for AXES definition

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
 *	  - RX buffer is empty on entry (return XIO_EAGAIN)
 *	  - no more chars to read from RX buffer (return XIO_EAGAIN)
 *	  - read would cause output buffer overflow (return XIO_BUFFER_FULL)
 *	  - read returns complete line (returns XIO_OK)
 *
 *	Note: LINEMODE flag in device struct is ignored. It's ALWAYS LINEMODE here.
 */
int xio_gets_spi(xioDev *d, char *buf, const int size)
{
//	xioDevice *d = &ds[dev];					// init device struct pointer

/*
	if (IN_LINE(d->flags) == 0) {				// first time thru initializations
		d->len = 0;								// zero buffer
		d->status = 0;
		d->size = size;
		d->buf = buf;
		d->signal = XIO_SIG_OK;					// reset signal register
		d->flags |= XIO_FLAG_IN_LINE_bm;		// yes, we are busy getting a line
	}
	while (true) {
		switch (d->status = _xio_readc_usart(dev, d->buf)) {
			case (XIO_BUFFER_EMPTY): return (XIO_EAGAIN);		// empty condition
			case (XIO_BUFFER_FULL_NON_FATAL): return (d->status);// overrun err
			case (XIO_EOL): return (XIO_OK);					// got complete line
			case (XIO_EAGAIN): break;							// loop
		}
		// +++ put a size check here or buffers can overrun.
	}
*/
	return (XIO_OK);
}

/*
 * xio_getc_spi() - stdio compatible character RX routine
 */
int xio_getc_spi(FILE *stream)
{
	return (0);
}

/*
 * xio_putc_spi() - stdio compatible character TX routine
 *
 *	Use xio_puts_spi() for more efficient string transmission
 *	Uses Mode3, MSB first. See Atmel Xmega A 8077.doc, page 231
 */
int xio_putc_spi(const char c, FILE *stream)
{
//	xioDev *d = (xioDev *)stream->udata;			// get SPI device struct pointer
//	xioSpi *dx = (xioSpi *)d->x;					// get SPI extended struct pointer
	xioSpi *dx = ((xioDev *)stream->udata)->x;			// get SPI device struct pointer
//	char incoming = 0;								// incoming data from MISO

/*
	// transmit character
	dx->ssel_port->OUTCLR = dx->ssbit;				// drive slave select lo (active)

	for (int8_t i=7; i>=0; i--) {
		dx->data_port->OUTCLR = SPI_SCK_bm; 		// drive clock lo
		if (c & (1<<i))  dx->data_port->OUTSET = SPI_MOSI_bm; 	// set data bit hi 
					else dx->data_port->OUTCLR = SPI_MOSI_bm; 	// set data bit lo

		dx->data_port->OUTSET = SPI_SCK_bm; 		// drive clock hi (take data / read data)
		if (dx->data_port->IN & SPI_MISO_bm) incoming |= (1<<i); // collect incoming data bits 
	}
//	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->ssel_port->OUTSET = dx->ssbit;				// drive slave select hi
*/

	dx->ssel_port->OUTCLR = dx->ssbit;				// drive slave select lo (active)
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;			// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 			// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 			// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->ssel_port->OUTSET = dx->ssbit;				// drive slave select hi

/*
	dx->ssel_port->OUTCLR = dx->ssbit;				// drive slave select lo (active)
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo

//	dx->data_port->OUT = (dx->data_port->OUT & ~SPI_MOSI_bm) | (c>>2 & SPI_MOSI_bm);

	if (c & 0x80) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;	 	// set data bit hi 
//	if (dx->data_port->IN & SPI_MISO_bm) incoming |= (1<<0);  // collect incoming data bit

	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi
	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo

	dx->data_port->OUT &= (c>>1 | ~SPI_MOSI_bm);			// set data bit
	if (c & 0x40) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x20) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x10) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x08) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x04) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x02) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->data_port->OUTCLR = SPI_SCK_bm; 					// drive clock lo
	if (c & 0x01) dx->data_port->OUTSET = SPI_MOSI_bm;		// set data bit lo
			else  dx->data_port->OUTCLR = SPI_MOSI_bm;
	dx->data_port->OUTSET = SPI_SCK_bm; 					// drive clock hi

	dx->ssel_port->OUTSET = dx->ssbit;				// drive slave select hi
*/
	return (XIO_OK);
}
