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
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking TX

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"
#include "../tinyg.h"					// needed for AXES definition

/*
 *	xio_init_spi() - init entire SPI system
 */
void xio_init_spi(void)
{
	xio_init_dev(XIO_DEV_SPI1, xio_open, xio_ctrl, xio_gets, xio_getc_spi, xio_putc_spi);
	xio_init_spi_dev(XIO_DEV_SPI1, SPI_INIT_bm, BIT_BANG, &SPI_PORT, &SPI_PORT, SPI_SS1_bm,
					 SPI_INBITS_bm, SPI_OUTBITS_bm, SPI_OUTCLR_bm, SPI_OUTSET_bm);
}

/*
 *	xio_init_spi_dev() - init a single SPI device
 */
void xio_init_spi_dev(const uint8_t dev, 			// index into device array (ds)
					const uint32_t control,			// control bits
					const struct USART_struct *usart_addr,
					const struct PORT_struct *data_port,
					const struct PORT_struct *ss_port,
					const uint8_t slave_select,
					const uint8_t inbits, 
					const uint8_t outbits, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
	// do all the bindings first (and in this order)
	struct xioDEVICE *d = &ds[dev];					// setup device struct pointer
	d->x = &sp[dev - XIO_DEV_SPI_OFFSET];			// bind SPI struct to device
	struct xioSPI *dx = (struct xioSPI *)d->x;		// setup SPI struct pointer
	dx->usart = (struct USART_struct *)usart_addr;	// bind USART used for SPI 
	dx->data_port = (struct PORT_struct *)data_port;// bind PORT used for SPI data transmission
	dx->ss_port = (struct PORT_struct *)ss_port;	// bind port used for slave select for this device
	dx->slave_select = slave_select;				// save bit used to select this slave

	// set control flags
	(void)xio_ctrl(dev, control);					// generic setflags - doesn't validate flags

	// setup internal RX/TX buffers
//	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
//	dx->rx_buf_tail = 1;
//	dx->tx_buf_head = 1;
//	dx->tx_buf_tail = 1;

	// baud rate and USART setup
//	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);
//	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
//	xio_set_baud_usart(dev, baud);		// usart must be bound first
//	dx->usart->CTRLB = (USART_TXEN_bm | USART_RXEN_bm);// enable tx and rx
//	dx->usart->CTRLA = CTRLA_RXON_TXON;	// enable tx and rx IRQs

	dx->data_port->DIRCLR = inbits;		// setup input bits on port
	dx->data_port->DIRSET = outbits;		// setup output bits on port
	dx->data_port->OUTCLR = outclr;
	dx->data_port->OUTSET = outset;
}

int xio_gets_spi(const uint8_t dev, char *buf, const int size)
{
	return (NUL);
}

int xio_getc_spi(FILE *stream)
{
	return (NUL);
}

/*
 * xio_putc_spi() - stdio compatible character TX routine
 *
 *	Use xio_puts_spi() for more efficient string transmission
 *	Uses Mode3, MSB first. See Atmel Xmega A 8077.doc, page 231
 */
int xio_putc_spi(const char c, FILE *stream)
{
//	struct xioDEVICE *d = ((struct xioDEVICE *)((stream)->udata))->d;	// get SPI device struct pointer
	struct xioSPI *dx = ((struct xioDEVICE *)((stream)->udata))->x;		// get SPI extended struct pointer

	char outc = 0;
/*
	while (1) {
		dx->data_port->DIRSET = 0xff;		// setup output bits on port
//		dx->data_port->OUTCLR = 0xFF;
//		dx->data_port->OUTSET = 0xFF;
		dx->data_port->OUT = 0x00;
		dx->data_port->OUT = 0xFF;

		dx->data_port->DIRSET = 0x00;		// setup output bits on port
		PORTB.OUT = 0x00;
		PORTB.OUT = 0xFF;
//		PORTD.OUT = 0x00;
//		PORTD.OUT = 0xFF;
	}
*/
	dx->ss_port->OUTCLR = dx->slave_select;			// drive slave select lo (active)

	for (int8_t i=7; i>=0; i--) {
		dx->data_port->OUTCLR = SPI_SCK_bm; 		// drive clock lo
		if ((c & (1<<i)) == 0) {
			dx->data_port->OUTCLR = SPI_MOSI_bm; 	// set data bit lo
		} else {
			dx->data_port->OUTSET = SPI_MOSI_bm; 	// set data bit hi 
		}
		dx->data_port->OUTSET = SPI_SCK_bm; 		// drive clock hi (take data / read data)

		if ((dx->data_port->IN & SPI_MISO_bm) != 0) {
			outc |= (1<<i); 
		}
	}

	dx->ss_port->OUTCLR = dx->slave_select;			// drive slave select hi


	// transmit character

	// finish up
	return (XIO_OK);
}




