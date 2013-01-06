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
#include <avr/sleep.h>					// needed for blocking character writes

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

#include "../tinyg.h"					// needed for AXES definition
#include "../gpio.h"					// needed for XON/XOFF LED indicator
#include "../util.h"					// needed to pick up __debug defines


void xio_init_spi(void)
{
	// setup SPI1
	xio_init_dev(XIO_DEV_SPI1, xio_cntl_spi, xio_putc_spi, xio_getc_spi, xio_gets_spi);
	xio_init_spi_dev(XIO_DEV_SPI1, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
}

/*
 *	xio_init_spi()
 */

void xio_init_spi_dev(const uint8_t dev, 				// index into device array (ds)
					const uint32_t control,			// control bits
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
	uint8_t channel = (dev - XIO_DEV_SPI1);			// remove the offset of any other devices

	// do all the bindings first (and in this order)
	struct xioDEVICE *d = &ds[dev];					// setup device struct pointer
	d->x = &sp[channel-1];							// bind SPI struct to device
	struct xioSPI *dx = (struct xioSPI *)d->x;		// setup SPI struct pointer
	dx->index = channel-1;							// used as fdev.udata
	dx->usart = (struct USART_struct *)usart_addr;	// bind USART used for SPI 
	dx->port = (struct PORT_struct *)port_addr;		// bind PORT used for SPI

	// set control flags
	(void)xio_cntl(dev, control);					// generic setflags -doesn't validate flags

	// setup internal RX/TX buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;
	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;

	// baud rate and USART setup
//	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);
//	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
//	xio_set_baud_usart(dev, baud);		// usart must be bound first

//	dx->usart->CTRLB = (USART_TXEN_bm | USART_RXEN_bm);// enable tx and rx
//	dx->usart->CTRLA = CTRLA_RXON_TXON;	// enable tx and rx IRQs

	dx->port->DIRCLR = dirclr;
	dx->port->DIRSET = dirset;
	dx->port->OUTCLR = outclr;
	dx->port->OUTSET = outset;

	// bind channel index into fdev as udata
	fdev_set_udata(d->fdev, &dx->index);
}

FILE * xio_open_spi(uint8_t dev)
{
	return(SPI.fdev);
}

int xio_cntl_spi(const uint32_t control)
{
	return (NUL);
}

int xio_putc_spi(const char c, FILE *stream)
{
	uint8_t *channel = fdev_get_udata(stream);
	return (NUL);
}

int xio_getc_spi(FILE *stream)
{
	return (NUL);
}

int xio_gets_spi(char *buf, const int size)
{
	return (NUL);
}


