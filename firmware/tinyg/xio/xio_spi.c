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


/* SPI device wrappers for generic SPI routines */

FILE * xio_open_spi() { return(SPI.fdev); }
//int xio_cntl_usb(const uint32_t control) return xio_cntl(XIO_DEV_USB, control); // SEE NOTE
//int xio_getc_usb(FILE *stream) return xio_getc_usart(XIO_DEV_USB, stream);
//int xio_gets_usb(char *buf, const int size) return xio_gets_usart(XIO_DEV_USB, buf, size);
//void xio_queue_RX_char_usb(const char c) xio_queue_RX_char_usart(XIO_DEV_USB, c);
//void xio_queue_RX_string_usb(const char *buf) xio_queue_RX_string_usart(XIO_DEV_USB, buf);

//void xio_init_usb()	// USB inits
//{
//	xio_init_dev(XIO_DEV_USB, xio_open_usb, xio_cntl_usb, xio_putc_usb, xio_getc_usb, xio_gets_usb);
//	xio_init_usart(XIO_DEV_USB, XIO_DEV_USB_OFFSET, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
//}

void xio_init_spi(void)
{
	xio_init_dev(XIO_DEV_SPI1, xio_open_spi, xio_cntl_spi, xio_putc_spi, xio_getc_spi, xio_gets_spi);
//	for ()
//	xio_init_spi_chan(XIO_DEV_SPI1, XIO_DEV_USB_OFFSET, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
}

/*
 *	xio_init_spi_chan()
 */

void xio_init_spi_chan(const uint8_t dev, 			// index into device array (ds)
					const uint8_t offset,			// index into SPI array (sp)
					const uint32_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset) 
{
	// do all the bindings first (and in this order)
	struct xioDEVICE *d = &ds[dev];					// setup device struct pointer
	d->x = &us[offset];								// bind USART struct to device
	struct xioUSART *dx = (struct xioUSART *)d->x;	// setup USART struct pointer
	dx->usart = (struct USART_struct *)usart_addr;	// bind USART 
	dx->port = (struct PORT_struct *)port_addr;		// bind PORT

	// set flags
	(void)xio_cntl(dev, control);// generic setflags -doesn't validate flags
	if (EN_XOFF(d->flags) == true) {				// transfer flow control setting 
		dx->fc_state = FC_IN_XON;					// resting state 
	}

	// setup internal RX/TX buffers
	dx->rx_buf_head = 1;		// can't use location 0 in circular buffer
	dx->rx_buf_tail = 1;
	dx->tx_buf_head = 1;
	dx->tx_buf_tail = 1;

	// baud rate and USART setup
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);
	if (baud == XIO_BAUD_UNSPECIFIED) { baud = XIO_BAUD_DEFAULT; }
	xio_set_baud_usart(dev, baud);		// usart must be bound first

	dx->usart->CTRLB = (USART_TXEN_bm | USART_RXEN_bm);// enable tx and rx
	dx->usart->CTRLA = CTRLA_RXON_TXON;	// enable tx and rx IRQs

	dx->port->DIRCLR = dirclr;
	dx->port->DIRSET = dirset;
	dx->port->OUTCLR = outclr;
	dx->port->OUTSET = outset;
}

int xio_cntl_spi(const uint32_t control)
{
	return (NUL);
}

int xio_putc_spi(const char c, FILE *stream)
{
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


