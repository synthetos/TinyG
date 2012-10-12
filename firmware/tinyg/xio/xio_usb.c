/*
 * xio_usb.c	- FTDI USB device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
 *------
 *
 *	This version implements signal capture at the ISR level
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
//#include <avr/sleep.h>				// needed if blocking reads & writes are used

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"
#include "../gpio.h"

//#define USB ds[XIO_DEV_USB]			// device struct accessor
//#define USBu us[XIO_DEV_USB_OFFSET]	// usart extended struct accessor

/* USB device wrappers for generic USART routines */
FILE * xio_open_usb() {return(USB.fdev);}
int xio_cntl_usb(const uint32_t control) {return xio_cntl(XIO_DEV_USB, control);} // SEE NOTE
int xio_putc_usb(const char c, FILE *stream) {return xio_putc_usart(XIO_DEV_USB, c, stream);}
int xio_getc_usb(FILE *stream) {return xio_getc_usart(XIO_DEV_USB, stream);}
int xio_gets_usb(char *buf, const int size) {return xio_gets_usart(XIO_DEV_USB, buf, size);}
void xio_queue_RX_char_usb(const char c) {xio_queue_RX_char_usart(XIO_DEV_USB, c);}
void xio_queue_RX_string_usb(const char *buf) {xio_queue_RX_string_usart(XIO_DEV_USB, buf);}

void xio_init_usb()	// USB inits
{
	xio_init_dev(XIO_DEV_USB, xio_open_usb, xio_cntl_usb, xio_putc_usb, xio_getc_usb, xio_gets_usb);
	xio_init_usart(XIO_DEV_USB, XIO_DEV_USB_OFFSET, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
}

// NOTE: Might later expand setflags() to validate control bits and return errors


/* 
 * USB_TX_ISR - USB transmitter interrupt (TX)
 *
 *	The TX interrupt dilemma: TX interrupts occur when the USART DATA register is 
 *	empty (and the ISR must disable interrupts when nothing's left to read, or they 
 *	keep firing). If the TX buffer is completely empty (TXCIF is set) then enabling
 *	interrupts does no good. The USART won't interrupt and the TX circular buffer 
 *	never empties. So the routine that puts chars in the TX buffer must always force
 *	an interrupt.
 */

ISR(USB_TX_ISR_vect)	//ISR(USARTC0_DRE_vect)	// USARTC0 data register empty
{
	if (USBu.fc_char == NUL) {						// normal char TX path
		if (USBu.tx_buf_head != USBu.tx_buf_tail) {	// buffer has data
			if (--(USBu.tx_buf_tail) == 0) {		// advance tail and wrap 
				USBu.tx_buf_tail = TX_BUFFER_SIZE-1;// -1 avoids OBOE
			}
			USBu.usart->DATA = USBu.tx_buf[USBu.tx_buf_tail];
		} else {
			USBu.usart->CTRLA = CTRLA_RXON_TXOFF;	// force another interrupt
		}
	} else {										// need to send XON or XOFF
		USBu.usart->DATA = USBu.fc_char;
		if (USBu.fc_char == XOFF) {
			gpio_set_bit_on(0x01);					// turn on XOFF LED
		} else {
			gpio_set_bit_off(0x01);					// turn off XOFF LED
		}
		USBu.fc_char = NUL;
	}
}

/* 
 * USB_RX_ISR - USB receiver interrupt (RX)
 *
 * RX buffer states can be one of:
 *	- buffer has space	(CTS should be asserted)
 *	- buffer is full 	(CTS should be not_asserted)
 *	- buffer becomes full with this character (write char and assert CTS)
 *
 * Signals:
 *	- Signals are captured at the ISR level and either dispatched or flag-set
 *	- As RX ISR is a critical code region signal handling is stupid and fast
 *	- signal characters are not put in the RX buffer
 *
 * Flow Control:
 *	- Flow control is not implemented. Need to work RTS line.
 *	- Flow control should cut off at high water mark, re-enable at low water mark
 *	- High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
 *	- Low water mark about 50% full
 *
 * 	See end notes in xio.h for a discussion of how the circular buffers work
 */

ISR(USB_RX_ISR_vect)	//ISR(USARTC0_RXC_vect)	// serial port C0 RX int 
{
	char c = USBu.usart->DATA;					// can only read DATA once

	// trap signals - do not insert character into RX queue
	if (c == CHAR_ABORT) {	 					// trap Kill signal
		USB.signal = XIO_SIG_ABORT;				// set signal value
		sig_abort();							// call sig handler
		return;
	}
	if (c == CHAR_FEEDHOLD) {					// trap feedhold signal
		USB.signal = XIO_SIG_FEEDHOLD;
		sig_feedhold();
		return;
	}
	if (c == CHAR_CYCLE_START) {				// trap cycle start signal
		USB.signal = XIO_SIG_CYCLE_START;
		sig_cycle_start();
		return;
	}
	// filter out CRs and LFs if they are to be ignored
	if ((IGNORECR(USB.flags) == true) && (c == CR)) { return;}
	if ((IGNORELF(USB.flags) == true) && (c == LF)) { return;}

	// normal character path
	if ((--USBu.rx_buf_head) == 0) { 			// adv buffer head with wrap
		USBu.rx_buf_head = RX_BUFFER_SIZE-1;	// -1 avoids off-by-one error
	}
	if (USBu.rx_buf_head != USBu.rx_buf_tail) {	// buffer is not full
		USBu.rx_buf[USBu.rx_buf_head] = c;		// write char unless full
		if ((EN_XOFF(USB.flags) == true) && (xio_get_rx_bufcount_usart(&USBu) > XOFF_RX_HI_WATER_MARK)) {
			xio_xoff_usart(XIO_DEV_USB);
		}
	} else { // buffer-full - toss the incoming character
		if ((++USBu.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
			USBu.rx_buf_head = 1;
		}
	}
}
