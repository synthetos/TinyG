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
#include <avr/sleep.h>					// needed for blocking TX

#include "xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

/*
 * xio_init_usb() - initialization
 */
void xio_init_usb()
{
	xio_init_dev(XIO_DEV_USB, xio_open, xio_ctrl, xio_gets_usart, xio_getc_usart, xio_putc_usb);
	xio_init_usart(XIO_DEV_USB, USB_INIT_bm, &USB_USART, &USB_PORT, USB_DIRCLR_bm, USB_DIRSET_bm, USB_OUTCLR_bm, USB_OUTSET_bm);
}

/*
 * xio_putc_usb() 
 * USB_TX_ISR - USB transmitter interrupt (TX) used by xio_usb_putc()
 *
 * 	These are co-routines that work in tandem.
 * 	xio_putc_usb() is a more efficient form derived from xio_putc_usart()
 * 
 *	The TX interrupt dilemma: TX interrupts occur when the USART DATA register is 
 *	empty (and the ISR must disable interrupts when nothing's left to read, or they 
 *	keep firing). If the TX buffer is completely empty (TXCIF is set) then enabling
 *	interrupts does no good. The USART won't interrupt and the TX circular buffer 
 *	never empties. So the routine that puts chars in the TX buffer must always force
 *	an interrupt.
 */

int xio_putc_usb(const char c, FILE *stream)
{
	BUFFER_T next_tx_buf_head = USBu.tx_buf_head-1;		// set next head while leaving current one alone
	if (next_tx_buf_head == 0)
		next_tx_buf_head = TX_BUFFER_SIZE-1; 			// detect wrap and adjust; -1 avoids off-by-one
	while (next_tx_buf_head == USBu.tx_buf_tail) 
		sleep_mode(); 									// sleep until there is space in the buffer
	USBu.usart->CTRLA = CTRLA_RXON_TXOFF;				// disable TX interrupt (mutex region)
	USBu.tx_buf_head = next_tx_buf_head;				// accept next buffer head
	USBu.tx_buf[USBu.tx_buf_head] = c;					// write char to buffer

	// expand <LF> to <LF><CR> if $ec is set
	if (c == '\n') {
		if ((USB.flags & XIO_FLAG_CRLF_bm) != 0) {
			USBu.usart->CTRLA = CTRLA_RXON_TXON;		// force interrupt to send the queued <CR>
			BUFFER_T next_tx_buf_head = USBu.tx_buf_head-1;
			if (next_tx_buf_head == 0) 
				next_tx_buf_head = TX_BUFFER_SIZE-1;
			while (next_tx_buf_head == USBu.tx_buf_tail) 
				sleep_mode();
			USBu.usart->CTRLA = CTRLA_RXON_TXOFF;		// MUTEX region
			USBu.tx_buf_head = next_tx_buf_head;
			USBu.tx_buf[USBu.tx_buf_head] = CR;
		}
	}

	// finish up
	USBu.usart->CTRLA = CTRLA_RXON_TXON;			// force interrupt to send char(s) - doesn't work if you just |= it
	return (XIO_OK);
}

ISR(USB_TX_ISR_vect) //ISR(USARTC0_DRE_vect)		// USARTC0 data register empty
{
	if (USBu.fc_char == NUL) {						// normal char TX path
		if (USBu.tx_buf_head != USBu.tx_buf_tail) {	// buffer has data
			if (--USBu.tx_buf_tail == 0) {			// advance tail and wrap 
				USBu.tx_buf_tail = TX_BUFFER_SIZE-1;// -1 avoids OBOE
			}
			USBu.usart->DATA = USBu.tx_buf[USBu.tx_buf_tail];
		} else {
			USBu.usart->CTRLA = CTRLA_RXON_TXOFF;	// force another interrupt
		}
	} else {										// need to send XON or XOFF
	// comment out the XON/XOFF indicator for efficient ISR handling
	//	if (USBu.fc_char == XOFF) { gpio_set_bit_on(0x01);// turn on XOFF LED
	//	} else { gpio_set_bit_off(0x01);}				// turn off XOFF LED
		USBu.usart->DATA = USBu.fc_char;
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
 *  See https://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Module-Details#Notes_on_Circular_Buffers
 *  for a discussion of how the circular buffers work
 */

ISR(USB_RX_ISR_vect)	//ISR(USARTC0_RXC_vect)	// serial port C0 RX int 
{
	char c = USBu.usart->DATA;					// can only read DATA once

	// trap signals - do not insert character into RX queue
	if (c == CHAR_RESET) {	 					// trap Kill signal
		USB.signal = XIO_SIG_RESET;				// set signal value
		sig_reset();							// call sig handler
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
	if ((c == CR) && (USB.flag_ignorecr)) return;
	if ((c == LF) && (USB.flag_ignorelf)) return;

//	if ((c == CR) && (IGNORECR(USB.flags) == true)) { return;} +++++++++++++++++++
//	if ((c == LF) && (IGNORELF(USB.flags) == true)) { return;}


	// normal character path
	if ((--USBu.rx_buf_head) == 0) { 			// adv buffer head with wrap
		USBu.rx_buf_head = RX_BUFFER_SIZE-1;	// -1 avoids off-by-one error
	}
	if (USBu.rx_buf_head != USBu.rx_buf_tail) {	// buffer is not full
		USBu.rx_buf[USBu.rx_buf_head] = c;		// write char unless full
		if ((USB.flag_xoff) && (xio_get_rx_bufcount_usart(&USBu) > XOFF_RX_HI_WATER_MARK)) {
//		if ((EN_XOFF(USB.flags) == true) && (xio_get_rx_bufcount_usart(&USBu) > XOFF_RX_HI_WATER_MARK)) {
			xio_xoff_usart(XIO_DEV_USB);
		}
	} else { // buffer-full - toss the incoming character
		if ((++USBu.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
			USBu.rx_buf_head = 1;
		}
	}
}
