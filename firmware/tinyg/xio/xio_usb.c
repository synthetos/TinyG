/*
 * xio_usb.c - FTDI USB device driver for xmega family
 * 			 - works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart Jr.
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

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking TX

#include "../xio.h"						// includes for all devices are in here
#include "../xmega/xmega_interrupts.h"

// application specific stuff that's littered into the USB handler
#include "../tinyg.h"
#include "../config.h"					// needed to find flow control setting
#include "../hardware.h"
#include "../controller.h"
#include "../canonical_machine.h"		// trapped characters communicate directly with the canonical machine

// Resume and queue flush are recognized between lines (leading space is allowed).
// Feedhold remains immediate even mid-line; it does not change the framing state.
// Terminate a partial line before resuming or flushing. Reset/enquiry/flow control are unchanged.
static volatile bool rx_in_line;

// Framing heuristic (E195): a line in progress that has had no byte for about a second is taken to be
// left by a host that went away, and a '{' after it (how a host opens a session) starts a fresh line.
// The RX interrupt puts RX_STALE_LINE_MARK in the ring ahead of that '{', so every such boundary
// travels with the bytes, and the reader drops the stale start when it gets there. Any host pause of
// that length just before a '{' inside a line has the same effect.
#define RX_STALE_LINE_TICKS 103					// RTC ticks (9.77 ms each): just over a second
static volatile uint8_t rx_quiet_ticks;			// RTC ticks since the last byte for the ring, saturating

void xio_usb_rtc_callback(void)					// RTC interrupt (lo)
{
	uint8_t saved_sreg = SREG;					// the RX interrupt (med) zeroes the count: keep the increment whole
	cli();
	if (rx_quiet_ticks < UINT8_MAX) {
		rx_quiet_ticks++;
	}
	SREG = saved_sreg;
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

static int _putc_usb_raw(const char c)
{
	buffer_t next_tx_buf_head = USBu.tx_buf_head-1;		// set next head while leaving current one alone
	if (next_tx_buf_head == 0)
		next_tx_buf_head = TX_BUFFER_SIZE-1; 			// detect wrap and adjust; -1 avoids off-by-one
	while (next_tx_buf_head == USBu.tx_buf_tail)
		sleep_mode(); 									// sleep until there is space in the buffer
	USBu.usart->CTRLA = CTRLA_RXON_TXOFF;				// disable TX interrupt (mutex region)
	USBu.tx_buf[next_tx_buf_head] = c;					// write char to buffer, then publish it: the RX
	USBu.tx_buf_head = next_tx_buf_head;				// ISR (XON) or CTS ISR may re-enable TX at any time

	// expand <LF> to <LF><CR> if $ec is set
	if ((c == '\n') && (USB.flag_crlf)) {
		USBu.usart->CTRLA = CTRLA_RXON_TXON;			// force interrupt to send the queued <CR>
		buffer_t next_tx_buf_head = USBu.tx_buf_head-1;
		if (next_tx_buf_head == 0) next_tx_buf_head = TX_BUFFER_SIZE-1;
		while (next_tx_buf_head == USBu.tx_buf_tail) sleep_mode();
		USBu.usart->CTRLA = CTRLA_RXON_TXOFF;			// MUTEX region
		USBu.tx_buf[next_tx_buf_head] = CR;
		USBu.tx_buf_head = next_tx_buf_head;
	}
	// finish up
	USBu.usart->CTRLA = CTRLA_RXON_TXON;			// force interrupt to send char(s) - doesn't work if you just |= it
	return (XIO_OK);
}

int xio_putc_usb(const char c, FILE *stream)
{
	if (xio.json_escape) {						// inside a {"txt":"..."} string: keep the reply one valid JSON line (E137)
		if ((c == '"') || (c == '\\')) { _putc_usb_raw('\\'); return (_putc_usb_raw(c)); }
		if (c == '\n') { _putc_usb_raw('\\'); return (_putc_usb_raw('n')); }
		if (c == '\t') { _putc_usb_raw('\\'); return (_putc_usb_raw('t')); }
		if ((uint8_t)c < 0x20) { return (XIO_OK); }	// other control bytes have no place in the string
	}
	return (_putc_usb_raw(c));
}

ISR(USB_TX_ISR_vect) //ISR(USARTC0_DRE_vect)		// USARTC0 data register empty
{
	// If the CTS pin (FTDI's RTS) is HIGH, then we cannot send anything, so exit
	if ((xio.enable_flow_control == FLOW_CONTROL_RTS) && (USBu.port->IN & USB_CTS_bm)) {
		USBu.usart->CTRLA = CTRLA_RXON_TXOFF;		// stop TX interrupts; the CTS edge ISR re-enables them
		return;
	}

	// Send an RX-side XON or XOFF character if queued
	if (USBu.fc_char_rx != NUL) {					// an XON/ of XOFF needs to be sent
		USBu.usart->DATA = USBu.fc_char_rx;			// send the XON/XOFF char and exit
		USBu.fc_char_rx = NUL;
		return;
	}

	// Halt transmission while in TX-side XOFF. Disable the interrupt, or it re-fires continuously;
	// XON and the next xio_putc_usb() re-enable it.
	if (USBu.fc_state_tx == FC_IN_XOFF) {
		USBu.usart->CTRLA = CTRLA_RXON_TXOFF;
		return;
	}

	// Otherwise process normal TX transmission
	if (USBu.tx_buf_head != USBu.tx_buf_tail) {		// buffer has data
		advance_buffer(USBu.tx_buf_tail, TX_BUFFER_SIZE);
		USBu.usart->DATA = USBu.tx_buf[USBu.tx_buf_tail];
	} else {
		USBu.usart->CTRLA = CTRLA_RXON_TXOFF;		// buffer empty: stop TX interrupts until xio_putc_usb()
	}
}

/*
 * Pin Change (edge-detect) interrupt for CTS pin.
 */

ISR(USB_CTS_ISR_vect)
{
	USBu.usart->CTRLA = CTRLA_RXON_TXON;		// force another interrupt
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
 */

// Remember where input was lost: the reader must not splice what follows onto this line (E162)
static void _usb_rx_lost(void)				// lost right after the last byte kept, at the head
{
	if (!USBu.rx_lost) {
		USBu.rx_lost = true;
		USBu.rx_lost_first = USBu.rx_buf_head;
	}
	USBu.rx_lost_last = USBu.rx_buf_head;
	if (USBu.rx_flush_marked && !USBu.rx_lost_after_mark) {	// lost after a '%': the flush must keep it
		USBu.rx_lost_after_mark = true;
		USBu.rx_lost_post_first = USBu.rx_buf_head;		// may equal the mark if nothing was kept since
	}
	if (USBu.rx_buf_head == USBu.rx_buf_tail) {		// the reader took that byte (only a USART loss, E193):
		USBu.rx_lost_open = true;						// the loss is where it reads now, so the stretch is
		advance_buffer(USBu.rx_lost_last, RX_BUFFER_SIZE);	// open and ends at the next byte kept, or the
	}												// reader reports it if the ring is still empty
}

ISR(USB_RX_ISR_vect)	//ISR(USARTC0_RXC_vect)	// serial port C0 RX int
{
	uint8_t status = USBu.usart->STATUS;		// read before DATA: the error flags travel with the frame
	char c = USBu.usart->DATA;					// can only read DATA once

	if (status & USART_BUFOVF_bm) {				// the USART lost a frame before this one while this
		_usb_rx_lost();							// interrupt could not run (E193)
	}

	// trap async commands - do not insert character into RX queue
	if (c == CHAR_RESET) {	 					// trap Kill signal
		hw_hard_reset();             // a blocked main loop must not defer reset
		return;
	}
	if (c == CHAR_FEEDHOLD) {					// trap feedhold signal
		cm_request_feedhold();
		return;
	}
	if (c == CHAR_KILL_JOB) {					// trap job kill (^d), in or out of a line: alarm from the main loop (E096)
		cm_request_job_kill();
		return;
	}
	if (!rx_in_line && c == CHAR_QUEUE_FLUSH) {				// trap queue flush signal
		cm_request_queue_flush();
		if (cm.queue_flush_input) {				// honored: what the ring holds now came before the '%'
			USBu.rx_flush_mark = USBu.rx_buf_head;	// the flush drops up to here, not what follows (E146)
			USBu.rx_flush_marked = true;
			USBu.rx_lost_after_mark = false;		// losses so far came before the '%' (E162)
		}
		return;
	}
	if (!rx_in_line && c == CHAR_END_HOLD) {				    // trap cycle start signal
		cm_request_end_hold();
		return;
	}
	if (c == CHAR_ENQUIRY) {				    // trap ENC request
    	controller_request_enquiry();
    	return;
	}
	if (USB.flag_xoff && xio.enable_flow_control == FLOW_CONTROL_XON) {
		if (c == XOFF) {						// trap incoming XON/XOFF signals
			USBu.fc_state_tx = FC_IN_XOFF;
			return;
		}
		if (c == XON) {
			USBu.fc_state_tx = FC_IN_XON;
			USBu.usart->CTRLA = CTRLA_RXON_TXON;	// re-enable the TX interrupt to resume sending
			return;
		}
	}

    if (c == RX_STALE_LINE_MARK) {				// reserved for the ring (E195): a host's own is dropped
        return;
    }
    bool stale = (c == '{') && rx_in_line && (rx_quiet_ticks >= RX_STALE_LINE_TICKS);	// E195
    rx_quiet_ticks = 0;

    // Keep reset, enquiry and flow control independent of line framing.
    if (c == '\r' || c == '\n') {
        rx_in_line = false;
    } else if (c != ' ' && c != '\t') {
        rx_in_line = true;
    }

	// normal character path; a stale line's mark goes in ahead of the '{' (E195)
	for (uint8_t n = (stale ? 2 : 1); n > 0; n--) {
		char k = ((n == 2) ? RX_STALE_LINE_MARK : c);
		advance_buffer(USBu.rx_buf_head, RX_BUFFER_SIZE);
		if (USBu.rx_buf_head != USBu.rx_buf_tail) {	// buffer is not full
			USBu.rx_buf[USBu.rx_buf_head] = k;		// write char unless full
			USBu.rx_buf_count++;
			if ((USB.flag_xoff) && (xio_get_rx_bufcount_usart(&USBu) > XOFF_RX_HI_WATER_MARK)) {
				xio_xoff_usart(&USBu);
			}
		} else { 										// buffer-full - toss the incoming character
			if ((++USBu.rx_buf_head) > RX_BUFFER_SIZE-1) {	// reset the head
				USBu.rx_buf_count = RX_BUFFER_SIZE-1;	// reset count for good measure
				USBu.rx_buf_head = 1;
			}
			_usb_rx_lost();
		}
	}
}

/*
 * xio_get_usb_rx_free() - returns free space in the USB RX buffer
 *
 *	Remember: The queues fill from top to bottom, w/0 being the wrap location
 */
buffer_t xio_get_usb_rx_free(void)
{
	return (RX_BUFFER_SIZE - xio_get_rx_bufcount_usart(&USBu));
}

/*
 * xio_reset_usb_rx_buffers() - clears the USB RX buffer
 */
void xio_reset_usb_rx_buffers(void)
{
    rx_in_line = false;
	// reset xio_gets() buffer
	USB.len = 0;
	USB.flag_in_line = false;

	// reset RX interrupt circular buffer
	USBu.rx_buf_head = 1;		// can't use location 0 in circular buffer
	USBu.rx_buf_tail = 1;
	USBu.rx_flush_marked = false;
	USBu.rx_lost = false;		// nothing kept: no lost input to account for (E162)
	USBu.rx_lost_after_mark = false;
	USBu.rx_lost_open = false;
	USBu.rx_line_damaged = false;
	USBu.rx_gap_report = false;
}

/*
 * xio_drop_usb_rx_before_mark() - drop what arrived before a host '%', keep what followed (E146)
 *
 *	Caller masks interrupts. Unread ring bytes up to the mark and the line being read go; bytes the
 *	host sent after the '%' stay in the ring. With no mark (a '%' line or {qf}) everything goes.
 */
void xio_drop_usb_rx_before_mark(void)
{
	if (!USBu.rx_flush_marked) {
		xio_reset_usb_rx_buffers();
		return;
	}
	// Lost input before the '%' goes with the dropped bytes; lost input after it still counts (E162).
	// The ISR records the order, since a loss right after the '%' sits at the mark itself.
	USBu.rx_lost_open = false;
	USBu.rx_line_damaged = false;
	USBu.rx_gap_report = false;
	if (!USBu.rx_lost_after_mark) {
		USBu.rx_lost = false;
	} else if (USBu.rx_lost_post_first == USBu.rx_flush_mark) {	// lost right after the '%': the first line read lost its head
		USBu.rx_line_damaged = true;
		USBu.rx_gap_report = true;								// and the loss is reported at once
		USBu.rx_lost_open = (USBu.rx_lost_last != USBu.rx_flush_mark);	// later losses: inside the stretch from the start
		USBu.rx_lost = USBu.rx_lost_open;
	} else {
		USBu.rx_lost_first = USBu.rx_lost_post_first;
	}
	USBu.rx_lost_after_mark = false;
	USBu.rx_buf_tail = USBu.rx_flush_mark;
	USBu.rx_flush_marked = false;
	USBu.rx_buf_count = xio_get_rx_bufcount_usart(&USBu);
	USB.len = 0;				// the partial line was read before the mark
	USB.flag_in_line = false;	// rx_in_line stays: it follows the bytes kept in the ring
}
