/*
 * xio_rs485.c 	- RS-485 device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
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
/*
 * The RS485 driver has some specializations in the putc() that requires
 * its own routines (in this file). As with other devices, its interrupts
 * are also in this file.
 *
 * The RS485 driver is a half duplex driver that works over a single A/B
 * differential pair. So the USART can only be in RX or TX mode at any
 * given time, never both. Most of the specialized logic in the RS485
 * driver deals with this constraint.
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/sleep.h>					// needed for blocking character writes

#include "../xio.h"
#include "../xmega/xmega_interrupts.h"

#include "../tinyg.h"					// needed for canonical machine
#include "../hardware.h"				// needed for hardware reset
#include "../controller.h"				// needed for trapping kill char
#include "../canonical_machine.h"		// needed for fgeedhold and cycle start

// Fast accessors
#define RS ds[XIO_DEV_RS485]
#define RSu us[XIO_DEV_RS485 - XIO_DEV_USART_OFFSET]

/*
 * Helper functions
 *	xio_enable_rs485_tx() - specialized routine to enable rs488 TX mode
 *	xio_enable_rs485_rx() - specialized routine to enable rs488 RX mode
 *
 *	enables one mode and disables the other
 */
void xio_enable_rs485_tx()
{
	// enable TX, related interrupts & set DE and RE lines (disabling RX)
	RSu.usart->CTRLB = USART_TXEN_bm;
	RSu.usart->CTRLA = CTRLA_RXOFF_TXON_TXCON;
	RSu.port->OUTSET = (RS485_DE_bm | RS485_RE_bm);
}

void xio_enable_rs485_rx()
{
	// enable RX, related interrupts & clr DE and RE lines (disabling TX)
	RSu.usart->CTRLB = USART_RXEN_bm;
	RSu.usart->CTRLA = CTRLA_RXON_TXOFF_TXCON;
	RSu.port->OUTCLR = (RS485_DE_bm | RS485_RE_bm);
}

/*
 * xio_putc_rs485() - stdio compatible char writer for rs485 devices
 *
 * 	The TX putc() / interrupt dilemma: TX interrupts occur when the
 *	USART DATA register is empty (ready for TX data) - and will keep
 *	firing as long as the TX buffer is completely empty (ready for TX
 *	data). So putc() and its ISR henchmen must disable interrupts when
 *	there's nothing left to write or they will just keep firing.
 *
 *	To make matters worse, for some reason if you enable the TX interrupts
 *	and TX DATA is ready, it won't actually generate an interrupt. Putc()
 *	must "prime" the first write itself. This requires a mutual exclusion
 *	region around the dequeue operation to make sure the ISR and main
 *	routines don't collide.
 *
 *	Lastly, the system must detect the end of transmission (TX complete)
 *	to know when to revert the RS485 driver to RX mode. So there are 2 TX
 *	interrupt conditions and handlers, not 1 like other USART TXs.
 *
 *	NOTE: Finding a buffer empty condition on the first byte of a string
 *		  is common as the TX byte is often written by the task itself.
 */
int xio_putc_rs485(const char c, FILE *stream)
{
	buffer_t next_tx_buf_head;

	if ((next_tx_buf_head = (RSu.tx_buf_head)-1) == 0) { // adv. head & wrap
		next_tx_buf_head = TX_BUFFER_SIZE-1;	 // -1 avoids the off-by-one
	}
	while(next_tx_buf_head == RSu.tx_buf_tail) { // buf full. sleep or ret
		if (RS.flag_block) {
			sleep_mode();
		} else {
			RS.signal = XIO_SIG_EAGAIN;
			return(_FDEV_ERR);
		}
	};
	// enable TX mode and write data to TX buffer
	xio_enable_rs485_tx();							// enable for TX
	RSu.tx_buf_head = next_tx_buf_head;				// accept next buffer head
	RSu.tx_buf[RSu.tx_buf_head] = c;				// ...write char to buffer

	if ((c == '\n') && (RS.flag_crlf)) {			// detect LF & add CR
		return RS.x_putc('\r', stream);				// recurse
	}
	// force a TX interupt to attempt to send the character
	RSu.usart->CTRLA = CTRLA_RXON_TXON;				// doesn't work if you just |= it
	return (XIO_OK);
}

/*
 * RS485_TX_ISR - RS485 transmitter interrupt (TX)
 * RS485_TXC_ISR - RS485 transmission complete (See notes in xio_putc_rs485)
 */

ISR(RS485_TX_ISR_vect)		//ISR(USARTC1_DRE_vect)	// USARTC1 data register empty
{
	// NOTE: Assumes the USART is in TX mode before this interrupt is fired
	if (RSu.tx_buf_head == RSu.tx_buf_tail) {		// buffer empty - disable ints (NOTE)
		RSu.usart->CTRLA = CTRLA_RXON_TXOFF_TXCON;	// doesn't work if you just &= it
		return;
	}
	if (--(RSu.tx_buf_tail) == 0) {					// advance tail and wrap if needed
		RSu.tx_buf_tail = TX_BUFFER_SIZE-1;			// -1 avoids off-by-one error (OBOE)
	}
	RSu.usart->DATA = RSu.tx_buf[RSu.tx_buf_tail];	// write char to TX DATA reg
}

ISR(RS485_TXC_ISR_vect)	// ISR(USARTC1_TXC_vect)
{
	xio_enable_rs485_rx();							// revert to RX mode
}

/*
 * RS485_RX_ISR - RS485 receiver interrupt (RX)
 */

ISR(RS485_RX_ISR_vect)	//ISR(USARTC1_RXC_vect)		// serial port C0 RX isr
{
	char c;

	if ((RSu.usart->STATUS & USART_RX_DATA_READY_bm) != 0) {
		c = RSu.usart->DATA;						// can only read DATA once
	} else {
		return;										// shouldn't ever happen; bit of a fail-safe here
	}

	// trap async commands - do not insert into RX queue
	if (c == CHAR_RESET) {	 						// trap Kill character
		hw_request_hard_reset();
		return;
	}
	if (c == CHAR_FEEDHOLD) {						// trap feedhold signal
		cm_request_feedhold();
		return;
	}
	if (c == CHAR_CYCLE_START) {					// trap end_feedhold signal
		cm_request_cycle_start();
		return;
	}
	// filter out CRs and LFs if they are to be ignored
	if ((c == CR) && (RS.flag_ignorecr)) return;
	if ((c == LF) && (RS.flag_ignorelf)) return;

	// normal character path
	advance_buffer(RSu.rx_buf_head, RX_BUFFER_SIZE);
	if (RSu.rx_buf_head != RSu.rx_buf_tail) {		// write char unless buffer full
		RSu.rx_buf[RSu.rx_buf_head] = c;			// (= USARTC1.DATA;)
		RSu.rx_buf_count++;
		// flow control detection goes here - should it be necessary
		return;
	}
	// buffer-full handling
	if ((++RSu.rx_buf_head) > RX_BUFFER_SIZE -1) {	// reset the head
		RSu.rx_buf_count = RX_BUFFER_SIZE-1;		// reset count for good measure
		RSu.rx_buf_head = 1;
	}
}
