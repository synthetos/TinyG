/*
 * xio_rs485.c 	- RS-485 device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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

#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>		// needed for blocking character writes
#include <util/delay.h>		// +++ used to test DE timing

#include "xio.h"
#include "signals.h"			// application specific signal handlers
#include "xmega_interrupts.h"

#define RS ds[XIO_DEV_RS485]			// device struct accessoor
#define RSu us[XIO_DEV_RS485_OFFSET]	// usart extended struct accessor

// local helper functions
static void _xio_enable_rs485_tx(void);	// enable rs485 TX mode (no RX)
static void _xio_enable_rs485_rx(void);	// enable rs485 RX mode (no TX)

// RS485 device drivers that fall back to generic USART routines
struct __file * xio_open_rs485() { return(RS.fdev); }
int xio_cntrl_rs485(const uint16_t control) {return xio_cntrl(XIO_DEV_RS485, control);} // See NOTE
int xio_getc_rs485(FILE *stream) {return xio_getc_usart(XIO_DEV_RS485, stream);}
int xio_gets_rs485(char *buf, const uint8_t size) {return xio_gets_usart(XIO_DEV_RS485, buf, size);}
void xio_queue_RX_char_rs485(const char c) {xio_queue_RX_char_usart(XIO_DEV_RS485, c);}
void xio_queue_RX_string_rs485(const char *buf) {xio_queue_RX_string_usart(XIO_DEV_RS485, buf);}

// RS485 device-specifc drivers 

/*
 * xio_init_rs485() - initialization
 */
void xio_init_rs485()	// RS485 init
{
	xio_init_dev(XIO_DEV_RS485, xio_open_rs485, xio_cntrl_rs485, xio_putc_rs485, xio_getc_rs485, xio_gets_rs485);
	xio_init_usart(XIO_DEV_RS485, XIO_DEV_RS485_OFFSET, RS485_INIT_bm, &RS485_USART, &RS485_PORT, RS485_DIRCLR_bm, RS485_DIRSET_bm, RS485_OUTCLR_bm, RS485_OUTSET_bm);
	_xio_enable_rs485_rx(); // set initially for RX mode
}

/*
 *	_xio_enable_rs485_tx() - specialized routine to enable rs488 TX mode
 *	_xio_enable_rs485_rx() - specialized routine to enable rs488 RX mode
 *
 *	enables one mode and disables the other
 */
static void _xio_enable_rs485_tx()
{
	// enable TX, related interrupts & set DE and RE lines (disabling RX) 
	RSu.usart->CTRLB = USART_TXEN_bm;
	RSu.usart->CTRLA = CTRLA_RXOFF_TXON_TXCON;
	RSu.port->OUTSET = (RS485_DE_bm | RS485_RE_bm);
}

static void _xio_enable_rs485_rx()
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
 *	interrupt conditions and handlers, not 1 like all the other USART TXs.
 *
 *	NOTE: Finding a buffer empty condition on the first byte of a string 
 *		  is common as the TX byte is often written by the task itself.
 */

int xio_putc_rs485(const char c, FILE *stream)
{
	if ((RSu.next_tx_buf_head = (RSu.tx_buf_head)-1) == 0) { // adv. head & wrap
		RSu.next_tx_buf_head = TX_BUFFER_SIZE-1;	 // -1 avoids the off-by-one
	}
	while(RSu.next_tx_buf_head == RSu.tx_buf_tail) { // buf full. sleep or ret
		if (BLOCKING(RS.flags)) {
			sleep_mode();
		} else {
			RS.sig = XIO_SIG_EAGAIN;
			return(_FDEV_ERR);
		}
	};
	// enable TX mode and write data to TX buffer
	_xio_enable_rs485_tx();							// enable for TX
	RSu.tx_buf_head = RSu.next_tx_buf_head;			// accept next buffer head
	RSu.tx_buf[RSu.tx_buf_head] = c;				// ...write char to buffer

	if (CRLF(RS.flags) && (c == '\n')) {			// detect LF & add CR
		return RS.x_putc('\r', stream);				// recurse
	}

	// dequeue the buffer if TX DATA register is ready
	if (RSu.usart->STATUS & USART_TX_REGISTER_READY_bm) { // 0x20
		if (RSu.tx_buf_head == RSu.tx_buf_tail) {// buf might be empty if IRQ got it
			return (XIO_OK);
		}
		RS.flags |= XIO_FLAG_TX_MUTEX_bm;			// mutual exclusion from ISR
		if (--(RSu.tx_buf_tail) == 0) {				// advance tail & wrap if needed
			RSu.tx_buf_tail = TX_BUFFER_SIZE-1;		// -1 avoid off-by-one err (OBOE)
		}
		RSu.usart->DATA = RSu.tx_buf[RSu.tx_buf_tail];// write to TX DATA reg
		RS.flags &= ~XIO_FLAG_TX_MUTEX_bm;			// release mutual exclusion lock
	}
	PMIC_EnableLowLevel(); 							// enable USART TX interrupts
	sei();											// enable global interrupts
	return (XIO_OK);
}

/* 
 * RS485_TX_ISR - RS485 transmitter interrupt (TX)
 * RS485_TXC_ISR - RS485 transmission complete
 *	(See notes in xio_putc_rs485)
 */

ISR(RS485_TX_ISR_vect)		//ISR(USARTC1_DRE_vect)	// USARTC1 data register empty
{
	// NOTE: Assumes the USART is in TX mode before this interrupt is fired 
	if (RSu.tx_buf_head == RSu.tx_buf_tail) {	// buffer empty - disable ints (NOTE)
		RSu.usart->CTRLA = CTRLA_RXON_TXOFF_TXCON;	// doesn't work if you just &= it
//		PMIC_DisableLowLevel(); 				// disable USART TX interrupts
		return;
	}
	if (!TX_MUTEX(RS.flags)) {
		if (--(RSu.tx_buf_tail) == 0) {			// advance tail and wrap if needed
			RSu.tx_buf_tail = TX_BUFFER_SIZE-1;	// -1 avoids off-by-one error (OBOE)
		}
		RSu.usart->DATA = RSu.tx_buf[RSu.tx_buf_tail];	// write char to TX DATA reg
	}
}

ISR(RS485_TXC_ISR_vect)		// ISR(USARTC1_TXC_vect) 
{
	_xio_enable_rs485_rx();	// revert to RX mode
}

/* 
 * RS485_RX_ISR - RS485 receiver interrupt (RX)
 */

ISR(RS485_RX_ISR_vect)	//ISR(USARTC1_RXC_vect)	// serial port C0 RX isr 
{
	uint8_t c;

	if (RSu.usart->STATUS & USART_RX_DATA_READY_bm) {
		c = RSu.usart->DATA;					// can only read DATA once
	} else {
		return;			// shouldn't ever happen; bit of a fail-safe here
	}

	// trap signals - do not insert into RX queue
	if (c == SIG_KILL_CHAR) {	 				// trap Kill signal
		RS.sig = XIO_SIG_KILL;					// set signal value
		sig_kill();								// call app-specific sig handler
		return;
	}
	if (c == SIG_TERM_CHAR) {					// trap Terminate signal
		RS.sig = XIO_SIG_KILL;
		sig_term();
		return;
	}
	if (c == SIG_PAUSE_CHAR) {					// trap Pause signal
		RS.sig = XIO_SIG_PAUSE;
		sig_pause();
		return;
	}
	if (c == SIG_RESUME_CHAR) {					// trap Resume signal
		RS.sig = XIO_SIG_RESUME;
		sig_resume();
		return;
	}

	// normal path
	if ((--RSu.rx_buf_head) == 0) { 			// advance buffer head with wrap
		RSu.rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids the off-by-one error
	}
	if (RSu.rx_buf_head != RSu.rx_buf_tail) {	// write char unless buffer full
		RSu.rx_buf[RSu.rx_buf_head] = c;		// (= USARTC1.DATA;)
		return;
	}
	// buffer-full handling
	if ((++RSu.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		RSu.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}

