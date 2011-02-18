/*
 * xio_rs485.c 	- RS-485 device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <stdio.h>
//#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>	// needed if blocking character reads enabled
//#include <util/delay.h>

#include "xio.h"
#include "signals.h"			// application specific signal handlers
#include "xmega_interrupts.h"

#define RS ds[XIO_DEV_RS485]			// device struct accessoor
#define RSu us[XIO_DEV_RS485_OFFSET]	// usart extended struct accessor

/* RS485 device specific entry points to USART routines */
struct __file * xio_open_rs485() { return(RS.fdev); }
int xio_cntrl_rs485(const uint16_t control) {return xio_cntrl(XIO_DEV_RS485, control);} // See NOTE
int xio_putc_rs485(const char c, FILE *stream) {return xio_putc_usart(XIO_DEV_RS485, c, stream);}
int xio_getc_rs485(FILE *stream) {return xio_getc_usart(XIO_DEV_RS485, stream);}
int xio_gets_rs485(char *buf, const uint8_t size) {return xio_gets_usart(XIO_DEV_RS485, buf, size);}
void xio_queue_RX_char_rs485(const char c) {xio_queue_RX_char_usart(XIO_DEV_RS485, c);}
void xio_queue_RX_string_rs485(const char *buf) {xio_queue_RX_string_usart(XIO_DEV_RS485, buf);}

void xio_init_rs485()	// RS485 init
{
//	xio_init_dev(XIO_DEV_RS485, xio_open_rs485, xio_setflags_rs485, xio_putc_rs485, xio_getc_rs485, xio_gets_rs485);
	xio_init_dev(XIO_DEV_RS485, xio_open_rs485, xio_cntrl_rs485, xio_putc_rs485, xio_getc_rs485, xio_gets_rs485);
	xio_init_usart(XIO_DEV_RS485, XIO_DEV_RS485_OFFSET, RS485_INIT_bm, &RS485_USART, &RS485_PORT, RS485_DIRCLR_bm, RS485_DIRSET_bm, RS485_OUTCLR_bm, RS485_OUTSET_bm);
}

// NOTE: Might later expand setflags() to validate control bits and return errors

/* 
 * RS485_TX_ISR - RS485 transmitter interrupt (TX)
 *
 * 	The TX interrupt dilemma: TX interrupts occur when the USART DATA 
 *	register is empty (and the ISR must disable interrupts when nothing's
 *	left to read, or they keep firing). If the TX buffer is completely 
 *	empty (TXCIF is set) then enabling interrupts does no good. The USART 
 *	won't interrupt and the TX circular buffer never empties.
 *
 * 	So we define a dequeue function that can be called from either the ISR 
 *	or be called from the putc() if it detects TXCIfr. Care should be taken 
 *	to make sure these two callers don't collide (like only enabling 
 *	interrupts in putc() AFTER the dequeue has occurred).
 */

ISR(RS485_TX_ISR_vect)		//ISR(USARTC1_DRE_vect)	// USARTC1 data register empty
{
	if (RSu.tx_buf_head == RSu.tx_buf_tail) {	// buffer empty - disable ints (Note)
		RSu.usart->CTRLA = CTRLA_RXON_TXOFF_TXCON;	// doesn't work if you just &= it
//		PMIC_DisableLowLevel(); 				// disable USART TX interrupts
		return;
	}
	if (!TX_MUTEX(RS.flags)) {
		if (--(RSu.tx_buf_tail) == 0) {			// advance tail and wrap if needed
			RSu.tx_buf_tail = TX_BUFFER_SIZE-1;	// -1 avoids off-by-one error (OBOE)
		}
		RSu.port->OUTSET = (RS485_DE_bm | RS485_RE_bm);	// enable DE (TX, active hi)
														// disable RE (RX, active lo)
		RSu.usart->DATA = RSu.tx_buf[RSu.tx_buf_tail];	// write char to TX DATA reg
	}
}

/* Note: Finding a buffer empty condition on the first byte of a string is common 
 * as the TX byte is often written by the task itself */

ISR(RS485_TXC_ISR_vect)		// ISR(USARTC1_TXC_vect) // USARTC1 transmission complete
{
//	_delay_us(10);
	RSu.port->OUTCLR = (RS485_DE_bm | RS485_RE_bm);	// disable DE (TX), enable RE (RX)
}

/* 
 * RS485_RX_ISR - RS485 receiver interrupt (RX)
 */

ISR(RS485_RX_ISR_vect)	//ISR(USARTC1_RXC_vect)	// serial port C0 RX interrupt 
{
	uint8_t c = RSu.usart->DATA;				// can only read DATA once

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
