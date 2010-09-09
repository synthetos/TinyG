/*
  xio_usb.c - FTDI USB device driver for xmega family
  			- works with avr-gcc stdio library

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>			// needed for blocking character reads

#include "xmega_support.h"		// put this early as it has the F_CPU value
#include "xio.h"
#include "xio_usb.h"

extern uint8_t bsel[];			// shared baud rate selection values
extern uint8_t bscale[];		// shared baud rate scaling values

static struct xioUSART f;		// create a local USART control struct
								// *** must be static or collisions occur ***
/* 
 *	xio_usb_init() - initialize and set controls for USB device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_RD		  <null>	Y	Enable device for reads
 *	XIO_WR		  <null>	Y	Enable device for write
 *	XIO_BLOCK	  <null>	Y	Enable blocking reads
 *	XIO_NOBLOCK   <null>		Disable blocking reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_CRLF	  <null>		Send <cr><lf> if <lf> detected
 *	XIO_NOCRLF	  <null>	Y	Do not convert <lf> to <cr><lf>
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 *
 *	XIO_BAUD_xxxxx <null>		One of the supported baud rate enums
 */

void xio_usb_init(uint16_t control)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// transfer control flags to internal flag bits
	f.flags = XIO_FLAG_DEFAULT_gm;			// set flags to defaults & initial state
	if (control & XIO_RD) {
		f.flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_WR) {
		f.flags |= XIO_FLAG_WR_bm;
	}
	if (control & XIO_BLOCK) {
		f.flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		f.flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		f.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		f.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		f.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		f.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		f.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		f.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		f.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		f.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	// setup internal RX/TX buffers
	f.rx_buf_head = 1;						// can't use location 0
	f.rx_buf_tail = 1;
	f.tx_buf_head = 1;
	f.tx_buf_tail = 1;

	// device assignment
	f.usart = &USB_USART;					// bind USART
	f.port = &USB_PORT;						// bind PORT

	// baud rate and USART setup
	if (baud == XIO_BAUD_UNSPECIFIED) {
		baud = XIO_BAUD_DEFAULT;
	}
	f.usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	f.usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	f.usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	f.usart->CTRLA = USART_RXCINTLVL_MED_gc;		 // receive interrupt medium level

	f.port->DIRCLR = USB_RX_bm;	 			// clr RX pin as input
	f.port->DIRSET = USB_TX_bm; 			// set TX pin as output
	f.port->OUTSET = USB_TX_bm;				// set TX HI as initial state
	f.port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
	f.port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
	f.port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
//	f.port->OUTCLR = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
}

/*	
 *	xio_usb_control() - set controls for USB device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_BLOCK	  <null>	Y	Enable blocking reads
 *	XIO_NOBLOCK   <null>		Disable blocking reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_CRLF	  <null>		Send <cr><lf> if <lf> detected
 *	XIO_NOCRLF	  <null>	Y	Do not convert <lf> to <cr><lf>
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 *
 *	XIO_BAUD_xxxxx	<null>		One of the supported baud rate enums
 */

int8_t xio_usb_control(uint16_t control, int16_t arg)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// group 1 commands (do not have argument)
	if (baud != XIO_BAUD_UNSPECIFIED) {
		f.usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
		f.usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	}
	if (control & XIO_BLOCK) {
		f.flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		f.flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		f.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		f.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		f.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		f.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		f.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		f.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		f.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		f.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	// group 2 commands (have argument)	// no argument commands for this device
	return (0);
}

/* 
 * USB_RX_ISR - USB receiver interrupt (RX)
 *
 *	RX buffer states can be one of:
 *	- buffer has space	(CTS should be asserted)
 *	- buffer is full 	(CTS should be not_asserted)
 *	- buffer becomes full with this character (write char and assert CTS)
 *
 *	We use expressions like fd_usb.rx_buf_head instead of fd_ptrs[FD_USB]->rx_buf_head
 *	because it's more efficient and this is an interrupt and it's hard-wired anyway
 *
 *	Flow control is not implemented. Need to work RTS line.
 *	Flow control should cut off at high water mark, re-enable at low water mark
 *	High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
 *	Low water mark about 50% full
 */

ISR(USB_RX_ISR_vect)		//ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	// normal path
	if ((--f.rx_buf_head) == 0) { 				// wrap condition
		f.rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids the off-by-one error
	}
	if (f.rx_buf_head != f.rx_buf_tail) {		// write char unless buffer full
		f.rx_buf[f.rx_buf_head] = f.usart->DATA;// (= USARTC0.DATA;)
		return;
	}
	// buffer-full handling
	if ((++f.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		f.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}

/* 
 *	xio_usb_putc() - char writer for USB device 
 */

int xio_usb_putc(char c, FILE *stream)
{
	while(!(f.usart->STATUS & USART_DREIF_bm)); // spin until TX data register is available
	f.usart->DATA = c;							// write data register

	if (CRLF(f.flags) && (c == '\n')) {			// follow <cr> with <lf>
		return xio_usb_putc('\r', stream);	
	}
	return 0;
}

/*
 *  xio_usb_getc() - char reader for USB device
 *
 *  Get next character from RX buffer. Flags that affect behavior:
 *
 *  BLOCKING behaviors
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 if non-blocking
 *		- return character or sleep() if blocking
 *
 *  LINEMODE and SEMICOLONS behaviors
 *		- consider <cr> and <lf> to be EOL chars (not just <lf>)
 *		- also consider semicolons (';') to be EOL chars if SEMICOLONS enabled
 *		- convert any EOL char to <lf> to signal end-of-string (e.g. to fgets())
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc should expand newlines to <cr><lf>
 */

int xio_usb_getc(FILE *stream)
{
	// figure out heads and tails

	while (f.rx_buf_head == f.rx_buf_tail) {	// buffer empty
		if (!BLOCKING(f.flags)) {
			return (-1);
		}
		sleep_mode();							// sleep until next interrupt
	}
	if (--(f.rx_buf_tail) == 0) {				// decrement and wrap if needed
		f.rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one error
	}
	char c = f.rx_buf[f.rx_buf_tail];			// get character from buffer

	if (!LINEMODE(f.flags)) {			// processing is simple if not LINEMODE
		if (ECHO(f.flags)) {
			xio_usb_putc(c, stdout);
		}
		return (c);
	}
	// now do the LINEMODE stuff
	if (c == '\r') {					// perform newline substitutions
		c = '\n';
	} else if ((SEMICOLONS(f.flags)) && (c == ';')) {
		c = '\n';
	}
	if (ECHO(f.flags)) {
		xio_usb_putc(c, stdout);
	}
	return (c);
}

