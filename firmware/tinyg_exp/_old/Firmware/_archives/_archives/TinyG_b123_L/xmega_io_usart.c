/*
  xmega_io_usart.c - Xmega IO drivers - USARTs
	subordinate to xmega_io.c

  Copyright (c) 2010 Alden S. Hart, Jr.

TODO:
	- build out the generic _read_USART() and _write_USART() routines so that...
	- ...you can bind them to the fsUSART structure so that...
	- ...they can be overwritten by the USB module when it sub-classes...
	- ...but at least you will have gotten it right.	

*/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>			// needed for blocking character reads

#include "xmega_support.h"		// put this early as it has the F_CPU value
#include <util/delay.h>
#include "xmega_io.h"
#include "xmega_io_usart.h"
#include "xmega_errno.h"

/* 
 * Variables and functions with scope to this module only 
 */

// USART lookups
static const struct USART_struct *usel[] PROGMEM = 		// USART base addresses
		{ &USARTC0,&USARTC1,&USARTD0,&USARTD1,&USARTE0,&USARTE1,&USARTF0,&USARTF1 };

static const struct PORT_struct *psel[] PROGMEM = 		// PORT base addresses
		{ &PORTC, &PORTC, &PORTD, &PORTD, &PORTE, &PORTE, &PORTF, &PORTF };

static const uint8_t bsel[] PROGMEM = 		// baud rates. See xmega_io.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

static const uint8_t bscale[] PROGMEM =  	// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };


/**********************************************************************************
 * NATIVE USART ROUTINES (GENERIC)
 **********************************************************************************/

/* 
 *	xio_open_USART() - initialize and set controls for USART
 */

int8_t xio_open_USART(uint8_t dev, uint32_t control)
{
	struct fdUSART *f;						// ptr to our fd structure
	uint8_t u;								// index into usart settings arrays
	uint8_t fd;								// local temp for fd
	
	fd = xio_get_fd(dev); 					// lookup file descriptor
	f = xio_get_fd_ptr(fd);					// get fd struct pointer from ptr array

	f->fd = fd;
	f->rx_buf_head = 1;						// can't use location 0
	f->rx_buf_tail = 1;
	f->tx_buf_head = 1;
	f->tx_buf_tail = 1;

	// buffer overflow protection values
	f->rx_size_max = READ_BUFFER_SIZE-1;	// default read buffer size - the NUL
	f->tx_size_max = NO_LIMIT;

	// device flags
	if ((control & (IO_RDONLY | IO_WRONLY)) == (IO_RDONLY | IO_WRONLY)) {
		errno = EINVAL;						// can't have both RDONLY & WRONLY set
		return (-1);
	}
	f->flags = IO_FLAG_DEFAULT_gm;			// set flags to defaults
	if (control & IO_RDONLY) {
		f->flags &= ~IO_FLAG_WR_bm;			// clear write flag
	} else if (control && IO_RDONLY) {
		f->flags &= ~IO_FLAG_RD_bm;			// clear read flag
	}
	if (control & IO_NOECHO) {
		f->flags &= ~IO_FLAG_ECHO_CHAR_bm;	// clear line echo flag
	}
	if (control & IO_RDNONBLOCK) {
		f->flags &= ~IO_FLAG_RD_BLOCK_bm;	// clear read blocking flag
	}

	// device assignment
	u = dev - DEV_USARTC0;					// zero justify the USART #s for lookups
	f->usart = (struct USART_struct *)pgm_read_word(&usel[u]);	// bind USART to fd
	f->port = (struct PORT_struct *)pgm_read_word(&psel[u]);	// bind PORT to fd

	// baud rate and USART setup
	if ((f->baud = (uint8_t)(control & IO_BAUD_gm)) == IO_BAUD_UNSPECIFIED) {
		f->baud = IO_BAUD_DEFAULT;
	}
	f->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[f->baud]);
	f->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[f->baud]);
	f->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	f->usart->CTRLA = USART_RXCINTLVL_MED_gc;		 // receive interrupt medium level

	if (u & 1) {					// test if this is an odd USART (e.g. USARTC1)
		f->port->DIRCLR = USART_RX_odd_bm; 	// clr RX pin as input
		f->port->DIRSET = USART_TX_odd_bm; 	// set TX pin as output
		f->port->OUTSET = USART_TX_odd_bm;	// set TX HI as initial state
	} else {
		f->port->DIRCLR = USART_RX_even_bm;	// as above
		f->port->DIRSET = USART_TX_even_bm;
		f->port->OUTSET = USART_TX_even_bm;
	}

	// bind functions to structure
	f->close = (*xio_close_USART);
	f->control = (*xio_control_USART);
//	f->read = (*xio_read_USART);
//	f->write = (*xio_write_USART);
	f->getc = (*xio_getc_USART);
	f->putc = (*xio_putc_USART);

	_delay_us(10);							// give UART a chance to settle before use
	return (f->fd);
}

/* 
 *	xio_close_USART() - close USART port (disable) 
 */

int8_t xio_close_USART(struct fdUSART *f)
{
	return (0);
}

/*	
 *	xio_control_USART() - set controls for USART device 
 *
 *	Control		   Data		Notes
 *	
 *	IO_BAUD_xxxxx	0		One of the supported baud rate enums
 *	IO_ECHO			0		Enable echo
 *	IO_NOECHO		0		Disable echo
 *	IO_RDBLOCK		0		Enable blocking reads
 *	IO_RDNONBLOCK	0		Disable blocking reads
 *	IO_WRBLOCK		0		Enable blocking writes (not implemented)
 *	IO_WRNONBLOCK	0		Disable blocking writes (not implemented)
 *
 *  IO_RD_SIZE_MAX	1-32767, NO_LIMIT
 *  IO_WR_SIZE_MAX	1-32767, NO_LIMIT
 */

int8_t xio_control_USART(struct fdUSART *f, uint32_t control, int16_t arg)
{
	// group 1 commands (do not have argument)
	if ((control & IO_BAUD_gm) != IO_BAUD_UNSPECIFIED) {
		f->baud = (uint8_t)(control & IO_BAUD_gm);
		f->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[f->baud]);
		f->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[f->baud]);
//		_delay_us(100);							// let it settle before use
	}
	if (control & IO_ECHO) {
		f->flags |= IO_FLAG_ECHO_CHAR_bm;		// set echo flag
	}
	if (control & IO_NOECHO) {
		f->flags &= ~IO_FLAG_ECHO_CHAR_bm;		// clear echo flag
	}
	if (control & IO_RDBLOCK) {
		f->flags |= IO_FLAG_RD_BLOCK_bm;		// set read blocking flag
	}
	if (control & IO_RDNONBLOCK) {
		f->flags &= ~IO_FLAG_RD_BLOCK_bm;		// clear read blocking flag
	}
	if (control & IO_WRBLOCK) {
		f->flags |= IO_FLAG_WR_BLOCK_bm;		// set write blocking flag
	}
	if (control & IO_WRNONBLOCK) {
		f->flags &= ~IO_FLAG_WR_BLOCK_bm;		// clear write blocking flag
	}

	// group 2 commands (have argument)
	if (control & IO_RD_SIZE_MAX) {
		f->rx_size_max = arg;
		return (0);
	}
	if (control & IO_WR_SIZE_MAX) {
		f->tx_size_max = arg;
		return (0);
	}
	return (0);
}

/*
 *  xio_getc_USART() - char reader for USARTS 
 *
 *	Execute blocking or non-blocking read depending on controls
 *	Return character or -1 if non-blocking
 *	Return character or sleep() if blocking
 */

char xio_getc_USART(struct fdUSART *f)
{
	while (f->rx_buf_head == f->rx_buf_tail) {		// buffer empty
		if (!BLOCKING_ENABLED(f->flags)) {
			errno = EAGAIN;
			return (-1);
		}
		sleep_mode();								// sleep until next interrupt
	}
	if (--(f->rx_buf_tail) == 0) {					// decrement and wrap if needed
		f->rx_buf_tail = USART_RX_BUFSIZE-1;		// -1 avoids off-by-one error
	}
	char c = f->rx_buf[f->rx_buf_tail];				// get character from buffer
	if (ECHO_ENABLED(f->flags)) {
		_echo_to_console(c);
	}
	return c;
}

/* 
 *	xio_putc_USART() - char writer for USARTS 
 */

char xio_putc_USART(struct fdUSART *f, const char c)
{
	while(!(f->usart->STATUS & USART_DREIF_bm)); // spin until TX data register is available
	f->usart->DATA = c;							 // write data register
	return c;
}

