/*
  xmega_io_usart.c - Xmega IO drivers - USARTs
	lives under xmega_io.c

  Copyright (c) 2010 Alden S. Hart, Jr.

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

// local USART table lookups
static const struct USART_struct *usel[] PROGMEM = 	// USART base addresses
		{ &USARTC0,&USARTC1,&USARTD0,&USARTD1,&USARTE0,&USARTE1,&USARTF0,&USARTF1 };

static const struct PORT_struct *psel[] PROGMEM = 	// PORT base addresses
		{ &PORTC, &PORTC, &PORTD, &PORTD, &PORTE, &PORTE, &PORTF, &PORTF };

static const uint8_t bsel[] PROGMEM = 				// baud rates. See xmega_io.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

static const uint8_t bscale[] PROGMEM =  			// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };


/**********************************************************************************
 * NATIVE USART ROUTINES (GENERIC)
 **********************************************************************************/

/* USARTxx_RX_ISR - USART receiver interrupt (RX)
 *
 *	This is provided as a generic example ISR
 *	If the USART is taken over by a derived device the derived device should 
 *	  provide the ISR routine (that implements it's version of flow control, etc.)
 */

/*
ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	// normal path
	if ((--fd_usb.rx_buf_head) == 0) { 				// wrap condition
		fd_usb.rx_buf_head = USART_RX_BUFSIZE-1;	// -1 avoids the off-by-one error
	}
	if (fd_usb.rx_buf_head != fd_usb.rx_buf_tail) {	// write char unless buffer full
		fd_usb.rx_buf[fd_usb.rx_buf_head] = fd_usb.usart->DATA; // (= USARTC0.DATA;)
		return;
	}
	// buffer-full handling
	if ((++fd_usb.rx_buf_head) > USART_RX_BUFSIZE -1) { // reset the head
		fd_usb.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}
*/

/* 
 *	xio_open_USART() - initialize and set controls for USART
 */

int8_t xio_open_USART(uint8_t dev, uint32_t control)
{
	struct fdUSART *f;						// ptr to our fd structure
	uint8_t u;								// index into usart settings arrays
	uint8_t fd;								// local temp for fd
	
	if ((dev < DEV_USARTC0) || (dev > DEV_USARTF1)) {	// assertion code
		errno = EBADF;						// wrong device for this routine
		return (-1);
	}
	fd = xio_get_fd(dev); 					// lookup file descriptor
	f = xio_get_fd_ptr(fd);					// get fd struct pointer from ptr array

	// bind functions to structure
	f->close = (*xio_close_USART);
	f->control = (*xio_control_USART);
	f->read = (*xio_read_USART);
	f->write = (*xio_write_USART);
	f->getc = (*xio_getc_USART);
	f->putc = (*xio_putc_USART);

	// set variables
	f->fd = fd;
	f->rx_buf_head = 1;						// can't use location 0
	f->rx_buf_tail = 1;
	f->tx_buf_head = 1;
	f->tx_buf_tail = 1;

	// buffer overflow protection values
	f->read_size_max = READ_BUFFER_DEFAULT_SIZE-1;// default rd buffer size -1 for NUL
	f->write_size_max = NO_LIMIT;

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
		f->flags &= ~IO_FLAG_ECHO_bm;		// clear echo flag
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
	_delay_us(10);							// give USART a chance to settle before use
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
		f->flags |= IO_FLAG_ECHO_bm;			// set echo flag
	}
	if (control & IO_NOECHO) {
		f->flags &= ~IO_FLAG_ECHO_bm;			// clear echo flag
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
		f->read_size_max = arg;
		return (0);
	}
	if (control & IO_WR_SIZE_MAX) {
		f->write_size_max = arg;
		return (0);
	}
	return (0);
}

/* 
 *	xio_read_USART() - USART line reader (see io_read() for semantics) 
 *
 *	Note: LINE_MODE (-1) and STR_MODE (-2) are valid modes. PSTR_MODE (-3) is not.
 */

int16_t xio_read_USART(struct fdUSART *f, char *buf, int16_t size)
{
	char c;										// character temp
	int	i = 0;									// output buffer index (buf[i])
	int8_t mode;

	// get the size and mode variables right
	if (size == 0) { return (0); }							// special case of 0
	if (size > f->read_size_max) { errno = EFBIG; return (-1); } // too big a req made
	if (size < STR_MODE) { errno = EINVAL; return (-1); }	// invalid (negative) # 
	if (size > 0) {
		mode = SIZE_MODE; 
	} else {
		mode = (int8_t)size;
		size = f->read_size_max;				// sets max size or NO_LIMIT
	}

	// dispatch to smaller, tighter, more maintainable read loops depending on mode
	if (mode == SIZE_MODE) {
		while (TRUE) {
//			if ((buf[i++] = _read_char_USART(f)) == -1) {
			if ((buf[i++] = f->getc(f)) == -1) {// late bound char reader
				return (-1);					// passes errno through			
			}
			if (--size == 0) {					// test if size is complete
				return (i);
			}
		}
	} else if (mode == LINE_MODE) {
		while (TRUE) {
//			if ((c = _read_char_USART(f)) == -1) {
			if ((c = f->getc(f)) == -1) {		// late bound character reader
				return (-1);					// passes errno through			
			}
			buf[i++] = c;
			if (size != NO_LIMIT) {				// using 2 lines forces eval order
				if (--size == 0) {				// test if size is complete
					buf[i] = NUL;
					errno = EMSGSIZE;			// means was read until buffer full
					return (-1);
				}
			}
			if ((c == '\r') || (c == '\n') || (c == ';')) { 
				buf[i] = NUL;
				return (i);
			}
			if (c == NUL) {						// read a NUL
				return (i);
			}
		}
	} else if (mode == STR_MODE) {
		while (TRUE) {
//			if ((c = _read_char_USART(f)) == -1) {
			if ((c = f->getc(f)) == -1) {		// late bound character reader
				return (-1);					// passes errno through			
			}
			buf[i++] = c;
			if (size != NO_LIMIT) {
				if (--size == 0) {				// test if size is complete
					buf[i] = NUL;
					errno = EFBIG;
					return (-1);
				}
			}
			if (c == NUL) {						// read a NUL
				return (i);
			}
		}		
	}
	errno = EWTF;		// shouldn't ever get here.
	return (-1);
}

/* 
 *	xio_write_USART() - USB line writer 
 *
 *	NOTE: LINE_MODE (-1), STR_MODE (-2), and PSTR_MODE (-3) are all valid modes.
 */

int16_t xio_write_USART(struct fdUSART *f, const char *buf, int16_t size)
{
	char c;										// character temp
	int i = 0; 									// input buffer index
	int8_t mode;

	// get the size and mode variables right
	if (size == 0) { return (0); }							// special case of 0
	if (size > f->read_size_max) { errno = EFBIG; return (-1); } // too big req made
	if (size < PSTR_MODE) { errno = EINVAL; return (-1); }	// invalid (negative) # 
	if (size > 0) {
		mode = SIZE_MODE; 
	} else {
		mode = (int8_t)size;
		size = f->read_size_max;				// sets max size or NO_LIMIT
	}

	// dispatch to smaller, tighter, more maintainable write loops depending on mode
	if (mode == SIZE_MODE) {
		while (TRUE) {
			if (--size == 0) {					// size is complete. return
				return (i);
			}
//			if (_write_char_USART(f,buf[i++]) == -1) {	// write char to output
			if (f->putc(f,buf[i++]) == -1) {	// write char to output
				return (-1);					// passes errno through
			}
		}
	} else if (mode == LINE_MODE) {
		while (TRUE) {
			if (size != NO_LIMIT) {				// using 2 lines forces eval order
				if (--size == 0) {				// test if size is complete
					errno = EMSGSIZE;			// means a truncated write occurred
					return (-1);
				}
			}
			c = buf[i++];
			if (c == NUL) {
				return (i);						// don't write nul, just return
			}
//			if (_write_char_USART(f,c) == -1) {	// write char to output channel
			if (f->putc(f,c) == -1) {			// write char to output channel
				return (-1);					// passes errno through
			}
			if ((c == '\r') || (c == '\n') || (c == ';')) { 
				return (i);						// time to go
			}
		}
	} else if (mode == STR_MODE) {
		while (TRUE) {
			if (size != NO_LIMIT) {				// using 2 lines forces eval order
				if (--size == 0) {				// test if size is complete
					errno = EMSGSIZE;			// means a truncated write occurred
					return (-1);
				}
			}
			c = buf[i++];
			if (c == NUL) {
				return (i);						// don't write nul, just return
			}
//			if (_write_char_USART(f,c) == -1) {	// write char to output channel
			if (f->putc(f,c) == -1) {			// write char to output channel
				return (-1);					// passes errno through
			}
		}
	} else if (mode == PSTR_MODE) {
		while (TRUE) {
			if (size != NO_LIMIT) {				// using 2 lines forces eval order
				if (--size == 0) {				// test if size is complete
					errno = EMSGSIZE;			// means a truncated write occurred
					return (-1);
				}
			}
			c = pgm_read_byte(&buf[i++]);
			if (c == NUL) {
				return (i);						// don't write nul, just return
			}
//			if (_write_char_USART(f,c) == -1) {	// write char to output channel
			if (f->putc(f,c) == -1) {			// write char to output channel
				return (-1);					// passes errno through
			}
		}
	}
	errno = EWTF;		// shouldn't ever get here.
	return (-1);
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
	while (f->rx_buf_head == f->rx_buf_tail) {	// buffer empty
		if (!BLOCKING_ENABLED(f->flags)) {
			errno = EAGAIN;
			return (-1);
		}
		sleep_mode();							// sleep until next interrupt
	}
	if (--(f->rx_buf_tail) == 0) {				// decrement and wrap if needed
		f->rx_buf_tail = RX_BUFFER_DEFAULT_SIZE-1;// -1 avoids off-by-one error
	}
	char c = f->rx_buf[f->rx_buf_tail];			// get character from buffer
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

