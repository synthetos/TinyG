/*
  xmega_io_usb.c - Xmega IO drivers - USB port
	subordinate to xmega_io.c and xmega_io_usart.c


  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>			// needed for blocking character reads

#include "xmega_support.h"		// put this early as it has the F_CPU value
#include "xmega_io.h"
#include "xmega_io_usart.h"
#include "xmega_io_usb.h"
#include "xmega_errno.h"

extern struct fdUSART fd_usb;	// pick this up from the main xmega_io.c file

/**********************************************************************************
 * USB ROUTINES
 **********************************************************************************/

/* USB_RX_ISR - USB receiver interrupt (RX)
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

/* 
 *	xio_open_usb() - initialize and set controls for USB device 
 *
 *	This routine essentially subclasses the USARTC0 open to 
 *	extend it for use as a USB port. Mind, you it's all done at compile time.
 */

int8_t xio_open_usb(uint8_t dev, uint32_t control)
{
	struct fdUSART *f;						// USART struct used by USB port
	uint8_t fd;								// temp for file descriptor

	if ((fd = xio_open_USART(DEV_USARTC0, control)) == -1) {
		return -1;
	}
	f = xio_get_fd_ptr(fd);					// get fd struct pointer from ptr array

	// setup USB RTS/CTS 
	f->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
	f->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
	f->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
//	f->port->OUTCLR = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)

	// bind our own routines to the structure
	f->close = (*xio_close_usb);
	f->control = (*xio_control_usb);
	f->read = (*xio_read_usb);
	f->write = (*xio_write_usb);
	f->getc = (*xio_getc_usb);
	f->putc = (*xio_putc_usb);

	return (f->fd);
}

/*
 *	xio_close_usb() - close usb port (disable)
 */

int8_t xio_close_usb(struct fdUSART *f)
{
	return (0);
}

/*
 *	xio_control_usb() - set controls for USB device
 */

int8_t xio_control_usb(struct fdUSART *f, uint32_t control, int16_t arg)
{
	return (xio_control_USART(f, control, arg));
}

/* 
 *	xio_read_usb() - USB line reader (see io_read() for semantics) 
 *
 *	Note: LINE_MODE (-1) and STR_MODE (-2) are valid modes. PSTR_MODE (-3) is not.
 */

int16_t xio_read_usb(struct fdUSART *f, char *buf, int16_t size)
{
	char c;										// character temp
	int	i = 0;									// output buffer index (buf[i])
	int8_t mode;

	// get the size and mode variables right
	if (size == 0) { return (0); }							// special case of 0
	if (size > f->rx_size_max) { errno = EFBIG; return (-1); } // too big a request made
	if (size < STR_MODE) { errno = EINVAL; return (-1); }	// invalid (negative) # 
	if (size > 0) {
		mode = SIZE_MODE; 
	} else {
		mode = (int8_t)size;
		size = f->rx_size_max;					// sets max size or NO_LIMIT
	}

	// dispatch to smaller, tighter, more maintainable read loops depending on mode
	if (mode == SIZE_MODE) {
		while (TRUE) {
//			if ((buf[i++] = _read_char_USART(f)) == -1) {
			if ((buf[i++] = f->getc(f)) == -1) {	// late bound char reader
				return (-1);					// passes errno through			
			}
			if (--size == 0) {					// test if size is complete
				return (i);
			}
		}
	} else if (mode == LINE_MODE) {
		while (TRUE) {
//			if ((c = _read_char_USART(f)) == -1) {
			if ((c = f->getc(f)) == -1) {	// late bound character reader
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
			if ((c = f->getc(f)) == -1) {	// late bound character reader
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
 *	xio_write_usb() - USB line writer 
 *
 *	NOTE: LINE_MODE (-1), STR_MODE (-2), and PSTR_MODE (-3) are all valid modes.
 */

int16_t xio_write_usb(struct fdUSART *f, const char *buf, int16_t size)
{
	char c;										// character temp
	int i = 0; 									// input buffer index
	int8_t mode;

	// get the size and mode variables right
	if (size == 0) { return (0); }							// special case of 0
	if (size > f->rx_size_max) { errno = EFBIG; return (-1); } // too big a request made
	if (size < PSTR_MODE) { errno = EINVAL; return (-1); }	// invalid (negative) # 
	if (size > 0) {
		mode = SIZE_MODE; 
	} else {
		mode = (int8_t)size;
		size = f->rx_size_max;					// sets max size or NO_LIMIT
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
			if (f->putc(f,c) == -1) {		// write char to output channel
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
			if (f->putc(f,c) == -1) {		// write char to output channel
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
			if (f->putc(f,c) == -1) {		// write char to output channel
				return (-1);					// passes errno through
			}
		}
	}
	errno = EWTF;		// shouldn't ever get here.
	return (-1);
}

/*
 *  xio_getc_usb() - char reader for USB device
 *
 *	Execute blocking or non-blocking read depending on controls
 *	Return character or -1 if non-blocking
 *	Return character or sleep() if blocking
 */

char xio_getc_usb(struct fdUSART *f)
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
 *	xio_putc_usb() - char writer for USB device 
 */

char xio_putc_usb(struct fdUSART *f, const char c)
{
	while(!(f->usart->STATUS & USART_DREIF_bm)); // spin until TX data register is available
	f->usart->DATA = c;							 // write data register
	return c;
}
