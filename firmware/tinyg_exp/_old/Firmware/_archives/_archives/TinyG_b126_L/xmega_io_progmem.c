/*
  xmega_io_progmem.c - Xmega IO drivers - Program memory as read-only file
	subordinate to xmega_io.c

  Copyright (c) 2010 Alden S. Hart, Jr.

  Function:
	Treats strings in program memory as read only "files"

*/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "xmega_support.h"		// put this early as it has the F_CPU value
//#include <util/delay.h>
#include "xmega_io.h"
#include "xmega_io_progmem.h"
#include "xmega_errno.h"

/* 
 * Variables and functions with scope to this module only 
 */

/**********************************************************************************
 * NATIVE PROGMEM ROUTINES
 **********************************************************************************/

/* 
 *	xio_open_pgm() - initialize and set controls for program memory
 *
 *	dev		must be DEV_PROGMEM
 *	addr	address of program memory string
 *
 *		The normal control word is used for memory address. 
 *		Settings are: read only, echo enabled
 *	
 *  returns	fd success 
 *			-1 error. errno is:
 *
 *		EBADF	Bad file. Called this routine with the wrong device number 
 *		EROFS	Read-only file system. Attempted to open pgm for write
 *		ENOSYS	Function not supported. Attempted to open as a read-blocking device
 */

int8_t xio_open_pgm(uint8_t dev, uint32_t addr)
{
	struct fdUSART *f;						// ptr to our fd structure
	uint8_t fd;								// local temp for fd
	
	if (dev != DEV_PROGMEM) {				// assertion code
		errno = EBADF;						// wrong device for this routine
		return (-1);
	}
	fd = xio_get_fd(dev); 					// lookup file descriptor
	f = xio_get_fd_ptr(fd);					// get fd struct pointer from ptr array

	// bind functions to structure
	f->close = (*xio_close_pgm);
	f->control = (*xio_control_pgm);
	f->read = (*xio_read_pgm);
	f->write = (*xio_write_pgm);
	f->getc = (*xio_getc_pgm);
	f->putc = (*xio_putc_pgm);

	// set variables
	f->fd = fd;

	// buffer overflow protection values
	f->read_size_max = READ_BUFFER_DEFAULT_SIZE-1;// default rd buffer size -1 for nul
	f->flags = (IO_FLAG_RD_bm | IO_FLAG_ECHO_bm); // set flags to defaults
	f->
}

/* 
 *	xio_close_pgm() - close program memory file
 */

int8_t xio_close_pgm(struct fdUSART *f)
{
	return (0);
}

/*	
 *	xio_control_pgm() - set controls for progmem reads
 *
 *  Most 
 *
 *  IO_RD_SIZE_MAX	1-32767, NO_LIMIT
 *  IO_WR_SIZE_MAX	1-32767, NO_LIMIT
 */

int8_t xio_control_pgm(struct fdUSART *f, uint32_t control, int16_t arg)
{
	// group 1 commands (do not have argument)
		if (control & IO_RDBLOCK) {
		errno = ENOSYS;						// error: device is only non-blocking
		return (-1);
	}
	if (control & IO_ECHO) {
		f->flags |= IO_FLAG_ECHO_bm;			// set echo flag
	}
	if (control & IO_NOECHO) {
		f->flags &= ~IO_FLAG_ECHO_bm;			// clear echo flag
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
 *	xio_read_pgm() - program memory line reader (see io_read() for semantics) 
 *
 *	The following modes are supported:
 *
 *		SIZE_MODE	Read N characters into buf. Buf is not nul terminated
 *		LINE_MODE 	Read until line termination or nul. 
 *					Line terminator is copied to buf and nul terminated.
 *		STR_MODE	Read until nul. Returns nul terminated buf.
 */

int16_t xio_read_pgm(struct fdUSART *f, char *buf, int16_t size)
{
	char c;										// character temp
	int	i = 0;									// output buffer index (buf[i])
	int8_t mode;

	// get the size and mode variables right
	if (size == 0) { return (0); }							// special case of 0
	if (size > f->read_size_max) { errno = EFBIG; return (-1); } // too big req made
	if (size < STR_MODE) { errno = EINVAL; return (-1); }	// invalid (negative) # 
	if (size > 0) {
		mode = SIZE_MODE; 
	} else {
		mode = (int8_t)size;
		size = f->read_size_max;					// sets max size or NO_LIMIT
	}

	// dispatch to smaller, tighter, more maintainable read loops depending on mode
	if (mode == SIZE_MODE) {
		while (TRUE) {
			if ((buf[i++] = f->getc(f)) == -1) {// late bound char reader
				return (-1);					// passes errno through			
			}
			if (--size == 0) {					// test if size is complete
				return (i);
			}
		}
	} else if (mode == LINE_MODE) {
		while (TRUE) {
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
 *	xio_write_pgm() - program memory line writer 
 */

int16_t xio_write_pgm(struct fdUSART *f, const char *buf, int16_t size)
{
	errno = EROFS;		// error: read-only file system
	return (-1);		// always return error
}

/*
 *  xio_getc_pgm() - char reader for prpgram memory 
 */

char xio_getc_pgm(struct fdUSART *f)
{
	c = pgm_read_byte(&buf[i++]);

	while (f->rx_buf_head == f->rx_buf_tail) {	// buffer empty
		if (!BLOCKING_ENABLED(f->flags)) {
			errno = EAGAIN;
			return (-1);
		}
		sleep_mode();							// sleep until next interrupt
	}
	if (--(f->rx_buf_tail) == 0) {				// decrement and wrap if needed
		f->rx_buf_tail = RX_BUFFER_DEFAULT_SIZE-1; // -1 avoids off-by-one error
	}
	char c = f->rx_buf[f->rx_buf_tail];			// get character from buffer
	if (ECHO_ENABLED(f->flags)) {
		_echo_to_console(c);
	}
	return c;
}

/* 
 *	xio_putc_pgm() - char writer for program memory 
 */

char xio_putc_pgm(struct fdUSART *f, const char c)
{
	errno = EROFS;		// error: read-only file system
	return (-1);		// always return error
}

