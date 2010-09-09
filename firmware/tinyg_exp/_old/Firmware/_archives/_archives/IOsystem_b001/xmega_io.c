/*
  xmega_io.c - serial functions for xmega family
  Modeled after unix serial io
  .
  Copyright (c) 2010 Alden S. Hart, Jr.

  This IO subsystem looks like Unix IO but there are sigificant differences
  	- It's Kabuki Theater. There is no malloc, so everything is pre-allocated
	- You can read a file descriptor to the next delimiter. This is very convenient
	- io_control() is not ioctl(). They are totally different

---- Coding conventions ----

  Adopted the following xmega and C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	VARNAME_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	VARNAME_bp		- single bit position, e.g. 4 for the above example
	VARNAME_gm		- group bit mask, e.g. 0x0F
	VARNAME_gc		- group configuration, e.g. 0x0A is 2 bits in the above _gm
	VARNAME_ptr		- try to call pointers _ptrs. (but NOT array indexes)
	VARNAME_vect	- interrupt or other vectors

  These conventions are used for internal variables, but are relaxed for old 
  UNIX vars and defines that don't follow these conventions.

---- Note on circular buffer operation (I always get this confused) ----

  We are using circular buffers (char arrays) for character IO whose state is kept 
  by a head and a tail index (an array index, not a pointer). 
  
  Chars are written to the head and read from the tail. 

  The array is left with the head indexed to the character that was LAST WRITTEN
  - meaning that on write the character is written then the head is advanced.
  If the head advance exceeds the buffer size it resets to zero (a simple comparison 
  that does not require a modulus operation - unlike the Arduino wiring_serial code). 
  If the advance of the head would overwrite the tail the buffer is full and the 
  device should go into flow control if it implements this.

  The array is left with the tail indexed to the character the is NEXT TO-BE-READ 
  - meaning that on read the character is read first and then the tail is advanced.
  Unless, of course, the tail = the head, wherein there is no charcater to read and 
  the routine should either return with no data or block until there is data 
  (depending on blocking mode)

  Reading a character from a buffer that is in flow control should clear flow control

*/

//#include <math.h>
//#include <stdio.h>
//#include <stdarg.h>

#include "xmega_support.h"		// put this first as it has the F_CPU value

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "xmega_io.h"
#include "xmega_errno.h"


/* variables and functions with scope to this IO module */

int	errno=0;								// global error number

static struct fdUSART *fd_ptrs[FD_MAX];		// array of pointers to IO structs
static struct fdUSART fd_usb, fd_rs485;		// pre-allocate 2 USART structs

static const uint8_t bsel[] PROGMEM = 		// See .h file for source of these values
				{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

static const uint8_t bscale[] PROGMEM =  
				{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

// device-specific helper routines
static int8_t _open_usb(uint8_t dev, uint32_t control);
static int16_t _read_usb(struct fdUSART *f, char *buf, int count);
static int16_t _write_usb(struct fdUSART *f, char *buf, int count);
static int8_t _read_char_USART(struct fdUSART *f);
static int8_t _write_char_USART(struct fdUSART *f, char c);


/* io_init() - init serial and "file" io sub-system */

void io_init(void)
{ 
	fd_ptrs[0] = 0;							// null device
	fd_ptrs[1] = &fd_usb;					// this gets assigned to serial port C0
	fd_ptrs[2] = &fd_rs485;					// this gets assigned to serial port C1

	errno = 0;
	return;
}

/* interrupt routines (See module header note on circular buffers) */

/* USB_RX_ISR - USB receiver interrupt (RX)

   RX buffer states can be one of:
	- buffer has space	(CTS should be asserted)
	- buffer is full 	(CTS should be not_asserted)
	- buffer becomes full with this character (write char and assert CTS)

   We use expressions like fd_usb.rx_buf_head instead of fd_ptrs[FD_USB]->rx_buf_head
     because it's more efficient and this is an interrupt and it's hard-wired anyway

   Flow control is not implemented. Need to work RTS line.
   Flow control should cut off at high water mark, re-enable at low water mark
   High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
   Low water mark about 50% full

*/

ISR(USB_RX_ISR_vect)		//ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	uint8_t next_head;

	if ((next_head = fd_usb.rx_buf_head +1) >= RX_BUFSIZE) { // wrap
		next_head = 0;
	}
	if (next_head != fd_usb.rx_buf_tail) {
		fd_usb.rx_buf[fd_usb.rx_buf_head] = fd_usb.usart->DATA; // (= USARTC0.DATA;)
		fd_usb.rx_buf_head = next_head;
		return;
	}
	// ought to activate flow control here or before it gets to this level
}


/* 
ISR(USARTC1_RXC_vect)	// serial port C1 RX interrupt 
{
	unsigned char c = USARTC1.DATA;
	uint8_t i = (rx_buffer_head + 1) & RX_BUFFER_MASK;

	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}
*/

/* io_open() - open a device such as a serial port or program memory "file" handle 

	dev		Device specifier (takes the place of Unix path variable)
				Device number 0 - N to specify a device (see #defines DEV_XXXXX)

	control	Valid parameters for io_open() and io_control()
				IO_RDONLY		enable read only - attempt to write will cause error
				IO_WRONLY		enable write only - attempt to read will cause error
				IO_RDWR			enable read & write operations 
				IO_RDNONBLOCK	reads return immediately if char(s) not avail
				IO_WRNONBLOCK	writes do not wait for char(s) to be written
				IO_RDWRNONBLOCK	enable nonblocking for both read and write
				IO_ECHO			echo reads from device to the console (line level)
				IO_BAUD_XXXXX	baud rate for RX and TX (not independently settable)
				[ADDR]			address of program memory to read. See Address mode.

			Defaults are:
				IO_RDWR			enable both read and write
				IO_RDBLOCK	 	enable blocking on read
				IO_WRECHO		enable echo characters read from device to console
				IO_BAUD_DEFAULT	set baud rate to default (which is 115200)

			Address mode:
				Address mode is enabled if device expects an addr (eg.DEV_PROGMEM)
				In address mode device parameters must be set using io_control(). 
				Default settings are IO_RDONLY, IO_ECHO

	returns	File descriptor for device (fd)

			Error returns -1 and sets errno

				ENODEV	requested dev is not supported or illegal

				EINVAL	requesting IO_RDONLY and IO_WRONLY is an error. Use IO_RDWR

---- Enough of the theoretical stuff. Notes about this implementation ----

	Only recognizes the synthetic device DEV_USB. All else will fail.
	Ignores baud rate config. Always sets up 115200 baud
	Ignores all parameters and behaves as follows: 
	Implements blocking reads to delimiters (-1 behavior)
	Implements blocking writes to delimiters (-1 behavior)

*/

int8_t io_open(uint8_t dev, uint32_t control)
{
	switch (dev) {							// just a simple dispatcher
		case DEV_USB: return _open_usb(dev, control);
		default: errno = ENODEV; return (-1);
	}
	return (0);
}

/* _open_usb() open USB device */

static int8_t _open_usb(uint8_t dev, uint32_t control)
{
	struct fdUSART *f; 						// assign the fd struct...
	f = fd_ptrs[FD_USB];					// ...to its spot in fd pointer array

	f->fd = FD_USB;

	f->rx_buf_max = RX_BUFSIZE;
	f->rx_buf_head = 0;
	f->rx_buf_tail = 0;

	f->tx_buf_max = TX_BUFSIZE;
	f->tx_buf_head = 0;
	f->tx_buf_tail = 0;

	// flags
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
		f->flags &= ~IO_FLAG_ECHO_LINE_bm;	// clear line echo flag
	}
	if (control & IO_RDNONBLOCK) {
		f->flags &= ~IO_FLAG_RD_BLOCK_bm;	// clear read blocking flag
	}

	// device settings
	f->usart = &USB_USART;					// bind USB USART to struct
	f->port = &USB_PORT;					// bind corresponding port to struct

	f->port->DIRCLR = USB_RX_bm; 			// clr RX pin as input
	f->port->DIRSET = USB_TX_bm; 			// set TX pin as output
	f->port->OUTSET = USB_TX_bm;			// set TX HI as initial state

// DEBUG THIS ONCE ALL THE REST IS WORKING
//	f->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
//	f->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
//	f->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)

	if ((f->baud = (uint8_t)(control & IO_BAUD_gm)) == IO_BAUD_UNSPECIFIED) {
		f->baud = IO_BAUD_DEFAULT;
	}
	f->usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[f->baud]);
	f->usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[f->baud]);
	f->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	f->usart->CTRLA = USART_RXCINTLVL_MED_gc;		 // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use

	return (f->fd);
}


/* io_close() - close device 
	
	Close FD device. Stops all operations. Frees resources.
	In theory. In fact it's a lot like Hotel California.

	returns	0 if successful. 
	
			Error returns -1 and sets errno
				EBADF	fd isn't a valid open file descriptor.
				EINTR	The close() call was interrupted by a signal.
				EIO		An I/O error occurred
*/

int8_t io_close(uint8_t fd)
{
	return (0);
}


/* io_read() - read one or more characters from device 

	fd		valid device handle (file descriptor) returned from io_open()

	buf		address of buffer to read into
			this will be a RAM (string) address unless if DEV_EEPROM or DEV_PROGMEM selected

	count		Number of characters to read

				0		Returns zero and no other results
				1-N		Count mode: Read 1-N chars. return error if > than SSIZE_MAX

				-1		Line mode (TO_LINE): Read until line delimiter or NULL
						Return null-terminated string (NULL overwrites delimiter)

				-2		Null mode (TO_NULL): Read until NULL encountered

	returns	Number of characters read

			Error returns -1 and sets errno

				EBADF	fd is not a valid file descriptor or is not open for reading.

				EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
						no data was immediately available for reading.

				EFBIG	Block requested is too big (exceeds SSIZE_MAX)

				EIO 	General I/O error. (like Dbase2 "Disk is Full" error)

	NOTE: This routine and its helpers do not check for buffer overflow explicitly, 
		  SSIZE_MAX is used, but is not specific to a device nor read or write operations
		  Could tighten this up by having a value in the fd struct.
*/

int16_t io_read(uint8_t fd, char *buf, int count)
{
	struct fdUSART *f;

	if (fd == FD_USB) {
		f = &fd_usb;
		return (_read_usb(f, buf, count));
	} else {
		errno = EBADF;
		return (-1);
	}
}

/* _read_usb() - USB line reader (see io_read() for semantics) 

  NOTE: Need some protection here against buffer overflow in all modes.

*/

static int16_t _read_usb(struct fdUSART *f, char *buf, int count)
{
	char c;										// character temp
	int	i = 0;									// output buffer index (buf[i])
	uint8_t mode = count;						// preliminary setting

	if (count == 0) {							// special case of 0 count
		return (0);
	} else if (count > 0) {
		mode = 0;								// set mode to count mode
	} else if (count >= SSIZE_MAX) {
		errno = EFBIG;							// too big a request made
		return (-1);
	}

	while((c = _read_char_USART(f)) != -1) {	// while characters available
		buf[i] = c;
		i = i+1;								// optimizes better than a post-inc
		if (mode == 0) {						// count mode
			if (--count == 0) {					// count is complete
				buf[i] = 0;						// null terminate at count+1
				return (i);
			} else {
				continue;
			}
		} else if (c == 0) { 					// pick up both null cases
			return (i);
		} else if (mode == TO_LINE) {
			if ((c == '\r') || (c == '\n') || (c == ';')) { 
			buf[i] = 0;
			return (i);
		}
	}
	return (-1);								// Note: errno set in _read_char()
}

/* _read_char_USART() - lowest level char reader for USARTS 

	Execute blocking or non-blocking read depending on controls
	Return character or -1 if non-blocking
	Return character or sleep() if blocking
*/

static int8_t _read_char_USART(struct fdUSART *f)
{
	while (TRUE) {
		if (f->rx_buf_head == f->rx_buf_tail) {	// buffer empty
			if (!IF_BLOCKING) {
				errno = EAGAIN;
				return (-1);
			} else {
				sleep_mode();					// sleep until next interrupt
				continue;
			}
		}
	}
	char c = f->rx_buf[f->rx_buf_tail];			// get character from buffer
	if (++f->rx_buf_tail >= RX_BUFSIZE) {
		f->rx_buf_tail = 0;
	}
	return c;
} 

/* io_write() - write one or more characters to device

	fd			valid device handle (file descriptor) returned from io_open()

	buf			address of buffer to write from. This will be a RAM (string) 
				address unless if DEV_EEPROM or DEV_PROGMEM selected

	count		Number of characters to write

				0		Returns zero and no other results
				1-N		Write 1-N chars. Return error if greater than SSIZE_MAX
				-1		Write until next delimiter or NULL encountered (TO_NEXT)
				-2		Write until NULL encountered (TO_NULL)

	Returns:	Success: number of characters written

				Error:	 -1 and sets errno

				EBADF	fd is not a valid file descriptor or is not open for reading.

				EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
						no data was immediately available for reading.

				EFBIG	An attempt was made to write a file that exceeds the SSIZE_MAX

				ENOSPC	The device containing the file referred to by fd has no room for the data

				EIO 	General I/O error. (like Dbase2 "Disk is Full" error)
				
				EFAULT buf is outside your accessible address space.
				EINTR The call was interrupted by a signal before any data was read.
				EINVAL fd is attached to an object which is unsuitable for reading; or the file was opened with the O_DIRECT flag, and either the address specified in buf, the value specified in count, or the current file offset is not suitably aligned.
				EISDIR fd refers to a directory.
				EPIPE fd is connected to a pipe or socket whose reading end is closed. When this happens the writing process will also receive a SIGPIPE signal. (Thus, the write return value is seen only if the program catches, blocks or ignores this signal.)

*/
int16_t io_write(uint8_t fd, char *buf, int count)
{
//	struct fdUSART *f ;

	if (fd == FD_USB) {
		struct fdUSART *f = &fd_usb;
		return (_write_usb(f, buf, count));
	} else {
		errno = EBADF;
		return (char *)-1;
	}
}

/* _write_usb() - USB line writer */

static int16_t _write_usb(struct fdUSART *f, char *buf, int count)
{
	char c;					// character temp
	int i = 0; 				// input buffer index
	uint8_t mode = 0;		// 0 = count mode, 1 = line mode, 2 = null termination mode

	if (count == 0) {								// special case of 0 count
		return 0;
	} else if (count == TO_LINE) {					// line termination mode
		mode = 1; 
	} else if (count == TO_NULL) {					// NULL termination mode
		mode = 2;
	} else if (count >= SSIZE_MAX) {				// too big a request made
		errno = EFBIG;
		return -1;
	}

	while (c = buf[i]) {
		i += 1;										// putting it here optimizes better
		if (_write_char_USART(f,c) == -1) {
			return -1;								// pass errno through
		}
		if ((mode == 0) && (--count == 0)) {		// count complete
			return (i);
		} else if ((mode > 0) && (c == 0)) { 		// null terminated or line terminates w/null
			return (i);
		} else if ((mode == 1) && ((c == '\r') || (c == '\n') || (c == ';'))) { 
			return (i);
		}
	}
	return (i);
}

/* _write_char_USART() - lowest level char reader for USARTS 

	Execute blocking or non-blocking read depending on controls
	Return character or -1 if non-blocking
	Return character or sleep() if blocking
*/

static int8_t _write_char_USART(struct fdUSART *f, char c)
{
	while(!(USARTC0.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC0.DATA = c;							// write data register
	return (c);
}

/* io_control() - set device parameters  

	This isn't to ioctl. It's works differently. 
	Provides a rehash of the io_open() parameter settings only with an fd

	fd		valid device handle (file descriptor) returned from io_open()

	parms	Valid open parameters. See io_open() for details
	
	return	Success: File descriptor for device (fd) 
			Error:	 -1 and sets errno. See io_open()

*/

int8_t io_control(uint8_t fd, uint32_t parms)
{
	return (0);
}




