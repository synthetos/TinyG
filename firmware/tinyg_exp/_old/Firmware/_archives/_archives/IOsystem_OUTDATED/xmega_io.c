/*
  xmega_io.c - IO functions for xmega family
  Modeled after UNIX io: open(), close(), read(), write(), ioctl()

  Copyright (c) 2010 Alden S. Hart, Jr.

  IO subsystem features
  	- Looks and works like Unix IO .
	- Syntax, semantics and operation of UNIX IO largely copied
		- open() returns integer (unint8_t) file descriptors
		- read() and write() obey fd, buffer and size conventions (in SIZE mode)
	- Macros are defined to expose routines using UNIX names (open(), close()...)
		as opposed to the module names (xio_open(), xio_close()...)
	- Framework to organize IO drivers for the 37 (by my count) Xmega IO devices
	- Extensible to support synthetic devices such as USB ports, RS-485, etc.
	- Can be used to provide the putc / getc needed by AVR-GCC stdio

  Notable differences from UNIX IO
  	- It's Kabuki Theater: everything is pre-allocated (no malloc calls)
	- read() and write() extended to handle lines and strings (in addition to SIZE)
		- LINE_MODE: read/write to defined line delimiter (e.g. \r, \n, ';')
		- NUL_MODE:  read/write to end-of-string (aka: zero, ASCII nul, 0, \0)
	- xio_control() is NOT ioctl(). Very different interfaces


---- Coding conventions ----

  Adopted the following xmega and C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	varname_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	varname_bp		- single bit position, e.g. 4 for the above example
	varname_gm		- group bit mask, e.g. 0x0F
	varname_gc		- group configuration, e.g. 0x0A is 2 bits in the above _gm
	varname_ptr		- indicates a pointer. (but NOT array indexes)
	varname_idx		- indicates an array index (if not simply called i or j)
	varname_vect	- interrupt or other vectors

  These conventions are used for internal variables but may be relaxed for old 
  UNIX vars and DEFINES that don't follow these conventions.

---- Notes on the circular buffers ----

  An attempt has beeen made to make the circular buffers used by low-level 
  character read / write as efficient as possible. This opens up higher-speed 
  IO between 100K and 1Mbaud and better supports high-speed parallel operations.

  The circular buffers are unsigned char arrays that count down from the top 
  element and wrap back to the top when index zero is reached. This allows 
  pre-decrement operations, zero tests, and eliminates modulus, mask, substraction 
  and other less efficient array bounds checking. Buffer indexes are all 
  unint_fast8_t which limits these buffers to 254 usable locations. (one is lost 
  to head/tail collision detection and one is lost to the zero position) All this 
  enables the compiler to do better optimization.

  Chars are written to the *head* and read from the *tail*. 

  The head is left "pointing to" the character that was previously written - 
  meaning that on write the head is pre-decremented (and wrapped, if necessary), 
  then the new character is written.

  The tail is left "pointing to" the character that was previouly read - 
  meaning that on read the tail is pre-decremented (and wrapped, if necessary),
  then the new character is read.

  The head is only allowed to equal the tail if there are no characters to read.

  On read: If the head = the tail there is nothing to read, so it exits or blocks.

  On write: If the head pre-increment causes the head to equal the tail the buffer
  is full. The head is reset to its previous value and the device should go into 
  flow control (and the byte in the device is not read). Reading a character from 
  a buffer that is in flow control should clear flow control

  (Note: More sophisticated flow control would detect the full condition earlier, 
   say at a high water mark of 95% full, and may go out of flow control at some low
   water mark like 33% full).

---- Other Stuff ----

  In this code:
  	"NULL" refers to a null (uninitialized) pointer 
   	"NUL" refers to the ASCII string termination character - zero
			See http://home.netcom.com/~tjensen/ptr/  (chapter 3)

---- To Do ----

  Make RX and TX buffers count backwards to zero for efficiency
  Put buffer overrun code in to protect buf in LINE and NUL modes
  Flow control for USB low-level read and write

*/

#include <stdio.h>
#include <stdarg.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "xmega_support.h"		// put this early as it has the F_CPU value
#include <util/delay.h>
#include "xmega_io.h"
#include "xmega_errno.h"


/* Variables and functions with scope to this module only 
	If you want to move some functions out into other files or extend
	with new devices you need to look at this section carefully.
*/

int	errno=0;								// global error number
static struct fdUSART *fd_ptrs[FD_MAX];		// array of pointers to IO structs
static struct fdUSART *fd_ptrs[FD_MAX];		// array of pointers to IO structs
static struct fdUSART fd_usb, fd_rs485;		// pre-allocate 2 USART structs
// put other types of pre-allocated structs here. See xio_init().


// File descriptor assignments. Device numbers look up FDs via this table
static const uint8_t fdes[] PROGMEM = 		// device IDs assigned to file descriptor
				{ 0,						// NULL device (position 0) assigned to 0 
				  0, 0, 0, 0, 0, 0, 0, 0, 	// Ports A, B, C, D, E, F, G, H   (no I)
				  0, 0, 0, 0, 0, 0, 0, 0,	// Ports J, K, L, M, N, P, Q, R   (no O)
				  1, 2, 0, 0, 0, 0, 0, 0, 	// USARTS C0, C1, D0, D1, E0, E1, F0, F1
				  0, 0, 0, 0,				// SPI interfaces C, D, E, F
				  0, 0,						// Two Wire interfaces C, E
				  0, 	 					// IR communications module
				  0, 						// AES accelerator
				  0, 0,						// ADCA, ADCB
				  0, 0,						// DACA, DACB
				  0, 0, 0, 0, 0, 			// SRAM, EEPROM, PGM, TABLE, BOOT
				  0, 						// CONSOLE, 
				  1, 2, 0, 0,				// USB, RS485, ENCODERS, BRIDGE
				  0, 						// a couple of not-yet-defined devices...
				  0 };						// ...for illustration purposes

// Note: USARTC0 and USB share the same file descriptor (as do USARTC1 and RS485).
// This is because USB first configures the USART, then takes it over.
// Calls to FD1 call the USB routines, not the generic USART routines.


// USART lookups
static const struct USART_struct *usel[] PROGMEM = 		// USART base addresses
		{ &USARTC0,&USARTC1,&USARTD0,&USARTD1,&USARTE0,&USARTE1,&USARTF0,&USARTF1 };

static const struct PORT_struct *psel[] PROGMEM = 		// PORT base addresses
		{ &PORTC, &PORTC, &PORTD, &PORTD, &PORTE, &PORTE, &PORTF, &PORTF };

static const uint8_t bsel[] PROGMEM = 		// baud rates. See xmega_io.h
		{ 0, 207, 103, 51, 34, 33, 31, 27, 19, 1, 1 };

static const uint8_t bscale[] PROGMEM =  	// more baud rate data
		{ 0, 0, 0, 0, 0, (-1<<4), (-2<<4), (-3<<4), (-4<<4), (1<<4), 1 };

/* Helper routines and device-specific routines */
void _echo_to_console(char c);

// USART device handlers
static int8_t _open_USART(uint8_t dev, uint32_t control);
int8_t _read_char_USART(struct fdUSART *fd_ptr);
int8_t _write_char_USART(struct fdUSART *fd_ptr, char c);

// USB device handlers
static int8_t _open_usb(uint8_t dev, uint32_t control);
int16_t _read_usb(struct fdUSART *fd_ptr, char *buf, int size);
static int16_t _write_usb(struct fdUSART *fd_ptr, const char *buf, int size);


/**************************************************************
 *
 *          XIO_MAIN ROUTINES (NOT DEVICE SPECIFIC)
 *
 * These are the dispatchers to the device specific routines
 *
 **************************************************************/

/* xio_init() - init serial and "file" io sub-system 

	All the structs are pre-assigned to the FD array. 
	These must line up with the FD values in the fdes table
*/

void xio_init(void)
{ 
	fd_ptrs[0] = NULL;						// /dev/null
	fd_ptrs[1] = &fd_usb;					// this gets assigned to serial port C0
	fd_ptrs[2] = &fd_rs485;					// this gets assigned to serial port C1
	errno = 0;
	return;
}

/* xio_open() - open a device such as a serial port or program memory "file" handle 

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

	Only recognizes the synthetic devices DEV_USB and DEV_RS485. All else will fail.
	Uses a very inefficient case statement because I don't care about optimization...
	...I care more about code clarity.
*/

int8_t xio_open(uint8_t dev, uint32_t control)
{
	switch (dev) {
		case DEV_USARTC0: errno = ENODEV; return (-1); 	// can't open C0 - use USB
		case DEV_USARTC1: errno = ENODEV; return (-1); 	// can't open C1 - use RS485
		case DEV_USB: return _open_usb(dev, control);
//		case DEV_RS485: return _open_rs485(dev, control);

		default: errno = ENODEV; return (-1);
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}

/* xio_control() - set device parameters  

	This isn't to ioctl. It's works differently. 
	Provides a rehash of the io_open() parameter settings only with an fd

	fd		valid device handle (file descriptor) returned from io_open()

	parms	Valid open parameters. See io_open() for details
	
	return	Success: File descriptor for device (fd) 
			Error:	 -1 and sets errno. See io_open()

*/

int8_t xio_control(uint8_t fd, uint32_t control)
{
	struct fdUSART *fd_ptr;

	if (fd == FD_USB) {
		fd_ptr = &fd_usb;
//		return (_control_usb(fd_ptr, control));
		return 0;
	} else {
		errno = EBADF;
		return (-1);
	}
}


/* xio_close() - close device 
	
	Close FD device. Stops all operations. Frees resources.
	In theory. In fact it's a lot like Hotel California.

	returns	0 if successful. 
	
			Error returns -1 and sets errno
				EBADF	fd isn't a valid open file descriptor.
				EINTR	The close() call was interrupted by a signal.
				EIO		An I/O error occurred
*/

int8_t xio_close(uint8_t fd)
{
	return (0);
}


/* xio_read() - read one or more characters from device 

	fd		Valid device handle (file descriptor) returned from io_open()
	buf		Address of buffer to read into - this will be a RAM (string)
	size	Number of characters to read
			  0		Returns zero and no other results
			  1-N	Size mode: Read 1-N chars. Returns error if > than SSIZE_MAX
			 -1		Line mode: Read until line delimiter or NUL
					Returns null-terminated string (NUL overwrites delimiter)
			 -2		Nul mode: Read until NUL. Nul is written to buffer.

	returns	  1-N	Number of characters read
			 -1 	Error returns -1 and sets errno

			 EBADF	fd is not a valid file descriptor or is not open for reading.
			 EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
					no data was immediately available for reading.
			 EFBIG	Block requested is too big (exceeds SSIZE_MAX)
			 EINVAL	Invalid argument - some negative number other than -1 or -2

	NOTE: This routine and its helpers do not check for buffer overflow explicitly, 
		  SSIZE_MAX is used, but is not specific to a device nor read or write 
		  operations.
*/

int16_t xio_read(uint8_t fd, char *buf, int size)
{
	struct fdUSART *fd_ptr;

	if (fd == FD_USB) {
		fd_ptr = &fd_usb;
		return (_read_usb(fd_ptr, buf, size));
	} else {
		errno = EBADF;
		return (-1);
	}
}

/* xio_write() - write one or more characters to device

	fd			valid device handle (file descriptor) returned from io_open()

	buf			address of buffer to write from. This will be a RAM (string) 
				address unless if DEV_EEPROM or DEV_PROGMEM selected

	size		Number of characters to write
				0		Returns zero and no other results
				1-N		Write 1-N chars. Return error if greater than SSIZE_MAX
				-1		Write until next delimiter or NULL encountered (TO_NEXT)
				-2		Write until NULL encountered (TO_NULL)

	Returns:	Success: number of characters written

				Error:	 -1 and sets errno
				EBADF	fd is not a valid file descriptor or is not open for reading.
				EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
						no data was immediately available for reading.
			    EINVAL	Invalid argument - some negative number other than -1 or -2
*/

int16_t xio_write(uint8_t fd, const char *buf, int size)
{
//	struct fdUSART *f ;

	if (fd == FD_USB) {
		struct fdUSART *f = &fd_usb;
		return (_write_usb(f, buf, size));
	} else {
		errno = EBADF;
		return -1;
	}
}

/**********************************************************************************
 * DEVICE SPECIFIC ROUTINES - NATIVE DEVICES
 **********************************************************************************/


/**********************************************************************************/

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
	// normal write path
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

 
ISR(USARTC1_RXC_vect)	// serial port C1 RX interrupt 
{
	// normal write path
	if ((--fd_rs485.rx_buf_head) == 0) { 			// wrap condition
		fd_rs485.rx_buf_head = USART_RX_BUFSIZE-1;	// -1 avoids the off-by-one error
	}
	if (fd_rs485.rx_buf_head != fd_rs485.rx_buf_tail) {	// write char unless full
		fd_rs485.rx_buf[fd_rs485.rx_buf_head] = fd_rs485.usart->DATA;
		return;
	}
	// buffer-full handling
	if ((++fd_rs485.rx_buf_head) > USART_RX_BUFSIZE -1) { // reset the head
		fd_rs485.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}


/* _open_USART() - initialize and set controls for USART */

static int8_t _open_USART(uint8_t dev, uint32_t control)
{
	struct fdUSART *f;						// ptr to our fd structure
	uint8_t u;								// index into usart settings arrays
	uint8_t fd;								// local temp for fd
	
	fd = (uint8_t)pgm_read_byte(&fdes[dev]); // lookup file descriptor
	f = fd_ptrs[fd];						// get fd struct pointer from ptr array
	f->fd = fd;
	f->rx_buf_head = 0;
	f->rx_buf_tail = 0;
	f->tx_buf_head = 0;
	f->tx_buf_tail = 0;

//	f->read = &(_read_USART);
//	f->write = &(_write_USART);

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
		f->flags &= ~IO_FLAG_ECHO_CHAR_bm;	// clear line echo flag
	}
	if (control & IO_RDNONBLOCK) {
		f->flags &= ~IO_FLAG_RD_BLOCK_bm;	// clear read blocking flag
	}

	// device settings
	u = dev - DEV_USARTC0;					// zero justify the USART #s for lookups
	f->usart = (struct USART_struct *)pgm_read_word(&usel[u]);	// bind USART to fd
	f->port = (struct PORT_struct *)pgm_read_word(&psel[u]);	// bind PORT to fd

	if (u & 1) {					// test if this is an odd USART (e.g. USARTC1)
		f->port->DIRCLR = USART_RX_odd_bm; 	// clr RX pin as input
		f->port->DIRSET = USART_TX_odd_bm; 	// set TX pin as output
		f->port->OUTSET = USART_TX_odd_bm;	// set TX HI as initial state
	} else {
		f->port->DIRCLR = USART_RX_even_bm;	// as above
		f->port->DIRSET = USART_TX_even_bm;
		f->port->OUTSET = USART_TX_even_bm;
	}

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


/* _open_usb() - initialize and set controls for USB device 

  This routine essentially subclasses the USARTC0 open to 
  extend it for use as a USB port. Mind, you it's all done 
  at compile time.
*/

static int8_t _open_usb(uint8_t dev, uint32_t control)
{
	struct fdUSART *f;						// USART struct used by USB port
	uint8_t fd;								// temp for file descriptor

	if ((fd = _open_USART(DEV_USARTC0, control)) == -1) {
		return -1;
	}
	f = fd_ptrs[fd];						// get struct pointer from fd pointer array

	// setup USB RTS/CTS 
	f->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
	f->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
	f->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
//	f->port->OUTCLR = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)

	return (f->fd);
}


/* _read_usb() - USB line reader (see io_read() for semantics) 

  NOTE: Need some protection here against buffer overflow in all modes.

*/

int16_t _read_usb(struct fdUSART *fd_ptr, char *buf, int size)
{
	char c;										// character temp
	int	i = 0;									// output buffer index (buf[i])
	int8_t mode = (int8_t)size;					// preliminary set to LINE or NUL

	if (size == 0) {							// special case of 0 count
		return (0);
	} else if (size > 0) {
		mode = SIZE_MODE;
	} else if (size >= SSIZE_MAX) {
		errno = EFBIG;							// too big a request made
		return (-1);
	} else {
		errno = EINVAL;							// invalid (negative) number 
		return (-1);
	}

	while((c = _read_char_USART(fd_ptr)) != -1) { // while characters available
		buf[i] = c;
		i = i+1;								// optimizes better than a post-inc
		if (mode == SIZE_MODE) {
			if (--size == 0) {					// count is complete
				buf[i] = NUL;					// \0 terminate at count+1
				return (i);
			} else {
				continue;
			}
		} else if (c == NUL) { 					// do both LINE and NUL \0 cases
			return (i);
		} else if (mode == LINE_MODE) {
			if ((c == '\r') || (c == '\n') || (c == ';')) { 
				buf[i] = NUL;
				return (i);
			}
		}
	}
	return (-1); // Note: errno set by _read_char(), typ EAGAIN if non-blocking read
}

/* _read_char_USART() - lowest level char reader for USARTS 

	Execute blocking or non-blocking read depending on controls
	Return character or -1 if non-blocking
	Return character or sleep() if blocking
*/

int8_t _read_char_USART(struct fdUSART *f)
{
	while (f->rx_buf_head == f->rx_buf_tail) {		// buffer empty
		if (!BLOCKING_ENABLED(f->flags)) {
			errno = EAGAIN;
			return (-1);
		}
		sleep_mode();								// sleep until next interrupt
	}
	if (--(f->rx_buf_tail) == 0) {					// decrement and wrap if needed
		fd_usb.rx_buf_tail = USART_RX_BUFSIZE-1;	// -1 avoids off-by-one error
	}
	char c = f->rx_buf[f->rx_buf_tail];				// get character from buffer
	if (ECHO_ENABLED(f->flags)) {
		_echo_to_console(c);
	}
	return c;
}


/* _echo_to_console() */

void _echo_to_console(char c)
{
	_write_char_USART(&fd_usb, c);
}



/* _write_usb() - USB line writer */

int16_t _write_usb(struct fdUSART *fd_ptr, const char *buf, int size)
{
	char c;										// character temp
	int i = 0; 									// input buffer index
	int8_t mode = (int8_t)size;					// preliminary set to LINE or NUL

	if (size == 0) {							// special case of 0 count
		return (0);
	} else if (size > 0) {
		mode = SIZE_MODE;
	} else if (size >= SSIZE_MAX) {
		errno = EFBIG;							// too big a request made
		return (-1);
	} else {
		errno = EINVAL;							// invalid (negative) number 
		return (-1);
	}

	while (TRUE) {
		c = buf[i];
		i += 1;									// putting inc here optimizes better
		if ((mode != SIZE_MODE) && (c == 0)) {	// trap EOS in LINE & NUL MODES
			return (i);							// don't write it, just return
		}
		if (_write_char_USART(fd_ptr, c) == -1) {
			return -1;							// passes errno through
		}
		if (mode == NUL_MODE) {					// nothing else to do in NULL mode
			continue;
		}
		if (mode == SIZE_MODE) {
			if (--size == 0) {					// count complete
				return (i);
			}
		} else if ((c == '\r') || (c == '\n') || (c == ';')) { // LINE mode
			return (i);
		}
	}
	return (i);
}

/* _write_char_USART() - lowest level char reader for USARTS */

int8_t _write_char_USART(struct fdUSART *fd_ptr, char c)
{
	struct fdUSART *f = fd_ptr;

	while(!(f->usart->STATUS & USART_DREIF_bm)); // spin until TX data register is available
	f->usart->DATA = c;							 // write data register
	return c;
}



