/*
  xmega_io.c - IO functions for xmega family
  Modeled after UNIX io: open(), close(), read(), write(), ioctl()

  Copyright (c) 2010 Alden S. Hart, Jr.

  To Do
	- Flow control for USB low-level read and write
	- Change the _routines (that were helper routines) to xio_ routines
	- Change the FS pointer table to works with void *s so you can do proper polymorphism
	- Add some real flow control to the USARTs
	- Add xio_putc() xio_getc() to every level and cross wire the USB and USARTS 
		accordingly. Include them in the ailases and in the function popinter bindings.

*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>				// required for wiring_serial compatibility (printFloat())

#include "xmega_support.h"		// put this early as it has the F_CPU value
#include "xmega_io.h"
#include "xmega_errno.h"
// Mind the children! List them all here so dispatches can occur
#include "xmega_io_usart.h"	
#include "xmega_io_usb.h"

/*
 * Core FD table - reflects the state of the entire device system
 */
struct fdUSART *fd_ptrs[FD_MAX];			// array of pointers to IO structs
struct fdUSART fd_usb, fd_rs485;			// pre-allocate 2 USART structs
// put other types of pre-allocated structs here. See xio_init().

/*
 * File descriptor assignments. 
 * Use device number as an index to look up the FD via this table
 */
const uint8_t fdes[] PROGMEM = 	// NATIVE DEVICES
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
				  				// DERIVED DEVICES
				  1, 						// CONSOLE DEVICE - set to actual device
				  1, 2,						// USB, RS485, 
				  0, 0, 					// ENCODERS, BRIDGE
				  0, 						// a couple of not-yet-defined devices...
				  0 };						// ...for illustration purposes

// Note: USARTC0 and USB share the same file descriptor (as do USARTC1 and RS485).
// This is because USB first configures the USART, then takes it over.
// Calls to FD1 call the USB routines, not the generic USART routines.


/******************************************************************************
 *
 * XIO_MAIN ROUTINES (NOT DEVICE SPECIFIC)
 *
 * These are the dispatchers to the device specific routines in the other files
 *
 ******************************************************************************/

/* 
 *	xio_init() - init serial and "file" io sub-system 
 *
 *	All the structs are pre-assigned to the FD array. 
 *	These must line up with the FD values in the fdes table
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

	control	Valid parameters for io_open()
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
*/

int8_t xio_open(uint8_t dev, uint32_t control)
{
	switch (dev) {
		case DEV_USARTC0: errno = ENODEV; return (-1); 	// can't open C0 - use USB
		case DEV_USARTC1: errno = ENODEV; return (-1); 	// can't open C1 - use RS485
		case DEV_USB: return xio_open_usb(dev, control);
//		case DEV_RS485: return _open_rs485(dev, control);
		default: errno = ENODEV; return (-1);
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}

/* xio_control() - set device parameters  

	This isn't ioctl. It's works differently. 
	Provides a rehash of most of the io_open() parameter settings only with an fd
	Read the individual devices as different devices may work differently

	fd		valid file descriptor returned from io_open()
	control	device parameters
	arg		argument required by some parameters
	
	return	Success: File descriptor for device (fd) 
			Error:	 -1 and sets errno. See io_open()

*/

int8_t xio_control(uint8_t fd, uint32_t control, int16_t arg)
{
	switch (fd) {
		case FD_USB: return xio_control_usb(fd_ptrs[fd], control, arg);
//		case FD_RS485: return xio_control_rs485(fd_ptrs[fd], control, arg);
		default: errno = ENODEV; return (-1);
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}


/* xio_close() - close device 
	
	Close FD device. Stops all operations. Frees resources.
	In theory. In fact it's a lot like Hotel California.

	returns	  0 if successful. 
	
			 -1 Error returns -1 and sets errno

			 EBADF	fd isn't a valid open file descriptor.
			 EINTR	the close() call was interrupted by a signal.
			 EIO	an I/O error occurred
*/

int8_t xio_close(uint8_t fd)
{
	return (0);
}


/* xio_read() - read one or more characters from device 

	fd		Valid device handle (file descriptor) returned from io_open()
	buf		Address of buffer to read into - this will be a RAM (string)
	size	Number of characters to read (See Read/Write Modes for more details)
			  0		Returns zero and no other results
			  1-N	SIZE_MODE: Read 1-N chars. Returns error if N > RX_SIZE_MAX
			 -1		LINE_MODE: Read until delimiter is read from device
			 -2		STR_MODE:  Read until nul is read from device

	returns	  1-N	Number of characters read
			 -1 	Error returns -1 and sets errno:

			 EBADF	fd is not a valid file descriptor or is not open for reading.
			 EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
					 no data was immediately available for reading.
			 EINVAL	Invalid argument - some negative number other than -1 or -2
			 EFBIG	String requested is too big and was not read
			 EMSGSIZE String exceeded maximum size and was read up to the max
*/

int16_t xio_read(uint8_t fd, char *buf, int16_t size)
{
	switch (fd) {
		case (FD_USB): return (xio_read_usb(fd_ptrs[fd], buf, size));
		default: { errno = EBADF; return -1; }
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}

/* xio_write() - write one or more characters to device

	fd		valid device handle (file descriptor) returned from io_open()

	buf		address of buffer to write from. This will be a RAM (string) 
				address unless if DEV_EEPROM or DEV_PROGMEM selected

	size	Number of characters to write (See Read/Write Modes for more details)
			  0		Returns zero and no other results
			  1-N	SIZE_MODE: Write 1-N chars. Returns error if N > TX_SIZE_MAX
			 -1		LINE_MODE: Write until delimiter is found in source buffer
			 -2		STR_MODE:  Write until nul is found in source buffer
			 -3 	PSTR_MODE  Write string from program memory until NUL found

	returns   1-N	Number of characters written
			 -1 	Error returns -1 and sets errno:

			 EBADF	fd is not a valid file descriptor or is not open for reading.
			 EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
					 no data was immediately available for reading.
			 EINVAL	Invalid argument - some negative number other than -1 or -2
			 EFBIG  String exceeded maximum size and was not written
			 EMSGSIZE String exceeded maximum size and was written up to the max
*/

int16_t xio_write(uint8_t fd, const char *buf, int16_t size)
{
	switch (fd) {
		case (FD_USB): return (xio_write_usb(fd_ptrs[fd], buf, size));
		default: { errno = EBADF; return -1; }
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}


/* xio_getc() - read one character from device
   xio_putc() - write one character to device

	fd		valid device handle (file descriptor) returned from io_open()
	c 		character to write

	Blocking and other behaviors set by xio_open() / xio_control()
 */

char xio_getc(uint8_t fd)
{
	switch (fd) {
		case (FD_USB): return(xio_getc_USART(fd_ptrs[fd]));
		default: { errno = EBADF; return ERR_EOF; }
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}

char xio_putc(uint8_t fd, const char c)
{
	switch (fd) {
		case (FD_USB): return(xio_putc_USART(fd_ptrs[fd],c));
		default: { errno = EBADF; return ERR_EOF; }
	}
	errno = EWTF;
	return (-1);		// should never execute this return
}

/**********************************************************************************
 * UTILITY ROUTINES
 **********************************************************************************/

/*
 *	_echo_to_console()
 */

void _echo_to_console(char c)
{
	xio_putc_USART(&fd_usb, c);
}

/*
 * xio_get_fd() - get the FD given the device number 
 */

uint8_t xio_get_fd(uint8_t dev)
{
	return (uint8_t)pgm_read_byte(&fdes[dev]);
}

/* 
 * xio_get_fd_ptr() - get the FD pointer given the FD 
 */

struct fdUSART *xio_get_fd_ptr(uint8_t fd)
{
	return fd_ptrs[fd];
}

/**********************************************************************************
 * Compatibility with wiring_serial.c
 **********************************************************************************/

/* printIntegerInBase() */

void printIntegerInBase(unsigned long n, unsigned long base)
{ 
	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
	unsigned long i = 0;

	if (n == 0) {
		printByte('0');
		return;
	} 

	while (n > 0) {
		buf[i++] = n % base;
		n /= base;
	}

	for (; i > 0; i--)
		printByte(buf[i - 1] < 10 ?
			'0' + buf[i - 1] :
			'A' + buf[i - 1] - 10);
}

/* printInteger() */

void printInteger(long n)
{
	if (n < 0) {
		printByte('-');
		n = -n;
	}
	printIntegerInBase(n, 10);
}

/* printFloat() */

void printFloat(double n)
{
	double integer_part, fractional_part;
	fractional_part = modf(n, &integer_part);
	printInteger(integer_part);
	printByte('.');
	printInteger(round(fractional_part*1000));
}

/* printHex() */

void printHex(unsigned long n)
{
	printIntegerInBase(n, 16);
}
