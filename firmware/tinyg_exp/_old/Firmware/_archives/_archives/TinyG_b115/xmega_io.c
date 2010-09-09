/*
  xmega_io.c - serial functions for xmega family
  Modeled after *unix serial io
  .
  Copyright (c) 2010 Alden S. Hart, Jr.

  This IO subsystem looks like Unix IO but there are sigificant differences
  	- It's Kabuki Theater. There is no malloc, so everything is pre-allocated
	- You can read a file descriptor to the next delimiter. This is very convenient
	- io_control() is not ioctl(). They are totally different
*/

//#include <math.h>
//#include <stdio.h>
//#include <stdarg.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "xmega_support.h"
#include "config.h"
#include "xmega_io.h"
#include "xmega_errno.h"


/* variables and functions with scope to this IO module */

struct fdUSART fd_usb, fd_rs485;			// pre-allocate 2 USART structs
struct fdUSART *fd_ptr[2];					// array of pointers to the structs

int8_t _open_dev_usb(uint32_t *dev, uint32_t *parms);  // helper routine to open USB



/* io_init() - init serial and "file" io sub-system */

void io_init(void)
{
//	fd_ptr[0] = (struct fdSerial)0;			// null device
	fd_ptr[1] = &fd_usb;					// this gets assigned to serial port C0
	fd_ptr[2] = &fd_rs485;					// this gets assigned to serial port C1

	errno = 0;
	return;
}

/* interrupt routines */

ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	uint8_t i;			// buffer index 
	unsigned char c;	// receoved character

  /* Buffer state can be one of:
  	  - buffer has space (CTS should be HI)
	  - buffer is full 	 (CTS should be LO)
	  - buffer becomes full with this character (write char and set CTS LO)
  */

	if ((++fd_ptr[FD_USB]->rx_buf_head) >= RX_BUFSIZE) {
		fd_ptr[FD_USB]->rx_buf_head = 0;					// wrap condition
	}

/*	  One of:If the buffer head would advance We should be storing the received character into the location
		just before the tail (meaning that the head would advance to the
		current location of the tail), we're about to overflow the buffer
		and so we don't write the character or advance the head. */

	c = USARTC0.DATA;

	if ((++fd_ptr[FD_USB]->rx_buf_head) >= RX_BUFSIZE) {
		fd_ptr[FD_USB]->rx_buf_head = 0;					// wrap condition
	}



	if (i != fd_ptr[1]->rx_buf_tail) {
		fd_ptr[1]->rx_buf[fd_ptr[1]->rx_buf_head] = c;
		fd_ptr[1]->rx_buf_head = i;
	}
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

	parms	Valid parameters for io_open() and io_control()
				IO_RDONLY		read only - attempt to write will cause error
				IO_WRONLY		write only - attempt to read will cause error
				IO_RDWR			read & write operations supported 
				IO_RDNONBLOCK	reads will return immediately if char(s) not avail
				IO_WRNONBLOCK	writes do not wait for char(s) to be written
				IO_WRECHO		writes will echo character to console
				IO_RDCIRCULAR	treats buffer as circular buffer - no overflow
				[BAUD]			baud rate enum value for RX and TX 
								(not independently settable)
				[ADDR]			address of program memory to read. If if program 
								memory is selected device parameters must be set 
								using io_control(). Default is read-only

	returns	File descriptor for device (fd) 
			Error returns -1 and sets errno

				ENODEV	requested dev is not supported or illegal

				EACCES	The requested access to the file is not allowed, or search permission is denied for one of the directories in the path prefix of pathname, or the file did not exist yet and write access to the parent directory is not allowed. (See also path_resolution(2).)
				EEXIST	pathname already exists and O_CREAT and O_EXCL were used.
				EFAULT	pathname points outside your accessible address space.
				EISDIR	pathname refers to a directory and the access requested involved writing (that is, O_WRONLY or O_RDWR is set).
				ELOOP	Too many symbolic links were encountered in resolving pathname, or O_NOFOLLOW was specified but pathname was a symbolic link.
				EMFILE	The process already has the maximum number of files open.
				ENAMETOOLONG	pathname was too long.
				ENFILE	The system limit on the total number of open files has been reached.
				ENOENT	O_CREAT is not set and the named file does not exist. Or, a directory component in pathname does not exist or is a dangling symbolic link.
				ENOMEM	Insufficient kernel memory was available.
				ENOSPC	pathname was to be created but the device containing pathname has no room for the new file.
				ENOTDIR	A component used as a directory in pathname is not, in fact, a directory, or O_DIRECTORY was specified and pathname was not a directory.
				ENXIO	O_NONBLOCK | O_WRONLY is set, the named file is a FIFO and no process has the file open for reading. Or, the file is a device special file and no corresponding device exists.
				EOVERFLOW pathname refers to a regular file, too large to be opened; see O_LARGEFILE above.
				EPERM The O_NOATIME flag was specified, but the effective user ID of the caller did not match the owner of the file and the caller was not privileged (CAP_FOWNER).
				EROFS pathname refers to a file on a read-only filesystem and write access was requested.
				ETXTBSY	pathname refers to an executable image which is currently being executed and write access was requested.
				EWOULDBLOCK The O_NONBLOCK flag was specified, and an incompatible lease was held on the file (see fcntl(2)).

---- Enough of the theoretical stuff. Notes about this implementation ----

	Only recognizes the synthetic device DEV_USB. All else will fail.
	Ignores baud rate config. Always sets up 115200 baud
	Ignores all parameters and behaves as follows: 
	Implements blocking reads to delimiters (-1 behavior)
	Implements blocking writes to delimiters (-1 behavior)


*/

int8_t io_open(uint32_t dev, uint32_t control)
{
//	int8_t fd;

	switch (dev) {
		case DEV_USB: return _open_dev_usb(&dev, &control);
		default: errno = ENODEV; return (-1);
	}
	return (0);
}

int8_t _open_dev_usb(uint32_t *dev, uint32_t *control)
{
	struct fdUSART *f = fd_ptr[FD_USB];	// assign the fd struct 
											// to its spot in the fd pointer array
	f->fd = FD_USB;
	f->control = *control;					// save control parameters
	f->usart = &USB_USART;					// bind USB USART to struct
	f->port = &USB_PORT;					// bind corresponding port to struct

	f->port->DIRCLR = USB_RX_bm; 			// clr RX pin as input
	f->port->DIRSET = USB_TX_bm; 			// set TX pin as output
	f->port->OUTSET = USB_TX_bm;			// set TX HI as initial state

//	PORTC.DIRCLR = USB_RX_bm; 				// clr RX pin as input
//	PORTC.DIRSET = USB_TX_bm; 				// set TX pin as output
//	PORTC.OUTSET = USB_TX_bm;				// set TX HI as initial state

// DEBUG THIS ONCE ALL THE REST IS WORKING
//	f->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
//	f->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
//	f->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)

//	USARTC0.BAUDCTRLA = USB_BSEL;
//	USARTC0.BAUDCTRLB = USB_BSCALE;
//	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
//	USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;		   // receive interrupt medium level

	f->usart->BAUDCTRLA = USB_BSEL;
	f->usart->BAUDCTRLB = USB_BSCALE;
	f->usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	f->usart->CTRLA = USART_RXCINTLVL_MED_gc;		 // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use

	return (1);
}


/* io_close() - close device 
	
	Close FD device. Stops all operations. Frees resources.

	returns	0 if successful. Error returns -1 and sets errno

				EBADF	fd isn't a valid open file descriptor.
				EINTR	The close() call was interrupted by a signal.
				EIO		An I/O error occurred.

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
	
				1-N		Read 1-N chars. Behavior varies depending on blocking mode
						Will return error if greater than SSIZE_MAX 

				-1		Read until next delimiter or NULL encountered (TO_NEXT)
						Return null-terminated string 
						(NULL overwrites delimiter character)

				-2		Read until NULL encountered	(TO_EOF)

	returns	Number of characters read
			Error returns -1 and sets errno

				EBADF	fd is not a valid file descriptor or is not open for reading.

				EAGAIN	Non-blocking I/O has been selected using RDNONBLOCK and
						no data was immediately available for reading.

				EIO 	General I/O error. (like Dbase2 "Disk is Full" error)
				
				EFAULT buf is outside your accessible address space.
				EINTR The call was interrupted by a signal before any data was read.
				EINVAL fd is attached to an object which is unsuitable for reading; or the file was opened with the O_DIRECT flag, and either the address specified in buf, the value specified in count, or the current file offset is not suitably aligned.
				EISDIR fd refers to a directory.
*/

int io_read(uint8_t fd, char *buf, int count)
{
	return (0);
}

/* io_write() - write one or more characters to device

	fd		valid device handle (file descriptor) returned from io_open()

	buf		address of buffer to write from
			this will be a RAM (string) address unless if DEV_EEPROM or DEV_PROGMEM selected

	count		Number of characters to read

				0		Returns zero and no other results
	
				1-N		Read 1-N chars. Behavior varies depending on blocking mode
						Will return error if greater than SSIZE_MAX 

				-1		Read until next delimiter or NULL encountered (TO_NEXT)
						Return null-terminated string 
						(NULL overwrites delimiter character)

				-2		Read until NULL encountered	(TO_EOF)

	Returns:	Success: Number of characters read
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
int io_write(uint8_t fd, char *buf, int count)
{
	return (0);
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




