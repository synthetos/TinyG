/*
  xmega_io.c - serial functions for xmega family
  Modeled after *unix serial io
  .
  Copyright (c) 2010 Alden S. Hart, Jr.

  This IO subsystem looks like Unix IO but there are sigificant differences
  	- It's Kabuki Theater. There is no malloc, so everything is pre-allocated
	- You can read a file descriptor to the next delimiter. This is very convenient
	- io_control() is not ioctl(). They are totally different

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
#include <util/delay.h>

#include "config.h"
#include "xmega_io.h"
#include "xmega_errno.h"


/* variables and functions with scope to this IO module */

int	errno=0;								// global error number

struct fdUSART fd_usb, fd_rs485;			// pre-allocate 2 USART structs
struct fdUSART *fd_ptr[2];					// array of pointers to the structs

int8_t _open_usb(uint32_t dev, uint32_t control);  // helper routine to open USB
int _read_usb(struct fdUSART *f, char *buf, int count);
int _read_char_USART(struct fdUSART *f);


/* io_init() - init serial and "file" io sub-system */

void io_init(void)
{ // CAN'T seem to make this first assignment work. Cast null pointer to USART struct
//	fd_ptr[0] = (struct fdUSART)0;			// null device
	fd_ptr[1] = &fd_usb;					// this gets assigned to serial port C0
	fd_ptr[2] = &fd_rs485;					// this gets assigned to serial port C1

	errno = 0;
	return;
}

/* interrupt routines (See module header note on circular buffers) */

/* USB_RX_ISR - USB receiver interrupt (RX)

   RX buffer states can be one of:
	- buffer has space	(CTS should be asserted)
	- buffer is full 	(CTS should be not_asserted)
	- buffer becomes full with this character (write char and assert CTS)

   We use expressions like fd_usb.rx_buf_head instead of fd_ptr[FD_USB]->rx_buf_head
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
				IO_ECHO			echo to console
				IO_BAUD_XXXXX	baud rate for RX and TX (not independently settable)
				[ADDR]			address of program memory to read. See Address mode.

			Defaults are:
				IO_RDWR			read and write
			   ~IO_RDWRNONBLOCK blocking reads and writes
				IO_WRECHO		will echo
				IO_BAUD_115200	baud rate set to 1115200\

			Address mode:
				Address mode is enabled if device expects an addr (eg.DEV_PROGMEM)
				In address mode device parameters must be set using io_control(). 
				Default is IO_RDONLY, IO_ECHO

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
		case DEV_USB: return _open_usb(dev, control);
		default: errno = ENODEV; return (-1);
	}
	return (0);
}

int8_t _open_usb(uint32_t dev, uint32_t control)
{
	uint32_t ctrl = (IO_RDWR || IO_WRECHO ||IO_BAUD_115200);  // defaults

	struct fdUSART *f = fd_ptr[FD_USB];		// assign the fd struct 
											// to its spot in the fd pointer array
	f->fd = FD_USB;
	f->control = control;					// save control parameters
	f->rx_buf_head = 0;
	f->rx_buf_tail = 0;
	f->usart = &USB_USART;					// bind USB USART to struct
	f->port = &USB_PORT;					// bind corresponding port to struct

	f->port->DIRCLR = USB_RX_bm; 			// clr RX pin as input
	f->port->DIRSET = USB_TX_bm; 			// set TX pin as output
	f->port->OUTSET = USB_TX_bm;			// set TX HI as initial state

// DEBUG THIS ONCE ALL THE REST IS WORKING
//	f->port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
//	f->port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
//	f->port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)

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

				EFBIG	Block requested is too big (exceeds SSIZE_MAX)

				EIO 	General I/O error. (like Dbase2 "Disk is Full" error)
*/

int io_read(uint8_t fd, char *buf, int count)
{
	if (fd == FD_USB) {
		struct fdUSART *f = &fd_usb;
		return (_read_usb(f, buf, count));
//		return (_read_usb((struct fdUSART *f = &fd_usb), buf, &count)); // make this work???
	} else {
		errno = EBADF;
		return (-1);
	}
}

/* _read_usb() - USB line reader 
*/

int _read_usb(struct fdUSART *f, char *buf, int count)
{
	char c;				// character temp
	int	i = 0;			// output buffer index (buf[i])
	uint8_t mode = 0;	// 0 = count mode, 1 = line mode, 2 = null termination mode

	if (count == 0) {								// special case of 0 count
		return (0);
	} else if (count == READ_LINE) {				// line termination mode
		mode = 1; 
	} else if (count == READ_TO_NULL) {				// NULL termination mode
		mode = 2;
	} else if (count >= SSIZE_MAX) {				// too big a request made
		errno = EFBIG;
		return (-1);
	}

	while((c = _read_char_USART(f)) != -1) {		// while characters available
		buf[i++] = c;
		if ((mode == 0) && (--count == 0)) {		// count complete
			buf[i] = 0;
			return (i);
		} else if ((mode > 0) && (c == 0)) { 		// null terminated or line terminates w/null
			return (i);
		} else if ((mode == 1) && ((c == '\r') || (c == '\n') || (c == ';'))) { 
			buf[i] = 0;
			return (i);
		}
	}
	errno = EAGAIN;		// return for non-blocking operation
	return (-1);
}

/* _read_char_USART() - lowest level char reader for USARTS 

	Execute blocking or non-blocking read depending on controls
	Return character or -1 if non-blocking
	Return character or sleep() if blocking
*/

int _read_char_USART(struct fdUSART *f)
{
	char c;

	if (f->rx_buf_head == f->rx_buf_tail) {
		errno = EAGAIN;
		return (-1);
	} else {
		char c = f->rx_buf[f->rx_buf_tail];
		if (++f->rx_buf_tail >= RX_BUFSIZE) { 
			f->rx_buf_tail = 0;
		}
		return c;
	}
	errno = EAGAIN;
	return (-1);
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

//	while(!(USARTC0.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
//	USARTC0.DATA = c;							// write data register

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




