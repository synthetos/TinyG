/*
  xmega_serial.c - serial functions for xmega family
  Modeled after *nix serial io
  .
  Copyright (c) 2010 Alden S. Hart, Jr.

  Changes:
  - All the device register names are different from the ATmega series
  - ISRs are called differently
  - wiring_serial routines not (sup)ported are:
	- printMode()
	- printNewline()
	- printOctal()
	- printBinary()
	- print()

  - implemented more efficient constructs as per:
	http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235799875 

  ToDo:
  - put buffers in structs and generalize routines to support multiple serial channels
  - Make beginSerial accept a channel argument
  - make beginSerial() actually use the baud rate in the input arg
*/

#include <math.h>
#include <avr/pgmspace.h>
#include "xmega_support.h"
#include "wiring_private.h"
#include "wiring_serial.h"
#include "config.h"
#include "xmega_serial.h"


/* io_init() - init serial and "file" io sub-system */

void io_init(void)
{
	errno = 0;
	return
}

/* io_open() - open a device such as a serial port or program memory "file" handle 

	dev		Device specifier (takes the place of Unix path variable)
				Device number 0 - N to specify a device (see #defines DEV_XXXXX)

	parms	Valid open parameters
				RDONLY		read only - attempt to write will cause error
				WRONLY		write only - attempt to read will cause error
				RDWR		read & write operations supported 
				RDNONBLOCK	reads will return immediately if char(s) not available
				WRNONBLOCK	writes do not wait for char(s) to be written
				WRECHO		writes will echo character to console
				RDCIRCULAR	treats buffer as circular buffer - no overflow
				BAUD		baud rate enumeration for RX and TX (not independently settable)
				[ADDR]		address of program memory to read. If if program memory 
							is selected device parameters must be set using 
							io_control(). Default is read-only

	returns	File descriptor for device (fd) 
			Error returns -1 and sets errno

				EACCES	The requested access to the file is not allowed, or search permission is denied for one of the directories in the path prefix of pathname, or the file did not exist yet and write access to the parent directory is not allowed. (See also path_resolution(2).)
				EEXIST	pathname already exists and O_CREAT and O_EXCL were used.
				EFAULT	pathname points outside your accessible address space.
				EISDIR	pathname refers to a directory and the access requested involved writing (that is, O_WRONLY or O_RDWR is set).
				ELOOP	Too many symbolic links were encountered in resolving pathname, or O_NOFOLLOW was specified but pathname was a symbolic link.
				EMFILE	The process already has the maximum number of files open.
				ENAMETOOLONG	pathname was too long.
				ENFILE	The system limit on the total number of open files has been reached.
				ENODEV	pathname refers to a device special file and no corresponding device exists. (This is a Linux kernel bug; in this situation ENXIO must be returned.)
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
*/

int8_t io_open(uint32_t dev, unit32_t parms)
{
	
}

/* io_close() - close device 
	
	Close FD device. Stops all operations. Frees resources.

	return	0. Error returns -1 and sets errno

				EBADF	fd isn't a valid open file descriptor.
				EINTR	The close() call was interrupted by a signal.
				EIO		An I/O error occurred.

*/

int io_close(uint8_t fd)
{
	return (0)
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

int io_read(unit8_t fd, char *buf, int count)
{
	return (0)
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
int io_write(uint8_t fd, char *buf, int count);
{
	return (0)
}


/* io_control() - set device parameters  

	This isn't to ioctl. It's works differently. 
	Provides a rehash of the io_open() parameter settings only with an fd

	fd		valid device handle (file descriptor) returned from io_open()

	parms	Valid open parameters. See io_open() for details
	
	return	Success: File descriptor for device (fd) 
			Error:	 -1 and sets errno

				EACCES	The requested access to the file is not allowed, or search permission is denied for one of the directories in the path prefix of pathname, or the file did not exist yet and write access to the parent directory is not allowed. (See also path_resolution(2).)
				EEXIST	pathname already exists and O_CREAT and O_EXCL were used.
				EFAULT	pathname points outside your accessible address space.
				EISDIR	pathname refers to a directory and the access requested involved writing (that is, O_WRONLY or O_RDWR is set).
				ELOOP	Too many symbolic links were encountered in resolving pathname, or O_NOFOLLOW was specified but pathname was a symbolic link.
				EMFILE	The process already has the maximum number of files open.
				ENAMETOOLONG	pathname was too long.
				ENFILE	The system limit on the total number of open files has been reached.
				ENODEV	pathname refers to a device special file and no corresponding device exists. (This is a Linux kernel bug; in this situation ENXIO must be returned.)
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
*/

int8_t io_control(uint8_t fd, unit32_t parms)
{

	return (0)
}

int io_flush(void);
void io_ioctl(uint8_t *fd);










/*  Define constants and variables for buffering incoming serial data.  
	We're using a circular buffer, in which rx_buffer_head is the index of the
	location to which to write the next incoming character and rx_buffer_tail
	is the index of the location from which to read.
*/

#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1) // new constant introduced
unsigned char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;					// changed from type int
uint8_t rx_buffer_tail = 0;					// changed from type int

/* beginSerial() - xmega serial routine
   
   Hijacked this routine to configure PORTC, USARTD1 (PORTC:7=Tx, PORTF:6=Rx)
   "baud" value is ignored and set internally to the routine
 */

void beginSerial(long baud) 
{
	PORTC.DIRCLR = (1<<2); 					// clr PORTC:6 receive pin as input
	PORTC.DIRSET = (1<<3); 					// set PORTC:7 transmit pin as output
	PORTC.OUTSET = (1<<3);					// set TX pin HI as initial state

	USARTC0.BAUDCTRLA = USB_BSEL;
	USARTC0.BAUDCTRLB = USB_BSCALE;
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;		   // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use
}

void beginSerialC1(long baud) 
{
	PORTC.DIRCLR = (1<<6); 					// clr PORTC:6 receive pin as input
	PORTC.DIRSET = (1<<7); 					// set PORTC:7 transmit pin as output
	PORTC.OUTSET = (1<<7);					// set TX pin HI as initial state

//	PORTC.DIRSET = enableDE_bm;				// set PORTC:5 for DE line as output
//	PORTC.OUTCLR = enableDE_bm;        		// clr PORTC:5 (disabled)
//	PORTC.DIRSET = enableRE_bm;				// set PORTC:4 for ~RE line as output
//	PORTC.OUTCLR = enableRE_bm;				// clr PORTC:4 (enabled) 

	USARTC1.BAUDCTRLA = USB_BSEL;
	USARTC1.BAUDCTRLB = USB_BSCALE;
	USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	USARTC1.CTRLA = USART_RXCINTLVL_MED_gc;		   // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use
}

/* ISR for USB serial RX - receive serial byte from USB port*/

ISR(USARTC0_RXC_vect)
{
	unsigned char c = USARTC0.DATA;
	uint8_t i = (rx_buffer_head + 1) & RX_BUFFER_MASK;

	/*  If we should be storing the received character into the location
		just before the tail (meaning that the head would advance to the
		current location of the tail), we're about to overflow the buffer
		and so we don't write the character or advance the head. */

	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

ISR(USARTC1_RXC_vect)
{
	unsigned char c = USARTC1.DATA;
	uint8_t i = (rx_buffer_head + 1) & RX_BUFFER_MASK;

	/*  If we should be storing the received character into the location
		just before the tail (meaning that the head would advance to the
		current location of the tail), we're about to overflow the buffer
		and so we don't write the character or advance the head. */

	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

/* serialWrite() - write character to serial port */

void serialWrite(unsigned char c) 
{
	while(!(USARTC0.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC0.DATA = c;							// write data register
}

/*
void serialWrite(unsigned char c) 
{
	while(!(USARTC1.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC1.DATA = c;							// write data register
}
*/

/* serialAvailable() - optimized for 8 bit operation */

uint8_t serialAvailable() 
{
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) & RX_BUFFER_MASK;
}

/* serialRead() - optimized for 8 bit architecture 
	
	This routine is modified from the original wiring_serial code in 2 ways:

	- The original routine returns -1 if there is no character to read. 
	  Returning a -1 as a character type seems to cause problems
	  This value has beeen changed to ascii EOT (0x04) which is the closest thing
	  I could not find a "no data here, move along folks" character.

	- The buffer wraparound math as been replaced with masking instead of modulus.
	  This requires that the buffer size be a binary multiple. It's currently 128.
*/

char serialRead() 
{
	/* if the head isn't ahead of the tail, we don't have any characters */
	if (rx_buffer_head == rx_buffer_tail) {
		return 0x04;											// ASCII EOT
	} else {
		char c = rx_buffer[rx_buffer_tail];
//		rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;	// original code
		rx_buffer_tail = (rx_buffer_tail + 1) & RX_BUFFER_MASK;	// optimized code
		return c;
	}
}

void serialFlush()
{
	/*  don't reverse this or there may be problems if the RX interrupt
		occurs after reading the value of rx_buffer_head but before writing
		the value to rx_buffer_tail; the previous value of rx_buffer_head
		may be written to rx_buffer_tail, making it appear as if the buffer
		were full, not empty. */
	rx_buffer_head = rx_buffer_tail;
}

/* printByte has been removed in favor of a macro pointing to serialWrite
void printByte(unsigned char c)
{
	serialWrite(c);
}
*/

void printString(const char *s)
{
	while (*s) {
		printByte(*s++);
	}
}

/*  printPgmString() - print a string stored in PGM-memory */
void printPgmString(const char *s)
{
	unsigned char c;
	while ((c = pgm_read_byte_near(s++))) {
		printByte(c);
	}
}

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

/* Unsupported wiring_serial routines 
void printMode(int mode)
{
	// do nothing, we only support serial printing, not lcd.
}

void printNewline()
{
	printByte('\n');
}
 
void printOctal(unsigned long n)
{
	printIntegerInBase(n, 8);
}

void printBinary(unsigned long n)
{
	printIntegerInBase(n, 2);
}

 * Including print() adds approximately 1500 bytes to the binary size,
 * so we replace it with the smaller and less-confusing printString(),
 * printInteger(), etc.
void print(const char *format, ...)
{
	char buf[256];
	va_list ap;
	
	va_start(ap, format);
	vsnprintf(buf, 256, format, ap);
	va_end(ap);
	
	printString(buf);
}
*/


extern int serial_open (char *, int *,  int, speed_t);
extern int serial_close (int);
extern int serial_readchar (int, uint8_t *);
extern int serial_write (int, void *, size_t);
extern int serial_read (int, void *, size_t);


#include <sys/types.h>
#include <sys/fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <err.h>

#include "serialio.h"

/*
 * Serial I/O routines.
 */

static int serial_setup (int fd, speed_t baud);

/*
 * Read a character from the serial port. Note that this
 * routine will block until a valid character is read.
 */

int
serial_readchar (int fd, uint8_t * c)
{
        char            b;
        int             r;

        while ((r = read (fd, &b, 1)) == -1)
                ;

        if (r != -1) {
                *c = b;
#ifdef MSR_DEBUG
                printf ("[0x%x]\n", b);
#endif
        }

        return (r);
}

/*
 * Read a series of characters from the serial port. This
 * routine will block until the desired number of characters
 * is read.
 */

int
serial_read (int fd, void * buf, size_t len)
{
        size_t i;
        uint8_t b, *p;

        p = buf;

#ifdef SERIAL_DEBUG
        printf("[RX %.3d]", len);
#endif
        for (i = 0; i < len; i++) {
                serial_readchar (fd, &b);
#ifdef SERIAL_DEBUG
                printf(" %.2x", b);
#endif
                p[i] = b;
        }
#ifdef SERIAL_DEBUG
        printf("\n");
#endif

        return (0);
}

int
serial_write (int fd, void * buf, size_t len)
{
        return (write (fd, buf, len));
}

/*
 * Set serial line options. We need to set the baud rate and
 * turn off most of the internal processing in the tty layer in
 * order to avoid having some of the output from the card reader
 * interpreted as control characters and swallowed.
 */

static int
serial_setup (int fd, speed_t baud)
{
        struct termios  options;


        if (tcgetattr(fd, &options) == -1)
                return (-1);

        /* Set baud rate */
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        /* Control modes */

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        /*
         * Local modes
         * We have to clear the ISIG flag to defeat signal
         * processing in order to see the file separator
         * character (0x1C) which the device will send as
         * part of its end of record markers.
         */

        options.c_lflag &= ~ICANON;
        options.c_lflag &= ~ECHO;
        options.c_lflag &= ~ECHONL;
        options.c_lflag &= ~ISIG;
        options.c_lflag &= ~IEXTEN;

        /* Input modes */

        options.c_iflag &= ~ICRNL;
        options.c_iflag &= ~IXON;
        options.c_iflag &= ~IXOFF;
        options.c_iflag &= ~IXANY;
        options.c_iflag |= IGNBRK;
        options.c_iflag |= IGNPAR;

        /* Output modes */

        options.c_oflag &= ~OPOST;

        if (tcsetattr(fd, TCSANOW, &options) == -1)
                return (-1);

        return (0);
}

int
serial_open(char *path, int * fd, int blocking, speed_t baud)
{
        int             f;

        f = open(path, blocking | O_RDWR | O_FSYNC);

        if (f == -1)
                return (-1);

        if (serial_setup (f, baud) != 0) {
                close (f);
                return (-1);
        }

        *fd = f;

        return (0);
}

int
serial_close(int fd)
{
        close (fd);
        return (0);
}
