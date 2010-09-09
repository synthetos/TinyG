/*
  xmega_io.h - serial and "file" IO functions for xmega family
  core module
  Modeled after unix fileio 

  Copyright (c) 2010 Alden S. Hart, Jr.

  See module headers in xmega_io.c and see xmega_io_doc.h for details

  Procedure to use the USB port from mac OSX:
	- Install the FTDI virtual serial port driver
	- Find your tty device in /dev directory, e.g.
		/dev/tty.usbserial-A700eUQo
	- Invoke screen using your tty device at 115200 baud. From terminal prompt, e.g:
		screen /dev/tty.usbserial-A700eUQo 115200

  If you are running screen (under terminal) in OSX you may want to do this first:
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
  (ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)

*/

#ifndef xmega_io_h
#define xmega_io_h

/* 
 * Global variables
 */

int	errno;									// error number

/* 
 * Function prototypes and aliases
 */

// base functions
void xio_init(void);
int8_t xio_open(uint8_t dev, uint32_t control);
int8_t xio_close(uint8_t fd);
int8_t xio_control(uint8_t fd, uint32_t control, int16_t arg);
int16_t xio_read(uint8_t fd, char *buf, int16_t size);
int16_t xio_write(uint8_t fd, const char *buf, int16_t size);
char xio_putc(uint8_t fd, const char c);
char xio_getc(uint8_t fd);					// see stdio.h to get this right

// helper routines for export to other device modules 
void _echo_to_console(char c);
uint8_t xio_get_fd(uint8_t dev);
struct fdUSART *xio_get_fd_ptr(uint8_t fd);

// aliases to redefine names to the UNIX IO names
#define open(d,c) xio_open(d,c)		
#define close(fd) xio_close(fd)
#define control(fd,c) xio_control(fd,c)
#define read(fd,b,s) xio_read(fd,b,s)
#define write(fd,b,s) xio_write(fd,b,s)
//#define getc() xio_getc(FD_CON)			// redefines stdio.h getc()
//#define putc(c) xio_putc(FD_CON,c)		// redefines stdio.h putc()

// aliases to define line, string and program memory versions
#define readln(fd,b) xio_read(fd,b,LINE_MODE)
#define writeln(fd,b) xio_write(fd,b,LINE_MODE)
#define readstr(fd,b) xio_read(fd,b,STR_MODE)
#define writestr(fd,b) xio_write(fd,b,STR_MODE)
#define readpstr(fd,b) xio_read(fd,b,PSTR_MODE)
#define writepstr(fd,b) xio_write(fd,b,PSTR_MODE)

// wiring_serial.c compatibility (see also xmega_io.c section)
#define serialRead() xio_getc(FD_CON)
#define printByte(c) xio_putc(FD_CON,c)
#define printString(b) xio_write(FD_CON,(const char *)b,STR_MODE)
#define printPgmString(b) xio_write(FD_CON,b,PSTR_MODE)
void printIntegerInBase(unsigned long n, unsigned long base);
void printInteger(long n);
void printFloat(double n);
void printHex(unsigned long n);

/*************************************
 *
 * IO Subsystem General Assignments
 *
 *************************************/

// configure the fd_ptr array
#define FD_USB 1					// file descriptor for USB port
#define FD_RS485 2					// file descriptor for RS485 port
#define FD_MAX 3					// size of FD pointer array
#define FD_CON FD_USB				// stand-in for standard IO / console

// various read/write modes
#define SIZE_MODE 0					// read / write by size
#define LINE_MODE -1				// read / write to delimiter
#define STR_MODE -2					// read / write string to ASCII nul (0, \0)
#define PSTR_MODE -3				// read / write program memory string

// nifty global constants
#define NUL 0						// ASCII NUL char (is not NULL: that's for ptrs)
#define ERR_EOF 0					// EOF used for putc / getc error returns
#define NO_LIMIT -1					// no limit on read or write size
#define READ_BUFFER_SIZE 80			// default size for read buffers (buf)

/* 
 *	Devices recognized by IO system functions 
 *	(Note: by leaving these contiguous you have a better chance the compiler
 *	 will implement an efficient switch statement - like a computed goto)
 */

// native xmega devices
#define DEV_NULL	0				// NULL device (OK, not native)

#define DEV_PORTA	1				// Define ports as IO devices
#define DEV_PORTB	2
#define DEV_PORTC	3
#define DEV_PORTD	4
#define DEV_PORTE	5
#define DEV_PORTF	6
#define DEV_PORTG	7				// not implemented on xmega A3s
#define DEV_PORTH	8				// not implemented on xmega A3s
#define DEV_PORTJ	9				// not implemented on xmega A3s
#define DEV_PORTK	10				// not implemented on xmega A3s
#define DEV_PORTL	11				// not implemented on xmega A3s
#define DEV_PORTM	12				// not implemented on xmega A3s
#define DEV_PORTN	13				// not implemented on xmega A3s
#define DEV_PORTP	14				// not implemented on xmega A3s
#define DEV_PORTQ	15				// not implemented on xmega A3s
#define DEV_PORTR	16				// special purpose port - programming bits only 

#define DEV_USARTC0	17				// USARTS C0 - F1
#define DEV_USARTC1	18
#define DEV_USARTD0	19
#define DEV_USARTD1	20
#define DEV_USARTE0	21
#define DEV_USARTE1	22
#define DEV_USARTF0	23
#define DEV_USARTF1	24

#define DEV_SPIC	25				// SPI interfaces C - F
#define DEV_SPID	26
#define DEV_SPIE	27
#define DEV_SPIF	28

#define DEV_TWIC	29				// Two Wire interfaces C and E
#define DEV_TWIE	30

#define DEV_IRCOM	31				// IR communications module
#define DEV_AES		32				// AES crypto accelerator

#define DEV_ADCA	33				// ADCs
#define DEV_ADCB	34

#define DEV_DACA	35				// DACs
#define DEV_DACB	36

#define	DEV_SRAM	37				// string in static RAM
#define DEV_EEPROM	38				// string in EEPROM
#define DEV_PROGMEM 39				// string in application program memory (FLASH)
#define DEV_TABLEMEM 40				// string in app table program memory (FLASH)
#define DEV_BOOTMEM 41				// string in boot program memory (FLASH)


// derived devices (synthetic devices)
#define DEV_CONSOLE	42				// mapped to USB, here for convenience
#define DEV_USB		43				// USB comm and controls packaged
#define DEV_RS485	44				// RS485 comm and controls packaged
#define DEV_ENCODERS 45				// Encoder comm and controls packaged
#define DEV_LIMITS	46				// limit switches
#define DEV_BRIDGE	47				// USB to RS485 bridge


/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
 */
                             		// BSEL	  BSCALE
#define	IO_BAUD_UNSPECIFIED 0		//	0		0		// use default value 
#define IO_BAUD_9600 1				//	207		0
#define IO_BAUD_19200 2				//	103		0
#define IO_BAUD_38400 3				//	51		0
#define IO_BAUD_57600 4				//	34		0
#define IO_BAUD_115200 5			//	33		(-1<<4)
#define IO_BAUD_230400 6			//	31		(-2<<4)
#define IO_BAUD_460800 7			//	27		(-3<<4)
#define IO_BAUD_921600 8			//	19		(-4<<4)
#define IO_BAUD_500000 9			//	1		(1<<4)
#define IO_BAUD_1000000 10			//	1		0
#define	IO_BAUD_DEFAULT IO_BAUD_115200

// io_open() io_control() control bits
#define IO_BAUD_gm		0x0000000F	// baud rate enumeration mask (keep in LSbyte)

#define IO_RDONLY		(1<<8) 		// read enable bit
#define IO_WRONLY		(1<<9)		// write enable only
#define IO_RDWR			(0) 		// read & write

#define IO_ECHO			(1<<10)		// echo reads from device to console (line level)
#define IO_NOECHO		(1<<11)		// disable echo

#define IO_RDBLOCK		(1<<12)		// enable blocking reads
#define IO_RDNONBLOCK	(1<<13)		// disable blocking reads
#define IO_WRBLOCK		(1<<14)		// enable blocking writes (not implemented)
#define IO_WRNONBLOCK	(1<<15)		// disable blocking writes (not implemented)
//#define IO_RDWRBLOCK 	(IO_RDBLOCK | IO_WRBLOCK)  // enable blocking on RD/WR
//#define IO_RDWRNONBLOCK (IO_RDNONBLOCK | IO_WRNONBLOCK)

#define IO_RD_SIZE_MAX	(1<<4)		// set read size limit (xio_control() only)
#define IO_WR_SIZE_MAX	(1<<5)		// set write size limit (xio_control() only)

// fd.flags flags (which are NOT the similar bits in the control word, above)
#define IO_FLAG_RD_bm		(1<<0)	// enabled for read
#define IO_FLAG_WR_bm		(1<<1)	// enabled for write
#define IO_FLAG_RD_BLOCK_bm	(1<<2)	// enable blocking read
#define IO_FLAG_WR_BLOCK_bm	(1<<3)	// enable blocking write
#define IO_FLAG_ECHO_CHAR_bm (1<<4)	// echo read chars to console 
#define IO_FLAG_FLOW_CONTROL_ENABLE_bm (<<5)	// enable flow control for device
#define IO_FLAG_FLOW_CONTROL_ON_bm (1<<6) 		// device is in flow control (now)

#define IO_FLAG_DEFAULT_gm (IO_FLAG_RD_bm | IO_FLAG_WR_bm | IO_FLAG_RD_BLOCK_bm | IO_FLAG_ECHO_CHAR_bm)

#define READ_ENABLED(a) (a & IO_FLAG_RD_bm)			// TRUE if read enabled
#define WRITE_ENABLED(a) (a & IO_FLAG_WR_bm)		// TRUE if write enabled
#define BLOCKING_ENABLED(a) (a & IO_FLAG_RD_BLOCK_bm)// TRUE if read blocking enabled
#define ECHO_ENABLED(a) (a & IO_FLAG_ECHO_CHAR_bm)	// TRUE if char echo mode enabled

#endif
