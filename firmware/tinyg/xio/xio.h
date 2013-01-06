/*
 * xio.h - Xmega IO devices - common header file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* XIO devices are compatible with avr-gcc stdio, so formatted printing 
 * is supported. To use this sub-system outside of TinyG you may need 
 * some defines in tinyg.h. See notes at end of this file for more details.
 */
/* Note: anything that includes xio.h first needs the following:
 * 	#include <stdio.h>				// needed for FILE def'n
 *	#include <stdbool.h>			// needed for true and false 
 *	#include <avr/pgmspace.h>		// defines prog_char, PSTR
 */

#ifndef xio_h
#define xio_h

// put all sub-includes here so only xio.h is needed elsewhere
#include "xio_file.h"
#include "xio_usart.h"
#include "xio_spi.h"
#include "xio_signals.h"
// Note: stdin, stdout and stderr are defined in stdio.h and are used by XIO

/*************************************************************************
 *	Device configurations
 *************************************************************************/
// Pre-allocated XIO devices (configured devices)
// Unused devices are commented out. All this needs to line up.

enum xioDevice {		// device enumerations
						// TYPE:	DEVICE:
	XIO_DEV_USB,		// USART	USB device
	XIO_DEV_RS485,		// USART	RS485 device
	XIO_DEV_SPI1,		// SPI		SPI channel #1
	XIO_DEV_SPI2,		// SPI		SPI channel #2
//	XIO_DEV_SPI3,		// SPI		SPI channel #3
//	XIO_DEV_SPI4,		// SPI		SPI channel #4
//	XIO_DEV_SPI5,		// SPI		SPI channel #5
//	XIO_DEV_SPI6,		// SPI		SPI channel #6
	XIO_DEV_PGM,		// FILE		Program memory file  (read only)
//	XIO_DEV_SD,			// FILE		SD card (not implemented)
	XIO_DEV_COUNT		// total device count (must be last entry)
};
// If your change these ^, check these v

#define XIO_DEV_USART_COUNT 	2 				// # of USART devices
#define XIO_DEV_USART_OFFSET	0				// offset for computing index into extended device struct array

#define XIO_DEV_SPI_COUNT 		2 				// # of SPI devices
#define XIO_DEV_SPI_OFFSET		XIO_DEV_USART_COUNT	// offset for computing index into extended device struct array

#define XIO_DEV_FILE_COUNT		1				// # of FILE devices
#define XIO_DEV_FILE_OFFSET		(XIO_DEV_USART_COUNT + XIO_DEV_SPI_COUNT) // index into FILES

// aliases for stdio devices (aka pointers, streams)
//#define fdev_rs485 (ds[XIO_DEV_RS485].fdev)	// RS485 device for stdio functions
//#define fdev_usb	(ds[XIO_DEV_USB].fdev)		// USB device for stdio functions
//#define fdev_spi1	(ds[XIO_DEV_SPI1].fdev)		// SPI channel #1
//#define fdev_spi2	(ds[XIO_DEV_SPI2].fdev)		// SPI channel #2
//#define fdev_pgm	(ds[XIO_DEV_PGM].fdev)		// Program memory device

/*************************************************************************
 *	Function Prototypes
 *************************************************************************/

#define PGMFILE (const PROGMEM char *)			// extends pgmspace.h
typedef void (*fptr_void_void) (void); 			// returns void, void args
typedef int (*fptr_int_void) (void); 			// returns int, void args

void xio_init(void);									// xio system general init
FILE *xio_open(const uint8_t dev, const char *addr);	// generic open function
int xio_cntl(const uint8_t dev, const uint32_t control);
//int xio_rctl(const uint8_t dev, uint32_t *control);
int xio_gets(const uint8_t dev, char *buf, const int size);
int xio_getc(const uint8_t dev);
int xio_putc(const uint8_t dev, const char c);

// generic device init (must be followed by device-specific init
void xio_init_dev(uint8_t dev,									// device number
	FILE *(*x_open)(const uint8_t dev, const char *addr), 		// device open routine
	int (*dev_cntl)(const uint8_t dev, const uint32_t control),	// set device control flags
//	int (*dev_rctl)(const uint8_t dev, uint32_t *control),		// get device control flags
	int (*dev_gets)(const uint8_t dev, char *buf, int size),	// specialized line reader
	int (*dev_getc)(FILE *),										// read char (stdio compatible)
	int (*dev_putc)(char, FILE *)								// write char (stdio compatible)
	);

// std devices
void xio_init_stdio(void);						// set std devs & do startup prompt
void xio_set_stdin(const uint8_t dev);
void xio_set_stdout(const uint8_t dev);
void xio_set_stderr(const uint8_t dev);

/*************************************************************************
 *	Device structures
 *************************************************************************/
// NOTE" "FILE *" is another way of saying "struct __file *"

struct xioDEVICE {							// common device struct (one per dev)
	uint8_t status;							// completion status 
	uint8_t signal;							// signal value
	char c;									// char temp
	uint8_t len;							// chars read so far (buf array index)
	int size;								// text buffer length (dynamic)
	uint32_t flags;							// common control flags
	FILE *(*x_open)(const uint8_t dev, const char *addr);		// device open routine
	int (*x_cntl)(const uint8_t dev, const uint32_t control);	// set device control flags
//	int (*x_rctl)(const uint8_t dev, uint32_t *control);		// get device control flags
	int (*x_gets)(const uint8_t dev, char *buf, const int size);// specialized line reader
	int (*x_getc)(FILE *);					// read char (stdio compatible)
	int (*x_putc)(char, FILE *);			// write char (stdio compatible)

	void *x;								// device-specific struct binding (static)
	FILE *fdev;								// stdio fdev binding (static)
	char *buf;								// text buffer binding (can be dynamic)
};

/*
 * Static structure allocations for XIO 
 * See xio.h for device struct definition
 * See xio_usart.h for usart device struct definition
 * See xio_file.h for file device struct definition
 * See xio_signal.h for signal flag struct definition
 */
struct xioDEVICE 	ds[XIO_DEV_COUNT];		// allocate top-level dev structs
struct __file 		ss[XIO_DEV_COUNT];		// ...stdio stream for each dev
struct xioUSART 	us[XIO_DEV_USART_COUNT];// ...USART extended IO structs
struct xioSPI 		sp[XIO_DEV_SPI_COUNT];	// ...SPI extended IO structs
struct xioFILE 		fs[XIO_DEV_FILE_COUNT];	// ...FILE extended IO structs
struct xioSIGNALS	sig;					// ...signal flags
extern struct controllerSingleton tg;		// needed by init() for default source

/*************************************************************************
 * SUPPORTING DEFINTIONS - SHOULD NOT NEED TO CHANGE
 *************************************************************************/
/*
 * xio control flag values
 */
// must cast 1 to uint32_t for bit evaluations to work correctly
#define XIO_BAUD_gm		0x0000000F				// baud rate enum mask (keep in LSdigit)
#define XIO_RD			((uint32_t)1<<4) 		// read enable bit
#define XIO_WR			((uint32_t)1<<5)		// write enable only
#define XIO_RDWR		(XIO_RD | XIO_WR) 		// read & write
#define XIO_BLOCK		((uint32_t)1<<6)		// enable blocking reads
#define XIO_NOBLOCK		((uint32_t)1<<7)		// disable blocking reads
#define XIO_XOFF 		((uint32_t)1<<8)		// enable XON/OFF flow control
#define XIO_NOXOFF 		((uint32_t)1<<9)		// disable XON/XOFF flow control
#define XIO_ECHO		((uint32_t)1<<10)		// echo reads from device to stdio
#define XIO_NOECHO		((uint32_t)1<<11)		// disable echo
#define XIO_CRLF		((uint32_t)1<<12)		// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		((uint32_t)1<<13)		// do not convert <LF> to <CR><LF> on writes
#define XIO_IGNORECR	((uint32_t)1<<14)		// ignore <CR> on reads
#define XIO_NOIGNORECR	((uint32_t)1<<15)		// don't ignore <CR> on reads
#define XIO_IGNORELF	((uint32_t)1<<16)		// ignore <LF> on reads
#define XIO_NOIGNORELF	((uint32_t)1<<17)		// don't ignore <LF> on reads
#define XIO_LINEMODE	((uint32_t)1<<18)		// special <CR><LF> read handling
#define XIO_NOLINEMODE	((uint32_t)1<<19)		// no special <CR><LF> read handling

// internal control flags (which are NOT the similar bits in the control word, above)
// static configuration states
#define XIO_FLAG_RD_bm		((uint32_t)1<<0)	// enabled for read
#define XIO_FLAG_WR_bm		((uint32_t)1<<1)	// enabled for write
#define XIO_FLAG_BLOCK_bm	((uint32_t)1<<2)	// enable blocking RD and WR
#define XIO_FLAG_XOFF_bm 	((uint32_t)1<<3)	// XOFF flow control enabled
#define XIO_FLAG_ECHO_bm 	((uint32_t)1<<4)	// echo received chars to stderr
#define XIO_FLAG_CRLF_bm 	((uint32_t)1<<5)	// convert <LF> to <CR><LF> on writes
#define XIO_FLAG_IGNORECR_bm ((uint32_t)1<<6)	// ignore <LF> on reads
#define XIO_FLAG_IGNORELF_bm ((uint32_t)1<<7)	// ignore <LF> on reads
#define XIO_FLAG_LINEMODE_bm ((uint32_t)1<<8)	// special handling for line-oriented text
// transient states
#define XIO_FLAG_EOL_bm		((uint32_t)1<<9)	// detected EOL (/n, /r, ;)
#define XIO_FLAG_EOF_bm 	((uint32_t)1<<10)	// detected EOF (NUL)
#define XIO_FLAG_IN_LINE_bm	((uint32_t)1<<11)	// partial line is in buffer
#define XIO_FLAG_RESET_gm	(0x01FF)			// used to clear the transient state bits

// Bit evaluations that return actual TRUE and FALSE
// Just using the (a & blahblah) returns FALSE and not_FALSE 
// ...but not actually TRUE (which = 1)
#define READ(a) 		((a & XIO_FLAG_RD_bm) ? true : false)
#define WRITE(a)	 	((a & XIO_FLAG_WR_bm) ? true : false)
#define BLOCKING(a) 	((a & XIO_FLAG_BLOCK_bm) ? true : false)
#define EN_XOFF(a)		((a & XIO_FLAG_XOFF_bm) ? true : false)
#define ECHO(a)		 	((a & XIO_FLAG_ECHO_bm) ? true : false)
#define CRLF(a) 		((a & XIO_FLAG_CRLF_bm) ? true : false)
#define IGNORECR(a) 	((a & XIO_FLAG_IGNORECR_bm) ? true : false)
#define IGNORELF(a) 	((a & XIO_FLAG_IGNORELF_bm) ? true : false)
#define LINEMODE(a)		((a & XIO_FLAG_LINEMODE_bm) ? true : false)
#define IN_LINE(a)		((a & XIO_FLAG_IN_LINE_bm) ? true : false)

/*
 * Generic XIO signals and error conditions. 
 * See signals.h for application specific signal defs and routines.
 */

enum xioSignals {
	XIO_SIG_OK,				// OK
	XIO_SIG_EAGAIN,			// would block
	XIO_SIG_EOL,			// end-of-line encountered (string has data)
	XIO_SIG_EOF,			// end-of-file encountered (string has no data)
	XIO_SIG_RESET,			// cancel operation immediately
	XIO_SIG_FEEDHOLD,		// pause operation
	XIO_SIG_CYCLE_START,	// start or resume operation
	XIO_SIG_DELETE,			// backspace or delete character (BS, DEL)
	XIO_SIG_BELL			// BELL character (BEL, ^g)
};

/* Some useful ASCII definitions */

#define NUL (char)0x00		//  ASCII NUL char (0) (not "NULL" which is a pointer)
#define ETX (char)0x03		// ^c - ETX
#define ENQ (char)0x05		// ^e - ENQuire status report
#define BEL (char)0x07		// ^g - BEL
#define BS  (char)0x08		// ^h - backspace 
#define TAB (char)0x09		// ^i - character
#define LF	(char)0x0A		// ^j - line feed
#define VT	(char)0x0B		// ^k - kill stop
#define CR	(char)0x0D		// ^m - carriage return
#define XON (char)0x11		// ^q - DC1, XON, resume
#define XOFF (char)0x13		// ^s - DC3, XOFF, pause
#define CAN (char)0x18		// ^x - Cancel, abort
#define ESC (char)0x1B		// ^[ - ESC(ape)
#define DEL (char)0x7F		//  DEL(ete)

/* Signal character mappings */

#define CHAR_RESET CAN
#define CHAR_FEEDHOLD (char)'!'
#define CHAR_CYCLE_START (char)'~'

/* XIO return codes
 * These codes are the "inner nest" for the TG_ return codes. 
 * The first N TG codes correspond directly to these codes.
 * This eases using XIO by itself (without tinyg) and simplifes using
 * tinyg codes with no mapping when used together. This comes at the cost 
 * of making sure these lists are aligned. TG_should be based on this list.
 */

enum xioCodes {
	XIO_OK = 0,				// OK - ALWAYS ZERO
	XIO_ERR,				// generic error return (errors start here)
	XIO_EAGAIN,				// function would block here (must be called again)
	XIO_NOOP,				// function had no-operation	
	XIO_COMPLETE,			// operation complete
	XIO_TERMINATE,			// operation terminated (gracefully)
	XIO_RESET,				// operation reset (ungraceful)
	XIO_EOL,				// function returned end-of-line
	XIO_EOF,				// function returned end-of-file 
	XIO_FILE_NOT_OPEN,		// file is not open
	XIO_FILE_SIZE_EXCEEDED, // maximum file size exceeded
	XIO_NO_SUCH_DEVICE,		// illegal or unavailable device
	XIO_BUFFER_EMPTY,		// more of a statement of fact than an error code
	XIO_BUFFER_FULL_FATAL,
	XIO_BUFFER_FULL_NON_FATAL,
	XIO_INITIALIZING		// system initializing, not ready for use
};
#define XIO_ERRNO_MAX XIO_BUFFER_FULL_NON_FATAL

/* ASCII characters used by Gcode or otherwise unavailable for special use. 
	See NIST sections 3.3.2.2, 3.3.2.3 and Appendix E for Gcode uses.
	See http://www.json.org/ for JSON notation

	hex		char	name		used by:
	----	----	----------	--------------------
	0x00	NUL	 	null		everything
	0x01	SOH		ctl-A
	0x02	STX		ctl-B
	0x03	ETX		ctl-C
	0x04	EOT		ctl-D
	0x05	ENQ		ctl-E		Status query
	0x06	ACK		ctl-F
	0x07	BEL		ctl-G
	0x08	BS		ctl-H
	0x09	HT		ctl-I
	0x0A	LF		ctl-J
	0x0B	VT		ctl-K
	0x0C	FF		ctl-L
	0x0D	CR		ctl-M
	0x0E	SO		ctl-N
	0x0F	SI		ctl-O
	0x10	DLE		ctl-P
	0x11	DC1		ctl-Q		XOFF
	0x12	DC2		ctl-R		feedhold
	0x13	DC3		ctl-S		XON
	0x14	DC4		ctl-T		end feedhold
	0x15	NAK		ctl-U
	0x16	SYN		ctl-V
	0x17	ETB		ctl-W
	0x18	CAN		ctl-X		software reset
	0x19	EM		ctl-Y
	0x1A	SUB		ctl-Z
	0x1B	ESC		ctl-[
	0x1C	FS		ctl-\
	0x1D	GS		ctl-]
	0x1E	RS		ctl-^
	0x1F	US		ctl-_

	0x20	<space>				Gcode blocks
	0x21	!		excl point	Kill, Terminate signals
	0x22	"		quote		JSON notation
	0x23	#		number		Gcode parameter prefix
	0x24	$		dollar		TinyG / grbl settings prefix
	0x25	&		ampersand	universal symbol for logical AND (not used here)
	0x26	%		percent		Reserved for in-cycle command escape character
	0x27	'		single quot	
	0x28	(		open paren	Gcode comments
	0x29	)		close paren	Gcode comments
	0x2A	*		asterisk	Gcode expressions
	0x2B	+		plus		Gcode numbers, parameters and expressions
	0x2C	,		comma		JSON notation
	0x2D	-		minus		Gcode numbers, parameters and expressions
	0x2E	.		period		Gcode numbers, parameters and expressions
	0x2F	/		fwd slash	Gcode expressions & block delete char
	0x3A	:		colon		JSON notation
	0x3B	;		semicolon
	0x3C	<		less than	Gcode expressions
	0x3D	=		equals		Gcode expressions
	0x3E	>		greaterthan	Gcode expressions
	0x3F	?		question mk	TinyG / grbl query prefix
	0x40	@		at symbol	TinyG feedhold

	0x5B	[		open bracketGcode expressions
	0x5C	\		backslash	JSON notation (escape)
	0x5D	]		close brack	Gcode expressions
	0x5E	^		caret
	0x5F	_		underscore

	0x60	`		grave accnt	
	0x7B	{					JSON notation
	0x7C	|					universal symbol for logical OR (not used here)
	0x7D	}					JSON notation
	0x7E	~		tilde		TinyG cycle start
	0x7F	DEL	
*/

#define __UNIT_TEST_XIO			// include and run xio unit tests
#ifdef __UNIT_TEST_XIO
void xio_unit_tests(void);
#define	XIO_UNITS xio_unit_tests();
#else
#define	XIO_UNITS
#endif // __UNIT_TEST_XIO

#endif
