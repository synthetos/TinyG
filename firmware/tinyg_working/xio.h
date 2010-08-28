/*
 * xio.h - Xmega IO devices - common file
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * -----
 * XIO devices are compatible with avr-gcc stdio, so formatted printing is supported
 * To use this sub-system outside of TinyG you will need some defines in tinyg.h
 * See the end of this file for further documentation.
 */

#ifndef xio_h
#define xio_h

#include <avr/pgmspace.h>		// defines prog_char

// include all xio subsystem header files here.
#include "xio_file.h"
#include "xio_usart.h"


/******************************************************************************
 *
 *	Definitions
 *
 ******************************************************************************/

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define PGMFILE (const PROGMEM char *)	// extends pgmspace.h

/* XIO return codes
 * These codes are the "inner nest" for the TG_ return codes. 
 * The first N TG codes correspond directly to these codes.
 * This eases using XIO by itself (without tinyg) and simplifes using
 * tinyg codes with no mapping when used together. This comes at the cost of 
 * making sure these lists are aligned. TG_should be based on this list.
 */

#define XIO_OK 0				// OK - ALWAYS ZERO
#define XIO_ERR 1				// generic error return (errors start here)
#define XIO_EAGAIN 2			// function would block here (must be called again)
#define XIO_NOOP 3				// function had no-operation	
#define XIO_EOL 4				// function returned end-of-line
#define XIO_EOF 5				// function returned end-of-file 
#define XIO_FILE_NOT_OPEN 6		// file is not open
#define XIO_NO_SUCH_DEVICE 7	// illegal or unavailable device
#define XIO_BUFFER_EMPTY 8		// more of a statement of fact than an error code
#define XIO_BUFFER_FULL_FATAL 9
#define XIO_BUFFER_FULL_NON_FATAL 10
#define XIO_ERRNO_MAX XIO_BUFFER_FULL_NON_FATAL

/*
 * Common typedefs
 */										// pointers to functions:

typedef void (*fptr_void_void) (void); 	// returns void, void args
typedef int (*fptr_int_void) (void); 	// returns int, void args


/******************************************************************************
 *
 *	Device configurations
 *
 ******************************************************************************/

/* Known XIO devices (configured devices) */

enum xioDevice {				// device enumerations
								// TYPE:	DEVICE:
	XIO_DEV_RS485,				// USART	RS485 device (typ. network port)
	XIO_DEV_USB,				// USART	USB device
	XIO_DEV_TTL,				// USART	TTL device (typ. Arduino)
	XIO_DEV_PGM,				// FILE		program memory file (read only)
	XIO_DEV_EEP,				// FILE		EEPROM (not implemented)
	XIO_DEV_SDC,				// FILE		SD card (not implemented)
	XIO_DEV_ENC,				// HW		Encoder port
	XIO_DEV_LIM,				// HW		Limit switch port
	XIO_DEV_COUNT				// total device count (must be last entry)
};

// If your change these ^, check these v

#define XIO_DEV_RS485_OFFSET XIO_DEV_RS485			// index into USARTS 
#define XIO_DEV_USB_OFFSET XIO_DEV_USB	
#define XIO_DEV_TTL_OFFSET XIO_DEV_TTL
#define XIO_DEV_USART_COUNT (3) 					// count of USART devices

#define XIO_DEV_PGM_OFFSET (XIO_DEV_PGM - XIO_DEV_PGM)// index into FILES
#define XIO_DEV_EEP_OFFSET (XIO_DEV_EEP - XIO_DEV_PGM)
#define XIO_DEV_SDC_OFFSET (XIO_DEV_SDC - XIO_DEV_PGM)
#define XIO_DEV_FILE_COUNT (3)						// count of FILE devices

// aliases for stdio devices (pointers)
#define fdev_rs485 (ds[XIO_DEV_RS485].fdev)// RS485 device for stdio functions
#define fdev_usb (ds[XIO_DEV_USB].fdev)	// USB device for stdio functions
#define fdev_pgm (ds[XIO_DEV_PGM].fdev)	// Program memory device

/*
 * Device configurations (all of them)
 */

/* Device configurations */
// USART devices: See xio_usart.h for USART-based device configs
// FILE devices:  See xio_file/h for FILE-based device configs

/*
 * xio control flag values
 */

#define XIO_BAUD_gm		0x0000000F		// baud rate enum mask (keep in LSbyte)
#define XIO_RD			(1<<4) 			// read enable bit
#define XIO_WR			(1<<5)			// write enable only
#define XIO_RDWR		(XIO_RD | XIO_WR) // read & write
#define XIO_BLOCK		(1<<6)			// enable blocking reads
#define XIO_NOBLOCK		(1<<7)			// disable blocking reads
#define XIO_ECHO		(1<<8)			// echo reads from device to stdio
#define XIO_NOECHO		(1<<9)			// disable echo
#define XIO_CRLF		(1<<10)			// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		(1<<11)			// do not convert <LF> to <CR><LF> on writes
#define XIO_LINEMODE	(1<<12)			// special <cr><lf> read handling
#define XIO_NOLINEMODE	(1<<13)			// no special <cr><lf> read handling
#define XIO_SEMICOLONS	(1<<14)			// treat semicolons as line breaks
#define XIO_NOSEMICOLONS (1<<15)		// don't treat semicolons as line breaks

// internal control flags (which are NOT the similar bits in the control word, above)
// static configuration states
#define XIO_FLAG_RD_bm		(1<<0)		// enabled for read
#define XIO_FLAG_WR_bm		(1<<1)		// enabled for write
#define XIO_FLAG_BLOCK_bm	(1<<2)		// enable blocking reads and writes
#define XIO_FLAG_FLOW_CONTROL_bm (1<<3)	// enable flow control for device
#define XIO_FLAG_ECHO_bm 	(1<<4)		// echo received chars to stderr output
#define XIO_FLAG_CRLF_bm 	(1<<5)		// convert <LF> to <CR><LF> on writes
#define XIO_FLAG_LINEMODE_bm (1<<6)		// special handling for line-oriented text
#define XIO_FLAG_SEMICOLONS_bm (1<<7)	// treat semicolons as line breaks (Arduino)
// transient control states
#define XIO_FLAG_TX_MUTEX_bm (1<<11)	// TX dequeue mutual exclusion flag
#define XIO_FLAG_EOL_bm		(1<<12)		// detected EOL (/n, /r, ;)
#define XIO_FLAG_EOF_bm 	(1<<13)		// detected EOF (NUL)
#define XIO_FLAG_IN_LINE_bm	(1<<14) 	// partial line is in buffer
#define XIO_FLAG_IN_FLOW_CONTROL_bm (1<<15) // device is in flow control

#define XIO_FLAG_RESET_gm	(0x0FFF)	// used to clear the top bits

#define READ(a) (a & XIO_FLAG_RD_bm)	// TRUE if read enabled
#define WRITE(a) (a & XIO_FLAG_WR_bm)	// TRUE if write enabled
#define BLOCKING(a) (a & XIO_FLAG_BLOCK_bm)	// etc.
#define ECHO(a) (a & XIO_FLAG_ECHO_bm)
#define CRLF(a) (a & XIO_FLAG_CRLF_bm)
#define LINEMODE(a) (a & XIO_FLAG_LINEMODE_bm)
#define SEMICOLONS(a) (a & XIO_FLAG_SEMICOLONS_bm)
#define TX_MUTEX(a) (a & XIO_FLAG_TX_MUTEX_bm)
#define IN_LINE(a) (a & XIO_FLAG_IN_LINE_bm)
#define IN_FLOW_CONTROL(a) (a & XIO_FLAG_IN_FLOW_CONTROL_bm)

/*
 * Generic XIO signals and error conditions. 
 * See signals.h for application specific signal defs and routines.
 */

enum xioSignals {
	XIO_SIG_OK,						// OK
	XIO_SIG_EAGAIN,					// would block
	XIO_SIG_EOL,					// end-of-line encountered (string has data)
	XIO_SIG_EOF,					// end-of-file encountered (string has no data)
	XIO_SIG_KILL,					// cancel operation immediately (^c, ETX, 0x04)
	XIO_SIG_TERMINATE,				// cancel operation nicely (^x, CAN, 0x24)
	XIO_SIG_PAUSE,					// pause operation (^s, XOFF, DC3, 0x13)
	XIO_SIG_RESUME,					// resume operation (^q, XON, DC1, 0x11)
//	XIO_SIG_SHIFTOUT,				// shift to mode (^n) (NOT IMPLEMENTED)
//	XIO_SIG_SHIFTIN,				// shift back (^o) (NOT IMPLEMENTED)
	XIO_SIG_ESCAPE,					// ESC. Typically mapped to ^c or ^x functions
	XIO_SIG_DELETE,					// backspace or delete character (BS, DEL)
	XIO_SIG_BELL					// BELL character (BEL, ^g)
};


/* Some useful ASCII definitions */

#define NUL 0x00				// ASCII NUL character (0) (not "NULL" which is a pointer)
#define ETX 0x03				// ^c - aka ETX
#define KILL ETX				// 		synonym
#define BEL 0x07				// ^g - aka BEL
#define BS  0x08				// ^h - aka backspace 
#define LF	0x0A				//  line feed
#define CR	0x0D				//  carriage return
#define SHIFTOUT 0x0E			// ^n - aka shift out 
#define SHITFTIN 0x0F			// ^o - aka shift in
#define XOFF 0x11				// ^q - aka DC1, XOFF, pause
#define XON 0x12				// ^s - aka DC3, XON, resume
#define ESC 0x1B				// ESC(ape)
#define DEL 0x7F				// DEL(ete)

#define CTRL_C ETX
#define CTRL_G BEL
#define CTRL_H BS	
#define CTRL_N SHIFTOUT	
#define CTRL_O SHIFTIN	
#define CTRL_Q XOFF
#define CTRL_S XON
#define CTRL_X 0x18				// ^x - aka CAN(cel)


/******************************************************************************
 *
 *	Device structures
 *
 ******************************************************************************/

struct xioDEVICE {						// common device struct (one per device)
	uint16_t flags;						// common control flags
	uint8_t status;						// completion status 
	uint8_t sig;						// signal value
	uint8_t c;							// char temp
	uint8_t len;						// chars read so far (buf array index)
	uint8_t size;						// test buffer length (dynamic)
	FILE *(*x_open)(const prog_char *addr);// device open routine
	int (*x_setflags)(const uint16_t control);// set device control flags
	int (*x_putc)(char, struct __file *);	// write char (stdio compatible)
	int (*x_getc)(struct __file *);		// read char (stdio compatible)
	int (*x_readln)(char *buf, const uint8_t size);// specialized line reader

	void *x;							// extended IO parameter binding (static)
	FILE *fdev;							// stdio fdev binding (static)
	char *buf;							// text buffer binding (dynamic)
};

// structure exports. See xio.c for defintions
extern struct xioDEVICE ds[XIO_DEV_COUNT];		// makes fdev_xxxx descriptors valid
extern struct xioUSART us[XIO_DEV_USART_COUNT];	// allocate USART extended IO structs
extern struct xioFILE fs[XIO_DEV_FILE_COUNT];	// ref FILE extended IO structs

// Notes: See sub-system .h's for extended IO structs (e.g. xio_usart.h)

/******************************************************************************
 *
 *	Function Prototypes
 *
 ******************************************************************************/

void xio_init(void);							// xio system general init
void xio_init_stdio(void);						// set std devs & do startup prompt
void xio_init_rs485(void);						// device-specific inits
void xio_init_usb(void);

int xio_setflags(const uint8_t dev, const uint16_t control);
void xio_set_stdin(const uint8_t dev);
void xio_set_stdout(const uint8_t dev);
void xio_set_stderr(const uint8_t dev);

int xio_getc(const uint8_t dev);
int xio_putc(const uint8_t dev, const char c);
int xio_readln(const uint8_t dev, char *buf, const uint8_t size);

void xio_init_dev(uint8_t dev,					// device number
	FILE *(*dev_open)(const prog_char *addr),	// device open routine
	int (*dev_setflags)(const uint16_t control),// set device control flags
	int (*dev_putc)(char, struct __file *),		// write char (stdio compatible)
	int (*dev_getc)(struct __file *),			// read char (stdio compatible)
	int (*dev_readln)(char *buf, uint8_t size)	// specialized line reader
	); 

void xio_signal_etx(void);			// ^c signal handler



/**** NOTES ON XIO ****/

/*---- Notes on the circular buffers ----

  An attempt has beeen made to make the circular buffers used by low-level 
  putc/getc as efficient as possible. This enables high-speed serial IO 
  operating between 100K and 1Mbaud.

  The circular buffers are unsigned char arrays that fill down from the top 
  element and wrap back to the top when index zero is reached. This allows 
  pre-decrement operations, zero tests, and eliminates modulus, masks, 
  substractions and other less efficient array bounds checking. Buffer indexes 
  are all unint_fast8_t which limits these buffers to 254 usable locations. 
  (one location is lost to head/tail collision detection and one is lost to 
  the zero position) All this enables the compiler to do better optimization.

  Chars are written to the *head* and read from the *tail*. 

  The head is left "pointing to" the character that was previously written - 
  meaning that on write the head is pre-decremented (and wrapped, if necessary), 
  then the new character is written.

  The tail is left "pointing to" the character that was previouly read - 
  meaning that on read the tail is pre-decremented (and wrapped, if necessary),
  then the new character is read.

  The head is only allowed to equal the tail if there are no characters to read.

  On read: If the head = the tail there is nothing to read, so the function either 
  exits with TG_EAGAIN or blocks (depending on the blocking mode selected).

  On write: If the head pre-deccrement causes the head to equal the tail the 
  buffer is full. The head is left at its original value and the device should 
  go into flow control (and the byte in the USART device is not read, and 
  therefore remains in the USART (VERIFY THAT I DIDN'T BREAK THIS BEHAVIOR!)). 
  Reading a character from a buffer that is in flow control should clear 
  flow control.

  (Note: More sophisticated flow control would detect the full condition earlier, 
   say at a high water mark of 95% full, and may go out of flow control at some 
   low water mark like 33% full).
*/
/*---- Notes on control characters and signals ----

  The underlying USART RX ISRs (used by getc() and readln()) trap control 
  characters and treat them as signals. 
  
  On receipt of a signal the signal value (see enum xioSignals) is written to
  xioDEVICE.sig and a signal handler specific to that signal is invoked 
  (see signals.c). The signal character is not written into the RX buffer.
  
  The signal handler runs at the ISR level, so it might be run, set some flag
  somewhere, or just return, relying on the application to detect the sig 
  value being set. It's up to the app to reset sig. If a new signal arrives 
  before the previous sig is handled or cleared the new sig will overwrite 
  the previous sig value.

  For now, the control chars and their mapping to signals are hard-coded into 
  the ISR for simplicity and speed. A more flexible system of bindings and 
  callbacks could be written at some sacrifice to execution speed. 

  IMPORTANT--> Since signals are trapped at the ISR level it is not necessary
  to be actively reading a device for it to receive signals. Any configured IO
  device will process signals. This allows input lines to come from one source 
  (e.g. a file device), while ^c, ^q, ^p are still active from another device 
  (e.g. the USB port being used as a console).
  
  Common signal definitions are:

   ^c,^x, ESC	Abort current action

   ^h, DEL		Delete previous character
  				Only affects top_parser() buffer
				Echoed to stdout if ECHO mode is on

   ^n			Shift out - change to another mode
   				Next received character determines mode to enter
				  'C'onfig mode
				  'G'code mode
				  'D'rive mode (Direct to motor commands)
				  'I'pa mode
				  'F'ile mode (switch stdin to file - requires an address token)

   ^o			Shift in - revert to control mode
   				Exit current mode but do not abort currently executing command

   ^q			Pause - Pause reading input until ^s received
   ^s			Resume - Resume reading input
*/

/*---- Notes on signal callbacks ----
  An earlier version of the code had signals implemented as callbacks. 
  I suppose you could find a pre 203 version, but here's how it worked.

The struct had sig_func and line_func callback addresses:

struct xioUSART {
	// PUBLIC VARIABLES - must be the same in every device type
	uint16_t flags;						// control flags
	uint8_t sig;						// signal or error value
	uint8_t c;							// line buffer character temp
	uint8_t i;							// line buffer pointer
	uint8_t len;						// line buffer maximum length (zero based)
	char *buf;							// pointer to input line buffer
	int (*sig_func)(uint8_t sig);		// pointer to signal handler function
	int (*line_func)(char * buf);		// pointer to line handler function

	// PRIVATE VARIABLES - in this case for USART. Can be different by device type
	volatile uint_fast8_t rx_buf_tail;	// RX buffer read index
	volatile uint_fast8_t rx_buf_head;	// RX buffer write index (written by ISR)
	volatile uint_fast8_t tx_buf_tail;	// TX buffer read index (written by ISR)
	volatile uint_fast8_t tx_buf_head;	// TX buffer write index
	uint_fast8_t next_tx_buf_head;		// next TX buffer write index
	volatile unsigned char rx_buf[RX_BUFFER_SIZE];  // (written by ISR)
	volatile unsigned char tx_buf[TX_BUFFER_SIZE];

	// hardware bindings
	struct USART_struct *usart;			// USART structure
	struct PORT_struct *port;			// corresponding port
};

Bindings occurred during init in xio_usb_init():

	// bind signal and line handlers to struct
	f.sig_func = &xio_null_signal;			// bind null signal handler
	f.line_func = &xio_null_line;			// bind null line handler

...and as controls in xio_usb_control():

	// commands with args - only do one flag if there's an arg
	if (control & XIO_SIG_FUNC) {
		f.sig_func = (fptr_int_uint8)arg;
		return (0);	
	}
	if (control & XIO_LINE_FUNC) {
		f.line_func = (fptr_int_char_p)arg;
		return (0);
	}

  Using these defines:

#define XIO_SIG_FUNC	(1<<4)			// signal handler function (see note 1)
#define XIO_LINE_FUNC	(1<<5)			// line handler function (see note 1)

Applications may call the control functions to bind signal handlers:

	xio_control(XIO_DEV_USB, XIO_SIG_FUNC, (int)&tg_signal); // bind sig handler
	xio_control(XIO_DEV_RS485, XIO_SIG_FUNC, (int)&tg_signal);
	xio_control(XIO_DEV_AUX, XIO_SIG_FUNC, (int)&tg_signal);

*/

#endif
