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
 * Xmega IO devices made compatible with avr-gcc stdio
 * To use this sub-system outside of TinyG you will need some defines in tinyg.h
 * See the end of this file for further documentation.
 */

#ifndef xio_h
#define xio_h

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/*
 * Global Scope Data and Functions
 */

void xio_init(void);				// initialize xio system
int xio_fget_ln(uint8_t dev, char *buf, uint8_t len);
int8_t xio_control(uint8_t dev, const uint16_t control, const int16_t arg);
int8_t xio_dev_init(uint8_t dev, const int16_t arg);

int xio_null_signal(uint8_t sig);	// no-op signal handler
int xio_null_line(char * buf);		// no-op line handler

FILE *stddev;						// a convenient alias for stdin. stdout, stderr

/*
 * XIO devices (configured devices)
 */

enum xioDevice {					// device enumerations
	XIO_DEV_NULL,					// null device
	XIO_DEV_USB,					// USB device
	XIO_DEV_RS485,					// RS485 device (typically network port)
	XIO_DEV_AUX,					// AUX device (typically Arduino)
	XIO_DEV_PGM,					// program memory file
	XIO_DEV_MAX						// *** must be last ***
};

/*
 * XIO signals and error conditions (loads f.signals register)
 */

enum xio_SIGS {
	XIO_SIG_OK,						// OK
	XIO_SIG_EOL,					// end-of-line encountered (string has data)
	XIO_SIG_EOF,					// end-of-file encountered (string has no data)
	XIO_SIG_WOULDBLOCK,				// would block - no character returned
	XIO_SIG_KILL,					// cancel operation immediately (^c, ESC)
	XIO_SIG_TERMINATE,				// terminate operation nicely (^x)
	XIO_SIG_PAUSE,					// pause operation (^q)
	XIO_SIG_RESUME,					// resume operation (^p)
	XIO_SIG_SHIFTOUT,				// shift to mode (^n)
	XIO_SIG_SHIFTIN,				// shift back (^o)
	XIO_SIG_DELETE,					// backspace or delete character (BS, DEL)
	XIO_SIG_BELL					// BELL character (BEL, ^g)
};

/*
 * Some useful ASCII definitions
 */

#define NUL 0x00				// ASCII NUL character (0) (not "NULL" which is a pointer)
#define ETX 0x03				// ^c - aka ETX
#define BEL 0x07				// ^g - aka BEL
#define BS  0x08				// ^h - aka backspace 
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

/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
 */

enum xio_BAUDRATES {         			// BSEL	  BSCALE
		XIO_BAUD_UNSPECIFIED,			//	0		0		// use default value 
		XIO_BAUD_9600,					//	207		0
		XIO_BAUD_19200,					//	103		0
		XIO_BAUD_38400,					//	51		0
		XIO_BAUD_57600,					//	34		0
		XIO_BAUD_115200,				//	33		(-1<<4)
		XIO_BAUD_230400,				//	31		(-2<<4)
		XIO_BAUD_460800,				//	27		(-3<<4)
		XIO_BAUD_921600,				//	19		(-4<<4)
		XIO_BAUD_500000,				//	1		(1<<4)
		XIO_BAUD_1000000				//	1		0
};		// Note: cannot have more than 16 without changing XIO_BAUD_gm, below

#define	XIO_BAUD_DEFAULT XIO_BAUD_115200


// _init() io_ctl() control bits
#define XIO_BAUD_gm		0x0000000F		// baud rate enum mask (keep in LSbyte)

#define XIO_SIG_FUNC	(1<<4)			// signal handler function (see note 1)
#define XIO_LINE_FUNC	(1<<5)			// line handler function (see note 1)
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
#define XIO_NOSEMICOLONS (1<<15)			// don't treat semicolons as line breaks

// (note 1) The handler function flags share positions 4 & 5 with RD and WR flags
//			RD and WR are only valid in init(), handlers only valid in control()

// f.flags flags (which are NOT the similar bits in the control word, above)
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
 * USART control structure - here becuase it's shared by multiple devices.
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable (see xmega_io.c circular buffer note) 
 */

#define RX_BUFFER_SIZE 18	// device rx buffer - written by ISRs (2 bytes unusable)
#define TX_BUFFER_SIZE 18	// device tx buffer - read by ISRs (2 bytes unusable)
//#define TX_BUFFER_SIZE 255 // device tx buffer MAXUIMUM SIZE
//#define CHAR_BUFFER_SIZE 80 // size of read line buffer (one based)

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


/**** NOTES ON XIO ****

---- control characters and signals ----

  The underlying getc() and readln() routines trap some control characters 
  and treat them as signals.

  In the case of readln they are passed to the signal handler a callback.registered.
  See xio_usb_readln() for an example.
  
  In the case of getc they are passed via udata to the calling stdio routine.
  Here's an example of use:

	The device-level struct must have a "uint8_t signal" variable
	Bind the variable to the udata pointer int the stdio FILE struct as so:

	f.signal = 0;							// initialize signal (none)
	dev_usb.udata = &(f.signal);			// bind signals register to device FILE struct

	To use the signal via the stdio routines:

	if (fgets(tg.buf, BUF_LEN-1, tg.srcin) == NULL) {	
		// get the signal from the underlying device struct (do this once. Ugh)
		tg.signal = *(uint8_t *)tg.srcin->udata;


  Details: A control character is trapped by the stdin get_char() routine.
  get_char() sets a flag in xio_signals and returns an error. The flag can be gotten
  to via the pointer set in __file.udata. Control characters are not echoed at the 
  get_char() level, but they may be by top_parser(); depends on what makes sense.
  
  top_parser() exhibits the following control code behaviors:

   ^c,^x, ESC	Abort current action
  				Sends a "STOP" to the currently active mode
				Does not echo control character
				Exits the current mode (reverts to control mode)
				Echos "STOP"

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

   ^q			Pause
   				Pause reading input until ^s received
				No echo

   ^s			Resume
   				Resume reading input
				No echo


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
*/

#endif
