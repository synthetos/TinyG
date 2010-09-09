/*
  xio.h - Xmega IO devices - common file
  Copyright (c) 2010 Alden S. Hart, Jr.
  This is a bit unorthodox, but see the end of this file for some explanations
*/

#ifndef xio_h
#define xio_h

/* 
 * Function Prototypes and Aliases
 */

void xio_init(void);

typedef char (*fptr_char) (void); 	// declare fptr_char to be a pointer to a function that takes on arguments and returns a char
typedef void (*fptr_void) (void);  	// declare fptr_void to be a pointer to a function that takes on arguments and returns void

/* 
 * Common Devices and Configurations
 */

#define RX_BUFFER_SIZE 18	// device rx buffer - written by ISRs (2 bytes unusable)
#define TX_BUFFER_SIZE 3	// device tx buffer - (not used)

/*
 * Some ASCII definitions we need
 */

#define NUL 0x00				// ASCII NUL character (0) (not "NULL" which is a pointer)
#define CTRL_C 0x03				// ^c - aka ETX
#define CTRL_G 0x07				// ^g - aka BEL
#define CTRL_H 0x08				// ^h - aka backspace 
#define CTRL_N 0x0E				// ^n - aka shift out 
#define CTRL_O 0x0F				// ^o - aka shift in
#define CTRL_Q 0x11				// ^q - aka DC1, XOFF 
#define CTRL_S 0x13				// ^s - aka DC3, XON
#define CTRL_X 0x18				// ^x - aka CAN(cel)
#define ESC 0x1B				// ESC(ape)
#define DEL 0x7F				// DEL(ete)

#define BEL CTRL_G				// BEL
#define BS CTRL_H				// BackSpace

// how the above map into the signals register
#define NUL_bm		(1<<0)
#define CTRL_C_bm	(1<<1)
#define CTRL_G_bm	(1<<2)
#define CTRL_H_bm	(1<<3)
#define CTRL_N_bm	(1<<4)
#define CTRL_O_bm	(1<<5)
#define CTRL_Q_bm	(1<<6)
#define CTRL_S_bm	(1<<7)
#define CTRL_X_bm	(1<<8)
#define ESC_bm		(1<<9)
#define DEL_bm		(1<<10)

/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
 */
                             		// BSEL	  BSCALE
#define	XIO_BAUD_UNSPECIFIED 0		//	0		0		// use default value 
#define XIO_BAUD_9600 1				//	207		0
#define XIO_BAUD_19200 2			//	103		0
#define XIO_BAUD_38400 3			//	51		0
#define XIO_BAUD_57600 4			//	34		0
#define XIO_BAUD_115200 5			//	33		(-1<<4)
#define XIO_BAUD_230400 6			//	31		(-2<<4)
#define XIO_BAUD_460800 7			//	27		(-3<<4)
#define XIO_BAUD_921600 8			//	19		(-4<<4)
#define XIO_BAUD_500000 9			//	1		(1<<4)
#define XIO_BAUD_1000000 10			//	1		0
#define	XIO_BAUD_DEFAULT XIO_BAUD_115200

// _init() io_ctl() control bits
#define XIO_BAUD_gm		0x0000000F	// baud rate enumeration mask (keep in LSbyte)
#define XIO_RD			(1<<8) 		// read enable bit
#define XIO_WR			(1<<9)		// write enable only
#define XIO_RDWR		(XIO_RD | XIO_WR) // read & write
#define XIO_BLOCK		(1<<10)		// enable blocking reads
#define XIO_NOBLOCK		(1<<11)		// disable blocking reads
#define XIO_ECHO		(1<<12)		// echo reads from device to stdio
#define XIO_NOECHO		(1<<13)		// disable echo
#define XIO_CRLF		(1<<14)		// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		(1<<15)		// do not convert <LF> to <CR><LF> on writes
#define XIO_LINEMODE	(1<<4)		// special <cr><lf> read handling
#define XIO_NOLINEMODE	(1<<5)		// no special <cr><lf> read handling
#define XIO_SEMICOLONS	(1<<6)		// treat semicolons as line breaks
#define XIO_NOSEMICOLONS (1<<7)		// don't treat semicolons as line breaks

// f.flags flags (which are NOT the similar bits in the control word, above)
#define XIO_FLAG_RD_bm		(1<<0)	// enabled for read
#define XIO_FLAG_WR_bm		(1<<1)	// enabled for write
#define XIO_FLAG_BLOCK_bm	(1<<2)	// enable blocking reads
#define XIO_FLAG_FLOW_CONTROL_bm (1<<3)	// enable flow control for device
#define XIO_FLAG_ECHO_bm 	(1<<4)	// echo read chars to stdio
#define XIO_FLAG_CRLF_bm 	(1<<5)	// convert <LF> to <CR><LF> on writes
#define XIO_FLAG_LINEMODE_bm (1<<6)	// special <cr><lf> read handling
#define XIO_FLAG_SEMICOLONS_bm (1<<7) // treat semicolons as line breaks
#define XIO_FLAG_EOL_bm		(1<<13) // detected EOL (/n, /r, ;)
#define XIO_FLAG_EOF_bm 	(1<<14)	// detected EOF (NUL)
#define XIO_FLAG_IN_FLOW_CONTROL_bm (1<<15) // device is in flow control (now)

#define XIO_FLAG_RESET_gm	(0x0FFF)	// used to clear the top bits
#define XIO_FLAG_USB_DEFS_gm (XIO_FLAG_RD_bm | XIO_FLAG_WR_bm | XIO_FLAG_BLOCK_bm | XIO_FLAG_ECHO_bm)
#define XIO_FLAG_PGM_DEFS_gm (XIO_FLAG_RD_bm | XIO_FLAG_WR_bm | XIO_FLAG_BLOCK_bm | XIO_FLAG_ECHO_bm)

#define READ(a) (a & XIO_FLAG_RD_bm)	// TRUE if read enabled
#define WRITE(a) (a & XIO_FLAG_WR_bm)	// TRUE if write enabled
#define BLOCKING(a) (a & XIO_FLAG_BLOCK_bm)	// etc.
#define ECHO(a) (a & XIO_FLAG_ECHO_bm)
#define CRLF(a) (a & XIO_FLAG_CRLF_bm)
#define LINEMODE(a) (a & XIO_FLAG_LINEMODE_bm)
#define SEMICOLONS(a) (a & XIO_FLAG_SEMICOLONS_bm)

/* 
 * USART control structure - here becuase it's shared by multiple devices.
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable (see xmega_io.c circular buffer note) 
 */

struct xioUSART {					// device control struct for USARTS
	// variables and queues
	uint16_t flags;					// control flags
	uint_fast8_t rx_buf_tail;		// RX buffer read index
	volatile uint8_t rx_buf_head;	// RX buffer write index (written by ISR)
	volatile uint8_t tx_buf_tail;	// TX buffer read index (written by ISR)
	uint_fast8_t tx_buf_head;		// TX buffer write index
	volatile unsigned char rx_buf[RX_BUFFER_SIZE];  // (written by ISR)
	volatile unsigned char tx_buf[TX_BUFFER_SIZE];

	// hardware bindings
	struct USART_struct *usart;		// USART structure
	struct PORT_struct *port;		// corresponding port
};

/* FURTHER NOTES

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
