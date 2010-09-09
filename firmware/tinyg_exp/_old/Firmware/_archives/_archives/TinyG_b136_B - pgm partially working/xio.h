/*
  xio.h - Xmega IO devices - common file
  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xio_h
#define xio_h

/* 
 * Function Prototypes and Aliases
 */

void xio_init(void);
char * fgets2(char *s, int size, FILE *stream);

/* 
 * Common Devices and Configurations
 */

#define RX_BUFFER_SIZE 18	// device rx buffer - written by ISRs (2 bytes unusable)
#define TX_BUFFER_SIZE 3	// device tx buffer - (not used)

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
#define XIO_FLAG_GOT_EOL_bm	(1<<14) // device received an EOL character
#define XIO_FLAG_FLOW_CONTROL_ON_bm (1<<15) // device is in flow control (now)

#define XIO_FLAG_DEFAULT_gm (XIO_FLAG_RD_bm | XIO_FLAG_WR_bm | XIO_FLAG_BLOCK_bm | XIO_ECHO | XIO_CRLF | XIO_LINEMODE)

#define READ(a) (a & XIO_FLAG_RD_bm)	// TRUE if read enabled
#define WRITE(a) (a & XIO_FLAG_WR_bm)	// TRUE if write enabled
#define BLOCKING(a) (a & XIO_FLAG_BLOCK_bm)	// etc.
#define ECHO(a) (a & XIO_FLAG_ECHO_bm)
#define CRLF(a) (a & XIO_FLAG_CRLF_bm)
#define LINEMODE(a) (a & XIO_FLAG_LINEMODE_bm)
#define SEMICOLONS(a) (a & XIO_FLAG_SEMICOLONS_bm)

/* 
 * USART control structure
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

#endif
