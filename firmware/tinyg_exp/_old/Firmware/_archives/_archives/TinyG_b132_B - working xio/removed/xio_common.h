/*
  xio_common.h - common defines for xmega device IO
  Copyright (c) 2010 Alden S. Hart, Jr.

  Use the following line to declare and set up the device:

FILE usb_str = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);

  Then call the init sometime before use:

	xio_usb_init(XIO_BAUD_115200);		// or whatever controls you want

*/

#ifndef xio_usb_h
#define xio_usb_h

/* 
 * Function prototypes and aliases
 */

void xio_usb_init(uint32_t control);
int8_t xio_usb_control(uint32_t control, int16_t arg);
int xio_usb_putc(char c, FILE *stream);
int xio_usb_getc(FILE *stream);
//int xio_usb_ungetc(FILE *stream);	// not implemented


/* 
 * Device Configuration 
 */

#define USB_USART USARTC0			// FTDI USB chip is wired to USARTC0 on the board
#define USB_RX_ISR_vect USARTC0_RXC_vect	// RX ISR
#define USB_TX_ISR_vect USARTC0_TXC_vect	// TX ISR

#define USB_PORT PORTC				// port where the USART is located
#define USB_RX_bm (1<<2)			// RX pin	- these pins are wired on the board
#define USB_TX_bm (1<<3)			// TX pin
#define USB_RTS_bm (1<<1)			// RTS pin
#define USB_CTS_bm (1<<0)			// CTS pin

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
#define XIO_ECHO		(1<<10)		// echo reads from device to stdio
#define XIO_NOECHO		(1<<11)		// disable echo
#define XIO_BLOCK		(1<<12)		// enable blocking reads
#define XIO_NOBLOCK		(1<<13)		// disable blocking reads
#define XIO_CRLF		(1<<14)		// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		(1<<14)		// do not convert <LF> to <CR><LF> on writes


// fd.flags flags (which are NOT the similar bits in the control word, above)
#define XIO_FLAG_RD_bm		(1<<0)	// enabled for read
#define XIO_FLAG_WR_bm		(1<<1)	// enabled for write
#define XIO_FLAG_ECHO_bm 	(1<<2)	// echo read chars to stdio
#define XIO_FLAG_CRLF_bm 	(1<<3)	// convert <LF> to <CR><LF> on writes
#define XIO_FLAG_BLOCK_bm	(1<<4)	// enable blocking reads
#define XIO_FLAG_FLOW_CONTROL_ENABLE_bm (<<5)	// enable flow control for device
#define XIO_FLAG_FLOW_CONTROL_ON_bm (1<<7) 		// device is in flow control (now)

#define XIO_FLAG_DEFAULT_gm (XIO_FLAG_RD_bm | XIO_FLAG_WR_bm | XIO_FLAG_BLOCK_bm | XIO_FLAG_ECHO_bm)

#define READ_ENABLED(a) (a & XIO_FLAG_RD_bm)		// TRUE if read enabled
#define WRITE_ENABLED(a) (a & XIO_FLAG_WR_bm)		// TRUE if write enabled
#define BLOCKING_ENABLED(a) (a & XIO_FLAG_BLOCK_bm)	// TRUE if read blocking enab
#define ECHO_ENABLED(a) (a & XIO_FLAG_ECHO_bm)		// TRUE if echo mode enabled
#define CRLF_ENABLED(a) (a & XIO_FLAG_CRLF_bm)		// TRUE if CRLF mode enabled

/* 
 * USART control structure
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable (see xmega_io.c circular buffer note) 
 */

struct xioUSART {					// device control struct for USARTS
	// variables and queues
	uint_fast8_t flags;				// control flags
	uint_fast8_t baud;				// baud rate index
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
