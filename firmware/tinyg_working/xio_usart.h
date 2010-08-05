/*
 * xio_usart.h - Common USART definitions 
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
 */

#ifndef xio_usart_h
#define xio_usart_h

/* USART DEVICE HARDWARE CONFIGS */

// RS485 device
#define RS485_INIT_bm (XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_BAUD_115200)

#define RS485_USART USARTC1					// RS485 usart
#define RS485_RX_ISR_vect USARTC1_RXC_vect 	// (RX) reception complete IRQ
#define RS485_TX_ISR_vect USARTC1_DRE_vect	// (TX) data register empty IRQ

#define RS485_PORT PORTC					// port where USART is located
#define RS485_RE_bm (1<<4)					// RE (Receive Enable) pin - active lo
#define RS485_DE_bm (1<<5)					// DE (Data Enable)(TX) - active hi
#define RS485_RX_bm (1<<6)					// RX pin
#define RS485_TX_bm (1<<7)					// TX pin

#define RS485_DIRCLR_bm (RS485_RX_bm)							// input bits
#define RS485_DIRSET_bm (RS485_RE_bm | RS485_DE_bm | RS485_TX_bm)// output bits
#define RS485_OUTCLR_bm (RS485_RE_bm | RS485_DE_bm)				// outputs init'd to 0
#define RS485_OUTSET_bm (RS485_TX_bm)							// outputs init'd to 1


// USB device
#define USB_INIT_bm (XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200)

#define USB_USART USARTC0					// USB usart
#define USB_RX_ISR_vect USARTC0_RXC_vect 	// (RX) reception complete IRQ
#define USB_TX_ISR_vect USARTC0_DRE_vect	// (TX) data register empty IRQ

#define USB_PORT PORTC						// port where the USART is located
#define USB_CTS_bm (1<<0)					// CTS pin (pins are wired on the board)
#define USB_RTS_bm (1<<1)					// RTS pin
#define USB_RX_bm (1<<2)					// RX pin
#define USB_TX_bm (1<<3)					// TX pin

#define USB_DIRCLR_bm (USB_CTS_bm | USB_RX_bm)	// input bits
#define USB_DIRSET_bm (USB_RTS_bm | USB_TX_bm)	// output bits
#define USB_OUTCLR_bm (0)						// outputs init'd to 0
#define USB_OUTSET_bm (USB_RTS_bm | USB_TX_bm)	// outputs init'd to 1


// TTL device (Arduino)
#define TTL_INIT_bm (XIO_RDWR | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_SEMICOLONS | XIO_BAUD_115200)

#define TTL_USART USARTC0					// Arduino usart
#define TTL_PORT PORTC						// port where the USART is located
#define TTL_CTS_bm (1<<0)					// CTS pin
#define TTL_RTS_bm (1<<1)					// RTS pin
#define TTL_RX_bm (1<<2)					// RX pin
#define TTL_TX_bm (1<<3)					// TX pin

#define TTL_DIRCLR_bm (USB_RX_bm)
#define TTL_DIRSET_bm (USB_TX_bm)
#define TTL_OUTCLR_bm (0)
#define TTL_OUTSET_bm (USB_TX_bm)


/* 
 * USART DEVICE CONSTANTS AND PARAMETERS
 */

#define RX_BUFFER_SIZE 18		// USART ISR RX buffer size (255 max)
#define TX_BUFFER_SIZE 18		// USART ISR TX buffer size (255 max)

/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
 */

// for turning USART interrupts on and off
#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_LO_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)

// Baud rate configuration
enum xioBAUDRATES {         			// BSEL	  BSCALE
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

/*
 * xio_control values
 */

// _init() & _control() control bits
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

// (note 1) The handler function flags share positions 4 & 5 with RD and WR flags
//			RD and WR are only valid in init(), handlers only valid in control()

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
 * USART extended control structure 
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable
 */

struct xioUSART {
	volatile uint_fast8_t rx_buf_tail;	// RX buffer read index
	volatile uint_fast8_t rx_buf_head;	// RX buffer write index (written by ISR)
	volatile uint_fast8_t tx_buf_tail;	// TX buffer read index (written by ISR)
	volatile uint_fast8_t tx_buf_head;	// TX buffer write index
	uint_fast8_t next_tx_buf_head;		// next TX buffer write index

	struct USART_struct *usart;			// USART structure
	struct PORT_struct *port;			// corresponding port

	volatile unsigned char rx_buf[RX_BUFFER_SIZE];  // (written by ISR)
	volatile unsigned char tx_buf[TX_BUFFER_SIZE];
};

/* 
 * USART DEVICE FUNCTION PROTOTYPES
 */


// functions common to all USART devices
void xio_init_usart(const uint8_t dev, 
					const uint16_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset);

void xio_set_baud_usart(const uint8_t dev, const uint8_t baud);

// RS485 functions
int xio_putc_rs485(const char c, FILE *stream);
int xio_getc_rs485(FILE *stream);
int xio_readln_rs485();						// non-blocking read line function
void xio_rs485_queue_RX_char(char c);		// simulate char rcvd into RX buffer
void xio_rs485_queue_RX_string(char *buf);	// simulate receving a whole string


// USB functions
int xio_putc_usb(const char c, FILE *stream);
int xio_getc_usb(FILE *stream);
int xio_readln_usb();						// non-blocking read line function
void xio_usb_queue_RX_char(char c);			// simulate char rcvd into RX buffer
void xio_usb_queue_RX_string(char *buf);	// simulate receving a whole string


// TTL usart functions (Arduino)

#endif
