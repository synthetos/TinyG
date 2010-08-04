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

/* 
 * USART config values and tables
 */

// this part is hardware - you cannot change it without changing the board
#define RS4_USART USARTC1		// RS485 usart
#define RS4_PORT PORTC			// port where the above USART is located
#define RS4_RE_bm (1<<4)		// RE (Receive Enable) pin - active lo
#define RS4_DE_bm (1<<5)		// DE (Data Enable) pin (TX enable) - active hi
#define RS4_RX_bm (1<<6)		// RX pin - these pins are wired on the board
#define RS4_TX_bm (1<<7)		// TX pin

#define USB_USART USARTC0		// USB usart
#define USB_PORT PORTC			// port where the USART is located
#define USB_CTS_bm (1<<0)		// CTS pin
#define USB_RTS_bm (1<<1)		// RTS pin
#define USB_RX_bm (1<<2)		// RX pin	- these pins are wired on the board
#define USB_TX_bm (1<<3)		// TX pin

#define TTL_USART USARTC0		// Arduino usart
#define TTL_PORT PORTC			// port where the USART is located
#define TTL_CTS_bm (1<<0)		// CTS pin
#define TTL_RTS_bm (1<<1)		// RTS pin
#define TTL_RX_bm (1<<2)		// RX pin	- these pins are wired on the board
#define TTL_TX_bm (1<<3)		// TX pin

// this part is software - change as needed
#define RS4_DIRCLR_bm (RS4_RX_bm)							// input bits
#define RS4_DIRSET_bm (RS4_RE_bm | RS4_DE_bm | RS4_TX_bm)	// output bits
#define RS4_OUTCLR_bm (RS4_RE_bm | RS4_DE_bm)				// output set to 0
#define RS4_OUTSET_bm (RS4_TX_bm)							// output set to 1

#define USB_DIRCLR_bm (USB_CTS_bm | USB_RX_bm)				// as above
#define USB_DIRSET_bm (USB_RTS_bm | USB_TX_bm)
#define USB_OUTCLR_bm (0)
#define USB_OUTSET_bm (USB_RTS_bm | USB_TX_bm)

#define TTL_DIRCLR_bm (USB_RX_bm)
#define TTL_DIRSET_bm (USB_TX_bm)
#define TTL_OUTCLR_bm (0)
#define TTL_OUTSET_bm (USB_TX_bm)

// some constants for turning interrupts on and off
#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_LO_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)

/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
 */

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
 * USART control structure - here because it's shared by multiple devices.
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable (see xmega_io.c circular buffer note) 
 */

//#define RX_BUFFER_SIZE 255	// rx buf (255 max) - written by ISRs (2 bytes unusable)
#define RX_BUFFER_SIZE 18	// rx buf (255 max) - written by ISRs (2 bytes unusable)
#define TX_BUFFER_SIZE 18	// tx buf (255 max) - read by ISRs (2 bytes unusable)

struct xioUSART {
	// PUBLIC VARIABLES - must be the same in every device type
	uint16_t flags;						// control flags
	uint8_t status;						// completion status 
	uint8_t sig;						// signal or error value
	uint8_t c;							// line buffer character temp
	uint8_t i;							// line buffer pointer (persistent)
	uint8_t len;						// line buffer maximum length (zero based)
	char *buf;							// pointer to input line buffer

	// PRIVATE VARIABLES - in this case for USART. Can be different by device type
	volatile uint_fast8_t rx_buf_tail;	// RX buffer read index
	volatile uint_fast8_t rx_buf_head;	// RX buffer write index (written by ISR)
	volatile uint_fast8_t tx_buf_tail;	// TX buffer read index (written by ISR)
	volatile uint_fast8_t tx_buf_head;	// TX buffer write index
	uint_fast8_t next_tx_buf_head;		// next TX buffer write index

	// hardware bindings
	struct USART_struct *usart;			// USART structure
	struct PORT_struct *port;			// corresponding port

	// buffers (do these last)
	volatile unsigned char rx_buf[RX_BUFFER_SIZE];  // (written by ISR)
	volatile unsigned char tx_buf[TX_BUFFER_SIZE];

};

/*
 * Global Scope Functions
 */

void xio_init_usart(uint8_t dev, struct xioUSART *u, const uint16_t control);

//void xio_usb_init(uint16_t control);
//int8_t xio_usb_control(uint16_t control, int16_t arg);
//int xio_usb_putc(char c, FILE *stream);
//int xio_usb_getc(FILE *stream);
//void xio_usb_queue_RX_char(char c);		// simulate char received into RX buffer
//void xio_usb_queue_RX_string(char *buf);// do a whole string
//int xio_usb_readln(char *buf, uint8_t len);	// non-blocking read line function

//extern FILE dev_usb;					// declare the FILE handle for external use

#endif
