/*
 * xio_usart.h - Common USART definitions
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * Rob: The usart.h and .c files can be considered the parent class for the
 * USB and RS485 devices which are derived from them. The usart.h file acts
 * as the header file for all three classes: usart.c, usb.c and rs485.c
 */

#ifndef xio_usart_h
#define xio_usart_h

/******************************************************************************
 * USART DEVICE CONFIGS (applied during device-specific inits)
 ******************************************************************************/

// Serial IO Interrupt levels
// Maps both RX and TX to medium interrupt levels
#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF_TXCON (USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_MED_gc)
#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc)
#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_MED_gc)

// Maps RX to medium and TX to lo interrupt levels
// But don't use this or exception reports and other prints from medium interrupts
// can cause the system to lock up if the TX buffer is full. See xio.h for explanation.
//#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_LO_gc)
//#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)
//#define CTRLA_RXON_TXOFF_TXCON (USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_LO_gc)
//#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_LO_gc | USART_TXCINTLVL_LO_gc)
//#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_LO_gc)

// Buffer sizing
#define buffer_t uint_fast8_t					// fast, but limits buffer to 255 char max
//#define buffer_t uint16_t						// larger buffers

// Must reserve 2 bytes for buffer management
#define RX_BUFFER_SIZE (buffer_t)254			// buffer_t can be 8 bits
#define TX_BUFFER_SIZE (buffer_t)254			// buffer_t can be 8 bits
//#define RX_BUFFER_SIZE (buffer_t)255			// buffer_t can be 8 bits
//#define TX_BUFFER_SIZE (buffer_t)255			// buffer_t can be 8 bits

// Alternates for larger buffers - mostly for debugging
//#define buffer_t uint16_t						// slower, but larger buffers
//#define RX_BUFFER_SIZE (buffer_t)510			// buffer_t must be 16 bits if >255
//#define TX_BUFFER_SIZE (buffer_t)510			// buffer_t must be 16 bits if >255
//#define RX_BUFFER_SIZE (buffer_t)1022			// 2046 is the practical upper limit
//#define TX_BUFFER_SIZE (buffer_t)1022			// 2046 is practical upper limit given RAM

// XON/XOFF hi and lo watermarks. At 115.200 the host has approx. 100 uSec per char
// to react to an XOFF. 90% (0.9) of 255 chars gives 25 chars to react, or about 2.5 ms.
#define XOFF_RX_HI_WATER_MARK (RX_BUFFER_SIZE * 0.8)	// % to issue XOFF
#define XOFF_RX_LO_WATER_MARK (RX_BUFFER_SIZE * 0.1)	// % to issue XON
#define XOFF_TX_HI_WATER_MARK (TX_BUFFER_SIZE * 0.9)	// % to issue XOFF
#define XOFF_TX_LO_WATER_MARK (TX_BUFFER_SIZE * 0.05)	// % to issue XON

// General
#define USART_TX_REGISTER_READY_bm USART_DREIF_bm
#define USART_RX_DATA_READY_bm USART_RXCIF_bm

//**** USB device configuration ****
//NOTE: XIO_BLOCK / XIO_NOBLOCK affects reads only. Writes always block. (see xio.h)

#define USB_BAUD  XIO_BAUD_115200
#define USB_FLAGS (XIO_BLOCK |  XIO_ECHO | XIO_XOFF | XIO_LINEMODE )

#define USB_USART USARTC0						// USB usart
#define USB_RX_ISR_vect USARTC0_RXC_vect	 	// (RX) reception complete IRQ
#define USB_TX_ISR_vect USARTC0_DRE_vect		// (TX) data register empty IRQ

#define USB_PORT PORTC							// port where the USART is located
#define USB_CTS_bp (1)							// CTS - bit position (pin is wired on board)
#define USB_CTS_bm (1<<USB_CTS_bp)				// CTS - bit mask
#define USB_CTS_PINCTRL PIN1CTRL				// CTS - PINxCTRL assignment
#define USB_CTS_ISR_vect PORTC_INT0_vect		// CTS - Interrupt Vector (PORTC_INT0_vect or PORTC_INT1_vect)
#define USB_CTS_INTMSK INT0MASK					// CTS - Interrupt Mask Register (INT0MASK or INT1MASK)
#define USB_CTS_INTLVL (PORT_INT0LVL_LO_gc)

#define USB_RTS_bp (0)							// RTS - bit position (pin is wired on board)
#define USB_RTS_bm (1<<USB_RTS_bp)				// RTS - bit mask

#define USB_RX_bm (1<<2)						// RX pin bit mask
#define USB_TX_bm (1<<3)						// TX pin bit mask

#define USB_INBITS_bm (USB_CTS_bm | USB_RX_bm)	// input bits
#define USB_OUTBITS_bm (USB_RTS_bm | USB_TX_bm)	// output bits
#define USB_OUTCLR_bm (USB_RTS_bm)				// outputs init'd to 0
#define USB_OUTSET_bm (USB_TX_bm)				// outputs init'd to 1

//**** RS485 device configuration (no echo or CRLF) ****
#define RS485_BAUD	   XIO_BAUD_115200
#define RS485_FLAGS (XIO_NOBLOCK | XIO_NOECHO | XIO_LINEMODE)

#define RS485_USART USARTC1						// RS485 usart
#define RS485_RX_ISR_vect USARTC1_RXC_vect 		// (RX) reception complete IRQ
#define RS485_TX_ISR_vect USARTC1_DRE_vect		// (TX) data register empty IRQ
#define RS485_TXC_ISR_vect USARTC1_TXC_vect		// (TX) transmission complete IRQ

#define RS485_PORT PORTC						// port where USART is located
#define RS485_RE_bm (1<<4)						// RE (Receive Enable) pin - active lo
#define RS485_DE_bm (1<<5)						// DE (Data Enable)(TX) - active hi
#define RS485_RX_bm (1<<6)						// RX pin
#define RS485_TX_bm (1<<7)						// TX pin

#define RS485_INBITS_bm (RS485_RX_bm)			// input bits
#define RS485_OUTBITS_bm (RS485_RE_bm | RS485_DE_bm | RS485_TX_bm)// output bits
#define RS485_OUTCLR_bm (RS485_RE_bm| RS485_DE_bm)	// outputs init'd to 0
#define RS485_OUTSET_bm (RS485_TX_bm)				// outputs init'd to 1

/*
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	Assumes CTRLB CLK2X bit (0x04) is not enabled
 *	These are carried in the bsel and bscale tables in xio_usart.c
 */

// Baud rate configuration
#define	XIO_BAUD_DEFAULT XIO_BAUD_115200

enum xioBAUDRATES {         		// BSEL	  BSCALE
		XIO_BAUD_UNSPECIFIED = 0,	//	0		0	  // use default value
		XIO_BAUD_9600,				//	207		0
		XIO_BAUD_19200,				//	103		0
		XIO_BAUD_38400,				//	51		0
		XIO_BAUD_57600,				//	34		0
		XIO_BAUD_115200,			//	33		(-1<<4)
		XIO_BAUD_230400,			//	31		(-2<<4)
		XIO_BAUD_460800,			//	27		(-3<<4)
		XIO_BAUD_921600,			//	19		(-4<<4)
		XIO_BAUD_500000,			//	1		(1<<4)
		XIO_BAUD_1000000			//	1		0
};

enum xioFCState {
		FC_DISABLED = 0,			// flow control is disabled
		FC_IN_XON,					// normal, un-flow-controlled state
		FC_IN_XOFF					// flow controlled state
};

/******************************************************************************
 * STRUCTURES
 ******************************************************************************/
/*
 * USART extended control structure
 * Note: As defined this struct won't do buffers larger than 256 chars -
 *	     or a max of 254 characters usable
 */
typedef struct xioUSART {
	uint8_t fc_char_rx;			 			// RX-side flow control character to send
	volatile uint8_t fc_state_rx;			// flow control state on RX side
	volatile uint8_t fc_state_tx;			// flow control state on TX side

	volatile buffer_t rx_buf_tail;			// RX buffer read index
	volatile buffer_t rx_buf_head;			// RX buffer write index (written by ISR)
	volatile buffer_t rx_buf_count;			// RX buffer counter for flow control

	volatile buffer_t tx_buf_tail;			// TX buffer read index  (written by ISR)
	volatile buffer_t tx_buf_head;			// TX buffer write index
	volatile buffer_t tx_buf_count;

	USART_t *usart;							// xmega USART structure
	PORT_t	*port;							// corresponding port

	volatile char rx_buf[RX_BUFFER_SIZE];	// (written by ISR)
	volatile char tx_buf[TX_BUFFER_SIZE];
} xioUsart_t;

/******************************************************************************
 * USART CLASS AND DEVICE FUNCTION PROTOTYPES AND ALIASES
 ******************************************************************************/

void xio_init_usart(void);
FILE *xio_open_usart(const uint8_t dev, const char *addr, const flags_t flags);
void xio_set_baud_usart(xioUsart_t *dx, const uint8_t baud);
void xio_xoff_usart(xioUsart_t *dx);
void xio_xon_usart(xioUsart_t *dx);
int xio_gets_usart(xioDev_t *d, char *buf, const int size);
int xio_getc_usart(FILE *stream);
int xio_putc_usart(const char c, FILE *stream);
int xio_putc_usb(const char c, FILE *stream);	// stdio compatible put character
int xio_putc_rs485(const char c, FILE *stream);	// stdio compatible put character
void xio_enable_rs485_rx(void);					// needed for startup
void xio_enable_rs485_tx(void);					// included for completeness

// handy helpers
buffer_t xio_get_rx_bufcount_usart(const xioUsart_t *dx);
buffer_t xio_get_tx_bufcount_usart(const xioUsart_t *dx);
buffer_t xio_get_usb_rx_free(void);
void xio_reset_usb_rx_buffers(void);

void xio_queue_RX_char_usart(const uint8_t dev, const char c);
void xio_queue_RX_string_usart(const uint8_t dev, const char *buf);
void xio_queue_RX_char_usb(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_usb(const char *buf);	// simulate receving a whole string
void xio_queue_RX_char_rs485(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_rs485(const char *buf);// simulate rec'ving a whole string

#endif
