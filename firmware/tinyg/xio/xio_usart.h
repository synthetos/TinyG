/*
 * xio_usart.h - Common USART definitions 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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

//**** General ****
#define USB ds[XIO_DEV_USB]			// device struct accessor
#define USBu us[XIO_DEV_USB - XIO_DEV_USART_OFFSET]	// usart extended struct accessor

#define USART_TX_REGISTER_READY_bm USART_DREIF_bm
#define USART_RX_DATA_READY_bm USART_RXCIF_bm

//**** Serial IO Interrupt levels ****
#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_LO_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF_TXCON (USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_LO_gc)
#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_LO_gc | USART_TXCINTLVL_LO_gc)
#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_LO_gc)
// alternate: map TX to MED interrupt levels
//#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc)
//#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_MED_gc)

// Buffer sizing
#define BUFFER_T uint_fast8_t				// fast, but limits buffer to 255 char max
#define RX_BUFFER_SIZE (BUFFER_T)255		// BUFFER_T can be 8 bits
#define TX_BUFFER_SIZE (BUFFER_T)255		// BUFFER_T can be 8 bits

// Alternates for larger buffers - mostly for debugging
//#define BUFFER_T uint16_t					// slower, but larger buffers
//#define RX_BUFFER_SIZE (BUFFER_T)512		// BUFFER_T must be 16 bits if >255
//#define TX_BUFFER_SIZE (BUFFER_T)512		// BUFFER_T must be 16 bits if >255
//#define RX_BUFFER_SIZE (BUFFER_T)1024		// 2048 is the practical upper limit
//#define TX_BUFFER_SIZE (BUFFER_T)1024		// 2048 is practical upper limit given RAM

// XON/XOFF hi and lo watermarks. At 115.200 the host has approx. 100 uSec per char 
// to react to an XOFF. 90% (0.9) of 255 chars gives 25 chars to react, or about 2.5 ms.  
#define XOFF_RX_HI_WATER_MARK (RX_BUFFER_SIZE * 0.8)	// % to issue XOFF
#define XOFF_RX_LO_WATER_MARK (RX_BUFFER_SIZE * 0.1)	// % to issue XON
#define XOFF_TX_HI_WATER_MARK (TX_BUFFER_SIZE * 0.9)	// % to issue XOFF
#define XOFF_TX_LO_WATER_MARK (TX_BUFFER_SIZE * 0.05)	// % to issue XON


//**** USB device configuration ****
//NOTE: XIO_BLOCK / XIO_NOBLOCK affects reads only. Writes always block. (see xio.h)

#define USB_BAUD	 XIO_BAUD_115200
#define USB_INIT_bm (XIO_BLOCK |  XIO_ECHO | XIO_XOFF | XIO_LINEMODE )
//#define USB_INIT_bm (XIO_RDWR | XIO_BLOCK |  XIO_ECHO | XIO_XOFF | XIO_LINEMODE )

#define USB_USART USARTC0					// USB usart
#define USB_RX_ISR_vect USARTC0_RXC_vect 	// (RX) reception complete IRQ
#define USB_TX_ISR_vect USARTC0_DRE_vect	// (TX) data register empty IRQ

#define USB_PORT PORTC						// port where the USART is located
#define USB_CTS_bp (0)						// CTS - bit position (pin is wired on board)
#define USB_CTS_bm (1<<USB_CTS_bp)			// CTS - bit mask
#define USB_RTS_bp (1)						// RTS - bit position (pin is wired on board)
#define USB_RTS_bm (1<<USB_RTS_bp)			// RTS - bit mask
#define USB_RX_bm (1<<2)					// RX pin bit mask
#define USB_TX_bm (1<<3)					// TX pin bit mask

#define USB_DIRCLR_bm (USB_CTS_bm | USB_RX_bm)	// input bits
#define USB_DIRSET_bm (USB_RTS_bm | USB_TX_bm)	// output bits
#define USB_OUTCLR_bm (0)						// outputs init'd to 0
#define USB_OUTSET_bm (USB_RTS_bm | USB_TX_bm)	// outputs init'd to 1


//**** RS485 device configuration (no echo or CRLF) ****
#define RS485_BAUD	   XIO_BAUD_115200
#define RS485_INIT_bm (XIO_NOBLOCK | XIO_NOECHO | XIO_LINEMODE)
//#define RS485_INIT_bm (XIO_RDWR | XIO_NOBLOCK | XIO_NOECHO | XIO_LINEMODE)

#define RS485_USART USARTC1					// RS485 usart
#define RS485_RX_ISR_vect USARTC1_RXC_vect 	// (RX) reception complete IRQ
#define RS485_TX_ISR_vect USARTC1_DRE_vect	// (TX) data register empty IRQ
#define RS485_TXC_ISR_vect USARTC1_TXC_vect	// (TX) transmission complete IRQ

#define RS485_PORT PORTC					// port where USART is located
#define RS485_RE_bm (1<<4)					// RE (Receive Enable) pin - active lo
#define RS485_DE_bm (1<<5)					// DE (Data Enable)(TX) - active hi
#define RS485_RX_bm (1<<6)					// RX pin
#define RS485_TX_bm (1<<7)					// TX pin

#define RS485_DIRCLR_bm (RS485_RX_bm)							 // input bits
#define RS485_DIRSET_bm (RS485_RE_bm | RS485_DE_bm | RS485_TX_bm)// output bits

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
};	// Note: cannot have more than 16 without changing XIO_BAUD_gm, below

enum xioFCState { 
		FC_DISABLED = 0,			// flo control is disabled
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
struct xioUSART {
	uint8_t fc_char;			 			// flow control character to send
	volatile uint8_t fc_state;				// flow control state

	volatile BUFFER_T rx_buf_tail;			// RX buffer read index
	volatile BUFFER_T rx_buf_head;			// RX buffer write index (written by ISR)
	volatile BUFFER_T rx_buf_count;			// RX buffer counter for flow control

	volatile BUFFER_T tx_buf_tail;			// TX buffer read index  (written by ISR)
	volatile BUFFER_T tx_buf_head;			// TX buffer write index
	volatile BUFFER_T tx_buf_count;

	struct USART_struct *usart;				// USART structure
	struct PORT_struct *port;				// corresponding port

	volatile char rx_buf[RX_BUFFER_SIZE];	// (written by ISR)
	volatile char tx_buf[TX_BUFFER_SIZE];
};
typedef struct xioUSART xioUsart;

/******************************************************************************
 * USART DEVICE FUNCTION PROTOTYPES AND ALIASES
 ******************************************************************************/

// Common functions (common to all USART devices)
void xio_init_usart(const uint8_t dev, 
					uint8_t baud, 
					const uint32_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t inbits, 
					const uint8_t outbits, 
					const uint8_t outclr, 
					const uint8_t outset);

void xio_set_baud_usart(xioUsart *dx, const uint8_t baud);
void xio_xoff_usart(xioUsart *dx);
void xio_xon_usart(xioUsart *dx);
int xio_gets_usart(const uint8_t dev, char *buf, const int size);
int xio_getc_usart(FILE *stream);
int xio_putc_usart(const char c, FILE *stream);

void xio_init_usb(void);						// USB specific functions (subclassing usart.c versions)
int xio_putc_usb(const char c, FILE *stream);	// stdio compatible put character

void xio_init_rs485(void);						// RS485 specific functions (subclassing usart.c versions)
int xio_putc_rs485(const char c, FILE *stream);	// stdio compatible put character

// handy helpers
BUFFER_T xio_get_rx_bufcount_usart(const xioUsart *dx);
BUFFER_T xio_get_tx_bufcount_usart(const xioUsart *dx);
BUFFER_T xio_get_usb_rx_free(void);

void xio_queue_RX_char_usart(const uint8_t dev, const char c);
void xio_queue_RX_string_usart(const uint8_t dev, const char *buf);
void xio_queue_RX_char_usb(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_usb(const char *buf);	// simulate receving a whole string
void xio_queue_RX_char_rs485(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_rs485(const char *buf);// simulate rec'ving a whole string
//void xio_dump_RX_queue_usart(void);

#endif
