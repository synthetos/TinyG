/*
 * xio_usart.h - Common USART definitions 
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

#ifndef xio_usart_h
#define xio_usart_h

/* 
 * USART DEVICE FUNCTION PROTOTYPES AND ALIASES
 */

//#define xio_gets_usb(buf, siz) xio_gets_usart(XIO_DEV_USB, buf, siz)

// Common functions (common to all USART devices)
void xio_init_usart(const uint8_t dev, 
					const uint8_t offset,
					const uint32_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset);

void xio_set_baud_usart(const uint8_t dev, const uint8_t baud);
void xio_xoff_usart(const uint8_t dev);
void xio_xon_usart(const uint8_t dev);
void xio_deassert_rts_usart(const uint8_t dev);
void xio_assert_rts_usart(const uint8_t dev);
int xio_putc_usart(const uint8_t dev, const char c, FILE *stream);
int xio_getc_usart(const uint8_t dev, FILE *stream);
int xio_gets_usart(const uint8_t dev, char *buf, const int size);
void xio_queue_RX_char_usart(const uint8_t dev, const char c);
void xio_queue_RX_string_usart(const uint8_t dev, const char *buf);


// RS485 specific functions
FILE * xio_open_rs485();						// returns stdio fdev handle (note)
int xio_cntl_rs485(const uint32_t control);		// set control flags w/validation
int xio_putc_rs485(const char c, FILE *stream);	// stdio compatible put character
int xio_getc_rs485(FILE *stream);				// stdio compatible get character
int xio_gets_rs485(char *buf, const int size);	// non-blocking read line function
void xio_queue_RX_char_rs485(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_rs485(const char *buf);// simulate rec'ving a whole string

// USB specific functions
FILE * xio_open_usb();							// returns stdio fdev handle
int xio_cntl_usb(const uint32_t control);		// set control flags w/validation
int xio_putc_usb(const char c, FILE *stream);	// stdio compatible put character
int xio_getc_usb(FILE *stream);					// stdio compatible get character
int xio_gets_usb(char *buf, const int size);	// non-blocking read line function
void xio_queue_RX_char_usb(const char c);		// simulate char rcvd into RX buffer
void xio_queue_RX_string_usb(const char *buf);	// simulate receving a whole string
void xio_dump_RX_queue_usart(void);				//+++++++++++++++++++++

// Note: don't put void in the open() prototypes- it conflicts with the file open()s

// TTL usart specific functions (Arduino)

/* 
 * USART DEVICE CONFIGS (applied during device-specific inits)
 *
 *	NOTE: XIO_BLOCK / XIO_NOBLOCK affects reads only (see xio.h)
 */

// general USART defines

#define USB ds[XIO_DEV_USB]			// device struct accessor
#define USBu us[XIO_DEV_USB_OFFSET]	// usart extended struct accessor

#define USART_TX_REGISTER_READY_bm USART_DREIF_bm
#define USART_RX_DATA_READY_bm USART_RXCIF_bm

//**** Serial IO Interrupt levels are mapped to MED level here ****

#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF_TXCON (USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_MED_gc)
#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc)
#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_MED_gc)

// **** Same as above, but with TX in LO interrupt ****
/*
#define CTRLA_RXON_TXON (USART_RXCINTLVL_MED_gc | USART_DREINTLVL_LO_gc)
#define CTRLA_RXON_TXOFF (USART_RXCINTLVL_MED_gc)
#define CTRLA_RXON_TXOFF_TXCON (USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_LO_gc)
#define CTRLA_RXOFF_TXON_TXCON (USART_DREINTLVL_LO_gc | USART_TXCINTLVL_LO_gc)
#define CTRLA_RXOFF_TXOFF_TXCON (USART_TXCINTLVL_LO_gc)
*/

// **** RS485 device configuration (no echo or CRLF) ****
#define RS485_INIT_bm (XIO_RDWR | XIO_NOBLOCK | XIO_NOECHO | XIO_LINEMODE | XIO_BAUD_115200)

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

//**** USB device configuration ****
#define USB_INIT_bm (XIO_RDWR | XIO_BLOCK |  XIO_ECHO | XIO_XOFF | XIO_LINEMODE | XIO_BAUD_115200)

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

//**** TTL device (Arduino) ****  (not used. Arduino piggybacks, USB serial inputs)
#define TTL_INIT_bm (XIO_RDWR | XIO_BLOCK | XIO_ECHO | XIO_CRLF | XIO_LINEMODE | XIO_BAUD_115200)

#define TTL_USART USARTC0					// Arduino usart
#define TTL_PORT PORTC						// port where the USART is located
#define TTL_CTS_bm (1<<0)					// CTS bit mask
#define TTL_RTS_bm (1<<1)					// RTS bit mask
#define TTL_RX_bm (1<<2)					// RX pin bit mask
#define TTL_TX_bm (1<<3)					// TX pin bit mask

#define TTL_DIRCLR_bm (USB_RX_bm)
#define TTL_DIRSET_bm (USB_TX_bm)
#define TTL_OUTCLR_bm (0)
#define TTL_OUTSET_bm (USB_TX_bm)

/* 
 * USART DEVICE CONSTANTS AND PARAMETERS
 */

//Choose one: (this define sets the data type in all places that need it)
#define BUFFER_T uint_fast8_t	// faster, but limits buffer to 255 char max
//#define BUFFER_T uint16_t		// slower, but larger buffers

#define RX_BUFFER_SIZE (BUFFER_T)255	// BUFFER_T can be 8 bits
//#define RX_BUFFER_SIZE (BUFFER_T)512	// BUFFER_T must be 16 bits if >255
//#define RX_BUFFER_SIZE (BUFFER_T)1024	// 2048 is the practical upper limit
#define TX_BUFFER_SIZE (BUFFER_T)255	// BUFFER_T can be 8 bits
//#define TX_BUFFER_SIZE (BUFFER_T)512	// BUFFER_T must be 16 bits if >255
//#define TX_BUFFER_SIZE (BUFFER_T)1024	// 2048 is practical upper limit given RAM

// XON/XOFF hi and lo watermarks. At 115.200 the host has approx. 100 uSec
// per character to react to an XOFF. 90% (0.9) of 255 chars gives 25 chars
// to react, or about 2.5 ms.  
#define XOFF_RX_HI_WATER_MARK (RX_BUFFER_SIZE * 0.8)	// % to issue XOFF
#define XOFF_RX_LO_WATER_MARK (RX_BUFFER_SIZE * 0.1)	// % to issue XON
#define XOFF_TX_HI_WATER_MARK (TX_BUFFER_SIZE * 0.9)	// % to issue XOFF
#define XOFF_TX_LO_WATER_MARK (TX_BUFFER_SIZE * 0.05)	// % to issue XON

/* 
 * Serial Configuration Settings
 *
 * 	Serial config settings are here because various modules will be opening devices
 *	The BSEL / BSCALE values provided below assume a 32 Mhz clock
 *	These are carried in the bsel and bscale tables in xmega_io.c
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

/* 
 * USART extended control structure 
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable
 */

struct xioUSART {
//	uint16_t uflags;				// usart sub-system flags (UNUSED)
	uint8_t fc_char;	 			// flow control character to send
	volatile uint8_t fc_state;		// flow control state
	BUFFER_T rx_buf_tail;			// RX buffer read index  (not VOLATILE)
	volatile BUFFER_T rx_buf_head;	// RX buffer write index (written by ISR)
	volatile BUFFER_T tx_buf_tail;	// TX buffer read index  (written by ISR)
	BUFFER_T tx_buf_head;			// TX buffer write index (not VOLATILE)

	struct USART_struct *usart;		// USART structure
	struct PORT_struct *port;		// corresponding port

	volatile char rx_buf[RX_BUFFER_SIZE];  // (written by ISR)
	volatile char tx_buf[TX_BUFFER_SIZE];
};

// down here by their lonesome because they need xioUSART defined
BUFFER_T xio_get_rx_bufcount_usart(const struct xioUSART *dx);
BUFFER_T xio_get_tx_bufcount_usart(const struct xioUSART *dx);
uint16_t xio_get_usb_rx_free(void);

#endif
