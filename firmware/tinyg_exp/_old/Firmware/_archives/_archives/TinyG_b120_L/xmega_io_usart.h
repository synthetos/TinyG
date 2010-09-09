/*
  xmega_io_usart.h - serial and "file" IO functions for xmega family
  USART module

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xmega_io_usart_h
#define xmega_io_usart_h

#include "xmega_io.h"

/*
 * Major IO subsystem configs, constants, and device structures 
 */

#define USART_RX_BUFSIZE 18			// rx buffer - written by ISR (2 bytes unusable)
#define USART_TX_BUFSIZE 18			// tx buffer - (not used)
#define SSIZE_MAX USART_RX_BUFSIZE	// maximum bytes for read or write (progmem)

/* 
 * USART IO structure
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable (see xmega_io.c circular buffer note) 
 */

struct fdUSART {					// file descriptor struct for serial IO
	uint_fast8_t fd;				// the assigned FD number
	uint_fast8_t baud;				// baud rate index
	uint_fast8_t flags;				// control flags

	uint_fast8_t rx_buf_tail;		// RX buffer read index
	volatile uint8_t rx_buf_head;	// RX buffer write index (written by ISR)
	int16_t rx_size_max;			// max chars read before throwing EFBIG exception
									// valid is 1-32767.  -1 means no limit

	volatile uint8_t tx_buf_tail;	// TX buffer read index (written by ISR)
	uint_fast8_t tx_buf_head;		// TX buffer write index
	int16_t tx_size_max;			// max chars written before throwing EFBIG except
									// valid is 1-32767.  -1 means no limit

	volatile unsigned char rx_buf[USART_RX_BUFSIZE];  // (written by ISR)
//	volatile unsigned char tx_buf[USART_TX_BUFSIZE];

	struct USART_struct *usart;		// USART structure
	struct PORT_struct *port;		// corresponding port

	// somehow I'd like to get this to work
//	int16_t (*xio_read)(struct fdUSART *fd_ptr, char *buf, int size);
	int16_t (*read)();
	int16_t (*write)();
	int8_t (*close)();
	int8_t (*control)();
};

/* 
 * Function prototypes and aliases
 */

// USART device handlers
int8_t _open_USART(uint8_t dev, uint32_t control);
int8_t _close_USART(struct fdUSART *f);
int8_t _control_USART(struct fdUSART *f, uint32_t control, int16_t arg);
char _read_char_USART(struct fdUSART *fd_ptr);
char _write_char_USART(struct fdUSART *fd_ptr, const char c);

/*
 * generic USART device assignments
 */

#define USART_TX_even_bm (1<<3)		// TX pin for even USARTs (e.g. USARTC0)
#define USART_RX_even_bm (1<<2)		// RX pin 
#define USART_RTS_even_bm (1<<1)	// RTS pin (or extra for other purposes)
#define USART_CTS_even_bm (1<<0)	// CTS pin (or extra for other purposes)

#define USART_TX_odd_bm (1<<7)		// TX pin for even USARTs (e.g. USARTC1)
#define USART_RX_odd_bm (1<<6)		// RX pin 
#define USART_RTS_odd_bm (1<<5)		// RTS pin (or extra for other purposes)
#define USART_CTS_odd_bm (1<<4)		// CTS pin (or extra for other purposes)

#endif
