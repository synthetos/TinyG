/*
  xmega_io_usart.h - serial and "file" IO functions for xmega family
  USART module

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xmega_io_usart_h
#define xmega_io_usart_h

#include "xmega_io.h"

/*
 * Local subsystem configs, constants, and device structures 
 */

#define USART_TX_even_bm (1<<3)		// TX pin for even USARTs (e.g. USARTC0)
#define USART_RX_even_bm (1<<2)		// RX pin 
#define USART_RTS_even_bm (1<<1)	// RTS pin (or extra for other purposes)
#define USART_CTS_even_bm (1<<0)	// CTS pin (or extra for other purposes)

#define USART_TX_odd_bm (1<<7)		// TX pin for even USARTs (e.g. USARTC1)
#define USART_RX_odd_bm (1<<6)		// RX pin 
#define USART_RTS_odd_bm (1<<5)		// RTS pin (or extra for other purposes)
#define USART_CTS_odd_bm (1<<4)		// CTS pin (or extra for other purposes)

/* 
 * Function prototypes and aliases
 */

int8_t xio_open_USART(uint8_t dev, uint32_t control);
int8_t xio_close_USART(struct fdXIO *f);
int8_t xio_control_USART(struct fdXIO *f, uint32_t control, int16_t arg);
//int16_t xio_read_USART(struct fdXIO *fd_ptr, char *buf, int16_t size);
//int16_t xio_write_USART(struct fdXIO *fd_ptr, const char *buf, int16_t size);
char xio_getc_USART(struct fdXIO *fd_ptr);
char xio_putc_USART(struct fdXIO *fd_ptr, const char c);

#endif
