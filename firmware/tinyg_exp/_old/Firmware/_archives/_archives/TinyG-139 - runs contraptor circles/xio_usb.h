/*
  xio_usb.h - FTDI USB port driver for xmega familt
  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xio_usb_h
#define xio_usb_h

/* 
 * Function prototypes and aliases
 */

void xio_usb_init(uint16_t control);
int8_t xio_usb_control(uint16_t control, int16_t arg);
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

#endif
