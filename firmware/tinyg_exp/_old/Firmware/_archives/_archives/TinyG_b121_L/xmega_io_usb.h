/*
  xmega_io_usb.h - serial and "file" IO functions for xmega family
  USB module

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xmega_io_usb_h
#define xmega_io_usb_h

#include "xmega_io.h"
#include "xmega_io_usart.h"

/* 
 * Function prototypes and aliases
 */

// USB device handlers
int8_t _open_usb(uint8_t dev, uint32_t control);
int8_t _close_usb(struct fdUSART *f);
int8_t _control_usb(struct fdUSART *f, uint32_t control, int16_t arg);
int16_t _read_usb(struct fdUSART *fd_ptr, char *buf, int16_t size);
int16_t _write_usb(struct fdUSART *fd_ptr, const char *buf, int16_t size);

/*
 * USB port assignments
 */

#define USB_USART USARTC0			// USARTC0 is wired to USB chip on the board
#define USB_RX_ISR_vect USARTC0_RXC_vect	// RX ISR
#define USB_TX_ISR_vect USARTC0_TXC_vect	// TX ISR

#define USB_PORT PORTC				// port where the USART is located
#define USB_RX_bm (1<<2)			// RX pin	- these pins are wired on the board
#define USB_TX_bm (1<<3)			// TX pin
#define USB_RTS_bm (1<<1)			// RTS pin
#define USB_CTS_bm (1<<0)			// CTS pin

#endif
