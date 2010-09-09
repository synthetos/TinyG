/*
 * xio_usb.h - FTDI USB port driver for xmega family
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef xio_usb_h
#define xio_usb_h

/*
 * Global Scope Functions
 */

void xio_usb_init(uint16_t control);
int8_t xio_usb_control(uint16_t control, int16_t arg);
int xio_usb_putc(char c, FILE *stream);
int xio_usb_getc(FILE *stream);
void xio_usb_queue_RX_char(char c);		// simulate char received into RX buffer
void xio_usb_queue_RX_string(char *buf);// do a whole string
int xio_usb_readln(char *buf, uint8_t len);	// non-blocking read line function

extern FILE dev_usb;					// declare the FILE handle for external use

#endif
