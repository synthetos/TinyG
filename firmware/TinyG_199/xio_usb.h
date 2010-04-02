/*
 * xio_usb.h - FTDI USB port driver for xmega family
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
