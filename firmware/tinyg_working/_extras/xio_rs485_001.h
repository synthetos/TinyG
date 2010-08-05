/*
 * xio_rs485.h - RS-485 port driver for xmega family
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

#ifndef xio_rs485_h
#define xio_rs485_h

/*
 * Global Scope Functions
 */

void xio_rs485_init(uint16_t control);
int xio_rs485_control(uint16_t control, int16_t arg);
int xio_rs485_putc(char c, FILE *stream);
int xio_rs485_getc(FILE *stream);
int xio_rs485_readln(char *buf, uint8_t len);// non-blocking read line function

void xio_rs485_queue_RX_char(char c);	  // simulate char received into RX buffer
void xio_rs485_queue_RX_string(char *buf);// do a whole string

extern FILE dev_rs485;				// declare the FILE handle for external use

#endif
