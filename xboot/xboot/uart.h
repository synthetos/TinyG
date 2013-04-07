/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* UART Module                                                          */
/*                                                                      */
/* uart.h                                                               */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2010 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef __UART_H
#define __UART_H

#include "xboot.h"

// Globals

// Defines
#ifdef __AVR_XMEGA__

// nonzero if character has been received
#define uart_char_received() (UART_DEVICE.STATUS & USART_RXCIF_bm)
// current character in UART receive buffer
#define uart_cur_char() UART_DEVICE.DATA
// send character
#define uart_send_char(c) UART_DEVICE.DATA = (c)
// send character, block until it is completely sent
#define uart_send_char_blocking(c) do {uart_send_char(c); \
    while (!(UART_DEVICE.STATUS & USART_TXCIF_bm)) { } \
    UART_DEVICE.STATUS |= USART_TXCIF_bm; } while (0)

#else // __AVR_XMEGA__

// nonzero if character has been received
#define uart_char_received() (UART_UCSRA & _BV(RXC0))
// current character in UART receive buffer
#define uart_cur_char() UART_UDR
// send character
#define uart_send_char(c) UART_UDR = (c)
// send character, block until it is completely sent
#define uart_send_char_blocking(c) do {uart_send_char(c); \
    while (!(UART_UCSRA & _BV(TXC0))) { } \
    UART_UCSRA |= _BV(TXC0); } while (0)

#endif // __AVR_XMEGA__

// Prototypes
extern void uart_init(void);
extern void uart_deinit(void);
//extern void uart_send_string(char *s);

#endif // __UART_H

