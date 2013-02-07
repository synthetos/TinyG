/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* FT245/2232 asynchronous Fifo Module                                  */
/*                                                                      */
/* fifo.h                                                               */
/*                                                                      */
/* Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de                   */
/*                                                                      */
/* Copyright (c) 2011 Uwe Bonnes                                        */
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

#ifndef __FIFO_H
#define __FIFO_H

#include "xboot.h"

// Globals

// Defines

#ifdef __AVR_XMEGA__
// nonzero if character has been received
#define fifo_char_received() ((FIFO_CTL_PORT.IN & _BV(FIFO_RXF_N)) != _BV(FIFO_RXF_N))
#else // __AVR_XMEGA__
// nonzero if character has been received
#define fifo_char_received() ((FIFO_CTL_PORT_PIN & _BV(FIFO_RXF_N)) != _BV(FIFO_RXF_N))
#endif // __AVR_XMEGA__

// current character in UART receive buffer
extern uint8_t fifo_cur_char(void);
// send character
extern void fifo_send_char(uint8_t c); 
// send character, block until it is completely sent
extern void fifo_send_char_blocking(uint8_t c); 

// Prototypes
extern void fifo_init(void);
extern void fifo_deinit(void);

#endif // __UART_H
