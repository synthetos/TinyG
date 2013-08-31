/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* UART Module                                                          */
/*                                                                      */
/* uart.c                                                               */
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

#include "uart.h"

// Interrupts
#ifdef USE_INTERRUPTS
#ifdef USE_UART
ISR(UART_DEVICE_RXC_ISR)
{
        if (comm_mode == MODE_UNDEF)
        {
                comm_mode = MODE_UART;
                #ifdef USE_I2C
                #ifdef __AVR_XMEGA__
                // disable I2C interrupt
                I2C_DEVICE.SLAVE.CTRLA = 0;
                #endif // __AVR_XMEGA__
                #endif // USE_I2C
        }
        if (rx_char_cnt == 0)
        {
                rx_buff0 = UART_DEVICE.DATA;
                rx_char_cnt = 1;
        }
        else
        {
                rx_buff1 = UART_DEVICE.DATA;
                rx_char_cnt = 2;
        }
}

ISR(UART_DEVICE_TXC_ISR)
{
        tx_char_cnt = 0;
}
#endif // USE_UART
#endif // USE_INTERRUPTS

// Initialize UART
void uart_init(void)
{
        UART_PORT.DIRSET = (1 << UART_TX_PIN);
        UART_DEVICE.BAUDCTRLA = (UART_BSEL_VALUE & USART_BSEL_gm);
        UART_DEVICE.BAUDCTRLB = ((UART_BSCALE_VALUE << USART_BSCALE_gp) & USART_BSCALE_gm) | ((UART_BSEL_VALUE >> 8) & ~USART_BSCALE_gm);
        #if UART_CLK2X
        UART_DEVICE.CTRLB = USART_RXEN_bm | USART_CLK2X_bm | USART_TXEN_bm;
        #else
        UART_DEVICE.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
        #endif // UART_CLK2X
        #ifdef USE_INTERRUPTS
        UART_DEVICE.CTRLA = USART_RXCINTLVL0_bm | USART_TXCINTLVL0_bm;
        #endif // USE_INTERRUPTS
}

// Shut down UART
void uart_deinit(void)
{
        UART_DEVICE.CTRLB = 0;
        #ifdef USE_INTERRUPTS
        UART_DEVICE.CTRLA = 0;
        #endif // USE_INTERRUPTS
        UART_DEVICE.BAUDCTRLA = 0;
        UART_DEVICE.BAUDCTRLB = 0;
        UART_PORT.DIRCLR = (1 << UART_TX_PIN);
}
/*
void uart_send_string(char *s) 
{
	while (*s != 0) {
		uart_send_char_blocking(*s); 
//		UART_DEVICE.DATA = *s;
//		while (!(UART_DEVICE.STATUS & USART_TXCIF_bm)) {}
//		UART_DEVICE.STATUS |= USART_TXCIF_bm;
		s++;
	}
}

*/
