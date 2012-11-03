/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* FT245/2232 asynchronous Fifo Module                                  */
/*                                                                      */
/* fifo.c                                                               */
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

#include "fifo.h"
/* As discussed in
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=41613
 * Accessing a bitrev table in bootloader flash will not be faster, as 
 * a our character needs to be added to the table converted to a far address
 * and probaly also the NVM needs to be cared about
 */ 

#define REVERSE(a) do                     \
{                                         \
  a=((a>>1)&0x55)|((a<<1)&0xaa);          \
  a=((a>>2)&0x33)|((a<<2)&0xcc);          \
  asm volatile("swap %0":"=r"(a):"0"(a)); \
} while(0)

// Initialize FIFO
void __attribute__ ((always_inline)) fifo_init(void)
{
        #ifdef __AVR_XMEGA__
        FIFO_DATA_PORT.DIR = 0;
        FIFO_CTL_PORT.OUTSET = FIFO_RD_N_bm | FIFO_WR_N_bm;
        FIFO_CTL_PORT.DIRSET = FIFO_RD_N_bm | FIFO_WR_N_bm;
        #endif // __AVR_XMEGA__
}

// Shut down UART
void __attribute__ ((always_inline)) fifo_deinit(void)
{
        #ifdef __AVR_XMEGA__
        FIFO_DATA_PORT.DIR = 0xff;
        FIFO_CTL_PORT.OUTCLR = FIFO_RD_N_bm | FIFO_WR_N_bm;
        FIFO_CTL_PORT.DIRCLR = FIFO_RD_N_bm | FIFO_WR_N_bm;
        #endif // __AVR_XMEGA__
}

uint8_t __attribute__ ((always_inline)) fifo_cur_char(void)
{
    uint8_t ret;
    FIFO_CTL_PORT.OUTCLR = FIFO_RD_N_bm;
    ret = FIFO_DATA_PORT.IN;
#ifdef  FIFO_BIT_REVERSE
    REVERSE(ret);
#endif
    FIFO_CTL_PORT.OUTSET = FIFO_RD_N_bm;
    return ret;
}

void __attribute__ ((always_inline))  fifo_send_char(uint8_t c)
{
    if ((FIFO_CTL_PORT.IN & FIFO_TXE_N_bm) !=  FIFO_TXE_N_bm)
    {
        FIFO_DATA_PORT.DIR = 0xff;
#ifdef  FIFO_BIT_REVERSE
        REVERSE(c);
#endif
        FIFO_DATA_PORT.OUT = c;
        FIFO_DATA_PORT.DIR = 0xff;
        FIFO_CTL_PORT.OUTCLR = FIFO_WR_N_bm;
        FIFO_DATA_PORT.DIR = 0;
        FIFO_CTL_PORT.OUTSET = FIFO_WR_N_bm;
    }
}

void __attribute__ ((always_inline))  fifo_send_char_blocking(uint8_t c)
{
    while(FIFO_CTL_PORT.IN & FIFO_TXE_N_bm)
    {
    };
    fifo_send_char(c);
}
