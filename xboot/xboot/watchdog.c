/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* Watchdog Module                                                      */
/*                                                                      */
/* watchdog.c                                                           */
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

#include "watchdog.h"

void WDT_EnableAndSetTimeout( void )
{
        uint8_t temp = WDT_ENABLE_bm | WDT_CEN_bm | WATCHDOG_TIMEOUT;
        CCP = CCP_IOREG_gc;
        WDT.CTRL = temp;
        
        /* Wait for WD to synchronize with new settings. */
        while(WDT_IsSyncBusy());
}

void WDT_Disable( void )
{
        uint8_t temp = (WDT.CTRL & ~WDT_ENABLE_bm) | WDT_CEN_bm;
        CCP = CCP_IOREG_gc;
        WDT.CTRL = temp;
}





