/************************************************************************/
/* Generic ATMEL Flash Driver                                           */
/*                                                                      */
/* flash.c                                                              */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2011 Alex Forencich                                    */
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

#include "flash.h"

#ifdef __AVR_XMEGA__

// XMega functions
// (sp_driver wrapper)

void Flash_ProgramPage(uint32_t page, uint8_t *buf, uint8_t erase)
{
        Flash_LoadFlashPage(buf);
        
        if (erase)
        {
                Flash_EraseWriteApplicationPage(page);
        }
        else
        {
                Flash_WriteApplicationPage(page);
        }
        
        Flash_WaitForSPM();
}

#else

// ATMega Functions

void Flash_EraseApplicationSection(void)
{
        for (uint32_t addr = 0; addr < APP_SECTION_END; addr += SPM_PAGESIZE) 
        {
                boot_page_erase(addr);
                boot_spm_busy_wait();
        }
        boot_rww_enable();
}

void Flash_EraseWriteApplicationPage(uint32_t addr)
{
        boot_page_erase(addr);
        boot_spm_busy_wait();
        boot_page_write(addr);
        boot_spm_busy_wait();
}

void Flash_LoadFlashPage(uint8_t *data)
{
        uint16_t w;
        
        for (uint16_t i = 0; i < SPM_PAGESIZE; i += 2)
        {
                w = *(data++);
                w |= *(data++) << 8;
                boot_page_fill(i, w);
        }
}

void Flash_ReadFlashPage(uint8_t *data, uint32_t addr)
{
        for (uint16_t i = 0; i < SPM_PAGESIZE; i++)
        {
                data[i] = PGM_READ_BYTE(addr++);
        }
}

void Flash_ProgramPage(uint32_t page, uint8_t *buf, uint8_t erase)
{
        uint16_t i;
        
        eeprom_busy_wait ();
        
        if (erase)
        {
                boot_page_erase (page);
                boot_spm_busy_wait ();
        }
        
        for (i=0; i<SPM_PAGESIZE; i+=2)
        {
                uint16_t w = *buf++;
                w += (*buf++) << 8;
                boot_page_fill (page + i, w);
        }
        
        boot_page_write(page);
        boot_spm_busy_wait();
        boot_rww_enable();
}

#endif // __AVR_XMEGA__


