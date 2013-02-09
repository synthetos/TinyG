/************************************************************************/
/* XBoot Extensible AVR Bootloader API                                  */
/*                                                                      */
/* xbootapi.c                                                           */
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

#include "xbootapi.h"

// defines
#if PROGMEM_SIZE > 0x020000
#define NEED_EIND
#endif

#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#define PGM_READ_WORD pgm_read_word_far
#define PGM_READ_DWORD pgm_read_dword_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#define PGM_READ_WORD pgm_read_word_near
#define PGM_READ_DWORD pgm_read_dword_near
#endif

// globals
uint8_t api_version = 0;

uint8_t init_api(void)
{
        if (api_version > 0)
                return XB_SUCCESS;
        
        struct xboot_jump_table_s jp;
        
        *((uint32_t *)(&jp)) = PGM_READ_DWORD(JUMP_TABLE_LOCATION);
        
        if ((jp.id[0] == 'X') && (jp.id[1] == 'B') && (jp.id[2] == 'j'))
        {
                api_version = jp.ver;
                return XB_SUCCESS;
        }
        
        return XB_ERR_NO_API;
}

// General Functions
uint8_t xboot_get_version(uint16_t *ver)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(0));
                if (ptr == 0 || ptr == 0xffff)
                        return XB_ERR_NOT_FOUND;
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(uint16_t *)) ptr )(ver);
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_get_api_version(uint8_t *ver)
{
        uint8_t ret = init_api();
        if (ret != XB_SUCCESS)
                return ret;
        
        *ver = api_version;
        return XB_SUCCESS;
}


// Low level flash access
uint8_t xboot_spm_wrapper(void)
{
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_erase_application_page(uint32_t address)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(2));
                if (ptr == 0 || ptr == 0xffff)
                        return XB_ERR_NOT_FOUND;
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(uint32_t)) ptr )(address);
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_write_application_page(uint32_t address, uint8_t *data, uint8_t erase)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(3));
                if (ptr == 0 || ptr == 0xffff)
                        return XB_ERR_NOT_FOUND;
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(uint32_t, uint8_t *, uint8_t)) ptr )(address, data, erase);
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}

#ifdef __AVR_XMEGA__
uint8_t xboot_write_user_signature_row(uint8_t *data)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(4));
                if (ptr == 0 || ptr == 0xffff)
                        return XB_ERR_NOT_FOUND;
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(uint8_t *)) ptr )(data);
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}
#endif // __AVR_XMEGA__


// Higher level firmware update functions
uint8_t xboot_app_temp_erase(void)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(5));
                if (ptr == 0 || ptr == 0xffff)
                {
                        for (uint32_t addr = XB_APP_TEMP_START; addr < XB_APP_TEMP_END; addr += SPM_PAGESIZE)
                        {
                                ret = xboot_erase_application_page(addr);
                                if (ret != XB_SUCCESS)
                                        break;
                        }
                        return ret;
                }
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(void)) ptr )();
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_app_temp_write_page(uint32_t addr, uint8_t *data, uint8_t erase)
{
        uint8_t ret = init_api();
        uint16_t ptr;
        
        #ifdef NEED_EIND
        uint8_t saved_eind;
        #endif // NEED_EIND
        
        if (ret != XB_SUCCESS)
                return ret;
        
        if (api_version == 1)
        {
                ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(6));
                if (ptr == 0 || ptr == 0xffff)
                {
                        ret = xboot_write_application_page(addr + XB_APP_TEMP_START, data, erase);
                        return ret;
                }
                
                #ifdef NEED_EIND
                saved_eind = EIND;
                EIND = PROGMEM_SIZE >> 17;
                #endif // NEED_EIND
                
                ret = ( (uint8_t(*)(uint32_t, uint8_t *, uint8_t)) ptr )(addr, data, erase);
                
                #ifdef NEED_EIND
                EIND = saved_eind;
                #endif // NEED_EIND
                
                return ret;
        }
        
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_app_temp_crc16_block(uint32_t start, uint32_t length, uint16_t *crc)
{
        return xboot_app_crc16_block(XB_APP_TEMP_START + start, length, crc);
}

uint8_t xboot_app_temp_crc16(uint16_t *crc)
{
        return xboot_app_temp_crc16_block(0, XB_APP_TEMP_SIZE, crc);
}

uint8_t xboot_app_crc16_block(uint32_t start, uint32_t length, uint16_t *crc)
{
        uint16_t _crc = 0;
        uint8_t b;
        
        for (uint32_t i = 0; i < length; i++)
        {
                b = PGM_READ_BYTE(start++);
                _crc = _crc16_update(_crc, b);
        }
        
        *crc = _crc;
        
        return XB_SUCCESS;
}

uint8_t xboot_app_crc16(uint16_t *crc)
{
        return xboot_app_crc16_block(0, XB_APP_SIZE, crc);
}

uint8_t xboot_install_firmware(uint16_t crc)
{
        uint8_t buffer[SPM_PAGESIZE];
        
        for (uint16_t i = 0; i < SPM_PAGESIZE; i++)
        {
                buffer[i] = PGM_READ_BYTE(XB_APP_TEMP_START + XB_APP_TEMP_SIZE - SPM_PAGESIZE + i);
        }
        
        buffer[SPM_PAGESIZE-6] = 'X';
        buffer[SPM_PAGESIZE-5] = 'B';
        buffer[SPM_PAGESIZE-4] = 'I';
        buffer[SPM_PAGESIZE-3] = 'F';
        buffer[SPM_PAGESIZE-2] = (crc >> 8) & 0xff;
        buffer[SPM_PAGESIZE-1] = crc & 0xff;
        
        return xboot_app_temp_write_page(XB_APP_TEMP_SIZE - SPM_PAGESIZE, buffer, 1);
}

void __attribute__ ((noreturn)) xboot_reset(void)
{
        // disable interrupts
        cli();
        
        // reset chip
        #ifdef __AVR_XMEGA__
        // can do this directly on xmega
        CCP = CCP_IOREG_gc;
        RST.CTRL = RST_SWRST_bm;
        #else // __AVR_XMEGA__
        // need to force a watchdog reset on atmega
        wdt_disable();  
        wdt_enable(WDTO_15MS);
        #endif // __AVR_XMEGA__
        
        // don't return
        while (1) { };
}





