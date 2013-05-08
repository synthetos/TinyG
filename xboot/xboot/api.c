/************************************************************************/
/* XBoot Extensible AVR Bootloader API                                  */
/*                                                                      */
/* api.c                                                                */
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

#include "api.h"

// jump table

#ifdef ENABLE_API


#if USE_API_VERSION == 1

// Version 1
// XBj\x1
struct xboot_jump_table_s api_jump_table __attribute((section(".vectors"))) = {
        {'X', 'B', 'j'}, 1,
        {
                // General Functions
                (uint16_t)(xboot_get_version),
                
                // Low level flash access
                #ifdef ENABLE_API_LOW_LEVEL_FLASH
                #ifdef ENABLE_API_SPM_WRAPPER
                (uint16_t)(xboot_spm_wrapper),
                #else // ENABLE_API_SPM_WRAPPER
                0,
                #endif // ENABLE_API_SPM_WRAPPER
                (uint16_t)(xboot_erase_application_page),
                (uint16_t)(xboot_write_application_page),
                (uint16_t)(xboot_write_user_signature_row),
                #else // ENABLE_API_LOW_LEVEL_FLASH
                0,
                0,
                0,
                0,
                #endif // ENABLE_API_LOW_LEVEL_FLASH
                
                // Higher level firmware update functions
                #ifdef ENABLE_API_FIRMWARE_UPDATE
                (uint16_t)(xboot_app_temp_erase),
                (uint16_t)(xboot_app_temp_write_page),
                #else // ENABLE_API_FIRMWARE_UPDATE
                0,
                0,
                #endif // ENABLE_API_FIRMWARE_UPDATE
        }
};

#endif // USE_API_VERSION

#endif // ENABLE_API

// General Functions
uint8_t xboot_get_version(uint16_t *ver)
{
        *ver = (XBOOT_VERSION_MAJOR << 8) | (XBOOT_VERSION_MINOR);
        return XB_SUCCESS;
}

// Low level flash access
uint8_t xboot_spm_wrapper(void)
{
        return XB_ERR_NOT_FOUND;
}

uint8_t xboot_erase_application_page(uint32_t address)
{
        uint8_t saved_status = SREG;
        
        if (address > BOOT_SECTION_START)
                return XB_INVALID_ADDRESS;
        
        cli();
        
        Flash_EraseApplicationPage(address);
        Flash_WaitForSPM();        
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        SREG = saved_status;
        return XB_SUCCESS;
}

uint8_t xboot_write_application_page(uint32_t address, uint8_t *data, uint8_t erase)
{
        uint8_t saved_status = SREG;
        
        if (address > BOOT_SECTION_START)
                return XB_INVALID_ADDRESS;
        
        cli();   
        Flash_ProgramPage(address, data, erase);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        SREG = saved_status;
        return XB_SUCCESS;
}

#ifdef __AVR_XMEGA__
uint8_t xboot_write_user_signature_row(uint8_t *data)
{
        uint8_t saved_status = SREG;
        cli();
        
        Flash_LoadFlashPage(data);
        Flash_EraseUserSignatureRow();
        Flash_WaitForSPM();
        Flash_WriteUserSignatureRow();
        Flash_WaitForSPM();
        
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        
        SREG = saved_status;
        return XB_SUCCESS;
}
#endif // __AVR_XMEGA__

// Higher level firmware update functions
uint8_t xboot_app_temp_erase(void)
{
        uint8_t saved_status = SREG;
        cli();
        
        for (uint32_t addr = XB_APP_TEMP_START; addr < XB_APP_TEMP_END; addr += SPM_PAGESIZE)
        {
                Flash_EraseApplicationPage(addr);
                Flash_WaitForSPM();
        }
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
        SREG = saved_status;
        return XB_SUCCESS;
}

uint8_t xboot_app_temp_write_page(uint32_t addr, uint8_t *data, uint8_t erase)
{
        return xboot_write_application_page(addr + XB_APP_TEMP_START, data, erase);
}


