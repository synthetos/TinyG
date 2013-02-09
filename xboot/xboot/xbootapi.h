/************************************************************************/
/* XBoot Extensible AVR Bootloader API                                  */
/*                                                                      */
/* xbootapi.h                                                           */
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

#ifndef __XBOOTAPI_H
#define __XBOOTAPI_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#ifndef __AVR_XMEGA__
#include <avr/wdt.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// defines
// offsets and addresses
#ifndef PROGMEM_SIZE
#define PROGMEM_SIZE (FLASHEND + 1UL)
#endif

#ifndef BOOT_SECTION_SIZE
#error BOOT_SECTION_SIZE not defined!
#endif

#ifndef BOOT_SECTION_START
#define BOOT_SECTION_START (PROGMEM_SIZE - BOOT_SECTION_SIZE)
#endif

#ifndef APP_SECTION_START
#define APP_SECTION_START 0
#endif

#ifndef APP_SECTION_SIZE
#define APP_SECTION_SIZE (PROGMEM_SIZE - BOOT_SECTION_SIZE)
#endif

#ifndef APP_SECTION_END
#define APP_SECTION_END (APP_SECTION_START + APP_SECTION_SIZE - 1UL)
#endif

#define JUMP_TABLE_LOCATION (BOOT_SECTION_START + _VECTORS_SIZE)
#define JUMP_TABLE_INDEX(k) (JUMP_TABLE_LOCATION + 4UL + 2UL * (k))

#define XB_APP_START APP_SECTION_START
#define XB_APP_SIZE (APP_SECTION_SIZE / 2UL)
#define XB_APP_END (XB_APP_START + XB_APP_SIZE - 1UL)
#define XB_APP_TEMP_START (XB_APP_END + 1UL)
#define XB_APP_TEMP_SIZE XB_APP_SIZE
#define XB_APP_TEMP_END (XB_APP_TEMP_START + XB_APP_TEMP_SIZE - 1UL)

// status codes
#define XB_SUCCESS 0
#define XB_ERR_NO_API 1
#define XB_ERR_NOT_FOUND 2
#define XB_INVALID_ADDRESS 3

// jump table struct
struct xboot_jump_table_s {
        uint8_t id[3];
        uint8_t ver;
        uint16_t ptr[];
};

// Functions

// General Functions
uint8_t xboot_get_version(uint16_t *ver);
uint8_t xboot_get_api_version(uint8_t *ver);

// Low level flash access
uint8_t xboot_spm_wrapper(void);
uint8_t xboot_erase_application_page(uint32_t address);
uint8_t xboot_write_application_page(uint32_t address, uint8_t *data, uint8_t erase);
#ifdef __AVR_XMEGA__
uint8_t xboot_write_user_signature_row(uint8_t *data);
#endif // __AVR_XMEGA__

// Higher level firmware update functions
uint8_t xboot_app_temp_erase(void);
uint8_t xboot_app_temp_write_page(uint32_t addr, uint8_t *data, uint8_t erase);
uint8_t xboot_app_temp_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);
uint8_t xboot_app_temp_crc16(uint16_t *crc);
uint8_t xboot_app_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);
uint8_t xboot_app_crc16(uint16_t *crc);
uint8_t xboot_install_firmware(uint16_t crc);
void __attribute__ ((noreturn)) xboot_reset(void);

#ifdef __cplusplus
}
#endif

#endif // __XBOOTAPI_H

