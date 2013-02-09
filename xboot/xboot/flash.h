/************************************************************************/
/* Generic ATMEL Flash Driver                                           */
/*                                                                      */
/* flash.h                                                              */
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

#ifndef __FLASH_H
#define __FLASH_H

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef __AVR_XMEGA__
#include "sp_driver.h"
#else // __AVR_XMEGA__
#include <avr/boot.h>
#endif // __AVR_XMEGA__

#include "xboot.h"

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

#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#define PGM_READ_WORD pgm_read_word_far
#define PGM_READ_DWORD pgm_read_dword_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#define PGM_READ_WORD pgm_read_word_near
#define PGM_READ_DWORD pgm_read_dword_near
#endif



#ifdef __AVR_XMEGA__

// XMega functions
// (sp_driver wrapper)

#define Flash_ReadByte SP_ReadByte
#define Flash_ReadWord SP_ReadWord
#define Flash_LoadFlashWord SP_LoadFlashWord
#define Flash_EraseApplicationSection SP_EraseApplicationSection
#define Flash_EraseApplicationPage SP_EraseApplicationPage
#define Flash_EraseWriteApplicationPage SP_EraseWriteApplicationPage
#define Flash_WriteApplicationPage SP_WriteApplicationPage
#define Flash_EraseUserSignatureRow SP_EraseUserSignatureRow
#define Flash_WriteUserSignatureRow SP_WriteUserSignatureRow
#define Flash_LoadFlashPage SP_LoadFlashPage
#define Flash_ReadFlashPage SP_ReadFlashPage
#define Flash_WaitForSPM SP_WaitForSPM

#else

// ATMega Functions

#define Flash_ReadByte PGM_READ_BYTE
#define Flash_ReadWord PGM_READ_WORD
#define Flash_LoadFlashWord boot_page_fill
void Flash_EraseApplicationSection(void);
#define Flash_EraseApplicationPage boot_page_erase
void Flash_EraseWriteApplicationPage(uint32_t addr);
#define Flash_WriteApplicationPage boot_page_write
void Flash_LoadFlashPage(uint8_t *data);
void Flash_ReadFlashPage(uint8_t *data, uint32_t addr);
#define Flash_WaitForSPM boot_spm_busy_wait

#endif // __AVR_XMEGA__

void Flash_ProgramPage(uint32_t page, uint8_t *buf, uint8_t erase);


#endif // __FLASH_H
