/*******************************************************************************
 *
 * XMEGA EEPROM driver header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the XMEGA EEPROM driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA EEPROM module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * Notes:
 *      See AVR1315: Accessing the XMEGA EEPROM + Code eeprom_driver.c / .h
 *
 * Author: * \author
 *      Original Author: Atmel Corporation: http://www.atmel.com \n
 *		Adapted by: Alden S. Hart Jr; 04/02/2010
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/*
 * Ref: ATMEL AVR10__ app note and code
 * Contains some minor mods by ASH to adapt to GCC: search on "(ash mod)"
 */

#ifndef xmega_eeprom_h
#define xmega_eeprom_h

#include <avr/io.h>			// Was #include "avr_compiler.h"  (ash mod)

#ifndef MAPPED_EEPROM_START
#define MAPPED_EEPROM_START 0x1000
#endif

#define EEPROM_PAGESIZE 32
#define EEPROM(_pageAddr, _byteAddr) \
	((uint8_t *) MAPPED_EEPROM_START)[_pageAddr*EEPROM_PAGESIZE + _byteAddr]


/* Function prototypes for Atmel functions. */
void EEPROM_WriteByte( uint8_t pageAddr, uint8_t byteAddr, uint8_t value );
uint8_t EEPROM_ReadByte( uint8_t pageAddr, uint8_t byteAddr );
void EEPROM_WaitForNVM( void );
void EEPROM_FlushBuffer( void );
void EEPROM_LoadByte( uint8_t byteAddr, uint8_t value );
void EEPROM_LoadPage( const uint8_t * values );
void EEPROM_AtomicWritePage( uint8_t pageAddr );
void EEPROM_ErasePage( uint8_t pageAddress );
void EEPROM_SplitWritePage( uint8_t pageAddr );
void EEPROM_EraseAll( void );

/* function prototypes for Grbl functions */

char eeprom_get_char(unsigned int addr);
void eeprom_put_char(unsigned int addr, char new_value);
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);


/* Definitions of macros. */

/* Enable EEPROM block sleep-when-not-used mode.
 *
 *  This macro enables power reduction mode for EEPROM.
 *  It means that the EEPROM block is disabled when not used.
 *  Note that there will be a penalty of 6 CPU cycles if EEPROM
 *  is accessed.
 */
#define EEPROM_EnablePowerReduction() ( NVM.CTRLB |= NVM_EPRM_bm )

/* Disable EEPROM block sleep-when-not-used mode.
 *
 *  This macro disables power reduction mode for EEPROM.
 */
#define EEPROM_DisablePowerReduction() ( NVM.CTRLB &= ~NVM_EPRM_bm )

/* Enable EEPROM mapping into data space.
 *
 *  This macro enables mapping of EEPROM into data space.
 *  EEPROM starts at EEPROM_START in data memory. Read access
 *  can be done similar to ordinary SRAM access.
 *
 *  \note This disables IO-mapped access to EEPROM, although page erase and
 *        write operations still needs to be done through IO register.
 */
#define EEPROM_EnableMapping() ( NVM.CTRLB |= NVM_EEMAPEN_bm )

/* Disable EEPROM mapping into data space.
 *
 *  This macro disables mapping of EEPROM into data space.
 *  IO mapped access is now enabled.
 */
#define EEPROM_DisableMapping() ( NVM.CTRLB &= ~NVM_EEMAPEN_bm )

/* Non-Volatile Memory Execute Command
 *
 *  This macro set the CCP register before setting the CMDEX bit in the
 *  NVM.CTRLA register.
 *
 *  \note The CMDEX bit must be set within 4 clock cycles after setting the
 *        protection byte in the CCP register.
 */
#define NVM_EXEC()  asm("push r30"      "\n\t"	\
			    "push r31"      "\n\t"	\
    			"push r16"      "\n\t"	\
    			"push r18"      "\n\t"	\
			    "ldi r30, 0xCB" "\n\t"	\
			    "ldi r31, 0x01" "\n\t"	\
			    "ldi r16, 0xD8" "\n\t"	\
			    "ldi r18, 0x01" "\n\t"	\
			    "out 0x34, r16" "\n\t"	\
			    "st Z, r18"	    "\n\t"	\
    			"pop r18"       "\n\t"	\
			    "pop r16"       "\n\t"	\
			    "pop r31"       "\n\t"	\
			    "pop r30"       "\n\t"	\
			    )
#endif
