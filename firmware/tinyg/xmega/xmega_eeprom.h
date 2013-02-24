/*************************************************************************
 *
 * XMEGA EEPROM driver header file.
 *
 *      This file contains the function prototypes and enumerator 
 *		definitions for various configuration parameters for the 
 *		XMEGA EEPROM driver.
 *
 *      The driver is not intended for size and/or speed critical code, 
 *		since most functions are just a few lines of code, and the 
 *		function call overhead would decrease code performance. The driver
 *		is intended for rapid prototyping and documentation purposes for 
 *		getting started with the XMEGA EEPROM module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * Notes:
 *      See AVR1315: Accessing the XMEGA EEPROM + Code eeprom_driver.c /.h
 *
 * Author: * \author
 *      Original Author: Atmel Corporation: http://www.atmel.com \n
 *		Adapted by: Alden S. Hart Jr; 04/02/2010
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products 
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY 
 * DIRECT,INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR FART (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/

#ifndef xmega_eeprom_h
#define xmega_eeprom_h

#include <avr/io.h>

/* Configuration settings */
// UNCOMMENT FOR TEST ONLY - uses a RAM block to simulate EEPROM
//#define __NNVM				// uncomment to use non-non-volatile RAM
#define NNVM_SIZE 2000 		// size of emulation RAM block - xmega192/256 has 4096

#ifndef MAPPED_EEPROM_START
#define MAPPED_EEPROM_START 0x1000
#endif //__NNVM

#define EEPROM_PAGESIZE 32				// if this changes ...change below:
#define EEPROM_BYTE_ADDR_MASK_gm 0x1F	// range of valid byte addrs in page
#define EEPROM_ADDR1_MASK_gm 0x0F		// EEPROM is 4K = 0x0F, 2K = 0x07

#define EEPROM(_pageAddr, _byteAddr) \
	((uint8_t *) MAPPED_EEPROM_START)[_pageAddr*EEPROM_PAGESIZE + _byteAddr]

/* function prototypes for TinyG added functions */
uint16_t EEPROM_WriteString(const uint16_t address, const char *buf, const uint8_t terminate);
uint16_t EEPROM_ReadString(const uint16_t address, char *buf, const uint16_t size);
uint16_t EEPROM_WriteBytes(const uint16_t address, const int8_t *buf, const uint16_t size);
uint16_t EEPROM_ReadBytes(const uint16_t address, int8_t *buf, const uint16_t size);

//#ifdef __UNIT_TEST_EEPROM
void EEPROM_unit_tests(void);
//#endif

/* Function prototypes for Atmel and Atmel-derived functions */
uint8_t EEPROM_ReadByte(uint16_t address);
void EEPROM_WriteByte(uint16_t address, uint8_t value);
void EEPROM_WaitForNVM( void );
void EEPROM_FlushBuffer( void );
void EEPROM_LoadByte( uint8_t byteAddr, uint8_t value );
void EEPROM_LoadPage( const uint8_t * values );
void EEPROM_AtomicWritePage( uint8_t pageAddr );
void EEPROM_ErasePage( uint8_t pageAddress );
void EEPROM_SplitWritePage( uint8_t pageAddr );
void EEPROM_EraseAll( void );

/* Some MACRO Definitions */

// Note: the ByPage macros rely on pagesize = 32, and are as yet untested
#define EEPROM_ReadChar (char)EEPROM_ReadByte
#define EEPROM_ReadByteByPage(p,b) EEPROM_ReadByte( (p<<5) | (b) )
#define EEPROM_ReadCharByPage(p,b) (char)EEPROM_ReadByte( (p<<5) | (b) )
#define EEPROM_WriteByteByPage(p,b,v) EEPROM_ReadByte( ((p<<5) | (b)), v )

#ifndef max
#define max(a, b) (((a)>(b))?(a):(b))
#endif
#ifndef min
#define min(a, b) (((a)<(b))?(a):(b))
#endif

/* Enable EEPROM block sleep-when-not-used mode.
 *
 *	This macro enables power reduction mode for EEPROM. It means that 
 *	the EEPROM block is disabled when not used. Note that there will be 
 *	a penalty of 6 CPU cycles if EEPROM is accessed.
 */
#define EEPROM_EnablePowerReduction() ( NVM.CTRLB |= NVM_EPRM_bm )

/* Disable EEPROM block sleep-when-not-used mode.
 *
 *  This macro disables power reduction mode for EEPROM.
 */
#define EEPROM_DisablePowerReduction() ( NVM.CTRLB &= ~NVM_EPRM_bm )

/* Enable EEPROM mapping into data space.
 *
 *	This macro enables mapping of EEPROM into data space. EEPROM starts at 
 *	EEPROM_START in data memory. Read access can be done similar to ordinary 
 *	SRAM access.
 *
 *  Note: This disables IO-mapped access to EEPROM, although page erase and
 *        write operations still needs to be done through IO register.
 */
#define EEPROM_EnableMapping() ( NVM.CTRLB |= NVM_EEMAPEN_bm )

/* Disable EEPROM mapping into data space.
 *
 *  This macro disables mapping of EEPROM into data space.
 *  IO mapped access is now enabled.
 */
#define EEPROM_DisableMapping() ( NVM.CTRLB &= ~NVM_EEMAPEN_bm )

/* 
 * NVM_EXEC() - Non-Volatile Memory Execute Command (MACRO)
 *
 *  This macro sets the CCP register before setting the CMDEX bit in the
 *  NVM.CTRLA register. The CMDEX bit must be set within 4 clock cycles 
 *	after setting the protection byte in the CCP register.
 */
/*
#define NVM_EXEC()  asm("push r30"	"\n\t"	\
			    "push r31"			"\n\t"	\
    			"push r16"			"\n\t"	\
    			"push r18"			"\n\t"	\
			    "ldi r30, 0xCB"		"\n\t"	\
			    "ldi r31, 0x01"		"\n\t"	\
			    "ldi r16, 0xD8"		"\n\t"	\
			    "ldi r18, 0x01"		"\n\t"	\
			    "out 0x34, r16"		"\n\t"	\
			    "st Z, r18"			"\n\t"	\
    			"pop r18"			"\n\t"	\
			    "pop r16"			"\n\t"	\
			    "pop r31"			"\n\t"	\
			    "pop r30"			"\n\t"	\
			    )
*/
/* ------------------------------------------------------------------------
 * Modified version from jl_1978 - Feb 01, 2010 - 06:30 PM in thread:
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=87793&start=0
 * 
 * "This enables a high level interrupt when the EEPROM transaction is 
 * complete which wakes the CPU back up. In your .c eeprom write/erase 
 * functions, set the flag to enable sleep mode: 
 *
 *	SLEEP.CTRL = 1; 
 *
 * Oh yeah, and disable other interrupts. My system only uses low and 
 * medium interrupts for other tasks. The high interrupt is only used 
 * for the stuff above."
 */
/*
#define NVM_EXEC() asm("push r30" "\n\t"	\
				"push r31" 		"\n\t"	\
				"push r16" 		"\n\t"	\
				"push r18" 		"\n\t"	\
				"ldi r30, 0xCB" "\n\t"	\
				"ldi r31, 0x01" "\n\t"	\
				"ldi r16, 0xD8" "\n\t"	\
				"ldi r18, 0x01" "\n\t"	\
				"out 0x34, r16" "\n\t"	\
				"st Z, r18"	 	"\n\t"	\
				"ldi r30, 0xCD" "\n\t"	\
				"ldi r31, 0x01" "\n\t"	\
				"ldi r18, 0x0C" "\n\t"	\
				"st Z, r18"	 	"\n\t"	\
				"sleep"	 		"\n\t"	\
				"pop r18" 		"\n\t"	\
				"pop r16" 		"\n\t"	\
				"pop r31" 		"\n\t"	\
				"pop r30" 		"\n\t"	\
				)

*/

//#define __UNIT_TEST_EEPROM
#ifdef __UNIT_TEST_EEPROM
void EEPROM_unit_tests(void);
#define	EEPROM_UNITS EEPROM_unit_tests();
#else
#define	EEPROM_UNITS
#endif

#endif

