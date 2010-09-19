/* Xmega Non Volatile Memory functions 
 *
 * Ahem. Before you waste a day trying to figure out why none of this works
 * in the simulator, you should realize that IT DOESN'T WORK IN THE WINAVR
 * SIMULATOR (now I'll calm down now.) 
 */
/***************************************************************************
 *
 * XMEGA EEPROM driver source file.
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
 *      a function call (or just inline the functions, bonehead).
 *
 *		Besides which, it doesn't work in the simulator, so how would
 *		you ever know?
 *
 * Notes:
 *      See AVR1315: Accessing the XMEGA EEPROM + Code eeprom_driver.c /.h
 *
 * Author: * \author
 *      Original Author: Atmel Corporation: http://www.atmel.com
 *		Adapted by: Alden S. Hart Jr; 2010
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
 **************************************************************************/

/* 
 * Modified to support Xmega family processors
 * Modifications Copyright (c) 2010 Alden S. Hart, Jr.
 */

#include "xmega_eeprom.h"
#include <string.h>
#include <avr/interrupt.h>
#include "tinyg.h"		// only used to define __UNIT_TESTS. can be removed.

#ifdef __UNIT_TESTS
#include <stdio.h>
#include <string.h>						// for memset()
#include <avr/pgmspace.h>
#include "xio.h"						// all device includes are nested here
#endif

/****** Functions added for TinyG ******/

// workaround for EEPROM not working in Simulator2
//#define __TEST_EEPROM_WRITE			// comment out for production 

#ifdef __TEST_EEPROM_WRITE
	uint8_t testbuffer[32];			// fake out the page buffer
#endif

/* 
 * EEPROM_WriteString() - write string to EEPROM; may span multiple pages
 *
 *	This function writes a character string to IO mapped EEPROM. 
 *	If memory mapped EEPROM is enabled this function will not work. 
 *	This functiom will cancel all ongoing EEPROM page buffer loading 
 *	operations, if any.
 *
 *	A string may span multiple EEPROM pages. For each affected page it:
 *		- loads the page buffer from EEPROM with the contents for that page
 *		- writes the string bytes for that page into the page buffer
 *		- performs an atomic write for that page (erase and write)
 *		- does the next page until the string is complete.
 *
 *	If 'terminate' is TRUE, add a null to the string, if FALSE, don't.
 *
 *	Note that only the page buffer locations that have been copied into
 *	the page buffer (during string copy) will be affected when writing to 
 *	the EEPROM (using AtomicPageWrite). Page buffer locations that have not 
 *	been loaded will be left untouched in EEPROM.
 *
 *	  address 	 must be between 0 and top-of-EEPROM (0x0FFF in a 256 or 192)
 *	  string 	 must be null-terminated
 *	  terminate	 set TRUE to write string termination NULL to EEPROM
 *				 set FALSE to not write NULL (useful for writing "files")
 *
 *	  returns pointer to next EEPROM location past the last byte written
 *
 *	Background: The xmega EEPROM is rated at 100,000 operations (endurance).
 *	The endurance figure is dominated by the erase operation. Reads do not
 *	affect the endurance and writes have minimal effect. Erases occur at 
 *	the page level, so a strategy to minimize page erases is needed. This
 *	function is a reasonably high endurance solution since erases only occur
 *	once for each time a string crosses a page, as opposed to once per byte
 *	written (as in EEPROM_WriteByte()). A slightly higher endurance solution 
 *	would be to store up multiple contiguous string writes and perform them 
 *	as single page operations.
 */

uint16_t EEPROM_WriteString(uint16_t address, char *string, uint8_t terminate)
{
	uint8_t i = 0;			// index into string
	uint16_t curaddr;		// starting address of string remaining to write
	uint16_t endaddr;		// ending address, adjusted for termination
	uint16_t strnlen;		// remaining unwritten string length (zero based)
	uint8_t curpage;		// current page number (0 - 127)
	uint8_t endpage;		// ending page number  (0 - 127)
	uint8_t byteidx;		// index into page
	uint8_t byteend;		// ending byte number in page

	// initialize variables
	curaddr = address;
	strnlen = strlen(string) + terminate - 1;	// terminate will be 1 or 0
	endaddr = address + strnlen;
	curpage = (curaddr >> 5) & 0x7F;			// mask it just to be safe
	endpage = (endaddr >> 5) & 0x7F;			// mask it just to be safe

	while (curpage <= endpage) {
		// initialize addresses and variables for this write page
		byteidx = curaddr & EEPROM_BYTE_ADDR_MASK_gm;
		byteend = min((byteidx + strnlen), EEPROM_PAGESIZE-1);
		strnlen = strnlen - (byteend - byteidx) - 1;// chars left in string
		curaddr = curaddr + (byteend - byteidx) + 1;// bump current address
		NVM.ADDR1 = curpage++;			 			// set upper addr bytes
		NVM.ADDR2 = 0x00;

		// load page buffer w/string contents and optional NULL termination
		EEPROM_FlushBuffer();	// ensure no unintentional data is written
		NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc; // Page Load command
		while (byteidx <= byteend) {
#ifdef __TEST_EEPROM_WRITE
			NVM.ADDR0 = byteidx;
			testbuffer[byteidx++] = string[i++];
#else
			// use EEPROM (the real code)
			NVM.ADDR0 = byteidx++;	// set buffer location for data
			NVM.DATA0 = string[i++];// writing DATA0 triggers write to buffer
#endif
		}
		// run EEPROM Atomic Write (Erase&Write) command.
		NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;	// load command
		NVM_EXEC();	//write the protection signature and execute command
	}
	return (curaddr);
}

/* 
 * EEPROM_ReadString() - read string from EEPROM; may span multiple pages
 *
 *	This function reads a character string to IO mapped EEPROM. 
 *	If memory mapped EEPROM is enabled this function will not work. 
 *	A string may span multiple EEPROM pages. 
 *
 *		address		starting address of string in EEPROM space
 *		buf			buffer to read string into
 *		max_len		cutoff string and terminate at this length 
 *		return		next address past string termination
 */

uint16_t EEPROM_ReadString(uint16_t address, char *buf, uint16_t max_len)
{
#ifdef __TEST_EEPROM_WRITE
	uint8_t j = address & 0x1F;
#endif
	uint16_t local_addr = address;
	uint16_t i = 0;

	for (i = 0; i < max_len; i++) {
		NVM.ADDR0 = local_addr & 0xFF;		// set read address
		NVM.ADDR1 = (local_addr++ >> 8) & EEPROM_ADDR1_MASK_gm;
		NVM.ADDR2 = 0x00;

		EEPROM_WaitForNVM();				// Wait until NVM is not busy
		NVM.CMD = NVM_CMD_READ_EEPROM_gc;	// issue EEPROM Read command
		NVM_EXEC();
#ifdef __TEST_EEPROM_WRITE
		if (!(buf[i] = testbuffer[j++])) {
			break;
		}
#else
		if (!(buf[i] = NVM.DATA0)) {
			break;
		}		
#endif
	}
	if (i == max_len) {		// null terinate the buffer overflow case
		buf[i] = 0;
	}
	return local_addr;
}

/****** Functions from Atmel eeprom_driver.c (and some derived from them) ******/

/* 
 * EEPROM_WaitForNVM() - Wait for any NVM access to finish
 *
 *  This function blocks waiting for any NVM access to finish including EEPROM
 *  Use this function before any EEPROM accesses if you are not certain that 
 *	any previous operations are finished yet, like an EEPROM write.
 */

inline void EEPROM_WaitForNVM( void )
{
	do {
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}

/* 
 * EEPROM_FlushBuffer() - Flush temporary EEPROM page buffer.
 *
 *  This function flushes the EEPROM page buffers, and will cancel
 *  any ongoing EEPROM page buffer loading operations, if any.
 *  This function also works for memory mapped EEPROM access.
 *
 *  Note: The EEPROM write operations will automatically flush the buffer for you
 */

inline void EEPROM_FlushBuffer( void )
{
	EEPROM_WaitForNVM();						// Wait until NVM is not busy
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) { 	// Flush page buffer if necessary
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}

/* 
 * EEPROM_ReadByte() 	   - Read one byte from EEPROM using IO mapping.
 * EEPROM_ReadChar() 	   - Read one char from EEPROM using IO mapping.
 * EEPROM_ReadByteByPage() - Read one byte using page addressing
 * EEPROM_ReadCharByPage() - Read one char using page addressing
 *
 *  This function reads one byte from EEPROM using IO-mapped access.
 *  If memory mapped EEPROM is enabled, this function will not work.
 *	The other funcs are defined as macros. See the .h file.
 *
 *	  address  	EEPROM address, between 0 and EEPROM_SIZE
 *	  returns 	byte value read from EEPROM.
 */

uint8_t EEPROM_ReadByte(uint16_t address)
{
	EEPROM_WaitForNVM();				// Wait until NVM is not busy
	NVM.ADDR0 = address & 0xFF;			// set read address
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;	// issue EEPROM Read command
	NVM_EXEC();
	return NVM.DATA0;
}

/* 
 * EEPROM_WriteByte() 		- write one byte to EEPROM using IO mapping
 * EEPROM_WriteByteByPage() - write one byte using page addressing (MACRO)
 *
 *	This function writes one byte to EEPROM using IO-mapped access.
 *	If memory mapped EEPROM is enabled this function will not work.
 *  This function flushes the EEPROM page buffers, and will cancel
 *  any ongoing EEPROM page buffer loading operations, if any.
 *
 *	  address  	EEPROM address, between 0 and EEPROM_SIZE
 *	  value     Byte value to write to EEPROM.
 *
 *	Note: DO NOT USE THIS FUNCTION IF YOU CAN AVOID IT AS ENDURANCE SUCKS
 *		  Use EEPROM_WriteString(), or write a new routine for binary blocks.
 */

inline void EEPROM_WriteByte(uint16_t address, uint8_t value)
{
	EEPROM_FlushBuffer();		// make sure no unintentional data is written
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;	// Page Load command
	NVM.ADDR0 = address & 0xFF;					// set write address
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.DATA0 = value;	// load write data - triggers EEPROM page buffer load
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;// Atomic Write (Erase&Write)
	NVM_EXEC(); 	// Load command, write protection signature & exec command
}

/* 
 * EEPROM_LoadByte() - Load single byte into temporary page buffer.
 *
 *  This function loads one byte into the temporary EEPROM page buffers.
 *  Make sure that the buffer is flushed before starting to load bytes.
 *  Also, if multiple bytes are loaded into the same location, they will
 *  be ANDed together, thus 0x55 and 0xAA will result in 0x00 in the buffer.
 *  If memory mapped EEPROM is enabled this function will not work.
 *
 *  Note: Only one page buffer exists, thus only one page can be loaded with
 *        data and programmed into one page. If data needs to be written to
 *        different pages the loading and writing needs to be repeated.
 *
 *    byteAddr  EEPROM Byte address, between 0 and EEPROM_PAGESIZE.
 *    value     Byte value to write to buffer.
 */

inline void EEPROM_LoadByte(uint8_t byteAddr, uint8_t value)
{
	EEPROM_WaitForNVM(); 						// wait until NVM is not busy
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;	// prepare NVM command
	NVM.ADDR0 = byteAddr & EEPROM_ADDR1_MASK_gm;// set address
	NVM.ADDR1 = 0x00;
	NVM.ADDR2 = 0x00;
	NVM.DATA0 = value; // Set data, which triggers loading EEPROM page buffer
}

/* 
 * EEPROM_LoadPage() - Load entire page into temporary EEPROM page buffer.
 *
 *  This function loads an entire EEPROM page from an SRAM buffer to
 *  the EEPROM page buffers. 
 *	Make sure that the buffer is flushed before starting to load bytes. 
 *	If memory mapped EEPROM is enabled this function will not work.
 *
 *  Note: Only the lower part of the address is used to address the buffer.
 *        Therefore, no address parameter is needed. In the end, the data
 *        is written to the EEPROM page given by the address parameter to the
 *        EEPROM write page operation.
 *
 *    values   Pointer to SRAM buffer containing an entire page.
 */

inline void EEPROM_LoadPage( const uint8_t * values )
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;
	NVM.ADDR1 = 0x00;							// set upper addr's to zero
	NVM.ADDR2 = 0x00;

	for (uint8_t i = 0; i < EEPROM_PAGESIZE; ++i) { // load multiple bytes
		NVM.ADDR0 = i;
		NVM.DATA0 = *values;
		++values;
	}
}

/* 
 * EEPROM_AtomicWritePage() - Write already loaded page into EEPROM.
 *
 *	This function writes the contents of an already loaded EEPROM page
 *	buffer into EEPROM memory.  As this is an atomic write, the page in 
 *	EEPROM will be erased automatically before writing. Note that only the 
 *	page buffer locations that have been loaded will be used when writing 
 *	to EEPROM. Page buffer locations that have not been loaded will be left
 *	untouched in EEPROM.
 *
 *	  pageAddr  EEPROM Page address between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */

inline void EEPROM_AtomicWritePage(uint8_t pageAddr)
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);
	NVM.ADDR0 = address & 0xFF;					// set addresses
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc; // erase & write page command
	NVM_EXEC();
}

/* 
 * EEPROM_ErasePage() - Erase EEPROM page.
 *
 *  This function erases one EEPROM page, so that every location reads 0xFF.
 *
 *	  pageAddr  EEPROM Page address between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */

inline void EEPROM_ErasePage( uint8_t pageAddr )
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);
	NVM.ADDR0 = address & 0xFF;					// set addresses
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.CMD = NVM_CMD_ERASE_EEPROM_PAGE_gc;		// erase page command
	NVM_EXEC();
}

/* 
 * EEPROM_SplitWritePage() - Write (without erasing) EEPROM page.
 *
 *  This function writes the contents of an already loaded EEPROM page
 *  buffer into EEPROM memory. As this is a split write, the page in 
 *	EEPROM will *NOT* be erased before writing.
 *
 *	  pageAddr  EEPROM Page address between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */

inline void EEPROM_SplitWritePage( uint8_t pageAddr )
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);
	NVM.ADDR0 = address & 0xFF;					// set addresses
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.CMD = NVM_CMD_WRITE_EEPROM_PAGE_gc;		// split write command
	NVM_EXEC();
}

/* 
 * EEPROM_EraseAll() - Erase entire EEPROM memory to 0xFF
 */

inline void EEPROM_EraseAll( void )
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;			// erase all command
	NVM_EXEC();
}

/*
 *	EEPROM Unit tests
 */

#ifdef __UNIT_TESTS

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

void _EEPROM_test_write(void);
void _EEPROM_test_write_and_read(void);

void EEPROM_tests()
{
	_EEPROM_test_write();
	_EEPROM_test_write_and_read();
}

void _EEPROM_test_write_and_read()
{
	uint16_t address = 0;
	char tbuf[16];

	EEPROM_WriteString(address, "0123456789", TRUE);	// 10 chars + termination
	EEPROM_ReadString(address, tbuf, 16);
	printf("%s\n", tbuf);
}

void _EEPROM_test_write()
{
	// write fits easily in page 0, starts at 0, not terminated (return 0x06)
	EEPROM_WriteString(0x00, "0123\n", FALSE);

	// write fits easily in page 0, starts at 1, not terminated
	EEPROM_WriteString(0x01, "0123\n", FALSE);

	// write fits easily in page 0, starts at 2, terminated
	EEPROM_WriteString(0x02, "01234567\n", TRUE);

	// write overflows page 0, starts at 0x1C, terminated (return 0x26)
	EEPROM_WriteString(0x1C, "01234567\n", TRUE);

	// write fills page 1 completely, terminated (return 0x40)
	EEPROM_WriteString(0x20, "0123456789abcdefghijklmnopqrst\n", TRUE);

	// write fills page 1 completely, spills into page 2, terminated (return 0x46)
	EEPROM_WriteString(0x20, "0123456789abcdefghijklmnopqrstuvwxyz\n", TRUE);

	// write fills page 1 and 2 completely, spills into page 3, terminated (return 0x68)
	EEPROM_WriteString(0x20, "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ<>{}[]()\n", TRUE);
}

#endif
