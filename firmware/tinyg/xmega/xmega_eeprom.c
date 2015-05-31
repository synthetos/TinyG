/* Xmega Non Volatile Memory functions
 *
 * Ahem. Before you waste days trying to figure out why none of this works
 * in the simulator, you should realize that IT DOESN'T WORK IN THE WINAVR
 * SIMULATOR (now I'll calm down now.).
 *
 * Then before you waste more time figuring out why it doesn't work AT ALL,
 * realize that there is a serious bug in the xmega A3 processor family.
 * This is documented in Atmel release note AVR1008 and in the chip Errata.
 * This file contains workarounds to those problems.
 *
 * Some code was incorporated from the avr-xboot project:
 *	https://github.com/alexforencich/xboot
 *	http://code.google.com/p/avr-xboot/wiki/Documentation
 *
 * These refs were also helpful:
 *	http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=669385
 *	http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=88416
 *	http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=89810&start=0
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
 * Author:
 * \author
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

#include <stdbool.h>
#include "xmega_eeprom.h"
#include <string.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "../tinyg.h"				// only used to define __UNIT_TEST_EEPROM. can be removed.

#ifdef __UNIT_TEST_EEPROM
#include <stdio.h>
#include <avr/pgmspace.h>			// precursor for xio.h
#include <string.h>					// for memset()
#include <avr/pgmspace.h>
#include "xio.h"					// all device includes are nested here
#endif

#define __USE_AVR1008_EEPROM		// use the AVR1008 workaround code
#define ARBITRARY_MAX_LENGTH 80		// string max for NNVM write

/**** Inline assembly to support NVM operations ****/

static inline void NVM_EXEC(void)
{
	void *z = (void *)&NVM_CTRLA;

	__asm__ volatile("out %[ccp], %[ioreg]"  "\n\t"
	"st z, %[cmdex]"
	:
	: [ccp] "I" (_SFR_IO_ADDR(CCP)),
	[ioreg] "d" (CCP_IOREG_gc),
	[cmdex] "r" (NVM_CMDEX_bm),
		[z] "z" (z)
                     );
}

/**** AVR1008 fixes ****/

#ifdef __USE_AVR1008_EEPROM

//Interrupt handler for for EEPROM write "done" interrupt

ISR(NVM_EE_vect)
{
	NVM.INTCTRL = (NVM.INTCTRL & ~NVM_EELVL_gm); // Disable EEPROM interrupt
}

// Wrapper for NVM_EXEC that executes the workaround code

static inline void NVM_EXEC_WRAPPER(void)
{
	uint8_t sleepCtr = SLEEP.CTRL;			// Save the Sleep register
        									// Set sleep mode to IDLE
	SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP.CTRL) | SLEEP_SMODE_IDLE_gc;
	uint8_t statusStore = PMIC.STATUS;		// Save the PMIC Status...
	uint8_t pmicStore = PMIC.CTRL;			//...and control registers
        									// Enable only  hi interrupts
	PMIC.CTRL = (PMIC.CTRL & ~(PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm)) | PMIC_HILVLEN_bm;
	uint8_t globalInt = SREG;				// Save SREG for later use
	sei();									// Enable global interrupts
	SLEEP.CTRL |= SLEEP_SEN_bm;				// Set sleep enabled
	uint8_t eepromintStore = NVM.INTCTRL;	// Save eeprom int settings
	NVM_EXEC();								// exec EEPROM command
	NVM.INTCTRL = NVM_EELVL0_bm | NVM_EELVL1_bm;// Enable EEPROM int
	sleep_cpu();							// Sleep before 2.5uS passed
	SLEEP.CTRL = sleepCtr;					// Restore sleep settings
	PMIC.STATUS = statusStore;				// Restore PMIC status...
	PMIC.CTRL = pmicStore;					//...and control registers
	NVM.INTCTRL = eepromintStore;			// Restore EEPROM int settings
	SREG = globalInt;						// Restore global int settings
}
#else
#define NVM_EXEC_WRAPPER NVM_EXEC
#endif // __USE_AVR1008_EEPROM


/*************************************************************************
 ****** Functions added for TinyG ****************************************
 *************************************************************************/

/*
 * Non Non-Volatile Memory RAM array &
 * functions to substitute for EEPROM for testing.
 */

#ifdef __NNVM
char nnvm[NNVM_SIZE]; // not non-volatile memory - emulate xmega192 or 256 has 4096

void NNVM_WriteString(const uint16_t address, const char *buf, const uint8_t unused);
void NNVM_ReadString(const uint16_t address, char *buf, const uint8_t size);
void NNVM_WriteBytes(const uint16_t address, const int8_t *buf, const uint16_t size);
void NNVM_ReadBytes(const uint16_t address, int8_t *buf, const uint16_t size);

void NNVM_WriteString(const uint16_t address, const char *buf, const uint8_t unused)
{
	uint16_t j = address;		// NNVM pointer

	for (uint16_t i = 0; i < ARBITRARY_MAX_LENGTH; i++) {
		nnvm[j++] = buf[i];
		if (!buf[i]) {
			return;
		}
	}
}

void NNVM_ReadString(const uint16_t address, char *buf, const uint8_t size)
{
	uint16_t j = address;		// NNVM pointer

	for (uint16_t i = 0; i < size; i++) {
		buf[i] = nnvm[j++];
		if (!buf[i]) {
			return;
		}
	}
}

void NNVM_WriteBytes(const uint16_t address, const int8_t *buf, const uint16_t size)
{
	uint16_t j = address;		// NNVM pointer

	for (uint16_t i = 0; i < size; i++) {
		nnvm[j++] = buf[i];
	}
	return;
}

void NNVM_ReadBytes(const uint16_t address, int8_t *buf, const uint16_t size)
{
	uint16_t j = address;		// NNVM pointer

	for (uint16_t i = 0; i < size; i++) {
		buf[i] = nnvm[j++];
	}
	return;
}


#endif // __NNVM

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

uint16_t EEPROM_WriteString(const uint16_t address, const char *buf, const uint8_t terminate)
{
#ifdef __NNVM
	NNVM_WriteString(address, buf, true);
	return (address);
#else
	uint16_t addr = address;	// local copy
	uint8_t i = 0;				// index into string

	EEPROM_DisableMapping();
	while (buf[i]) {
		EEPROM_WriteByte(addr++, buf[i++]);
	}
	if (terminate) {
		EEPROM_WriteByte(addr++, 0);
	}
	return (addr); 				// return next address in EEPROM
#endif //__NNVM
}

/* //+++ this is broken and will need to be fixed to work with NNVM
#ifdef __TEST_EEPROM_WRITE
	uint8_t testbuffer[32];			// fake out the page buffer
#endif	// __TEST_EEPROM_WRITE

uint16_t EEPROM_WriteString(const uint16_t address, const char *string, const uint8_t terminate)
{
	uint8_t i = 0;			// index into string
	uint16_t curaddr;		// starting addr of string remaining to write
	uint16_t endaddr;		// ending address, adjusted for termination
	uint16_t strnlen;		// remaining unwritten string len (zero based)
	uint8_t curpage;		// current page number (0 - 127)
	uint8_t endpage;		// ending page number  (0 - 127)
	uint8_t byteidx;		// index into page
	uint8_t byteend;		// ending byte number in page

	// initialize variables
	EEPROM_DisableMapping();
	curaddr = address;
	strnlen = strlen(buf) + terminate - 1;		// terminate will be 1 or 0
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
#ifdef __NNVM
			NVM.ADDR0 = byteidx;
			testbuffer[byteidx++] = buf[i++];
#else
			// use EEPROM (the real code)
			NVM.ADDR0 = byteidx++;	// set buffer location for data
			NVM.DATA0 = buf[i++];	// writing DATA0 triggers write to buffer
#endif
		}
		// run EEPROM Atomic Write (Erase&Write) command.
		NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;	// load command
		NVM_EXEC_WRAPPER();	//write protection signature and execute cmd
	}
	return (curaddr);
}
*/

/*
 * EEPROM_ReadString() - read string from EEPROM; may span multiple pages
 *
 *	This function reads a character string to IO mapped EEPROM.
 *	If memory mapped EEPROM is enabled this function will not work.
 *	A string may span multiple EEPROM pages.
 *
 *		address		starting address of string in EEPROM space
 *		buf			buffer to read string into
 *		size		cutoff string and terminate at this length
 *		return		next address past string termination
 */

uint16_t EEPROM_ReadString(const uint16_t address, char *buf, const uint16_t size)
{
#ifdef __NNVM
	NNVM_ReadString(address, buf, size);
	return(address + sizeof(buf));
#else
	uint16_t addr = address;				// local copy
	uint16_t i = 0;							// index into strings

	EEPROM_DisableMapping();

	for (i = 0; i < size; i++) {
		NVM.ADDR0 = addr & 0xFF;			// set read address
		NVM.ADDR1 = (addr++ >> 8) & EEPROM_ADDR1_MASK_gm;
		NVM.ADDR2 = 0x00;

		EEPROM_WaitForNVM();				// Wait until NVM is not busy
		NVM.CMD = NVM_CMD_READ_EEPROM_gc;	// issue EEPROM Read command
		NVM_EXEC();
		if (!(buf[i] = NVM.DATA0)) {
			break;
		}
	}
	if (i == size) {		// null terinate the buffer overflow case
		buf[i] = 0;
	}
	return (addr);
#endif //__NNVM
}

/*
 * EEPROM_WriteBytes() - write N bytes to EEPROM; may span multiple pages
 *
 *	This function writes a byte buffer to IO mapped EEPROM.
 *	If memory mapped EEPROM is enabled this function will not work.
 *	This functiom will cancel all ongoing EEPROM page buffer loading
 *	operations, if any.
 *
 *	Returns address past the write
 */

uint16_t EEPROM_WriteBytes(const uint16_t address, const int8_t *buf, const uint16_t size)
{
#ifdef __NNVM
	NNVM_WriteBytes(address, buf, size);
	return(address + size);
#else
	uint16_t i;
	uint16_t addr = address;	// local copy

	EEPROM_DisableMapping();
	for (i=0; i<size; i++) {
		EEPROM_WriteByte(addr++, buf[i]);
	}
	return (addr); 				// return next address in EEPROM
#endif //__NNVM
}

/*
 * EEPROM_ReadBytes() - read N bytes to EEPROM; may span multiple pages
 *
 *	This function reads a character string to IO mapped EEPROM.
 *	If memory mapped EEPROM is enabled this function will not work.
 *	A string may span multiple EEPROM pages.
 */

uint16_t EEPROM_ReadBytes(const uint16_t address, int8_t *buf, const uint16_t size)
{
#ifdef __NNVM
	NNVM_ReadBytes(address, buf, size);
	return(address + size);
#else
	uint16_t i;
	uint16_t addr = address;				// local copy

	EEPROM_DisableMapping();

	for (i=0; i<size; i++) {
		NVM.ADDR0 = addr & 0xFF;			// set read address
		NVM.ADDR1 = (addr++ >> 8) & EEPROM_ADDR1_MASK_gm;
		NVM.ADDR2 = 0x00;

		EEPROM_WaitForNVM();				// Wait until NVM is not busy
		NVM.CMD = NVM_CMD_READ_EEPROM_gc;	// issue EEPROM Read command
		NVM_EXEC();
		buf[i] = NVM.DATA0;
	}
	return (addr);
#endif //__NNVM
}

/*************************************************************************
 ****** Functions from Atmel eeprom_driver.c w/some changes **************
 *************************************************************************/

// Look for NVM_EXEC_WRAPPER in places.

/*
 * EEPROM_WaitForNVM() - Wait for any NVM access to finish
 *
 *  This function blocks waiting for any NVM access to finish including EEPROM
 *  Use this function before any EEPROM accesses if you are not certain that
 *	any previous operations are finished yet, like an EEPROM write.
 */

void EEPROM_WaitForNVM( void )
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

void EEPROM_FlushBuffer( void )
{
	EEPROM_WaitForNVM();						// Wait until NVM is not busy
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) { 	// Flush page buffer if necessary
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
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

void EEPROM_WriteByte(uint16_t address, uint8_t value)
{
	EEPROM_DisableMapping();				// *** SAFETY ***
	EEPROM_FlushBuffer();					// prevent unintentional write
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;// load page_load command
	NVM.ADDR0 = address & 0xFF; 			// set buffer addresses
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.DATA0 = value;	// load write data - triggers EEPROM page buffer load
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;// Atomic Write (Erase&Write)
	NVM_EXEC_WRAPPER(); // Load command, write protection signature & exec command
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
	EEPROM_DisableMapping();				// *** SAFETY ***
	EEPROM_WaitForNVM();					// Wait until NVM is not busy
	NVM.ADDR0 = address & 0xFF;				// set read address
	NVM.ADDR1 = (address >> 8) & EEPROM_ADDR1_MASK_gm;
	NVM.ADDR2 = 0x00;
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;		// issue EEPROM Read command
	NVM_EXEC();
	return NVM.DATA0;
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

void EEPROM_LoadByte(uint8_t byteAddr, uint8_t value)
{
	EEPROM_DisableMapping();					// +++ SAFETY
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

void EEPROM_LoadPage( const uint8_t * values )
{
	EEPROM_DisableMapping();					// +++ SAFETY
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
	NVM_EXEC_WRAPPER();
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
	NVM_EXEC_WRAPPER();
}

/*
 * EEPROM_EraseAll() - Erase entire EEPROM memory to 0xFF
 */

inline void EEPROM_EraseAll( void )
{
	EEPROM_WaitForNVM();						// wait until NVM not busy
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;			// erase all command
	NVM_EXEC_WRAPPER();
}

