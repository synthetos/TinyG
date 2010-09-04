/********************************************************************************
 *
 * XMEGA EEPROM driver source file.
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

/* TinyG Notes:
 *  Modified to support Xmega family processors
 *  Modifications Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * The Grbl EEPROM issue. It's totally different in xmegas
 *	ref: http://old.nabble.com/xmega-support-td21322852.html
 *		 http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=84542&start=0
 *
 * Need to:
 *	- replace eeprom.c/.h with eeprom_xmega.c/.h
 *	- the above implements the Grbl eeprom calls by wrapping xmega 
 *		calls provided in App note AVR 1315 code example: eeprom_driver.c/.h
 *	- alternately wait for AVR GCC to get memory mapped eeprom IO working (i.e. address 0x1000)
 */

#include "xmega_eeprom.h"

/****** Functions from Grbl eeprom.c ******/

char eeprom_get_char( unsigned int addr )
{
	uint8_t pageAddr = (uint8_t)(addr / EEPROM_PAGESIZE);
	uint8_t byteAddr = (uint8_t)(addr % EEPROM_PAGESIZE);
	return (EEPROM_ReadByte( pageAddr, byteAddr ));
}

void eeprom_put_char( unsigned int addr, char value )
{
	uint8_t pageAddr = (uint8_t)(addr / EEPROM_PAGESIZE);
	uint8_t byteAddr = (uint8_t)(addr % EEPROM_PAGESIZE);

	EEPROM_WriteByte(pageAddr, byteAddr, value );
}

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}


/****** Functions from Atmel eeprom_driver.c ******/

/* Write one byte to EEPROM using IO mapping.
 *
 *  This function writes one byte to EEPROM using IO-mapped access.
 *  If memory mapped EEPROM is enabled, this function will not work.
 *  This functiom will cancel all ongoing EEPROM page buffer loading
 *  operations, if any.
 *
 *  \param  pageAddr  EEPROM Page address, between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 *  \param  byteAddr  EEPROM Byte address, between 0 and EEPROM_PAGESIZE.
 *  \param  value     Byte value to write to EEPROM.
 */
void EEPROM_WriteByte( uint8_t pageAddr, uint8_t byteAddr, uint8_t value )
{
	/*  Flush buffer to make sure no unintetional data is written and load
	 *  the "Page Load" command into the command register.
	 */
	EEPROM_FlushBuffer();
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

	/* Calculate address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE)|(byteAddr & (EEPROM_PAGESIZE-1));

	/* Set address to write to. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Load data to write, which triggers the loading of EEPROM page buffer. */
	NVM.DATA0 = value;

	/*  Issue EEPROM Atomic Write (Erase&Write) command. Load command, write
	 *  the protection signature and execute command.
	 */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();
}

/* Read one byte from EEPROM using IO mapping.
 *
 *  This function reads one byte from EEPROM using IO-mapped access.
 *  If memory mapped EEPROM is enabled, this function will not work.
 *
 *  \param  pageAddr  EEPROM Page address, between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 *  \param  byteAddr  EEPROM Byte address, between 0 and EEPROM_PAGESIZE.
 *
 *  \return  Byte value read from EEPROM.
 */
uint8_t EEPROM_ReadByte( uint8_t pageAddr, uint8_t byteAddr )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Calculate address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE)
	                            |(byteAddr & (EEPROM_PAGESIZE-1));

	/* Set address to read from. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Read command. */
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;
	NVM_EXEC();

	return NVM.DATA0;
}


/* Wait for any NVM access to finish, including EEPROM.
 *
 *  This function is blcoking and waits for any NVM access to finish,
 *  including EEPROM. Use this function before any EEPROM accesses,
 *  if you are not certain that any previous operations are finished yet,
 *  like an EEPROM write.
 */
void EEPROM_WaitForNVM( void )
{
	do {
		/* Block execution while waiting for the NVM to be ready. */
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}


/* Flush temporary EEPROM page buffer.
 *
 *  This function flushes the EEPROM page buffers. This function will cancel
 *  any ongoing EEPROM page buffer loading operations, if any.
 *  This function also works for memory mapped EEPROM access.
 *
 *  \note The EEPROM write operations will automatically flush the buffer for you.
 */
void EEPROM_FlushBuffer( void )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Flush EEPROM page buffer if necessary. */
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}


/* Load single byte into temporary page buffer.
 *
 *  This function loads one byte into the temporary EEPROM page buffers.
 *  If memory mapped EEPROM is enabled, this function will not work.
 *  Make sure that the buffer is flushed before starting to load bytes.
 *  Also, if multiple bytes are loaded into the same location, they will
 *  be ANDed together, thus 0x55 and 0xAA will result in 0x00 in the buffer.
 *
 *  \note Only one page buffer exist, thus only one page can be loaded with
 *        data and programmed into one page. If data needs to be written to
 *        different pages, the loading and writing needs to be repeated.
 *
 *  \param  byteAddr  EEPROM Byte address, between 0 and EEPROM_PAGESIZE.
 *  \param  value     Byte value to write to buffer.
 */
void EEPROM_LoadByte( uint8_t byteAddr, uint8_t value )
{
	/* Wait until NVM is not busy and prepare NVM command.*/
	EEPROM_WaitForNVM();
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

	/* Set address. */
	NVM.ADDR0 = byteAddr & 0xFF;
	NVM.ADDR1 = 0x00;
	NVM.ADDR2 = 0x00;

	/* Set data, which triggers loading of EEPROM page buffer. */
	NVM.DATA0 = value;
}


/* Load entire page into temporary EEPROM page buffer.
 *
 *  This function loads an entire EEPROM page from an SRAM buffer to
 *  the EEPROM page buffers. If memory mapped EEPROM is enabled, this
 *  function will not work. Make sure that the buffer is flushed before
 *  starting to load bytes.
 *
 *  \note Only the lower part of the address is used to address the buffer.
 *        Therefore, no address parameter is needed. In the end, the data
 *        is written to the EEPROM page given by the address parameter to the
 *        EEPROM write page operation.
 *
 *  \param  values   Pointer to SRAM buffer containing an entire page.
 */
void EEPROM_LoadPage( const uint8_t * values )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();
	NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

	/*  Set address to zero, as only the lower bits matters. ADDR0 is
	 *  maintained inside the loop below.
	 */
	NVM.ADDR1 = 0x00;
	NVM.ADDR2 = 0x00;

	/* Load multible bytes into page buffer. */
	for (uint8_t i = 0; i < EEPROM_PAGESIZE; ++i) {
		NVM.ADDR0 = i;
		NVM.DATA0 = *values;
		++values;
	}
}

/* Write already loaded page into EEPROM.
 *
 *  This function writes the contents of an already loaded EEPROM page
 *  buffer into EEPROM memory.
 *
 *  As this is an atomic write, the page in EEPROM will be erased
 *  automatically before writing. Note that only the page buffer locations
 *  that have been loaded will be used when writing to EEPROM. Page buffer
 *  locations that have not been loaded will be left untouched in EEPROM.
 *
 *  \param  pageAddr  EEPROM Page address, between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */
void EEPROM_AtomicWritePage( uint8_t pageAddr )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Calculate page address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);

	/* Set address. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Atomic Write (Erase&Write) command. */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();
}


/* Erase EEPROM page.
 *
 *  This function erases one EEPROM page, so that every location reads 0xFF.
 *
 *  \param  pageAddr  EEPROM Page address, between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */
void EEPROM_ErasePage( uint8_t pageAddr )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Calculate page address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);

	/* Set address. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Erase command. */
	NVM.CMD = NVM_CMD_ERASE_EEPROM_PAGE_gc;
	NVM_EXEC();
}


/* Write (without erasing) EEPROM page.
 *
 *  This function writes the contents of an already loaded EEPROM page
 *  buffer into EEPROM memory.
 *
 *  As this is a split write, the page in EEPROM will _not_ be erased
 *  before writing.
 *
 *  \param  pageAddr  EEPROM Page address, between 0 and EEPROM_SIZE/EEPROM_PAGESIZE
 */
void EEPROM_SplitWritePage( uint8_t pageAddr )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Calculate page address */
	uint16_t address = (uint16_t)(pageAddr*EEPROM_PAGESIZE);

	/* Set address. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0x1F;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Split Write command. */
	NVM.CMD = NVM_CMD_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();
}

/* Erase entire EEPROM memory.
 *
 *  This function erases the entire EEPROM memory block to 0xFF.
 */
void EEPROM_EraseAll( void )
{
	/* Wait until NVM is not busy. */
	EEPROM_WaitForNVM();

	/* Issue EEPROM Erase All command. */
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
	NVM_EXEC();
}

