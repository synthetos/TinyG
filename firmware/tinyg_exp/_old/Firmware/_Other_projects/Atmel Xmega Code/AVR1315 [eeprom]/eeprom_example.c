/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA EEPROM driver example source.
 *
 *      This file contains an example application that demonstrates the EEPROM
 *      driver. It shows how to read and write to EEPROM, both in atomic
 *      mode (erase+write) and split mode (erase and/or write). Also, both IO-mapped
 *      and memory mapped access is demonstrated.
 *
 * \par Application note:
 *      AVR1315: Accessing the XMEGA EEPROM
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1569 $
 * $Date: 2008-04-22 13:03:43 +0200 (ti, 22 apr 2008) $  \n
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

#include "avr_compiler.h"
#include "eeprom_driver.h"

#define TEST_BYTE_1 0x55
#define TEST_BYTE_2 0xAA

#define TEST_BYTE_ADDR_1 0x00
#define TEST_BYTE_ADDR_2 0x08

#define TEST_PAGE_ADDR_1    0  /* Page address always on a page boundary. */
#define TEST_PAGE_ADDR_2    2  /* Page address always on a page boundary. */
#define TEST_PAGE_ADDR_3    5  /* Page address always on a page boundary. */

/*! Testbuffer to write into EEPROM. */
uint8_t testBuffer[EEPROM_PAGESIZE] = {"Accessing Atmel AVR XMEGA EEPROM"};

/*! \brief Example code writing and reading the EEPROM with different methods.
 *
 *   This example code show how to read and write the EEPROM while it is both
 *   memory mapped and IO mapped.
 *
 */
int main( void )
{
	/* Assume OK from the start. Set to 0 on first error.*/
	bool test_ok = true;

	/* Flush buffer just to be sure when we start.*/
	EEPROM_FlushBuffer();


	/* ***
	 * Write and read two bytes using IO-mapped access.
	 * ***/

	EEPROM_DisableMapping();

	/* Write bytes. */
	EEPROM_WriteByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1, TEST_BYTE_1);
	EEPROM_WriteByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2, TEST_BYTE_2);

	/* Read back and check bytes.*/
	if (EEPROM_ReadByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) != TEST_BYTE_1) {
		test_ok = false;
	}

	if (EEPROM_ReadByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) != TEST_BYTE_2) {
		test_ok = false;
	}

	/* Now write the other way round.*/
	EEPROM_WriteByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1, TEST_BYTE_2);
	EEPROM_WriteByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2, TEST_BYTE_1);

	/* Again, read back and check bytes. */
	if (EEPROM_ReadByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) != TEST_BYTE_2) {
		test_ok = false;
	}

	if (EEPROM_ReadByte(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) != TEST_BYTE_1) {
		test_ok = false;
	}

	/* ***
	 * Do a full page with split operations.
	 * ***/

	/* Load, erase and write. */
	EEPROM_LoadPage(testBuffer);
	EEPROM_ErasePage(TEST_PAGE_ADDR_2);
	EEPROM_SplitWritePage(TEST_PAGE_ADDR_2);

	/* Read back and check. */
	for (uint8_t i = 0; i < EEPROM_PAGESIZE; ++i) {
		if (EEPROM_ReadByte(TEST_PAGE_ADDR_2, i ) != testBuffer[i] ) {
			test_ok = false;
			break;
		}
	}

	/* ***
	 * Now write and read two bytes using memory mapped access.
	 * ***/

	EEPROM_EnableMapping();

	/* Write bytes.*/
	EEPROM_WaitForNVM();
	EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) = TEST_BYTE_1;
	EEPROM_AtomicWritePage(TEST_PAGE_ADDR_1);
	EEPROM_WaitForNVM();
	EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) = TEST_BYTE_2;
	EEPROM_AtomicWritePage(TEST_PAGE_ADDR_1);

	/* Read back and check. */
	EEPROM_WaitForNVM();
	if (EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) != TEST_BYTE_1) {
		test_ok = false;
	}
	if (EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) != TEST_BYTE_2) {
		test_ok = false;
	}


	/* ***
	 * Test the other way round with memory mapped access too.
	 * ***/

	/* Write bytes. */
	EEPROM_WaitForNVM();
	EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) = TEST_BYTE_2;
	EEPROM_AtomicWritePage(TEST_PAGE_ADDR_1);
	EEPROM_WaitForNVM();
	EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) = TEST_BYTE_1;
	EEPROM_AtomicWritePage(TEST_PAGE_ADDR_1);

	/* Read back and check.	*/
	EEPROM_WaitForNVM();
	if (EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_1) != TEST_BYTE_2) {
		test_ok = false;
	}

	if (EEPROM(TEST_PAGE_ADDR_1, TEST_BYTE_ADDR_2) != TEST_BYTE_1) {
		test_ok = false;
	}

	/* ***
	 * Do a full page write with split operations and memory mapped access.
	 * ***/

	/* Load buffer, erase and write. */
	EEPROM_WaitForNVM();
	for (uint8_t i = 0; i < EEPROM_PAGESIZE; ++i) {
		EEPROM(TEST_PAGE_ADDR_3, i) = testBuffer[i];
	}

	/*  Erase bytes in eeprom page corresponding to page buffer. The page
	 *  buffer will not be flushed.
	 */
	EEPROM_ErasePage(TEST_PAGE_ADDR_3);

	/*  Split write page buffer to eeprom page. Buffer will be flushed after
	 *  write operation.
	 */
	EEPROM_SplitWritePage(TEST_PAGE_ADDR_3);

	/* Read back and check.	*/
	EEPROM_WaitForNVM();
	for (uint8_t i = 0; i < EEPROM_PAGESIZE; ++i) {
		if (EEPROM(TEST_PAGE_ADDR_3, i) != testBuffer[i] ) {
			test_ok = false;
			break;
		}
	}

	/* ***
	 * Now, check if everything went well.
	 * ***/
	if (true == test_ok) {
		while(1) {
			/* Success. */
			nop();
		}
	} else {
		while(1) {
			/* Failure. */
			nop();
		}
	}
}
