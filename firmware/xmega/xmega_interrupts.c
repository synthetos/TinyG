/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA PMIC driver header file.
 *
 *      This file contains the function implementations for the XMEGA
 *      Programmable Multi-level Interrupt Controller driver/example.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA PMIC module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * \par Application note:
 *      AVR1305: XMEGA Interrupts and the Programmable Multi-level Interrupt Controller
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
 *
 ******************************************************************************/
#include "xmega_interrupts.h"

/*! \brief Move interrupt vector table to boot area.
 *
 *  This function moves the interrupt vector table to boot area.
 *  The function writes the correct signature to the Configuration
 *  Change Protection register before writing the CTRL register. Interrupts are
 *  automatically ignored during the change enable period.
 */
void PMIC_SetVectorLocationToBoot( void )
{
	uint8_t temp = PMIC.CTRL | PMIC_IVSEL_bm;
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = temp;
}

/*! \brief Move interrupt vector table to application area.
 *
 *  This function moves the interrupt vector table to application area.
 *  The function writes the correct signature to the Configuration
 *  Change Protection register before writing the CTRL register. Interrupts are
 *  automatically ignored during the change enable period.
 */
void PMIC_SetVectorLocationToApplication( void )
{
	uint8_t temp = PMIC.CTRL & ~PMIC_IVSEL_bm;
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = temp;
}
