/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA PMIC driver example source.
 *
 *      This file contains an example application that demonstrates the PMIC
 *      driver. It includes som simple setup code for Timer/Counter C and uses
 *      three compare match interrupts very close to each other to demonstrate
 *      how different interrupt levels interact.
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
 *****************************************************************************/
#include "avr_compiler.h"
#include "pmic_driver.h"

/*! \brief Counter incremented in Compare Match C handler. */
volatile uint8_t compareMatchCCount = 0;

/*! \brief Counter incremented in Compare Match B handler. */
volatile uint8_t compareMatchBCount = 0;

/*! \brief Counter incremented in Compare Match A handler. */
volatile uint8_t compareMatchACount = 0;



/*! \brief Timer Counter C0 Compare/Capture A.
 *
 *  This routine increments compareMatchACount each time it is called.
 */
ISR(TCC0_CCA_vect)
{
	++compareMatchACount;
}



/*! \brief Timer Counter C0 Compare/Capture B.
 *
 *  This ISR increments compareMatchBCount each time it is called.
 */
ISR(TCC0_CCB_vect)
{
	++compareMatchBCount;
}



/*! \brief Timer Counter C0 Compare/Capture C.
 *
 *  This routine increments compareMatchCCount each time it is called.
 */
ISR(TCC0_CCC_vect)
{
	++compareMatchCCount;
}



/*! \brief Example function for the PMIC.
 *
 *  This function illustrates the use of the XMEGA PMIC.
 *
 *  In this example, a counter(Timer/Counter C0) is set to count continuously to
 *  0xff. Three different compare match values are programmed to give
 *  three different types of interrupts, with different interrupt levels.
 *
 *  Compare match A (0x60) triggers a low level interrupt.
 *  Compare macth B (0x50) triggers a medium level interrupt.
 *  Compare macth C (0x70) triggers a high level interrupt.
 *
 *  When setup is complete, the main program enters an infinite loop, and the
 *  interrupt handling and counter values can be observed.
 */
int main( void )
{
	/* Enable all interrupt levels. */
	PMIC_SetVectorLocationToApplication();
	PMIC_EnableLowLevel();
	PMIC_EnableMediumLevel();
	PMIC_EnableHighLevel();
	sei();

	/* Set up Timer/Counter 0 with three Compare Match interrupts. */
	TCC0.CTRLB = TC0_CCCEN_bm | TC0_CCBEN_bm | TC0_CCAEN_bm | TC_WGMODE_NORMAL_gc;
	TCC0.INTCTRLB = (uint8_t) TC_CCCINTLVL_HI_gc | TC_CCBINTLVL_MED_gc | TC_CCAINTLVL_LO_gc;
	TCC0.PER = 0xff; /* Period: 0xff */
	TCC0.CCA = 0x60; /* Compare Match A, will need to wait for both Compare Match C and B. */
	TCC0.CCB = 0x50; /* Compare Match B, will be interrupted by higher level Compare Match C. */
	TCC0.CCC = 0x70; /* Compare Match C. */

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; /* Run at maximum speed. */

	while(true) {
	}
}
