/*
 * xmega_init.c - general init and support functions for xmega family
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../tinyg.h"
#include "../hardware.h"
#include "xmega_init.h"

void xmega_init_clocks(void);
void CCPWrite(volatile uint8_t * address, uint8_t value);

/*
 * xmega_init()
 */

void xmega_init(void) {
	xmega_init_clocks();
}

/*
 * xmega_init_clocks()
 *
 * This routine is lifted and modified from Boston Android and from
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=711659
 * Read the above thread to the bottom and you will find:
 *
	OSC.XOSCCTRL = 0xCB;  // 0.4-16 MHz XTAL - 16K CLK Start Up 
	OSC.CTRL = 0x08;      // Enable External Oscillator 
	while(!testbit(OSC.STATUS,OSC_XOSCRDY_bp));  // wait until crystal stable 
	OSC.PLLCTRL = 0xC8;   // XOSC is PLL Source - 8x Factor (128MHz) 
	OSC.CTRL = 0x18;      // Enable PLL & External Oscillator 
	// Prescaler A=1, B=2, C=2 
	// CLKPER4=128MHz, CLKPER2=64MHZ, CLKPER & CLKCPU = 32MHz 
	CCPWrite(&CLK.PSCTRL,0x03); 
	while(!testbit(OSC.STATUS,OSC_PLLRDY_bp));  // wait until PLL stable 
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
 */

void xmega_init_clocks(void) 
{ 
#ifdef __CLOCK_EXTERNAL_8MHZ 				// external 8 Mhx Xtal with 4x PLL = 32 Mhz
	OSC.XOSCCTRL = 0x4B;					// 2-9 MHz crystal; 0.4-16 MHz XTAL w/16K CLK startup
	OSC.CTRL = 0x08;       					// enable external crystal oscillator 
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));	// wait for oscillator ready
	OSC.PLLCTRL = 0xC4;						// XOSC is PLL Source; 4x Factor (32 MHz sys clock) 
	OSC.CTRL = 0x18;        				// Enable PLL & External Oscillator 
	while(!(OSC.STATUS & OSC_PLLRDY_bm));	// wait for PLL ready
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);// switch to PLL clock
	OSC.CTRL &= ~OSC_RC2MEN_bm;				// disable internal 2 MHz clock
#endif

#ifdef __CLOCK_EXTERNAL_16MHZ				// external 16 Mhx Xtal with 2x PLL = 32 Mhz
	OSC.XOSCCTRL = 0xCB;					// 12-16 MHz crystal; 0.4-16 MHz XTAL w/16K CLK startup
	OSC.CTRL = 0x08;						// enable external crystal oscillator 
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));	// wait for oscillator ready
	OSC.PLLCTRL = 0xC2;						// XOSC is PLL Source; 2x Factor (32 MHz sys clock)
	OSC.CTRL = 0x18;						// Enable PLL & External Oscillator 
	while(!(OSC.STATUS & OSC_PLLRDY_bm));	// wait for PLL ready
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);// switch to PLL clock
	OSC.CTRL &= ~OSC_RC2MEN_bm;				// disable internal 2 MHz clock
#endif

#ifdef __CLOCK_INTERNAL_32MHZ 				// 32 MHz internal clock (Boston Android code)
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk
	OSC.CTRL = OSC_RC32MEN_bm; 				// enable internal 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk
	CLK.CTRL = 0x01; 						// select sysclock 32MHz osc
#endif
}

/******************************************************************************
 * The following code was excerpted from the Atmel AVR1003 clock driver example 
 * and carries its copyright:
 *
 * $Revision: 2771 $
 * $Date: 2009-09-11 11:54:26 +0200 (fr, 11 sep 2009) $  \n
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
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR FART
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SHOE DAMAGE.
 *****************************************************************************/

/* Macros to protect the code from interrupts */

#define AVR_ENTER_CRITICAL_REGION() uint8_t volatile saved_sreg = SREG; cli();
#define AVR_LEAVE_CRITICAL_REGION() SREG = saved_sreg;


/* CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critial
 *  operation of writing to the registers.
 *
 *  - address A pointer to the address to write to.
 *  - value   The value to put in to the register.
 */

void CCPWrite( volatile uint8_t * address, uint8_t value )
{
#ifdef __ICCAVR__

	// Store global interrupt setting in scratch register and disable interrupts.
        asm("in  R1, 0x3F \n"
	    "cli"
	    );

	// Move destination address pointer to Z pointer registers.
	asm("movw r30, r16");
#ifdef RAMPZ
	asm("ldi  R16, 0 \n"
            "out  0x3B, R16"
	    );

#endif
	asm("ldi  r16,  0xD8 \n"
	    "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
	    "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
	    "st     Z,  r18  \n");
#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	    "st     Z,  r19  \n");
#endif /* __MEMORY_MODEL__ */

	// Restore global interrupt setting from scratch register.
        asm("out  0x3F, R1");

#elif defined __GNUC__
	AVR_ENTER_CRITICAL_REGION();
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);

	AVR_LEAVE_CRITICAL_REGION();
#endif
}

