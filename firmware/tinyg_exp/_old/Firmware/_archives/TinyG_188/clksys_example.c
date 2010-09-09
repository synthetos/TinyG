/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA Clock System driver example source.
 *
 *      This file contains an example application that demonstrates the Clock
 *      System driver. The recommended setup for this demonstration is to connect
 *      an external 2-9MHz crystal to the XTAL1/XTAL2 pins with proper decoupling.
 *
 *
 * \par Application note:
 *      AVR1003: Using the XMEGA Clock System
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
#include "clksys_driver.h"

/* The LED to use for visual feedback. */
#define LEDPORT PORTD
#define LEDMASK 0xFF

/* Which switches to listen to */
#define SWITCHPORT PORTC
#define SWITCHMASK 0xFF


/* Prototype function. */
void WaitForSwitches( void );


/*! \brief Example code that switch between different clocks
 *
 *  This example code shows how to change between 5 different system clocks
 *  and how to prescale and divide the clocks to make use of dynamic
 *  clocking for the different parts on the device. When changes are
 *  made to the clocks, blocking functions are used to make sure the
 *  clocks are stable before they can be used.
 */
int main( void )
{
	/* Set up user interface. */
	LEDPORT.DIRSET = LEDMASK;
	LEDPORT.OUTSET = LEDMASK;
	SWITCHPORT.DIRCLR = SWITCHMASK;

	/* Set up Timer/Counter 0 to work from CPUCLK/64, with period 10000 and
	 * enable overflow interrupt.
	 */
	TCC0.PER = 10000;
	TCC0.CTRLA = ( TCC0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;
	TCC0.INTCTRLA = ( TCC0.INTCTRLA & ~TC0_OVFINTLVL_gm ) |
	                TC_OVFINTLVL_MED_gc;

	/* Enable low interrupt level in PMIC and enable global interrupts. */
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	sei();


	/*  This while loop run through and switch between the different
	 *  system clock sources available.
	 */
	while(1) {

		/* Wait for user input while the LEDs toggle. */
		WaitForSwitches();


		/*  Enable internal 32 MHz ring oscillator and wait until it's
		 *  stable. Divide clock by two with the prescaler C and set the
		 *  32 MHz ring oscillator as the main clock source. Wait for
		 *  user input while the LEDs toggle.
		 */
		CLKSYS_Enable( OSC_RC32MEN_bm );
		CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_2_gc );
		do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
		CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
		WaitForSwitches( );


		/* Enable for external 2-9 MHz crystal with quick startup time
		 * (256CLK). Check if it's stable and set the external
		 * oscillator as the main clock source. Wait for user input
		 * while the LEDs toggle.
		 */
		CLKSYS_XOSC_Config( OSC_FRQRANGE_2TO9_gc,
		                    false,
		                    OSC_XOSCSEL_EXTCLK_gc );
		CLKSYS_Enable( OSC_XOSCEN_bm );
		do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
		CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_XOSC_gc );
		CLKSYS_Disable( OSC_RC32MEN_bm );
		WaitForSwitches();


		/*  Divide Prescaler C by two and Prescaler C by two, and wait
		 *  for user input.
		 */
		CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_2_2_gc );
		WaitForSwitches();


		/*  Enable internal 32 kHz calibrated oscillator and check for
		 *  it to be stable and set prescaler A, B and C to none. Set
		 *  the 32 kHz oscillator as the main clock source. Wait for
		 *  user input while the LEDs toggle.
		 */
		CLKSYS_Enable( OSC_RC32KEN_bm );
		CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
		do {} while ( CLKSYS_IsReady( OSC_RC32KRDY_bm ) == 0 );
		CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32K_gc );
		CLKSYS_Disable( OSC_XOSCEN_bm );
		WaitForSwitches();


		/*  Configure PLL with the 2 MHz RC oscillator as source and
		 *  multiply by 30 to get 60 MHz PLL clock and enable it. Wait
		 *  for it to be stable and set prescaler C to divide by two
		 *  to set the CPU clock to 30 MHz. Disable unused clock and
		 *  wait for user input.
		 */
		CLKSYS_PLL_Config( OSC_PLLSRC_RC2M_gc, 30 );
		CLKSYS_Enable( OSC_PLLEN_bm );
		CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_2_gc );
		do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );
		CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
		CLKSYS_Disable( OSC_XOSCEN_bm );
		WaitForSwitches( );


		/*  Select 2 MHz RC oscillator as main clock source and diable
		 *  unused clock.
		 */
		do {} while ( CLKSYS_IsReady( OSC_RC2MRDY_bm ) == 0 );
		CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC2M_gc );
		CLKSYS_Disable( OSC_PLLEN_bm );

	}
}

/*! \brief This function waits for a button push and release before proceeding.
 */
void WaitForSwitches( void )
{
	do {} while ( ( SWITCHPORT.IN & SWITCHMASK ) == SWITCHMASK );
	delay_us( 1000 );
	do {} while ( ( SWITCHPORT.IN & SWITCHMASK ) != SWITCHMASK );
	delay_us( 1000 );
}


/*! Just toggle LED(s) when interrupt occurs. */
ISR(TCC0_OVF_vect)
{
	LEDPORT.OUTTGL = LEDMASK;
}
