/*
 * xmega_init.c - general init and support functions for xmega family
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * Notes:
 *	- add full interrupt tables and dummy interrupt routine
 *	- add crystal oscillator setup and failover
 *	- add watchdog timer functions
 *
 */

#include <avr/io.h>
#include "avr_compiler.h"
#include "xmega_clksys.h"

#include "xmega_init.h"

void xmega_init_clocks(void);


/*
 * xmega_init()
 */

void xmega_init(void) {
	xmega_init_clocks();
//	config32MHzInternalClock();				// set system clock to 32 MHz, internal
}


/*
 * xmega_init_clocks()
 *
 * This routine is lifted from the Atmel AVR1003 app note example code
 *
 * First:
 *	Enable internal 32 MHz ring oscillator and wait until it's stable. 
 *	Set the 32 MHz ring oscillator as the main clock source.
 *	Disable the 2 MHz internal oscillator
 *
 * Next:
 *	Enable external oscillator for 2-9 MHz crystal with quick startup time (256CLK) 
 *	Wait until it's stable
 *	Set the PLL to 4x for 32 Mhz operation from the 8 MHz crystal
 *	Set the external oscillator as the main clock source.
 *
 * Headscratcher:
 *	The external clock enabled bit never sets. 
 *	If you uncomment it the routine hangs.
 *	So the CPU never actually switches to the external clock as it won't enable
 *	a clock that's not working.
 */

void xmega_init_clocks(void) 
{ 
	// do the inernal 32 MHz ring oscillator
	CLKSYS_Enable(OSC_RC32MEN_bm);
	while (!CLKSYS_IsReady(OSC_RC32MRDY_bm));
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);
	CLKSYS_Disable(OSC_RC2MEN_bm);

	OSC.XOSCCTRL  = 0x40;	// 2-9 MHz crystal
	OSC.XOSCCTRL |= 0x07;	// 0.4-16 MHz XTAL - 1K CLK Start Up 

	OSC.PLLCTRL  = 0xC0;	// XOSC is PLL Source 
	OSC.PLLCTRL |= 0x04;	// 4x Factor (32 MHz) 

	OSC.CTRL = 0x18;        // Enable PLL & External Oscillator 

	// Prescaler A=1, B=2, C=2 
	// CLKPER4=128MHz, CLKPER2=64MHZ, CLKPER & CLKCPU = 32MHz 
//	CCPWrite(&CLK.PSCTRL,0x03);	// prescaler to 1 
//	while(!testbit(OSC.STATUS,OSC_PLLRDY_bp));  // wait until PLL stable 
	while (!CLKSYS_IsReady(OSC_PLLRDY_bm));	// never returns 
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
	CLKSYS_Disable(OSC_RC32MEN_bm);				// Disable internal 32 MHz clock

/*

	OSC.XOSCCTRL = 0xC7;    // 0.4-16 MHz XTAL - 1K CLK Start Up 
	OSC.PLLCTRL = 0xC8;     // XOSC is PLL Source - 8x Factor (128MHz) 
	OSC.CTRL = 0x18;        // Enable PLL & External Oscillator 
	// Prescaler A=1, B=2, C=2 
	// CLKPER4=128MHz, CLKPER2=64MHZ, CLKPER & CLKCPU = 32MHz 
	CCPWrite(&CLK.PSCTRL,0x03); 
//	while(!testbit(OSC.STATUS,OSC_PLLRDY_bp));  // wait until PLL stable 
	while (!CLKSYS_IsReady(OSC_PLLRDY_bm));	// never returns 
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
	CLKSYS_Disable(OSC_RC32MEN_bm);				// Disable internal 32 MHz clock

*/


/*

	// do the external 8 MHz crystal and 4x PLL (using the 16K clock)
//	CLKSYS_XOSC_Config(OSC_FRQRANGE_2TO9_gc, false, OSC_XOSCSEL_XTAL_16KCLK_gc);
	CLKSYS_XOSC_Config(OSC_FRQRANGE_2TO9_gc, false, OSC_XOSCSEL_XTAL_256CLK_gc);
	while (!CLKSYS_IsReady(OSC_XOSCRDY_bm));	// wait for osc to stabilize
	CLKSYS_PLL_Config(OSC_PLLSRC_XOSC_gc, 4);
	while (!CLKSYS_IsReady(OSC_PLLRDY_bm));	// never returns 


	CLKSYS_Enable(OSC_XOSCEN_bm);
	while (!CLKSYS_IsReady(OSC_XOSCRDY_bm));// never returns 
//	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_XOSC_gc);
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_XOSC_gc);
	CLKSYS_Disable(OSC_RC32MEN_bm);
*/
}

/*
	CLKSYS_XOSC_Config(OSC_FRQRANGE_2TO9_gc, false, OSC_XOSCSEL_EXTCLK_gc);
	CLKSYS_Enable(OSC_XOSCEN_bm);
	while (!CLKSYS_IsReady(OSC_XOSCRDY_bm));// never returns 
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_XOSC_gc);
	CLKSYS_PLL_Config(CLK_SCLKSEL_XOSC_gc, 4);
	CLKSYS_Disable(OSC_RC32MEN_bm);
*/

/* Configure 32 MHz clock (xmega) */
void config32MHzInternalClock(void) 
{
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk 

	// initialize clock source to be 32MHz internal oscillator (no PLL)
	OSC.CTRL = OSC_RC32MEN_bm; 				// enable internal 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk
	CLK.CTRL = 0x01; 						// select sysclock 32MHz osc
};

void config32MHzExternalClock(void) 
{
	CCP = CCP_IOREG_gc;  					// Security Signature to modify clk

	// initialize clock source to be 32MHz external oscillator
	OSC.CTRL = OSC_XOSCEN_bm; 				// Enable external OSC
	//OSC.XOSCCTRL = OSC_XOSCSEL_EXTCLK_gc;
	OSC.XOSCCTRL = OSC_FRQRANGE_9TO12_gc | OSC_XOSCSEL_EXTCLK_gc;  //Set freq range.
   	//OSC.CTRL |= OSC_XOSCEN_bm; 
   	while (!(OSC.STATUS & OSC_XOSCRDY_bm));	//Wait for xternal osc to go stable?? Never does?
	CCP = CCP_IOREG_gc;

   	CLK.CTRL = 0x03; 						//Set external clock. However never gets here.. 
};



