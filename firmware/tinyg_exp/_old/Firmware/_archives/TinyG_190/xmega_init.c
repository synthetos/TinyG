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
}


/*
 * xmega_init_clocks()
 *
 * This routine is lifted from the Atmel AVR1003 app note example code, and ref:
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=711659
 * Read this thread to the bottom and you will find:
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
	OSC.XOSCCTRL = 0x4B;	// 2-9 MHz crystal; 0.4-16 MHz XTAL - 16K CLK Start Up 
	OSC.CTRL = 0x08;        // Enable External Oscillator 
	while (!CLKSYS_IsReady(OSC_XOSCRDY_bm));	// stabilize oscillator
	OSC.PLLCTRL = 0xC4;		// XOSC is PLL Source; 4x Factor (32 MHz) 
	OSC.CTRL = 0x18;        // Enable PLL & External Oscillator 
	while (!CLKSYS_IsReady(OSC_PLLRDY_bm));		// stabilize PLL
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
	CLKSYS_Disable(OSC_RC2MEN_bm);				// disable internal 2 MHz clock
}

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


