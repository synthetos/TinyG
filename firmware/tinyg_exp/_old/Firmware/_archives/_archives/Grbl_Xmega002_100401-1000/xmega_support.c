/*
  xmega_support.c - support for Grbl xmega port 
  Part of Grbl xmega port

  Copyright (c) 2010 Alden S. Hart Jr.

Notes:

	- add full interrupt tables and dummy interrupt routine
	- add crystal oscillator setup and failover
	- add watchdog timer functions

*/

#include <avr/io.h>
#include "xmega_support.h"

void xmega_init(void){
	config32MHzClock();						// set system clock to 32 MHz
}


/* Configure 32 MHz clock (xmega) */
void config32MHzClock(void) 
{
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk 

	// initialize clock source to be 32MHz internal oscillator (no PLL)
	OSC.CTRL = OSC_RC32MEN_bm; 				// enable internal 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
	CCP = CCP_IOREG_gc; 					// Security Signature to modify clk
	CLK.CTRL = 0x01; 						// select sysclock 32MHz osc
};
