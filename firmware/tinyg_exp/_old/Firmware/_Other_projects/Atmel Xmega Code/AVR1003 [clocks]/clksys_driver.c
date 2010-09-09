/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA Clock System driver source file.
 *
 *      This file contains the function implementations for the XMEGA Clock
 *      System driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA Clock System.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *      Several functions use the following construct:
 *          "some_register = ... | (some_parameter ? SOME_BIT_bm : 0) | ..."
 *      Although the use of the ternary operator ( if ? then : else ) is
 *      discouraged, in some occasions the operator makes it possible to
 *      write pretty clean and neat code. In this driver, the construct is
 *      used to set or not set a configuration bit based on a boolean input
 *      parameter, such as the "some_parameter" in the example above.
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
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "clksys_driver.h"

/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the timecritial
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
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
	AVR_ENTER_CRITICAL_REGION( );
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

	AVR_LEAVE_CRITICAL_REGION( );
#endif
}

/*! \brief This function configures the external oscillator.
 *
 *  \note Note that the oscillator cannot be used as a main system clock
 *        source without being enabled and stable first. Check the ready flag
 *        before using the clock. The macro CLKSYS_IsReady( _oscSel )
 *        can be used to check this.
 *
 *  \param  freqRange          Frequency range for high-frequency crystal, does
 *                             not apply for external clock or 32kHz crystals.
 *  \param  lowPower32kHz      True of high-quality watch crystals are used and
 *                             low-power oscillator is desired.
 *  \param  xoscModeSelection  Combined selection of oscillator type (or
 *                             external clock) and startup times.
 */
void CLKSYS_XOSC_Config( OSC_FRQRANGE_t freqRange,
                         bool lowPower32kHz,
                         OSC_XOSCSEL_t xoscModeSelection )
{
	OSC.XOSCCTRL = (uint8_t) freqRange |
	               ( lowPower32kHz ? OSC_X32KLPM_bm : 0 ) |
	               xoscModeSelection;
}


/*! \brief This function configures the internal high-frequency PLL.
 *
 *  Configuration of the internal high-frequency PLL to the correct
 *  values. It is used to define the input of the PLL and the factor of
 *  multiplication of the input clock source.
 *
 *  \note Note that the oscillator cannot be used as a main system clock
 *        source without being enabled and stable first. Check the ready flag
 *        before using the clock. The macro CLKSYS_IsReady( _oscSel )
 *        can be used to check this.
 *
 *  \param  clockSource Reference clock source for the PLL,
 *                      must be above 0.4MHz.
 *  \param  factor      PLL multiplication factor, must be
 *                      from 1 to 31, inclusive.
 */
void CLKSYS_PLL_Config( OSC_PLLSRC_t clockSource, uint8_t factor )
{
	factor &= OSC_PLLFAC_gm;
	OSC.PLLCTRL = (uint8_t) clockSource | ( factor << OSC_PLLFAC_gp );
}


/*! \brief This function disables the selected oscillator.
 *
 *  This function will disable the selected oscillator if possible.
 *  If it is currently used as a main system clock source, hardware will
 *  disregard the disable attempt, and this function will return zero.
 *  If it fails, change to another main system clock source and try again.
 *
 *  \param oscSel  Bitmask of selected clock. Can be one of the following
 *                 OSC_RC2MEN_bm, OSC_RC32MEN_bm, OSC_RC32KEN_bm,
 *                 OSC_XOSCEN_bm, OSC_PLLEN_bm.
 *
 *  \return  Non-zero if oscillator was disabled successfully.
 */
uint8_t CLKSYS_Disable( uint8_t oscSel )
{
	OSC.CTRL &= ~oscSel;
	uint8_t clkEnabled = OSC.CTRL & oscSel;
	return clkEnabled;
}


/*! \brief This function changes the prescaler configuration.
 *
 *  Change the configuration of the three system clock
 *  prescaler is one single operation. The user must make sure that
 *  the main CPU clock does not exceed recommended limits.
 *
 *  \param  PSAfactor   Prescaler A division factor, OFF or 2 to 512 in
 *                      powers of two.
 *  \param  PSBCfactor  Prescaler B and C division factor, in the combination
 *                      of (1,1), (1,2), (4,1) or (2,2).
 */
void CLKSYS_Prescalers_Config( CLK_PSADIV_t PSAfactor,
                               CLK_PSBCDIV_t PSBCfactor )
{
	uint8_t PSconfig = (uint8_t) PSAfactor | PSBCfactor;
	CCPWrite( &CLK.PSCTRL, PSconfig );
}


/*! \brief This function selects the main system clock source.
 *
 *  Hardware will disregard any attempts to select a clock source that is not
 *  enabled or not stable. If the change fails, make sure the source is ready
 *  and running and try again.
 *
 *  \param  clockSource  Clock source to use as input for the system clock
 *                       prescaler block.
 *
 *  \return  Non-zero if change was successful.
 */
uint8_t CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_t clockSource )
{
	uint8_t clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | clockSource;
	CCPWrite( &CLK.CTRL, clkCtrl );
	clkCtrl = ( CLK.CTRL & clockSource );
	return clkCtrl;
}


/*! \brief This function selects a Real-Time Counter clock source.
 *
 *  Selects the clock source for use by the Real-Time Counter (RTC)
 *  and enables clock signal routing to the RTC module.
 *
 *  \param  clockSource  Clock source to use for the RTC.
 */
void CLKSYS_RTC_ClockSource_Enable( CLK_RTCSRC_t clockSource )
{
	CLK.RTCCTRL = ( CLK.RTCCTRL & ~CLK_RTCSRC_gm ) |
	              clockSource |
	              CLK_RTCEN_bm;
}


/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  extReference True if external crystal should be used as reference.
 */
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_bm) {
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}
}


/*! \brief This function enables the External Oscillator Failure Detection
 *         (XOSCFD) feature.
 *
 *  The feature will stay enabled until next reset. Note that the
 *  XOSCFD _will_ issue the XOSCF Non-maskable Interrupt (NMI) regardless of
 *  any interrupt priorities and settings. Therefore, make sure that a handler
 *  is implemented for the XOSCF NMI when you enable it.
 */
void CLKSYS_XOSC_FailureDetection_Enable( void )
{
	CCPWrite( &OSC.XOSCFAIL, ( OSC_XOSCFDIF_bm | OSC_XOSCFDEN_bm ) );
}


/*! \brief This function lock the entire clock system configuration.
 *
 *  This will lock the configuration until the next reset, or until the
 *  External Oscillator Failure Detections (XOSCFD) feature detects a failure
 *  and switches to internal 2MHz RC oscillator.
 */
void CLKSYS_Configuration_Lock( void )
{
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );
}

