/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA Clock System driver header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the XMEGA Clock System driver.
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
 * $Revision: 1665 $
 * $Date: 2008-06-05 09:21:50 +0200 (to, 05 jun 2008) $  \n
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
#ifndef CLKSYS_DRIVER_H
#define CLKSYS_DRIVER_H

#include "avr_compiler.h"


/* Definitions of macros. */

/*! \brief This macro enables the selected oscillator.
 *
 *  \note Note that the oscillator cannot be used as a main system clock
 *        source without being enabled and stable first. Check the ready flag
 *        before using the clock. The function CLKSYS_IsReady( _oscSel )
 *        can be used to check this.
 *
 *  \param  _oscSel Bitmask of selected clock. Can be one of the following
 *                  OSC_RC2MEN_bm, OSC_RC32MEN_bm, OSC_RC32KEN_bm, OSC_XOSCEN_bm,
 *                  OSC_PLLEN_bm.
 */
#define CLKSYS_Enable( _oscSel ) ( OSC.CTRL |= (_oscSel) )

/*! \brief This macro check if selected oscillator is ready.
 *
 *  This macro will return non-zero if is is running, regardless if it is
 *  used as a main clock source or not.
 *
 *  \param _oscSel Bitmask of selected clock. Can be one of the following
 *                 OSC_RC2MEN_bm, OSC_RC32MEN_bm, OSC_RC32KEN_bm, OSC_XOSCEN_bm,
 *                 OSC_PLLEN_bm.
 *
 *  \return  Non-zero if oscillator is ready and running.
 */
#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

/*! \brief This macro disables routing of clock signals to the Real-Time
 *         Counter (RTC).
 *
 *  Disabling the RTC saves power if the RTC is not in use.
 */
#define CLKSYS_RTC_ClockSource_Disable() ( CLK.RTCCTRL &= ~CLK_RTCEN_bm )

/*! \brief This macro disables the automatic calibration of the selected
 *         internal oscillator.
 *
 *  \param _clk  Clock source calibration to disable, either DFLLRC2M or DFLLRC32M.
 */
#define CLKSYS_AutoCalibration_Disable( _clk ) ( (_clk).CTRL &= ~DFLL_ENABLE_bm )


/* Prototyping of function. Detailed information is found in source file. */
void CCPWrite( volatile uint8_t * address, uint8_t value );
void CLKSYS_XOSC_Config( OSC_FRQRANGE_t freqRange,
                         bool lowPower32kHz,
                         OSC_XOSCSEL_t xoscModeSelection );
void CLKSYS_PLL_Config( OSC_PLLSRC_t clockSource, uint8_t factor );
uint8_t CLKSYS_Disable( uint8_t oscSel );
void CLKSYS_Prescalers_Config( CLK_PSADIV_t PSAfactor,
                               CLK_PSBCDIV_t PSBCfactor );
uint8_t CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_t clockSource );
void CLKSYS_RTC_ClockSource_Enable( CLK_RTCSRC_t clockSource );
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference );
void CLKSYS_XOSC_FailureDetection_Enable( void );
void CLKSYS_Configuration_Lock( void );


#endif
