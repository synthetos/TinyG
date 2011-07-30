/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA PMIC driver header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the XMEGA Programmable Multi-level
 *      Interrupt Controller driver/example.
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

/*
 * Ref: ATMEL AVR10__ app note and code
 * Contains some minor mods by ASH to adapt to GCC: search on "(ash mod)"
 */
 
#ifndef xmega_interrupts_h
#define xmega_interrupts_h

#include <avr/io.h>			// Was #include "avr_compiler.h"  (ash mod)


/* Definitions of macros. */

/*! \brief Enable low-level interrupts. */
#define PMIC_EnableLowLevel() (PMIC.CTRL |= PMIC_LOLVLEN_bm)


/*! \brief Disable low-level interrupts. */
#define PMIC_DisableLowLevel() (PMIC.CTRL &= ~PMIC_LOLVLEN_bm)


/*! \brief Enable medium-level interrupts. */
#define PMIC_EnableMediumLevel() (PMIC.CTRL |= PMIC_MEDLVLEN_bm)


/*! \brief Disable medium-level interrupts. */
#define PMIC_DisableMediumLevel() (PMIC.CTRL &= ~PMIC_MEDLVLEN_bm)


/*! \brief Enable high-level interrupts. */
#define PMIC_EnableHighLevel() (PMIC.CTRL |= PMIC_HILVLEN_bm)


/*! \brief Disable high-level interrupts. */
#define PMIC_DisableHighLevel() (PMIC.CTRL &= ~PMIC_HILVLEN_bm)


/*! \brief Enable round-robin scheduling for low-level interrupts. */
#define	PMIC_EnableRoundRobin() (PMIC.CTRL |= PMIC_RREN_bm)


/*! \brief Disable round-robin scheduling for low-level interrupts. */
#define PMIC_DisableRoundRobin() (PMIC.CTRL &= ~PMIC_RREN_bm)



/*! \brief Set interrupt priority for round-robin scheduling.
 *
 *  This macro selects which low-level interrupt has highest priority.
 *  Use this function together with round-robin scheduling.
 *
 *  \note The INTPRI register wants the vector _number_ (not address) of the lowest
 *        prioritized interrupt among low-level interrupts. Since vector addresses
 *        lies on 4-byte boundaries, we divide by 4.
 *
 *  \param  _vectorAddress  Number between 0 and the maximum vector address for the device.
 */
#define PMIC_SetNextRoundRobinInterrupt(_vectorAddress) (PMIC.INTPRI = (_vectorAddress >> 2) - 1)



/*! \brief Check if a high-level interrupt handler is currently executing.
 *
 *  \return  Non-zero if interrupt handler is executing. Zero otherwise.
 */
#define PMIC_IsHighLevelExecuting() (PMIC.STATUS & PMIC_HILVLEX_bm)



/*! \brief Check if a medium-level interrupt handler is currently executing.
 *
 *  \return  Non-zero if interrupt handler is executing. Zero otherwise.
 */
#define PMIC_IsMediumLevelExecuting() (PMIC.STATUS & PMIC_MEDLVLEX_bm)



/*! \brief Check if a low-level interrupt handler is currently executing.
 *
 *  \return  Non-zero if interrupt handler is executing. Zero otherwise.
 */
#define PMIC_IsLowLevelExecuting() (PMIC.STATUS & PMIC_LOLVLEX_bm)



/*! \brief Check if an NMI handler is currently executing.
 *
 *  \return  Non-zero if interrupt handler is executing. Zero otherwise.
 */
#define PMIC_IsNMIExecuting() (PMIC.STATUS & PMIC_NMIEX_bm)



/* Prototype of functions. */

void PMIC_SetVectorLocationToBoot( void );
void PMIC_SetVectorLocationToApplication( void );

#endif
