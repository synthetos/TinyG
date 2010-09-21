/*
 * limit_switches.c - limit switch interfaces 
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "tinyg.h"
#include "config.h"
#include "hardware.h"
#include "stepper.h"
#include "controller.h"
#include "limit_switches.h"
#include "canonical_machine.h"

/*
 * Interrupt vectors - these are hard-wired to ports in the xmega
 * if you change axis port assignments all these need to change, too
 */

#define X_MIN_ISR_vect	PORTA_INT0_vect
#define X_MAX_ISR_vect	PORTA_INT1_vect
#define Y_MIN_ISR_vect	PORTF_INT0_vect
#define Y_MAX_ISR_vect	PORTF_INT1_vect
#define Z_MIN_ISR_vect	PORTE_INT0_vect
#define Z_MAX_ISR_vect	PORTE_INT1_vect
#define A_MIN_ISR_vect	PORTD_INT0_vect
#define A_MAX_ISR_vect	PORTD_INT1_vect

//static void _ls_handler(void);
static void _ls_stop(void);
static void _ls_start(void);
static void _ls_end(void);

/*
 * ls_init() - initialize limit switches
 *
 * This function assumes st_init() has been run previously.
 */

#define	LS_OPC_gc PORT_OPC_PULLUP_gc				// totem poll pullup mode
#define LS_ISC_gc PORT_ISC_RISING_gc				// ISRs on rising edge

void ls_init(void) 
{
	for (uint8_t i=X; i<=A; i++) {
		AXIS(i).port->DIRCLR = (1<<MIN_LIMIT_BIT_bp);		// min - set as input
		AXIS(i).port->DIRCLR = (1<<MAX_LIMIT_BIT_bp);		// max - set as input
		AXIS(i).port->PIN6CTRL = (LS_OPC_gc | LS_ISC_gc);	// min - pin modes
		AXIS(i).port->PIN7CTRL = (LS_OPC_gc | LS_ISC_gc);	// max - pin modes
		AXIS(i).port->INT0MASK = (1<<MIN_LIMIT_BIT_bp);		// min - INT0
		AXIS(i).port->INT1MASK = (1<<MAX_LIMIT_BIT_bp);		// max - INT1
		// set interrupt levels. Interrupts must be enabled in main()
		AXIS(i).port->INTCTRL = (PORT_INT0LVL_MED_gc | PORT_INT1LVL_MED_gc);
	}
}

/*
 * ISRs - Limit switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ _ls_end(); }
ISR(X_MAX_ISR_vect)	{ _ls_end(); }
ISR(Y_MIN_ISR_vect)	{ _ls_end(); }
ISR(Y_MAX_ISR_vect)	{ _ls_end(); }
ISR(Z_MIN_ISR_vect)	{ _ls_end(); }
ISR(Z_MAX_ISR_vect)	{ _ls_end(); }
ISR(A_MIN_ISR_vect)	{ _ls_start(); }
ISR(A_MAX_ISR_vect)	{ _ls_stop(); }

void _ls_start() {
//	cm_async_start();
	return;
}

void _ls_stop() {
//	cm_async_stop();
	return;
}

void _ls_end() {
//	cm_async_end();
	return;
}

