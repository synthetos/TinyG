/*
 * limit_switches.c - limit switch interfaces 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * 	The limit switches trigger an interrupt on the leading edge (falling)
 *	and lockout subsequent interrupts for the defined lockout period. 
 *	This beats doing debouncing as an integration as it fires immediately.
 *
 *	Note: This mocule assumes the switches are normally open (and active LO).
 *	At some point it should support NC switches by configuration option.
 */

#include <avr/interrupt.h>

#include "xio.h"			// for debug statements
#include "tinyg.h"
#include "system.h"			// switch port bits are mapped here.
#include "config.h"
//#include "stepper.h"		// st_init() must run before ls_init()
#include "controller.h"
#include "limit_switches.h"
#include "canonical_machine.h"

/*
 * settings and locals
 */

#define	LS_OPC_gc PORT_OPC_PULLUP_gc		// totem poll pullup mode
//#define LS_ISC_gc PORT_ISC_RISING_gc		// ISRs on *trailing* edge
#define LS_ISC_gc PORT_ISC_FALLING_gc		// ISRs on *leading* edge
#define LS_LOCKOUT_TICKS 25					// ticks are ~10ms each


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

/*
 * ls_init() - initialize limit switches
 *
 * This function assumes st_init() has been run previously.
 */

void ls_init(void) 
{
	uint8_t i;

	for (i=0; i<MOTORS; i++) {
		// set initial port bit state to OFF
		device.port[i]->DIRSET = MIN_LIMIT_BIT_bm;	// set min to output
		device.port[i]->OUTSET = MIN_LIMIT_BIT_bm;	// min bit off
		device.port[i]->DIRSET = MAX_LIMIT_BIT_bm;	// set max to output
		device.port[i]->OUTSET = MAX_LIMIT_BIT_bm;	// max bit off

		// setup ports bits as inputs
		device.port[i]->DIRCLR = MIN_LIMIT_BIT_bm;		 // set min input
		device.port[i]->PIN6CTRL = (LS_OPC_gc | LS_ISC_gc);// pin modes
		device.port[i]->INT0MASK = MIN_LIMIT_BIT_bm;	 // min on INT0

		device.port[i]->DIRCLR = MAX_LIMIT_BIT_bm;		 // set max input
		device.port[i]->PIN7CTRL = (LS_OPC_gc | LS_ISC_gc);// pin modes
		device.port[i]->INT1MASK = MAX_LIMIT_BIT_bm;	 // max on INT1

		// set interrupt levels. Interrupts must be enabled in main()
		device.port[i]->INTCTRL = (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc);
	}
	ls_clear_limit_switches();
	ls.count = 0;
}

/*
 * _ls_show_limit_switch() - simple display routine
 */

#ifdef __dbSHOW_LIMIT_SWITCH
static void _ls_show_limit_switch(void);
static void _ls_show_limit_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown %d %d %d %d   %d %d %d %d\n"), 
		ls.flag[LS_X_MIN], ls.flag[LS_X_MAX], 
		ls.flag[LS_Y_MIN], ls.flag[LS_Y_MAX], 
		ls.flag[LS_Z_MIN], ls.flag[LS_Z_MAX], 
		ls.flag[LS_A_MIN], ls.flag[LS_A_MAX]);
}
#endif

/*
 * ISRs - Limit switch interrupt handler routine and vectors
 */

ISR(X_MIN_ISR_vect)	{ ls_isr_helper(LS_X_MIN);}
ISR(X_MAX_ISR_vect)	{ ls_isr_helper(LS_X_MAX);}
ISR(Y_MIN_ISR_vect)	{ ls_isr_helper(LS_Y_MIN);}
ISR(Y_MAX_ISR_vect)	{ ls_isr_helper(LS_Y_MAX);}
ISR(Z_MIN_ISR_vect)	{ ls_isr_helper(LS_Z_MIN);}
ISR(Z_MAX_ISR_vect)	{ ls_isr_helper(LS_Z_MAX);}
ISR(A_MIN_ISR_vect)	{ ls_isr_helper(LS_A_MIN);}
ISR(A_MAX_ISR_vect)	{ ls_isr_helper(LS_A_MAX);}

void ls_isr_helper(uint8_t flag)
{
	if (!ls.count) {
		cm_async_end();			// stop all motion immediately
		ls.thrown = TRUE;		// triggers the ls_handler tasks
		ls.flag[flag] = TRUE;
		ls.count = LS_LOCKOUT_TICKS;
	}
}

/*
 * ls_clear_limit_switches() - clear all limit switches but not lockout count
 *
 * Note: can't use memset on the flags because they are Volatile
 */

void ls_clear_limit_switches() 
{
	ls.thrown = FALSE;
	for (uint8_t i=0; i < LS_FLAG_SIZE; i++) {
		ls.flag[i] = FALSE; 
	}
}

/*
 * ls_read_limit_switches() - read the switches & set flags
 *
 *	As configured, switches are active LO
 */

void ls_read_limit_switches() 
{
#ifdef __SIMULATION_MODE
	ls_clear_limit_switches();		// clear flags and thrown
	return;
#endif

	uint8_t i;
	uint8_t flag = 0;				// used to index flag array

	ls_clear_limit_switches();		// clear flags and thrown

	for (i=0; i<MOTORS; i++) {
		if (!(device.port[i]->IN & MIN_LIMIT_BIT_bm)) { 	// min
			ls.flag[flag] = TRUE;
			ls.thrown = TRUE;
		}
		flag++;
		if (!(device.port[i]->IN & MAX_LIMIT_BIT_bm)) { 	// max
			ls.flag[flag] = TRUE;
			ls.thrown = TRUE;
		}
		flag++;
	}
#ifdef __dbSHOW_LIMIT_SWITCH
	_ls_show_limit_switch();
#endif
}

/*
 * getters - return TRUE if switch is thrown
 */

uint8_t ls_any_thrown(void)	{ return (ls.thrown); }
uint8_t ls_xmin_thrown(void) { return (ls.flag[LS_X_MIN]); }
uint8_t ls_xmax_thrown(void) { return (ls.flag[LS_X_MAX]); }
uint8_t ls_ymin_thrown(void) { return (ls.flag[LS_Y_MIN]); }
uint8_t ls_ymax_thrown(void) { return (ls.flag[LS_Y_MAX]); }
uint8_t ls_zmin_thrown(void) { return (ls.flag[LS_Z_MIN]); }
uint8_t ls_zmax_thrown(void) { return (ls.flag[LS_Z_MAX]); }
uint8_t ls_amin_thrown(void) { return (ls.flag[LS_A_MIN]); }
uint8_t ls_amax_thrown(void) { return (ls.flag[LS_A_MAX]); }

/*
 * ls_timer_callback() - call from timer for each clock tick.
 *
 * Also, once the lockout expires (gets to zero) it reads the switches
 * sets ls.thrown to picked up by ls_handler if the switch was thrown and
 * remained thrown (as can happen in some homing recovery cases)
 */

inline void ls_rtc_callback(void)
{
	if (ls.count) {
		--ls.count;
	}
}

/*
 * ls_handler() - main limit switch handler; called from controller loop
 */

uint8_t ls_handler(void)
{
	if (!ls.thrown) {		// leave if no switches are thrown
		return (TG_NOOP);
	}
#ifdef __dbSHOW_LIMIT_SWITCH
	_ls_show_limit_switch();
#endif
	if (cfg.homing_state == HOMING_COMPLETE) { 
//	if (cfg.homing_state != HOMING_IN_PROCESS) {
		tg_application_startup();			// initiate homing cycle
		return (TG_OK);
	}
	ls_clear_limit_switches();				// do this last, not before
	return (TG_OK);
}


