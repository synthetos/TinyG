/*
 * switch.c - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Switch Modes
 *
 *	The switches are considered to be homing switches when machine_state is
 *	MACHINE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge 
 *	and lockout subsequent interrupts for the defined lockout period. This approach 
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge 
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include <avr/interrupt.h>

#include "tinyg.h"
#include "config.h"
#include "switch.h"
#include "hardware.h"
#include "canonical_machine.h"
#include "text_parser.h"

/*
 * variables and settings 
 */
											// timer for debouncing switches
#define SW_LOCKOUT_TICKS 25					// 25=250ms. RTC ticks are ~10ms each
#define SW_DEGLITCH_TICKS 3					// 3=30ms

static void _switch_isr_helper(uint8_t sw_num);

/*
 * switch_init() - initialize homing/limit switches
 *
 *	This function assumes sys_init() and st_init() have been run previously to 
 *	bind the ports and set bit IO directions, repsectively. See system.h for details
 */
/* Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm). 
 *	v6 and earlier use internal pullups only. Internal pullups are set 
 *	regardless of board type but are extraneous for v7 boards.
 */
#define PIN_MODE PORT_OPC_PULLUP_gc				// pin mode. see iox192a3.h for details
//#define PIN_MODE PORT_OPC_TOTEM_gc			// alternate pin mode for v7 boards

void switch_init(void)
{
	for (uint8_t i=0; i<NUM_SWITCH_PAIRS; i++) {
		// old code from when switches fired on one edge or the other:
		//	uint8_t int_mode = (sw.switch_type == SW_TYPE_NORMALLY_OPEN) ? PORT_ISC_FALLING_gc : PORT_ISC_RISING_gc;

		// setup input bits and interrupts (previously set to inputs by st_init())
		if (sw.mode[MIN_SWITCH(i)] != SW_MODE_DISABLED) {
			hw.sw_port[i]->DIRCLR = SW_MIN_BIT_bm;		 	// set min input - see 13.14.14
			hw.sw_port[i]->PIN6CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
			hw.sw_port[i]->INT0MASK = SW_MIN_BIT_bm;	 	// interrupt on min switch
		} else {
			hw.sw_port[i]->INT0MASK = 0;	 				// disable interrupt
		}
		if (sw.mode[MAX_SWITCH(i)] != SW_MODE_DISABLED) {
			hw.sw_port[i]->DIRCLR = SW_MAX_BIT_bm;		 	// set max input - see 13.14.14
			hw.sw_port[i]->PIN7CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
			hw.sw_port[i]->INT1MASK = SW_MAX_BIT_bm;		// max on INT1
		} else {
			hw.sw_port[i]->INT1MASK = 0;
		}
		// set interrupt levels. Interrupts must be enabled in main()
		hw.sw_port[i]->INTCTRL = GPIO1_INTLVL;				// see gpio.h for setting
	}
	reset_switches();
}

/*
 * Switch closure processing routines
 *
 * ISRs 				 - switch interrupt handler vectors
 * _isr_helper()		 - common code for all switch ISRs
 * switch_rtc_callback() - called from RTC for each RTC tick.
 *
 *	These functions interact with each other to process switch closures and firing.
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */

ISR(X_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_X);}
ISR(Y_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Y);}
ISR(Z_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_Z);}
ISR(A_MIN_ISR_vect)	{ _switch_isr_helper(SW_MIN_A);}
ISR(X_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_X);}
ISR(Y_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Y);}
ISR(Z_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_Z);}
ISR(A_MAX_ISR_vect)	{ _switch_isr_helper(SW_MAX_A);}

static void _switch_isr_helper(uint8_t sw_num)
{
	if (sw.mode[sw_num] == SW_MODE_DISABLED) return;	// this is never supposed to happen
	if (sw.debounce[sw_num] == SW_LOCKOUT) return;		// exit if switch is in lockout
	sw.debounce[sw_num] = SW_DEGLITCHING;				// either transitions state from IDLE or overwrites it
	sw.count[sw_num] = -SW_DEGLITCH_TICKS;				// reset deglitch count regardless of entry state
	read_switch(sw_num);							// sets the state value in the struct
}

void switch_rtc_callback(void)
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) { 
		if (sw.debounce[i] == SW_IDLE) continue;
		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.debounce[i] = SW_IDLE; continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.debounce[i] = SW_LOCKOUT;
//			sw_show_switch();							// only called if __DEBUG enabled

			if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {		// regardless of switch type
				cm_request_feedhold();
			} else if (sw.mode[i] & SW_LIMIT_BIT) {		// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
}

/*
 * get_switch_mode()  - return switch mode setting
 * get_limit_thrown() - return true if a limit was tripped
 * get_switch_num()   - return switch number most recently thrown
 */

uint8_t get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
uint8_t get_limit_switch_thrown(void) { return(sw.limit_flag);}
uint8_t get_switch_thrown(void) { return(sw.sw_num_thrown);}

/*
 * reset_switches() - reset all switches and reset limit flag
 */

void reset_switches() 
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		sw.debounce[i] = SW_IDLE;
//		sw.count[i] = -SW_DEGLITCH_TICKS;
	}
	sw.limit_flag = false;
}

/*
 * read_switch() - read a switch directly with no interrupts or deglitching
 */
uint8_t read_switch(uint8_t sw_num)
{
	if ((sw_num < 0) || (sw_num >= NUM_SWITCHES)) return (SW_DISABLED);

	uint8_t read = 0;
	switch (sw_num) {
		case SW_MIN_X: { read = hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_X: { read = hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Y: { read = hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Y: { read = hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Z: { read = hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Z: { read = hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_A: { read = hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_A: { read = hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm; break;}
	}
	if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
		sw.state[sw_num] = ((read == 0) ? SW_CLOSED : SW_OPEN);// confusing. An NO switch drives the pin LO when thrown
		return (sw.state[sw_num]);
	} else {
		sw.state[sw_num] = ((read != 0) ? SW_CLOSED : SW_OPEN);
		return (sw.state[sw_num]);
	}
}

/*
 * _show_switch() - simple display routine
 */
#ifdef __DEBUG
void sw_show_switch(void)
{
	fprintf_P(stderr, PSTR("Limit Switch Thrown Xmin %d Xmax %d  Ymin %d Ymax %d  \
		Zmin %d Zmax %d Amin %d Amax %d\n"), 
		sw.state[SW_MIN_X], sw.state[SW_MAX_X],
		sw.state[SW_MIN_Y], sw.state[SW_MAX_Y],
		sw.state[SW_MIN_Z], sw.state[SW_MAX_Z],
		sw.state[SW_MIN_A], sw.state[SW_MAX_A]);
}
#else
void sw_show_switch(void) {}
#endif

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

stat_t sw_set_st(cmdObj_t *cmd)			// switch type (global)
{
//	if (cmd->value > SW_MODE_MAX_VALUE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	set_01(cmd);
	switch_init();
	return (STAT_OK);
}

stat_t sw_set_sw(cmdObj_t *cmd)			// switch setting
{
	if (cmd->value > SW_MODE_MAX_VALUE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	set_ui8(cmd);
	switch_init();
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_st[] PROGMEM = "[st]  switch type%18d [0=NO,1=NC]\n";
void sw_print_st(cmdObj_t *cmd) { text_print_ui8(cmd, fmt_st);}

//static const char fmt_ss[] PROGMEM = "Switch %s state:     %d\n";
//void sw_print_ss(cmdObj_t *cmd) { fprintf(stderr, fmt_ss, cmd->token, (uint8_t)cmd->value);}

/*
static const char msg_sw0[] PROGMEM = "Disabled";
static const char msg_sw1[] PROGMEM = "NO homing";
static const char msg_sw2[] PROGMEM = "NO homing & limit";
static const char msg_sw3[] PROGMEM = "NC homing";
static const char msg_sw4[] PROGMEM = "NC homing & limit";
static const char *const msg_sw[] PROGMEM = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };
*/

#endif

