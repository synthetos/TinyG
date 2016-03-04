/*
 * switch.h - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
/* Switch processing functions under Motate
 *
 *	Switch processing turns pin transitions into reliable switch states.
 *	There are 2 main operations:
 *
 *	  - read pin		get raw data from a pin
 *	  - read switch		return processed switch closures
 *
 *	Read pin may be a polled operation or an interrupt on pin change. If interrupts
 *	are used they must be provided for both leading and trailing edge transitions.
 *
 *	Read switch contains the results of read pin and manages edges and debouncing.
 */
#ifndef SWITCH_H_ONCE
#define SWITCH_H_ONCE

#include "xmega/xmega_rtc.h"

/*
 * Common variables and settings
 */
											// timer for debouncing switches
#define SW_LOCKOUT_TICKS 25					// 25=250ms. RTC ticks are ~10ms each
#define SW_DEGLITCH_TICKS 3					// 3=30ms

#define SW_LOCKOUT_MS 250					// Note: RTC ticks only have 10 ms resolution
#define SW_DEGLITCH_MS 30					// Note: RTC ticks only have 10 ms resolution

// switch modes
#define SW_HOMING_BIT 0x01
#define SW_LIMIT_BIT 0x02
#define SW_MODE_DISABLED 		0								// disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT					// enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT					// enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)	// homing and limits
#define SW_MODE_MAX_VALUE 		SW_MODE_HOMING_LIMIT

typedef enum {
	SW_ACTIVE_LO = 0,               // SW_TYPE_NORMALLY_OPEN = 0,
	SW_ACTIVE_HI                    // SW_TYPE_NORMALLY_CLOSED
} swType;

typedef enum {
	SW_DISABLED = -1,
	SW_INACTIVE = 0,                // also reads as 'false', aka switch is "open"
	SW_ACTIVE                       // also reads as 'true' , aka switch is "closed"
} swState;

/*
 * Defines for old switch handling code
 */
// macros for finding the index into the switch table give the axis number
#define MIN_SWITCH(axis) (axis*2)
#define MAX_SWITCH(axis) (axis*2+1)

typedef enum {						// state machine for managing debouncing and lockout
	SW_IDLE = 0,
	SW_DEGLITCHING,
	SW_LOCKOUT
} swDebounce;

typedef enum {	 			        // indexes into switch arrays
	SW_MIN_X = 0,
	SW_MAX_X,
	SW_MIN_Y,
	SW_MAX_Y,
	SW_MIN_Z,
	SW_MAX_Z,
	SW_MIN_A,
	SW_MAX_A,
	NUM_SWITCHES 			        // must be last one. Used for array sizing and for loops
}swNums;
#define SW_OFFSET SW_MAX_X	        // offset between MIN and MAX switches
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)

/*
 * Defines for new switch handling code
 */
/*
// switch array configuration / sizing
#define SW_PAIRS				HOMING_AXES	// number of axes that can have switches
#define SW_POSITIONS			2			// swPosition is either SW_MIN or SW)MAX

typedef enum {
	SW_MIN = 0,
	SW_MAX
} swPosition;

typedef enum {
	SW_EDGE_NONE = 0,
	SW_EDGE_LEADING,
	SW_EDGE_TRAILING
} swEdge;
*/
/*
 * Interrupt levels and vectors - The vectors are hard-wired to xmega ports
 * If you change axis port assignments you need to change these, too.
 */
// Interrupt level: pick one:
//#define GPIO1_INTLVL (PORT_INT0LVL_HI_gc|PORT_INT1LVL_HI_gc)	// can't be hi
#define GPIO1_INTLVL (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc)
//#define GPIO1_INTLVL (PORT_INT0LVL_LO_gc|PORT_INT1LVL_LO_gc)	// shouldn;t be low

// port assignments for vectors
#define X_MIN_ISR_vect PORTA_INT0_vect	// these must line up with the SWITCH assignments in system.h
#define Y_MIN_ISR_vect PORTD_INT0_vect
#define Z_MIN_ISR_vect PORTE_INT0_vect
#define A_MIN_ISR_vect PORTF_INT0_vect
#define X_MAX_ISR_vect PORTA_INT1_vect
#define Y_MAX_ISR_vect PORTD_INT1_vect
#define Z_MAX_ISR_vect PORTE_INT1_vect
#define A_MAX_ISR_vect PORTF_INT1_vect

/*
 * Switch control structures
 *
 * Note: The term "thrown" is used because switches could be active low (normally-open)
 *		 or active high (normally-closed). "Thrown" means activated or hit.
 */
struct swStruct {								// switch state
	swType switch_type;						    // 0=NO, 1=NC - applies to all switches
	uint8_t sw_num_thrown;						// number of switch that was just thrown
	swState state[NUM_SWITCHES];				// 0=OPEN, 1=CLOSED (depends on switch type)

    uint8_t mode[NUM_SWITCHES];		            // 0=disabled, 1=homing, 2=homing+limit, 3=limit
    int8_t count[NUM_SWITCHES];		            // deglitching and lockout counter
    swDebounce debounce[NUM_SWITCHES];	        // switch debouncer state machine
    Timeout_t timeout[NUM_SWITCHES];            // deglitching and lockout timer

//	volatile uint8_t mode[NUM_SWITCHES];		// 0=disabled, 1=homing, 2=homing+limit, 3=limit
//	volatile int8_t count[NUM_SWITCHES];		// deglitching and lockout counter
//	volatile swDebounce debounce[NUM_SWITCHES];	// switch debouncer state machine - see swDebounce
};
struct swStruct sw;

/****************************************************************************************
 * Function prototypes
 */
void switch_init(void);
void reset_switches(void);

void switch_rtc_callback(void);

uint8_t get_switch_mode(uint8_t sw_num);
uint8_t get_switch_thrown(void);
void set_switch_type(uint8_t switch_type);
uint8_t get_switch_type();

/*
 * Switch config accessors and text functions
 */
stat_t sw_set_st(nvObj_t *nv);
stat_t sw_set_sw(nvObj_t *nv);

#ifdef __TEXT_MODE
	void sw_print_st(nvObj_t *nv);
#else
	#define sw_print_st tx_print_stub
#endif // __TEXT_MODE

#endif // End of include guard: SWITCH_H_ONCE
