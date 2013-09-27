/*
 * switch.h - switch handling functions
 * Part of TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
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

/*
 * Interrupt levels and vectors - The vectors are hard-wired to xmega ports
 * If you change axis port assignments you need to chanage these, too.
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

// macros for finding the index into the switch table give the axis number
#define MIN_SWITCH(axis) (axis*2)
#define MAX_SWITCH(axis) (axis*2+1)

/*
 * Global Scope Definitions, Functions and Data
 */

enum swNums {	 			// indexes into switch arrays
	SW_MIN_X = 0,
	SW_MAX_X,
	SW_MIN_Y,
	SW_MAX_Y,
	SW_MIN_Z, 
	SW_MAX_Z,
	SW_MIN_A,
	SW_MAX_A,
	NUM_SWITCHES 			// must be last one. Used for array sizing and for loops
};
#define SW_OFFSET SW_MAX_X	// offset between MIN and MAX switches
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)

#define SW_DISABLED -1
#define SW_OPEN 	 0
#define SW_CLOSED	 1

// switch mode settings
#define SW_HOMING_BIT 0x01
#define SW_LIMIT_BIT 0x02

#define SW_MODE_DISABLED 		0								// disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT					// enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT					// enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)	// homing and limits
#define SW_MODE_MAX_VALUE 		SW_MODE_HOMING_LIMIT

enum swType {
	SW_TYPE_NORMALLY_OPEN = 0,
	SW_TYPE_NORMALLY_CLOSED
};

enum swDebounce {							// state machine for managing debouncing and lockout
	SW_IDLE = 0,
	SW_DEGLITCHING,
	SW_LOCKOUT
};

struct swStruct {							// switch state
	uint8_t switch_type;					// 0=NO, 1=NC - applies to all switches
	uint8_t limit_flag;						// 1=limit switch thrown - do a lockout
	uint8_t sw_num_thrown;					// number of switch that was just thrown
	uint8_t state[NUM_SWITCHES];			// 0=OPEN, 1=CLOSED (depends on switch type)
	volatile uint8_t mode[NUM_SWITCHES];	// 0=disabled, 1=homing, 2=homing+limit, 3=limit
	volatile uint8_t debounce[NUM_SWITCHES];// switch debouncer state machine - see swDebounce
	volatile int8_t count[NUM_SWITCHES];	// deglitching and lockout counter
};
struct swStruct sw;

// Note 1: The term "thrown" is used because switches could be normally-open 
//		   or normally-closed. "Thrown" means activated or hit.

void switch_init(void);
void switch_rtc_callback(void);
uint8_t get_switch_mode(uint8_t sw_num);
uint8_t get_limit_switch_thrown(void);
uint8_t get_switch_thrown(void);
void reset_switches(void);
uint8_t read_switch(uint8_t sw_num);
void sw_show_switch(void);

stat_t sw_set_st(cmdObj_t *cmd);
stat_t sw_set_sw(cmdObj_t *cmd);


#ifdef __TEXT_MODE
	void sw_print_st(cmdObj_t *cmd);
#else 
	#define sw_print_st tx_print_stub
#endif // __TEXT_MODE


/*============== G2 switch code - completely different, for now ===================

*
 * variables and settings 
 *
#define SW_LOCKOUT_TICKS 100	// in milliseconds
//#define SW_DEGLITCH_TICKS 30	// in milliseconds

#define SW_PAIRS AXES			// array sizing
#define SW_POSITIONS 2			// array sizing

*
 * definitions
 *
// macros for finding the index into the switch table give the axis number
#define MIN_SWITCH(axis) (axis*2)
#define MAX_SWITCH(axis) (axis*2+1)

enum swPosition {
	SW_MIN = 0,
	SW_MAX
};

enum swType {
	SW_NORMALLY_OPEN = 0,
	SW_NORMALLY_CLOSED
};

enum swState {
	SW_OPEN = 0,					// also read as 'false'
	SW_CLOSED						// also read as 'true'
};

enum swEdge {
	SW_NO_EDGE = 0,
	SW_LEADING,
	SW_TRAILING,
};

// switch modes
#define SW_HOMING_BIT 0x01
#define SW_LIMIT_BIT 0x02
#define SW_MODE_DISABLED 		0							 // disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT				 // enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT				 // enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)// homing and limits
#define SW_MODE_MAX_VALUE 		SW_MODE_HOMING_LIMIT

typedef struct swSwitch {			// one struct per switch
	uint8_t type;					// swType: 0=NO, 1=NC
	uint8_t mode;					// 0=disabled, 1=homing, 2=limit, 3=homing+limit
	uint8_t state;					// set true if switch is closed
	uint8_t edge;					// keeps a transient record of edges for immediate inquiry
	uint16_t debounce_ticks;		// number of millisecond ticks for debounce lockout 
	uint32_t debounce_timeout;		// time to expire current debounce lockout, or 0 if no lockout
	void (*when_open)(struct swSwitch *s);		// callback to action function when sw is open - passes *s, returns void
	void (*when_closed)(struct swSwitch *s);	// callback to action function when closed
	void (*on_leading)(struct swSwitch *s);		// callback to action function for leading edge onset
	void (*on_trailing)(struct swSwitch *s);	// callback to action function for trailing edge
} switch_t;
typedef void (*sw_callback)(switch_t *s); // typedef for switch action callback

typedef struct swSwitchArray {		// array of switches
	uint8_t type;					// switch type for entire array
	switch_t s[SW_PAIRS][SW_POSITIONS];
} switches_t;
extern switches_t sw;

// function prototypes

void switch_init(void);
stat_t poll_switches(void);
uint8_t read_switch(switch_t *s, uint8_t pin_value);
uint8_t get_switch_mode(uint8_t sw_num);

//void switch_rtc_callback(void);
//uint8_t switch_get_limit_thrown(void);
//uint8_t switch_get_sw_thrown(void);
//void switch_reset_switches(void);
//uint8_t switch_read_switch(uint8_t sw_num);


// unit test setup 

//#define __UNIT_TEST_GPIO				// uncomment to enable GPIO unit tests
#ifdef __UNIT_TEST_GPIO
void switch_unit_tests(void);
#define	GPIO_UNITS switch_unit_tests();
#else
#define	GPIO_UNITS
#endif // __UNIT_TEST_GPIO
*/

#endif // End of include guard: SWITCH_H_ONCE


