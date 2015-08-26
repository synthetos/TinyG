/*
 * gpio.h - Digital IO  handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
 * Copyright (c) 2015 Robert Giseburt
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
#ifndef GPIO_H_ONCE
#define GPIO_H_ONCE

/*
 * GPIO defines
 */
//--- change as required for board and switch hardware ---//

#ifdef __ARM
#define D_IN_CHANNELS       9  // v9    // number of digital inputs supported
#else
#define D_IN_CHANNELS	    8  // v8    // number of digital inputs supported
#define D_IN_PAIRS (D_IN_CHANNELS/2)
#endif

#define D_OUT_CHANNELS	    4           // number of digital outputs supported
#define A_IN_CHANNELS	    0           // number of analog inputs supported
#define A_OUT_CHANNELS	    0           // number of analog outputs supported

#define INPUT_LOCKOUT_MS    50          // milliseconds to go dead after input firing

//--- do not change from here down ---//

typedef enum {
    INPUT_MODE_DISABLED = -1,           // input is disabled
    INPUT_ACTIVE_LOW = 0,               // input is active low (aka normally open)
    INPUT_ACTIVE_HIGH = 1,              // input is active high (aka normally closed)
    INPUT_MODE_MAX                      // unused. Just for range checking
} inputMode;
#define NORMALLY_OPEN   INPUT_ACTIVE_LOW    // equivalent
#define NORMALLY_CLOSED INPUT_ACTIVE_HIGH   // equivalent

typedef enum {                          // actions are initiated from within the input's ISR
    INPUT_ACTION_NONE = 0,
    INPUT_ACTION_STOP,                  // stop at normal jerk - preserves positional accuracy
    INPUT_ACTION_FAST_STOP,             // stop at high jerk - preserves positional accuracy
    INPUT_ACTION_HALT,                  // stop immediately - not guaranteed to preserve position
    INPUT_ACTION_PANIC,                 // initiate a panic. stops everything immediately - does not preserve position
    INPUT_ACTION_RESET,                 // reset system
	INPUT_ACTION_MAX                    // unused. Just for range checking
} inputAction;

typedef enum {                          // functions are requested from the ISR, run from the main loop
    INPUT_FUNCTION_NONE = 0,
    INPUT_FUNCTION_LIMIT,               // limit switch processing
    INPUT_FUNCTION_INTERLOCK,           // interlock processing
    INPUT_FUNCTION_SHUTDOWN,            // shutdown in support of external emergency stop
//    INPUT_FUNCTION_SPINDLE_READY,     // signal that spindle is ready (up to speed)
	INPUT_FUNCTION_MAX                  // unused. Just for range checking
} inputFunc;

typedef enum {
    INPUT_DISABLED = -1,                // value returned if input is disabled
    INPUT_INACTIVE = 0,                 // aka switch open, also read as 'false'
    INPUT_ACTIVE = 1                    // aka switch closed, also read as 'true'
} inputState;

typedef enum {
    INPUT_EDGE_NONE = 0,                // no edge detected or edge flag reset (must be zero)
    INPUT_EDGE_LEADING,                 // flag is set when leading edge is detected
    INPUT_EDGE_TRAILING                 // flag is set when trailing edge is detected
} inputEdgeFlag;

/*
 * GPIO structures
 */
typedef struct ioDigitalInput {		    // one struct per digital input
	inputMode mode;					    // -1=disabled, 0=active low (NO), 1= active high (NC)
	inputAction action;                 // 0=none, 1=stop, 2=halt, 3=stop_steps, 4=reset
	inputFunc function;                 // function to perform when activated / deactivated
    inputState state;                   // input state 0=inactive, 1=active, -1=disabled
    inputEdgeFlag edge;                 // keeps a transient record of edges for immediate inquiry
    bool homing_mode;                   // set true when input is in homing mode.
    bool probing_mode;                  // set true when input is in probing mode.
	uint16_t lockout_ms;                // number of milliseconds for debounce lockout
	uint32_t lockout_timer;             // time to expire current debounce lockout, or 0 if no lockout
} d_in_t;

typedef struct gpioDigitalOutput {      // one struct per digital output
    inputMode mode;
} d_out_t;

typedef struct gpioAnalogInput {        // one struct per analog input
    inputMode mode;
} a_in_t;

typedef struct gpioAnalogOutput {       // one struct per analog output
    inputMode mode;
} a_out_t;

extern d_in_t   d_in[D_IN_CHANNELS];
extern d_out_t  d_out[D_OUT_CHANNELS];
extern a_in_t   a_in[A_IN_CHANNELS];
extern a_out_t  a_out[A_OUT_CHANNELS];

/*
 * GPIO function prototypes
 */

void gpio_init(void);
void gpio_reset(void);

bool gpio_read_input(const uint8_t input_num);
void gpio_set_homing_mode(const uint8_t input_num, const bool is_homing);
void gpio_set_probing_mode(const uint8_t input_num, const bool is_probing);

stat_t io_set_mo(nvObj_t *nv);
stat_t io_set_ac(nvObj_t *nv);
stat_t io_set_fn(nvObj_t *nv);

stat_t io_get_input(nvObj_t *nv);

#ifdef __TEXT_MODE
	void io_print_mo(nvObj_t *nv);
	void io_print_ac(nvObj_t *nv);
	void io_print_fn(nvObj_t *nv);
	void io_print_in(nvObj_t *nv);
#else
	#define io_print_mo tx_print_stub
	#define io_print_ac tx_print_stub
	#define io_print_fn tx_print_stub
	#define io_print_in tx_print_stub
#endif // __TEXT_MODE

//**************************************************************************************
//**** OLD SWITCHES ********************************************************************
//**************************************************************************************

#define SW_LOCKOUT_TICKS 25					// 25=250ms. RTC ticks are ~10ms each
#define SW_DEGLITCH_TICKS 3					// 3=30ms

// macros for finding the index into the switch table give the axis number
#define MIN_SWITCH(axis) (axis*2)
#define MAX_SWITCH(axis) (axis*2+1)

// switch modes
#define SW_HOMING_BIT 0x01
#define SW_LIMIT_BIT 0x02
#define SW_MODE_DISABLED 		0								// disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT					// enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT					// enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)	// homing and limits
#define SW_MODE_MAX_VALUE 		SW_MODE_HOMING_LIMIT

typedef enum {
	SW_TYPE_NORMALLY_OPEN = 0,
	SW_TYPE_NORMALLY_CLOSED
} swType;

typedef enum {
	SW_DISABLED = -1,
	SW_OPEN = 0,					// also read as 'false'
	SW_CLOSED						// also read as 'true'
} swState;

typedef enum {					// state machine for managing debouncing and lockout
	SW_IDLE = 0,
	SW_DEGLITCHING,
	SW_LOCKOUT
} swDebounce;

typedef enum {	 			    // indexes into switch arrays
	SW_MIN_X = 0,
	SW_MAX_X,
	SW_MIN_Y,
	SW_MAX_Y,
	SW_MIN_Z,
	SW_MAX_Z,
	SW_MIN_A,
	SW_MAX_A,
	NUM_SWITCHES 			// must be last one. Used for array sizing and for loops
} swNums;
#define SW_OFFSET SW_MAX_X	// offset between MIN and MAX switches
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)

/* Switch control structures
 * Note: The term "thrown" is used because switches could be normally-open
 *       or normally-closed. "Thrown" means activated or hit.
 */

struct swStruct {								// switch state
	uint8_t switch_type;						// 0=NO, 1=NC - applies to all switches
	uint8_t limit_flag;							// 1=limit switch thrown - do a lockout
	uint8_t sw_num_thrown;						// number of switch that was just thrown
	uint8_t state[NUM_SWITCHES];				// 0=OPEN, 1=CLOSED (depends on switch type)
	volatile uint8_t mode[NUM_SWITCHES];		// 0=disabled, 1=homing, 2=homing+limit, 3=limit
	volatile uint8_t debounce[NUM_SWITCHES];	// switch debouncer state machine - see swDebounce
	volatile int8_t count[NUM_SWITCHES];		// deglitching and lockout counter
};
struct swStruct sw;

/*
 * Function prototypes
 */

uint8_t read_switch(uint8_t sw_num);
uint8_t get_switch_mode(uint8_t sw_num);
void switch_rtc_callback(void);
uint8_t get_limit_switch_thrown(void);

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

//**************************************************************************************
//**** OLD OUTPUTS *********************************************************************
//**************************************************************************************
void IndicatorLed_set(void);
void IndicatorLed_clear(void);
void IndicatorLed_toggle(void);

void gpio_led_on(uint8_t led);
void gpio_led_off(uint8_t led);
void gpio_led_toggle(uint8_t led);

uint8_t gpio_read_bit(uint8_t b);
void gpio_set_bit_on(uint8_t b);
void gpio_set_bit_off(uint8_t b);




#endif // End of include guard: GPIO_H_ONCE
