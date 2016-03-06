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
#ifndef SWITCH_H_ONCE
#define SWITCH_H_ONCE

#include "xmega/xmega_rtc.h"    // needed for Timeout function

/*
 * Switch defines
 */
//--- change as required for board and switch hardware ---//

#define SW_LOCKOUT_MS 50       // timer for debouncing switches - Note: only has 10 ms resolution

// switch modes
#define SW_HOMING_BIT 0x01
#define SW_LIMIT_BIT 0x02
#define SW_PROBE_BIT 0x04
#define SW_MODE_DISABLED 		0								// disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT					// enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT					// enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)	// homing and limits
#define SW_MODE_PROBE           SW_PROBE_BIT
#define SW_MODE_MAX_VALUE 		SW_MODE_PROBE

typedef enum {
	SW_ACTIVE_LO = 0,               // SW_TYPE_NORMALLY_OPEN = 0,
	SW_ACTIVE_HI                    // SW_TYPE_NORMALLY_CLOSED
} swType;

typedef enum {
	SW_DISABLED = -1,
	SW_INACTIVE = 0,                // also reads as 'false', aka switch is "open"
	SW_ACTIVE = 1                   // also reads as 'true' , aka switch is "closed"
} swState;

typedef enum {                      // Note: Do not change ordering of these values
    SW_EDGE_NONE = -1,              // no edge detected or edge flag reset (must be zero)
    SW_EDGE_TRAILING = 0,           // flag is set when trailing edge is detected
    SW_EDGE_LEADING = 1             // flag is set when leading edge is detected
} swEdge;

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
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)
#define MIN_SWITCH(axis) (axis*2)   // macros for finding index into switch table given axis number
#define MAX_SWITCH(axis) (axis*2+1)

/*
 * Switch control structures
 */

typedef struct swSwitch {
    uint8_t mode;		            // 0=disabled, 1=homing, 2=homing+limit, 3=limit
    swType type;
    swState state;
    swEdge edge;
    Timeout_t timeout;              // lockout timer
    uint16_t lockout_ms;            // lockout time in ms
} switch_t;

typedef struct swSingleton {
	swType switch_type;             // global setting for switch type
    switch_t s[NUM_SWITCHES];       // switch objects
} switches_t;
switches_t sw;

/****************************************************************************************
 * Function prototypes
 */
void switch_init(void);
void reset_switches(void);
swState read_switch (const uint8_t sw_num);
uint8_t get_switch_mode(const uint8_t sw_num);
int8_t find_probe_switch(void);

// Switch config accessors and text functions
stat_t sw_set_st(nvObj_t *nv);
stat_t sw_set_sw(nvObj_t *nv);

#ifdef __TEXT_MODE
	void sw_print_st(nvObj_t *nv);
#else
	#define sw_print_st tx_print_stub
#endif

#endif // End of include guard: SWITCH_H_ONCE
