/*
 * gpio.cpp - digital IO handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
 * Copyright (c) 2015 Robert Giseburt
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
 *	The switches are considered to be homing switches when cycle_state is
 *	CYCLE_HOMING. At all other times they are treated as limit switches:
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

#include "tinyg.h"      // #1
#include "config.h"     // #2
#include "gpio.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"
#include "canonical_machine.h"

#include "text_parser.h"
#include "controller.h"
#include "util.h"
#include "report.h"
//#include "xio.h"						// signals

#ifdef __AVR
    #include <avr/interrupt.h>
#else
    #include "MotateTimers.h"
    using Motate::SysTickTimer;
#endif

/**** Allocate structures ****/

//io_di_t din[DI_CHANNELS];
io_di_t d_in[DI_CHANNELS];
io_do_t d_out[DO_CHANNELS];
io_ai_t a_in[AI_CHANNELS];
io_ao_t a_out[AO_CHANNELS];

//io_t io;

/**** Defines and static functions ****/

static bool _read_raw_pin(const uint8_t input_num_ext);
static uint8_t _condition_pin(const uint8_t input_num_ext, const int8_t pin_value);
static void _dispatch_pin(const uint8_t input_num_ext);

//static uint8_t _condition_avr(const uint8_t input_num_ext);

/**** Setup Low Level Stuff ****/

#ifdef __ARM

// WARNING: These return raw pin values, NOT corrected for NO/NC Active high/low
//          Also, these take EXTERNAL pin numbers -- 1-based
static InputPin<kInput1_PinNumber> input_1_pin(kPullUp);
static InputPin<kInput2_PinNumber> input_2_pin(kPullUp);
static InputPin<kInput3_PinNumber> input_3_pin(kPullUp);
static InputPin<kInput4_PinNumber> input_4_pin(kPullUp);
static InputPin<kInput5_PinNumber> input_5_pin(kPullUp);
static InputPin<kInput6_PinNumber> input_6_pin(kPullUp);
static InputPin<kInput7_PinNumber> input_7_pin(kPullUp);
static InputPin<kInput8_PinNumber> input_8_pin(kPullUp);
static InputPin<kInput9_PinNumber> input_9_pin(kPullUp);
static InputPin<kInput10_PinNumber> input_10_pin(kPullUp);
static InputPin<kInput11_PinNumber> input_11_pin(kPullUp);
static InputPin<kInput12_PinNumber> input_12_pin(kPullUp);
#endif //__ARM

//#ifdef __AVR
/* Note: v7 boards have external strong pullups on GPIO2 pins (2.7K ohm).
 *	v6 and earlier use internal pullups only. Internal pullups are set
 *	regardless of board type but are extraneous for v7 boards.
 */
//#define PIN_MODE PORT_OPC_PULLUP_gc				// pin mode. see iox192a3.h for details
//#define PIN_MODE PORT_OPC_TOTEM_gc			// alternate pin mode for v7 boards

/* Digital Input (switch) interrupt levels and vectors
 * The vectors are hard-wired to xmega ports. If you change axis port assignments
 * above you need to change these, too.
 * Interrupt level should not be PORT_INT1LVL_HI_gc or PORT_INT1LVL_LO_gc
 */
/*
#define GPIO1_INTLVL (PORT_INT0LVL_MED_gc|PORT_INT1LVL_MED_gc)
#define X_MIN_ISR_vect PORTA_INT0_vect
#define X_MAX_ISR_vect PORTA_INT1_vect
#define Y_MIN_ISR_vect PORTD_INT0_vect
#define Y_MAX_ISR_vect PORTD_INT1_vect
#define Z_MIN_ISR_vect PORTE_INT0_vect
#define Z_MAX_ISR_vect PORTE_INT1_vect
#define A_MIN_ISR_vect PORTF_INT0_vect
#define A_MAX_ISR_vect PORTF_INT1_vect
*/
//#endif

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 *
 *	AVR code assumes sys_init() and st_init() have been run previously to
 *	bind the ports and set bit IO directions, respectively. See hardware.h for details
 */

#ifdef __ARM
void gpio_init(void)
{
    /* Priority only needs set once in the system during startup.
     * However, if we wish to switch the interrupt trigger, here are other options:
     *  kPinInterruptOnRisingEdge
     *  kPinInterruptOnFallingEdge
     *
     * To change the trigger, just call pin.setInterrupts(value) at any point.
     * Note that it may cause an interrupt to fire *immediately*!
     */
    input_1_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_2_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_3_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_4_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_5_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_6_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_7_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_8_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_9_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_10_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_11_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_12_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
	return(gpio_reset());
}
#endif

#ifdef __AVR
void gpio_init(void)
{
    for (uint8_t i=0; i<DI_INPUT_PAIRS; i++) {

        // Setup input bits and interrupts
        // Must have been previously set to inputs by stepper_init()
        if (d_in[i].mode == INPUT_MODE_DISABLED) {
            hw.sw_port[i]->INT0MASK = 0;                    // disable interrupts
        } else {
            hw.sw_port[i]->DIRCLR = SW_MIN_BIT_bm;          // set min input
            hw.sw_port[i]->PIN6CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
            hw.sw_port[i]->INT0MASK = SW_MIN_BIT_bm;        // interrupt on min switch
        }
        if (d_in[i+1].mode == INPUT_MODE_DISABLED) {
            hw.sw_port[i]->INT1MASK = 0;
        } else {
            hw.sw_port[i]->DIRCLR = SW_MAX_BIT_bm;          // set max input
            hw.sw_port[i]->PIN7CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
            hw.sw_port[i]->INT1MASK = SW_MAX_BIT_bm;        // max on INT1
        }
        // Set interrupt level. Interrupts must be enabled in main()
        hw.sw_port[i]->INTCTRL = GPIO1_INTLVL;                  // see hardware.h for setting
    }
    return(gpio_reset());
}
/*
    for (uint8_t i=0; i<NUM_SWITCH_PAIRS; i++) {
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
        hw.sw_port[i]->INTCTRL = GPIO1_INTLVL;				// see hardware.h for setting
    }
    return(gpio_reset());
}
*/
#endif

void gpio_reset(void)
{
#ifdef __AVR
    for (uint8_t i=0; i < NUM_SWITCHES; i++) {
        sw.debounce[i] = SW_IDLE;
        read_switch(i);
    }
    sw.limit_flag = false;
#endif

    io_di_t *in;

    for (uint8_t i=0; i<DI_CHANNELS; i++) {
        in = &d_in[i];
        if (in->mode == INPUT_MODE_DISABLED) {
            in->state = INPUT_DISABLED;
            continue;
        }
        int8_t pin_value_corrected = (_read_raw_pin(i+1) ^ (in->mode ^ 1));	// correct for NO or NC mode
		in->state = pin_value_corrected;
//        in->state = (_read_raw_pin(i+1) ^ (in->mode ^ 1));    // correct for NO or NC mode
        in->lockout_ms = INPUT_LOCKOUT_MS;
#ifdef __ARM
        in->lockout_timer = SysTickTimer.getValue();
#else
        in->lockout_timer = SysTickTimer_getValue();
#endif
    }


/*
//#ifdef __ARM
	for (uint8_t i=0; i<DI_CHANNELS; i++) {
        if (io.in[i].mode == INPUT_MODE_DISABLED) {
            io.in[i].state = INPUT_DISABLED;
            continue;
        }
//        int8_t pin_value_corrected = (_read_raw_pin(i+1) ^ (io.in[i].mode ^ 1));	// correct for NO or NC mode
//		io.in[i].state = pin_value_corrected;
        io.in[i].state = (_read_raw_pin(i+1) ^ (io.in[i].mode ^ 1));    // correct for NO or NC mode
        io.in[i].lockout_ms = INPUT_LOCKOUT_MS;
#ifdef __ARM
		io.in[i].lockout_timer = SysTickTimer.getValue();
#else
		io.in[i].lockout_timer = SysTickTimer_getValue();
#endif
	}
//#endif
*/
}

/*
 * _read_raw_pin() - primitive to read an input pin without any conditioning
 */
static bool _read_raw_pin(const uint8_t input_num_ext)
{
#ifdef __ARM
    switch(input_num_ext) {
        case 1: { return (input_1_pin.get() != 0); }
        case 2: { return (input_2_pin.get() != 0); }
        case 3: { return (input_3_pin.get() != 0); }
        case 4: { return (input_4_pin.get() != 0); }
        case 5: { return (input_5_pin.get() != 0); }
        case 6: { return (input_6_pin.get() != 0); }
        case 7: { return (input_7_pin.get() != 0); }
        case 8: { return (input_8_pin.get() != 0); }
        case 9: { return (input_9_pin.get() != 0); }
        case 10: { return (input_10_pin.get() != 0); }
        case 11: { return (input_11_pin.get() != 0); }
        case 12: { return (input_12_pin.get() != 0); }
        default: { return false; } // ERROR
    }
#endif //__ARM

#ifdef __AVR
    switch (input_num_ext) {
        case 1: { return ((hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm) != 0); }
        case 2: { return ((hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm) != 0); }
        case 3: { return ((hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm) != 0); }
        case 4: { return ((hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm) != 0); }
        case 5: { return ((hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm) != 0); }
        case 6: { return ((hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm) != 0); }
        case 7: { return ((hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm) != 0); }
        case 8: { return ((hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm) != 0); }
        default: { return false; } // ERROR
    }
#endif //__AVR
}

/******************************
 * Interrupt Service Routines *
 ******************************/
/*
 * ARM pin change interrupts
 *
 * NOTE: InputPin<>.get() returns a uint32_t, and will NOT necessarily be 1 for true.
 * The actual values will be the pin's port mask or 0, so you must check for non-zero.
 */

#ifdef __ARM
MOTATE_PIN_INTERRUPT(kInput1_PinNumber) { _dispatch_pin(_condition_pin(1, (input_1_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput2_PinNumber) { _dispatch_pin(_condition_pin(2, (input_2_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput3_PinNumber) { _dispatch_pin(_condition_pin(3, (input_3_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput4_PinNumber) { _dispatch_pin(_condition_pin(4, (input_4_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput5_PinNumber) { _dispatch_pin(_condition_pin(5, (input_5_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput6_PinNumber) { _dispatch_pin(_condition_pin(6, (input_6_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput7_PinNumber) { _dispatch_pin(_condition_pin(7, (input_7_pin.get() != 0))); }
MOTATE_PIN_INTERRUPT(kInput8_PinNumber) { _dispatch_pin(_condition_pin(8, (input_8_pin.get() != 0))); }
//MOTATE_PIN_INTERRUPT(kInput9_PinNumber) { _dispatch_pin(_condition_pin(9, (input_9_pin.get() != 0))); }
//MOTATE_PIN_INTERRUPT(kInput10_PinNumber) { _dispatch_pin(_condition_pin(9, (input_10_pin.get() != 0))); }
//MOTATE_PIN_INTERRUPT(kInput11_PinNumber) { _dispatch_pin(_condition_pin(10, (input_11_pin.get() != 0))); }
//MOTATE_PIN_INTERRUPT(kInput12_PinNumber) { _dispatch_pin(_condition_pin(11, (input_12_pin.get() != 0))); }
/*
MOTATE_PIN_INTERRUPT(kInput1_PinNumber) { _handle_pin_changed(1, (input_1_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput2_PinNumber) { _handle_pin_changed(2, (input_2_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput3_PinNumber) { _handle_pin_changed(3, (input_3_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput4_PinNumber) { _handle_pin_changed(4, (input_4_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput5_PinNumber) { _handle_pin_changed(5, (input_5_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput6_PinNumber) { _handle_pin_changed(6, (input_6_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput7_PinNumber) { _handle_pin_changed(7, (input_7_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput8_PinNumber) { _handle_pin_changed(8, (input_8_pin.get() != 0)); }
//MOTATE_PIN_INTERRUPT(kInput9_PinNumber) { _handle_pin_changed(9, (input_9_pin.get() != 0)); }
//MOTATE_PIN_INTERRUPT(kInput10_PinNumber) { _handle_pin_changed(9, (input_10_pin.get() != 0)); }
//MOTATE_PIN_INTERRUPT(kInput11_PinNumber) { _handle_pin_changed(10, (input_11_pin.get() != 0)); }
//MOTATE_PIN_INTERRUPT(kInput12_PinNumber) { _handle_pin_changed(11, (input_12_pin.get() != 0)); }
*/
#endif

#ifdef __AVR
ISR(X_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(1, (hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(X_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(2, (hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(Y_MIN_ISR_vect)	{
    _dispatch_pin(_condition_pin(3, (hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm) != 0));
}
ISR(Y_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(4, (hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(Z_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(5, (hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(Z_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(6, (hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(A_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(7, (hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(A_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(8, (hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm) != 0)); }

/*
ISR(X_MIN_ISR_vect)	{ _condition_avr(1);}
ISR(X_MAX_ISR_vect)	{ _condition_avr(2);}
ISR(Y_MIN_ISR_vect)	{ _condition_avr(3);}
ISR(Y_MAX_ISR_vect)	{ _condition_avr(4);}
ISR(Z_MIN_ISR_vect)	{ _condition_avr(5);}
ISR(Z_MAX_ISR_vect)	{ _condition_avr(6);}
ISR(A_MIN_ISR_vect)	{ _condition_avr(7);}
ISR(A_MAX_ISR_vect)	{ _condition_avr(8);}

static uint8_t _condition_avr(const uint8_t input_num_ext)
{
    uint8_t sw_num = input_num_ext-1;

    if (sw.mode[sw_num] == SW_MODE_DISABLED) {      // this is never supposed to happen
        return (0);
    }
    if (sw.debounce[sw_num] == SW_LOCKOUT) {		// exit if switch is in lockout
        return (0);
    }
    sw.debounce[sw_num] = SW_DEGLITCHING;			// either transitions state from IDLE or overwrites it
    sw.count[sw_num] = -SW_DEGLITCH_TICKS;			// reset deglitch count regardless of entry state

    uint8_t raw_pin = 0;
    switch (input_num_ext) {
        case 1: { raw_pin = hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm; break;}
        case 2: { raw_pin = hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm; break;}
        case 3: { raw_pin = hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm; break;}
        case 4: { raw_pin = hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm; break;}
        case 5: { raw_pin = hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm; break;}
        case 6: { raw_pin = hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm; break;}
        case 7: { raw_pin = hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm; break;}
        case 8: { raw_pin = hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm; break;}
        default: { return (0); } // ERROR
    }
    if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
        sw.state[sw_num] = ((raw_pin == 0) ? SW_CLOSED : SW_OPEN);// confusing. An NO switch drives the pin LO when thrown
    } else {
        sw.state[sw_num] = ((raw_pin != 0) ? SW_CLOSED : SW_OPEN);
    }
//    sw.state[sw_num] = (raw_pin == 0) ^ (sw.switch_type == SW_TYPE_NORMALLY_CLOSED); // correct for NO/NC setting
    return (input_num_ext);
}
*/
#endif //__AVR

/*
 * _condition_pin() - debounce and condition raw pin state
 *
 *  Input numbers are external, meaning they start at 1.
 *  Return pin number 0 if no further action is required (no dispatch)
 */
static uint8_t _condition_pin(const uint8_t input_num_ext, const int8_t pin_value)
{
//    io_di_t *in = &io.in[input_num_ext-1];  // array index is one less than input number
    io_di_t *in = &d_in[input_num_ext-1];  // array index is one less than input number

    // return if input is disabled (not supposed to happen)
    if (in->mode == INPUT_MODE_DISABLED) {
        in->state = INPUT_DISABLED;
        return (0);
    }

    // return if the input is in lockout period (take no action)
#ifdef __ARM
    if (SysTickTimer.getValue() < in->lockout_timer) { return (0); }
#else
    if (SysTickTimer_getValue() < in->lockout_timer) { return (0); }
#endif
    // return if no change in state
    int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));	// correct for NO or NC mode
    if (in->state == pin_value_corrected) {
        return (0);
    }

    // record the changed state
    in->state = pin_value_corrected;
#ifdef __ARM
    in->lockout_timer = SysTickTimer.getValue() + in->lockout_ms;
#else
    in->lockout_timer = SysTickTimer_getValue() + in->lockout_ms;
#endif
    if (pin_value_corrected == INPUT_ACTIVE) {
        in->edge = INPUT_EDGE_LEADING;
    } else {
        in->edge = INPUT_EDGE_TRAILING;
    }
    return (input_num_ext);
}

/*
 * _dispatch_pin() - execute pin changes
 *
 *  Run _condition_pin() before calling this function.
 *  Take no action if input number is zero
 */

static void _dispatch_pin(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {               // no action if input number is zero
        return;
    }

//    io_di_t *in = &io.in[input_num_ext-1];  // array index is one less than input number
    io_di_t *in = &d_in[input_num_ext-1];  // array index is one less than input number

    // perform homing operations if in homing mode
    if (in->homing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
//            cm_start_hold();
            cm_request_feedhold();
        }
        return;
    }

    // perform probing operations if in probing mode
    if (in->probing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
//            cm_start_hold();
            cm_request_feedhold();
        }
        return;
    }

    // *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***

    // trigger the action on leading edges

    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->action == INPUT_ACTION_STOP) {
//			cm_start_hold();
            cm_request_feedhold();
        }
        if (in->action == INPUT_ACTION_FAST_STOP) {
//			cm_start_hold();                        // for now is same as STOP
            cm_request_feedhold();
        }
        if (in->action == INPUT_ACTION_HALT) {
            cm_halt_all();					        // hard stop, including spindle and coolant
        }
        if (in->action == INPUT_ACTION_PANIC) {
            char msg[10];
            sprintf_P(msg, PSTR("input %d"), input_num_ext);
            cm_panic(STAT_PANIC, msg);
        }
        if (in->action == INPUT_ACTION_RESET) {
            hw_hard_reset();
        }
    }

    // these functions trigger on the leading edge
    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->function == INPUT_FUNCTION_LIMIT) {
            cm.limit_requested = input_num_ext;

        } else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
            cm.shutdown_requested = input_num_ext;

        } else if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_disengaged = input_num_ext;
        }
    }

    // trigger interlock release on trailing edge
    if (in->edge == INPUT_EDGE_TRAILING) {
        if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_reengaged = input_num_ext;
        }
    }
    sr_request_status_report(SR_TIMED_REQUEST);     // v8 style
//    sr_request_status_report(SR_REQUEST_TIMED);   // g2 style
}

/*
 * _handle_pin_changed() - ISR helper
 *
 * Since we set the interrupt to kPinInterruptOnChange _handle_pin_changed() should
 * only be called when the pin *changes* values, so we can assume that the current
 * pin value is not the same as the previous value. Note that the value may have
 * changed rapidly, and may even have changed again since the interrupt was triggered.
 * In this case a second interrupt will likely follow this one immediately after exiting.
 *
 *  input_num is the input channel, 1 - N
 *  pin_value = 1 if pin is set, 0 otherwise
 */
/*
static void _handle_pin_changed(const uint8_t input_num_ext, const int8_t pin_value)
{
    io_di_t *in = &io.in[input_num_ext-1];  // array index is one less than input number

    // return if input is disabled (not supposed to happen)
	if (in->mode == INPUT_MODE_DISABLED) {
    	in->state = INPUT_DISABLED;
        return;
    }

    // return if the input is in lockout period (take no action)
#ifdef __ARM
    if (SysTickTimer.getValue() < in->lockout_timer) { return; }
#else
    if (SysTickTimer_getValue() < in->lockout_timer) { return; }
#endif
	// return if no change in state
	int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));	// correct for NO or NC mode
	if ( in->state == pin_value_corrected ) {
//    	in->edge = INPUT_EDGE_NONE;        // edge should only be reset by function or opposite edge
    	return;
	}

	// record the changed state
    in->state = pin_value_corrected;
#ifdef __ARM
	in->lockout_timer = SysTickTimer.getValue() + in->lockout_ms;
#else
	in->lockout_timer = SysTickTimer_getValue() + in->lockout_ms;
#endif
    if (pin_value_corrected == INPUT_ACTIVE) {
        in->edge = INPUT_EDGE_LEADING;
    } else {
        in->edge = INPUT_EDGE_TRAILING;
    }

    // perform homing operations if in homing mode
    if (in->homing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
//            cm_start_hold();
        }
        return;
    }

    // perform probing operations if in probing mode
    if (in->probing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
//            cm_start_hold();
        }
        return;
    }

	// *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***

    // trigger the action on leading edges

    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->action == INPUT_ACTION_STOP) {
//			cm_start_hold();
        }
        if (in->action == INPUT_ACTION_FAST_STOP) {
//			cm_start_hold();                        // for now is same as STOP
        }
        if (in->action == INPUT_ACTION_HALT) {
	        cm_halt_all();					        // hard stop, including spindle and coolant
        }
        if (in->action == INPUT_ACTION_PANIC) {
	        char msg[10];
	        sprintf_P(msg, PSTR("input %d"), input_num_ext);
	        cm_panic(STAT_PANIC, msg);
        }
        if (in->action == INPUT_ACTION_RESET) {
            hw_hard_reset();
        }
    }

	// these functions trigger on the leading edge
    if (in->edge == INPUT_EDGE_LEADING) {
		if (in->function == INPUT_FUNCTION_LIMIT) {
			cm.limit_requested = input_num_ext;

		} else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
			cm.shutdown_requested = input_num_ext;

		} else if (in->function == INPUT_FUNCTION_INTERLOCK) {
		    cm.safety_interlock_disengaged = input_num_ext;
		}
    }

    // trigger interlock release on trailing edge
    if (in->edge == INPUT_EDGE_TRAILING) {
        if (in->function == INPUT_FUNCTION_INTERLOCK) {
		    cm.safety_interlock_reengaged = input_num_ext;
        }
    }
//    sr_request_status_report(SR_TIMED_REQUEST);   //+++++ Put this one back in.
}
*/

/********************************************
 **** Digital Input Supporting Functions ****
 ********************************************/
/*
 * switch_rtc_callback() - called from RTC for each RTC tick.
 *
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */

#ifdef __AVR
void switch_rtc_callback(void)
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		if (sw.mode[i] == SW_MODE_DISABLED || sw.debounce[i] == SW_IDLE)
            continue;

		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.debounce[i] = SW_IDLE;
            // check if the state has changed while we were in lockout...
            uint8_t old_state = sw.state[i];
            if(old_state != read_switch(i)) {
                sw.debounce[i] = SW_DEGLITCHING;
                sw.count[i] = -SW_DEGLITCH_TICKS;
            }
            continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.debounce[i] = SW_LOCKOUT;
			if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {		// regardless of switch type
				cm_request_feedhold();
			} else if (sw.mode[i] & SW_LIMIT_BIT) {		// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
}
#endif //__AVR

/*
 * gpio_set_homing_mode()   - set/clear input to homing mode
 * gpio_set_probing_mode()  - set/clear input to probing mode
 * gpio_read_input()        - read conditioned input
 *
 (* Note: input_num_ext means EXTERNAL input number -- 1-based
 */
void  gpio_set_homing_mode(const uint8_t input_num_ext, const bool is_homing)
{
    if (input_num_ext == 0) {
        return;
    }
//    io.in[input_num_ext-1].homing_mode = is_homing;
    d_in[input_num_ext-1].homing_mode = is_homing;
}

void  gpio_set_probing_mode(const uint8_t input_num_ext, const bool is_probing)
{
    if (input_num_ext == 0) {
        return;
    }
//    io.in[input_num_ext-1].probing_mode = is_probing;
    d_in[input_num_ext-1].probing_mode = is_probing;
}

bool gpio_read_input(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {
        return false;
    }
//    return (io.in[input_num_ext-1].state);
    return (d_in[input_num_ext-1].state);
}

/* Xmega Functions (retire these as possible)
 * get_switch_mode()  - return switch mode setting
 * get_limit_thrown() - return true if a limit was tripped
 * get_switch_num()   - return switch number most recently thrown
 * set_switch_type()
 * get_switch_type()
 */

#ifdef __AVR
uint8_t get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
uint8_t get_limit_switch_thrown(void) { return(sw.limit_flag);}
uint8_t get_switch_thrown(void) { return(sw.sw_num_thrown);}
void set_switch_type( uint8_t switch_type ) { sw.switch_type = switch_type; }
uint8_t get_switch_type() { return sw.switch_type; }
#endif

/*
 * read_switch() - read a switch directly with no interrupts or deglitching
 */
#ifdef __AVR
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
#endif //__AVR

//======================== Parallel IO Functions ===============================
/*
 * IndicatorLed_set() 	- fake out for IndicatorLed.set() until we get Motate running
 * IndicatorLed_clear() - fake out for IndicatorLed.clear() until we get Motate running
 * IndicatorLed_toggle()- fake out for IndicatorLed.toggle() until we get Motate running
 */

void IndicatorLed_set()
{
	gpio_led_on(INDICATOR_LED);
	cs.led_state = 1;
}

void IndicatorLed_clear()
{
	gpio_led_off(INDICATOR_LED);
	cs.led_state = 0;
}

void IndicatorLed_toggle()
{
	if (cs.led_state == 0) {
		gpio_led_on(INDICATOR_LED);
		cs.led_state = 1;
	} else {
		gpio_led_off(INDICATOR_LED);
		cs.led_state = 0;
	}
}

/*
 * gpio_led_on() - turn led on - assumes TinyG LED mapping
 * gpio_led_off() - turn led on - assumes TinyG LED mapping
 * gpio_led_toggle()
 */

void gpio_led_on(uint8_t led)
{
//	if (led == 0) return (gpio_set_bit_on(0x08));
//	if (led == 1) return (gpio_set_bit_on(0x04));
//	if (led == 2) return (gpio_set_bit_on(0x02));
//	if (led == 3) return (gpio_set_bit_on(0x01));

	if (led == 0) gpio_set_bit_on(0x08); else
	if (led == 1) gpio_set_bit_on(0x04); else
	if (led == 2) gpio_set_bit_on(0x02); else
	if (led == 3) gpio_set_bit_on(0x01);
}

void gpio_led_off(uint8_t led)
{
//	if (led == 0) return (gpio_set_bit_off(0x08));
//	if (led == 1) return (gpio_set_bit_off(0x04));
//	if (led == 2) return (gpio_set_bit_off(0x02));
//	if (led == 3) return (gpio_set_bit_off(0x01));

	if (led == 0) gpio_set_bit_off(0x08); else
	if (led == 1) gpio_set_bit_off(0x04); else
	if (led == 2) gpio_set_bit_off(0x02); else
	if (led == 3) gpio_set_bit_off(0x01);
}

void gpio_led_toggle(uint8_t led)
{
	if (led == 0) {
		if (gpio_read_bit(0x08)) {
			gpio_set_bit_off(0x08);
		} else {
			gpio_set_bit_on(0x08);
		}
	} else if (led == 1) {
		if (gpio_read_bit(0x04)) {
			gpio_set_bit_off(0x04);
		} else {
			gpio_set_bit_on(0x04);
		}
	} else if (led == 2) {
		if (gpio_read_bit(0x02)) {
			gpio_set_bit_off(0x02);
		} else {
			gpio_set_bit_on(0x02);
		}
	} else if (led == 3) {
		if (gpio_read_bit(0x08)) {
			gpio_set_bit_off(0x08);
		} else {
			gpio_set_bit_on(0x08);
		}
	}
}

/*
 * gpio_read_bit() - return true if bit is on, false if off
 * gpio_set_bit_on() - turn bit on
 * gpio_set_bit_off() - turn bit on
 *
 *	These functions have an inner remap depending on what hardware is running
 */

uint8_t gpio_read_bit(uint8_t b)
{
	if (b & 0x08) { return (hw.out_port[0]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x04) { return (hw.out_port[1]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x02) { return (hw.out_port[2]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x01) { return (hw.out_port[3]->IN & GPIO1_OUT_BIT_bm); }
	return (0);
}

void gpio_set_bit_on(uint8_t b)
{
	if (b & 0x08) { hw.out_port[0]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { hw.out_port[1]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { hw.out_port[2]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { hw.out_port[3]->OUTSET = GPIO1_OUT_BIT_bm; }
}

void gpio_set_bit_off(uint8_t b)
{
	if (b & 0x08) { hw.out_port[0]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { hw.out_port[1]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { hw.out_port[2]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { hw.out_port[3]->OUTCLR = GPIO1_OUT_BIT_bm; }
}



/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

static stat_t _io_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
{
	if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_ui8(nv);		// will this work in -1 is a valid value?
	gpio_reset();
	return (STAT_OK);
}

stat_t io_set_mo(nvObj_t *nv)			// input type or disabled
{
	if ((nv->value < INPUT_MODE_DISABLED) || (nv->value >= INPUT_MODE_MAX)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_int8(nv);
	gpio_reset();
	return (STAT_OK);
}

stat_t io_set_ac(nvObj_t *nv)			// input action
{
	return (_io_set_helper(nv, INPUT_ACTION_NONE, INPUT_ACTION_MAX));
}

stat_t io_set_fn(nvObj_t *nv)			// input function
{
	return (_io_set_helper(nv, INPUT_FUNCTION_NONE, INPUT_FUNCTION_MAX));
}

/*
 *  io_get_input() - return input state given an nv object
 */
stat_t io_get_input(nvObj_t *nv)
{
    // the token has been stripped down to an ASCII digit string - use it as an index
//    nv->value = io.in[strtol(nv->token, NULL, 10)-1].state;
    nv->value = d_in[strtol(nv->token, NULL, 10)-1].state;
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

stat_t sw_set_st(nvObj_t *nv)			// switch type (global)
{
	set_01(nv);
	gpio_init();
	return (STAT_OK);
}

stat_t sw_set_sw(nvObj_t *nv)			// switch setting
{
	if (nv->value > SW_MODE_MAX_VALUE) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	set_ui8(nv);
	gpio_init();
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_st[] PROGMEM = "[st]  switch type%18d [0=NO,1=NC]\n";
void sw_print_st(nvObj_t *nv) { text_print_ui8(nv, fmt_st);}

//static const char fmt_ss[] PROGMEM = "Switch %s state:     %d\n";
//void sw_print_ss(nvObj_t *nv) { fprintf(stderr, fmt_ss, nv->token, (uint8_t)nv->value);}

/*
static const char msg_sw0[] PROGMEM = "Disabled";
static const char msg_sw1[] PROGMEM = "NO homing";
static const char msg_sw2[] PROGMEM = "NO homing & limit";
static const char msg_sw3[] PROGMEM = "NC homing";
static const char msg_sw4[] PROGMEM = "NC homing & limit";
static const char *const msg_sw[] PROGMEM = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };
*/

static const char fmt_gpio_mo[] PROGMEM = "[%smo] input mode%15d [-1=disabled, 0=NO,1=NC]\n";
static const char fmt_gpio_ac[] PROGMEM = "[%sac] input action%13d [0=none,1=stop,2=halt,3=stop_steps,4=panic,5=reset]\n";
static const char fmt_gpio_fn[] PROGMEM = "[%sfn] input function%11d [0=none,1=limit,2=interlock,3=shutdown]\n";
static const char fmt_gpio_in[] PROGMEM = "Input %s state: %5d\n";

static void _print_di(nvObj_t *nv, const char *format)
{
    printf_P(format, nv->group, (int)nv->value);
}

void io_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_mo);}
void io_print_ac(nvObj_t *nv) {_print_di(nv, fmt_gpio_ac);}
void io_print_fn(nvObj_t *nv) {_print_di(nv, fmt_gpio_fn);}
void io_print_in(nvObj_t *nv) { printf_P(fmt_gpio_in, nv->token, (int)nv->value); }
#endif
