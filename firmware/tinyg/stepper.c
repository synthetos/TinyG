/*
 * stepper.c - stepper motor controls
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
/* 	This module provides the low-level stepper drivers and some related
 * 	functions. It dequeues lines queued by the motor_queue routines.
 * 	This is some of the most heavily optimized code in the project.
 *
 *	Note that if you want to use this for something other than TinyG
 *	you may need to stretch the step pulses. They run about 1 uSec 
 *	which is fine for the TI DRV8811/DRV8818 chips in TinyG but may 
 *	not suffice for other stepper driver hardware.
 */
/* 
 * See stepper.h for a detailed explanation of this part of the code 
 */

#include "tinyg.h"
#include "config.h"
#include "stepper.h" 	
#include "planner.h"
//#include "motatePins.h"		// defined in hardware.h   Not needed here
//#include "motateTimers.h"
//#include "hardware.h"
#include "system.h"				// Xmega only. Goes away in favor of hardware.h
#include "util.h"

#include <avr/interrupt.h>
#include "xmega/xmega_rtc.h"	// Xmega only. Goes away with RTC refactoring

//#define ENABLE_DIAGNOSTICS
#ifdef ENABLE_DIAGNOSTICS
#define INCREMENT_DIAGNOSTIC_COUNTER(motor) st_run.m[motor].step_count_diagnostic++;
#else
#define INCREMENT_DIAGNOSTIC_COUNTER(motor)	// chose this one to disable counters
#endif

// Setup local resources

static void _load_move(void);
static void _request_load_move(void);

/*
 * Stepper structures
 *
 *	There are 4 sets of structures involved in this operation;
 *
 *	data structure:						static to:		runs at:
 *	  mpBuffer planning buffers (bf)	  planner.c		  main loop
 *	  mrRuntimeSingleton (mr)			  planner.c		  MED ISR
 *	  stPrepSingleton (sp)				  stepper.c		  MED ISR
 *	  stRunSingleton (st)				  stepper.c		  HI ISR
 *  
 *	Care has been taken to isolate actions on these structures to the 
 *	execution level in which they run and to use the minimum number of 
 *	volatiles in these structures. This allows the compiler to optimize
 *	the stepper inner-loops better.
 */

// Runtime structure. Used exclusively by step generation ISR (HI)

typedef struct stRunMotor { 		// one per controlled motor
	int32_t phase_increment;		// total steps in axis times substeps factor
	int32_t phase_accumulator;		// DDA phase angle accumulator for axis
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity
} stRunMotor_t;

typedef struct stRunSingleton {		// Stepper static values and axis parameters
	uint16_t magic_start;			// magic number to test memory integrity	
	int32_t dda_ticks_downcount;	// tick down-counter (unscaled)
	int32_t dda_ticks_X_substeps;	// ticks multiplied by scaling factor
	uint32_t motor_disable_systick;	// sys_tick at which to disable the motors
	stRunMotor_t m[MOTORS];			// runtime motor structures
} stRunSingleton_t;

// Prep-time structure. Used by exec/prep ISR (MED) and read-only during load
// Must be careful about volatiles in this one

typedef struct stPrepMotor {
 	uint32_t phase_increment; 		// total steps in axis times substep factor
	int8_t dir;						// b0 = direction
} stPrepMotor_t;

typedef struct stPrepSingleton {
	uint16_t magic_start;			// magic number to test memory integrity	
	uint8_t move_type;				// move type
	volatile uint8_t exec_state;	// move execution state 
	volatile uint8_t reset_flag;	// TRUE if accumulator should be reset
	uint32_t prev_ticks;			// tick count from previous move
	uint16_t dda_period;			// DDA or dwell clock period setting
	uint32_t dda_ticks;				// DDA or dwell ticks for the move
	uint32_t dda_ticks_X_substeps;	// DDA ticks scaled by substep factor
//	float segment_velocity;			// record segment velocity for diagnostics
	stPrepMotor_t m[MOTORS];		// per-motor structs
} stPrepSingleton_t;

// Allocate static structures
static stRunSingleton_t st_run;
static stPrepSingleton_t st_prep;

magic_t st_get_stepper_run_magic() { return (st_run.magic_start);}
magic_t st_get_stepper_prep_magic() { return (st_prep.magic_start);}

/* 
 * stepper_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 *		This init is a precursor for gpio_init()
 * 	  - microsteps are setup during config_init()
 *	  - motor polarity is setup during config_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void stepper_init()
{
//	You can assume all values are zeroed. If not, use this:
//	memset(&st, 0, sizeof(st));	// clear all values, pointers and status

	memset(&st_run, 0, sizeof(st_run));		// clear all values, pointers and status
	st_run.magic_start = MAGICNUM;
	st_prep.magic_start = MAGICNUM;

	// Configure virtual ports
	PORTCFG.VPCTRLA = PORTCFG_VP0MAP_PORT_MOTOR_1_gc | PORTCFG_VP1MAP_PORT_MOTOR_2_gc;
	PORTCFG.VPCTRLB = PORTCFG_VP2MAP_PORT_MOTOR_3_gc | PORTCFG_VP3MAP_PORT_MOTOR_4_gc;

	// setup ports
	for (uint8_t i=0; i<MOTORS; i++) {
		device.st_port[i]->DIR = MOTOR_PORT_DIR_gm;  // sets outputs for motors & GPIO1, and GPIO2 inputs
		device.st_port[i]->OUT = MOTOR_ENABLE_BIT_bm;// zero port bits AND disable motor
	}
	// setup DDA timer
	TIMER_DDA.CTRLA = STEP_TIMER_DISABLE;		// turn timer off
	TIMER_DDA.CTRLB = STEP_TIMER_WGMODE;		// waveform mode
	TIMER_DDA.INTCTRLA = TIMER_DDA_INTLVL;		// interrupt mode

	// setup DWELL timer
	TIMER_DWELL.CTRLA = STEP_TIMER_DISABLE;		// turn timer off
	TIMER_DWELL.CTRLB = STEP_TIMER_WGMODE;		// waveform mode
	TIMER_DWELL.INTCTRLA = TIMER_DWELL_INTLVL;	// interrupt mode

	// setup software interrupt load timer
	TIMER_LOAD.CTRLA = STEP_TIMER_DISABLE;		// turn timer off
	TIMER_LOAD.CTRLB = STEP_TIMER_WGMODE;		// waveform mode
	TIMER_LOAD.INTCTRLA = TIMER_LOAD_INTLVL;	// interrupt mode
	TIMER_LOAD.PER = SWI_PERIOD;				// set period

	// setup software interrupt exec timer
	TIMER_EXEC.CTRLA = STEP_TIMER_DISABLE;		// turn timer off
	TIMER_EXEC.CTRLB = STEP_TIMER_WGMODE;		// waveform mode
	TIMER_EXEC.INTCTRLA = TIMER_EXEC_INTLVL;	// interrupt mode
	TIMER_EXEC.PER = SWI_PERIOD;				// set period

	st_prep.exec_state = PREP_BUFFER_OWNED_BY_EXEC;
}

/* 
 * st_set_motor_disable_timeout() - set the timeout in the config
 */

void st_set_motor_disable_timeout(float seconds)
{
	cfg.motor_disable_timeout = min(STEPPER_MAX_TIMEOUT_SECONDS, max(seconds, STEPPER_MIN_TIMEOUT_SECONDS));
}
/*
void st_set_motor_disable_timeout(uint32_t seconds)
{
	st_run.motor_disable_systick = SysTickTimer_getValue() + (seconds * 1000);
}
*/
/* 
 * st_do_motor_disable_timeout()  - execute the timeout
 *
 *	Sets a point N seconds in the future when the motors will be disabled (time out)
 *	Can be called at any time to extend N seconds from the current time
 */

void st_do_motor_disable_timeout()
{
	st_run.motor_disable_systick = SysTickTimer_getValue() + (uint32_t)(cfg.motor_disable_timeout * 1000);
}

/* 
 * st_enable_motor()  - enable a motor
 * st_disable_motor() - disable a motor
 * st_enable_motors() - enable all motors with $pm set to POWER_MODE_DELAYED_DISABLE
 * st_disable_motors()- disable all motors
 * st_motor_disable_callback()
 */

void st_enable_motor(const uint8_t motor)
{
	if (motor == MOTOR_1) { PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_2) { PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_3) { PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_4) { PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
}

void st_disable_motor(const uint8_t motor)
{
	if (motor == MOTOR_1) { PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_2) { PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_3) { PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_4) { PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
}

void st_enable_motors()
{
	if (cfg.m[MOTOR_1].power_mode == ENABLE_AXIS_DURING_CYCLE) { st_enable_motor(MOTOR_1);}
	if (cfg.m[MOTOR_2].power_mode == ENABLE_AXIS_DURING_CYCLE) { st_enable_motor(MOTOR_2);}
	if (cfg.m[MOTOR_3].power_mode == ENABLE_AXIS_DURING_CYCLE) { st_enable_motor(MOTOR_3);}
	if (cfg.m[MOTOR_4].power_mode == ENABLE_AXIS_DURING_CYCLE) { st_enable_motor(MOTOR_4);}
	st_do_motor_disable_timeout();
}

void st_disable_motors()
{
	st_disable_motor(MOTOR_1);
	st_disable_motor(MOTOR_2);
	st_disable_motor(MOTOR_3);
	st_disable_motor(MOTOR_4);
}

stat_t st_motor_disable_callback() 	// called by controller
{
	if (SysTickTimer_getValue() < st_run.motor_disable_systick ) {
		return (STAT_NOOP);
	}
	st_disable_motors();
	return (STAT_OK);
}

/***** Interrupt Service Routines *****
 *
 * ISR - DDA timer interrupt routine - service ticks from DDA timer
 *
 *	The step bit pulse width is ~1 uSec, which is OK for the TI DRV8811's.
 *	If you need to stretch the pulse I recommend moving the port OUTCLRs
 *	to the end of the routine. If you need more time than that use a 
 *	pulse OFF timer like grbl does so as not to spend any more time in 
 *	the ISR, which would limit the upper range of the DDA frequency.
 *
 *	Uses direct struct addresses and literal values for hardware devices -
 *	it's faster than using indexed timer and port accesses. I checked.
 *	Even when -0s or -03 is used.
 */

ISR(TIMER_DDA_ISR_vect)
{
	if ((st_run.m[MOTOR_1].phase_accumulator += st_run.m[MOTOR_1].phase_increment) > 0) {
		PORT_MOTOR_1_VPORT.OUT |= STEP_BIT_bm;	// turn step bit on
 		st_run.m[MOTOR_1].phase_accumulator -= st_run.dda_ticks_X_substeps;
		PORT_MOTOR_1_VPORT.OUT &= ~STEP_BIT_bm;	// turn step bit off in ~1 uSec
	}
	if ((st_run.m[MOTOR_2].phase_accumulator += st_run.m[MOTOR_2].phase_increment) > 0) {
		PORT_MOTOR_2_VPORT.OUT |= STEP_BIT_bm;
 		st_run.m[MOTOR_2].phase_accumulator -= st_run.dda_ticks_X_substeps;
		PORT_MOTOR_2_VPORT.OUT &= ~STEP_BIT_bm;
	}
	if ((st_run.m[MOTOR_3].phase_accumulator += st_run.m[MOTOR_3].phase_increment) > 0) {
		PORT_MOTOR_3_VPORT.OUT |= STEP_BIT_bm;
 		st_run.m[MOTOR_3].phase_accumulator -= st_run.dda_ticks_X_substeps;
		PORT_MOTOR_3_VPORT.OUT &= ~STEP_BIT_bm;
	}
	if ((st_run.m[MOTOR_4].phase_accumulator += st_run.m[MOTOR_4].phase_increment) > 0) {
		PORT_MOTOR_4_VPORT.OUT |= STEP_BIT_bm;
 		st_run.m[MOTOR_4].phase_accumulator -= st_run.dda_ticks_X_substeps;
		PORT_MOTOR_4_VPORT.OUT &= ~STEP_BIT_bm;
	}
	if (--st_run.dda_ticks_downcount == 0) {	// end move
 		TIMER_DDA.CTRLA = STEP_TIMER_DISABLE;	// disable DDA timer
		st_do_motor_disable_timeout();
		// power-down motors if this feature is enabled
		if (cfg.m[MOTOR_1].power_mode == DISABLE_AXIS_WHEN_IDLE) PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_2].power_mode == DISABLE_AXIS_WHEN_IDLE) PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_3].power_mode == DISABLE_AXIS_WHEN_IDLE) PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_4].power_mode == DISABLE_AXIS_WHEN_IDLE) PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		_load_move();							// load the next move
	}
}

ISR(TIMER_DWELL_ISR_vect) {						// DWELL timer interrupt
	if (--st_run.dda_ticks_downcount == 0) {
 		TIMER_DWELL.CTRLA = STEP_TIMER_DISABLE;	// disable DWELL timer
//		mp_end_dwell();							// free the planner buffer
		_load_move();
	}
}

ISR(TIMER_LOAD_ISR_vect) {						// load steppers SW interrupt
 	TIMER_LOAD.CTRLA = STEP_TIMER_DISABLE;		// disable SW interrupt timer
	_load_move();
}

ISR(TIMER_EXEC_ISR_vect) {						// exec move SW interrupt
 	TIMER_EXEC.CTRLA = STEP_TIMER_DISABLE;		// disable SW interrupt timer

	// exec_move
   	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
	   	if (mp_exec_move() != STAT_NOOP) {
		   	st_prep.exec_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
		   	_request_load_move();
	   	}
   	}
	
}

/* Software interrupts to fire the above
 * st_test_exec_state()	   - return TRUE if exec/prep can run
 * _request_load_move()    - SW interrupt to request to load a move
 *	st_request_exec_move() - SW interrupt to request to execute a move
 */

uint8_t st_test_exec_state()
{
	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
		return (true);
	}
	return (false);
}

void st_request_exec_move()
{
	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {	// bother interrupting
		TIMER_EXEC.PER = SWI_PERIOD;
		TIMER_EXEC.CTRLA = STEP_TIMER_ENABLE;			// trigger a LO interrupt
	}
}

static void _request_load_move()
{
	if (st_run.dda_ticks_downcount == 0) {				// bother interrupting
		TIMER_LOAD.PER = SWI_PERIOD;
		TIMER_LOAD.CTRLA = STEP_TIMER_ENABLE;			// trigger a HI interrupt
	} 	// else don't bother to interrupt. You'll just trigger an 
		// interrupt and find out the load routine is not ready for you
}

/*
 * _load_move() - Dequeue move and load into stepper struct
 *
 *	This routine can only be called be called from an ISR at the same or 
 *	higher level as the DDA or dwell ISR. A software interrupt has been 
 *	provided to allow a non-ISR to request a load (see st_request_load_move())
 */

void _load_move()
{
	if (st_run.dda_ticks_downcount != 0) return;					// exit if it's still busy
	if (st_prep.exec_state != PREP_BUFFER_OWNED_BY_LOADER) return;	// if there are no more moves

	// handle aline loads first (most common case)  NB: there are no more lines, only alines
	if (st_prep.move_type == MOVE_TYPE_ALINE) {
		st_run.dda_ticks_downcount = st_prep.dda_ticks;
		st_run.dda_ticks_X_substeps = st_prep.dda_ticks_X_substeps;
		TIMER_DDA.PER = st_prep.dda_period;
//+++++	st_set_motor_disable_timeout(cfg.motor_disable_timeout);

		// This section is somewhat optimized for execution speed 
		// All axes must set steps and compensate for out-of-range pulse phasing. 
		// If axis has 0 steps the direction setting can be omitted
		// If axis has 0 steps enabling motors is req'd to support power mode = 1

		st_run.m[MOTOR_1].phase_increment = st_prep.m[MOTOR_1].phase_increment;			// set steps
		if (st_prep.reset_flag == true) {				// compensate for pulse phasing
			st_run.m[MOTOR_1].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_1].phase_increment != 0) {
			// For ideal optimizations, only set or clear a bit at a time.
			if (st_prep.m[MOTOR_1].dir == 0) {
				PORT_MOTOR_1_VPORT.OUT &= ~DIRECTION_BIT_bm;// CW motion (bit cleared)
			} else {
				PORT_MOTOR_1_VPORT.OUT |= DIRECTION_BIT_bm;	// CCW motion
			}
			PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;	// enable motor
		}

		st_run.m[MOTOR_2].phase_increment = st_prep.m[MOTOR_2].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_2].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_2].phase_increment != 0) {
			if (st_prep.m[MOTOR_2].dir == 0) {
				PORT_MOTOR_2_VPORT.OUT &= ~DIRECTION_BIT_bm;
			} else {
				PORT_MOTOR_2_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
		}

		st_run.m[MOTOR_3].phase_increment = st_prep.m[MOTOR_3].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_3].phase_accumulator = -(st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_3].phase_increment != 0) {
			if (st_prep.m[MOTOR_3].dir == 0) {
				PORT_MOTOR_3_VPORT.OUT &= ~DIRECTION_BIT_bm;
			} else {
				PORT_MOTOR_3_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
		}

		st_run.m[MOTOR_4].phase_increment = st_prep.m[MOTOR_4].phase_increment;
		if (st_prep.reset_flag == true) {
			st_run.m[MOTOR_4].phase_accumulator = (st_run.dda_ticks_downcount);
		}
		if (st_run.m[MOTOR_4].phase_increment != 0) {
			if (st_prep.m[MOTOR_4].dir == 0) {
				PORT_MOTOR_4_VPORT.OUT &= ~DIRECTION_BIT_bm;
			} else {
				PORT_MOTOR_4_VPORT.OUT |= DIRECTION_BIT_bm;
			}
			PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
		}

		TIMER_DDA.CTRLA = STEP_TIMER_ENABLE;				// enable the DDA timer

	// handle dwells
	} else if (st_prep.move_type == MOVE_TYPE_DWELL) {
		st_run.dda_ticks_downcount = st_prep.dda_ticks;
		TIMER_DWELL.PER = st_prep.dda_period;				// load dwell timer period
 		TIMER_DWELL.CTRLA = STEP_TIMER_ENABLE;				// enable the dwell timer
	}

	// all other cases drop to here (e.g. Null moves after Mcodes skip to here) 
	st_prep.exec_state = PREP_BUFFER_OWNED_BY_EXEC;			// flip it back
	st_request_exec_move();									// exec and prep next move
}

/* 
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 *
 *	Used by M codes, tool and spindle changes
 */

void st_prep_null()
{
	st_prep.move_type = MOVE_TYPE_NULL;
}

/* 
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
	st_prep.move_type = MOVE_TYPE_DWELL;
	st_prep.dda_period = _f_to_period(F_DWELL);
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * F_DWELL);
}

/*
 * st_prep_line() - Prepare the next move for the loader
 *
 *	This function does the math on the next pulse segment and gets it ready for 
 *	the loader. It deals with all the DDA optimizations and timer setups so that
 *	loading can be performed as rapidly as possible. It works in joint space 
 *	(motors) and it works in steps, not length units. All args are provided as 
 *	floats and converted to their appropriate integer types for the loader. 
 *
 * Args:
 *	steps[] are signed relative motion in steps (can be non-integer values)
 *	Microseconds - how many microseconds the segment should run 
 */

stat_t st_prep_line(float steps[], float microseconds)
{
	uint8_t i;
	float f_dda = F_DDA;		// starting point for adjustment
	float dda_substeps = DDA_SUBSTEPS;

	// *** defensive programming ***
	// trap conditions that would prevent queueing the line
	if (st_prep.exec_state != PREP_BUFFER_OWNED_BY_EXEC) { return (STAT_INTERNAL_ERROR);
	} else if (isfinite(microseconds) == false) { return (STAT_MINIMUM_LENGTH_MOVE_ERROR);
	} else if (microseconds < EPSILON) { return (STAT_MINIMUM_TIME_MOVE_ERROR);
	}
	st_prep.reset_flag = false;		// initialize accumulator reset flag for this move.

	// setup motor parameters
	for (i=0; i<MOTORS; i++) {
		st_prep.m[i].dir = ((steps[i] < 0) ? 1 : 0) ^ cfg.m[i].polarity;
		st_prep.m[i].phase_increment = (uint32_t)fabs(steps[i] * dda_substeps);
	}
	st_prep.dda_period = _f_to_period(f_dda);
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * f_dda);
	st_prep.dda_ticks_X_substeps = st_prep.dda_ticks * dda_substeps;	// see FOOTNOTE

	// anti-stall measure in case change in velocity between segments is too great 
	if ((st_prep.dda_ticks * ACCUMULATOR_RESET_FACTOR) < st_prep.prev_ticks) {  // NB: uint32_t math
		st_prep.reset_flag = true;
	}
	st_prep.prev_ticks = st_prep.dda_ticks;
	st_prep.move_type = MOVE_TYPE_ALINE;
	return (STAT_OK);
}
// FOOTNOTE: This expression was previously computed as below but floating 
// point rounding errors caused subtle and nasty accumulated position errors:
//	sp.dda_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);

/*
 * st_isbusy() - return TRUE if motors are running or a dwell is running
 */
inline uint8_t st_isbusy()
{
	if (st_run.dda_ticks_downcount == 0) {
		return (false);
	} 
	return (true);
}

/* 
 * st_set_polarity() - setter needed by the config system
 */

void st_set_polarity(const uint8_t motor, const uint8_t polarity)
{
	st_run.m[motor].polarity = polarity;
}

/* 
 * st_set_microsteps() - set microsteps in hardware
 *
 *	For now the microstep_mode is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */

void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode)
{
	if (microstep_mode == 8) {
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 4) {
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 2) {
		device.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	} else if (microstep_mode == 1) {
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		device.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	}
}


/**** DEBUG routines ****/
/*
 * st_dump_stepper_state()
 */

#ifdef __DEBUG

static const char sts_sing[] PROGMEM = "stSingleton dda_ticks (remaining):%d\n";
static const char sts_timr[] PROGMEM = "  timer %s  enabled:%d  period:%d\n";
static const char sts_motr[] PROGMEM = "  motor[%d] pol:%d  steps:%d  counter:%d\n";

void st_dump_stepper_state()
{
	uint8_t i;

	fprintf_P(stderr, (PGM_P)sts_sing, st_run.dda_ticks_downcount);

	fprintf_P(stderr, (PGM_P)sts_timr, "dda", TIMER_DDA.CTRLA, TIMER_DDA.PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "dwl", TIMER_DWELL.CTRLA, TIMER_DWELL.PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "load", TIMER_LOAD.CTRLA, TIMER_LOAD.PER);
	fprintf_P(stderr, (PGM_P)sts_timr, "exec", TIMER_EXEC.CTRLA, TIMER_EXEC.PER);

	for (i=0; i<MOTORS; i++) {
		fprintf_P(stderr, (PGM_P)sts_motr, i, 
			st_run.m[i].polarity,
			st_run.m[i].phase_increment,
			st_run.m[i].phase_accumulator);
	}
}

// dump_set_f_dda(*f_dda, *dda_substeps, major_axis_steps, microseconds, f_dda_base);

#endif
