/*
 * stepper.c - stepper motor controls
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
 * Copyright (c) 2013 Robert Giseburt
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
/* 	This module provides the low-level stepper drivers and some related functions.
 *	See stepper.h for a detailed explanation of this module.
 */

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "planner.h"
#include "hardware.h"
#include "text_parser.h"
#include "util.h"

/**** Allocate structures ****/

stConfig_t st_cfg;
static stRunSingleton_t st_run;
static stPrepSingleton_t st_pre;

/**** Setup local functions ****/

static void _load_move(void);
static void _request_load_move(void);

// handy macro
#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/

/* 
 * stepper_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 * 	  - microsteps are setup during config_init()
 *	  - motor polarity is setup during config_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void stepper_init()
{
	memset(&st_run, 0, sizeof(st_run));			// clear all values, pointers and status
	st_run.magic_end = MAGICNUM;
	st_run.magic_start = MAGICNUM;
	st_pre.magic_end = MAGICNUM;
	st_pre.magic_start = MAGICNUM;


#ifdef __STEP_DIAGNOSTICS
	_clear_step_diagnostics();
#endif

	// Configure virtual ports
	PORTCFG.VPCTRLA = PORTCFG_VP0MAP_PORT_MOTOR_1_gc | PORTCFG_VP1MAP_PORT_MOTOR_2_gc;
	PORTCFG.VPCTRLB = PORTCFG_VP2MAP_PORT_MOTOR_3_gc | PORTCFG_VP3MAP_PORT_MOTOR_4_gc;

	// setup ports and data structures
	for (uint8_t i=0; i<MOTORS; i++) {
		hw.st_port[i]->DIR = MOTOR_PORT_DIR_gm;  // sets outputs for motors & GPIO1, and GPIO2 inputs
		hw.st_port[i]->OUT = MOTOR_ENABLE_BIT_bm;// zero port bits AND disable motor
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

	st_pre.exec_state = PREP_BUFFER_OWNED_BY_EXEC;
}

/*
 * st_assertions() - test assertions, return error code if violation exists
 */
stat_t st_assertions()
{
	if (st_run.magic_end	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_run.magic_start	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_pre.magic_end	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (st_pre.magic_start	!= MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * st_cycle_start() - Initializes values for beginning a new cycle.
 * st_cycle_end()
 * st_clc()
 *
 * st_cycle_start() is called from cm_cycle_start().  
 */

void st_cycle_start(void)
{
	st_pre.cycle_start = true;		// triggers stepper resets
	en_reset_encoders();			// see explanation in en_reset_encoders()
}

void st_cycle_end(void)
{
	en_print_encoders();
}

stat_t st_clc(cmdObj_t *cmd)	// clear diagnostic counters, reset stepper prep
{
	st_cycle_end();
	return(STAT_OK);
}

/*
 * stepper_isbusy() - return TRUE if motors are running or a dwell is running
 */
uint8_t stepper_isbusy()
{
	if (st_run.dda_ticks_downcount == 0) {
		return (false);
	} 
	return (true);
}

/*
 * Motor power management functions
 *
 * _energize_motor()			- apply power to a motor
 * _deenergize_motor()			- remove power from a motor
 * st_set_motor_power()			- set motor a specified power level
 * st_energize_motors()			- apply power to all motors
 * st_deenergize_motors()		- remove power from all motors
 * st_motor_power_callback()	- callback to manage motor power sequencing
 */
static void _energize_motor(const uint8_t motor)
{
	switch(motor) {
		case (MOTOR_1): { PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_2): { PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_3): { PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_4): { PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; break; }
	}
//	st_run.mot[motor].power_state = MOTOR_POWERED;
	st_run.mot[motor].power_state = MOTOR_START_IDLE_TIMEOUT;
}

static void _deenergize_motor(const uint8_t motor)
{
	switch (motor) {
		case (MOTOR_1): { PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_2): { PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_3): { PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; break; }
		case (MOTOR_4): { PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; break; }
	}
	st_run.mot[motor].power_state = MOTOR_OFF;
}

static void _set_motor_power_level(const uint8_t motor, const float power_level)
{
	return;	
}

void st_energize_motors()
{
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
		_energize_motor(motor);
		st_run.mot[motor].power_state = MOTOR_START_IDLE_TIMEOUT;
	}
}

void st_deenergize_motors()
{
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
		_deenergize_motor(motor);
	}
}

stat_t st_motor_power_callback() 	// called by controller
{
	// manage power for each motor individually - facilitates advanced features
	for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {

		if (st_cfg.mot[motor].power_mode == MOTOR_ENERGIZED_DURING_CYCLE) {

			switch (st_run.mot[motor].power_state) {
				case (MOTOR_START_IDLE_TIMEOUT): {
					st_run.mot[motor].power_systick = SysTickTimer_getValue() + (uint32_t)(st_cfg.motor_idle_timeout * 1000);
					st_run.mot[motor].power_state = MOTOR_TIME_IDLE_TIMEOUT;
					break;
				}

				case (MOTOR_TIME_IDLE_TIMEOUT): {
					if (SysTickTimer_getValue() > st_run.mot[motor].power_systick ) { 
						st_run.mot[motor].power_state = MOTOR_IDLE;
						_deenergize_motor(motor);
					}
					break;
				}
			}
		} else if(st_cfg.mot[motor].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
			switch (st_run.mot[motor].power_state) {
				case (MOTOR_START_IDLE_TIMEOUT): {
					st_run.mot[motor].power_systick = SysTickTimer_getValue() + (uint32_t)(250);
					st_run.mot[motor].power_state = MOTOR_TIME_IDLE_TIMEOUT;
					break;
				}

				case (MOTOR_TIME_IDLE_TIMEOUT): {
					if (SysTickTimer_getValue() > st_run.mot[motor].power_systick ) { 
						st_run.mot[motor].power_state = MOTOR_IDLE;
						_deenergize_motor(motor);
					}
					break;
				}
			}

//		} else if(st_run.mot[motor].power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {	// ARM ONLY
			
//		} else if(st_run.mot[motor].power_mode == DYNAMIC_MOTOR_POWER) {				// ARM ONLY
			
		}
	}
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
	if ((st_run.mot[MOTOR_1].substep_accumulator += st_run.mot[MOTOR_1].substep_increment) > 0) {
		PORT_MOTOR_1_VPORT.OUT |= STEP_BIT_bm;		// turn step bit on
		st_run.mot[MOTOR_1].substep_accumulator -= st_run.dda_ticks_X_substeps;
		en.en[MOTOR_1].steps_run += en.en[MOTOR_1].step_sign;	
	}
	if ((st_run.mot[MOTOR_2].substep_accumulator += st_run.mot[MOTOR_2].substep_increment) > 0) {
		PORT_MOTOR_2_VPORT.OUT |= STEP_BIT_bm;
		st_run.mot[MOTOR_2].substep_accumulator -= st_run.dda_ticks_X_substeps;
		en.en[MOTOR_2].steps_run += en.en[MOTOR_2].step_sign;	
	}
	if ((st_run.mot[MOTOR_3].substep_accumulator += st_run.mot[MOTOR_3].substep_increment) > 0) {
		PORT_MOTOR_3_VPORT.OUT |= STEP_BIT_bm;
		st_run.mot[MOTOR_3].substep_accumulator -= st_run.dda_ticks_X_substeps;
		en.en[MOTOR_3].steps_run += en.en[MOTOR_3].step_sign;	
	}
	if ((st_run.mot[MOTOR_4].substep_accumulator += st_run.mot[MOTOR_4].substep_increment) > 0) {
		PORT_MOTOR_4_VPORT.OUT |= STEP_BIT_bm;
		st_run.mot[MOTOR_4].substep_accumulator -= st_run.dda_ticks_X_substeps;
		en.en[MOTOR_4].steps_run += en.en[MOTOR_4].step_sign;	
	}

	// pulse stretching for using external drivers.- turn step bits off
	PORT_MOTOR_1_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 5 uSec pulse width
	PORT_MOTOR_2_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 4 uSec
	PORT_MOTOR_3_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 3 uSec
	PORT_MOTOR_4_VPORT.OUT &= ~STEP_BIT_bm;				// ~ 2 uSec

	if (--st_run.dda_ticks_downcount != 0) return;

	TIMER_DDA.CTRLA = STEP_TIMER_DISABLE;				// disable DDA timer
	_load_move();										// load the next move
}

ISR(TIMER_DWELL_ISR_vect) {								// DWELL timer interrupt
	if (--st_run.dda_ticks_downcount == 0) {
 		TIMER_DWELL.CTRLA = STEP_TIMER_DISABLE;			// disable DWELL timer
//		mp_end_dwell();									// free the planner buffer
		_load_move();
	}
}

ISR(TIMER_LOAD_ISR_vect) {								// load steppers SW interrupt
 	TIMER_LOAD.CTRLA = STEP_TIMER_DISABLE;				// disable SW interrupt timer
	_load_move();
}

ISR(TIMER_EXEC_ISR_vect) {								// exec move SW interrupt
 	TIMER_EXEC.CTRLA = STEP_TIMER_DISABLE;				// disable SW interrupt timer

	// exec_move
   	if (st_pre.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
	   	if (mp_exec_move() != STAT_NOOP) {
		   	st_pre.exec_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
		   	_request_load_move();
	   	}
   	}
}

/****************************************************************************************
 * Exec sequencing code - computes and prepares next load segment
 * st_request_exec_move()	- SW interrupt to request to execute a move
 * exec_timer interrupt		- interrupt handler for calling exec function
 */

/* Software interrupts
 *
 * st_request_exec_move() - SW interrupt to request to execute a move
 * _request_load_move()   - SW interrupt to request to load a move
 */

void st_request_exec_move()
{
	if (st_pre.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {	// bother interrupting
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
static void _load_move()
{
	// Be aware that dda_ticks_downcount must equal zero for the loader to run.
	// So the initial load must also have this set to zero as part of initialization
	if (st_run.dda_ticks_downcount != 0) return;					// exit if it's still busy

	if (st_pre.exec_state != PREP_BUFFER_OWNED_BY_LOADER) {		// if there are no moves to load...
		for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
			st_run.mot[motor].power_state = MOTOR_START_IDLE_TIMEOUT;	// ...start motor power timeouts
		}
		return;
	}

	// handle aline loads first (most common case)  NB: there are no more lines, only alines
	if (st_pre.move_type == MOVE_TYPE_ALINE) {

		//**** initializations and state management ****

		if (st_pre.cycle_start == true) {						// setup direction bits and initial accumulator value
			st_pre.cycle_start = false;
			for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++) {
				st_pre.mot[motor].direction_change = true;
				st_run.mot[motor].substep_accumulator = 0; 			// will become max negative during per-motor setup;
			}
		}

		//**** setup the new segment ****

		st_run.dda_ticks_downcount = st_pre.dda_ticks;
		st_run.dda_ticks_X_substeps = st_pre.dda_ticks_X_substeps;
		TIMER_DDA.PER = st_pre.dda_period;

		//**** MOTOR_1 LOAD ****

		// These sections are somewhat optimized for execution speed. The whole load operation
		// is supposed to take < 10 uSec (Xmega). Be careful if you mess with this.

		// if() either sets the substep increment value or zeroes it

		if ((st_run.mot[MOTOR_1].substep_increment = st_pre.mot[MOTOR_1].substep_increment) != 0) {

			// Set the direction bit in hardware
			// Compensate for direction change in the accumulator
		 	// NB: If motor has 0 steps this is all skipped
			if (st_pre.mot[MOTOR_1].direction_change == true) {
				if (st_pre.mot[MOTOR_1].direction == DIRECTION_CW) 		// CW motion (bit cleared)
					PORT_MOTOR_1_VPORT.OUT &= ~DIRECTION_BIT_bm; else 
					PORT_MOTOR_1_VPORT.OUT |= DIRECTION_BIT_bm;			// CCW motion
				st_run.mot[MOTOR_1].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_1].substep_accumulator);
			}
			// Enable the stepper and start motor power management
			PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;				// energize motor
			st_run.mot[MOTOR_1].power_state = MOTOR_RUNNING;			// set power management state
			en.en[MOTOR_1].step_sign = st_pre.mot[MOTOR_1].step_sign;	// transfer in the signed step increment

		} else {  // Motor has 0 steps; might need to energize motor for power mode processing
			if (st_cfg.mot[MOTOR_1].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;			// energize motor
				st_run.mot[MOTOR_1].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}
		// accumulate counted steps to the step position and zero out counted steps for the segment currently being loaded
		en.en[MOTOR_1].position_steps += en.en[MOTOR_1].steps_run;		// NB: steps_run can be + or - value
		en.en[MOTOR_1].steps_run = 0;

		//**** MOTOR_2 LOAD ****

		if ((st_run.mot[MOTOR_2].substep_increment = st_pre.mot[MOTOR_2].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_2].direction_change == true) {
				if (st_pre.mot[MOTOR_2].direction == DIRECTION_CW)
					PORT_MOTOR_2_VPORT.OUT &= ~DIRECTION_BIT_bm; else
					PORT_MOTOR_2_VPORT.OUT |= DIRECTION_BIT_bm; 
				st_run.mot[MOTOR_2].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_2].substep_accumulator);
			}
			PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
			st_run.mot[MOTOR_2].power_state = MOTOR_RUNNING;
			en.en[MOTOR_2].step_sign = st_pre.mot[MOTOR_2].step_sign;
		} else {
			if (st_cfg.mot[MOTOR_2].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_2].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}
		en.en[MOTOR_2].position_steps += en.en[MOTOR_2].steps_run;
		en.en[MOTOR_2].steps_run = 0;

		//**** MOTOR_3 LOAD ****

		if ((st_run.mot[MOTOR_3].substep_increment = st_pre.mot[MOTOR_3].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_3].direction_change == true) {
				if (st_pre.mot[MOTOR_3].direction == DIRECTION_CW)
					PORT_MOTOR_3_VPORT.OUT &= ~DIRECTION_BIT_bm; else 
					PORT_MOTOR_3_VPORT.OUT |= DIRECTION_BIT_bm;
				st_run.mot[MOTOR_3].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_3].substep_accumulator);
			}
			PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
			st_run.mot[MOTOR_3].power_state = MOTOR_RUNNING;
			en.en[MOTOR_3].step_sign = st_pre.mot[MOTOR_3].step_sign;
		} else {
			if (st_cfg.mot[MOTOR_3].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_3].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}
		en.en[MOTOR_3].position_steps += en.en[MOTOR_3].steps_run;
		en.en[MOTOR_3].steps_run = 0;

		//**** MOTOR_4 LOAD ****

		if ((st_run.mot[MOTOR_4].substep_increment = st_pre.mot[MOTOR_4].substep_increment) != 0) {
			if (st_pre.mot[MOTOR_4].direction_change == true) {
				if (st_pre.mot[MOTOR_4].direction == DIRECTION_CW)
					PORT_MOTOR_4_VPORT.OUT &= ~DIRECTION_BIT_bm; else
					PORT_MOTOR_4_VPORT.OUT |= DIRECTION_BIT_bm;
				st_run.mot[MOTOR_4].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_4].substep_accumulator);
			}
			PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
			st_run.mot[MOTOR_4].power_state = MOTOR_RUNNING;
			en.en[MOTOR_4].step_sign = st_pre.mot[MOTOR_4].step_sign;
		} else {
			if (st_cfg.mot[MOTOR_4].power_mode == MOTOR_IDLE_WHEN_STOPPED) {
				PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
				st_run.mot[MOTOR_4].power_state = MOTOR_START_IDLE_TIMEOUT;
			}
		}
		en.en[MOTOR_4].position_steps += en.en[MOTOR_4].steps_run;
		en.en[MOTOR_4].steps_run = 0;

		//**** do this last ****

		TIMER_DDA.CTRLA = STEP_TIMER_ENABLE;				// enable the DDA timer

	// handle dwells
	} else if (st_pre.move_type == MOVE_TYPE_DWELL) {
		st_run.dda_ticks_downcount = st_pre.dda_ticks;
		TIMER_DWELL.PER = st_pre.dda_period;				// load dwell timer period
 		TIMER_DWELL.CTRLA = STEP_TIMER_ENABLE;				// enable the dwell timer
	}

	// all other cases drop to here (e.g. Null moves after Mcodes skip to here) 
	st_pre.exec_state = PREP_BUFFER_OWNED_BY_EXEC;			// flip it back
	st_request_exec_move();									// exec and prep next move
}

/***********************************************************************************
 * st_prep_line() - Prepare the next move for the loader
 *
 *	This function does the math on the next pulse segment and gets it ready for 
 *	the loader. It deals with all the DDA optimizations and timer setups so that
 *	loading can be performed as rapidly as possible. It works in joint space 
 *	(motors) and it works in steps, not length units. All args are provided as 
 *	floats and converted to their appropriate integer types for the loader. 
 *
 * Args:
 *	  - steps[] are signed relative motion in steps for each motor. Steps are floats 
 *		that typically have fractional values (fractional steps). The sign indicates
 *		direction. Motors that are not in the move should be 0 steps on input.
 *
 *	  - microseconds - how many microseconds the segment should run. If timing is not 
 *		100% accurate this will affect the move velocity, but not the distance traveled.
 *
 *	  - target[] is a vector used to record the next target position for use by the encoders. 
 *		When new_target is true, target should contain the endpoint position of the current 
 *		move. 
 *
 *	  - new_target is a flag to signal that the target should be captured. It's passed 
 *		as a pointer so it can be reset from within this function.
 *
 * NOTE:  Many of the expressions are sensitive to casting and execution order to avoid long-term 
 *		  accuracy errors due to floating point round off. One earlier failed attempt was:
 *		    dda_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);
 *
 * NOTE:  The handling of the last_segment flag is a bit squirrely. If last_segment is true 
 *		  in the input args then st_pre.last_segment is set at the END of that prep call.
 *		  On the NEXT prep call st_pre.last_segment is acted on and reset. This synchronizes
 *		  the prep with the LOAD and STEP that actually delivered the last segment. If the 
 *		  last_segment is indeed the last segment of the moves in the planner the final prep 
 *		  will never be called - but this is OK as no more correction is required or possible.
 */

//stat_t st_prep_line(float steps[], float microseconds, uint8_t last_segment)
stat_t st_prep_line(float steps[], float microseconds, int32_t encoder_error[])
{
	// trap conditions that would prevent queueing the line
	if (st_pre.exec_state != PREP_BUFFER_OWNED_BY_EXEC) { return (STAT_INTERNAL_ERROR);
	} else if (isinf(microseconds)) { return (cm_hard_alarm(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE));
	} else if (isnan(microseconds)) { return (cm_hard_alarm(STAT_PREP_LINE_MOVE_TIME_IS_NAN));
	} else if (microseconds < EPSILON) { return (STAT_MINIMUM_TIME_MOVE_ERROR);
	}
	// setup segment parameters
	// - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
	// - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)

	st_pre.dda_period = _f_to_period(FREQUENCY_DDA);
	st_pre.dda_ticks = (int32_t)((microseconds / 1000000) * FREQUENCY_DDA);
	st_pre.dda_ticks_X_substeps = st_pre.dda_ticks * DDA_SUBSTEPS;

	// setup motor parameters

	uint8_t previous_direction;
	for (uint8_t i=0; i<MOTORS; i++) {

		// Skip this motor if there are no new steps. Leave all values intact.
        if (fp_ZERO(steps[i])) { st_pre.mot[i].substep_increment = 0; continue;}

		// Direction - set the direction, compensating for polarity. 
		// Set the step_sign which is used by the stepper IRQ to accumulate step position
		// Detect direction changes. Needed for accumulator adjustment

		previous_direction = st_pre.mot[i].direction;
		if (steps[i] >= 0) {					// positive direction
			st_pre.mot[i].direction = DIRECTION_CW ^ st_cfg.mot[i].polarity;
			st_pre.mot[i].step_sign = 1;
		} else {			
			st_pre.mot[i].direction = DIRECTION_CCW ^ st_cfg.mot[i].polarity;
			st_pre.mot[i].step_sign = -1;
		}
		st_pre.mot[i].direction_change = st_pre.mot[i].direction ^ previous_direction;

		// error term processing - try positive errors first

/*+++
		if (st_pre.last_segment == true) {
			if (en.en[i].position_error_steps > ERROR_CORRECTION_THRESHOLD) {
				if (steps[i] > en.en[i].position_error_steps) {
					steps[i] -= en.en[i].position_error_steps;		// remove some steps
					printf("%li",en.en[i].position_error_steps);
				}
			} else if (-en.en[i].position_error_steps > ERROR_CORRECTION_THRESHOLD) {
				steps[i] -= en.en[i].position_error_steps;			// add some steps
				printf("%li",en.en[i].position_error_steps);
			}
		}
*/
/*
		if (st_pre.last_segment == true) {
			if (en.en[i].position_error_steps > ERROR_CORRECTION_THRESHOLD) {
				if (steps[i] > ERROR_CORRECTION_THRESHOLD) {
					steps[i] -= 1;									// remove a step
					printf("-");
				}
			} else if (-en.en[i].position_error_steps > ERROR_CORRECTION_THRESHOLD) {
				steps[i] += 1;
				printf("+");
			}
		}
*/
		// Compute substeb increment. The accumulator must be *exactly* the incoming 
		// fractional steps times the substep multipler or positional drift will occur. 
		// Rounding is performed to eliminate a negative bias in the int32 conversion 
		// that results in long-term negative drift. (fabs/round order doesn't matter)

		st_pre.mot[i].substep_increment = round(fabs(steps[i] * DDA_SUBSTEPS));
	}
	st_pre.move_type = MOVE_TYPE_ALINE;
	return (STAT_OK);
}

/* 
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 *
 *	Used by M codes, tool and spindle changes
 */
void st_prep_null()
{
	st_pre.move_type = MOVE_TYPE_NULL;
}

/* 
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */
void st_prep_dwell(double microseconds)
{
	st_pre.move_type = MOVE_TYPE_DWELL;
	st_pre.dda_period = _f_to_period(FREQUENCY_DWELL);
	st_pre.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DWELL);
}

/*
 * _set_hw_microsteps() - set microsteps in hardware
 *
 *	For now the microsteps is the same as the microsteps (1,2,4,8)
 *	This may change if microstep morphing is implemented.
 */
static void _set_hw_microsteps(const uint8_t motor, const uint8_t microsteps)
{
#ifdef __ARM
	switch (motor) {
		case (MOTOR_1): { motor_1.setMicrosteps(microsteps); break; }
		case (MOTOR_2): { motor_2.setMicrosteps(microsteps); break; }
		case (MOTOR_3): { motor_3.setMicrosteps(microsteps); break; }
		case (MOTOR_4): { motor_4.setMicrosteps(microsteps); break; }
		case (MOTOR_5): { motor_5.setMicrosteps(microsteps); break; }
		case (MOTOR_6): { motor_6.setMicrosteps(microsteps); break; }
	}
#endif //__ARM
#ifdef __AVR
	if (microsteps == 8) {
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microsteps == 4) {
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_1_bm;
	} else if (microsteps == 2) {
		hw.st_port[motor]->OUTSET = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	} else if (microsteps == 1) {
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_0_bm;
		hw.st_port[motor]->OUTCLR = MICROSTEP_BIT_1_bm;
	}
#endif // __AVR
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * _get_motor() - helper to return motor number as an index or -1 if na
 */

static int8_t _get_motor(const index_t index)
{
	char_t *ptr;
	char_t motors[] = {"123456"};
	char_t tmp[CMD_TOKEN_LEN+1];
	
//	strncpy_P(tmp, cfgArray[index].group, CMD_GROUP_LEN);
	strcpy_P(tmp, cfgArray[index].group);	// assumes group strings in the array are well behaved
	if ((ptr = strchr(motors, tmp[0])) == NULL) {
		return (-1);
	}
	return (ptr - motors);
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static void _set_motor_steps_per_unit(cmdObj_t *cmd) 
{
	uint8_t m = _get_motor(cmd->index);
//	st_cfg.mot[m].steps_per_unit = (360 / (st_cfg.mot[m].step_angle / st_cfg.mot[m].microsteps) / st_cfg.mot[m].travel_rev);
//	st_cfg.mot[m].units_per_step = st_cfg.mot[m].travel_rev / (360 / st_cfg.mot[m].step_angle) / st_cfg.mot[m].microsteps;
//	faster and more numerically accurate formulation of the above:
	st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) / (360 * st_cfg.mot[m].microsteps);
	st_cfg.mot[m].steps_per_unit = 1 / st_cfg.mot[m].units_per_step;
}

stat_t st_set_sa(cmdObj_t *cmd)			// motor step angle
{ 
	set_flt(cmd);
	_set_motor_steps_per_unit(cmd); 
	return(STAT_OK);
}

stat_t st_set_tr(cmdObj_t *cmd)			// motor travel per revolution
{ 
	set_flu(cmd);
	_set_motor_steps_per_unit(cmd); 
	return(STAT_OK);
}

stat_t st_set_mi(cmdObj_t *cmd)			// motor microsteps
{
	if (fp_NE(cmd->value,1) && fp_NE(cmd->value,2) && fp_NE(cmd->value,4) && fp_NE(cmd->value,8)) {
		cmd_conditional_message((const char_t *)"*** WARNING *** Setting non-standard microstep value");
	}
	set_ui8(cmd);							// set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
	_set_hw_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

stat_t st_set_pm(cmdObj_t *cmd)			// motor power mode
{ 
	ritorno (set_01(cmd));
	if (fp_ZERO(cmd->value)) { // people asked this setting take effect immediately, hence:
		_energize_motor(_get_motor(cmd->index));
	} else {
		_deenergize_motor(_get_motor(cmd->index));
	}
	return (STAT_OK);
}

stat_t st_set_mt(cmdObj_t *cmd)
{
	st_cfg.motor_idle_timeout = min(IDLE_TIMEOUT_SECONDS_MAX, max(cmd->value, IDLE_TIMEOUT_SECONDS_MIN));
	return (STAT_OK);
}

/*
 * st_set_md() - disable motor power
 * st_set_me() - enable motor power
 *
 * Calling me or md with NULL will enable or disable all motors
 * Setting a value of 0 will enable or disable all motors
 * Setting a value from 1 to MOTORS will enable or disable that motor only
 */ 
stat_t st_set_md(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
	if (((uint8_t)cmd->value == 0) || (cmd->objtype == TYPE_NULL)) {
		st_deenergize_motors();
	} else {
		_deenergize_motor((uint8_t)cmd->value-1);
	}
	return (STAT_OK);
}

stat_t st_set_me(cmdObj_t *cmd)	// Make sure this function is not part of initialization --> f00
{
	if (((uint8_t)cmd->value == 0) || (cmd->objtype == TYPE_NULL)) {
		st_energize_motors();
	} else {
		_energize_motor((uint8_t)cmd->value-1);
	}
	return (STAT_OK);
}

stat_t st_set_mp(cmdObj_t *cmd)	// motor power level
{
	if (cmd->value < (float)0) cmd->value = 0;
	if (cmd->value > (float)1) cmd->value = 1;
	set_flt(cmd);				// set the value in the motor config struct (st)
	
	uint8_t motor = _get_motor(cmd->index);
	st_run.mot[motor].power_level = cmd->value;
	_set_motor_power_level(motor, cmd->value);
	return(STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
static const char msg_units1[] PROGMEM = " mm";
static const char msg_units2[] PROGMEM = " deg";
static const char *const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char fmt_me[] PROGMEM = "motors energized\n";
static const char fmt_md[] PROGMEM = "motors de-energized\n";
static const char fmt_mt[] PROGMEM = "[mt]  motor idle timeout%14.2f Sec\n";
static const char fmt_0ma[] PROGMEM = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] PROGMEM = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] PROGMEM = "[%s%s] m%s travel per revolution%9.3f%s\n";
static const char fmt_0mi[] PROGMEM = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char fmt_0po[] PROGMEM = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] PROGMEM = "[%s%s] m%s power management%10d [0=remain powered,1=power down when idle]\n";
static const char fmt_0mp[] PROGMEM = "[%s%s] m%s motor power level%13.3f [0.000=minimum, 1.000=maximum]\n";

void st_print_mt(cmdObj_t *cmd) { text_print_flt(cmd, fmt_mt);}
void st_print_me(cmdObj_t *cmd) { text_print_nul(cmd, fmt_me);}
void st_print_md(cmdObj_t *cmd) { text_print_nul(cmd, fmt_md);}

static void _print_motor_ui8(cmdObj_t *cmd, const char *format)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value);
}

static void _print_motor_flt_units(cmdObj_t *cmd, const char *format, uint8_t units)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, cmd->value, GET_TEXT_ITEM(msg_units, units));
}

static void _print_motor_flt(cmdObj_t *cmd, const char *format)
{
	fprintf_P(stderr, format, cmd->group, cmd->token, cmd->group, cmd->value);
}

void st_print_ma(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0ma);}
void st_print_sa(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0sa, DEGREE_INDEX);}
void st_print_tr(cmdObj_t *cmd) { _print_motor_flt_units(cmd, fmt_0tr, cm_get_units_mode(MODEL));}
void st_print_mi(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0mi);}
void st_print_po(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0po);}
void st_print_pm(cmdObj_t *cmd) { _print_motor_ui8(cmd, fmt_0pm);}
void st_print_mp(cmdObj_t *cmd) { _print_motor_flt(cmd, fmt_0mp);}

#endif // __TEXT_MODE

