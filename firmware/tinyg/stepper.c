/*
 * stepper.c - stepper motor controls
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

/**** Line planning and execution ****
 *
 *	Move planning, execution and pulse generation takes place at 3 levels:
 *
 *	Move planning occurs in the main-loop. The canonical machine calls the
 *	planner to generate lines, arcs, dwells and synchronous stop/starts.
 *	The planner module generates blocks (bf's) that hold parameters for 
 *	lines and the other move types. The blocks are backplanned to join 
 *	lines, and to take dwells and stops into account. ("plan" stage).
 *
 *	Arc movement is planned above the above the line planner. The arc 
 *	planner generates short lines that are passed to the line planner.
 *
 *	Move execution and load prep takes place at the LOW interrupt level. 
 *	Move execution generates the next acceleration, cruise, or deceleration
 *	segment for planned lines, or just transfers parameters needed for 
 *	dwells and stops. This layer also prepares moves for loading by 
 *	pre-calculating the values needed by the DDA, and converting the 
 *	executed move into parameters that can be directly loaded into the 
 *	steppers ("exec" and "prep" stages).
 *
 *	Pulse train generation takes place at the HI interrupt level. 
 *	The stepper DDA fires timer interrupts that generate the stepper pulses. 
 *	This level also transfers new stepper parameters once each pulse train
 *	("segment") is complete ("load" and "run" stages). 
 */
/* 	What happens when the pulse generator is done with the current pulse train 
 *	(segment) is a multi-stage "pull" queue that looks like this:
 *
 *	As long as the steppers are running the sequence of events is:
 *	  - The stepper interrupt (HI) runs the DDA to generate a pulse train
 *	  	  for the current move. This runs for the length of the pulse train
 *		  currently executing - the "segment", usually 5ms worth of pulses
 *
 *	  - When the current segment is finished the stepper interrupt LOADs the next 
 *		  segment from the prep buffer, reloads the timers, and starts the 
 *		  next segment. At the end of the load the stepper interrupt routine
 *		  requests an "exec" of the next move in order to prepare for the 
 *		  next load operation. It does this by calling the exec using a 
 *		  software interrupt (actually a timer, since that's all we've got).
 *
 *	  - As a result of the above, the EXEC handler fires at the LO interrupt 
 *		  level. It computes the next accel/decel segment for the current move 
 *		  (i.e. the move in the planner's runtime buffer) by calling back to 
 *		  the exec routine in planner.c. Or it gets and runs the next buffer 
 *		  in the planning queue - depending on the move_type and state. 
 *
 *	  - Once the segment has been computed the exec handler finshes up by running 
 *		  the PREP routine in stepper.c. This computes the DDA values and gets 
 *		  the segment into the prep buffer - and ready for the next LOAD operation.
 *
 *	  - The main loop runs in background to receive gcode blocks, parse them,
 *		  and send them to the planner in order to keep the planner queue 
 *		  full so that when the planner's runtime buffer completes the next move
 *		  (a gcode block or perhaps an arc segment) is ready to run.
 *
 *	If the steppers are not running the above is similar, except that the exec
 * 	is invoked from the main loop by the software interrupt, and the stepper 
 *	load is invoked from the exec by another software interrupt.
 */
/*	Control flow can be a bit confusing. This is a typical sequence for planning 
 *	executing, and running an acceleration planned line:
 *
 *	 1  planner.mp_aline() is called, which populates a planning buffer (bf) 
 *		and back-plans any pre-existing buffers.
 *
 *	 2  When a new buffer is added _mp_queue_write_buffer() tries to invoke
 *	    execution of the move by calling stepper.st_request_exec_move(). 
 *
 *	 3a If the steppers are running this request is ignored.
 *	 3b If the steppers are not running this will set a timer to cause an 
 *		EXEC "software interrupt" that will ultimately call st_exec_move().
 *
 *   4  At this point a call to _exec_move() is made, either by the 
 *		software interrupt from 3b, or once the steppers finish running 
 *		the current segment and have loaded the next segment. In either 
 *		case the call is initated via the EXEC software interrupt which 
 *		causes _exec_move() to run at the MEDium interupt level.
 *		 
 *	 5	_exec_move() calls back to planner.mp_exec_move() which generates 
 *		the next segment using the mr singleton.
 *
 *	 6	When this operation is complete mp_exec_move() calls the appropriate
 *		PREP routine in stepper.c to derive the stepper parameters that will 
 *		be needed to run the move - in this example st_prep_line().
 *
 *	 7	st_prep_line() generates the timer and DDA values and stages these into 
 *		the prep structure (sp) - ready for loading into the stepper runtime struct
 *
 *	 8	stepper.st_prep_line() returns back to planner.mp_exec_move(), which 
 *		frees the planning buffer (bf) back to the planner buffer pool if the 
 *		move is complete. This is done by calling _mp_request_finalize_run_buffer()
 *
 *	 9	At this point the MED interrupt is complete, but the planning buffer has 
 *		not actually been returned to the pool yet. The buffer will be returned
 *		by the main-loop prior to testing for an available write buffer in order
 *		to receive the next Gcode block. This handoff prevents possible data 
 *		conflicts between the interrupt and main loop.
 *
 *	10	The final step in the sequence is _load_move() requesting the next 
 *		segment to be executed and prepared by calling st_request_exec() 
 *		- control goes back to step 4.
 *
 *	Note: For this to work you have to be really careful about what structures
 *	are modified at what level, and use volatiles where necessary.
 */
/* Partial steps and phase angle compensation
 *
 *	The DDA accepts partial steps as input. Fractional steps are managed by the 
 *	sub-step value as explained elsewhere. The fraction initially loaded into 
 *	the DDA and the remainder left at the end of a move (the "residual") can
 *	be thought of as a phase angle value for the DDA accumulation. Each 360
 *	degrees of phase angle results in a step being generated. 
 */

#include <avr/interrupt.h>

#include "tinyg.h"
#include "util.h"
#include "system.h"
#include "config.h"
#include "stepper.h" 	
#include "planner.h"
#include "xmega/xmega_rtc.h"

static void _exec_move(void);
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

// Runtime structs. Used exclusively by step generation ISR (HI)

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

// Prep-time structs. Used by exec/prep ISR (MED) and read-only during load 
// Must be careful about volatiles in this one

enum prepBufferState {
	PREP_BUFFER_OWNED_BY_LOADER = 0,// staging buffer is ready for load
	PREP_BUFFER_OWNED_BY_EXEC		// staging buffer is being loaded
};

typedef struct stPrepMotor {
 	uint32_t phase_increment; 		// total steps in axis times substep factor
	int8_t dir;						// b0 = direction
} stPrepMotor_t;

typedef struct stPrepSingleton {
	uint16_t magic_start;			// magic number to test memory integrity	
	uint8_t move_type;				// move type
	uint8_t prep_state;				// set TRUE to load, false to skip
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

uint16_t st_get_stepper_run_magic() { return (st_run.magic_start);}
uint16_t st_get_stepper_prep_magic() { return (st_prep.magic_start);}

/* 
 * st_init() - initialize stepper motor subsystem 
 *
 *	Notes:
 *	  - This init requires sys_init() to be run beforehand
 *		This init is a precursor for gpio_init()
 * 	  - microsteps are setup during config_init()
 *	  - motor polarity is setup during config_init()
 *	  - high level interrupts must be enabled in main() once all inits are complete
 */

void st_init()
{
//	You can assume all values are zeroed. If not, use this:
//	memset(&st, 0, sizeof(st));	// clear all values, pointers and status

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
 * st_enable_motor()  - enable a motor
 * st_enable_motors() - enable all motors with $pm set to POWER_MODE_DELAYED_DISABLE
 * st_disable_motor() - disable a motor
 * st_disable_motors()- disable all motors
 * st_set_motor_disable_timeout()
 * st_motor_disable_callback()
 */
void st_enable_motor(const uint8_t motor)
{
	if (motor == MOTOR_1) { PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_2) { PORT_MOTOR_2_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_3) { PORT_MOTOR_3_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_4) { PORT_MOTOR_4_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm; }
}

void st_enable_motors()
{
	if (cfg.m[MOTOR_1].power_mode == POWER_MODE_DELAYED_DISABLE) { st_enable_motor(MOTOR_1);}
	if (cfg.m[MOTOR_2].power_mode == POWER_MODE_DELAYED_DISABLE) { st_enable_motor(MOTOR_2);}
	if (cfg.m[MOTOR_3].power_mode == POWER_MODE_DELAYED_DISABLE) { st_enable_motor(MOTOR_3);}
	if (cfg.m[MOTOR_4].power_mode == POWER_MODE_DELAYED_DISABLE) { st_enable_motor(MOTOR_4);}
	st_set_motor_disable_timeout(cfg.motor_disable_timeout);
}

void st_disable_motor(const uint8_t motor)
{
	if (motor == MOTOR_1) { PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_2) { PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_3) { PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
	if (motor == MOTOR_4) { PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm; }
}

void st_disable_motors()
{
	st_disable_motor(MOTOR_1);
	st_disable_motor(MOTOR_2);
	st_disable_motor(MOTOR_3);
	st_disable_motor(MOTOR_4);
}

void st_set_motor_disable_timeout(uint32_t seconds)
{
	st_run.motor_disable_systick = SysTickTimer_getValue() + (seconds * 1000);
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
		st_set_motor_disable_timeout(cfg.motor_disable_timeout);
		// power-down motors if this feature is enabled - set to 0 to disable
		if (cfg.m[MOTOR_1].power_mode == POWER_MODE_DISABLE_ON_IDLE) PORT_MOTOR_1_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_2].power_mode == POWER_MODE_DISABLE_ON_IDLE) PORT_MOTOR_2_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_3].power_mode == POWER_MODE_DISABLE_ON_IDLE) PORT_MOTOR_3_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		if (cfg.m[MOTOR_4].power_mode == POWER_MODE_DISABLE_ON_IDLE) PORT_MOTOR_4_VPORT.OUT |= MOTOR_ENABLE_BIT_bm;
		_load_move();							// load the next move
	}
}

ISR(TIMER_DWELL_ISR_vect) {						// DWELL timer interupt
	if (--st_run.dda_ticks_downcount == 0) {
 		TIMER_DWELL.CTRLA = STEP_TIMER_DISABLE;	// disable DWELL timer
		mp_end_dwell();							// free the planner buffer
		_load_move();
	}
}

ISR(TIMER_LOAD_ISR_vect) {						// load steppers SW interrupt
 	TIMER_LOAD.CTRLA = STEP_TIMER_DISABLE;		// disable SW interrupt timer
	_load_move();
}

ISR(TIMER_EXEC_ISR_vect) {						// exec move SW interrupt
 	TIMER_EXEC.CTRLA = STEP_TIMER_DISABLE;		// disable SW interrupt timer
	_exec_move();								// NULL state
}

/* Software interrupts to fire the above
 * st_test_exec_state()	   - return TRUE if exec/prep can run
 * _request_load_move()    - SW interrupt to request to load a move
 *	st_request_exec_move() - SW interrupt to request to execute a move
 * _exec_move() 		   - Run a move from the planner and prepare it for loading
 *
 *	_exec_move() can only be called be called from an ISR at a level lower
 *	than DDA, Only use st_request_exec_move() to call it.
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

static void _exec_move()
{
   	if (st_prep.exec_state == PREP_BUFFER_OWNED_BY_EXEC) {
//		if (mp_exec_move(state) != STAT_NOOP) {
		if (mp_exec_move() != STAT_NOOP) {
			st_prep.exec_state = PREP_BUFFER_OWNED_BY_LOADER; // flip it back
			_request_load_move();
		}
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
		if (st_prep.prep_state == true) {
			st_run.dda_ticks_downcount = st_prep.dda_ticks;
			TIMER_DWELL.PER = st_prep.dda_period;			// load dwell timer period
 			TIMER_DWELL.CTRLA = STEP_TIMER_ENABLE;			// enable the dwell timer
		}
	}

	// all other cases drop to here (e.g. Null moves after Mcodes skip to here) 
	st_prep.exec_state = PREP_BUFFER_OWNED_BY_EXEC;			// flip it back
	st_prep.prep_state = false;
	st_request_exec_move();									// exec and prep next move
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
	st_prep.prep_state = true;
	return (STAT_OK);
}
// FOOTNOTE: This expression was previously computed as below but floating 
// point rounding errors caused subtle and nasty accumulated position errors:
//	sp.dda_ticks_X_substeps = (uint32_t)((microseconds/1000000) * f_dda * dda_substeps);

/* 
 * st_prep_null() - Keeps the loader happy. Otherwise performs no action
 *
 *	Used by M codes, tool and spindle changes
 */

void st_prep_null()
{
	st_prep.move_type = MOVE_TYPE_NULL;
	st_prep.prep_state = true;
}

/* 
 * st_prep_dwell() 	 - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
	st_prep.move_type = MOVE_TYPE_DWELL;
	st_prep.prep_state = true;
	st_prep.dda_period = _f_to_period(F_DWELL);
	st_prep.dda_ticks = (uint32_t)((microseconds/1000000) * F_DWELL);
}

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
