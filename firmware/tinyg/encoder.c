/*
 * encoder.c - encoder interface
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart, Jr.
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

#include "tinyg.h"
#include "config.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "kinematics.h"
#include "hardware.h"

/**** Allocate Structures ****/

enEncoders_t en;

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/

/* 
 * encoder_init() - initialize encoders 
 */

void encoder_init()
{
	memset(&en, 0, sizeof(en));		// clear all values, pointers and status
	en.magic_end = MAGICNUM;
	en.magic_start = MAGICNUM;
}

/*
 * en_assertions() - test assertions, return error code if violation exists
 */

stat_t en_assertions()
{
	if (en.magic_end   != MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	if (en.magic_start != MAGICNUM) return (STAT_STEPPER_ASSERTION_FAILURE);
	return (STAT_OK);
}

/* 
 * en_reset_encoders() - initialize encoder values and position
 *
 *	en_reset_encoder() sets the encoder_position to match the MODEL position. 
 *	This establishes the "step grid" relative to the current machine position. 
 *	Note that encoder_position is in integer steps, so it's not an exact 
 *	representation of machine position except if the machine is at zero. 
 *
 *	Reset is called on cycle start which can have the following cases:
 *
 *	  -	New cycle from G0. Position and target from Gcode model (MODEL). (canonical_machine, cm_straight_traverse()
 *	  -	New cycle from G1. Position and target from Gcode model (MODEL). (canonical_machine, cm_straight_feed()
 *	  -	New cycle from G2/G3. Position &target from Gcode model (MODEL). (plan_arc.c,  cm_arc_feed()
 *
 *	The above is also true of cycle starts called from within homing, probing, jogging or other canned cycles.
 *
 *	  - Cycle (re)start from feedhold. Position and target from runtime exec (RUNTIME);
 *		(canonical_machine.c, cm_request_cycle_start() )		
 *
 *	  - mp_exec_move() can also perform a cycle start, but wouldn't this always be 
 *		started by the calling G0/G1/G2/G3? Test this (planner.c, ln 175)
 */

void en_reset_encoders(void)
{
	float initial_position[MOTORS];
	ik_kinematics(cm.gmx.position, initial_position);	// as steps in floating point

	for (uint8_t i=0; i<MOTORS; i++) {
		en.en[i].encoder_steps = (int32_t)round(initial_position[i]);
	}
}

/* 
 * en_sample_encoder()
 *
 *	The stepper ISR count steps into steps_run(). These values are accumulated to 
 *	encoder_position during LOAD (HI interrupt level). The encoder position is 
 *	therefore always stable. But be advised: the position lags target and position
 *	valaues elsewherein the system becuase the sample is taken when the steps for 
 *	that segment are complete.
 */

int32_t en_sample_encoder(uint8_t motor)
{
	return(en.en[motor].encoder_steps);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

#endif // __TEXT_MODE

