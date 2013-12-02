/*
 * encoder.c - encoder interface
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
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
#include "canonical_machine.h"
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
	en_reset_encoders();
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
 * en_reset_encoder() - initialize encoder values and position
 * en_reset_encoders() - initialize encoders
 */

void en_reset_encoder(const uint8_t motor)
{
	en.en[motor].motor = motor;			// establish motor mapping
	en.en[motor].steps_run = 0;
	en.en[motor].steps_total = 0;
	en.en[motor].steps_float = 0;
	en.en[motor].position = cm.gmx.position[motor];
}

void en_reset_encoders()
{
	for (uint8_t i=0; i<MOTORS; i++) {
		en_reset_encoder(i);
	}
}

/* 
 * en_set_target() 	 - provide a new target
 * en_get_position() - process position, error term and stage next position measurement
 *
 *	This pair of routines works in tandem to generate an error term. 
 *	The whole thing is very dependent on timing. The target must be set first.
 *	When the position is updated it compares the position to the target and 
 *	populates the error term. A positive error means the position is beyond
 *	the target. Negative is reversed.
 *
 *	en_get_position() contains a critical region. Postion is updated from steps_total, 
 *	which is cleared in the next statement. Ideally this would be atomic.
 *
 *	en_get_position() is typically called form LO interrupt by exec() or prep().
 *	The only other things that should write to steps_total are:
 *	  _load_move() which runs from HI interrupt but is sequenced with exec and prep
 *	  en_reset_encoder() which is called from st_cycle_start at the beginning of cycles
 *	  encopder_init()
 */

void en_set_target(const uint8_t motor, float target)
{
	en.en[motor].next_target = target;
}

void en_get_position(const uint8_t motor)
{
	en.en[motor].steps_total_display = en.en[motor].steps_total;
	//cli();	// critical region
	en.en[motor].position += en.en[motor].steps_total / st_cfg.mot[motor].steps_per_unit;
	en.en[motor].steps_total = 0;
	//sei();
	en.en[motor].error = en.en[motor].position - en.en[motor].target;
	en.en[motor].target = en.en[motor].next_target;	// transfer staged target to the actual target variable
}

/*
 * en_add_incoming_steps() - add new incoming steps
 *
 * This is a handy diagnostic. It's not used for anything else.
 */

void en_add_incoming_steps(const uint8_t motor, float steps)
{
	en.en[motor].steps_float += steps;
}

/*
 * en_print_encoder()
 *	(double)((double)fabs(en.en[i].steps_total) - fabs(en.en[i].steps_float)),
 */

void en_print_encoders()
{
	for (uint8_t i=0; i<MOTORS; i++) {
		printf("{\"en%d\":{\"steps_flt\":%0.3f,\"steps_tot\":%0.0f,\"tgt\":%0.5f,\"pos\":%0.5f,\"st_err\":%0.5f}}\n",
			i+1,
			(double)en.en[i].steps_float,
			(double)en.en[i].steps_total_display, 
			(double)en.en[i].target,
			(double)en.en[i].position,
			(double)en.en[i].error);
	}
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

