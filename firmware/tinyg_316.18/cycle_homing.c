/*
 * cycle_homing - homing cycle extension to canonical_machine.c
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>		// needed for exception strings

#include "util.h"
#include "tinyg.h"

#include "gcode.h"
#include "config.h"
#include "planner.h"
#include "canonical_machine.h"
#include "limit_switches.h"

/* NOTE: locate function prototypes in canonical_machine.h */

uint8_t _cm_run_homing_cycle(void);

/* 
 * cm_homing_cycle() - homing cycle using limit switches
 * cm_run_homing_cycle() - continuation for the above
 *
 *	The homing_cycle is coded as a continuation state machine. See end 
 *	notes in controller.c for how to code continuations. It sequences 
 *	through the various homing moves and reacts to limit switch closures. 
 *	It's a bit tricky because the routine can be re-entered if a limit switch
 *	is hit (ANY limit switch) or if the previously queued move completes.
 *
 *	Operation sequence (pseudocode):
 *	  - cm_homing_cycle()
 *		- zero the gcode mode coordinates
 *		- save the current gcode model state (into gt)
 *		- setup for incremental travel & other inits
 *
 *	  - cm_run_homing_cycle()   
 *		  (entered from from the controller loop if no lower-level functions
 *		    are still executing) 
 *		- only run the continuation if state is not OFF, & motors are idle
 *			(i.e. sync execution to the move queue and steppers) 
 *		- for each axis to be homed:
 *		  - issue a move to minus the travel max in that dimension
 *		  - when the move completes or a limit switch is hit backoff from 
 *			that edge by a nominal amount
 *		  - set position using travel offset value (pos'n relative to zero)
 *		- when all axes are homed:
 *		  - restore the previous model state
 *		  - perform a seek from the current position to zero
 *
 *	The continuation is coded as an outer "wrapper" routine and an inner 
 *	routine. The wrapper handles trivial noop cases and translates the return
 *	codes from the lower routines so the continuation sends well-behaved 
 *	return codes back to the controller.
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued 
 *	move per entry into the continuation, then you must exit. The status of 
 *	the call must be communicated back to the controller wrapper, so the
 *	call should be wrapped in a return().
 *
 *	Another Note: When coding a cycle (like this one) you must wait until 
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done (setting cfg.cycle_active = FALSE). Otherwise there
 *	is a nasty race condition in the tg_controller() that will accept the 
 *	next command before the position of the final move has been set.
 *
 *	Cheat: The routine doesn't actually check *which* limit switch was 
 *	hit, just that one was hit. I'm not sure what I'd do anyway if the 
 *	wrong switch was hit. The axis will have stopped anyway as the END 
 *	invoked from the limit switch ISR stops all axes (is not axis specific). 
 *	This may need to be fixed at some point.
 */

#define HOMING_BACKOFF_MOVE cm_straight_traverse
#define HOMING_ZERO_MOVE cm_straight_traverse
//#define HOMING_BACKOFF_MOVE cm_straight_feed
//#define HOMING_ZERO_MOVE cm_straight_feed
#define HOMING_ZERO_RATE 500

uint8_t cm_homing_cycle()
{
	double flags[AXES] = { 1,1,1,1,1,1 };

	// initialize this whole operation
	cfg.cycle_active = TRUE;			// tell the world you are a Homer
	cfg.homing_state = HOMING_IN_PROCESS;

	// it is necessary to zero the gcode model
	cm_set_target(cm_set_vector(0,0,0,0,0,0), flags);
	cm_set_origin_offsets(cm_set_vector(0,0,0,0,0,0));

	// copy gm to gt
	cm_save_gcode_model();
	cm_use_length_units(MILLIMETER_MODE);
	cm_set_distance_mode(INCREMENTAL_MODE);
	ls_clear_limit_switches();				// reset the switch flags
	cy.state = CY_STATE_NEW;
	return (TG_OK);
}

uint8_t cm_run_homing_cycle()				// outer runtime routine
{
	if (cy.state == CY_STATE_OFF) { 
		return (TG_NOOP);
	}
	if (mp_isbusy() == TRUE) {   			// sync to the move queue
		return (TG_EAGAIN); 
	}
	if (_cm_run_homing_cycle() == TG_COMPLETE) { 
		return (TG_OK); 
	} 
	return (TG_EAGAIN);
}

uint8_t _cm_run_homing_cycle()				// inner runtime routine
{
	// handle any initial switch closures by backing off the switch
	if (cy.state == CY_STATE_NEW) {
		cy.state = CY_STATE_HOMING_X_START;
		ls_read_limit_switches();
		if (ls_xmin_thrown() == TRUE) {
			ls_clear_limit_switches();
			cm_set_vector(CFG(X).homing_backoff, 0, 0, 0, 0, 0);
			return(HOMING_BACKOFF_MOVE(vector));
		}
		if (ls_ymin_thrown() == TRUE) {
			ls_clear_limit_switches();
			cm_set_vector(0, CFG(Y).homing_backoff, 0, 0, 0, 0);
			return(HOMING_BACKOFF_MOVE(vector));
		}
		if (ls_zmin_thrown() == TRUE) {
			ls_clear_limit_switches();
			cm_set_vector(0, 0, CFG(Z).homing_backoff, 0, 0, 0);
			return(HOMING_BACKOFF_MOVE(vector));
		}
		if (ls_amin_thrown() == TRUE) {
			ls_clear_limit_switches();
			cm_set_vector(0, 0, 0, CFG(A).homing_backoff, 0, 0);
			return(HOMING_BACKOFF_MOVE(vector));
		}
	}

	// if X homing is enabled issue a max_travel X move
	if ((CFG(X).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_X_START)) {
		cy.state = CY_STATE_HOMING_X_WAIT;
		(void)cm_set_feed_rate(CFG(X).homing_rate);
		cm_set_vector(-(CFG(X).travel_hard_limit), 0, 0, 0, 0, 0);
		return(cm_straight_feed(vector));
	}
	// wait for the end of the X move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_X_WAIT) {
		cy.state = CY_STATE_HOMING_Y_START;
		ls_clear_limit_switches();
		gt.position[X] = CFG(X).homing_offset + CFG(X).homing_backoff;
		cm_set_vector(CFG(X).homing_backoff, 0, 0, 0, 0, 0);
		return(HOMING_BACKOFF_MOVE(vector));
	}

	// if Y homing is enabled issue a max_travel Y move
	if ((CFG(Y).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_Y_START)) {
		cy.state = CY_STATE_HOMING_Y_WAIT;
		(void)cm_set_feed_rate(CFG(Y).homing_rate);
		cm_set_vector(0, -(CFG(Y).travel_hard_limit), 0, 0, 0, 0);
		return(cm_straight_feed(vector));
	}
	// wait for the end of the Y move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_Y_WAIT) {
		cy.state = CY_STATE_HOMING_Z_START;
		ls_clear_limit_switches();
		gt.position[Y] = CFG(Y).homing_offset + CFG(Y).homing_backoff;
		cm_set_vector(0, CFG(Y).homing_backoff, 0, 0, 0, 0);
		return(HOMING_BACKOFF_MOVE(vector));
	}

	// if Z homing is enabled issue a max_travel Z move
	if ((CFG(Z).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_Z_START)) {
		cy.state = CY_STATE_HOMING_Z_WAIT;
		(void)cm_set_feed_rate(CFG(Z).homing_rate);
		cm_set_vector(0, 0, -(CFG(Z).travel_hard_limit), 0, 0, 0);
		return(cm_straight_feed(vector));
	}
	// wait for the end of the Z move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_Z_WAIT) {
		cy.state = CY_STATE_HOMING_A_START;
		ls_clear_limit_switches();
		gt.position[Z] = CFG(Z).homing_offset + CFG(Z).homing_backoff;
		cm_set_vector(0, 0, CFG(Z).homing_backoff, 0, 0, 0);
		return(HOMING_BACKOFF_MOVE(vector));
	}

	// if A homing is enabled issue a max_travel A move
	if ((CFG(A).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_A_START)) {
		cy.state = CY_STATE_HOMING_A_WAIT;
		(void)cm_set_feed_rate(CFG(A).homing_rate);
		cm_set_vector(0, 0, 0, -(CFG(A).travel_hard_limit), 0, 0);
		return(cm_straight_feed(vector));
	}
	// wait for the end of the A move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_A_WAIT) {
		cy.state = CY_STATE_HOMING_RTZ_START;
		ls_clear_limit_switches();
		gt.position[A] = CFG(A).homing_offset + CFG(A).homing_backoff;
		cm_set_vector(0, 0, 0, CFG(A).homing_backoff, 0, 0);
		return(HOMING_BACKOFF_MOVE(vector));
	}

	// Return-to-zero move - return to zero and reset the models
	if (cy.state != CY_STATE_HOMING_RTZ_WAIT) {
		cy.state = CY_STATE_HOMING_RTZ_WAIT;
		cm_restore_gcode_model();
		(void)mp_set_position(gt.position);	// MP layer must agree with gt position
		(void)cm_set_distance_mode(ABSOLUTE_MODE);
		(void)cm_set_feed_rate(HOMING_ZERO_RATE);
		cm_set_vector(0,0,0,0,0,0);
		return(HOMING_ZERO_MOVE(vector));
	}
	// wait until return to zero is complete before releasing the cycle
	if (cy.state == CY_STATE_HOMING_RTZ_WAIT) {
		cfg.cycle_active = FALSE;						// not a homer anymore
		cfg.homing_state = HOMING_COMPLETE;				//...and we're done
		cy.state = CY_STATE_OFF;						//...don't come back
		return (TG_COMPLETE);
	}
	return (TG_OK);
}


