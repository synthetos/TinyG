/*
 * plan_exec.c - execution function for acceleration managed lines
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2016 Rob Giseburt
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
#include "controller.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"

// execute routines (NB: These are all called from the LO interrupt)
static stat_t _exec_aline_head(void);
static stat_t _exec_aline_body(void);
static stat_t _exec_aline_tail(void);
static stat_t _exec_aline_segment(void);
static void _init_forward_diffs(float Vi, float Vt);

/*************************************************************************
 * mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *	Dequeues the buffer queue and executes the move continuations.
 *	Manages run buffers and other details
 */

stat_t mp_exec_move()
{
	mpBuf_t *bf;

	if ((bf = mp_get_run_buffer()) == NULL) {			// NULL means nothing's running
		st_prep_null();
		return (STAT_NOOP);
	}
	// Manage motion state transitions
	if (bf->move_type == MOVE_TYPE_ALINE) { 			// cycle auto-start for lines only
    	if ((cm.motion_state != MOTION_RUN) && (cm.motion_state != MOTION_HOLD)) {
        	cm_set_motion_state(MOTION_RUN);
    	}
	}
	if (bf->bf_func == NULL) {
        return(cm_panic_P(STAT_INTERNAL_ERROR, PSTR("mp_exec_move"))); // never supposed to get here
    }
	return (bf->bf_func(bf)); 							// run the move callback in the planner buffer
}

/*************************************************************************/
/**** ALINE EXECUTION ROUTINES *******************************************/
/*************************************************************************
 * ---> Everything here fires from interrupts and must be interrupt safe
 *
 *  _exec_aline()		  - acceleration line main routine
 *	_exec_aline_head()	  - helper for acceleration section
 *	_exec_aline_body()	  - helper for cruise section
 *	_exec_aline_tail()	  - helper for deceleration section
 *	_exec_aline_segment() - helper for running a segment
 *
 *	Returns:
 *	 STAT_OK		move is done
 *	 STAT_EAGAIN	move is not finished - has more segments to run
 *	 STAT_NOOP		cause no operation from the steppers - do not load the move
 *	 STAT_xxxxx		fatal error. Ends the move and frees the bf buffer
 *
 *	This routine is called from the (LO) interrupt level. The interrupt
 *	sequencing relies on the behaviors of the routines being exactly correct.
 *	Each call to _exec_aline() must execute and prep *one and only one*
 *	segment. If the segment is the not the last segment in the bf buffer the
 *	_aline() must return STAT_EAGAIN. If it's the last segment it must return
 *	STAT_OK. If it encounters a fatal error that would terminate the move it
 *	should return a valid error code. Failure to obey this will introduce
 *	subtle and very difficult to diagnose bugs (trust me on this).
 *
 *	Note 1 Returning STAT_OK ends the move and frees the bf buffer.
 *		   Returning STAT_OK at this point does NOT advance position meaning any
 *		   position error will be compensated by the next move.
 *
 *	Note 2 Solves a potential race condition where the current move ends but the
 * 		   new move has not started because the previous move is still being run
 *		   by the steppers. Planning can overwrite the new move.
 */
/* OPERATION:
 *	Aline generates jerk-controlled S-curves as per Ed Red's course notes:
 *	  http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
 *	  http://www.scribd.com/doc/63521608/Ed-Red-Ch5-537-Jerk-Equations
 *
 *	A full trapezoid is divided into 5 periods Periods 1 and 2 are the
 *	first and second halves of the acceleration ramp (the concave and convex
 *	parts of the S curve in the "head"). Periods 3 and 4 are the first
 *	and second parts of the deceleration ramp (the tail). There is also
 *	a period for the constant-velocity plateau of the trapezoid (the body).
 *	There are various degraded trapezoids possible, including 2 section
 *	combinations (head and tail; head and body; body and tail), and single
 *	sections - any one of the three.
 *
 *	The equations that govern the acceleration and deceleration ramps are:
 *
 *	  Period 1	  V = Vi + Jm*(T^2)/2
 *	  Period 2	  V = Vh + As*T - Jm*(T^2)/2
 *	  Period 3	  V = Vi - Jm*(T^2)/2
 *	  Period 4	  V = Vh + As*T + Jm*(T^2)/2
 *
 * 	These routines play some games with the acceleration and move timing
 *	to make sure this actually all works out. move_time is the actual time of the
 *	move, accel_time is the time valaue needed to compute the velocity - which
 *	takes the initial velocity into account (move_time does not need to).
 */
/* --- State transitions - hierarchical state machine ---
 *
 *	bf->move_state transitions:
 *	 from _NEW to _RUN on first call (sub_state set to _OFF)
 *	 from _RUN to _OFF on final call
 * 	 or just remains _OFF
 *
 *	mr.move_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *	Within each section state may be
 *	 _NEW - trigger initialization
 *	 _RUN1 - run the first part
 *	 _RUN2 - run the second part
 */
/*	Note:
 *	For a version of these routines that execute using the original equation-of-motion
 *	math (as opposed to the forward difference math) please refer to build 444.01 or earlier.
 *	The Kahan corrections have also been removed in 445.01 as they were not needed.
 */

/**** NOTICE ** NOTICE ** NOTICE ****
 **
 **    mp_exec_aline() is called in
 **     --INTERRUPT CONTEXT!!--
 **
 **    Things we MUST NOT do (even indirectly):
 **       mp_plan_buffer()
 **       mp_plan_block_list()
 **       printf()
 **
 **** NOTICE ** NOTICE ** NOTICE ****/

stat_t mp_exec_aline(mpBuf_t *bf)
{
	if (bf->move_state == MOVE_OFF) {
        return (STAT_NOOP);
    }

    // Initialize all new blocks, regardless of normal or feedhold operation

	if (mr.move_state == MOVE_OFF) {

        // too short lines have already been removed...
        // so is the following code is no longer needed ++++ ash
        // But let's still alert the condition should it ever occur
        if (fp_ZERO(bf->length)) {						    // ...looks for an actual zero here
            rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, "mp_exec_aline() zero length move");
        }

        // Start a new move by setting up the runtime singleton (mr)
		memcpy(&mr.gm, &(bf->gm), sizeof(GCodeState_t));    // copy in the gcode model state
//		bf->replannable = false;
		bf->move_state = MOVE_RUN;
		mr.move_state = MOVE_NEW;
		mr.section = SECTION_HEAD;
		mr.section_state = SECTION_NEW;
		mr.jerk = bf->jerk;

		mr.head_length = bf->head_length;
		mr.body_length = bf->body_length;
		mr.tail_length = bf->tail_length;

		mr.entry_velocity = bf->entry_velocity;
		mr.cruise_velocity = bf->cruise_velocity;
		mr.exit_velocity = bf->exit_velocity;

		copy_vector(mr.unit, bf->unit);
		copy_vector(mr.target, bf->gm.target);			// save the final target of the move
        copy_vector(mr.axis_flags, bf->axis_flags);

		// generate the waypoints for position correction at section ends
		for (uint8_t axis=0; axis<AXES; axis++) {
			mr.waypoint[SECTION_HEAD][axis] = mr.position[axis] + mr.unit[axis] * mr.head_length;
			mr.waypoint[SECTION_BODY][axis] = mr.position[axis] + mr.unit[axis] * (mr.head_length + mr.body_length);
			mr.waypoint[SECTION_TAIL][axis] = mr.position[axis] + mr.unit[axis] * (mr.head_length + mr.body_length + mr.tail_length);
		}
	}

    // Feedhold Processing - We need to handle the following cases (listed in rough sequence order):
    //  (1) - We have a block midway through normal execution and a new feedhold request
    //   (1a) - The deceleration will fit in the length remaining in the running block (mr)
    //   (1b) - The deceleration will not fit in the running block
    //   (1c) - 1a, except the remaining length would be zero or EPSILON close to zero (unlikely)
    //  (2) - We have a new block and a new feedhold request that arrived at EXACTLY the same time (unlikely, but handled)
    //  (3) - We are in the middle of a block that is currently decelerating
    //  (4) - We have decelerated a block to some velocity > zero (needs continuation in next block)
    //  (5) - We have decelerated a block to zero velocity
    //  (6) - We have finished all the runtime work now we have to wait for the steppers to stop
    //  (7) - The steppers have stopped. No motion should occur
    //  (8) - We are removing the hold state and there is queued motion (handled outside this routine)
    //  (9) - We are removing the hold state and there is no queued motion (also handled outside this routine)

    if (cm.motion_state == MOTION_HOLD) {

        // Case (3) is a no-op and is not trapped. It just continues the deceleration.

        // Case (7) - all motion has ceased
        if (cm.hold_state == FEEDHOLD_HOLD) {
            return (STAT_NOOP);                 // VERY IMPORTANT to exit as a NOOP. No more movement
        }

        // Case (6) - wait for the steppers to stop
        if (cm.hold_state == FEEDHOLD_PENDING) {
            if (mp_runtime_is_idle()) {                                 // wait for the steppers to actually clear out
                cm.hold_state = FEEDHOLD_HOLD;
                mp_zero_segment_velocity();                             // for reporting purposes
                sr_request_status_report(SR_REQUEST_ASAP);
                cs.controller_state = CONTROLLER_READY;                 // remove controller readline() PAUSE
            }
            return (STAT_OK);                                           // hold here. No more movement
        }

        // Case (5) - decelerated to zero
        // Update the run buffer then force a replan of the whole planner queue
        if (cm.hold_state == FEEDHOLD_DECEL_END) {
            mr.move_state = MOVE_OFF;	                                // invalidate mr buffer to reset the new move
            bf->move_state = MOVE_NEW;                                  // tell _exec to re-use the bf buffer
            bf->length = get_axis_vector_length(mr.target, mr.position);// reset length
            bf->entry_vmax = 0;                                         // set bp+0 as hold point
            cm.hold_state = FEEDHOLD_PENDING;
            return (STAT_OK);
        }

        // Cases (1a, 1b), Case (2), Case (4)
        // Build a tail-only move from here. Decelerate as fast as possible in the space we have.
        if ((cm.hold_state == FEEDHOLD_SYNC) ||
        ((cm.hold_state == FEEDHOLD_DECEL_CONTINUE) && (mr.move_state == MOVE_NEW))) {
            if (mr.section == SECTION_TAIL) {   // if already in a tail don't decelerate. You already are
                if (fp_ZERO(mr.exit_velocity)) {
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                } else {
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                }
            } else {
                mr.entry_velocity = mr.segment_velocity;
                if (mr.section == SECTION_HEAD) {
                    mr.entry_velocity += mr.forward_diff_5; // compute velocity for next segment (this new one)
                }
                mr.cruise_velocity = mr.entry_velocity;

                mr.section = SECTION_TAIL;
                mr.section_state = SECTION_NEW;
                mr.jerk = bf->jerk;
                mr.head_length = 0;
                mr.body_length = 0;

                float available_length = get_axis_vector_length(mr.target, mr.position);
                mr.tail_length = mp_get_target_length(mr.cruise_velocity, 0, bf);   // braking length

                if (fp_ZERO(available_length - mr.tail_length)) { // (1c) the deceleration time is almost exactly the remaining of the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.exit_velocity = 0;
                    mr.tail_length = available_length;
                } else if (available_length < mr.tail_length) { // (1b) the deceleration has to span multiple moves
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                    mr.tail_length = available_length;
                    mr.exit_velocity = mr.cruise_velocity - mp_get_target_velocity(0, mr.tail_length, bf);
                } else {                                        // (1a)the deceleration will fit into the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.exit_velocity = 0;
                }
            }
        }
    }
    mr.move_state = MOVE_RUN;

    // NB: from this point on the contents of the bf buffer do not affect execution

	//**** main dispatcher to process segments ***
	stat_t status = STAT_OK;
	if (mr.section == SECTION_HEAD) { status = _exec_aline_head();} else
	if (mr.section == SECTION_BODY) { status = _exec_aline_body();} else
	if (mr.section == SECTION_TAIL) { status = _exec_aline_tail();} else 
	{  return(cm_panic_P(STAT_INTERNAL_ERROR, PSTR("mp_exec_aline"))); }  // never supposed to get here

	// Feedhold Case (5): Look for the end of the deceleration to go into HOLD state
	if ((cm.hold_state == FEEDHOLD_DECEL_TO_ZERO) && (status == STAT_OK)) {
    	cm.hold_state = FEEDHOLD_DECEL_END;
    	bf->move_state = MOVE_NEW;                      // reset bf so it can restart the rest of the move
	}

	// There are 4 things that can happen here depending on return conditions:
	//  status       bf->move_state   Description
	//  -----------	 --------------   ----------------------------------------
	//  STAT_EAGAIN  <don't care>     mr buffer has more segments to run
	//  STAT_OK       MOVE_RUN        mr and bf buffers are done
	//  STAT_OK       MOVE_NEW        mr done; bf must be run again (it's been reused)
	//  There is no fourth thing. Nobody expects the Spanish Inquisition

	if (status == STAT_EAGAIN) {
    	sr_request_status_report(SR_REQUEST_TIMED);		// continue reporting mr buffer
	} else {
    	mr.move_state = MOVE_OFF;						// invalidate mr buffer (reset)
    	mr.section_state = SECTION_OFF;

    	if (bf->move_state == MOVE_RUN) {
        	if (mp_free_run_buffer() && cm.hold_state == FEEDHOLD_OFF) {
            	cm_cycle_end();	// free buffer & end cycle if planner is empty
        	}
    	}
	}
	return (status);
}


/*
 * mp_exit_hold_state() - end a feedhold
 *
 *	Feedhold is executed as cm.hold_state transitions executed inside _exec_aline()
 *  Invoke a feedhold by calling cm_request_hold() or cm_start_hold() directly
 *  Return from feedhold by calling cm_request_end_hold() or cm_end_hold directly.
 *  See canonical_macine.c for a more detailed explanation of feedhold operation.
 */

void mp_exit_hold_state()
{
	cm.hold_state = FEEDHOLD_OFF;
	if (mp_has_runnable_buffer()) {
	    cm_set_motion_state(MOTION_RUN);
        st_request_exec_move();
	    sr_request_status_report(SR_REQUEST_ASAP);
    } else {
		cm_set_motion_state(MOTION_STOP);
	}
}

/* Forward difference math explained:
 *
 *	We are using a quintic (fifth-degree) Bezier polynomial for the velocity curve.
 *	This gives us a "linear pop" velocity curve; with pop being the sixth derivative of position:
 *	velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
 *
 * The Bezier curve takes the form:
 *
 *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
 *
 * Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
 * through B_5(t) are the Bernstein basis as follows:
 *
 *		B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
 *		B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
 *		B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
 *		B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
 *		B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
 *		B_5(t) =             t^5  =    t^5
 *		                              ^       ^       ^       ^       ^       ^
 *		                              |       |       |       |       |       |
 *		                              A       B       C       D       E       F
 *
 *
 *  We use forward-differencing to calculate each position through the curve.
 *	This requires a formula of the form:
 *
 *		V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
 *
 *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
 *  through t of the Bezier form of V(t), we can determine that:
 *
 *		A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
 *		B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
 *		C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
 *		D =  10*P_0 - 20*P_1 + 10*P_2
 *		E = - 5*P_0 +  5*P_1
 *		F =     P_0
 *
 *	Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
 *	We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
 *	which, after simplification, resolves to:
 *
 *		A = - 6*P_i +  6*P_t
 *		B =  15*P_i - 15*P_t
 *		C = -10*P_i + 10*P_t
 *		D = 0
 *		E = 0
 *		F = P_i
 *
 *	Given an interval count of I to get from P_i to P_t, we get the parametric "step" size of h = 1/I.
 *	We need to calculate the initial value of forward differences (F_0 - F_5) such that the inital
 *	velocity V = P_i, then we iterate over the following I times:
 *
 *		V   += F_5
 *		F_5 += F_4
 *		F_4 += F_3
 *		F_3 += F_2
 *		F_2 += F_1
 *
 *	See http://www.drdobbs.com/forward-difference-calculation-of-bezier/184403417 for an example of
 *	how to calculate F_0 - F_5 for a cubic bezier curve. Since this is a quintic bezier curve, we
 *	need to extend the formulas somewhat. I'll not go into the long-winded step-by-step here,
 *	but it gives the resulting formulas:
 *
 *		a = A, b = B, c = C, d = D, e = E, f = F
 *		F_5(t+h)-F_5(t) = (5ah)t^4 + (10ah^2 + 4bh)t^3 + (10ah^3 + 6bh^2 + 3ch)t^2 +
 *			(5ah^4 + 4bh^3 + 3ch^2 + 2dh)t + ah^5 + bh^4 + ch^3 + dh^2 + eh
 *
 *		a = 5ah
 *		b = 10ah^2 + 4bh
 *		c = 10ah^3 + 6bh^2 + 3ch
 *		d = 5ah^4 + 4bh^3 + 3ch^2 + 2dh
 *
 *  (After substitution, simplification, and rearranging):
 *		F_4(t+h)-F_4(t) = (20ah^2)t^3 + (60ah^3 + 12bh^2)t^2 + (70ah^4 + 24bh^3 + 6ch^2)t +
 *			30ah^5 + 14bh^4 + 6ch^3 + 2dh^2
 *
 *		a = (20ah^2)
 *		b = (60ah^3 + 12bh^2)
 *		c = (70ah^4 + 24bh^3 + 6ch^2)
 *
 *  (After substitution, simplification, and rearranging):
 *		F_3(t+h)-F_3(t) = (60ah^3)t^2 + (180ah^4 + 24bh^3)t + 150ah^5 + 36bh^4 + 6ch^3
 *
 *  (You get the picture...)
 *		F_2(t+h)-F_2(t) = (120ah^4)t + 240ah^5 + 24bh^4
 *		F_1(t+h)-F_1(t) = 120ah^5
 *
 *  Normally, we could then assign t = 0, use the A-F values from above, and get out initial F_* values.
 *  However, for the sake of "averaging" the velocity of each segment, we actually want to have the initial
 *  V be be at t = h/2 and iterate I-1 times. So, the resulting F_* values are (steps not shown):
 *
 *		F_5 = (121Ah^5)/16 + 5Bh^4 + (13Ch^3)/4 + 2Dh^2 + Eh
 *		F_4 = (165Ah^5)/2 + 29Bh^4 + 9Ch^3 + 2Dh^2
 *		F_3 = 255Ah^5 + 48Bh^4 + 6Ch^3
 *		F_2 = 300Ah^5 + 24Bh^4
 *		F_1 = 120Ah^5
 *
 *  Note that with our current control points, D and E are actually 0.
 */

static void _init_forward_diffs(float Vi, float Vt)
{
	float A =  -6.0*Vi +  6.0*Vt;
	float B =  15.0*Vi - 15.0*Vt;
	float C = -10.0*Vi + 10.0*Vt;
	// D = 0
	// E = 0
	// F = Vi

	float h   = 1/(mr.segments);
//	float h_3 = h * h * h;
//	float h_4 = h_3 * h;
//	float h_5 = h_4 * h;

	float Ah_5 = A * h * h * h * h * h;
	float Bh_4 = B * h * h * h * h;
	float Ch_3 = C * h * h * h;

	mr.forward_diff_5 = (121.0/16.0)*Ah_5 + 5.0*Bh_4 + (13.0/4.0)*Ch_3;
	mr.forward_diff_4 = (165.0/2.0)*Ah_5 + 29.0*Bh_4 + 9.0*Ch_3;
	mr.forward_diff_3 = 255.0*Ah_5 + 48.0*Bh_4 + 6.0*Ch_3;
	mr.forward_diff_2 = 300.0*Ah_5 + 24.0*Bh_4;
	mr.forward_diff_1 = 120.0*Ah_5;

	// Calculate the initial velocity by calculating V(h/2)
	float half_h = h/2.0;
	float half_Ch_3 = C * half_h * half_h * half_h;
	float half_Bh_4 = B * half_h * half_h * half_h * half_h;
	float half_Ah_5 = A * half_h * half_h * half_h * half_h * half_h;
	mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + Vi;
}

/*********************************************************************************************
 * _exec_aline_head()
 */

static stat_t _exec_aline_head()
{
	if (mr.section_state == SECTION_NEW) {							// initialize the move singleton (mr)
		if (fp_ZERO(mr.head_length)) {
			mr.section = SECTION_BODY;
			return(_exec_aline_body());								// skip ahead to the body generator
		}
		mr.gm.move_time = 2*mr.head_length / (mr.entry_velocity + mr.cruise_velocity);// time for entire accel region
		mr.segments = ceil(uSec(mr.gm.move_time) / NOM_SEGMENT_USEC);// # of segments for the section
		mr.segment_time = mr.gm.move_time / mr.segments;
		_init_forward_diffs(mr.entry_velocity, mr.cruise_velocity);
		mr.segment_count = (uint32_t)mr.segments;
		if (mr.segment_time < MIN_SEGMENT_TIME)
            return(STAT_MINIMUM_TIME_MOVE);                         // exit without advancing position
		mr.section = SECTION_HEAD;
		mr.section_state = SECTION_1st_HALF;						// Note: Set to SECTION_1st_HALF for one segment
	}
	// For forward differencing we should have one segment in SECTION_1st_HALF
	// However, if it returns from that as STAT_OK, then there was only one segment in this section.
	if (mr.section_state == SECTION_1st_HALF) {						// FIRST HALF (concave part of accel curve)
		if (_exec_aline_segment() == STAT_OK) { 					// set up for second half
			mr.section = SECTION_BODY;
			mr.section_state = SECTION_NEW;
		} else {
			mr.section_state = SECTION_2nd_HALF;
		}
		return(STAT_EAGAIN);
	}
	if (mr.section_state == SECTION_2nd_HALF) {						// SECOND HALF (convex part of accel curve)
		mr.segment_velocity += mr.forward_diff_5;

		if (_exec_aline_segment() == STAT_OK) { 					// set up for body
			if ((fp_ZERO(mr.body_length)) && (fp_ZERO(mr.tail_length)))
                return(STAT_OK);                                    // ends the move
			mr.section = SECTION_BODY;
			mr.section_state = SECTION_NEW;
		} else {
			mr.forward_diff_5 += mr.forward_diff_4;
			mr.forward_diff_4 += mr.forward_diff_3;
			mr.forward_diff_3 += mr.forward_diff_2;
			mr.forward_diff_2 += mr.forward_diff_1;
		}
	}
	return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_body()
 *
 *	The body is broken into little segments even though it is a straight line so that
 *	feedholds can happen in the middle of a line with a minimum of latency
 */
static stat_t _exec_aline_body()
{
	if (mr.section_state == SECTION_NEW) {
		if (fp_ZERO(mr.body_length)) {
			mr.section = SECTION_TAIL;
			return(_exec_aline_tail());						// skip ahead to tail periods
		}
		mr.gm.move_time = mr.body_length / mr.cruise_velocity;
		mr.segments = ceil(uSec(mr.gm.move_time) / NOM_SEGMENT_USEC);
		mr.segment_time = mr.gm.move_time / mr.segments;
		mr.segment_velocity = mr.cruise_velocity;
		mr.segment_count = (uint32_t)mr.segments;
		if (mr.segment_time < MIN_SEGMENT_TIME)
            return(STAT_MINIMUM_TIME_MOVE);                 // exit without advancing position
		mr.section = SECTION_BODY;
		mr.section_state = SECTION_2nd_HALF;				// uses PERIOD_2 so last segment detection works
	}
	if (mr.section_state == SECTION_2nd_HALF) {				// straight part (period 3)
		if (_exec_aline_segment() == STAT_OK) {				// OK means this section is done
			if (fp_ZERO(mr.tail_length))
                return(STAT_OK);	                        // ends the move
			mr.section = SECTION_TAIL;
			mr.section_state = SECTION_NEW;
		}
	}
	return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_tail()
 */

static stat_t _exec_aline_tail()
{
	if (mr.section_state == SECTION_NEW) {							// INITIALIZATION
		if (fp_ZERO(mr.tail_length))
            return(STAT_OK);                                        // end the move
		mr.gm.move_time = 2*mr.tail_length / (mr.cruise_velocity + mr.exit_velocity); // len/avg. velocity
		mr.segments = ceil(uSec(mr.gm.move_time) / NOM_SEGMENT_USEC);// # of segments for the section
		mr.segment_time = mr.gm.move_time / mr.segments;			// time to advance for each segment
		_init_forward_diffs(mr.cruise_velocity, mr.exit_velocity);
		mr.segment_count = (uint32_t)mr.segments;
		if (mr.segment_time < MIN_SEGMENT_TIME)
            return(STAT_MINIMUM_TIME_MOVE);                         // exit without advancing position
		mr.section = SECTION_TAIL;
		mr.section_state = SECTION_1st_HALF;
	}
	if (mr.section_state == SECTION_1st_HALF) {						// FIRST HALF - convex part (period 4)
		if (_exec_aline_segment() == STAT_OK) {
			// For forward differencing we should have one segment in SECTION_1st_HALF.
			// However, if it returns from that as STAT_OK, then there was only one segment in this section.
			// Show that we did complete section 2 ... effectively.
			mr.section_state = SECTION_2nd_HALF;
			return STAT_OK;
		} else {
			mr.section_state = SECTION_2nd_HALF;
		}
		return(STAT_EAGAIN);
	}
	if (mr.section_state == SECTION_2nd_HALF) {						// SECOND HALF - concave part (period 5)
		mr.segment_velocity += mr.forward_diff_5;

		if (_exec_aline_segment() == STAT_OK) { 					// set up for body
			return STAT_OK;
		} else {
			mr.forward_diff_5 += mr.forward_diff_4;
			mr.forward_diff_4 += mr.forward_diff_3;
			mr.forward_diff_3 += mr.forward_diff_2;
			mr.forward_diff_2 += mr.forward_diff_1;
		}
	}
	return(STAT_EAGAIN);									// should never get here
}

/*********************************************************************************************
 * _exec_aline_segment() - segment runner helper
 *
 * NOTES ON STEP ERROR CORRECTION:
 *
 *	The commanded_steps are the target_steps delayed by one more segment.
 *	This lines them up in time with the encoder readings so a following error can be generated
 *
 *	The following_error term is positive if the encoder reading is greater than (ahead of)
 *	the commanded steps, and negative (behind) if the encoder reading is less than the
 *	commanded steps. The following error is not affected by the direction of movement -
 *	it's purely a statement of relative position. Examples:
 *
 *    Encoder Commanded   Following Err
 *	  	  100	     90	       +10		encoder is 10 steps ahead of commanded steps
 *	      -90	   -100	       +10		encoder is 10 steps ahead of commanded steps
 *		   90	    100	       -10		encoder is 10 steps behind commanded steps
 *	     -100	    -90	       -10		encoder is 10 steps behind commanded steps
 */

static stat_t _exec_aline_segment()
{
	uint8_t i;
	float travel_steps[MOTORS];

	// Set target position for the segment
	// If the segment ends on a section waypoint synchronize to the head, body or tail end
	// Otherwise if not at a section waypoint compute target from segment time and velocity
	// Don't do waypoint correction if you are going into a hold.

	if ((--mr.segment_count == 0) && (mr.section_state == SECTION_2nd_HALF) &&
		(cm.motion_state == MOTION_RUN) && (cm.cycle_state == CYCLE_MACHINING)) {
		copy_vector(mr.gm.target, mr.waypoint[mr.section]);
	} else {
		float segment_length = mr.segment_velocity * mr.segment_time;
		for (i=0; i<AXES; i++) {
			mr.gm.target[i] = mr.position[i] + (mr.unit[i] * segment_length);
		}
	}

	// Convert target position to steps
	// Bucket-brigade the old target down the chain before getting the new target from kinematics
	//
	// NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
	//	   Other kinematics may require transforming travel distance as opposed to simply subtracting steps.

	for (i=0; i<MOTORS; i++) {
		mr.commanded_steps[i] = mr.position_steps[i];		// previous segment's position, delayed by 1 segment
		mr.position_steps[i] = mr.target_steps[i];			// previous segment's target becomes position
		mr.encoder_steps[i] = en_read_encoder(i);			// get current encoder position (time aligns to commanded_steps)
		mr.following_error[i] = mr.encoder_steps[i] - mr.commanded_steps[i];
	}
	kn_inverse_kinematics(mr.gm.target, mr.target_steps);   // now determine the target steps...
	for (i=0; i<MOTORS; i++) {								// and compute the distances to be traveled
		travel_steps[i] = mr.target_steps[i] - mr.position_steps[i];
	}

	// Call the stepper prep function

	ritorno(st_prep_line(travel_steps, mr.following_error, mr.segment_time));
	copy_vector(mr.position, mr.gm.target); 				// update position from target
	if (mr.segment_count == 0) return (STAT_OK);			// this section has run all its segments
	return (STAT_EAGAIN);									// this section still has more segments to run
}
