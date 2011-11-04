/*
 * plan_arc.c - arc planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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
#include <math.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "xio.h"				// support trap and debug statements
#include "tinyg.h"
#include "controller.h"			// only needed for line number
#include "canonical_machine.h"
#include "util.h"
#include "config.h"
#include "planner.h"
#include "plan_arc.h"
#include "kinematics.h"

struct arArcSingleton 		{	// persistent planner and runtime variables
	uint8_t run_state;			// runtime state machine sequence
	double linenum;				// gcode line number (Nxxxxx)

	double endpoint[AXES];		// endpoint position
	double position[AXES];		// accumulating runtime position
	double target[AXES];		// runtime target position

	double length;				// length of line or helix in mm
	double time;				// total running time (derived)
	double theta;				// total angle specified by arc
	double radius;				// computed via offsets
	double angular_travel;		// travel along the arc
	double linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_time;		// constant time per aline segment
	double segment_theta;		// angular motion per segment
	double segment_linear_travel;// linear motion per segment
	double center_1;			// center of circle at axis 1 (typ X)
	double center_2;			// center of circle at axis 2 (typ Y)
};
static struct arArcSingleton ar;

/*
 * mp_arc() - setup an arc move for runtime
 *
 *	Generates an arc by queueing line segments to the move buffer.
 *	The arc is approximated by generating a large number of tiny, linear
 *	segments. The length of the segments is configured in motion_control.h
 *	as MM_PER_ARC_SEGMENT.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */
uint8_t ar_arc( const double target[], 
				const double i, const double j, const double k, 
				const double theta, 		// starting angle
				const double radius, 		// radius of the circle in mm
				const double angular_travel,// radians along arc (+CW, -CCW)
				const double linear_travel, 
				const uint8_t axis_1, 		// circle plane in tool space
				const uint8_t axis_2,  		// circle plane in tool space
				const uint8_t axis_linear,	// linear travel if helical motion
				const double minutes)		// time to complete the move
{
	if (ar.run_state != MOVE_STATE_OFF) {
		INFO(PSTR("Called mp_arc() before current arc is done"));
		return (TG_MOTION_CONTROL_ERROR);			// (not supposed to fail)
	}
	// get the line number as a debugging convenience
	if (tg.linenum > EPSILON) { ar.linenum = tg.linenum; }
	else { ar.linenum = tg.linecount; }

	// "move_length" is the total mm of travel of the helix (or just arc)
	ar.length = hypot(angular_travel * radius, fabs(linear_travel));	
	if (ar.length < cfg.arc_segment_len) {	// too short to draw
		INFO(PSTR("mp_arc() too short to draw"));
		return (TG_ZERO_LENGTH_MOVE);
	}

	// load the move struct for an arc
//	mp_get_plan_position(ar.position);	// set initial arc position
	cm_get_gcode_model_position(ar.position);	// set initial arc position
//	copy_axis_vector(ar.target, target);		// set target for arc
	copy_axis_vector(ar.endpoint, target);		// set endpoint for arc
	ar.time = minutes;
	ar.theta = theta;
	ar.radius = radius;
	ar.axis_1 = axis_1;
	ar.axis_2 = axis_2;
	ar.axis_linear = axis_linear;
	ar.angular_travel = angular_travel;
	ar.linear_travel = linear_travel;
	
	// find the minimum segments by time and by distance as the segments
	// can't be shorter than the min update interval or the min seg length
	ar.segments = ceil(min(
					(ar.time * MICROSECONDS_PER_MINUTE / ESTD_SEGMENT_USEC),
					(ar.length / cfg.arc_segment_len)));

	ar.segment_count = (uint32_t)ar.segments;
	ar.segment_theta = ar.angular_travel / ar.segments;
	ar.segment_linear_travel = ar.linear_travel / ar.segments;
	ar.segment_time = ar.time / ar.segments;
	ar.center_1 = ar.position[ar.axis_1] - sin(ar.theta) * ar.radius;
	ar.center_2 = ar.position[ar.axis_2] - cos(ar.theta) * ar.radius;
	ar.target[ar.axis_linear] = ar.position[ar.axis_linear];
	ar.run_state = MOVE_STATE_RUNNING;
	return (TG_OK);
}

/*
 * ar_run_arc() - generate an arc
 *
 *	ar_run_arc() is structured as a continuation called by mp_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */

uint8_t ar_run_arc() 
{
	if (ar.run_state == MOVE_STATE_OFF) { return (TG_NOOP);}
	if (mp_test_write_buffer() == FALSE) { return (TG_EAGAIN);}
	if (ar.run_state == MOVE_STATE_RUNNING) {
		ar.theta += ar.segment_theta;
		ar.target[ar.axis_1] = ar.center_1 + sin(ar.theta) * ar.radius;
		ar.target[ar.axis_2] = ar.center_2 + cos(ar.theta) * ar.radius;
		ar.target[ar.axis_linear] += ar.segment_linear_travel;
		(void)MP_LINE(ar.target, ar.segment_time);
		copy_axis_vector(ar.position, ar.target);	// update runtime position	
		if (ar.segment_count-- > 0) {
			return (TG_EAGAIN);
		}
	}
	ar.run_state = MOVE_STATE_OFF;
	return (TG_OK);
}

//##########################################
//############## UNIT TESTS ################
//##########################################

#ifdef __UNIT_TESTS


void mp_plan_arc_unit_tests()
{
//	_mp_test_buffers();
}

#endif
