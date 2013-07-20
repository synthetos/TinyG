/*
 * plan_arc.c - arc planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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

#include "xio/xio.h"			// support trap and debug statements
#include "tinyg.h"
#include "config.h"
#include "controller.h"			// only needed for line number
#include "canonical_machine.h"
#include "util.h"
#include "plan_arc.h"
#include "planner.h"
#include "kinematics.h"

/*
 * Local functions
 */
static stat_t _compute_center_arc(void);
static stat_t _get_arc_radius(void);
static float _get_arc_time (const float linear_travel, const float angular_travel, const float radius);
static float _get_theta(const float x, const float y);

/*****************************************************************************
 * mp_arc() - setup an arc move for runtime
 *
 *	Generates an arc by queueing line segments to the move buffer.
 *	The arc is approximated by generating a large number of tiny, linear
 *	segments. The length of the segments is configured in motion_control.h
 *	as MM_PER_ARC_SEGMENT.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */
stat_t ar_arc( const float target[], 
				const float i, const float j, const float k, 
				const float theta, 		// starting angle
				const float radius, 		// radius of the circle in mm
				const float angular_travel,// radians along arc (+CW, -CCW)
				const float linear_travel, 
				const uint8_t axis_1, 		// circle plane in tool space
				const uint8_t axis_2,  		// circle plane in tool space
				const uint8_t axis_linear,	// linear travel if helical motion
				const float minutes,		// time to complete the move
				const float work_offset[],	// offset from work coordinate system
				const float min_time)		// minimum time for arc for replanning purposes
{
	if (ar.run_state != MOVE_STATE_OFF) {
		return (STAT_INTERNAL_ERROR);			// (not supposed to fail)
	}
	ar.linenum = cm_get_model_linenum();	// get gcode model line number as debugging convenience

	// "move_length" is the total mm of travel of the helix (or just arc)
	ar.length = hypot(angular_travel * radius, fabs(linear_travel));	
	if (ar.length < cfg.arc_segment_len) {	// too short to draw
		return (STAT_MINIMUM_LENGTH_MOVE_ERROR);
	}

	// load the move struct for an arc
	cm_get_model_canonical_position_vector(ar.position);// set initial arc position

//	copy_axis_vector(ar.endpoint, target);
	ar.endpoint[axis_1] = target[0];					// save the arc endpoint
	ar.endpoint[axis_2] = target[1];
	ar.endpoint[axis_linear] = target[2];

	copy_axis_vector(ar.work_offset, work_offset);		// propagate the work offset
	ar.time = minutes;
	ar.min_time = min_time;
	ar.theta = theta;
	ar.radius = radius;
	ar.axis_1 = axis_1;
	ar.axis_2 = axis_2;
	ar.axis_linear = axis_linear;
	ar.angular_travel = angular_travel;
	ar.linear_travel = linear_travel;
	
	// Find the minimum number of segments that meets these constraints...
	float segments_required_for_chordal_accuracy = ar.length / sqrt(4*cfg.chordal_tolerance * (2 * radius - cfg.chordal_tolerance));
	float segments_required_for_minimum_distance = ar.length / cfg.arc_segment_len;
	float segments_required_for_minimum_time = ar.time * MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC;
	ar.segments = floor(min3(segments_required_for_chordal_accuracy,
							 segments_required_for_minimum_distance,
							 segments_required_for_minimum_time));
	ar.segments = max(ar.segments,1);		//...but is at least 1 segment

	ar.segment_count = (uint32_t)ar.segments;
	ar.segment_theta = ar.angular_travel / ar.segments;
	ar.segment_linear_travel = ar.linear_travel / ar.segments;
	ar.segment_time = ar.time / ar.segments;
	ar.center_1 = ar.position[ar.axis_1] - sin(ar.theta) * ar.radius;
	ar.center_2 = ar.position[ar.axis_2] - cos(ar.theta) * ar.radius;
	ar.target[ar.axis_linear] = ar.position[ar.axis_linear];
	ar.run_state = MOVE_STATE_RUN;
	return (STAT_OK);
}

/*
 * ar_arc_callback() - generate an arc
 *
 *	ar_arc_callback() is structured as a continuation called by mp_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */

stat_t ar_arc_callback() 
{
	if (ar.run_state == MOVE_STATE_OFF) { return (STAT_NOOP);}
	if (mp_get_planner_buffers_available() == 0) { return (STAT_EAGAIN);}
	if (ar.run_state == MOVE_STATE_RUN) {
		if (--ar.segment_count > 0) {
			ar.theta += ar.segment_theta;
			ar.target[ar.axis_1] = ar.center_1 + sin(ar.theta) * ar.radius;
			ar.target[ar.axis_2] = ar.center_2 + cos(ar.theta) * ar.radius;
			ar.target[ar.axis_linear] += ar.segment_linear_travel;
			(void)MP_LINE(ar.target, ar.segment_time, ar.work_offset, 0);
			copy_axis_vector(ar.position, ar.target);	// update runtime position	
			return (STAT_EAGAIN);
		} else {
			(void)MP_LINE(ar.endpoint, ar.segment_time, ar.work_offset,0);// do last segment to the exact endpoint
		}
	}
	ar.run_state = MOVE_STATE_OFF;
	return (STAT_OK);
}

/*
 * ar_abort_arc() - stop an arc.
 *
 *	OK to call if no arc is running
 */

void ar_abort_arc() 
{
	ar.run_state = MOVE_STATE_OFF;
}

/*****************************************************************************
 * Canonical Machining arc functions (arc prep for planning and runtime)
 * cm_arc_feed() 		 - entry point for arc prep
 * _compute_center_arc() - compute arc from I and J (arc center point)
 * _get_arc_radius() 	 - compute arc center (offset) from radius.
 * _get_arc_time()		 - compute time to complete arc at current feed rate
 */
stat_t cm_arc_feed(float target[], float flags[],	// arc endpoints
				   float i, float j, float k, 		// offsets
				   float radius, 					// non-zero sets radius mode
				   uint8_t motion_mode)				// defined motion mode
{
	uint8_t status = STAT_OK;

	// copy parameters into the current state
	gm.motion_mode = motion_mode;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == false) && (gm.feed_rate == 0)) {
		return (STAT_GCODE_FEEDRATE_ERROR);
	}

	// Trap conditions where no arc movement will occur, 
	// but the system is still in arc motion mode - this is not an error.
	// This can happen when a F word or M word is by itself.
	// (The tests below are organized for execution efficiency)
	if ((i==0) && (j==0) && (radius==0) && (k==0)) {
		if ((flags[AXIS_X] + flags[AXIS_Y] + flags[AXIS_Z] + 
			 flags[AXIS_A] + flags[AXIS_B] + flags[AXIS_C]) == 0) {
			return (STAT_OK);
		}
	}
	// set parameters
	cm_set_model_target(target,flags);
	cm_set_model_arc_offset(i,j,k);
	cm_set_model_arc_radius(radius);

	// A non-zero radius is a radius arc. Compute the IJK offset coordinates.
	// These will override any IJK offsets provided in the call
	if (radius > EPSILON) {
		if ((_get_arc_radius() != STAT_OK)) {
			return (status);					// error return
		}
	}

	// Introduce a short dwell if the machine is idle to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
//	if (st_isbusy() == false) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}

	// execute the move
	status = _compute_center_arc();
	cm_set_model_endpoint_position(status);
	return (status);
}


/*
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 *
 *	The theta calculation sets up an clockwise or counterclockwise arc from 
 *	the current position to the target position around the center designated 
 *	by the offset vector. All theta-values measured in radians of deviance 
 *	from the positive y-axis. 
 *
 *                      | <- theta == 0
 *                    * * *
 *                  *       *
 *                *           *
 *                *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
 *                *   /
 *                  C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))
 */

static stat_t _compute_center_arc()
{
	// calculate the theta (angle) of the current point (see header notes)
	float theta_start = _get_theta(-gm.arc_offset[gm.plane_axis_0], -gm.arc_offset[gm.plane_axis_1]);
	if(isnan(theta_start) == true) { return(STAT_ARC_SPECIFICATION_ERROR);}

	// calculate the theta (angle) of the target point
	float theta_end = _get_theta(
		gm.target[gm.plane_axis_0] - gm.arc_offset[gm.plane_axis_0] - gm.position[gm.plane_axis_0], 
 		gm.target[gm.plane_axis_1] - gm.arc_offset[gm.plane_axis_1] - gm.position[gm.plane_axis_1]);
	if(isnan(theta_end) == true) { return (STAT_ARC_SPECIFICATION_ERROR); }

	// ensure that the difference is positive so we have clockwise travel
	if (theta_end < theta_start) { theta_end += 2*M_PI; }

	// compute angular travel and invert if gcode wants a counterclockwise arc
	// if angular travel is zero interpret it as a full circle
	float angular_travel = theta_end - theta_start;
	if (angular_travel == 0) {
		if (gm.motion_mode == MOTION_MODE_CCW_ARC) { 
			angular_travel -= 2*M_PI;
		} else {
			angular_travel = 2*M_PI;
		}
	} else {
		if (gm.motion_mode == MOTION_MODE_CCW_ARC) { 
			angular_travel -= 2*M_PI;
		}
	}

	// Find the radius, calculate travel in the depth axis of the helix,
	// and compute the time it should take to perform the move
	float radius_tmp = hypot(gm.arc_offset[gm.plane_axis_0], gm.arc_offset[gm.plane_axis_1]);
	float linear_travel = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];
	float move_time = _get_arc_time(linear_travel, angular_travel, radius_tmp);

	// Trace the arc
	set_vector(gm.target[gm.plane_axis_0], gm.target[gm.plane_axis_1], gm.target[gm.plane_axis_2],
			   gm.target[AXIS_A], gm.target[AXIS_B], gm.target[AXIS_C]);

	return(ar_arc(vector,
				  gm.arc_offset[gm.plane_axis_0],
				  gm.arc_offset[gm.plane_axis_1],
				  gm.arc_offset[gm.plane_axis_2],
				  theta_start, radius_tmp, angular_travel, linear_travel, 
				  gm.plane_axis_0, gm.plane_axis_1, gm.plane_axis_2, 
				  move_time, gm.work_offset, gm.min_time));
}

/* 
 * _get_arc_radius() - compute arc center (offset) from radius. 
 *
 *  We need to calculate the center of the circle that has the designated 
 *	radius and passes through both the current position and the target position
 *		  
 *	This method calculates the following set of equations where:
 *	`  [x,y] is the vector from current to target position, 
 *		d == magnitude of that vector, 
 *		h == hypotenuse of the triangle formed by the radius of the circle, 
 *			 the distance to the center of the travel vector. 
 *		  
 *	A vector perpendicular to the travel vector [-y,x] is scaled to the length
 *	of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2]
 *	to form the new point [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the 
 *	center of our arc.
 *        
 *		d^2 == x^2 + y^2
 *		h^2 == r^2 - (d/2)^2
 *		i == x/2 - y/d*h
 *		j == y/2 + x/d*h
 *                                        O <- [i,j]
 *                                     -  |
 *                           r      -     |
 *                               -        |
 *                            -           | h
 *                         -              |
 *           [0,0] ->  C -----------------+--------------- T  <- [x,y]
 *                     | <------ d/2 ---->|
 *                  
 *		C - Current position
 *		T - Target position
 *		O - center of circle that pass through both C and T
 *		d - distance from C to T
 *		r - designated radius
 *		h - distance from center of CT to O
 *  
 *	Expanding the equations:
 *		d -> sqrt(x^2 + y^2)
 *		h -> sqrt(4 * r^2 - x^2 - y^2)/2
 *		i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
 *		j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
 * 
 *	Which can be written:  
 *		i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
 *		j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
 *  
 *	Which we for size and speed reasons optimize to:
 *		h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
 *		i = (x - (y * h_x2_div_d))/2
 *		j = (y + (x * h_x2_div_d))/2
 *
 * ----Computing clockwise vs counter-clockwise motion ----
 *
 *	The counter clockwise circle lies to the left of the target direction. 
 *	When offset is positive, the left hand circle will be generated - 
 *	when it is negative the right hand circle is generated.
 *
 *                                   T  <-- Target position
 *  
 *                                   ^ 
 *      Clockwise circles with       |     Clockwise circles with
 *		this center will have        |     this center will have
 *      > 180 deg of angular travel  |     < 180 deg of angular travel, 
 *                        \          |      which is a good thing!
 *                         \         |         /
 *  center of arc when  ->  x <----- | -----> x <- center of arc when 
 *  h_x2_div_d is positive           |             h_x2_div_d is negative
 *                                   |
 *                                   C  <-- Current position
 */

static stat_t _get_arc_radius()
{
	float x;
	float y;
	float h_x2_div_d;

	// Calculate the change in position along each selected axis
	x = gm.target[gm.plane_axis_0]-gm.position[gm.plane_axis_0];
	y = gm.target[gm.plane_axis_1]-gm.position[gm.plane_axis_1];

	gm.arc_offset[0] = 0;	// reset the offsets
	gm.arc_offset[1] = 0;
	gm.arc_offset[2] = 0;

	// == -(h * 2 / d)
	h_x2_div_d = -sqrt(4 * square(gm.arc_radius) - (square(x) - square(y))) / hypot(x,y);

	// If r is smaller than d the arc is now traversing the complex plane beyond
	// the reach of any real CNC, and thus - for practical reasons - we will 
	// terminate promptly
	if(isnan(h_x2_div_d) == true) { return (STAT_FLOATING_POINT_ERROR);}

	// Invert the sign of h_x2_div_d if circle is counter clockwise (see header notes)
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d;}

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees
	// of travel" (go figure!), even though it is advised against ever generating
	// such circles in a single line of g-code. By inverting the sign of 
	// h_x2_div_d the center of the circles is placed on the opposite side of 
	// the line of travel and thus we get the unadvisably long arcs as prescribed.
	if (gm.arc_radius < 0) { h_x2_div_d = -h_x2_div_d; }

	// Complete the operation by calculating the actual center of the arc
	gm.arc_offset[gm.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	gm.arc_offset[gm.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	return (STAT_OK);
} 
    
/*
 * _get_arc_time ()
 *
 *	This is a naiive rate-limiting function. The arc drawing time is computed 
 *	not to exceed the time taken in the slowest dimension - in the arc plane
 *	or in linear travel. Maximum feed rates are compared in each dimension,
 *	but the comparison assumes that the arc will have at least one segment
 *	where the unit vector is 1 in that dimension. This is not true for any
 *	arbitrary arc, with the result that the time returned may be less than 
 *	optimal.
 *
 *	Room for improvement: At least take the hypotenuse of the planar movement 
 *	and the linear travel into account, but how many people actually use helixes?
 */

static float _get_arc_time (const float linear_travel, 	// in mm
							 const float angular_travel, 	// in radians
							 const float radius)			// in mm
{
	float tmp;
	float move_time=0;	// picks through the times and retains the slowest
	float planar_travel = fabs(angular_travel * radius);// travel in arc plane

	if (gm.inverse_feed_rate_mode == true) {
		move_time = gm.inverse_feed_rate;
	} else {
		move_time = sqrt(square(planar_travel) + square(linear_travel)) / gm.feed_rate;
	}
	if ((tmp = planar_travel/cfg.a[gm.plane_axis_0].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = planar_travel/cfg.a[gm.plane_axis_1].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = fabs(linear_travel/cfg.a[gm.plane_axis_2].feedrate_max)) > move_time) {
		move_time = tmp;
	}
	return (move_time);
}

/* 
 * _get_theta(float x, float y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static float _get_theta(const float x, const float y)
{
	float theta = atan(x/fabs(y));

	if (y>0) {
		return (theta);
	} else {
		if (theta>0) {
			return ( M_PI-theta);
    	} else {
			return (-M_PI-theta);
		}
	}
}

//##########################################
//############## UNIT TESTS ################
//##########################################

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_PLANNER

void mp_plan_arc_unit_tests()
{
//	_mp_test_buffers();
}

#endif
#endif
