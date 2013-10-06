/*
 * plan_arc.c - arc planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "util.h"

#ifdef __cplusplus
extern "C"{
#endif

// Allocate arc planner singleton structure

arc_t arc;

/*
 * Local functions
 */
static stat_t _compute_center_arc(void);
static stat_t _get_arc_radius(void);
static float _get_arc_time (const float linear_travel, const float angular_travel, const float radius);
static float _get_theta(const float x, const float y);

static stat_t _setup_arc(const GCodeState_t *gm_arc, 	// gcode model state
			  const float i, const float j, const float k,
			  const float theta, const float radius, const float angular_travel, // radians along arc (+CW, -CCW)
			  const float linear_travel, 
			  const uint8_t axis_1, const uint8_t axis_2, const uint8_t axis_linear);

/*****************************************************************************
 * Canonical Machining arc functions (arc prep for planning and runtime)
 *
 * cm_arc_init()	 - initialize arcs
 * cm_arc_feed() 	 - canonical machine entry point for arc
 * cm_arc_callback() - mail-loop callback for arc generation
 */

/*
 * cm_arc_init() - initialize arc structures
 */
/*
void cm_arc_init()
{
	arc.magic_start = MAGICNUM;
	arc.magic_end = MAGICNUM;
}
*/

/*
 * cm_arc_feed() - canonical machine entry point for arc
 *
 * Generates an arc by queueing line segments to the move buffer.
 * The arc is approximated by generating a large number of tiny, linear
 * segments.
 */

stat_t cm_arc_feed(float target[], float flags[],	// arc endpoints
				   float i, float j, float k, 		// offsets
				   float radius, 					// non-zero sets radius mode
				   uint8_t motion_mode)				// defined motion mode
{
	stat_t status = STAT_OK;

	// copy parameters into the current state
	gm.motion_mode = motion_mode;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == false) && (fp_ZERO(gm.feed_rate))) {	
		return (STAT_GCODE_FEEDRATE_ERROR);
	}

	// Trap conditions where no arc movement will occur, 
	// but the system is still in arc motion mode - this is not an error.
	// This can happen when a F word or M word is by itself.
	// (The tests below are organized for execution efficiency)
	if ( fp_ZERO(i) && fp_ZERO(j) && fp_ZERO(k) && fp_ZERO(radius) ) {
		if ( fp_ZERO((flags[AXIS_X] + flags[AXIS_Y] + flags[AXIS_Z] + flags[AXIS_A] + flags[AXIS_B] + flags[AXIS_C]))) {
			return (STAT_OK);
		}
	}
	// set parameters
	cm_set_model_target(target,flags);
	cm_set_model_arc_offset(i,j,k);
	cm_set_model_arc_radius(radius);

	// A non-zero radius is a radius arc. Compute the IJK offset coordinates.
	// These will override any IJK offsets provided in the call
	if (fp_NOT_ZERO(radius)) { ritorno(_get_arc_radius());}	// returns if error

	// Introduce a short dwell if the machine is idle to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
// DOESN'T WORK - TRY SOMETHING ELSE
//	if (st_isbusy() == false) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}

	// execute the move
	status = _compute_center_arc();
	cm_conditional_set_model_position(status);	// set endpoint position if the move was successful
	return (status);
}

/*
 * cm_arc_callback() - generate an arc
 *
 *	cm_arc_callback() is structured as a continuation called by mp_move_dispatcher.
 *	Each time it's called it queues as many arc segments (lines) as it can 
 *	before it blocks, then returns.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */

stat_t cm_arc_callback() 
{
	if (arc.run_state == MOVE_STATE_OFF) { return (STAT_NOOP);}
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { return (STAT_EAGAIN);}
	if (arc.run_state == MOVE_STATE_RUN) {
		if (--arc.segment_count > 0) {
			arc.theta += arc.segment_theta;
			arc.gm.target[arc.axis_1] = arc.center_1 + sin(arc.theta) * arc.radius;
			arc.gm.target[arc.axis_2] = arc.center_2 + cos(arc.theta) * arc.radius;
			arc.gm.target[arc.axis_linear] += arc.segment_linear_travel;
			mp_aline(&arc.gm);								// run the line
			copy_axis_vector(arc.position, arc.gm.target);	// update arc current position	
			return (STAT_EAGAIN);
		} else {
			mp_aline(&arc.gm);		// do last segment to the exact endpoint
			arc.run_state = MOVE_STATE_OFF;
		}
	}
	return (STAT_OK);
}

/*
 * cm_abort_arc() - stop arc movement without maintaining position
 *
 *	OK to call if no arc is running
 */

void cm_abort_arc() 
{
	arc.run_state = MOVE_STATE_OFF;
}

/************************************************************************************
 * Arc helper functions
 *
 * _setup_arc()			 - set up arc singleton for an arc move
 * _compute_center_arc() - compute arc from I and J (arc center point)
 * _get_arc_radius() 	 - compute arc center (offset) from radius.
 * _get_arc_time()		 - compute time to complete arc at current feed rate
 */
/*
 * _setup_arc() - setup an arc move for runtime
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */
static stat_t _setup_arc(const GCodeState_t *gm_arc, 	// gcode model state
			  const float i,
			  const float j,
			  const float k,
			  const float theta, 			// starting angle
			  const float radius, 			// radius of the circle in mm
			  const float angular_travel,	// radians along arc (+CW, -CCW)
			  const float linear_travel, 
			  const uint8_t axis_1, 		// circle plane in tool space
			  const uint8_t axis_2,  		// circle plane in tool space
			  const uint8_t axis_linear)	// linear travel if helical motion
{
	if (arc.run_state != MOVE_STATE_OFF) { return (STAT_INTERNAL_ERROR); } // (not supposed to fail)

	arc.gm.linenum = cm_get_linenum(MODEL);

	// length is the total mm of travel of the helix (or just a planar arc)
	arc.length = hypot(angular_travel * radius, fabs(linear_travel));	
	if (arc.length < cm.arc_segment_len) return (STAT_MINIMUM_LENGTH_MOVE_ERROR); // too short to draw

	// load the arc controller singleton
	memcpy(&arc.gm, gm_arc, sizeof(GCodeState_t));	// get the entire GCode context - some will be overwritten to run segments
	copy_axis_vector(arc.position, gmx.position);	// set initial arc position from gcode model

	arc.endpoint[axis_1] = gm_arc->target[0];		// save the arc endpoint
	arc.endpoint[axis_2] = gm_arc->target[1];
	arc.endpoint[axis_linear] = gm_arc->target[2];
	arc.arc_time = gm_arc->move_time;
	arc.theta = theta;
	arc.radius = radius;
	arc.axis_1 = axis_1;
	arc.axis_2 = axis_2;
	arc.axis_linear = axis_linear;
	arc.angular_travel = angular_travel;
	arc.linear_travel = linear_travel;
	
	// Find the minimum number of segments that meets these constraints...
	float segments_required_for_chordal_accuracy = arc.length / sqrt(4*cm.chordal_tolerance * (2 * radius - cm.chordal_tolerance));
	float segments_required_for_minimum_distance = arc.length / cm.arc_segment_len;
	float segments_required_for_minimum_time = arc.arc_time * MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC;
	arc.segments = floor(min3(segments_required_for_chordal_accuracy,
							 segments_required_for_minimum_distance,
							 segments_required_for_minimum_time));

	arc.segments = max(arc.segments,1);				//...but is at least 1 segment
	arc.gm.move_time = arc.arc_time / arc.segments;	// gcode state struct gets segment_time, not arc time

	arc.segment_count = (uint32_t)arc.segments;
	arc.segment_theta = arc.angular_travel / arc.segments;
	arc.segment_linear_travel = arc.linear_travel / arc.segments;
	arc.center_1 = arc.position[arc.axis_1] - sin(arc.theta) * arc.radius;
	arc.center_2 = arc.position[arc.axis_2] - cos(arc.theta) * arc.radius;
	arc.gm.target[arc.axis_linear] = arc.position[arc.axis_linear];
	arc.run_state = MOVE_STATE_RUN;
	return (STAT_OK);
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
	float theta_start = _get_theta(-gmx.arc_offset[gmx.plane_axis_0], -gmx.arc_offset[gmx.plane_axis_1]);
	if(isnan(theta_start) == true) { return(STAT_ARC_SPECIFICATION_ERROR);}

	// calculate the theta (angle) of the target point
	float theta_end = _get_theta(
		gm.target[gmx.plane_axis_0] - gmx.arc_offset[gmx.plane_axis_0] - gmx.position[gmx.plane_axis_0], 
 		gm.target[gmx.plane_axis_1] - gmx.arc_offset[gmx.plane_axis_1] - gmx.position[gmx.plane_axis_1]);
	if(isnan(theta_end) == true) { return (STAT_ARC_SPECIFICATION_ERROR); }

	// ensure that the difference is positive so we have clockwise travel
	if (theta_end < theta_start) { theta_end += 2*M_PI; }

	// compute angular travel and invert if gcode wants a counterclockwise arc
	// if angular travel is zero interpret it as a full circle
	float angular_travel = theta_end - theta_start;
	if (fp_ZERO(angular_travel)) {		
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
	float radius_tmp = hypot(gmx.arc_offset[gmx.plane_axis_0], gmx.arc_offset[gmx.plane_axis_1]);
	float linear_travel = gm.target[gmx.plane_axis_2] - gmx.position[gmx.plane_axis_2];
	gm.move_time = _get_arc_time(linear_travel, angular_travel, radius_tmp);

	// Trace the arc
	cm_set_work_offsets(&gm);						// capture the fully resolved offsets to the state
	set_vector(gm.target[gmx.plane_axis_0], gm.target[gmx.plane_axis_1], gm.target[gmx.plane_axis_2],
			   gm.target[AXIS_A], gm.target[AXIS_B], gm.target[AXIS_C]);

	return(_setup_arc(&gm, gmx.arc_offset[gmx.plane_axis_0],
					   gmx.arc_offset[gmx.plane_axis_1],
					   gmx.arc_offset[gmx.plane_axis_2],
					   theta_start, radius_tmp, angular_travel, linear_travel, 
					   gmx.plane_axis_0, gmx.plane_axis_1, gmx.plane_axis_2));
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
	x = gm.target[gmx.plane_axis_0]-gmx.position[gmx.plane_axis_0];
	y = gm.target[gmx.plane_axis_1]-gmx.position[gmx.plane_axis_1];

	gmx.arc_offset[0] = 0;	// reset the offsets
	gmx.arc_offset[1] = 0;
	gmx.arc_offset[2] = 0;

	// == -(h * 2 / d)
	h_x2_div_d = -sqrt(4 * square(gmx.arc_radius) - (square(x) - square(y))) / hypot(x,y);

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
	if (gmx.arc_radius < 0) { h_x2_div_d = -h_x2_div_d; }

	// Complete the operation by calculating the actual center of the arc
	gmx.arc_offset[gmx.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	gmx.arc_offset[gmx.plane_axis_1] = (y+(x*h_x2_div_d))/2;
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

static float _get_arc_time (const float linear_travel, 		// in mm
							 const float angular_travel, 	// in radians
							 const float radius)			// in mm
{
	float tmp;
	float move_time=0;	// picks through the times and retains the slowest
	float planar_travel = fabs(angular_travel * radius);// travel in arc plane

	if (gm.inverse_feed_rate_mode == true) {
		move_time = gmx.inverse_feed_rate;
	} else {
		move_time = sqrt(square(planar_travel) + square(linear_travel)) / gm.feed_rate;
	}
	if ((tmp = planar_travel/cm.a[gmx.plane_axis_0].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = planar_travel/cm.a[gmx.plane_axis_1].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = fabs(linear_travel/cm.a[gmx.plane_axis_2].feedrate_max)) > move_time) {
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

#ifdef __cplusplus
}
#endif
