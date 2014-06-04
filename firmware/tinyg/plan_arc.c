/*
 * plan_arc.c - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
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
/* This module actually contains some parts that belong ion the canonical machine, 
 * and other parts that belong at the motion planner level, but the whole thing is 
 * treated as if it were part of the motion planner.
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


// Local functions

static stat_t _compute_arc(void);
static stat_t _compute_arc_offsets_from_radius(void);
static float _get_arc_time (const float linear_travel, const float angular_travel, const float radius);
static float _get_theta(const float x, const float y);
//static stat_t _test_arc_soft_limits(void);

/*****************************************************************************
 * Canonical Machining arc functions (arc prep for planning and runtime)
 *
 * cm_arc_init()	 - initialize arcs
 * cm_arc_feed() 	 - canonical machine entry point for arc
 * cm_arc_callback() - mail-loop callback for arc generation
 * cm_abort_arc()	 - stop an arc in process
 */

/*
 * cm_arc_init() - initialize arc structures
 */
void cm_arc_init()
{
	arc.magic_start = MAGICNUM;
	arc.magic_end = MAGICNUM;
}

/*
 * cm_arc_feed() - canonical machine entry point for arc
 *
 * Generates an arc by queueing line segments to the move buffer. The arc is 
 * approximated by generating a large number of tiny, linear segments.
 */
stat_t cm_arc_feed(float target[], float flags[],// arc endpoints
				   float i, float j, float k, 	 // raw arc offsets
				   float radius, 				 // non-zero radius implies radius mode
				   uint8_t motion_mode)			 // defined motion mode
{
	// trap zero feed rate condition
	if ((cm.gm.feed_rate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feed_rate))) {
//		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
		return (STAT_GCODE_FEEDRATE_ERROR);		//++++
	}

	// Trap conditions where no arc movement will occur, but the system is still in 
	// arc motion mode - this is not an error. This can happen when a F word or M 
	// word is by itself.(The tests below are organized for execution efficiency)
	if ( fp_ZERO(i) && fp_ZERO(j) && fp_ZERO(k) && fp_ZERO(radius) ) {
		if ( fp_ZERO((flags[AXIS_X] + flags[AXIS_Y] + flags[AXIS_Z] + 
					  flags[AXIS_A] + flags[AXIS_B] + flags[AXIS_C]))) {
			return (STAT_OK);
		}
	}

	// set values in the Gcode model state & copy it (linenum was already captured)
	cm_set_model_target(target, flags);
	cm.gm.motion_mode = motion_mode;
	cm_set_work_offsets(&cm.gm);					// capture the fully resolved offsets to gm
	memcpy(&arc.gm, &cm.gm, sizeof(GCodeState_t));	// copy GCode context to arc singleton - some will be overwritten to run segments

	// populate the arc control singleton
	copy_vector(arc.position, cm.gmx.position);		// set initial arc position from gcode model
	arc.radius = _to_millimeters(radius);			// set arc radius or zero
	arc.offset[0] = _to_millimeters(i);				// copy offsets with conversion to canonical form (mm)
	arc.offset[1] = _to_millimeters(j);
	arc.offset[2] = _to_millimeters(k);

	// Set the arc plane for the current G17/G18/G19 setting 
	// Plane axis 0 and 1 are the arc plane, 2 is the linear axis normal to the arc plane 
	if (cm.gm.select_plane == CANON_PLANE_XY) {	// G17 - the vast majority of arcs are in the G17 (XY) plane
		arc.plane_axis_0 = AXIS_X;		
		arc.plane_axis_1 = AXIS_Y;
		arc.linear_axis  = AXIS_Z;
	} else if (cm.gm.select_plane == CANON_PLANE_XZ) {	// G18
		arc.plane_axis_0 = AXIS_X;		
		arc.plane_axis_1 = AXIS_Z;
		arc.linear_axis  = AXIS_Y;
	} else if (cm.gm.select_plane == CANON_PLANE_YZ) {	// G19
		arc.plane_axis_0 = AXIS_Y;
		arc.plane_axis_1 = AXIS_Z;
		arc.linear_axis  = AXIS_X;
	}

	// compute arc runtime values and prep for execution by the callback
	ritorno(_compute_arc());
//	ritorno(_test_arc_soft_limits());			// test if arc will trip soft limits
	cm_cycle_start();							// if not already started
	arc.run_state = MOVE_RUN;					// enable arc to be run from the callback
	cm_set_model_position(STAT_OK);				// set endpoint position if the arc was successful
	return (STAT_OK);
}

/*
 * cm_arc_callback() - generate an arc
 *
 *	cm_arc_callback() is called from the controller main loop. Each time it's called it 
 *	queues as many arc segments (lines) as it can before it blocks, then returns.
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */

stat_t cm_arc_callback() 
{
	if (arc.run_state == MOVE_OFF) { return (STAT_NOOP);}
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { return (STAT_EAGAIN);}

	arc.theta += arc.segment_theta;
	arc.gm.target[arc.plane_axis_0] = arc.center_0 + sin(arc.theta) * arc.radius;
	arc.gm.target[arc.plane_axis_1] = arc.center_1 + cos(arc.theta) * arc.radius;
	arc.gm.target[arc.linear_axis] += arc.segment_linear_travel;
	mp_aline(&arc.gm);								// run the line
	copy_vector(arc.position, arc.gm.target);		// update arc current position	

	if (--arc.segment_count > 0) return (STAT_EAGAIN);
	arc.run_state = MOVE_OFF;
	return (STAT_OK);
}

/*
 * cm_abort_arc() - stop arc movement without maintaining position
 *
 *	OK to call if no arc is running
 */

void cm_abort_arc() 
{
	arc.run_state = MOVE_OFF;
}

/*
 * _compute_arc() - compute arc from I and J (arc center point)
 *
 *	The theta calculation sets up an clockwise or counterclockwise arc from the current 
 *	position to the target position around the center designated by the offset vector. 
 *	All theta-values measured in radians of deviance from the positive y-axis. 
 *
 *                      | <- theta == 0
 *                    * * *
 *                  *       *
 *                *           *
 *                *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
 *                *   /
 *                  C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))
 *
 *  Parts of this routine were originally sourced from the grbl project.
 */
static stat_t _compute_arc()
{
	// A non-zero radius value indicates a radius arc
	// Compute IJK offset coordinates. These override any current IJK offsets
	if (fp_NOT_ZERO(arc.radius)) ritorno(_compute_arc_offsets_from_radius()); // returns if error

	// Calculate the theta (angle) of the current point (see header notes)
	// Arc.theta is starting point for theta (theta_start)
	arc.theta = _get_theta(-arc.offset[arc.plane_axis_0], -arc.offset[arc.plane_axis_1]);
	if(isnan(arc.theta) == true) return(STAT_ARC_SPECIFICATION_ERROR);

	// calculate the theta (angle) of the target point
	float theta_end = _get_theta(
		arc.gm.target[arc.plane_axis_0] - arc.offset[arc.plane_axis_0] - arc.position[arc.plane_axis_0], 
 		arc.gm.target[arc.plane_axis_1] - arc.offset[arc.plane_axis_1] - arc.position[arc.plane_axis_1]);
	if(isnan(theta_end) == true) return (STAT_ARC_SPECIFICATION_ERROR);

	// ensure that the difference is positive so we have clockwise travel
	if (theta_end < arc.theta) { theta_end += 2*M_PI; }

	// compute angular travel and invert if gcode wants a counterclockwise arc
	// if angular travel is zero interpret it as a full circle
	arc.angular_travel = theta_end - arc.theta;
	if (fp_ZERO(arc.angular_travel)) {
		if (cm.gm.motion_mode == MOTION_MODE_CCW_ARC) {
			arc.angular_travel -= 2*M_PI;
		} else {
			arc.angular_travel = 2*M_PI;
		}
	} else {
		if (cm.gm.motion_mode == MOTION_MODE_CCW_ARC) {
			arc.angular_travel -= 2*M_PI;
		}
	}

	// Find the radius, calculate travel in the depth axis of the helix,
	// and compute the time it should take to perform the move
	arc.radius = hypot(arc.offset[arc.plane_axis_0], arc.offset[arc.plane_axis_1]);
	arc.linear_travel = arc.gm.target[arc.linear_axis] - arc.position[arc.linear_axis];

	// length is the total mm of travel of the helix (or just a planar arc)
	arc.length = hypot(arc.angular_travel * arc.radius, fabs(arc.linear_travel));
	if (arc.length < cm.arc_segment_len) return (STAT_MINIMUM_LENGTH_MOVE_ERROR); // too short to draw

	arc.time = _get_arc_time(arc.linear_travel, arc.angular_travel, arc.radius);

	// Find the minimum number of segments that meets these constraints...
	float segments_required_for_chordal_accuracy = arc.length / sqrt(4*cm.chordal_tolerance * (2 * arc.radius - cm.chordal_tolerance));
	float segments_required_for_minimum_distance = arc.length / cm.arc_segment_len;
	float segments_required_for_minimum_time = arc.time * MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC;
	arc.segments = floor(min3(segments_required_for_chordal_accuracy,
							   segments_required_for_minimum_distance,
							   segments_required_for_minimum_time));

	arc.segments = max(arc.segments, 1);		//...but is at least 1 segment
	arc.gm.move_time = arc.time / arc.segments;	// gcode state struct gets segment_time, not arc time
	arc.segment_count = (int32_t)arc.segments;
	arc.segment_theta = arc.angular_travel / arc.segments;
	arc.segment_linear_travel = arc.linear_travel / arc.segments;
	arc.center_0 = arc.position[arc.plane_axis_0] - sin(arc.theta) * arc.radius;
	arc.center_1 = arc.position[arc.plane_axis_1] - cos(arc.theta) * arc.radius;
	arc.gm.target[arc.linear_axis] = arc.position[arc.linear_axis];	// initialize the linear target
	return (STAT_OK);
}

/* 
 * _compute_arc_offsets_from_radius() - compute arc center (offset) from radius. 
 *
 *  Needs to calculate the center of the circle that has the designated radius and 
 *	passes through both the current position and the target position
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
 *	center of the arc.
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
 *	When offset is positive the left hand circle will be generated - 
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
 *
 *
 *	Assumes arc singleton has been pre-loaded with target and position.
 *	Parts of this routine were originally sourced from the grbl project.
 */
static stat_t _compute_arc_offsets_from_radius()
{
	// Calculate the change in position along each selected axis
	float x = cm.gm.target[arc.plane_axis_0] - cm.gmx.position[arc.plane_axis_0];
	float y = cm.gm.target[arc.plane_axis_1] - cm.gmx.position[arc.plane_axis_1];

	float disc = 4 * square(arc.radius) - (square(x) + square(y));
	// If the distance between endpoints is greater than the arc diameter, disc 
	// will be negative indicating that the arc is offset into the complex plane
	// beyond the reach of any real CNC. However, numerical errors can flip the
	// sign of disc as it approaches zero (which happens as the arc angle approaches
	// 180 degrees). To avoid mishandling these arcs we use the closest real
	// solution (which will be 0 when disc <= 0). This risks obscuring g-code errors
	// where the radius is actually too small (they will be treated as half circles),
	// but ensures that all valid arcs end up reasonably close to their intended
	// paths regardless of any numerical issues.

	// == -(h * 2 / d)
	float h_x2_div_d = (disc > 0) ? -sqrt(disc) / hypot(x,y) : 0;

	// Invert the sign of h_x2_div_d if circle is counter clockwise (see header notes)
	if (cm.gm.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d;}

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees
	// of travel" (go figure!), even though it is advised against ever generating
	// such circles in a single line of g-code. By inverting the sign of 
	// h_x2_div_d the center of the circles is placed on the opposite side of 
	// the line of travel and thus we get the unadvisably long arcs as prescribed.
	if (arc.radius < 0) { h_x2_div_d = -h_x2_div_d; }

	// Complete the operation by calculating the actual center of the arc
	arc.offset[arc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	arc.offset[arc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	arc.offset[arc.linear_axis] = 0;
	return (STAT_OK);
}

/*
 * _get_arc_time ()
 *
 *	This is a naiive rate-limiting function. The arc drawing time is computed not 
 *	to exceed the time taken in the slowest dimension - in the arc plane or in 
 *	linear travel. Maximum feed rates are compared in each dimension, but the 
 *	comparison assumes that the arc will have at least one segment where the unit 
 *	vector is 1 in that dimension. This is not true for any arbitrary arc, with 
 *	the result that the time returned may be less than optimal.
 *
 *	Room for improvement: At least take the hypotenuse of the planar movement and
 *	the linear travel into account, but how many people actually use helixes?
 */
static float _get_arc_time (const float linear_travel,	// in mm
							const float angular_travel,	// in radians
							const float radius)			// in mm
{
	float tmp;
	float move_time=0;	// picks through the times and retains the slowest
	float planar_travel = fabs(angular_travel * radius);// travel in arc plane

	if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {
		move_time = cm.gm.feed_rate;	// feed rate has been normalized to minutes
		cm.gm.feed_rate = 0;			// reset feed rate so next block requires an explicit feed rate setting
		cm.gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
	} else {
		move_time = sqrt(square(planar_travel) + square(linear_travel)) / cm.gm.feed_rate;
	}
	if ((tmp = planar_travel/cm.a[arc.plane_axis_0].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = planar_travel/cm.a[arc.plane_axis_1].feedrate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = fabs(linear_travel/cm.a[arc.linear_axis].feedrate_max)) > move_time) {
		move_time = tmp;
	}
	return (move_time);
}

/* 
 * _get_theta(float x, float y)
 *
 *	Find the angle in radians of deviance from the positive y axis
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


/* 
 * _test_arc_soft_limits() - return error code if soft limit is exceeded
 *
 *	Test if arc extends beyond arc plane boundaries set in soft limits.
 *
 *	The arc starting position (S) and target (T) define 2 points that divide the 
 *	arc plane into 9 rectangles. The center of the arc is (C). S and T define the 
 *	endpoints of two possible arcs; one that is less than or equal to 180 degrees (acute) 
 *	and one that is greater than 180 degrees (obtuse), depending on the location of (C).
 *
 *	-------------------------------  plane boundaries in X and Y
 *  |         |         |         |
 *  |    1    |    2    |    3    |
 *  |                   |         |
 *	--------- S -------------------
 *  |                   |         |
 *  |    4    |    5    |    6    |
 *  |         |                   |
 *	------------------- T ---------
 *  |        C|                   |  C shows one of many possible center locations
 *  |    7    |    8    |    9    |
 *  |         |         |         |
 *	-------------------------------
 *
 *	C will fall along a diagnonal bisecting 7, 5 and 3, but there is some tolerance in the 
 *	circle algorithm that allows C to deviate from the centerline slightly. As the centerline 
 *	approaches the line connecting S and T the acute arcs will be "above" S and T in sections 
 *	5 or 3, and the obtuse arcs will be "below" in sections 5 or 7. But it's simpler, because 
 *	we know if the arc is greater than 180 degrees if the angular travel value is > pi.
 *
 *	The example below only tests the X axis (0 axis), but testing the other axis is similar
 *
 *	  - If Cx <= Sx and arc is acute; no test is needed
 *
 *	  - If Cx <= Sx and arc is obtuse; test if the radius is greater than 
 *			the distance from Cx to the negative X boundary
 *
 *	  - If Sx < Cx < Tx and arc is acute; test if the radius is greater than
 *			the distance from Cx to the positive X boundary
 *	
 *	  - If Sx < Cx < Tx and arc is obtuse; test if the radius is greater than
 *			the distance from Cx to the positive X boundary
 *
 *	The arc plane is defined by 0 and 1 depending on G17/G18/G19 plane selected,
 *	corresponding to arc planes XY, XZ, YZ, respectively.
 *	
 *	Must be called with all the following set in the arc struct
 *	  -	arc starting position (arc.position)
 *	  - arc ending position (arc.gm.target)
 *	  - arc center (arc.center_0, arc.center_1)
 *	  - arc.radius (arc.radius)
 *	  - arc angular travel in radians (arc.angular_travel)
 *	  - max and min travel in axis 0 and axis 1 (in cm struct)
 *
 */
/*
static stat_t _test_arc_soft_limits()
{
	// test is target falls outside boundaries. This is a 3 dimensional test
	return (cm_test_soft_limits(arc.gm.target));

	// test the 4 arc center point
//	if (arc.gm.target[arc.plane_axis_0] 
	return(STAT_OK);
}
*/

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
