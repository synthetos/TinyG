/*
 * plan_arc.c - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

// Allocate arc planner singleton structure

arc_t arc;

// Local functions
static stat_t _compute_arc(void);
static stat_t _compute_arc_offsets_from_radius(void);
static void _estimate_arc_time(void);
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
 * Generates an arc by queuing line segments to the move buffer. The arc is
 * approximated by generating a large number of tiny, linear arc_segments.
 */
stat_t cm_arc_feed(float target[], float flags[],       // arc endpoints
				   float i, float j, float k,           // raw arc offsets
				   float radius,                        // non-zero radius implies radius mode
				   uint8_t motion_mode)                 // defined motion mode
{
	////////////////////////////////////////////////////
	// Set axis plane and trap arc specification errors

	// trap missing feed rate
	if ((cm.gm.feed_rate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feed_rate))) {
    	return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}

    // set radius mode flag and do simple test(s)
	bool radius_f = fp_NOT_ZERO(cm.gf.arc_radius);			    // set true if radius arc
    if ((radius_f) && (cm.gn.arc_radius < MIN_ARC_RADIUS)) {    // radius value must be + and > minimum radius
        return (STAT_ARC_RADIUS_OUT_OF_TOLERANCE);
    }

    // setup some flags
	bool target_x = fp_NOT_ZERO(flags[AXIS_X]);	                // set true if X axis has been specified
	bool target_y = fp_NOT_ZERO(flags[AXIS_Y]);
	bool target_z = fp_NOT_ZERO(flags[AXIS_Z]);

    bool offset_i = fp_NOT_ZERO(cm.gf.arc_offset[0]);	        // set true if offset I has been specified
    bool offset_j = fp_NOT_ZERO(cm.gf.arc_offset[1]);           // J
    bool offset_k = fp_NOT_ZERO(cm.gf.arc_offset[2]);           // K

	// Set the arc plane for the current G17/G18/G19 setting and test arc specification
	// Plane axis 0 and 1 are the arc plane, the linear axis is normal to the arc plane.
	if (cm.gm.select_plane == CANON_PLANE_XY) {	// G17 - the vast majority of arcs are in the G17 (XY) plane
    	arc.plane_axis_0 = AXIS_X;
    	arc.plane_axis_1 = AXIS_Y;
    	arc.linear_axis  = AXIS_Z;
        if (radius_f) {
            if (!(target_x || target_y)) {                      // must have at least one endpoint specified
        	    return (STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE);
            }
        } else { // center format arc tests
            if (offset_k) { // it's OK to be missing either or both i and j, but error if k is present
        	    return (STAT_ARC_SPECIFICATION_ERROR);
            }
        }

    } else if (cm.gm.select_plane == CANON_PLANE_XZ) {	// G18
    	arc.plane_axis_0 = AXIS_X;
    	arc.plane_axis_1 = AXIS_Z;
    	arc.linear_axis  = AXIS_Y;
        if (radius_f) {
            if (!(target_x || target_z))
                return (STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE);
        } else {
            if (offset_j)
                return (STAT_ARC_SPECIFICATION_ERROR);
        }

    } else if (cm.gm.select_plane == CANON_PLANE_YZ) {	// G19
    	arc.plane_axis_0 = AXIS_Y;
    	arc.plane_axis_1 = AXIS_Z;
    	arc.linear_axis  = AXIS_X;
        if (radius_f) {
            if (!(target_y || target_z))
                return (STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE);
        } else {
            if (offset_i)
                return (STAT_ARC_SPECIFICATION_ERROR);
        }
	}

	// set values in the Gcode model state & copy it (linenum was already captured)
	cm_set_model_target(target, flags);

    // in radius mode it's an error for start == end
    if(radius_f) {
        if ((fp_EQ(cm.gmx.position[AXIS_X], cm.gm.target[AXIS_X])) &&
            (fp_EQ(cm.gmx.position[AXIS_Y], cm.gm.target[AXIS_Y])) &&
            (fp_EQ(cm.gmx.position[AXIS_Z], cm.gm.target[AXIS_Z]))) {
            return (STAT_ARC_ENDPOINT_IS_STARTING_POINT);
        }
    }

    // now get down to the rest of the work setting up the arc for execution
	cm.gm.motion_mode = motion_mode;
	cm_set_work_offsets(&cm.gm);					// capture the fully resolved offsets to gm
	memcpy(&arc.gm, &cm.gm, sizeof(GCodeState_t));	// copy GCode context to arc singleton - some will be overwritten to run segments
	copy_vector(arc.position, cm.gmx.position);		// set initial arc position from gcode model

	arc.radius = _to_millimeters(radius);			// set arc radius or zero

	arc.offset[0] = _to_millimeters(i);				// copy offsets with conversion to canonical form (mm)
	arc.offset[1] = _to_millimeters(j);
	arc.offset[2] = _to_millimeters(k);

	arc.rotations = floor(fabs(cm.gn.parameter));   // P must be a positive integer - force it if not

	// determine if this is a full circle arc. Evaluates true if no target is set
	arc.full_circle = (fp_ZERO(flags[arc.plane_axis_0]) & fp_ZERO(flags[arc.plane_axis_1]));

	// compute arc runtime values
	ritorno(_compute_arc());

	if (fp_ZERO(arc.length)) {
        return (STAT_MINIMUM_LENGTH_MOVE);          // trap zero length arcs that _compute_arc can throw
    }

/*	// test arc soft limits
	stat_t status = _test_arc_soft_limits();
	if (status != STAT_OK) {
    	cm.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
    	copy_vector(cm.gm.target, cm.gmx.position);		// reset model position
    	return (cm_soft_alarm(status));
	}
*/
	cm_cycle_start();						// if not already started
	arc.run_state = MOVE_RUN;				// enable arc to be run from the callback
	cm_finalize_move();
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
	if (arc.run_state == MOVE_OFF)
        return (STAT_NOOP);

	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM)
        return (STAT_EAGAIN);

	arc.theta += arc.arc_segment_theta;
	arc.gm.target[arc.plane_axis_0] = arc.center_0 + sin(arc.theta) * arc.radius;
	arc.gm.target[arc.plane_axis_1] = arc.center_1 + cos(arc.theta) * arc.radius;
	arc.gm.target[arc.linear_axis] += arc.arc_segment_linear_travel;
	mp_aline(&arc.gm);								// run the line
	copy_vector(arc.position, arc.gm.target);		// update arc current position

	if (--arc.arc_segment_count > 0)
        return (STAT_EAGAIN);
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
	// Compute radius. A non-zero radius value indicates a radius arc
    if (fp_NOT_ZERO(arc.radius)) {                  // indicates a radius arc
        _compute_arc_offsets_from_radius();
    } else {                                        // compute start radius
        arc.radius = hypotf(-arc.offset[arc.plane_axis_0], -arc.offset[arc.plane_axis_1]);
    }

    // Test arc specification for correctness according to:
    // http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G2-G3-Arc
    // "It is an error if: when the arc is projected on the selected plane, the distance from
    //  the current point to the center differs from the distance from the end point to the
    //  center by more than (.05 inch/.5 mm) OR ((.0005 inch/.005mm) AND .1% of radius)."

    // Compute end radius from the center of circle (offsets) to target endpoint
    float end_0 = arc.gm.target[arc.plane_axis_0] - arc.position[arc.plane_axis_0] - arc.offset[arc.plane_axis_0];
    float end_1 = arc.gm.target[arc.plane_axis_1] - arc.position[arc.plane_axis_1] - arc.offset[arc.plane_axis_1];
    float err = fabs(hypotf(end_0, end_1) - arc.radius);   // end radius - start radius
    if ( (err > ARC_RADIUS_ERROR_MAX) || 
        ((err < ARC_RADIUS_ERROR_MIN) && 
         (err > arc.radius * ARC_RADIUS_TOLERANCE)) ) {
//        return (STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT);
        return (STAT_ARC_SPECIFICATION_ERROR);
    }

	// Calculate the theta (angle) of the current point (position)
	// arc.theta is angular starting point for the arc (also needed later for calculating center point)
    arc.theta = atan2(-arc.offset[arc.plane_axis_0], -arc.offset[arc.plane_axis_1]);

    // g18_correction is used to invert G18 XZ plane arcs for proper CW orientation
    float g18_correction = (cm.gm.select_plane == CANON_PLANE_XZ) ? -1 : 1;

	if (arc.full_circle) {                                  // if full circle you can skip the stuff in the else clause
    	arc.angular_travel = 0;                             // angular travel always starts as zero for full circles
    	if (fp_ZERO(arc.rotations)) {                       // handle the valid case of a full circle arc w/P=0
            arc.rotations = 1.0;
        }
    } else {                                                // ... it's not a full circle
        arc.theta_end = atan2(end_0, end_1);

        // Compute the angular travel
        if (fp_EQ(arc.theta_end, arc.theta)) {
	        arc.angular_travel = 0;                         // very large radii arcs can have zero angular travel (thanks PartKam)
        } else {
	        if (arc.theta_end < arc.theta) {                // make the difference positive so we have clockwise travel
                arc.theta_end += (2*M_PI * g18_correction);
            }
	        arc.angular_travel = arc.theta_end - arc.theta; // compute positive angular travel
    	    if (cm.gm.motion_mode == MOTION_MODE_CCW_ARC) { // reverse travel direction if it's CCW arc
                arc.angular_travel -= (2*M_PI * g18_correction);
            }
        }
	}

    // Add in travel for rotations
    if (cm.gm.motion_mode == MOTION_MODE_CW_ARC) {
        arc.angular_travel += (2*M_PI * arc.rotations * g18_correction);
    } else {
        arc.angular_travel -= (2*M_PI * arc.rotations * g18_correction);
    }

	// Calculate travel in the depth axis of the helix and compute the time it should take to perform the move
	// arc.length is the total mm of travel of the helix (or just a planar arc)
	arc.linear_travel = arc.gm.target[arc.linear_axis] - arc.position[arc.linear_axis];
	arc.planar_travel = arc.angular_travel * arc.radius;
	arc.length = hypotf(arc.planar_travel, arc.linear_travel);  // NB: hypot is insensitive to +/- signs
	_estimate_arc_time();	// get an estimate of execution time to inform arc_segment calculation

	// Find the minimum number of arc_segments that meets these constraints...
	float arc_segments_for_chordal_accuracy = arc.length / sqrt(4*cm.chordal_tolerance * (2 * arc.radius - cm.chordal_tolerance));
	float arc_segments_for_minimum_distance = arc.length / cm.arc_segment_len;
	float arc_segments_for_minimum_time = arc.arc_time * MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC;

	arc.arc_segments = floor(min3(arc_segments_for_chordal_accuracy,
							      arc_segments_for_minimum_distance,
							      arc_segments_for_minimum_time));

	arc.arc_segments = max(arc.arc_segments, 1);            //...but is at least 1 arc_segment
 	arc.gm.move_time = arc.arc_time / arc.arc_segments;     // gcode state struct gets arc_segment_time, not arc time
	arc.arc_segment_count = (int32_t)arc.arc_segments;
	arc.arc_segment_theta = arc.angular_travel / arc.arc_segments;
	arc.arc_segment_linear_travel = arc.linear_travel / arc.arc_segments;
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

	// *** From Forrest Green - Other Machine Co, 3/27/14
	// If the distance between endpoints is greater than the arc diameter, disc
	// will be negative indicating that the arc is offset into the complex plane
	// beyond the reach of any real CNC. However, numerical errors can flip the
	// sign of disc as it approaches zero (which happens as the arc angle approaches
	// 180 degrees). To avoid mishandling these arcs we use the closest real
	// solution (which will be 0 when disc <= 0). This risks obscuring g-code errors
	// where the radius is actually too small (they will be treated as half circles),
	// but ensures that all valid arcs end up reasonably close to their intended
	// paths regardless of any numerical issues.
	float disc = 4 * square(arc.radius) - (square(x) + square(y));

	// h_x2_div_d == -(h * 2 / d)
	float h_x2_div_d = (disc > 0) ? -sqrt(disc) / hypotf(x,y) : 0;

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
 * _estimate_arc_time ()
 *
 *	Returns a naiive estimate of arc execution time to inform segment calculation.
 *	The arc time is computed not to exceed the time taken in the slowest dimension
 *	in the arc plane or in linear travel. Maximum feed rates are compared in each
 *	dimension, but the comparison assumes that the arc will have at least one segment
 *	where the unit vector is 1 in that dimension. This is not true for any arbitrary arc,
 *	with the result that the time returned may be less than optimal.
 */
static void _estimate_arc_time ()
{
	// Determine move time at requested feed rate
	if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {
		arc.arc_time = cm.gm.feed_rate;	                // inverse feed rate has been normalized to minutes
		cm.gm.feed_rate = 0;                            // reset feed rate so next block requires an explicit feed rate setting
		cm.gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
	} else {
		arc.arc_time = arc.length / cm.gm.feed_rate;
	}

	// Downgrade the time if there is a rate-limiting axis
	arc.arc_time = max(arc.arc_time, arc.planar_travel/cm.a[arc.plane_axis_0].feedrate_max);
	arc.arc_time = max(arc.arc_time, arc.planar_travel/cm.a[arc.plane_axis_1].feedrate_max);
	if (fabs(arc.linear_travel) > 0) {
		arc.arc_time = max(arc.arc_time, fabs(arc.linear_travel/cm.a[arc.linear_axis].feedrate_max));
	}
}

/*
 * _test_arc_soft_limits() - return error status if soft limit is exceeded
 *
 *	Test if arc extends beyond arc plane boundaries set in soft limits.
 *
 *	The arc starting position (P) and target (T) define 2 points that divide the
 *	arc plane into 9 rectangles. The center of the arc is (C). P and T define the
 *	endpoints of two possible arcs; one that is less than or equal to 180 degrees (acute)
 *	and one that is greater than 180 degrees (obtuse), depending on the location of (C).
 *
 *	-------------------------------  plane boundaries in X and Y
 *  |         |         |         |
 *  |    1    |    2    |    3    |
 *  |                   |         |
 *	--------- P -------------------
 *  |                   |         |
 *  |    4    |    5    |    6    |
 *  |         |                   |
 *	------------------- T ---------
 *  |        C|                   |  C shows one of many possible center locations
 *  |    7    |    8    |    9    |
 *  |         |         |         |
 *	-------------------------------
 *
 *	C will fall along a diagonal bisecting 7, 5 and 3, but there is some tolerance in the
 *	circle algorithm that allows C to deviate from the centerline slightly. As the centerline
 *	approaches the line connecting S and T the acute arcs will be "above" S and T in sections
 *	5 or 3, and the obtuse arcs will be "below" in sections 5 or 7. But it's simpler, because
 *	we know that the arc is > 180 degrees (obtuse) if the angular travel value is > pi.
 *
 *	The example below only tests the X axis (0 plane axis), but testing the other axes is similar
 *
 *	  (1) If Cx <= Px and arc is acute; no test is needed
 *
 *	  (2) If Cx <= Px and arc is obtuse; test if the radius is greater than
 *			the distance from Cx to the negative X boundary
 *
 *	  (3) If Px < Cx < Tx and arc is acute; test if the radius is greater than
 *			the distance from Cx to the positive X boundary
 *
 *	  (4) If Px < Cx < Tx and arc is obtuse; test if the radius is greater than
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
 */
/* UNUSED
static stat_t _test_arc_soft_limit_plane_axis(float center, uint8_t plane_axis)
{
	if (center <= arc.position[plane_axis]) {
		if (arc.angular_travel < M_PI) {							// case (1)
			return (STAT_OK);
		}
		if ((center - arc.radius) < cm.a[plane_axis].travel_min) {	// case (2)
			return (STAT_SOFT_LIMIT_EXCEEDED);
		}
	}
	if ((center + arc.radius) > cm.a[plane_axis].travel_max) {		// cases (3) and (4)
		return (STAT_SOFT_LIMIT_EXCEEDED);
	}
	return(STAT_OK);
}

static stat_t _test_arc_soft_limits()
{
	if (cm.soft_limit_enable == true) {

		// Test if target falls outside boundaries. This is a 3 dimensional test
		// so it also checks the linear axis of the arc (helix axis)
		ritorno(cm_test_soft_limits(arc.gm.target));

		// test arc extents
		ritorno(_test_arc_soft_limit_plane_axis(arc.center_0, arc.plane_axis_0));
		ritorno(_test_arc_soft_limit_plane_axis(arc.center_1, arc.plane_axis_1));
	}
	return(STAT_OK);
}

*/