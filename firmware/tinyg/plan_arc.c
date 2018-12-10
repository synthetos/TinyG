/*
 * plan_arc.c - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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
/* This module actually contains some parts that belong in the canonical machine,
 * and other parts that belong at the motion planner level, but the whole thing is
 * treated as if it were part of the motion planner.
 */

#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "util.h"

#include "controller.h"     //+++++

// Allocate arc planner singleton structure

arc_t arc;

// Local functions

static stat_t _compute_arc(const bool radius_f);
static void _compute_arc_offsets_from_radius(void);
static float _estimate_arc_time (float arc_time);
static stat_t _test_arc_soft_limits(void);

/*****************************************************************************
 * Canonical Machining arc functions (arc prep for planning and runtime)
 *
 * cm_arc_init()     - initialize arcs
 * cm_arc_feed()     - canonical machine entry point for arc
 * cm_arc_callback() - main-loop callback for arc generation
 * cm_abort_arc()    - stop an arc in process
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
 * cm_abort_arc() - stop arc movement without maintaining position
 *
 *	OK to call if no arc is running
 */

void cm_abort_arc()
{
	arc.run_state = BLOCK_IDLE;
}

/*
 * cm_arc_callback() - generate an arc
 *
 *  cm_arc_cycle_callback() is called from the controller main loop. Each time it's called
 *  it queues as many arc segments (lines) as it can before it blocks, then returns.
 *
 *  Parts of this routine were informed by the grbl project.
 */

stat_t cm_arc_callback()
{
	if (arc.run_state == BLOCK_IDLE) {
        return (STAT_NOOP);
    }
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) {
        return (STAT_EAGAIN);
    }
	arc.theta += arc.segment_theta;
	arc.gm.target[arc.plane_axis_0] = arc.center_0 + sin(arc.theta) * arc.radius;
	arc.gm.target[arc.plane_axis_1] = arc.center_1 + cos(arc.theta) * arc.radius;
	arc.gm.target[arc.linear_axis] += arc.segment_linear_travel;

	mp_aline(&arc.gm);								// run the line
	copy_vector(arc.position, arc.gm.target);		// update arc current position

	if (--arc.segment_count > 0) {
        return (STAT_EAGAIN);
    }
	arc.run_state = BLOCK_IDLE;
	return (STAT_OK);
}

/*
 * cm_arc_feed() - canonical machine entry point for arcs
 *
 * Generates an arc by queuing line segments to the move buffer. The arc is
 * approximated by generating a large number of tiny, linear segments.
 */

stat_t cm_arc_feed(const float target[], const bool target_f[],     // target endpoint
                   const float offset[], const bool offset_f[],     // IJK offsets
                   const float radius, const bool radius_f,         // radius if radius mode
                   const float P_word, const bool P_word_f,         // parameter
                   const bool modal_g1_f,                           // modal group flag for motion group
                   const uint8_t motion_mode)                       // defined motion mode
{
	// Start setting up the arc and trapping arc specification errors

    // Trap some precursor cases. Since motion mode (MODAL_GROUP_G1) persists from the
    // previous move it's possible for non-modal commands such as F or P to arrive here
    // when no motion has actually been specified. It's also possible to run an arc as
    // simple as "I25" if CW or CCW motion mode was already set by a previous block.
    // Here are 2 cases to handle if CW or CCW motion mode was set by a previous block:
    //
    // Case 1: F, P or other non modal is specified but no movement is specified 
    //         (no offsets or radius). This is OK: return STAT_OK
    //
    // Case 2: Movement is specified w/o a new G2 or G3 word in the (new) block.
    //         This is OK: continue the move
    //
    if ((!modal_g1_f) &&                                                // G2 or G3 not present
        (!(offset_f[AXIS_X] | offset_f[AXIS_Y] | offset_f[AXIS_Z])) &&  // no offsets are present
        (!radius_f)) {                                                  // radius not present
        return (STAT_OK);
    }

    // Some things you might think are errors but are not:
    //  - offset specified for linear axis (i.e. not one of the plane axes). Ignored
    //  - rotary axes are present. Ignored

	// trap missing feed rate
	if (fp_ZERO(cm.gm.feed_rate)) {
    	return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}

	// Set the arc plane for the current G17/G18/G19 setting and test arc specification
	// Plane axis 0 and 1 are the arc plane, the linear axis is normal to the arc plane.
	if (cm.gm.select_plane == CANON_PLANE_XY) {	        // G17 - the vast majority of arcs are in the G17 (XY) plane
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
    } else {
        return(cm_panic_P(STAT_GCODE_ACTIVE_PLANE_IS_MISSING, PSTR("no plane axis"))); // plane axis has impossible value
    }

    // test if no endpoints are specified in the selected plane
    arc.full_circle = false;        // initial condition
    if (!(target_f[arc.plane_axis_0] || target_f[arc.plane_axis_1])) {
        if (radius_f) {             // in radius mode arcs missing both endpoints is an error
            return (STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE);
        } else {
            arc.full_circle = true; // in center format arc this specifies a full circle
        }
    }

    // test radius arcs for radius tolerance
    if (radius_f) {
        arc.radius = _to_millimeters(radius);           // set radius to internal format (mm)
        if (fabs(arc.radius) < MIN_ARC_RADIUS) {        // radius value must be > minimum radius
            return (STAT_ARC_RADIUS_OUT_OF_TOLERANCE);
        }
    } 
    else {  // test that center format absolute distance mode arcs have both offsets specified
        if (cm.gm.arc_distance_mode == ABSOLUTE_MODE) {
            if (!(offset_f[arc.plane_axis_0] && offset_f[arc.plane_axis_1])) {  // if one or both offsets are missing
                return (STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE);
            }
        }
    }

    // Set arc rotations using P word
    if (P_word_f) {
        if (P_word < 0) {  // If P is present it must be a positive integer
            return (STAT_P_WORD_IS_NEGATIVE);
        }
        if (floor(P_word) - (P_word) > 0) {
            return (STAT_P_WORD_IS_NOT_AN_INTEGER);
        }
        arc.rotations = P_word;
    } else {
        if (arc.full_circle) {      // arc rotations default to 1 for full circles
            arc.rotations = 1;
        } else {
            arc.rotations = 0;      // no rotations
        }
    }

	// set values in the Gcode model state & copy it (linenum was already captured)
	cm_set_model_target(target, target_f);

    // in radius mode it's an error for start == end
    if (radius_f) {
        if ((fp_EQ(cm.gmx.position[AXIS_X], cm.gm.target[AXIS_X])) &&
            (fp_EQ(cm.gmx.position[AXIS_Y], cm.gm.target[AXIS_Y])) &&
            (fp_EQ(cm.gmx.position[AXIS_Z], cm.gm.target[AXIS_Z]))) {
            return (cm_alarm_P(STAT_ARC_ENDPOINT_IS_STARTING_POINT, PSTR("arc start end end point cannot be the same in a radius arc")));
        }
    }

    // *** now get down to the rest of the work setting up the arc for execution ***
	cm.gm.motion_mode = motion_mode;
	cm_set_work_offsets();                          // capture the fully resolved offsets to gm
	memcpy(&arc.gm, &cm.gm, sizeof(GCodeState_t));  // copy GCode context to arc singleton - some will be overwritten to run segments
	copy_vector(arc.position, cm.gmx.position);     // set initial arc position from gcode model

    // setup offsets
    arc.offset[OFS_I] = _to_millimeters(offset[OFS_I]); // copy offsets with conversion to canonical form (mm)
    arc.offset[OFS_J] = _to_millimeters(offset[OFS_J]);
    arc.offset[OFS_K] = _to_millimeters(offset[OFS_K]);

    if (arc.gm.arc_distance_mode == ABSOLUTE_MODE) {    // adjust offsets if in absolute mode
         arc.offset[OFS_I] -= arc.position[AXIS_X];
         arc.offset[OFS_J] -= arc.position[AXIS_Y];
         arc.offset[OFS_K] -= arc.position[AXIS_Z];
    }

    if ((fp_ZERO(arc.offset[OFS_I])) &&             // it's an error if no offsets are provided
        (fp_ZERO(arc.offset[OFS_J])) &&
        (fp_ZERO(arc.offset[OFS_K]))) {
        return (cm_alarm_P(STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE, PSTR("arc offsets missing or zero")));
    }

	// compute arc runtime values
	ritorno(_compute_arc(radius_f));

	// test arc soft limits
	stat_t status = _test_arc_soft_limits();
	if (status != STAT_OK) {
    	cm.gm.motion_mode = MOTION_MODE_CANCEL;
    	copy_vector(cm.gm.target, arc.position);		// reset model position
	    return (cm_alarm_P(status, PSTR("arc soft_limits")));   // throw an alarm
	}

	cm_cycle_start();						            // if not already started
	arc.run_state = BLOCK_RUNNING;				        // enable arc to be run from the callback
	cm_finalize_move();
	return (STAT_OK);
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
 *  Parts of this routine were informed by the grbl project.
 */

static stat_t _compute_arc(const bool radius_f)
{
    // Compute IJK offsets and starting radius
    if (radius_f) {                         // indicates a radius arc
        _compute_arc_offsets_from_radius();
    } else {                                // compute start radius
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
    if ((err > ARC_RADIUS_ERROR_MAX) ||
       ((err > ARC_RADIUS_ERROR_MIN) && (err > arc.radius * ARC_RADIUS_TOLERANCE))) {
        return (cm_alarm_P(STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT, PSTR("arc center point error exceeds limits")));
    }

    // Compute the angular travel
    // Calculate the theta angle of the current position (theta is also needed for calculating center point)
    // Note: gcc atan2 reverses args, i.e.: atan2(Y,X)
    arc.theta = atan2(-arc.offset[arc.plane_axis_0], -arc.offset[arc.plane_axis_1]);

    // Compute angular travel if not a full circle arc
    if (!arc.full_circle) {
        arc.angular_travel = atan2(end_0, end_1) - arc.theta; // travel = theta_end - theta_start

        // correct for atan2 output quadrants
        if (arc.gm.motion_mode == MOTION_MODE_CW_ARC) {
            if (arc.angular_travel <= 0) { arc.angular_travel += 2*M_PI; }
        } else {
            if (arc.angular_travel > 0)  { arc.angular_travel -= 2*M_PI; }
        }
        // add in travel for rotations
        if (arc.angular_travel >= 0) { arc.angular_travel += 2*M_PI * arc.rotations; }
        else                         { arc.angular_travel -= 2*M_PI * arc.rotations; }
    }
    // Compute full-circle arcs
    else {
        if (arc.gm.motion_mode == MOTION_MODE_CCW_ARC) { arc.rotations *= -1; }
        arc.angular_travel = 2 * M_PI * arc.rotations;
    }

    // Trap zero movement arcs
    if (fp_ZERO(arc.angular_travel)) {
        return (cm_alarm_P(STAT_ARC_ENDPOINT_IS_STARTING_POINT, PSTR("arc has no movement - identical start and end points")));
    }

    // Calculate travel in the plane and the depth axis of the helix
    // Length is the total mm of travel of the helix (or just the planar arc)
    arc.linear_travel = arc.gm.target[arc.linear_axis] - arc.position[arc.linear_axis];
    arc.planar_travel = arc.angular_travel * arc.radius;
    arc.length = hypotf(arc.planar_travel, fabs(arc.linear_travel));

    // Find the minimum number of segments that meet accuracy and time constraints...
    // Note: removed segment_length test as segment_time accounts for this (build 083.37)
    float arc_time;
    float segments_for_minimum_time = _estimate_arc_time(arc_time) * (MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC);
    float segments_for_chordal_accuracy = arc.length / sqrt(4*cm.chordal_tolerance * (2 * arc.radius - cm.chordal_tolerance));
    arc.segments = floor(min(segments_for_chordal_accuracy, segments_for_minimum_time));
    arc.segments = max(arc.segments, (float)1.0);		//...but is at least 1 segment

    if (arc.gm.feed_rate_mode == INVERSE_TIME_MODE) {
        arc.gm.feed_rate /= arc.segments; 
    }    
    // setup the rest of the arc parameters
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
 *  Parts of this routine were informed by the grbl project.
 */
static void _compute_arc_offsets_from_radius()
{
	// Calculate the change in position along each selected axis
	float x = arc.gm.target[arc.plane_axis_0] - arc.position[arc.plane_axis_0];
	float y = arc.gm.target[arc.plane_axis_1] - arc.position[arc.plane_axis_1];

	// *** From Forrest Green - Other Machine Co, 3/27/14
	// If the distance between endpoints is greater than the arc diameter, disc will be
	// negative indicating that the arc is offset into the complex plane beyond the reach 
    // of any real CNC. However, numerical errors can flip the sign of disc as it approaches
    // zero (which happens as the arc angle approaches 180 degrees). To avoid mishandling 
    // these arcs we use the closest real solution (which will be 0 when disc <= 0). This 
    // risks obscuring g-code errors where the radius is actually too small (they will be 
    // treated as half circles), but ensures that all valid arcs end up reasonably close 
    // to their intended paths regardless of any numerical issues.
	float disc = 4 * square(arc.radius) - (square(x) + square(y));

	// h_x2_div_d == -(h * 2 / d)
	float h_x2_div_d = (disc > 0) ? -sqrt(disc) / hypotf(x,y) : 0;

	// Invert the sign of h_x2_div_d if circle is counter clockwise (see header notes)
	if (arc.gm.motion_mode == MOTION_MODE_CCW_ARC) {
        h_x2_div_d = -h_x2_div_d;
    }

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" 
    // (go figure!), even though it is advised against ever generating such circles in a 
    // single Gcode block. By inverting the sign of h_x2_div_d the center of the circles is 
    // placed on the opposite side of the line of travel and thus we get the inadvisably 
    // long arcs as prescribed.
	if (arc.radius < 0) {
        h_x2_div_d = -h_x2_div_d;
        arc.radius *= -1;           // and flip the radius sign while you are at it
    }

	// Complete the operation by calculating the actual center of the arc
	arc.offset[arc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	arc.offset[arc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	arc.offset[arc.linear_axis] = 0;
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
static float _estimate_arc_time (float arc_time)
{
	// Determine move time at requested feed rate
	if (arc.gm.feed_rate_mode == INVERSE_TIME_MODE) {
		arc_time = arc.gm.feed_rate;    // inverse feed rate has been normalized to minutes
	} else {
		arc_time = arc.length / cm.gm.feed_rate;
	}

	// Downgrade the time if there is a rate-limiting axis
	arc_time = max(arc_time, (float)fabs(arc.planar_travel/cm.a[arc.plane_axis_0].feedrate_max));
	arc_time = max(arc_time, (float)fabs(arc.planar_travel/cm.a[arc.plane_axis_1].feedrate_max));
	if (fabs(arc.linear_travel) > 0) {
		arc_time = max(arc_time, (float)fabs(arc.linear_travel/cm.a[arc.linear_axis].feedrate_max));
	}
    return (arc_time);
}   

/*
 * _test_arc_soft_limits() - return error code if soft limit is exceeded
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
 *	The example below only tests the X axis (0 axis), but testing the other axis is similar
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
/*
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
*/
static stat_t _test_arc_soft_limits()
{
/*
    if (cm.soft_limit_enable == true) {

        // Test if target falls outside boundaries. This is a 3 dimensional test
        // so it also checks the linear axis of the arc (helix axis)
        ritorno(cm_test_soft_limits(arc.gm.target));

        // test arc extents
        ritorno(_test_arc_soft_limit_plane_axis(arc.center_0, arc.plane_axis_0));
        ritorno(_test_arc_soft_limit_plane_axis(arc.center_1, arc.plane_axis_1));
    }
*/
    return(STAT_OK);
}
