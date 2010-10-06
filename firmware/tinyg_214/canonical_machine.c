/*
 * canonical_machine.c - rs274/ngc canonical machine for cartesian robot.
 * Part of TinyG
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 * Sections of this code are adapted from Simen Svale Skogsrud's grbl
 * 
 * Copyright (c) 2010 Alden S Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * The canonical machine is the layer between the Gcode parser and the
 * motion control code for a specific robot. It keeps state and executes
 * commands - passing the simplest caommands it can down to the motion 
 * control layer. See the notes at the end of gcode.h for more details.
 */

#include <math.h>
#include <stdio.h>
#include <avr/pgmspace.h>			// needed for exception strings

#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "motion_control.h"
#include "canonical_machine.h"
#include "spindle.h"

/* data structures (see notes in gcode.c) */
static struct GCodeModel gm;	// gcode model
static uint8_t cm_status;

/* useful macros */
#ifndef ZERO_MODEL_STATE
#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#endif

//static double _to_millimeters(double value);
static void _cm_set_endpoint_position(uint8_t status);
static double _theta(double x, double y);
static int _gc_compute_radius_arc(void);
static int _gc_compute_center_arc(void);

#define _to_millimeters(a) (gm.inches_mode ? (a * MM_PER_INCH) : a)


/*************************************************************************
 *
 * CANONICAL MACHINING FUNCTIONS
 *
 *	Values are passed in pre-unit_converted state
 *	All operations occur on gm (current model state)
 *
 ************************************************************************/

/*
 * Getters
 *
 * cm_get_next_action() - get next_action from the gm struct
 * cm_get_motion_mode() - get motion mode from the gm struct
 * cm_get_position() - return position from the gm struct into gn struct form
 */

inline uint8_t cm_get_next_action() { return gm.next_action; }

inline uint8_t cm_get_motion_mode() { return gm.motion_mode; }

inline double cm_get_position(uint8_t axis) 
{
	return (gm.inches_mode ? (gm.position[axis] / MM_PER_INCH) : gm.position[axis]);
}

/*
 * Setters - these inhale gn values into the gm struct
 *
 *	Input coordinates are in native block formats (gn form);
 *	i.e. they are not unit adjusted or otherwise pre-processed.
 *	The setters take care of coordinate system, units, and 
 *	distance mode conversions and normalizations.
 *
 * cm_set_positions()	- set all XYZ positions
 * cm_set_targets()		- set all XYZ targets
 * cm_set_offsets()		- set all IJK offsets
 * cm_set_position() 	- set one XYZ position
 * cm_set_target()		- set one XYZ target
 * cm_set_offset()		- set one IJK offset
 * cm_set_radius()		- set radius value
 */

inline void cm_set_targets(double x, double y, double z, double a) 
{ 
	if (gm.absolute_mode || gm.absolute_override) {
		gm.target[X] = _to_millimeters(x);
		gm.target[Y] = _to_millimeters(y);
		gm.target[Z] = _to_millimeters(z);
		gm.target[A] = a;
	} else {
		gm.target[X] += _to_millimeters(x);
		gm.target[Y] += _to_millimeters(y);
		gm.target[Z] += _to_millimeters(z);
		gm.target[A] += a;
	}
}

inline void cm_set_offsets(double i, double j, double k) 
{ 
	gm.offset[0] = _to_millimeters(i);
	gm.offset[1] = _to_millimeters(j);
	gm.offset[2] = _to_millimeters(k);
}

inline void cm_set_radius(double r) 
{ 
	gm.radius = _to_millimeters(r);
}

inline void cm_set_absolute_override(uint8_t setting) 
{ 
	gm.absolute_override = setting;
}

/* 
 * _cm_set_endpoint_position()	- uses internal coordinates only
 *
 * Note: As far as the canonical machine is concerned the final position 
 *	is achieved as soon at the move is executed and the position is now 
 *	the target. In reality, motion_control / steppers will still be 
 *	processing the action and the real tool position is still close to 
 *	the starting point. 
 *
 * Note: This routine will not move the endpoint position if status 
 *	indicates that an error has occurred.
 */

inline void _cm_set_endpoint_position(uint8_t status) 
{ 
	if ((status == TG_OK) || (status == TG_EAGAIN)) {
		memcpy(gm.position, gm.target, sizeof(gm.target));
//		gm.position[X] = gm.target[X];
//		gm.position[Y] = gm.target[Y];
//		gm.position[Z] = gm.target[Z];
//		gm.position[A] = gm.target[A];
	}
}

/* 
 * _theta(double x, double y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static double _theta(double x, double y)
{
	double theta = atan(x/fabs(y));

	if (y>0) {
		return(theta);
	} else {
		if (theta>0) 
	    {
			return(M_PI-theta);
    	} else {
			return(-M_PI-theta);
		}
	}
}

/*--- CANONICAL MACHINING FUNCTIONS ---*/

/* 
 * Initialization and Termination (4.3.2)
 *
 * cm_init_canon() 
 */

void cm_init_canon()
{
	ZERO_MODEL_STATE(&gm);	
	cm_select_plane(CANON_PLANE_XY);		// default planes, 0, 1 and 2
	gm.inches_mode = FALSE;					// FALSE = mm, TRUE = inches
	gm.absolute_mode = TRUE;				// default to absolute mode (G90)
}

/* 
 * Representation (4.3.3)
 *
 * cm_select_plane() - select axis plane Defaults to XY on erroneous specification
 * cm_set_origin_offsets() - G92
 * cm_use_length_units()  - G20, G21
 * cm_set_distance_mode() - G90, G91
 */

uint8_t cm_select_plane(uint8_t plane) 
{
	if (plane == CANON_PLANE_YZ) {
		gm.plane_axis_0 = Y;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = X;
	} else if (plane == CANON_PLANE_XZ) {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Z;
		gm.plane_axis_2 = Y;
	} else {
		gm.plane_axis_0 = X;
		gm.plane_axis_1 = Y;
		gm.plane_axis_2 = Z;
	}
	return (TG_OK);
}

uint8_t cm_set_origin_offsets(double x, double y, double z, double a)
{
	gm.position[X] = _to_millimeters(x);
	gm.position[Y] = _to_millimeters(y);
	gm.position[Z] = _to_millimeters(z);
	gm.position[A] = a;						// in degrees

	mc_set_position(gm.position[X], gm.position[Y], gm.position[Z], gm.position[A]);
	return (TG_OK);
}

uint8_t cm_use_length_units(uint8_t inches_mode)
{
	gm.inches_mode = inches_mode;
	return (TG_OK);
}

uint8_t cm_set_distance_mode(uint8_t absolute_mode)
{
	gm.absolute_mode = absolute_mode;
	return (TG_OK);
}

/* 
 * Free Space Motion (4.3.4)
 *
 * cm_set_traverse_rate() - set seek rate
 * cm_straight_traverse() - G0 linear seek
 */

uint8_t cm_set_traverse_rate(double seek_rate)
{
	gm.seek_rate = _to_millimeters(seek_rate);
	return (TG_OK);
}

uint8_t cm_straight_traverse(double x, double y, double z, double a)
{
	double	axis_time;		// time for an axis to make the move
	double	longest_time=0;	// slowest axis in the move (time, not distance)

	// setup current state (gm struct)
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_targets(x, y, z, a);

	// find the slowest axis in the move
	for (uint8_t i = 0; i < AXES; i++) {
		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
		if (longest_time < axis_time) {
			longest_time = axis_time;
		}
	}

	// skip 0 length moves
	if (longest_time == 0) {
		return (TG_ZERO_LENGTH_MOVE);
	}

	// execute the move
	cm_status = mc_line(gm.target[X], gm.target[Y], 
						gm.target[Z], gm.target[A], longest_time);
	_cm_set_endpoint_position(cm_status);
	return (cm_status);
}

/* 
 * Machining Attributes (4.3.5)
 */ 

/*
 * cm_set_feed_rate() - F parameter
 *
 * Sets feed rate; or sets inverse feed rate if it's active.
 * Converts all values to internal format (mm's)
 * Errs out of feed rate exceeds maximum, but doesn't compute maximum for 
 * inverse feed rate as this would require knowing the move length in advance.
 */

uint8_t cm_set_feed_rate(double feed_rate)
{
	if (gm.inverse_feed_rate_mode) {
		gm.inverse_feed_rate = feed_rate; // minutes per motion for this block only
	} else {
		gm.feed_rate = _to_millimeters(feed_rate);
	}
	return (TG_OK);
}

/*
 * cm_set_inverse_feed_rate() - G93, G94
 *
 *	TRUE = inverse time feed rate in effect - for this block only
 *	FALSE = units per minute feed rate in effect
 */

inline uint8_t cm_set_inverse_feed_rate_mode(uint8_t mode)
{
	gm.inverse_feed_rate_mode = mode;
	return (TG_OK);
}

/*
 * cm_set_motion_control_mode() - G61, G61.1, G64
 */

uint8_t cm_set_motion_control_mode(uint8_t mode)
{
	return (TG_OK);
}

/* 
 * Machining Functions (4.3.6)
 *
 * (see end of file for arc_feed. It's a long one)
 * cm_dwell() - G4, P parameter (seconds)
 * cm_straight_feed() - G1
 */ 

uint8_t cm_dwell(double seconds)
{
	gm.dwell_time = seconds;
	mc_dwell(seconds);
	return (TG_OK);
}

uint8_t cm_straight_feed(double x, double y, double z, double a)
{
	double	move_time;	// time to complete move at desired feedrate
	double	axis_time;	// time for an axis to do its part of the move
	double	longest_time=0;	// slowest axis in the move (time, not distance)

	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
	cm_set_targets(x, y, z, a);

	// get or compute the time it should take to perform the move
	if (gm.inverse_feed_rate_mode) {
		move_time = gm.inverse_feed_rate;
	} else {
		double length = sqrt(square(gm.target[X] - gm.position[X]) +
				 			 square(gm.target[Y] - gm.position[Y]) +
							 square(gm.target[Z] - gm.position[Z]));
		move_time = length / gm.feed_rate;
	}

	// check if any axis requires more time than the move allows
	for (uint8_t i = 0; i < AXES; i++) {
		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
		if (longest_time < axis_time) {
			longest_time = axis_time;
		}
	}

	// skip 0 length moves
	if (longest_time == 0) {
		return (TG_ZERO_LENGTH_MOVE);
	}

	// execute the move
	cm_status = mc_line(gm.target[X], gm.target[Y], 
						gm.target[Z], gm.target[A], 
						max(longest_time, move_time));

	_cm_set_endpoint_position(cm_status);
	return (cm_status);
}

/* 
 * Spindle Functions (4.3.7)
 *
 * cm_set_spindle_speed() - S parameter
 * cm_start_spindle_clockwise() - M3
 * cm_start_spindle_counterclockwise() - M4
 * cm_stop_spindle_turning() - M5
 */

uint8_t cm_set_spindle_speed(double speed)
{
//	if (speed > gm.max_spindle speed) {
//		return (TG_MAX_SPINDLE_SPEED_EXCEEDED);
//	}
	gm.spindle_speed = speed;
	return (TG_OK);
}

uint8_t cm_start_spindle_clockwise(void)
{
	return (TG_OK);
}

uint8_t cm_start_spindle_counterclockwise(void)
{
	return (TG_OK);
}

uint8_t cm_stop_spindle_turning(void)
{
	return (TG_OK);
}

/* 
 * Tool Functions (4.3.8)
 *
 * cm_change_tool() - M6 (This might become a complete tool change cycle)
 * cm_select_tool() - T parameter
 */

uint8_t cm_change_tool(uint8_t tool)
{
	gm.tool = tool;
	return (TG_OK);
}

uint8_t cm_select_tool(uint8_t tool)
{
	gm.tool = tool;
	return (TG_OK);
}

/* 
 * Miscellaneous Functions (4.3.9)
 *
 * cm_comment() - ignore comments (I do)
 * cm_message() - send message to console
 */

uint8_t cm_comment(char *comment)
{
	return (TG_OK);		// no operation
}

uint8_t cm_message(char *message)
{
	printf_P(PSTR("%s\n"), message);
	return (TG_OK);
}

/*
 * Program Functions (4.3.10)
 *
 * This group implements stop, start and end. 
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	cm_program_stop()			(M0, M60)
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	cm_async_stop()				(no code)
 *	cm_async_start()			(no code)
 *	cm_async_end()				(no code)
 *	cm_stop()					(no code, not implemented)
 *
 * cm_program_stop and cm_optional_program_stop are synchronous Gcode 
 * commands that are received through the interpreter. They cause all motion
 * to stop at the end of the current command, including spindle motion. 
 * Note that the stop occurs at the end of the immediately preceding command
 * (i.e. the stop is queued behind the last command).
 *
 * cm_program_end is a stop that also resets the machine to initial state
 *
 * The three asynchronous commands are not specified in RS724. These commands
 * "jump the queue" and are effective immediately. Async_stop and async_start 
 * can be used in sequence to stop motion in the middle of a move then resume.
 * These are meant to be linked to the keyboard "signals" as so:
 *
 *		<ctrl> c	end immediately (ETX, KILL)
 *		<ctrl> x	end immediately (TERM)
 *		<ctrl> s	stop motion immediately (XOFF)
 *		<ctrl> q	restart motion from  async or queued stop (XON)
 *
 * cm_stop() is a cycle specified by RS274 where the machine pauses for some
 * unspecified length of time then resumes. This is not implemented until 
 * someone cen tell me who uses this and for what, and how it's invoked 
 * - given that there is no coresponding gcode for it.
 */

uint8_t cm_program_stop()					// M0, M60
{
	mc_queued_stop();
	return (TG_OK);
}

uint8_t cm_optional_program_stop()			// M1
{
	mc_queued_stop();
	return (TG_OK);
}

uint8_t cm_program_end()					// M2, M30
{
	mc_queued_end();
	return (TG_OK);
}

uint8_t cm_async_stop()
{
	mc_async_stop();
	return (TG_OK);
}

uint8_t cm_async_start()
{
	mc_async_start();
	return (TG_OK);
}

uint8_t cm_async_end()
{
	mc_async_end();
	return (TG_OK);
}


/*--- CANONICAL MACHINING CYCLES ---*/

uint8_t cm_stop()							// not implemented
{
	return (TG_OK);
}

/* 
 * cm_return_to_home() - G28
 */

uint8_t cm_return_to_home()
{
	return (TG_OK);
}


/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _gc_compute_radius_arc() - compute arc center (offset) from radius.
 * _gc_compute_center_arc() - compute arc from I and J (arc center point)
 *
 * Note: This is mostly original grbl code with little modification other
 * 		 than calling differently and refactoring into smaller routines.
 *		 Some changes include:
 *			- trap arc specification errors 
 *			- added A axis motion (e.g. for extruders and similar)
 *			- added code to handle arc moves requested at > max feedrate
 *			- flipped feedrate into time for mc_arc() call
 */

uint8_t cm_arc_feed(double x, double y, double z, double a, // endpoints
					double i, double j, double k, 			// offsets
					double radius, 				  // non-zero sets radius mode
					uint8_t motion_mode)		  // defined motion mode
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = motion_mode;
	cm_set_targets(x, y, z, a);
	cm_set_offsets(i, j, k);
	cm_set_radius(radius);
	cm_status = TG_OK;

	// execute the move
	if (radius) {
		if ((_gc_compute_radius_arc() != TG_OK)) {
			return (cm_status);						// error return
		}
	}
	cm_status = _gc_compute_center_arc();
	_cm_set_endpoint_position(cm_status);
	return (cm_status);
}

/* _gc_compute_radius_arc() - compute arc center (offset) from radius. */

int _gc_compute_radius_arc()
{
	double x;
	double y;
	double h_x2_div_d;

/*  We need to calculate the center of the circle that has the designated 
	radius and passes through both the current position and the target position
		  
	This method calculates the following set of equations where:
	`  [x,y] is the vector from current to target position, 
		d == magnitude of that vector, 
		h == hypotenuse of the triangle formed by the radius of the circle, 
			 the distance to the center of the travel vector. 
		  
	A vector perpendicular to the travel vector [-y,x] is scaled to the length
	of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2]
	to form the new point [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the 
	center of our arc.
          
       d^2 == x^2 + y^2
       h^2 == r^2 - (d/2)^2
       i == x/2 - y/d*h
       j == y/2 + x/d*h
                                                          O <- [i,j]
                                            -  |
                                  r      -     |
                                      -        |
                                   -           | h
                                -              |
                  [0,0] ->  C -----------------+--------------- T  <- [x,y]
                            | <------ d/2 ---->|
                    
       C - Current position
       T - Target position
       O - center of circle that pass through both C and T
       d - distance from C to T
       r - designated radius
       h - distance from center of CT to O
          
	Expanding the equations:

      	d -> sqrt(x^2 + y^2)
        h -> sqrt(4 * r^2 - x^2 - y^2)/2
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
         
	Which can be written:
          
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          
	Which we for size and speed reasons optimize to:

       	h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
      	i = (x - (y * h_x2_div_d))/2
      	j = (y + (x * h_x2_div_d))/2  
	*/
        
	// Calculate the change in position along each selected axis
	x = gm.target[gm.plane_axis_0]-gm.position[gm.plane_axis_0];
	y = gm.target[gm.plane_axis_1]-gm.position[gm.plane_axis_1];

	clear_vector(&gm.offset);
	// == -(h * 2 / d)
	h_x2_div_d = -sqrt(4 * gm.radius*gm.radius - ((x*x) - (y*y))) / hypot(x,y);

	// If r is smaller than d the arc is now traversing the complex plane beyond
	// the reach of any real CNC, and thus - for practical reasons - we will 
	// terminate promptly (well spoken Simen!)
	if(isnan(h_x2_div_d)) { 
		cm_status = TG_FLOATING_POINT_ERROR; 
		return(cm_status); 
	}

	// Invert the sign of h_x2_div_d if circle is counter clockwise 
	// (see sketch below)
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) {
		h_x2_div_d = -h_x2_div_d;
	}

	/*	The counter clockwise circle lies to the left of the target direction. 
		When offset is positive, the left hand circle will be generated - 
		when it is negative the right hand circle is generated.

    
                                     T  <-- Target position
    
                                     ^ 
        Clockwise circles with       |     Clockwise circles with
		this center will have        |     this center will have
        > 180 deg of angular travel  |     < 180 deg of angular travel, 
                          \          |      which is a good thing!
                           \         |         /
    center of arc when  ->  x <----- | -----> x <- center of arc when 
    h_x2_div_d is positive           |             h_x2_div_d is negative
                                     |
    
                                     C  <-- Current position
	*/                

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees
	// of travel" (go figure!), even though it is advised against ever generating
	// such circles in a single line of g-code. By inverting the sign of 
	// h_x2_div_d the center of the circles is placed on the opposite side of 
	// the line of travel and thus we get the unadvisably long arcs as prescribed.
	if (gm.radius < 0) { 
		h_x2_div_d = -h_x2_div_d; 
	}        
        
	// Complete the operation by calculating the actual center of the arc
	gm.offset[gm.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	gm.offset[gm.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	return (cm_status);
} 
    
/*
 * _gc_compute_center_arc() - compute arc from I and J (arc center point)
 */

int _gc_compute_center_arc()
{
	double theta_start;
	double theta_end;
	double angular_travel;
	double radius_tmp;
	double depth;

    /*	This segment sets up an clockwise or counterclockwise arc from the current
		position to the target position around the center designated by the offset
		vector. All theta-values measured in radians of deviance from the positive 
		y-axis. 

                        | <- theta == 0
                      * * *
                    *       *
                  *           *
                  *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
                  *   /
                    C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))
 	*/

	// calculate the theta (angle) of the current point
	theta_start = _theta(-gm.offset[gm.plane_axis_0], -gm.offset[gm.plane_axis_1]);
	if(isnan(theta_start)) { 
		cm_status = TG_ARC_SPECIFICATION_ERROR;
		return(cm_status); 
	}

	// calculate the theta (angle) of the target point
	theta_end = _theta(gm.target[gm.plane_axis_0] 
					- gm.offset[gm.plane_axis_0] 
					- gm.position[gm.plane_axis_0], 
 					  gm.target[gm.plane_axis_1] 
					- gm.offset[gm.plane_axis_1] 
					- gm.position[gm.plane_axis_1]);

	if(isnan(theta_end)) { 
		cm_status = TG_ARC_SPECIFICATION_ERROR; 
		return(cm_status);
	}

	// ensure that the difference is positive so that we have clockwise travel
	if (theta_end < theta_start) {
		theta_end += 2*M_PI;
	}
	angular_travel = theta_end - theta_start;

	// Invert angular motion if the g-code wanted a counterclockwise arc
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) {
		angular_travel = angular_travel - 2*M_PI;
	}

	// Find the radius
	radius_tmp = hypot(gm.offset[gm.plane_axis_0], gm.offset[gm.plane_axis_1]);

	// Calculate the motion along the depth axis of the helix
	depth = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];

/*




	// get or compute the time it should take to perform the move
	if (gm.inverse_feed_rate_mode) {
		move_time = gm.inverse_feed_rate;
	} else {
		double length = sqrt(square(gm.target[X] - gm.position[X]) +
				 			 square(gm.target[Y] - gm.position[Y]) +
							 square(gm.target[Z] - gm.position[Z]));
		move_time = length * gm.feed_rate;
	}

	// check if any axis requires more time than the move allows
	for (uint8_t i = 0; i < AXES; i++) {
		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
		if (longest_time < axis_time) {
			longest_time = axis_time;
		}
	}




*/



	// Trace the arc
	cm_status = mc_arc(theta_start, angular_travel, radius_tmp, depth, 
					   gm.plane_axis_0, gm.plane_axis_1, gm.plane_axis_2, 
        	   		  (gm.inverse_feed_rate_mode) ? gm.inverse_feed_rate : gm.feed_rate, 
					   gm.inverse_feed_rate_mode);

    // Finish off with a line to make sure we arrive exactly where we think we are
	//--> For this to work correctly it must be delivered ONLY after the arc generator 
	// has completed the arc. So the endpoint should be passed to the generator and
	// executed there.
//	cm_status = mc_line(gp.target[X_AXIS], gp.target[Y_AXIS], gp.target[Z_AXIS], 
//					   (gp.inverse_feed_rate_mode) ? gp.inverse_feed_rate : 
//						gp.feed_rate, gp.inverse_feed_rate_mode);
	return (cm_status);
}

/*
 * cm_print_machine_state()
 */

#define GC_MSG_MOTION 0	// these line up with the memory string indexes below
#define GC_MSG_PLANE 5
#define GC_MSG_DISTANCE 8
#define GC_MSG_FEEDRATEMODE 10
#define GC_MSG_UNITS 12
#define GC_MSG_STOP 14
#define GC_MSG_X 17
#define GC_MSG_Y 18
#define GC_MSG_Z 19
#define GC_MSG_A 20
#define GC_MSG_I 21
#define GC_MSG_J 22
#define GC_MSG_SEEKRATE 23
#define GC_MSG_FEEDRATE 24

// put display strings in program memory
char gms00[] PROGMEM = "Motion mode:     G0  - linear traverse (seek)\n";
char gms01[] PROGMEM = "Motion mode:     G1  - linear feed\n";
char gms02[] PROGMEM = "Motion mode:     G2  - clockwise arc feed\n";
char gms03[] PROGMEM = "Motion mode:     G3  - counter clockwise arc feed\n";
char gms04[] PROGMEM = "Motion mode:     G80 - cancel motion mode (none active)\n";
char gms05[] PROGMEM = "Plane selection: G17 - XY plane\n";
char gms06[] PROGMEM = "Plane selection: G18 - XZ plane\n";
char gms07[] PROGMEM = "Plane selection: G19 - YZ plane\n";
char gms08[] PROGMEM = "Distance mode:   G91 - incremental distance\n";// This pair is inverted
char gms09[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
char gms10[] PROGMEM = "Feed rate mode:  G94 - units per minute\n";	// This pair is inverted
char gms11[] PROGMEM = "Feed rate mode:  G93 - inverse time\n";
char gms12[] PROGMEM = "Units:           G21 - millimeters\n";		// This pair is inverted
char gms13[] PROGMEM = "Units:           G20 - inches\n";
char gms14[] PROGMEM = "Stop / end:      --  - running\n";
char gms15[] PROGMEM = "Stop / end:      M0, M1, M30  - stopped\n";
char gms16[] PROGMEM = "Stop / end:      M2, M60  - end\n";
char gms17[] PROGMEM = "Position X:   %8.3f %s\n";
char gms18[] PROGMEM = "Position Y:   %8.3f %s\n";
char gms19[] PROGMEM = "Position Z:   %8.3f %s\n";
char gms20[] PROGMEM = "Position A:   %8.3f degrees\n";
char gms21[] PROGMEM = "Offset I:     %8.3f %s\n";
char gms22[] PROGMEM = "Offset J:     %8.3f %s\n";
char gms23[] PROGMEM = "Seek Rate:    %8.3f %s \\ min\n";
char gms24[] PROGMEM = "Feed Rate:    %8.3f %s \\ min\n";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P gcMsg[] PROGMEM = {	
	gms00, gms01, gms02, gms03, gms04, gms05, gms06, gms07, gms08, gms09,
	gms10, gms11, gms12, gms13, gms14, gms15, gms16, gms17, gms18, gms19,
	gms20, gms21, gms22, gms23, gms24
};

void cm_print_machine_state()
{
	char units[8] = "mm";

	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.motion_mode + GC_MSG_MOTION)]));
	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.set_plane + GC_MSG_PLANE)]));
	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.absolute_mode + GC_MSG_DISTANCE)]));
	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.inverse_feed_rate_mode + GC_MSG_FEEDRATEMODE)]));
	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.inches_mode + GC_MSG_UNITS)]));
	printf_P((PGM_P)pgm_read_word(&gcMsg[(gm.program_flow + GC_MSG_STOP)]));

	if (gm.inches_mode) {
		strncpy(units,"inches", 8);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_X]), gm.position[X] / (25.4), units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_Y]), gm.position[Y] / (25.4), units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_Z]), gm.position[Z] / (25.4), units);
//		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_A]), gm.position[A],"degrees");
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_SEEKRATE]), gm.seek_rate / (25.4), units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_FEEDRATE]), gm.feed_rate / (25.4), units);
	} else {
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_X]), gm.position[X], units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_Y]), gm.position[Y], units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_Z]), gm.position[Z], units);
//		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_A]), gm.position[A],"degrees");
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_SEEKRATE]), gm.seek_rate, units);
		printf_P((PGM_P)pgm_read_word(&gcMsg[GC_MSG_FEEDRATE]), gm.feed_rate, units);
	}
}
