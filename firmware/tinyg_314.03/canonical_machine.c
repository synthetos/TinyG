/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 */
/* TinyG is free software: you can redistribute it and/or modify it 
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
 */
/* The canonical machine is the layer between the Gcode parser and the
 * motion control code for a specific robot. It keeps state and executes
 * commands - passing the simplest caommands it can down to the motion 
 * control layer. See the notes at the end of gcode.h for more details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>				// needed for memcpy, memset
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "planner.h"
#include "canonical_machine.h"
#include "controller.h"
#include "limit_switches.h"
#include "spindle.h"

/* data structures (see notes in gcode.c) */
static uint8_t cm_status;

/* useful macros */
#ifndef ZERO_MODEL_STATE
#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#endif
#define _to_millimeters(a) ((gm.inches_mode == TRUE) ? (a * MM_PER_INCH) : a)

/* local function prototypes */
static double _cm_get_move_time();
static void _cm_set_endpoint_position(uint8_t status);
static uint8_t _cm_compute_radius_arc(void);
static uint8_t _cm_compute_center_arc(void);
static double _cm_get_arc_time (const double linear_travel, 
								const double angular_travel, const double radius);
static double _theta(const double x, const double y);

/*************************************************************************
 *
 * HELPERS AND UTILITY FUNCTIONS
 *
 *	These functions are not part of the NIST defined functions
 *
 ************************************************************************/

/*
 * Save and restore gcode model
 */

void cm_save_gcode_model()   {memcpy(&gt, &gm, sizeof(struct GCodeModel));}
void cm_restore_gcode_model(){memcpy(&gm, &gt, sizeof(struct GCodeModel));}

/*
 * Getters
 *
 * cm_get_next_action() - get next_action from the gm struct
 * cm_get_motion_mode() - get motion mode from the gm struct
 * cm_get_inches_mode() - get inches mode from the gm struct
 * cm_get_absolute_mode() - get absolute mode from the gm struct
 * cm_get_position() - return position from the gm struct into gn struct form
 */

inline uint8_t cm_get_next_action() { return gm.next_action; }
inline uint8_t cm_get_motion_mode() { return gm.motion_mode; }
inline uint8_t cm_get_inches_mode() { return gm.inches_mode; }
inline uint8_t cm_get_absolute_mode() { return gm.absolute_mode; }
inline uint8_t cm_get_path_control_mode() { return gm.path_control_mode; }

inline double cm_get_position(uint8_t axis) 
{
	return ((gm.inches_mode == TRUE) ? (gm.position[axis] / MM_PER_INCH) : gm.position[axis]);
}

/*
 * Setters - these inhale gn values into the gm struct
 *
 *	Input coordinates are in native block formats (gn form);
 *	i.e. they are not unit adjusted or otherwise pre-processed.
 *	The setters take care of coordinate system, units, and 
 *	distance mode conversions and normalizations.
 *
 * cm_set_vector()	- utility function to load values into vector form
 * cm_set_offset()	- set all IJK offsets
 * cm_set_radius()	- set radius value
 * cm_set_absolute_override()
 * cm_set_target()	- set all XYZABC targets
 */

double *cm_set_vector(double x, double y, double z, 
					  double a, double b, double c)
{
	vector[X] = x;
	vector[Y] = y;
	vector[Z] = z;
	vector[A] = a;
	vector[B] = b;
	vector[C] = c;
	return (vector);
}

void cm_set_offset(double i, double j, double k) 
{ 
	gm.offset[0] = _to_millimeters(i);
	gm.offset[1] = _to_millimeters(j);
	gm.offset[2] = _to_millimeters(k);
}

void cm_set_radius(double r) 
{ 
	gm.radius = _to_millimeters(r);
}

void cm_set_absolute_override(uint8_t absolute_override) 
{ 
	gm.absolute_override = absolute_override;
}

/* 
 * _cm_set_target() - set target ector in GM model
 *
 * This is a core routine. It handles:
 *	- conversion of linear units to internal canonical form (mm)
 *	- conversion of relative mode to absolute (canonical form)
 *	- computation and application of axis modes as so:
 *
 *		DISABLED
 *		  - Incoming value is ignored. Target value is not changed
 *
 *		ENABLED 
 *		  - Convert axis values to canonical format and store as target
 *
 *		INHIBITED
 *	  	  - Same processing as ENABLED, but axis will not be run
 *
 * 		RADIUS
 *		  - ABC axis value is provided in Gcode block in linear units
 *		  - Target is set to degrees based on axis' Radius value
 *
 *		SLAVE MODES (X, Y, Z, XY, XZ, YZ, XYZ spaces)
 *		  - Axis value is computed from path length of specified space
 *		  - Target is set to degrees based on axis' Radius value
 *		  - Any value input for that axis is ignored 
 *
 *	  Radius and slave modes are only processed for ABC axes.
 *	  Attempts to apply them for XYZ are ignored.
 */

void cm_set_target(double target[])
{ 
	uint8_t i;
	double length;
	double target_tmp = 0;

	// process XYZ
	for (i=0; i<A; i++) {
		switch (cfg.a[i].axis_mode) {
			case (AXIS_DISABLED): {
				continue;				// no updates; ignore the axis.
			}
			case (AXIS_STANDARD): 
			case (AXIS_INHIBITED): {
				if ((gm.absolute_mode == TRUE) || (gm.absolute_override == TRUE)) {
					gm.target[i] = _to_millimeters(target[i]);
				} else {
					gm.target[i] += _to_millimeters(target[i]);
				}
				break;
			}
			TRAP1(PSTR("%c axis using unsupported axis mode"), cfg_get_axis_char(i));
		}
	}
	// process ABC
	for (i=A; i<C; i++) {
		switch (cfg.a[i].axis_mode) {
			case (AXIS_DISABLED): {
				continue;
			}
			case (AXIS_STANDARD): 
			case (AXIS_INHIBITED): {
				target_tmp = target[i];	// degrees - no conversion required
				break;
			}
			case (AXIS_RADIUS): {
				if (gf.target[i] == TRUE) {	// don't process unchanged axes
					target_tmp = _to_millimeters(target[i]) * 360 / (2 * M_PI * cfg.a[i].radius);
					break;
				}
			}
			case (AXIS_SLAVE_X): {
				length = (target[X] - gm.position[X]);
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
				break;
			}
			case (AXIS_SLAVE_Y): {
				length = (target[Y] - gm.position[Y]);
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
				break;
			}
			case (AXIS_SLAVE_Z): {
				length = (target[Z] - gm.position[Z]);
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
				break;
			}
			case (AXIS_SLAVE_XY): {
				length = sqrt(square(target[X] - gm.position[X]) + 
							  square(target[Y] - gm.position[Y]));
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
				break;
			}
			case (AXIS_SLAVE_XZ): {
				length = sqrt(square(target[X] - gm.position[X]) + 
							  square(target[Z] - gm.position[Z]));
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
				break;
			}
			case (AXIS_SLAVE_YZ): {
				length = sqrt(square(target[Y] - gm.position[Y]) + 
							  square(target[Z] - gm.position[Z]));
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
			}
			case (AXIS_SLAVE_XYZ): {
				length = sqrt(square(target[X] - gm.position[X]) + 
							  square(target[Y] - gm.position[Y]) +
							  square(target[Z] - gm.position[Z]));
				target_tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
			}
		}
		if ((gm.absolute_mode == TRUE) || (gm.absolute_override == TRUE)) {
			gm.target[i] = target_tmp;
		} else {
			gm.target[i] += target_tmp;
		}
	}
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

static void _cm_set_endpoint_position(uint8_t status) 
{ 
	if ((status == TG_OK) || (status == TG_EAGAIN)) {
		mp_copy_vector(gm.position, gm.target);
	}
}

/* 
 * _cm_get_move_time() - get required time for move
 *
 * Compute the optimum time for the move. This will either be the 
 * length / rate (feedrate or seekrate), or just time specified by inverse
 * feed rate if G93 is active. Then test the move against the axis max 
 * feed rates and increase the time to accommodate the rate limiting axis. 
 * The axis modes are taken into account. See gcode.h for mode details
 *
 * The following times are compared, and the longest is returned:
 *		G93 inverse time (if G93 is active)
 *		time for coordinated move at requested feed rate
 *		time that the slowest axis would require for the move
 */

static double _cm_get_move_time()
{
	uint8_t i;
	double inv_time=0;	//inverse time if doing a feed in G93 mode
	double xyz_time=0;	// coordinated move linear part at req feed rate
	double abc_time=0;	// coordinated move rotary part at req feed rate
	double axis_time;	// time for axis to complete at its max feed rate
	double max_time=0;	// time required for the rate-limiting axis

	if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
		if (gm.inverse_feed_rate_mode == TRUE) {
			inv_time = gm.inverse_feed_rate;
		} else {
			xyz_time = sqrt(square(gm.target[X] - gm.position[X]) + // in mm
							square(gm.target[Y] - gm.position[Y]) +
							square(gm.target[Z] - gm.position[Z])) 
							/ gm.feed_rate;

			abc_time = sqrt(square(gm.target[A] - gm.position[A]) + // in deg
							square(gm.target[B] - gm.position[B]) +
							square(gm.target[C] - gm.position[C])) 
							/ gm.feed_rate;
		}
	}
 	for (i=0; i<AXES; i++) {
		if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
			if ((axis_time = fabs(gm.target[i] - gm.position[i]) / 
				cfg.a[i].feed_rate_max) > max_time) {
				max_time = axis_time;
			}
		} else {
			if ((axis_time = fabs(gm.target[i] - gm.position[i]) / 
				cfg.a[i].seek_rate_max) > max_time) {
				max_time = axis_time;
			}
		}
	}
	// 4-way max using ternary operations! ugh.
	return (max (max(inv_time, max_time), max(xyz_time, abc_time)));
}

/*************************************************************************
 *
 * CANONICAL MACHINING FUNCTIONS
 *
 *	Values are passed in pre-unit_converted state
 *	All operations occur on gm (current model state)
 *
 ************************************************************************/

/* 
 * Initialization and Termination (4.3.2)
 *
 * cm_init_canon() 
 *
 *	Most of canonical machine initialization is done thru the config system
 */

void cm_init_canon()
{
	ZERO_MODEL_STATE(&gm);
	ZERO_MODEL_STATE(&gt);
	cfg_init_gcode_model();			// set all the gcode defaults
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
	gm.set_plane = plane;
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

uint8_t cm_set_origin_offsets(double offset[])
{
	gm.position[X] = _to_millimeters(offset[X]);
	gm.position[Y] = _to_millimeters(offset[Y]);
	gm.position[Z] = _to_millimeters(offset[Z]);
	gm.position[A] = offset[A];	// in degrees
	gm.position[B] = offset[B];	// in degrees
	gm.position[C] = offset[C];	// in degrees

	// make the lower layer agree with this
	(void)mp_set_position(gm.position);
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

uint8_t cm_straight_traverse(double target[])
{
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_target(target);
	cm_status = MP_LINE(gm.target, _cm_get_move_time());
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
	if (gm.inverse_feed_rate_mode == TRUE) {
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
	gm.path_control_mode = mode;
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
	(void)mp_dwell(seconds);
	return (TG_OK);
}

uint8_t cm_straight_feed(double target[])
{
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == FALSE) && (gm.feed_rate == 0)) {
		TRAP1(PSTR("Attempted move %s with feed rate = zero"), tg.buf);
		cm_status = TG_ZERO_LENGTH_MOVE;
		return (cm_status);
	}
	cm_set_target(target);
	cm_status = MP_LINE(gm.target, _cm_get_move_time());
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
 * someone can tell me who uses this and for what, and how it's invoked 
 * - given that there is no corresponding gcode for it.
 */

uint8_t cm_program_stop()			// M0, M60
{
	mp_queued_stop();
	return (TG_OK);
}

uint8_t cm_optional_program_stop()	// M1
{
	mp_queued_stop();
	return (TG_OK);
}

uint8_t cm_program_end()			// M2, M30
{
	tg_reset_source();	// stop reading from a file (return to std device)
	mp_queued_end();
	return (TG_OK);
}

uint8_t cm_async_stop()
{
	mp_async_stop();
	return (TG_OK);
}

uint8_t cm_async_start()
{
	mp_async_start();
	return (TG_OK);
}

uint8_t cm_async_end()
{
	mp_async_end();
	return (TG_OK);
}


/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _cm_compute_radius_arc() - compute arc center (offset) from radius.
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 *
 */

uint8_t cm_arc_feed(double target[],				// arc endpoints
					double i, double j, double k, 	// offsets
					double radius, 			// non-zero sets radius mode
					uint8_t motion_mode)	// defined motion mode
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = motion_mode;
	cm_set_target(target);
	cm_set_offset(i, j, k);
	cm_set_radius(radius);
	cm_status = TG_OK;

	// execute the move - non-zero radius is a radius arc
	if (radius > 0) {
		if ((_cm_compute_radius_arc() != TG_OK)) {
			return (cm_status);				// error return
		}
	}
	cm_status = _cm_compute_center_arc();
	_cm_set_endpoint_position(cm_status);
	return (cm_status);
}

/* _cm_compute_radius_arc() - compute arc center (offset) from radius. */

uint8_t _cm_compute_radius_arc()
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

//	clear_vector(&gm.offset);
	gm.offset[0] = 0;	// reset the offsets
	gm.offset[1] = 0;
	gm.offset[2] = 0;

	// == -(h * 2 / d)
//	h_x2_div_d = -sqrt(4 * gm.radius*gm.radius - ((x*x) - (y*y))) / hypot(x,y);
	h_x2_div_d = -sqrt(4 * square(gm.radius) - (square(x) - square(y))) / hypot(x,y);

	// If r is smaller than d the arc is now traversing the complex plane beyond
	// the reach of any real CNC, and thus - for practical reasons - we will 
	// terminate promptly (well spoken Simen!)
	if(isnan(h_x2_div_d) == TRUE) { 
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
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 */

uint8_t _cm_compute_center_arc()
{
	double theta_start;
	double theta_end;
	double radius_tmp;
	double angular_travel;
	double linear_travel;
//	double mm_of_travel;
	double move_time;

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
	theta_start = _theta(
		gm.position[gm.plane_axis_0] - gm.offset[gm.plane_axis_0], 
		gm.position[gm.plane_axis_1] - gm.offset[gm.plane_axis_1]);

	if(isnan(theta_start) == TRUE) { 
		cm_status = TG_ARC_SPECIFICATION_ERROR;
		return(cm_status); 
	}

	// calculate the theta (angle) of the target point
	theta_end = _theta(
		gm.target[gm.plane_axis_0] - gm.offset[gm.plane_axis_0] - gm.position[gm.plane_axis_0], 
 		gm.target[gm.plane_axis_1] - gm.offset[gm.plane_axis_1] - gm.position[gm.plane_axis_1]);

	if(isnan(theta_end) == TRUE) { 
		cm_status = TG_ARC_SPECIFICATION_ERROR; 
		return(cm_status);
	}

	// ensure that the difference is positive so we have clockwise travel
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
	linear_travel = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];

	// compute the time it should take to perform the move
	move_time = _cm_get_arc_time(linear_travel, angular_travel, radius_tmp);

	// Trace the arc
	cm_set_vector(	gm.target[gm.plane_axis_0],
					gm.target[gm.plane_axis_1],
					gm.target[gm.plane_axis_2],
					gm.target[A], gm.target[B], gm.target[C]);

	cm_status = mp_arc(	vector,
						gm.offset[gm.plane_axis_0],
						gm.offset[gm.plane_axis_1],
						gm.offset[gm.plane_axis_2],
						theta_start, radius_tmp, 
						angular_travel, linear_travel, 
					   	gm.plane_axis_0, gm.plane_axis_1, gm.plane_axis_2,
						move_time);

    // Finish off with a line to make sure we arrive exactly where we think we are
	//--> For this to work correctly it must be delivered ONLY after the arc generator 
	// has completed the arc. So the endpoint should be passed to the generator and
	// executed there.
//	cm_status = mp_line(gp.target[X_AXIS], gp.target[Y_AXIS], gp.target[Z_AXIS], 
//					   (gp.inverse_feed_rate_mode) ? gp.inverse_feed_rate : 
//						gp.feed_rate, gp.inverse_feed_rate_mode);
	return (cm_status);
}

/* 
 * _cm_get_arc_time ()
 */

static double _cm_get_arc_time (const double linear_travel, 	// in mm
								const double angular_travel, 	// in radians
								const double radius)			// in mm
{
	double tmp;
	double move_time=0;	// picks through the times and retains the slowest
	double planar_travel = fabs(angular_travel * radius);// travel in arc plane

	if (gm.inverse_feed_rate_mode == TRUE) {
		move_time = gm.inverse_feed_rate;
	} else {
		move_time = sqrt(square(planar_travel) + square(linear_travel)) / gm.feed_rate;
	}
	if ((tmp = planar_travel/cfg.a[gm.plane_axis_0].feed_rate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = planar_travel/cfg.a[gm.plane_axis_1].feed_rate_max) > move_time) {
		move_time = tmp;
	}
	if ((tmp = fabs(linear_travel/cfg.a[gm.plane_axis_2].feed_rate_max)) > move_time) {
		move_time = tmp;
	}
	return (move_time);
}

/* 
 * _theta(double x, double y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static double _theta(const double x, const double y)
{
	double theta = atan(x/fabs(y));

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
 * cm_print_machine_state()
 */
// these must line up with the memory string indexes below
#define GC_MSG_UNITS 00 
#define GC_MSG_MOTION 02	
#define GC_MSG_PLANE 07
#define GC_MSG_DISTANCE 10
#define GC_MSG_FEEDRATEMODE 12
#define GC_MSG_STOP 14

// put display strings in program memory
static char gms00[] PROGMEM = "Units:           G21 - millimeter mode\n";	// This pair is inverted
static char gms01[] PROGMEM = "Units:           G20 - inches mode\n";
static char gms02[] PROGMEM = "Motion mode:     G0  - linear traverse (seek)\n";
static char gms03[] PROGMEM = "Motion mode:     G1  - linear feed\n";
static char gms04[] PROGMEM = "Motion mode:     G2  - clockwise arc feed\n";
static char gms05[] PROGMEM = "Motion mode:     G3  - counter clockwise arc feed\n";
static char gms06[] PROGMEM = "Motion mode:     G80 - cancel motion mode (none active)\n";
static char gms07[] PROGMEM = "Plane selection: G17 - XY plane\n";
static char gms08[] PROGMEM = "Plane selection: G18 - XZ plane\n";
static char gms09[] PROGMEM = "Plane selection: G19 - YZ plane\n";
static char gms10[] PROGMEM = "Distance mode:   G91 - incremental distance\n";// This pair is inverted
static char gms11[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
static char gms12[] PROGMEM = "Feed rate mode:  G94 - units per minute\n";	// This pair is inverted
static char gms13[] PROGMEM = "Feed rate mode:  G93 - inverse time\n";
static char gms14[] PROGMEM = "Run state:       Running\n";
static char gms15[] PROGMEM = "Run state:       Stopped (M0,M1,M30, Pause)\n";
static char gms16[] PROGMEM = "Run state:       Ended (M2,M60)\n";

static char gmsPosX[] PROGMEM = "Position X:   %8.3f %s\n";
static char gmsPosY[] PROGMEM = "Position Y:   %8.3f %s\n";
static char gmsPosZ[] PROGMEM = "Position Z:   %8.3f %s\n";
static char gmsPosA[] PROGMEM = "Position A:   %8.3f degrees\n";
static char gmsPosB[] PROGMEM = "Position B:   %8.3f degrees\n";
static char gmsPosC[] PROGMEM = "Position C:   %8.3f degrees\n";
static char gmsOfsI[] PROGMEM = "Offset I:     %8.3f %s\n";
static char gmsOfsJ[] PROGMEM = "Offset J:     %8.3f %s\n";
static char gmsOfsK[] PROGMEM = "Offset K:     %8.3f %s\n";
static char gmsFeed[] PROGMEM = "Feed Rate:    %8.3f %s \\ min\n";
//static char gmsLimit[] PROGMEM = "Limit Switches: %3.0f %s\n";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
static PGM_P gcMsg[] PROGMEM = {	
	gms00, gms01, gms02, gms03, gms04, gms05, gms06, gms07, gms08, gms09,
	gms10, gms11, gms12, gms13, gms14, gms15, gms16
};

void cm_print_machine_state()
{
	char units[8] = "mm";

	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.inches_mode + GC_MSG_UNITS)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.motion_mode + GC_MSG_MOTION)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.set_plane + GC_MSG_PLANE)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.absolute_mode + GC_MSG_DISTANCE)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.inverse_feed_rate_mode + GC_MSG_FEEDRATEMODE)]));

	if (gm.inches_mode == TRUE) {
		strncpy(units,"inches", 8);// unnecessary, but useful to know about
		fprintf_P(stderr, (PGM_P)gmsFeed, gm.feed_rate / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosX, gm.position[X] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosY, gm.position[Y] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosZ, gm.position[Z] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosA, gm.position[A]);
		fprintf_P(stderr, (PGM_P)gmsPosB, gm.position[B]);
		fprintf_P(stderr, (PGM_P)gmsPosC, gm.position[C]);
		fprintf_P(stderr, (PGM_P)gmsOfsI, gm.offset[0] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsOfsJ, gm.offset[1] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsOfsK, gm.offset[2] / (25.4), units);
	} else {
		fprintf_P(stderr, (PGM_P)gmsFeed, gm.feed_rate, units);
		fprintf_P(stderr, (PGM_P)gmsPosX, gm.position[X], units);
		fprintf_P(stderr, (PGM_P)gmsPosY, gm.position[Y], units);
		fprintf_P(stderr, (PGM_P)gmsPosZ, gm.position[Z], units);
		fprintf_P(stderr, (PGM_P)gmsPosA, gm.position[A]);
		fprintf_P(stderr, (PGM_P)gmsPosB, gm.position[B]);
		fprintf_P(stderr, (PGM_P)gmsPosC, gm.position[C]);
		fprintf_P(stderr, (PGM_P)gmsOfsI, gm.offset[0], units);
		fprintf_P(stderr, (PGM_P)gmsOfsJ, gm.offset[1], units);
		fprintf_P(stderr, (PGM_P)gmsOfsK, gm.offset[2], units);
	}
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.program_flow + GC_MSG_STOP)]));
//	fprintf_P(stderr, (PGM_P)gmsLimit, ls.min[X]);
}


/***********************************************************************/
/*--- CANONICAL MACHINING CYCLES ---*/

uint8_t cm_stop()					// stop cycle. not implemented
{
	return (TG_OK);
}


