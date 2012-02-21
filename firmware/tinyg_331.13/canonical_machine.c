/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S Hart, Jr.
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* The canonical machine is the layer between the Gcode parser and the
 * motion control code for a specific robot. It keeps state and executes
 * commands - passing the stateless commands to the motion control layer. 
 */
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>				// needed for memcpy, memset
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "gpio.h"

// NOTE: The canonical machine singleton "cm" would normally be declared here 
// but it's also used by cycles so it's in canonical_machine.h instead.

static double _get_move_time();
static void _set_gcode_model_endpoint_position(uint8_t status);
static uint8_t _compute_center_arc(void);
static uint8_t _get_arc_radius(void);
static double _get_arc_time (const double linear_travel, const double angular_travel, const double radius);
static double _get_theta(const double x, const double y);

#define _to_millimeters(a) ((gm.units_mode == INCHES) ? (a * MM_PER_INCH) : a)

/*************************************************************************
 *
 * HELPERS AND UTILITY FUNCTIONS
 *
 *	These functions are not part of the NIST defined functions
 *
 ************************************************************************/

/*
 * Simple Getters
 *
 * cm_get_next_action() - get next_action from the gm struct
 * cm_get_motion_mode() - get motion mode from the gm struct
 * cm_get_units_mode() - get units mode from the gm struct
 * cm_get_path_control() - get path control mode from gm struct
 * cm_get_distance_mode() - get distance mode from the gm struct
 * cm_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
 */

uint8_t cm_get_machine_state() { return cm.machine_state; }
uint8_t cm_get_motion_mode() { return gm.motion_mode; }
uint8_t cm_get_coord_system() { return gm.coord_system; }
uint8_t cm_get_units_mode() { return gm.units_mode; }
uint8_t cm_get_select_plane() { return gm.select_plane; }
uint8_t cm_get_path_control() { return gm.path_control; }
uint8_t cm_get_distance_mode() { return gm.distance_mode; }
uint32_t cm_get_linenum() { return gm.linenum; }
uint8_t cm_isbusy() { return (mp_isbusy());}

/* Position and Offset getters
 *
 * cm_get_coord_offset() - return the currently active coordinate offset for an axis
 * cm_get_model_work_position() - return position from the gm struct into gn struct form (external form)
 * cm_get_model_work_position_vector() - return model position vector in externalized form
 * cm_get_model_canonical_position_vector() - return model position vector in internal canonical form
 * cm_get_runtime_machine_position() - return current machine position in external form 
 * cm_get_runtime runtome_position() - return current work coordinate position in external form 
 */

double cm_get_coord_offset(uint8_t axis)
{
	if (gm.origin_offset_mode == true) {
		return (cfg.offset[gm.coord_system][axis] + gm.origin_offset[axis]);
	} else {
		return (cfg.offset[gm.coord_system][axis]);
	}
}

double cm_get_model_work_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return ((gm.position[axis] - cm_get_coord_offset(axis)) / MM_PER_INCH);
	} else {
		return (gm.position[axis] - cm_get_coord_offset(axis));
	}
}

double *cm_get_model_work_position_vector(double position[]) 
{
	for (uint8_t i=0; i<AXES; i++) {
		position[i] = cm_get_model_work_position(i);
	}
	return (position);
}

double *cm_get_model_canonical_position_vector(double position[])
{
	copy_axis_vector(position, gm.position);	
	return (position);
}

double cm_get_runtime_machine_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return (mp_get_runtime_position(axis) / MM_PER_INCH);
	} else {
		return (mp_get_runtime_position(axis));
	}
}

double cm_get_runtime_work_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return ((mp_get_runtime_position(axis) - cm_get_coord_offset(axis)) / MM_PER_INCH);
	} else {
		return (mp_get_runtime_position(axis) - cm_get_coord_offset(axis));
	}
}

/*
 * Setters - these inhale gn values into the gm struct
 *
 *	Input coordinates are in native block formats (gn form);
 *	i.e. they are not unit adjusted or otherwise pre-processed.
 *	The setters take care of coordinate system, units, and 
 *	distance mode conversions and normalizations.
 *
 * cm_set_arc_offset()	- set all IJK offsets
 * cm_set_radius()	- set radius value
 * cm_set_absolute_override()
 * cm_set_linenum()
 * cm_set_target()	- set all XYZABC targets
 */

void cm_set_arc_offset(double i, double j, double k)
{ 
	gm.arc_offset[0] = _to_millimeters(i);
	gm.arc_offset[1] = _to_millimeters(j);
	gm.arc_offset[2] = _to_millimeters(k);
}

void cm_set_arc_radius(double r) 
{ 
	gm.arc_radius = _to_millimeters(r);
}

void cm_set_absolute_override(uint8_t absolute_override) 
{ 
	gm.absolute_override = absolute_override;
}

void cm_set_linenum(uint32_t linenum)
{
	if (linenum != 0) {
		gm.linenum = linenum;
	} else {
		gm.linenum++;			// autoincrement if no line number
	}
}

/* 
 * cm_set_target() - set target vector in GM model
 *
 * This is a core routine. It handles:
 *	- conversion of linear units to internal canonical form (mm)
 *	- conversion of relative mode to absolute (internal canonical form)
 *	- translation of work coordinates to machine coordinates (internal canonical form)
 *	- computation and application of axis modes as so:
 *
 *		DISABLED
 *		  - Incoming value is ignored. Target value is not changed
 *
 *		ENABLED 
 *		  - Convert axis values to canonical format and store as target
 *
 *		INHIBITED
 *	  	  - Same processing as ENABLED, but axis will not actually be run
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
 *
 *	Target coordinates are provided in target[]
 *	Axes that need processing are signaled in flag[]
 *	All that flag checking in the slaves traps erroneous rotary inputs
 */

void cm_set_target(double target[], double flag[])
{ 
	uint8_t i;
	double length;
	double tmp = 0;

	// process XYZABC for lower modes
	for (i=X; i<=Z; i++) {
		if ((flag[i] < EPSILON) || (cfg.a[i].axis_mode == AXIS_DISABLED)) {
			continue;
		} else if ((cfg.a[i].axis_mode == AXIS_STANDARD) || (cfg.a[i].axis_mode == AXIS_INHIBITED)) {
			if ((gm.distance_mode == ABSOLUTE_MODE) || (gm.absolute_override == true)) {
				gm.target[i] = cm_get_coord_offset(i) + _to_millimeters(target[i]);
			} else {
				gm.target[i] += _to_millimeters(target[i]);
			}
//		} else if (i<A) {
//			INFO1(PSTR("%c axis using unsupported axis mode"), cfg_get_configuration_group_char(i));
		}
	}
	// FYI: The ABC loop below relies on the XYZ loop having been run first
	for (i=A; i<=C; i++) {
		// skip axis if not flagged for update or its disabled
		if ((flag[i] < EPSILON) || (cfg.a[i].axis_mode == AXIS_DISABLED)) {
			continue;

		} else if ((cfg.a[i].axis_mode == AXIS_STANDARD) || (cfg.a[i].axis_mode == AXIS_INHIBITED)) {
			tmp = target[i];	// no mm conversion - it's in degrees

		} else if ((cfg.a[i].axis_mode == AXIS_RADIUS) && (flag[i] > EPSILON)) {
			tmp = _to_millimeters(target[i]) * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_X) && (flag[X] > EPSILON)) {
			tmp = (target[X] - gm.position[X]) * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_Y) && (flag[Y] > EPSILON)) {
			tmp = (target[Y] - gm.position[Y]) * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_Z) && (flag[Z] > EPSILON)) {
			tmp = (target[Z] - gm.position[Z]) * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XY) && 
			((flag[X] > EPSILON) || (flag[Y] > EPSILON))) {
			length = sqrt(square(target[X] - gm.position[X]) + 
						  square(target[Y] - gm.position[Y]));
			tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XZ) && 
			((flag[X] > EPSILON) || (flag[Z] > EPSILON))) {
			length = sqrt(square(target[X] - gm.position[X]) + 
						  square(target[Z] - gm.position[Z]));
			tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_YZ) && 
			((flag[Y] > EPSILON) || (flag[Z] > EPSILON))) {
			length = sqrt(square(target[Y] - gm.position[Y]) + 
						  square(target[Z] - gm.position[Z]));
			tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

		} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XYZ) && 
			((flag[X] > EPSILON) || (flag[Y] > EPSILON) || (flag[Z] > EPSILON))) {
			length = sqrt(square(target[X] - gm.position[X]) + 
						  square(target[Y] - gm.position[Y]) +
						  square(target[Z] - gm.position[Z]));
			tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
		}
		if ((gm.distance_mode == ABSOLUTE_MODE) || (gm.absolute_override == true)) {
			gm.target[i] = tmp;
		} else {
			gm.target[i] += tmp;
		}
	}
}

/* 
 * _set_gcode_model_endpoint_position() - uses internal coordinates only
 *
 * 	This routine sets the endpoint position in the gccode model if the move
 *	was is successfully completed (no errors). Leaving the endpoint position
 *	alone for errors allows too-short-lines to accumulate into longer lines.
 *
 * 	Note: As far as the canonical machine is concerned the final position 
 *	is achieved as soon at the move is executed and the position is now 
 *	the target. In reality the planner(s) and steppers will still be 
 *	processing the action and the real tool position is still close to 
 *	the starting point. 
 */

static void _set_gcode_model_endpoint_position(uint8_t status) 
{
	if (status == TG_OK) { copy_axis_vector(gm.position, gm.target);}
}

/* 
 * _get_move_time() - get required time for move
 *
 *	Compute the optimum time for the move. This will either be the 
 *	length / rate (feedrate or seekrate), or just time specified by inverse
 *	feed rate if G93 is active. Then test the move against the maximum 
 *	feed or seek rates for each axis in the move and increase the time to 
 *	accommodate the rate limiting axis. Axis modes are taken into account
 *	by having cm_set_target load the targets.
 *
 *	The following times are compared, and the longest is returned:
 *	  -	G93 inverse time (if G93 is active)
 *	  -	time for coordinated move at requested feed rate
 *	  -	time that the slowest axis would require for the move
 */

static double _get_move_time()
{
	uint8_t i;
	double inv_time=0;	// inverse time if doing a feed in G93 mode
	double xyz_time=0;	// coordinated move linear part at req feed rate
	double abc_time=0;	// coordinated move rotary part at req feed rate
	double max_time=0;	// time required for the rate-limiting axis

	// compute times for feed motion
	if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
		if (gm.inverse_feed_rate_mode == true) {
			inv_time = gm.inverse_feed_rate;
		} else {
			xyz_time = sqrt(square(gm.target[X] - gm.position[X]) + // in mm
							square(gm.target[Y] - gm.position[Y]) +
							square(gm.target[Z] - gm.position[Z])) / gm.feed_rate;

			abc_time = sqrt(square(gm.target[A] - gm.position[A]) + // in deg
							square(gm.target[B] - gm.position[B]) +
							square(gm.target[C] - gm.position[C])) / gm.feed_rate;
		}
	}
 	for (i=0; i<AXES; i++) {
		if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) / cfg.a[i].feedrate_max));
		} else { // gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE
			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) / cfg.a[i].velocity_max));
		}
	}
	return (max4(inv_time, max_time, xyz_time, abc_time));
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
 * cm_init() 
 *
 *	The config init (cfg_init() must have been run beforehand. Many parameters 
 *	used by the canonical machine ae actually set during cfg_init().
 */

void cm_init()
{
	memset(&cm, 0, sizeof(cm));			// reset canonicalMachineSingleton
	memset(&gn, 0, sizeof(gn));			// clear all values, pointers and status
	memset(&gf, 0, sizeof(gf));
	memset(&gm, 0, sizeof(gm));

	// set gcode defaults
	cm_set_units_mode(cfg.units_mode);
	cm_set_coord_system(cfg.coord_system);
	cm_set_machine_coords(cfg.offset[gm.coord_system]);
	cm_select_plane(cfg.select_plane);
	cm_set_path_control(cfg.path_control);
	cm_set_distance_mode(cfg.distance_mode);
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
	gm.select_plane = plane;
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

uint8_t cm_set_origin_offsets(double offset[], double flag[])	// G92
{
	gm.origin_offset_mode = true;
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {	 	// behaves according to NIST 3.5.18
			gm.origin_offset[i] = gm.position[i] 
								  - cfg.offset[gm.coord_system][i] 
								  - _to_millimeters(offset[i]);
		}
	}
	return (TG_OK);
}

uint8_t cm_reset_origin_offsets() 		// G92.1
{
	gm.origin_offset_mode = false;
	for (uint8_t i=0; i<AXES; i++) {
		gm.origin_offset[i] = 0;
	}
	return (TG_OK);
}

uint8_t cm_suspend_origin_offsets()		// G92.2
{
	gm.origin_offset_mode = true;
	return (TG_OK);
}

uint8_t cm_resume_origin_offsets()		// G92.3
{
	gm.origin_offset_mode = false;
	return (TG_OK);
}

uint8_t cm_set_machine_coords(double offset[])
{
	copy_axis_vector(gm.position, offset);
	copy_axis_vector(gm.target, gm.position);
	mp_set_axis_position(gm.position);
	return (TG_OK);
}

uint8_t	cm_set_coord_system(uint8_t coord_system)
{
	gm.coord_system = coord_system;	
	return (TG_OK);
}

uint8_t	cm_set_coord_offsets(uint8_t coord_system, double offset[], double flag[])
{
	if ((coord_system < 1) || (coord_system > COORD_SYSTEM_MAX)) { // you can't set G53
		return (TG_RANGE_ERROR);
	}
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {
			cfg.offset[coord_system][i] = offset[i];
			cmd_persist_offset(coord_system, i, offset[i]); // persist to NVM
		}
	}
	return (TG_OK);
}

uint8_t cm_set_units_mode(uint8_t mode)
{
	gm.units_mode = mode;		// 0 = inches, 1 = mm.
	return (TG_OK);
}

uint8_t cm_set_distance_mode(uint8_t mode)
{
	gm.distance_mode = mode;	// 0 = absolute mode, 1 = incremental
	return (TG_OK);
}

/* 
 * Free Space Motion (4.3.4)
 *
 * cm_straight_traverse() - G0 linear seek
 */

uint8_t cm_straight_traverse(double target[], double flags[])
{
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_target(target, flags);
	cm_cycle_start();							//required for homing & other cycles
	uint8_t status = MP_LINE(gm.target, _get_move_time());
	_set_gcode_model_endpoint_position(status);
	return (status);
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
	if (gm.inverse_feed_rate_mode == true) {
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
 * cm_set_path_control() - G61, G61.1, G64
 */

uint8_t cm_set_path_control(uint8_t mode)
{
	gm.path_control = mode;
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

uint8_t cm_straight_feed(double target[], double flags[])
{
	uint8_t status;

	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == false) && (gm.feed_rate == 0)) {
		return (TG_GCODE_FEEDRATE_ERROR);
	}

	// Introduce a short dwell if the machine is not busy to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
//	if (st_isbusy() == false) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}

	cm_set_target(target, flags);
	cm_cycle_start();							//required for homing & other cycles
	status = MP_LINE(gm.target, _get_move_time());
	_set_gcode_model_endpoint_position(status);
	return (status);
}

/* 
 * Spindle Functions (4.3.7)
 *
 * cm_set_spindle_speed() - S parameter
 * cm_start_spindle_clockwise() - M3
 * cm_start_spindle_counterclockwise() - M4
 * cm_stop_spindle_turning() - M5
 * cm_spindle_control() - integrated spindle control command
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

uint8_t cm_spindle_control(uint8_t spindle_mode)
{
	gm.spindle_mode = spindle_mode;
 	if (spindle_mode == SPINDLE_CW) {
		return (cm_start_spindle_clockwise());
	} else if (spindle_mode == SPINDLE_CCW) {
		return (cm_start_spindle_counterclockwise());
	}
	(void)cm_stop_spindle_turning();	// failsafe: any error causes stop
	return (TG_INTERNAL_ERROR);
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

void cm_comment(char *comment)
{
	return;
//	return (TG_OK);		// no operation
}

void cm_message(char *message)
{
	printf_P(PSTR("%s\n"), message);
//	return (TG_OK);
}

/*
 * Program Functions (4.3.10)
 *
 * This group implements stop, start, end, and hold. 
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	cm_cycle_start()			(no Gcode)
 *	cm_program_stop()			(M0, M60)
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	cm_feedhold()				(no Gcode)
 *
 * cm_program_stop and cm_optional_program_stop are synchronous Gcode 
 * commands that are received through the interpreter. They cause all motion
 * to stop at the end of the current command, including spindle motion. 
 * Note that the stop occurs at the end of the immediately preceding command
 * (i.e. the stop is queued behind the last command).
 *
 * cm_program_end is a stop that also resets the machine to initial state
 *
 * See planner.c for feedhold details.
 */

void cm_cycle_start()
{
	// the machine state model simplifies to this:
	if (cm.machine_state == MACHINE_HOLD) {
		cm.machine_state = MACHINE_END_HOLD;
	} else {
		cm.machine_state = MACHINE_RUN;
	}
/*
//	Here's more readable but slightly less efficient code:
	switch (cm.machine_state) {
		case (MACHINE_RESET):	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_RUN): 	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_STOP):  	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_HOLD):	{ cm.machine_state = MACHINE_END_HOLD; break;}
		case (MACHINE_END_HOLD):{ cm.machine_state = MACHINE_RUN; break;}
	}
*/
}

void cm_program_stop()			// M0, M60
{
	mp_queue_program_stop();		// insert a prpgram stop in the queue
}

void cm_optional_program_stop()	// M1
{
	mp_queue_program_stop();		// insert a prpgram stop in the queue
}

void cm_program_end()			// M2, M30
{
	tg_reset_source();				// stop reading from a file (return to std device)
	mp_queue_program_end();			// insert a prpgram stop in the queue
}

void cm_feedhold()
{
	if ((cm.machine_state == MACHINE_RUN) && (cm.hold_state == FEEDHOLD_OFF)) {
		cm.machine_state = MACHINE_HOLD;
		cm.hold_state = FEEDHOLD_SYNC;
	}
}

void cm_exec_stop() 
{
	cm.machine_state = MACHINE_STOP;// machine is stopped...
	cm.hold_state = FEEDHOLD_OFF;	//...and any feedhold is ended
}

void cm_exec_end() 
{
	cm.machine_state = MACHINE_RESET;
	cm.hold_state = FEEDHOLD_OFF;	// end any residual feedhold
}

/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 * _cm_get_arc_radius() 	- compute arc center (offset) from radius.
 * _cm_get_arc_time()
 */

uint8_t cm_arc_feed(double target[], double flags[],// arc endpoints
					double i, double j, double k, 	// offsets
					double radius, 					// non-zero sets radius mode
					uint8_t motion_mode)			// defined motion mode
{
	uint8_t status = TG_OK;

	// copy parameters into the current state
	gm.motion_mode = motion_mode;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == false) && (gm.feed_rate == 0)) {
		return (TG_GCODE_FEEDRATE_ERROR);
	}

	// set parameters
	cm_set_target(target, flags);
	cm_set_arc_offset(i,j,k);
	cm_set_arc_radius(radius);

	// execute the move - non-zero radius is a radius arc
	if (radius > EPSILON) {
		if ((_get_arc_radius() != TG_OK)) {
			return (status);					// error return
		}
	}
	// Introduce a short dwell if the machine is idle to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
//	if (st_isbusy() == false) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}
	status = _compute_center_arc();
	_set_gcode_model_endpoint_position(status);
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

uint8_t _compute_center_arc()
{
	double theta_start;
	double theta_end;
	double radius_tmp;
	double angular_travel;
	double linear_travel;
	double move_time;

	// calculate the theta (angle) of the current point (see header notes)
	theta_start = _get_theta(-gm.arc_offset[gm.plane_axis_0], -gm.arc_offset[gm.plane_axis_1]);
	if(isnan(theta_start) == true) { return(TG_ARC_SPECIFICATION_ERROR);}

	// calculate the theta (angle) of the target point
	theta_end = _get_theta(
		gm.target[gm.plane_axis_0] - gm.arc_offset[gm.plane_axis_0] - gm.position[gm.plane_axis_0], 
 		gm.target[gm.plane_axis_1] - gm.arc_offset[gm.plane_axis_1] - gm.position[gm.plane_axis_1]);
	if(isnan(theta_end) == true) { return (TG_ARC_SPECIFICATION_ERROR); }

	// ensure that the difference is positive so we have clockwise travel
	if (theta_end < theta_start) { theta_end += 2*M_PI; }

	// compute angular travel and invert if gcode wants a counterclockwise arc
	angular_travel = theta_end - theta_start;
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) { angular_travel -= 2*M_PI;}

	// Find the radius, calculate travel in the depth axis of the helix,
	// and compute the time it should take to perform the move
	radius_tmp = hypot(gm.arc_offset[gm.plane_axis_0], gm.arc_offset[gm.plane_axis_1]);
	linear_travel = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];
	move_time = _get_arc_time(linear_travel, angular_travel, radius_tmp);

	// Trace the arc
	set_vector(gm.target[gm.plane_axis_0], gm.target[gm.plane_axis_1], gm.target[gm.plane_axis_2],
			   gm.target[A], gm.target[B], gm.target[C]);

	return(ar_arc(vector,
				  gm.arc_offset[gm.plane_axis_0],
				  gm.arc_offset[gm.plane_axis_1],
				  gm.arc_offset[gm.plane_axis_2],
				  theta_start, radius_tmp, angular_travel, linear_travel, 
				  gm.plane_axis_0, gm.plane_axis_1, gm.plane_axis_2, move_time));
}

/* 
 * _cm_get_arc_radius() - compute arc center (offset) from radius. 
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

uint8_t _get_arc_radius()
{
	double x;
	double y;
	double h_x2_div_d;

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
	if(isnan(h_x2_div_d) == true) { return (TG_FLOATING_POINT_ERROR);}

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
	return (TG_OK);
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

static double _get_arc_time (const double linear_travel, 	// in mm
								const double angular_travel, 	// in radians
								const double radius)			// in mm
{
	double tmp;
	double move_time=0;	// picks through the times and retains the slowest
	double planar_travel = fabs(angular_travel * radius);// travel in arc plane

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
 * _get_theta(double x, double y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static double _get_theta(const double x, const double y)
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

