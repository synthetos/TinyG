/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S Hart, Jr.
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
/* --- Synchronous and immediate commands ---
 *
 *	Some commands in the canonical machine need to be executed immediately and 
 *	some need to be synchronized with movement (the planner queue). In general,
 *	commands that only affect the gcode model are done immediately whereas 
 *	commands that have a physcial effect must be synchronized.
 *
 *	Immediate commands are obvious - just write to the GM struct. Synchronous
 *	commands work like this:
 *
 *	  - Call the cm_xxx_xxx() function which will do any input validation and 
 *		return an error if it detects one.
 *
 *	  - The cm_ function calls mp_queue_command(). Arguments are a callback to
 *		the _exec_...() function, which is the runtime execution routine, and
 *		any arguments that rae needed by the runtime. See typedef for *exec in
 *		planner.h for details
 *
 *	  - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *	  - When planner execution reaches the buffer is tectures the callback w/ the 
 *		args.  Take careful note that the callback executes under an interrupt, 
 *		so beware of variables that may need to be Volatile.
 *
 *	For a list of the synchronous commands see the static function profotoypes
 *	for the planner queue callbacks. Some other notes:
 *
 *	  - All getters are immediate. These just return values from the Gcode model (gm).
 *
 *	  - Commands that are used to set the gm model state for interpretation of the
 *		current Gcode block. For example, cm_set_feed_rate(). This sets the model
 *		so the move time is properly calculated for the current (and subsequent) 
 *		blocks, so it's effected immediately. Note that the "feed rate" (actually 
 *		move time) is carried forward into the planner - planned moves are not 
 *		affected by upstream changes to the gm model. Many other vars also fall into
 *		this category.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>				// needed for memcpy, memset
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "arc.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "report.h"
#include "gpio.h"
#include "system.h"

// NOTE: The canonical machine singleton "cm" would normally be declared here 
// but it's also used by cycles so it's in canonical_machine.h instead.

static double _get_move_times(double *min_time);

// planner queue callbacks
static void _exec_set_coord_offsets(uint8_t coord_system, double d, double offset[], double flag[]);
static void _exec_set_origin_offsets(uint8_t u, double d, double offset[], double flag[]);
static void _exec_modify_origin_offsets(uint8_t dot, double d, double o[], double f[]);
static void _exec_set_machine_axis_position(uint8_t axis, double position, double o[], double f[]);
static void _exec_select_plane(uint8_t plane, double d, double o[], double f[]);
static void _exec_set_units_mode(uint8_t mode, double d, double o[], double f[]);
static void _exec_set_distance_mode(uint8_t mode, double d, double o[], double f[]);
static void _exec_set_coord_system(uint8_t coord_system, double d, double o[], double f[]);
static void _exec_set_feed_rate(uint8_t u, double feed_rate, double o[], double f[]);
static void _exec_set_inverse_feed_rate_mode(uint8_t mode, double d, double o[], double f[]);
static void _exec_change_tool(uint8_t tool, double d, double o[], double f[]);
static void _exec_select_tool(uint8_t tool, double d, double o[], double f[]);
static void _exec_mist_coolant_control(uint8_t mist_coolant, double d, double o[], double f[]);
static void _exec_flood_coolant_control(uint8_t flood_coolant, double d, double o[], double f[]);


static void _exec_program_finalize(uint8_t machine_state);

#define _to_millimeters(a) ((gm.units_mode == INCHES) ? (a * MM_PER_INCH) : a)

/*************************************************************************
 *
 * HELPERS AND UTILITY FUNCTIONS
 *
 *	These functions are not part of the NIST defined functions
 *
 ************************************************************************/

/*
 * Low-level Getters and Setters (work directly on the Gcode model struct)
 */
// get parameter from cm struct
uint8_t cm_get_machine_state() { return cm.machine_state;}
uint8_t cm_get_cycle_state() { return cm.cycle_state;}
uint8_t cm_get_motion_state() { return cm.motion_state;}
uint8_t cm_get_hold_state() { return cm.hold_state;}
uint8_t cm_get_homing_state() { return cm.homing_state;}

// get parameter from gm struct
uint8_t cm_get_motion_mode() { return gm.motion_mode;}
uint8_t cm_get_coord_system() { return gm.coord_system;}
uint8_t cm_get_units_mode() { return gm.units_mode;}
uint8_t cm_get_select_plane() { return gm.select_plane;}
uint8_t cm_get_path_control() { return gm.path_control;}
uint8_t cm_get_distance_mode() { return gm.distance_mode;}
uint8_t cm_get_inverse_feed_rate_mode() { return gm.inverse_feed_rate_mode;}
uint8_t cm_get_spindle_mode() { return gm.spindle_mode;} 
uint32_t cm_get_model_linenum() { return gm.linenum;}
uint8_t cm_isbusy() { return (mp_isbusy());}

// set parameters in gm struct

void cm_set_absolute_override(uint8_t absolute_override) { gm.absolute_override = absolute_override;}
void cm_set_spindle_mode(uint8_t spindle_mode) { gm.spindle_mode = spindle_mode;} 
void cm_set_tool_number(uint8_t tool) { gm.tool = tool;}
void cm_set_spindle_speed_parameter(double speed) { gm.spindle_speed = speed;}

//void cm_sync_tool_number(uint8_t tool) { mp_sync_command(SYNC_TOOL_NUMBER, (double)tool);}
//void cm_sync_spindle_speed_parameter(double speed) { mp_sync_command(SYNC_SPINDLE_SPEED, speed);}

/* 
 * cm_get_combined_state() - combines raw states into something a user might want to see
 */

uint8_t cm_get_combined_state() 
{
	if (cm.machine_state == MACHINE_CYCLE) {
		if (cm.motion_state == MOTION_RUN) cm.combined_state = COMBINED_RUN;
		if (cm.motion_state == MOTION_HOLD) cm.combined_state = COMBINED_HOLD;
		if (cm.cycle_state == CYCLE_HOMING) cm.combined_state = COMBINED_HOMING;
		if (cm.cycle_state == CYCLE_PROBE) cm.combined_state = COMBINED_PROBE;
		if (cm.cycle_state == CYCLE_JOG) cm.combined_state = COMBINED_JOG;
	} else {
		cm.combined_state = cm.machine_state;
	}
	return cm.combined_state;
}

/* Position and Offset getters
 *
 * cm_get_coord_offset() - return the currently active coordinate offset for an axis
 * cm_get_model_work_position() - return position from the gm struct into gn struct form (external form)
 * cm_get_model_work_position_vector() - return model position vector in externalized form
 * cm_get_model_canonical_target() - return model target in internal canonical form
 * cm_get_model_canonical_position_vector() - return model position vector in internal canonical form
 * cm_get_runtime_machine_position() - return current machine position in external form 
 * cm_get_runtime runtome_position() - return current work coordinate position in external form 
 */

double cm_get_coord_offset(uint8_t axis)
{
	if (gm.absolute_override == true) {
		return (0);						// no work offset if in abs override mode
	}
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
/*
double *cm_get_model_work_position_vector(double position[]) 
{
	for (uint8_t i=0; i<AXES; i++) {
		position[i] = cm_get_model_work_position(i);
	}
	return (position);
}
*/
double cm_get_model_canonical_target(uint8_t axis) 
{
	return (gm.target[axis]);
}

double *cm_get_model_canonical_position_vector(double position[])
{
	copy_axis_vector(position, gm.position);	
	return (position);
}

double cm_get_runtime_machine_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return (mp_get_runtime_machine_position(axis) / MM_PER_INCH);
	} else {
		return (mp_get_runtime_machine_position(axis));
	}
}

double cm_get_runtime_work_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return (mp_get_runtime_work_position(axis) / MM_PER_INCH);
	} else {
		return (mp_get_runtime_work_position(axis));
	}
//	if (gm.units_mode == INCHES) {
//		return ((mp_get_runtime_position(axis) - cm_get_coord_offset(axis)) / MM_PER_INCH);
//	} else {
//		return (mp_get_runtime_position(axis) - cm_get_coord_offset(axis));
//	}
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
 * cm_set_radius()		- set radius value
 * cm_set_model_linenum() - set line number in the model (this is NOT the runtime line number)
 * cm_set_model_lineindex() - set line index in the model (this is NOT the runtime line index)
 * cm_incr_model_lineindex() - increment line index in the model (this is NOT the runtime line index)
 * cm_set_target()		- set all XYZABC targets
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

void cm_set_model_linenum(uint32_t linenum)
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
static double _calc_ABC(uint8_t i, double target[], double flag[]);

void cm_set_target(double target[], double flag[])
{ 
	uint8_t i;
	double tmp = 0;

	// process XYZABC for lower modes
	for (i=X; i<=Z; i++) {
		if ((flag[i] < EPSILON) || (cfg.a[i].axis_mode == AXIS_DISABLED)) {
			continue;
		} else if ((cfg.a[i].axis_mode == AXIS_STANDARD) || (cfg.a[i].axis_mode == AXIS_INHIBITED)) {
			if (gm.distance_mode == ABSOLUTE_MODE) {
				gm.target[i] = cm_get_coord_offset(i) + _to_millimeters(target[i]);
			} else {
				gm.target[i] += _to_millimeters(target[i]);
			}
		}
	}
	// FYI: The ABC loop below relies on the XYZ loop having been run first
	for (i=A; i<=C; i++) {
		// skip axis if not flagged for update or its disabled
		if ((flag[i] < EPSILON) || (cfg.a[i].axis_mode == AXIS_DISABLED)) {
			continue;
		} else tmp = _calc_ABC(i, target, flag);		
		
		if (gm.distance_mode == ABSOLUTE_MODE) {
			gm.target[i] = tmp + cm_get_coord_offset(i); // sacidu93's fix to Issue #22
		} else {
			gm.target[i] += tmp;
		}
	}
}

// ESTEE: fix to workaround a gcc compiler bug wherein it runs out of spill registers
// we moved this block into its own function so that we get a fresh stack push
// ALDEN: This shows up in avr-gcc 4.7.0 and avr-libc 1.8.0
static double _calc_ABC(uint8_t i, double target[], double flag[])
{
	double tmp = 0;
	
	if ((cfg.a[i].axis_mode == AXIS_STANDARD) || (cfg.a[i].axis_mode == AXIS_INHIBITED)) {
		tmp = target[i];	// no mm conversion - it's in degrees

	} else if ((cfg.a[i].axis_mode == AXIS_RADIUS) && (flag[i] > EPSILON)) {
		tmp = _to_millimeters(target[i]) * 360 / (2 * M_PI * cfg.a[i].radius);

/* COMMENTED OUT THE SLAVE MODES
	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_X) && (flag[X] > EPSILON)) {
		tmp = (target[X] - gm.position[X]) * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_Y) && (flag[Y] > EPSILON)) {
		tmp = (target[Y] - gm.position[Y]) * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_Z) && (flag[Z] > EPSILON)) {
		tmp = (target[Z] - gm.position[Z]) * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XY) && ((flag[X] > EPSILON) || (flag[Y] > EPSILON))) {
		double length = sqrt(square(target[X] - gm.position[X]) + square(target[Y] - gm.position[Y]));
		tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XZ) && ((flag[X] > EPSILON) || (flag[Z] > EPSILON))) {
		double length = sqrt(square(target[X] - gm.position[X]) + square(target[Z] - gm.position[Z]));
		tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_YZ) && ((flag[Y] > EPSILON) || (flag[Z] > EPSILON))) {
		double length = sqrt(square(target[Y] - gm.position[Y]) + square(target[Z] - gm.position[Z]));
		tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);

	} else if ((cfg.a[i].axis_mode == AXIS_SLAVE_XYZ) && ((flag[X] > EPSILON) || (flag[Y] > EPSILON) || (flag[Z] > EPSILON))) {
		double length = sqrt(square(target[X] - gm.position[X]) + square(target[Y] - gm.position[Y]) + square(target[Z] - gm.position[Z]));
		tmp = length * 360 / (2 * M_PI * cfg.a[i].radius);
*/
	}
	return tmp;
}

/* 
 * cm_set_gcode_model_endpoint_position() - uses internal coordinates only
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

void cm_set_gcode_model_endpoint_position(uint8_t status) 
{
	if (status == TG_OK) { copy_axis_vector(gm.position, gm.target);}
}

/* 
 * _get_move_times() - get required time for move and minimum time for move
 *
 *	Compute the optimum time for the move. This will either be the 
 *	length / rate (feedrate or traverse rate), or just time specified by inverse
 *	feed rate if G93 is active. Then test the move against the maximum feed
 *	feed or traverse rates for each axis in the move and increase the time to 
 *	accommodate the rate limiting axis. Axis modes are taken into account
 *	by having cm_set_target load the targets before calling this function.
 *
 *	The following times are compared and the longest is returned:
 *	  -	G93 inverse time (if G93 is active)
 *	  -	time for coordinated move at requested feed rate
 *	  -	time that the slowest axis would require for the move
 */
/* The following is verbatim text from NIST RS274NGC_v3. As I interpret A for 
 * moves that combine both linear and rotational movement, the feed rate should
 * apply to the XYZ movement, with the rotational axis (or axes) timed to start
 * and end at the same time the linear move is performed. It is possible under 
 * this case for the rotational move to rate-limit the linear move.
 *
 * 	2.1.2.5 Feed Rate
 *
 *	The rate at which the controlled point or the axes move is nominally a steady 
 *	rate which may be set by the user. In the Interpreter, the interpretation of 
 *	the feed rate is as follows unless inverse time feed rate mode is being used 
 *	in the RS274/NGC view (see Section 3.5.19). The canonical machining functions 
 *	view of feed rate, as described in Section 4.3.5.1, has conditions under which 
 *	the set feed rate is applied differently, but none of these is used in the 
 *	Interpreter.
 *
 *	A. 	For motion involving one or more of the X, Y, and Z axes (with or without 
 *		simultaneous rotational axis motion), the feed rate means length units 
 *		per minute along the programmed XYZ path, as if the rotational axes were 
 *		not moving.
 *
 *	B.	For motion of one rotational axis with X, Y, and Z axes not moving, the 
 *		feed rate means degrees per minute rotation of the rotational axis.
 *
 *	C.	For motion of two or three rotational axes with X, Y, and Z axes not moving, 
 *		the rate is applied as follows. Let dA, dB, and dC be the angles in degrees 
 *		through which the A, B, and C axes, respectively, must move. 
 *		Let D = sqrt(dA^2 + dB^2 + dC^2). Conceptually, D is a measure of total 
 *		angular motion, using the usual Euclidean metric. Let T be the amount of 
 *		time required to move through D degrees at the current feed rate in degrees 
 *		per minute. The rotational axes should be moved in coordinated linear motion 
 *		so that the elapsed time from the start to the end of the motion is T plus 
 *		any time required for acceleration or deceleration.
 */

static double _get_move_times(double *min_time)
{
	uint8_t i;
	double inv_time=0;	// inverse time if doing a feed in G93 mode
	double xyz_time=0;	// coordinated move linear part at req feed rate
	double abc_time=0;	// coordinated move rotary part at req feed rate
	double max_time=0;	// time required for the rate-limiting axis
	double tmp_time=0;	// used in computation
	*min_time = 1234567;// arbitrarily large number

	// compute times for feed motion
	if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
		if (gm.inverse_feed_rate_mode == true) {
			inv_time = gm.inverse_feed_rate;
		} else {
			xyz_time = sqrt(square(gm.target[X] - gm.position[X]) + // in mm
							square(gm.target[Y] - gm.position[Y]) +
							square(gm.target[Z] - gm.position[Z])) / gm.feed_rate; // in linear units
			if (xyz_time ==0) {
				abc_time = sqrt(square(gm.target[A] - gm.position[A]) + // in deg
							square(gm.target[B] - gm.position[B]) +
							square(gm.target[C] - gm.position[C])) / gm.feed_rate; // in degree units
			}
		}
	}
 	for (i=0; i<AXES; i++) {
		if (gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) {
//			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) / cfg.a[i].feedrate_max));
			tmp_time = fabs(gm.target[i] - gm.position[i]) / cfg.a[i].feedrate_max;
		} else { // gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE
//			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) / cfg.a[i].velocity_max));
			tmp_time = fabs(gm.target[i] - gm.position[i]) / cfg.a[i].velocity_max;
		}
		max_time = max(max_time, tmp_time);
		*min_time = min(*min_time, tmp_time);
	}
	return (max4(inv_time, max_time, xyz_time, abc_time));
}

/*************************************************************************
 *
 * CANONICAL MACHINING FUNCTIONS
 *
 *	Values are passed in pre-unit_converted state (from gn structure)
 *	All operations occur on gm (current model state)
 *
 ************************************************************************/

/* 
 * Initialization and Termination (4.3.2)
 *
 * cm_init() 
 *
 *	Config init cfg_init() must have been run beforehand. Many parameters 
 *	used by the canonical machine are actually set during cfg_init().
 */

void cm_init()
{
	memset(&cm, 0, sizeof(cm));			// reset canonicalMachineSingleton
	memset(&gn, 0, sizeof(gn));			// clear all values, pointers and status
	memset(&gf, 0, sizeof(gf));
	memset(&gm, 0, sizeof(gm));

	cm_spindle_init();					// init spindle PWM and variables

	// set gcode defaults
	_exec_set_units_mode(cfg.units_mode,0,0,0);
	_exec_set_coord_system(cfg.coord_system,0,0,0);
	_exec_select_plane(cfg.select_plane,0,0,0);
//	_exec_set_path_control(cfg.path_control,0,0,0);
	_exec_set_distance_mode(cfg.distance_mode,0,0,0);

//	cm_set_units_mode(cfg.units_mode);
//	cm_set_coord_system(cfg.coord_system);
//	cm_select_plane(cfg.select_plane);
	cm_set_path_control(cfg.path_control);
//	cm_set_distance_mode(cfg.distance_mode);
	
	// signal that the machine is ready for action
	cm.machine_state = MACHINE_RESET;	
	cm.combined_state = COMBINED_RESET;
}

/* 
 * Representation (4.3.3)
 *
 * cm_set_machine_coords() - update machine coordinates
 * cm_set_machine_zero() - set machine coordinates to zero
 * cm_set_machine_axis_position() - set the position of a single axis
 * cm_select_plane() - G17,G18,G19 select axis plane
 * cm_set_units_mode()  - G20, G21
 * cm_set_distance_mode() - G90, G91
 * cm_set_coord_system() - G54-G59
 * cm_set_coord_system_offsets() - G10
 * cm_set_origin_offsets() - G92
 * cm_reset_origin_offsets() - G92.1
 * cm_suspend_origin_offsets() - G92.2
 * cm_resume_origin_offsets() - G92.3
 */
/*
uint8_t cm_set_machine_coords(double offset[])
{
	copy_axis_vector(gm.position, offset);
	copy_axis_vector(gm.target, gm.position);
	mp_set_axis_position(gm.position);
	return (TG_OK);
}

uint8_t cm_set_machine_zero()
{
	copy_axis_vector(gm.position, set_vector(0,0,0,0,0,0));
	copy_axis_vector(gm.target, gm.position);
	mp_set_axes_position(gm.position);
	return (TG_OK);
}
*/

/*
 * cm_set_machine_axis_position() - set the position of a single axis
 */
uint8_t cm_set_machine_axis_position(uint8_t axis, const double position)
{
//	return(mp_queue_command(_exec_set_machine_axis_position, axis, position,0,0));
	_exec_set_machine_axis_position(axis, position,0,0); 
	return (TG_OK);
}
static void _exec_set_machine_axis_position(uint8_t axis, double position, double o[], double f[])
{
	gm.position[axis] = position;
	gm.target[axis] = position;
	mp_set_axis_position(axis, position);
}

/*
 * cm_select_plane() - G17,G18,G19 select axis plane
 */
uint8_t cm_select_plane(uint8_t plane) 
{
//	return(mp_queue_command(_exec_select_plane, plane,0,0,0));
	_exec_select_plane(plane,0,0,0);
	return (TG_OK);
}
static void _exec_select_plane(uint8_t plane, double d, double o[], double f[])
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
}

/*
 * cm_set_units_mode() - G20, G21
 */
uint8_t cm_set_units_mode(uint8_t mode)
{
//	return(mp_queue_command(_exec_set_units_mode, mode,0,0,0));
	_exec_set_units_mode(mode,0,0,0);
	return (TG_OK);
}
static void _exec_set_units_mode(uint8_t mode, double d, double o[], double f[])
{
	gm.units_mode = mode;	// 0 = inches, 1 = mm.
}
uint8_t cm_set_units_now(uint8_t mode)
{
	gm.units_mode = mode;	// 0 = inches, 1 = mm.
	return(TG_OK);
}

/*
 * cm_set_distance_mode() - G90, G91
 */
uint8_t cm_set_distance_mode(uint8_t mode)
{
//	return(mp_queue_command(_exec_set_distance_mode, mode,0,0,0));
	_exec_set_distance_mode(mode,0,0,0);
	return (TG_OK);
}
static void _exec_set_distance_mode(uint8_t mode, double d, double o[], double f[])
{
	gm.distance_mode = mode;	// 0 = absolute mode, 1 = incremental
}

/*
 * cm_set_coord_system() - G54-G59
 */
uint8_t	cm_set_coord_system(uint8_t coord_system)
{
//	return(mp_queue_command(_exec_set_coord_system, coord_system,0,0,0));
	_exec_set_coord_system(coord_system,0,0,0);
	return (TG_OK);
}
static void _exec_set_coord_system(uint8_t coord_system, double d, double o[], double f[])
{
	gm.coord_system = coord_system;	
}

/*
 * cm_set_coord_system_offsets() - G10 L2 Pn
 */
uint8_t	cm_set_coord_offsets(uint8_t coord_system, double offset[], double flag[])
{
	if ((coord_system < G54) || (coord_system > COORD_SYSTEM_MAX)) { // you can't set G53
		return (TG_INTERNAL_RANGE_ERROR);
	}
//	return(mp_queue_command(_exec_set_coord_offsets, coord_system, 0, offset, flag));
	_exec_set_coord_offsets(coord_system, 0, offset, flag);
	return (TG_OK);
}
static void _exec_set_coord_offsets(uint8_t coord_system, double d, double offset[], double flag[])
{
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {
			cfg.offset[coord_system][i] = offset[i];
			cm.g10_flag = true;	// this will persist offsets to NVM once move has stopped
		}
	}
	// ########################################see if it's OK to write them now, or if they need to wait until STOP
	if (cm.machine_state != MACHINE_CYCLE) {
		cmd_persist_offsets(cm.g10_flag);
	}
}

/*
 * cm_set_origin_offsets() - G92
 */
uint8_t cm_set_origin_offsets(double offset[], double flag[])
{
//	return(mp_queue_command(_exec_set_origin_offsets, 0,0, offset, flag));
	_exec_set_origin_offsets(0,0, offset, flag);
	return (TG_OK);
}
static void _exec_set_origin_offsets(uint8_t u, double d, double offset[], double flag[])
{
	gm.origin_offset_mode = true;
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {	 		// behaves according to NIST 3.5.18
			gm.origin_offset[i] = gm.position[i] - cfg.offset[gm.coord_system][i] - _to_millimeters(offset[i]);
		}
	}
}

/*
 * cm_reset_origin_offsets() - G92.1
 * cm_suspend_origin_offsets() - G92.2
 * cm_resume_origin_offsets() - G92.3
 */
uint8_t cm_reset_origin_offsets()
{
//	return(mp_queue_command(_exec_modify_origin_offsets, 1,0,0,0));
	_exec_modify_origin_offsets(1,0,0,0);
	return (TG_OK);
}
uint8_t cm_suspend_origin_offsets()
{
//	return(mp_queue_command(_exec_modify_origin_offsets, 2,0,0,0));
	_exec_modify_origin_offsets(2,0,0,0);
	return (TG_OK);
}
uint8_t cm_resume_origin_offsets()
{
//	return(mp_queue_command(_exec_modify_origin_offsets, 3,0,0,0));
	_exec_modify_origin_offsets(3,0,0,0);
	return (TG_OK);
}
static void _exec_modify_origin_offsets(uint8_t dot, double d, double o[], double f[])
{
	if (dot == 1) {							// reset offsets
		gm.origin_offset_mode = false;
		for (uint8_t i=0; i<AXES; i++) 
			gm.origin_offset[i] = 0;
	} else if (dot == 2)					// suspend offsets
		gm.origin_offset_mode = true;
	else									// resume offsets
		gm.origin_offset_mode = false;
}

/* 
 * Free Space Motion (4.3.4)
 *
 * cm_straight_traverse() - G0 linear rapid
 */

uint8_t cm_straight_traverse(double target[], double flags[])
{
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_target(target,flags);
	cm_exec_cycle_start();					//required for homing & other cycles
	uint8_t status = MP_LINE(gm.target, _get_move_times(&gm.min_time), gm.work_offset, gm.min_time);
	cm_set_gcode_model_endpoint_position(status);
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
//	return(mp_queue_command(_exec_set_feed_rate, 0, feed_rate, 0,0));	
	_exec_set_feed_rate(0, feed_rate, 0,0);	
	return (TG_OK);
}
static void _exec_set_feed_rate(uint8_t u, double feed_rate, double o[], double f[])
{
	if (gm.inverse_feed_rate_mode == true) {
		gm.inverse_feed_rate = feed_rate; // minutes per motion for this block only
	} else {
		gm.feed_rate = _to_millimeters(feed_rate);
	}
}

/*
 * cm_set_inverse_feed_rate() - G93, G94
 *
 *	TRUE = inverse time feed rate in effect - for this block only
 *	FALSE = units per minute feed rate in effect
 */

inline uint8_t cm_set_inverse_feed_rate_mode(uint8_t mode)
{
//	return(mp_queue_command(_exec_set_inverse_feed_rate_mode, mode,0,0,0));	
	_exec_set_inverse_feed_rate_mode(mode,0,0,0);	
	return (TG_OK);
}
static void _exec_set_inverse_feed_rate_mode(uint8_t mode, double d, double o[], double f[])
{
	gm.inverse_feed_rate_mode = mode;
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
 * cm_arc_feed() - see arc.c
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
	cm_exec_cycle_start();					//required for homing & other cycles
	status = MP_LINE(gm.target, _get_move_times(&gm.min_time), gm.work_offset, gm.min_time);

	cm_set_gcode_model_endpoint_position(status);
	return (status);
}

/* 
 * Spindle Functions (4.3.7) - see spindle.c, spindle.h
 * Tool Functions (4.3.8)
 *
 * cm_change_tool() - M6 (This might become a complete tool change cycle)
 * cm_select_tool() - T parameter
 */

uint8_t cm_change_tool(uint8_t tool)
{
//	return(mp_queue_command(_exec_change_tool, tool,0,0,0));	
	_exec_change_tool(tool,0,0,0);	
	return (TG_OK);
}
static void _exec_change_tool(uint8_t tool, double d, double o[], double f[])
{
//	cm_sync_tool_number(tool);
	gm.tool = tool;
}

uint8_t cm_select_tool(uint8_t tool)
{
//	return(mp_queue_command(_exec_select_tool, tool,0,0,0));	
	_exec_select_tool(tool,0,0,0);	
	return (TG_OK);
}
static void _exec_select_tool(uint8_t tool, double d, double o[], double f[])
{
//	cm_sync_tool_number(tool);
	gm.tool = tool;
}

/* 
 * Miscellaneous Functions (4.3.9)
 *
 * cm_mist_coolant_control() - M7
 * cm_flood_coolant_control() - M8, M9
 * cm_feed_override_enable() - M48, M49
 * cm_comment() - ignore comments (I do)
 * cm_message() - send message to console
 */

/*
 * cm_mist_coolant_control() - M7
 */
/*
uint8_t cm_mist_coolant_control(uint8_t mist_coolant)
{
	return(mp_queue_command(_exec_mist_coolant_control, mist_coolant,0,0,0));
	return (TG_OK);
}
*/
static void _exec_mist_coolant_control(uint8_t mist_coolant, double d, double o[], double f[])
{
	gm.mist_coolant = mist_coolant;
	if (mist_coolant == true) {
		gpio_set_bit_on(MIST_COOLANT_BIT);
	} else {
		gpio_set_bit_off(MIST_COOLANT_BIT);
	}
}

uint8_t cm_mist_coolant_control(uint8_t mist_coolant)
{
	if (mist_coolant == true) {
		mp_sync_mcode(SYNC_MIST_COOLANT_ON);
	} 
	return (TG_OK);
}

void cm_exec_mist_coolant_control(uint8_t mist_coolant)
{
	gm.mist_coolant = mist_coolant;
	if (mist_coolant == true) {
		gpio_set_bit_on(MIST_COOLANT_BIT);
	} else {
		gpio_set_bit_off(MIST_COOLANT_BIT);
	}
}

/*
 * cm_flood_coolant_control() - M8, M9
 */
/*
uint8_t cm_flood_coolant_control(uint8_t flood_coolant)
{
	return(mp_queue_command(_exec_flood_coolant_control, flood_coolant,0,0,0));
	return (TG_OK);
}
*/
static void _exec_flood_coolant_control(uint8_t flood_coolant, double d, double o[], double f[])
{
	gm.flood_coolant = flood_coolant;
	if (flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		_exec_mist_coolant_control(false,0,0,0);	// M9 special function
	}
}

uint8_t cm_flood_coolant_control(uint8_t flood_coolant)
{
	if (flood_coolant == true) {
		mp_sync_mcode(SYNC_FLOOD_COOLANT_ON);
	} else {
		mp_sync_mcode(SYNC_FLOOD_COOLANT_OFF);
	}
	return (TG_OK);
}

void cm_exec_flood_coolant_control(uint8_t flood_coolant)
{
	gm.flood_coolant = flood_coolant;
	if (flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		cm_mist_coolant_control(false);		// M9 special function
	}
}

uint8_t cm_feed_override_enable(uint8_t feed_override)
{
	if (feed_override == true) {
		mp_sync_mcode(SYNC_FEED_OVERRIDE_ON);
	} else {
		mp_sync_mcode(SYNC_FEED_OVERRIDE_OFF);
	}
	return (TG_OK);
}

void cm_exec_feed_override_enable(uint8_t feed_override)
{
	gm.feed_override_enable = feed_override;
}


void cm_comment(char *comment)
{
	return;
}

void cm_message(char *message)
{
	cmd_add_string("msg", message);		// adds the message to the response object
}

/*
 * Program Functions (4.3.10)
 *
 * This group implements stop, start, end, and hold. 
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	cm_exec_cycle_start()		(no Gcode)  Do a cycle start right now
 *	cm_exec_cycle_end()			(no Gcode)	Do a cycle end right now
 *	cm_exec_feedhold()			(no Gcode)	Initiate a feedhold right now	
 *	cm_program_stop()			(M0, M60)	Queue a program stop
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	cm_exec_program_stop()
 *	cm_exec_program_end()
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

void cm_exec_cycle_start()
{
	cm.cycle_start_flag = true;
	cm.machine_state = MACHINE_CYCLE;
	if (cm.cycle_state == CYCLE_OFF) {
		cm.cycle_state = CYCLE_STARTED;	// don't change homing, probe or other cycles
	}
}

void cm_exec_cycle_end() 
{
	if (cm.cycle_state == CYCLE_STARTED) {
		cm_exec_program_stop();		// don't stop if it's in a homing or other specialized cycle
	}
}

void cm_exec_feedhold()
{
	if ((cm.motion_state == MOTION_RUN) && (cm.hold_state == FEEDHOLD_OFF)) {
		cm.motion_state = MOTION_HOLD;
		cm.hold_state = FEEDHOLD_SYNC;
		cm.cycle_start_flag = false;
	}
}

void cm_program_stop() { mp_sync_mcode(SYNC_PROGRAM_STOP);}

void cm_optional_program_stop()	{ mp_sync_mcode(SYNC_PROGRAM_STOP); }

void cm_program_end()				// M2, M30
{
	tg_reset_source();				// stop reading from a file (return to std device)
	mp_sync_mcode(SYNC_PROGRAM_END);
}

void cm_exec_program_stop() { _exec_program_finalize(MACHINE_PROGRAM_STOP);}

void cm_exec_program_end() { _exec_program_finalize(MACHINE_PROGRAM_END);}

void _exec_program_finalize(uint8_t machine_state)
{
	cm.machine_state = machine_state;
	cm.cycle_state = CYCLE_OFF;
	cm.motion_state = MOTION_STOP;
	cm.hold_state = FEEDHOLD_OFF;		//...and any feedhold is ended
	cm.cycle_start_flag = false;
	mp_zero_segment_velocity();			// for reporting purposes
	rpt_request_status_report();		// request final status report (if enabled)
	cmd_persist_offsets(cm.g10_flag);	// persist offsets (if any changes made)
}


