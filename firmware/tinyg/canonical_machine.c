/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S Hart, Jr.
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
 *	Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *	Some commands in the canonical machine need to be executed immediately and 
 *	some need to be synchronized with movement (the planner queue). In general,
 *	commands that only affect the gcode model are done immediately whereas 
 *	commands that have a physcial effect must be synchronized.
 *
 *	Immediate commands are obvious - just write to the GM struct. 
 *	Synchronous commands work like this:
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
 *	For a list of the synchronous commands see the static function prototypes
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
#include "canonical_machine.h"
#include "plan_arc.h"
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
static void _exec_offset(uint8_t coord_system, double float_val);
static void _exec_change_tool(uint8_t tool, double float_val);
static void _exec_select_tool(uint8_t tool, double float_val);
static void _exec_mist_coolant_control(uint8_t mist_coolant, double float_val);
static void _exec_flood_coolant_control(uint8_t flood_coolant, double float_val);
//static void _exec_feed_override_enable(uint8_t feed_override, double float_val);
static void _exec_program_finalize(uint8_t machine_state, double float_val);

#define _to_millimeters(a) ((gm.units_mode == INCHES) ? (a * MM_PER_INCH) : a)

/*************************************************************************
 *
 * HELPERS AND UTILITY FUNCTIONS
 *
 *	These functions are not part of the NIST defined functions
 *
 ************************************************************************/

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
void cm_set_motion_mode(uint8_t motion_mode) {gm.motion_mode = motion_mode;} 
void cm_set_absolute_override(uint8_t absolute_override) { gm.absolute_override = absolute_override;}
void cm_set_spindle_mode(uint8_t spindle_mode) { gm.spindle_mode = spindle_mode;} 
void cm_set_spindle_speed_parameter(double speed) { gm.spindle_speed = speed;}
void cm_set_tool_number(uint8_t tool) { gm.tool = tool;}

//void cm_sync_tool_number(uint8_t tool) { mp_sync_command(SYNC_TOOL_NUMBER, (double)tool);}
//void cm_sync_spindle_speed_parameter(double speed) { mp_sync_command(SYNC_SPINDLE_SPEED, speed);}

/* Position and Offset getters
 *
 * cm_get_coord_offset() - return the currently active coordinate offset for an axis
 * cm_get_coord_offset_vector() - return currently active coordinate offsets as a vector
 * cm_get_model_work_position() - return position from the gm struct into gn struct form (external form)
 * cm_get_model_work_position_vector() - return model position vector in externalized form
 * cm_get_model_canonical_target() - return model target in internal canonical form
 * cm_get_model_canonical_position_vector() - return model position vector in internal canonical form
 * cm_get_runtime_machine_position() - return current machine position in external form 
 * cm_get_runtime work_position() - return current work coordinate position in external form 
 * cm_get_runtime work_offset() - return current work offset
 * cm_get_runtime work_scaling() - return current work scaling factor
 */

double cm_get_coord_offset(uint8_t axis)
{
	if (gm.absolute_override == true) {
		return (0);						// no work offset if in abs override mode
	}
	if (gm.origin_offset_enable == 1) {
		return (cfg.offset[gm.coord_system][axis] + gm.origin_offset[axis]); // includes G5x and G92 compoenents
	} else {
		return (cfg.offset[gm.coord_system][axis]);		// just the g5x coordinate system components
	}
}

double *cm_get_coord_offset_vector(double vector[])
{
	for (uint8_t i=0; i<AXES; i++) {
		vector[i] = cm_get_coord_offset(i);
	}
	return (vector);
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
	return (mp_get_runtime_machine_position(axis));

// deprecated behavior
//	if (gm.units_mode == INCHES) {
//		return (mp_get_runtime_machine_position(axis) / MM_PER_INCH);
//	} else {
//		return (mp_get_runtime_machine_position(axis));
//	}
}

double cm_get_runtime_work_position(uint8_t axis) 
{
	if (gm.units_mode == INCHES) {
		return (mp_get_runtime_work_position(axis) / MM_PER_INCH);
	} else {
		return (mp_get_runtime_work_position(axis));
	}
}

double cm_get_runtime_work_offset(uint8_t axis) 
{
	return (mp_get_runtime_work_offset(axis));
}

/*
 * Setters - these inhale gn values into the gm struct
 *
 *	Input coordinates are in native block formats (gn form);
 *	i.e. they are not unit adjusted or otherwise pre-processed.
 *	The setters take care of coordinate system, units, and 
 *	distance mode conversions and normalizations.
 *
 * cm_set_arc_offset()	  - set all IJK offsets
 * cm_set_radius()		  - set radius value
 * cm_set_model_linenum() - set line number in the model
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
	gm.linenum = linenum;
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
 *		  - Radius mode is only processed for ABC axes. Application to XYZ is ignored.
 *
 *	Target coordinates are provided in target[]
 *	Axes that need processing are signaled in flag[]
 */
static double _calc_ABC(uint8_t i, double target[], double flag[]);

//void cm_set_target(double target[], double flag[], uint8_t machine_coords)
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
 * cm_set_gcode_model_endpoint_position() - uses internal canonical coordinates only
 *
 * 	This routine sets the endpoint position in the gccode model if the move was
 *	successfully completed (no errors). Leaving the endpoint position alone for 
 *	errors allows too-short-lines to accumulate into longer lines (line interpolation).
 *
 * 	Note: As far as the canonical machine is concerned the final position is achieved 
 *	as soon at the move is executed and the position is now the target. In reality 
 *	the planner(s) and steppers will still be processing the action and the real tool 
 *	position is still close to the starting point. 
 */

void cm_set_gcode_model_endpoint_position(uint8_t status) 
{
	if (status == TG_OK) copy_axis_vector(gm.position, gm.target);
}

/* 
 * _get_move_times() - get minimum and optimal move times
 *
 *	The minimum time is the fastest the move can be performed given the velocity 
 *	constraints on each particpating axis - regardless of the feedrate requested. 
 *	The minimum time is the time limited by the rate-limiting axis. The minimum 
 *	time is needed to compute the optimal time and is recorded for possible 
 *	feed override computation..
 *
 *	The optimal time is either the time resulting from the requested feedrate or 
 *	the minimum time if the requested feedrate is not achievable. Optimal times for 
 *	traverses are always the minimum time.
 *
 *	Axis modes are taken into account by having cm_set_target() load the targets 
 *	before calling this function.
 *
 *	The following times are compared and the longest is returned:
 *	  -	G93 inverse time (if G93 is active)
 *	  -	time for coordinated move at requested feed rate
 *	  -	time that the slowest axis would require for the move
 *
 *	Returns:
 *	  - Optimal time returned as the function return
 *	  - Minimum time is set via the function argument
 */
/* --- NIST RS274NGC_v3 Guidance ---
 *
 * The following is verbatim text from NIST RS274NGC_v3. As I interpret A for 
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
			tmp_time = fabs(gm.target[i] - gm.position[i]) / cfg.a[i].feedrate_max;
		} else { // gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE
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
 * cm_shutdown() 
 * cm_flush_planner()
 *
 *	Config init cfg_init() must have been run beforehand. Many parameters 
 *	used by the canonical machine are actually set during cfg_init().
 */

void cm_init()
{
// You can assume all memory has been zeroed by a hard reset. If not, use this code:
//	memset(&cm, 0, sizeof(cm));		// reset canonicalMachineSingleton
//	memset(&gn, 0, sizeof(gn));		// clear all values, pointers and status
//	memset(&gf, 0, sizeof(gf));
//	memset(&gm, 0, sizeof(gm));

	// setup magic numbers
	cm.magic_start = MAGICNUM;
	cm.magic_end = MAGICNUM;
	gm.magic_start = MAGICNUM;
	gm.magic_end = MAGICNUM;

	// set gcode defaults
	cm_set_units_mode(cfg.units_mode);
	cm_set_coord_system(cfg.coord_system);
	cm_select_plane(cfg.select_plane);
	cm_set_path_control(cfg.path_control);
	cm_set_distance_mode(cfg.distance_mode);

	// never start a machine in a motion mode	
	gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;

	// signal that the machine is ready for action
	cm.machine_state = MACHINE_READY;	
	cm.combined_state = COMBINED_READY;
}

/*
 * cm_shutdown() - shut down machine
 */

void cm_shutdown(uint8_t value)
{
	// stop the steppers and the spindle
	st_disable();
	cm_spindle_control(SPINDLE_OFF);

	// disable all MCode functions
//	gpio_set_bit_off(SPINDLE_BIT);			//###### this current stuff is temporary
//	gpio_set_bit_off(SPINDLE_DIR);
//	gpio_set_bit_off(SPINDLE_PWM);
//	gpio_set_bit_off(MIST_COOLANT_BIT);		//###### replace with exec function
//	gpio_set_bit_off(FLOOD_COOLANT_BIT);	//###### replace with exec function

	rpt_exception(TG_SHUTDOWN,value);		// send shutdown message
	cm.machine_state = MACHINE_SHUTDOWN;
}

/*
 * cm_flush_planner() - Flush planner queue and correct model positions
 */
uint8_t cm_flush_planner()
{
	mp_flush_planner();

	for (uint8_t i=0; i<AXES; i++) {
		mp_set_axis_position(i, mp_get_runtime_machine_position(i));	// set mm from mr
		gm.position[i] = mp_get_runtime_machine_position(i);
		gm.target[i] = gm.position[i];
	}
	return (TG_OK);
}

/* 
 * Representation (4.3.3)
 *
 * cm_set_machine_axis_position()- set the position of a single axis
 * cm_select_plane()			- G17,G18,G19 select axis plane
 * cm_set_units_mode()			- G20, G21
 * cm_set_distance_mode()		- G90, G91
 * cm_set_coord_system()		- G54-G59
 * cm_set_coord_system_offsets()- G10 (does not persist)
 * cm_set_origin_offsets()		- G92
 * cm_reset_origin_offsets()	- G92.1
 * cm_suspend_origin_offsets()	- G92.2
 * cm_resume_origin_offsets()	- G92.3
 */

/*
 * cm_set_machine_axis_position() - set the position of a single axis
 */
uint8_t cm_set_machine_axis_position(uint8_t axis, const double position)
{
	gm.position[axis] = position;
	gm.target[axis] = position;
	mp_set_axis_position(axis, position);
	return (TG_OK);
}

/*
 * cm_select_plane() - G17,G18,G19 select axis plane
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

/*
 * cm_set_units_mode() - G20, G21
 */
uint8_t cm_set_units_mode(uint8_t mode)
{
	gm.units_mode = mode;	// 0 = inches, 1 = mm.
	return(TG_OK);
}

/*
 * cm_set_distance_mode() - G90, G91
 */
uint8_t cm_set_distance_mode(uint8_t mode)
{
	gm.distance_mode = mode;	// 0 = absolute mode, 1 = incremental
	return (TG_OK);
}

/*
 * cm_set_coord_system() - G54-G59
 */
uint8_t	cm_set_coord_system(uint8_t coord_system)
{
	gm.coord_system = coord_system;	
	mp_queue_command(_exec_offset, coord_system,0);
	return (TG_OK);
}
static void _exec_offset(uint8_t coord_system, double float_val)
{
	double offsets[AXES];
	for (uint8_t i=0; i<AXES; i++) {
		offsets[i] = cfg.offset[coord_system][i] + (gm.origin_offset[i] * gm.origin_offset_enable);
	}
	mp_set_runtime_work_offset(offsets);
}

/*
 * cm_set_coord_system_offsets() - G10 L2 Pn
 *
 *	Note: This function appies the offset to the GM model but does not persist
 *	the offsets (as Gcode expects). If you want to persist coordinate system 
 *	offsets use $g54x - $g59c config functions instead.
 */
uint8_t	cm_set_coord_offsets(uint8_t coord_system, double offset[], double flag[])
{
	if ((coord_system < G54) || (coord_system > COORD_SYSTEM_MAX)) { // you can't set G53
		return (TG_INTERNAL_RANGE_ERROR);
	}
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {
			cfg.offset[coord_system][i] = offset[i];
			cm.g10_persist_flag = true;		// this will persist offsets to NVM once move has stopped
		}
	}
	return (TG_OK);
}

/*
 * cm_set_absolute_origin() - G28.3
 *
 *	This is an "unofficial gcode" command to allow arbitrarily setting an axis 
 *	to an absolute position. This is needed to support the Otherlab infinite 
 *	Y axis. USE: With the axis(or axes) where you want it, issue g92.4 y0 
 *	(for example). The Y axis will now be set to 0 (or whatever value provided)
 */
uint8_t cm_set_absolute_origin(double origin[], double flag[])
{
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {
			cm_set_machine_axis_position(i, cfg.offset[gm.coord_system][i] + _to_millimeters(origin[i]));
			cm.homed[i] = true;
		}
	}
	return (TG_OK);
}

/* 
 * cm_set_origin_offsets() - G92
 * cm_reset_origin_offsets() - G92.1
 * cm_suspend_origin_offsets() - G92.2
 * cm_resume_origin_offsets() - G92.3
 *
 * G92's behave according to NIST 3.5.18 & LinuxCNC G92
 * http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G92-G92.1-G92.2-G92.3
 */
uint8_t cm_set_origin_offsets(double offset[], double flag[])
{
	gm.origin_offset_enable = 1;
	for (uint8_t i=0; i<AXES; i++) {
		if (flag[i] > EPSILON) {
			gm.origin_offset[i] = gm.position[i] - cfg.offset[gm.coord_system][i] - _to_millimeters(offset[i]);
		}
	}
	mp_queue_command(_exec_offset, gm.coord_system,0);
	return (TG_OK);
}

uint8_t cm_reset_origin_offsets()
{
	gm.origin_offset_enable = 0;
	for (uint8_t i=0; i<AXES; i++) 
		gm.origin_offset[i] = 0;
	mp_queue_command(_exec_offset, gm.coord_system,0);
	return (TG_OK);
}

uint8_t cm_suspend_origin_offsets()
{
	gm.origin_offset_enable = 0;
	mp_queue_command(_exec_offset, gm.coord_system,0);
	return (TG_OK);
}

uint8_t cm_resume_origin_offsets()
{
	gm.origin_offset_enable = 1;
	mp_queue_command(_exec_offset, gm.coord_system,0);
	return (TG_OK);
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
	cm_cycle_start();							// required for homing & other cycles
	uint8_t status = MP_LINE(gm.target, 
							_get_move_times(&gm.min_time), 
							cm_get_coord_offset_vector(gm.work_offset), 
							gm.min_time);
	cm_set_gcode_model_endpoint_position(status);
	return (status);
}

/*
 * cm_set_g28_position()  - G28.1
 * cm_goto_g28_position() - G28
 * cm_set_g30_position()  - G30.1
 * cm_goto_g30_position() - G30
 */

uint8_t cm_set_g28_position(void)
{
	copy_axis_vector(gm.g28_position, gm.position);
	return (TG_OK);
}

uint8_t cm_goto_g28_position(double target[], double flags[])
{
	cm_set_absolute_override(true);
	cm_straight_traverse(target, flags);
	while (mp_get_planner_buffers_available() == 0); 	// make sure you have an available buffer
	double f[] = {1,1,1,1,1,1};
	return(cm_straight_traverse(gm.g28_position, f));
}

uint8_t cm_set_g30_position(void)
{
	copy_axis_vector(gm.g30_position, gm.position);
	return (TG_OK);
}

uint8_t cm_goto_g30_position(double target[], double flags[])
{
	cm_set_absolute_override(true);
	cm_straight_traverse(target, flags);
	while (mp_get_planner_buffers_available() == 0); 	// make sure you have an available buffer
	double f[] = {1,1,1,1,1,1};
	return(cm_straight_traverse(gm.g30_position, f));
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
 * cm_arc_feed() - see arc.c
 * cm_dwell() - G4, P parameter (seconds)
 * cm_straight_feed() - G1
 */ 

uint8_t cm_dwell(double seconds)
{
	gm.parameter = seconds;
	(void)mp_dwell(seconds);
	return (TG_OK);
}

uint8_t cm_straight_feed(double target[], double flags[])
{
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
	cm_cycle_start();						// required for homing & other cycles
	uint8_t status = MP_LINE(gm.target, 
							 _get_move_times(&gm.min_time), 
							 cm_get_coord_offset_vector(gm.work_offset), 
							 gm.min_time);

	cm_set_gcode_model_endpoint_position(status);
	return (status);
}

/* 
 * Spindle Functions (4.3.7)
 */
// see spindle.c, spindle.h

/*
 * Tool Functions (4.3.8)
 *
 * cm_change_tool() - M6 (This might become a complete tool change cycle)
 * cm_select_tool() - T parameter
 *
 * These functions are stubbed out for now and don't actually do anything
 */

uint8_t cm_change_tool(uint8_t tool)
{
	mp_queue_command(_exec_change_tool, tool, 0);
	return (TG_OK);
}
static void _exec_change_tool(uint8_t tool, double float_val)
{
	gm.tool = tool;
}

uint8_t cm_select_tool(uint8_t tool)
{
	mp_queue_command(_exec_select_tool, tool, 0);
	return (TG_OK);
}
static void _exec_select_tool(uint8_t tool, double float_val)
{
	gm.tool = tool;
}

/* 
 * Miscellaneous Functions (4.3.9)
 *
 * cm_mist_coolant_control() - M7
 * cm_flood_coolant_control() - M8, M9
 */

uint8_t cm_mist_coolant_control(uint8_t mist_coolant)
{
	mp_queue_command(_exec_mist_coolant_control, mist_coolant,0);
	return (TG_OK);
}

uint8_t cm_flood_coolant_control(uint8_t flood_coolant)
{
	mp_queue_command(_exec_flood_coolant_control, flood_coolant,0);
	return (TG_OK);
}

static void _exec_mist_coolant_control(uint8_t mist_coolant, double float_val)
{
	gm.mist_coolant = mist_coolant;
	if (mist_coolant == true) {
		gpio_set_bit_on(MIST_COOLANT_BIT);
	} else {
		gpio_set_bit_off(MIST_COOLANT_BIT);
	}
}

static void _exec_flood_coolant_control(uint8_t flood_coolant, double float_val)
{
	gm.flood_coolant = flood_coolant;
	if (flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		_exec_mist_coolant_control(false,0);	// M9 special function
	}
}

/*
 * cm_override_enables() - M48, M49
 * cm_feed_rate_override_enable() - M50
 * cm_feed_rate_override_factor() - M50.1
 * cm_traverse_override_enable() - M50.2
 * cm_traverse_override_factor() - M50.3
 * cm_spindle_override_enable() - M51
 * cm_spindle_override_factor() - M51.1
 *
 *	Override enables are kind of a mess in Gcode. This is an attempt to sort them out.
 *	See http://www.linuxcnc.org/docs/2.4/html/gcode_main.html#sec:M50:-Feed-Override
 */

uint8_t cm_override_enables(uint8_t flag)			// M48, M49
{
	gm.feed_rate_override_enable = flag;
	gm.traverse_override_enable = flag;
	gm.spindle_override_enable = flag;
	return (TG_OK);
}

uint8_t cm_feed_rate_override_enable(uint8_t flag)	// M50
{
	if ((gf.parameter == true) && (gn.parameter == 0)) {
		gm.feed_rate_override_enable = false;
	} else {
		gm.feed_rate_override_enable = true;
	}
	return (TG_OK);
}

uint8_t cm_feed_rate_override_factor(uint8_t flag)	// M50.1
{
	gm.feed_rate_override_enable = flag;
	gm.feed_rate_override_factor = gn.parameter;
//	mp_feed_rate_override(flag, gn.parameter);		// replan the queue for new feed rate
	return (TG_OK);
}

uint8_t cm_traverse_override_enable(uint8_t flag)	// M50.2
{
	if ((gf.parameter == true) && (gn.parameter == 0)) {
		gm.traverse_override_enable = false;
	} else {
		gm.traverse_override_enable = true;
	}
	return (TG_OK);
}

uint8_t cm_traverse_override_factor(uint8_t flag)	// M51
{
	gm.traverse_override_enable = flag;
	gm.traverse_override_factor = gn.parameter;
//	mp_feed_rate_override(flag, gn.parameter);		// replan the queue for new feed rate
	return (TG_OK);
}

uint8_t cm_spindle_override_enable(uint8_t flag)	// M51.1
{
	if ((gf.parameter == true) && (gn.parameter == 0)) {
		gm.spindle_override_enable = false;
	} else {
		gm.spindle_override_enable = true;
	}
	return (TG_OK);
}

uint8_t cm_spindle_override_factor(uint8_t flag)	// M50.1
{
	gm.spindle_override_enable = flag;
	gm.spindle_override_factor = gn.parameter;
//	change spindle speed
	return (TG_OK);
}

/*
 * cm_message() - send a message to the console (or JSON)
 */

void cm_message(char *message)
{
	cmd_add_message(message);			// conditionally adds the message to the response object
//	cmd_add_string("msg", message);		// adds the message to the response object
}

/*
 * Program Functions (4.3.10)
 *
 * This group implements stop, start, end, and hold. 
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	cm_cycle_start()			(no Gcode)  Do a cycle start right now
 *	cm_cycle_end()				(no Gcode)	Do a cycle end right now
 *	cm_feedhold()				(no Gcode)	Initiate a feedhold right now	
 *	cm_program_stop()			(M0, M60)	Queue a program stop
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	_exec_program_finalize()
 *
 * cm_program_stop and cm_optional_program_stop are synchronous Gcode 
 * commands that are received through the interpreter. They cause all motion
 * to stop at the end of the current command, including spindle motion. 
 *
 * Note that the stop occurs at the end of the immediately preceding command
 * (i.e. the stop is queued behind the last command).
 *
 * cm_program_end is a stop that also resets the machine to initial state
 */
void cm_cycle_start()
{
	cm.cycle_start_flag = true;
	cm.machine_state = MACHINE_CYCLE;
	if (cm.cycle_state == CYCLE_OFF) {
		cm.cycle_state = CYCLE_STARTED;	// don't change homing, probe or other cycles
	}
}

void cm_cycle_end() 
{
	if (cm.cycle_state == CYCLE_STARTED) {
		_exec_program_finalize(MACHINE_PROGRAM_STOP,0);
	}
}

void cm_feedhold()
{
	if ((cm.motion_state == MOTION_RUN) && (cm.hold_state == FEEDHOLD_OFF)) {
		cm.motion_state = MOTION_HOLD;
		cm.hold_state = FEEDHOLD_SYNC;
		cm.cycle_start_flag = false;
	}
}

void cm_program_stop() 
{ 
	mp_queue_command(_exec_program_finalize, MACHINE_PROGRAM_STOP,0);
}

void cm_optional_program_stop()	
{ 
	mp_queue_command(_exec_program_finalize, MACHINE_PROGRAM_STOP,0);
}

void cm_program_end()				// M2, M30
{
//	cm_set_motion_mode(MOTION_MODE_CANCEL_MOTION_MODE);
	mp_queue_command(_exec_program_finalize, MACHINE_PROGRAM_END,0);
}

static void _exec_program_finalize(uint8_t machine_state, double f)
{
	cm.machine_state = machine_state;
	cm.cycle_state = CYCLE_OFF;
	cm.motion_state = MOTION_STOP;
	cm.hold_state = FEEDHOLD_OFF;			//...and any feedhold is ended
	cm.cycle_start_flag = false;
	mp_zero_segment_velocity();				// for reporting purposes
	rpt_request_status_report();			// request final status report (not unfiltered)
	cmd_persist_offsets(cm.g10_persist_flag); // persist offsets if any changes made
}

