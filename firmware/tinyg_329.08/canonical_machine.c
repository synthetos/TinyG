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
 * commands - passing the simplest caommands it can down to the motion 
 * control layer. See the notes at the end of gcode.h for more details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>				// needed for memcpy, memset
#include <avr/pgmspace.h>		// needed for exception strings

#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "util.h"
#include "gpio.h"

// NOTE: The canonical machine singleton "cm" would normally be declared here 
// but it's also used by cycles so it's in canonical_machine.h instead.

/* useful macros */
#ifndef ZERO_MODEL_STATE	// isolates this rather dangerous memset
#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#endif
#define _to_millimeters(a) ((gm.inches_mode == TRUE) ? (a * MM_PER_INCH) : a)

/* local function prototypes */
static double _cm_get_move_time();
static void _cm_set_gcode_model_endpoint_position(uint8_t status);
static uint8_t _cm_compute_center_arc(void);
static uint8_t _cm_get_arc_radius(void);
static double _cm_get_arc_time (const double linear_travel, const double angular_travel, const double radius);
static double _cm_get_theta(const double x, const double y);

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
 * cm_get_position() - return position from the gm struct into gn struct form
 * cm_get_gcode_model_position() - return position from the model in internal form
 * cm_get_next_action() - get next_action from the gm struct
 * cm_get_motion_mode() - get motion mode from the gm struct
 * cm_get_inches_mode() - get inches mode from the gm struct
 * cm_get_absolute_mode() - get absolute mode from the gm struct
 * cm_isbusy() - return TRUE if motion control busy (i.e. robot is moving)
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

double *cm_get_gcode_model_position(double position[])
{
	copy_axis_vector(position, gm.position);	
	return (position);
}

uint8_t cm_isbusy() { return (mp_isbusy());}

/*
 * Setters - these inhale gn values into the gm struct
 *
 *	Input coordinates are in native block formats (gn form);
 *	i.e. they are not unit adjusted or otherwise pre-processed.
 *	The setters take care of coordinate system, units, and 
 *	distance mode conversions and normalizations.
 *
 * cm_set_offset()	- set all IJK offsets
 * cm_set_radius()	- set radius value
 * cm_set_absolute_override()
 * cm_set_target()	- set all XYZABC targets
 */

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
 * cm_set_target() - set target vector in GM model
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
		} else if ((cfg.a[i].axis_mode == AXIS_STANDARD) || 
				   (cfg.a[i].axis_mode == AXIS_INHIBITED)) {
			if ((gm.absolute_mode == TRUE) || (gm.absolute_override == TRUE)) {
				gm.target[i] = _to_millimeters(target[i]);
			} else {
				gm.target[i] += _to_millimeters(target[i]);
			}
		} else if (i<A) {
			INFO1(PSTR("%c axis using unsupported axis mode"), cfg_get_configuration_group_char(i));
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
		if ((gm.absolute_mode == TRUE) || (gm.absolute_override == TRUE)) {
			gm.target[i] = tmp;
		} else {
			gm.target[i] += tmp;
		}
	}
}

/* 
 * _cm_set_gcode_model_endpoint_position() - uses internal coordinates only
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

static void _cm_set_gcode_model_endpoint_position(uint8_t status) 
{ 
//	if ((status == TG_OK) || (status == TG_EAGAIN)) {
	if (status == TG_OK) {
		copy_axis_vector(gm.position, gm.target);
	}
}

/* 
 * _cm_get_move_time() - get required time for move
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

static double _cm_get_move_time()
{
	uint8_t i;
	double inv_time=0;	// inverse time if doing a feed in G93 mode
	double xyz_time=0;	// coordinated move linear part at req feed rate
	double abc_time=0;	// coordinated move rotary part at req feed rate
	double max_time=0;	// time required for the rate-limiting axis

	// compute times for feed motion
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
			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) /
									  cfg.a[i].feed_rate_max));
		} else { // gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE
			max_time = max(max_time, (fabs(gm.target[i] - gm.position[i]) /
									  cfg.a[i].seek_rate_max));
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
 * cm_init_canon() 
 *
 *	Most of canonical machine initialization is done thru the config system
 */

void cm_init_canon()
{
	ZERO_MODEL_STATE(&gm);
	ZERO_MODEL_STATE(&gt);
	cfg_init_gcode_model();			// set all the gcode defaults
	cm_init_status_report();
	cm.linecount = 0;
	cm.linenum = 0;
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
	copy_axis_vector(gm.target, gm.position);
	(void)mp_set_axis_position(gm.position);
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
	cm_set_target(target, gf.target);
	cm_cycle_start();							//required for homing & other cycles
	uint8_t status = MP_LINE(gm.target, _cm_get_move_time());
	_cm_set_gcode_model_endpoint_position(status);
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
	uint8_t status;

	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == FALSE) && (gm.feed_rate == 0)) {
		INFO1(PSTR("Attempted move %s with feed rate = zero"), tg.buf);
		return (TG_ZERO_LENGTH_MOVE);
	}

	// Introduce a short dwell if the machine is not busy to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
//	if (st_isbusy() == FALSE) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}

	cm_set_target(target, gf.target);
	cm_cycle_start();							//required for homing & other cycles
	status = MP_LINE(gm.target, _cm_get_move_time());
	_cm_set_gcode_model_endpoint_position(status);
	return (status);
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
 * This group implements stop, start, end, and hold. 
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	cm_cycle_start()			(no Gcode)
 *	cm_program_stop()			(M0, M60)
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	cm_feedhold()				(no Gcode)
 *	cm_abort()					(no Gcode)
 *
 * cm_abort is as close as you can get to an eStop. It shuts things down as
 * quickly as possible and resets the system.
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

uint8_t cm_cycle_start()
{
	// the machine state model simplifies to this:
	if (cm.machine_state == MACHINE_HOLD) {
		cm.machine_state = MACHINE_END_HOLD;
	} else {
		cm.machine_state = MACHINE_RUN;
	}
	return (TG_OK);

/*	Here's more readable but slightly less efficient code:
	switch (cm.machine_state) {
		case (MACHINE_RESET):	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_RUN): 	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_STOP):  	{ cm.machine_state = MACHINE_RUN; break;}
		case (MACHINE_HOLD):	{ cm.machine_state = MACHINE_END_HOLD; break;}
		case (MACHINE_END_HOLD):{ cm.machine_state = MACHINE_RUN; break;}
	}
	return (TG_OK); 
 */
}

uint8_t cm_program_stop()			// M0, M60
{
	mp_queue_program_stop();		// insert a prpgram stop in the queue
	return (TG_OK);					// machine state changes when it's executed
}

uint8_t cm_optional_program_stop()	// M1
{
	mp_queue_program_stop();		// insert a prpgram stop in the queue
	return (TG_OK);					// machine state changes when it's executed
}

uint8_t cm_program_end()			// M2, M30
{
	tg_reset_source();				// stop reading from a file (return to std device)
	mp_queue_program_end();			// insert a prpgram stop in the queue
	return (TG_OK);					// machine state changes when it's executed
}

uint8_t cm_feedhold()
{
	if ((cm.machine_state == MACHINE_RUN) && (cm.hold_state == FEEDHOLD_OFF)) {
		cm.machine_state = MACHINE_HOLD;
		cm.hold_state = FEEDHOLD_SYNC;
	}
	return (TG_OK);
}

uint8_t cm_abort()					// CANNOT BE CALLED FROM AN INTERRUPT
{
//	tg_system_init();
	tg_application_init();
	cm.machine_state = MACHINE_RESET;
	return (TG_OK);
}

uint8_t cm_exec_stop() 
{
	cm.machine_state = MACHINE_STOP;
	return (TG_OK);	
}

uint8_t cm_exec_end() 
{
	cm.machine_state = MACHINE_RESET;
	return (TG_OK);	
}

/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 * _cm_get_arc_radius() 	- compute arc center (offset) from radius.
 * _cm_get_arc_time()
 */

uint8_t cm_arc_feed(double target[],				// arc endpoints
					double i, double j, double k, 	// offsets
					double radius, 					// non-zero sets radius mode
					uint8_t motion_mode)			// defined motion mode
{
	uint8_t status = TG_OK;

	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = motion_mode;

	// trap zero feed rate condition
	if ((gm.inverse_feed_rate_mode == FALSE) && (gm.feed_rate == 0)) {
		INFO1(PSTR("Attempted arc %s with feed rate = zero"), tg.buf);
		return (TG_ZERO_LENGTH_MOVE);
	}

	// set parameters
	cm_set_target(target, gf.target);
	cm_set_offset(i,j,k);
	cm_set_radius(radius);

	// execute the move - non-zero radius is a radius arc
	if (radius > EPSILON) {
		if ((_cm_get_arc_radius() != TG_OK)) {
			return (status);					// error return
		}
	}
	// Introduce a short dwell if the machine is idle to enable the planning
	// queue to begin to fill (avoids first block having to plan down to zero)
//	if (st_isbusy() == FALSE) {
//		cm_dwell(PLANNER_STARTUP_DELAY_SECONDS);
//	}
	status = _cm_compute_center_arc();
	_cm_set_gcode_model_endpoint_position(status);
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

uint8_t _cm_compute_center_arc()
{
	double theta_start;
	double theta_end;
	double radius_tmp;
	double angular_travel;
	double linear_travel;
	double move_time;

	// calculate the theta (angle) of the current point (see header notes)
	theta_start = _cm_get_theta(-gm.offset[gm.plane_axis_0], -gm.offset[gm.plane_axis_1]);
	if(isnan(theta_start) == TRUE) { return(TG_ARC_SPECIFICATION_ERROR);}

	// calculate the theta (angle) of the target point
	theta_end = _cm_get_theta(
		gm.target[gm.plane_axis_0] - gm.offset[gm.plane_axis_0] - gm.position[gm.plane_axis_0], 
 		gm.target[gm.plane_axis_1] - gm.offset[gm.plane_axis_1] - gm.position[gm.plane_axis_1]);
	if(isnan(theta_end) == TRUE) { return (TG_ARC_SPECIFICATION_ERROR); }

	// ensure that the difference is positive so we have clockwise travel
	if (theta_end < theta_start) { theta_end += 2*M_PI; }

	// compute angular travel and invert if gcode wants a counterclockwise arc
	angular_travel = theta_end - theta_start;
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) { angular_travel -= 2*M_PI;}

	// Find the radius, calculate travel in the depth axis of the helix,
	// and compute the time it should take to perform the move
	radius_tmp = hypot(gm.offset[gm.plane_axis_0], gm.offset[gm.plane_axis_1]);
	linear_travel = gm.target[gm.plane_axis_2] - gm.position[gm.plane_axis_2];
	move_time = _cm_get_arc_time(linear_travel, angular_travel, radius_tmp);

	// Trace the arc
	set_vector(gm.target[gm.plane_axis_0],
			   gm.target[gm.plane_axis_1],
			   gm.target[gm.plane_axis_2],
			   gm.target[A], gm.target[B], gm.target[C]);

	return(ar_arc(vector,
				  gm.offset[gm.plane_axis_0],
				  gm.offset[gm.plane_axis_1],
				  gm.offset[gm.plane_axis_2],
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

uint8_t _cm_get_arc_radius()
{
	double x;
	double y;
	double h_x2_div_d;

	// Calculate the change in position along each selected axis
	x = gm.target[gm.plane_axis_0]-gm.position[gm.plane_axis_0];
	y = gm.target[gm.plane_axis_1]-gm.position[gm.plane_axis_1];

	gm.offset[0] = 0;	// reset the offsets
	gm.offset[1] = 0;
	gm.offset[2] = 0;

	// == -(h * 2 / d)
	h_x2_div_d = -sqrt(4 * square(gm.radius) - (square(x) - square(y))) / hypot(x,y);

	// If r is smaller than d the arc is now traversing the complex plane beyond
	// the reach of any real CNC, and thus - for practical reasons - we will 
	// terminate promptly
	if(isnan(h_x2_div_d) == TRUE) { return (TG_FLOATING_POINT_ERROR);}

	// Invert the sign of h_x2_div_d if circle is counter clockwise (see header notes)
	if (gm.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d;}

	// Negative R is g-code-alese for "I want a circle with more than 180 degrees
	// of travel" (go figure!), even though it is advised against ever generating
	// such circles in a single line of g-code. By inverting the sign of 
	// h_x2_div_d the center of the circles is placed on the opposite side of 
	// the line of travel and thus we get the unadvisably long arcs as prescribed.
	if (gm.radius < 0) { h_x2_div_d = -h_x2_div_d; }

	// Complete the operation by calculating the actual center of the arc
	gm.offset[gm.plane_axis_0] = (x-(y*h_x2_div_d))/2;
	gm.offset[gm.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	return (TG_OK);
} 
    
/*
 * _cm_get_arc_time ()
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
 * _cm_get_theta(double x, double y)
 *
 *	Find the angle in radians of deviance from the positive y axis. 
 *	negative angles to the left of y-axis, positive to the right.
 */

static double _cm_get_theta(const double x, const double y)
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

// Independent format strings - not in an array
static char msg_PosX[] PROGMEM = "Position X:   %8.3f %s\n";
static char msg_PosY[] PROGMEM = "Position Y:   %8.3f %s\n";
static char msg_PosZ[] PROGMEM = "Position Z:   %8.3f %s\n";
static char msg_PosA[] PROGMEM = "Position A:   %8.3f degrees\n";
static char msg_PosB[] PROGMEM = "Position B:   %8.3f degrees\n";
static char msg_PosC[] PROGMEM = "Position C:   %8.3f degrees\n";
static char msg_OfsI[] PROGMEM = "Offset I:     %8.3f %s\n";
static char msg_OfsJ[] PROGMEM = "Offset J:     %8.3f %s\n";
static char msg_OfsK[] PROGMEM = "Offset K:     %8.3f %s\n";
static char msg_Feed[] PROGMEM = "Feed Rate:    %8.3f %s \\ min\n";
//static char msg_Limit[] PROGMEM = "Limit Switches: %3.0f %s\n";

// Format strings with indexing arrays
static char msg_g21[] PROGMEM = "Units:           G21 - millimeter mode\n";
static char msg_g20[] PROGMEM = "Units:           G20 - inches mode\n";
static PGM_P msgUnitsMode[] PROGMEM = { msg_g21, msg_g20 };

static char msg_g00[] PROGMEM = "Motion mode:     G0  - linear traverse (seek)\n";
static char msg_g01[] PROGMEM = "Motion mode:     G1  - linear feed\n";
static char msg_g02[] PROGMEM = "Motion mode:     G2  - clockwise arc feed\n";
static char msg_g03[] PROGMEM = "Motion mode:     G3  - counter clockwise arc feed\n";
static char msg_g80[] PROGMEM = "Motion mode:     G80 - cancel motion mode (none active)\n";
static PGM_P msgMotionMode[] PROGMEM = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static char msg_g17[] PROGMEM = "Plane selection: G17 - XY plane\n";
static char msg_g18[] PROGMEM = "Plane selection: G18 - XZ plane\n";
static char msg_g19[] PROGMEM = "Plane selection: G19 - YZ plane\n";
static PGM_P msgPlaneSelect[] PROGMEM = { msg_g17, msg_g18, msg_g19 };

static char msg_g91[] PROGMEM = "Distance mode:   G91 - incremental distance\n";
static char msg_g90[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
static PGM_P msgDistanceMode[] PROGMEM = { msg_g91, msg_g90 };

static char msg_g94[] PROGMEM = "Feed rate mode:  G94 - units per minute\n";
static char msg_g93[] PROGMEM = "Feed rate mode:  G93 - inverse time\n";
static PGM_P msgFeedRateMode[] PROGMEM = { msg_g94, msg_g93 };

static char msg_ms0[] PROGMEM = "Machine state:   Reset\n";
static char msg_ms1[] PROGMEM = "Machine state:   Run\n";
static char msg_ms2[] PROGMEM = "Machine state:   Stop\n";
static char msg_ms3[] PROGMEM = "Machine state:   Feedhold\n";
static char msg_ms4[] PROGMEM = "Machine state:   End Feedhold\n";
static char msg_ms5[] PROGMEM = "Machine state:   Homing\n";
static PGM_P msgMachineState[] PROGMEM = { msg_ms0, msg_ms1, msg_ms2, msg_ms3, msg_ms4, msg_ms5 };

static char st_ms0[] PROGMEM = "\"reset\"";		// used for status reports
static char st_ms1[] PROGMEM = "\"run\"";
static char st_ms2[] PROGMEM = "\"stop\"";
static char st_ms3[] PROGMEM = "\"hold\"";
static char st_ms4[] PROGMEM = "\"resume\"";
static char st_ms5[] PROGMEM = "\"homing\"";
static PGM_P stMachineState[] PROGMEM = { st_ms0, st_ms1, st_ms2, st_ms3, st_ms4, st_ms5 };


void cm_print_machine_state()
{
	double conversion = 1;
	char units[8] = "mm";

	if (gm.inches_mode == true) {
		conversion = (INCH_PER_MM);	
		strncpy(units,"inches", 8);
	}
	mp_get_runtime_position(vector);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgUnitsMode[(gm.inches_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMotionMode[(gm.motion_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgPlaneSelect[(gm.set_plane)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgDistanceMode[(gm.absolute_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgFeedRateMode[(gm.inverse_feed_rate_mode)]));
	fprintf_P(stderr,(PGM_P)msg_Feed, gm.feed_rate * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosX, vector[X] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosY, vector[Y] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosZ, vector[Z] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosA, vector[A]);
	fprintf_P(stderr,(PGM_P)msg_PosB, vector[B]);
	fprintf_P(stderr,(PGM_P)msg_PosC, vector[C]);
	fprintf_P(stderr,(PGM_P)msg_OfsI, gm.offset[0] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsJ, gm.offset[1] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsK, gm.offset[2] * conversion, units);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMachineState[(cm.machine_state)]));
//	fprintf_P(stderr,(PGM_P)msg_Limit, ls.min[X]);
}

/*
 * cm_init_status_report()
 * cm_decr_status_report()
 * cm_try_status_report() 	- send a status report if it's time to send one
 * cm_force_status_report() - force status report to send on next try attempt
 * cm_run_status_report() 	- send a status report
 *
 * Enable can be one of:
 *	0 = disabled
 *	1 = enabled for basic values (xyz, vel, mm)
 *	2 = enabled for extended values (abc)
 */

void cm_init_status_report() 
{ 
	cm.status_report_counter = cfg.status_report_interval; 
}

void cm_decr_status_report() 
{ 
	if (cm.status_report_counter != 0) {	// stick at zero
		cm.status_report_counter--; 
	}
}

void cm_force_status_report()
{
	cm.status_report_counter = 0;
}

uint8_t cm_try_status_report() 
{
	if ((cfg.status_report_enabled >= true) && (cm.status_report_counter == 0)) {
		cm_run_status_report();
		cm.status_report_counter = cfg.status_report_interval;
		return (TG_OK);
	}
	return (TG_NOOP);
}

void cm_run_status_report()
{
	mp_get_runtime_position(vector);
	double velocity = mp_get_runtime_velocity();
	uint8_t distance_mode = 1;			// indicating mm mode

	if (gm.inches_mode == TRUE) {
		vector[X] = vector[X] * (INCH_PER_MM);
		vector[Y] = vector[Y] * (INCH_PER_MM);
		vector[Z] = vector[Z] * (INCH_PER_MM);
		velocity = velocity * (INCH_PER_MM);
		distance_mode = 0;
	}
	fprintf_P(stderr,PSTR("{"));		// first line
	fprintf_P(stderr,PSTR("\"ln\":%1.0f"), mp_get_runtime_linenum()); 	// notice it has no leading comma / space
	fprintf_P(stderr,PSTR(", \"x\":%1.4f"), vector[X]);
	fprintf_P(stderr,PSTR(", \"y\":%1.4f"), vector[Y]);
	fprintf_P(stderr,PSTR(", \"z\":%1.4f"), vector[Z]);

	if (cfg.status_report_enabled > 1) {
		fprintf_P(stderr,PSTR(", \"a\":%1.4f"), vector[A]);
		fprintf_P(stderr,PSTR(", \"b\":%1.4f"), vector[B]);
		fprintf_P(stderr,PSTR(", \"c\":%1.4f"), vector[C]);
	}
	fprintf_P(stderr,PSTR(", \"vel\":%5.2f"), velocity);
	fprintf_P(stderr,PSTR(", \"mm\":%d"), distance_mode);
	fprintf_P(stderr,PSTR(", \"stat\":%S"),(PGM_P)pgm_read_word(&stMachineState[cm.machine_state]));

	fprintf_P(stderr,PSTR("}\n"));		// last line
}

/***********************************************************************/
/*--- CANONICAL MACHINING CYCLES ---*/

uint8_t cm_stop()					// stop cycle. not implemented
{
	return (TG_OK);
}


