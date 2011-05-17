/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S Hart, Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 *
 * TinyG is free software: you can redistribute it and/or modify it 
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
/*
 * The canonical machine is the layer between the Gcode parser and the
 * motion control code for a specific robot. It keeps state and executes
 * commands - passing the simplest caommands it can down to the motion 
 * control layer. See the notes at the end of gcode.h for more details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>		// needed for exception strings
#include <math.h>
#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "planner.h"
#include "canonical_machine.h"
#include "controller.h"
#include "limit_switches.h"
#include "spindle.h"

/* data structures (see notes in gcode.c) */
static struct GCodeModel gm;	// gcode model
static struct GCodeModel gt;	// temp storage for model during cycles
static uint8_t cm_status;

/* useful macros */
#ifndef ZERO_MODEL_STATE
#define ZERO_MODEL_STATE(g) memset(g, 0, sizeof(struct GCodeModel))
#endif
#define _to_millimeters(a) ((gm.inches_mode == TRUE) ? (a * MM_PER_INCH) : a)

/* function prototypes */
//static double _to_millimeters(double value);
static uint8_t _cm_run_homing_cycle(void);
static void _cm_set_endpoint_position(uint8_t status);
static double _theta(double x, double y);
static uint8_t _cm_compute_radius_arc(void);
static uint8_t _cm_compute_center_arc(void);

struct canonicalMachineCycle {		// struct to manage cycles
	uint8_t state;					// cycle state
}; struct canonicalMachineCycle cy;

enum cyCycleState {
	CY_STATE_OFF,					// cycle is OFF (must be zero)
	CY_STATE_NEW,					// initial call to cycle
	CY_STATE_HOMING_X_START,		// start X homing move
	CY_STATE_HOMING_X_WAIT,			// wait for limit switch or end-of-move
	CY_STATE_HOMING_Y_START,
	CY_STATE_HOMING_Y_WAIT,
	CY_STATE_HOMING_Z_START,
	CY_STATE_HOMING_Z_WAIT,
	CY_STATE_HOMING_A_START,
	CY_STATE_HOMING_A_WAIT,
	CY_STATE_HOMING_RTZ_START,		// return to zero move
	CY_STATE_HOMING_RTZ_WAIT,
	CY_STATE_MAX
};

// handy defines
#define ABSOLUTE_MODE 1	
#define INCREMENTAL_MODE 0
#define MILLIMETER_MODE 0
#define INCHES_MODE 1


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
 * cm_get_absolute_mode() - get absolute mode from the gm struct
 * cm_get_position() - return position from the gm struct into gn struct form
 */

inline uint8_t cm_get_next_action() { return gm.next_action; }
inline uint8_t cm_get_motion_mode() { return gm.motion_mode; }
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
 * cm_set_targets()		- set all XYZ targets
 * cm_set_offsets()		- set all IJK offsets
 * cm_set_radius()		- set radius value
 * cm_set_absolute_override()
 */

void cm_set_targets(double x, double y, double z, double a) 
{ 
	if ((gm.absolute_mode == TRUE) || (gm.absolute_override == TRUE)) {
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

void cm_set_offsets(double i, double j, double k) 
{ 
	gm.offset[0] = _to_millimeters(i);
	gm.offset[1] = _to_millimeters(j);
	gm.offset[2] = _to_millimeters(k);
}

inline void cm_set_radius(double r) 
{ 
	gm.radius = _to_millimeters(r);
}

inline void cm_set_absolute_override(uint8_t absolute_override) 
{ 
	gm.absolute_override = absolute_override;
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
//		memcpy(&gm.position, &gm.target, sizeof(gm.target));
		gm.position[X] = gm.target[X];
		gm.position[Y] = gm.target[Y];
		gm.position[Z] = gm.target[Z];
		gm.position[A] = gm.target[A];
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
		return (theta);
	} else {
		if (theta>0) {
			return ( M_PI-theta);
    	} else {
			return (-M_PI-theta);
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
	(void)cm_select_plane(CANON_PLANE_XY);	// default planes, 0, 1 and 2
	gm.inches_mode = FALSE;					// FALSE = mm, TRUE = inches
	gm.absolute_mode = TRUE;				// default to absolute mode (G90)
	(void)cm_set_feed_rate(cfg.gcode_feed_rate);	// set a default
	(void)cm_set_motion_control_mode(cfg.gcode_path_control);
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

	// make the lower layer agree with this
	(void)mp_set_position(gm.position[X], gm.position[Y], gm.position[Z], 
						  gm.position[A]);
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
	uint8_t i;
	double	axis_time;		// time for an axis to make the move
	double	longest_time=0;	// slowest axis in the move (time, not distance)

	// setup current state (gm struct)
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	(void)cm_set_targets(x, y, z, a);
	cm_status = TG_OK;

	// find the slowest axis in the move
	for (i=0; i < AXES; i++) {
		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
		if (longest_time < axis_time) {
			longest_time = axis_time;
		}
	}

	// skip 0 length moves
	if (longest_time < ROUNDING_ERROR) {
		return (TG_ZERO_LENGTH_MOVE);
	}

	// execute the move
	if (cfg.accel_enabled == TRUE) {
		cm_status = mp_aline(gm.target[X], gm.target[Y], gm.target[Z], 
							 gm.target[A], 
//							 gm.target[A],  gm.target[B], gm.target[C], 
							 longest_time);
	} else {
		cm_status = mp_line(gm.target[X], gm.target[Y], gm.target[Z],  
							gm.target[A], 
//							gm.target[A], gm.target[B], gm.target[C], 
							longest_time);
	}
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

uint8_t cm_straight_feed(double x, double y, double z, double a)
{
	uint8_t i;
	double	move_time;	// time to complete move at desired feedrate
	double	axis_time;	// time for an axis to do its part of the move
	double	linear_time;
	double	rotary_time;
	double	fastest_time=0;	// slowest axis time in the move (see notes)

	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
	cm_set_targets(x, y, z, a);

	// Get or compute the time it should take to perform the move in 
	// both linear time and rotational time and take the maximum.
	if (gm.inverse_feed_rate_mode == TRUE) {
		move_time = gm.inverse_feed_rate;
	} else {
		linear_time = sqrt(square(gm.target[X] - gm.position[X]) +
				 		 square(gm.target[Y] - gm.position[Y]) +
						 square(gm.target[Z] - gm.position[Z])) /
						 gm.feed_rate;
		rotary_time = fabs((gm.target[A] - gm.position[A]) / gm.feed_rate);
		move_time = max(linear_time, rotary_time);
	}

	// Compute the fastest time the move can take given all axis limits.
	// "Fastest_time" is the fastest time the coordinated move can be made 
	// given the speed of the slowest axis in the move.
	for (i=0; i<AXES; i++) {
		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
		if (fastest_time < axis_time) {
			fastest_time = axis_time;
		}
	}
	// skip 0 length moves
	if (fastest_time < ROUNDING_ERROR) {
		return (TG_ZERO_LENGTH_MOVE);
	}
	// execute the move
	if (cfg.accel_enabled == TRUE) {
		cm_status = mp_aline(gm.target[X], gm.target[Y], gm.target[Z], 
							 gm.target[A], 
							 max(fastest_time, move_time));
	} else {
		cm_status = mp_line(gm.target[X], gm.target[Y], gm.target[Z], 
							gm.target[A], 
							max(fastest_time, move_time));
	}
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
 *	cm_async_toggle()			(no code)
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

/***********************************************************************/
/*--- CANONICAL MACHINING CYCLES ---*/

uint8_t cm_stop()					// stop cycle. not implemented
{
	return (TG_OK);
}

/* 
 * cm_homing_cycle() - homing cycle using limit switches
 * cm_run_homing_cycle() - continuation for the above
 *
 *	The homing_cycle is coded as a continuation state machine. See end 
 *	notes in controller.c for how to code continuations. It sequences 
 *	through the various homing moves and reacts to limit switch closures. 
 *	It's a bit tricky because the routine can be re-entered if a limit switch
 *	is hit (ANY limit switch) or if the previously queued move completes.
 *
 *	Operation sequence (pseudocode):
 *	  - cm_homing_cycle()
 *		- zero the gcode mode coordinates
 *		- save the current gcode model state (into gt)
 *		- setup for incremental travel & other inits
 *
 *	  - cm_run_homing_cycle()   
 *		  (entered from from the controller loop if no lower-level functions
 *		    are still executing) 
 *		- only run the continuation if state is not OFF, & motors are idle
 *			(i.e. sync execution to the move queue and steppers) 
 *		- for each axis to be homed:
 *		  - issue a move to minus the travel max in that dimension
 *		  - when the move completes or a limit switch is hit backoff from 
 *			that edge by a nominal amount
 *		  - set position using travel offset value (pos'n relative to zero)
 *		- when all axes are homed:
 *		  - restore the previous model state
 *		  - perform a seek from the current position to zero
 *
 *	The continuation is coded as an outer "wrapper" routine and an inner 
 *	routine. The wrapper handles trivial noop cases and translates the return
 *	codes from the lower routines so the continuation sends well-behaved 
 *	return codes back to the controller.
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued 
 *	move per entry into the continuation, then you must exit. The status of 
 *	the call must be communicated back to the controller wrapper, so the
 *	call should be wrapped in a return().
 *
 *	Another Note: When coding a cycle (like this one) you must wait until 
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done (setting cfg.cycle_active = FALSE). Otherwise there
 *	is a nasty race condition in the tg_controller() that will accept the 
 *	next command before the position of the final move has been set.
 *
 *	Cheat: The routine doesn't actually check *which* limit switch was 
 *	hit, just that one was hit. I'm not sure what I'd do anyway if the 
 *	wrong switch was hit. The axis will have stopped anyway as the END 
 *	invoked from the limit switch ISR stops all axes (is not axis specific). 
 *	This may need to be fixed at some point.
 */

#define HOMING_BACKOFF_MOVE cm_straight_traverse
#define HOMING_ZERO_MOVE cm_straight_traverse
//#define HOMING_BACKOFF_MOVE cm_straight_feed
//#define HOMING_ZERO_MOVE cm_straight_feed
#define HOMING_ZERO_RATE 500

uint8_t cm_homing_cycle()
{
	// initialize this whole operation
	cfg.cycle_active = TRUE;			// tell the world you are a Homer
	cfg.homing_state = HOMING_IN_PROCESS;
	(void)cm_set_targets(0, 0, 0, 0);			// this is necessary
	(void)cm_set_origin_offsets(0, 0, 0, 0);	// zero gcode model
	memcpy(&gt, &gm, sizeof(struct GCodeModel));// save gcode model
	(void)cm_use_length_units(MILLIMETER_MODE);
	(void)cm_set_distance_mode(INCREMENTAL_MODE);
	ls_clear_limit_switches();					// reset the switch flags
	cy.state = CY_STATE_NEW;
	return (TG_OK);
}

uint8_t cm_run_homing_cycle()					// outer wrapper
{
	if (cy.state == CY_STATE_OFF) { 
		return (TG_NOOP);
	}
	if (mp_isbusy() == TRUE) {   				// sync to the move queue
		return (TG_EAGAIN); 
	}
	if (_cm_run_homing_cycle() == TG_COMPLETE) { 
		return (TG_OK); 
	} 
	return (TG_EAGAIN);
}

uint8_t _cm_run_homing_cycle()					// inner routine
{
	// handle any initial switch closures by backing off the switch
	if (cy.state == CY_STATE_NEW) {
		cy.state = CY_STATE_HOMING_X_START;
		ls_read_limit_switches();
		if (ls_xmin_thrown() == TRUE) {
			ls_clear_limit_switches();
			return(HOMING_BACKOFF_MOVE(CFG(X).homing_backoff, 0, 0, 0));
		}
		if (ls_ymin_thrown() == TRUE) {
			ls_clear_limit_switches();
			return(HOMING_BACKOFF_MOVE(0, CFG(Y).homing_backoff, 0, 0));
		}
		if (ls_zmin_thrown() == TRUE) {
			ls_clear_limit_switches();
			return(HOMING_BACKOFF_MOVE(0, 0, CFG(Z).homing_backoff, 0));
		}
		if (ls_amin_thrown() == TRUE) {
			ls_clear_limit_switches();
			return(HOMING_BACKOFF_MOVE(0, 0, 0, CFG(A).homing_backoff));
		}
	}

	// if X homing is enabled issue a max_travel X move
	if ((CFG(X).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_X_START)) {
		cy.state = CY_STATE_HOMING_X_WAIT;
		(void)cm_set_feed_rate(CFG(X).homing_rate);
		return(cm_straight_feed(-(CFG(X).travel_max), 0, 0, 0));
	}
	// wait for the end of the X move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_X_WAIT) {
		cy.state = CY_STATE_HOMING_Y_START;
		ls_clear_limit_switches();
		gt.position[X] = CFG(X).homing_offset + CFG(X).homing_backoff;
		return(HOMING_BACKOFF_MOVE(CFG(X).homing_backoff, 0, 0, 0));
	}

	// if Y homing is enabled issue a max_travel Y move
	if ((CFG(Y).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_Y_START)) {
		cy.state = CY_STATE_HOMING_Y_WAIT;
		(void)cm_set_feed_rate(CFG(Y).homing_rate);
		return(cm_straight_feed(0, -(CFG(Y).travel_max), 0, 0));
	}
	// wait for the end of the Y move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_Y_WAIT) {
		cy.state = CY_STATE_HOMING_Z_START;
		ls_clear_limit_switches();
		gt.position[Y] = CFG(Y).homing_offset + CFG(Y).homing_backoff;
		return(HOMING_BACKOFF_MOVE(0, CFG(Y).homing_backoff, 0, 0));
	}

	// if Z homing is enabled issue a max_travel Z move
	if ((CFG(Z).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_Z_START)) {
		cy.state = CY_STATE_HOMING_Z_WAIT;
		(void)cm_set_feed_rate(CFG(Z).homing_rate);
		return(cm_straight_feed(0, 0, -(CFG(Z).travel_max), 0));
	}
	// wait for the end of the Z move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_Z_WAIT) {
		cy.state = CY_STATE_HOMING_A_START;
		ls_clear_limit_switches();
		gt.position[Z] = CFG(Z).homing_offset + CFG(Z).homing_backoff;
		return(HOMING_BACKOFF_MOVE(0, 0, CFG(Z).homing_backoff, 0));
	}

	// if A homing is enabled issue a max_travel A move
	if ((CFG(A).homing_enable  == TRUE) && (cy.state == CY_STATE_HOMING_A_START)) {
		cy.state = CY_STATE_HOMING_A_WAIT;
		(void)cm_set_feed_rate(CFG(A).homing_rate);
		return(cm_straight_feed(0, 0, 0, -(CFG(A).travel_max)));
	}
	// wait for the end of the A move or a limit switch closure
	if (cy.state == CY_STATE_HOMING_A_WAIT) {
		cy.state = CY_STATE_HOMING_RTZ_START;
		ls_clear_limit_switches();
		gt.position[A] = CFG(A).homing_offset + CFG(A).homing_backoff;
		return(HOMING_BACKOFF_MOVE(0, 0, 0, CFG(A).homing_backoff));
	}

	// Return-to-zero move - return to zero and reset the models
	if (cy.state != CY_STATE_HOMING_RTZ_WAIT) {
		cy.state = CY_STATE_HOMING_RTZ_WAIT;
		memcpy(&gm, &gt, sizeof(struct GCodeModel));	// restore gcode model
		(void)mp_set_position(gt.position[X], gt.position[Y], // MP layer must agree...
							  gt.position[Z], gt.position[A]);//...with gt position
		(void)cm_set_distance_mode(ABSOLUTE_MODE);
		(void)cm_set_feed_rate(HOMING_ZERO_RATE);
		return(HOMING_ZERO_MOVE(0,0,0,0));
	}
	// wait until return to zero is complete before releasing the cycle
	if (cy.state == CY_STATE_HOMING_RTZ_WAIT) {
		cfg.cycle_active = FALSE;						// not a homer anymore
		cfg.homing_state = HOMING_COMPLETE;				//...and we're done
		cy.state = CY_STATE_OFF;						//...don't come back
		return (TG_COMPLETE);
	}
	return (TG_OK);
}


/***********************************************************************
 *
 * cm_arc_feed() - G2, G3
 * _cm_compute_radius_arc() - compute arc center (offset) from radius.
 * _cm_compute_center_arc() - compute arc from I and J (arc center point)
 *
 */

uint8_t cm_arc_feed(double x, double y, double z, double a, // endpoints
					double i, double j, double k, 			// offsets
					double radius, 			// non-zero sets radius mode
					uint8_t motion_mode)	// defined motion mode
{
	// copy parameters into the current state
	gm.next_action = NEXT_ACTION_MOTION;
	gm.motion_mode = motion_mode;
	cm_set_targets(x, y, z, a);
	cm_set_offsets(i, j, k);
	cm_set_radius(radius);
	cm_status = TG_OK;

	// execute the move
	if (radius > 0) {
		if ((_cm_compute_radius_arc() != TG_OK)) {
			return (cm_status);						// error return
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
	double mm_of_travel;
	double move_time;
//	double axis_time;

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
//	theta_start = _theta(gm.position[gm.plane_axis_0] - gm.offset[gm.plane_axis_0], 
//					     gm.position[gm.plane_axis_1] - gm.offset[gm.plane_axis_1]);

	if(isnan(theta_start) == TRUE) { 
		cm_status = TG_ARC_SPECIFICATION_ERROR;
		return(cm_status); 
	}

	// calculate the theta (angle) of the target point
	theta_end = _theta
		(gm.target[gm.plane_axis_0] - gm.offset[gm.plane_axis_0] - gm.position[gm.plane_axis_0], 
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

	// get or compute the time it should take to perform the move
	if (gm.inverse_feed_rate_mode == TRUE) {
		move_time = gm.inverse_feed_rate;
	} else {
		mm_of_travel = hypot((angular_travel * radius_tmp), fabs(linear_travel));
		move_time = mm_of_travel / gm.feed_rate;
	}

	// check if any axis requires more time than the move allows
	// +++ proper max velocity checking has been put off for now
//	for (uint8_t i = 0; i < AXES; i++) {
//		axis_time = (fabs(gm.target[i] - gm.position[i])) / CFG(i).max_seek_rate;
//		if (longest_time < axis_time) {
//			longest_time = axis_time;
//		}
//	}

	// Trace the arc
	cm_status = mp_arc(	gm.target[gm.plane_axis_0],
						gm.target[gm.plane_axis_1],
						gm.target[gm.plane_axis_2],
						gm.target[A],
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
 * cm_print_machine_state()
 */

#define GC_MSG_MOTION 0	// these line up with the memory string indexes below
#define GC_MSG_PLANE 5
#define GC_MSG_DISTANCE 8
#define GC_MSG_FEEDRATEMODE 10
#define GC_MSG_UNITS 12
#define GC_MSG_STOP 14

// put display strings in program memory
static char gms00[] PROGMEM = "Motion mode:     G0  - linear traverse (seek)\n";
static char gms01[] PROGMEM = "Motion mode:     G1  - linear feed\n";
static char gms02[] PROGMEM = "Motion mode:     G2  - clockwise arc feed\n";
static char gms03[] PROGMEM = "Motion mode:     G3  - counter clockwise arc feed\n";
static char gms04[] PROGMEM = "Motion mode:     G80 - cancel motion mode (none active)\n";
static char gms05[] PROGMEM = "Plane selection: G17 - XY plane\n";
static char gms06[] PROGMEM = "Plane selection: G18 - XZ plane\n";
static char gms07[] PROGMEM = "Plane selection: G19 - YZ plane\n";
static char gms08[] PROGMEM = "Distance mode:   G91 - incremental distance\n";// This pair is inverted
static char gms09[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
static char gms10[] PROGMEM = "Feed rate mode:  G94 - units per minute\n";	// This pair is inverted
static char gms11[] PROGMEM = "Feed rate mode:  G93 - inverse time\n";
static char gms12[] PROGMEM = "Units:           G21 - millimeters\n";		// This pair is inverted
static char gms13[] PROGMEM = "Units:           G20 - inches\n";
static char gms14[] PROGMEM = "Stop / end:      --  - running\n";
static char gms15[] PROGMEM = "Stop / end:      M0, M1, M30  - stopped\n";
static char gms16[] PROGMEM = "Stop / end:      M2, M60  - end\n";

static char gmsPosX[] PROGMEM = "Position X:   %8.3f %s\n";
static char gmsPosY[] PROGMEM = "Position Y:   %8.3f %s\n";
static char gmsPosZ[] PROGMEM = "Position Z:   %8.3f %s\n";
static char gmsPosA[] PROGMEM = "Position A:   %8.3f degrees\n";
static char gmsOfsI[] PROGMEM = "Offset I:     %8.3f %s\n";
static char gmsOfsJ[] PROGMEM = "Offset J:     %8.3f %s\n";
static char gmsOfsK[] PROGMEM = "Offset K:     %8.3f %s\n";
static char gmsSeek[] PROGMEM = "Seek Rate:    %8.3f %s \\ min\n";
static char gmsFeed[] PROGMEM = "Feed Rate:    %8.3f %s \\ min\n";
static char gmsLimit[] PROGMEM = "Limit Switches: %3.0f %s\n";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
static PGM_P gcMsg[] PROGMEM = {	
	gms00, gms01, gms02, gms03, gms04, gms05, gms06, gms07, gms08, gms09,
	gms10, gms11, gms12, gms13, gms14, gms15, gms16
};

void cm_print_machine_state()
{
	char units[8] = "mm";

	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.motion_mode + GC_MSG_MOTION)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.set_plane + GC_MSG_PLANE)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.absolute_mode + GC_MSG_DISTANCE)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.inverse_feed_rate_mode + GC_MSG_FEEDRATEMODE)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.inches_mode + GC_MSG_UNITS)]));
	fprintf_P(stderr, (PGM_P)pgm_read_word(&gcMsg[(gm.program_flow + GC_MSG_STOP)]));

	if (gm.inches_mode == TRUE) {
		strncpy(units,"inches", 8);
		fprintf_P(stderr, (PGM_P)gmsPosX, gm.position[X] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosY, gm.position[Y] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosZ, gm.position[Z] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsPosA, gm.position[A],"degrees");
		fprintf_P(stderr, (PGM_P)gmsOfsI, gm.offset[0] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsOfsJ, gm.offset[1] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsOfsK, gm.offset[2] / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsSeek, gm.seek_rate / (25.4), units);
		fprintf_P(stderr, (PGM_P)gmsFeed, gm.feed_rate / (25.4), units);
	} else {
		fprintf_P(stderr, (PGM_P)gmsPosX, gm.position[X], units);
		fprintf_P(stderr, (PGM_P)gmsPosY, gm.position[Y], units);
		fprintf_P(stderr, (PGM_P)gmsPosZ, gm.position[Z], units);
		fprintf_P(stderr, (PGM_P)gmsPosA, gm.position[A],"degrees");
		fprintf_P(stderr, (PGM_P)gmsOfsI, gm.offset[0], units);
		fprintf_P(stderr, (PGM_P)gmsOfsJ, gm.offset[1], units);
		fprintf_P(stderr, (PGM_P)gmsOfsK, gm.offset[2], units);
		fprintf_P(stderr, (PGM_P)gmsSeek, gm.seek_rate, units);
		fprintf_P(stderr, (PGM_P)gmsFeed, gm.feed_rate, units);
	}
//	fprintf_P(stderr, (PGM_P)gmsLimit, ls.min[X]);

}
