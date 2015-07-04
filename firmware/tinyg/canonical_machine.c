/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2015 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * 	This code is a loose implementation of Kramer, Proctor and Messina's canonical
 *	machining functions as described in the NIST RS274/NGC v3
 *
 *	The canonical machine is the layer between the Gcode parser and the motion control
 *	code for a specific robot. It keeps state and executes commands - passing the
 *	stateless commands to the motion planning layer.
 */
/* --- System state contexts - Gcode models ---
 *
 *	Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *	There are 3 temporal contexts for system state:
 *	  - The gcode model in the canonical machine (the MODEL context, held in gm)
 *	  - The gcode model used by the planner (PLANNER context, held in bf's and mm)
 *	  - The gcode model used during motion for reporting (RUNTIME context, held in mr)
 *
 *	It's a bit more complicated than this. The 'gm' struct contains the core Gcode model
 *	context. This originates in the canonical machine and is copied to each planner buffer
 *	(bf buffer) during motion planning. Finally, the gm context is passed to the runtime
 *	(mr) for the RUNTIME context. So at last count the Gcode model exists in as many as
 *	30 copies in the system. (1+28+1)
 *
 *	Depending on the need, any one of these contexts may be called for reporting or by
 *	a function. Most typically, all new commends from the gcode parser work form the MODEL
 *	context, and status reports pull from the RUNTIME while in motion, and from MODEL when
 *	at rest. A convenience is provided in the ACTIVE_MODEL pointer to point to the right
 *	context.
 */
/* --- Synchronizing command execution ---
 *
 *	Some gcode commands only set the MODEL state for interpretation of the current Gcode
 *	block. For example, cm_set_feed_rate(). This sets the MODEL so the move time is
 *	properly calculated for the current (and subsequent) blocks, so it's effected
 *	immediately.
 *
 *	"Synchronous commands" are commands that affect the runtime need to be synchronized
 *	with movement. Examples include G4 dwells, program stops and ends, and most M commands.
 *	These are queued into the planner queue and execute from the queue. Synchronous commands
 *	work like this:
 *
 *	  - Call the cm_xxx_xxx() function which will do any input validation and return an
 *		error if it detects one.
 *
 *	  - The cm_ function calls mp_queue_command(). Arguments are a callback to the _exec_...()
 *		function, which is the runtime execution routine, and any arguments that are needed
 *		by the runtime. See typedef for *exec in planner.h for details
 *
 *	  - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *	  - When planner execution reaches the buffer it executes the callback w/ the args.
 *		Take careful note that the callback executes under an interrupt, so beware of
 *		variables that may need to be volatile.
 *
 *	Note:
 *	  - The synchronous command execution mechanism uses 2 vectors in the bf buffer to store
 *		and return values for the callback. It's obvious, but impractical to pass the entire
 *		bf buffer to the callback as some of these commands are actually executed locally
 *		and have no buffer.
 */

#include "tinyg.h"			// #1
#include "config.h"			// #2
#include "text_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "report.h"
#include "gpio.h"
#include "switch.h"
#include "hardware.h"
#include "util.h"
#include "xio.h"			// for serial queue flush
/*
#ifdef __cplusplus
extern "C"{
#endif
*/
/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmSingleton_t cm;		// canonical machine controller singleton

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

// command execution callbacks from planner queue
static void _exec_offset(float *value, float *flag);
static void _exec_change_tool(float *value, float *flag);
static void _exec_select_tool(float *value, float *flag);
static void _exec_mist_coolant_control(float *value, float *flag);
static void _exec_flood_coolant_control(float *value, float *flag);
static void _exec_absolute_origin(float *value, float *flag);
static void _exec_program_finalize(float *value, float *flag);

static int8_t _get_axis(const index_t index);
static int8_t _get_axis_type(const index_t index);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

/********************************
 * Internal getters and setters *
 ********************************/
/*
 * Canonical Machine State functions
 *
 * cm_get_combined_state() - combines raw states into something a user might want to see
 * cm_get_machine_state()
 * cm_get_motion_state()
 * cm_get_cycle_state()
 * cm_get_hold_state()
 * cm_get_homing_state()
 * cm_set_motion_state() - adjusts active model pointer as well
 */
uint8_t cm_get_combined_state()
{
	if (cm.cycle_state == CYCLE_OFF) { cm.combined_state = cm.machine_state;}
	else if (cm.cycle_state == CYCLE_PROBE) { cm.combined_state = COMBINED_PROBE;}
	else if (cm.cycle_state == CYCLE_HOMING) { cm.combined_state = COMBINED_HOMING;}
	else if (cm.cycle_state == CYCLE_JOG) { cm.combined_state = COMBINED_JOG;}
	else {
		if (cm.motion_state == MOTION_RUN) cm.combined_state = COMBINED_RUN;
		if (cm.motion_state == MOTION_HOLD) cm.combined_state = COMBINED_HOLD;
	}
	if (cm.machine_state == MACHINE_SHUTDOWN) { cm.combined_state = COMBINED_SHUTDOWN;}

	return cm.combined_state;
}

uint8_t cm_get_machine_state() { return cm.machine_state;}
uint8_t cm_get_cycle_state() { return cm.cycle_state;}
uint8_t cm_get_motion_state() { return cm.motion_state;}
uint8_t cm_get_hold_state() { return cm.hold_state;}
uint8_t cm_get_homing_state() { return cm.homing_state;}

void cm_set_motion_state(uint8_t motion_state)
{
	cm.motion_state = motion_state;

	switch (motion_state) {
		case (MOTION_STOP): { ACTIVE_MODEL = MODEL; break; }
		case (MOTION_RUN):  { ACTIVE_MODEL = RUNTIME; break; }
		case (MOTION_HOLD): { ACTIVE_MODEL = RUNTIME; break; }
	}
}

/***********************************
 * Model State Getters and Setters *
 ***********************************/

/*	These getters and setters will work on any gm model with inputs:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		PLANNER		(GCodeState_t *)&bf->gm		// relative to buffer *bf is currently pointing to
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *		ACTIVE_MODEL cm.am						// active model pointer is maintained by state management
 */
uint32_t cm_get_linenum(GCodeState_t *gcode_state) { return gcode_state->linenum;}
uint8_t cm_get_motion_mode(GCodeState_t *gcode_state) { return gcode_state->motion_mode;}
uint8_t cm_get_coord_system(GCodeState_t *gcode_state) { return gcode_state->coord_system;}
uint8_t cm_get_units_mode(GCodeState_t *gcode_state) { return gcode_state->units_mode;}
uint8_t cm_get_select_plane(GCodeState_t *gcode_state) { return gcode_state->select_plane;}
uint8_t cm_get_path_control(GCodeState_t *gcode_state) { return gcode_state->path_control;}
uint8_t cm_get_distance_mode(GCodeState_t *gcode_state) { return gcode_state->distance_mode;}
uint8_t cm_get_feed_rate_mode(GCodeState_t *gcode_state) { return gcode_state->feed_rate_mode;}
uint8_t cm_get_tool(GCodeState_t *gcode_state) { return gcode_state->tool;}
uint8_t cm_get_spindle_mode(GCodeState_t *gcode_state) { return gcode_state->spindle_mode;}
uint8_t	cm_get_block_delete_switch() { return cm.gmx.block_delete_switch;}
uint8_t cm_get_runtime_busy() { return (mp_get_runtime_busy());}

float cm_get_feed_rate(GCodeState_t *gcode_state) { return gcode_state->feed_rate;}

void cm_set_motion_mode(GCodeState_t *gcode_state, uint8_t motion_mode) { gcode_state->motion_mode = motion_mode;}
void cm_set_spindle_mode(GCodeState_t *gcode_state, uint8_t spindle_mode) { gcode_state->spindle_mode = spindle_mode;}
void cm_set_spindle_speed_parameter(GCodeState_t *gcode_state, float speed) { gcode_state->spindle_speed = speed;}
void cm_set_tool_number(GCodeState_t *gcode_state, uint8_t tool) { gcode_state->tool = tool;}

void cm_set_absolute_override(GCodeState_t *gcode_state, uint8_t absolute_override)
{
	gcode_state->absolute_override = absolute_override;
	cm_set_work_offsets(MODEL);				// must reset offsets if you change absolute override
}

void cm_set_model_linenum(uint32_t linenum)
{
	cm.gm.linenum = linenum;				// you must first set the model line number,
	nv_add_object((const char_t *)"n");	// then add the line number to the nv list
}

/***********************************************************************************
 * COORDINATE SYSTEMS AND OFFSETS
 * Functions to get, set and report coordinate systems and work offsets
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/
/*
 * Notes on Coordinate System and Offset functions
 *
 * All positional information in the canonical machine is kept as absolute coords and in
 *	canonical units (mm). The offsets are only used to translate in and out of canonical form
 *	during interpretation and response.
 *
 * Managing the coordinate systems & offsets is somewhat complicated. The following affect offsets:
 *	- coordinate system selected. 1-9 correspond to G54-G59
 *	- absolute override: forces current move to be interpreted in machine coordinates: G53 (system 0)
 *	- G92 offsets are added "on top of" the coord system offsets -- if origin_offset_enable == true
 *	- G28 and G30 moves; these are run in absolute coordinates
 *
 * The offsets themselves are considered static, are kept in cm, and are supposed to be persistent.
 *
 * To reduce complexity and data load the following is done:
 *	- Full data for coordinates/offsets is only accessible by the canonical machine, not the downstream
 *	- A fully resolved set of coord and G92 offsets, with per-move exceptions can be captured as "work_offsets"
 *	- The core gcode context (gm) only knows about the active coord system and the work offsets
 */

/*
 * cm_get_active_coord_offset() - return the currently active coordinate offset for an axis
 *
 *	Takes G5x, G92 and absolute override into account to return the active offset for this move
 *
 *	This function is typically used to evaluate and set offsets, as opposed to cm_get_work_offset()
 *	which merely returns what's in the work_offset[] array.
 */

float cm_get_active_coord_offset(uint8_t axis)
{
	if (cm.gm.absolute_override == true) return (0);		// no offset if in absolute override mode
	float offset = cm.offset[cm.gm.coord_system][axis];
	if (cm.gmx.origin_offset_enable == true)
		offset += cm.gmx.origin_offset[axis];				// includes G5x and G92 components
	return (offset);
}

/*
 * cm_get_work_offset() - return a coord offset from the gcode_state
 *
 *	This function accepts as input:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		PLANNER		(GCodeState_t *)&bf->gm		// relative to buffer *bf is currently pointing to
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *		ACTIVE_MODEL cm.am						// active model pointer is maintained by state management
 */

float cm_get_work_offset(GCodeState_t *gcode_state, uint8_t axis)
{
	return (gcode_state->work_offset[axis]);
}

/*
 * cm_set_work_offsets() - capture coord offsets from the model into absolute values in the gcode_state
 *
 *	This function accepts as input:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		PLANNER		(GCodeState_t *)&bf->gm		// relative to buffer *bf is currently pointing to
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *		ACTIVE_MODEL cm.am						// active model pointer is maintained by state management
 */

void cm_set_work_offsets(GCodeState_t *gcode_state)
{
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		gcode_state->work_offset[axis] = cm_get_active_coord_offset(axis);
	}
}

/*
 * cm_get_absolute_position() - get position of axis in absolute coordinates
 *
 *	This function accepts as input:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *
 *	NOTE: Only MODEL and RUNTIME are supported (no PLANNER or bf's)
 *	NOTE: Machine position is always returned in mm mode. No units conversion is performed
 */

float cm_get_absolute_position(GCodeState_t *gcode_state, uint8_t axis)
{
	if (gcode_state == MODEL) return (cm.gmx.position[axis]);
	return (mp_get_runtime_absolute_position(axis));
}

/*
 * cm_get_work_position() - return work position in external form
 *
 *	... that means in prevailing units (mm/inch) and with all offsets applied
 *
 * NOTE: This function only works after the gcode_state struct as had the work_offsets setup by
 *		 calling cm_get_model_coord_offset_vector() first.
 *
 *	This function accepts as input:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *
 * NOTE: Only MODEL and RUNTIME are supported (no PLANNER or bf's)
 */

float cm_get_work_position(GCodeState_t *gcode_state, uint8_t axis)
{
	float position;

	if (gcode_state == MODEL) {
		position = cm.gmx.position[axis] - cm_get_active_coord_offset(axis);
	} else {
		position = mp_get_runtime_work_position(axis);
	}
	if (gcode_state->units_mode == INCHES) { position /= MM_PER_INCH; }
	return (position);
}

/***********************************************************************************
 * CRITICAL HELPERS
 * Core functions supporting the canonical machining functions
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/
/*
 * cm_finalize_move() - perform final operations for a traverse or feed
 * cm_update_model_position_from_runtime() - set endpoint position from final runtime position
 *
 * 	These routines set the point position in the gcode model.
 *
 * 	Note: As far as the canonical machine is concerned the final position of a Gcode block (move)
 *	is achieved as soon as the move is planned and the move target becomes the new model position.
 *	In reality the planner will (in all likelihood) have only just queued the move for later
 *	execution, and the real tool position is still close to the starting point.
 */

void cm_finalize_move() {
	copy_vector(cm.gmx.position, cm.gm.target);		// update model position

	// if in ivnerse time mode reset feed rate so next block requires an explicit feed rate setting
	if ((cm.gm.feed_rate_mode == INVERSE_TIME_MODE) && (cm.gm.motion_mode == MOTION_MODE_STRAIGHT_FEED)) {
		cm.gm.feed_rate = 0;
	}
}

void cm_update_model_position_from_runtime() { copy_vector(cm.gmx.position, mr.gm.target); }

/*
 * cm_deferred_write_callback() - write any changed G10 values back to persistence
 *
 *	Only runs if there is G10 data to write, there is no movement, and the serial queues are quiescent
 *	This could be made tighter by issuing an XOFF or ~CTS beforehand and releasing it afterwards.
 */

stat_t cm_deferred_write_callback()
{
	if ((cm.cycle_state == CYCLE_OFF) && (cm.deferred_write_flag == true)) {
#ifdef __AVR
		if (xio_isbusy()) return (STAT_OK);		// don't write back if serial RX is not empty
#endif
		cm.deferred_write_flag = false;
		nvObj_t nv;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf((char *)nv.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				nv.index = nv_get_index((const char_t *)"", nv.token);
				nv.value = cm.offset[i][j];
				nv_persist(&nv);				// Note: only writes values that have changed
			}
		}
	}
	return (STAT_OK);
}

/*
 * cm_set_model_target() - set target vector in GM model
 *
 * This is a core routine. It handles:
 *	- conversion of linear units to internal canonical form (mm)
 *	- conversion of relative mode to absolute (internal canonical form)
 *	- translation of work coordinates to machine coordinates (internal canonical form)
 *	- computation and application of axis modes as so:
 *
 *	DISABLED  - Incoming value is ignored. Target value is not changed
 *	ENABLED	  - Convert axis values to canonical format and store as target
 *	INHIBITED - Same processing as ENABLED, but axis will not actually be run
 * 	RADIUS	  - ABC axis value is provided in Gcode block in linear units
 *			  - Target is set to degrees based on axis' Radius value
 *			  - Radius mode is only processed for ABC axes. Application to XYZ is ignored.
 *
 *	Target coordinates are provided in target[]
 *	Axes that need processing are signaled in flag[]
 */

// ESTEE: _calc_ABC is a fix to workaround a gcc compiler bug wherein it runs out of spill
//        registers we moved this block into its own function so that we get a fresh stack push
// ALDEN: This shows up in avr-gcc 4.7.0 and avr-libc 1.8.0

static float _calc_ABC(uint8_t axis, float target[], float flag[])
{
	if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
		return(target[axis]);	// no mm conversion - it's in degrees
	}
	return(_to_millimeters(target[axis]) * 360 / (2 * M_PI * cm.a[axis].radius));
}

void cm_set_model_target(float target[], float flag[])
{
	uint8_t axis;
	float tmp = 0;

	// process XYZABC for lower modes
	for (axis=AXIS_X; axis<=AXIS_Z; axis++) {
		if ((fp_FALSE(flag[axis])) || (cm.a[axis].axis_mode == AXIS_DISABLED)) {
			continue;		// skip axis if not flagged for update or its disabled
		} else if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
			if (cm.gm.distance_mode == ABSOLUTE_MODE) {
				cm.gm.target[axis] = cm_get_active_coord_offset(axis) + _to_millimeters(target[axis]);
			} else {
				cm.gm.target[axis] += _to_millimeters(target[axis]);
			}
		}
	}
	// FYI: The ABC loop below relies on the XYZ loop having been run first
	for (axis=AXIS_A; axis<=AXIS_C; axis++) {
		if ((fp_FALSE(flag[axis])) || (cm.a[axis].axis_mode == AXIS_DISABLED)) {
			continue;		// skip axis if not flagged for update or its disabled
		} else {
			tmp = _calc_ABC(axis, target, flag);
		}
		if (cm.gm.distance_mode == ABSOLUTE_MODE) {
			cm.gm.target[axis] = tmp + cm_get_active_coord_offset(axis); // sacidu93's fix to Issue #22
		} else {
			cm.gm.target[axis] += tmp;
		}
	}
}

/*
 * cm_test_soft_limits() - return error code if soft limit is exceeded
 *
 *	Must be called with target properly set in GM struct. Best done after cm_set_model_target().
 *
 *	Tests for soft limit for any homed axis if min and max are different values. You can set min
 *	and max to 0,0 to disable soft limits for an axis. Also will not test a min or a max if the
 *	value is < -1000000 (negative one million). This allows a single end to be tested w/the other
 *	disabled, should that requirement ever arise.
 */
stat_t cm_test_soft_limits(float target[])
{
	if (cm.soft_limit_enable == true) {
		for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
			if (cm.homed[axis] != true) continue;		// don't test axes that are not homed

			if (fp_EQ(cm.a[axis].travel_min, cm.a[axis].travel_max)) continue;

			if ((cm.a[axis].travel_min > DISABLE_SOFT_LIMIT) && (target[axis] < cm.a[axis].travel_min)) {
				return (STAT_SOFT_LIMIT_EXCEEDED);
			}

			if ((cm.a[axis].travel_max > DISABLE_SOFT_LIMIT) && (target[axis] > cm.a[axis].travel_max)) {
				return (STAT_SOFT_LIMIT_EXCEEDED);
			}
		}
	}
	return (STAT_OK);
}

/*************************************************************************
 * CANONICAL MACHINING FUNCTIONS
 *	Values are passed in pre-unit_converted state (from gn structure)
 *	All operations occur on gm (current model state)
 *
 * These are organized by section number (x.x.x) in the order they are
 * found in NIST RS274 NGCv3
 ************************************************************************/

/******************************************
 * Initialization and Termination (4.3.2) *
 ******************************************/
/*
 * canonical_machine_init() - Config init cfg_init() must have been run beforehand
 */

void canonical_machine_init()
{
// If you can assume all memory has been zeroed by a hard reset you don't need this code:
//	memset(&cm, 0, sizeof(cm));					// do not reset canonicalMachineSingleton once it's been initialized
	memset(&cm.gm, 0, sizeof(GCodeState_t));	// clear all values, pointers and status
	memset(&cm.gn, 0, sizeof(GCodeInput_t));
	memset(&cm.gf, 0, sizeof(GCodeInput_t));

	canonical_machine_init_assertions();		// establish assertions
	ACTIVE_MODEL = MODEL;						// setup initial Gcode model pointer

	// set gcode defaults
	cm_set_units_mode(cm.units_mode);
	cm_set_coord_system(cm.coord_system);
	cm_select_plane(cm.select_plane);
	cm_set_path_control(cm.path_control);
	cm_set_distance_mode(cm.distance_mode);
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);// always the default

	cm.gmx.block_delete_switch = true;

	// never start a machine in a motion mode
	cm.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;

	// reset request flags
	cm.feedhold_requested = false;
	cm.queue_flush_requested = false;
	cm.cycle_start_requested = false;

	// signal that the machine is ready for action
	cm.machine_state = MACHINE_READY;
	cm.combined_state = COMBINED_READY;

	// sub-system inits
	cm_spindle_init();
	cm_arc_init();
}

/*
 * canonical_machine_init_assertions()
 * canonical_machine_test_assertions() - test assertions, return error code if violation exists
 */

void canonical_machine_init_assertions(void)
{
	cm.magic_start = MAGICNUM;
	cm.magic_end = MAGICNUM;
	cm.gmx.magic_start = MAGICNUM;
	cm.gmx.magic_end = MAGICNUM;
	arc.magic_start = MAGICNUM;
	arc.magic_end = MAGICNUM;
}

stat_t canonical_machine_test_assertions(void)
{
	if ((cm.magic_start 	!= MAGICNUM) || (cm.magic_end 	  != MAGICNUM)) return (STAT_CANONICAL_MACHINE_ASSERTION_FAILURE);
	if ((cm.gmx.magic_start != MAGICNUM) || (cm.gmx.magic_end != MAGICNUM)) return (STAT_CANONICAL_MACHINE_ASSERTION_FAILURE);
	if ((arc.magic_start 	!= MAGICNUM) || (arc.magic_end    != MAGICNUM)) return (STAT_CANONICAL_MACHINE_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * cm_soft_alarm() - alarm state; send an exception report and stop processing input
 * cm_clear() 	   - clear soft alarm
 * cm_hard_alarm() - alarm state; send an exception report and shut down machine
 */

stat_t cm_soft_alarm(stat_t status)
{
	rpt_exception(status);					// send alarm message
	cm.machine_state = MACHINE_ALARM;
	return (status);						// NB: More efficient than inlining rpt_exception() call.
}

stat_t cm_clear(nvObj_t *nv)				// clear soft alarm
{
	if (cm.cycle_state == CYCLE_OFF) {
		cm.machine_state = MACHINE_PROGRAM_STOP;
	} else {
		cm.machine_state = MACHINE_CYCLE;
	}
	return (STAT_OK);
}

stat_t cm_hard_alarm(stat_t status)
{
	// stop the motors and the spindle
	stepper_init();							// hard stop
	cm_spindle_control(SPINDLE_OFF);

	// disable all MCode functions
//	gpio_set_bit_off(SPINDLE_BIT);			//++++ this current stuff is temporary
//	gpio_set_bit_off(SPINDLE_DIR);
//	gpio_set_bit_off(SPINDLE_PWM);
//	gpio_set_bit_off(MIST_COOLANT_BIT);		//++++ replace with exec function
//	gpio_set_bit_off(FLOOD_COOLANT_BIT);	//++++ replace with exec function

	rpt_exception(status);					// send shutdown message
	cm.machine_state = MACHINE_SHUTDOWN;
	return (status);
}

/**************************
 * Representation (4.3.3) *
 **************************/

/**************************************************************************
 * Representation functions that affect the Gcode model only (asynchronous)
 *
 *	cm_select_plane()			- G17,G18,G19 select axis plane
 *	cm_set_units_mode()			- G20, G21
 *	cm_set_distance_mode()		- G90, G91
 *	cm_set_coord_offsets()		- G10 (delayed persistence)
 *
 *	These functions assume input validation occurred upstream.
 */

stat_t cm_select_plane(uint8_t plane)
{
	cm.gm.select_plane = plane;
	return (STAT_OK);
}

stat_t cm_set_units_mode(uint8_t mode)
{
	cm.gm.units_mode = mode;		// 0 = inches, 1 = mm.
	return(STAT_OK);
}

stat_t cm_set_distance_mode(uint8_t mode)
{
	cm.gm.distance_mode = mode;		// 0 = absolute mode, 1 = incremental
	return (STAT_OK);
}

/*
 * cm_set_coord_offsets() - G10 L2 Pn (affects MODEL only)
 *
 *	This function applies the offset to the GM model but does not persist the offsets
 *	during the Gcode cycle. The persist flag is used to persist offsets once the cycle
 *	has ended. You can also use $g54x - $g59c config functions to change offsets.
 *
 *	It also does not reset the work_offsets which may be accomplished by calling
 *	cm_set_work_offsets() immediately afterwards.
 */

stat_t cm_set_coord_offsets(uint8_t coord_system, float offset[], float flag[])
{
	if ((coord_system < G54) || (coord_system > COORD_SYSTEM_MAX)) {	// you can't set G53
		return (STAT_INPUT_VALUE_RANGE_ERROR);
	}
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (fp_TRUE(flag[axis])) {
			cm.offset[coord_system][axis] = _to_millimeters(offset[axis]);
			cm.deferred_write_flag = true;								// persist offsets once machining cycle is over
		}
	}
	return (STAT_OK);
}

/******************************************************************************************
 * Representation functions that affect gcode model and are queued to planner (synchronous)
 */
/*
 * cm_set_coord_system() - G54-G59
 * _exec_offset() - callback from planner
 */
stat_t cm_set_coord_system(uint8_t coord_system)
{
	cm.gm.coord_system = coord_system;

	float value[AXES] = { (float)coord_system,0,0,0,0,0 };	// pass coordinate system in value[0] element
	mp_queue_command(_exec_offset, value, value);			// second vector (flags) is not used, so fake it
	return (STAT_OK);
}

static void _exec_offset(float *value, float *flag)
{
	uint8_t coord_system = ((uint8_t)value[0]);				// coordinate system is passed in value[0] element
	float offsets[AXES];
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		offsets[axis] = cm.offset[coord_system][axis] + (cm.gmx.origin_offset[axis] * cm.gmx.origin_offset_enable);
	}
	mp_set_runtime_work_offset(offsets);
	cm_set_work_offsets(MODEL);								// set work offsets in the Gcode model
}

/*
 * cm_set_position() - set the position of a single axis in the model, planner and runtime
 *
 *	This command sets an axis/axes to a position provided as an argument.
 *	This is useful for setting origins for homing, probing, and other operations.
 *
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	!!!!! DO NOT CALL THIS FUNCTION WHILE IN A MACHINING CYCLE !!!!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *	More specifically, do not call this function if there are any moves in the planner or
 *	if the runtime is moving. The system must be quiescent or you will introduce positional
 *	errors. This is true because the planned / running moves have a different reference frame
 *	than the one you are now going to set. These functions should only be called during
 *	initialization sequences and during cycles (such as homing cycles) when you know there
 *	are no more moves in the planner and that all motion has stopped.
 *	Use cm_get_runtime_busy() to be sure the system is quiescent.
 */

void cm_set_position(uint8_t axis, float position)
{
	// TODO: Interlock involving runtime_busy test
	cm.gmx.position[axis] = position;
	cm.gm.target[axis] = position;
	mp_set_planner_position(axis, position);
	mp_set_runtime_position(axis, position);
	mp_set_steps_to_runtime_position();
}

/*** G28.3 functions and support ***
 *
 * cm_set_absolute_origin() - G28.3 - model, planner and queue to runtime
 * _exec_absolute_origin()  - callback from planner
 *
 *	cm_set_absolute_origin() takes a vector of origins (presumably 0's, but not necessarily)
 *	and applies them to all axes where the corresponding position in the flag vector is true (1).
 *
 *	This is a 2 step process. The model and planner contexts are set immediately, the runtime
 *	command is queued and synchronized with the planner queue. This includes the runtime position
 *	and the step recording done by the encoders. At that point any axis that is set is also marked
 *	as homed.
 */

stat_t cm_set_absolute_origin(float origin[], float flag[])
{
	float value[AXES];

	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (fp_TRUE(flag[axis])) {
			value[axis] = _to_millimeters(origin[axis]);
			cm.gmx.position[axis] = value[axis];		// set model position
			cm.gm.target[axis] = value[axis];			// reset model target
			mp_set_planner_position(axis, value[axis]);	// set mm position
		}
	}
	mp_queue_command(_exec_absolute_origin, value, flag);
	return (STAT_OK);
}

static void _exec_absolute_origin(float *value, float *flag)
{
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (fp_TRUE(flag[axis])) {
			mp_set_runtime_position(axis, value[axis]);
			cm.homed[axis] = true;	// G28.3 is not considered homed until you get here
		}
	}
	mp_set_steps_to_runtime_position();
}

/*
 * cm_set_origin_offsets() 		- G92
 * cm_reset_origin_offsets() 	- G92.1
 * cm_suspend_origin_offsets() 	- G92.2
 * cm_resume_origin_offsets() 	- G92.3
 *
 * G92's behave according to NIST 3.5.18 & LinuxCNC G92
 * http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G92-G92.1-G92.2-G92.3
 */
stat_t cm_set_origin_offsets(float offset[], float flag[])
{
	// set offsets in the Gcode model extended context
	cm.gmx.origin_offset_enable = 1;
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (fp_TRUE(flag[axis])) {
			cm.gmx.origin_offset[axis] = cm.gmx.position[axis] -
									  cm.offset[cm.gm.coord_system][axis] - _to_millimeters(offset[axis]);
		}
	}
	// now pass the offset to the callback - setting the coordinate system also applies the offsets
	float value[AXES] = { (float)cm.gm.coord_system,0,0,0,0,0 }; // pass coordinate system in value[0] element
	mp_queue_command(_exec_offset, value, value);				  // second vector is not used
	return (STAT_OK);
}

stat_t cm_reset_origin_offsets()
{
	cm.gmx.origin_offset_enable = 0;
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm.gmx.origin_offset[axis] = 0;
	}
	float value[AXES] = { (float)cm.gm.coord_system,0,0,0,0,0 };
	mp_queue_command(_exec_offset, value, value);
	return (STAT_OK);
}

stat_t cm_suspend_origin_offsets()
{
	cm.gmx.origin_offset_enable = 0;
	float value[AXES] = { (float)cm.gm.coord_system,0,0,0,0,0 };
	mp_queue_command(_exec_offset, value, value);
	return (STAT_OK);
}

stat_t cm_resume_origin_offsets()
{
	cm.gmx.origin_offset_enable = 1;
	float value[AXES] = { (float)cm.gm.coord_system,0,0,0,0,0 };
	mp_queue_command(_exec_offset, value, value);
	return (STAT_OK);
}

/*****************************
 * Free Space Motion (4.3.4) *
 *****************************/
/*
 * cm_straight_traverse() - G0 linear rapid
 */

stat_t cm_straight_traverse(float target[], float flags[])
{
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_model_target(target, flags);

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) return (cm_soft_alarm(status));

	// prep and plan the move
	cm_set_work_offsets(&cm.gm);				// capture the fully resolved offsets to the state
	cm_cycle_start();							// required for homing & other cycles
	mp_aline(&cm.gm);							// send the move to the planner
	cm_finalize_move();
	return (STAT_OK);
}

/*
 * cm_set_g28_position()  - G28.1
 * cm_goto_g28_position() - G28
 * cm_set_g30_position()  - G30.1
 * cm_goto_g30_position() - G30
 */

stat_t cm_set_g28_position(void)
{
	copy_vector(cm.gmx.g28_position, cm.gmx.position);
	return (STAT_OK);
}

stat_t cm_goto_g28_position(float target[], float flags[])
{
	cm_set_absolute_override(MODEL, true);
	cm_straight_traverse(target, flags);			 // move through intermediate point, or skip
	while (mp_get_planner_buffers_available() == 0); // make sure you have an available buffer
	float f[] = {1,1,1,1,1,1};
	return(cm_straight_traverse(cm.gmx.g28_position, f));// execute actual stored move
}

stat_t cm_set_g30_position(void)
{
	copy_vector(cm.gmx.g30_position, cm.gmx.position);
	return (STAT_OK);
}

stat_t cm_goto_g30_position(float target[], float flags[])
{
	cm_set_absolute_override(MODEL, true);
	cm_straight_traverse(target, flags);			 // move through intermediate point, or skip
	while (mp_get_planner_buffers_available() == 0); // make sure you have an available buffer
	float f[] = {1,1,1,1,1,1};
	return(cm_straight_traverse(cm.gmx.g30_position, f));// execute actual stored move
}

/********************************
 * Machining Attributes (4.3.5) *
 ********************************/
/*
 * cm_set_feed_rate() - F parameter (affects MODEL only)
 *
 * Normalize feed rate to mm/min or to minutes if in inverse time mode
 */

stat_t cm_set_feed_rate(float feed_rate)
{
	if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {
		cm.gm.feed_rate = 1 / feed_rate;	// normalize to minutes (NB: active for this gcode block only)
	} else {
		cm.gm.feed_rate = _to_millimeters(feed_rate);
	}
	return (STAT_OK);
}

/*
 * cm_set_feed_rate_mode() - G93, G94 (affects MODEL only)
 *
 *	INVERSE_TIME_MODE = 0,			// G93
 *	UNITS_PER_MINUTE_MODE,			// G94
 *	UNITS_PER_REVOLUTION_MODE		// G95 (unimplemented)
 */

stat_t cm_set_feed_rate_mode(uint8_t mode)
{
	cm.gm.feed_rate_mode = mode;
	return (STAT_OK);
}

/*
 * cm_set_path_control() - G61, G61.1, G64 (affects MODEL only)
 */

stat_t cm_set_path_control(uint8_t mode)
{
	cm.gm.path_control = mode;
	return (STAT_OK);
}

/*******************************
 * Machining Functions (4.3.6) *
 *******************************/
/*
 * cm_arc_feed() - SEE plan_arc.c(pp)
 */

/*
 * cm_dwell() - G4, P parameter (seconds)
 */
stat_t cm_dwell(float seconds)
{
	cm.gm.parameter = seconds;
	mp_dwell(seconds);
	return (STAT_OK);
}

/*
 * cm_straight_feed() - G1
 */
stat_t cm_straight_feed(float target[], float flags[])
{
	// trap zero feed rate condition
	if ((cm.gm.feed_rate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feed_rate))) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
	cm_set_model_target(target, flags);

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) return (cm_soft_alarm(status));

	// prep and plan the move
	cm_set_work_offsets(&cm.gm);				// capture the fully resolved offsets to the state
	cm_cycle_start();							// required for homing & other cycles
	status = mp_aline(&cm.gm);					// send the move to the planner
	cm_finalize_move();
	return (status);
}

/*****************************
 * Spindle Functions (4.3.7) *
 *****************************/
// see spindle.c, spindle.h

/**************************
 * Tool Functions (4.3.8) *
 **************************/
/*
 * cm_select_tool()		- T parameter
 * _exec_select_tool()	- execution callback
 *
 * cm_change_tool()		- M6 (This might become a complete tool change cycle)
 * _exec_change_tool()	- execution callback
 *
 * Note: These functions don't actually do anything for now, and there's a bug
 *		 where T and M in different blocks don;t work correctly
 */
stat_t cm_select_tool(uint8_t tool_select)
{
	float value[AXES] = { (float)tool_select,0,0,0,0,0 };
	mp_queue_command(_exec_select_tool, value, value);
	return (STAT_OK);
}

static void _exec_select_tool(float *value, float *flag)
{
	cm.gm.tool_select = (uint8_t)value[0];
}

stat_t cm_change_tool(uint8_t tool_change)
{
	float value[AXES] = { (float)cm.gm.tool_select,0,0,0,0,0 };
	mp_queue_command(_exec_change_tool, value, value);
	return (STAT_OK);
}

static void _exec_change_tool(float *value, float *flag)
{
	cm.gm.tool = (uint8_t)value[0];
}

/***********************************
 * Miscellaneous Functions (4.3.9) *
 ***********************************/
/*
 * cm_mist_coolant_control() - M7
 * cm_flood_coolant_control() - M8, M9
 */

stat_t cm_mist_coolant_control(uint8_t mist_coolant)
{
	float value[AXES] = { (float)mist_coolant,0,0,0,0,0 };
	mp_queue_command(_exec_mist_coolant_control, value, value);
	return (STAT_OK);
}
static void _exec_mist_coolant_control(float *value, float *flag)
{
	cm.gm.mist_coolant = (uint8_t)value[0];

#ifdef __AVR
	if (cm.gm.mist_coolant == true)
		gpio_set_bit_on(MIST_COOLANT_BIT);	// if
	gpio_set_bit_off(MIST_COOLANT_BIT);		// else
#endif // __AVR

#ifdef __ARM
	if (cm.gm.mist_coolant == true)
		coolant_enable_pin.set();	// if
	coolant_enable_pin.clear();		// else
#endif // __ARM
}

stat_t cm_flood_coolant_control(uint8_t flood_coolant)
{
	float value[AXES] = { (float)flood_coolant,0,0,0,0,0 };
	mp_queue_command(_exec_flood_coolant_control, value, value);
	return (STAT_OK);
}
static void _exec_flood_coolant_control(float *value, float *flag)
{
	cm.gm.flood_coolant = (uint8_t)value[0];

#ifdef __AVR
	if (cm.gm.flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		float vect[] = { 0,0,0,0,0,0 };				// turn off mist coolant
		_exec_mist_coolant_control(vect, vect);		// M9 special function
	}
#endif // __AVR

#ifdef __ARM
	if (cm.gm.flood_coolant == true) {
		coolant_enable_pin.set();
	} else {
		coolant_enable_pin.clear();
		float vect[] = { 0,0,0,0,0,0 };				// turn off mist coolant
		_exec_mist_coolant_control(vect, vect);		// M9 special function
	}
#endif // __ARM
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

stat_t cm_override_enables(uint8_t flag)			// M48, M49
{
	cm.gmx.feed_rate_override_enable = flag;
	cm.gmx.traverse_override_enable = flag;
	cm.gmx.spindle_override_enable = flag;
	return (STAT_OK);
}

stat_t cm_feed_rate_override_enable(uint8_t flag)	// M50
{
	if (fp_TRUE(cm.gf.parameter) && fp_ZERO(cm.gn.parameter)) {
		cm.gmx.feed_rate_override_enable = false;
	} else {
		cm.gmx.feed_rate_override_enable = true;
	}
	return (STAT_OK);
}

stat_t cm_feed_rate_override_factor(uint8_t flag)	// M50.1
{
	cm.gmx.feed_rate_override_enable = flag;
	cm.gmx.feed_rate_override_factor = cm.gn.parameter;
//	mp_feed_rate_override(flag, cm.gn.parameter);	// replan the queue for new feed rate
	return (STAT_OK);
}

stat_t cm_traverse_override_enable(uint8_t flag)	// M50.2
{
	if (fp_TRUE(cm.gf.parameter) && fp_ZERO(cm.gn.parameter)) {
		cm.gmx.traverse_override_enable = false;
	} else {
		cm.gmx.traverse_override_enable = true;
	}
	return (STAT_OK);
}

stat_t cm_traverse_override_factor(uint8_t flag)	// M51
{
	cm.gmx.traverse_override_enable = flag;
	cm.gmx.traverse_override_factor = cm.gn.parameter;
//	mp_feed_rate_override(flag, cm.gn.parameter);	// replan the queue for new feed rate
	return (STAT_OK);
}

stat_t cm_spindle_override_enable(uint8_t flag)		// M51.1
{
	if (fp_TRUE(cm.gf.parameter) && fp_ZERO(cm.gn.parameter)) {
		cm.gmx.spindle_override_enable = false;
	} else {
		cm.gmx.spindle_override_enable = true;
	}
	return (STAT_OK);
}

stat_t cm_spindle_override_factor(uint8_t flag)		// M50.1
{
	cm.gmx.spindle_override_enable = flag;
	cm.gmx.spindle_override_factor = cm.gn.parameter;
//	change spindle speed
	return (STAT_OK);
}

/*
 * cm_message() - queue a RAM string as a message in the response (unconditionally)
 *
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

void cm_message(char_t *message)
{
	nv_add_string((const char_t *)"msg", message);	// add message to the response object
}

/******************************
 * Program Functions (4.3.10) *
 ******************************/
/*
 * This group implements stop, start, end, and hold.
 * It is extended beyond the NIST spec to handle various situations.
 *
 *	_exec_program_finalize()
 *	cm_cycle_start()			(no Gcode)  Do a cycle start right now
 *	cm_cycle_end()				(no Gcode)	Do a cycle end right now
 *	cm_feedhold()				(no Gcode)	Initiate a feedhold right now
 *	cm_program_stop()			(M0, M60)	Queue a program stop
 *	cm_optional_program_stop()	(M1)
 *	cm_program_end()			(M2, M30)
 *	cm_machine_ready()			puts machine into a READY state
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

/*
 * cm_request_feedhold()
 * cm_request_queue_flush()
 * cm_request_cycle_start()
 * cm_feedhold_sequencing_callback() - process feedholds, cycle starts & queue flushes
 * cm_flush_planner() - Flush planner queue and correct model positions
 *
 * Feedholds, queue flushes and cycles starts are all related. The request functions set
 *	flags for these. The sequencing callback interprets the flags according to the
 *	following rules:
 *
 *	A feedhold request received during motion should be honored
 *	A feedhold request received during a feedhold should be ignored and reset
 *	A feedhold request received during a motion stop should be ignored and reset
 *
 *	A queue flush request received during motion should be ignored but not reset
 *	A queue flush request received during a feedhold should be deferred until
 *		the feedhold enters a HOLD state (i.e. until deceleration is complete)
 *	A queue flush request received during a motion stop should be honored
 *
 *	A cycle start request received during motion should be ignored and reset
 *	A cycle start request received during a feedhold should be deferred until
 *		the feedhold enters a HOLD state (i.e. until deceleration is complete)
 *		If a queue flush request is also present the queue flush should be done first
 *	A cycle start request received during a motion stop should be honored and
 *		should start to run anything in the planner queue
 */

void cm_request_feedhold(void) { cm.feedhold_requested = true; }
void cm_request_queue_flush(void) { cm.queue_flush_requested = true; }
void cm_request_cycle_start(void) { cm.cycle_start_requested = true; }

stat_t cm_feedhold_sequencing_callback()
{
	if (cm.feedhold_requested == true) {
		if ((cm.motion_state == MOTION_RUN) && (cm.hold_state == FEEDHOLD_OFF)) {
			cm_set_motion_state(MOTION_HOLD);
			cm.hold_state = FEEDHOLD_SYNC;	// invokes hold from aline execution
		}
		cm.feedhold_requested = false;
	}
	if (cm.queue_flush_requested == true) {
		if (((cm.motion_state == MOTION_STOP) ||
			((cm.motion_state == MOTION_HOLD) && (cm.hold_state == FEEDHOLD_HOLD))) &&
			!cm_get_runtime_busy()) {
			cm.queue_flush_requested = false;
			cm_queue_flush();
		}
	}
	bool feedhold_processing =				// added feedhold processing lockout from omco fork
		cm.hold_state == FEEDHOLD_SYNC ||
		cm.hold_state == FEEDHOLD_PLAN ||
		cm.hold_state == FEEDHOLD_DECEL;
	if ((cm.cycle_start_requested == true) && (cm.queue_flush_requested == false) && !feedhold_processing) {
		cm.cycle_start_requested = false;
		cm.hold_state = FEEDHOLD_END_HOLD;
		cm_cycle_start();
		mp_end_hold();
	}
	return (STAT_OK);
}

stat_t cm_queue_flush()
{
	if (cm_get_runtime_busy() == true)
        return (STAT_COMMAND_NOT_ACCEPTED);

#ifdef __AVR
	xio_reset_usb_rx_buffers();				// flush serial queues
#endif
	mp_flush_planner();						// flush planner queue
	qr_request_queue_report(0);				// request a queue report, since we've changed the number of buffers available
	rx_request_rx_report();

	// Note: The following uses low-level mp calls for absolute position.
	//		 It could also use cm_get_absolute_position(RUNTIME, axis);
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm_set_position(axis, mp_get_runtime_absolute_position(axis)); // set mm from mr
	}
	float value[AXES] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
	_exec_program_finalize(value, value);	// finalize now, not later
	return (STAT_OK);
}

/*
 * Program and cycle state functions
 *
 * _exec_program_finalize() 	- helper
 * cm_cycle_start()
 * cm_cycle_end()
 * cm_program_stop()			- M0
 * cm_optional_program_stop()	- M1
 * cm_program_end()				- M2, M30
 *
 * cm_program_end() implements M2 and M30
 * The END behaviors are defined by NIST 3.6.1 are:
 *	1. Axis offsets are set to zero (like G92.2) and origin offsets are set to the default (like G54)
 *	2. Selected plane is set to CANON_PLANE_XY (like G17)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *	5. Feed and speed overrides are set to ON (like M48)
 *	6. Cutter compensation is turned off (like G40)
 *	7. The spindle is stopped (like M5)
 *	8. The current motion mode is set to G_1 (like G1)
 *	9. Coolant is turned off (like M9)
 *
 * cm_program_end() implments things slightly differently:
 *	1. Axis offsets are set to G92.1 CANCEL offsets (instead of using G92.2 SUSPEND Offsets)
 *	   Set default coordinate system (uses $gco, not G54)
 *	2. Selected plane is set to default plane ($gpl) (instead of setting it to G54)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 * 	5. Not implemented
 *	6. Not implemented
 *	7. The spindle is stopped (like M5)
 *	8. Motion mode is canceled like G80 (not set to G1)
 *	9. Coolant is turned off (like M9)
 *	+  Default INCHES or MM units mode is restored ($gun)
 */

static void _exec_program_finalize(float *value, float *flag)
{
	cm.machine_state = (uint8_t)value[0];
	cm_set_motion_state(MOTION_STOP);
	if (cm.cycle_state == CYCLE_MACHINING) {
		cm.cycle_state = CYCLE_OFF;						// don't end cycle if homing, probing, etc.
	}
	cm.hold_state = FEEDHOLD_OFF;						// end feedhold (if in feed hold)
	cm.cycle_start_requested = false;					// cancel any pending cycle start request
	mp_zero_segment_velocity();							// for reporting purposes

	// perform the following resets if it's a program END
	if (cm.machine_state == MACHINE_PROGRAM_END) {
		cm_reset_origin_offsets();						// G92.1 - we do G91.1 instead of G92.2
	//	cm_suspend_origin_offsets();					// G92.2 - as per Kramer
		cm_set_coord_system(cm.coord_system);			// reset to default coordinate system
		cm_select_plane(cm.select_plane);				// reset to default arc plane
		cm_set_distance_mode(cm.distance_mode);
//++++	cm_set_units_mode(cm.units_mode);				// reset to default units mode +++ REMOVED +++
		cm_spindle_control(SPINDLE_OFF);				// M5
		cm_flood_coolant_control(false);				// M9
		cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);	// G94
	//	cm_set_motion_mode(MOTION_MODE_STRAIGHT_FEED);	// NIST specifies G1, but we cancel motion mode. Safer.
		cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
	}
	sr_request_status_report(SR_IMMEDIATE_REQUEST);		// request a final status report (not unfiltered)
}

void cm_cycle_start()
{
	cm.machine_state = MACHINE_CYCLE;
	if (cm.cycle_state == CYCLE_OFF) {					// don't (re)start homing, probe or other canned cycles
		cm.cycle_state = CYCLE_MACHINING;
		qr_init_queue_report();							// clear queue reporting buffer counts
	}
}

void cm_cycle_end()
{
	if (cm.cycle_state != CYCLE_OFF) {
		float value[AXES] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
		_exec_program_finalize(value, value);
	}
}

void cm_program_stop()
{
	float value[AXES] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
	mp_queue_command(_exec_program_finalize, value, value);
}

void cm_optional_program_stop()
{
	float value[AXES] = { (float)MACHINE_PROGRAM_STOP, 0,0,0,0,0 };
	mp_queue_command(_exec_program_finalize, value, value);
}

void cm_program_end()
{
	float value[AXES] = { (float)MACHINE_PROGRAM_END, 0,0,0,0,0 };
	mp_queue_command(_exec_program_finalize, value, value);
}

/**************************************
 * END OF CANONICAL MACHINE FUNCTIONS *
 **************************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

// Strings for writing settings as nvObj string values
// Ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=120881&start=0

#ifdef __TEXT_MODE

static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
static const char msg_units1[] PROGMEM = " mm";
static const char msg_units2[] PROGMEM = " deg";
static const char *const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char msg_am00[] PROGMEM = "[disabled]";
static const char msg_am01[] PROGMEM = "[standard]";
static const char msg_am02[] PROGMEM = "[inhibited]";
static const char msg_am03[] PROGMEM = "[radius]";
static const char *const msg_am[] PROGMEM = { msg_am00, msg_am01, msg_am02, msg_am03};

static const char msg_g20[] PROGMEM = "G20 - inches mode";
static const char msg_g21[] PROGMEM = "G21 - millimeter mode";
static const char *const msg_unit[] PROGMEM = { msg_g20, msg_g21 };

static const char msg_stat0[] PROGMEM = "Initializing";	// combined state (stat) uses this array
static const char msg_stat1[] PROGMEM = "Ready";
static const char msg_stat2[] PROGMEM = "Alarm";
static const char msg_stat3[] PROGMEM = "Stop";
static const char msg_stat4[] PROGMEM = "End";
static const char msg_stat5[] PROGMEM = "Run";
static const char msg_stat6[] PROGMEM = "Hold";
static const char msg_stat7[] PROGMEM = "Probe";
static const char msg_stat8[] PROGMEM = "Cycle";
static const char msg_stat9[] PROGMEM = "Homing";
static const char msg_stat10[] PROGMEM = "Jog";
static const char msg_stat11[] PROGMEM = "Shutdown";
static const char *const msg_stat[] PROGMEM = { msg_stat0, msg_stat1, msg_stat2, msg_stat3,
												msg_stat4, msg_stat5, msg_stat6, msg_stat7,
												msg_stat8, msg_stat9, msg_stat10, msg_stat11 };

static const char msg_macs0[] PROGMEM = "Initializing";
static const char msg_macs1[] PROGMEM = "Ready";
static const char msg_macs2[] PROGMEM = "Alarm";
static const char msg_macs3[] PROGMEM = "Stop";
static const char msg_macs4[] PROGMEM = "End";
static const char msg_macs5[] PROGMEM = "Cycle";
static const char msg_macs6[] PROGMEM = "Shutdown";
static const char *const msg_macs[] PROGMEM = { msg_macs0, msg_macs1, msg_macs2, msg_macs3,
												msg_macs4, msg_macs5, msg_macs6 };

static const char msg_cycs0[] PROGMEM = "Off";
static const char msg_cycs1[] PROGMEM = "Machining";
static const char msg_cycs2[] PROGMEM = "Probe";
static const char msg_cycs3[] PROGMEM = "Homing";
static const char msg_cycs4[] PROGMEM = "Jog";
static const char *const msg_cycs[] PROGMEM = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3,  msg_cycs4 };

static const char msg_mots0[] PROGMEM = "Stop";
static const char msg_mots1[] PROGMEM = "Run";
static const char msg_mots2[] PROGMEM = "Hold";
static const char *const msg_mots[] PROGMEM = { msg_mots0, msg_mots1, msg_mots2 };

static const char msg_hold0[] PROGMEM = "Off";
static const char msg_hold1[] PROGMEM = "Sync";
static const char msg_hold2[] PROGMEM = "Plan";
static const char msg_hold3[] PROGMEM = "Decel";
static const char msg_hold4[] PROGMEM = "Hold";
static const char msg_hold5[] PROGMEM = "End Hold";
static const char *const msg_hold[] PROGMEM = { msg_hold0, msg_hold1, msg_hold2, msg_hold3,
												msg_hold4,  msg_hold5 };

static const char msg_home0[] PROGMEM = "Not Homed";
static const char msg_home1[] PROGMEM = "Homed";
static const char msg_home2[] PROGMEM = "Homing";
static const char *const msg_home[] PROGMEM = { msg_home0, msg_home1, msg_home2 };

static const char msg_g53[] PROGMEM = "G53 - machine coordinate system";
static const char msg_g54[] PROGMEM = "G54 - coordinate system 1";
static const char msg_g55[] PROGMEM = "G55 - coordinate system 2";
static const char msg_g56[] PROGMEM = "G56 - coordinate system 3";
static const char msg_g57[] PROGMEM = "G57 - coordinate system 4";
static const char msg_g58[] PROGMEM = "G58 - coordinate system 5";
static const char msg_g59[] PROGMEM = "G59 - coordinate system 6";
static const char *const msg_coor[] PROGMEM = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static const char msg_g00[] PROGMEM = "G0  - linear traverse (seek)";
static const char msg_g01[] PROGMEM = "G1  - linear feed";
static const char msg_g02[] PROGMEM = "G2  - clockwise arc feed";
static const char msg_g03[] PROGMEM = "G3  - counter clockwise arc feed";
static const char msg_g80[] PROGMEM = "G80 - cancel motion mode (none active)";
static const char *const msg_momo[] PROGMEM = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static const char msg_g17[] PROGMEM = "G17 - XY plane";
static const char msg_g18[] PROGMEM = "G18 - XZ plane";
static const char msg_g19[] PROGMEM = "G19 - YZ plane";
static const char *const msg_plan[] PROGMEM = { msg_g17, msg_g18, msg_g19 };

static const char msg_g61[] PROGMEM = "G61 - exact path mode";
static const char msg_g6a[] PROGMEM = "G61.1 - exact stop mode";
static const char msg_g64[] PROGMEM = "G64 - continuous mode";
static const char *const msg_path[] PROGMEM = { msg_g61, msg_g6a, msg_g64 };

static const char msg_g90[] PROGMEM = "G90 - absolute distance mode";
static const char msg_g91[] PROGMEM = "G91 - incremental distance mode";
static const char *const msg_dist[] PROGMEM = { msg_g90, msg_g91 };

static const char msg_g93[] PROGMEM = "G93 - inverse time mode";
static const char msg_g94[] PROGMEM = "G94 - units-per-minute mode (i.e. feedrate mode)";
static const char msg_g95[] PROGMEM = "G95 - units-per-revolution mode";
static const char *const msg_frmo[] PROGMEM = { msg_g93, msg_g94, msg_g95 };

#else

#define msg_units NULL
#define msg_unit NULL
#define msg_stat NULL
#define msg_macs NULL
#define msg_cycs NULL
#define msg_mots NULL
#define msg_hold NULL
#define msg_home NULL
#define msg_coor NULL
#define msg_momo NULL
#define msg_plan NULL
#define msg_path NULL
#define msg_dist NULL
#define msg_frmo NULL
#define msg_am NULL

#endif // __TEXT_MODE

/***** AXIS HELPERS *****************************************************************
 * cm_get_axis_char() - return ASCII char for axis given the axis number
 * _get_axis()		  - return axis number or -1 if NA
 * _get_axis_type()	  - return 0 -f axis is linear, 1 if rotary, -1 if NA
 */

char_t cm_get_axis_char(const int8_t axis)
{
	char_t axis_char[] = "XYZABC";
	if ((axis < 0) || (axis > AXES)) return (' ');
	return (axis_char[axis]);
}

static int8_t _get_axis(const index_t index)
{
	char_t *ptr;
	char_t tmp[TOKEN_LEN+1];
	char_t axes[] = {"xyzabc"};

	strncpy_P(tmp, cfgArray[index].token, TOKEN_LEN);	// kind of a hack. Looks for an axis
	if ((ptr = strchr(axes, tmp[0])) == NULL) {			// character in the 0 and 3 positions
		if ((ptr = strchr(axes, tmp[3])) == NULL) {		// to accommodate 'xam' and 'g54x' styles
			return (-1);
		}
	}
	return (ptr - axes);
}

static int8_t _get_axis_type(const index_t index)
{
	int8_t axis = _get_axis(index);
	if (axis >= AXIS_A) return (1);
	if (axis == -1) return (-1);
	return (0);
}

/**** Functions called directly from cfgArray table - mostly wrappers ****
 * _get_msg_helper() - helper to get string values
 *
 * cm_get_stat() - get combined machine state as value and string
 * cm_get_macs() - get raw machine state as value and string
 * cm_get_cycs() - get raw cycle state as value and string
 * cm_get_mots() - get raw motion state as value and string
 * cm_get_hold() - get raw hold state as value and string
 * cm_get_home() - get raw homing state as value and string
 *
 * cm_get_unit() - get units mode as integer and display string
 * cm_get_coor() - get goodinate system
 * cm_get_momo() - get runtime motion mode
 * cm_get_plan() - get model gcode plane select
 * cm_get_path() - get model gcode path control mode
 * cm_get_dist() - get model gcode distance mode
 * cm_get_frmo() - get model gcode feed rate mode
 * cm_get_tool() - get tool
 * cm_get_feed() - get feed rate
 * cm_get_mline()- get model line number for status reports
 * cm_get_line() - get active (model or runtime) line number for status reports
 * cm_get_vel()  - get runtime velocity
 * cm_get_ofs()  - get current work offset (runtime)
 * cm_get_pos()  - get current work position (runtime)
 * cm_get_mpos() - get current machine position (runtime)
 *
 * cm_print_pos()- print work position (with proper units)
 * cm_print_mpos()- print machine position (always mm units)
 * cm_print_coor()- print coordinate offsets with linear units
 * cm_print_corr()- print coordinate offsets with rotary units
 */

stat_t _get_msg_helper(nvObj_t *nv, const char *const msg_array[], uint8_t value)
{
	nv->value = (float)value;
	nv->valuetype = TYPE_INTEGER;
	return(nv_copy_string(nv, (const char_t *)GET_TEXT_ITEM(msg_array, value)));
}

stat_t cm_get_stat(nvObj_t *nv) { return(_get_msg_helper(nv, msg_stat, cm_get_combined_state()));}
stat_t cm_get_macs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_macs, cm_get_machine_state()));}
stat_t cm_get_cycs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_cycs, cm_get_cycle_state()));}
stat_t cm_get_mots(nvObj_t *nv) { return(_get_msg_helper(nv, msg_mots, cm_get_motion_state()));}
stat_t cm_get_hold(nvObj_t *nv) { return(_get_msg_helper(nv, msg_hold, cm_get_hold_state()));}
stat_t cm_get_home(nvObj_t *nv) { return(_get_msg_helper(nv, msg_home, cm_get_homing_state()));}

stat_t cm_get_unit(nvObj_t *nv) { return(_get_msg_helper(nv, msg_unit, cm_get_units_mode(ACTIVE_MODEL)));}
stat_t cm_get_coor(nvObj_t *nv) { return(_get_msg_helper(nv, msg_coor, cm_get_coord_system(ACTIVE_MODEL)));}
stat_t cm_get_momo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_momo, cm_get_motion_mode(ACTIVE_MODEL)));}
stat_t cm_get_plan(nvObj_t *nv) { return(_get_msg_helper(nv, msg_plan, cm_get_select_plane(ACTIVE_MODEL)));}
stat_t cm_get_path(nvObj_t *nv) { return(_get_msg_helper(nv, msg_path, cm_get_path_control(ACTIVE_MODEL)));}
stat_t cm_get_dist(nvObj_t *nv) { return(_get_msg_helper(nv, msg_dist, cm_get_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_frmo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_frmo, cm_get_feed_rate_mode(ACTIVE_MODEL)));}

stat_t cm_get_toolv(nvObj_t *nv)
{
	nv->value = (float)cm_get_tool(ACTIVE_MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_mline(nvObj_t *nv)
{
	nv->value = (float)cm_get_linenum(MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_line(nvObj_t *nv)
{
	nv->value = (float)cm_get_linenum(ACTIVE_MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_vel(nvObj_t *nv)
{
	if (cm_get_motion_state() == MOTION_STOP) {
		nv->value = 0;
	} else {
		nv->value = mp_get_runtime_velocity();
		if (cm_get_units_mode(RUNTIME) == INCHES) nv->value *= INCHES_PER_MM;
	}
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_feed(nvObj_t *nv)
{
	nv->value = cm_get_feed_rate(ACTIVE_MODEL);
	if (cm_get_units_mode(ACTIVE_MODEL) == INCHES) nv->value *= INCHES_PER_MM;
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_pos(nvObj_t *nv)
{
	nv->value = cm_get_work_position(ACTIVE_MODEL, _get_axis(nv->index));
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_mpo(nvObj_t *nv)
{
	nv->value = cm_get_absolute_position(ACTIVE_MODEL, _get_axis(nv->index));
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_ofs(nvObj_t *nv)
{
	nv->value = cm_get_work_offset(ACTIVE_MODEL, _get_axis(nv->index));
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

/*
 * AXIS GET AND SET FUNCTIONS
 *
 * cm_get_am()	- get axis mode w/enumeration string
 * cm_set_am()	- set axis mode w/exception handling for axis type
 * cm_set_sw()	- run this any time you change a switch setting
 */

stat_t cm_get_am(nvObj_t *nv)
{
	get_ui8(nv);
	return(_get_msg_helper(nv, msg_am, nv->value));
}

stat_t cm_set_am(nvObj_t *nv)		// axis mode
{
	if (_get_axis_type(nv->index) == 0) {	// linear
		if (nv->value > AXIS_MODE_MAX_LINEAR) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}
	} else {
		if (nv->value > AXIS_MODE_MAX_ROTARY) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}
	}
	set_ui8(nv);
	return(STAT_OK);
}

/**** Jerk functions
 * cm_get_axis_jerk() - returns jerk for an axis
 * cm_set_axis_jerk() - sets the jerk for an axis, including recirpcal and cached values
 *
 * cm_set_xjm()		  - set jerk max value
 * cm_set_xjh()		  - set jerk halt value (used by homing and other stops)
 *
 *	Jerk values can be rather large, often in the billions. This makes for some pretty big
 *	numbers for people to deal with. Jerk values are stored in the system in truncated format;
 *	values are divided by 1,000,000 then reconstituted before use.
 *
 *	The set_xjm() nad set_xjh() functions will accept either truncated or untruncated jerk
 *	numbers as input. If the number is > 1,000,000 it is divided by 1,000,000 before storing.
 *	Numbers are accepted in either millimeter or inch mode and converted to millimeter mode.
 *
 *	The axis_jerk() functions expect the jerk in divided-by 1,000,000 form
 */
float cm_get_axis_jerk(uint8_t axis)
{
	return (cm.a[axis].jerk_max);
}

void cm_set_axis_jerk(uint8_t axis, float jerk)
{
	cm.a[axis].jerk_max = jerk;
	cm.a[axis].recip_jerk = 1/(jerk * JERK_MULTIPLIER);
}

stat_t cm_set_xjm(nvObj_t *nv)
{
	if (nv->value > JERK_MULTIPLIER) nv->value /= JERK_MULTIPLIER;
	set_flu(nv);
	cm_set_axis_jerk(_get_axis(nv->index), nv->value);
	return(STAT_OK);
}

stat_t cm_set_xjh(nvObj_t *nv)
{
	if (nv->value > JERK_MULTIPLIER) nv->value /= JERK_MULTIPLIER;
	set_flu(nv);
	return(STAT_OK);
}

/*
 * Commands
 *
 * cm_run_qf() - flush planner queue
 * cm_run_home() - run homing sequence
 */

stat_t cm_run_qf(nvObj_t *nv)
{
	cm_request_queue_flush();
	return (STAT_OK);
}

stat_t cm_run_home(nvObj_t *nv)
{
	if (fp_TRUE(nv->value)) { cm_homing_cycle_start();}
	return (STAT_OK);
}

/*
 * Debugging Commands
 *
 * cm_dam() - dump active model
 */

stat_t cm_dam(nvObj_t *nv)
{
	printf("Active model:\n");
	cm_print_vel(nv);
	cm_print_feed(nv);
	cm_print_line(nv);
	cm_print_stat(nv);
	cm_print_macs(nv);
	cm_print_cycs(nv);
	cm_print_mots(nv);
	cm_print_hold(nv);
	cm_print_home(nv);
	cm_print_unit(nv);
	cm_print_coor(nv);
	cm_print_momo(nv);
	cm_print_plan(nv);
	cm_print_path(nv);
	cm_print_dist(nv);
	cm_print_frmo(nv);
	cm_print_tool(nv);

	return (STAT_OK);
}

/***********************************************************************************
 * AXIS JOGGING
 ***********************************************************************************/

float cm_get_jogging_dest(void)
{
	return cm.jogging_dest;
}

stat_t cm_run_jogx(nvObj_t *nv)
{
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_X);
	return (STAT_OK);
}

stat_t cm_run_jogy(nvObj_t *nv)
{
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_Y);
	return (STAT_OK);
}

stat_t cm_run_jogz(nvObj_t *nv)
{
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_Z);
	return (STAT_OK);
}

stat_t cm_run_joga(nvObj_t *nv)
{
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_A);
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

/* model state print functions */

const char fmt_vel[]  PROGMEM = "Velocity:%17.3f%s/min\n";
const char fmt_feed[] PROGMEM = "Feed rate:%16.3f%s/min\n";
const char fmt_line[] PROGMEM = "Line number:%10.0f\n";
const char fmt_stat[] PROGMEM = "Machine state:       %s\n"; // combined machine state
const char fmt_macs[] PROGMEM = "Raw machine state:   %s\n"; // raw machine state
const char fmt_cycs[] PROGMEM = "Cycle state:         %s\n";
const char fmt_mots[] PROGMEM = "Motion state:        %s\n";
const char fmt_hold[] PROGMEM = "Feedhold state:      %s\n";
const char fmt_home[] PROGMEM = "Homing state:        %s\n";
const char fmt_unit[] PROGMEM = "Units:               %s\n"; // units mode as ASCII string
const char fmt_coor[] PROGMEM = "Coordinate system:   %s\n";
const char fmt_momo[] PROGMEM = "Motion mode:         %s\n";
const char fmt_plan[] PROGMEM = "Plane:               %s\n";
const char fmt_path[] PROGMEM = "Path Mode:           %s\n";
const char fmt_dist[] PROGMEM = "Distance mode:       %s\n";
const char fmt_frmo[] PROGMEM = "Feed rate mode:      %s\n";
const char fmt_tool[] PROGMEM = "Tool number          %d\n";

const char fmt_pos[] PROGMEM = "%c position:%15.3f%s\n";
const char fmt_mpo[] PROGMEM = "%c machine posn:%11.3f%s\n";
const char fmt_ofs[] PROGMEM = "%c work offset:%12.3f%s\n";
const char fmt_hom[] PROGMEM = "%c axis homing state:%2.0f\n";

const char fmt_gpl[] PROGMEM = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
const char fmt_gun[] PROGMEM = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
const char fmt_gco[] PROGMEM = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
const char fmt_gpa[] PROGMEM = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
const char fmt_gdi[] PROGMEM = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

void cm_print_vel(nvObj_t *nv) { text_print_flt_units(nv, fmt_vel, GET_UNITS(ACTIVE_MODEL));}
void cm_print_feed(nvObj_t *nv) { text_print_flt_units(nv, fmt_feed, GET_UNITS(ACTIVE_MODEL));}
void cm_print_line(nvObj_t *nv) { text_print_int(nv, fmt_line);}
void cm_print_stat(nvObj_t *nv) { text_print_str(nv, fmt_stat);}
void cm_print_macs(nvObj_t *nv) { text_print_str(nv, fmt_macs);}
void cm_print_cycs(nvObj_t *nv) { text_print_str(nv, fmt_cycs);}
void cm_print_mots(nvObj_t *nv) { text_print_str(nv, fmt_mots);}
void cm_print_hold(nvObj_t *nv) { text_print_str(nv, fmt_hold);}
void cm_print_home(nvObj_t *nv) { text_print_str(nv, fmt_home);}
void cm_print_unit(nvObj_t *nv) { text_print_str(nv, fmt_unit);}
void cm_print_coor(nvObj_t *nv) { text_print_str(nv, fmt_coor);}
void cm_print_momo(nvObj_t *nv) { text_print_str(nv, fmt_momo);}
void cm_print_plan(nvObj_t *nv) { text_print_str(nv, fmt_plan);}
void cm_print_path(nvObj_t *nv) { text_print_str(nv, fmt_path);}
void cm_print_dist(nvObj_t *nv) { text_print_str(nv, fmt_dist);}
void cm_print_frmo(nvObj_t *nv) { text_print_str(nv, fmt_frmo);}
void cm_print_tool(nvObj_t *nv) { text_print_int(nv, fmt_tool);}

void cm_print_gpl(nvObj_t *nv) { text_print_int(nv, fmt_gpl);}
void cm_print_gun(nvObj_t *nv) { text_print_int(nv, fmt_gun);}
void cm_print_gco(nvObj_t *nv) { text_print_int(nv, fmt_gco);}
void cm_print_gpa(nvObj_t *nv) { text_print_int(nv, fmt_gpa);}
void cm_print_gdi(nvObj_t *nv) { text_print_int(nv, fmt_gdi);}

/* system state print functions */

const char fmt_ja[] PROGMEM = "[ja]  junction acceleration%8.0f%s\n";
const char fmt_ct[] PROGMEM = "[ct]  chordal tolerance%17.4f%s\n";
const char fmt_sl[] PROGMEM = "[sl]  soft limit enable%12d\n";
const char fmt_ml[] PROGMEM = "[ml]  min line segment%17.3f%s\n";
const char fmt_ma[] PROGMEM = "[ma]  min arc segment%18.3f%s\n";
const char fmt_ms[] PROGMEM = "[ms]  min segment time%13.0f uSec\n";

void cm_print_ja(nvObj_t *nv) { text_print_flt_units(nv, fmt_ja, GET_UNITS(ACTIVE_MODEL));}
void cm_print_ct(nvObj_t *nv) { text_print_flt_units(nv, fmt_ct, GET_UNITS(ACTIVE_MODEL));}
void cm_print_sl(nvObj_t *nv) { text_print_ui8(nv, fmt_sl);}
void cm_print_ml(nvObj_t *nv) { text_print_flt_units(nv, fmt_ml, GET_UNITS(ACTIVE_MODEL));}
void cm_print_ma(nvObj_t *nv) { text_print_flt_units(nv, fmt_ma, GET_UNITS(ACTIVE_MODEL));}
void cm_print_ms(nvObj_t *nv) { text_print_flt_units(nv, fmt_ms, GET_UNITS(ACTIVE_MODEL));}

/*
 * axis print functions
 *
 *	_print_axis_ui8() - helper to print an integer value with no units
 *	_print_axis_flt() - helper to print a floating point linear value in prevailing units
 *	_print_pos_helper()
 *
 *	cm_print_am()
 *	cm_print_fr()
 *	cm_print_vm()
 *	cm_print_tm()
 *	cm_print_tn()
 *	cm_print_jm()
 *	cm_print_jh()
 *	cm_print_jd()
 *	cm_print_ra()
 *	cm_print_sn()
 *	cm_print_sx()
 *	cm_print_lv()
 *	cm_print_lb()
 *	cm_print_zb()
 *
 *	cm_print_pos() - print position with unit displays for MM or Inches
 * 	cm_print_mpo() - print position with fixed unit display - always in Degrees or MM
 */

static const char fmt_Xam[] PROGMEM = "[%s%s] %s axis mode%18d %s\n";
static const char fmt_Xfr[] PROGMEM = "[%s%s] %s feedrate maximum%11.0f%s/min\n";
static const char fmt_Xvm[] PROGMEM = "[%s%s] %s velocity maximum%11.0f%s/min\n";
static const char fmt_Xtm[] PROGMEM = "[%s%s] %s travel maximum%17.3f%s\n";
static const char fmt_Xtn[] PROGMEM = "[%s%s] %s travel minimum%17.3f%s\n";
static const char fmt_Xjm[] PROGMEM = "[%s%s] %s jerk maximum%15.0f%s/min^3 * 1 million\n";
static const char fmt_Xjh[] PROGMEM = "[%s%s] %s jerk homing%16.0f%s/min^3 * 1 million\n";
static const char fmt_Xjd[] PROGMEM = "[%s%s] %s junction deviation%14.4f%s (larger is faster)\n";
static const char fmt_Xra[] PROGMEM = "[%s%s] %s radius value%20.4f%s\n";
static const char fmt_Xsn[] PROGMEM = "[%s%s] %s switch min%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char fmt_Xsx[] PROGMEM = "[%s%s] %s switch max%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char fmt_Xsv[] PROGMEM = "[%s%s] %s search velocity%12.0f%s/min\n";
static const char fmt_Xlv[] PROGMEM = "[%s%s] %s latch velocity%13.0f%s/min\n";
static const char fmt_Xlb[] PROGMEM = "[%s%s] %s latch backoff%18.3f%s\n";
static const char fmt_Xzb[] PROGMEM = "[%s%s] %s zero backoff%19.3f%s\n";
static const char fmt_cofs[] PROGMEM = "[%s%s] %s %s offset%20.3f%s\n";
static const char fmt_cpos[] PROGMEM = "[%s%s] %s %s position%18.3f%s\n";

static void _print_axis_ui8(nvObj_t *nv, const char *format)
{
	fprintf_P(stderr, format, nv->group, nv->token, nv->group, (uint8_t)nv->value);
}

static void _print_axis_flt(nvObj_t *nv, const char *format)
{
	char *units;
	if (_get_axis_type(nv->index) == 0) {	// linear
		units = (char *)GET_UNITS(MODEL);
	} else {
		units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
	}
	fprintf_P(stderr, format, nv->group, nv->token, nv->group, nv->value, units);
}

static void _print_axis_coord_flt(nvObj_t *nv, const char *format)
{
	char *units;
	if (_get_axis_type(nv->index) == 0) {	// linear
		units = (char *)GET_UNITS(MODEL);
	} else {
		units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
	}
	fprintf_P(stderr, format, nv->group, nv->token, nv->group, nv->token, nv->value, units);
}

static void _print_pos(nvObj_t *nv, const char *format, uint8_t units)
{
	char axes[] = {"XYZABC"};
	uint8_t axis = _get_axis(nv->index);
	if (axis >= AXIS_A) { units = DEGREES;}
	fprintf_P(stderr, format, axes[axis], nv->value, GET_TEXT_ITEM(msg_units, units));
}

void cm_print_am(nvObj_t *nv)	// print axis mode with enumeration string
{
	fprintf_P(stderr, fmt_Xam, nv->group, nv->token, nv->group, (uint8_t)nv->value,
	GET_TEXT_ITEM(msg_am, (uint8_t)nv->value));
}

void cm_print_fr(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xfr);}
void cm_print_vm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xvm);}
void cm_print_tm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtm);}
void cm_print_tn(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtn);}
void cm_print_jm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjm);}
void cm_print_jh(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjh);}
void cm_print_jd(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjd);}
void cm_print_ra(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xra);}
void cm_print_sn(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xsn);}
void cm_print_sx(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xsx);}
void cm_print_sv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xsv);}
void cm_print_lv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlv);}
void cm_print_lb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlb);}
void cm_print_zb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xzb);}

void cm_print_cofs(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cofs);}
void cm_print_cpos(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cpos);}

void cm_print_pos(nvObj_t *nv) { _print_pos(nv, fmt_pos, cm_get_units_mode(MODEL));}
void cm_print_mpo(nvObj_t *nv) { _print_pos(nv, fmt_mpo, MILLIMETERS);}
void cm_print_ofs(nvObj_t *nv) { _print_pos(nv, fmt_ofs, MILLIMETERS);}

#endif // __TEXT_MODE
/*
#ifdef __cplusplus
}
#endif
*/
