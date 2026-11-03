/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2016 Robert Giseburt
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
#include "canonical_machine.h"
#include "controller.h"
#include "text_parser.h"

#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "runtime.h"
#include "report.h"
#include "gpio.h"
#include "switch.h"
#include "hardware.h"
#include "util.h"
#include "xio.h"			// for serial queue flush

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmSingleton_t cm;		// canonical machine controller singleton

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

// command execution callbacks from planner queue
static void _exec_offset(float *value, bool *flags);
static void _exec_g10_offset(float *value, bool *flags);
static void _exec_change_tool(float *value, bool *flags);
static void _exec_mist_coolant_control(float *value, bool *flags);
static void _exec_flood_coolant_control(float *value, bool *flags);
static void _exec_absolute_origin(float *value, bool *flags);
static void _exec_program_finalize(float *value, bool *flags);
static void _exec_program_end(float *value, bool *flags);
static void _exec_program_stop(float *value, bool *flags);

static int8_t _get_axis(const index_t index);
static int8_t _get_axis_type(const index_t index);
static uint8_t _report_units(void);
static float _get_report_position(const uint8_t axis);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

/*************************************
 * Internal getters and setters      *
 * Canonical Machine State functions *
 *************************************/
/*
 * cm_set_motion_state() - adjusts active model pointer as well
 */
void cm_set_motion_state(const cmMotionState motion_state)
{
    if (motion_state == MOTION_RUN) {
        rt_qt_invalidate();                     // backstop: motion starting (or resuming) ends quiescence
    }
    cm.motion_state = motion_state;

    switch (motion_state) {
        case (MOTION_STOP): { ACTIVE_MODEL = MODEL; break; }
        case (MOTION_RUN):  { ACTIVE_MODEL = RUNTIME; break; }
        case (MOTION_HOLD): { ACTIVE_MODEL = RUNTIME; break; }
    }
    cm_position_changed();                      // a report may now read the other model (E166)
}

/*
 * cm_get_machine_state()
 * cm_get_cycle_state()
 * cm_get_motion_state()
 * cm_get_hold_state()
 * cm_get_homing_state()
 */
 cmMachineState  cm_get_machine_state() { return cm.machine_state;}
 cmCycleState    cm_get_cycle_state()   { return cm.cycle_state;}
 cmMotionState   cm_get_motion_state()  { return cm.motion_state;}
 cmFeedholdState cm_get_hold_state()    { return cm.hold_state;}
 cmHomingState   cm_get_homing_state()  { return cm.homing_state;}

/*
 * cm_get_combined_state() - combines raw states into something a user might want to see
 *
 *  Note:
 *  On issuing a gcode command we call cm_cycle_start() before the motion gets queued. We don't go
 *  to MOTION_RUN until the command is executed by mp_exec_aline(), planned, queued, and started.
 *  So MOTION_STOP must actually return COMBINED_RUN to address this case, even though under some
 *  circumstances it might actually ne an exception case. Therefore this assertion isn't valid:
 *      cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "mots2"));//"mots is stop but machine is in cycle"
 *      return (COMBINED_PANIC);
 */
cmCombinedState cm_get_combined_state()
{
    if (cm.machine_state <= MACHINE_PROGRAM_END) {  // replaces first 5 cm.machine_state cases
        return ((cmCombinedState)cm.machine_state); //...where MACHINE_xxx == COMBINED_xxx
    }
    switch(cm.machine_state) {
        case MACHINE_INTERLOCK:     { return (COMBINED_INTERLOCK); }
        case MACHINE_SHUTDOWN:      { return (COMBINED_SHUTDOWN); }
        case MACHINE_PANIC:         { return (COMBINED_PANIC); }
        case MACHINE_CYCLE: {
            switch(cm.cycle_state) {
                case CYCLE_HOMING:  { return (COMBINED_HOMING); }
                case CYCLE_PROBE:   { return (COMBINED_PROBE); }
                case CYCLE_JOG:     { return (COMBINED_JOG); }
                case CYCLE_MACHINING: case CYCLE_OFF: {
                    switch(cm.motion_state) {
                        case MOTION_STOP:     { return (COMBINED_RUN); }    // See NOTE_1, above
                        case MOTION_RUN:      { return (COMBINED_RUN); }
                        case MOTION_HOLD:     { return (COMBINED_HOLD); }
                        default: {
                            cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() mots bad"));    // "mots has impossible value"
                            return (COMBINED_PANIC);
                        }
                    }
                }
                default: {
                    cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() cycs bad"));    // "cycs has impossible value"
                    return (COMBINED_PANIC);
                }
            }
        }
        default: {
            cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() macs bad"));    // "macs has impossible value"
            return (COMBINED_PANIC);
        }
    }
}

/***********************************
 * Model State Getters and Setters *
 ***********************************/
/*	These getters and setters will work on any gm model with inputs:
 *		MODEL 		(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
 *		RUNTIME		(GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
 *		ACTIVE_MODEL cm.am						// active model pointer is maintained by state management
 */
uint32_t cm_get_linenum(const GCodeState_t *gcode_state) { return gcode_state->linenum;}
uint8_t cm_get_motion_mode(const GCodeState_t *gcode_state) { return gcode_state->motion_mode;}
uint8_t cm_get_coord_system(const GCodeState_t *gcode_state) { return gcode_state->coord_system;}
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state) { return gcode_state->units_mode;}
uint8_t cm_get_select_plane(const GCodeState_t *gcode_state) { return gcode_state->select_plane;}
uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return gcode_state->path_control;}
uint8_t cm_get_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->distance_mode;}
uint8_t cm_get_arc_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->arc_distance_mode;}
uint8_t cm_get_feed_rate_mode(const GCodeState_t *gcode_state) { return gcode_state->feed_rate_mode;}
uint8_t cm_get_tool(const GCodeState_t *gcode_state) { return gcode_state->tool;}
uint8_t	cm_get_block_delete_switch() { return cm.gmx.block_delete_switch;}
uint8_t cm_get_runtime_busy() { return (mp_get_runtime_busy());}
float cm_get_feed_rate(const GCodeState_t *gcode_state) { return gcode_state->feed_rate;}

uint8_t cm_get_spindle_mode(const GCodeState_t *gcode_state) { return gcode_state->spindle_mode;}

void cm_set_motion_mode(GCodeState_t *gcode_state, const uint8_t motion_mode)
{
    gcode_state->motion_mode = (cmMotionMode)motion_mode;
}

void cm_set_tool_number(GCodeState_t *gcode_state, const uint8_t tool)
{
    gcode_state->tool = tool;
}

void cm_set_spindle_mode(GCodeState_t *gcode_state, const uint8_t spindle_mode)
{
    gcode_state->spindle_mode = spindle_mode;
}

void cm_set_spindle_speed_parameter(GCodeState_t *gcode_state, const float speed)
{
    gcode_state->spindle_speed = speed;
}

void cm_set_absolute_override(const uint8_t absolute_override)
{
	cm.gm.absolute_override = absolute_override;
	cm_set_work_offsets();				    // must reset offsets if you change absolute override
}

void cm_set_model_linenum(const uint32_t linenum)
{
	cm.gm.linenum = linenum;				// you must first set the model line number,
	nv_add_object((const char *)"n");	    // then add the line number to the nv list
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
 *	canonical units, which are millimeters (mm). The offsets are only used to translate
 *  in and out of canonical form during interpretation and display.
 *
 * Managing the coordinate systems & offsets is somewhat complicated. The following affect offsets:
 *	- coordinate system selected. 1-9 correspond to G54-G59
 *	- absolute override: forces current move to be interpreted in machine coordinates: G53 (system 0)
 *	- G92 offsets are added "on top of" the coord system offsets -- if origin_offset_enable == true
 *  - tool length compensation is applied to all axes, even though it's usually only used for Z (or X)
 *	- G28 and G30 moves; these are run in absolute coordinates
 *
 * The offsets themselves are considered static, are kept in cm.gmx, and are supposed to be persistent.
 *
 * To reduce complexity and data load the following is done:
 *	- Full data for coordinates/offsets is only accessible by the canonical machine, not downstream
 *	- A fully resolved set of coordinate, G92 and tool offsets, with per-move exceptions, is captured as "work_offsets"
 *	- The core gcode context (gm) only knows about the active coord system, tool offsets, and the work offsets
 */

/*
 * cm_get_active_coord_offset() - return the currently active coordinate offset for an axis
 *
 *	Takes G5x, G92 and absolute override into account to return the active offset for this move
 *
 *	This function is typically used to evaluate and set offsets, as opposed to cm_get_work_offset()
 *	which merely returns what's in the work_offset[] array.
 */

float cm_get_active_coord_offset(const uint8_t axis)
{
    if (cm.gm.absolute_override == ABSOLUTE_OVERRIDE_ON) {  // no offset if in absolute override mode
        return (0.0);
    }
	float offset = cm.offset[cm.gm.coord_system][axis];
	if (cm.gmx.origin_offset_enable == true) {
		offset += cm.gmx.origin_offset[axis];				// includes G5x and G92 components
    }
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

float cm_get_work_offset(const GCodeState_t *gcode_state, const uint8_t axis)
{
	if (gcode_state == RUNTIME) {
		return (mp_get_runtime_work_offset(axis));	// the exec rewrites it (E166)
	}
	return (gcode_state->work_offset[axis]);
}

/*
 * cm_set_work_offsets() - capture coord & tool offsets from the model into absolute values in the model
 */

void cm_set_work_offsets()
{
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm.gm.work_offset[axis] = cm_get_active_coord_offset(axis) + cm.gmx.tool_offset[axis];
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

float cm_get_absolute_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
	if (gcode_state == MODEL) {
        return (cm.gmx.position[axis]);
    }
	return (mp_get_runtime_absolute_position(axis));
}

/*
 * cm_get_work_position() - return work position in external form
 *
 *	... that means in prevailing units (mm/inch) and with all coord & tool offsets applied
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

static float _get_work_position_mm(const GCodeState_t *gcode_state, const uint8_t axis)
{
	if (gcode_state == MODEL) {
		return (cm.gmx.position[axis] - cm_get_active_coord_offset(axis) - cm.gmx.tool_offset[axis]);
	}
	return (mp_get_runtime_work_position(axis));
}

float cm_get_work_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
	float position = _get_work_position_mm(gcode_state, axis);

    if (axis <= AXIS_Z) {
        if (gcode_state->units_mode == INCHES) {
            position /= MM_PER_INCH;
        }
    }
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

	// if in inverse time mode reset feed rate so next block requires an explicit feed rate setting
	if ((cm.gm.feed_rate_mode == INVERSE_TIME_MODE) && ((cm.gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) ||
        (cm.gm.motion_mode == MOTION_MODE_CW_ARC) || (cm.gm.motion_mode == MOTION_MODE_CCW_ARC))) {
		cm.gm.feed_rate = 0;
	}
}

void cm_update_model_position_from_runtime() { copy_vector(cm.gmx.position, mr.gm.target); }

/*
 * _persist_coord_offsets() - main-loop write-back, before an idle G10 response or after motion
 * cm_deferred_write_callback() - service offsets left pending during motion
 *
 * The callback waits for empty serial rings. That check does not reserve the wire:
 * a host streaming data during a deferred write still needs persistence-aware flow control.
 */

static void _persist_coord_offsets()
{
	cm.deferred_write_flag = false;
	nvObj_t nv;
	for (uint8_t i=1; i<=COORDS; i++) {
		for (uint8_t j=0; j<AXES; j++) {
			nv.token[0] = 'g';						// g54x .. g59c, built without sprintf: older
			nv.token[1] = '5';						// avr-gcc can't prove a %u/%d fits the token
			nv.token[2] = (char)('3' + i);			// i is 1..6 (G54..G59)
			nv.token[3] = ("xyzabc")[j];
			nv.token[4] = NUL;
			nv.index = nv_get_index((const char *)"", nv.token);
			nv.value_flt = cm.offset[i][j];
			nv_persist(&nv);				// Note: only writes values that have changed
		}
	}
}

stat_t cm_deferred_write_callback()
{
    if ((cm.cycle_state == CYCLE_OFF) && cm.deferred_write_flag && !st_aline_in_flight()) {
#ifdef __AVR
        if (xio_isbusy()) { return (STAT_OK); }
#endif
        _persist_coord_offsets();
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

static float _calc_ABC(const uint8_t axis, const float target[])
{
	if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
		return(target[axis]);	// no mm conversion - it's in degrees
	}
	return(_to_millimeters(target[axis]) * 360 / (2 * M_PI * cm.a[axis].radius));
}

void cm_set_model_target(const float target[], const bool flags[])
{
    copy_vector(cm.gm.target, cm.gmx.position); // start from the last accepted endpoint
	uint8_t axis;
	float tmp = 0;

	// process XYZABC for lower modes
	for (axis=AXIS_X; axis<=AXIS_Z; axis++) {
		if (!flags[axis] || cm.a[axis].axis_mode == AXIS_DISABLED) {
			continue;		// skip axis if not flagged for update or its disabled
		} else if ((cm.a[axis].axis_mode == AXIS_STANDARD) || (cm.a[axis].axis_mode == AXIS_INHIBITED)) {
			if (cm.gm.distance_mode == ABSOLUTE_MODE) {
				cm.gm.target[axis] = cm_get_active_coord_offset(axis) + _to_millimeters(target[axis]);
				if (cm.gm.absolute_override == ABSOLUTE_OVERRIDE_OFF) {
					cm.gm.target[axis] += cm.gmx.tool_offset[axis];	// work coordinates only: not G53
				}
			} else {
				cm.gm.target[axis] += _to_millimeters(target[axis]);	// a distance: the offset is already in the position
			}
		}
	}
	// FYI: The ABC loop below relies on the XYZ loop having been run first
	for (axis=AXIS_A; axis<=AXIS_C; axis++) {
		if (!flags[axis] || cm.a[axis].axis_mode == AXIS_DISABLED) {
			continue;		// skip axis if not flagged for update or its disabled
		} else {
			tmp = _calc_ABC(axis, target);
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
 * canonical_machine_reset()
 */

void canonical_machine_init()
{
// If you can assume all memory has been zeroed by a hard reset you don't need this memset code.
// Do not reset canonicalMachineSingleton once it's been initialized - which would be:
//	memset(&cm, 0, sizeof(cm));	                    // do not reset canonicalMachineSingleton once it's been initialized
	memset(&cm.gm, 0, sizeof(GCodeState_t));	    // clear the Gcode MODEL of all values, pointers and status
	memset(&cm.gn, 0, sizeof(GCodeInput_t));        // clear the Gcode new data struct
	memset(&cm.gf, 0, sizeof(GCodeFlags_t));        // clear the Gcode data flags struct

	canonical_machine_init_assertions();		    // establish assertions
	ACTIVE_MODEL = MODEL;						    // setup initial Gcode model pointer

	// sub-system inits
	spindle_init();
	cm_arc_init();
    canonical_machine_reset();                      // always finish an init with reset
}

void canonical_machine_reset()
{
	// set gcode defaults
	cm_set_units_mode(cm.default_units_mode);
	cm_set_coord_system(cm.default_coord_system);
	cm_select_plane(cm.default_select_plane);
	cm_set_path_control(cm.default_path_control);
	cm_set_distance_mode(cm.default_distance_mode);
	cm_set_arc_distance_mode(INCREMENTAL_MODE);     // always the default
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);   // always the default
//    cm_reset_overrides();                         // set overrides to initial conditions

    // NOTE: Should unhome axes here

	// reset request flags
//	cm.feedhold_requested = false;                  //+++++ TO BE REMOVED
//	cm.queue_flush_requested = false;               //+++++ TO BE REMOVED
//	cm.cycle_start_requested = false;               //+++++ TO BE REMOVED

    cm.queue_flush_state = FLUSH_OFF;
    cm.queue_flush_input = false;
    cm.queue_flush_input_due = false;
    cm.program_paused = false;
    cm.end_hold_requested = false;
    cm.limit_requested = 0;                     // resets switch closures that occurred during initialization
    cm.safety_interlock_disengaged = 0;         // ditto
    cm.safety_interlock_reengaged = 0;          // ditto
    cm.shutdown_requested = 0;                  // ditto

	// signal that the machine is ready for action
    cm.cycle_state = CYCLE_OFF;
    cm.motion_state = MOTION_STOP;
    cm.hold_state = FEEDHOLD_OFF;
	cm.gmx.block_delete_switch = true;
	cm.gm.motion_mode = MOTION_MODE_CANCEL;     // never start a machine in a motion mode
	cm.machine_state = MACHINE_READY;
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
    if ((BAD_MAGIC(cm.magic_start)) || (BAD_MAGIC(cm.magic_end)) ||
        (BAD_MAGIC(cm.gmx.magic_start)) || (BAD_MAGIC(cm.gmx.magic_end)) ||
        (BAD_MAGIC(arc.magic_start)) || (BAD_MAGIC(arc.magic_end))) {
        return(cm_panic_P(STAT_CANONICAL_MACHINE_ASSERTION_FAILURE, PSTR("canonical_machine_test_assertions()")));
    }
    return (STAT_OK);
}

/**************************
 * Alarms                 *
 **************************/

/********************************************************************************
 *  ALARM, SHUTDOWN, and PANIC are nested dolls.
 *
 * cm_alrm()  - command-line wrapper for alarm
 * cm_shutd() - command-line wrapper for shutdown from command
 * cm_pnic()  - command-line wrapper for panic from command
 * cm_clr()   - command-line wrapper for clear alarm or shutdown
 *
 * The alarm states can be invoked from the above commands for testing and clearing
 */
stat_t cm_alrm(nvObj_t *nv)               // invoke alarm from command
{
    cm_alarm_P(STAT_ALARM, PSTR("sent by host"));
    return (STAT_OK);
}

stat_t cm_shutd(nvObj_t *nv)              // invoke shutdown from command
{
    char msg_buf[sizeof("sent by host")]; 
    strcpy_P(msg_buf, PSTR("sent by host"));
    cm_shutdown(STAT_SHUTDOWN, msg_buf);
    return (STAT_OK);
}

stat_t cm_pnic(nvObj_t *nv)               // invoke panic from command
{
    cm_panic_P(STAT_PANIC, PSTR("sent by host"));
    return (STAT_OK);
}

stat_t cm_clr(nvObj_t *nv)                // clear alarm or shutdown from command line
{
    cm_clear();
    return (STAT_OK);
}

/*
 * cm_clear() - clear ALARM and SHUTDOWN states
 */

void cm_clear()
{
    if (cm.machine_state == MACHINE_ALARM) {
        cm.machine_state = MACHINE_PROGRAM_STOP;
        // An alarm latches a hold (E044). Once nothing is left to run (e.g. after a '%' flush)
        // no '~' will come, so clearing also ends the hold and the cycle; otherwise the cycle stays
        // MACHINING and persisted settings are refused as 'moving' until a reset (E144).
#ifdef __AVR
        uint8_t saved_sreg = SREG;
        cli();
#endif
        // Drained means no queued block, an idle runtime AND no prepared segment waiting for the
        // loader (the last block can be freed before its final segment loads); snapshot together.
        bool drained = !mp_has_runnable_buffer() && mp_runtime_is_idle() &&
                       (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER);
#ifdef __AVR
        SREG = saved_sreg;
#endif
        // once drained only the main loop (this code) can queue motion, so act outside the cli window
        if (drained) {
            if (cm.hold_state != FEEDHOLD_OFF) {
                cm.end_hold_requested = false;
                mp_exit_hold_state();
            }
            cm_cycle_end();                     // no-op if the cycle already ended
        }
    } else if (cm.machine_state == MACHINE_SHUTDOWN) {
        cm.machine_state = MACHINE_READY;
    }
}

/*
 * cm_is_alarmed() - return alarm status code or OK if no alarms
 */

stat_t cm_is_alarmed()
{
    if (cm.machine_state == MACHINE_ALARM)    { return (STAT_COMMAND_REJECTED_BY_ALARM); }
    if (cm.machine_state == MACHINE_PANIC)    { return (STAT_COMMAND_REJECTED_BY_PANIC); }
    if (cm.machine_state == MACHINE_SHUTDOWN) { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
    return (STAT_OK);
}

/*
 * cm_halt_all() - stop, spindle and coolant immediately
 * cm_halt_motion() - stop motion immediately. Does not affect spindle, coolant, or other IO
 *
 * Stop motors and reset all system states accordingly.
 * Does not de-energize motors as in some cases the motors must remain energized
 * in order to prevent an axis from crashing.
 */

void cm_halt_all(void)
{
    cm_halt_motion();
//	cm_spindle_off_immediate();
//	cm_coolant_off_immediate();
}

void cm_halt_motion(void)
{
    mp_halt_runtime();                  // stop the runtime. Do this immediately. (Reset is in cm_clear)
    canonical_machine_reset();          // reset Gcode model
	cm.cycle_state = CYCLE_OFF;         // Note: leaves machine_state alone
	cm.motion_state = MOTION_STOP;
	cm.hold_state = FEEDHOLD_OFF;
}

/*
 * cm_alarm()   - enter ALARM state
 * cm_alarm_P() - enter ALARM state - accepts message as program string
 *
 * An ALARM sets the ALARM machine state, starts a feedhold to stop motion, stops the
 * spindle, turns off coolant, clears out queued planner moves and serial input,
 * and rejects new action commands (gcode blocks, SET commands, and other actions)
 * until the alarm is cleared.
 *
 * ALARM is typically entered by a soft limit or a limit switch being hit. In the
 * limit switch case the INPUT_ACTION will override the feedhold - i.e. if the
 * input action is "FAST_STOP" or "HALT" that setting will take precedence over
 * the feedhold native to the alarm function.
 *
 * Gcode and machine state is preserved. It may be possible to recover the job from
 * an alarm, but in many cases this is not possible. Since ALARM attempts to preserve
 * Gcode and machine state it does not END the job.
 *
 * ALARM may also be invoked from the command line using {alarm:n} or $alarm
 * ALARM can be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * ALARMs will also clear on receipt of an M30 or M2 command if one is received
 * while draining the host command queue.
 */

stat_t cm_alarm_P(const stat_t status, const char *msg_P)
{
    char msg_buf[STATUS_MSG_LEN];
    strcpy_P(msg_buf, msg_P);
    return(cm_alarm(status, msg_buf));
}

stat_t cm_alarm(const stat_t status, const char *msg)
{
    if ((cm.machine_state == MACHINE_ALARM) ||
        (cm.machine_state == MACHINE_PANIC) ||
        (cm.machine_state == MACHINE_SHUTDOWN)) {
        return (STAT_OK);                       // don't alarm if already in an alarm state
    }
	cm.machine_state = MACHINE_ALARM;

    // Nothing moving (idle, a dwell, moves not started yet) means no hold will ever start, so the
    // flush would wait forever while queued moves run in ALARM (E044). Decide that atomically with
    // the exec: with interrupts off, a stopped runtime is put straight into HOLD, which keeps
    // mp_exec_aline() from starting another move. A move already running takes the normal hold.
#ifdef __AVR
    uint8_t sreg = SREG;
    cli();
#endif
    bool idle = (cm.motion_state == MOTION_STOP) && (cm.hold_state == FEEDHOLD_OFF);
    bool segment_in_flight = false;
    if (idle) {
        segment_in_flight = st_aline_in_flight();   // a block's last segment can outlive MOTION_STOP
        cm_set_motion_state(MOTION_HOLD);
        cm.hold_state = FEEDHOLD_HOLD;          // no new move starts; a last segment finishes on its own
    }
#ifdef __AVR
    SREG = sreg;
#endif
    if (idle) {
        if (!segment_in_flight) {
            stepper_reset();                    // only a dwell (or nothing) is running: stop it now
        }
        cm.queue_flush_state = FLUSH_REQUESTED; // flush input and planner once the runtime is idle
    } else {
        cm_request_feedhold();                  // stop motion
        cm_request_planner_flush();             // flush the planner once runtime is not busy; host input is
                                                // kept and rejected line by line while alarmed (E145)
    }

    spindle_reset();                            // stop spindle immediately (a queued M5 would be flushed); S is kept
    cm_coolant_off_immediate();                 // stop coolant immediately
//	cm_spindle_optional_pause(spindle.pause_on_hold);
//	cm_coolant_optional_pause(coolant.pause_on_hold);

	rpt_exception(status, msg);	                // send alarm message
    return (status);
}

/*
 * cm_shutdown() - enter shutdown state
 *
 * SHUTDOWN stops all motion, spindle and coolant immediately, sets a SHUTDOWN machine
 * state, clears out queued moves and serial input, and rejects new action commands
 * (gcode blocks, SET commands, and some others).
 *
 * Shutdown is typically invoked as an electrical input signal sent to the board as
 * part of an external emergency stop (Estop). Shutdown is meant to augment but not
 * replace the external Estop functions that shut down power to motors, spindles and
 * other moving parts.
 *
 * Shutdown may also be invoked from the command line using {shutd:n} or $shutd
 * Shutdown must be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * Shutdown does not clear on M30 or M2 Gcode commands
 */

stat_t cm_shutdown(const stat_t status, const char *msg)
{
    if ((cm.machine_state == MACHINE_SHUTDOWN) || (cm.machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't shutdown if shutdown or panic'd
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    spindle_reset();                            // stop spindle immediately (S is kept, like M5)
    cm_coolant_off_immediate();                 // stop coolant immediately
    cm_queue_flush();                           // flush all queues and reset positions

    for (uint8_t i = 0; i < HOMING_AXES; i++) { // unhome axes and the machine
        cm.homed[i] = false;
    }
    cm.homing_state = HOMING_NOT_HOMED;

	cm.machine_state = MACHINE_SHUTDOWN;        // do this after all other activity
	rpt_exception(status, msg);	                // send exception report
    return (status);
}

/*
 * cm_panic()   - enter PANIC state
 * cm_panic_P() - enter PANIC state - accepts message as program string
 *
 * PANIC occurs if the firmware has detected an unrecoverable internal error
 * such as an assertion failure or a code condition that should never occur.
 * It sets PANIC machine state, and leaves the system inspect able (if possible).
 *
 * PANIC can only be exited by a hardware reset or soft reset (^x)
 */

stat_t cm_panic_P(const stat_t status, const char *msg_P)
{
    char msg_buf[STATUS_MSG_LEN];
    strcpy_P(msg_buf, msg_P);
    return(cm_panic(status, msg_buf));
}

stat_t cm_panic(const stat_t status, const char *msg)
{
    if (cm.machine_state == MACHINE_PANIC) {    // only do this once
        return (STAT_OK);
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    spindle_reset();                            // stop spindle immediately (S is kept, like M5)
    cm_coolant_off_immediate();                 // stop coolant immediately
    cm_queue_flush();                           // flush all queues and reset positions

	cm.machine_state = MACHINE_PANIC;           // don't reset anything. Panics are not recoverable
	rpt_exception(status, msg);			        // send panic report
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
 *  cm_set_arc_distance_mode()  - G90.1, G91.1
 *	cm_set_coord_offsets()		- G10 (delayed persistence)
 *
 *	These functions assume input validation occurred upstream.
 */

stat_t cm_select_plane(const uint8_t plane)
{
    cm.gm.select_plane = (cmCanonicalPlane)plane;
    return (STAT_OK);
}

stat_t cm_set_units_mode(const uint8_t mode)
{
    cm.gm.units_mode = (cmUnitsMode)mode;		    // 0 = inches, 1 = mm.
    return(STAT_OK);
}

stat_t cm_set_distance_mode(const uint8_t mode)
{
    cm.gm.distance_mode = (cmDistanceMode)mode;		// 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

stat_t cm_set_arc_distance_mode(const uint8_t mode)
{
    cm.gm.arc_distance_mode = (cmDistanceMode)mode;	// 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

/*
 * cm_set_coord_offsets() - G10 L2/L20 Pn (model now, runtime in queue order)
 *
 *	This function applies the offset to the GM model but does not persist the offsets
 *	during the Gcode cycle. The persist flag is used to persist offsets once the cycle
 *	has ended. You can also use $g54x - $g59c config functions to change offsets.
 *
 *	Edits to the active system capture new model offsets and queue a runtime snapshot.
 *
 *  Both L2 and L20 forms are supported
 */

stat_t cm_set_coord_offsets(const float coord_system,
                            const uint8_t L_word,
                            const float offset[], const bool flag[])
{
    if ((!cm.gf.parameter) || !(coord_system >= 0 && coord_system <= COORD_SYSTEM_MAX) ||
        (coord_system != floor(coord_system))) {
        return (STAT_P_WORD_IS_INVALID);
    }
    uint8_t selected = (coord_system == 0) ? cm.gm.coord_system : (uint8_t)coord_system;
    if ((selected < G54) || (selected > COORD_SYSTEM_MAX)) { // you can't set G53
        return (STAT_P_WORD_IS_INVALID);
    }
    if (!cm.gf.L_word) {
        return (STAT_L_WORD_IS_MISSING);
    }
    if ((L_word != 2) && (L_word != 20)) {
        return (STAT_L_WORD_IS_INVALID);
    }
    cm_set_work_offsets();
    bool changed = false;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            float value = (axis < AXIS_A) ? _to_millimeters(offset[axis]) : offset[axis];
            if (L_word == 2) {
                cm.offset[selected][axis] = value;
            } else {
                // Isolate signed tool compensation using the same convention as the DRO.
                float tool = cm.gm.work_offset[axis] - cm_get_active_coord_offset(axis);
                float origin = cm.gmx.origin_offset_enable ? cm.gmx.origin_offset[axis] : 0;
                cm.offset[selected][axis] = cm.gmx.position[axis] - value - origin - tool;
            }
            changed = true;
            cm.deferred_write_flag = true; // persist once the machining cycle is over
        }
    }
    // An idle stop-and-wait host may send the next line as soon as our response
    // arrives. Finish EEPROM writes before returning to the parser, not after
    // that response. During motion, retain the existing deferred write-back.
    if (changed && (cm.cycle_state == CYCLE_OFF) && !st_aline_in_flight()) {
        _persist_coord_offsets();
    }
    if (changed && (selected == cm.gm.coord_system)) {
        cm_set_work_offsets();
        mp_queue_command(_exec_g10_offset, cm.gm.work_offset, FLAGS_NONE);
    }
    return (STAT_OK);
}

static void _exec_g10_offset(float *value, bool *flags)
{
    mp_set_runtime_work_offset(value); // use the queued snapshot, not later model edits
}

/******************************************************************************************
 * Representation functions that affect gcode model and are queued to planner (synchronous)
 */
/*
 * cm_set_coord_system() - G54-G59
 * _exec_offset() - callback from planner
 */
stat_t cm_set_coord_system(const uint8_t coord_system)
{
    cm.gm.coord_system = (cmCoordSystem)coord_system;

    float value[AXES] = { (float)coord_system };	            // pass coordinate system in value[0] element
    mp_queue_command(_exec_offset, value, FLAGS_ONE);	    // NB: flags vector is not actually used
	return (STAT_OK);
}

static void _exec_offset(float *value, bool *flags)
{
	uint8_t coord_system = ((uint8_t)value[0]);				// coordinate system is passed in value[0] element
	float offsets[AXES];
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		offsets[axis] = cm.offset[coord_system][axis] + (cm.gmx.origin_offset[axis] * cm.gmx.origin_offset_enable);
	}
	mp_set_runtime_work_offset(offsets);
	cm_set_work_offsets();								    // set work offsets in the Gcode model
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

void cm_set_position(const uint8_t axis, const float position)
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

stat_t cm_set_absolute_origin(const float origin[], bool flags[])
{
	float value[AXES];

	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (flags[axis]) {
			value[axis] = (axis < AXIS_A) ? _to_millimeters(origin[axis]) : origin[axis];	// rotary words are degrees (E155)
			cm.gmx.position[axis] = value[axis];		// set model position
			cm.gm.target[axis] = value[axis];			// reset model target
			mp_set_planner_position(axis, value[axis]);	// set mm position
		}
	}
	mp_queue_command(_exec_absolute_origin, value, flags);
	return (STAT_OK);
}

static void _exec_absolute_origin(float *value, bool *flags)
{
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (flags[axis]) {
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
stat_t cm_set_origin_offsets(const float offset[], bool flags[])
{
	// set offsets in the Gcode model extended context
	cm.gmx.origin_offset_enable = 1;
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (flags[axis]) {
			cm.gmx.origin_offset[axis] = cm.gmx.position[axis] - cm.offset[cm.gm.coord_system][axis] -
									  cm.gmx.tool_offset[axis] - ((axis < AXIS_A) ? _to_millimeters(offset[axis]) : offset[axis]);	// work position = offset
		}
	}
	// now pass the offset to the callback - setting the coordinate system also applies the offsets
	float value[AXES] = { (float)cm.gm.coord_system };  // pass coordinate system in value[0] element
	mp_queue_command(_exec_offset, value, flags);       // NB: flags are not used
	return (STAT_OK);
}

stat_t cm_reset_origin_offsets()
{
	cm.gmx.origin_offset_enable = 0;
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm.gmx.origin_offset[axis] = 0;
	}
	float value[AXES] = { (float)cm.gm.coord_system };
	mp_queue_command(_exec_offset, value, FLAGS_ONE);
	return (STAT_OK);
}

stat_t cm_suspend_origin_offsets()
{
	cm.gmx.origin_offset_enable = 0;
	float value[AXES] = { (float)cm.gm.coord_system };
	mp_queue_command(_exec_offset, value, FLAGS_ONE);
	return (STAT_OK);
}

stat_t cm_resume_origin_offsets()
{
	cm.gmx.origin_offset_enable = 1;
	float value[AXES] = { (float)cm.gm.coord_system };
	mp_queue_command(_exec_offset, value, FLAGS_ONE);
	return (STAT_OK);
}

/*****************************
 * Free Space Motion (4.3.4) *
 *****************************/
/*
 * cm_straight_traverse() - G0 linear rapid
 */

/*
 * _queue_line() - plan the move in cm.gm inside a machining cycle (G0/G1)
 *
 *	The cycle starts before the move is queued so a short move cannot finish and end the cycle
 *	before it was started. But mp_aline() plans before it reserves its buffer, and meanwhile the
 *	runtime can finish the previous last block and end the cycle (plan_exec.c, mp_free_run_buffer()
 *	then cm_cycle_end()). So once the move is queued, check again with interrupts off: restart the
 *	cycle if anything is queued or running, end it if nothing is (a too-short or rejected move).
 */
static stat_t _queue_line()
{
	cm_cycle_start();
	stat_t status = mp_aline(&cm.gm);
	if (cm_is_alarmed() != STAT_OK) {			// mp_aline can alarm or panic (failed reservation): the
		return (status);						// cycle repair below must not overwrite that state
	}
#ifdef __AVR
	uint8_t sreg = SREG;
	cli();
#endif
	bool runnable = mp_has_runnable_buffer();
	if (runnable) {
		cm_cycle_start();						// cheap; must not race the runtime's cycle end
	}
#ifdef __AVR
	SREG = sreg;
#endif
	if (!runnable) {
		cm_cycle_end();							// only the main loop queues, so this can't race a new move
	}
	if ((status == STAT_MINIMUM_LENGTH_MOVE) || (status == STAT_MINIMUM_TIME_MOVE)) {
		status = STAT_OK;						// too short to plan; not an error
	}
	return (status);
}

static stat_t _straight_traverse(const float target[], const bool flags[], const bool machine_coordinates)
{
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;

    // it's legal for a G0 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] || flags[AXIS_Y] || flags[AXIS_Z] ||
          flags[AXIS_A] || flags[AXIS_B] || flags[AXIS_C])) {
        return(STAT_OK);
    }
    if (machine_coordinates) {
        copy_vector(cm.gm.target, cm.gmx.position);
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flags[axis] && (cm.a[axis].axis_mode != AXIS_DISABLED)) {
                cm.gm.target[axis] = target[axis];  // stored machine coordinates are already mm/degrees
            }
        }
    } else {
        cm_set_model_target(target, flags);
    }

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) {
        return (cm_alarm(status, cs.saved_buf));
    }
	// prep and plan the move
	cm_set_work_offsets();				        // capture the fully resolved offsets to the state

    status = _queue_line();                     // plan the move inside a machining cycle
    cm_finalize_move();
	return (status);
}

stat_t cm_straight_traverse(const float target[], const bool flags[])
{
    return (_straight_traverse(target, flags, false));
}

stat_t cm_cancel_motion_mode()
{
	cm.gm.motion_mode = MOTION_MODE_CANCEL;    // modal only: queued moves keep their cycle
    return (STAT_OK);
}

/* M114/M115 report immediately; they do not wait for queued motion. */
stat_t cm_get_position(void)
{
    float positions[AXES];
    cm_position_snapshot_begin();                   // all six axes from one instant, read as pos is (E166)
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        positions[axis] = _get_report_position(axis);
    }
    cm_position_snapshot_end();
    char token[5];
    strcpy_P(token, PSTR("posx"));
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        char letter = pgm_read_byte(PSTR("XYZABC") + axis);
        float position = positions[axis];
        if (cs.comm_mode == TEXT_MODE) {
            fprintf_P(stderr, PSTR("%c:%.4f%c"), letter, position, (axis == AXES-1) ? '\n' : ' ');
        } else {
            token[3] = letter + ('a' - 'A');
            nvObj_t *nv = nv_add_object(token);
            if (nv == NULL) { return (STAT_JSON_TOO_MANY_PAIRS); }
            // Response fields are peers of gc, not members of the pos group.
            strcpy(nv->token, token);
            nv->group[0] = NUL;
            nv->depth = 1;
            nv->value_flt = position;
            nv->precision = 4;
        }
    }
    return (STAT_OK);
}

stat_t cm_get_firmware(void)
{
    if (cs.comm_mode == TEXT_MODE) {
        fprintf_P(stderr, PSTR("FIRMWARE_NAME:TinyG, FIRMWARE_VERSION:%.2f, FIRMWARE_BUILD:%.2f\n"),
                  cs.fw_version, cs.fw_build);
    } else {
        char token[3];
        strcpy_P(token, PSTR("fb"));
        nvObj_t *nv = nv_add_object(token);
        if (nv == NULL) { return (STAT_JSON_TOO_MANY_PAIRS); }
        nv->depth = 1;
        strcpy_P(token, PSTR("fv"));
        nv = nv_add_object(token);
        if (nv == NULL) { return (STAT_JSON_TOO_MANY_PAIRS); }
        nv->depth = 1;
    }
    return (STAT_OK);
}

/*
 * cm_set_g28_position()  - G28.1
 * cm_goto_g28_position() - G28
 * cm_set_g30_position()  - G30.1
 * cm_goto_g30_position() - G30
 */

static stat_t _goto_stored_position(const float stored[], const float target[], const bool flags[])
{
    // Normal dispatch reserves this headroom; reject ungated calls instead of blocking in a hold.
    if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) {
        return (STAT_BUFFER_FULL);
    }
    uint8_t saved_override = cm.gm.absolute_override;
    cm_set_absolute_override(false);
    stat_t status = cm_straight_traverse(target, flags); // intermediate point uses work coordinates
    if (status == STAT_OK) {
        cm_set_absolute_override(true);
        status = _straight_traverse(stored, FLAGS_ALL, true);
    }
    cm_set_absolute_override(saved_override);
    return (status);
}

stat_t cm_set_g28_position(void)
{
	copy_vector(cm.gmx.g28_position, cm.gmx.position);
	return (STAT_OK);
}

stat_t cm_goto_g28_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g28_position, target, flags));
}

stat_t cm_set_g30_position(void)
{
	copy_vector(cm.gmx.g30_position, cm.gmx.position);
	return (STAT_OK);
}

stat_t cm_goto_g30_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g30_position, target, flags));
}

/********************************
 * Machining Attributes (4.3.5) *
 ********************************/
/*
 * cm_set_feed_rate() - F parameter (affects MODEL only)
 *
 * Normalize feed rate to mm/min or to minutes if in inverse time mode
 */

stat_t cm_set_feed_rate(const float feed_rate)
{
	if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {
        if (!(feed_rate > 0) || !isfinite(feed_rate)) { return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED); }
        float inverse_time = 1 / feed_rate;
        if (!isfinite(inverse_time)) { return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED); }
        cm.gm.feed_rate = inverse_time;	// minutes, active for this block only
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

stat_t cm_set_feed_rate_mode(const uint8_t mode)
{
	cm.gm.feed_rate_mode = mode;
	return (STAT_OK);
}

/*
 * cm_set_path_control() - G61, G61.1, G64 (affects MODEL only)
 */

stat_t cm_set_path_control(const uint8_t mode)
{
	cm.gm.path_control = mode;
	return (STAT_OK);
}
/*
stat_t cm_set_path_control(GCodeState_t *gcode_state, const uint8_t mode)
{
    gcode_state->path_control = (cmPathControl)mode;
    return (STAT_OK);
}
*/

/*******************************
 * Machining Functions (4.3.6) *
 *******************************/
/*
 * cm_arc_feed() - SEE plan_arc.c(pp)
 */

/*
 * cm_dwell() - G4, P parameter (seconds)
 */
stat_t cm_dwell(const float seconds)
{
	if (seconds < 0) { return (STAT_P_WORD_IS_NEGATIVE); }
	mp_dwell(seconds);
	return (STAT_OK);
}

/*
 * cm_straight_feed() - G1
 */
stat_t cm_straight_feed(const float target[], const bool flags[])
{
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

    // it's legal for a G1 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] || flags[AXIS_Y] || flags[AXIS_Z] ||
          flags[AXIS_A] || flags[AXIS_B] || flags[AXIS_C])) {
        return(STAT_OK);
    }
	// trap zero feed rate condition
	if ((cm.gm.feed_rate_mode == INVERSE_TIME_MODE) ? (cm.gm.feed_rate <= 0) : fp_ZERO(cm.gm.feed_rate)) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}
	cm_set_model_target(target, flags);

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) {
        return (cm_alarm(status, cs.saved_buf));
    }
	// prep and plan the move
	cm_set_work_offsets();                      // capture the fully resolved offsets to the state
    status = _queue_line();                     // plan the move inside a machining cycle
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
 *
 * cm_change_tool()		- M6 (This might become a complete tool change cycle)
 * _exec_change_tool()	- execution callback
 *
 * Note: These functions don't actually do anything for now, and there's a bug
 *		 where T and M in different blocks don't work correctly
 */
stat_t cm_select_tool(const uint8_t tool_select)
{
	// T only selects (RS274NGC 3.7.3): set the model now, so an M6 in the same block or behind
	// queued moves changes to this tool. Queuing it let M6 snapshot the previous selection.
	cm.gm.tool_select = tool_select;
	return (STAT_OK);
}

stat_t cm_change_tool(const uint8_t tool_change)
{
	float value[AXES] = { (float)cm.gm.tool_select };
	mp_queue_command(_exec_change_tool, value, FLAGS_ONE);
	return (STAT_OK);
}

static void _exec_change_tool(float *value, bool *flags)
{
    cm.gm.tool = (uint8_t)value[0];
}

/* Tool length offsets
 *
 * cm_tool_offset_set()    - G43.1
 * cm_tool_offset_cancel() - G49
 *
 *  Tool length offset is positive for a longer tool: it moves the controlled point out along
 *  the spindle axis (RS274NGC v3 2.1.2.3, same as LinuxCNC and g2core). If Z is at 0,
 *  G43.1 Z10 makes Z appear to be at -10 mm (the tool tip is 10 mm below the gauge point),
 *  and a following G0 Z0 raises the spindle 10 mm. Before build 449.02 edge used the
 *  opposite sign.
 *
 *  G43.1 does not cause any movement but will change status report displays
 *  (DRO values). Movement will occur on next feed or traverse.
 *
 *  Changes to tool length offsets take effect immediately (i.e. in the model)
 *  As they affect the creation of new moves. The display of tool length
 *  changes must be synchronized with the first block affected by the change,
 *  hence the offset is captured in the work_offset term and passed to the
 *  runtime model.
 *
 *  Canoncial_machine functions affected by tool length offset:
 *  - cm_set_model_target()
 *  - cm_set_work_offsets()
 *  - cm_get_work_position()
 */

stat_t cm_tool_offset_set(const float target[], const bool flags[])
{
    bool flag = false;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (!flags[axis]) {
            continue;
        }
        cm.gmx.tool_offset[axis] = (axis < AXIS_A) ? _to_millimeters(target[axis]) : target[axis];   // degrees for A/B/C (E155)
        flag = true;
    }
    if (!flag) {
       return (STAT_GCODE_AXIS_IS_MISSING);
    }
    return (STAT_OK);
}

stat_t cm_tool_offset_cancel()
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm.gmx.tool_offset[axis] = 0;
    }
    return (STAT_OK);
}

/***********************************
 * Miscellaneous Functions (4.3.9) *
 ***********************************/
/*
 * cm_mist_coolant_control() - M7
 * cm_flood_coolant_control() - M8, M9
 */

stat_t cm_mist_coolant_control(const uint8_t mist_coolant)
{
	float value[AXES] = { (float)mist_coolant };
	mp_queue_command(_exec_mist_coolant_control, value, FLAGS_ONE);
	return (STAT_OK);
}
// Coolant outputs as last driven, for the g2core-named cof and com readouts: the model runs ahead of the
// outputs (M2/M30 turns it off when parsed), so the readouts follow the executors instead.
static volatile uint8_t coolant_out_mist, coolant_out_flood;

stat_t coolant_get_cof(nvObj_t *nv) { nv->value_int = coolant_out_flood; nv->valuetype = TYPE_INTEGER; return (STAT_OK); }
stat_t coolant_get_com(nvObj_t *nv) { nv->value_int = coolant_out_mist; nv->valuetype = TYPE_INTEGER; return (STAT_OK); }

static void _exec_mist_coolant_control(float *value, bool *flags)
{
	cm.gm.mist_coolant = cm_outputs_inhibited() ? false : (uint8_t)value[0];
	coolant_out_mist = cm.gm.mist_coolant;

#ifdef __AVR
	if (cm.gm.mist_coolant == true) {
		gpio_set_bit_on(MIST_COOLANT_BIT);
    } else {
	    gpio_set_bit_off(MIST_COOLANT_BIT);
    }
#endif // __AVR

#ifdef __ARM
	if (cm.gm.mist_coolant == true) {
		coolant_enable_pin.set();
    } else {
	    coolant_enable_pin.clear();
    }
#endif // __ARM
}

stat_t cm_flood_coolant_control(const uint8_t flood_coolant)
{
	float value[AXES] = { (float)flood_coolant };
	mp_queue_command(_exec_flood_coolant_control, value, FLAGS_ONE);
	return (STAT_OK);
}
/*
 * cm_outputs_inhibited() - true in ALARM, SHUTDOWN and PANIC: spindle and coolant may only turn off
 *
 *  A queued M3/M7/M8 can still reach its exec function after an alarm (e.g. at the end of a running
 *  dwell, before the queue flush runs), so the exec functions check this at execution time.
 */
bool cm_outputs_inhibited()
{
    return ((cm.machine_state == MACHINE_ALARM) || (cm.machine_state == MACHINE_SHUTDOWN) ||
            (cm.machine_state == MACHINE_PANIC));
}

/*
 * cm_coolant_off_immediate() - turn mist and flood coolant off now (alarm, shutdown, panic)
 */
void cm_coolant_off_immediate()
{
    float value[] = { 0,0,0,0,0,0 };
    _exec_flood_coolant_control(value, FLAGS_NONE);   // the M9 path also turns mist off
}

static void _exec_flood_coolant_control(float *value, bool *flags)
{
	cm.gm.flood_coolant = cm_outputs_inhibited() ? false : (uint8_t)value[0];
	coolant_out_flood = cm.gm.flood_coolant;

#ifdef __AVR
	if (cm.gm.flood_coolant == true) {
		gpio_set_bit_on(FLOOD_COOLANT_BIT);
	} else {
		gpio_set_bit_off(FLOOD_COOLANT_BIT);
		float vect[] = { 0,0,0,0,0,0 };				    // turn off mist coolant
		_exec_mist_coolant_control(vect, FLAGS_NONE);   // M9 special function
	}
#endif // __AVR

#ifdef __ARM
	if (cm.gm.flood_coolant == true) {
		coolant_enable_pin.set();
	} else {
		coolant_enable_pin.clear();
		float vect[] = { 0,0,0,0,0,0 };				    // turn off mist coolant
		_exec_mist_coolant_control(vect, FLAGS_NONE);   // M9 special function
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
	if (cm.gf.parameter && fp_ZERO(cm.gn.parameter)) {
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
	return (STAT_OK);
}

stat_t cm_traverse_override_enable(uint8_t flag)	// M50.2
{
	if (cm.gf.parameter && fp_ZERO(cm.gn.parameter)) {
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
	return (STAT_OK);
}

stat_t cm_spindle_override_enable(uint8_t flag)		// M51.1
{
	if (cm.gf.parameter && fp_ZERO(cm.gn.parameter)) {
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
//	STUBBED: change spindle speed
	return (STAT_OK);
}

/*
 * cm_message() - queue a RAM string as a message in the response (unconditionally)
 *
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

void cm_message(char *message)
{
	nv_add_string((const char *)"msg", message);	// add message to the response object
}

/**************************************************
 * Feedhold and Related Functions (no NIST ref #) *
 **************************************************/
/*
 * Feedholds, queue flushes and end_holds are all related. The request functions change
 * states to "_____REQUESTED". The sequencing callback interprets the flags as so:
 *
 *    - A feedhold request received during motion should be honored
 *    - A feedhold request received during a feedhold should be ignored
 *    - A feedhold request received during a motion stop should be ignored
 *
 *    - A queue flush request received during a motion stop should be honored
 *    - A queue flush request received during a feedhold should be honored.
 *      A request received during a feedhold should be deferred until cmFeedholdState
 *      enters FEEDHOLD_HOLD (i.e. until deceleration is complete and motors stop).
 *
 *    - An end_hold (cycle start) request should only be honored while in a feedhold
 *    - Said end_hold request received during a feedhold should be deferred until the
 *      feedhold enters a HOLD state (i.e. until deceleration is complete).
 *      If a queue flush request is also present the queue flush should be done first
 *
 *	Below the request level, feedholds work like this:
 *
 *    - The hold is initiated by calling cm_start_hold()
 *      cm.hold_state is set to FEEDHOLD_SYNC, motion_state is set to MOTION_HOLD
 *      The spindle is turned off (if it is on)
 *      The remainder of feedhold processing occurs in plan_exec.c / mp_exec_aline()
 *
 *	  - MOTION_HOLD and FEEDHOLD_SYNC tells mp_exec_aline() to begin feedhold processing
 *      after the current move segment is finished (< 5 ms later).
 *      See plan_exec.c for details of how feedholds are handled. In short:
 *
 *      - FEEDHOLD_SYNC causes the current move in mr to be replanned into a deceleration.
 *        If the distance remaining in the mr move is sufficient for a full deceleration
 *        then motion will stop in the current block. Otherwise the deceleration phase
 *        will extend across as many blocks necessary until one will stop.
 *
 *      - Once deceleration is complete hold_state transitions to FEEDHOLD_HOLD and the
 *        distance remaining in the bf last block is replanned up from zero velocity.
 *        The move in the bf block is NOT released (unlike normal operation), as it
 *        will be used again to restart from hold.
 *
 *      - When cm_end_hold() is called it releases the hold, restarts the move and restarts
 *        the spindle if the spindle is active.
 */
/*
 * cm_request_feedhold()
 * cm_request_end_hold() - cycle restart
 * cm_request_queue_flush()
 */
/*
 * cm_request_job_kill() - ^d from the RX ISR (E096)
 *
 *	The job kill is an ALARM (g2core Job-Exception-Handling): motion stops, spindle and coolant stop,
 *	the planner is flushed and commands are rejected until {clear:n}, $clear, M2 or M30, in or out of
 *	a feedhold. The ISR only flags it; cm_feedhold_sequencing_callback() raises the alarm.
 */
void cm_request_job_kill(void)
{
    rt_qt_invalidate();
    cm.job_kill_requested = true;
}

void cm_request_feedhold(void) {
    if (mp_hold_startup()) { return; }
    // honor request if not already in a feedhold and you are moving
    if ((cm.hold_state == FEEDHOLD_OFF) && (cm.motion_state != MOTION_STOP)) {
        cm.hold_state = FEEDHOLD_REQUESTED;
    }
}

void cm_request_end_hold(void)
{
    if (cm.hold_state != FEEDHOLD_OFF) {
        cm.end_hold_requested = true;
    }
}

void cm_request_queue_flush()
{
    cm_request_planner_flush();
    if (cm.queue_flush_state == FLUSH_REQUESTED) {  // honored now, or already pending from an alarm
        rt_qt_invalidate();
        cm.queue_flush_input = true;                // the host asked: drop what it sent before the '%'
    }
}

void cm_request_planner_flush()
{
    if ((cm.hold_state != FEEDHOLD_OFF) &&          // don't honor request unless you are in a feedhold
        (cm.queue_flush_state == FLUSH_OFF)) {      // ...and only once
        cm.queue_flush_state = FLUSH_REQUESTED;     // flushed from the main loop
    }
}

/*
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 */

/*
 * _run_requested_queue_flush() - complete a host-requested flush ('%' or alarm) from the main loop
 *
 *  Waits until motion has stopped and no hold is still decelerating, then flushes the planner and,
 *  for a host '%', drops all buffered input (lines received before the '%'). An alarm's flush keeps
 *  host input: those lines are rejected one by one while alarmed (E145). cm_queue_flush() consumes the
 *  host's request atomically with the end of the flush and cm_feedhold_sequencing_callback() drops the
 *  input. A hold that was requested but never
 *  started (nothing was moving) is cancelled rather than waited for. Direct callers of
 *  cm_queue_flush() (homing, panic) keep the host input unless a '%' was pending.
 */
// Hand a pending host '%' to queue_flush_input_due, atomically against the RX ISR that sets it (E145)
static void _take_input_flush_request()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    if (cm.queue_flush_input) {
        cm.queue_flush_input = false;
        cm.queue_flush_input_due = true;
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

// Drop the input a host '%' asked for; main loop only, never inside a direct flush (E145)
static void _service_input_flush()
{
    if (cm.queue_flush_input_due) {
        cm.queue_flush_input_due = false;
        xio_flush_input();
    }
}

static void _run_requested_queue_flush()
{
    if (!mp_runtime_is_idle() ||
        ((cm.hold_state != FEEDHOLD_OFF) && (cm.hold_state != FEEDHOLD_REQUESTED) &&
         (cm.hold_state != FEEDHOLD_HOLD))) {
        return;
    }
    _take_input_flush_request();                    // drop a host '%''s input before the planner work below,
    _service_input_flush();                         // so a line sent right after the '%' is not cut (E145)
    cm_queue_flush();                               // a '%' landing meanwhile is handed off and serviced next
    if (cm.hold_state == FEEDHOLD_REQUESTED) {
        cm.hold_state = FEEDHOLD_OFF;
        cm.end_hold_requested = false;              // a '~' for the cancelled hold must not resume a later one
    }
}

stat_t cm_feedhold_sequencing_callback()
{
    if (cm.job_kill_requested) {
        cm.job_kill_requested = false;
        cm_alarm_P(STAT_KILL_JOB, PSTR("job kill"));  // requests the hold and flush handled below
    }
	if ((cm.hold_state == FEEDHOLD_REQUESTED) || (cm.hold_state == FEEDHOLD_SYNC)) {
		cm_start_hold();                            // feed won't run unless the machine is moving
	}
	if (cm.queue_flush_state == FLUSH_REQUESTED) {
        _run_requested_queue_flush();               // runs once motion has stopped
	}
	_service_input_flush();                         // a flush consumed a host '%': drop its input here (E145)
	if (cm.end_hold_requested) {
        if (cm.queue_flush_state == FLUSH_OFF) {    // either no flush or wait until it's done flushing
			cm_end_hold();
		}
	}
	return (STAT_OK);
}

/*
 * cm_start_hold() - start a feedhhold by signalling the exec
 * cm_end_hold()   - end a feedhold by returning the system to normal operation
 * cm_has_hold()   - return true if a hold condition exists (or a pending hold request)
 */

void cm_start_hold()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // The exec may free the last block before its prepared segment reaches the loader.
    // Decide and latch together; an empty queue alone does not mean the motor has stopped.
    if ((cm.hold_state == FEEDHOLD_REQUESTED) || (cm.hold_state == FEEDHOLD_SYNC)) {
        bool runnable = mp_has_runnable_buffer();
        if ((cm.hold_state == FEEDHOLD_REQUESTED) && runnable) {
            cm_set_motion_state(MOTION_HOLD);
            cm.hold_state = FEEDHOLD_SYNC;
        } else if (!runnable && mp_runtime_is_idle() &&
                   (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER)) {
            cm_set_motion_state(MOTION_HOLD);
            cm.hold_state = FEEDHOLD_HOLD;           // let pending '~' or '%' finish normally
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

void cm_end_hold()
{
	if (cm.hold_state == FEEDHOLD_HOLD) {
        cm.program_paused = false;
        cm.end_hold_requested = false;
        mp_replan_held_block();                     // the held block resumes from zero velocity
        if ((cm.machine_state != MACHINE_ALARM) && mp_has_runnable_buffer()) {
            cm_cycle_start();                       // before the exec may run a block: a block that ends the
        }                                           // cycle (a dwell emptying the queue) must find it started (E165)
	    mp_exit_hold_state();

        // State machine cases:
        if (cm.machine_state == MACHINE_ALARM) {
//            cm_spindle_off_immediate();
//            cm_coolant_off_immediate();

        } else if (cm.motion_state == MOTION_STOP) { // && (! MACHINE_ALARM)
//            cm_spindle_off_immediate();
//            cm_coolant_off_immediate();
		    cm_cycle_end();

        } else {    // (MOTION_RUN || MOTION_PLANNING)  && (! MACHINE_ALARM): cycle started above
//            cm_spindle_resume(spindle.dwell_seconds);
//            cm_coolant_resume();
            st_request_exec_move();
        }
    }
}

bool cm_has_hold()
{
    return (cm.hold_state != FEEDHOLD_OFF);
}

/*
 * cm_queue_flush() - Flush planner queue and correct model positions
 *
 * This one's complicated. See here first:
 * https://github.com/synthetos/g2/wiki/Alarm-Processing
 * https://github.com/synthetos/g2/wiki/Job-Exception-Handling
 *
 * We want to use queue flush for a few different use cases, as per the above wiki pages.
 * The % behavior implements Exception Handling cases 1 and 2 - Stop a Single Move and
 * Stop Multiple Moves. This is complicated further by the processing in single USB and
 * dual USB being different. Also, the state handling is located in xio.cpp / readline(),
 * controller.cpp _dispatch_kernel() and cm_request_queue_flush(), below.
 * So it's documented here.
 *
 * Single or Dual USB Channels:
 *  - If a % is received outside of a feed hold or ALARM state, ignore it.
 *      Change the % to a ; comment symbol (xio)
 *      This behavior supports % as file beginnings and ends, and Inkscape comments.
 *
 * Single USB Channel Operation (TinyG v8's only mode)
 *  - Enter a feedhold (!)
 *  - Receive a queue flush (%) Both dispatch it and store a marker (ACK) in the input
 *      buffer in place of the the % (xio)
 *  - Execute the feedhold to a hold condition (plan_exec)
 *  - Execute the dispatched % to flush queues (canonical_machine)
 *  - Silently reject any commands up to the % in the input queue (controller)
 *  - When ETX is encountered transition to STOP state (controller/canonical_machine)
 *
 * Dual USB Channel Operation (Available on g2 only)
 *  - Same as above except that we expect the % to arrive on the control channel
 *  - The system will read and dump all commands in the data channel until either a
 *    clear is encountered ({clear:n} or $clear), or an ETX is encountered on either
 *    channel, but it really should be on the data channel to ensure all queued commands
 *    are dumped. It is the host's responsibility to both write the clear (or ETX), and
 *    to ensure that it either arrives on the data channel or that the data channel is
 *    empty before writing it to the control channel.
 */

void cm_queue_flush()
{
    if (mp_runtime_is_idle()) {                     // can't flush planner during movement
        mp_flush_planner();

        for (uint8_t axis = AXIS_X; axis < AXES; axis++) { // set all positions
            cm_set_position(axis, mp_get_runtime_absolute_position(axis));
        }
        if(cm.hold_state == FEEDHOLD_HOLD) {        // end feedhold if we're in one
            cm_end_hold();
        }
        // Consume a host '%' together with the state change: the RX ISR sets both (E145). A '%' that
        // lands before this is handed off; after it, the flush is over and the '%' is not honored.
        // Callers include panic paths, so the input pool is reset later, by the sequencing callback.
#ifdef __AVR
        uint8_t saved_sreg = SREG;
        cli();
#endif
        _take_input_flush_request();                // nests: restores the cleared SREG
        cm.queue_flush_state = FLUSH_OFF;
#ifdef __AVR
        SREG = saved_sreg;
#endif
        qr_request_queue_report(0);                 // request a queue report, since we've changed the number of buffers available
    }
}

/******************************
 * Program Functions (4.3.10) *
 ******************************/
/* This group implements stop, start, and end functions.
 * It is extended beyond the NIST spec to handle various situations.
 *
 * _exec_program_finalize()     - helper
 * cm_cycle_start()
 * cm_cycle_end()
 * cm_program_stop()            - M0
 * cm_optional_program_stop()   - M1
 * cm_program_end()             - M2, M30
 */
/*
 * Program and cycle state functions
 *
 * cm_program_stop() and cm_optional_program_stop() are synchronous Gcode commands
 * that are received through the interpreter. They cause all motion to stop
 * at the end of the current command, including spindle motion. All Gcode state
 * is otherwise left intact.
 *
 * Note that the stop actually occurs at the end of the immediately preceding
 * block - i.e. the stop is queued behind the last executing block.
 *
 *
 * cm_program_end() is a stop that also resets the machine to initial state.
 * The END behaviors are defined by NIST 3.6.1 are:
 *	1. Axis offsets are set to zero (like G92.2) and...
 *  1a. Origin offsets are set to the default (like G54)
 *	2. Selected plane is set to CANON_PLANE_XY (like G17)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *	5. Feed and speed overrides are set to ON (like M48)
 *	6. Cutter compensation is turned off (like G40)
 *	7. The spindle is stopped (like M5)
 *	8. The current motion mode is set to G_1 (like G1)
 *	9. Coolant is turned off (like M9)
 *
 * cm_program_end() performs things slightly differently than NIST:
 *	1. Axis offsets are set to G92.1 CANCEL offsets (instead of using G92.2 SUSPEND Offsets)
 *	1a. Set default coordinate system (uses $gco, not G54)
 *	2. Selected plane is set to default plane ($gpl) (instead of setting it to G54)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 * 	5. Not implemented
 *	6. Not implemented
 *	7. The spindle is stopped (like M5)
 *	8. Motion mode is canceled like G80 (not set to G1)
 *	9. Coolant is turned off (like M9)
 *	10 Default INCHES or MM units mode is restored ($gun)
 */

static void _exec_program_finalize(float *value, bool *flags)
{
	cm.machine_state = (uint8_t)value[0];               // set to one of STOP or END
	cm_set_motion_state(MOTION_STOP);
	if (cm.cycle_state == CYCLE_MACHINING) {
		cm.cycle_state = CYCLE_OFF;						// don't end cycle if homing, probing, etc.
	}
	cm.hold_state = FEEDHOLD_OFF;						// end feedhold (if in feed hold)
	cm.hold_fast = false;
	cm.hold_fast_request = false;
    cm.end_hold_requested = false;					    // cancel any pending end hold request
	mp_zero_segment_velocity();							// for reporting purposes

	// A program END's model resets were applied when the M2/M30 was parsed (cm_program_end()).
	// This runs from the stepper loader ISR, so it must not touch the gcode model or queue commands.
	sr_request_status_report(SR_REQUEST_ASAP);		    // request a final status report (not unfiltered)
}

void cm_cycle_start()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // Program-stop callbacks run in the HI loader, so masking only LO/MED is insufficient.
    if (!cm.program_paused) {
        cm.machine_state = MACHINE_CYCLE;
        if (cm.cycle_state == CYCLE_OFF) {					// don't (re)start homing, probe or other canned cycles
            cm.cycle_state = CYCLE_MACHINING;		// keep qi/qo: buffers queued before the cycle started count (E091)
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

void cm_cycle_end()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // Program-stop callbacks run in the HI loader, so masking only LO/MED is insufficient.
    if (!cm.program_paused) {
        if (cm.cycle_state != CYCLE_OFF) {
            float value[] = { (float)MACHINE_PROGRAM_STOP };
            _exec_program_finalize(value, FLAGS_ONE);
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

static void _exec_program_stop(float *value, bool *flags)
{
    _exec_program_finalize(value, flags);
    cm.program_paused = true;
    cm.hold_state = FEEDHOLD_HOLD;
    cm_set_motion_state(MOTION_HOLD);
}

void cm_program_stop()
{
	float value[AXES] = { (float)MACHINE_PROGRAM_STOP };
	mp_queue_command(_exec_program_stop, value, FLAGS_ONE);
}

void cm_optional_program_stop()
{
    if (cm.optional_stop_enabled) { cm_program_stop(); }
}

void cm_program_end()
{
	// Reset the model now, in the main loop, so blocks parsed after the M2/M30 keep their modes, and
	// queue ONE runtime command for the rest: the per-block planner demand stays bounded (E069, E088)
	cm.gmx.origin_offset_enable = 0;				// 1.  G92.1 - we do G91.1 instead of G92.2
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm.gmx.origin_offset[axis] = 0;
	}
//	cm_suspend_origin_offsets();					// 1.  G92.2 - as per NIST
	cm.gm.coord_system = cm.default_coord_system;	// 1a. reset to default coordinate system
	cm_select_plane(cm.default_select_plane);		// 2.  reset to default arc plane
	cm_set_distance_mode(cm.default_distance_mode); // 3.  reset to default distance mode
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);	// 4.  G94
	cm.gm.spindle_mode = SPINDLE_OFF;				// 7.  M5 (model; the output goes off at runtime)
//	cm_set_motion_mode(MOTION_MODE_STRAIGHT_FEED);	// 8.  NIST specifies G1...
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL);  // 8.  but we cancel motion mode. Safer.
	cm.gm.mist_coolant = false;						// 9.  M9 (model)
	cm.gm.flood_coolant = false;
//	cm_set_units_mode(cm.units_mode);				// 10. reset to default units mode +++ REMOVED +++

	float value[AXES] = { (float)MACHINE_PROGRAM_END, (float)cm.default_coord_system };	// mp_queue_command reads AXES values
	mp_queue_command(_exec_program_end, value, FLAGS_ONE);
}

/*
 * _exec_program_end() - runtime half of M2/M30, in queue order behind the moves before it
 *
 *	One queued command (it must not queue more): apply the reset work offsets, stop the spindle and
 *	coolant outputs directly, then finalize the program end.
 */
static void _exec_program_end(float *value, bool *flags)
{
	uint8_t coord_system = (uint8_t)value[1];
	float offsets[AXES];
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		offsets[axis] = cm.offset[coord_system][axis];	// origin offsets were reset with the model
	}
	mp_set_runtime_work_offset(offsets);
	spindle_reset();								// M5 now (immediate; S is kept)
	cm_coolant_off_immediate();						// M9 now
	_exec_program_finalize(value, flags);			// value[0] is MACHINE_PROGRAM_END
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
static const char msg_stat11[] PROGMEM = "Interlock";
static const char msg_stat12[] PROGMEM = "Shutdown";
static const char msg_stat13[] PROGMEM = "Panic";
static const char *const msg_stat[] PROGMEM = { msg_stat0, msg_stat1, msg_stat2, msg_stat3,
                                                msg_stat4, msg_stat5, msg_stat6, msg_stat7,
                                                msg_stat8, msg_stat9, msg_stat10, msg_stat11,
                                                msg_stat12, msg_stat13 };

static const char msg_macs0[] PROGMEM = "Initializing";
static const char msg_macs1[] PROGMEM = "Ready";
static const char msg_macs2[] PROGMEM = "Alarm";
static const char msg_macs3[] PROGMEM = "Stop";
static const char msg_macs4[] PROGMEM = "End";
static const char msg_macs5[] PROGMEM = "Cycle";
static const char msg_macs6[] PROGMEM = "Interlock";
static const char msg_macs7[] PROGMEM = "SHUTDOWN";
static const char msg_macs8[] PROGMEM = "PANIC";
static const char *const msg_macs[] PROGMEM = { msg_macs0, msg_macs1, msg_macs2, msg_macs3,
                                                msg_macs4, msg_macs5, msg_macs6, msg_macs7,
                                                msg_macs8 };

static const char msg_cycs0[] PROGMEM = "Off";
static const char msg_cycs1[] PROGMEM = "Machining";
static const char msg_cycs2[] PROGMEM = "Homing";
static const char msg_cycs3[] PROGMEM = "Probe";
static const char msg_cycs4[] PROGMEM = "Jog";
static const char *const msg_cycs[] PROGMEM = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3,  msg_cycs4 };

static const char msg_mots0[] PROGMEM = "Stop";
static const char msg_mots1[] PROGMEM = "Run";
static const char msg_mots2[] PROGMEM = "Hold";
static const char *const msg_mots[] PROGMEM = { msg_mots0, msg_mots1, msg_mots2 };

static const char msg_hold0[] PROGMEM = "Off";
static const char msg_hold1[] PROGMEM = "Requested";
static const char msg_hold2[] PROGMEM = "Sync";
static const char msg_hold3[] PROGMEM = "Decel-Continue";
static const char msg_hold4[] PROGMEM = "Decel-To-Zero";
static const char msg_hold5[] PROGMEM = "Pending";
static const char msg_hold6[] PROGMEM = "Hold";
static const char *const msg_hold[] PROGMEM = { msg_hold0, msg_hold1, msg_hold2, msg_hold3,
												msg_hold4, msg_hold5, msg_hold6 };

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

static const char msg_g00[] PROGMEM = "G0  - linear traverse";
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

static const char msg_g901[] PROGMEM = "G90.1 - absolute distance mode";
static const char msg_g911[] PROGMEM = "G91.1 - incremental distance mode (default mode)";
static const char *const msg_admo[] PROGMEM = { msg_g901, msg_g911 };

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
#define msg_admo NULL
#define msg_frmo NULL
#define msg_am NULL

#endif // __TEXT_MODE

/***** AXIS HELPERS *****************************************************************
 * cm_get_axis_char() - return ASCII char for axis given the axis number
 * _get_axis()		  - return axis number or -1 if NA
 * _get_axis_type()	  - return 0 -f axis is linear, 1 if rotary, -1 if NA
 */

char cm_get_axis_char(const int8_t axis)
{
	char axis_char[] = "XYZABC";
	if ((axis < 0) || (axis > AXES)) return (' ');
	return (axis_char[axis]);
}

static int8_t _get_axis(const index_t index)
{
	char *ptr;
	char tmp[TOKEN_LEN+1];
	char axes[] = {"xyzabc"};

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
	nv->value_int = value;
	nv->valuetype = TYPE_INTEGER;
	return(nv_copy_string(nv, (const char *)GET_TEXT_ITEM(msg_array, value)));
}

stat_t cm_get_stat(nvObj_t *nv) { return(_get_msg_helper(nv, msg_stat, cm_get_combined_state()));}
stat_t cm_get_macs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_macs, cm_get_machine_state()));}
stat_t cm_get_cycs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_cycs, cm_get_cycle_state()));}
stat_t cm_get_mots(nvObj_t *nv) { return(_get_msg_helper(nv, msg_mots, cm_get_motion_state()));}
stat_t cm_get_hold(nvObj_t *nv) { return(_get_msg_helper(nv, msg_hold, cm_get_hold_state()));}
stat_t cm_get_home(nvObj_t *nv) { return(_get_msg_helper(nv, msg_home, cm_get_homing_state()));}

stat_t cm_get_unit(nvObj_t *nv) { return(_get_msg_helper(nv, msg_unit, _report_units()));}
stat_t cm_get_coor(nvObj_t *nv) { return(_get_msg_helper(nv, msg_coor, cm_get_coord_system(ACTIVE_MODEL)));}
stat_t cm_get_momo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_momo, cm_get_motion_mode(ACTIVE_MODEL)));}
stat_t cm_get_plan(nvObj_t *nv) { return(_get_msg_helper(nv, msg_plan, cm_get_select_plane(ACTIVE_MODEL)));}
stat_t cm_get_path(nvObj_t *nv) { return(_get_msg_helper(nv, msg_path, cm_get_path_control(ACTIVE_MODEL)));}
stat_t cm_get_dist(nvObj_t *nv) { return(_get_msg_helper(nv, msg_dist, cm_get_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_admo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_admo, cm_get_arc_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_frmo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_frmo, cm_get_feed_rate_mode(ACTIVE_MODEL)));}

stat_t cm_get_toolv(nvObj_t *nv)
{
	nv->value_int = cm_get_tool(ACTIVE_MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_mline(nvObj_t *nv)
{
	nv->value_int = cm_get_linenum(MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_line(nvObj_t *nv)
{
	nv->value_int = cm_get_linenum(ACTIVE_MODEL);
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t cm_get_vel(nvObj_t *nv)
{
	if (cm_get_motion_state() == MOTION_STOP) {
		nv->value_flt = 0;
	} else {
		nv->value_flt = mp_get_runtime_velocity();
		if (cm_get_units_mode(RUNTIME) == INCHES) {
            nv->value_flt *= INCHES_PER_MM;
        }
	}
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_feed(nvObj_t *nv)
{
	nv->value_flt = cm_get_feed_rate(ACTIVE_MODEL);
	if (cm_get_units_mode(ACTIVE_MODEL) == INCHES) {
        nv->value_flt *= INCHES_PER_MM;
    }
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

/*
 * _position_model() - the model a position report reads (E090)
 *
 *	Motion that has been accepted but has not started yet (a cycle running with motion stopped:
 *	a move or arc queued but not executing, a refused arc segment, a paused program) has already
 *	moved the model to its end point. The runtime still holds where the machine is.
 */
static GCodeState_t *_position_model()
{
	if ((cm.motion_state == MOTION_STOP) && (cm.cycle_state != CYCLE_OFF)) {
		return (RUNTIME);
	}
	return (ACTIVE_MODEL);
}

/*
 * cm_position_snapshot_begin() / cm_position_snapshot_end() - one instant for a whole report (E166)
 *
 *	A report reads its positions one axis at a time. Between two reads, and during them, interrupts
 *	change what it reads: the exec moves the runtime position and loads the next block's offsets and
 *	units, the step loader runs queued offset changes and the program end, and a motion state change
 *	switches the model a report reads. begin() copies, at one instant, the model's absolute and work
 *	positions and offsets and the units the report is shown in; the getters return that copy until
 *	end(). Calls nest: only the outermost pair copies, so a single read copies for itself unless a
 *	report already has a copy.
 *
 *	The report is not copied with interrupts off as a whole. Each interrupt path that changes reported
 *	state moves cm_position_epoch on (cm_set_motion_state(), the exec, the loader's queued commands),
 *	and the copy is taken again if the epoch moved while it was being taken. (Each runtime read in
 *	the copy still turns interrupts off for its two floats; see plan_line.c.) Those interrupts come at
 *	most once per segment, so a second pass is rare and a third rarer still.
 */
volatile uint8_t cm_position_epoch;

static struct {
	uint8_t depth;
	uint8_t units;							// the units the report is shown in (the active model's)
	float position[AXES];					// absolute position of the model the report reads (mm)
	float work_position[AXES];				// its work position (mm)
	float work_offset[AXES];				// its work offsets
} pos_snap;

static void _copy_positions()
{
	const GCodeState_t *gcode_state = _position_model();
	pos_snap.units = ACTIVE_MODEL->units_mode;
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		pos_snap.position[axis] = cm_get_absolute_position(gcode_state, axis);
		pos_snap.work_position[axis] = _get_work_position_mm(gcode_state, axis);
		pos_snap.work_offset[axis] = cm_get_work_offset(gcode_state, axis);
	}
}

void cm_position_snapshot_begin()
{
	if (pos_snap.depth++ != 0) {
		return;
	}
	uint8_t epoch;
	do {
		epoch = cm_position_epoch;
		__asm__ __volatile__ ("" ::: "memory");	// copy after reading the epoch...
		_copy_positions();
		__asm__ __volatile__ ("" ::: "memory");	// ...and before checking it again
	} while (epoch != cm_position_epoch);
}

void cm_position_snapshot_end()
{
	if (pos_snap.depth != 0) {
		pos_snap.depth--;
	}
}

static uint8_t _report_units()
{
	return ((pos_snap.depth != 0) ? pos_snap.units : cm_get_units_mode(ACTIVE_MODEL));
}

/* the work position in the units the unit report shows; call inside a snapshot */
static float _get_report_position(const uint8_t axis)
{
	float position = pos_snap.work_position[axis];
	if ((axis <= AXIS_Z) && (pos_snap.units == INCHES)) {
		position /= MM_PER_INCH;
	}
	return (position);
}

stat_t cm_get_pos(nvObj_t *nv)
{
	cm_position_snapshot_begin();
	nv->value_flt = _get_report_position(_get_axis(nv->index));
	cm_position_snapshot_end();
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_mpo(nvObj_t *nv)
{
	cm_position_snapshot_begin();
	nv->value_flt = pos_snap.position[_get_axis(nv->index)];
	cm_position_snapshot_end();
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_ofs(nvObj_t *nv)
{
	cm_position_snapshot_begin();
	nv->value_flt = pos_snap.work_offset[_get_axis(nv->index)];
	cm_position_snapshot_end();
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
	return(_get_msg_helper(nv, msg_am, nv->value_int));
}

stat_t cm_set_am(nvObj_t *nv)		// axis mode
{
	if (_get_axis_type(nv->index) == 0) {	// linear
		if (nv->value_int > AXIS_MODE_MAX_LINEAR) {
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
	} else {
		if (nv->value_int > AXIS_MODE_MAX_ROTARY) {
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
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
float cm_get_axis_jerk(const uint8_t axis)
{
	return (cm.a[axis].jerk_max);
}

void cm_set_axis_jerk(const uint8_t axis, const float jerk)
{
	cm.a[axis].jerk_max = jerk;
	cm.a[axis].recip_jerk = 1/(jerk * JERK_MULTIPLIER);
}

/* M201.3 uses units/second^3; stored jerk uses million units/minute^3. */
stat_t cm_set_jerk(const float target[], const bool flags[])
{
    float jerk[AXES];
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (!flags[axis]) { continue; }
        float scale = (60.f * 60.f * 60.f) / JERK_MULTIPLIER;
        if (axis <= AXIS_Z && cm.gm.units_mode == INCHES) { scale *= MM_PER_INCH; }
        jerk[axis] = target[axis] * scale;
        if (!isfinite(jerk[axis])) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
        jerk[axis] = max(1, jerk[axis]);
        if (!isfinite(jerk[axis] * JERK_MULTIPLIER)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
    }
    // Validate the whole block before changing any axis; do not persist the values.
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flags[axis]) { cm_set_axis_jerk(axis, jerk[axis]); }
    }
    return (STAT_OK);
}

/* Axis rate limits must remain positive after conversion to canonical units. */
stat_t cm_set_axis_rate(nvObj_t *nv)
{
    if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (_get_axis_type(nv->index) == 0) {
        if ((cm_get_units_mode(MODEL) == INCHES) &&
            !isfinite(nv->value_flt * MM_PER_INCH)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        return (set_flu(nv));
    }
    return (set_flt(nv));
}

stat_t cm_set_xjm(nvObj_t *nv)
{
	if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {	// a zero jerk breaks the planner (E158)
		return (STAT_INPUT_LESS_THAN_MIN_VALUE);
	}
	if (nv->value_flt > JERK_MULTIPLIER) {
        nv->value_flt /= JERK_MULTIPLIER;
    }
	if (_get_axis_type(nv->index) == 0) {		// linear: inches to mm; rotary jerk is in degrees (E110)
		set_flu(nv);
	} else {
		set_flt(nv);
	}
	cm_set_axis_jerk(_get_axis(nv->index), nv->value_flt);
	return(STAT_OK);
}

stat_t cm_set_xjh(nvObj_t *nv)
{
	if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {	// a zero jerk breaks the planner (E158)
		return (STAT_INPUT_LESS_THAN_MIN_VALUE);
	}
	if (nv->value_flt > JERK_MULTIPLIER) {
        nv->value_flt /= JERK_MULTIPLIER;
    }
	if (_get_axis_type(nv->index) == 0) {		// linear: inches to mm; rotary jerk is in degrees (E110)
		set_flu(nv);
	} else {
		set_flt(nv);
	}
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
	if (nv->value_int == true) {
        cm_homing_cycle_start();
    }
	return (STAT_OK);
}

/*
 * Debugging Commands
 *
 * cm_dam() - dump active model
 */

stat_t cm_dam(nvObj_t *nv)
{
	printf_P(PSTR("Active model:\n"));
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
	cm_print_admo(nv);
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

/*
 * _jog_allowed() - a jog starts only from idle or from a completed feedhold (E157)
 *
 *	While a program runs, the jogging cycle would wait for the current move and then flush the queued
 *	program; during homing, probing or another jog it would overwrite the cycle and its saved modes.
 *	Jogging out of a feedhold discards the held queue, as before.
 */
static stat_t _jog_allowed(void)
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	// Snapshot together: the exec can free the last block and end the cycle before its final
	// segment is delivered, so stopped and empty alone do not mean the motors are idle (as cm_clear).
	bool hold_done = (cm.hold_state == FEEDHOLD_HOLD) &&
	                 ((cm.cycle_state == CYCLE_MACHINING) || (cm.cycle_state == CYCLE_OFF));
	bool idle = (cm.cycle_state == CYCLE_OFF) && (cm.hold_state == FEEDHOLD_OFF) &&
	            (cm.motion_state == MOTION_STOP) && !mp_has_runnable_buffer() && mp_runtime_is_idle() &&
	            (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER);
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return ((hold_done || idle) ? STAT_OK : STAT_COMMAND_NOT_ACCEPTED);
}

stat_t cm_run_jogx(nvObj_t *nv)
{
	ritorno(_jog_allowed());
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_X);
	return (STAT_OK);
}

stat_t cm_run_jogy(nvObj_t *nv)
{
	ritorno(_jog_allowed());
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_Y);
	return (STAT_OK);
}

stat_t cm_run_jogz(nvObj_t *nv)
{
	ritorno(_jog_allowed());
	set_flt(nv);
	cm_jogging_cycle_start(AXIS_Z);
	return (STAT_OK);
}

stat_t cm_run_joga(nvObj_t *nv)
{
	ritorno(_jog_allowed());
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
const char fmt_line[] PROGMEM = "Line number:%10lu\n";
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
const char fmt_admo[] PROGMEM = "Arc Distance mode:   %s\n";
const char fmt_frmo[] PROGMEM = "Feed rate mode:      %s\n";
const char fmt_tool[] PROGMEM = "Tool number          %d\n";

const char fmt_pos[] PROGMEM = "%c position:%15.3f%s\n";
const char fmt_mpo[] PROGMEM = "%c machine posn:%11.3f%s\n";
const char fmt_ofs[] PROGMEM = "%c work offset:%12.3f%s\n";
const char fmt_hom[] PROGMEM = "%c axis homed:%9d\n";

const char fmt_gpl[] PROGMEM = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
const char fmt_gun[] PROGMEM = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
const char fmt_gco[] PROGMEM = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
const char fmt_gpa[] PROGMEM = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
const char fmt_gdi[] PROGMEM = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

void cm_print_vel(nvObj_t *nv) { text_print_flt_units(nv, fmt_vel, GET_UNITS(ACTIVE_MODEL));}
void cm_print_feed(nvObj_t *nv) { text_print_flt_units(nv, fmt_feed, GET_UNITS(ACTIVE_MODEL));}
void cm_print_line(nvObj_t *nv) { text_print(nv, fmt_line);}
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
void cm_print_admo(nvObj_t *nv) { text_print_str(nv, fmt_admo);}
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

void cm_print_ja(nvObj_t *nv) { text_print_flt_units(nv, fmt_ja, GET_UNITS(ACTIVE_MODEL));}
void cm_print_ct(nvObj_t *nv) { text_print_flt_units(nv, fmt_ct, GET_UNITS(ACTIVE_MODEL));}
void cm_print_sl(nvObj_t *nv) { text_print(nv, fmt_sl);}

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
static const char fmt_Xsn[] PROGMEM = "[%s%s] %s switch min%17d [0=off,1=homing,2=limit,3=limit+homing,4=probe]\n";
static const char fmt_Xsx[] PROGMEM = "[%s%s] %s switch max%17d [0=off,1=homing,2=limit,3=limit+homing,4=probe]\n";
static const char fmt_Xsv[] PROGMEM = "[%s%s] %s search velocity%12.0f%s/min\n";
static const char fmt_Xlv[] PROGMEM = "[%s%s] %s latch velocity%13.0f%s/min\n";
static const char fmt_Xlb[] PROGMEM = "[%s%s] %s latch backoff%18.3f%s\n";
static const char fmt_Xzb[] PROGMEM = "[%s%s] %s zero backoff%19.3f%s\n";
static const char fmt_cofs[] PROGMEM = "[%s%s] %s %s offset%20.3f%s\n";
static const char fmt_cpos[] PROGMEM = "[%s%s] %s %s position%18.3f%s\n";

static void _print_axis_ui8(nvObj_t *nv, const char *format)
{
    char msg[NV_MESSAGE_LEN];
	sprintf_P(msg, format, nv->group, nv->token, nv->group, nv->value_int);
    text_finalize_message(msg);

}

static void _print_axis_flt(nvObj_t *nv, const char *format)
{
	char *units;
	if (_get_axis_type(nv->index) == 0) {	// linear
		units = (char *)GET_UNITS(MODEL);
	} else {
		units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
	}
    char msg[NV_MESSAGE_LEN];
    sprintf_P(msg, format, nv->group, nv->token, nv->group, nv->value_flt, units);
    text_finalize_message(msg);
}

static void _print_axis_coord_flt(nvObj_t *nv, const char *format)
{
	char *units;
	if (_get_axis_type(nv->index) == 0) {	// linear
		units = (char *)GET_UNITS(MODEL);
	} else {
		units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
	}
    char msg[NV_MESSAGE_LEN];
    sprintf_P(msg, format, nv->group, nv->token, nv->group, nv->token, nv->value_flt, units);
    text_finalize_message(msg);
}

static void _print_pos(nvObj_t *nv, const char *format, uint8_t units)
{
	char axes[] = {"XYZABC"};
	uint8_t axis = _get_axis(nv->index);
	if (axis >= AXIS_A) { units = DEGREES;}
    char msg[NV_MESSAGE_LEN];
    sprintf_P(msg, format, axes[axis], nv->value_flt, GET_TEXT_ITEM(msg_units, units));
    text_finalize_message(msg);
}

void cm_print_am(nvObj_t *nv)	// print axis mode with enumeration string
{
    char msg[NV_MESSAGE_LEN];
    sprintf_P(msg, fmt_Xam, nv->group, nv->token, nv->group, (uint8_t)nv->value_int,
              GET_TEXT_ITEM(msg_am, (uint8_t)nv->value_int));
    text_finalize_message(msg);
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

void cm_print_hom(nvObj_t *nv)	// homed flag: an integer, not a position (get_ui8 fills value_int)
{
	char axes[] = {"XYZABC"};
    char msg[NV_MESSAGE_LEN];
    sprintf_P(msg, fmt_hom, axes[_get_axis(nv->index)], (int)nv->value_int);
    text_finalize_message(msg);
}
void cm_print_mpo(nvObj_t *nv) { _print_pos(nv, fmt_mpo, MILLIMETERS);}
void cm_print_ofs(nvObj_t *nv) { _print_pos(nv, fmt_ofs, MILLIMETERS);}

#endif // __TEXT_MODE
