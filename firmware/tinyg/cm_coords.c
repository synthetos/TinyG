/*
 * cm_coords.c - canonical coordinates, offsets and position snapshots.
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
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "cm_internal.h"
#include "controller.h"
#include "text_parser.h"
#include "planner.h"
#include "stepper.h"
#include "hardware.h"
#include "util.h"
#include "xio.h"

static void _exec_offset(float *value, bool *flags);
static void _exec_g10_offset(float *value, bool *flags);
static void _exec_absolute_origin(float *value, bool *flags);
static float _get_report_position(const uint8_t axis);

void cm_set_absolute_override(const uint8_t absolute_override)
{
	cm.gm.absolute_override = absolute_override;
	cm_set_work_offsets();				    // must reset offsets if you change absolute override
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
 *		Queued snapshots must be restored into a full model before these accessors are used.
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
 * Each write loses the bytes that arrive during it, about 30 ms a value (N83). A save left here by a host that kept
 * sending waits until no byte has been received for G10_QUIET_MS. A host polling faster than that would put the save
 * off for good and the offset would go with the power, so after G10_SAVE_WAIT_MS at rest it starts just after a byte,
 * where the gap is longest and a stream of lines leaves none. A reply to such a G10 does not mean it is saved.
 */

/*
 * cm_persistence_busy() - an EEPROM write must wait (N68)
 *
 *	Each persisted value masks the medium and low interrupt levels while it is written (AVR1008 workaround), so
 *	serial input and segment preparation stall. A lone G4 dwell starts no cycle but keeps the runtime busy.
 *	Queued commands that have not started do not count: G10 queues its own offset update.
 */
bool cm_persistence_busy()
{
	return ((cm.cycle_state != CYCLE_OFF) || !mp_runtime_is_idle());
}

#ifdef __AVR
static uint32_t last_rx;					// when a byte was last seen arriving
#endif

/*
 * _host_awaits_reply() - is the sender of this G10 waiting for its reply? (N83)
 *
 *	Such a host is silent until the reply, the one time a write cannot lose its input (N01). A host streaming ahead of
 *	the replies has its next line in the RX ring or on the wire, and an empty ring cannot tell them apart: listen for
 *	G10_HOST_WAIT_MS. This blocks, but only at rest, and about as long as the write that follows. SysTick moves in
 *	10 ms steps, so the time is up to one step short.
 */
static bool _host_awaits_reply()
{
#ifdef __AVR
	uint32_t start = SysTickTimer_getValue();
	(void)xio_usb_take_rx_seen();					// the G10's own bytes
	while ((SysTickTimer_getValue() - start) < G10_HOST_WAIT_MS) {
		if (xio_get_rx_bufcount_usart(&USBu) != 0) { break; }
		if (xio_usb_take_rx_seen()) {
			last_rx = SysTickTimer_getValue();
			return (false);
		}
	}
	if (xio_get_rx_bufcount_usart(&USBu) != 0) {
		last_rx = SysTickTimer_getValue();
		return (false);
	}
#endif
	return (true);
}

static void _persist_coord_offsets()
{
	bool saved = true;
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
			if (nv_persist(&nv) != STAT_OK) {	// Note: only writes values that have changed
				saved = false;
			}
		}
	}
	cm.deferred_write_flag = !saved;		// pending until every offset is saved
}

stat_t cm_deferred_write_callback()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;				// the rest and the latch as one: exec_ran may only clear at rest
    cli();
#endif
    bool ready = cm.deferred_write_flag && !cm_persistence_busy() && !st_aline_in_flight();
#ifdef __AVR
    bool ran = mb.exec_ran;
    if (ready) { mb.exec_ran = false; }		// not before: a dwell sets it once, as it starts, and may end unseen
    SREG = saved_sreg;
    static uint32_t ready_since;
    uint32_t now = SysTickTimer_getValue();
    if (xio_usb_take_rx_seen()) { last_rx = now; }
    if (!ready || ran) { ready_since = now; }	// a block ran, seen or not: the rest starts again
    if (ready) {
        uint32_t gap = now - last_rx;
        bool polled = ((now - ready_since) >= G10_SAVE_WAIT_MS) && (gap >= G10_QUIET_MIN_MS) &&
                      (gap < (G10_QUIET_MIN_MS + G10_QUIET_SLOT_MS));
        if (xio_isbusy() || ((gap < G10_QUIET_MS) && !polled)) { return (STAT_OK); }
    }
#endif
    if (ready) { _persist_coord_offsets(); }
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
    if (!(cm.a[axis].radius > 0) || !isfinite(cm.a[axis].radius)) { return NAN; }
	return(_to_millimeters(target[axis]) * 360 / (2 * M_PI * cm.a[axis].radius));
}

stat_t cm_set_model_target(const float target[], const bool flags[])
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
        if (!fp_bounded_at(&cm.gm.target[axis], MAX_GEOMETRY_COORDINATE)) { goto invalid_target; }
	}
	// FYI: The ABC loop below relies on the XYZ loop having been run first
	for (axis=AXIS_A; axis<=AXIS_C; axis++) {
		if (!flags[axis] || cm.a[axis].axis_mode == AXIS_DISABLED) {
			continue;		// skip axis if not flagged for update or its disabled
		} else {
			tmp = _calc_ABC(axis, target);
            if (!isfinite(tmp)) {
                copy_vector(cm.gm.target, cm.gmx.position);
                return (STAT_INPUT_VALUE_RANGE_ERROR);
            }
		}
		if (cm.gm.distance_mode == ABSOLUTE_MODE) {
			cm.gm.target[axis] = tmp + cm_get_active_coord_offset(axis); // sacidu93's fix to Issue #22
		} else {
			cm.gm.target[axis] += tmp;
		}
        if (!fp_bounded_at(&cm.gm.target[axis], MAX_GEOMETRY_COORDINATE)) { goto invalid_target; }
	}
    return (STAT_OK);
invalid_target:
    copy_vector(cm.gm.target, cm.gmx.position);
    return (STAT_INPUT_VALUE_RANGE_ERROR);
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

stat_t cm_set_coord_value(nvObj_t *nv)
{
    float value = nv->value_flt;
    if (cfg_has_flag(nv->index, F_CONVERT) && cm_get_units_mode(MODEL) == INCHES) {
        value *= MM_PER_INCH;
    }
    if (!isfinite(value) || fabs(value) > MAX_GEOMETRY_COORDINATE) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    nv->value_flt = value;
    return (set_flt(nv));
}

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
    float values[AXES];
    bool changed = false;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            float value = (axis < AXIS_A) ? _to_millimeters(offset[axis]) : offset[axis];
            if (L_word == 2) {
                values[axis] = value;
            } else {
                // Isolate signed tool compensation using the same convention as the DRO.
                float tool = cm.gm.work_offset[axis] - cm_get_active_coord_offset(axis);
                float origin = cm.gmx.origin_offset_enable ? cm.gmx.origin_offset[axis] : 0;
                values[axis] = cm.gmx.position[axis] - value - origin - tool;
            }
            if (!isfinite(values[axis]) || fabs(values[axis]) > MAX_GEOMETRY_COORDINATE) {
                return (STAT_INPUT_VALUE_RANGE_ERROR);
            }
        }
    }
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            cm.offset[selected][axis] = values[axis];
            changed = true;
            cm.deferred_write_flag = true; // persist once the machining cycle is over
        }
    }
    if (changed && !cm_persistence_busy() && !st_aline_in_flight() && _host_awaits_reply()) {	// not during a dwell either (N68)
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
    (void)flags;
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
    (void)flags;
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

	cm_run_short_move();								// before the rebase below, which would discard it (N87)
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

stat_t cm_set_g28_position(void)
{
	copy_vector(cm.gmx.g28_position, cm.gmx.position);
	return (STAT_OK);
}

stat_t cm_set_g30_position(void)
{
	copy_vector(cm.gmx.g30_position, cm.gmx.position);
	return (STAT_OK);
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

uint8_t cm_report_units()
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
	nv->value_flt = _get_report_position(cm_cfg_axis_index(nv->index));
	cm_position_snapshot_end();
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_mpo(nvObj_t *nv)
{
	cm_position_snapshot_begin();
	nv->value_flt = pos_snap.position[cm_cfg_axis_index(nv->index)];
	cm_position_snapshot_end();
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t cm_get_ofs(nvObj_t *nv)
{
	cm_position_snapshot_begin();
	nv->value_flt = pos_snap.work_offset[cm_cfg_axis_index(nv->index)];
	cm_position_snapshot_end();
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}
