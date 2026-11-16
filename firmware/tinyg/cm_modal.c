/*
 * cm_modal.c - canonical model modes, tools, overrides and jerk.
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
#include "planner.h"
#include "spindle.h"
#include "override.h"
#include "util.h"

static void _exec_change_tool(float *value, bool *flags);

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


void cm_set_model_linenum(const uint32_t linenum)
{
	cm.gm.linenum = linenum;				// you must first set the model line number,
	nv_add_object((const char *)"n");	    // then add the line number to the nv list
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

stat_t cm_cancel_motion_mode()
{
	cm.gm.motion_mode = MOTION_MODE_CANCEL;    // modal only: queued moves keep their cycle
    return (STAT_OK);
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
        if (inverse_time > MAX_INVERSE_TIME_MINUTES) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
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
    (void)tool_change;
	float value[AXES] = { (float)cm.gm.tool_select };
	mp_queue_command(_exec_change_tool, value, FLAGS_ONE);
	return (STAT_OK);
}

static void _exec_change_tool(float *value, bool *flags)
{
    (void)flags;
    cm.gm.tool = (uint8_t)value[0];
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

/* Executed in queue order, never at parse time ahead of earlier motion. */
static void _exec_feed_permission(float *value, bool *flags)
{
    (void)flags;
    if (value[1] != 2) ov_set_program_feed(value[0] != 0);
    if (value[1] != 1) spindle_override_program(value[0] != 0);
}
stat_t cm_override_enables(uint8_t flag)
{
    float value[AXES] = {0};
    value[0] = flag;
    mp_queue_command(_exec_feed_permission, value, FLAGS_ONE);
    ov_queue_program_feed(flag);
    return STAT_OK;
}
stat_t cm_feed_rate_override_enable(uint8_t flag)
{
    (void)flag;
    float value[AXES] = { !cm.gf.parameter || cm.gn.parameter != 0, 1 };
    mp_queue_command(_exec_feed_permission, value, FLAGS_ONE);
    ov_queue_program_feed(value[0] != 0);
    return STAT_OK;
}

stat_t cm_feed_rate_override_factor(uint8_t flag)	// M50.1
{
	cm.gmx.feed_rate_override_enable = flag;
	cm.gmx.feed_rate_override_factor = cm.gn.parameter;
	return (STAT_OK);
}

stat_t cm_traverse_override_enable(uint8_t flag)	// M50.2
{
    (void)flag;
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

stat_t cm_spindle_override_enable(uint8_t flag)
{
    (void)flag;
    float value[AXES] = { !cm.gf.parameter || cm.gn.parameter != 0, 2 };
    mp_queue_command(_exec_feed_permission, value, FLAGS_ONE);
    return STAT_OK;
}

stat_t cm_spindle_override_factor(uint8_t flag)		// M50.1
{
	cm.gmx.spindle_override_enable = flag;
	cm.gmx.spindle_override_factor = cm.gn.parameter;
//	STUBBED: change spindle speed
	return (STAT_OK);
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
