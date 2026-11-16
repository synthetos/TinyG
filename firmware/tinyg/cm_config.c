/*
 * cm_config.c - canonical-machine configuration adapters.
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
#include "planner.h"
#include "stepper.h"
#include "hardware.h"
#include "util.h"

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
    (void)nv;
    cm_alarm_P(STAT_ALARM, PSTR("sent by host"));
    return (STAT_OK);
}

stat_t cm_shutd(nvObj_t *nv)              // invoke shutdown from command
{
    (void)nv;
    char msg_buf[sizeof("sent by host")]; 
    strcpy_P(msg_buf, PSTR("sent by host"));
    cm_shutdown(STAT_SHUTDOWN, msg_buf);
    return (STAT_OK);
}

stat_t cm_pnic(nvObj_t *nv)               // invoke panic from command
{
    (void)nv;
    cm_panic_P(STAT_PANIC, PSTR("sent by host"));
    return (STAT_OK);
}

stat_t cm_clr(nvObj_t *nv)                // clear alarm or shutdown from command line
{
    (void)nv;
    cm_clear();
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
 * cm_message() - queue a RAM string as a message in the response (unconditionally)
 *
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

void cm_message(char *message)
{
	nv_add_string((const char *)"msg", message);	// add message to the response object
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

// Strings for writing settings as nvObj string values
// Ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=120881&start=0

#ifdef __TEXT_MODE

static const char msg_am00[] PROGMEM = "[disabled]";
static const char msg_am01[] PROGMEM = "[standard]";
static const char msg_am02[] PROGMEM = "[inhibited]";
static const char msg_am03[] PROGMEM = "[radius]";
const char *const msg_am[] PROGMEM = { msg_am00, msg_am01, msg_am02, msg_am03};

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
 * cm_cfg_axis_index()		  - return axis number or -1 if NA
 * cm_cfg_axis_type()	  - return 0 -f axis is linear, 1 if rotary, -1 if NA
 */

char cm_get_axis_char(const int8_t axis)
{
	char axis_char[] = "XYZABC";
	if ((axis < 0) || (axis > AXES)) return (' ');
	return (axis_char[axis]);
}

int8_t cm_cfg_axis_index(const index_t index)
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

int8_t cm_cfg_axis_type(const index_t index)
{
	int8_t axis = cm_cfg_axis_index(index);
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

static stat_t _get_msg_helper(nvObj_t *nv, const char *const msg_array[], uint8_t value)
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

stat_t cm_get_unit(nvObj_t *nv) { return(_get_msg_helper(nv, msg_unit, cm_report_units()));}
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
	if (cm_cfg_axis_type(nv->index) == 0) {	// linear
		if (nv->value_int > AXIS_MODE_MAX_LINEAR) {
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
	} else {
		if (nv->value_int > AXIS_MODE_MAX_ROTARY) {
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
	}
	set_ui8(nv);
    st_refresh_move_time();
	return(STAT_OK);
}

/* Axis rate limits must remain positive after conversion to canonical units. */
stat_t cm_set_axis_rate(nvObj_t *nv)
{
    if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    float canonical = nv->value_flt;
    if ((cm_cfg_axis_type(nv->index) == 0) && (cm_get_units_mode(MODEL) == INCHES)) {
        canonical *= MM_PER_INCH;
    }
    // A rate below the numeric zero tolerance cannot describe a usable feed.
    if (canonical < 0.00001f) { return (STAT_INPUT_LESS_THAN_MIN_VALUE); }
    if (cm_cfg_axis_type(nv->index) == 0) {
        if ((cm_get_units_mode(MODEL) == INCHES) &&
            !isfinite(nv->value_flt * MM_PER_INCH)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        return (set_flu(nv));
    }
    return (set_flt(nv));
}

stat_t cm_set_ja(nvObj_t *nv)
{
    if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    float value = _to_millimeters(nv->value_flt);
    if (!(value > 0) || !isfinite(value)) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    return (set_flu(nv));
}

// Radius is a divisor when linear travel is converted to rotary degrees.
stat_t cm_set_radius(nvObj_t *nv)
{
    if (!(nv->value_flt > 0) || !isfinite(nv->value_flt)) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    float value = nv->value_flt;
    if (cm_get_units_mode(MODEL) == INCHES) { value *= MM_PER_INCH; }
    if (!isfinite(value) || !(value > 0) || value > MAX_ROTARY_RADIUS) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    float degrees_per_unit = 360.0f / (2.0f * (float)M_PI * value);
    if (!isfinite(degrees_per_unit) || !(degrees_per_unit > 0)) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    return (set_flu(nv));
}

// Chordal tolerance is a positive, finite distance in canonical millimeters.
stat_t cm_set_ct(nvObj_t *nv)
{
    float value = nv->value_flt;
    if (cm_get_units_mode(MODEL) == INCHES) { value *= MM_PER_INCH; }
    if (!(value > 0) || !isfinite(value)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
    return (set_flu(nv));
}

stat_t cm_set_jd(nvObj_t *nv)
{
    float value = nv->value_flt;
    if ((cm_cfg_axis_type(nv->index) == 0) && (cm_get_units_mode(MODEL) == INCHES)) {
        value *= MM_PER_INCH;
    }
    if (!isfinite(value) || value > MAX_JUNCTION_DEVIATION) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    if (value < 0.00001f) { return (STAT_INPUT_LESS_THAN_MIN_VALUE); }
    nv->value_flt = value;
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
    // At least one million canonical units/min^3, also the M201.3 minimum.
    // This keeps supported jerks well above the planner cache's 1000-unit tolerance.
    float canonical = nv->value_flt;
    if ((cm_cfg_axis_type(nv->index) == 0) && (cm_get_units_mode(MODEL) == INCHES)) {
        canonical *= MM_PER_INCH;
    }
    if (canonical < 1.0f) { return (STAT_INPUT_LESS_THAN_MIN_VALUE); }
    // Reject overflow in the expanded canonical jerk before storage.
    if (!isfinite(canonical * JERK_MULTIPLIER)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	if (cm_cfg_axis_type(nv->index) == 0) {		// linear: inches to mm; rotary jerk is in degrees (E110)
		set_flu(nv);
	} else {
		set_flt(nv);
	}
	cm_set_axis_jerk(cm_cfg_axis_index(nv->index), nv->value_flt);
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
    // At least one million canonical units/min^3, also the M201.3 minimum.
    // This keeps supported jerks well above the planner cache's 1000-unit tolerance.
    float canonical = nv->value_flt;
    if ((cm_cfg_axis_type(nv->index) == 0) && (cm_get_units_mode(MODEL) == INCHES)) {
        canonical *= MM_PER_INCH;
    }
    if (canonical < 1.0f) { return (STAT_INPUT_LESS_THAN_MIN_VALUE); }
    // Reject overflow in the expanded canonical jerk before storage.
    if (!isfinite(canonical * JERK_MULTIPLIER)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	if (cm_cfg_axis_type(nv->index) == 0) {		// linear: inches to mm; rotary jerk is in degrees (E110)
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
    (void)nv;
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
