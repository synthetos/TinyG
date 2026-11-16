/*
 * canonical_machine_text.inc - canonical-machine text presentation.
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
 * Included only by canonical_machine.c after its configuration adapters. Keeping
 * these printers in the same translation unit preserves private helper access and
 * compiler inlining; model state and report snapshots remain owned by that file.
 */

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
