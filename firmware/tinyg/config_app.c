/*
 * config_app.c - application-specific part of configuration data
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 *	See config.h for a Config system overview and a bunch of details.
 */

#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "gcode_parser.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"
#include "planner.h"
#include "stepper.h"
//#include "switch.h"
#include "gpio.h"
#include "report.h"
#include "test.h"
#include "util.h"
#include "help.h"

#include "system.h"
#include "network.h"

#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

/*** structures ***/

cfgParameters_t cfg; 				// application specific configuration parameters

/***********************************************************************************
 **** application-specific internal functions **************************************
 ***********************************************************************************/
// See config.cpp/.h for generic variables and functions that are not specific to 
// TinyG or the motion control application domain

// helpers (most helpers are defined immediate above their usage so they don't need prototypes here)

static stat_t _do_motors(cmdObj_t *cmd);// print parameters for all motor groups
static stat_t _do_axes(cmdObj_t *cmd);	// print parameters for all axis groups
static stat_t _do_offsets(cmdObj_t *cmd);// print offsets for G54-G59, G92
static stat_t _do_all(cmdObj_t *cmd);	// print all parameters

// Gcode domain specific functions

static stat_t set_flu(cmdObj_t *cmd);	// set a float with unit conversion
static stat_t get_flu(cmdObj_t *cmd);	// get float with unit conversion
static void print_lin(cmdObj_t *cmd);	// print linear values
static void print_rot(cmdObj_t *cmd);	// print rotary values

// system and application control variables and functions

static stat_t set_hv(cmdObj_t *cmd);		// set hardware version
static stat_t get_id(cmdObj_t *cmd);		// get device ID

static stat_t get_gc(cmdObj_t *cmd);		// get current gcode block
static stat_t run_gc(cmdObj_t *cmd);		// run a gcode block
static stat_t run_home(cmdObj_t *cmd);		// invoke a homing cycle
static stat_t run_boot(cmdObj_t *cmd);		// jump to the bootloader

static stat_t set_sr(cmdObj_t *cmd);		// set status report specification
static stat_t get_sr(cmdObj_t *cmd);		// run status report (as data)
static void print_sr(cmdObj_t *cmd);		// run status report (as printout)
static stat_t set_si(cmdObj_t *cmd);		// set status report interval
static stat_t run_sx(cmdObj_t *cmd);		// send XOFF, XON

static stat_t set_jv(cmdObj_t *cmd);		// set JSON verbosity
static stat_t get_qr(cmdObj_t *cmd);		// get a queue report (as data)
static stat_t run_qf(cmdObj_t *cmd);		// execute a queue flush block
static stat_t get_er(cmdObj_t *cmd);		// invoke a bogus exception report for testing purposes
static stat_t get_rx(cmdObj_t *cmd);		// get bytes in RX buffer

static stat_t set_md(cmdObj_t *cmd);		// disable all motors
static stat_t set_me(cmdObj_t *cmd);		// enable motors with power-mode set to 0 (on)

// communications settings

static stat_t set_ic(cmdObj_t *cmd);		// ignore CR or LF on RX input
static stat_t set_ec(cmdObj_t *cmd);		// expand CRLF on TX outout
static stat_t set_ee(cmdObj_t *cmd);		// enable character echo
static stat_t set_ex(cmdObj_t *cmd);		// enable XON/XOFF and RTS/CTS flow control
static stat_t set_baud(cmdObj_t *cmd);		// set USB baud rate

// gcode model variables and functions

static stat_t get_pos(cmdObj_t *cmd);		// get runtime work position...
static stat_t get_mpos(cmdObj_t *cmd);		// get runtime machine position...
static stat_t get_ofs(cmdObj_t *cmd);		// get runtime work offset...
static void print_pos(cmdObj_t *cmd);		// print runtime work position in prevailing units
static void print_mpos(cmdObj_t *cmd);		// print runtime work position always in MM uints
static void print_ss(cmdObj_t *cmd);		// print switch state
static void print_coor(cmdObj_t *cmd);	// print coordinate offsets with linear units
static void print_corr(cmdObj_t *cmd);	// print coordinate offsets with rotary units

static stat_t get_line(cmdObj_t *cmd);		// get runtime line number
static stat_t get_stat(cmdObj_t *cmd);		// get combined machine state as value and string
static stat_t get_macs(cmdObj_t *cmd);		// get raw machine state as value and string
static stat_t get_cycs(cmdObj_t *cmd);		// get raw cycle state (etc etc)...
static stat_t get_mots(cmdObj_t *cmd);		// get raw motion state...
static stat_t get_hold(cmdObj_t *cmd);		// get raw hold state...
static stat_t get_home(cmdObj_t *cmd);		// get raw homing state...
static stat_t get_unit(cmdObj_t *cmd);		// get unit mode...
static stat_t get_coor(cmdObj_t *cmd);		// get coordinate system in effect...
static stat_t get_momo(cmdObj_t *cmd);		// get motion mode...
static stat_t get_plan(cmdObj_t *cmd);		// get active plane...
static stat_t get_path(cmdObj_t *cmd);		// get patch control mode...
static stat_t get_dist(cmdObj_t *cmd);		// get distance mode...
static stat_t get_frmo(cmdObj_t *cmd);		// get feedrate mode...
static stat_t get_vel(cmdObj_t *cmd);		// get runtime velocity...

// motor and axis variables and functions

static stat_t set_sa(cmdObj_t *cmd);		// set motor step angle
static stat_t set_tr(cmdObj_t *cmd);		// set motor travel per revolution
static stat_t set_mi(cmdObj_t *cmd);		// set microsteps
static stat_t set_po(cmdObj_t *cmd);		// set motor polarity
static stat_t set_pm(cmdObj_t *cmd);		// set motor power mode

static stat_t get_am(cmdObj_t *cmd);		// get axis mode
static stat_t set_am(cmdObj_t *cmd);		// set axis mode
static void print_am(cmdObj_t *cmd);		// print axis mode
//static stat_t get_jrk(cmdObj_t *cmd);		// get jerk with 1,000,000 correction
//static stat_t set_jrk(cmdObj_t *cmd);		// set jerk with 1,000,000 correction
static stat_t set_sw(cmdObj_t *cmd);		// must run any time you change a switch setting

//static void pr_ma_str(cmdObj_t *cmd); // generic print functions for motors and axes
static void pr_ma_ui8(cmdObj_t *cmd);
//static void pr_ma_int(cmdObj_t *cmd); // placeholder
//static void pr_ma_flt(cmdObj_t *cmd); // placeholder
static void pr_ma_lin(cmdObj_t *cmd);
static void pr_ma_rot(cmdObj_t *cmd);

/***********************************************************************************
 **** FLASH STRINGS AND STRING ARRAYS **********************************************
 ***********************************************************************************/
/* Format strings for printf() formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */

static const char_t PROGMEM msg_units0[] = " in";	// used by generic print functions
static const char_t PROGMEM msg_units1[] = " mm";
static const char_t PROGMEM msg_units2[] = " deg";
static PGM_P const  PROGMEM msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char_t PROGMEM msg_g20[] = "G20 - inches mode";
static const char_t PROGMEM msg_g21[] = "G21 - millimeter mode";
static PGM_P const  PROGMEM msg_unit[] = { msg_g20, msg_g21 };

static const char_t PROGMEM msg_stat0[] = "Initializing";	// combined state (stat) uses this array
static const char_t PROGMEM msg_stat1[] = "Ready";
static const char_t PROGMEM msg_stat2[] = "Shutdown";
static const char_t PROGMEM msg_stat3[] = "Stop";
static const char_t PROGMEM msg_stat4[] = "End";
static const char_t PROGMEM msg_stat5[] = "Run";
static const char_t PROGMEM msg_stat6[] = "Hold";
static const char_t PROGMEM msg_stat7[] = "Probe";
static const char_t PROGMEM msg_stat8[] = "Cycle";
static const char_t PROGMEM msg_stat9[] = "Homing";
static const char_t PROGMEM msg_stat10[] = "Jog";
static PGM_P const  PROGMEM msg_stat[] = { msg_stat0, msg_stat1, msg_stat2, msg_stat3, msg_stat4, msg_stat5, 
										   msg_stat6, msg_stat7, msg_stat8, msg_stat9, msg_stat10};

static const char_t PROGMEM msg_macs0[] = "Initializing";
static const char_t PROGMEM msg_macs1[] = "Reset";
static const char_t PROGMEM msg_macs2[] = "Cycle";
static const char_t PROGMEM msg_macs3[] = "Stop";
static const char_t PROGMEM msg_macs4[] = "End";
static PGM_P const  PROGMEM msg_macs[] = { msg_macs0, msg_macs1, msg_macs2, msg_macs3 , msg_macs4};

static const char_t PROGMEM msg_cycs0[] = "Off";
static const char_t PROGMEM msg_cycs1[] = "Started";
static const char_t PROGMEM msg_cycs2[] = "Homing";
static const char_t PROGMEM msg_cycs3[] = "Probe";
static PGM_P const  PROGMEM msg_cycs[] = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3 };

static const char_t PROGMEM msg_mots0[] = "Stop";
static const char_t PROGMEM msg_mots1[] = "Run";
static const char_t PROGMEM msg_mots2[] = "Hold";
static PGM_P const  PROGMEM msg_mots[] = { msg_mots0, msg_mots1, msg_mots2 };

static const char_t PROGMEM msg_hold0[] = "Off";
static const char_t PROGMEM msg_hold1[] = "Sync";
static const char_t PROGMEM msg_hold2[] = "Plan";
static const char_t PROGMEM msg_hold3[] = "Decel";
static const char_t PROGMEM msg_hold4[] = "Hold";
static PGM_P const  PROGMEM msg_hold[] = { msg_hold0, msg_hold1, msg_hold2, msg_hold3, msg_hold4 };

static const char_t PROGMEM msg_home0[] = "Not Homed";
static const char_t PROGMEM msg_home1[] = "Homed";
static PGM_P const  PROGMEM msg_home[] = { msg_home0, msg_home1 };

static const char_t PROGMEM msg_baud0[] = "0";
static const char_t PROGMEM msg_baud1[] = "9600";
static const char_t PROGMEM msg_baud2[] = "19200";
static const char_t PROGMEM msg_baud3[] = "38400";
static const char_t PROGMEM msg_baud4[] = "57600";
static const char_t PROGMEM msg_baud5[] = "115200";
static const char_t PROGMEM msg_baud6[] = "230400";
static PGM_P const  PROGMEM msg_baud[] = { msg_baud0, msg_baud1, msg_baud2, msg_baud3, msg_baud4, msg_baud5, msg_baud6 };

static const char_t PROGMEM msg_sw0[] = "Disabled";
static const char_t PROGMEM msg_sw1[] = "NO homing";
static const char_t PROGMEM msg_sw2[] = "NO homing & limit";
static const char_t PROGMEM msg_sw3[] = "NC homing";
static const char_t PROGMEM msg_sw4[] = "NC homing & limit";
static PGM_P const  PROGMEM msg_sw[] = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };

static const char_t PROGMEM msg_g53[] = "G53 - machine coordinate system";
static const char_t PROGMEM msg_g54[] = "G54 - coordinate system 1";
static const char_t PROGMEM msg_g55[] = "G55 - coordinate system 2";
static const char_t PROGMEM msg_g56[] = "G56 - coordinate system 3";
static const char_t PROGMEM msg_g57[] = "G57 - coordinate system 4";
static const char_t PROGMEM msg_g58[] = "G58 - coordinate system 5";
static const char_t PROGMEM msg_g59[] = "G59 - coordinate system 6";
static PGM_P const  PROGMEM msg_coor[] = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static const char_t PROGMEM msg_g00[] = "G0  - linear traverse (seek)";
static const char_t PROGMEM msg_g01[] = "G1  - linear feed";
static const char_t PROGMEM msg_g02[] = "G2  - clockwise arc feed";
static const char_t PROGMEM msg_g03[] = "G3  - counter clockwise arc feed";
static const char_t PROGMEM msg_g80[] = "G80 - cancel motion mode (none active)";
static PGM_P const  PROGMEM msg_momo[] = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static const char_t PROGMEM msg_g17[] = "G17 - XY plane";
static const char_t PROGMEM msg_g18[] = "G18 - XZ plane";
static const char_t PROGMEM msg_g19[] = "G19 - YZ plane";
static PGM_P const  PROGMEM msg_plan[] = { msg_g17, msg_g18, msg_g19 };

static const char_t PROGMEM msg_g61[] = "G61 - exact stop mode";
static const char_t PROGMEM msg_g6a[] = "G61.1 - exact path mode";
static const char_t PROGMEM msg_g64[] = "G64 - continuous mode";
static PGM_P const  PROGMEM msg_path[] = { msg_g61, msg_g61, msg_g64 };

static const char_t PROGMEM msg_g90[] = "G90 - absolute distance mode";
static const char_t PROGMEM msg_g91[] = "G91 - incremental distance mode";
static PGM_P const  PROGMEM msg_dist[] = { msg_g90, msg_g91 };

static const char_t PROGMEM msg_g94[] = "G94 - units-per-minute mode (i.e. feedrate mode)";
static const char_t PROGMEM msg_g93[] = "G93 - inverse time mode";
static PGM_P const  PROGMEM msg_frmo[] = { msg_g94, msg_g93 };

static const char_t PROGMEM msg_am00[] = "[disabled]";
static const char_t PROGMEM msg_am01[] = "[standard]";
static const char_t PROGMEM msg_am02[] = "[inhibited]";
static const char_t PROGMEM msg_am03[] = "[radius]";
static PGM_P const  PROGMEM msg_am[] = { msg_am00, msg_am01, msg_am02, msg_am03};

/* PROGMEM strings for print formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */
static const char_t PROGMEM fmt_nul[] = "";
static const char_t PROGMEM fmt_ui8[] = "%d\n";	// generic format for ui8s
static const char_t PROGMEM fmt_flt[] = "%f\n";	// generic format for floats
static const char_t PROGMEM fmt_str[] = "%s\n";	// generic format for string message (with no formatting)

// System group and ungrouped formatting strings
static const char_t PROGMEM fmt_fb[] = "[fb]  firmware build%18.2f\n";
static const char_t PROGMEM fmt_fv[] = "[fv]  firmware version%16.2f\n";
static const char_t PROGMEM fmt_hv[] = "[hv]  hardware version%16.2f\n";
static const char_t PROGMEM fmt_id[] = "[id]  TinyG ID%30s\n";

static const char_t PROGMEM fmt_ja[] = "[ja]  junction acceleration%8.0f%S\n";
static const char_t PROGMEM fmt_ct[] = "[ct]  chordal tolerance%16.3f%S\n";

static const char_t PROGMEM fmt_ml[] = "[ml]  min line segment%17.3f%S\n";
static const char_t PROGMEM fmt_ma[] = "[ma]  min arc segment%18.3f%S\n";
static const char_t PROGMEM fmt_ms[] = "[ms]  min segment time%13.0f uSec\n";

static const char_t PROGMEM fmt_st[] = "[st]  switch type%18d [0=NO,1=NC]\n";
static const char_t PROGMEM fmt_si[] = "[si]  status interval%14.0f ms\n";

static const char_t PROGMEM fmt_ic[] = "[ic]  ignore CR or LF on RX%8d [0=off,1=CR,2=LF]\n";
static const char_t PROGMEM fmt_ec[] = "[ec]  expand LF to CRLF on TX%6d [0=off,1=on]\n";
static const char_t PROGMEM fmt_ee[] = "[ee]  enable echo%18d [0=off,1=on]\n";
static const char_t PROGMEM fmt_ex[] = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";

static const char_t PROGMEM fmt_fs[] = "[fs]  footer style%17d [0=old,1]\n";
static const char_t PROGMEM fmt_ej[] = "[ej]  enable json mode%13d [0=text,1=JSON]\n";
static const char_t PROGMEM fmt_jv[] = "[jv]  json verbosity%15d [0=silent,1=footer,2=messages,3=configs,4=linenum,5=verbose]\n";
static const char_t PROGMEM fmt_tv[] = "[tv]  text verbosity%15d [0=silent,1=verbose]\n";
static const char_t PROGMEM fmt_sv[] = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";
static const char_t PROGMEM fmt_qv[] = "[qv]  queue report verbosity%7d [0=off,1=filtered,2=verbose]\n";
static const char_t PROGMEM fmt_baud[] = "[baud] USB baud rate%15d [1=9600,2=19200,3=38400,4=57600,5=115200,6=230400]\n";
static const char_t PROGMEM fmt_net[] = "[net]  network mode%16d [0=master]\n";

static const char_t PROGMEM fmt_qr[] = "qr:%d\n";
static const char_t PROGMEM fmt_rx[] = "rx:%d\n";

static const char_t PROGMEM fmt_md[] = "motors disabled\n";
static const char_t PROGMEM fmt_me[] = "motors enabled\n";
static const char_t PROGMEM fmt_mt[] = "[mt]  motor disable timeout%8d Sec\n";
static const char_t PROGMEM fmt_dd[] = "[dd]  stepper disable delay%8d mSec\n";

// Gcode model values for reporting purposes
static const char_t PROGMEM fmt_vel[]  = "Velocity:%17.3f%S/min\n";
static const char_t PROGMEM fmt_line[] = "Line number:%10.0f\n";
static const char_t PROGMEM fmt_feed[] = "Feed rate:%16.3f%S/min\n";
static const char_t PROGMEM fmt_stat[] = "Machine state:       %s\n"; // combined machine state
static const char_t PROGMEM fmt_macs[] = "Raw machine state:   %s\n"; // raw machine state
static const char_t PROGMEM fmt_cycs[] = "Cycle state:         %s\n";
static const char_t PROGMEM fmt_mots[] = "Motion state:        %s\n";
static const char_t PROGMEM fmt_hold[] = "Feedhold state:      %s\n";
static const char_t PROGMEM fmt_home[] = "Homing state:        %s\n";
static const char_t PROGMEM fmt_unit[] = "Units:               %s\n"; // units mode as ASCII string
static const char_t PROGMEM fmt_coor[] = "Coordinate system:   %s\n";
static const char_t PROGMEM fmt_momo[] = "Motion mode:         %s\n";
static const char_t PROGMEM fmt_plan[] = "Plane:               %s\n";
static const char_t PROGMEM fmt_path[] = "Path Mode:           %s\n";
static const char_t PROGMEM fmt_dist[] = "Distance mode:       %s\n";
static const char_t PROGMEM fmt_frmo[] = "Feed rate mode:      %s\n";
static const char_t PROGMEM fmt_ss[]   = "Switch %s state:     %d\n";

static const char_t PROGMEM fmt_pos[]  = "%c position:%15.3f%S\n";
static const char_t PROGMEM fmt_mpos[] = "%c machine posn:%11.3f%S\n";
static const char_t PROGMEM fmt_ofs[]  = "%c work offset:%12.3f%S\n";
static const char_t PROGMEM fmt_hom[]  = "%c axis homing state:%2.0f\n";

// Motor print formatting strings
static const char_t PROGMEM fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char_t PROGMEM fmt_0sa[] = "[%s%s] m%s step angle%20.3f%S\n";
static const char_t PROGMEM fmt_0tr[] = "[%s%s] m%s travel per revolution%9.3f%S\n";
static const char_t PROGMEM fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char_t PROGMEM fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char_t PROGMEM fmt_0pm[] = "[%s%s] m%s power management%10d [0=remain powered,1=power down when idle]\n";

// Axis print formatting strings
static const char_t PROGMEM fmt_Xam[] = "[%s%s] %s axis mode%18d %S\n";
static const char_t PROGMEM fmt_Xfr[] = "[%s%s] %s feedrate maximum%15.3f%S/min\n";
static const char_t PROGMEM fmt_Xvm[] = "[%s%s] %s velocity maximum%15.3f%S/min\n";
static const char_t PROGMEM fmt_Xtm[] = "[%s%s] %s travel maximum%17.3f%S\n";
static const char_t PROGMEM fmt_Xjm[] = "[%s%s] %s jerk maximum%15.0f%S/min^3\n";
static const char_t PROGMEM fmt_Xjh[] = "[%s%s] %s jerk homing%16.0f%S/min^3\n";
//static const char_t PROGMEM fmt_Xjm[] = "[%s%s] %s jerk maximum (xM)%10.0f%s/min^3\n";
//static const char_t PROGMEM fmt_Xjh[] = "[%s%s] %s jerk homing (xM)%11.0f%s/min^3\n";
static const char_t PROGMEM fmt_Xjd[] = "[%s%s] %s junction deviation%14.4f%S (larger is faster)\n";
static const char_t PROGMEM fmt_Xra[] = "[%s%s] %s radius value%20.4f%S\n";
static const char_t PROGMEM fmt_Xsn[] = "[%s%s] %s switch min%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char_t PROGMEM fmt_Xsx[] = "[%s%s] %s switch max%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char_t PROGMEM fmt_Xsv[] = "[%s%s] %s search velocity%16.3f%S/min\n";
static const char_t PROGMEM fmt_Xlv[] = "[%s%s] %s latch velocity%17.3f%S/min\n";
static const char_t PROGMEM fmt_Xlb[] = "[%s%s] %s latch backoff%18.3f%S\n";
static const char_t PROGMEM fmt_Xzb[] = "[%s%s] %s zero backoff%19.3f%S\n";

// PWM strings
static const char_t PROGMEM fmt_p1frq[] = "[p1frq] pwm frequency   %15.3f Hz\n";
static const char_t PROGMEM fmt_p1csl[] = "[p1csl] pwm cw speed lo %15.3f RPM\n";
static const char_t PROGMEM fmt_p1csh[] = "[p1csh] pwm cw speed hi %15.3f RPM\n";
static const char_t PROGMEM fmt_p1cpl[] = "[p1cpl] pwm cw phase lo %15.3f [0..1]\n";
static const char_t PROGMEM fmt_p1cph[] = "[p1cph] pwm cw phase hi %15.3f [0..1]\n";
static const char_t PROGMEM fmt_p1wsl[] = "[p1wsl] pwm ccw speed lo%15.3f RPM\n";
static const char_t PROGMEM fmt_p1wsh[] = "[p1wsh] pwm ccw speed hi%15.3f RPM\n";
static const char_t PROGMEM fmt_p1wpl[] = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char_t PROGMEM fmt_p1wph[] = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char_t PROGMEM fmt_p1pof[] = "[p1pof] pwm phase off   %15.3f [0..1]\n";

// Coordinate system offset print formatting strings
static const char_t PROGMEM fmt_cofs[] = "[%s%s] %s %s offset%20.3f%S\n";
static const char_t PROGMEM fmt_cloc[] = "[%s%s] %s %s location%18.3f%S\n";

// Gcode model power-on reset default values
static const char_t PROGMEM fmt_gpl[] = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
static const char_t PROGMEM fmt_gun[] = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
static const char_t PROGMEM fmt_gco[] = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
static const char_t PROGMEM fmt_gpa[] = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
static const char_t PROGMEM fmt_gdi[] = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

/***********************************************************************************
 **** CONFIG TABLE  ****************************************************************
 ***********************************************************************************
 *	NOTES:
 *	- Token matching occurs from the most specific to the least specific.
 *	  This means that if shorter tokens overlap longer ones the longer one
 *	  must precede the shorter one. E.g. "gco" needs to come before "gc"
 *
 *	- Mark group strings for entries that have no group as nul -->" ". 
 *	  This is important for group expansion.
 *
 *	- Groups do not have groups. Neither do uber-groups, e.g.
 *	  'x' is --> { "", "x",  	and 'm' is --> { "", "m",  
 */

const cfgItem_t PROGMEM cfgArray[] = {
	// group token flags p, format*, print_func, get_func, set_func, target for get/set,   	default value
	{ "sys", "fb", _f07, 2, fmt_fb, print_flt, get_flt, set_nul, (float *)&cs.fw_build,   TINYG_FIRMWARE_BUILD }, // MUST BE FIRST!
	{ "sys", "fv", _f07, 3, fmt_fv, print_flt, get_flt, set_nul, (float *)&cs.fw_version, TINYG_FIRMWARE_VERSION },
//	{ "sys", "hp", _f07, 0, fmt_hp, print_flt, get_flt, set_flt, (float *)&cs.hw_platform, TINYG2_HARDWARE_PLATFORM },
	{ "sys", "hv", _f07, 0, fmt_hv, print_flt, get_flt, set_hv,  (float *)&cs.hw_version, TINYG_HARDWARE_VERSION },
	{ "sys", "id", _fns, 0, fmt_id, print_str, get_id,  set_nul, (float *)&cs.null, 0 },		// device ID (ASCII signature)

	// dynamic model attributes for reporting purposes (up front for speed)
	{ "",   "n",   _fin, 0, fmt_line, print_int, get_int,  set_int,(float *)&gm.linenum,0 },	// Gcode line number - gets model line number
	{ "",   "line",_fin, 0, fmt_line, print_int, get_line, set_int,(float *)&gm.linenum,0 },	// Gcode line number - gets runtime line number
	{ "",   "feed",_f00, 2, fmt_feed, print_lin, get_flu,  set_nul,(float *)&cs.null, 0 },		// feed rate
	{ "",   "stat",_f00, 0, fmt_stat, print_str, get_stat, set_nul,(float *)&cs.null, 0 },		// combined machine state
	{ "",   "macs",_f00, 0, fmt_macs, print_str, get_macs, set_nul,(float *)&cs.null, 0 },		// raw machine state
	{ "",   "cycs",_f00, 0, fmt_cycs, print_str, get_cycs, set_nul,(float *)&cs.null, 0 },		// cycle state
	{ "",   "mots",_f00, 0, fmt_mots, print_str, get_mots, set_nul,(float *)&cs.null, 0 },		// motion state
	{ "",   "hold",_f00, 0, fmt_hold, print_str, get_hold, set_nul,(float *)&cs.null, 0 },		// feedhold state
	{ "",   "vel", _f00, 2, fmt_vel,  print_lin, get_vel,  set_nul,(float *)&cs.null, 0 },		// current velocity
	{ "",   "unit",_f00, 0, fmt_unit, print_str, get_unit, set_nul,(float *)&cs.null, 0 },		// units mode
	{ "",   "coor",_f00, 0, fmt_coor, print_str, get_coor, set_nul,(float *)&cs.null, 0 },		// coordinate system
	{ "",   "momo",_f00, 0, fmt_momo, print_str, get_momo, set_nul,(float *)&cs.null, 0 },		// motion mode
	{ "",   "plan",_f00, 0, fmt_plan, print_str, get_plan, set_nul,(float *)&cs.null, 0 },		// plane select
	{ "",   "path",_f00, 0, fmt_path, print_str, get_path, set_nul,(float *)&cs.null, 0 },		// path control mode
	{ "",   "dist",_f00, 0, fmt_dist, print_str, get_dist, set_nul,(float *)&cs.null, 0 },		// distance mode
	{ "",   "frmo",_f00, 0, fmt_frmo, print_str, get_frmo, set_nul,(float *)&cs.null, 0 },		// feed rate mode

	{ "mpo","mpox",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// X machine position
	{ "mpo","mpoy",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// Y machine position
	{ "mpo","mpoz",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// Z machine position
	{ "mpo","mpoa",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// A machine position
	{ "mpo","mpob",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// B machine position
	{ "mpo","mpoc",_f00, 3, fmt_mpos, print_mpos, get_mpos, set_nul,(float *)&cs.null, 0 },		// C machine position

	{ "pos","posx",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// X work position
	{ "pos","posy",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// Y work position
	{ "pos","posz",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// Z work position
	{ "pos","posa",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// A work position
	{ "pos","posb",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// B work position
	{ "pos","posc",_f00, 3, fmt_pos, print_pos, get_pos, set_nul,(float *)&cs.null, 0 },		// C work position

	{ "ofs","ofsx",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// X work offset
	{ "ofs","ofsy",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// Y work offset
	{ "ofs","ofsz",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// Z work offset
	{ "ofs","ofsa",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// A work offset 
	{ "ofs","ofsb",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// B work offset 
	{ "ofs","ofsc",_f00, 3, fmt_ofs, print_mpos, get_ofs, set_nul,(float *)&cs.null, 0 },		// C work offset

	{ "hom","home",_f00, 0, fmt_home,print_str, get_home,run_home,(float *)&cs.null, 0 },	   // homing state, invoke homing cycle
	{ "hom","homx",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_X], false },// X homed - Homing status group
	{ "hom","homy",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_Y], false },// Y homed
	{ "hom","homz",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_Z], false },// Z homed
	{ "hom","homa",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_A], false },// A homed
	{ "hom","homb",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_B], false },// B homed
	{ "hom","homc",_f00, 0, fmt_hom, print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_C], false },// C homed

	// Reports, tests, help, and messages
	{ "", "sr",  _f00, 0, fmt_nul, print_sr,  get_sr,  set_sr , (float *)&cs.null, 0 },			// status report object
	{ "", "qr",  _f00, 0, fmt_qr,  print_int, get_qr,  set_nul, (float *)&cs.null, 0 },			// queue report setting
	{ "", "qf",  _f00, 0, fmt_nul, print_nul, get_nul, run_qf,  (float *)&cs.null, 0 },			// queue flush
	{ "", "er",  _f00, 0, fmt_nul, print_nul, get_er,  set_nul, (float *)&cs.null, 0 },			// invoke bogus exception report for testing
	{ "", "rx",  _f00, 0, fmt_rx,  print_int, get_rx,  set_nul, (float *)&cs.null, 0 },			// space in RX buffer
	{ "", "msg", _f00, 0, fmt_str, print_str, get_nul, set_nul, (float *)&cs.null, 0 },			// string for generic messages
	{ "", "defa",_f00, 0, fmt_nul, print_nul, print_defaults_help, set_defaults,(float *)&cs.null,0},	// set/print defaults / help screen
	{ "", "test",_f00, 0, fmt_nul, print_nul, print_test_help, tg_test, (float *)&cs.null,0 },			// run tests, print test help screen
	{ "", "boot",_f00, 0, fmt_nul, print_nul, print_boot_loader_help,run_boot,(float *)&cs.null,0 },
	{ "", "help",_f00, 0, fmt_nul, print_nul, print_config_help, set_nul, (float *)&cs.null,0 },		// prints config help screen
	{ "", "h",   _f00, 0, fmt_nul, print_nul, print_config_help, set_nul, (float *)&cs.null,0 },		// alias for "help"
	{ "", "sx",  _f00, 0, fmt_nul, print_nul, run_sx,  run_sx , (float *)&cs.null, 0 },					// send XOFF, XON test

	// Motor parameters
	{ "1","1ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_1].motor_map,	M1_MOTOR_MAP },
	{ "1","1sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt , set_sa, (float *)&cfg.m[MOTOR_1].step_angle,	M1_STEP_ANGLE },
	{ "1","1tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu , set_tr, (float *)&cfg.m[MOTOR_1].travel_rev,	M1_TRAVEL_PER_REV },
	{ "1","1mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_1].microsteps,	M1_MICROSTEPS },
	{ "1","1po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_1].polarity,	M1_POLARITY },
	{ "1","1pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_1].power_mode,	M1_POWER_MODE },
#if (MOTORS >= 2)
	{ "2","2ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_2].motor_map,	M2_MOTOR_MAP },
	{ "2","2sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt, set_sa, (float *)&cfg.m[MOTOR_2].step_angle,	M2_STEP_ANGLE },
	{ "2","2tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu, set_tr, (float *)&cfg.m[MOTOR_2].travel_rev,	M2_TRAVEL_PER_REV },
	{ "2","2mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_2].microsteps,	M2_MICROSTEPS },
	{ "2","2po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_2].polarity,	M2_POLARITY },
	{ "2","2pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_2].power_mode,	M2_POWER_MODE },
#endif
#if (MOTORS >= 3)
	{ "3","3ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_3].motor_map,	M3_MOTOR_MAP },
	{ "3","3sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt, set_sa, (float *)&cfg.m[MOTOR_3].step_angle,	M3_STEP_ANGLE },
	{ "3","3tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu, set_tr, (float *)&cfg.m[MOTOR_3].travel_rev,	M3_TRAVEL_PER_REV },
	{ "3","3mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_3].microsteps,	M3_MICROSTEPS },
	{ "3","3po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_3].polarity,	M3_POLARITY },
	{ "3","3pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_3].power_mode,	M3_POWER_MODE },
#endif
#if (MOTORS >= 4)
	{ "4","4ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_4].motor_map,	M4_MOTOR_MAP },
	{ "4","4sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt, set_sa, (float *)&cfg.m[MOTOR_4].step_angle,	M4_STEP_ANGLE },
	{ "4","4tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu, set_tr, (float *)&cfg.m[MOTOR_4].travel_rev,	M4_TRAVEL_PER_REV },
	{ "4","4mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_4].microsteps,	M4_MICROSTEPS },
	{ "4","4po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_4].polarity,	M4_POLARITY },
	{ "4","4pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_4].power_mode,	M4_POWER_MODE },
#endif
#if (MOTORS >= 5)
	{ "5","5ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_5].motor_map,	M5_MOTOR_MAP },
	{ "5","5sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt, set_sa, (float *)&cfg.m[MOTOR_5].step_angle,	M5_STEP_ANGLE },
	{ "5","5tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu, set_tr, (float *)&cfg.m[MOTOR_5].travel_rev,	M5_TRAVEL_PER_REV },
	{ "5","5mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_5].microsteps,	M5_MICROSTEPS },
	{ "5","5po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_5].polarity,	M5_POLARITY },
	{ "5","5pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_5].power_mode,	M5_POWER_MODE },
#endif
#if (MOTORS >= 6)
	{ "6","6ma",_fip, 0, fmt_0ma, pr_ma_ui8, get_ui8, set_ui8,(float *)&cfg.m[MOTOR_6].motor_map,	M6_MOTOR_MAP },
	{ "6","6sa",_fip, 2, fmt_0sa, pr_ma_rot, get_flt, set_sa, (float *)&cfg.m[MOTOR_6].step_angle,	M6_STEP_ANGLE },
	{ "6","6tr",_fip, 3, fmt_0tr, pr_ma_lin, get_flu, set_tr, (float *)&cfg.m[MOTOR_6].travel_rev,	M6_TRAVEL_PER_REV },
	{ "6","6mi",_fip, 0, fmt_0mi, pr_ma_ui8, get_ui8, set_mi, (float *)&cfg.m[MOTOR_6].microsteps,	M6_MICROSTEPS },
	{ "6","6po",_fip, 0, fmt_0po, pr_ma_ui8, get_ui8, set_po, (float *)&cfg.m[MOTOR_6].polarity,	M6_POLARITY },
	{ "6","6pm",_fip, 0, fmt_0pm, pr_ma_ui8, get_ui8, set_pm, (float *)&cfg.m[MOTOR_6].power_mode,	M6_POWER_MODE },
#endif

	// Axis parameters
	{ "x","xam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_X].axis_mode,	X_AXIS_MODE },
	{ "x","xvm",_fip, 0, fmt_Xvm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].velocity_max,	X_VELOCITY_MAX },
	{ "x","xfr",_fip, 0, fmt_Xfr, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].feedrate_max,	X_FEEDRATE_MAX },
	{ "x","xtm",_fip, 0, fmt_Xtm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].travel_max,	X_TRAVEL_MAX },
	{ "x","xjm",_fip, 0, fmt_Xjm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].jerk_max,		X_JERK_MAX },
	{ "x","xjh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].jerk_homing,	X_JERK_HOMING },
	{ "x","xjd",_fip, 4, fmt_Xjd, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].junction_dev,	X_JUNCTION_DEVIATION },
	{ "x","xsn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[0],					X_SWITCH_MODE_MIN },
	{ "x","xsx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[1],					X_SWITCH_MODE_MAX },
	{ "x","xsv",_fip, 0, fmt_Xsv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].search_velocity,X_SEARCH_VELOCITY },
	{ "x","xlv",_fip, 0, fmt_Xlv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].latch_velocity,X_LATCH_VELOCITY },
	{ "x","xlb",_fip, 3, fmt_Xlb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].latch_backoff,X_LATCH_BACKOFF },
	{ "x","xzb",_fip, 3, fmt_Xzb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_X].zero_backoff,	X_ZERO_BACKOFF },

	{ "y","yam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_Y].axis_mode,	Y_AXIS_MODE },
	{ "y","yvm",_fip, 0, fmt_Xvm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].velocity_max,	Y_VELOCITY_MAX },
	{ "y","yfr",_fip, 0, fmt_Xfr, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].feedrate_max,	Y_FEEDRATE_MAX },
	{ "y","ytm",_fip, 0, fmt_Xtm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].travel_max,	Y_TRAVEL_MAX },
	{ "y","yjm",_fip, 0, fmt_Xjm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].jerk_max,		Y_JERK_MAX },
	{ "y","yjh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].jerk_homing,	Y_JERK_HOMING },
	{ "y","yjd",_fip, 4, fmt_Xjd, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].junction_dev,	Y_JUNCTION_DEVIATION },
	{ "y","ysn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[2],					Y_SWITCH_MODE_MIN },
	{ "y","ysx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[3],					Y_SWITCH_MODE_MAX },
	{ "y","ysv",_fip, 0, fmt_Xsv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].search_velocity,Y_SEARCH_VELOCITY },
	{ "y","ylv",_fip, 0, fmt_Xlv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].latch_velocity,Y_LATCH_VELOCITY },
	{ "y","ylb",_fip, 3, fmt_Xlb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].latch_backoff,Y_LATCH_BACKOFF },
	{ "y","yzb",_fip, 3, fmt_Xzb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Y].zero_backoff,	Y_ZERO_BACKOFF },

	{ "z","zam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_Z].axis_mode,	Z_AXIS_MODE },
	{ "z","zvm",_fip, 0, fmt_Xvm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].velocity_max,	Z_VELOCITY_MAX },
	{ "z","zfr",_fip, 0, fmt_Xfr, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].feedrate_max,	Z_FEEDRATE_MAX },
	{ "z","ztm",_fip, 0, fmt_Xtm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].travel_max,	Z_TRAVEL_MAX },
	{ "z","zjm",_fip, 0, fmt_Xjm, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].jerk_max,		Z_JERK_MAX },
	{ "z","zjh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].jerk_homing,	Z_JERK_HOMING },
	{ "z","zjd",_fip, 4, fmt_Xjd, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].junction_dev,	Z_JUNCTION_DEVIATION },
	{ "z","zsn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[4],					Z_SWITCH_MODE_MIN },
	{ "z","zsx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[5],					Z_SWITCH_MODE_MAX },
	{ "z","zsv",_fip, 0, fmt_Xsv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].search_velocity,Z_SEARCH_VELOCITY },
	{ "z","zlv",_fip, 0, fmt_Xlv, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].latch_velocity,Z_LATCH_VELOCITY },
	{ "z","zlb",_fip, 3, fmt_Xlb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].latch_backoff,Z_LATCH_BACKOFF },
	{ "z","zzb",_fip, 3, fmt_Xzb, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_Z].zero_backoff,	Z_ZERO_BACKOFF },

	{ "a","aam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_A].axis_mode,	A_AXIS_MODE },
	{ "a","avm",_fip, 0, fmt_Xvm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].velocity_max,	A_VELOCITY_MAX },
	{ "a","afr",_fip, 0, fmt_Xfr, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].feedrate_max, A_FEEDRATE_MAX },
	{ "a","atm",_fip, 0, fmt_Xtm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].travel_max,	A_TRAVEL_MAX },
	{ "a","ajm",_fip, 0, fmt_Xjm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].jerk_max,		A_JERK_MAX },
	{ "a","ajh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_A].jerk_homing,	A_JERK_HOMING },
	{ "a","ajd",_fip, 4, fmt_Xjd, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].junction_dev,	A_JUNCTION_DEVIATION },
	{ "a","ara",_fip, 3, fmt_Xra, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].radius,		A_RADIUS},
	{ "a","asn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[6],					A_SWITCH_MODE_MIN },
	{ "a","asx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[7],					A_SWITCH_MODE_MAX },
	{ "a","asv",_fip, 0, fmt_Xsv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].search_velocity,A_SEARCH_VELOCITY },
	{ "a","alv",_fip, 0, fmt_Xlv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].latch_velocity,A_LATCH_VELOCITY },
	{ "a","alb",_fip, 3, fmt_Xlb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].latch_backoff,A_LATCH_BACKOFF },
	{ "a","azb",_fip, 3, fmt_Xzb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_A].zero_backoff,	A_ZERO_BACKOFF },

	{ "b","bam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_B].axis_mode,	B_AXIS_MODE },
	{ "b","bvm",_fip, 0, fmt_Xvm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].velocity_max,	B_VELOCITY_MAX },
	{ "b","bfr",_fip, 0, fmt_Xfr, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].feedrate_max, B_FEEDRATE_MAX },
	{ "b","btm",_fip, 0, fmt_Xtm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].travel_max,	B_TRAVEL_MAX },
	{ "b","bjm",_fip, 0, fmt_Xjm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].jerk_max,		B_JERK_MAX },
//	{ "b","bjh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_B].jerk_homing,	B_JERK_HOMING },
	{ "b","bjd",_fip, 0, fmt_Xjd, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].junction_dev,	B_JUNCTION_DEVIATION },
	{ "b","bra",_fip, 3, fmt_Xra, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].radius,		B_RADIUS },
//	{ "a","asn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[6],					B_SWITCH_MODE_MIN },
//	{ "a","asx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[7],					B_SWITCH_MODE_MAX },
//	{ "b","bsv",_fip, 0, fmt_Xsv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].search_velocity,B_SEARCH_VELOCITY },
//	{ "b","blv",_fip, 0, fmt_Xlv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].latch_velocity,B_LATCH_VELOCITY },
//	{ "b","blb",_fip, 3, fmt_Xlb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].latch_backoff,B_LATCH_BACKOFF },
//	{ "b","bzb",_fip, 3, fmt_Xzb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_B].zero_backoff,	B_ZERO_BACKOFF },

	{ "c","cam",_fip, 0, fmt_Xam, print_am,  get_am,  set_am, (float *)&cfg.a[AXIS_C].axis_mode,	C_AXIS_MODE },
	{ "c","cvm",_fip, 0, fmt_Xvm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].velocity_max,	C_VELOCITY_MAX },
	{ "c","cfr",_fip, 0, fmt_Xfr, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].feedrate_max,	C_FEEDRATE_MAX },
	{ "c","ctm",_fip, 0, fmt_Xtm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].travel_max,	C_TRAVEL_MAX },
	{ "c","cjm",_fip, 0, fmt_Xjm, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].jerk_max,		C_JERK_MAX },
//	{ "c","cjh",_fip, 0, fmt_Xjh, pr_ma_lin, get_flu, set_flu,(float *)&cfg.a[AXIS_C].jerk_homing,	C_JERK_HOMING },
	{ "c","cjd",_fip, 0, fmt_Xjd, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].junction_dev,	C_JUNCTION_DEVIATION },
	{ "c","cra",_fip, 3, fmt_Xra, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].radius,		C_RADIUS },
//	{ "c","csn",_fip, 0, fmt_Xsn, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[6],					C_SWITCH_MODE_MIN },
//	{ "c","csx",_fip, 0, fmt_Xsx, pr_ma_ui8, get_ui8, set_sw, (float *)&sw.mode[7],					C_SWITCH_MODE_MAX },
//	{ "c","csv",_fip, 0, fmt_Xsv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].search_velocity,C_SEARCH_VELOCITY },
//	{ "c","clv",_fip, 0, fmt_Xlv, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].latch_velocity,C_LATCH_VELOCITY },
//	{ "c","clb",_fip, 3, fmt_Xlb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].latch_backoff,C_LATCH_BACKOFF },
//	{ "c","czb",_fip, 3, fmt_Xzb, pr_ma_rot, get_flt, set_flt,(float *)&cfg.a[AXIS_C].zero_backoff,	C_ZERO_BACKOFF },

	// PWM settings
    { "p1","p1frq",_fip, 0, fmt_p1frq, print_flt, get_flt, set_flt,(float *)&cfg.p.frequency,		P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, 0, fmt_p1csl, print_flt, get_flt, set_flt,(float *)&cfg.p.cw_speed_lo,		P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, 0, fmt_p1csh, print_flt, get_flt, set_flt,(float *)&cfg.p.cw_speed_hi,		P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, 3, fmt_p1cpl, print_flt, get_flt, set_flt,(float *)&cfg.p.cw_phase_lo,		P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, 3, fmt_p1cph, print_flt, get_flt, set_flt,(float *)&cfg.p.cw_phase_hi,		P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, 0, fmt_p1wsl, print_flt, get_flt, set_flt,(float *)&cfg.p.ccw_speed_lo,	P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, 0, fmt_p1wsh, print_flt, get_flt, set_flt,(float *)&cfg.p.ccw_speed_hi,	P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, 3, fmt_p1wpl, print_flt, get_flt, set_flt,(float *)&cfg.p.ccw_phase_lo,	P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, 3, fmt_p1wph, print_flt, get_flt, set_flt,(float *)&cfg.p.ccw_phase_hi,	P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, 3, fmt_p1pof, print_rot, get_flt, set_flt,(float *)&cfg.p.phase_off,		P1_PWM_PHASE_OFF },

	// Coordinate system offsets (G54-G59 and G92)
	{ "g54","g54x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_X], G54_X_OFFSET },
	{ "g54","g54y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_Y], G54_Y_OFFSET },
	{ "g54","g54z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_Z], G54_Z_OFFSET },
	{ "g54","g54a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_A], G54_A_OFFSET },
	{ "g54","g54b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_B], G54_B_OFFSET },
	{ "g54","g54c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G54][AXIS_C], G54_C_OFFSET },

	{ "g55","g55x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_X], G55_X_OFFSET },
	{ "g55","g55y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_Y], G55_Y_OFFSET },
	{ "g55","g55z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_Z], G55_Z_OFFSET },
	{ "g55","g55a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_A], G55_A_OFFSET },
	{ "g55","g55b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_B], G55_B_OFFSET },
	{ "g55","g55c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G55][AXIS_C], G55_C_OFFSET },

	{ "g56","g56x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_X], G56_X_OFFSET },
	{ "g56","g56y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_Y], G56_Y_OFFSET },
	{ "g56","g56z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_Z], G56_Z_OFFSET },
	{ "g56","g56a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_A], G56_A_OFFSET },
	{ "g56","g56b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_B], G56_B_OFFSET },
	{ "g56","g56c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G56][AXIS_C], G56_C_OFFSET },

	{ "g57","g57x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_X], G57_X_OFFSET },
	{ "g57","g57y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_Y], G57_Y_OFFSET },
	{ "g57","g57z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_Z], G57_Z_OFFSET },
	{ "g57","g57a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_A], G57_A_OFFSET },
	{ "g57","g57b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_B], G57_B_OFFSET },
	{ "g57","g57c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G57][AXIS_C], G57_C_OFFSET },

	{ "g58","g58x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_X], G58_X_OFFSET },
	{ "g58","g58y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_Y], G58_Y_OFFSET },
	{ "g58","g58z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_Z], G58_Z_OFFSET },
	{ "g58","g58a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_A], G58_A_OFFSET },
	{ "g58","g58b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_B], G58_B_OFFSET },
	{ "g58","g58c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G58][AXIS_C], G58_C_OFFSET },

	{ "g59","g59x",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_X], G59_X_OFFSET },
	{ "g59","g59y",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_Y], G59_Y_OFFSET },
	{ "g59","g59z",_fip, 3, fmt_cofs, print_coor, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_Z], G59_Z_OFFSET },
	{ "g59","g59a",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_A], G59_A_OFFSET },
	{ "g59","g59b",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_B], G59_B_OFFSET },
	{ "g59","g59c",_fip, 3, fmt_cofs, print_corr, get_flu, set_flu,(float *)&cfg.offset[G59][AXIS_C], G59_C_OFFSET },

	{ "g92","g92x",_fin, 3, fmt_cofs, print_coor, get_flu, set_nul,(float *)&gm.origin_offset[AXIS_X], 0 },// G92 handled differently
	{ "g92","g92y",_fin, 3, fmt_cofs, print_coor, get_flu, set_nul,(float *)&gm.origin_offset[AXIS_Y], 0 },
	{ "g92","g92z",_fin, 3, fmt_cofs, print_coor, get_flu, set_nul,(float *)&gm.origin_offset[AXIS_Z], 0 },
	{ "g92","g92a",_fin, 3, fmt_cofs, print_corr, get_flt, set_nul,(float *)&gm.origin_offset[AXIS_A], 0 },
	{ "g92","g92b",_fin, 3, fmt_cofs, print_corr, get_flt, set_nul,(float *)&gm.origin_offset[AXIS_B], 0 },
	{ "g92","g92c",_fin, 3, fmt_cofs, print_corr, get_flt, set_nul,(float *)&gm.origin_offset[AXIS_C], 0 },

	{ "g28","g28x",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g28_position[AXIS_X], 0 },// g28 handled differently
	{ "g28","g28y",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g28_position[AXIS_Y], 0 },
	{ "g28","g28z",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g28_position[AXIS_Z], 0 },
	{ "g28","g28a",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g28_position[AXIS_A], 0 },
	{ "g28","g28b",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g28_position[AXIS_B], 0 },
	{ "g28","g28c",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g28_position[AXIS_C], 0 },

	{ "g30","g30x",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g30_position[AXIS_X], 0 },// g30 handled differently
	{ "g30","g30y",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g30_position[AXIS_Y], 0 },
	{ "g30","g30z",_fin, 3, fmt_cloc, print_coor, get_flu, set_nul,(float *)&gm.g30_position[AXIS_Z], 0 },
	{ "g30","g30a",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g30_position[AXIS_A], 0 },
	{ "g30","g30b",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g30_position[AXIS_B], 0 },
	{ "g30","g30c",_fin, 3, fmt_cloc, print_corr, get_flt, set_nul,(float *)&gm.g30_position[AXIS_C], 0 },

	// System parameters
	{ "sys","ja",  _f07, 0, fmt_ja, print_lin, get_flu, set_flu, (float *)&cfg.junction_acceleration,	JUNCTION_ACCELERATION },
	{ "sys","ct",  _f07, 4, fmt_ct, print_lin, get_flu, set_flu, (float *)&cfg.chordal_tolerance,		CHORDAL_TOLERANCE },
//	{ "sys","dd",  _f07, 0, fmt_dd, print_int, get_int, set_int, (float *)&cfg.stepper_disable_delay,	DISABLE_DELAY },
	{ "sys","st",  _f07, 0, fmt_st, print_ui8, get_ui8, set_sw,  (float *)&sw.switch_type,				SWITCH_TYPE },
	{ "sys","mt",  _f07, 0, fmt_mt, print_int, get_int, set_int, (float *)&cfg.motor_disable_timeout,	MOTOR_DISABLE_TIMEOUT},
	// Note:"me" must initialize after "mt" so it can use the timeout value
	{ "",   "me",  _fin, 0, fmt_me, print_str, set_me,  set_me,  (float *)&cs.null, 0 },				// enable all motors
	{ "",   "md",  _f00, 0, fmt_md, print_str, set_md,  set_md,  (float *)&cs.null, 0 },				// disable all motors
	
	{ "sys","ej",  _f07, 0, fmt_ej, print_ui8, get_ui8, set_01,  (float *)&cfg.comm_mode,				COMM_MODE },
	{ "sys","jv",  _f07, 0, fmt_jv, print_ui8, get_ui8, set_jv,  (float *)&cfg.json_verbosity,			JSON_VERBOSITY },
	{ "sys","tv",  _f07, 0, fmt_tv, print_ui8, get_ui8, set_01,  (float *)&cfg.text_verbosity,			TEXT_VERBOSITY },
	{ "sys","qv",  _f07, 0, fmt_qv, print_ui8, get_ui8, set_0123,(float *)&cfg.queue_report_verbosity,	QR_VERBOSITY },
	{ "sys","sv",  _f07, 0, fmt_sv, print_ui8, get_ui8, set_012, (float *)&cfg.status_report_verbosity,	SR_VERBOSITY },
	{ "sys","si",  _f07, 0, fmt_si, print_flt, get_int, set_si,  (float *)&cfg.status_report_interval,	STATUS_REPORT_INTERVAL_MS },

	{ "sys","ic",  _f07, 0, fmt_ic, print_ui8, get_ui8, set_ic,  (float *)&cfg.ignore_crlf,				COM_IGNORE_CRLF },
	{ "sys","ec",  _f07, 0, fmt_ec, print_ui8, get_ui8, set_ec,  (float *)&cfg.enable_cr,				COM_EXPAND_CR },
	{ "sys","ee",  _f07, 0, fmt_ee, print_ui8, get_ui8, set_ee,  (float *)&cfg.enable_echo,				COM_ENABLE_ECHO },
	{ "sys","ex",  _f07, 0, fmt_ex, print_ui8, get_ui8, set_ex,  (float *)&cfg.enable_flow_control,		COM_ENABLE_FLOW_CONTROL },
	{ "sys","fs",  _f07, 0, fmt_fs, print_ui8, get_ui8, set_ui8, (float *)&cfg.footer_style,			0 },
	{ "sys","baud",_fns, 0, fmt_baud,print_ui8,get_ui8, set_baud,(float *)&cfg.usb_baud_rate,			XIO_BAUD_115200 },
	{ "sys","net", _fip, 0, fmt_net,print_ui8, get_ui8, set_ui8, (float *)&cs.network_mode,				NETWORK_MODE },

	// switch state readers
	{ "ss","ss0",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[0], 0 },
	{ "ss","ss1",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[1], 0 },
	{ "ss","ss2",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[2], 0 },
	{ "ss","ss3",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[3], 0 },
	{ "ss","ss4",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[4], 0 },
	{ "ss","ss5",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[5], 0 },
	{ "ss","ss6",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[6], 0 },
	{ "ss","ss7",  _f00, 0, fmt_ss, print_ss, get_ui8, set_nul, (float *)&sw.state[7], 0 },

	// NOTE: The ordering within the gcode defaults is important for token resolution
	{ "sys","gpl", _f07, 0, fmt_gpl, print_ui8, get_ui8, set_012, (float *)&cfg.select_plane,		GCODE_DEFAULT_PLANE },
	{ "sys","gun", _f07, 0, fmt_gun, print_ui8, get_ui8, set_01,  (float *)&cfg.units_mode,			GCODE_DEFAULT_UNITS },
	{ "sys","gco", _f07, 0, fmt_gco, print_ui8, get_ui8, set_ui8, (float *)&cfg.coord_system,		GCODE_DEFAULT_COORD_SYSTEM },
	{ "sys","gpa", _f07, 0, fmt_gpa, print_ui8, get_ui8, set_012, (float *)&cfg.path_control,		GCODE_DEFAULT_PATH_CONTROL },
	{ "sys","gdi", _f07, 0, fmt_gdi, print_ui8, get_ui8, set_01,  (float *)&cfg.distance_mode,		GCODE_DEFAULT_DISTANCE_MODE },
	{ "",   "gc",  _f00, 0, fmt_nul, print_nul, get_gc,  run_gc,  (float *)&cs.null, 0 }, // gcode block - must be last in this group

	// removed from system group as "hidden" parameters
	{ "",   "ms",  _fip, 0, fmt_ms, print_lin, get_flt, set_flt, (float *)&cfg.estd_segment_usec,	NOM_SEGMENT_USEC },
	{ "",   "ml",  _fip, 4, fmt_ml, print_lin, get_flu, set_flu, (float *)&cfg.min_segment_len,		MIN_LINE_LENGTH },
	{ "",   "ma",  _fip, 4, fmt_ma, print_lin, get_flu, set_flu, (float *)&cfg.arc_segment_len,		ARC_SEGMENT_LENGTH },
	{ "",   "qrh", _fip, 0, fmt_ui8,print_ui8, get_ui8, set_ui8, (float *)&cfg.queue_report_hi_water,QR_HI_WATER },
	{ "",   "qrl", _fip, 0, fmt_ui8,print_ui8, get_ui8, set_ui8, (float *)&cfg.queue_report_lo_water,QR_LO_WATER },
	{ "",   "qrl", _fip, 0, fmt_ui8,print_ui8, get_ui8, set_ui8, (float *)&cfg.queue_report_lo_water,QR_LO_WATER },

	// Persistence for status report - must be in sequence
	// *** Count must agree with CMD_STATUS_REPORT_LEN in config.h ***
	{ "","se00",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[0],0 },
	{ "","se01",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[1],0 },
	{ "","se02",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[2],0 },
	{ "","se03",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[3],0 },
	{ "","se04",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[4],0 },
	{ "","se05",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[5],0 },
	{ "","se06",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[6],0 },
	{ "","se07",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[7],0 },
	{ "","se08",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[8],0 },
	{ "","se09",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[9],0 },
	{ "","se10",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[10],0 },
	{ "","se11",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[11],0 },
	{ "","se12",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[12],0 },
	{ "","se13",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[13],0 },
	{ "","se14",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[14],0 },
	{ "","se15",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[15],0 },
	{ "","se16",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[16],0 },
	{ "","se17",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[17],0 },
	{ "","se18",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[18],0 },
	{ "","se19",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[19],0 },
	{ "","se20",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[20],0 },
	{ "","se21",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[21],0 },
	{ "","se22",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[22],0 },
	{ "","se23",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[23],0 },
	{ "","se24",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[24],0 },
	{ "","se25",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[25],0 },
	{ "","se26",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[26],0 },
	{ "","se27",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[27],0 },
	{ "","se28",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[28],0 },
	{ "","se29",_fpe, 0, fmt_nul, print_nul, get_int, set_int,(float *)&cfg.status_report_list[29],0 },

	// Group lookups - must follow the single-valued entries for proper sub-string matching
	// *** Must agree with CMD_COUNT_GROUPS below ****
	{ "","sys",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// system group
	{ "","p1", _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// PWM 1 group
	{ "","1",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// motor groups
	{ "","2",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","3",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","4",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","x",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// axis groups
	{ "","y",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","z",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","a",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","b",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","c",  _f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","ss", _f00, 0, fmt_nul, print_nul, get_grp, set_nul,(float *)&cs.null,0 },
	{ "","g54",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// coord offset groups
	{ "","g55",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g56",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g57",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g58",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g59",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g92",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// origin offsets
	{ "","g28",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// g28 home position
	{ "","g30",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// g30 home position
	{ "","mpo",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// machine position group
	{ "","pos",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// work position group
	{ "","ofs",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// work offset group
	{ "","hom",_f00, 0, fmt_nul, print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// axis homing state group

	// Uber-group (groups of groups, for text-mode displays only)
	// *** Must agree with CMD_COUNT_UBER_GROUPS below ****
	{ "", "m", _f00, 0, fmt_nul, print_nul, _do_motors, set_nul,(float *)&cs.null,0 },
	{ "", "q", _f00, 0, fmt_nul, print_nul, _do_axes,   set_nul,(float *)&cs.null,0 },
	{ "", "o", _f00, 0, fmt_nul, print_nul, _do_offsets,set_nul,(float *)&cs.null,0 },
	{ "", "$", _f00, 0, fmt_nul, print_nul, _do_all,    set_nul,(float *)&cs.null,0 }
};

/***** Make sure these defines line up with any changes in the above table *****/

#define CMD_COUNT_GROUPS 		26		// count of simple groups
#define CMD_COUNT_UBER_GROUPS 	4 		// count of uber-groups

/* <DO NOT MESS WITH THESE DEFINES> */
#define CMD_INDEX_MAX (sizeof cfgArray / sizeof(cfgItem_t))
#define CMD_INDEX_END_SINGLES		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS - CMD_STATUS_REPORT_LEN)
#define CMD_INDEX_START_GROUPS		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS)
#define CMD_INDEX_START_UBER_GROUPS (CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS)
/* </DO NOT MESS WITH THESE DEFINES> */

index_t	cmd_index_max() { return ( CMD_INDEX_MAX );}
uint8_t cmd_index_lt_max(index_t index) { return ((index < CMD_INDEX_MAX) ? true : false);}
uint8_t cmd_index_ge_max(index_t index) { return ((index >= CMD_INDEX_MAX) ? true : false);}
uint8_t cmd_index_is_single(index_t index) { return ((index <= CMD_INDEX_END_SINGLES) ? true : false);}
uint8_t cmd_index_is_group(index_t index) { return (((index >= CMD_INDEX_START_GROUPS) && (index < CMD_INDEX_START_UBER_GROUPS)) ? true : false);}
uint8_t cmd_index_lt_groups(index_t index) { return ((index <= CMD_INDEX_START_GROUPS) ? true : false);}

/***********************************************************************************
 **** APPLICATION SPECIFIC FUNCTIONS ***********************************************
 ***********************************************************************************/

/***** HELPERS *********************************************************************
 *
 * Helpers are defined first so they don't need separate function prototypes
 *
 * _get_motor()	- return motor number as an index or -1 if na
 * _get_axis()	- return axis number or -1 if NA
 * _get_pos_axis()- return axis number for pos values or -1 if none - e.g. posx
 */

static int8_t _get_motor(const index_t i)
{
	char_t *ptr;
	char_t motors[] = {"1234"};
	char_t tmp[CMD_TOKEN_LEN+1];

	strcpy_P(tmp, cfgArray[i].group);
	if ((ptr = strchr(motors, tmp[0])) == NULL) {
		return (-1);
	}
	return (ptr - motors);
}
/*
static int8_t _get_axis(const index_t i)
{
	char_t *ptr;
	char_t tmp[CMD_TOKEN_LEN+1];
	char_t axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[0])) == NULL) { return (-1);}
	return (ptr - axes);
}
*/
static int8_t _get_pos_axis(const index_t i)
{
	char_t *ptr;
	char_t tmp[CMD_TOKEN_LEN+1];
	char_t axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[3])) == NULL) { return (-1);}
	return (ptr - axes);
}

/***** DOMAIN SPECIFIC EXTENSIONS TO GENERIC FUNCTIONS ************************
 * get_flu()   - get floating point number with Gcode units conversion
 * set_flu()   - set floating point number with Gcode units conversion
 * print_lin() - print linear axis value with Gcode units conversion
 * print_rot() - print rotary axis value with Gcode units conversion
 */
static stat_t set_flu(cmdObj_t *cmd)
{
	if (cm_get_model_units_mode() == INCHES) { cmd->value *= MM_PER_INCH;}
	*((float *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT_UNITS;
	return(STAT_OK);
}

static stat_t get_flu(cmdObj_t *cmd)
{
	get_flt(cmd);
	if (cm_get_model_units_mode() == INCHES) {
		cmd->value *= INCH_PER_MM;
	}
	//	cmd->objtype = TYPE_FLOAT_UNITS;	// ++++ UNTESTED
	return (STAT_OK);
}

static void print_lin(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void print_rot(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[DEGREE_INDEX]));
}

/******* SYSTEM ID AND CONTROL VARIABLES ****************************************************
 * set_hv() - set hardware version number
 * get_id() - get device ID (signature)
 */
static stat_t set_hv(cmdObj_t *cmd) 
{
	if (cmd->value > TINYG_HARDWARE_VERSION_MAX) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	set_flt(cmd);					// record the hardware version
	sys_port_bindings(cmd->value);	// reset port bindings
	gpio_init();					// re-initialize the GPIO ports
	return (STAT_OK);
}

static stat_t get_id(cmdObj_t *cmd) 
{
	char_t tmp[SYS_ID_LEN];
	sys_get_id(tmp);
	cmd->objtype = TYPE_STRING;
	ritorno(cmd_copy_string(cmd, tmp));
	return (STAT_OK);
}

/**** REPORT AND COMMAND FUNCTIONS ********************************************************
 * get_qr() - run a queue report (as data)
 * run_qf() - request a planner buffer flush
 * get_er()	- invoke a bogus exception report for testing purposes (it's not real)
 * get_rx()	- get bytes available in RX buffer
 * set_si()	- set status report interval
 * get_sr()	- run status report
 * set_sr()	- set status report elements
 * print_sr() - print multiline text status report
 * set_md() - disable all motors
 * set_me() - enable motors with $Npm=0
 * run_sx()	- send XOFF, XON
 * set_jv() - set JSON verbosity level (exposed) - for details see jsonVerbosity in config.h
 * get_gc()	- get gcode block
 * run_gc()	- launch the gcode parser on a block of gcode
 * run_home() - invoke a homing cycle
 * run_boot() - request boot loader entry
 */

static stat_t get_qr(cmdObj_t *cmd) 
{
	cmd->value = (float)mp_get_planner_buffers_available();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t run_qf(cmdObj_t *cmd) 
{
	cm_request_queue_flush();
	return (STAT_OK);
}

static stat_t get_er(cmdObj_t *cmd) 
{
	rpt_exception(STAT_INTERNAL_ERROR, 42);	// bogus exception report
	return (STAT_OK);
}

static stat_t get_rx(cmdObj_t *cmd)
{
	cmd->value = (float)xio_get_usb_rx_free();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t set_si(cmdObj_t *cmd)
{
	if (cmd->value < STATUS_REPORT_MIN_MS) { cmd->value = STATUS_REPORT_MIN_MS;}
	cfg.status_report_interval = (uint32_t)cmd->value;
	return(STAT_OK);
}

static stat_t get_sr(cmdObj_t *cmd)
{
	rpt_populate_unfiltered_status_report();
	return (STAT_OK);
}

static stat_t set_sr(cmdObj_t *cmd)
{
	return (rpt_set_status_report(cmd));
}

static void print_sr(cmdObj_t *cmd)
{
	rpt_populate_unfiltered_status_report();
}

static stat_t set_md(cmdObj_t *cmd)
{
	st_disable_motors();
	return (STAT_OK);
}

static stat_t set_me(cmdObj_t *cmd)
{
	st_enable_motors();
	return (STAT_OK);
}

static stat_t run_sx(cmdObj_t *cmd)
{
	xio_putc(XIO_DEV_USB, XOFF);
	xio_putc(XIO_DEV_USB, XON);
	return (STAT_OK);
}

static stat_t set_jv(cmdObj_t *cmd) 
{
	if (cmd->value > JV_VERBOSE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.json_verbosity = cmd->value;

	cfg.echo_json_footer = false;
	cfg.echo_json_messages = false;
	cfg.echo_json_configs = false;
	cfg.echo_json_linenum = false;
	cfg.echo_json_gcode_block = false;

	if (cmd->value >= JV_FOOTER) 	{ cfg.echo_json_footer = true;}
	if (cmd->value >= JV_MESSAGES)	{ cfg.echo_json_messages = true;}
	if (cmd->value >= JV_CONFIGS)	{ cfg.echo_json_configs = true;}
	if (cmd->value >= JV_LINENUM)	{ cfg.echo_json_linenum = true;}
	if (cmd->value >= JV_VERBOSE)	{ cfg.echo_json_gcode_block = true;}

	return(STAT_OK);
}

static stat_t get_gc(cmdObj_t *cmd)
{
	ritorno(cmd_copy_string(cmd, cs.in_buf));
	cmd->objtype = TYPE_STRING;
	return (STAT_OK);
}

static stat_t run_gc(cmdObj_t *cmd)
{
	return(gc_gcode_parser(*cmd->stringp));
}

static stat_t run_home(cmdObj_t *cmd)
{
	if (cmd->value == true) { cm_homing_cycle_start();}
	return (STAT_OK);
}

static stat_t run_boot(cmdObj_t *cmd)
{
	hardware_request_bootloader();
	return(STAT_OK);
}

/**** GCODE MODEL ITEMS ****************************************
 * _get_msg_helper() - helper to get display message
 *
 * get_stat() - get combined machine state as value and string
 * get_macs() - get raw machine state as value and string
 * get_cycs() - get raw cycle state as value and string
 * get_mots() - get raw motion state as value and string
 * get_hold() - get raw hold state as value and string
 * get_home() - get raw homing state as value and string
 * get_unit() - get units mode as integer and display string
 * get_coor() - get goodinate system
 * get_momo() - get runtime motion mode
 * get_plan() - get model gcode plane select
 * get_path() - get model gcode path control mode
 * get_dist() - get model gcode distance mode
 * get_frmo() - get model gcode feed rate mode
 * get_feed() - get feed rate 
 * get_line() - get runtime line number for status reports
 * get_vel()  - get runtime velocity
 * get_ofs()  - get runtime work offset
 * get_pos()  - get runtime work position
 * get_mpos() - get runtime machine position
 * 
 * print_pos()- print work position (with proper units)
 * print_mpos()- print machine position (always mm units)
 * print_coor()- print coordinate offsets with linear units
 * print_corr()- print coordinate offsets with rotary units
 */
static stat_t _get_msg_helper(cmdObj_t *cmd, char_P msg, uint8_t value)
{
	cmd->value = (float)value;
	cmd->objtype = TYPE_INTEGER;
	ritorno(cmd_copy_string_P(cmd, (PGM_P)pgm_read_word(&msg[value*2]))); // hack alert: direct computation of index
	return (STAT_OK);
//	return((char_t *)pgm_read_word(&msg[(uint8_t)value]));
/* ARM code:
	cmd->value = (float)value;
	cmd->objtype = TYPE_INTEGER;
	return (cmd_copy_string(cmd, msg_array[value]));
*/
}

static stat_t get_stat(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_stat, cm_get_combined_state()));

/* how to do this w/o calling the helper routine - See 331.09 for original routines
	cmd->value = cm_get_machine_state();
	cmd->objtype = TYPE_INTEGER;
	ritorno(cmd_copy_string_P(cmd, (PGM_P)pgm_read_word(&msg_stat[(uint8_t)cmd->value]),CMD_STRING_LEN));
	return (STAT_OK);
 */
//	strncpy_P(cmd->string_value,(PGM_P)pgm_read_word(&msg_stat[(uint8_t)cmd->value]),CMD_STRING_LEN);
}

static stat_t get_macs(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_macs, cm_get_machine_state()));
}

static stat_t get_cycs(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_cycs, cm_get_cycle_state()));
}

static stat_t get_mots(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_mots, cm_get_motion_state()));
}

static stat_t get_hold(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_hold, cm_get_hold_state()));
}

static stat_t get_home(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_home, cm_get_homing_state()));
}

static stat_t get_unit(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_unit, cm_get_model_units_mode()));
}

static stat_t get_coor(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_coor, cm_get_model_coord_system()));
}

static stat_t get_momo(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_momo, cm_get_runtime_motion_mode()));
}

static stat_t get_plan(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_plan, cm_get_model_select_plane()));
}

static stat_t get_path(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_path, cm_get_model_path_control()));
}

static stat_t get_dist(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_dist, cm_get_model_distance_mode()));
}

static stat_t get_frmo(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (char_P)msg_frmo, cm_get_model_inverse_feed_rate_mode()));
}

static stat_t get_line(cmdObj_t *cmd)
{
	cmd->value = (float)mp_get_runtime_linenum();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t get_vel(cmdObj_t *cmd) 
{
	cmd->value = mp_get_runtime_velocity();
	if (cm_get_model_units_mode() == INCHES) cmd->value *= INCH_PER_MM;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t get_pos(cmdObj_t *cmd) 
{
	cmd->value = cm_get_runtime_work_position(_get_pos_axis(cmd->index));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t get_mpos(cmdObj_t *cmd) 
{
	cmd->value = cm_get_runtime_machine_position(_get_pos_axis(cmd->index));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t get_ofs(cmdObj_t *cmd) 
{
	cmd->value = cm_get_runtime_work_offset(_get_pos_axis(cmd->index));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static void _print_pos_helper(cmdObj_t *cmd, uint8_t units)
{
	cmd_get(cmd);
	char_t axes[6] = {"XYZABC"};
	char_t format[CMD_FORMAT_LEN+1];
	uint8_t axis = _get_pos_axis(cmd->index);
	if (axis >= AXIS_A) { units = DEGREES;}
	fprintf(stderr, get_format(cmd->index,format), axes[axis], cmd->value, (PGM_P)pgm_read_word(&msg_units[(uint8_t)units]));
}

static void print_pos(cmdObj_t *cmd)		// print position with unit displays for MM or Inches
{
	_print_pos_helper(cmd, cm_get_model_units_mode());
}

static void print_mpos(cmdObj_t *cmd)		// print position with fixed unit display - always in Degrees or MM
{
	_print_pos_helper(cmd, MILLIMETERS);
}

static void print_coor(cmdObj_t *cmd)		// print coordinate offsets with linear units
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->token, cmd->value,
	(PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void print_corr(cmdObj_t *cmd)		// print coordinate offsets with rotary units
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->token, cmd->value,
	(PGM_P)pgm_read_word(&msg_units[DEGREE_INDEX]));
}

/**** AXIS AND MOTOR FUNCTIONS ************************************************
 * _set_motor_steps_per_unit() - update this derived value
 *
 * get_am() - get axis mode w/enumeration string
 * set_am() - set axis mode w/exception handling for axis type
 * get_jrk() - get jerk value w/1,000,000 correction
 * set_jrk() - set jerk value w/1,000,000 correction
 * set_sw() - run this any time you change a switch setting
 *	
 * set_sa() - set motor step_angle & recompute steps_per_unit
 * set_tr() - set motor travel_per_rev & recompute steps_per_unit
 * set_mi() - set microsteps & recompute steps_per_unit
 * set_po() - set polarity and update stepper structs
 * set_pm() - set motor power mode and take action
 *
 * pr_ma_ui8() - print motor or axis uint8 value w/no units or unit conversion
 * pr_ma_lin() - print linear value with units and in/mm unit conversion
 * pr_ma_rot() - print rotary value with units
 * print_am()	- print axis mode with enumeration string
 */

// helper. This function will need to be rethought if microstep morphing is implemented
static stat_t _set_motor_steps_per_unit(cmdObj_t *cmd) 
{
	uint8_t m = _get_motor(cmd->index);
	cfg.m[m].steps_per_unit = (360 / (cfg.m[m].step_angle / cfg.m[m].microsteps) / cfg.m[m].travel_rev);
	return (STAT_OK);
}

static stat_t get_am(cmdObj_t *cmd)
{
	get_ui8(cmd);
	return(_get_msg_helper(cmd, (char_P)msg_am, cmd->value)); // see 331.09 for old method
}

static stat_t set_am(cmdObj_t *cmd)		// axis mode
{
	char_t linear_axes[] = {"xyz"};
	if (strchr(linear_axes, cmd->token[0]) != NULL) { // true if it's a linear axis
		if (cmd->value > AXIS_MAX_LINEAR) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	} else {
		if (cmd->value > AXIS_MAX_ROTARY) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	}
	set_ui8(cmd);
	return(STAT_OK);
}

stat_t get_jrk(cmdObj_t *cmd)
{
	get_flt(cmd);
	if (cm_get_model_units_mode() == INCHES) {
		cmd->value *= (INCH_PER_MM)/1000000;
	} else {
		cmd->value /= 1000000;
	}
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t set_jrk(cmdObj_t *cmd)
{
	if (cm_get_model_units_mode() == INCHES) { cmd->value *= MM_PER_INCH;}
	*((float *)cfgArray[cmd->index].target) = (1000000 * cmd->value);
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->objtype = TYPE_FLOAT_UNITS;
	return(STAT_OK);
}

static stat_t set_sw(cmdObj_t *cmd)			// switch setting
{
	if (cmd->value > SW_MODE_MAX_VALUE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	set_ui8(cmd);
	gpio_init();
	return (STAT_OK);
}

static void print_ss(cmdObj_t *cmd)			// print switch state
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->token, cmd->value);
}

static stat_t set_sa(cmdObj_t *cmd)			// motor step angle
{ 
	set_flt(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (STAT_OK);
}

static stat_t set_tr(cmdObj_t *cmd)			// motor travel per revolution
{ 
	set_flu(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (STAT_OK);
}

static stat_t set_mi(cmdObj_t *cmd)			// motor microsteps
{
	if (fp_NE(cmd->value,1) && fp_NE(cmd->value,2) && fp_NE(cmd->value,4) && fp_NE(cmd->value,8)) {
		cmd_add_message_P(PSTR("*** WARNING *** Setting non-standard microstep value"));
	}
	set_ui8(cmd);							// set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
	st_set_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

static stat_t set_po(cmdObj_t *cmd)			// motor polarity
{ 
	ritorno (set_01(cmd));
	st_set_polarity(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

static stat_t set_pm(cmdObj_t *cmd)			// motor power mode
{ 
	ritorno (set_01(cmd));
	if (fp_ZERO(cmd->value)) {				// zero means enable motor - i.e. disable power management mode
		st_enable_motor(_get_motor(cmd->index));
	} else {
		st_disable_motor(_get_motor(cmd->index));
	}
	return (STAT_OK);
}

static void pr_ma_ui8(cmdObj_t *cmd)		// print uint8_t value
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value);
}

static void pr_ma_lin(cmdObj_t *cmd)		// print a linear value in prevailing units
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->value, 
					(PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void pr_ma_rot(cmdObj_t *cmd)		// print a rotary value in degrees units
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->value,
					(PGM_P)pgm_read_word(&msg_units[DEGREE_INDEX]));
}

static void print_am(cmdObj_t *cmd)			// print axis mode with enumeration string
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value,
					(PGM_P)pgm_read_word(&msg_am[(uint8_t)cmd->value]));
}

/**** COMMUNICATIONS SETTINGS *************************************************
 * set_ic() - ignore CR or LF on RX
 * set_ec() - enable CRLF on TX
 * set_ee() - enable character echo
 * set_ex() - enable XON/XOFF or RTS/CTS flow control
 * set_baud() - set USB baud rate
 *	The above assume USB is the std device
 */
static stat_t _set_comm_helper(cmdObj_t *cmd, uint32_t yes, uint32_t no)
{
	if (fp_NOT_ZERO(cmd->value)) { 
		(void)xio_ctrl(XIO_DEV_USB, yes);
	} else { 
		(void)xio_ctrl(XIO_DEV_USB, no);
	}
	return (STAT_OK);
}

static stat_t set_ic(cmdObj_t *cmd) 				// ignore CR or LF on RX
{
	if (cmd->value > IGNORE_LF) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.ignore_crlf = (uint8_t)cmd->value;
	(void)xio_ctrl(XIO_DEV_USB, XIO_NOIGNORECR);	// clear them both
	(void)xio_ctrl(XIO_DEV_USB, XIO_NOIGNORELF);

	if (cfg.ignore_crlf == IGNORE_CR) {				// $ic=1
		(void)xio_ctrl(XIO_DEV_USB, XIO_IGNORECR);
	} else if (cfg.ignore_crlf == IGNORE_LF) {		// $ic=2
		(void)xio_ctrl(XIO_DEV_USB, XIO_IGNORELF);
	}
	return (STAT_OK);
}

static stat_t set_ec(cmdObj_t *cmd) 				// expand CR to CRLF on TX
{
	if (cmd->value > true) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_cr = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_CRLF, XIO_NOCRLF));
}

static stat_t set_ee(cmdObj_t *cmd) 				// enable character echo
{
	if (cmd->value > true) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_echo = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_ECHO, XIO_NOECHO));
}

static stat_t set_ex(cmdObj_t *cmd)				// enable XON/XOFF or RTS/CTS flow control
{
	if (cmd->value > FLOW_CONTROL_RTS) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_flow_control = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_XOFF, XIO_NOXOFF));
}

/*
 * set_baud() - set USB baud rate
 *
 *	See xio_usart.h for valid values. Works as a callback.
 *	The initial routine changes the baud config setting and sets a flag
 *	Then it posts a user message indicating the new baud rate
 *	Then it waits for the TX buffer to empty (so the message is sent)
 *	Then it performs the callback to apply the new baud rate
 */

static stat_t set_baud(cmdObj_t *cmd)
{
	uint8_t baud = (uint8_t)cmd->value;
	if ((baud < 1) || (baud > 6)) {
		cmd_add_message_P(PSTR("*** WARNING *** Illegal baud rate specified"));
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	cfg.usb_baud_rate = baud;
	cfg.usb_baud_flag = true;
	char_t message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, PSTR("*** NOTICE *** Restting baud rate to %S"),(PGM_P)pgm_read_word(&msg_baud[baud]));
	cmd_add_message(message);
	return (STAT_OK);
}

stat_t set_baud_callback(void)
{
	if (cfg.usb_baud_flag == false) { return(STAT_NOOP);}
	cfg.usb_baud_flag = false;
	xio_set_baud(XIO_DEV_USB, cfg.usb_baud_rate);
	return (STAT_OK);
}

/**** UberGroup Operations ****************************************************
 * Uber groups are groups of groups organized for convenience:
 *	- motors	- group of all motor groups
 *	- axes		- group of all axis groups
 *	- offsets	- group of all offsets and stored positions
 *	- all		- group of all groups
 *
 * _do_group_list()	- get and print all groups in the list (iteration)
 * _do_motors()		- get and print motor uber group 1-4
 * _do_axes()		- get and print axis uber group XYZABC
 * _do_offsets()	- get and print offset uber group G54-G59, G28, G30, G92
 * _do_all()		- get and print all groups uber group
 */

static stat_t _do_group_list(cmdObj_t *cmd, char_t list[][CMD_TOKEN_LEN+1]) // helper to print multiple groups in a list
{
	for (uint8_t i=0; i < CMD_MAX_OBJECTS; i++) {
		if (list[i][0] == NUL) { return (STAT_COMPLETE);}
		cmd_reset_list();
		cmd = cmd_body;
		strncpy(cmd->token, list[i], CMD_TOKEN_LEN);
		cmd->index = cmd_get_index("", cmd->token);
//		cmd->objtype = TYPE_PARENT;
		cmd_get_cmdObj(cmd);
		cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
	}
	return (STAT_COMPLETE);
}

static stat_t _do_motors(cmdObj_t *cmd)	// print parameters for all motor groups
{
	char_t list[][CMD_TOKEN_LEN+1] = {"1","2","3","4",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_axes(cmdObj_t *cmd)	// print parameters for all axis groups
{
	char_t list[][CMD_TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_offsets(cmdObj_t *cmd)	// print offset parameters for G54-G59,G92, G28, G30
{
	char_t list[][CMD_TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92","g28","g30",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_all(cmdObj_t *cmd)	// print all parameters
{
	strcpy(cmd->token,"sys");			// print system group
	get_grp(cmd);
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	_do_motors(cmd);					// print all motor groups
	_do_axes(cmd);						// print all axis groups

	strcpy(cmd->token,"p1");			// print PWM group		
	get_grp(cmd);
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	return (_do_offsets(cmd));			// print all offsets
}
