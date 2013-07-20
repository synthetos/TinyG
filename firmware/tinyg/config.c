/*
 * config.c - configuration handling and persistence; master function table
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
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>			// precursor for xio.h
#include <avr/pgmspace.h>	// precursor for xio.h

#include "tinyg.h"			// config reaches into almost everything
#include "util.h"
#include "config.h"
#include "report.h"
#include "settings.h"
#include "controller.h"
#include "canonical_machine.h"
#include "gcode_parser.h"
#include "json_parser.h"
#include "planner.h"
#include "stepper.h"
#include "gpio.h"
#include "test.h"
#include "help.h"
#include "system.h"
#include "network.h"
#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

typedef char PROGMEM *prog_char_ptr;	// access to PROGMEM arrays of PROGMEM strings

//*** STATIC STUFF ***********************************************************

typedef struct cfgItem {
	char group[CMD_GROUP_LEN+1];		// group prefix (with NUL termination)
	char token[CMD_TOKEN_LEN+1];		// token - stripped of group prefix (w/NUL termination)
	uint8_t flags;						// operations flags - see defines below
	int8_t precision;					// decimal precision for display (JSON)
	const char *format;					// pointer to formatted print string
	fptrPrint print;					// print binding: aka void (*print)(cmdObj_t *cmd);
	fptrCmd get;						// GET binding aka uint8_t (*get)(cmdObj_t *cmd)
	fptrCmd set;						// SET binding aka uint8_t (*set)(cmdObj_t *cmd)
	float *target;						// target for writing config value
	float def_value;					// default value for config item
} cfgItem_t;

// operations flags and shorthand
#define F_INITIALIZE	0x01			// initialize this item (run set during initialization)
#define F_PERSIST 		0x02			// persist this item when set is run
#define F_NOSTRIP		0x04			// do not strip to group from the token
#define _f00			0x00
#define _fin			F_INITIALIZE
#define _fpe			F_PERSIST
#define _fip			(F_INITIALIZE | F_PERSIST)
#define _fns			F_NOSTRIP
#define _f07			(F_INITIALIZE | F_PERSIST | F_NOSTRIP)

// prototypes are divided into generic functions and parameter-specific functions

// generic internal functions
static stat_t _set_nul(cmdObj_t *cmd);	// noop
static stat_t _set_ui8(cmdObj_t *cmd);	// set a uint8 value
static stat_t _set_01(cmdObj_t *cmd);	// set a 0 or 1 value w/validation
static stat_t _set_012(cmdObj_t *cmd);	// set a 0, 1 or 2 value w/validation
static stat_t _set_0123(cmdObj_t *cmd);	// set a 0, 1, 2 or 3 value w/validation
static stat_t _set_int(cmdObj_t *cmd);	// set a uint32 integer value
static stat_t _set_dbl(cmdObj_t *cmd);	// set a float value
static stat_t _set_dbu(cmdObj_t *cmd);	// set a float with unit conversion

static stat_t _get_nul(cmdObj_t *cmd);	// get null value type
static stat_t _get_ui8(cmdObj_t *cmd);	// get uint8 value
static stat_t _get_int(cmdObj_t *cmd);	// get uint32 integer value
static stat_t _get_dbl(cmdObj_t *cmd);	// get float value
static stat_t _get_dbu(cmdObj_t *cmd);	// get float with unit conversion

static void _print_nul(cmdObj_t *cmd);	// print nothing
static void _print_str(cmdObj_t *cmd);	// print a string value
static void _print_ui8(cmdObj_t *cmd);	// print unit8_t value w/no units
static void _print_int(cmdObj_t *cmd);	// print integer value w/no units
static void _print_dbl(cmdObj_t *cmd);	// print float value w/no units
static void _print_lin(cmdObj_t *cmd);	// print linear values
static void _print_rot(cmdObj_t *cmd);	// print rotary values

//static void _pr_ma_str(cmdObj_t *cmd); // generic print functions for motors and axes
static void _pr_ma_ui8(cmdObj_t *cmd);
//static void _pr_ma_int(cmdObj_t *cmd); // placeholder
//static void _pr_ma_dbl(cmdObj_t *cmd); // placeholder
static void _pr_ma_lin(cmdObj_t *cmd);
static void _pr_ma_rot(cmdObj_t *cmd);
static void _print_coor(cmdObj_t *cmd);	// print coordinate offsets with linear units
static void _print_corr(cmdObj_t *cmd);	// print coordinate offsets with rotary units

// helpers for generic functions
static char *_get_format(const index_t i, char *format);
static int8_t _get_motor(const index_t i);
//static int8_t _get_axis(const index_t i);
static int8_t _get_pos_axis(const index_t i);
static stat_t _text_parser(char *str, cmdObj_t *c);
static stat_t _get_msg_helper(cmdObj_t *cmd, prog_char_ptr msg, uint8_t value);
static void _print_text_inline_pairs();
static void _print_text_inline_values();
static void _print_text_multiline_formatted();

static stat_t _set_grp(cmdObj_t *cmd);	// set data for a group
static stat_t _get_grp(cmdObj_t *cmd);	// get data for a group
static stat_t _do_motors(cmdObj_t *cmd);	// print parameters for all motor groups
static stat_t _do_axes(cmdObj_t *cmd);	// print parameters for all axis groups
static stat_t _do_offsets(cmdObj_t *cmd);// print offsets for G54-G59, G92
static stat_t _do_all(cmdObj_t *cmd);	// print all parameters

/*****************************************************************************
 **** PARAMETER-SPECIFIC CODE REGION *****************************************
 **** This code and data will change as you add / update config parameters ***
 *****************************************************************************/

// parameter-specific internal functions
static stat_t _set_hv(cmdObj_t *cmd);		// set hardware version
static stat_t _get_sr(cmdObj_t *cmd);		// run status report (as data)
static void _print_sr(cmdObj_t *cmd);		// run status report (as printout)
static stat_t _set_sr(cmdObj_t *cmd);		// set status report specification
static stat_t _set_si(cmdObj_t *cmd);		// set status report interval
static stat_t _run_boot(cmdObj_t *cmd);	// jump to the bootloader
static stat_t _get_id(cmdObj_t *cmd);		// get device ID
static stat_t _set_jv(cmdObj_t *cmd);		// set JSON verbosity
static stat_t _get_qr(cmdObj_t *cmd);		// get a queue report (as data)
static stat_t _run_qf(cmdObj_t *cmd);		// execute a queue flush block
static stat_t _get_er(cmdObj_t *cmd);		// invoke a bogus exception report for testing purposes
static stat_t _get_rx(cmdObj_t *cmd);		// get bytes in RX buffer
static stat_t _set_md(cmdObj_t *cmd);		// disable all motors
static stat_t _set_me(cmdObj_t *cmd);		// enable motors with power-mode set to 0 (on)

static stat_t _get_gc(cmdObj_t *cmd);		// get current gcode block
static stat_t _run_gc(cmdObj_t *cmd);		// run a gcode block
static stat_t _run_home(cmdObj_t *cmd);	// invoke a homing cycle

static stat_t _get_line(cmdObj_t *cmd);	// get runtime line number
static stat_t _get_stat(cmdObj_t *cmd);	// get combined machine state as value and string
static stat_t _get_macs(cmdObj_t *cmd);	// get raw machine state as value and string
static stat_t _get_cycs(cmdObj_t *cmd);	// get raw cycle state (etc etc)...
static stat_t _get_mots(cmdObj_t *cmd);	// get raw motion state...
static stat_t _get_hold(cmdObj_t *cmd);	// get raw hold state...
static stat_t _get_home(cmdObj_t *cmd);	// get raw homing state...
static stat_t _get_unit(cmdObj_t *cmd);	// get unit mode...
static stat_t _get_coor(cmdObj_t *cmd);	// get coordinate system in effect...
static stat_t _get_momo(cmdObj_t *cmd);	// get motion mode...
static stat_t _get_plan(cmdObj_t *cmd);	// get active plane...
static stat_t _get_path(cmdObj_t *cmd);	// get patch control mode...
static stat_t _get_dist(cmdObj_t *cmd);	// get distance mode...
static stat_t _get_frmo(cmdObj_t *cmd);	// get feedrate mode...
static stat_t _get_vel(cmdObj_t *cmd);		// get runtime velocity...
static stat_t _get_pos(cmdObj_t *cmd);		// get runtime work position...
static stat_t _get_mpos(cmdObj_t *cmd);	// get runtime machine position...
static stat_t _get_ofs(cmdObj_t *cmd);		// get runtime work offset...
static void _print_pos(cmdObj_t *cmd);		// print runtime work position in prevailing units
static void _print_mpos(cmdObj_t *cmd);		// print runtime work position always in MM uints

static stat_t _set_defa(cmdObj_t *cmd);	// reset config to default values

static stat_t _set_sa(cmdObj_t *cmd);		// set motor step angle
static stat_t _set_tr(cmdObj_t *cmd);		// set motor travel per revolution
static stat_t _set_mi(cmdObj_t *cmd);		// set microsteps
static stat_t _set_po(cmdObj_t *cmd);		// set motor polarity
static stat_t _set_pm(cmdObj_t *cmd);		// set motor power mode

static stat_t _set_sw(cmdObj_t *cmd);		// must run any time you change a switch setting
static stat_t _get_am(cmdObj_t *cmd);		// get axis mode
static stat_t _set_am(cmdObj_t *cmd);		// set axis mode
static void _print_am(cmdObj_t *cmd);		// print axis mode

static stat_t _set_ic(cmdObj_t *cmd);		// ignore CR or LF on RX input
static stat_t _set_ec(cmdObj_t *cmd);		// expand CRLF on TX outout
static stat_t _set_ee(cmdObj_t *cmd);		// enable character echo
static stat_t _set_ex(cmdObj_t *cmd);		// enable XON/XOFF and RTS/CTS flow control
static stat_t _set_baud(cmdObj_t *cmd);	// set USB baud rate

/***** PROGMEM Strings ******************************************************/

/* strings used by formatted print functions */

static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
static const char msg_units1[] PROGMEM = " mm";
static const char msg_units2[] PROGMEM = " deg";
static PGM_P const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
#define F_DEG 2

static const char msg_g20[] PROGMEM = "G20 - inches mode";
static const char msg_g21[] PROGMEM = "G21 - millimeter mode";
static PGM_P const msg_unit[] PROGMEM = { msg_g20, msg_g21 };

static const char msg_stat0[] PROGMEM = "Initializing";	// combined state (stat) uses this array
static const char msg_stat1[] PROGMEM = "Ready";
static const char msg_stat2[] PROGMEM = "Shutdown";
static const char msg_stat3[] PROGMEM = "Stop";
static const char msg_stat4[] PROGMEM = "End";
static const char msg_stat5[] PROGMEM = "Run";
static const char msg_stat6[] PROGMEM = "Hold";
static const char msg_stat7[] PROGMEM = "Probe";
static const char msg_stat8[] PROGMEM = "Cycle";
static const char msg_stat9[] PROGMEM = "Homing";
static const char msg_stat10[] PROGMEM = "Jog";
static PGM_P const msg_stat[] PROGMEM = { msg_stat0, msg_stat1, msg_stat2, msg_stat3, msg_stat4, msg_stat5, 
										  msg_stat6, msg_stat7, msg_stat8, msg_stat9, msg_stat10};

static const char msg_macs0[] PROGMEM = "Initializing";
static const char msg_macs1[] PROGMEM = "Reset";
static const char msg_macs2[] PROGMEM = "Cycle";
static const char msg_macs3[] PROGMEM = "Stop";
static const char msg_macs4[] PROGMEM = "End";
static PGM_P const msg_macs[] PROGMEM = { msg_macs0, msg_macs1, msg_macs2, msg_macs3 , msg_macs4};

static const char msg_cycs0[] PROGMEM = "Off";
static const char msg_cycs1[] PROGMEM = "Started";
static const char msg_cycs2[] PROGMEM = "Homing";
static const char msg_cycs3[] PROGMEM = "Probe";
static PGM_P const msg_cycs[] PROGMEM = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3 };

static const char msg_mots0[] PROGMEM = "Stop";
static const char msg_mots1[] PROGMEM = "Run";
static const char msg_mots2[] PROGMEM = "Hold";
static PGM_P const msg_mots[] PROGMEM = { msg_mots0, msg_mots1, msg_mots2 };

static const char msg_hold0[] PROGMEM = "Off";
static const char msg_hold1[] PROGMEM = "Sync";
static const char msg_hold2[] PROGMEM = "Plan";
static const char msg_hold3[] PROGMEM = "Decel";
static const char msg_hold4[] PROGMEM = "Hold";
static PGM_P const msg_hold[] PROGMEM = { msg_hold0, msg_hold1, msg_hold2, msg_hold3, msg_hold4 };

static const char msg_home0[] PROGMEM = "Not Homed";
static const char msg_home1[] PROGMEM = "Homed";
static PGM_P const msg_home[] PROGMEM = { msg_home0, msg_home1 };

static const char msg_baud0[] PROGMEM = "0";
static const char msg_baud1[] PROGMEM = "9600";
static const char msg_baud2[] PROGMEM = "19200";
static const char msg_baud3[] PROGMEM = "38400";
static const char msg_baud4[] PROGMEM = "57600";
static const char msg_baud5[] PROGMEM = "115200";
static const char msg_baud6[] PROGMEM = "230400";
static PGM_P const msg_baud[] PROGMEM = { msg_baud0, msg_baud1, msg_baud2, msg_baud3, msg_baud4, msg_baud5, msg_baud6 };

static const char msg_sw0[] PROGMEM = "Disabled";
static const char msg_sw1[] PROGMEM = "NO homing";
static const char msg_sw2[] PROGMEM = "NO homing & limit";
static const char msg_sw3[] PROGMEM = "NC homing";
static const char msg_sw4[] PROGMEM = "NC homing & limit";
static PGM_P const msg_sw[] PROGMEM = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };

static const char msg_g53[] PROGMEM = "G53 - machine coordinate system";
static const char msg_g54[] PROGMEM = "G54 - coordinate system 1";
static const char msg_g55[] PROGMEM = "G55 - coordinate system 2";
static const char msg_g56[] PROGMEM = "G56 - coordinate system 3";
static const char msg_g57[] PROGMEM = "G57 - coordinate system 4";
static const char msg_g58[] PROGMEM = "G58 - coordinate system 5";
static const char msg_g59[] PROGMEM = "G59 - coordinate system 6";
static PGM_P const msg_coor[] PROGMEM = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static const char msg_g00[] PROGMEM = "G0  - linear traverse (seek)";
static const char msg_g01[] PROGMEM = "G1  - linear feed";
static const char msg_g02[] PROGMEM = "G2  - clockwise arc feed";
static const char msg_g03[] PROGMEM = "G3  - counter clockwise arc feed";
static const char msg_g80[] PROGMEM = "G80 - cancel motion mode (none active)";
static PGM_P const msg_momo[] PROGMEM = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static const char msg_g17[] PROGMEM = "G17 - XY plane";
static const char msg_g18[] PROGMEM = "G18 - XZ plane";
static const char msg_g19[] PROGMEM = "G19 - YZ plane";
static PGM_P const msg_plan[] PROGMEM = { msg_g17, msg_g18, msg_g19 };

static const char msg_g61[] PROGMEM = "G61 - exact stop mode";
static const char msg_g6a[] PROGMEM = "G61.1 - exact path mode";
static const char msg_g64[] PROGMEM = "G64 - continuous mode";
static PGM_P const msg_path[] PROGMEM = { msg_g61, msg_g61, msg_g64 };

static const char msg_g90[] PROGMEM = "G90 - absolute distance mode";
static const char msg_g91[] PROGMEM = "G91 - incremental distance mode";
static PGM_P const msg_dist[] PROGMEM = { msg_g90, msg_g91 };

static const char msg_g94[] PROGMEM = "G94 - units-per-minute mode (i.e. feedrate mode)";
static const char msg_g93[] PROGMEM = "G93 - inverse time mode";
static PGM_P const msg_frmo[] PROGMEM = { msg_g94, msg_g93 };

static const char msg_am00[] PROGMEM = "[disabled]";
static const char msg_am01[] PROGMEM = "[standard]";
static const char msg_am02[] PROGMEM = "[inhibited]";
static const char msg_am03[] PROGMEM = "[radius]";
static PGM_P const msg_am[] PROGMEM = {
	msg_am00, msg_am01, msg_am02, msg_am03
};

/* PROGMEM strings for print formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */
static const char fmt_nul[] PROGMEM = "";
static const char fmt_ui8[] PROGMEM = "%d\n";	// generic format for ui8s
static const char fmt_dbl[] PROGMEM = "%f\n";	// generic format for floats
static const char fmt_str[] PROGMEM = "%s\n";	// generic format for string message (with no formatting)

// System group and ungrouped formatting strings
static const char fmt_fb[] PROGMEM = "[fb]  firmware build%18.2f\n";
static const char fmt_fv[] PROGMEM = "[fv]  firmware version%16.2f\n";
static const char fmt_hv[] PROGMEM = "[hv]  hardware version%16.2f\n";
static const char fmt_id[] PROGMEM = "[id]  TinyG ID%30s\n";
static const char fmt_ja[] PROGMEM = "[ja]  junction acceleration%8.0f%S\n";
static const char fmt_ml[] PROGMEM = "[ml]  min line segment%17.3f%S\n";
static const char fmt_ma[] PROGMEM = "[ma]  min arc segment%18.3f%S\n";
static const char fmt_ct[] PROGMEM = "[ct]  chordal tolerance%16.3f%S\n";
static const char fmt_ms[] PROGMEM = "[ms]  min segment time%13.0f uSec\n";
static const char fmt_st[] PROGMEM = "[st]  switch type%18d [0=NO,1=NC]\n";
static const char fmt_si[] PROGMEM = "[si]  status interval%14.0f ms\n";
static const char fmt_ic[] PROGMEM = "[ic]  ignore CR or LF on RX%8d [0=off,1=CR,2=LF]\n";
static const char fmt_ec[] PROGMEM = "[ec]  expand LF to CRLF on TX%6d [0=off,1=on]\n";
static const char fmt_ee[] PROGMEM = "[ee]  enable echo%18d [0=off,1=on]\n";
static const char fmt_ex[] PROGMEM = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";
static const char fmt_fs[] PROGMEM = "[fs]  footer style%17d [0=old,1]\n";
static const char fmt_ej[] PROGMEM = "[ej]  enable json mode%13d [0=text,1=JSON]\n";
static const char fmt_jv[] PROGMEM = "[jv]  json verbosity%15d [0=silent,1=footer,2=messages,3=configs,4=linenum,5=verbose]\n";
static const char fmt_tv[] PROGMEM = "[tv]  text verbosity%15d [0=silent,1=verbose]\n";
static const char fmt_sv[] PROGMEM = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";
static const char fmt_qv[] PROGMEM = "[qv]  queue report verbosity%7d [0=off,1=filtered,2=verbose]\n";
static const char fmt_baud[] PROGMEM = "[baud] USB baud rate%15d [1=9600,2=19200,3=38400,4=57600,5=115200,6=230400]\n";
static const char fmt_net[] PROGMEM = "[net]  network mode%16d [0=master]\n";

static const char fmt_qr[] PROGMEM = "qr:%d\n";
static const char fmt_rx[] PROGMEM = "rx:%d\n";

static const char fmt_md[] PROGMEM = "motors disabled\n";
static const char fmt_me[] PROGMEM = "motors enabled\n";
static const char fmt_mt[] PROGMEM = "[mt]  motor disble timeout%9d Sec\n";

// Gcode model values for reporting purposes
static const char fmt_vel[]  PROGMEM = "Velocity:%17.3f%S/min\n";
static const char fmt_line[] PROGMEM = "Line number:%10.0f\n";
static const char fmt_feed[] PROGMEM = "Feed rate:%16.3f%S/min\n";
static const char fmt_stat[] PROGMEM = "Machine state:       %s\n"; // combined machine state
static const char fmt_macs[] PROGMEM = "Raw machine state:   %s\n"; // raw machine state
static const char fmt_cycs[] PROGMEM = "Cycle state:         %s\n";
static const char fmt_mots[] PROGMEM = "Motion state:        %s\n";
static const char fmt_hold[] PROGMEM = "Feedhold state:      %s\n";
static const char fmt_home[] PROGMEM = "Machine homing stat: %s\n";
static const char fmt_unit[] PROGMEM = "Units:               %s\n"; // units mode as ASCII string
static const char fmt_coor[] PROGMEM = "Coordinate system:   %s\n";
static const char fmt_momo[] PROGMEM = "Motion mode:         %s\n";
static const char fmt_plan[] PROGMEM = "Plane:               %s\n";
static const char fmt_path[] PROGMEM = "Path Mode:           %s\n";
static const char fmt_dist[] PROGMEM = "Distance mode:       %s\n";
static const char fmt_frmo[] PROGMEM = "Feed rate mode:      %s\n";

static const char fmt_pos[]  PROGMEM = "%c position:%15.3f%S\n";
static const char fmt_mpos[] PROGMEM = "%c machine posn:%11.3f%S\n";
static const char fmt_ofs[]  PROGMEM = "%c work offset:%12.3f%S\n";
static const char fmt_hom[]  PROGMEM = "%c axis homing state:%2.0f\n";

// Motor print formatting strings
static const char fmt_0ma[] PROGMEM = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] PROGMEM = "[%s%s] m%s step angle%20.3f%S\n";
static const char fmt_0tr[] PROGMEM = "[%s%s] m%s travel per revolution%9.3f%S\n";
static const char fmt_0mi[] PROGMEM = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
static const char fmt_0po[] PROGMEM = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0pm[] PROGMEM = "[%s%s] m%s power management%10d [0=remain powered,1=shut off when idle]\n";

// Axis print formatting strings
static const char fmt_Xam[] PROGMEM = "[%s%s] %s axis mode%18d %S\n";
static const char fmt_Xfr[] PROGMEM = "[%s%s] %s feedrate maximum%15.3f%S/min\n";
static const char fmt_Xvm[] PROGMEM = "[%s%s] %s velocity maximum%15.3f%S/min\n";
static const char fmt_Xtm[] PROGMEM = "[%s%s] %s travel maximum%17.3f%S\n";
static const char fmt_Xjm[] PROGMEM = "[%s%s] %s jerk maximum%15.0f%S/min^3\n";
static const char fmt_Xjd[] PROGMEM = "[%s%s] %s junction deviation%14.4f%S (larger is faster)\n";
static const char fmt_Xra[] PROGMEM = "[%s%s] %s radius value%20.4f%S\n";
static const char fmt_Xsn[] PROGMEM = "[%s%s] %s switch min%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char fmt_Xsx[] PROGMEM = "[%s%s] %s switch max%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
static const char fmt_Xsv[] PROGMEM = "[%s%s] %s search velocity%16.3f%S/min\n";
static const char fmt_Xlv[] PROGMEM = "[%s%s] %s latch velocity%17.3f%S/min\n";
static const char fmt_Xlb[] PROGMEM = "[%s%s] %s latch backoff%18.3f%S\n";
static const char fmt_Xzb[] PROGMEM = "[%s%s] %s zero backoff%19.3f%S\n";
static const char fmt_Xjh[] PROGMEM = "[%s%s] %s jerk homing%16.0f%S/min^3\n";

// PWM strings
static const char fmt_p1frq[] PROGMEM = "[p1frq] pwm frequency   %15.3f Hz\n";
static const char fmt_p1csl[] PROGMEM = "[p1csl] pwm cw speed lo %15.3f RPM\n";
static const char fmt_p1csh[] PROGMEM = "[p1csh] pwm cw speed hi %15.3f RPM\n";
static const char fmt_p1cpl[] PROGMEM = "[p1cpl] pwm cw phase lo %15.3f [0..1]\n";
static const char fmt_p1cph[] PROGMEM = "[p1cph] pwm cw phase hi %15.3f [0..1]\n";
static const char fmt_p1wsl[] PROGMEM = "[p1wsl] pwm ccw speed lo%15.3f RPM\n";
static const char fmt_p1wsh[] PROGMEM = "[p1wsh] pwm ccw speed hi%15.3f RPM\n";
static const char fmt_p1wpl[] PROGMEM = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] PROGMEM = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] PROGMEM = "[p1pof] pwm phase off   %15.3f [0..1]\n";

// Coordinate system offset print formatting strings
static const char fmt_cofs[] PROGMEM = "[%s%s] %s %s offset%20.3f%S\n";
static const char fmt_cloc[] PROGMEM = "[%s%s] %s %s location%18.3f%S\n";

// Gcode model power-on reset default values
static const char fmt_gpl[] PROGMEM = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
static const char fmt_gun[] PROGMEM = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
static const char fmt_gco[] PROGMEM = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
static const char fmt_gpa[] PROGMEM = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
static const char fmt_gdi[] PROGMEM = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

/***** PROGMEM config array **************************************************
 *
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

const cfgItem_t cfgArray[] PROGMEM = {
	// grp  token flags p, format*, print_func, get_func, set_func  target for get/set,   	default value
	{ "sys","fb", _f07, 2, fmt_fb, _print_dbl, _get_dbl, _set_nul, (float *)&tg.fw_build,   TINYG_FIRMWARE_BUILD }, // MUST BE FIRST!
	{ "sys","fv", _f07, 3, fmt_fv, _print_dbl, _get_dbl, _set_nul, (float *)&tg.fw_version, TINYG_FIRMWARE_VERSION },
	{ "sys","hv", _f07, 0, fmt_hv, _print_dbl, _get_dbl, _set_hv,  (float *)&tg.hw_version, TINYG_HARDWARE_VERSION },
	{ "sys","id", _fns, 0, fmt_id, _print_str, _get_id,  _set_nul, (float *)&tg.null, 0 },		// device ID (ASCII signature)

	// dynamic model attributes for reporting purposes (up front for speed)
	{ "",   "n",   _fin, 0, fmt_line,_print_int, _get_int, _set_int,(float *)&gm.linenum,0 },// Gcode line number - gets model line number
	{ "",   "line",_fin, 0, fmt_line,_print_int, _get_line,_set_int,(float *)&gm.linenum,0 },// Gcode line number - gets runtime line number
	{ "",   "feed",_f00, 2, fmt_feed,_print_lin, _get_dbu, _set_nul,(float *)&tg.null, 0 },	// feed rate
	{ "",   "stat",_f00, 0, fmt_stat,_print_str, _get_stat,_set_nul,(float *)&tg.null, 0 },	// combined machine state
	{ "",   "macs",_f00, 0, fmt_macs,_print_str, _get_macs,_set_nul,(float *)&tg.null, 0 },	// raw machine state
	{ "",   "cycs",_f00, 0, fmt_cycs,_print_str, _get_cycs,_set_nul,(float *)&tg.null, 0 },	// cycle state
	{ "",   "mots",_f00, 0, fmt_mots,_print_str, _get_mots,_set_nul,(float *)&tg.null, 0 },	// motion state
	{ "",   "hold",_f00, 0, fmt_hold,_print_str, _get_hold,_set_nul,(float *)&tg.null, 0 },	// feedhold state
	{ "",   "vel", _f00, 2, fmt_vel, _print_lin, _get_vel, _set_nul,(float *)&tg.null, 0 },	// current velocity
	{ "",   "unit",_f00, 0, fmt_unit,_print_str, _get_unit,_set_nul,(float *)&tg.null, 0 },	// units mode
	{ "",   "coor",_f00, 0, fmt_coor,_print_str, _get_coor,_set_nul,(float *)&tg.null, 0 },	// coordinate system
	{ "",   "momo",_f00, 0, fmt_momo,_print_str, _get_momo,_set_nul,(float *)&tg.null, 0 },	// motion mode
	{ "",   "plan",_f00, 0, fmt_plan,_print_str, _get_plan,_set_nul,(float *)&tg.null, 0 },	// plane select
	{ "",   "path",_f00, 0, fmt_path,_print_str, _get_path,_set_nul,(float *)&tg.null, 0 },	// path control mode
	{ "",   "dist",_f00, 0, fmt_dist,_print_str, _get_dist,_set_nul,(float *)&tg.null, 0 },	// distance mode
	{ "",   "frmo",_f00, 0, fmt_frmo,_print_str, _get_frmo,_set_nul,(float *)&tg.null, 0 },	// feed rate mode

	{ "mpo","mpox",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// X machine position
	{ "mpo","mpoy",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// Y machine position
	{ "mpo","mpoz",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// Z machine position
	{ "mpo","mpoa",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// A machine position
	{ "mpo","mpob",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// B machine position
	{ "mpo","mpoc",_f00, 3, fmt_mpos,_print_mpos, _get_mpos,_set_nul,(float *)&tg.null, 0 },// C machine position

	{ "pos","posx",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// X work position
	{ "pos","posy",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// Y work position
	{ "pos","posz",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// Z work position
	{ "pos","posa",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// A work position
	{ "pos","posb",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// B work position
	{ "pos","posc",_f00, 3, fmt_pos, _print_pos, _get_pos, _set_nul,(float *)&tg.null, 0 },	// C work position

	{ "ofs","ofsx",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// X work offset
	{ "ofs","ofsy",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// Y work offset
	{ "ofs","ofsz",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// Z work offset
	{ "ofs","ofsa",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// A work offset 
	{ "ofs","ofsb",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// B work offset 
	{ "ofs","ofsc",_f00, 3, fmt_ofs, _print_mpos, _get_ofs, _set_nul,(float *)&tg.null, 0 },// C work offset

	{ "hom","home",_f00, 0, fmt_home,_print_str, _get_home,_run_home,(float *)&tg.null, 0 },	   // homing state, invoke homing cycle
	{ "hom","homx",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_X], false },// X homed - Homing status group
	{ "hom","homy",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_Y], false },// Y homed
	{ "hom","homz",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_Z], false },// Z homed
	{ "hom","homa",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_A], false },// A homed
	{ "hom","homb",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_B], false },// B homed
	{ "hom","homc",_f00, 0, fmt_hom, _print_pos, _get_ui8, _set_nul,(float *)&cm.homed[AXIS_C], false },// C homed

	// Reports, tests, help, and messages
	{ "", "sr",  _f00, 0, fmt_nul, _print_sr,  _get_sr,  _set_sr , (float *)&tg.null, 0 },	// status report object
	{ "", "qr",  _f00, 0, fmt_qr,  _print_int, _get_qr,  _set_nul, (float *)&tg.null, 0 },	// queue report setting
	{ "", "qf",  _f00, 0, fmt_nul, _print_nul, _get_nul, _run_qf,  (float *)&tg.null, 0 },	// queue flush
	{ "", "er",  _f00, 0, fmt_nul, _print_nul, _get_er,  _set_nul, (float *)&tg.null, 0 },	// invoke bogus exception report for testing
	{ "", "rx",  _f00, 0, fmt_rx,  _print_int, _get_rx,  _set_nul, (float *)&tg.null, 0 },	// space in RX buffer
	{ "", "msg", _f00, 0, fmt_str, _print_str, _get_nul, _set_nul, (float *)&tg.null, 0 },	// string for generic messages
	{ "", "test",_f00, 0, fmt_nul, _print_nul, print_test_help, tg_test, (float *)&tg.test,0 },// prints test help screen
	{ "", "defa",_f00, 0, fmt_nul, _print_nul, print_defaults_help,_set_defa,(float *)&tg.null,0},// prints defaults help screen
	{ "", "boot",_f00, 0, fmt_nul, _print_nul, print_boot_loader_help,_run_boot,(float *)&tg.null,0 },
	{ "", "help",_f00, 0, fmt_nul, _print_nul, print_config_help,_set_nul, (float *)&tg.null,0 },// prints config help screen
	{ "", "h",   _f00, 0, fmt_nul, _print_nul, print_config_help,_set_nul, (float *)&tg.null,0 },// alias for "help"


	// Motor parameters
	{ "1","1ma",_fip, 0, fmt_0ma, _pr_ma_ui8, _get_ui8, _set_ui8,(float *)&cfg.m[MOTOR_1].motor_map,	M1_MOTOR_MAP },
	{ "1","1sa",_fip, 2, fmt_0sa, _pr_ma_rot, _get_dbl ,_set_sa, (float *)&cfg.m[MOTOR_1].step_angle,	M1_STEP_ANGLE },
	{ "1","1tr",_fip, 3, fmt_0tr, _pr_ma_lin, _get_dbu ,_set_tr, (float *)&cfg.m[MOTOR_1].travel_rev,	M1_TRAVEL_PER_REV },
	{ "1","1mi",_fip, 0, fmt_0mi, _pr_ma_ui8, _get_ui8, _set_mi, (float *)&cfg.m[MOTOR_1].microsteps,	M1_MICROSTEPS },
	{ "1","1po",_fip, 0, fmt_0po, _pr_ma_ui8, _get_ui8, _set_po, (float *)&cfg.m[MOTOR_1].polarity,		M1_POLARITY },
	{ "1","1pm",_fip, 0, fmt_0pm, _pr_ma_ui8, _get_ui8, _set_pm, (float *)&cfg.m[MOTOR_1].power_mode,	M1_POWER_MODE },

	{ "2","2ma",_fip, 0, fmt_0ma, _pr_ma_ui8, _get_ui8, _set_ui8,(float *)&cfg.m[MOTOR_2].motor_map,	M2_MOTOR_MAP },
	{ "2","2sa",_fip, 2, fmt_0sa, _pr_ma_rot, _get_dbl, _set_sa, (float *)&cfg.m[MOTOR_2].step_angle,	M2_STEP_ANGLE },
	{ "2","2tr",_fip, 3, fmt_0tr, _pr_ma_lin, _get_dbu, _set_tr, (float *)&cfg.m[MOTOR_2].travel_rev,	M2_TRAVEL_PER_REV },
	{ "2","2mi",_fip, 0, fmt_0mi, _pr_ma_ui8, _get_ui8, _set_mi, (float *)&cfg.m[MOTOR_2].microsteps,	M2_MICROSTEPS },
	{ "2","2po",_fip, 0, fmt_0po, _pr_ma_ui8, _get_ui8, _set_po, (float *)&cfg.m[MOTOR_2].polarity,		M2_POLARITY },
	{ "2","2pm",_fip, 0, fmt_0pm, _pr_ma_ui8, _get_ui8, _set_pm, (float *)&cfg.m[MOTOR_2].power_mode,	M2_POWER_MODE },

	{ "3","3ma",_fip, 0, fmt_0ma, _pr_ma_ui8, _get_ui8, _set_ui8,(float *)&cfg.m[MOTOR_3].motor_map,	M3_MOTOR_MAP },
	{ "3","3sa",_fip, 2, fmt_0sa, _pr_ma_rot, _get_dbl, _set_sa, (float *)&cfg.m[MOTOR_3].step_angle,	M3_STEP_ANGLE },
	{ "3","3tr",_fip, 3, fmt_0tr, _pr_ma_lin, _get_dbu, _set_tr, (float *)&cfg.m[MOTOR_3].travel_rev,	M3_TRAVEL_PER_REV },
	{ "3","3mi",_fip, 0, fmt_0mi, _pr_ma_ui8, _get_ui8, _set_mi, (float *)&cfg.m[MOTOR_3].microsteps,	M3_MICROSTEPS },
	{ "3","3po",_fip, 0, fmt_0po, _pr_ma_ui8, _get_ui8, _set_po, (float *)&cfg.m[MOTOR_3].polarity,		M3_POLARITY },
	{ "3","3pm",_fip, 0, fmt_0pm, _pr_ma_ui8, _get_ui8, _set_pm, (float *)&cfg.m[MOTOR_3].power_mode,	M3_POWER_MODE },

	{ "4","4ma",_fip, 0, fmt_0ma, _pr_ma_ui8, _get_ui8, _set_ui8,(float *)&cfg.m[MOTOR_4].motor_map,	M4_MOTOR_MAP },
	{ "4","4sa",_fip, 2, fmt_0sa, _pr_ma_rot, _get_dbl, _set_sa, (float *)&cfg.m[MOTOR_4].step_angle,	M4_STEP_ANGLE },
	{ "4","4tr",_fip, 3, fmt_0tr, _pr_ma_lin, _get_dbu, _set_tr, (float *)&cfg.m[MOTOR_4].travel_rev,	M4_TRAVEL_PER_REV },
	{ "4","4mi",_fip, 0, fmt_0mi, _pr_ma_ui8, _get_ui8, _set_mi, (float *)&cfg.m[MOTOR_4].microsteps,	M4_MICROSTEPS },
	{ "4","4po",_fip, 0, fmt_0po, _pr_ma_ui8, _get_ui8, _set_po, (float *)&cfg.m[MOTOR_4].polarity,		M4_POLARITY },
	{ "4","4pm",_fip, 0, fmt_0pm, _pr_ma_ui8, _get_ui8, _set_pm, (float *)&cfg.m[MOTOR_4].power_mode,	M4_POWER_MODE },

	// Axis parameters
	{ "x","xam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_X].axis_mode,		X_AXIS_MODE },
	{ "x","xvm",_fip, 0, fmt_Xvm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].velocity_max,	X_VELOCITY_MAX },
	{ "x","xfr",_fip, 0, fmt_Xfr, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].feedrate_max,	X_FEEDRATE_MAX },
	{ "x","xtm",_fip, 0, fmt_Xtm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].travel_max,	X_TRAVEL_MAX },
	{ "x","xjm",_fip, 0, fmt_Xjm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].jerk_max,		X_JERK_MAX },
	{ "x","xjh",_fip, 0, fmt_Xjh, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].jerk_homing,	X_JERK_HOMING },
	{ "x","xjd",_fip, 4, fmt_Xjd, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].junction_dev,	X_JUNCTION_DEVIATION },
	{ "x","xsn",_fip, 0, fmt_Xsn, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[0],					X_SWITCH_MODE_MIN },
	{ "x","xsx",_fip, 0, fmt_Xsx, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[1],					X_SWITCH_MODE_MAX },
	{ "x","xsv",_fip, 0, fmt_Xsv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].search_velocity,X_SEARCH_VELOCITY },
	{ "x","xlv",_fip, 0, fmt_Xlv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].latch_velocity,X_LATCH_VELOCITY },
	{ "x","xlb",_fip, 3, fmt_Xlb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].latch_backoff,	X_LATCH_BACKOFF },
	{ "x","xzb",_fip, 3, fmt_Xzb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_X].zero_backoff,	X_ZERO_BACKOFF },

	{ "y","yam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_Y].axis_mode,		Y_AXIS_MODE },
	{ "y","yvm",_fip, 0, fmt_Xvm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].velocity_max,	Y_VELOCITY_MAX },
	{ "y","yfr",_fip, 0, fmt_Xfr, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].feedrate_max,	Y_FEEDRATE_MAX },
	{ "y","ytm",_fip, 0, fmt_Xtm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].travel_max,	Y_TRAVEL_MAX },
	{ "y","yjm",_fip, 0, fmt_Xjm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].jerk_max,		Y_JERK_MAX },
	{ "y","yjh",_fip, 0, fmt_Xjh, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].jerk_homing,	Y_JERK_HOMING },
	{ "y","yjd",_fip, 4, fmt_Xjd, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].junction_dev,	Y_JUNCTION_DEVIATION },
	{ "y","ysn",_fip, 0, fmt_Xsn, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[2],					Y_SWITCH_MODE_MIN },
	{ "y","ysx",_fip, 0, fmt_Xsx, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[3],					Y_SWITCH_MODE_MAX },
	{ "y","ysv",_fip, 0, fmt_Xsv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].search_velocity,Y_SEARCH_VELOCITY },
	{ "y","ylv",_fip, 0, fmt_Xlv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].latch_velocity,Y_LATCH_VELOCITY },
	{ "y","ylb",_fip, 3, fmt_Xlb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].latch_backoff,	Y_LATCH_BACKOFF },
	{ "y","yzb",_fip, 3, fmt_Xzb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Y].zero_backoff,	Y_ZERO_BACKOFF },

	{ "z","zam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_Z].axis_mode,		Z_AXIS_MODE },
	{ "z","zvm",_fip, 0, fmt_Xvm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].velocity_max,	Z_VELOCITY_MAX },
	{ "z","zfr",_fip, 0, fmt_Xfr, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].feedrate_max,	Z_FEEDRATE_MAX },
	{ "z","ztm",_fip, 0, fmt_Xtm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].travel_max,	Z_TRAVEL_MAX },
	{ "z","zjm",_fip, 0, fmt_Xjm, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].jerk_max,		Z_JERK_MAX },
	{ "z","zjh",_fip, 0, fmt_Xjh, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].jerk_homing,	Z_JERK_HOMING },
	{ "z","zjd",_fip, 4, fmt_Xjd, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].junction_dev,	Z_JUNCTION_DEVIATION },
	{ "z","zsn",_fip, 0, fmt_Xsn, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[4],					Z_SWITCH_MODE_MIN },
	{ "z","zsx",_fip, 0, fmt_Xsx, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[5],					Z_SWITCH_MODE_MAX },
	{ "z","zsv",_fip, 0, fmt_Xsv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].search_velocity,Z_SEARCH_VELOCITY },
	{ "z","zlv",_fip, 0, fmt_Xlv, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].latch_velocity,Z_LATCH_VELOCITY },
	{ "z","zlb",_fip, 3, fmt_Xlb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].latch_backoff,	Z_LATCH_BACKOFF },
	{ "z","zzb",_fip, 3, fmt_Xzb, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_Z].zero_backoff,	Z_ZERO_BACKOFF },

	{ "a","aam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_A].axis_mode,		A_AXIS_MODE },
	{ "a","avm",_fip, 0, fmt_Xvm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].velocity_max,	A_VELOCITY_MAX },
	{ "a","afr",_fip, 0, fmt_Xfr, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].feedrate_max, 	A_FEEDRATE_MAX },
	{ "a","atm",_fip, 0, fmt_Xtm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].travel_max,	A_TRAVEL_MAX },
	{ "a","ajm",_fip, 0, fmt_Xjm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].jerk_max,		A_JERK_MAX },
	{ "a","ajh",_fip, 0, fmt_Xjh, _pr_ma_lin, _get_dbu, _set_dbu,(float *)&cfg.a[AXIS_A].jerk_homing,	A_JERK_HOMING },
	{ "a","ajd",_fip, 4, fmt_Xjd, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].junction_dev,	A_JUNCTION_DEVIATION },
	{ "a","ara",_fip, 3, fmt_Xra, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].radius,		A_RADIUS},
	{ "a","asn",_fip, 0, fmt_Xsn, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[6],					A_SWITCH_MODE_MIN },
	{ "a","asx",_fip, 0, fmt_Xsx, _pr_ma_ui8, _get_ui8, _set_sw, (float *)&sw.mode[7],					A_SWITCH_MODE_MAX },
	{ "a","asv",_fip, 0, fmt_Xsv, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].search_velocity,A_SEARCH_VELOCITY },
	{ "a","alv",_fip, 0, fmt_Xlv, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].latch_velocity,A_LATCH_VELOCITY },
	{ "a","alb",_fip, 3, fmt_Xlb, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].latch_backoff,	A_LATCH_BACKOFF },
	{ "a","azb",_fip, 3, fmt_Xzb, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_A].zero_backoff,	A_ZERO_BACKOFF },

	{ "b","bam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_B].axis_mode,		B_AXIS_MODE },
	{ "b","bvm",_fip, 0, fmt_Xvm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].velocity_max,	B_VELOCITY_MAX },
	{ "b","bfr",_fip, 0, fmt_Xfr, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].feedrate_max, 	B_FEEDRATE_MAX },
	{ "b","btm",_fip, 0, fmt_Xtm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].travel_max,	B_TRAVEL_MAX },
	{ "b","bjm",_fip, 0, fmt_Xjm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].jerk_max,		B_JERK_MAX },
	{ "b","bjd",_fip, 0, fmt_Xjd, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].junction_dev,	B_JUNCTION_DEVIATION },
	{ "b","bra",_fip, 3, fmt_Xra, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_B].radius,		B_RADIUS },

	{ "c","cam",_fip, 0, fmt_Xam, _print_am,  _get_am,  _set_am, (float *)&cfg.a[AXIS_C].axis_mode,		C_AXIS_MODE },
	{ "c","cvm",_fip, 0, fmt_Xvm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].velocity_max,	C_VELOCITY_MAX },
	{ "c","cfr",_fip, 0, fmt_Xfr, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].feedrate_max,	C_FEEDRATE_MAX },
	{ "c","ctm",_fip, 0, fmt_Xtm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].travel_max,	C_TRAVEL_MAX },
	{ "c","cjm",_fip, 0, fmt_Xjm, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].jerk_max,		C_JERK_MAX },
	{ "c","cjd",_fip, 0, fmt_Xjd, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].junction_dev,	C_JUNCTION_DEVIATION },
	{ "c","cra",_fip, 3, fmt_Xra, _pr_ma_rot, _get_dbl, _set_dbl,(float *)&cfg.a[AXIS_C].radius,		C_RADIUS },

	// PWM settings
    { "p1","p1frq",_fip, 0, fmt_p1frq, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.frequency,		P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, 0, fmt_p1csl, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.cw_speed_lo,		P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, 0, fmt_p1csh, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.cw_speed_hi,		P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, 3, fmt_p1cpl, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.cw_phase_lo,		P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, 3, fmt_p1cph, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.cw_phase_hi,		P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, 0, fmt_p1wsl, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.ccw_speed_lo,		P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, 0, fmt_p1wsh, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.ccw_speed_hi,		P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, 3, fmt_p1wpl, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.ccw_phase_lo,		P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, 3, fmt_p1wph, _print_dbl, _get_dbl, _set_dbl,(float *)&cfg.p.ccw_phase_hi,		P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, 3, fmt_p1pof, _print_rot, _get_dbl, _set_dbl,(float *)&cfg.p.phase_off,		P1_PWM_PHASE_OFF },

	// Coordinate system offsets (G54-G59 and G92)
	{ "g54","g54x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_X],	G54_X_OFFSET },
	{ "g54","g54y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_Y],	G54_Y_OFFSET },
	{ "g54","g54z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_Z],	G54_Z_OFFSET },
	{ "g54","g54a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_A],	G54_A_OFFSET },
	{ "g54","g54b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_B],	G54_B_OFFSET },
	{ "g54","g54c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G54][AXIS_C],	G54_C_OFFSET },

	{ "g55","g55x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_X],	G55_X_OFFSET },
	{ "g55","g55y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_Y],	G55_Y_OFFSET },
	{ "g55","g55z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_Z],	G55_Z_OFFSET },
	{ "g55","g55a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_A],	G55_A_OFFSET },
	{ "g55","g55b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_B],	G55_B_OFFSET },
	{ "g55","g55c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G55][AXIS_C],	G55_C_OFFSET },

	{ "g56","g56x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_X],	G56_X_OFFSET },
	{ "g56","g56y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_Y],	G56_Y_OFFSET },
	{ "g56","g56z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_Z],	G56_Z_OFFSET },
	{ "g56","g56a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_A],	G56_A_OFFSET },
	{ "g56","g56b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_B],	G56_B_OFFSET },
	{ "g56","g56c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G56][AXIS_C],	G56_C_OFFSET },

	{ "g57","g57x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_X],	G57_X_OFFSET },
	{ "g57","g57y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_Y],	G57_Y_OFFSET },
	{ "g57","g57z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_Z],	G57_Z_OFFSET },
	{ "g57","g57a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_A],	G57_A_OFFSET },
	{ "g57","g57b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_B],	G57_B_OFFSET },
	{ "g57","g57c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G57][AXIS_C],	G57_C_OFFSET },

	{ "g58","g58x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_X],	G58_X_OFFSET },
	{ "g58","g58y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_Y],	G58_Y_OFFSET },
	{ "g58","g58z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_Z],	G58_Z_OFFSET },
	{ "g58","g58a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_A],	G58_A_OFFSET },
	{ "g58","g58b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_B],	G58_B_OFFSET },
	{ "g58","g58c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G58][AXIS_C],	G58_C_OFFSET },

	{ "g59","g59x",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_X],	G59_X_OFFSET },
	{ "g59","g59y",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_Y],	G59_Y_OFFSET },
	{ "g59","g59z",_fip, 3, fmt_cofs, _print_coor,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_Z],	G59_Z_OFFSET },
	{ "g59","g59a",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_A],	G59_A_OFFSET },
	{ "g59","g59b",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_B],	G59_B_OFFSET },
	{ "g59","g59c",_fip, 3, fmt_cofs, _print_corr,_get_dbu, _set_dbu,(float *)&cfg.offset[G59][AXIS_C],	G59_C_OFFSET },

	{ "g92","g92x",_fin, 3, fmt_cofs, _print_coor,_get_dbu, _set_nul,(float *)&gm.origin_offset[AXIS_X], 0 },// G92 handled differently
	{ "g92","g92y",_fin, 3, fmt_cofs, _print_coor,_get_dbu, _set_nul,(float *)&gm.origin_offset[AXIS_Y], 0 },
	{ "g92","g92z",_fin, 3, fmt_cofs, _print_coor,_get_dbu, _set_nul,(float *)&gm.origin_offset[AXIS_Z], 0 },
	{ "g92","g92a",_fin, 3, fmt_cofs, _print_corr,_get_dbl, _set_nul,(float *)&gm.origin_offset[AXIS_A], 0 },
	{ "g92","g92b",_fin, 3, fmt_cofs, _print_corr,_get_dbl, _set_nul,(float *)&gm.origin_offset[AXIS_B], 0 },
	{ "g92","g92c",_fin, 3, fmt_cofs, _print_corr,_get_dbl, _set_nul,(float *)&gm.origin_offset[AXIS_C], 0 },

	{ "g28","g28x",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g28_position[AXIS_X], 0 },// g28 handled differently
	{ "g28","g28y",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g28_position[AXIS_Y], 0 },
	{ "g28","g28z",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g28_position[AXIS_Z], 0 },
	{ "g28","g28a",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g28_position[AXIS_A], 0 },
	{ "g28","g28b",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g28_position[AXIS_B], 0 },
	{ "g28","g28c",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g28_position[AXIS_C], 0 },

	{ "g30","g30x",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g30_position[AXIS_X], 0 },// g30 handled differently
	{ "g30","g30y",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g30_position[AXIS_Y], 0 },
	{ "g30","g30z",_fin, 3, fmt_cloc, _print_coor,_get_dbu, _set_nul,(float *)&gm.g30_position[AXIS_Z], 0 },
	{ "g30","g30a",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g30_position[AXIS_A], 0 },
	{ "g30","g30b",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g30_position[AXIS_B], 0 },
	{ "g30","g30c",_fin, 3, fmt_cloc, _print_corr,_get_dbl, _set_nul,(float *)&gm.g30_position[AXIS_C], 0 },

	// System parameters
	{ "sys","ja",  _f07, 0, fmt_ja, _print_lin, _get_dbu, _set_dbu, (float *)&cfg.junction_acceleration,JUNCTION_ACCELERATION },
	{ "sys","ct",  _f07, 4, fmt_ct, _print_lin, _get_dbu, _set_dbu, (float *)&cfg.chordal_tolerance,	CHORDAL_TOLERANCE },
	{ "sys","st",  _f07, 0, fmt_st, _print_ui8, _get_ui8, _set_sw,  (float *)&sw.switch_type,			SWITCH_TYPE },
	{ "sys","mt",  _f07, 0, fmt_mt, _print_int, _get_int, _set_int, (float *)&cfg.motor_disable_timeout,MOTOR_DISABLE_TIMEOUT},
	// Note:"me" must initialize after "mt" so it can use the timeout value
	{ "",   "me",  _fin, 0, fmt_me, _print_str, _set_me,  _set_me,  (float *)&tg.null, 0 },
	{ "",   "md",  _f00, 0, fmt_md, _print_str, _set_md,  _set_md,  (float *)&tg.null, 0 },	// disable all motors
	
	{ "sys","ej",  _f07, 0, fmt_ej, _print_ui8, _get_ui8, _set_01,  (float *)&cfg.comm_mode,			COMM_MODE },
	{ "sys","jv",  _f07, 0, fmt_jv, _print_ui8, _get_ui8, _set_jv,  (float *)&cfg.json_verbosity,		JSON_VERBOSITY },
	{ "sys","tv",  _f07, 0, fmt_tv, _print_ui8, _get_ui8, _set_01,  (float *)&cfg.text_verbosity,		TEXT_VERBOSITY },
	{ "sys","qv",  _f07, 0, fmt_qv, _print_ui8, _get_ui8, _set_0123,(float *)&cfg.queue_report_verbosity,QR_VERBOSITY },
	{ "sys","sv",  _f07, 0, fmt_sv, _print_ui8, _get_ui8, _set_012, (float *)&cfg.status_report_verbosity,SR_VERBOSITY },
	{ "sys","si",  _f07, 0, fmt_si, _print_dbl, _get_int, _set_si,  (float *)&cfg.status_report_interval,STATUS_REPORT_INTERVAL_MS },

	{ "sys","ic",  _f07, 0, fmt_ic, _print_ui8, _get_ui8, _set_ic,  (float *)&cfg.ignore_crlf,			COM_IGNORE_CRLF },
	{ "sys","ec",  _f07, 0, fmt_ec, _print_ui8, _get_ui8, _set_ec,  (float *)&cfg.enable_cr,			COM_EXPAND_CR },
	{ "sys","ee",  _f07, 0, fmt_ee, _print_ui8, _get_ui8, _set_ee,  (float *)&cfg.enable_echo,			COM_ENABLE_ECHO },
	{ "sys","ex",  _f07, 0, fmt_ex, _print_ui8, _get_ui8, _set_ex,  (float *)&cfg.enable_flow_control,	COM_ENABLE_FLOW_CONTROL },
	{ "sys","fs",  _f07, 0, fmt_fs, _print_ui8, _get_ui8, _set_ui8, (float *)&cfg.footer_style,			0 },
	{ "sys","baud",_fns, 0, fmt_baud,_print_ui8,_get_ui8, _set_baud,(float *)&cfg.usb_baud_rate,		XIO_BAUD_115200 },
	{ "sys","net", _fip, 0, fmt_net,_print_ui8, _get_ui8, _set_ui8, (float *)&tg.network_mode,			NETWORK_MODE },

	// NOTE: The ordering within the gcode defaults is important for token resolution
	{ "sys","gpl", _f07, 0, fmt_gpl, _print_ui8, _get_ui8,_set_012, (float *)&cfg.select_plane,			GCODE_DEFAULT_PLANE },
	{ "sys","gun", _f07, 0, fmt_gun, _print_ui8, _get_ui8,_set_01,  (float *)&cfg.units_mode,			GCODE_DEFAULT_UNITS },
	{ "sys","gco", _f07, 0, fmt_gco, _print_ui8, _get_ui8,_set_ui8, (float *)&cfg.coord_system,			GCODE_DEFAULT_COORD_SYSTEM },
	{ "sys","gpa", _f07, 0, fmt_gpa, _print_ui8, _get_ui8,_set_012, (float *)&cfg.path_control,			GCODE_DEFAULT_PATH_CONTROL },
	{ "sys","gdi", _f07, 0, fmt_gdi, _print_ui8, _get_ui8,_set_01,  (float *)&cfg.distance_mode,		GCODE_DEFAULT_DISTANCE_MODE },
	{ "",   "gc",  _f00, 0, fmt_nul, _print_nul, _get_gc, _run_gc,  (float *)&tg.null, 0 }, // gcode block - must be last in this group

	// removed from system group as "hidden" parameters
	{ "",   "ms",  _fip, 0, fmt_ms, _print_lin, _get_dbl, _set_dbl, (float *)&cfg.estd_segment_usec,	NOM_SEGMENT_USEC },
	{ "",   "ml",  _fip, 4, fmt_ml, _print_lin, _get_dbu, _set_dbu, (float *)&cfg.min_segment_len,		MIN_LINE_LENGTH },
	{ "",   "ma",  _fip, 4, fmt_ma, _print_lin, _get_dbu, _set_dbu, (float *)&cfg.arc_segment_len,		ARC_SEGMENT_LENGTH },
	{ "",   "qrh", _fip, 0, fmt_ui8,_print_ui8, _get_ui8, _set_ui8, (float *)&cfg.queue_report_hi_water,QR_HI_WATER },
	{ "",   "qrl", _fip, 0, fmt_ui8,_print_ui8, _get_ui8, _set_ui8, (float *)&cfg.queue_report_lo_water,QR_LO_WATER },
	{ "",   "qrl", _fip, 0, fmt_ui8,_print_ui8, _get_ui8, _set_ui8, (float *)&cfg.queue_report_lo_water,QR_LO_WATER },

	// Persistence for status report - must be in sequence
	// *** Count must agree with CMD_STATUS_REPORT_LEN in config.h ***
	{ "","se00",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[0],0 },
	{ "","se01",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[1],0 },
	{ "","se02",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[2],0 },
	{ "","se03",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[3],0 },
	{ "","se04",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[4],0 },
	{ "","se05",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[5],0 },
	{ "","se06",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[6],0 },
	{ "","se07",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[7],0 },
	{ "","se08",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[8],0 },
	{ "","se09",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[9],0 },
	{ "","se10",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[10],0 },
	{ "","se11",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[11],0 },
	{ "","se12",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[12],0 },
	{ "","se13",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[13],0 },
	{ "","se14",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[14],0 },
	{ "","se15",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[15],0 },
	{ "","se16",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[16],0 },
	{ "","se17",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[17],0 },
	{ "","se18",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[18],0 },
	{ "","se19",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[19],0 },
	{ "","se20",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[20],0 },
	{ "","se21",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[21],0 },
	{ "","se22",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[22],0 },
	{ "","se23",_fpe, 0, fmt_nul, _print_nul, _get_int, _set_int,(float *)&cfg.status_report_list[23],0 },

	// Group lookups - must follow the single-valued entries for proper sub-string matching
	// *** Must agree with CMD_COUNT_GROUPS below ****
	{ "","sys",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// system group
	{ "","p1", _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// PWM 1 group
	{ "","1",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// motor groups
	{ "","2",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","3",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","4",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","x",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// axis groups
	{ "","y",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","z",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","a",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","b",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","c",  _f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g54",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// coord offset groups
	{ "","g55",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g56",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g57",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g58",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g59",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },
	{ "","g92",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// origin offsets
	{ "","g28",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// g28 home position
	{ "","g30",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// g30 home position
	{ "","mpo",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// machine position group
	{ "","pos",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// work position group
	{ "","ofs",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// work offset group
	{ "","hom",_f00, 0, fmt_nul, _print_nul, _get_grp, _set_grp,(float *)&tg.null,0 },	// axis homing state group

	// Uber-group (groups of groups, for text-mode displays only)
	// *** Must agree with CMD_COUNT_UBER_GROUPS below ****
	{ "", "m", _f00, 0, fmt_nul, _print_nul, _do_motors, _set_nul,(float *)&tg.null,0 },
	{ "", "q", _f00, 0, fmt_nul, _print_nul, _do_axes,   _set_nul,(float *)&tg.null,0 },
	{ "", "o", _f00, 0, fmt_nul, _print_nul, _do_offsets,_set_nul,(float *)&tg.null,0 },
	{ "", "$", _f00, 0, fmt_nul, _print_nul, _do_all,    _set_nul,(float *)&tg.null,0 }
};

#define CMD_COUNT_GROUPS 		25		// count of simple groups
#define CMD_COUNT_UBER_GROUPS 	4 		// count of uber-groups

#define CMD_INDEX_MAX (sizeof cfgArray / sizeof(cfgItem_t))
#define CMD_INDEX_END_SINGLES		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS - CMD_STATUS_REPORT_LEN)
#define CMD_INDEX_START_GROUPS		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS)
#define CMD_INDEX_START_UBER_GROUPS (CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS)

#define _index_is_single(i) ((i <= CMD_INDEX_END_SINGLES) ? true : false)	// Evaluators
#define _index_lt_groups(i) ((i <= CMD_INDEX_START_GROUPS) ? true : false)
#define _index_is_group(i) (((i >= CMD_INDEX_START_GROUPS) && (i < CMD_INDEX_START_UBER_GROUPS)) ? true : false)
#define _index_is_uber(i)   ((i >= CMD_INDEX_START_UBER_GROUPS) ? true : false)
#define _index_is_group_or_uber(i) ((i >= CMD_INDEX_START_GROUPS) ? true : false)

uint8_t cmd_index_is_group(index_t index) { return _index_is_group(index);}

/**** SYSTEM VARIABLES: Versions and IDs **************************************
 * _set_hv() - set hardweare version number
 * _get_id() - get device ID (signature)
 */
static stat_t _set_hv(cmdObj_t *cmd) 
{
	if (cmd->value > TINYG_HARDWARE_VERSION_MAX) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	_set_dbl(cmd);					// record the hardware version
	sys_port_bindings(cmd->value);	// reset port bindings
	gpio_init();					// re-initialize the GPIO ports
	return (STAT_OK);
}

static stat_t _get_id(cmdObj_t *cmd) 
{
	char tmp[SYS_ID_LEN];
	sys_get_id(tmp);
	ritorno(cmd_copy_string(cmd, tmp));
	cmd->objtype = TYPE_STRING;
	return (STAT_OK);
}

/**** REPORT FUNCTIONS ********************************************************
 * _set_md() 	- disable all motors
 * _set_me() 	- enable motors with $Npm=0
 * _set_qv() 	- get a queue report verbosity
 * _get_qr() 	- get a queue report (as data)
 * _run_qf() 	- execute a planner buffer flush
 * _get_er()	- invoke a bogus exception report for testing purposes (it's not real)
 * _get_rx()	- get bytes available in RX buffer
 * _get_sr()	- run status report
 * _set_sr()	- set status report elements
 * _print_sr()	- print multiline text status report
 * _set_si()	- set status report interval
 * _run_boot()  - request boot loader entry
 * cmd_set_jv() - set JSON verbosity level (exposed) - for details see jsonVerbosity in config.h
 */

static stat_t _set_md(cmdObj_t *cmd) 
{
	st_disable_motors();
	return (STAT_OK);
}

static stat_t _set_me(cmdObj_t *cmd) 
{
	st_enable_motors();
	return (STAT_OK);
}

static stat_t _get_qr(cmdObj_t *cmd) 
{
	cmd->value = (float)mp_get_planner_buffers_available();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t _run_qf(cmdObj_t *cmd) 
{
	cm_request_queue_flush();
//	cm_flush_planner();
	return (STAT_OK);
}

static stat_t _get_er(cmdObj_t *cmd) 
{
	rpt_exception(STAT_INTERNAL_ERROR, 42);	// bogus exception report
	return (STAT_OK);
}

static stat_t _get_rx(cmdObj_t *cmd)
{
	cmd->value = (float)xio_get_usb_rx_free();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t _get_sr(cmdObj_t *cmd)
{
	rpt_populate_unfiltered_status_report();
	return (STAT_OK);
}

static stat_t _set_sr(cmdObj_t *cmd)
{
	return (rpt_set_status_report(cmd));
}

static void _print_sr(cmdObj_t *cmd)
{
	rpt_populate_unfiltered_status_report();
}

static stat_t _set_si(cmdObj_t *cmd) 
{
	if (cmd->value < STATUS_REPORT_MIN_MS) { cmd->value = STATUS_REPORT_MIN_MS;}
	cfg.status_report_interval = (uint32_t)cmd->value;
	return(STAT_OK);
}

static stat_t _run_boot(cmdObj_t *cmd)
{
	tg_request_bootloader();
	return(STAT_OK);
}

//stat_t cmd_set_jv(cmdObj_t *cmd) 
static stat_t _set_jv(cmdObj_t *cmd) 
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


/**** GCODE MODEL ITEMS ****************************************
 * _get_msg_helper() - helper to get display message
 * _get_stat() - get combined machine state as value and string
 * _get_macs() - get raw machine state as value and string
 * _get_cycs() - get raw cycle state as value and string
 * _get_mots() - get raw motion state as value and string
 * _get_hold() - get raw hold state as value and string
 * _get_home() - get raw homing state as value and string
 * _get_unit() - get units mode as integer and display string
 * _get_coor() - get goodinate system
 * _get_momo() - get runtime motion mode
 * _get_plan() - get model gcode plane select
 * _get_path() - get model gcode path control mode
 * _get_dist() - get model gcode distance mode
 * _get_frmo() - get model gcode feed rate mode
 * _get_feed() - get feed rate 
 * _get_line() - get runtime line number for status reports
 * _get_vel()  - get runtime velocity
 * _get_pos()  - get runtime work position
 * _get_mpos() - get runtime machine position
 * _get_ofs()  - get runtime work offset
 * _print_pos()- print work position (with proper units)
 * _print_mpos()- print machine position (always mm units)
 */
static stat_t _get_msg_helper(cmdObj_t *cmd, prog_char_ptr msg, uint8_t value)
{
	cmd->value = (float)value;
	cmd->objtype = TYPE_INTEGER;
	ritorno(cmd_copy_string_P(cmd, (PGM_P)pgm_read_word(&msg[value*2]))); // hack alert: direct computation of index
	return (STAT_OK);
//	return((char *)pgm_read_word(&msg[(uint8_t)value]));
}

static stat_t _get_stat(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_stat, cm_get_combined_state()));

/* how to do this w/o calling the helper routine - See 331.09 for original routines
	cmd->value = cm_get_machine_state();
	cmd->objtype = TYPE_INTEGER;
	ritorno(cmd_copy_string_P(cmd, (PGM_P)pgm_read_word(&msg_stat[(uint8_t)cmd->value]),CMD_STRING_LEN));
	return (STAT_OK);
 */
//	strncpy_P(cmd->string_value,(PGM_P)pgm_read_word(&msg_stat[(uint8_t)cmd->value]),CMD_STRING_LEN);
}

static stat_t _get_macs(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_macs, cm_get_machine_state()));
}

static stat_t _get_cycs(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_cycs, cm_get_cycle_state()));
}

static stat_t _get_mots(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_mots, cm_get_motion_state()));
}

static stat_t _get_hold(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_hold, cm_get_hold_state()));
}

static stat_t _get_home(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_home, cm_get_homing_state()));
}

static stat_t _get_unit(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_unit, cm_get_model_units_mode()));
}

static stat_t _get_coor(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_coor, cm_get_model_coord_system()));
}

static stat_t _get_momo(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_momo, cm_get_runtime_motion_mode()));
}

static stat_t _get_plan(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_plan, cm_get_model_select_plane()));
}

static stat_t _get_path(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_path, cm_get_model_path_control()));
}

static stat_t _get_dist(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_dist, cm_get_model_distance_mode()));
}

static stat_t _get_frmo(cmdObj_t *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_frmo, cm_get_model_inverse_feed_rate_mode()));
}

static stat_t _get_line(cmdObj_t *cmd)
{
	cmd->value = (float)mp_get_runtime_linenum();
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t _get_vel(cmdObj_t *cmd) 
{
	cmd->value = mp_get_runtime_velocity();
	if (cm_get_model_units_mode() == INCHES) cmd->value *= INCH_PER_MM;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t _get_pos(cmdObj_t *cmd) 
{
	cmd->value = cm_get_runtime_work_position(_get_pos_axis(cmd->index));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t _get_mpos(cmdObj_t *cmd) 
{
	cmd->value = cm_get_runtime_machine_position(_get_pos_axis(cmd->index));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
//	cmd->objtype = TYPE_FLOAT_UNITS;	//++++ UNTESTED
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t _get_ofs(cmdObj_t *cmd) 
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
	char axes[6] = {"XYZABC"};
	char format[CMD_FORMAT_LEN+1];
	uint8_t axis = _get_pos_axis(cmd->index);
	if (axis >= AXIS_A) { units = DEGREES;}
	fprintf(stderr, _get_format(cmd->index,format), axes[axis], cmd->value, (PGM_P)pgm_read_word(&msg_units[(uint8_t)units]));
}

static void _print_pos(cmdObj_t *cmd)		// print position with unit displays for MM or Inches
{
	_print_pos_helper(cmd, cm_get_model_units_mode());
}

static void _print_mpos(cmdObj_t *cmd)		// print position with fixed unit display - always in Degrees or MM
{
	_print_pos_helper(cmd, MILLIMETERS);
}

/**** GCODE AND RELATED FUNCTIONS *********************************************
 * _get_gc()	- get gcode block
 * _run_gc()	- launch the gcode parser on a block of gcode
 * _run_home()	- invoke a homing cycle
 */
static stat_t _get_gc(cmdObj_t *cmd)
{
	ritorno(cmd_copy_string(cmd, tg.in_buf));
	cmd->objtype = TYPE_STRING;
	return (STAT_OK);
}

static stat_t _run_gc(cmdObj_t *cmd)
{
	return(gc_gcode_parser(*cmd->stringp));
}

static stat_t _run_home(cmdObj_t *cmd)
{
	if (cmd->value == true) { cm_homing_cycle_start();}
	return (STAT_OK);
}

/**** AXIS AND MOTOR FUNCTIONS ************************************************
 * _set_motor_steps_per_unit() - update this derived value
 * _get_am() - get axis mode w/enumeration string
 * _set_am() - set axis mode w/exception handling for axis type
 * _set_sw() - run this any time you change a switch setting	
 * _set_sa() - set motor step_angle & recompute steps_per_unit
 * _set_tr() - set motor travel_per_rev & recompute steps_per_unit
 * _set_mi() - set microsteps & recompute steps_per_unit
 * _set_po() - set polarity and update stepper structs
 * _set_pm() - set motor power mode and take action
 *
 * _pr_ma_ui8() - print motor or axis uint8 value w/no units or unit conversion
 * _pr_ma_lin() - print linear value with units and in/mm unit conversion
 * _pr_ma_rot() - print rotary value with units
 * _print_am()	- print axis mode with enumeration string
 * _print_coor()- print coordinate offsets with linear units
 * _print_corr()- print coordinate offsets with rotary units
 */

// helper. This function will need to be rethought if microstep morphing is implemented
static stat_t _set_motor_steps_per_unit(cmdObj_t *cmd) 
{
	uint8_t m = _get_motor(cmd->index);
	cfg.m[m].steps_per_unit = (360 / (cfg.m[m].step_angle / cfg.m[m].microsteps) / cfg.m[m].travel_rev);
	return (STAT_OK);
}

static stat_t _get_am(cmdObj_t *cmd)
{
	_get_ui8(cmd);
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_am, cmd->value)); // see 331.09 for old method
}

static stat_t _set_am(cmdObj_t *cmd)		// axis mode
{
	char linear_axes[] = {"xyz"};
	if (strchr(linear_axes, cmd->token[0]) != NULL) { // true if it's a linear axis
		if (cmd->value > AXIS_MAX_LINEAR) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	} else {
		if (cmd->value > AXIS_MAX_ROTARY) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	}
	_set_ui8(cmd);
	return(STAT_OK);
}

static stat_t _set_sw(cmdObj_t *cmd)		// switch setting
{
	if (cmd->value > SW_MODE_MAX_VALUE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	_set_ui8(cmd);
	gpio_init();
	return (STAT_OK);
}

static stat_t _set_sa(cmdObj_t *cmd)		// motor step angle
{ 
	_set_dbl(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (STAT_OK);
}

static stat_t _set_tr(cmdObj_t *cmd)		// motor travel per revolution
{ 
	_set_dbu(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (STAT_OK);
}

static stat_t _set_mi(cmdObj_t *cmd)		// motor microsteps
{
	if (fp_NE(cmd->value,1) && fp_NE(cmd->value,2) && fp_NE(cmd->value,4) && fp_NE(cmd->value,8)) {
		cmd_add_message_P(PSTR("*** WARNING *** Setting non-standard microstep value"));
	}
	_set_ui8(cmd);							// set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
	st_set_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

static stat_t _set_po(cmdObj_t *cmd)		// motor polarity
{ 
	ritorno (_set_01(cmd));
	st_set_polarity(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (STAT_OK);
}

static stat_t _set_pm(cmdObj_t *cmd)		// motor power mode
{ 
	ritorno (_set_01(cmd));
	if (fp_ZERO(cmd->value)) {				// zero means enable motor - i.e. disable power management mode
		st_enable_motor(_get_motor(cmd->index));
	} else {
		st_disable_motor(_get_motor(cmd->index));
	}
	return (STAT_OK);
}

static void _pr_ma_ui8(cmdObj_t *cmd)		// print uint8_t value
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value);
}

static void _pr_ma_lin(cmdObj_t *cmd)		// print a linear value in prevailing units
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->value, 
					(PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void _pr_ma_rot(cmdObj_t *cmd)		// print a rotary value in degrees units
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->value,
					(PGM_P)pgm_read_word(&msg_units[F_DEG]));
}

static void _print_am(cmdObj_t *cmd)		// print axis mode with enumeration string
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, (uint8_t)cmd->value,
					(PGM_P)pgm_read_word(&msg_am[(uint8_t)cmd->value]));
}

static void _print_coor(cmdObj_t *cmd)	// print coordinate offsets with linear units
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->token, cmd->value,
					(PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void _print_corr(cmdObj_t *cmd)	// print coordinate offsets with rotary units
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->group, cmd->token, cmd->group, cmd->token, cmd->value,
					(PGM_P)pgm_read_word(&msg_units[F_DEG]));
}

/**** COMMUNICATIONS SETTINGS *************************************************
 * _set_ic() - ignore CR or LF on RX
 * _set_ec() - enable CRLF on TX
 * _set_ee() - enable character echo
 * _set_ex() - enable XON/XOFF or RTS/CTS flow control
 * _set_baud() - set USB baud rate
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

static stat_t _set_ic(cmdObj_t *cmd) 				// ignore CR or LF on RX
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

static stat_t _set_ec(cmdObj_t *cmd) 				// expand CR to CRLF on TX
{
	if (cmd->value > true) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_cr = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_CRLF, XIO_NOCRLF));
}

static stat_t _set_ee(cmdObj_t *cmd) 				// enable character echo
{
	if (cmd->value > true) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_echo = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_ECHO, XIO_NOECHO));
}

static stat_t _set_ex(cmdObj_t *cmd)				// enable XON/XOFF or RTS/CTS flow control
{
	if (cmd->value > FLOW_CONTROL_RTS) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	cfg.enable_flow_control = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_XOFF, XIO_NOXOFF));
}

/*
 * _set_baud() - set USB baud rate
 *
 *	See xio_usart.h for valid values. Works as a callback.
 *	The initial routine changes the baud config setting and sets a flag
 *	Then it posts a user message indicating the new baud rate
 *	Then it waits for the TX buffer to empty (so the message is sent)
 *	Then it performs the callback to apply the new baud rate
 */

static stat_t _set_baud(cmdObj_t *cmd)
{
	uint8_t baud = (uint8_t)cmd->value;
	if ((baud < 1) || (baud > 6)) {
		cmd_add_message_P(PSTR("*** WARNING *** Illegal baud rate specified"));
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	cfg.usb_baud_rate = baud;
	cfg.usb_baud_flag = true;
	char message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, PSTR("*** NOTICE *** Restting baud rate to %S"),(PGM_P)pgm_read_word(&msg_baud[baud]));
	cmd_add_message(message);
	return (STAT_OK);
}

stat_t cfg_baud_rate_callback(void) 
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

static stat_t _do_group_list(cmdObj_t *cmd, char list[][CMD_TOKEN_LEN+1]) // helper to print multiple groups in a list
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
	char list[][CMD_TOKEN_LEN+1] = {"1","2","3","4",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_axes(cmdObj_t *cmd)	// print parameters for all axis groups
{
	char list[][CMD_TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_offsets(cmdObj_t *cmd)	// print offset parameters for G54-G59,G92, G28, G30
{
	char list[][CMD_TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92","g28","g30",""}; // must have a terminating element
	return (_do_group_list(cmd, list));
}

static stat_t _do_all(cmdObj_t *cmd)	// print all parameters
{
	strcpy(cmd->token,"sys");			// print system group
	_get_grp(cmd);
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	_do_motors(cmd);					// print all motor groups
	_do_axes(cmd);						// print all axis groups

	strcpy(cmd->token,"p1");			// print PWM group		
	_get_grp(cmd);
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	return (_do_offsets(cmd));			// print all offsets
}

/******************************************************************************
 ******************************************************************************
 ******************************************************************************
 *** END SETTING-SPECIFIC REGION **********************************************
 *** Code below should not require changes as parameters are added/updated ****
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************/

/******************************************************************************
 **** CMD FUNCTION ENTRY POINTS ***********************************************
 ******************************************************************************
 * These are the primary access points to cmd functions
 * These are the gatekeeper functions that check index ranges so others don't have to
 *
 * cmd_set() 	- Write a value or invoke a function - operates on single valued elements or groups
 * cmd_get() 	- Build a cmdObj with the values from the target & return the value
 *			   	  Populate cmd body with single valued elements or groups (iterates)
 * cmd_print()	- Output a formatted string for the value.
 * cmd_persist()- persist value to NVM. Takes special cases into account
 */

#define ASSERT_CMD_INDEX(a) if (cmd->index >= CMD_INDEX_MAX) return (a);

stat_t cmd_set(cmdObj_t *cmd)
{
	ASSERT_CMD_INDEX(STAT_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].set)))(cmd));
}

stat_t cmd_get(cmdObj_t *cmd)
{
	ASSERT_CMD_INDEX(STAT_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd));
}

void cmd_print(cmdObj_t *cmd)
{
	if (cmd->index >= CMD_INDEX_MAX) return;
	((fptrPrint)(pgm_read_word(&cfgArray[cmd->index].print)))(cmd);
}

void cmd_persist(cmdObj_t *cmd)
{
#ifdef __DISABLE_PERSISTENCE	// cutout for faster simulation in test
	return;
#endif
	if (_index_lt_groups(cmd->index) == false) return;
	if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_PERSIST) {
		cmd_write_NVM_value(cmd);
	}
}

/******************************************************************************
 * cfg_init() - called once on hard reset
 * _set_defa() - reset NVM with default values for active profile
 *
 * Performs one of 2 actions:
 *	(1) if NVM is set up or out-of-rev load RAM and NVM with settings.h defaults
 *	(2) if NVM is set up and at current config version use NVM data for config
 *
 *	You can assume the cfg struct has been zeroed by a hard reset. 
 *	Do not clear it as the version and build numbers have already been set by tg_init()
 */
void cfg_init()
{
	cmdObj_t *cmd = cmd_reset_list();
	cmdStr.magic_start = MAGICNUM;
	cmdStr.magic_end = MAGICNUM;
	cfg.magic_start = MAGICNUM;
	cfg.magic_end = MAGICNUM;

	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode
	cfg.nvm_base_addr = NVM_BASE_ADDR;
	cfg.nvm_profile_base = cfg.nvm_base_addr;
	cmd->index = 0;							// this will read the first record in NVM

	cmd_read_NVM_value(cmd);
	if (cmd->value != tg.fw_build) {
		cmd->value = true;					// case (1) NVM is not setup or not in revision
		_set_defa(cmd);	
	} else {								// case (2) NVM is setup and in revision
		rpt_print_loading_configs_message();
		for (cmd->index=0; _index_is_single(cmd->index); cmd->index++) {
			if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_INITIALIZE) {
				strcpy_P(cmd->token, cfgArray[cmd->index].token);	// read the token from the array
				cmd_read_NVM_value(cmd);
				cmd_set(cmd);
			}
		}
		rpt_init_status_report();
	}
}

// _set_defa() is both a helper and called directly from the $defa=1 command
static stat_t _set_defa(cmdObj_t *cmd) 
{
	if (cmd->value != true) {				// failsafe. Must set true or no action occurs
		print_defaults_help(cmd);
		return (STAT_OK);
	}
	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode

	for (cmd->index=0; _index_is_single(cmd->index); cmd->index++) {
		if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_INITIALIZE) {
			cmd->value = (float)pgm_read_float(&cfgArray[cmd->index].def_value);
			strcpy_P(cmd->token, cfgArray[cmd->index].token);
			cmd_set(cmd);
			cmd_persist(cmd);				// persist must occur when no other interrupts are firing
		}
	}
	rpt_print_initializing_message();		// don't start TX until all the NVM persistence is done
	rpt_init_status_report();				// reset status reports
	return (STAT_OK);
}

/******************************************************************************
 * cfg_text_parser() - update a config setting from a text block (text mode)
 * _text_parser() 	 - helper for above
 * 
 * Use cases handled:
 *	- $xfr=1200	set a parameter
 *	- $xfr		display a parameter
 *	- $x		display a group
 *	- ?			generate a status report (multiline format)
 */
stat_t cfg_text_parser(char *str)
{
	cmdObj_t *cmd = cmd_reset_list();		// returns first object in the body
	stat_t status = STAT_OK;

	// pre-process the command 
	if (str[0] == '?') {					// handle status report case
		rpt_run_text_status_report();
		return (STAT_OK);
	}
	if ((str[0] == '$') && (str[1] == NUL)) { // treat a lone $ as a sys request
		strcat(str,"sys");
	}

	// parse and execute the command (only processes 1 command per line)
	ritorno(_text_parser(str, cmd));		// run the parser to decode the command
	if ((cmd->objtype == TYPE_NULL) || (cmd->objtype == TYPE_PARENT)) {
		if (cmd_get(cmd) == STAT_COMPLETE) {// populate value, group values, or run uber-group displays
			return (STAT_OK);				// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		status = cmd_set(cmd);				// set (or run) single value
		cmd_persist(cmd);					// conditionally persist depending on flags in array
	}
	cmd_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT); // print the results
	return (status);
}

static stat_t _text_parser(char *str, cmdObj_t *cmd)
{
	char *ptr_rd, *ptr_wr;					// read and write pointers
	char separators[] = {" =:|\t"};			// any separator someone might use

	// pre-process and normalize the string
	cmd_reset_obj(cmd);						// initialize config object
	cmd_copy_string(cmd, str);				// make a copy for eventual reporting
	if (*str == '$') str++;					// ignore leading $
	for (ptr_rd = ptr_wr = str; *ptr_rd!=NUL; ptr_rd++, ptr_wr++) {
		*ptr_wr = tolower(*ptr_rd);			// convert string to lower case
		if (*ptr_rd==',') { *ptr_wr = *(++ptr_rd); } // skip over commas
	}
	*ptr_wr = NUL;							// terminate the string

	// parse fields into the cmd struct
	cmd->objtype = TYPE_NULL;
	if ((ptr_rd = strpbrk(str, separators)) == NULL) { // no value part
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
	} else {
		*ptr_rd = NUL;						// terminate at end of name
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
		str = ++ptr_rd;
		cmd->value = strtod(str, &ptr_rd);	// ptr_rd used as end pointer
		if (ptr_rd != str) {
			cmd->objtype = TYPE_FLOAT;
		}
	}

	// validate and post-process the token
	if ((cmd->index = cmd_get_index("",cmd->token)) == NO_MATCH) { // get index or fail it
		return (STAT_UNRECOGNIZED_COMMAND);
	}
	strcpy_P(cmd->group, cfgArray[cmd->index].group);	// capture the group string if there is one

	if (strlen(cmd->group) > 0) {			// see if you need to strip the token
		ptr_wr = cmd->token;
		ptr_rd = cmd->token + strlen(cmd->group);
		while (*ptr_rd != NUL) {
			*(ptr_wr)++ = *(ptr_rd)++;
		}
		*ptr_wr = NUL;
	}
	return (STAT_OK);
}

/*
 * cfg_cycle_check() - check if in a machining cycle and toss command if so
 */

stat_t cfg_cycle_check(void)
{
	return (STAT_OK);
}


/***** Generic Internal Functions *********************************************
 * Generic sets()
 * _set_nul() - set nothing (returns STAT_NOOP)
 * _set_ui8() - set value as 8 bit uint8_t value w/o unit conversion
 * _set_01()  - set a 0 or 1 uint8_t value with validation
 * _set_012() - set a 0, 1 or 2 uint8_t value with validation
 * _set_int() - set value as 32 bit integer w/o unit conversion
 * _set_dbl() - set value as float w/o unit conversion
 * _set_dbu() - set value as float w/unit conversion
 *
 * Generic gets()
 * _get_nul() - get nothing (returns STAT_NOOP)
 * _get_ui8() - get value as 8 bit uint8_t w/o unit conversion
 * _get_int() - get value as 32 bit integer w/o unit conversion
 * _get_dbl() - get value as float w/o unit conversion
 * _get_dbu() - get value as float w/unit conversion
 */
static stat_t _set_nul(cmdObj_t *cmd) { return (STAT_NOOP);}

static stat_t _set_ui8(cmdObj_t *cmd)
{
	*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

static stat_t _set_01(cmdObj_t *cmd)
{
	if (cmd->value > 1) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (_set_ui8(cmd));
	}
}

static stat_t _set_012(cmdObj_t *cmd)
{
	if (cmd->value > 2) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (_set_ui8(cmd));
	}
}

static stat_t _set_0123(cmdObj_t *cmd)
{
	if (cmd->value > 3) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (_set_ui8(cmd));
	}
}

static stat_t _set_int(cmdObj_t *cmd)
{
	*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

static stat_t _set_dbl(cmdObj_t *cmd)
{
	*((float *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT;
	return(STAT_OK);
}

static stat_t _set_dbu(cmdObj_t *cmd)
{
	if (cm_get_model_units_mode() == INCHES) { cmd->value *= MM_PER_INCH;}
	*((float *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT_UNITS;
	return(STAT_OK);
}

static stat_t _get_nul(cmdObj_t *cmd) 
{ 
	cmd->objtype = TYPE_NULL;
	return (STAT_NOOP);
}

static stat_t _get_ui8(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t _get_int(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

static stat_t _get_dbl(cmdObj_t *cmd)
{
	cmd->value = *((float *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

static stat_t _get_dbu(cmdObj_t *cmd)
{
	_get_dbl(cmd);
	if (cm_get_model_units_mode() == INCHES) {
		cmd->value *= INCH_PER_MM;
	}
//	cmd->objtype = TYPE_FLOAT_UNITS;	// ++++ UNTESTED
	return (STAT_OK);
}

/* Generic prints()
 * _print_nul() - print nothing
 * _print_str() - print string value
 * _print_ui8() - print uint8_t value w/no units or unit conversion
 * _print_int() - print integer value w/no units or unit conversion
 * _print_dbl() - print float value w/no units or unit conversion
 * _print_lin() - print linear value with units and in/mm unit conversion
 * _print_rot() - print rotary value with units
 */
static void _print_nul(cmdObj_t *cmd) {}

static void _print_str(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), *cmd->stringp);
}

static void _print_ui8(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), (uint8_t)cmd->value);
}

static void _print_int(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), (uint32_t)cmd->value);
}

static void _print_dbl(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value);
}

static void _print_lin(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[cm_get_model_units_mode()]));
}

static void _print_rot(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[F_DEG]));
}

/******************************************************************************
 * Accessors - get various data from an object given the index
 * _get_format() 	- return format string for an index
 * _get_motor()		- return the motor number as an index or -1 if na
 * _get_axis()		- return the axis as an index or -1 if na 
 * _get_pos_axis()	- return axis number for pos values or -1 if none - e.g. posx
 */
static char *_get_format(const index_t i, char *format)
{
	strncpy_P(format, (PGM_P)pgm_read_word(&cfgArray[i].format), CMD_FORMAT_LEN);
	return (format);
}

static int8_t _get_motor(const index_t i)
{
	char *ptr;
	char motors[] = {"1234"};
	char tmp[CMD_TOKEN_LEN+1];

	strcpy_P(tmp, cfgArray[i].group);
	if ((ptr = strchr(motors, tmp[0])) == NULL) {
		return (-1);
	}
	return (ptr - motors);
}
/*
static int8_t _get_axis(const index_t i)
{
	char *ptr;
	char tmp[CMD_TOKEN_LEN+1];
	char axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[0])) == NULL) { return (-1);}
	return (ptr - axes);
}
*/
static int8_t _get_pos_axis(const index_t i)
{
	char *ptr;
	char tmp[CMD_TOKEN_LEN+1];
	char axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[3])) == NULL) { return (-1);}
	return (ptr - axes);
}

/******************************************************************************
 * Group operations
 *
 *	Group operations work on parent/child groups where the parent is one of:
 *	  axis group 			x,y,z,a,b,c
 *	  motor group			1,2,3,4
 *	  PWM group				p1
 *	  coordinate group		g54,g55,g56,g57,g58,g59,g92
 *	  system group			"sys" - a collection of otherwise unrelated variables
 *
 *	Text mode can only GET groups. For example:
 *	  $x					get all members of an axis group
 *	  $1					get all members of a motor group
 *	  $<grp>				get any named group from the above lists
 *
 *	In JSON groups are carried as parent / child objects & can get and set elements:
 *	  {"x":""}						get all X axis parameters
 *	  {"x":{"vm":""}}				get X axis velocity max 
 *	  {"x":{"vm":1000}}				set X axis velocity max
 *	  {"x":{"vm":"","fr":""}}		get X axis velocity max and feed rate 
 *	  {"x":{"vm":1000,"fr";900}}	set X axis velocity max and feed rate
 *	  {"x":{"am":1,"fr":800,....}}	set multiple or all X axis parameters
 */

/*
 * _get_grp() - read data from axis, motor, system or other group
 *
 *	_get_grp() is a group expansion function that expands the parent group and 
 *	returns the values of all the children in that group. It expects the first 
 *	cmdObj in the cmdBody to have a valid group name in the token field. This 
 *	first object will be set to a TYPE_PARENT. The group field is left nul -  
 *	as the group field refers to a parent group, which this group has none.
 *
 *	All subsequent cmdObjs in the body will be populated with their values.
 *	The token field will be populated as will the parent name in the group field. 
 *
 *	The sys group is an exception where the childern carry a blank group field, 
 *	even though the sys parent is labeled as a TYPE_PARENT.
 */

static stat_t _get_grp(cmdObj_t *cmd)
{
	char *parent_group = cmd->token;		// token in the parent cmd object is the group
	char group[CMD_GROUP_LEN+1];			// group string retrieved from cfgArray child
	cmd->objtype = TYPE_PARENT;				// make first object the parent 
	for (index_t i=0; i<=CMD_INDEX_END_SINGLES; i++) {
		strcpy_P(group, cfgArray[i].group);  // don't need strncpy as it's always terminated
		if (strcmp(parent_group, group) != 0) continue;
		(++cmd)->index = i;
		cmd_get_cmdObj(cmd);
	}
	return (STAT_OK);
}

/*
 * _set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and 
 *	a setter. It iterates the group children and either gets the value or sets
 *	the value for each depending on the cmd->objtype.
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

static stat_t _set_grp(cmdObj_t *cmd)
{
	if (cfg.comm_mode == TEXT_MODE) return (STAT_UNRECOGNIZED_COMMAND);
	for (uint8_t i=0; i<CMD_MAX_OBJECTS; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if (cmd->objtype == TYPE_EMPTY) break;
		else if (cmd->objtype == TYPE_NULL)	// NULL means GET the value
			cmd_get(cmd);
		else {
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (STAT_OK);
}

/*
 * cmd_group_is_prefixed() - hack
 *
 *	This little function deals with the fact that some groups don't use the parent 
 *	token as a prefix to the child elements; SR being a good example.
 */
uint8_t cmd_group_is_prefixed(char *group)
{
	if (strstr("sr",group) != NULL) {	// you can extend like this: "sr,sys,xyzzy"
		return (false);
	}
	return (true);
}

/******************************************************************************
 ***** cmdObj functions *******************************************************
 ******************************************************************************/

/******************************************************************************
 * cmdObj helper functions and other low-level cmd helpers
 * cmd_get_type()		 - returns command type as a CMD_TYPE enum
 * cmd_persist_offsets() - write any changed G54 (et al) offsets back to NVM
 * cmd_get_index() 		 - get index from mnenonic token + group
 */
uint8_t cmd_get_type(cmdObj_t *cmd)
{
	if (cmd->token[0] == NUL) return (CMD_TYPE_NULL);
	if (strcmp("gc", cmd->token) == 0) return (CMD_TYPE_GCODE);
	if (strcmp("sr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("qr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("msg",cmd->token) == 0) return (CMD_TYPE_MESSAGE);
	if (strcmp("n",  cmd->token) == 0) return (CMD_TYPE_LINENUM);
	return (CMD_TYPE_CONFIG);
}

stat_t cmd_persist_offsets(uint8_t flag)
{
	if (flag == true) {
		cmdObj_t cmd;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf(cmd.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				cmd.index = cmd_get_index("", cmd.token);
				cmd.value = cfg.offset[i][j];
				cmd_persist(&cmd);				// only writes changed values
			}
		}
	}
	return (STAT_OK);
}

/* 
 * cmd_get_index() is the most expensive routine in the whole config. It does a linear table scan 
 * of the PROGMEM strings, which of course could be further optimized with indexes or hashing.
 */
index_t cmd_get_index(const char *group, const char *token)
{
	char c;
	char str[CMD_TOKEN_LEN+1];
	strcpy(str, group);
	strcat(str, token);

	for (index_t i=0; i<CMD_INDEX_MAX; i++) {
		if ((c = (char)pgm_read_byte(&cfgArray[i].token[0])) != str[0]) {	// 1st character mismatch 
			continue;
		}
		if ((c = (char)pgm_read_byte(&cfgArray[i].token[1])) == NUL) {
			if (str[1] == NUL) return(i);									// one character match
		}
		if (c != str[1]) continue;											// 2nd character mismatch
		if ((c = (char)pgm_read_byte(&cfgArray[i].token[2])) == NUL) {
			if (str[2] == NUL) return(i);									// two character match
		}
		if (c != str[2]) continue;											// 3rd character mismatch
		if ((c = (char)pgm_read_byte(&cfgArray[i].token[3])) == NUL) {
			if (str[3] == NUL) return(i);									// three character match
		}
		if (c != str[3]) continue;											// 4th character mismatch
		if ((c = (char)pgm_read_byte(&cfgArray[i].token[4])) == NUL) {
			if (str[4] == NUL) return(i);									// four character match
		}
		if (c != str[4]) continue;											// 5th character mismatch
		return (i);															// five character match
	}
	return (NO_MATCH);
}

/*
 * cmdObj low-level object and list operations
 * cmd_get_cmdObj()		- setup a cmd object by providing the index
 * cmd_reset_obj()		- quick clear for a new cmd object
 * cmd_reset_list()		- clear entire header, body and footer for a new use
 * cmd_copy_string()	- used to write a string to shared string storage and link it
 * cmd_copy_string_P()	- same, but for progmem string sources
 * cmd_add_object()		- write contents of parameter to  first free object in the body
 * cmd_add_integer()	- add an integer value to end of cmd body (Note 1)
 * cmd_add_float()		- add a floating point value to end of cmd body
 * cmd_add_string()		- add a string object to end of cmd body
 * cmd_add_string_P()	- add a program memory string as a string object to end of cmd body
 * cmd_add_message()	- add a message to cmd body
 * cmd_add_message_P()	- add a program memory message the the cmd body
 *
 *	Note: Functions that return a cmd pointer point to the object that was modified
 *	or a NULL pointer if there was an error
 *
 *	Note Adding a really large integer (like a checksum value) may lose precision 
 *	due to the cast to a float. Sometimes it's better to load an integer as a 
 *	string if all you want to do is display it.
 */

void cmd_get_cmdObj(cmdObj_t *cmd)
{
	if (cmd->index >= CMD_INDEX_MAX) return;
	index_t tmp = cmd->index;
	cmd_reset_obj(cmd);
	cmd->index = tmp;

	strcpy_P(cmd->token, cfgArray[cmd->index].token);	// token field is always terminated
	strcpy_P(cmd->group, cfgArray[cmd->index].group);	// group field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (cmd->group[0] != NUL) {
		if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_NOSTRIP) {
			cmd->group[0] = NUL;
		} else {
			strcpy(cmd->token, &cmd->token[strlen(cmd->group)]); // strip group from the token
		}
	}
	((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd);	// populate the value
}
 
cmdObj_t *cmd_reset_obj(cmdObj_t *cmd)	// clear a single cmdObj structure
{
	cmd->objtype = TYPE_EMPTY;				// selective clear is much faster than calling memset
	cmd->index = 0;
	cmd->value = 0;
	cmd->precision = 0;
	cmd->token[0] = NUL;
	cmd->group[0] = NUL;
	cmd->stringp = NULL;

	if (cmd->pv == NULL) { 				// set depth correctly
		cmd->depth = 0;
	} else {
		if (cmd->pv->objtype == TYPE_PARENT) { 
			cmd->depth = cmd->pv->depth + 1;
		} else {
			cmd->depth = cmd->pv->depth;
		}
	}
	return (cmd);
}

cmdObj_t *cmd_reset_list()					// clear the header and response body
{
	cmdStr.wp = 0;							// reset the shared string
	cmdObj_t *cmd = cmd_list;				// set up linked list and initialize elements	
	for (uint8_t i=0; i<CMD_LIST_LEN; i++, cmd++) {
		cmd->pv = (cmd-1);					// the ends are bogus & corrected later
		cmd->nx = (cmd+1);
		cmd->index = 0;
		cmd->depth = 1;						// header and footer are corrected later
		cmd->objtype = TYPE_EMPTY;
		cmd->token[0] = NUL;
	}
	(--cmd)->nx = NULL;
	cmd = cmd_list;							// setup response header element ('r')
	cmd->pv = NULL;
	cmd->depth = 0;
	cmd->objtype = TYPE_PARENT;
	strcpy(cmd->token, "r");
	return (cmd_body);						// this is a convenience for calling routines
}

stat_t cmd_copy_string(cmdObj_t *cmd, const char *src)
{
	if ((cmdStr.wp + strlen(src)) > CMD_SHARED_STRING_LEN) { return (STAT_BUFFER_FULL);}
	char *dst = &cmdStr.string[cmdStr.wp];
	strcpy(dst, src);						// copy string to current head position
	cmdStr.wp += strlen(src)+1;				// advance head for next string
	cmd->stringp = (char (*)[])dst;
	return (STAT_OK);
}

stat_t cmd_copy_string_P(cmdObj_t *cmd, const char *src_P)
{
	char buf[CMD_SHARED_STRING_LEN];
	strncpy_P(buf, src_P, CMD_SHARED_STRING_LEN);
	return (cmd_copy_string(cmd, buf));
}

cmdObj_t *cmd_add_object(char *token)		// add an object to the body using a token
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index("",token)) == NO_MATCH) { return (NULL);}
		cmd_get_cmdObj(cmd);				// populate the object from the index
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_integer(char *token, const uint32_t value)// add an integer object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = (float) value;
		cmd->objtype = TYPE_INTEGER;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_float(char *token, const float value)	// add a float object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = value;
		cmd->objtype = TYPE_FLOAT;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_string(char *token, const char *string)	// add a string object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		if (cmd_copy_string(cmd, string) != STAT_OK) { return (NULL);}
		cmd->index = cmd_get_index("", cmd->token);
		cmd->objtype = TYPE_STRING;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_string_P(char *token, const char *string)
{
	char message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, string);
	return(cmd_add_string(token, message));
}

cmdObj_t *cmd_add_message(const char *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (cfg.echo_json_messages != true)) { return (NULL);}
	return(cmd_add_string("msg", string));
}

cmdObj_t *cmd_add_message_P(const char *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (cfg.echo_json_messages != true)) { return (NULL);}
	char message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, string);
	return(cmd_add_string("msg", message));
}

/**** cmd_print_list() - print cmd_array as JSON or text **********************
 *
 * 	Generate and print the JSON and text mode output strings. Use this function 
 *	for all text and JSON output that wants to be in a response header. 
 *	Don't just printf stuff.
 *
 *	Inputs:
 *	  json_flags = JSON_OBJECT_FORMAT - print just the body w/o header or footer
 *	  json_flags = JSON_RESPONSE_FORMAT - print a full "r" object with footer
 *
 *	  text_flags = TEXT_INLINE_PAIRS - print text as name/value pairs on a single line
 *	  text_flags = TEXT_INLINE_VALUES - print text as comma separated values on a single line
 *	  text_flags = TEXT_MULTILINE_FORMATTED - print text one value per line with formatting string
 */
void cmd_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cfg.comm_mode == JSON_MODE) {
		switch (json_flags) {
			case JSON_NO_PRINT: { break; } 
			case JSON_OBJECT_FORMAT: { js_print_json_object(cmd_body); break; }
			case JSON_RESPONSE_FORMAT: { js_print_json_response(status); break; }
		}
	} else {
		switch (text_flags) {
			case TEXT_NO_PRINT: { break; } 
			case TEXT_INLINE_PAIRS: { _print_text_inline_pairs(); break; }
			case TEXT_INLINE_VALUES: { _print_text_inline_values(); break; }
			case TEXT_MULTILINE_FORMATTED: { _print_text_multiline_formatted();}
		}
	}
}

void _print_text_inline_pairs()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%s:%1.3f"), cmd->token, cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%s:%1.0f"), cmd->token, cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s:%s"), cmd->token, *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}		
	}
}

void _print_text_inline_values()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%1.3f"), cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%1.0f"), cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s"), *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}
	}
}

void _print_text_multiline_formatted()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		if (cmd->objtype != TYPE_PARENT) { cmd_print(cmd);}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype == TYPE_EMPTY) break;
	}
}

/******************************************************************************
 ***** EEPROM access functions ************************************************
 ******************************************************************************
 * cmd_read_NVM_value()	 - return value (as float) by index
 * cmd_write_NVM_value() - write to NVM by index, but only if the value has 
 * 	changed (see 331.09 or earlier for token/value record-oriented routines)
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */
stat_t cmd_read_NVM_value(cmdObj_t *cmd)
{
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
	return (STAT_OK);
}

stat_t cmd_write_NVM_value(cmdObj_t *cmd)
{
	if (cm.cycle_state != CYCLE_OFF) return (STAT_FILE_NOT_OPEN);	// can't write when machine is moving
	float tmp = cmd->value;
	ritorno(cmd_read_NVM_value(cmd));
	if (cmd->value != tmp) {		// catches the isnan() case as well
		cmd->value = tmp;
		int8_t nvm_byte_array[NVM_VALUE_LEN];
		memcpy(&nvm_byte_array, &tmp, NVM_VALUE_LEN);
		uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
	return (STAT_OK);
}

/****************************************************************************
 ***** Config Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_CONFIG

#define NVMwr(i,v) { cmd.index=i; cmd.value=v; cmd_write_NVM_value(&cmd);}
#define NVMrd(i)   { cmd.index=i; cmd_read_NVM_value(&cmd); printf("%f\n",cmd.value);}

void cfg_unit_tests()
{

// NVM tests
/*	cmdObj_t cmd;
	NVMwr(0, 329.01)
	NVMwr(1, 111.01)
	NVMwr(2, 222.02)
	NVMwr(3, 333.03)
	NVMwr(4, 444.04)
	NVMwr(10, 10.10)
	NVMwr(100, 100.100)
	NVMwr(479, 479.479)

	NVMrd(0)
	NVMrd(1)
	NVMrd(2)
	NVMrd(3)
	NVMrd(4)
	NVMrd(10)
	NVMrd(100)
	NVMrd(479)
*/

// config table tests

	index_t i;
//	float val;

//	_print_configs("$", NUL);					// no filter (show all)
//	_print_configs("$", 'g');					// filter for general parameters
//	_print_configs("$", '1');					// filter for motor 1
//	_print_configs("$", 'x');					// filter for x axis

	i = cmd_get_index("fb");
	i = cmd_get_index("xfr");
	i = cmd_get_index("g54");

//	i = _get_pos_axis(55);
//	i = _get_pos_axis(73);
//	i = _get_pos_axis(93);
//	i = _get_pos_axis(113);

/*
	for (i=0; i<CMD_MAX_INDEX; i++) {

		cmd_get(&cmd);

		cmd.value = 42;
		cmd_set(&cmd);

		val = _get_dbl_value(i);
		cmd_get_token(i, cmd.token);

//		_get_friendly(i, string);
		_get_format(i, cmd.vstring);
		_get_axis(i);							// uncomment main function to test
		_get_motor(i);
		cmd_set(i, &cmd);
		cmd_print(i);
	}

	_parse_config_string("$1po 1", &c);			// returns a number
	_parse_config_string("XFR=1200", &c);		// returns a number
	_parse_config_string("YFR 1300", &c);		// returns a number
	_parse_config_string("zfr	1400", &c);		// returns a number
	_parse_config_string("afr", &c);			// returns a null
	_parse_config_string("Bfr   ", &c);			// returns a null
	_parse_config_string("cfr=wordy", &c);		// returns a null

//	i = cfg_get_config_index("gc");
//	i = cfg_get_config_index("gcode");
//	i = cfg_get_config_index("c_axis_mode");
//	i = cfg_get_config_index("AINT_NOBODY_HOME");
	i = cfg_get_config_index("firmware_version");
*/
}

#endif
#endif

