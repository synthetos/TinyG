/*
 * config.c - configuration handling and persistence
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
 *	Config system overview
 *
 *	--- Config objects and the config list ---
 *
 *	The config system provides a structured way to access and set configuration variables.
 *	It also provides a way to get an arbitrary variable for reporting. Config operates
 *	as a collection of "objects" (OK, so they are not really objects) that encapsulate
 *	each variable. The objects are collected into a list (the body), which also has  
 *	header and footer objects. This way the internals don't care about how the variable
 *	is represented or communicated externally as all operations occur on the cmdObj list. 
 *	The list is populated by the text_parser or the JSON_parser depending on the mode.
 *	The lists are also used for responses and are read out (printed) by a text-mode or
 *	JSON print function.
 */
/*	--- Config variables, tables and strings ---
 *
 *	Each configuration value is identified by a short mnemonic string (token). The token 
 *	is resolved to an index into the cfgArray which is an array of structures with the 
 *	static assignments for each variable. The array is organized as:
 * 
 *	- cfgArray contains typed data in program memory (PROGMEM). Each cfgItem has:
 *		- group string identifying what group the variable is part of (if any)
 *		- token string - the token for that variable - pre-pended with the group (if any)
 *		- operations flags - flag if the value should be initialized and/or persisted to NVM
 *		- pointer to a formatted print string also in program memory (Used only for text mode)
 *		- function pointer for formatted print() method or text-mode readouts
 *		- function pointer for get() method - gets values from memory
 *		- function pointer for set() method - sets values and runs functions
 *		- target - memory location that the value is written to / read from
 *		- default value - for cold initialization
 *
 *	Additionally an NVM array contains values persisted to EEPROM as doubles; indexed by cfgArray index
 *
 *	The following rules apply to mnemonic tokens
 *	 - are up to 5 alphnuneric characters and cannot contain whitespace or separators
 *	 - must be unique (non colliding).
 *	 - axis tokens start with the axis letter and are typically 3 characters including the axis letter
 *	 - motor tokens start with the motor digit and are typically 3 characters including the motor digit
 *	 - non-axis or non-motor tokens are 2-5 characters and by convention generally should not start 
 *		with: xyzabcuvw0123456789 (but there can be exceptions)
 *
 *  "Groups" are collections of values that mimic REST resources. Groups include:
 *	 - axis groups prefixed by "xyzabc"		("uvw" are reserved)
 *	 - motor groups prefixed by "1234"		("56789" are reserved)
 *	 - PWM groups prefixed by p1, p2 	    (p3 - p9 are reserved)
 *	 - coordinate system groups prefixed by g54, g55, g56, g57, g59, g92
 *	 - a system group is identified by "sys" and contains a collection of otherwise unrelated values
 *
 *	"Uber-groups" are groups of groups that are only used for text-mode printing - e.g.
 *	 - group of all axes groups
 *	 - group of all motor groups
 *	 - group of all offset groups
 *	 - group of all groups
 */
/*  --- Making changes and adding new values
 *
 *	Adding a new value to config (or changing an existing one) involves touching the following places:
 *
 *	 - Add a formatting string to fmt_XXX strings. Not needed if there is no text-mode print function
 *	   of you are using one of the generic print strings.
 * 
 *	 - Create a new record in cfgArray[]. Use existing ones for examples. You can usually use existing
 *	   functions for get and set; or create a new one if you need a specialized function.
 *
 *	   The ordering of group displays is set by the order of items in cfgArray. None of the other 
 *	   orders matter but are generally kept sequenced for easier reading and code maintenance. Also,
 *	   Items earlier in the array will resolve token searches faster than ones later in the array.
 *
 *	   Note that matching will occur from the most specific to the least specific, meaning that
 *	   if tokens overlap the longer one should be earlier in the array: "gco" should precede "gc".
 */
/*  --- Rules, guidelines and random stuff
 *
 *	It's the responsibility of the object creator to set the index. Downstream functions
 *	all expect a valid index. Set the index by calling cmd_get_index(). This also validates
 *	the token and group if no lookup exists.
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
#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

typedef char PROGMEM *prog_char_ptr;	// access to PROGMEM arrays of PROGMEM strings

//*** STATIC STUFF ***********************************************************

struct cfgItem {						// structs for pgmArray
	char group[CMD_GROUP_LEN+1];		// group prefix (with NUL termination)
	char token[CMD_TOKEN_LEN+1];		// token - stripped of group prefix (w/NUL termination)
	uint8_t flags;						// operations flags - see defines below
	const char *format;					// pointer to formatted print string
	fptrPrint print;					// print binding: aka void (*print)(cmdObj *cmd);
	fptrCmd get;						// GET binding aka uint8_t (*get)(cmdObj *cmd)
	fptrCmd set;						// SET binding aka uint8_t (*set)(cmdObj *cmd)
	double *target;						// target for writing config value
	double def_value;					// default value for config item
};

// operations flags and shorthand
#define F_INITIALIZE	0x01
#define F_PERSIST 		0x02
#define _f00			0x00
#define _fin			F_INITIALIZE
#define _fpe			F_PERSIST
#define _fip			(F_INITIALIZE | F_PERSIST)

// prototypes are divided into generic functions and parameter-specific functions

// generic internal functions
static uint8_t _set_nul(cmdObj *cmd);	// noop
static uint8_t _set_ui8(cmdObj *cmd);	// set a uint8_t value
static uint8_t _set_int(cmdObj *cmd);	// set an integer value
static uint8_t _set_dbl(cmdObj *cmd);	// set a double value
static uint8_t _set_dbu(cmdObj *cmd);	// set a double with unit conversion
static uint8_t _get_nul(cmdObj *cmd);	// get null value type
static uint8_t _get_ui8(cmdObj *cmd);	// get uint8_t value
static uint8_t _get_int(cmdObj *cmd);	// get uint32_t integer value
static uint8_t _get_dbl(cmdObj *cmd);	// get double value
static uint8_t _get_dbu(cmdObj *cmd);	// get double with unit conversion
static void _print_nul(cmdObj *cmd);	// print nothing
static void _print_str(cmdObj *cmd);	// print a string value
static void _print_ui8(cmdObj *cmd);	// print unit8_t value w/no units
static void _print_int(cmdObj *cmd);	// print integer value w/no units
static void _print_dbl(cmdObj *cmd);	// print double value w/no units
static void _print_lin(cmdObj *cmd);	// print linear values
static void _print_rot(cmdObj *cmd);	// print rotary values

// helpers for generic functions
static char *_get_format(const INDEX_T i, char *format);
static int8_t _get_motor(const INDEX_T i);
//static int8_t _get_axis(const INDEX_T i);
static int8_t _get_pos_axis(const INDEX_T i);
static uint8_t _text_parser(char *str, struct cmdObject *c);
static uint8_t _get_msg_helper(cmdObj *cmd, prog_char_ptr msg, uint8_t value);
static void _print_text_inline_pairs();
static void _print_text_inline_values();
static void _print_text_multiline_formatted();

static uint8_t _set_grp(cmdObj *cmd);	// set data for a group
static uint8_t _get_grp(cmdObj *cmd);	// get data for a group
static uint8_t _do_motors(cmdObj *cmd);	// print parameters for all motor groups
static uint8_t _do_axes(cmdObj *cmd);	// print parameters for all axis groups
static uint8_t _do_offsets(cmdObj *cmd);// print offsets for G54-G59, G92
static uint8_t _do_all(cmdObj *cmd);	// print all parameters
static void _do_group_list(cmdObj *cmd, char list[][CMD_TOKEN_LEN+1]); // helper to print multiple groups in a list

/*****************************************************************************
 **** PARAMETER-SPECIFIC CODE REGION *****************************************
 **** This code and data will change as you add / update config parameters ***
 *****************************************************************************/

// parameter-specific internal functions
//static uint8_t _get_id(cmdObj *cmd);	// get device ID (signature)
static uint8_t _set_hv(cmdObj *cmd);	// set hardware version
static uint8_t _get_sr(cmdObj *cmd);	// run status report (as data)
static void _print_sr(cmdObj *cmd);		// run status report (as printout)
static uint8_t _set_sr(cmdObj *cmd);	// set status report specification
static uint8_t _set_si(cmdObj *cmd);	// set status report interval
static uint8_t _get_qr(cmdObj *cmd);	// run queue report (as data)
static uint8_t _get_rx(cmdObj *cmd);	// get bytes in RX buffer

static uint8_t _get_gc(cmdObj *cmd);	// get current gcode block
static uint8_t _run_gc(cmdObj *cmd);	// run a gcode block

static uint8_t _get_line(cmdObj *cmd);	// get runtime line number
static uint8_t _get_stat(cmdObj *cmd);	// get combined machine state as value and string
static uint8_t _get_macs(cmdObj *cmd);	// get raw machine state as value and string
static uint8_t _get_cycs(cmdObj *cmd);	// get raw cycle state (etc etc)...
static uint8_t _get_mots(cmdObj *cmd);	// get raw motion state...
static uint8_t _get_hold(cmdObj *cmd);	// get raw hold state...
static uint8_t _get_home(cmdObj *cmd);	// get raw homing state...
static uint8_t _get_unit(cmdObj *cmd);	// get unit mode...
static uint8_t _get_coor(cmdObj *cmd);	// get coordinate system in effect...
static uint8_t _get_momo(cmdObj *cmd);	// get motion mode...
static uint8_t _get_plan(cmdObj *cmd);	// get active plane...
static uint8_t _get_path(cmdObj *cmd);	// get patch control mode...
static uint8_t _get_dist(cmdObj *cmd);	// get distance mode...
static uint8_t _get_frmo(cmdObj *cmd);	// get feedrate mode...
static uint8_t _get_vel(cmdObj *cmd);	// get runtime velocity...
static uint8_t _get_pos(cmdObj *cmd);	// get runtime work position...
static uint8_t _get_mpos(cmdObj *cmd);	// get runtime machine position...
static void _print_pos(cmdObj *cmd);	// print runtime work position...

static uint8_t _set_defa(cmdObj *cmd);	// reset config to default values

static uint8_t _set_sa(cmdObj *cmd);	// set motor step angle
static uint8_t _set_tr(cmdObj *cmd);	// set motor travel per revolution
static uint8_t _set_mi(cmdObj *cmd);	// set microsteps
static uint8_t _set_po(cmdObj *cmd);	// set motor polarity
static uint8_t _set_motor_steps_per_unit(cmdObj *cmd);

static uint8_t _get_am(cmdObj *cmd);	// get axis mode
static uint8_t _set_am(cmdObj *cmd);	// set axis mode
static void _print_am(cmdObj *cmd);		// print axis mode
static uint8_t _set_sw(cmdObj *cmd);	// must run any time you change a switch setting

static uint8_t _set_ic(cmdObj *cmd);	// ignore CR or LF on RX input
//static uint8_t _set_ec(cmdObj *cmd);	// expand CRLF on TX outout
static uint8_t _set_ee(cmdObj *cmd);	// enable character echo
static uint8_t _set_ex(cmdObj *cmd);	// enable XON/XOFF
static uint8_t _set_baud(cmdObj *cmd);	// set USB baud rate

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

static const char msg_stat0[] PROGMEM = "Initializing";	// stat uses this array
static const char msg_stat1[] PROGMEM = "Reset";
static const char msg_stat2[] PROGMEM = "Stop";
static const char msg_stat3[] PROGMEM = "End";
static const char msg_stat4[] PROGMEM = "Run";
static const char msg_stat5[] PROGMEM = "Hold";
static const char msg_stat6[] PROGMEM = "Probe";
static const char msg_stat7[] PROGMEM = "Cycle";
static const char msg_stat8[] PROGMEM = "Homing";
static const char msg_stat9[] PROGMEM = "Jog";
static PGM_P const msg_stat[] PROGMEM = { msg_stat0, msg_stat1, msg_stat2, msg_stat3, msg_stat4, msg_stat5, msg_stat6, msg_stat7, msg_stat8, msg_stat9};

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
static const char msg_am04[] PROGMEM = "[slave X]";
static const char msg_am05[] PROGMEM = "[slave Y]";
static const char msg_am06[] PROGMEM = "[slave Z]";
static const char msg_am07[] PROGMEM = "[slave XY]";
static const char msg_am08[] PROGMEM = "[slave XZ]";
static const char msg_am09[] PROGMEM = "[slave YZ]";
static const char msg_am10[] PROGMEM = "[slave XYZ]";
static PGM_P const msg_am[] PROGMEM = {
	msg_am00, msg_am01, msg_am02, msg_am03, msg_am04, msg_am05, 
	msg_am06, msg_am07, msg_am08, msg_am09, msg_am10
};

/* PROGMEM strings for print formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */
static const char fmt_nul[] PROGMEM = "";
static const char fmt_ui8[] PROGMEM = "%d\n";	// generic format for ui8s
static const char fmt_dbl[] PROGMEM = "%f\n";	// generic format for doubles
static const char fmt_str[] PROGMEM = "%s\n";	// generic format for string message (with no formatting)

static const char fmt_fv[] PROGMEM = "[fv]  firmware_version%16.2f\n";
static const char fmt_fb[] PROGMEM = "[fb]  firmware_build%18.2f\n";
static const char fmt_hv[] PROGMEM = "[hv]  hardware_version%16.2f\n";
//static const char fmt_id[] PROGMEM = "[id]  id_device%16d\n";

// Gcode model values for reporting purposes
static const char fmt_vel[]  PROGMEM = "Velocity:%17.3f%S/min\n";
static const char fmt_line[] PROGMEM = "Line number:%10.0f\n";
static const char fmt_feed[] PROGMEM = "Feed rate:%16.3f%S/min\n";
static const char fmt_stat[] PROGMEM = "Machine state:       %s\n"; // combined machine state
static const char fmt_macs[] PROGMEM = "Raw machine state:   %s\n"; // raw machine state
static const char fmt_cycs[] PROGMEM = "Cycle state:         %s\n";
static const char fmt_mots[] PROGMEM = "Motion state:        %s\n";
static const char fmt_hold[] PROGMEM = "Feedhold state:      %s\n";
static const char fmt_home[] PROGMEM = "Homing state:        %s\n";
static const char fmt_unit[] PROGMEM = "Units:               %s\n"; // units mode as ASCII string
static const char fmt_coor[] PROGMEM = "Coordinate system:   %s\n";
static const char fmt_momo[] PROGMEM = "Motion mode:         %s\n";
static const char fmt_plan[] PROGMEM = "Plane:               %s\n";
static const char fmt_path[] PROGMEM = "Path Mode:           %s\n";
static const char fmt_dist[] PROGMEM = "Distance mode:       %s\n";
static const char fmt_frmo[] PROGMEM = "Feed rate mode:      %s\n";
static const char fmt_posx[] PROGMEM = "X position:%15.3f%S\n";
static const char fmt_posy[] PROGMEM = "Y position:%15.3f%S\n";
static const char fmt_posz[] PROGMEM = "Z position:%15.3f%S\n";
static const char fmt_posa[] PROGMEM = "A position:%15.3f%S\n";
static const char fmt_posb[] PROGMEM = "B position:%15.3f%S\n";
static const char fmt_posc[] PROGMEM = "C position:%15.3f%S\n";
static const char fmt_mpox[] PROGMEM = "X machine posn:%11.3f%S\n";
static const char fmt_mpoy[] PROGMEM = "Y machine posn:%11.3f%S\n";
static const char fmt_mpoz[] PROGMEM = "Z machine posn:%11.3f%S\n";
static const char fmt_mpoa[] PROGMEM = "A machine posn:%11.3f%S\n";
static const char fmt_mpob[] PROGMEM = "B machine posn:%11.3f%S\n";
static const char fmt_mpoc[] PROGMEM = "C machine posn:%11.3f%S\n";
static const char fmt_homx[] PROGMEM = "X axis homed:%4d\n";
static const char fmt_homy[] PROGMEM = "Y axis homed:%4d\n";
static const char fmt_homz[] PROGMEM = "Z axis homed:%4d\n";
static const char fmt_homa[] PROGMEM = "A axis homed:%4d\n";
static const char fmt_homb[] PROGMEM = "B axis homed:%4d\n";
static const char fmt_homc[] PROGMEM = "C axis homed:%4d\n";
static const char fmt_g92x[] PROGMEM = "X origin offset:%10.3f%S\n";
static const char fmt_g92y[] PROGMEM = "Y origin offset:%10.3f%S\n";
static const char fmt_g92z[] PROGMEM = "Z origin offset:%10.3f%S\n";
static const char fmt_g92a[] PROGMEM = "A origin offset:%10.3f%S\n";
static const char fmt_g92b[] PROGMEM = "B origin offset:%10.3f%S\n";
static const char fmt_g92c[] PROGMEM = "C origin offset:%10.3f%S\n";

// Gcode model power-on reset default values
static const char fmt_gpl[] PROGMEM = "[gpl] gcode_select_plane %10d [0,1,2]\n";
static const char fmt_gun[] PROGMEM = "[gun] gcode_units_mode   %10d [0,1]\n";
static const char fmt_gco[] PROGMEM = "[gco] gcode_coord_system %10d [1-6]\n";
static const char fmt_gpa[] PROGMEM = "[gpa] gcode_path_control %10d [0,1,2]\n";
static const char fmt_gdi[] PROGMEM = "[gdi] gcode_distance_mode%10d [0,1]\n";
static const char fmt_gc[]  PROGMEM = "[gc]";

// System settings
//static const char fmt_ea[] PROGMEM = "ea,enable_a,[ea]  enable_acceleration%10d [0,1]\n";
static const char fmt_ja[] PROGMEM = "[ja]  junction_acceleration%8.0f%S\n";
static const char fmt_ml[] PROGMEM = "[ml]  min_line_segment%17.3f%S\n";
static const char fmt_ma[] PROGMEM = "[ma]  min_arc_segment%18.3f%S\n";
static const char fmt_mt[] PROGMEM = "[mt]  min_segment_time%13.0f uSec\n";
static const char fmt_st[] PROGMEM = "[st]  switch_type%18d [0,1]\n";
static const char fmt_si[] PROGMEM = "[si]  status_interval%14.0f ms [0=off]\n";

static const char fmt_ic[] PROGMEM = "[ic]  ignore CR or LF on RX %7d [0,1=CR,2=LF]\n";
//static const char fmt_ec[] PROGMEM = "[ec]  enable_CR on TX%14d [0,1]\n";
static const char fmt_ee[] PROGMEM = "[ee]  enable_echo%18d [0,1]\n";
static const char fmt_ex[] PROGMEM = "[ex]  enable_xon_xoff%14d [0,1]\n";
static const char fmt_eq[] PROGMEM = "[eq]  enable_queue_reports%9d [0,1]\n";
static const char fmt_ej[] PROGMEM = "[ej]  enable_json_mode %12d [0,1]\n";
static const char fmt_jv[] PROGMEM = "[jv]  json_verbosity%15d [0-5]\n";
static const char fmt_tv[] PROGMEM = "[tv]  text_verbosity%15d [0-3]\n";
static const char fmt_baud[] PROGMEM = "[baud] USB baud rate%15d [0-6]\n";

// Motor strings
static const char fmt_1ma[] PROGMEM = "[1ma] m1_map_to_axis%15d [0=X, 1=Y...]\n";
static const char fmt_1sa[] PROGMEM = "[1sa] m1_step_angle%20.3f%S\n";
static const char fmt_1tr[] PROGMEM = "[1tr] m1_travel_per_revolution%9.3f%S\n";
static const char fmt_1mi[] PROGMEM = "[1mi] m1_microsteps%16d [1,2,4,8]\n";
static const char fmt_1po[] PROGMEM = "[1po] m1_polarity%18d [0,1]\n";
static const char fmt_1pm[] PROGMEM = "[1pm] m1_power_management%10d [0,1]\n";

static const char fmt_2ma[] PROGMEM = "[2ma] m2_map_to_axis%15d [0=X, 1=Y...]\n";
static const char fmt_2sa[] PROGMEM = "[2sa] m2_step_angle%20.3f%S\n";
static const char fmt_2tr[] PROGMEM = "[2tr] m2_travel_per_revolution%9.3f%S\n";
static const char fmt_2mi[] PROGMEM = "[2mi] m2_microsteps%16d [1,2,4,8]\n";
static const char fmt_2po[] PROGMEM = "[2po] m2_polarity%18d [0,1]\n";
static const char fmt_2pm[] PROGMEM = "[2pm] m2_power_management%10d [0,1]\n";

static const char fmt_3ma[] PROGMEM = "[3ma] m3_map_to_axis%15d [0=X, 1=Y...]\n";
static const char fmt_3sa[] PROGMEM = "[3sa] m3_step_angle%20.3f%S\n";
static const char fmt_3tr[] PROGMEM = "[3tr] m3_travel_per_revolution%9.3f%S\n";
static const char fmt_3mi[] PROGMEM = "[3mi] m3_microsteps%16d [1,2,4,8]\n";
static const char fmt_3po[] PROGMEM = "[3po] m3_polarity%18d [0,1]\n";
static const char fmt_3pm[] PROGMEM = "[3pm] m3_power_management%10d [0,1]\n";

static const char fmt_4ma[] PROGMEM = "[4ma] m4_map_to_axis%15d [0=X, 1=Y...]\n";
static const char fmt_4sa[] PROGMEM = "[4sa] m4_step_angle%20.3f%S\n";
static const char fmt_4tr[] PROGMEM = "[4tr] m4_travel_per_revolution%9.3f%S\n";
static const char fmt_4mi[] PROGMEM = "[4mi] m4_microsteps%16d [1,2,4,8]\n";
static const char fmt_4po[] PROGMEM = "[4po] m4_polarity%18d [0,1]\n";
static const char fmt_4pm[] PROGMEM = "[4pm] m4_power_management%10d [0,1]\n";

// Axis strings
static const char fmt_xam[] PROGMEM = "[xam] x_axis_mode%18d %S\n";
static const char fmt_xfr[] PROGMEM = "[xfr] x_feedrate_maximum%15.3f%S/min\n";
static const char fmt_xvm[] PROGMEM = "[xvm] x_velocity_maximum%15.3f%S/min\n";
static const char fmt_xtm[] PROGMEM = "[xtm] x_travel_maximum%17.3f%S\n";
static const char fmt_xjm[] PROGMEM = "[xjm] x_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_xjd[] PROGMEM = "[xjd] x_junction_deviation%14.4f%S (larger is faster)\n";
static const char fmt_xsn[] PROGMEM = "[xsn] x_switch_min%17d [0-4]\n";
static const char fmt_xsx[] PROGMEM = "[xsx] x_switch_max%17d [0-4]\n";
static const char fmt_xsv[] PROGMEM = "[xsv] x_search_velocity%16.3f%S/min\n";
static const char fmt_xlv[] PROGMEM = "[xlv] x_latch_velocity%17.3f%S/min\n";
static const char fmt_xlb[] PROGMEM = "[xlb] x_latch_backoff%18.3f%S\n";
static const char fmt_xzb[] PROGMEM = "[xzb] x_zero_backoff%19.3f%S\n";

static const char fmt_yam[] PROGMEM = "[yam] y_axis_mode%18d %S\n";
static const char fmt_yfr[] PROGMEM = "[yfr] y_feedrate_maximum%15.3f%S/min\n";
static const char fmt_yvm[] PROGMEM = "[yvm] y_velocity_maximum%15.3f%S/min\n";
static const char fmt_ytm[] PROGMEM = "[ytm] y_travel_maximum%17.3f%S\n";
static const char fmt_yjm[] PROGMEM = "[yjm] y_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_yjd[] PROGMEM = "[yjd] y_junction_deviation%14.4f%S (larger is faster)\n";
static const char fmt_ysn[] PROGMEM = "[ysn] y_switch_min%17d [0-4]\n";
static const char fmt_ysx[] PROGMEM = "[ysx] y_switch_max%17d [0-4]\n";
static const char fmt_ysv[] PROGMEM = "[ysv] y_search_velocity%16.3f%S/min\n";
static const char fmt_ylv[] PROGMEM = "[ylv] y_latch_velocity%17.3f%S/min\n";
static const char fmt_ylb[] PROGMEM = "[ylb] y_latch_backoff%18.3f%S\n";
static const char fmt_yzb[] PROGMEM = "[yzb] y_zero_backoff%19.3f%S\n";

static const char fmt_zam[] PROGMEM = "[zam] z_axis_mode%18d %S\n";
static const char fmt_zfr[] PROGMEM = "[zfr] z_feedrate_maximum%15.3f%S/min\n";
static const char fmt_zvm[] PROGMEM = "[zvm] z_velocity_maximum%15.3f%S/min\n";
static const char fmt_ztm[] PROGMEM = "[ztm] z_travel_maximum%17.3f%S\n";
static const char fmt_zjm[] PROGMEM = "[zjm] z_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_zjd[] PROGMEM = "[zjd] z_junction_deviation%14.4f%S (larger is faster)\n";
static const char fmt_zsn[] PROGMEM = "[zsn] z_switch_min%17d [0-4]\n";
static const char fmt_zsx[] PROGMEM = "[zsx] z_switch_max%17d [0-4]\n";
static const char fmt_zsv[] PROGMEM = "[zsv] z_search_velocity%16.3f%S/min\n";
static const char fmt_zlv[] PROGMEM = "[zlv] z_latch_velocity%17.3f%S/min\n";
static const char fmt_zlb[] PROGMEM = "[zlb] z_latch_backoff%18.3f%S\n";
static const char fmt_zzb[] PROGMEM = "[zzb] z_zero_backoff%19.3f%S\n";

static const char fmt_aam[] PROGMEM = "[aam] a_axis_mode%18d %S\n";
static const char fmt_afr[] PROGMEM = "[afr] a_feedrate_maximum%15.3f%S/min\n";
static const char fmt_avm[] PROGMEM = "[avm] a_velocity_maximum%15.3f%S/min\n";
static const char fmt_atm[] PROGMEM = "[atm] a_travel_maximum  %15.3f%S\n";
static const char fmt_ajm[] PROGMEM = "[ajm] a_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_ajd[] PROGMEM = "[ajd] a_junction_deviation%14.4f%S\n";
static const char fmt_ara[] PROGMEM = "[ara] a_radius_value%20.4f%S\n";
static const char fmt_asn[] PROGMEM = "[asn] a_switch_min%17d [0-4]\n";
static const char fmt_asx[] PROGMEM = "[asx] a_switch_max%17d [0-4]\n";
static const char fmt_asv[] PROGMEM = "[asv] a_search_velocity%16.3f%S/min\n";
static const char fmt_alv[] PROGMEM = "[alv] a_latch_velocity%17.3f%S/min\n";
static const char fmt_alb[] PROGMEM = "[alb] a_latch_backoff%18.3f%S\n";
static const char fmt_azb[] PROGMEM = "[azb] a_zero_backoff%19.3f%S\n";

static const char fmt_bam[] PROGMEM = "[bam] b_axis_mode%18d %S\n";
static const char fmt_bfr[] PROGMEM = "[bfr] b_feedrate_maximum%15.3f%S/min\n";
static const char fmt_bvm[] PROGMEM = "[bvm] b_velocity_maximum%15.3f%S/min\n";
static const char fmt_btm[] PROGMEM = "[btm] b_travel_maximum%17.3f%S\n";
static const char fmt_bjm[] PROGMEM = "[bjm] b_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_bjd[] PROGMEM = "[bjd] b_junction_deviation%14.4f%S\n";
static const char fmt_bra[] PROGMEM = "[bra] b_radius_value%20.4f%S\n";

static const char fmt_cam[] PROGMEM = "[cam] c_axis_mode%18d %S\n";
static const char fmt_cfr[] PROGMEM = "[cfr] c_feedrate_maximum%15.3f%S/min\n";
static const char fmt_cvm[] PROGMEM = "[cvm] c_velocity_maximum%15.3f%S/min\n";
static const char fmt_ctm[] PROGMEM = "[ctm] c_travel_maximum%17.3f%S\n";
static const char fmt_cjm[] PROGMEM = "[cjm] c_jerk_maximum%15.0f%S/min^3\n";
static const char fmt_cjd[] PROGMEM = "[cjd] c_junction_deviation%14.4f%S\n";
static const char fmt_cra[] PROGMEM = "[cra] c_radius_value%20.4f%S\n";

// PWM strings
static const char fmt_p1frq[] PROGMEM = "[p1frq] pwm_frequency   %15.3f Hz\n";
static const char fmt_p1csl[] PROGMEM = "[p1csl] pwm_cw_speed_lo %15.3f RPM\n";
static const char fmt_p1csh[] PROGMEM = "[p1csh] pwm_cw_speed_hi %15.3f RPM\n";
static const char fmt_p1cpl[] PROGMEM = "[p1cpl] pwm_cw_phase_lo %15.3f [0..1]\n";
static const char fmt_p1cph[] PROGMEM = "[p1cph] pwm_cw_phase_hi %15.3f [0..1]\n";
static const char fmt_p1wsl[] PROGMEM = "[p1wsl] pwm_ccw_speed_lo%15.3f RPM\n";
static const char fmt_p1wsh[] PROGMEM = "[p1wsh] pwm_ccw_speed_hi%15.3f RPM\n";
static const char fmt_p1wpl[] PROGMEM = "[p1wpl] pwm_ccw_phase_lo%15.3f [0..1]\n";
static const char fmt_p1wph[] PROGMEM = "[p1wph] pwm_ccw_phase_hi%15.3f [0..1]\n";
static const char fmt_p1pof[] PROGMEM = "[p1pof] pwm_phase_off   %15.3f [0..1]\n";

// Coordinate system offset groups
static const char fmt_g54x[] PROGMEM = "[g54x] g54_x_offset%20.3f%S\n";
static const char fmt_g54y[] PROGMEM = "[g54y] g54_y_offset%20.3f%S\n";
static const char fmt_g54z[] PROGMEM = "[g54z] g54_z_offset%20.3f%S\n";
static const char fmt_g54a[] PROGMEM = "[g54a] g54_a_offset%20.3f%S\n";
static const char fmt_g54b[] PROGMEM = "[g54b] g54_b_offset%20.3f%S\n";
static const char fmt_g54c[] PROGMEM = "[g54c] g54_c_offset%20.3f%S\n";

static const char fmt_g55x[] PROGMEM = "[g55x] g55_x_offset%20.3f%S\n";
static const char fmt_g55y[] PROGMEM = "[g55y] g55_y_offset%20.3f%S\n";
static const char fmt_g55z[] PROGMEM = "[g55z] g55_z_offset%20.3f%S\n";
static const char fmt_g55a[] PROGMEM = "[g55a] g55_a_offset%20.3f%S\n";
static const char fmt_g55b[] PROGMEM = "[g55b] g55_b_offset%20.3f%S\n";
static const char fmt_g55c[] PROGMEM = "[g55c] g55_c_offset%20.3f%S\n";

static const char fmt_g56x[] PROGMEM = "[g56x] g56_x_offset%20.3f%S\n";
static const char fmt_g56y[] PROGMEM = "[g56y] g56_y_offset%20.3f%S\n";
static const char fmt_g56z[] PROGMEM = "[g56z] g56_z_offset%20.3f%S\n";
static const char fmt_g56a[] PROGMEM = "[g56a] g56_a_offset%20.3f%S\n";
static const char fmt_g56b[] PROGMEM = "[g56b] g56_b_offset%20.3f%S\n";
static const char fmt_g56c[] PROGMEM = "[g56c] g56_c_offset%20.3f%S\n";

static const char fmt_g57x[] PROGMEM = "[g57x] g57_x_offset%20.3f%S\n";
static const char fmt_g57y[] PROGMEM = "[g57y] g57_y_offset%20.3f%S\n";
static const char fmt_g57z[] PROGMEM = "[g57z] g57_z_offset%20.3f%S\n";
static const char fmt_g57a[] PROGMEM = "[g57a] g57_a_offset%20.3f%S\n";
static const char fmt_g57b[] PROGMEM = "[g57b] g57_b_offset%20.3f%S\n";
static const char fmt_g57c[] PROGMEM = "[g57c] g57_c_offset%20.3f%S\n";

static const char fmt_g58x[] PROGMEM = "[g58x] g58_x_offset%20.3f%S\n";
static const char fmt_g58y[] PROGMEM = "[g58y] g58_y_offset%20.3f%S\n";
static const char fmt_g58z[] PROGMEM = "[g58z] g58_z_offset%20.3f%S\n";
static const char fmt_g58a[] PROGMEM = "[g58a] g58_a_offset%20.3f%S\n";
static const char fmt_g58b[] PROGMEM = "[g58b] g58_b_offset%20.3f%S\n";
static const char fmt_g58c[] PROGMEM = "[g58c] g58_c_offset%20.3f%S\n";

static const char fmt_g59x[] PROGMEM = "[g59x] g59_x_offset%20.3f%S\n";
static const char fmt_g59y[] PROGMEM = "[g59y] g59_y_offset%20.3f%S\n";
static const char fmt_g59z[] PROGMEM = "[g59z] g59_z_offset%20.3f%S\n";
static const char fmt_g59a[] PROGMEM = "[g59a] g59_a_offset%20.3f%S\n";
static const char fmt_g59b[] PROGMEM = "[g59b] g59_b_offset%20.3f%S\n";
static const char fmt_g59c[] PROGMEM = "[g59c] g59_c_offset%20.3f%S\n";

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

struct cfgItem const cfgArray[] PROGMEM = {
	// grp  token flags format*, print_func, get_func, set_func  target for get/set,   default value
	{ "sys","fb", _fip, fmt_fb, _print_dbl, _get_dbl, _set_nul, (double *)&cfg.fw_build,   TINYG_BUILD_NUMBER }, // MUST BE FIRST!
	{ "sys","fv", _fip, fmt_fv, _print_dbl, _get_dbl, _set_nul, (double *)&cfg.fw_version, TINYG_VERSION_NUMBER },
	{ "sys","hv", _fip, fmt_hv, _print_dbl, _get_dbl, _set_hv,  (double *)&cfg.hw_version, TINYG_HARDWARE_VERSION },
//	{ "",   "id", _f00, fmt_id, _print_int, _get_id,  _set_nul, (double *)&tg.null, 0},		// device ID (signature)

	// dynamic model attributes for reporting puropses (see also G92 offsets)
	{ "", "n",   _fin, fmt_line,_print_int, _get_int, _set_int, (double *)&gm.linenum,0 },	// Gcode line number - gets model line number
	{ "", "line",_fin, fmt_line,_print_int, _get_line,_set_int, (double *)&gm.linenum,0 },	// Gcode line number - gets runtime line number
	{ "", "feed",_f00, fmt_feed,_print_lin, _get_dbu, _set_nul, (double *)&tg.null, 0 },	// feed rate
	{ "", "stat",_f00, fmt_stat,_print_str, _get_stat,_set_nul, (double *)&tg.null, 0 },	// combined machine state
	{ "", "macs",_f00, fmt_macs,_print_str, _get_macs,_set_nul, (double *)&tg.null, 0 },	// raw machine state
	{ "", "cycs",_f00, fmt_cycs,_print_str, _get_cycs,_set_nul, (double *)&tg.null, 0 },	// cycle state
	{ "", "mots",_f00, fmt_mots,_print_str, _get_mots,_set_nul, (double *)&tg.null, 0 },	// motion state
	{ "", "hold",_f00, fmt_hold,_print_str, _get_hold,_set_nul, (double *)&tg.null, 0 },	// feedhold state
	{ "", "vel", _f00, fmt_vel, _print_lin, _get_vel, _set_nul, (double *)&tg.null, 0 },	// current velocity
	{ "", "unit",_f00, fmt_unit,_print_str, _get_unit,_set_nul, (double *)&tg.null, 0 },	// units mode
	{ "", "coor",_f00, fmt_coor,_print_str, _get_coor,_set_nul, (double *)&tg.null, 0 },	// coordinate system
	{ "", "momo",_f00, fmt_momo,_print_str, _get_momo,_set_nul, (double *)&tg.null, 0 },	// motion mode
	{ "", "plan",_f00, fmt_plan,_print_str, _get_plan,_set_nul, (double *)&tg.null, 0 },	// plane select
	{ "", "path",_f00, fmt_path,_print_str, _get_path,_set_nul, (double *)&tg.null, 0 },	// path control mode
	{ "", "dist",_f00, fmt_dist,_print_str, _get_dist,_set_nul, (double *)&tg.null, 0 },	// distance mode
	{ "", "frmo",_f00, fmt_frmo,_print_str, _get_frmo,_set_nul, (double *)&tg.null, 0 },	// feed rate mode
	{ "", "posx",_f00, fmt_posx,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// X position
	{ "", "posy",_f00, fmt_posy,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// Y position
	{ "", "posz",_f00, fmt_posz,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// Z position
	{ "", "posa",_f00, fmt_posa,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// A position
	{ "", "posb",_f00, fmt_posb,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// B position
	{ "", "posc",_f00, fmt_posc,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// C position
	{ "", "mpox",_f00, fmt_mpox,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// X machine position
	{ "", "mpoy",_f00, fmt_mpoy,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// Y machine position
	{ "", "mpoz",_f00, fmt_mpoz,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// Z machine position
	{ "", "mpoa",_f00, fmt_mpoa,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// A machine position
	{ "", "mpob",_f00, fmt_mpob,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// B machine position
	{ "", "mpoc",_f00, fmt_mpoc,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// C machine position
	{ "", "home",_f00, fmt_home,_print_str, _get_home,_set_nul, (double *)&tg.null, 0 },	// homing state
	{ "hom","homx",_f00,fmt_homx,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[X], false }, // X homed
	{ "hom","homy",_f00,fmt_homy,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[Y], false },	// Y homed
	{ "hom","homz",_f00,fmt_homz,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[Z], false },	// Z homed
	{ "hom","homa",_f00,fmt_homa,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[A], false },	// A homed
	{ "hom","homb",_f00,fmt_homb,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[B], false },	// B homed
	{ "hom","homc",_f00,fmt_homc,_print_int,_get_ui8, _set_nul, (double *)&cm.homed[C], false },	// C homed
// 38
	// Reports, tests, help, and messages
	{ "", "sr",  _f00, fmt_nul, _print_sr,  _get_sr,  _set_sr,  (double *)&tg.null, 0 },	// status report object
	{ "", "qr",  _f00, fmt_nul, _print_nul, _get_qr,  _set_nul, (double *)&tg.null, 0 },	// queue report setting
	{ "", "rx",  _f00, fmt_nul, _print_int, _get_rx,  _set_nul, (double *)&tg.null, 0 },	// space in RX buffer
	{ "", "msg", _f00, fmt_str, _print_str, _get_nul, _set_nul, (double *)&tg.null, 0 },	// string for generic messages
	{ "", "test",_f00, fmt_nul, _print_nul, print_test_help, tg_test, (double *)&tg.test,0 },// prints test help screen
	{ "", "defa",_f00, fmt_nul, _print_nul, print_defaults_help,_set_defa,(double *)&tg.null,0},// prints defaults help screen
	{ "", "help",_f00, fmt_nul, _print_nul, print_config_help,_set_nul, (double *)&tg.null,0 },// prints config help screen
	{ "", "h",   _f00, fmt_nul, _print_nul, print_config_help,_set_nul, (double *)&tg.null,0 },
// 46
	// System parameters
	// NOTE: The ordering within the gcode defaults is important for token resolution
	// NOTE: Some values have been removed from the group display but are still accessible as individual elements
	{ "sys","gpl", _fip, fmt_gpl, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.select_plane,		GCODE_DEFAULT_PLANE },
	{ "sys","gun", _fip, fmt_gun, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.units_mode,			GCODE_DEFAULT_UNITS },
	{ "sys","gco", _fip, fmt_gco, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.coord_system,		GCODE_DEFAULT_COORD_SYSTEM },
	{ "sys","gpa", _fip, fmt_gpa, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.path_control,		GCODE_DEFAULT_PATH_CONTROL },
	{ "sys","gdi", _fip, fmt_gdi, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.distance_mode,		GCODE_DEFAULT_DISTANCE_MODE },
	{ "",   "gc",  _f00, fmt_gc,  _print_nul, _get_gc, _run_gc,  (double *)&tg.null, 0 }, // gcode block - must be last in this group

	{ "sys","ja",  _fip, fmt_ja, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.junction_acceleration,JUNCTION_ACCELERATION },
	{ "",   "ml",  _fip, fmt_ml, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.min_segment_len,	MIN_LINE_LENGTH },
	{ "",   "ma",  _fip, fmt_ma, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.arc_segment_len,	ARC_SEGMENT_LENGTH },
	{ "",   "mt",  _fip, fmt_mt, _print_lin, _get_dbl, _set_dbl, (double *)&cfg.estd_segment_usec,	NOM_SEGMENT_USEC },
	{ "sys","st",  _fip, fmt_st, _print_ui8, _get_ui8, _set_sw,  (double *)&sw.switch_type,			SWITCH_TYPE },

	{ "sys","ic",  _fip, fmt_ic, _print_ui8, _get_ui8, _set_ic,  (double *)&cfg.ignore_crlf,		COM_IGNORE_CRLF },
//	{ "sys","ec",  _fip, fmt_ec, _print_ui8, _get_ui8, _set_ec,  (double *)&cfg.enable_cr,			COM_APPEND_TX_CR },
	{ "sys","ee",  _fip, fmt_ee, _print_ui8, _get_ui8, _set_ee,  (double *)&cfg.enable_echo,		COM_ENABLE_ECHO },
	{ "sys","ex",  _fip, fmt_ex, _print_ui8, _get_ui8, _set_ex,  (double *)&cfg.enable_xon,			COM_ENABLE_XON },
	{ "",   "eqh", _fip, fmt_ui8,_print_ui8, _get_ui8, _set_ui8, (double *)&cfg.qr_hi_water, 		COM_QR_HI_WATER },
	{ "",   "eql", _fip, fmt_ui8,_print_ui8, _get_ui8, _set_ui8, (double *)&cfg.qr_lo_water, 		COM_QR_LO_WATER },
	{ "sys","eq",  _fip, fmt_eq, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.enable_qr,			COM_ENABLE_QR },
	{ "sys","ej",  _fip, fmt_ej, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.comm_mode,			COM_COMM_MODE },
	{ "sys","jv",  _fip, fmt_jv, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.json_verbosity,		COM_JSON_VERBOSITY },
	{ "sys","tv",  _fip, fmt_tv, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.text_verbosity,		COM_TEXT_VERBOSITY },
	{ "sys","si",  _fip, fmt_si, _print_dbl, _get_int, _set_si,  (double *)&cfg.status_report_interval,STATUS_REPORT_INTERVAL_MS },
	{ "sys","baud",_f00, fmt_baud,_print_ui8,_get_ui8, _set_baud,(double *)&cfg.usb_baud_rate,		XIO_BAUD_115200 },
// 68
	// Motor parameters
	{ "1","1ma",_fip, fmt_1ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].motor_map,	M1_MOTOR_MAP },
	{ "1","1sa",_fip, fmt_1sa, _print_rot, _get_dbl ,_set_sa, (double *)&cfg.m[MOTOR_1].step_angle,	M1_STEP_ANGLE },
	{ "1","1tr",_fip, fmt_1tr, _print_lin, _get_dbu ,_set_tr, (double *)&cfg.m[MOTOR_1].travel_rev,	M1_TRAVEL_PER_REV },
	{ "1","1mi",_fip, fmt_1mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_1].microsteps,	M1_MICROSTEPS },
	{ "1","1po",_fip, fmt_1po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_1].polarity,	M1_POLARITY },
	{ "1","1pm",_fip, fmt_1pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].power_mode,	M1_POWER_MODE },

	{ "2","2ma",_fip, fmt_2ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].motor_map,	M2_MOTOR_MAP },
	{ "2","2sa",_fip, fmt_2sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_2].step_angle,	M2_STEP_ANGLE },
	{ "2","2tr",_fip, fmt_2tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_2].travel_rev,	M2_TRAVEL_PER_REV },
	{ "2","2mi",_fip, fmt_2mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_2].microsteps,	M2_MICROSTEPS },
	{ "2","2po",_fip, fmt_2po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_2].polarity,	M2_POLARITY },
	{ "2","2pm",_fip, fmt_2pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].power_mode,	M2_POWER_MODE },

	{ "3","3ma",_fip, fmt_3ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].motor_map,	M3_MOTOR_MAP },
	{ "3","3sa",_fip, fmt_3sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_3].step_angle,	M3_STEP_ANGLE },
	{ "3","3tr",_fip, fmt_3tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_3].travel_rev,	M3_TRAVEL_PER_REV },
	{ "3","3mi",_fip, fmt_3mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_3].microsteps,	M3_MICROSTEPS },
	{ "3","3po",_fip, fmt_3po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_3].polarity,	M3_POLARITY },
	{ "3","3pm",_fip, fmt_3pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].power_mode,	M3_POWER_MODE },

	{ "4","4ma",_fip, fmt_4ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].motor_map,	M4_MOTOR_MAP },
	{ "4","4sa",_fip, fmt_4sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_4].step_angle,	M4_STEP_ANGLE },
	{ "4","4tr",_fip, fmt_4tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_4].travel_rev,	M4_TRAVEL_PER_REV },
	{ "4","4mi",_fip, fmt_4mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_4].microsteps,	M4_MICROSTEPS },
	{ "4","4po",_fip, fmt_4po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_4].polarity,	M4_POLARITY },
	{ "4","4pm",_fip, fmt_4pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].power_mode,	M4_POWER_MODE },
// 92
	// Axis parameters
	{ "x","xam",_fip, fmt_xam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[X].axis_mode,		X_AXIS_MODE },
	{ "x","xvm",_fip, fmt_xvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].velocity_max,		X_VELOCITY_MAX },
	{ "x","xfr",_fip, fmt_xfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].feedrate_max,		X_FEEDRATE_MAX },
	{ "x","xtm",_fip, fmt_xtm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].travel_max,		X_TRAVEL_MAX },
	{ "x","xjm",_fip, fmt_xjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].jerk_max,			X_JERK_MAX },
	{ "x","xjd",_fip, fmt_xjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].junction_dev,		X_JUNCTION_DEVIATION },
	{ "x","xsn",_fip, fmt_xsn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[0],				X_SWITCH_MODE_MIN },
	{ "x","xsx",_fip, fmt_xsx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[1],				X_SWITCH_MODE_MAX },
	{ "x","xsv",_fip, fmt_xsv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].search_velocity,	X_SEARCH_VELOCITY },
	{ "x","xlv",_fip, fmt_xlv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].latch_velocity,	X_LATCH_VELOCITY },
	{ "x","xlb",_fip, fmt_xlb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].latch_backoff,	X_LATCH_BACKOFF },
	{ "x","xzb",_fip, fmt_xzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].zero_backoff,		X_ZERO_BACKOFF },

	{ "y","yam",_fip, fmt_yam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[Y].axis_mode,		Y_AXIS_MODE },
	{ "y","yvm",_fip, fmt_yvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].velocity_max,		Y_VELOCITY_MAX },
	{ "y","yfr",_fip, fmt_yfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].feedrate_max,		Y_FEEDRATE_MAX },
	{ "y","ytm",_fip, fmt_ytm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].travel_max,		Y_TRAVEL_MAX },
	{ "y","yjm",_fip, fmt_yjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].jerk_max,			Y_JERK_MAX },
	{ "y","yjd",_fip, fmt_yjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].junction_dev,		Y_JUNCTION_DEVIATION },
	{ "y","ysn",_fip, fmt_ysn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[2],				Y_SWITCH_MODE_MIN },
	{ "y","ysx",_fip, fmt_ysx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[3],				Y_SWITCH_MODE_MAX },
	{ "y","ysv",_fip, fmt_ysv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].search_velocity,	Y_SEARCH_VELOCITY },
	{ "y","ylv",_fip, fmt_ylv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].latch_velocity,	Y_LATCH_VELOCITY },
	{ "y","ylb",_fip, fmt_ylb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].latch_backoff,	Y_LATCH_BACKOFF },
	{ "y","yzb",_fip, fmt_yzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].zero_backoff,		Y_ZERO_BACKOFF },

	{ "z","zam",_fip, fmt_zam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[Z].axis_mode,		Z_AXIS_MODE },
	{ "z","zvm",_fip, fmt_zvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].velocity_max,	 	Z_VELOCITY_MAX },
	{ "z","zfr",_fip, fmt_zfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].feedrate_max,	 	Z_FEEDRATE_MAX },
	{ "z","ztm",_fip, fmt_ztm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].travel_max,		Z_TRAVEL_MAX },
	{ "z","zjm",_fip, fmt_zjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].jerk_max,			Z_JERK_MAX },
	{ "z","zjd",_fip, fmt_zjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].junction_dev,	 	Z_JUNCTION_DEVIATION },
	{ "z","zsn",_fip, fmt_zsn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[4],				Z_SWITCH_MODE_MIN },
	{ "z","zsx",_fip, fmt_zsx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[5],				Z_SWITCH_MODE_MAX },
	{ "z","zsv",_fip, fmt_zsv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].search_velocity,	Z_SEARCH_VELOCITY },
	{ "z","zlv",_fip, fmt_zlv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].latch_velocity,	Z_LATCH_VELOCITY },
	{ "z","zlb",_fip, fmt_zlb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].latch_backoff,	Z_LATCH_BACKOFF },
	{ "z","zzb",_fip, fmt_zzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].zero_backoff,		Z_ZERO_BACKOFF },
// 128
	{ "a","aam",_fip, fmt_aam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[A].axis_mode,		A_AXIS_MODE },
	{ "a","avm",_fip, fmt_avm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].velocity_max,	 	A_VELOCITY_MAX },
	{ "a","afr",_fip, fmt_afr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].feedrate_max, 	A_FEEDRATE_MAX },
	{ "a","atm",_fip, fmt_atm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].travel_max,		A_TRAVEL_MAX },
	{ "a","ajm",_fip, fmt_ajm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].jerk_max,			A_JERK_MAX },
	{ "a","ajd",_fip, fmt_ajd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].junction_dev,	 	A_JUNCTION_DEVIATION },
	{ "a","ara",_fip, fmt_ara, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].radius,			A_RADIUS},
	{ "a","asn",_fip, fmt_asn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[6],				A_SWITCH_MODE_MIN },
	{ "a","asx",_fip, fmt_asx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[7],				A_SWITCH_MODE_MAX },
	{ "a","asv",_fip, fmt_asv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].search_velocity,	A_SEARCH_VELOCITY },
	{ "a","alv",_fip, fmt_alv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].latch_velocity,	A_LATCH_VELOCITY },
	{ "a","alb",_fip, fmt_alb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].latch_backoff,	A_LATCH_BACKOFF },
	{ "a","azb",_fip, fmt_azb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].zero_backoff,		A_ZERO_BACKOFF },
// 141
	{ "b","bam",_fip, fmt_bam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[B].axis_mode,		B_AXIS_MODE },
	{ "b","bvm",_fip, fmt_bvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].velocity_max,	 	B_VELOCITY_MAX },
	{ "b","bfr",_fip, fmt_bfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].feedrate_max, 	B_FEEDRATE_MAX },
	{ "b","btm",_fip, fmt_btm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].travel_max,		B_TRAVEL_MAX },
	{ "b","bjm",_fip, fmt_bjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].jerk_max,			B_JERK_MAX },
	{ "b","bjd",_fip, fmt_bjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].junction_dev,	 	B_JUNCTION_DEVIATION },
	{ "b","bra",_fip, fmt_bra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].radius,			B_RADIUS },

	{ "c","cam",_fip, fmt_cam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[C].axis_mode,		C_AXIS_MODE },
	{ "c","cvm",_fip, fmt_cvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].velocity_max,	 	C_VELOCITY_MAX },
	{ "c","cfr",_fip, fmt_cfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].feedrate_max, 	C_FEEDRATE_MAX },
	{ "c","ctm",_fip, fmt_ctm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].travel_max,		C_TRAVEL_MAX },
	{ "c","cjm",_fip, fmt_cjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].jerk_max,			C_JERK_MAX },
	{ "c","cjd",_fip, fmt_cjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].junction_dev,		C_JUNCTION_DEVIATION },
	{ "c","cra",_fip, fmt_cra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].radius,			C_RADIUS },
// 155
	// PWM settings
    { "p1","p1frq",_fip, fmt_p1frq, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.frequency,		P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, fmt_p1csl, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.cw_speed_lo,	P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, fmt_p1csh, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.cw_speed_hi,	P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, fmt_p1cpl, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.cw_phase_lo,	P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, fmt_p1cph, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.cw_phase_hi,	P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, fmt_p1wsl, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_speed_lo,	P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, fmt_p1wsh, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_speed_hi,	P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, fmt_p1wpl, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_phase_lo,	P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, fmt_p1wph, _print_dbl, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_phase_hi,	P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, fmt_p1pof, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.phase_off,		P1_PWM_PHASE_OFF },
// 165
	// Coordinate system offsets (G54-G59 and G92)
	{ "g54","g54x",_fip, fmt_g54x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][X],	G54_X_OFFSET },
	{ "g54","g54y",_fip, fmt_g54y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][Y],	G54_Y_OFFSET },
	{ "g54","g54z",_fip, fmt_g54z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][Z],	G54_Z_OFFSET },
	{ "g54","g54a",_fip, fmt_g54a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][A],	G54_A_OFFSET },
	{ "g54","g54b",_fip, fmt_g54b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][B],	G54_B_OFFSET },
	{ "g54","g54c",_fip, fmt_g54c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][C],	G54_C_OFFSET },

	{ "g55","g55x",_fip, fmt_g55x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][X],	G55_X_OFFSET },
	{ "g55","g55y",_fip, fmt_g55y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][Y],	G55_Y_OFFSET },
	{ "g55","g55z",_fip, fmt_g55z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][Z],	G55_Z_OFFSET },
	{ "g55","g55a",_fip, fmt_g55a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][A],	G55_A_OFFSET },
	{ "g55","g55b",_fip, fmt_g55b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][B],	G55_B_OFFSET },
	{ "g55","g55c",_fip, fmt_g55c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][C],	G55_C_OFFSET },

	{ "g56","g56x",_fip, fmt_g56x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][X],	G56_X_OFFSET },
	{ "g56","g56y",_fip, fmt_g56y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][Y],	G56_Y_OFFSET },
	{ "g56","g56z",_fip, fmt_g56z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][Z],	G56_Z_OFFSET },
	{ "g56","g56a",_fip, fmt_g56a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][A],	G56_A_OFFSET },
	{ "g56","g56b",_fip, fmt_g56b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][B],	G56_B_OFFSET },
	{ "g56","g56c",_fip, fmt_g56c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][C],	G56_C_OFFSET },

	{ "g57","g57x",_fip, fmt_g57x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][X],	G57_X_OFFSET },
	{ "g57","g57y",_fip, fmt_g57y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][Y],	G57_Y_OFFSET },
	{ "g57","g57z",_fip, fmt_g57z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][Z],	G57_Z_OFFSET },
	{ "g57","g57a",_fip, fmt_g57a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][A],	G57_A_OFFSET },
	{ "g57","g57b",_fip, fmt_g57b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][B],	G57_B_OFFSET },
	{ "g57","g57c",_fip, fmt_g57c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][C],	G57_C_OFFSET },

	{ "g58","g58x",_fip, fmt_g58x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][X],	G58_X_OFFSET },
	{ "g58","g58y",_fip, fmt_g58y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][Y],	G58_Y_OFFSET },
	{ "g58","g58z",_fip, fmt_g58z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][Z],	G58_Z_OFFSET },
	{ "g58","g58a",_fip, fmt_g58a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][A],	G58_A_OFFSET },
	{ "g58","g58b",_fip, fmt_g58b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][B],	G58_B_OFFSET },
	{ "g58","g58c",_fip, fmt_g58c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][C],	G58_C_OFFSET },

	{ "g59","g59x",_fip, fmt_g59x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][X],	G59_X_OFFSET },
	{ "g59","g59y",_fip, fmt_g59y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][Y],	G59_Y_OFFSET },
	{ "g59","g59z",_fip, fmt_g59z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][Z],	G59_Z_OFFSET },
	{ "g59","g59a",_fip, fmt_g59a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][A],	G59_A_OFFSET },
	{ "g59","g59b",_fip, fmt_g59b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][B],	G59_B_OFFSET },
	{ "g59","g59c",_fip, fmt_g59c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][C],	G59_C_OFFSET },

	{ "g92","g92x",_fin, fmt_g92x,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[X], 0 },// G92 handled differently
	{ "g92","g92y",_fin, fmt_g92y,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[Y], 0 },
	{ "g92","g92z",_fin, fmt_g92z,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[Z], 0 },
	{ "g92","g92a",_fin, fmt_g92a,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[A], 0 },
	{ "g92","g92b",_fin, fmt_g92b,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[B], 0 },
	{ "g92","g92c",_fin, fmt_g92c,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[C], 0 },
// 205
	// Persistence for status report - must be in sequence
	// *** Count must agree with CMD_STATUS_REPORT_LEN in config.h ***
	{ "","se00",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[0],0 },
	{ "","se01",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[1],0 },
	{ "","se02",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[2],0 },
	{ "","se03",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[3],0 },
	{ "","se04",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[4],0 },
	{ "","se05",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[5],0 },
	{ "","se06",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[6],0 },
	{ "","se07",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[7],0 },
	{ "","se08",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[8],0 },
	{ "","se09",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[9],0 },
	{ "","se10",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[10],0 },
	{ "","se11",_fpe, fmt_nul, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_list[11],0 },
// 217
	// Group lookups - must follow the single-valued entries for proper sub-string matching
	// *** Must agree with CMD_COUNT_GROUPS below ****
	{ "","sys",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// system group
	{ "","p1", _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// PWM 1 group
	{ "","1",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// motor groups
	{ "","2",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","3",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","4",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","x",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// axis groups
	{ "","y",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","z",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","a",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","b",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","c",  _f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g54",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// coord offset groups
	{ "","g55",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g56",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g57",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g58",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g59",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ "","g92",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// origin offsets
	{ "","hom",_f00, fmt_nul, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// axis homing state
// 236
	// Uber-group (groups of groups, for text-mode displays only)
	// *** Must agree with CMD_COUNT_UBER_GROUPS below ****
	{ "", "m", _f00, fmt_nul, _print_nul, _do_motors, _set_nul,(double *)&tg.null,0 },
	{ "", "q", _f00, fmt_nul, _print_nul, _do_axes,   _set_nul,(double *)&tg.null,0 },
	{ "", "o", _f00, fmt_nul, _print_nul, _do_offsets,_set_nul,(double *)&tg.null,0 },
	{ "", "$", _f00, fmt_nul, _print_nul, _do_all,    _set_nul,(double *)&tg.null,0 }
};
// 240
#define CMD_COUNT_GROUPS 		20											// count of simple groups
#define CMD_COUNT_UBER_GROUPS 	4 											// count of uber-groups

#define CMD_INDEX_MAX (sizeof cfgArray / sizeof(struct cfgItem))
#define CMD_INDEX_END_SINGLES		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS - CMD_STATUS_REPORT_LEN)
#define CMD_INDEX_START_GROUPS		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS)
#define CMD_INDEX_START_UBER_GROUPS (CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS)

#define _index_is_single(i) ((i <= CMD_INDEX_END_SINGLES) ? true : false)	// Evaluators
#define _index_lt_groups(i) ((i <= CMD_INDEX_START_GROUPS) ? true : false)
#define _index_is_group(i) (((i >= CMD_INDEX_START_GROUPS) && (i < CMD_INDEX_START_UBER_GROUPS)) ? true : false)
#define _index_is_uber(i)   ((i >= CMD_INDEX_START_UBER_GROUPS) ? true : false)
#define _index_is_group_or_uber(i) ((i >= CMD_INDEX_START_GROUPS) ? true : false)

/**** VERSIONS AND IDs ****
 * _set_hv() - set hardweare version number
 * _get_id() - get device ID (signature)
 */
static uint8_t _set_hv(cmdObj *cmd) 
{
	_set_dbl(cmd);					// record the hardware version
	sys_port_bindings(cmd->value);	// reset port bindings
	gpio_init();					// re-initialize the GPIO ports
	return (TG_OK);
}

/*
static uint8_t _get_id(cmdObj *cmd) 
{
 *	 - group of all other groups
//	sys_read_signature(sig);
	return (TG_OK);
}
*/

/**** STATUS REPORT REPORT FUNCTIONS ****
 * _get_sr()   - run status report
 * _print_sr() - run status report
 * _set_sr()   - set status report specification
 * _set_si()   - set status report interval
 */
static uint8_t _get_sr(cmdObj *cmd)
{
	rpt_populate_status_report();
	return (TG_OK);
}

static void _print_sr(cmdObj *cmd)
{
	rpt_populate_status_report();
}

static uint8_t _set_sr(cmdObj *cmd)
{
	memset(cfg.status_report_list, 0 , sizeof(cfg.status_report_list));
	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		cfg.status_report_list[i] = cmd->index;
		cmd->value = cmd->index;	// you want to persist the index as the value
		cmd_persist(cmd);
	}
	return (TG_OK);
}

static uint8_t _set_si(cmdObj *cmd) 
{
	if ((cmd->value < STATUS_REPORT_MIN_MS) && (cmd->value!=0)) {
		cmd->value = STATUS_REPORT_MIN_MS;
	}
	cfg.status_report_interval = (uint32_t)cmd->value;
	return(TG_OK);
}

/**** REPORTING FUNCTIONS ****
 * _get_qr() - run queue report
 * _get_rx() - get bytes available in RX buffer
 */
static uint8_t _get_qr(cmdObj *cmd) 
{
	rpt_run_queue_report();
	return (TG_OK);
}

static uint8_t _get_rx(cmdObj *cmd)
{
	cmd->value = (double)xio_get_usb_rx_free();
	cmd->type = TYPE_INTEGER;
	return (TG_OK);
}

/**** STATUS REPORT FUNCTIONS ****************************************
 * _get_msg_helper() - helper to get display message
 * _get_stat() - get combined machine state as value and string
 * _get_macs() - get raw machine state as value and string
 * _get_cycs() - get raw cycle state as value and string
 * _get_mots() - get raw motion state as value and string
 * _get_hold() - get raw hold state as value and string
 * _get_home() - get raw homing state as value and string
 * _get_unit() - get units mode as string
 * _get_coor() - get goodinate system as string
 * _get_momo() - get motion mode as string
 * _get_plan() - get gcode plane select as string
 * _get_path() - get gcode path control mode as string
 * _get_dist() - get gcode distance mode as string
 * _get_frmo() - get gcode feed rate mode as string
 * _get_feed() - get feed rate 
 * _get_line() - get runtime line number for status reports
 * _get_vel()  - get runtime velocity
 * _get_pos()  - get runtime work position
 * _get_mpos() - get runtime machine position
 * _print_pos()- print work or machine position
 */
static uint8_t _get_msg_helper(cmdObj *cmd, prog_char_ptr msg, uint8_t value)
{
	cmd->value = (double)value;
	cmd->type = TYPE_INTEGER;
	strncpy_P(cmd->string, (PGM_P)pgm_read_word(&msg[value*2]), CMD_STRING_LEN); // hack alert: direct computation of index
	return (TG_OK);
//	return((char *)pgm_read_word(&msg[(uint8_t)value]));
}

static uint8_t _get_stat(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_stat, cm_get_combined_state()));

/* how to do this w/o calling the helper routine - See 331.09 for original routines
	cmd->value = cm_get_machine_state();
	cmd->type = TYPE_INTEGER;
	strncpy_P(cmd->string_value,(PGM_P)pgm_read_word(&msg_stat[(uint8_t)cmd->value]),CMD_STRING_LEN);
	return (TG_OK);
 */
}

static uint8_t _get_macs(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_macs, cm_get_machine_state()));
}

static uint8_t _get_cycs(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_cycs, cm_get_cycle_state()));
}

static uint8_t _get_mots(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_mots, cm_get_motion_state()));
}

static uint8_t _get_hold(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_hold, cm_get_hold_state()));
}

static uint8_t _get_home(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_home, cm_get_homing_state()));
}

static uint8_t _get_unit(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_unit, cm_get_units_mode()));
}

static uint8_t _get_coor(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_coor, cm_get_coord_system()));
}

static uint8_t _get_momo(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_momo, cm_get_motion_mode()));
}

static uint8_t _get_plan(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_plan, cm_get_select_plane()));
}

static uint8_t _get_path(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_path, cm_get_path_control()));
}

static uint8_t _get_dist(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_dist, cm_get_distance_mode()));
}

static uint8_t _get_frmo(cmdObj *cmd)
{
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_frmo, cm_get_inverse_feed_rate_mode()));
}

static uint8_t _get_line(cmdObj *cmd)
{
	cmd->value = (double)mp_get_runtime_linenum();
	cmd->type = TYPE_INTEGER;
	return (TG_OK);
}

static uint8_t _get_vel(cmdObj *cmd) 
{
	cmd->value = mp_get_runtime_velocity();
	if (cm_get_units_mode() == INCHES) cmd->value *= INCH_PER_MM;
	cmd->type = TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_pos(cmdObj *cmd) 
{
	cmd->value = cm_get_runtime_work_position(_get_pos_axis(cmd->index));
	cmd->type = TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_mpos(cmdObj *cmd) 
{
	cmd->value = cm_get_runtime_machine_position(_get_pos_axis(cmd->index));
	cmd->type = TYPE_FLOAT;
	return (TG_OK);
}

static void _print_pos(cmdObj *cmd)
{
	cmd_get(cmd);
	uint8_t axis = _get_pos_axis(cmd->index);
	uint8_t units = DEGREES;	// rotary
	char format[CMD_FORMAT_LEN+1];
	if (axis < A) { 
		units = cm_get_units_mode();
	}
	fprintf(stderr, _get_format(cmd->index,format), cmd->value, (PGM_P)pgm_read_word(&msg_units[(uint8_t)units]));
}

/**** GCODE FUNCTIONS ****************************************/
/* _get_gc() - get gcode block
 * _run_gc() - launch the gcode parser on a block of gcode
 */

static uint8_t _get_gc(cmdObj *cmd)
{
	strncpy(cmd->string, tg.in_buf, CMD_STRING_LEN);
	cmd->type = TYPE_STRING;
	return (TG_OK);
}

static uint8_t _run_gc(cmdObj *cmd)
{
	strncpy(tg.in_buf, cmd->string, INPUT_BUFFER_LEN);
	uint8_t status = gc_gcode_parser(tg.in_buf);
	return (status);
}

/**** AXIS AND MOTOR FUNCTIONS ****
 * _get_am() - get axis mode w/enumeration string
 * _set_am() - set axis mode w/exception handling for axis type
 * _print_am() - print axis mode w/enumeration string
 * _set_sw() - run this any time you change a switch setting	
 * _set_sa() - set motor step_angle & recompute steps_per_unit
 * _set_tr() - set motor travel_per_rev & recompute steps_per_unit
 * _set_mi() - set microsteps & recompute steps_per_unit
 * _set_po() - set polarity and update stepper structs
 * _set_motor_steps_per_unit() - update this derived value
 *	 This function will need to be rethought if microstep morphing is implemented
 */
static uint8_t _get_am(cmdObj *cmd)
{
	_get_ui8(cmd);
	return(_get_msg_helper(cmd, (prog_char_ptr)msg_am, cmd->value)); // see 331.09 for old method
}

static uint8_t _set_am(cmdObj *cmd)		// axis mode
{
	char linear_axes[] = {"xyz"};

	if (strchr(linear_axes, cmd->group[0]) != NULL) {		// true if it's a linear axis
		if (cmd->value > AXIS_MAX_LINEAR) {
			cmd->value = 0;
			char message[CMD_STRING_LEN]; 
			sprintf_P(message, PSTR("*** WARNING *** Unsupported linear axis mode. Axis DISABLED"));
			cmd_add_string("msg",message);
		//  The following method saves FLASH at the expense of RAM size:
		//	cmd_add_string("msg","*** WARNING *** Unsupported linear axis mode. Axis DISABLED");
		}
	} else {
		if (cmd->value > AXIS_MAX_ROTARY) {
			cmd->value = 0;
			char message[CMD_STRING_LEN]; 
			sprintf_P(message, PSTR("*** WARNING *** Unsupported rotary axis mode. Axis DISABLED"));
			cmd_add_string("msg",message);
		//	cmd_add_string("msg","*** WARNING *** Unsupported rotary axis mode. Axis DISABLED");
		}
	}
	_set_ui8(cmd);
	return(TG_OK);
}

static void _print_am(cmdObj *cmd)		// axis mode
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), (uint8_t)cmd->value, (PGM_P)pgm_read_word(&msg_am[(uint8_t)cmd->value]));
}

static uint8_t _set_sw(cmdObj *cmd)		// switch setting
{ 
	_set_ui8(cmd);
	gpio_init();
	return (TG_OK);
}

static uint8_t _set_sa(cmdObj *cmd)		// motor step angle
{ 
	_set_dbl(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (TG_OK);
}

static uint8_t _set_tr(cmdObj *cmd)		// motor travel per revolution
{ 
	_set_dbu(cmd);
	_set_motor_steps_per_unit(cmd); 
	return (TG_OK);
}

static uint8_t _set_mi(cmdObj *cmd)		// motor microsteps
{
	if (fp_NE(cmd->value,1) && fp_NE(cmd->value,2) && fp_NE(cmd->value,4) && fp_NE(cmd->value,8)) {
		char message[CMD_STRING_LEN]; 
		sprintf_P(message, PSTR("*** WARNING *** Non-standard microstep value"));
		cmd_add_string("msg",message);
	//	cmd_add_string("msg","*** WARNING *** Non-standard microstep value");
	}
	_set_ui8(cmd);						// but set it anyway, even if it's unsupported
	_set_motor_steps_per_unit(cmd);
	st_set_microsteps(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (TG_OK);
}

static uint8_t _set_po(cmdObj *cmd)		// motor polarity
{ 
	_set_ui8(cmd);
	st_set_polarity(_get_motor(cmd->index), (uint8_t)cmd->value);
	return (TG_OK);
}

static uint8_t _set_motor_steps_per_unit(cmdObj *cmd)
{
	uint8_t m = _get_motor(cmd->index);
	cfg.m[m].steps_per_unit = (360 / (cfg.m[m].step_angle / cfg.m[m].microsteps) / cfg.m[m].travel_rev);
	return (TG_OK);
}

/**** SERIAL IO SETTINGS ****
 * _set_ic() - ignore CR or LF on RX
 * _set_ec() - enable CRLF on TX
 * _set_ee() - enable character echo
 * _set_ex() - enable XON/XOFF
 * _set_baud() - set USB baud rate
 *	The above assume USB is the std device
 */

static uint8_t _set_comm_helper(cmdObj *cmd, uint32_t yes, uint32_t no)
{
	if (fp_NOT_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, yes);
	} else { 
		(void)xio_cntl(XIO_DEV_USB, no);
	}
	return (TG_OK);
}

static uint8_t _set_ic(cmdObj *cmd) 	// ignore CR or LF on RX
{
	cfg.ignore_crlf = (uint8_t)cmd->value;
	(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORECR);	// clear them both
	(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORELF);

	if (cfg.ignore_crlf == IGNORE_CR) {
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORECR);
	} else if (cfg.ignore_crlf == IGNORE_LF) {
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORELF);
	}
	return (TG_OK);
}
/*
static uint8_t _set_ec(cmdObj *cmd) 	// expand CR to CRLF on TX
{
	cfg.enable_cr = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_CRLF, XIO_NOCRLF));
}
*/
static uint8_t _set_ee(cmdObj *cmd) 	// enable character echo
{
	cfg.enable_echo = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_ECHO, XIO_NOECHO));
}

static uint8_t _set_ex(cmdObj *cmd)		// enable XON/XOFF
{
	cfg.enable_xon = (uint8_t)cmd->value;
	return(_set_comm_helper(cmd, XIO_XOFF, XIO_NOXOFF));
}

/*
 * _set_baud() - set USB baud rate
 *
 *	See xio_usart.h for valid values. Works as a callback.
 *	The initial routine changes the baud config setting and sets a flag
 *	Then it sends a message indicating the new baud rate
 *	Then it waits for the TX buffer to empty (so the message is sent)
 *	Then it performs the callback to apply the new baud rate
 */

static uint8_t _set_baud(cmdObj *cmd)
{
	uint8_t baud = (uint8_t)cmd->value;
	if ((baud < 1) || (baud > 6)) {
		char message[CMD_STRING_LEN]; 
		sprintf_P(message, PSTR("*** WARNING *** Illegal baud rate specified"));
		cmd_add_string("msg",message);
		return (TG_INPUT_VALUE_UNSUPPORTED);
	}
	cfg.usb_baud_rate = baud;
	cfg.usb_baud_flag = true;
	char message[CMD_STRING_LEN]; 
	sprintf_P(message, PSTR("*** NOTICE *** Restting baud rate to %S"),(PGM_P)pgm_read_word(&msg_baud[baud]));
	cmd_add_string("msg",message);
	return (TG_OK);
}

uint8_t cfg_baud_rate_callback(void) 
{
	if (cfg.usb_baud_flag == false) { return(TG_NOOP);}
	cfg.usb_baud_flag = false;
	xio_set_baud_usart(XIO_DEV_USB, cfg.usb_baud_rate);
	return (TG_OK);
}

/*****************************************************************************
 *****************************************************************************
 *****************************************************************************
 *** END SETTING-SPECIFIC REGION *********************************************
 *** Code below should not require changes as parameters are added/updated ***
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************/

/****************************************************************************/
/**** CMD FUNCTION ENTRY POINTS *********************************************/
/****************************************************************************/
/* These are the primary access points to cmd functions
 * These are the gatekeeper functions that check index ranges so others don't have to
 */

#define ASSERT_CMD_INDEX(a) if (cmd->index >= CMD_INDEX_MAX) return (a);

/*
 * cmd_set() - Write a value or invoke a function - operates on single valued elements or groups
 */
uint8_t cmd_set(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].set)))(cmd));
}

/*
 * cmd_get() - Build a cmdObj with the values from the target & return the value
 *			   Populate cmd body with single valued elements or groups (iterates)
 */
uint8_t cmd_get(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd));
}

/*
 * cmd_print()	- Output a formatted string for the value.
 */
void cmd_print(cmdObj *cmd)
{
	if (cmd->index >= CMD_INDEX_MAX) return;
	((fptrPrint)(pgm_read_word(&cfgArray[cmd->index].print)))(cmd);
}

/*
 * cmd_persist()- persist value to NVM. Takes special cases into account
 */
void cmd_persist(cmdObj *cmd)
{
#ifdef __DISABLE_PERSISTENCE	// cutout for faster simulation in test
	return;
#endif
	if (_index_lt_groups(cmd->index) == false) return;
	if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_PERSIST) {
		cmd_write_NVM_value(cmd);
	}
}

/****************************************************************************
 * cfg_init() - called once on hard reset
 *
 * Will perform one of 2 actions:
 *	(1) if NVM is set up or out-of-rev: load RAM and NVM with hardwired default settings
 *	(2) if NVM is set up and at current config version: use NVM data for config
 */

void cfg_init()
{
//	You can assume the cfg struct has been zeroed by a hard reset. 
//	Do not clear as the version and build numbers have already been set by tg_init()

	cmdObj cmd;
	cm_set_units_mode(MILLIMETERS);	// must do init in MM mode
	cmd_new_list();					// setup the cmd object lists. Do this first.
	cfg.comm_mode = TG_JSON_MODE;	// initial value until EEPROM is read
	cfg.nvm_base_addr = NVM_BASE_ADDR;
	cfg.nvm_profile_base = cfg.nvm_base_addr;
	cmd.index = 0;					// this will read the first record in NVM
	cmd_read_NVM_value(&cmd);

	// Case (1) NVM is not setup or not in revision
	if (cmd.value != cfg.fw_build) {
		cmd.value = true;
		_set_defa(&cmd);			// this subroutine called from here and from the $defa=1 command

	// Case (2) NVM is setup and in revision
	} else {
		tg_print_loading_configs_message();
		for (cmd.index=0; _index_is_single(cmd.index); cmd.index++) {
			if (pgm_read_byte(&cfgArray[cmd.index].flags) & F_INITIALIZE) {
				strcpy_P(cmd.token, cfgArray[cmd.index].token);		// read the token from the array
				cmd_read_NVM_value(&cmd);
				cmd_set(&cmd);
			}
		}
	}
	rpt_init_status_report(true);// requires special treatment (persist = true)
}

/*
 * _set_defa() - reset NVM with default values for active profile
 */ 

static uint8_t _set_defa(cmdObj *cmd) 
{
	if (cmd->value != true) {		// failsafe. Must set true or no action occurs
		print_defaults_help(cmd);
		return (TG_OK);
	}
	cm_set_units_mode(MILLIMETERS);	// must do init in MM mode
	tg_print_initializing_message();

	for (cmd->index=0; _index_is_single(cmd->index); cmd->index++) {
		if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_INITIALIZE) {
			cmd->value = (double)pgm_read_float(&cfgArray[cmd->index].def_value);
			strcpy_P(cmd->token, cfgArray[cmd->index].token);
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (TG_OK);
}

/****************************************************************************
 * cfg_text_parser() - update a config setting from a text block (text mode)
 *_text_parser() 	 - helper for above
 * 
 * Use cases handled:
 *	- $xfr=1200	set a parameter
 *	- $xfr		display a parameter
 *	- $x		display a group
 *	- ?			generate a status report (multiline format)
 */

uint8_t cfg_text_parser(char *str)
{
	cmdObj *cmd = cmd_body;					// point at first object in the body
	uint8_t status = TG_OK;

	if (str[0] == '?') {					// handle status report case
		rpt_run_multiline_status_report();
		return (TG_OK);
	}
	if ((str[0] == '$') && (str[1] == NUL)) {  // treat a lone $ as a sys request
		strcat(str,"sys");
	}
	// single-unit parser processing
	ritorno(_text_parser(str, cmd));		// decode the request or return if error
	if ((cmd->type == TYPE_PARENT) || (cmd->type == TYPE_NULL)) {
		if (cmd_get(cmd) == TG_COMPLETE) {	// populate value, group values, or run uber-group displays
			return (TG_OK);					// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		status = cmd_set(cmd);				// set single value
		cmd_persist(cmd);
	}
	cmd_print_list(status, TEXT_MULTILINE_FORMATTED); // print the results
	return (status);
}

static uint8_t _text_parser(char *str, cmdObj *cmd)
{
	char *ptr_rd, *ptr_wr;					// read and write pointers
	char separators[] = {" =:|\t"};			// any separator someone might use

	// string pre-processing
	cmd_new_obj(cmd);						// initialize config object
	if (*str == '$') str++;					// ignore leading $
	for (ptr_rd = ptr_wr = str; *ptr_rd!=NUL; ptr_rd++, ptr_wr++) {
		*ptr_wr = tolower(*ptr_rd);			// convert string to lower case
		if (*ptr_rd==',') {
			*ptr_wr = *(++ptr_rd);			// skip over comma
		}
	}
	*ptr_wr = NUL;

	// field processing
	cmd->type = TYPE_NULL;
	if ((ptr_rd = strpbrk(str, separators)) == NULL) { // no value part
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
	} else {
		*ptr_rd = NUL;						// terminate at end of name
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
		str = ++ptr_rd;
		cmd->value = strtod(str, &ptr_rd);	// ptr_rd used as end pointer
		if (ptr_rd != str) {
			cmd->type = TYPE_FLOAT;
		}
	}
	if ((cmd->index = cmd_get_index("",cmd->token)) == NO_INDEX) { 
		return (TG_UNRECOGNIZED_COMMAND);
	}
	return (TG_OK);
}

/***** Generic Internal Functions *******************************************
 * _set_nul() - set nothing (returns TG_NOOP)
 * _set_ui8() - set value as uint8_t w/o unit conversion
 * _set_int() - set value as integer w/o unit conversion
 * _set_dbl() - set value as double w/o unit conversion
 * _set_dbu() - set value as double w/unit conversion
 *
 * _get_nul() - get nothing (returns TG_NOOP)
 * _get_ui8() - get value as uint8_t w/o unit conversion
 * _get_int() - get value as integer w/o unit conversion
 * _get_dbl() - get value as double w/o unit conversion
 * _get_dbu() - get value as double w/unit conversion
 *
 * _print_nul() - print nothing
 * _print_str() - print string value
 * _print_ui8() - print uint8_t value w/no units or unit conversion
 * _print_int() - print integer value w/no units or unit conversion
 * _print_dbl() - print double value w/no units or unit conversion
 * _print_lin() - print linear value with units and in/mm unit conversion
 * _print_rot() - print rotary value with units
 */

static uint8_t _set_nul(cmdObj *cmd) { return (TG_NOOP);}

static uint8_t _set_ui8(cmdObj *cmd)
{
	*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->type = TYPE_INTEGER;
	return(TG_OK);
}

static uint8_t _set_int(cmdObj *cmd)
{
	*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->type = TYPE_INTEGER;
	return(TG_OK);
}

static uint8_t _set_dbl(cmdObj *cmd)
{
	*((double *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->type = TYPE_FLOAT;
	return(TG_OK);
}

static uint8_t _set_dbu(cmdObj *cmd)
{
	if (cm_get_units_mode() == INCHES) {
		*((double *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value * MM_PER_INCH;
	} else {
		*((double *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	}
	cmd->type = TYPE_FLOAT;
	return(TG_OK);
}

static uint8_t _get_nul(cmdObj *cmd) 
{ 
	cmd->type = TYPE_NULL;
	return (TG_NOOP);
}

static uint8_t _get_ui8(cmdObj *cmd)
{
	cmd->value = (double)*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->type = TYPE_INTEGER;
	return (TG_OK);
}

static uint8_t _get_int(cmdObj *cmd)
{
	cmd->value = (double)*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->type = TYPE_INTEGER;
	return (TG_OK);
}

static uint8_t _get_dbl(cmdObj *cmd)
{
	cmd->value = *((double *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->type = TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_dbu(cmdObj *cmd)
{
	_get_dbl(cmd);
	if (cm_get_units_mode() == INCHES) {
		cmd->value *= INCH_PER_MM;
	}
	return (TG_OK);
}

static void _print_nul(cmdObj *cmd) {}

static void _print_str(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->string);
}

static void _print_ui8(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), (uint8_t)cmd->value);
}

static void _print_int(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), (uint32_t)cmd->value);
}

static void _print_dbl(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value);
}

static void _print_lin(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[cm_get_units_mode()]));
}

static void _print_rot(cmdObj *cmd)
{
	cmd_get(cmd);
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[F_DEG]));
}

/***************************************************************************** 
 * Accessors - get various data from an object given the index
 * _get_format() 	- return format string for an index
 * _get_motor()		- return the axis an index applies to or -1 if na
 * _get_axis()		- return axis for a group variable or -1 if na - e.g. xvm
 * _get_pos_axis()	- return axis number for pos values or -1 if none - e.g. posx
 */

static char *_get_format(const INDEX_T i, char *format)
{
	strncpy_P(format, (PGM_P)pgm_read_word(&cfgArray[i].format), CMD_FORMAT_LEN);
	return (format);
}

static int8_t _get_motor(const INDEX_T i)
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
static int8_t _get_axis(const INDEX_T i)
{
	char *ptr;
	char tmp[CMD_TOKEN_LEN+1];
	char axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[0])) == NULL) { return (-1);}
	return (ptr - axes);
}
*/

static int8_t _get_pos_axis(const INDEX_T i)
{
	char *ptr;
	char tmp[CMD_TOKEN_LEN+1];
	char axes[] = {"xyzabc"};

	strcpy_P(tmp, cfgArray[i].token);
	if ((ptr = strchr(axes, tmp[3])) == NULL) { return (-1);}
	return (ptr - axes);
}

/****************************************************************************
 * Exposed cmdObj helper functions and other low-level cmd helpers
 * cmd_get_max_index()	 - utility function to return index array size				
 * cmd_new_obj() 	 	 - quick clear for a new cmd object
 * cmd_get_cmdObj() 	 - setup a cmd object by providing the index
 * cmd_get_index() 		 - get index from mnenonic token + group
 * cmd_get_type()		 - returns command type as a CMD_TYPE enum
 * cmd_persist_offsets() - write any changed G54 (et al) offsets back to NVM
 *
 *	cmd_get_index() is the most expensive routine in the whole config. It does a 
 *	linear table scan of the PROGMEM strings, which of course could be further 
 *	optimized with indexes or hashing.
 */

//INDEX_T cmd_get_max_index() { return (CMD_INDEX_MAX);}

cmdObj *cmd_new_obj(cmdObj *cmd)			// clear a single cmdObj structure
{
	cmd->type = TYPE_EMPTY;					// much faster than calling memset
	cmd->index = 0;
	cmd->value = 0;
	cmd->token[0] = NUL;
	cmd->group[0] = NUL;
	cmd->string[0] = NUL;

	if (cmd->pv != NULL) { 			// set depth correctly
		if (cmd->pv->type == TYPE_PARENT) { 
			cmd->depth = cmd->pv->depth + 1;
		} else {
			cmd->depth = cmd->pv->depth;
		}
	}
	return (cmd);
}

void cmd_get_cmdObj(cmdObj *cmd)
{
	if (cmd->index >= CMD_INDEX_MAX) return;
	INDEX_T tmp = cmd->index;
	cmd_new_obj(cmd);
	cmd->index = tmp;

	strcpy_P(cmd->group, cfgArray[cmd->index].group); // group field is always terminated
	strcpy_P(cmd->token, cfgArray[cmd->index].token); // token field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (cmd->group[0] != NUL) {
		if (strstr(cmd->group, "sys") != NULL) { 
			cmd->group[0] = NUL;
		} else {
			strcpy(cmd->token, &cmd->token[strlen(cmd->group)]); // strip group from the token
		}
	}
	((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd);	// populate the value
}

INDEX_T cmd_get_index(const char *group, const char *token)
{
	char c;
	char str[CMD_TOKEN_LEN+1];
	strcpy(str, group);
	strcat(str, token);

	for (INDEX_T i=0; i<CMD_INDEX_MAX; i++) {
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
	return (NO_INDEX);	// no match
}

uint8_t cmd_get_type(cmdObj *cmd)
{
	if (strcmp("gc", cmd->token) == 0) return (CMD_TYPE_GCODE);
	if (strcmp("sr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("qr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	return (CMD_TYPE_CONFIG);
}

uint8_t cmd_persist_offsets(uint8_t flag)
{
	if (flag == true) {
		cmdObj cmd;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf(cmd.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				cmd.index = cmd_get_index("", cmd.token);
				cmd.value = cfg.offset[i][j];
				cmd_persist(&cmd);				// only writes changed values
			}
		}
	}
	return (TG_OK);
}

/********************************************************************************
 ***** Group operations *********************************************************
 ********************************************************************************
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

static uint8_t _get_grp(cmdObj *cmd)
{
	char *parent_group = cmd->token;		// token in the parent cmd object is the group
	char group[CMD_GROUP_LEN+1];			// group string retrieved from cfgArray child
	cmd->type = TYPE_PARENT;				// make first object the parent 
	for (INDEX_T i=0; i<=CMD_INDEX_END_SINGLES; i++) {
		strcpy_P(group, cfgArray[i].group);  // don't need strncpy as it's always terminated
		if (strcmp(parent_group, group) != 0) continue;
		(++cmd)->index = i;
		cmd_get_cmdObj(cmd);
	}
	return (TG_OK);
}

/*
 * _set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and 
 *	a setter. It iterates the group children and either gets the value or sets
 *	the value for each depending on the cmd->type.
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

static uint8_t _set_grp(cmdObj *cmd)
{
	if (cfg.comm_mode == TG_TEXT_MODE) return (TG_UNRECOGNIZED_COMMAND);
	for (uint8_t i=0; i<CMD_MAX_OBJECTS; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if (cmd->type == TYPE_EMPTY) break;
		else if (cmd->type == TYPE_NULL)	// NULL means GET the value
			cmd_get(cmd);
		else {
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (TG_OK);
}

/**** UberGroup Operations ****
 * Uber groups are groups of groups organized for convenience:
 *	- motors	- group of all motor groups
 *	- axes		- group of all axis groups
 *	- offsets	- group of all offset groups
 *	- all		- group of all groups
 *
 * _do_group_list()	- get and print all groups in the list (iteration)
 * _do_motors()		- get and print motor uber group 1-4
 * _do_axes()		- get and print axis uber group XYZABC
 * _do_offsets()	- get and print offset uber group G54-G59
 * _do_all()		- get and print all groups uber group
 */

static void _do_group_list(cmdObj *cmd, char list[][CMD_TOKEN_LEN+1]) // helper to print multiple groups in a list
{
	for (uint8_t i=0; i < CMD_MAX_OBJECTS; i++) {
		if (list[i][0] == NUL) return;
		cmd = cmd_body;
		strncpy(cmd->token, list[i], CMD_TOKEN_LEN);
		cmd->index = cmd_get_index("", cmd->token);
//		cmd->type = TYPE_PARENT;
		cmd_get_cmdObj(cmd);
		cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);
	}
}

static uint8_t _do_motors(cmdObj *cmd)	// print parameters for all motor groups
{
	char list[][CMD_TOKEN_LEN+1] = {"1","2","3","4",""}; // must have a terminating element
	_do_group_list(cmd, list);
	return (TG_COMPLETE);
}

static uint8_t _do_axes(cmdObj *cmd)	// print parameters for all axis groups
{
	char list[][CMD_TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
	_do_group_list(cmd, list);
	return (TG_COMPLETE);
}

static uint8_t _do_offsets(cmdObj *cmd)	// print offset parameters for G54-G59,G92
{
	char list[][CMD_TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59",""}; // must have a terminating element
	_do_group_list(cmd, list);
	return (TG_COMPLETE);
}

static uint8_t _do_all(cmdObj *cmd)		// print all parameters
{
	// print system group
	strcpy(cmd->token,"sys");
	_get_grp(cmd);
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);

	_do_offsets(cmd);
	_do_motors(cmd);
	_do_axes(cmd);

	// print PWM group
	strcpy(cmd->token,"p1");
	_get_grp(cmd);
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);

	return (TG_COMPLETE);
}

/********************************************************************************
 ***** cmdObj list initialization and manipulation ******************************
 ********************************************************************************
 * cmd_new_list()	 - clear entire header, body and footer for a new use
 * cmd_new_body()	 - clear the body for a new use 
 * cmd_add_object()	 - write contents of parameter to  first free object in the body
 * cmd_add_string()	 - add a string object to end of cmd body
 * cmd_add_integer() - add an integer value to end of cmd body (Note 1)
 * cmd_add_float()	 - add a floating point value to end of cmd body
 *
 *	Note 1: adding a really large integer (like a checksum value) may lose 
 *	precision due to the cast to a double. Sometimes it's better to load an 
 *	integer as a string if all you want to do is display it.
 */

void cmd_new_list()						// clear the header, response body and footer
{
	// setup header ("r" parent)
	cmdObj *cmd = cmd_header;
	cmd->pv = 0;
	cmd->nx = cmd_body;
	cmd->index = 0;
	cmd->depth = 0;
	cmd->type = TYPE_PARENT;
	cmd->token[0] = 'r';
	cmd++;

	// setup body
	cmd_new_body(cmd_body);

	// setup footer
	cmd = cmd_footer;
	cmd->pv = &cmd_body[CMD_BODY_LEN-1];
	cmd->nx = (cmd+1);
	cmd->token[0] = 'f';
	cmd->token[1] = NUL;
	cmd->depth = 0;
	cmd->type = TYPE_ARRAY;

	cmd->nx->type = TYPE_EMPTY;				// setup terminating element
	cmd->nx->pv = (cmd-1);
	cmd->nx->nx = 0;
	return;
}

void cmd_new_body(cmdObj *cmd)			// clear the body list
{
	// setup body elements
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (i == 0) {
			cmd->pv = cmd_header;
		} else {
			cmd->pv = (cmd-1);
		}
		cmd->nx = (cmd+1);
		cmd->index = 0;
		cmd->token[0] = NUL;
		cmd->depth = 1;
		cmd->type = TYPE_EMPTY;
		cmd++;
	}
	(--cmd)->nx = cmd_footer;				// correct last element
}

uint8_t cmd_add_object(char *token)			// add an object to the body using a token
{
	cmdObj *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index("",token)) == NO_INDEX) {
			return (TG_UNRECOGNIZED_COMMAND);
		}
		cmd_get_cmdObj(cmd);				// populate the object from the index
		return (TG_OK);		
	}
	return (TG_NO_BUFFER_SPACE);
}

uint8_t cmd_add_string(char *token, char *string)	// add a string object to the body
{
	cmdObj *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
//		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
		strncpy(cmd->string, string, CMD_STRING_LEN);
		cmd->index = cmd_get_index("", cmd->token);	//#######################
		cmd->type = TYPE_STRING;
		return (TG_OK);
	}
	return (TG_NO_BUFFER_SPACE);
}

uint8_t cmd_add_integer(char *token, uint32_t value)// add an integer object to the body
{
	cmdObj *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
//		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
		cmd->value = (double) value;
		cmd->type = TYPE_INTEGER;
		return (TG_OK);
	}
	return (TG_NO_BUFFER_SPACE);
}

uint8_t cmd_add_float(char *token, double value)	// add a float object to the body
{
	cmdObj *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
//		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
		cmd->value = value;
		cmd->type = TYPE_FLOAT;
		return (TG_OK);
	}
	return (TG_NO_BUFFER_SPACE);
}

/**** cmd_print_list() - print cmd_array in JSON mode or one of the text modes ****
 *
 * 	Use this function for all text and JSON output (don't just printf stuff)
 * 	It generates and prints the JSON and text mode output strings 
 *	It also cleans up the lists and gets ready for the next use
 *	In JSON mode it generates the footer with the status code, buffer count and checksum
 *	In text mode it uses the the textmode variable to set the output format
 */

void cmd_print_list(uint8_t status, uint8_t textmode)
{
	if (cfg.comm_mode == TG_JSON_MODE) {
		js_print_list(status);
	} else {
		switch (textmode) {
			case TEXT_INLINE_PAIRS: { _print_text_inline_pairs(); break; }
			case TEXT_INLINE_VALUES: { _print_text_inline_values(); break; }
			case TEXT_MULTILINE_FORMATTED: { _print_text_multiline_formatted();}
		}
	}
	cmd_new_body(cmd_body);		// clear the cmd body to get ready for the next use
}

void _print_text_inline_pairs()
{
	cmdObj *cmd = cmd_body;

	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->type) {
			case TYPE_PARENT:	{ cmd = cmd->nx; continue; }
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%s:%1.3f"), cmd->token, cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%s:%1.0f"), cmd->token, cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s:%s"), cmd->token, cmd->string); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		cmd = cmd->nx;
		if (cmd->type != TYPE_EMPTY) {
			fprintf_P(stderr,PSTR(","));
		}		
	}
}

void _print_text_inline_values()
{
	cmdObj *cmd = cmd_body;

	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->type) {
			case TYPE_PARENT:	{ cmd = cmd->nx; continue; }
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%1.3f"), cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%1.0f"), cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s"), cmd->string); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		cmd = cmd->nx;
		if (cmd->type != TYPE_EMPTY) {
			fprintf_P(stderr,PSTR(","));
		}
	}
}

void _print_text_multiline_formatted()
{
	cmdObj *cmd = cmd_body;

	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		if (cmd->type != TYPE_PARENT) { cmd_print(cmd);}
		cmd = cmd->nx;
		if (cmd->type == TYPE_EMPTY) { break;}
	}
}

/************************************************************************************
 ***** EEPROM access functions ******************************************************
 ************************************************************************************
 * cmd_read_NVM_value()	 - return value (as double) by index
 * cmd_write_NVM_value() - write to NVM by index, but only if the value has changed
 * (see 331.09 or earlier for token/value record-oriented routines)
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */
uint8_t cmd_read_NVM_value(cmdObj *cmd)
{
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
	return (TG_OK);
}

uint8_t cmd_write_NVM_value(cmdObj *cmd)
{
	double tmp = cmd->value;
	ritorno(cmd_read_NVM_value(cmd));
	if (cmd->value != tmp) {		// catches the isnan() case as well
		cmd->value = tmp;
		int8_t nvm_byte_array[NVM_VALUE_LEN];
		memcpy(&nvm_byte_array, &tmp, NVM_VALUE_LEN);
		uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
	return (TG_OK);
}

/****************************************************************************
 ***** Config Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TEST_CONFIG

#define NVMwr(i,v) { cmd.index=i; cmd.value=v; cmd_write_NVM_value(&cmd);}
#define NVMrd(i)   { cmd.index=i; cmd_read_NVM_value(&cmd); printf("%f\n",cmd.value);}

void cfg_unit_tests()
{

// NVM tests
/*	cmdObj cmd;
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

	INDEX_T i;
//	double val;

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

