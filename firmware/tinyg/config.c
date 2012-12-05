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
 *	--- Config objects and the config list
 *
 *	The config system provides a formatted way to access and set configuration variables.
 *	It also provides a way to get an arbitrary variable for reporting. Config operates
 *	as a collection of "objects" (OK, so they are not really objects) that encapsulate
 *	each variable. The objects are collected into a list (the body), which also has  
 *	header and footer objects. This way the internals don't care about how the variable
 *	is represented or communicated externally as all operations occur on a cmdObj list. 
 *	The list is populated by the text_parser or the JSON_parser depending on the mode.
 *
 *	--- Config variables
 *
 *	Each configuration value is identified by a short mnemonic string (token). The token 
 *	is resolved to an index into the config array for that entry. The array has data and 
 *	function pointers needed to process that value. The array is organized as so:
 * 
 *	- PROGMEM array (cfgArray) contains typed data in program memory. Each cfgItem has:
 *		- function pointer for formatted print() method (used only in text mode)
 *		- function pointer for get() method - gets values from memory
 *		- function pointer for set() method - sets values and runs functions
 *		- target - memory location that the value is written to / read from
 *		- default value - for cold initialization
 *		- pointer to a comma separated list which carries:
 *			- token string
 *			- format string for formatted print
 *
 *	Additionally an NVM array contains values persisted to EEPROM as doubles; indexed by cfgArray index
 *
 *	The following rules apply to mnemonic tokens
 *	 - are up to 4 alphnuneric characters and cannot contain whitespace or separators
 *	 - must be unique (non colliding).
 *	 - axis tokens start with the axis letter and are typically 3 characters including the axis letter
 *	 - motor tokens start with the motor digit and are typically 3 characters including the motor digit
 *	 - non-axis or non-motor tokens are 2-4 characters and should not start with: xyzabcuvw0123456789
 *		(any exceptions to this rule must be added to GROUP_EXCLUSIONS in config.h)
 *
 *  "Groups" are collections of values that mimic REST resources. Groups include:
 *	 - axis groups prefixed by "xyzabc"		("uvw" are reserved)
 *	 - motor groups prefixed by "1234"		("56789" are reserved)
 *	 - coordinate system groups prefixed by g54, g55, g56, g57, g59, g92
 *	 - system groups identified by "sys" and containing a random collection of otherwise unrelated values
 *
 *	"Uber-groups" are groups of groups that are only used for text-mode printing - e.g.
 *	 - group of all axes groups
 *	 - group of all motor groups
 *	 - group of all other groups
 */
/*  --- Making changes and adding new values
 *
 *	Adding a new value to config (or changing an existing one) involves touching the following places:
 *	 - Add a token / formatting string to str_XXX strings (ensure unique token & name!)
 *	 - Create a new record in cfgArray[] which includes:
 *		- reference to the above string
 *		- an existing print() function or create a new one if necessary 
 *		- an existing apply() fucntion or create a new one if necessary
 *		- target pointer (a variable must exist somewhere, often in the cfg struct)
 *		- default value for the parameter
 *
 * 	The ordering of group displays is set by the order of items in cfgArray. None of the other orders 
 *	matter but are generally kept sequenced for easier reading and code maintenance.
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

struct cfgItem {			// structs for pgmArray
	const char *string;		// pointer to names composite string
	fptrPrint print;		// print binding: aka void (*print)(cmdObj *cmd);
	fptrCmd get;			// GET binding aka uint8_t (*get)(cmdObj *cmd)
	fptrCmd set;			// SET binding aka uint8_t (*set)(cmdObj *cmd)
	double *target;			// target for writing config value
	double def_value;		// default value for config item
};

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

static uint8_t _cmd_index_is_single(uint8_t index);
static uint8_t _cmd_index_is_group(uint8_t index);
static uint8_t _cmd_index_is_uber_group(uint8_t index);

// helpers for generic functions
static uint8_t _dont_initialize(cmdObj *cmd);
static char *_get_format(const INDEX_T i, char *format);
static int8_t _get_motor(const INDEX_T i);
static int8_t _get_pos_axis(const INDEX_T i);
//static int8_t _get_grp_axis(const INDEX_T i);
static uint8_t _parse_config_string(char *str, struct cmdObject *c);
static uint8_t _get_msg_helper(cmdObj *cmd, prog_char_ptr msg, uint8_t value);
static void _print_text_inline_pairs();
static void _print_text_inline_values();
static void _print_text_multiline_formatted();

static uint8_t _set_grp(cmdObj *cmd);	// set data for a group
static uint8_t _get_grp(cmdObj *cmd);	// get data for a group
static uint8_t _get_sys(cmdObj *cmd);	// get data for system group (special case)
static void _print_sys(cmdObj *cmd);	// print system group
static uint8_t _do_motors(cmdObj *cmd);	// print parameters for all motor groups
static uint8_t _do_axes(cmdObj *cmd);		// print parameters for all axis groups
static uint8_t _do_offsets(cmdObj *cmd);	// print offsets for G54-G59, G92
static uint8_t _do_all(cmdObj *cmd);		// print all parameters
static void _do_group_list(cmdObj *cmd, char list[][CMD_TOKEN_LEN+1]); // helper to print multiple groups in a list
//static void _print_groups(cmdObj *cmd, char *ptr); // helper to print multiple groups

/*****************************************************************************
 **** PARAMETER-SPECIFIC CODE REGION *****************************************
 **** This code and data will change as you add / update config parameters ***
 *****************************************************************************/

// parameter-specific internal functions
static uint8_t _get_id(cmdObj *cmd);	// get device ID (signature)
static uint8_t _get_sr(cmdObj *cmd);	// run status report (as data)
static void _print_sr(cmdObj *cmd);		// run status report (as printout)
static uint8_t _set_sr(cmdObj *cmd);	// set status report specification
static uint8_t _set_si(cmdObj *cmd);	// set status report interval
static uint8_t _get_qr(cmdObj *cmd);	// run queue report (as data)
static uint8_t _get_pb(cmdObj *cmd);	// get planner buffers available
static uint8_t _get_lx(cmdObj *cmd);	// get runtime line index
static uint8_t _set_lx(cmdObj *cmd);	// set runtime line index
static uint8_t _get_rx(cmdObj *cmd);	// get bytes in RX buffer

static uint8_t _get_gc(cmdObj *cmd);	// get current gcode block
static uint8_t _run_gc(cmdObj *cmd);	// run a gcode block

static uint8_t _get_line(cmdObj *cmd);	// get runtime line number
static uint8_t _get_stat(cmdObj *cmd);	// get combined machine state as value and string
static uint8_t _get_macs(cmdObj *cmd);	// get raw machine state as value and string
static uint8_t _get_cycs(cmdObj *cmd);	// get raw cycle state as value and string
static uint8_t _get_mots(cmdObj *cmd);	// get raw motion state as value and string
static uint8_t _get_hold(cmdObj *cmd);	// get raw hold state as value and string
static uint8_t _get_home(cmdObj *cmd);	// get raw homing state as value and string
static uint8_t _get_unit(cmdObj *cmd);	// (all these in this list are similar)
static uint8_t _get_coor(cmdObj *cmd);
static uint8_t _get_momo(cmdObj *cmd);
static uint8_t _get_plan(cmdObj *cmd);
static uint8_t _get_path(cmdObj *cmd);
static uint8_t _get_dist(cmdObj *cmd);
static uint8_t _get_frmo(cmdObj *cmd);
static uint8_t _get_vel(cmdObj *cmd);	// get runtime velocity
static uint8_t _get_pos(cmdObj *cmd);	// get runtime work position
static uint8_t _get_mpos(cmdObj *cmd);	// get runtime machine position
static void _print_pos(cmdObj *cmd);	// print runtime work position

static uint8_t _set_defa(cmdObj *cmd);	// reset config to defaults

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

/* PROGMEM strings for token match and print formatting
 *	Use accessors to get at elements in these combined strings.
 *
 *	NOTE: DO NOT USE TABS IN FORMAT STRINGS
 *	NOTE: LEAVE NO SPACE BEFORE OR AFTER COMMAS (TOKEN,NAME)
 *	NOTE: In general, any mnemonic that starts with a group character will be 
 *		  returned when that group is retrieved. Groups are 1 2 3 4 x y z a b c. 
 *		  For example, xam, xfr, xvm etc will all be returned when the 'x' group is queried.
 *		  In some cases this is not desired. To exclude an element from group 
 *		  retrieval you must list it in the GROUP_EXCLUDE string in config.h.
 *		  Currently only cycs(tate) and coor(inate system) and baud are excluded. 
 */
static const char str_fv[] PROGMEM = "fv,[fv]  firmware_version%16.2f\n";
static const char str_fb[] PROGMEM = "fb,[fb]  firmware_build%18.2f\n";
static const char str_id[] PROGMEM = "id,[id]  id_device%16d\n";

// Gcode model values for reporting purposes
static const char str_vel[]  PROGMEM = "vel,Velocity:%17.3f%S/min\n";
static const char str_line[] PROGMEM = "line,Line number:%10.0f\n";
static const char str_feed[] PROGMEM = "feed,Feed rate:%16.3f%S/min\n";
static const char str_stat[] PROGMEM = "stat,Machine state:       %s\n"; // combined machine state
static const char str_macs[] PROGMEM = "macs,Raw machine state:   %s\n"; // raw machine state
static const char str_cycs[] PROGMEM = "cycs,Cycle state:         %s\n";
static const char str_mots[] PROGMEM = "mots,Motion state:        %s\n";
static const char str_hold[] PROGMEM = "hold,Feedhold state:      %s\n";
static const char str_home[] PROGMEM = "home,Homing state:        %s\n";
static const char str_unit[] PROGMEM = "unit,Units:               %s\n"; // units mode as ASCII string
static const char str_coor[] PROGMEM = "coor,Coordinate system:   %s\n";
static const char str_momo[] PROGMEM = "momo,Motion mode:         %s\n";
static const char str_plan[] PROGMEM = "plan,Plane:               %s\n";
static const char str_path[] PROGMEM = "path,Path Mode:           %s\n";
static const char str_dist[] PROGMEM = "dist,Distance mode:       %s\n";
static const char str_frmo[] PROGMEM = "frmo,Feed rate mode:      %s\n";
static const char str_posx[] PROGMEM = "posx,X position:%15.3f%S\n";
static const char str_posy[] PROGMEM = "posy,Y position:%15.3f%S\n";
static const char str_posz[] PROGMEM = "posz,Z position:%15.3f%S\n";
static const char str_posa[] PROGMEM = "posa,A position:%15.3f%S\n";
static const char str_posb[] PROGMEM = "posb,B position:%15.3f%S\n";
static const char str_posc[] PROGMEM = "posc,C position:%15.3f%S\n";
static const char str_mpox[] PROGMEM = "mpox,X mach position:%10.3f%S\n";
static const char str_mpoy[] PROGMEM = "mpoy,Y mach position:%10.3f%S\n";
static const char str_mpoz[] PROGMEM = "mpoz,Z mach position:%10.3f%S\n";
static const char str_mpoa[] PROGMEM = "mpoa,A mach position:%10.3f%S\n";
static const char str_mpob[] PROGMEM = "mpob,B mach position:%10.3f%S\n";
static const char str_mpoc[] PROGMEM = "mpoc,C mach position:%10.3f%S\n";
static const char str_g92x[] PROGMEM = "g92x,X origin offset:%10.3f%S\n";
static const char str_g92y[] PROGMEM = "g92y,Y origin offset:%10.3f%S\n";
static const char str_g92z[] PROGMEM = "g92z,Z origin offset:%10.3f%S\n";
static const char str_g92a[] PROGMEM = "g92a,A origin offset:%10.3f%S\n";
static const char str_g92b[] PROGMEM = "g92b,B origin offset:%10.3f%S\n";
static const char str_g92c[] PROGMEM = "g92c,C origin offset:%10.3f%S\n";
static const char str_lx[]   PROGMEM = "lx,Line index:%13d\n";
static const char str_pb[]   PROGMEM = "pb,Planner buffers:%8d\n";
static const char str_rx[]   PROGMEM = "rx,";		// bytes available in RX buffer

// Reports, tests, help, messages 
static const char str_help[] PROGMEM = "help,";		// display configuration help
static const char str_test[] PROGMEM = "test,";		// specialized _print_test() function
static const char str_defa[] PROGMEM = "defa,";		// restore default settings
static const char str_msg[]  PROGMEM = "msg,%s\n";	// generic message (with no formatting)
static const char str_sr[]   PROGMEM = "sr,";		// status_report {"sr":""}  and ? command
static const char str_qr[]   PROGMEM = "qr,";		// queue_report {"qr":""}

// Gcode model power-on reset default value strings
static const char str_gpl[] PROGMEM = "gpl,[gpl] gcode_select_plane %10d [0,1,2]\n";
static const char str_gun[] PROGMEM = "gun,[gun] gcode_units_mode   %10d [0,1]\n";
static const char str_gco[] PROGMEM = "gco,[gco] gcode_coord_system %10d [1-6]\n";
static const char str_gpa[] PROGMEM = "gpa,[gpa] gcode_path_control %10d [0,1,2]\n";
static const char str_gdi[] PROGMEM = "gdi,[gdi] gcode_distance_mode%10d [0,1]\n";
static const char str_gc[]  PROGMEM = "gc,[gc]";

// System settings strings
//static const char str_ea[] PROGMEM = "ea,enable_a,[ea]  enable_acceleration%10d [0,1]\n";
static const char str_ja[] PROGMEM = "ja,[ja]  junction_acceleration%8.0f%S\n";
static const char str_ml[] PROGMEM = "ml,[ml]  min_line_segment%17.3f%S\n";
static const char str_ma[] PROGMEM = "ma,[ma]  min_arc_segment%18.3f%S\n";
static const char str_mt[] PROGMEM = "mt,[mt]  min_segment_time%13.0f uSec\n";
static const char str_st[] PROGMEM = "st,[st]  switch_type%18d [0,1]\n";
static const char str_si[] PROGMEM = "si,[si]  status_interval    %10.0f ms [0=off]\n";

static const char str_ic[] PROGMEM = "ic,[ic]  ignore CR or LF on RX %7d [0,1=CR,2=LF]\n";
//static const char str_ec[] PROGMEM = "ec,[ec]  enable_CR on TX%14d [0,1]\n";
static const char str_ee[] PROGMEM = "ee,[ee]  enable_echo      %12d [0,1]\n";
static const char str_ex[] PROGMEM = "ex,[ex]  enable_xon_xoff  %12d [0,1]\n";
static const char str_eq[] PROGMEM = "eq,[eq]  enable_queue_reports%9d [0,1]\n";
static const char str_ej[] PROGMEM = "ej,[ej]  enable_json_mode %12d [0,1]\n";
static const char str_je[] PROGMEM = "je,[je]  json_echo_mode %14d [0-4]\n";
static const char str_baud[] PROGMEM = "baud,[baud] USB baud rate%15d [0-6]\n";

// Motor strings 
static const char str_1ma[] PROGMEM = "1ma,[1ma] m1_map_to_axis%15d [0=X, 1=Y...]\n";
static const char str_1sa[] PROGMEM = "1sa,[1sa] m1_step_angle%20.3f%S\n";
static const char str_1tr[] PROGMEM = "1tr,[1tr] m1_travel_per_revolution%9.3f%S\n";
static const char str_1mi[] PROGMEM = "1mi,[1mi] m1_microsteps %15d [1,2,4,8]\n";
static const char str_1po[] PROGMEM = "1po,[1po] m1_polarity   %15d [0,1]\n";
static const char str_1pm[] PROGMEM = "1pm,[1pm] m1_power_management%10d [0,1]\n";

static const char str_2ma[] PROGMEM = "2ma,[2ma] m2_map_to_axis%15d [0=X, 1=Y...]\n";
static const char str_2sa[] PROGMEM = "2sa,[2sa] m2_step_angle%20.3f%S\n";
static const char str_2tr[] PROGMEM = "2tr,[2tr] m2_travel_per_revolution%9.3f%S\n";
static const char str_2mi[] PROGMEM = "2mi,[2mi] m2_microsteps %15d [1,2,4,8]\n";
static const char str_2po[] PROGMEM = "2po,[2po] m2_polarity   %15d [0,1]\n";
static const char str_2pm[] PROGMEM = "2pm,[2pm] m2_power_management%10d [0,1]\n";

static const char str_3ma[] PROGMEM = "3ma,[3ma] m3_map_to_axis%15d [0=X, 1=Y...]\n";
static const char str_3sa[] PROGMEM = "3sa,[3sa] m3_step_angle%20.3f%S\n";
static const char str_3tr[] PROGMEM = "3tr,[3tr] m3_travel_per_revolution%9.3f%S\n";
static const char str_3mi[] PROGMEM = "3mi,[3mi] m3_microsteps %15d [1,2,4,8]\n";
static const char str_3po[] PROGMEM = "3po,[3po] m3_polarity   %15d [0,1]\n";
static const char str_3pm[] PROGMEM = "3pm,[3pm] m3_power_management%10d [0,1]\n";

static const char str_4ma[] PROGMEM = "4ma,[4ma] m4_map_to_axis%15d [0=X, 1=Y...]\n";
static const char str_4sa[] PROGMEM = "4sa,[4sa] m4_step_angle%20.3f%S\n";
static const char str_4tr[] PROGMEM = "4tr,[4tr] m4_travel_per_revolution%9.3f%S\n";
static const char str_4mi[] PROGMEM = "4mi,[4mi] m4_microsteps %15d [1,2,4,8]\n";
static const char str_4po[] PROGMEM = "4po,[4po] m4_polarity   %15d [0,1]\n";
static const char str_4pm[] PROGMEM = "4pm,[4pm] m4_power_management%10d [0,1]\n";

// Axis strings
static const char str_xam[] PROGMEM = "xam,[xam] x_axis_mode%18d %S\n";
static const char str_xfr[] PROGMEM = "xfr,[xfr] x_feedrate_maximum%15.3f%S/min\n";
static const char str_xvm[] PROGMEM = "xvm,[xvm] x_velocity_maximum%15.3f%S/min\n";
static const char str_xtm[] PROGMEM = "xtm,[xtm] x_travel_maximum%17.3f%S\n";
static const char str_xjm[] PROGMEM = "xjm,[xjm] x_jerk_maximum%15.0f%S/min^3\n";
static const char str_xjd[] PROGMEM = "xjd,[xjd] x_junction_deviation%14.4f%S (larger is faster)\n";
static const char str_xsn[] PROGMEM = "xsn,[xsn] x_switch_min%17d [0-4]\n";
static const char str_xsx[] PROGMEM = "xsx,[xsx] x_switch_max%17d [0-4]\n";
static const char str_xsv[] PROGMEM = "xsv,[xsv] x_search_velocity%16.3f%S/min\n";
static const char str_xlv[] PROGMEM = "xlv,[xlv] x_latch_velocity%17.3f%S/min\n";
static const char str_xlb[] PROGMEM = "xlb,[xlb] x_latch_backoff%18.3f%S\n";
static const char str_xzb[] PROGMEM = "xzb,[xzb] x_zero_backoff%19.3f%S\n";

static const char str_yam[] PROGMEM = "yam,[yam] y_axis_mode%18d %S\n";
static const char str_yfr[] PROGMEM = "yfr,[yfr] y_feedrate_maximum%15.3f%S/min\n";
static const char str_yvm[] PROGMEM = "yvm,[yvm] y_velocity_maximum%15.3f%S/min\n";
static const char str_ytm[] PROGMEM = "ytm,[ytm] y_travel_maximum%17.3f%S\n";
static const char str_yjm[] PROGMEM = "yjm,[yjm] y_jerk_maximum%15.0f%S/min^3\n";
static const char str_yjd[] PROGMEM = "yjd,[yjd] y_junction_deviation%14.4f%S (larger is faster)\n";
static const char str_ysn[] PROGMEM = "ysn,[ysn] y_switch_min%17d [0-4]\n";
static const char str_ysx[] PROGMEM = "ysx,[ysx] y_switch_max%17d [0-4]\n";
static const char str_ysv[] PROGMEM = "ysv,[ysv] y_search_velocity%16.3f%S/min\n";
static const char str_ylv[] PROGMEM = "ylv,[ylv] y_latch_velocity%17.3f%S/min\n";
static const char str_ylb[] PROGMEM = "ylb,[ylb] y_latch_backoff%18.3f%S\n";
static const char str_yzb[] PROGMEM = "yzb,[yzb] y_zero_backoff%19.3f%S\n";

static const char str_zam[] PROGMEM = "zam,[zam] z_axis_mode%18d %S\n";
static const char str_zfr[] PROGMEM = "zfr,[zfr] z_feedrate_maximum%15.3f%S/min\n";
static const char str_zvm[] PROGMEM = "zvm,[zvm] z_velocity_maximum%15.3f%S/min\n";
static const char str_ztm[] PROGMEM = "ztm,[ztm] z_travel_maximum%17.3f%S\n";
static const char str_zjm[] PROGMEM = "zjm,[zjm] z_jerk_maximum%15.0f%S/min^3\n";
static const char str_zjd[] PROGMEM = "zjd,[zjd] z_junction_deviation%14.4f%S (larger is faster)\n";
static const char str_zsn[] PROGMEM = "zsn,[zsn] z_switch_min%17d [0-4]\n";
static const char str_zsx[] PROGMEM = "zsx,[zsx] z_switch_max%17d [0-4]\n";
static const char str_zsv[] PROGMEM = "zsv,[zsv] z_search_velocity%16.3f%S/min\n";
static const char str_zlv[] PROGMEM = "zlv,[zlv] z_latch_velocity%17.3f%S/min\n";
static const char str_zlb[] PROGMEM = "zlb,[zlb] z_latch_backoff%18.3f%S\n";
static const char str_zzb[] PROGMEM = "zzb,[zzb] z_zero_backoff%19.3f%S\n";

static const char str_aam[] PROGMEM = "aam,[aam] a_axis_mode%18d %S\n";
static const char str_afr[] PROGMEM = "afr,[afr] a_feedrate_maximum%15.3f%S/min\n";
static const char str_avm[] PROGMEM = "avm,[avm] a_velocity_maximum%15.3f%S/min\n";
static const char str_atm[] PROGMEM = "atm,[atm] a_travel_maximum  %15.3f%S\n";
static const char str_ajm[] PROGMEM = "ajm,[ajm] a_jerk_maximum%15.0f%S/min^3\n";
static const char str_ajd[] PROGMEM = "ajd,[ajd] a_junction_deviation%14.4f%S\n";
static const char str_ara[] PROGMEM = "ara,[ara] a_radius_value%20.4f%S\n";
static const char str_asn[] PROGMEM = "asn,[asn] a_switch_min%17d [0-4]\n";
static const char str_asx[] PROGMEM = "asx,[asx] a_switch_max%17d [0-4]\n";
static const char str_asv[] PROGMEM = "asv,[asv] a_search_velocity%16.3f%S/min\n";
static const char str_alv[] PROGMEM = "alv,[alv] a_latch_velocity%17.3f%S/min\n";
static const char str_alb[] PROGMEM = "alb,[alb] a_latch_backoff%18.3f%S\n";
static const char str_azb[] PROGMEM = "azb,[azb] a_zero_backoff%19.3f%S\n";

static const char str_bam[] PROGMEM = "bam,[bam] b_axis_mode%18d %S\n";
static const char str_bfr[] PROGMEM = "bfr,[bfr] b_feedrate_maximum%15.3f%S/min\n";
static const char str_bvm[] PROGMEM = "bvm,[bvm] b_velocity_maximum%15.3f%S/min\n";
static const char str_btm[] PROGMEM = "btm,[btm] b_travel_maximum%17.3f%S\n";
static const char str_bjm[] PROGMEM = "bjm,[bjm] b_jerk_maximum%15.0f%S/min^3\n";
static const char str_bjd[] PROGMEM = "bjd,[bjd] b_junction_deviation%14.4f%S\n";
static const char str_bra[] PROGMEM = "bra,[bra] b_radius_value%20.4f%S\n";
static const char str_bsn[] PROGMEM = "bsn,[bsn] b_switch_min%17d [0-4]\n";
static const char str_bsx[] PROGMEM = "bsx,[bsx] b_switch_max%17d [0-4]\n";
static const char str_bsv[] PROGMEM = "bsv,[bsv] b_search_velocity%16.3f%S/min\n";
static const char str_blv[] PROGMEM = "blv,[blv] b_latch_velocity%17.3f%S/min\n";
static const char str_blb[] PROGMEM = "blb,[blb] b_latch_backoff%18.3f%S\n";
static const char str_bzb[] PROGMEM = "bzb,[bzb] b_zero_backoff%19.3f%S\n";

static const char str_cam[] PROGMEM = "cam,[cam] c_axis_mode%18d %S\n";
static const char str_cfr[] PROGMEM = "cfr,[cfr] c_feedrate_maximum%15.3f%S/min\n";
static const char str_cvm[] PROGMEM = "cvm,[cvm] c_velocity_maximum%15.3f%S/min\n";
static const char str_ctm[] PROGMEM = "ctm,[ctm] c_travel_maximum%17.3f%S\n";
static const char str_cjm[] PROGMEM = "cjm,[cjm] c_jerk_maximum%15.0f%S/min^3\n";
static const char str_cjd[] PROGMEM = "cjd,[cjd] c_junction_deviation%14.4f%S\n";
static const char str_cra[] PROGMEM = "cra,[cra] c_radius_value%20.4f%S\n";
static const char str_csn[] PROGMEM = "csn,[csn] c_switch_min%17d [0-4]\n";
static const char str_csx[] PROGMEM = "csx,[csx] c_switch_max%17d [0-4]\n";
static const char str_csv[] PROGMEM = "csv,[csv] c_search_velocity%16.3f%S/min\n";
static const char str_clv[] PROGMEM = "clv,[clv] c_latch_velocity%17.3f%S/min\n";
static const char str_clb[] PROGMEM = "clb,[clb] c_latch_backoff%18.3f%S\n";
static const char str_czb[] PROGMEM = "czb,[czb] c_zero_backoff%19.3f%S\n";

// PWM strings
static const char str_pwmf[] PROGMEM = "pwmf,[pwmf] pwm_frequency   %15.3fHz\n";
static const char str_p3sl[] PROGMEM = "p3sl,[p3sl] pwm_cw_speed_lo %15.3fRPM\n";
static const char str_p3sh[] PROGMEM = "p3sh,[p3sh] pwm_cw_speed_hi %15.3fRPM\n";
static const char str_p3pl[] PROGMEM = "p3pl,[p3pl] pwm_cw_phase_lo %13.3f [0..1]\n";
static const char str_p3ph[] PROGMEM = "p3ph,[p3ph] pwm_cw_phase_hi %13.3f [0..1]\n";
static const char str_p4sl[] PROGMEM = "p4sl,[p4sl] pwm_ccw_speed_lo%15.3fRPM\n";
static const char str_p4sh[] PROGMEM = "p4sh,[p4sh] pwm_ccw_speed_hi%15.3fRPM\n";
static const char str_p4pl[] PROGMEM = "p4pl,[p4pl] pwm_ccw_phase_lo%13.3f [0..1]\n";
static const char str_p4ph[] PROGMEM = "p4ph,[p4ph] pwm_ccw_phase_hi%13.3f [0..1]\n";
static const char str_p5of[] PROGMEM = "p5of,[p5of] pwm_phase_off   %13.3f [0..1]\n";

// Coordinate system offset groups
static const char str_g54x[] PROGMEM = "g54x,[g54x] g54_x_offset%20.3f%S\n";
static const char str_g54y[] PROGMEM = "g54y,[g54y] g54_y_offset%20.3f%S\n";
static const char str_g54z[] PROGMEM = "g54z,[g54z] g54_z_offset%20.3f%S\n";
static const char str_g54a[] PROGMEM = "g54a,[g54a] g54_a_offset%20.3f%S\n";
static const char str_g54b[] PROGMEM = "g54b,[g54b] g54_b_offset%20.3f%S\n";
static const char str_g54c[] PROGMEM = "g54c,[g54c] g54_c_offset%20.3f%S\n";

static const char str_g55x[] PROGMEM = "g55x,[g55x] g55_x_offset%20.3f%S\n";
static const char str_g55y[] PROGMEM = "g55y,[g55y] g55_y_offset%20.3f%S\n";
static const char str_g55z[] PROGMEM = "g55z,[g55z] g55_z_offset%20.3f%S\n";
static const char str_g55a[] PROGMEM = "g55a,[g55a] g55_a_offset%20.3f%S\n";
static const char str_g55b[] PROGMEM = "g55b,[g55b] g55_b_offset%20.3f%S\n";
static const char str_g55c[] PROGMEM = "g55c,[g55c] g55_c_offset%20.3f%S\n";

static const char str_g56x[] PROGMEM = "g56x,[g56x] g56_x_offset%20.3f%S\n";
static const char str_g56y[] PROGMEM = "g56y,[g56y] g56_y_offset%20.3f%S\n";
static const char str_g56z[] PROGMEM = "g56z,[g56z] g56_z_offset%20.3f%S\n";
static const char str_g56a[] PROGMEM = "g56a,[g56a] g56_a_offset%20.3f%S\n";
static const char str_g56b[] PROGMEM = "g56b,[g56b] g56_b_offset%20.3f%S\n";
static const char str_g56c[] PROGMEM = "g56c,[g56c] g56_c_offset%20.3f%S\n";

static const char str_g57x[] PROGMEM = "g57x,[g57x] g57_x_offset%20.3f%S\n";
static const char str_g57y[] PROGMEM = "g57y,[g57y] g57_y_offset%20.3f%S\n";
static const char str_g57z[] PROGMEM = "g57z,[g57z] g57_z_offset%20.3f%S\n";
static const char str_g57a[] PROGMEM = "g57a,[g57a] g57_a_offset%20.3f%S\n";
static const char str_g57b[] PROGMEM = "g57b,[g57b] g57_b_offset%20.3f%S\n";
static const char str_g57c[] PROGMEM = "g57c,[g57c] g57_c_offset%20.3f%S\n";

static const char str_g58x[] PROGMEM = "g58x,[g58x] g58_x_offset%20.3f%S\n";
static const char str_g58y[] PROGMEM = "g58y,[g58y] g58_y_offset%20.3f%S\n";
static const char str_g58z[] PROGMEM = "g58z,[g58z] g58_z_offset%20.3f%S\n";
static const char str_g58a[] PROGMEM = "g58a,[g58a] g58_a_offset%20.3f%S\n";
static const char str_g58b[] PROGMEM = "g58b,[g58b] g58_b_offset%20.3f%S\n";
static const char str_g58c[] PROGMEM = "g58c,[g58c] g58_c_offset%20.3f%S\n";

static const char str_g59x[] PROGMEM = "g59x,[g59x] g59_x_offset%20.3f%S\n";
static const char str_g59y[] PROGMEM = "g59y,[g59y] g59_y_offset%20.3f%S\n";
static const char str_g59z[] PROGMEM = "g59z,[g59z] g59_z_offset%20.3f%S\n";
static const char str_g59a[] PROGMEM = "g59a,[g59a] g59_a_offset%20.3f%S\n";
static const char str_g59b[] PROGMEM = "g59b,[g59b] g59_b_offset%20.3f%S\n";
static const char str_g59c[] PROGMEM = "g59c,[g59c] g59_c_offset%20.3f%S\n";

// Persistence for status report vector
static const char str_sr00[] PROGMEM = "sr00,";
static const char str_sr01[] PROGMEM = "sr01,";
static const char str_sr02[] PROGMEM = "sr02,";
static const char str_sr03[] PROGMEM = "sr03,";
static const char str_sr04[] PROGMEM = "sr04,";
static const char str_sr05[] PROGMEM = "sr05,";
static const char str_sr06[] PROGMEM = "sr06,";
static const char str_sr07[] PROGMEM = "sr07,";
static const char str_sr08[] PROGMEM = "sr08,";
static const char str_sr09[] PROGMEM = "sr09,";
static const char str_sr10[] PROGMEM = "sr10,";
static const char str_sr11[] PROGMEM = "sr11,";
static const char str_sr12[] PROGMEM = "sr12,";
static const char str_sr13[] PROGMEM = "sr13,";
static const char str_sr14[] PROGMEM = "sr14,";
static const char str_sr15[] PROGMEM = "sr15,";
static const char str_sr16[] PROGMEM = "sr16,";
static const char str_sr17[] PROGMEM = "sr17,";
static const char str_sr18[] PROGMEM = "sr18,";
static const char str_sr19[] PROGMEM = "sr19,";

// Group strings
static const char str_1[] PROGMEM = "1,";			// motor groups
static const char str_2[] PROGMEM = "2,";
static const char str_3[] PROGMEM = "3,";
static const char str_4[] PROGMEM = "4,";
static const char str_x[] PROGMEM = "x,";			// axis groups
static const char str_y[] PROGMEM = "y,";
static const char str_z[] PROGMEM = "z,";
static const char str_a[] PROGMEM = "a,";
static const char str_b[] PROGMEM = "b,";
static const char str_c[] PROGMEM = "c,";
static const char str_g54[] PROGMEM = "g54,";		// coordinate system offset groups
static const char str_g55[] PROGMEM = "g55,";
static const char str_g56[] PROGMEM = "g56,";
static const char str_g57[] PROGMEM = "g57,";
static const char str_g58[] PROGMEM = "g58,";
static const char str_g59[] PROGMEM = "g59,";
static const char str_g92[] PROGMEM = "g92,";		// origin offsets
static const char str_sys[] PROGMEM = "sys,";		// system group
static const char str_s[] PROGMEM = "s,";			// system group alias
static const char str_pos[] PROGMEM = "pos,";		// work position group
static const char str_mpo[] PROGMEM = "mpo,";		// machine position group

// Groups of groups (for text-mode display only)
static const char str_moto[] PROGMEM = "m,";		// display all motor groups
static const char str_axes[] PROGMEM = "n,";		// display all axis groups
static const char str_ofs[] PROGMEM = "o,";			// display all offsets
static const char str_all[] PROGMEM = "$,";			// display all parameters

// Help screen
static const char str_h[] PROGMEM = "h,";			// help screen

/***** PROGMEM config array **************************************************
 *
 *	NOTE: Token matching occurs from the most specific to the least specific.
 *		  This means that if shorter tokens overlap longer ones the longer one
 *		  must precede the shorter one. E.g. "gco" needs to comes before "gc"
 */

struct cfgItem const cfgArray[] PROGMEM = {

//	 string *, print func, get func, set func  target for get/set,    default value
	{ str_fb, _print_dbl, _get_dbl, _set_nul, (double *)&tg.build,   TINYG_BUILD_NUMBER }, // MUST BE FIRST!
	{ str_fv, _print_dbl, _get_dbl, _set_nul, (double *)&tg.version, TINYG_VERSION_NUMBER },
	{ str_id, _print_int, _get_id,  _set_nul, (double *)&tg.null, 0},	// device ID (signature)

	// dynamic model attributes for reporting puropses
	{ str_line,_print_int, _get_line,_set_int, (double *)&gm.linenum, 0 }, // line number - gets runtime line number
	{ str_feed,_print_lin, _get_dbu, _set_nul, (double *)&tg.null, 0 },	// feed rate
	{ str_stat,_print_str, _get_stat,_set_nul, (double *)&tg.null, 0 },	// combined machine state
	{ str_macs,_print_str, _get_macs,_set_nul, (double *)&tg.null, 0 },	// raw machine state
	{ str_cycs,_print_str, _get_cycs,_set_nul, (double *)&tg.null, 0 },	// cycle state
	{ str_mots,_print_str, _get_mots,_set_nul, (double *)&tg.null, 0 },	// motion state
	{ str_hold,_print_str, _get_hold,_set_nul, (double *)&tg.null, 0 },	// feedhold state
	{ str_home,_print_str, _get_home,_set_nul, (double *)&tg.null, 0 },	// homing state
	{ str_vel, _print_lin, _get_vel, _set_nul, (double *)&tg.null, 0 },	// current velocity
	{ str_unit,_print_str, _get_unit,_set_nul, (double *)&tg.null, 0 },	// units mode
	{ str_coor,_print_str, _get_coor,_set_nul, (double *)&tg.null, 0 },	// coordinate system
	{ str_momo,_print_str, _get_momo,_set_nul, (double *)&tg.null, 0 },	// motion mode
	{ str_plan,_print_str, _get_plan,_set_nul, (double *)&tg.null, 0 },	// plane select
	{ str_path,_print_str, _get_path,_set_nul, (double *)&tg.null, 0 },	// path control mode
	{ str_dist,_print_str, _get_dist,_set_nul, (double *)&tg.null, 0 },	// distance mode
	{ str_frmo,_print_str, _get_frmo,_set_nul, (double *)&tg.null, 0 },	// feed rate mode
	{ str_posx,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// X position
	{ str_posy,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// Y position
	{ str_posz,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// Z position
	{ str_posa,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// A position
	{ str_posb,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// B position
	{ str_posc,_print_pos, _get_pos, _set_nul, (double *)&tg.null, 0 },	// C position
	{ str_mpox,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// X machine position
	{ str_mpoy,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// Y machine position
	{ str_mpoz,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// Z machine position
	{ str_mpoa,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// A machine position
	{ str_mpob,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// B machine position
	{ str_mpoc,_print_pos, _get_mpos,_set_nul, (double *)&tg.null, 0 },	// C machine position
	{ str_g92x,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[X], 0 },// G92 offsets
	{ str_g92y,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[Y], 0 },
	{ str_g92z,_print_lin, _get_dbu, _set_nul, (double *)&gm.origin_offset[Z], 0 },
	{ str_g92a,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[A], 0 },
	{ str_g92b,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[B], 0 },
	{ str_g92c,_print_rot, _get_dbl, _set_nul, (double *)&gm.origin_offset[C], 0 },
	{ str_lx,  _print_int, _get_lx,  _set_lx,  (double *)&tg.null ,0 },	// line index - get/set runtime line index
	{ str_pb,  _print_int, _get_pb,  _set_nul, (double *)&tg.null, 0 },	// planner buffers available
	{ str_rx,  _print_int, _get_rx,  _set_nul, (double *)&tg.null, 0 },	// space in RX buffer

	{ str_sr, _print_sr,  _get_sr,  _set_sr,  (double *)&tg.null, 0 },	// status report object
	{ str_qr, _print_nul, _get_qr,  _set_nul, (double *)&tg.null, 0 },	// queue report object

	// Reports, tests, help, and messages
	{ str_test,help_print_test_help,    _get_ui8, tg_test, (double *)&tg.test,0 },// prints test help screen
	{ str_help,help_print_config_help,  _get_nul,_set_nul, (double *)&tg.null,0 },// prints config help screen
	{ str_defa,help_print_defaults_help,_get_nul,_set_defa,(double *)&tg.null,0 },// prints defaults help screen
	{ str_msg, _print_str,              _get_nul,_set_nul, (double *)&tg.null,0 },// string for generic messages

	// System parameters
	// NOTE: The ordering within the gcode defaults is important for token resolution
	{ str_gpl, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.select_plane,			GCODE_DEFAULT_PLANE },
	{ str_gun, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.units_mode,			GCODE_DEFAULT_UNITS },
	{ str_gco, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.coord_system,			GCODE_DEFAULT_COORD_SYSTEM },
	{ str_gpa, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.path_control,			GCODE_DEFAULT_PATH_CONTROL },
	{ str_gdi, _print_ui8, _get_ui8,_set_ui8, (double *)&cfg.distance_mode,			GCODE_DEFAULT_DISTANCE_MODE },
	{ str_gc,  _print_nul, _get_gc, _run_gc,  (double *)&tg.null, 0 }, // gcode block - must be last in this group

//	{ str_ea, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.enable_acceleration, 	ENABLE_ACCELERATION },
	{ str_ja, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.junction_acceleration,	JUNCTION_ACCELERATION },
	{ str_ml, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.min_segment_len,		MIN_LINE_LENGTH },
	{ str_ma, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.arc_segment_len,		ARC_SEGMENT_LENGTH },
	{ str_mt, _print_lin, _get_dbl, _set_dbl, (double *)&cfg.estd_segment_usec,		NOM_SEGMENT_USEC },
	{ str_st, _print_ui8, _get_ui8, _set_sw,  (double *)&sw.switch_type,			SWITCH_TYPE },

	{ str_ic, _print_ui8, _get_ui8, _set_ic,  (double *)&cfg.ignore_crlf,			COM_IGNORE_CRLF },
//	{ str_ec, _print_ui8, _get_ui8, _set_ec,  (double *)&cfg.enable_cr,				COM_APPEND_TX_CR },
	{ str_ee, _print_ui8, _get_ui8, _set_ee,  (double *)&cfg.enable_echo,			COM_ENABLE_ECHO },
	{ str_ex, _print_ui8, _get_ui8, _set_ex,  (double *)&cfg.enable_xon,			COM_ENABLE_XON },
	{ str_eq, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.enable_qr,				COM_ENABLE_QR },
	{ str_ej, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.comm_mode,				COM_COMMUNICATIONS_MODE },
	{ str_je, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.json_echo_mode,		COM_JSON_ECHO_MODE },
	{ str_si, _print_dbl, _get_int, _set_si,  (double *)&cfg.status_report_interval,STATUS_REPORT_INTERVAL_MS },
	{ str_baud,_print_ui8,_get_ui8, _set_baud,(double *)&cfg.usb_baud_rate,			XIO_BAUD_115200 },

	// Motor parameters
	{ str_1ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].motor_map,	M1_MOTOR_MAP },
	{ str_1sa, _print_rot, _get_dbl ,_set_sa, (double *)&cfg.m[MOTOR_1].step_angle,	M1_STEP_ANGLE },
	{ str_1tr, _print_lin, _get_dbu ,_set_tr, (double *)&cfg.m[MOTOR_1].travel_rev,	M1_TRAVEL_PER_REV },
	{ str_1mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_1].microsteps,	M1_MICROSTEPS },
	{ str_1po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_1].polarity,	M1_POLARITY },
	{ str_1pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].power_mode,	M1_POWER_MODE },

	{ str_2ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].motor_map,	M2_MOTOR_MAP },
	{ str_2sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_2].step_angle,	M2_STEP_ANGLE },
	{ str_2tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_2].travel_rev,	M2_TRAVEL_PER_REV },
	{ str_2mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_2].microsteps,	M2_MICROSTEPS },
	{ str_2po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_2].polarity,	M2_POLARITY },
	{ str_2pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].power_mode,	M2_POWER_MODE },

	{ str_3ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].motor_map,	M3_MOTOR_MAP },
	{ str_3sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_3].step_angle,	M3_STEP_ANGLE },
	{ str_3tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_3].travel_rev,	M3_TRAVEL_PER_REV },
	{ str_3mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_3].microsteps,	M3_MICROSTEPS },
	{ str_3po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_3].polarity,	M3_POLARITY },
	{ str_3pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].power_mode,	M3_POWER_MODE },

	{ str_4ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].motor_map,	M4_MOTOR_MAP },
	{ str_4sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_4].step_angle,	M4_STEP_ANGLE },
	{ str_4tr, _print_lin, _get_dbu, _set_tr, (double *)&cfg.m[MOTOR_4].travel_rev,	M4_TRAVEL_PER_REV },
	{ str_4mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_4].microsteps,	M4_MICROSTEPS },
	{ str_4po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_4].polarity,	M4_POLARITY },
	{ str_4pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].power_mode,	M4_POWER_MODE },

	// Axis parameters
	{ str_xam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[X].axis_mode,		X_AXIS_MODE },
	{ str_xvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].velocity_max,		X_VELOCITY_MAX },
	{ str_xfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].feedrate_max,		X_FEEDRATE_MAX },
	{ str_xtm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].travel_max,		X_TRAVEL_MAX },
	{ str_xjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].jerk_max,			X_JERK_MAX },
	{ str_xjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].junction_dev,		X_JUNCTION_DEVIATION },
	{ str_xsn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[0],				X_SWITCH_MODE_MIN },
	{ str_xsx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[1],				X_SWITCH_MODE_MAX },
	{ str_xsv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].search_velocity,	X_SEARCH_VELOCITY },
	{ str_xlv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].latch_velocity,	X_LATCH_VELOCITY },
	{ str_xlb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].latch_backoff,	X_LATCH_BACKOFF },
	{ str_xzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].zero_backoff,		X_ZERO_BACKOFF },

	{ str_yam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[Y].axis_mode,		Y_AXIS_MODE },
	{ str_yvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].velocity_max,		Y_VELOCITY_MAX },
	{ str_yfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].feedrate_max,		Y_FEEDRATE_MAX },
	{ str_ytm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].travel_max,		Y_TRAVEL_MAX },
	{ str_yjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].jerk_max,			Y_JERK_MAX },
	{ str_yjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].junction_dev,		Y_JUNCTION_DEVIATION },
	{ str_ysn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[2],				Y_SWITCH_MODE_MIN },
	{ str_ysx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[3],				Y_SWITCH_MODE_MAX },
	{ str_ysv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].search_velocity,	Y_SEARCH_VELOCITY },
	{ str_ylv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].latch_velocity,	Y_LATCH_VELOCITY },
	{ str_ylb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].latch_backoff,	Y_LATCH_BACKOFF },
	{ str_yzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].zero_backoff,		Y_ZERO_BACKOFF },

	{ str_zam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[Z].axis_mode,		Z_AXIS_MODE },
	{ str_zvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].velocity_max, 	Z_VELOCITY_MAX },
	{ str_zfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].feedrate_max, 	Z_FEEDRATE_MAX },
	{ str_ztm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].travel_max,		Z_TRAVEL_MAX },
	{ str_zjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].jerk_max,			Z_JERK_MAX },
	{ str_zjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].junction_dev, 	Z_JUNCTION_DEVIATION },
	{ str_zsn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[4],				Z_SWITCH_MODE_MIN },
	{ str_zsx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[5],				Z_SWITCH_MODE_MAX },
	{ str_zsv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].search_velocity,	Z_SEARCH_VELOCITY },
	{ str_zlv, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].latch_velocity,	Z_LATCH_VELOCITY },
	{ str_zlb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].latch_backoff,	Z_LATCH_BACKOFF },
	{ str_zzb, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].zero_backoff,		Z_ZERO_BACKOFF },

	{ str_aam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[A].axis_mode,		A_AXIS_MODE },
	{ str_avm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].velocity_max, 	A_VELOCITY_MAX },
	{ str_afr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].feedrate_max, 	A_FEEDRATE_MAX },
	{ str_atm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].travel_max,		A_TRAVEL_MAX },
	{ str_ajm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].jerk_max,			A_JERK_MAX },
	{ str_ajd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].junction_dev, 	A_JUNCTION_DEVIATION },
	{ str_ara, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].radius,			A_RADIUS},
	{ str_asn, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[6],				A_SWITCH_MODE_MIN },
	{ str_asx, _print_ui8, _get_ui8, _set_sw, (double *)&sw.mode[7],				A_SWITCH_MODE_MAX },
	{ str_asv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].search_velocity,	A_SEARCH_VELOCITY },
	{ str_alv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].latch_velocity,	A_LATCH_VELOCITY },
	{ str_alb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].latch_backoff,	A_LATCH_BACKOFF },
	{ str_azb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].zero_backoff,		A_ZERO_BACKOFF },

	{ str_bam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[B].axis_mode,		B_AXIS_MODE },
	{ str_bvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].velocity_max, 	B_VELOCITY_MAX },
	{ str_bfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].feedrate_max, 	B_FEEDRATE_MAX },
	{ str_btm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].travel_max,		B_TRAVEL_MAX },
	{ str_bjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].jerk_max,			B_JERK_MAX },
	{ str_bjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].junction_dev, 	B_JUNCTION_DEVIATION },
	{ str_bra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].radius,			B_RADIUS },
	{ str_bsv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].search_velocity,	B_SEARCH_VELOCITY },
	{ str_blv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].latch_velocity,	B_LATCH_VELOCITY },
	{ str_blb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].latch_backoff,	B_LATCH_BACKOFF },
	{ str_bzb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].zero_backoff,		B_ZERO_BACKOFF },

	{ str_cam, _print_am,  _get_am,  _set_am, (double *)&cfg.a[C].axis_mode,		C_AXIS_MODE },
	{ str_cvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].velocity_max, 	C_VELOCITY_MAX },
	{ str_cfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].feedrate_max, 	C_FEEDRATE_MAX },
	{ str_ctm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].travel_max,		C_TRAVEL_MAX },
	{ str_cjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].jerk_max,			C_JERK_MAX },
	{ str_cjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].junction_dev,		C_JUNCTION_DEVIATION },
	{ str_cra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].radius,			C_RADIUS },
	{ str_csv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].search_velocity,	C_SEARCH_VELOCITY },
	{ str_clv, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].latch_velocity,	C_LATCH_VELOCITY },
	{ str_clb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].latch_backoff,	C_LATCH_BACKOFF },
	{ str_czb, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].zero_backoff,		C_ZERO_BACKOFF },

    // PWM settings
    { str_pwmf, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.frequency,			P1_PWM_FREQUENCY },
    { str_p3sl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.cw_speed_lo,        P1_CW_SPEED_LO },
    { str_p3sh, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.cw_speed_hi,        P1_CW_SPEED_HI },
    { str_p3pl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.cw_phase_lo,        P1_CW_PHASE_LO },
    { str_p3ph, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.cw_phase_hi,        P1_CW_PHASE_HI },
    { str_p4sl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_speed_lo,       P1_CCW_SPEED_LO },
    { str_p4sh, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_speed_hi,       P1_CCW_SPEED_HI },
    { str_p4pl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_phase_lo,       P1_CCW_PHASE_LO },
    { str_p4ph, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.ccw_phase_hi,       P1_CCW_PHASE_HI },
    { str_p5of, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.p.phase_off,          P1_PWM_PHASE_OFF },

	// Coordinate system offsets
	{ str_g54x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][X], G54_X_OFFSET },
	{ str_g54y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][Y], G54_Y_OFFSET },
	{ str_g54z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][Z], G54_Z_OFFSET },
	{ str_g54a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][A], G54_A_OFFSET },
	{ str_g54b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][B], G54_B_OFFSET },
	{ str_g54c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G54][C], G54_C_OFFSET },

	{ str_g55x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][X], G55_X_OFFSET },
	{ str_g55y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][Y], G55_Y_OFFSET },
	{ str_g55z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][Z], G55_Z_OFFSET },
	{ str_g55a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][A], G55_A_OFFSET },
	{ str_g55b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][B], G55_B_OFFSET },
	{ str_g55c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G55][C], G55_C_OFFSET },

	{ str_g56x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][X], G56_X_OFFSET },
	{ str_g56y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][Y], G56_Y_OFFSET },
	{ str_g56z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][Z], G56_Z_OFFSET },
	{ str_g56a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][A], G56_A_OFFSET },
	{ str_g56b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][B], G56_B_OFFSET },
	{ str_g56c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G56][C], G56_C_OFFSET },

	{ str_g57x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][X], G57_X_OFFSET },
	{ str_g57y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][Y], G57_Y_OFFSET },
	{ str_g57z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][Z], G57_Z_OFFSET },
	{ str_g57a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][A], G57_A_OFFSET },
	{ str_g57b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][B], G57_B_OFFSET },
	{ str_g57c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G57][C], G57_C_OFFSET },

	{ str_g58x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][X], G58_X_OFFSET },
	{ str_g58y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][Y], G58_Y_OFFSET },
	{ str_g58z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][Z], G58_Z_OFFSET },
	{ str_g58a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][A], G58_A_OFFSET },
	{ str_g58b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][B], G58_B_OFFSET },
	{ str_g58c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G58][C], G58_C_OFFSET },

	{ str_g59x, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][X], G59_X_OFFSET },
	{ str_g59y, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][Y], G59_Y_OFFSET },
	{ str_g59z, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][Z], G59_Z_OFFSET },
	{ str_g59a, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][A], G59_A_OFFSET },
	{ str_g59b, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][B], G59_B_OFFSET },
	{ str_g59c, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.offset[G59][C], G59_C_OFFSET },

	// Persistence for status report - must be in sequence
	{ str_sr00, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[0],0 },
	{ str_sr01, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[1],0 },
	{ str_sr02, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[2],0 },
	{ str_sr03, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[3],0 },
	{ str_sr04, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[4],0 },
	{ str_sr05, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[5],0 },
	{ str_sr06, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[6],0 },
	{ str_sr07, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[7],0 },
	{ str_sr08, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[8],0 },
	{ str_sr09, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[9],0 },
	{ str_sr10, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[10],0 },
	{ str_sr11, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[11],0 },
	{ str_sr12, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[12],0 },
	{ str_sr13, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[13],0 },
	{ str_sr14, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[14],0 },
	{ str_sr15, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[15],0 },
	{ str_sr16, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[16],0 },
	{ str_sr17, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[17],0 },
	{ str_sr18, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[18],0 },
	{ str_sr19, _print_nul, _get_int, _set_int,(double *)&cfg.status_report_spec[19],0 },
	
	// Group lookups - must follow the single-valued entries for proper sub-string matching
	{ str_sys, _print_nul, _get_sys, _set_grp,(double *)&tg.null,0 },	// system group 	   (must be 1st)
	{ str_s, _print_nul, _get_sys, _set_grp,(double *)&tg.null,0 },		// alias for sys group (must be 2nd)
	{ str_1, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },		// motor groups
	{ str_2, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_3, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_4, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_x, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },		// axis groups
	{ str_y, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_z, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_a, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_b, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_c, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g54, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// coord offset groups
	{ str_g55, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g56, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g57, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g58, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g59, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },
	{ str_g92, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// origin offsets
	{ str_pos, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// work position group
	{ str_mpo, _print_nul, _get_grp, _set_grp,(double *)&tg.null,0 },	// machine position group

	// Uber-group (groups of groups, for text-mode displays only)
	{ str_moto, _print_nul, _do_motors, _set_nul,(double *)&tg.null,0 },
	{ str_axes, _print_nul, _do_axes,   _set_nul,(double *)&tg.null,0 },
	{ str_ofs,  _print_nul, _do_offsets,_set_nul,(double *)&tg.null,0 },
	{ str_all,  _print_nul, _do_all,    _set_nul,(double *)&tg.null,0 },

	// Help display
	{ str_h, help_print_config_help, _get_nul, _set_nul,(double *)&tg.null,0 }

// *** REMEMBER TO UPDATE CMD_COUNT_GROUPS (BELOW) IF YOU CHANGE THE GROUPS ****
};
#define CMD_INDEX_MAX (sizeof cfgArray / sizeof(struct cfgItem))

// hack alert. Find a better way to do this
#define CMD_COUNT_STATUS 20		// number of status report persistence elements - see final array [index]
#define CMD_COUNT_GROUPS 22		// count of simple groups
#define CMD_COUNT_UBER_GROUPS 4 // count of uber-groups

#define CMD_INDEX_END_SINGLES (CMD_INDEX_MAX - CMD_COUNT_STATUS - CMD_COUNT_GROUPS - CMD_COUNT_UBER_GROUPS)
#define CMD_INDEX_START_GROUPS (CMD_INDEX_MAX - CMD_COUNT_GROUPS - CMD_COUNT_UBER_GROUPS)
#define CMD_INDEX_START_UBER_GROUPS (CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS)

// Evaluators for the above:
static uint8_t _cmd_index_is_single(uint8_t index) {
	if (index <= CMD_INDEX_END_SINGLES) { return (true);} return (false);
}
static uint8_t _cmd_index_is_group(uint8_t index) {
	if ((index >= CMD_INDEX_START_GROUPS) && (index < CMD_INDEX_START_UBER_GROUPS)) { return (true);} return (false);
}
static uint8_t _cmd_index_is_uber_group(uint8_t index) {
	if (index >= CMD_INDEX_START_UBER_GROUPS) { return (true);} return (false);
}

/**** DEVICE ID ****
 * _get_id() - get device ID (signature)
 */
static uint8_t _get_id(cmdObj *cmd) 
{
//	uint8_t sig[11];
//	sys_read_signature(sig);
	return (TG_OK);
}

/**** STATUS REPORT REPORT FUNCTIONS ****
 * _get_sr()   - run status report
 * _print_sr() - run status report
 * _set_sr()   - set status report specification
 * _set_si()   - set status report interval
 *
 *	Note: _set_sr() is called during initialization and during reset when 
 *	there's actually nothing to do. So it rejects all get requests except 
 *	those where cmd->type == true and cmd->value == true.
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
	memset(cfg.status_report_spec, -1 , sizeof(cfg.status_report_spec));
	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if ((cmd->type == true) && (cmd->value == true)) { // see function header note
			cfg.status_report_spec[i] = cmd->index;
			cmd->value = cmd->index;	// you want to persist the index as the value
			cmd_write_NVM_value(cmd);	// persist the value
		}
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
 * _get_pb() - get planner buffers available
 * _get_rx() - get bytes available in RX buffer
 */
static uint8_t _get_qr(cmdObj *cmd) 
{
	rpt_run_queue_report();
	return (TG_OK);
}

static uint8_t _get_pb(cmdObj *cmd)
{
	cmd->value = (double)mp_get_planner_buffers_available();
	cmd->type = TYPE_INTEGER;
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
 * _get_lx()   - get runtime line index for queue reports
 * _set_lx()   - set runtime line index for queue reports
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

static uint8_t _get_lx(cmdObj *cmd)
{
	cmd->value = (double)mp_get_runtime_lineindex();
	cmd->type = TYPE_INTEGER;
	return (TG_OK);
}

static uint8_t _set_lx(cmdObj *cmd)
{
	mp_set_planner_lineindex((uint32_t)cmd->value);
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
	fprintf(stderr, _get_format(cmd->index,format), cmd->value, (PGM_P)pgm_read_word(&msg_units[units]));
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

	if (strchr(linear_axes, cmd->token[0]) != NULL) {		// true if it's a linear axis
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

/****************************************************************************
 * cfg_init() - called once on system init
 *
 * Will perform one of 2 actions:
 *	(1) if NVM is set up and at current config version: use NVM data for config
 *	(2) if NVM is set up or out-of-rev: load RAM and NVM with hardwired default settings
 */

void cfg_init()
{
	cmdObj cmd;
	cm_set_units_mode(MILLIMETERS);	// must do init in MM mode
	cmd_clear_list();				// setup the cmd object lists. Do this first.
	cfg.comm_mode = TG_JSON_MODE;	// initial value until EEPROM is read

#ifdef __DISABLE_EEPROM_INIT		// cutout for debug simulation

	// Apply the hard-coded default values from settings.h and exit
	for (cmd.index=0; _cmd_index_is_single(cmd.index); cmd.index++) {
		cmd_get_token(cmd.index, cmd.token);
		if (_dont_initialize(&cmd) == true) continue;
		cmd.value = (double)pgm_read_float(&cfgArray[cmd.index].def_value);
		cmd_set(&cmd);
	}
	rpt_init_status_report(false);	// requires special treatment (persist = false)
#else
	cfg.nvm_base_addr = NVM_BASE_ADDR;
	cfg.nvm_profile_base = cfg.nvm_base_addr;
	cmd.index = 0;					// this will read the first record in NVM
	cmd_read_NVM_value(&cmd);

	if (fp_EQ(cmd.value,tg.build)) { // Case (1) NVM is set up and current revision. Load config from NVM
		tg_print_loading_configs_message();
		for (cmd.index=0; _cmd_index_is_single(cmd.index); cmd.index++) {
			cmd_read_NVM_value(&cmd);
			cmd_get_token(cmd.index, cmd.token);
			if (_dont_initialize(&cmd) == true) continue;
			cmd_set(&cmd);
			cmd_persist(&cmd);
		}
	} else {  // Case (2) NVM is out-of-rev or not set up. Use the defaults and set up NVM
		cmd.value = true;
		_set_defa(&cmd);// this subroutune runs from here and also from the $defa=1 command
	}
	rpt_init_status_report(true);// requires special treatment (persist = true)
#endif
}

/*
 * _set_defa() - reset NVM with default values for active profile
 */ 

static uint8_t _set_defa(cmdObj *cmd) 
{
	if (cmd->value != true) {		// failsafe. Must set true or no action occurs
		help_print_defaults_help(cmd);
		return (TG_OK);
	}
	cm_set_units_mode(MILLIMETERS);	// must do init in MM mode
	tg_print_initializing_message();

	for (cmd->index=0; _cmd_index_is_single(cmd->index); cmd->index++) {
		cmd_get_token(cmd->index, cmd->token);
		if (_dont_initialize(cmd) == true) continue;
		cmd->value = (double)pgm_read_float(&cfgArray[cmd->index].def_value);
		if (cmd_set(cmd) == TG_NOOP) continue;
		cmd_persist(cmd);
	}
	return (TG_OK);
}

/*
 * _dont_initialize() - replaces this:
 *	if (strstr(DONT_INITIALIZE, cmd_get_token(cmd.index, cmd.token)) != NULL) continue;
 *	See config.h for DONT_INITIALIZE string
 */
static uint8_t _dont_initialize(cmdObj *cmd) 
{
	if (strstr("st", cmd->token) != NULL) return (false);
	if (strstr(DONT_INITIALIZE, cmd->token) != NULL) return (true);
	return (false);
}

/****************************************************************************
 * cfg_text_parser() - update a config setting from a text block (text mode)
 * 
 * Use cases (execution paths handled)
 *	- $xfr=1200	single parameter set is requested
 *	- $xfr		single parameter display is requested
 *	- $x		group display is requested
 */

uint8_t cfg_text_parser(char *str)
{
	cmdObj *cmd = cmd_body;					// point at first object in the body

	// handle status report case
	if (str[0] == '?') {
		rpt_run_multiline_status_report();
		return (TG_OK);
	}
	// single-unit parser processing
	ritorno(_parse_config_string(str, cmd));// decode the first object
	if ((cmd->type == TYPE_PARENT) || (cmd->type == TYPE_NULL)) {
		if (cmd_get(cmd) == TG_COMPLETE) {	// populate value, group values, or run uber-group displays
			return (TG_OK);					// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		cmd_set(cmd);						// set single value
		cmd_persist(cmd);
	}
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED); // print the results
	return (TG_OK);
}

/****************************************************************************
 * _parse_config_string() - parse a text-mode command line
 */
static uint8_t _parse_config_string(char *str, cmdObj *cmd)
{
	char *tmp;
	char separators[] = {" =:|\t"};			// anything someone might use

	// pre-processing
	cmd_clear_obj(cmd);						// initialize config object
	if (*str == '$') str++;					// ignore leading $
	tmp = str;
	if (*tmp==NUL) *tmp='s';				// make $ behave as a system listing
	for (; *tmp!=NUL; tmp++) {
		*tmp = tolower(*tmp);				// convert string to lower case
		// todo: put comma tolerance in here
		// todo: insert separator for xfr1000 case in here
	}
	// field processing
	cmd->type = TYPE_NULL;
	if ((tmp = strpbrk(str, separators)) == NULL) {
		strncpy(cmd->friendly_name, str, CMD_STRING_LEN);// no value part
	} else {
		*tmp = NUL;							// terminate at end of name
		strncpy(cmd->friendly_name, str, CMD_STRING_LEN);
		str = ++tmp;
		cmd->value = strtod(str, &tmp);		// tmp is the end pointer
		if (tmp != str) {
			cmd->type = TYPE_FLOAT;
		}
	}
	if ((cmd->index = cmd_get_index(cmd->friendly_name)) == -1) { 
		return (TG_UNRECOGNIZED_COMMAND);
	}
	cmd_get_token(cmd->index, cmd->token);
	if ((_cmd_index_is_group(cmd->index)) || (_cmd_index_is_uber_group(cmd->index))) {
		cmd->type = TYPE_PARENT;	// indicating it's a group token
		strncpy(cmd->group, cmd->token, CMD_TOKEN_LEN+1);	// copy group token into string field
	}
	return (TG_OK);
}

/****************************************************************************/
/**** CMD FUNCTIONS *********************************************************/
/****************************************************************************/
/* These are the primary access points to cmd functions
 *
 *	cmd_get() - Build a cmdObj with the values from the target & return the value
 * 				Populate cmd body with single valued elements or groups (iterates)
 *
 *	cmd_set() - Write a value or invoke a function
 * 				Operates on single valued elements or groups
 *
 * Single object functions:
 *
 *	cmd_formatted_print() - Output a formatted string for the value.
 *	cmd_persist() 		  - persist value to NVM. Takes special cases into account
 *	cmd_get_cmdObj()	  - like cmd_get but returns cmdObj pointer instead of the value
 */

uint8_t cmd_get(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd));
}

uint8_t cmd_set(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].set)))(cmd));
}

void cmd_formatted_print(cmdObj *cmd)
{
	if ((cmd->index < 0) || (cmd->index >= CMD_INDEX_MAX)) return;
	((fptrPrint)(pgm_read_word(&cfgArray[cmd->index].print)))(cmd);
}

void cmd_persist(cmdObj *cmd)
{
	if ((cmd->index < 0) || (_cmd_index_is_single(cmd->index) == false)) return;
	if (strstr(DONT_PERSIST, cmd->token) != NULL) return;
	cmd_write_NVM_value(cmd);
}

uint8_t cmd_get_cmdObj(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_UNRECOGNIZED_COMMAND);
	INDEX_T tmp = cmd->index;
	cmd_clear_obj(cmd);
	cmd_get_token((cmd->index = tmp), cmd->token);
	if (_cmd_index_is_group(cmd->index)) { strncpy(cmd->group, cmd->token, CMD_GROUP_LEN);}
	((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd);
	return (cmd->value);
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
	fprintf(stderr, _get_format(cmd->index, format), cmd->value, (PGM_P)pgm_read_word(&msg_units[2]));
}


/***************************************************************************** 
 * Accessors - get various data from an object given the index
 * cmd_get_token()	- return token in &arg and return pointer to token string
 * cmd_get_group() 	- return the axis prefix, motor prefix, or 's' for system
 * _get_format() 	- return format string as above
 * _get_motor()		- return the axis an index applies to or -1 if na
 * _get_pos_axis()	- return axis number for pos values or -1 if none - e.g. posx
 * _get_grp_axis()	- return axis for a group variable or -1 if na - e.g. xvm
 *
 *  NOTE: Axis and motor functions rely on the token naming conventions
 */

char *cmd_get_token(const INDEX_T i, char *token)
{
	if ((i < 0) || (i >= CMD_INDEX_MAX)) { 
		*token = NUL;
		return (token);
	}
	strncpy_P(token,(PGM_P)pgm_read_word(&cfgArray[i].string), CMD_TOKEN_LEN+1);
	char *ptr = strstr(token,",");			// find the first separating comma
	*ptr = NUL;								// terminate the string after the token
	return (token);
}
/*
char cmd_get_group(const INDEX_T i)
{
	char *ptr;
	char chr;
	char groups[] = {"xyzabc1234"};

	if ((i < 0) || (i >= CMD_INDEX_MAX)) return (NUL);
	strncpy_P(&chr,(PGM_P)pgm_read_word(&cfgArray[i].string), 1);
	if ((ptr = strchr(groups, chr)) == NULL) {
		return ('s');
	}
	return (*ptr);
}
*/

static char *_get_format(const INDEX_T i, char *format)
{
	char *ptr;
	char tmp[CMD_STRING_FIELD_LEN];

	strncpy_P(tmp,(PGM_P)pgm_read_word(&cfgArray[i].string), CMD_STRING_FIELD_LEN);
	ptr = strstr(tmp,",");					// find the first separating comma
//	ptr = strstr(++ptr,",");				// find the second comma (removed w/friendly names)
	ptr++;
	while (*ptr == ' ') ptr++;				// find the first non-whitespace
	strncpy(format, ptr, CMD_FORMAT_LEN+1);
	return (format);
}

static int8_t _get_motor(const INDEX_T i)
{
	char *ptr;
	char motors[] = {"1234"};
	char tmp[CMD_TOKEN_LEN+1];

	strncpy_P(tmp,(PGM_P)pgm_read_word(&cfgArray[i].string), CMD_TOKEN_LEN+1);
	if ((ptr = strchr(motors, tmp[0])) == NULL) {
		return (-1);
	}
	return (ptr - motors);
}

static int8_t _get_pos_axis(const INDEX_T i)
{
	char *ptr;
	char tmp[CMD_TOKEN_LEN];
	char axes[] = {"xyzabc"};

	strncpy_P(tmp,(PGM_P)pgm_read_word(&cfgArray[i].string),CMD_TOKEN_LEN);
	if ((ptr = strchr(axes, tmp[3])) == NULL) { return (-1);}
	return (ptr - axes);
}
/*
static int8_t _get_grp_axis(const INDEX_T i)
{
	char tmp;
	char *ptr;
	char axes[] = {"xyzabc"};

	strncpy_P(&tmp,(PGM_P)pgm_read_word(&cfgArray[i].string),1);
	if ((ptr = strchr(axes, tmp)) == NULL) {
		return (-1);
	}
	return (ptr - axes);
}
*/

/****************************************************************************
 * cmdObj helper functions and other low-level cmd helpers
 * cmd_get_max_index()		- utility function to return index array size				
 * cmd_get_index() 			- get index from mnenonic token
 * cmd_is_group()			- returns true if the command is a group
 * cmd_get_type()			- returns command type as a CMD_TYPE enum
 * cmd_persist_offsets()	- write any changed G54 (et al) offsets back to NVM
 *
 *	cmd_get_index() is the most expensive routine in the whole config. It does a 
 *	linear table scan of the PROGMEM strings, which of course could be further 
 *	optimized with indexes or hashing.
 */

//INDEX_T cmd_get_max_index() { return (CMD_INDEX_MAX);}

INDEX_T cmd_get_index(const char *str)
{
	char tmp;
	const char *Ptr;

	for (INDEX_T i=0; i<CMD_INDEX_MAX; i++) {
		Ptr = (const char *)pgm_read_word(&cfgArray[i]);
		if ((char)pgm_read_byte(Ptr++) != str[0]) continue;			// 1st char mis-match
		if ((tmp = (char)pgm_read_byte(Ptr++)) == ',') return (i);	// single character match
		if (tmp != str[1]) continue;								// 2nd char mis-match
		if ((tmp = (char)pgm_read_byte(Ptr++)) == ',') return (i);	// 2 character match
		if (tmp != str[2]) continue;								// 3rd char mis-match
		if ((tmp = (char)pgm_read_byte(Ptr++)) == ',') return (i);	// 3 character match
		if (tmp != str[3]) continue;								// 4rd char mis-match
		return (i);													// 4 character match
	}
	return (-1);							// no match
}
/* less efficient but more standard approach 
INDEX_T cmd_get_index(const char *str)
{
	uint8_t len;
	char cmp[CMD_TOKEN_LEN+1];

	// append a comma so PROGMEM match is exact, and handle string-too-long case
	snprintf(cmp, CMD_TOKEN_LEN+1, "%s,", str);// copy the token and trailing comma
	if ((len = strlen(cmp)) > CMD_TOKEN_LEN) {// if the string is too long append a trailing comma 
		cmp[CMD_TOKEN_LEN] = ',';
	}
	// scan the cfgArray strings for an exact match
	for (INDEX_T i=0; i<CMD_INDEX_MAX; i++) {
		if ((strncmp_P(cmp,(PGM_P)pgm_read_word(&cfgArray[i]), len)) == 0) {
			return (i);						// matched
		}
	}
	return (-1);							// no match
}

INDEX_T cmd_get_index(const char *str)		// older code - uses friendly name
{
	char *end;
	char *friendly_name;
	char token[CMD_NAMES_FIELD_LEN];

	for (INDEX_T i=0; i<CMD_INDEX_MAX; i++) {
		strncpy_P(token,(PGM_P)pgm_read_word(&cfgArray[i]), CMD_NAMES_FIELD_LEN);
		friendly_name = strstr(token,",");	// find the separating comma
		*(friendly_name++) = NUL;			// split token & name strings, terminate token string
		end = strstr(friendly_name,",");	// find the terminating comma
		*end = NUL;							// terminate the name string
		if (strcmp(str, token) == 0) { return(i);}
		if (strcmp(str, friendly_name) == 0) { return(i);}
	}
	return (-1);							// no match
}
*/

uint8_t cmd_is_group(const char *str)
{
	if (strstr(GROUP_PREFIXES, str) != NULL) return (true);
	return (false);
}

uint8_t cmd_get_type(cmdObj *cmd)
{
	if (strstr("gc", cmd->token) != NULL) {
		return (CMD_TYPE_GCODE);
	}
	if (strstr("sr,qr", cmd->token) != NULL) {
		return (CMD_TYPE_REPORT);
	}
	return (CMD_TYPE_CONFIG);
}

uint8_t cmd_persist_offsets(uint8_t flag)
{
	if (flag == true) {
		cmdObj cmd;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf(cmd.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				cmd.index = cmd_get_index(cmd.token);
				cmd.value = cfg.offset[i][j];
				cmd_write_NVM_value(&cmd);	// only writes changed values
			}
		}
	}
	return (TG_OK);
}

/********************************************************************************
 ***** Group operations *********************************************************
 ********************************************************************************
 * _set_grp()	- get data for axis, motor or coordinate offset group
 * _get_grp()	- get data for axis, motor or coordinate offset group
 * _get_sys()	- get data for system group
 *
 *	Group operations work on parent/child groups where the parent object is 
 *	one of the following groups:
 *	axis group 			x,y,z,a,b,c
 *	motor group			1,2,3,4
 *	coordinate group	g54,g55,g56,g57,g58,g59
 *	system group		"sys" - a collection of otherwise unrelated system variables
 *
 *	Groups are carried as parent / child objects, e.g:
 *	{"x":""}							get all X axis parameters
 *	{"x":{"vm":""}}						get X axis velocity max 
 *	{"x":{"vm":1000}}					set X axis velocity max
 *	{"x":{"vm":"","fr":""}}				get X axis velocity max and feed rate 
 *	{"x":{"vm":1000,"fr";900}}			set X axis velocity max and feed rate
 *	{"x":{"am":1,"fr":800,....}}		set multiple or all X axis parameters
 *
 *	The group prefixes are stripped from the child tokens for better alignment 
 *	with host code. I.e a group object is represented as:
 *	{"x":{"am":1,"fr":800,....}}, not:
 *	{"x":{"xam":1,"xfr":800,....}},
 *
 *	This strip makes no difference for subsequent internal operations as the 
 *	index is used and tokens are ignored once the parameter index is known.
 *	But it's useful to be able to round-trip a group back to the JSON requestor.
 *
 *	NOTE: The 'cmd' arg in many group commands must be the address of the head of
 *	a cmd struct array (cmd_body), not a single cmd struct. These commands expand 
 *	into groups of multiple cmd structs, and assume the array provides the RAM. 
 */

/*
 * _set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and a setter
 *	It iterates the group children and either gets the value or sets the value for 
 *	each depending on the cmd->type. To get an entire group w/o specifying the 
 *	child elements use _set_grp() 
 */
static uint8_t _set_grp(cmdObj *cmd)
{
	for (uint8_t i=0; i<CMD_MAX_OBJECTS; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if (cmd->type == TYPE_EMPTY) {
			break;
		} else if (cmd->type == TYPE_NULL) {// NULL means GET the value
			cmd_get(cmd);
		} else {
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (TG_OK);
}

static uint8_t _get_grp(cmdObj *cmd)
{
	char *group = cmd->group;				// group token in the parent cmd object
	INDEX_T group_index = cmd->index;
	char token[CMD_TOKEN_LEN+1];			// token retrived from cmdArray list
	char exclude[] = { GROUP_EXCLUSIONS };	// see config.h

	cmd->type = TYPE_PARENT;				// make first object the parent 
	for (INDEX_T i=0; i<group_index; i++) {	// stop before you recurse
		cmd_get_token(i,token);
		if (strstr(token, group) == token) {
			if (strstr(exclude, token) != NULL) continue;
			(++cmd)->index = i;
			cmd_get_cmdObj(cmd);
			strncpy(cmd->token, &cmd->token[strlen(group)], CMD_TOKEN_LEN+1);// strip group prefixes from token
		}
	}
	return (TG_OK);
}

static uint8_t _get_sys(cmdObj *cmd)
{
	char token[CMD_TOKEN_LEN+1];			// token retrived from cmdObj
	char include[] = { SYSTEM_GROUP };		// see config.h
	char exclude[] = {"gc"};

	cmd->type = TYPE_PARENT;
	for (INDEX_T i=0; i<=CMD_INDEX_END_SINGLES; i++) {
		cmd_get_token(i, token);
		if (strstr(exclude, token) != NULL) continue;
		if (strstr(include, token) != NULL) {
			(++cmd)->index = i;
			cmd_get_cmdObj(cmd);
			(cmd-1)->nx = cmd;	// set next object of previous object to this object
		}
	}
	return (TG_OK);
}

static void _print_sys(cmdObj *cmd) 
{
	_get_sys(cmd);
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);
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
		strncpy(cmd->group, list[i], CMD_TOKEN_LEN);
		cmd->index = cmd_get_index(cmd->group);
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
	char list[][CMD_TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92",""}; // must have a terminating element
	_do_group_list(cmd, list);
	return (TG_COMPLETE);
}

static uint8_t _do_all(cmdObj *cmd)		// print all parameters
{
	_print_sys(cmd);
	_do_offsets(cmd);
	_do_motors(cmd);
	_do_axes(cmd);
	return (TG_COMPLETE);
}

/********************************************************************************
 ***** cmdObj list initialization and manipulation ******************************
 ********************************************************************************
 * cmd_clear_obj() 	 - clear a command object (that you actually passed in)
 * cmd_clear_list()	 - clear entire header, body and footer
 * cmd_clear_body()	 - clear body 
 * cmd_add_token()	 - write contents of parameter to  first free object in the body
 * cmd_add_string()	 - add a string to end of cmd body
 * cmd_add_float()	 - add a floating point value to end of cmd body
 * cmd_add_integer() - add an integer value to end of cmd body (Note 1)
 *
 *	Note 1: adding a really large integer (like a checksum value) may lose 
 *	precision due to the cast to a double. Sometimes it's better to load an 
 *	integer as a string if all you want to do is display it.
 */

cmdObj *cmd_clear_obj(cmdObj *cmd)			// clear a single cmdObj structure
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

void cmd_clear_list()						// clear the header, body and footer
{
	// setup header ("b" parent)
	cmdObj *cmd = cmd_header;
	cmd->pv = 0;
	cmd->nx = cmd_body;
	cmd->index = 0;
	cmd->depth = 0;
	cmd->type = TYPE_PARENT;
	cmd->token[0] = 'b';
	cmd++;

	// setup body
	cmd_clear_body(cmd_body);

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

void cmd_clear_body(cmdObj *cmd)			// clear the body list
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
		cmd->depth = 1;
		cmd->type = TYPE_EMPTY;
		cmd++;
	}
	(--cmd)->nx = cmd_footer;				// correct last element
}

uint8_t cmd_add_token(char *token)			// add an object to the body using a token
{
	cmdObj *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index(token)) == -1) {
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
		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
		strncpy(cmd->string, string, CMD_STRING_LEN);
		cmd->index = cmd_get_index(cmd->token);
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
		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
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
		cmd->token[CMD_TOKEN_LEN-1] = NUL;	// safety measure
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
	// JSON handling. Kind of a hack. Generate the JSON string w/o the checksum hash. 
	// Then calculate the checksum and add it into the JSON string with proper termination
	if (cfg.comm_mode == TG_JSON_MODE) {
		js_print_list(status);
	} else {
		switch (textmode) {
			case TEXT_INLINE_PAIRS: { _print_text_inline_pairs(); break; }
			case TEXT_INLINE_VALUES: { _print_text_inline_values(); break; }
			case TEXT_MULTILINE_FORMATTED: { _print_text_multiline_formatted(); break; }
		}
	}
	cmd_clear_body(cmd_body);		// clear the cmd body to get ready for the next use
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
		cmd_formatted_print(cmd);
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
 */
uint8_t cmd_read_NVM_value(cmdObj *cmd)
{
	ASSERT_CMD_INDEX(TG_INTERNAL_ERROR);
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
//	cmd->type = TYPE_FLOAT;
	return (TG_OK);
}

uint8_t cmd_write_NVM_value(cmdObj *cmd)
{
	double original_value = cmd->value;
	ritorno(cmd_read_NVM_value(cmd));
	if (cmd->value != original_value) {
		cmd->value = original_value;
		int8_t nvm_byte_array[NVM_VALUE_LEN];
		memcpy(&nvm_byte_array, &original_value, NVM_VALUE_LEN);
		uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
	return (TG_OK);
}

/*
	ASSERT_CMD_INDEX(TG_INTERNAL_ERROR);
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	double nvm_value;
	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&nvm_value, &nvm_byte_array, sizeof(double));
	if (cmd->value != nvm_value) {
		memcpy(&nvm_byte_array, &cmd->value, sizeof(double));
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
	return(TG_OK);
}
*/

/****************************************************************************
 ***** Config Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TEST_CONFIG

void cfg_unit_tests()
{
//	cmdObj cmd;

// NVM tests
/*
	strcpy(cmd.token, "fc");
	cmd.value = 329.01;

	cmd_write_NVM(0, &cmd);
	cmd.value = 0;
	cmd_read_NVM(0, &cmd);
	cmd.value = 0;
	cmd_read_NVM(0, &cmd);
	cmd.nesting_level = 0;
// 	cfg_dump_NVM(0,10,PSTR("NVM dump"));
*/

// config table tests

	INDEX_T i;
//	double val;

//	_print_configs("$", NUL);					// no filter (show all)
//	_print_configs("$", 'g');					// filter for general parameters
//	_print_configs("$", '1');					// filter for motor 1
//	_print_configs("$", 'x');					// filter for x axis

//	i = cmd_get_index("fb");
//	i = cmd_get_index("xfr");
//	i = cmd_get_index("g54");

	i = _get_pos_axis(55);
	i = _get_pos_axis(73);
	i = _get_pos_axis(93);
	i = _get_pos_axis(113);

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

