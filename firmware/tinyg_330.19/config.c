/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *	Config system overview
 *
 *	Config has been rewritten to support JSON objects and to be easier to extend 
 *	and modify. Each configuration value is identified by a friendly name (name).
 *	The friendly name uniquely maps to a short mnemonic string (token), 
 *	which in turn finds the index into the config arrays.
 *
 *	Config keeps the following arrays:
 * 
 *	- PROGMEM array (cfgArray) contains typed data in program memory. Each item has:
 *		- function pointer for print() method
 *		- function pointer for get() method
 *		- function pointer for set() method
 *		- target (memory location that the value is written to)
 *		- default value - for cold initialization
 *		- pointer to combined string, a comma separated list which carries:
 *			- token string
 *			- friendly name lookup string (just long enough for matching)
 *			- format string for print formatting
 *
 *	- NVM array - Contains tokens and values persisted to EEPROM (NVM)
 *		The tokens are used for data migration across firmware versions.
 *
 *	The following rules apply to friendly names:
 *	 - can be up to 24 chars and cannot contain whitespace or separators ( =  :  | , )  
 *	 - must be unique (non colliding).
 *	 - are case insensitive (and usually written as all lowercase)
 *	 - by convention axis and motor friendly names start with the axis letter 
 *		(e.g. x_feedrate) or motor designator (e.g. m1_microsteps)
 *
 *	The following rules apply to mnemonic tokens
 *	 - are 2 or 3 characters and cannot contain whitespace or separators ( =  :  | , )
 *	 - must be unique (non colliding).
 *	 - axis tokens start with the axis letter and are 3 characters including the axis letter
 *	 - motor tokens start with the motor digit and are 3 characters including the motor digit
 *	 - non-axis or non-motor tokens are 2 characters and cannot start with: xyzabcuvw0123456789
 *
 *	Adding a new value to config (or changing an existing one) involves touching the following places:
 *	 - Add a token / friendly name / formatting string to str_XXX strings (ensure unique token & name!)
 *	 - Create a new record in cfgArray[] with includes:
 *		- reference to the above string
 *		- an existing print() function or create a new one if necessary 
 *		- an existing apply() fucntion or create a new one if necessary
 *		- target pointer (a variable must exist somewhere, often in the cfg struct)
 *		- default value for the parameter
 *	 - Change CFG_VERSION in config.h to something different so it will migrate ye olde configs in NVM.
 *
 * 	The order of display is set by the order of strArray. None of the other orders 
 *	matter but are generally kept sequenced for easier reading and code maintenance.
 *
 *	Command line vs JSON operation
 *
 *	Config can be used as command line (text-based) or using JSON objects. 
 *	All functions are identical and can be accessed either way. 
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
#include "planner.h"
#include "stepper.h"
#include "system.h"
#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

//*** STATIC STUFF ***********************************************************

typedef void (*fptrConfig)(INDEX_T i);	// required for PROGMEM access

struct cfgItem {			// structs for pgmArray
	char *string;			// pointer to names composite string
	fptrConfig print;		// print binding: aka void (*print)(INDEX_T i);
	fptrCmd get;			// GET binding aka uint8_t (*get)(const INDEX_T i, struct cmdObject *cmd[])
	fptrCmd set;			// SET binding aka uint8_t (*set)(const INDEX_T i, struct cmdObject *cmd[])
	double *target;			// target for writing config value
	double def_value;		// default value for config item
};

// generic internal functions
static uint8_t _set_nul(const INDEX_T i, cmdObj *cmd);	// noop
static uint8_t _set_ui8(const INDEX_T i, cmdObj *cmd);	// set a uint8_t value
static uint8_t _set_int(const INDEX_T i, cmdObj *cmd);	// set an integer value
static uint8_t _set_dbl(const INDEX_T i, cmdObj *cmd);	// set a double value
static uint8_t _set_dbu(const INDEX_T i, cmdObj *cmd);	// set a double with unit conversion

static uint8_t _get_nul(const INDEX_T i, cmdObj *cmd);	// get null value type
static uint8_t _get_ui8(const INDEX_T i, cmdObj *cmd);	// get uint8_t value
static uint8_t _get_int(const INDEX_T i, cmdObj *cmd);	// get uint32_t integer value
static uint8_t _get_dbl(const INDEX_T i, cmdObj *cmd);	// get double value
static uint8_t _get_dbu(const INDEX_T i, cmdObj *cmd);	// get double with unit conversion
static uint8_t _get_dbls(const INDEX_T i, cmdObj *cmd);	// get double & convert to string

static uint8_t _get_ui8_value(const INDEX_T i);			// return uint8_t value
static uint8_t _get_int_value(const INDEX_T i);			// return uint32_t value
static double _get_dbl_value(const INDEX_T i);			// return double
static double _get_dbu_value(const INDEX_T i);			// return double with unit conversion

static void _print_nul(const INDEX_T i);				// print nothing
static void _print_ui8(const INDEX_T i);				// print unit8_t value w/no units
static void _print_int(const INDEX_T i);				// print integer value w/no units
static void _print_dbl(const INDEX_T i);				// print double value w/no units
static void _print_lin(const INDEX_T i);				// print linear values
static void _print_rot(const INDEX_T i);				// print rotary values

// helpers
static char *_get_format(const INDEX_T i, char *format);
static int8_t _get_axis(const INDEX_T i);
static int8_t _get_motor(const INDEX_T i);
static uint8_t _parse_config_string(char *str, struct cmdObject *c);

/*****************************************************************************
 **** PARAMETER-SPECIFIC CODE REGION *****************************************
 **** This code and data will change as you add / update config parameters ***
 *****************************************************************************/

// specialized internal functions
static uint8_t _get_sr(const INDEX_T i, cmdObj *cmd);	// run status report
static uint8_t _set_sr(const INDEX_T i, cmdObj *cmd);	// set status report specification
static uint8_t _get_si(const INDEX_T i, cmdObj *cmd);	// get status report interval
static uint8_t _set_si(const INDEX_T i, cmdObj *cmd);	// set status report interval

static uint8_t _run_gc(const INDEX_T i, cmdObj *cmd);	// run a line of gcode
static uint8_t _set_gd(const INDEX_T i, cmdObj *cmd);	// set gcode defaults

static uint8_t _get_ms(const INDEX_T i, cmdObj *cmd);	// get machine state as value and string
static uint8_t _get_vl(const INDEX_T i, cmdObj *cmd);	// get current velocity
static uint8_t _get_mp(const INDEX_T i, cmdObj *cmd);	// get current machine position
static uint8_t _get_wp(const INDEX_T i, cmdObj *cmd);	// get current work position
static uint8_t _get_am(const INDEX_T i, cmdObj *cmd);

static uint8_t _set_ic(const INDEX_T i, cmdObj *cmd);	// ignore CR on input
static uint8_t _set_il(const INDEX_T i, cmdObj *cmd);	// ignore LF on input
static uint8_t _set_ec(const INDEX_T i, cmdObj *cmd);	// expand CRLF on outout
static uint8_t _set_ee(const INDEX_T i, cmdObj *cmd);	// enable character echo
static uint8_t _set_ex(const INDEX_T i, cmdObj *cmd);	// enable XON/XOFF

static uint8_t _set_sa(const INDEX_T i, cmdObj *cmd);	// set motor step angle
static uint8_t _set_mi(const INDEX_T i, cmdObj *cmd);	// set microsteps
static uint8_t _set_po(const INDEX_T i, cmdObj *cmd);	// set motor polarity
static uint8_t _set_motor_steps_per_unit(const INDEX_T i, cmdObj *cmd);

static void _print_am(const INDEX_T i);					// print axis mode

//--- PROGMEM Strings -------------------------------------------------------
/* PROGMEM strings for token, friendly name match string, and print format
 *	Use accessors to get at elements in the combined strings.
 *
 *	NOTE: DO NOT USE TABS IN FORMAT STRINGS
 *	NOTE: LEAVE NO SPACE BEFORE OR AFTER FIRST COMMA (TOKEN,NAME)
 *		  LEAVE NO SPACE BEFORE SECOND COMMA (SPACE AFTER IS OK)
 */
char str_fc[] PROGMEM = "fc,config_v,[fc]  config_version   %14.2f\n";
char str_fv[] PROGMEM = "fv,firmware_v,[fv]  firmware_version %14.2f\n";
char str_fb[] PROGMEM = "fb,firmware_b,[fb]  firmware_build   %14.2f\n";

char str_ln[] PROGMEM = "ln,line_number,[ln]  line_number%8.0f\n";
char str_ms[] PROGMEM = "ms,machine_state,[ms]  machine_state %1.0d\n";
//char str_fs[] PROGMEM = "fs,feedhold_state,[fs]  feedhold_state %1.0d\n";
char str_vl[] PROGMEM = "vl,velocity,[vl]  velocity %8.3f %S/min\n";
//char str_un[] PROGMEM = "un,un,[un]  units %S\n";						// units in ASCII
char str_sr[] PROGMEM = "sr,status_r,[sr]";								// status_report
char str_si[] PROGMEM = "si,status_i,[si]  status_interval    %10.0f ms [0=off]\n";

char str_xmp[] PROGMEM = "xmp,x_m,[xmp] x_machine_position %14.3f%S\n";
char str_ymp[] PROGMEM = "ymp,y_m,[ymp] y_machine_position %14.3f%S\n";
char str_zmp[] PROGMEM = "zmp,z_m,[zmp] z_machine_position %14.3f%S\n";
char str_amp[] PROGMEM = "amp,a_m,[amp] a_machine_position %14.3f%S\n";
char str_bmp[] PROGMEM = "bmp,b_m,[bmp] b_machine_position %14.3f%S\n";
char str_cmp[] PROGMEM = "cmp,c_m,[cmp] c_machine_position %14.3f%S\n";

char str_xwp[] PROGMEM = "xwp,x_w,[xwp] x_work_position %17.3f%S\n";
char str_ywp[] PROGMEM = "ywp,y_w,[ywp] y_work_position %17.3f%S\n";
char str_zwp[] PROGMEM = "zwp,z_w,[zwp] z_work_position %17.3f%S\n";
char str_awp[] PROGMEM = "awp,a_w,[awp] a_work_position %17.3f%S\n";
char str_bwp[] PROGMEM = "bwp,b_w,[bwp] b_work_position %17.3f%S\n";
char str_cwp[] PROGMEM = "cwp,c_w,[cwp] c_work_position %17.3f%S\n";

char str_gc[] PROGMEM = "gc,gcod,[gc]";	// SPECIAL NOTE: This record must precede the defaults below
//char str_gd[] PROGMEM = "gd,gcode_d,[gd]  gcode_defaults            G%2.1f\n";

char str_gi[] PROGMEM = "gi,gcode_i,[gi]  gcode_inches_mode         G%2.0f [20,21]\n";
char str_gs[] PROGMEM = "gs,gcode_s,[gs]  gcode_select_plane        G%2.0f [17,18,19]\n";
char str_gp[] PROGMEM = "gp,gcode_p,[gp]  gcode_path_control        G%3.1f [61,61.1,64]\n";
char str_ga[] PROGMEM = "ga,gcode_a,[ga]  gcode_absolute_mode       G%2.0f [90,91]\n";

char str_ea[] PROGMEM = "ea,enable_a,[ea]  enable_acceleration%10.0d [0,1]\n";
char str_ja[] PROGMEM = "ja,corner_a,[ja]  corner_acceleration%10.0f%S\n";
char str_ml[] PROGMEM = "ml,min_l,   [ml]  min_line_segment   %14.3f%S\n";
char str_ma[] PROGMEM = "ma,min_a,   [ma]  min_arc_segment    %14.3f%S\n";
char str_mt[] PROGMEM = "mt,min_s,   [mt]  min_segment_time   %10.0f uSec\n";

char str_ic[] PROGMEM = "ic,ignore_c,[ic]  ignore_CR (on RX)  %10.0d [0,1]\n";
char str_il[] PROGMEM = "il,ignore_l,[il]  ignore_LF (on RX)  %10.0d [0,1]\n";
char str_ec[] PROGMEM = "ec,enable_c,[ec]  enable_CR (on TX)  %10.0d [0,1]\n";
char str_ee[] PROGMEM = "ee,enable_e,[ee]  enable_echo        %10.0d [0,1]\n";
char str_ex[] PROGMEM = "ex,enable_x,[ex]  enable_xon_xoff    %10.0d [0,1]\n";

// Motor strings in program memory 
char str_1ma[] PROGMEM = "1ma,m1_ma, [1ma] m1_map_to_axis          %5.0d [0=X, 1=Y...]\n";
char str_1sa[] PROGMEM = "1sa,m1_s,  [1sa] m1_step_angle           %9.3f%S\n";
char str_1tr[] PROGMEM = "1tr,m1_tr, [1tr] m1_travel_per_revolution%9.3f%S\n";
char str_1mi[] PROGMEM = "1mi,m1_mi, [1mi] m1_microsteps           %5.0d [1,2,4,8]\n";
char str_1po[] PROGMEM = "1po,m1_pol,[1po] m1_polarity             %5.0d [0,1]\n";
char str_1pm[] PROGMEM = "1pm,m1_pow,[1pm] m1_power_management     %5.0d [0,1]\n";

char str_2ma[] PROGMEM = "2ma,m2_ma, [2ma] m2_map_to_axis          %5.0d [0=X, 1=Y...]\n";
char str_2sa[] PROGMEM = "2sa,m2_s,  [2sa] m2_step_angle           %9.3f%S\n";
char str_2tr[] PROGMEM = "2tr,m2_tr, [2tr] m2_travel_per_revolution%9.3f%S\n";
char str_2mi[] PROGMEM = "2mi,m2_mi, [2mi] m2_microsteps           %5.0d [1,2,4,8]\n";
char str_2po[] PROGMEM = "2po,m2_pol,[2po] m2_polarity             %5.0d [0,1]\n";
char str_2pm[] PROGMEM = "2pm,m2_pow,[2pm] m2_power_management     %5.0d [0,1]\n";

char str_3ma[] PROGMEM = "3ma,m3_ma, [3ma] m3_map_to_axis          %5.0d [0=X, 1=Y...]\n";
char str_3sa[] PROGMEM = "3sa,m3_s,  [3sa] m3_step_angle           %9.3f%S\n";
char str_3tr[] PROGMEM = "3tr,m3_tr, [3tr] m3_travel_per_revolution%9.3f%S\n";
char str_3mi[] PROGMEM = "3mi,m3_mi, [3mi] m3_microsteps           %5.0d [1,2,4,8]\n";
char str_3po[] PROGMEM = "3po,m3_pol,[3po] m3_polarity             %5.0d [0,1]\n";
char str_3pm[] PROGMEM = "3pm,m3_pow,[3pm] m3_power_management     %5.0d [0,1]\n";

char str_4ma[] PROGMEM = "4ma,m4_ma, [4ma] m4_map_to_axis          %5.0d [0=X, 1=Y...]\n";
char str_4sa[] PROGMEM = "4sa,m4_s,  [4sa] m4_step_angle           %9.3f%S\n";
char str_4tr[] PROGMEM = "4tr,m4_tr, [4tr] m4_travel_per_revolution%9.3f%S\n";
char str_4mi[] PROGMEM = "4mi,m4_mi, [4mi] m4_microsteps           %5.0d [1,2,4,8]\n";
char str_4po[] PROGMEM = "4po,m4_pol,[4po] m4_polarity             %5.0d [0,1]\n";
char str_4pm[] PROGMEM = "4pm,m4_pow,[4pm] m4_power_management     %5.0d [0,1]\n";

// Axis strings in program memory
char str_xam[] PROGMEM = "xam,x_a,[xam] x_axis_mode       %11.0d %S\n";
char str_xfr[] PROGMEM = "xfr,x_f,[xfr] x_feedrate_maximum%15.3f%S/min\n";
char str_xvm[] PROGMEM = "xvm,x_v,[xvm] x_velocity_maximum%15.3f%S/min\n";
char str_xtm[] PROGMEM = "xtm,x_t,[xtm] x_travel_maximum  %15.3f%S\n";
char str_xjm[] PROGMEM = "xjm,x_je,[xjm] x_jerk_maximum    %11.0f%S/min^3\n";
char str_xjd[] PROGMEM = "xjd,x_ju,[xjd] x_junction_deviation%14.4f%S\n";
char str_xsm[] PROGMEM = "xsm,x_s,[xsm] x_switch_mode     %11.0d [0,1]\n";
char str_xht[] PROGMEM = "xth,x_homing_t,[xht] x_homing_travel         %9.3f%S\n";
char str_xhs[] PROGMEM = "xhs,x_homing_s,[xhs] x_homing_search_velocity%9.3f%S/min\n";
char str_xhl[] PROGMEM = "xhl,x_homing_l,[xhl] x_homing_latch_velocity %9.3f%S/min\n";
char str_xhz[] PROGMEM = "xhz,x_homing_z,[xhz] x_homing_zero_offset    %9.3f%S\n";
char str_xhw[] PROGMEM = "xhw,x_homing_w,[xhw] x_homing_work_offset    %9.3f%S\n";

char str_yam[] PROGMEM = "yam,y_a,[yam] y_axis_mode       %11.0d %S\n";
char str_yfr[] PROGMEM = "yfr,y_f,[yfr] y_feedrate_maximum%15.3f%S/min\n";
char str_yvm[] PROGMEM = "yvm,y_v,[yvm] y_velocity_maximum%15.3f%S/min\n";
char str_ytm[] PROGMEM = "ytm,y_t,[ytm] y_travel_maximum  %15.3f%S\n";
char str_yjm[] PROGMEM = "yjm,y_je,[yjm] y_jerk_maximum    %11.0f%S/min^3\n";
char str_yjd[] PROGMEM = "yjd,y_ju,[yjd] y_junction_deviation%14.4f%S\n";
char str_ysm[] PROGMEM = "ysm,y_s,[ysm] y_switch_mode     %11.0d [0,1]\n";
char str_yht[] PROGMEM = "yth,y_homing_t,[yht] y_homing_travel         %9.3f%S\n";
char str_yhs[] PROGMEM = "yhs,y_homing_s,[yhs] y_homing_search_velocity%9.3f%S/min\n";
char str_yhl[] PROGMEM = "yhl,y_homing_l,[yhl] y_homing_latch_velocity %9.3f%S/min\n";
char str_yhz[] PROGMEM = "yhz,y_homing_z,[yhz] y_homing_zero_offset    %9.3f%S\n";
char str_yhw[] PROGMEM = "yhw,y_homing_w,[yhw] y_homing_work_offset    %9.3f%S\n";

char str_zam[] PROGMEM = "zam,z_a,[zam] z_axis_mode       %11.0d %S\n";
char str_zfr[] PROGMEM = "zfr,z_f,[zfr] z_feedrate_maximum%15.3f%S/min\n";
char str_zvm[] PROGMEM = "zvm,z_v,[zvm] z_velocity_maximum%15.3f%S/min\n";
char str_ztm[] PROGMEM = "ztm,z_t,[ztm] z_travel_maximum  %15.3f%S\n";
char str_zjm[] PROGMEM = "zjm,z_je,[zjm] z_jerk_maximum    %11.0f%S/min^3\n";
char str_zjd[] PROGMEM = "zjd,z_ju,[zjd] z_junction_deviation%14.4f%S\n";
char str_zsm[] PROGMEM = "zsm,z_s,[zsm] z_switch_mode     %11.0d [0,1]\n";
char str_zht[] PROGMEM = "zth,z_homing_t,[zht] z_homing_travel         %9.3f%S\n";
char str_zhs[] PROGMEM = "zhs,z_homing_s,[zhs] z_homing_search_velocity%9.3f%S/min\n";
char str_zhl[] PROGMEM = "zhl,z_homing_l,[zhl] z_homing_latch_velocity %9.3f%S/min\n";
char str_zhz[] PROGMEM = "zhz,z_homing_z,[zhz] z_homing_zero_offset    %9.3f%S\n";
char str_zhw[] PROGMEM = "zhw,z_homing_w,[zhw] z_homing_work_offset    %9.3f%S\n";

char str_aam[] PROGMEM = "aam,a_a,[aam] a_axis_mode       %11.0d %S\n";
char str_afr[] PROGMEM = "afr,a_f,[afr] a_feedrate_maximum%15.3f%S/min\n";
char str_avm[] PROGMEM = "avm,a_v,[avm] a_velocity_maximum%15.3f%S/min\n";
char str_atm[] PROGMEM = "atm,a_t,[atm] a_travel_maximum  %15.3f%S\n";
char str_ajm[] PROGMEM = "ajm,a_je,[ajm] a_jerk_maximum    %11.0f%S/min^3\n";
char str_ajd[] PROGMEM = "ajd,a_ju,[ajc] a_junction_deviation%14.4f%S\n";
char str_ara[] PROGMEM = "ara,a_r,[ara] a_radius_value    %16.4f%S\n";
char str_asm[] PROGMEM = "asm,a_s,[asm] a_switch_mode     %11.0d [0,1]\n";
char str_aht[] PROGMEM = "ath,a_homing_t,[aht] a_homing_travel         %9.3f%S\n";
char str_ahs[] PROGMEM = "ahs,a_homing_s,[ahs] a_homing_search_velocity%9.3f%S/min\n";
char str_ahl[] PROGMEM = "ahl,a_homing_l,[ahl] a_homing_latch_vel      %9.3f%S/min\n";
char str_ahz[] PROGMEM = "ahz,a_homing_z,[ahz] a_homing_zero_offset    %9.3f%S\n";
char str_ahw[] PROGMEM = "ahw,a_homing_w,[ahw] a_homing_work_offset    %9.3f%S\n";

char str_bam[] PROGMEM = "bam,b_a,[bam] b_axis_mode       %11.0d %S\n";
char str_bfr[] PROGMEM = "bfr,b_f,[bfr] b_feedrate_maximum%15.3f%S/min\n";
char str_bvm[] PROGMEM = "bvm,b_v,[bvm] b_velocity_maximum%15.3f%S/min\n";
char str_btm[] PROGMEM = "btm,b_t,[btm] b_travel_maximum  %15.3f%S\n";
char str_bjm[] PROGMEM = "bjm,b_je,[bjm] b_jerk_maximum    %11.0f%S/min^3\n";
char str_bjd[] PROGMEM = "bcd,b_ju,[bjd] b_junction_deviation%14.4f%S\n";
char str_bra[] PROGMEM = "bra,b_r,[bra] b_radius_value    %16.4f%S\n";
char str_bsm[] PROGMEM = "bsm,b_s,[bsm] b_switch_mode     %11.0d [0,1]\n";
char str_bht[] PROGMEM = "bth,b_homing_t,[bht] b_homing_travel         %9.3f%S\n";
char str_bhs[] PROGMEM = "bhs,b_homing_s,[bhs] b_homing_search_velocity%9.3f%S/min\n";
char str_bhl[] PROGMEM = "bhl,b_homing_l,[bhl] b_homing_latch_velocity %9.3f%S/min\n";
char str_bhz[] PROGMEM = "bhz,b_homing_z,[bhz] b_homing_zero_offset    %9.3f%S\n";
char str_bhw[] PROGMEM = "bhw,b_homing_w,[bhw] b_homing_work_offset    %9.3f%S\n";

char str_cam[] PROGMEM = "cam,c_a,[cam] c_axis_mode       %11.0d %S\n";
char str_cfr[] PROGMEM = "cfr,c_f,[cfr] c_feedrate_maximum%15.3f%S/min\n";
char str_cvm[] PROGMEM = "cvm,c_v,[cvm] c_velocity_maximum%15.3f%S/min\n";
char str_ctm[] PROGMEM = "ctm,c_t,[ctm] c_travel_maximum  %15.3f%S\n";
char str_cjm[] PROGMEM = "cjm,c_je,[cjm] c_jerk_maximum    %11.0f%S/min^3\n";
char str_cjd[] PROGMEM = "cjd,c_ju,[cjd] c_junction_deviation%14.4f%S\n";
char str_cra[] PROGMEM = "cra,c_r,[cra] c_radius_value    %16.4f%S\n";
char str_csm[] PROGMEM = "csm,c_s,[csm] c_switch_mode     %11.0d [0,1]\n";
char str_cht[] PROGMEM = "cth,c_homing_t,[cht] c_homing_travel         %9.3f%S\n";
char str_chs[] PROGMEM = "chs,c_homing_s,[chs] c_homing_search_velocity%9.3f%S/min\n";
char str_chl[] PROGMEM = "chl,c_homing_l,[chl] c_homing_latch_velocity %9.3f%S/min\n";
char str_chz[] PROGMEM = "chz,c_homing_z,[chz] c_homing_zero_offset    %9.3f%S\n";
char str_chw[] PROGMEM = "chw,c_homing_w,[chw] c_homing_work_offset    %9.3f%S\n";

/***** PROGMEM config array **************************************************
 */
struct cfgItem cfgArray[] PROGMEM = {

//	 string *, print func, get func, set func  target for get/set,    default value
	{ str_fc, _print_dbl, _get_dbl, _set_nul, (double *)&cfg.version, TINYG_BUILD_NUMBER },	// must be first, but not mandatory
	{ str_fv, _print_dbl, _get_dbl, _set_nul, (double *)&tg.version,  TINYG_VERSION_NUMBER },
	{ str_fb, _print_dbl, _get_dbl, _set_nul, (double *)&tg.build,    TINYG_BUILD_NUMBER },

	{ str_ln, _print_int, _get_int, _set_int, (double *)&cm.linenum, 0 },// line number
	{ str_ms, _print_ui8, _get_ms,  _set_nul, (double *)&cm.machine_state,0 },// machine state
//	{ str_fs, _print_ui8, _get_ui8, _set_nul, (double *)&cm.hold_state,0 },// feedhold state
	{ str_vl, _print_lin, _get_vl,  _set_nul, (double *)&tg.null, 0 },	// current velocity
//	{ str_un, _print_uni, _get_nul, _set_nul, (double *)&tg.null, 0 },	// units mode in ascii
	{ str_sr, _print_nul, _get_sr,  _set_sr,  (double *)&tg.null, 0 },	// status report object
	{ str_si, _print_nul, _get_si,  _set_si,  (double *)&cfg.status_report_interval, STATUS_REPORT_INTERVAL },

	{ str_xmp, _print_lin, _get_mp, _set_nul, (double *)&tg.null, 0 },	// x machine position
	{ str_ymp, _print_lin, _get_mp, _set_nul, (double *)&tg.null, 0 },	// y machine position
	{ str_zmp, _print_lin, _get_mp, _set_nul, (double *)&tg.null, 0 },	// z machine position
	{ str_amp, _print_rot, _get_mp, _set_nul, (double *)&tg.null, 0 },	// a machine position
	{ str_bmp, _print_rot, _get_mp, _set_nul, (double *)&tg.null, 0 },	// b machine position
	{ str_cmp, _print_rot, _get_mp, _set_nul, (double *)&tg.null, 0 },	// c machine position

	{ str_xwp, _print_lin, _get_wp, _set_nul, (double *)&tg.null, 0 },	// x work position
	{ str_ywp, _print_lin, _get_wp, _set_nul, (double *)&tg.null, 0 },	// y work position
	{ str_zwp, _print_lin, _get_wp, _set_nul, (double *)&tg.null, 0 },	// z work position
	{ str_awp, _print_rot, _get_wp, _set_nul, (double *)&tg.null, 0 },	// a work position
	{ str_bwp, _print_rot, _get_wp, _set_nul, (double *)&tg.null, 0 },	// b work position
	{ str_cwp, _print_rot, _get_wp, _set_nul, (double *)&tg.null, 0 },	// c work position

	{ str_gc, _print_nul, _get_nul,  _run_gc, (double *)&tg.null, 0 },	 // gcode block
//	{ str_gd, _print_gd,  _get_gd,   _set_gd, (double *)&tg.null, 0 },	 // gcode defaault values
	{ str_gi, _print_dbl, _get_dbls, _set_gd, (double *)&cfg.inches_mode,  GCODE_INCH_MODE },
	{ str_gs, _print_dbl, _get_dbls, _set_gd, (double *)&cfg.select_plane, GCODE_SELECT_PLANE },
	{ str_gp, _print_dbl, _get_dbls, _set_gd, (double *)&cfg.path_control, GCODE_PATH_CONTROL },
	{ str_ga, _print_dbl, _get_dbls, _set_gd, (double *)&cfg.absolute_mode,GCODE_ABSOLUTE_MODE },

	{ str_ea, _print_ui8, _get_ui8, _set_ui8, (double *)&cfg.enable_acceleration, ENABLE_ACCELERATION },
	{ str_ja, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.corner_acceleration, CORNER_ACCELERATION },
	{ str_ml, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.min_segment_len,   MIN_LINE_LENGTH },
	{ str_ma, _print_lin, _get_dbu, _set_dbu, (double *)&cfg.arc_segment_len,   MM_PER_ARC_SEGMENT },
	{ str_mt, _print_lin, _get_dbl, _set_dbl, (double *)&cfg.estd_segment_usec, ESTD_SEGMENT_USEC },

	{ str_ic, _print_ui8, _get_ui8, _set_ic,  (double *)&cfg.ignore_cr,   COM_IGNORE_RX_CR },
	{ str_il, _print_ui8, _get_ui8, _set_il,  (double *)&cfg.ignore_lf,   COM_IGNORE_RX_LF },
	{ str_ec, _print_ui8, _get_ui8, _set_ec,  (double *)&cfg.enable_cr,   COM_APPEND_TX_CR },
	{ str_ee, _print_ui8, _get_ui8, _set_ee,  (double *)&cfg.enable_echo, COM_ENABLE_ECHO },
	{ str_ex, _print_ui8, _get_ui8, _set_ex,  (double *)&cfg.enable_xon,  COM_ENABLE_XON },

	{ str_1ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].motor_map,  M1_MOTOR_MAP },
	{ str_1sa, _print_rot, _get_dbl ,_set_sa, (double *)&cfg.m[MOTOR_1].step_angle, M1_STEP_ANGLE },
	{ str_1tr, _print_lin, _get_dbl ,_set_sa, (double *)&cfg.m[MOTOR_1].travel_rev, M1_TRAVEL_PER_REV },
	{ str_1mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_1].microsteps, M1_MICROSTEPS },
	{ str_1po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_1].polarity,   M1_POLARITY },
	{ str_1pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_1].power_mode, M1_POWER_MODE },

	{ str_2ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].motor_map,  M2_MOTOR_MAP },
	{ str_2sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_2].step_angle, M2_STEP_ANGLE },
	{ str_2tr, _print_lin, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_2].travel_rev, M2_TRAVEL_PER_REV },
	{ str_2mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_2].microsteps, M2_MICROSTEPS },
	{ str_2po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_2].polarity,   M2_POLARITY },
	{ str_2pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_2].power_mode, M2_POWER_MODE },

	{ str_3ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].motor_map,  M3_MOTOR_MAP },
	{ str_3sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_3].step_angle, M3_STEP_ANGLE },
	{ str_3tr, _print_lin, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_3].travel_rev, M3_TRAVEL_PER_REV },
	{ str_3mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_3].microsteps, M3_MICROSTEPS },
	{ str_3po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_3].polarity,   M3_POLARITY },
	{ str_3pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_3].power_mode, M3_POWER_MODE },

	{ str_4ma, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].motor_map,  M4_MOTOR_MAP },
	{ str_4sa, _print_rot, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_4].step_angle, M4_STEP_ANGLE },
	{ str_4tr, _print_lin, _get_dbl, _set_sa, (double *)&cfg.m[MOTOR_4].travel_rev, M4_TRAVEL_PER_REV },
	{ str_4mi, _print_ui8, _get_ui8, _set_mi, (double *)&cfg.m[MOTOR_4].microsteps, M4_MICROSTEPS },
	{ str_4po, _print_ui8, _get_ui8, _set_po, (double *)&cfg.m[MOTOR_4].polarity,   M4_POLARITY },
	{ str_4pm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.m[MOTOR_4].power_mode, M4_POWER_MODE },

	{ str_xam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[X].axis_mode,	X_AXIS_MODE },
	{ str_xfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].feedrate_max, X_FEEDRATE_MAX },
	{ str_xvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].velocity_max, X_VELOCITY_MAX },
	{ str_xtm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].travel_max,	X_TRAVEL_MAX },
	{ str_xjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].jerk_max,	    X_JERK_MAX },
	{ str_xjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].junction_dev, X_JUNCTION_DEVIATION },
//	{ str_xra, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].radius,		0 },
	{ str_xsm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[X].switch_mode,  X_SWITCH_MODE },
	{ str_xht, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].homing_travel,X_HOMING_SEARCH_TRAVEL },
	{ str_xhs, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].homing_search_velocity,X_HOMING_SEARCH_VELOCITY },
	{ str_xhl, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].homing_latch_velocity,X_HOMING_LATCH_VELOCITY },
	{ str_xhz, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].homing_zero_offset,X_HOMING_ZERO_OFFSET },
	{ str_xhw, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[X].homing_work_offset,X_HOMING_WORK_OFFSET },

	{ str_yam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[Y].axis_mode,	Y_AXIS_MODE },
	{ str_yfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].feedrate_max, Y_FEEDRATE_MAX },
	{ str_yvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].velocity_max, Y_VELOCITY_MAX },
	{ str_ytm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].travel_max,	Y_TRAVEL_MAX },
	{ str_yjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].jerk_max,	    Y_JERK_MAX },
	{ str_yjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].junction_dev, Y_JUNCTION_DEVIATION },
//	{ str_yra, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].radius,		0 },
	{ str_ysm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[Y].switch_mode,	Y_SWITCH_MODE },
	{ str_yht, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].homing_travel,Y_HOMING_SEARCH_TRAVEL },
	{ str_yhs, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].homing_search_velocity,Y_HOMING_SEARCH_VELOCITY },
	{ str_yhl, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].homing_latch_velocity,Y_HOMING_LATCH_VELOCITY },
	{ str_yhz, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].homing_zero_offset,Y_HOMING_ZERO_OFFSET },
	{ str_yhw, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Y].homing_work_offset,Y_HOMING_WORK_OFFSET },

	{ str_zam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[Z].axis_mode,	Z_AXIS_MODE },
	{ str_zfr, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].feedrate_max, Z_FEEDRATE_MAX },
	{ str_zvm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].velocity_max, Z_VELOCITY_MAX },
	{ str_ztm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].travel_max,	Z_TRAVEL_MAX },
	{ str_zjm, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].jerk_max,	    Z_JERK_MAX },
	{ str_zjd, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].junction_dev, Z_JUNCTION_DEVIATION },
//	{ str_zra, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].radius,		0 },
	{ str_zsm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[Z].switch_mode,	Z_SWITCH_MODE },
	{ str_zht, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].homing_travel,Z_HOMING_SEARCH_TRAVEL },
	{ str_zhs, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].homing_search_velocity,Z_HOMING_SEARCH_VELOCITY },
	{ str_zhl, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].homing_latch_velocity,Z_HOMING_LATCH_VELOCITY },
	{ str_zhz, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].homing_zero_offset,Z_HOMING_ZERO_OFFSET },
	{ str_zhw, _print_lin, _get_dbu, _set_dbu,(double *)&cfg.a[Z].homing_work_offset,Z_HOMING_WORK_OFFSET },

	{ str_aam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[A].axis_mode,	A_AXIS_MODE },
	{ str_afr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].feedrate_max, A_FEEDRATE_MAX },
	{ str_avm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].velocity_max, A_VELOCITY_MAX },
	{ str_atm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].travel_max,	A_TRAVEL_MAX },
	{ str_ajm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].jerk_max,	    A_JERK_MAX },
	{ str_ajd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].junction_dev, A_JUNCTION_DEVIATION },
	{ str_ara, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].radius,		A_RADIUS},
	{ str_asm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[A].switch_mode,	A_SWITCH_MODE },
	{ str_aht, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].homing_travel,A_HOMING_SEARCH_TRAVEL },
	{ str_ahs, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].homing_search_velocity,A_HOMING_SEARCH_VELOCITY },
	{ str_ahl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].homing_latch_velocity,A_HOMING_LATCH_VELOCITY },
	{ str_ahz, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].homing_zero_offset,A_HOMING_ZERO_OFFSET },
	{ str_ahw, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[A].homing_work_offset,A_HOMING_WORK_OFFSET },

	{ str_bam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[B].axis_mode,	B_AXIS_MODE },
	{ str_bfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].feedrate_max, B_FEEDRATE_MAX },
	{ str_bvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].velocity_max, B_VELOCITY_MAX },
	{ str_btm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].travel_max,	B_TRAVEL_MAX },
	{ str_bjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].jerk_max,	    B_JERK_MAX },
	{ str_bjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].junction_dev, B_JUNCTION_DEVIATION },
	{ str_bra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].radius,		B_RADIUS },
	{ str_bsm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[B].switch_mode,	B_SWITCH_MODE },
	{ str_bht, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].homing_travel,B_HOMING_SEARCH_TRAVEL },
	{ str_bhs, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].homing_search_velocity,B_HOMING_SEARCH_VELOCITY },
	{ str_bhl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].homing_latch_velocity,B_HOMING_LATCH_VELOCITY },
	{ str_bhz, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].homing_zero_offset,B_HOMING_ZERO_OFFSET },
	{ str_bhw, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[B].homing_work_offset,B_HOMING_WORK_OFFSET },

	{ str_cam, _print_am,  _get_am,  _set_ui8,(double *)&cfg.a[C].axis_mode,	C_AXIS_MODE },
	{ str_cfr, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].feedrate_max, C_FEEDRATE_MAX },
	{ str_cvm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].velocity_max, C_VELOCITY_MAX },
	{ str_ctm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].travel_max,	C_TRAVEL_MAX },
	{ str_cjm, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].jerk_max,	    C_JERK_MAX },
	{ str_cjd, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].junction_dev, C_JUNCTION_DEVIATION },
	{ str_cra, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].radius,		C_RADIUS },
	{ str_csm, _print_ui8, _get_ui8, _set_ui8,(double *)&cfg.a[C].switch_mode,	C_SWITCH_MODE },
	{ str_cht, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].homing_travel,C_HOMING_SEARCH_TRAVEL },
	{ str_chs, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].homing_search_velocity,C_HOMING_SEARCH_VELOCITY },
	{ str_chl, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].homing_latch_velocity,C_HOMING_LATCH_VELOCITY },
	{ str_chz, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].homing_zero_offset,C_HOMING_ZERO_OFFSET },
	{ str_chw, _print_rot, _get_dbl, _set_dbl,(double *)&cfg.a[C].homing_work_offset,C_HOMING_WORK_OFFSET }
};
#define CMD_MAX_INDEX (sizeof cfgArray / sizeof(struct cfgItem))
#define NVM_STATUS_REPORT (CMD_MAX_INDEX * NVM_RECORD_LEN)	// location of status report in NVM

/**** Specialized internal functions ****************************************/
/* _get_ms() - get machine state as value and string
 * _get_vl() - get current velocity
 * _get_mp() - get current machine position
 * _get_wp() - get current work position
 * _print_un() - print units for status report
 */
static char msg_ms0[] PROGMEM = "reset";		// used for status reports
static char msg_ms1[] PROGMEM = "run";
static char msg_ms2[] PROGMEM = "stop";
static char msg_ms3[] PROGMEM = "hold";
static char msg_ms4[] PROGMEM = "resume";
static char msg_ms5[] PROGMEM = "homing";
static PGM_P msg_ms[] PROGMEM = { msg_ms0, msg_ms1, msg_ms2, msg_ms3, msg_ms4, msg_ms5 };

static uint8_t _get_ms(const INDEX_T i, cmdObj *cmd)
{
	_get_ui8(i,cmd);
//	cmd->value = (double)*((uint8_t *)pgm_read_word(&cfgArray[i].target));
	strcpy_P(cmd->string,(PGM_P)pgm_read_word(&msg_ms[(uint8_t)cmd->value]));
	cmd->value_type = VALUE_TYPE_STRING;
	return (TG_OK);
}

static uint8_t _get_vl(const INDEX_T i, cmdObj *cmd) 
{
	cmd->value = mp_get_current_velocity();
	if (cm_get_inches_mode() == true) cmd->value *= INCH_PER_MM;
	cmd->value_type = VALUE_TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_mp(const INDEX_T i, cmdObj *cmd) 
{
	cmd->value = mp_get_machine_position(_get_axis(i));
	if (cm_get_inches_mode() == true) cmd->value *= INCH_PER_MM;
	cmd->value_type = VALUE_TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_wp(const INDEX_T i, cmdObj *cmd) 
{
	cmd->value = mp_get_work_position(_get_axis(i));
	if (cm_get_inches_mode() == true) cmd->value *= INCH_PER_MM;
	cmd->value_type = VALUE_TYPE_FLOAT;
	return (TG_OK);
}

/**** STATUS REPORT FUNCTIONS ****
 * _get_sr() - run status report
 * _set_sr() - set status report specification
 * _get_si() - get status report interval
 * _set_si() - set status report interval
 *
 *	Note: _get_sr() is called during initialization and reset when there's
 *	actually nothing to do. So it rejects all get requests except those where 
 *	cmd->value_type == true and cmd->value == true.
 */
static uint8_t _get_sr(const INDEX_T i, cmdObj *cmd) 
{
	rpt_json_status_report();
	return (TG_OK);
}

static uint8_t _set_sr(const INDEX_T i, cmdObj *cmd) 
{
	memset(cfg.status_report_spec, 0 , sizeof(cfg.status_report_spec));
	for (uint8_t j=0; j<CMD_MAX_OBJECTS; j++) {
		cmd++;
		if ((cmd->value_type == true) && (cmd->value == true)) { // see function header note
			cfg.status_report_spec[j] = cmd->index;
		}
		if (cmd->nx == NULL) break;
	}
	cmd_write_NVM_multi(NVM_STATUS_REPORT, "sr", (int8_t *)cfg.status_report_spec, sizeof(cfg.status_report_spec));
	return (TG_OK);
}

static uint8_t _set_si(const INDEX_T i, cmdObj *cmd) 
{
	if (cmd->value < STATUS_REPORT_MIN_MS) {
		cmd->value = STATUS_REPORT_MIN_MS;
	} else if (cmd->value > STATUS_REPORT_MAX_MS) {
		cmd->value = STATUS_REPORT_MAX_MS;
	}
	// convert value to segment timing
	cfg.status_report_interval = (uint8_t)ceil(cmd->value / (ESTD_SEGMENT_USEC / 1000));
	return (TG_OK);
}

static uint8_t _get_si(const INDEX_T i, cmdObj *cmd) 
{
	_get_ui8(i,cmd);
	cmd->value *= (ESTD_SEGMENT_USEC / 1000);
	return (TG_OK);
}

/**** GCODE FUNCTIONS ****
 * cfg_init_gcode_model() - initialize Gcode defaults
 * _set_gd() - set gcode defaults
 * _get_gd() - get gcode defaults
 * _run_gc() - launch the gcode parser on a block of gcode
 *
 *  Gcode requires special handling. The following model values can have defaults:
 *	  - G10 L2 Pn X... Y... Z... A... B... C...	coordinate system offsets 
 *	  -	G17/G18/G19		select plane
 *	  - G20/G21			units
 *	  - G61/G61.1/G64	path control mode
 *	  - G90/G91			absolute mode
 *
 *	$ mode behaviors:
 *
 *	The following are valid inputs to set values: (only one per line, however)
 *	  - $G17, $G18, $G19, $G20, $G21, $G61, $G61.1, $G64, $G90, $G91  
 *	  - G10 L2 Pn X... Y... Z... A... B... C...
 *

 *	JSON mode behaviors:
 *
 *	successful SET example
 *	  request:	{"g20":true} 
 *	  response:	{"g20":{"g20":"G20","st":0,"msg":"ok"}}
 *
 *	unsuccessful SET example
 *	  request:	{"g14":true} 
 *	  response:	{"g14":{"g14":"","st":25,"msg":"Gcode input error"}}
 *
 *	successful SET example
 *	  request:	{"gd":"g20"}
 *	  response:	{"gd":{"g20":"G20","st":0,"msg":"ok"}}


 

 *	Any Gcode inout is accepted as part of GD
 be defaultedis a little weird. The default settings are kept in the cfg struct 
 *	as doubles in 'G'word format, e.g. G21 for mm mode. The corresponding values
 *	in the gm struct are in a different form. The values are set in the cfg
 *	struct then are applied to the gm struct (by set). Get and Print both work
 *	with the cfg struct. This is also necessary as the gm struct gets wiped 
 *	perioidcally and needs some place to be restored from.
 */

void cfg_init_gcode_model()
{
	cmdObj cmd;
	INDEX_T i = cmd_get_index_by_token("gi");// inches mode
	_get_dbl(i,&cmd); 
	cmd_set(i,&cmd);
	i = cmd_get_index_by_token("gs"); 		// select plane
	_get_dbl(i,&cmd);
	cmd_set(i,&cmd);
	i = cmd_get_index_by_token("gp");		// path control mode
	_get_dbl(i,&cmd);
	cmd_set(i,&cmd);
	i = cmd_get_index_by_token("ga");		// distance mode
	_get_dbl(i,&cmd);
	cmd_set(i,&cmd);
}

static uint8_t _set_gd(const INDEX_T i, cmdObj *cmd)
{
	switch ((int) cmd->value * 10) {	// *10 to pick up 61.1 and any other NN.N gcode words
		case 170: {	cm_select_plane(CANON_PLANE_XY); cfg.select_plane = cmd->value; break;} // XY
		case 180: { cm_select_plane(CANON_PLANE_XZ); cfg.select_plane = cmd->value; break;}
		case 190: { cm_select_plane(CANON_PLANE_YZ); cfg.select_plane = cmd->value; break;}
		case 200: {	cm_set_inches_mode(true);  cfg.inches_mode = cmd->value; break;} // inches 
		case 210: { cm_set_inches_mode(false); cfg.inches_mode = cmd->value; break;} // mm
		case 610: { cm_set_path_control(PATH_EXACT_STOP); cfg.path_control = cmd->value; break;}
		case 611: { cm_set_path_control(PATH_EXACT_PATH); cfg.path_control = cmd->value; break;}
		case 640: { cm_set_path_control(PATH_CONTINUOUS); cfg.path_control = cmd->value; break;}
		case 900: { cm_set_absolute_mode(true);  cfg.absolute_mode = cmd->value; break;} // absolute mode
		case 910: { cm_set_absolute_mode(false); cfg.absolute_mode = cmd->value; break;} // incremental
	}
	return (TG_OK);
}
/*
static uint8_t _get_gd(const INDEX_T i, cmdObj *cmd)
{
	return (TG_OK);	
}
*/
static uint8_t _run_gc(const INDEX_T i, cmdObj *cmd)
{
	strncpy(tg.in_buf, cmd->string, INPUT_BUFFER_LEN);
	cmd->status = gc_gcode_parser(tg.in_buf);
	tg_make_json_gcode_response(cmd->status, tg.in_buf, tg.out_buf);
	return (TG_OK);
}

/**** MOTOR FUNCTIONS ****
 * _set_sa() - set step_angle or travel_per_rev & recompute steps_per_unit
 * _set_mi() - set microsteps & recompute steps_per_unit
 * _set_po() - set polarity and update stepper structs
 * _set_motor_steps_per_unit() - update this derived value
 *		This function will need to be rethought if microstep morphing is implemented, 
 */
static uint8_t _set_sa(const INDEX_T i, cmdObj *cmd)
{ 
	_set_dbl(i, cmd);
	_set_motor_steps_per_unit(i, cmd); 
	return (TG_OK);
}

static uint8_t _set_mi(const INDEX_T i, cmdObj *cmd)
{
	_set_ui8(i, cmd);
	_set_motor_steps_per_unit(i, cmd);
	st_set_microsteps(_get_motor(i), (uint8_t)cmd->value);
	return (TG_OK);
}

static uint8_t _set_po(const INDEX_T i, cmdObj *cmd)
{ 
	_set_ui8(i, cmd);
	st_set_polarity(_get_motor(i), (uint8_t)cmd->value);
	return (TG_OK);
}

static uint8_t _set_motor_steps_per_unit(const INDEX_T i, cmdObj *cmd)
{
	uint8_t m = _get_motor(i);
	cfg.m[m].steps_per_unit = (360 / (cfg.m[m].step_angle / cfg.m[m].microsteps) / cfg.m[m].travel_rev);
	return (TG_OK);
}

/**** AXIS FUNCTIONS ****
 * _get_am()	- get axis mode w/enumeration string
 * _print_am()	- print axis mode w/enumeration string
 */
static char msg_am00[] PROGMEM = "[disabled]";
static char msg_am01[] PROGMEM = "[standard]";
static char msg_am02[] PROGMEM = "[inhibited]";
static char msg_am03[] PROGMEM = "[radius]";
static char msg_am04[] PROGMEM = "[slave X]";
static char msg_am05[] PROGMEM = "[slave Y]";
static char msg_am06[] PROGMEM = "[slave Z]";
static char msg_am07[] PROGMEM = "[slave XY]";
static char msg_am08[] PROGMEM = "[slave XZ]";
static char msg_am09[] PROGMEM = "[slave YZ]";
static char msg_am10[] PROGMEM = "[slave XYZ]";
static PGM_P msg_am[] PROGMEM = {
	msg_am00, msg_am01, msg_am02, msg_am03, msg_am04, msg_am05, 
	msg_am06, msg_am07, msg_am08, msg_am09, msg_am10
};

static uint8_t _get_am(const INDEX_T i, cmdObj *cmd)
{
	_get_ui8(i,cmd);
//	cmd->value = (double)*((uint8_t *)pgm_read_word(&cfgArray[i].target));
	strcpy_P(cmd->string,(PGM_P)pgm_read_word(&msg_am[(uint8_t)cmd->value]));
	cmd->value_type = VALUE_TYPE_INT32;
	return (TG_OK);
}

static void _print_am(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	uint8_t m = _get_ui8_value(i);
	fprintf(stderr, _get_format(i,format), m, (PGM_P)pgm_read_word(&msg_am[m]));
}

/**** SERIAL IO FUNCTIONS ****
 * _set_ic() - ignore cr on RX
 * _set_il() - ignore lf on RX
 * _set_ec() - enable CRLF on TX
 * _set_ee() - enable character echo
 * _set_ex() - enable XON/XOFF
 *
 *	The above assume USB is the std device
 */
static uint8_t _set_ic(const INDEX_T i, cmdObj *cmd) 
{
	if (NE_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORECR);
	} else { 
		(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORECR);
	}
	cfg.ignore_cr = (uint8_t)cmd->value;
	return (TG_OK);
}

static uint8_t _set_il(const INDEX_T i, cmdObj *cmd) 
{
	if (NE_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORELF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORELF);
	}
	cfg.ignore_lf = (uint8_t)cmd->value;
	return (TG_OK);
}

static uint8_t _set_ec(const INDEX_T i, cmdObj *cmd) 
{
	if (NE_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, XIO_CRLF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOCRLF);
	}
	cfg.enable_cr = (uint8_t)cmd->value;
	return (TG_OK);
}

static uint8_t _set_ee(const INDEX_T i, cmdObj *cmd) 
{
	if (NE_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, XIO_ECHO);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOECHO);
	}
	cfg.enable_echo = (uint8_t)cmd->value;
	return (TG_OK);
}

static uint8_t _set_ex(const INDEX_T i, cmdObj *cmd)
{
	if (NE_ZERO(cmd->value)) { 
		(void)xio_cntl(XIO_DEV_USB, XIO_XOFF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOXOFF);
	}
	cfg.enable_xon = (uint8_t)cmd->value;
	return (TG_OK);
}

/*****************************************************************************
 *** END SETTING-SPECIFIC REGION *********************************************
 *** Code below should not require changes as parameters are added/updated ***
 *****************************************************************************/

/****************************************************************************
 * cfg_init() - called once on system init
 *
 *	Will perform one of 3 actions:
 *	(1) if NVM is set up and at current config version: use NVM data for config
 *	(2) if NVM is set up but out-of-rev: migrate and apply old settings that are still applicable, 
 *	(3) if NVM is not set up: load RAM and NVM with hardwired default settings
 */

void cfg_init() 
{
	INDEX_T i = 0;
	cmdObj cmd;
	char exclusions[] = {"gc,sr"};	// don't try to SET these tokens

#ifdef __DISABLE_EEPROM_INIT		// cutout for debug simulation
	// Apply the hard-coded default values from settings.h and exit
	for (i=0; i<CMD_MAX_INDEX; i++) {
		if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
		cmd.value = (double)pgm_read_float(&cfgArray[i].def_value);
		cmd_set(i, &cmd);
	}
#else
	// get config version from NVM
	char token[CMD_TOKEN_LEN+1];
	cfg.nvm_base_addr = NVM_BASE_ADDR;
	cfg.nvm_profile_base = cfg.nvm_base_addr;
	cfg.version = tg.build;				// use the build number as the config version
	cmd_get_token(0, token);			// get the token of the 0th record - supposed to be config version
	cmd_read_NVM_record(0, &cmd);		// read the first record in NVM

	if (strstr(token, cmd.token) == token) { 	// tokens match. EEPROM is set up

		// Case (1) NVM is set up and current revision. Load config from NVM
		if (fabs(cfg.version - cmd.value) < EPSILON) {
			fprintf_P(stderr,PSTR("Loading configs from EEPROM\n"));
			for (i=0; i<CMD_MAX_INDEX; i++) {
				cmd_read_NVM_record(i, &cmd);
				if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
				cmd_set(i, &cmd);
			}
		} else {  // Case (2) NVM is out-of-rev. Use what old settings you can and migrate
		// pre-load configs with the default settings
			fprintf_P(stderr,PSTR("Migrating configs from earlier version - check your settings before proceeding\n"));
			for (i=0; i<CMD_MAX_INDEX; i++) {
				cmd.value = (double)pgm_read_float(&cfgArray[i].def_value);
				if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
				cmd_set(i, &cmd);
			}
			// selectively update configs from NVM records that match current token set
			for (i=0; i<CMD_MAX_INDEX; i++) {
				cmd_read_NVM_record(i, &cmd);
				if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
				cmd_set(cmd_get_index_by_token(cmd.token), &cmd);	// -1 indexes are rejected
			}
			// write RAM settings back to NVM to update it - completing the migration
			for (i=0; i<CMD_MAX_INDEX; i++) {
				cmd_get_token(i, cmd.token);
				if (cmd_write_NVM_record(i, &cmd) != TG_OK) {
					INFO(PSTR("Failed to update NVM in cfg_init()"));
				}
				fprintf_P(stderr,PSTR("."));
			}
		}
	} else {	// Case (3) NVM is not set up. Use the defaults and set up NVM
		fprintf_P(stderr,PSTR("Initializing configs to default values\n"));
		for (i=0; i<CMD_MAX_INDEX; i++) {
			if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
			cmd.value = (double)pgm_read_float(&cfgArray[i].def_value);
			cmd_set(i, &cmd);
			cmd_get_token(i, cmd.token);
			if (cmd_write_NVM_record(i, &cmd) != TG_OK) {
				INFO(PSTR("Failed to update NVM in cfg_init()"));
			}
			fprintf_P(stderr,PSTR("."));
		}
	}
	fprintf_P(stderr,PSTR("\n"));
#endif
}

/****************************************************************************
 * cfg_config_parser() - update a config setting from a text block
 *			  		   - conditionally display the setting (if TRUE)
 *			 		   - conditionally persist setting to NVM (if TRUE)
 *
 * _parse_config_string() - parse a command line or NVM config
 *							Populates a cmdObject and returns a status 
 */

uint8_t cfg_config_parser(char *str)
{
	struct cmdObject cmd;

	/// $ display functions (truncted commands)
	if ((str[0] == '$') && ((str[1] == NUL) || (str[2] == NUL))) {
		rpt_print_configs(str); 			// print based on contents of the string
		return (TG_OK);
	}
	// parse the string; print cutout if NULL value found
	_parse_config_string(str, &cmd);
	if (cmd.index == -1) {
		return (TG_UNRECOGNIZED_COMMAND);
	}
	if (cmd.value_type == VALUE_TYPE_NULL) {
		cmd_print(cmd.index);				// if no value provided just print the value (GET)
		return (TG_OK);
	}
	cmd_set(cmd.index, &cmd);				// otherwise SET the value,
	cmd_print(cmd.index);					//...print it, and persist it.
	cmd_write_NVM_record(cmd.index, &cmd);
	return (TG_OK);
}

static uint8_t _parse_config_string(char *str, struct cmdObject *cmd)
{
	char *tmp;
	char separators[] = {" =:|\t"};			// anything someone might use

	// pre-processing
	cmd_new_object(cmd);					// initialize config object
	if (*str == '$') str++;					// ignore leading $
	tmp = str;
	for (; *tmp!=NUL; tmp++) {
		*tmp = tolower(*tmp);				// convert string to lower case
		// todo: put comma tolerance in here
		// todo: insert separator for xfr1000 case in here
	}

	// field processing
	cmd->value_type = VALUE_TYPE_NULL;
	if ((tmp = strpbrk(str, separators)) == NULL) {
		strcpy(cmd->name, str);				// no value part
	} else {
		*tmp = NUL;							// terminate at end of name
		strcpy(cmd->name, str);
		str = ++tmp;
		cmd->value = strtod(str, &tmp);		// tmp is the end pointer
		if (tmp != str) {
			cmd->value_type = VALUE_TYPE_FLOAT;
		}
	}
	if ((cmd->index = cmd_get_index(cmd->name)) == -1) { 
		return (TG_UNRECOGNIZED_COMMAND);
	}
	cmd_get_token(cmd->index, cmd->token);
	return (TG_OK);
}

/****************************************************************************/
/**** CMD FUNCTIONS *********************************************************/
/**** These are the primary external access points to cmd functions *********
 * cmd_get()   - get a value from the target - in external format
 * cmd_set()   - set a value or invoke a function
 * cmd_print() - invoke print function
 */
uint8_t cmd_get(const INDEX_T i, cmdObj *cmd)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) { 
		cmd->status = TG_UNRECOGNIZED_COMMAND;
		return (cmd->status);
	}
	return (((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i,cmd));
}

uint8_t cmd_set(const INDEX_T i, cmdObj *cmd)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) { 
		cmd->status = TG_UNRECOGNIZED_COMMAND;
		return (cmd->status);
	}
	return (((fptrCmd)(pgm_read_word(&cfgArray[i].set)))(i,cmd));
}

void cmd_print(INDEX_T i)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) return;
	((fptrConfig)(pgm_read_word(&cfgArray[i].print)))(i);
}

/****************************************************************************
 * Secondary cmd functions
 * cmd_new_object() 		- initialize a command object (that you actually passed in)
 * cmd_get_cmd()			- like cmd_get but populates the entire cmdObj struct
 * cmd_get_index_by_token() - get index from mnenonic token (most efficient scan)
 * cmd_get_index() 			- get index from mnenonic token or friendly name
 * cmd_get_token()			- returns token in arg string & returns pointer to string
 * cmd_get_group() 			- returns the axis prefix, motor prefix, or 'g' for general
 * cmd_get_max_index()		- utility function to return array size				
 *
 *	cmd_get_index() and cmd_get_index_by_token() are the most expensive routines 
 *	in the whole mess. They do a linear table scan of the PROGMEM strings, and
 *	could of course be further optimized. Use cmd_get_index_by_token() if you 
 *	know your input string is a token - ot's 10x faster than cmd_get_index()
 *
 *	The full string is not needed in the friendly name, just enough to match to
 *	uniqueness. This saves a fair amount of memory and time and is easier to use.
 */
struct cmdObject *cmd_new_object(struct cmdObject *cmd)
{
	memset(cmd, 0, sizeof(struct cmdObject));
	cmd->value_type = VALUE_TYPE_NULL;
	return (cmd);
}

uint8_t cmd_get_cmd(const INDEX_T i, cmdObj *cmd)
{
	cmd_new_object(cmd);
	if ((i < 0) || (i >= CMD_MAX_INDEX)) { 
		cmd->status = TG_UNRECOGNIZED_COMMAND;
		return (cmd->status);
	}
	cmd->index = i;
	cmd_get_token(i, cmd->token);
	((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i,cmd);
	return (cmd->value);
}

INDEX_T cmd_get_index_by_token(const char *str)
{
	char token[CMD_TOKEN_LEN];				// normally this would be +1, but we only need 3 chars.

	// Copy only as much as you need, which is 3 characters
	// The 1st comparison leverages execution order and early exit if [0] is not a match
	// The 2nd comparison matches or ignores the third character if it's a 2 char token
	for (INDEX_T i=0; i<CMD_MAX_INDEX; i++) {
		strncpy_P(token,(PGM_P)pgm_read_word(&cfgArray[i]), CMD_TOKEN_LEN);
		if ((token[0] != str[0]) || (token[1] != str[1])) continue;	
		if ((token[2] == str[2]) || (token[2] == ',')) return (i);
	}
	return (-1);							// no match
}

INDEX_T cmd_get_index(const char *str)
{
	char *name;								// pointer to friendly name
	char *end;
	char token[CMD_NAMES_FIELD_LEN];

	for (INDEX_T i=0; i<CMD_MAX_INDEX; i++) {
		strncpy_P(token,(PGM_P)pgm_read_word(&cfgArray[i]), CMD_NAMES_FIELD_LEN);
		name = strstr(token,",");			// find the separating comma
		*(name++) = NUL;					// split the token and name strings
		end = strstr(name,",");				// find the terminating comma
		*end = NUL;
	//	if (strstr(str, token) == str) return(i);
		if ((str[0] == token[0]) && (str[1] == token[1]) && (str[2] == token[2])) return (i); // slightly faster
		if (strstr(str, name) == str) return(i);
	}
	return (-1);							// no match
}

char *cmd_get_token(const INDEX_T i, char *token)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) { 
		*token = NUL;
		return (token);
	}
	strncpy_P(token,(PGM_P)pgm_read_word(&cfgArray[i].string), CMD_TOKEN_LEN+1);
	char *ptr = strstr(token,",");			// find the first separating comma
	*ptr = NUL;								// terminate the string after the token
	return (token);
}

char cmd_get_group(const INDEX_T i)
{
	char *ptr;
	char chr;
	char groups[] = {"xyzabc1234"};

	if ((i < 0) || (i >= CMD_MAX_INDEX)) return (NUL);
	strncpy_P(&chr,(PGM_P)pgm_read_word(&cfgArray[i].string), 1);
	if ((ptr = strchr(groups, chr)) == NULL) {
		return ('g');
	}
	return (*ptr);
}

inline INDEX_T cmd_get_max_index() { return (CMD_MAX_INDEX);}
inline INDEX_T cmd_get_sr_address() { return (NVM_STATUS_REPORT);}

/***** Generic internal functions *******************************************
 * _set_nul() - set nothing (noop)
 * _set_ui8() - set value as uint8_t w/o unit conversion
 * _set_int() - set value as integer w/o unit conversion
 * _set_dbl() - set value as double w/o unit conversion
 * _set_dbu() - set value as double w/unit conversion
 *
 * _get_nul() - returns -1
 * _get_ui8() - get value as uint8_t w/o unit conversion
 * _get_int() - get value as integer w/o unit conversion
 * _get_dbl() - get value as double w/o unit conversion
 * _get_dbu() - get value as double w/unit conversion
 * _get_dbls() - get value as double with conversion to string
 *
 * _get_ui8_value() - return uint8_t value (no cmd struct involved)
 * _get_int_value() - return integer value (no cmd struct involved)
 * _get_dbl_value() - return double value (no cmd struct involved)
 * _get_dbu_value() - return double value with unit conversion
 *
 * _print_nul() - print nothing
 * _print_ui8() - print uint8_t value w/no units or unit conversion
 * _print_int() - print integer value w/no units or unit conversion
 * _print_dbl() - print double value w/no units or unit conversion
 * _print_lin() - print linear value with units and in/mm unit conversion
 * _print_rot() - print rotary value with units
 */

static uint8_t _set_nul(const INDEX_T i, cmdObj *cmd) { return (TG_OK);}

static uint8_t _set_ui8(const INDEX_T i, cmdObj *cmd)
{
	*((uint8_t *)pgm_read_word(&cfgArray[i].target)) = cmd->value;
	return (TG_OK);
}

static uint8_t _set_int(const INDEX_T i, cmdObj *cmd)
{
	*((uint32_t *)pgm_read_word(&cfgArray[i].target)) = cmd->value;
	return (TG_OK);
}

static uint8_t _set_dbl(const INDEX_T i, cmdObj *cmd)
{
	*((double *)pgm_read_word(&cfgArray[i].target)) = cmd->value;
	return (TG_OK);
}

static uint8_t _set_dbu(const INDEX_T i, cmdObj *cmd)
{
	if (cm_get_inches_mode() == false) {
		*((double *)pgm_read_word(&cfgArray[i].target)) = cmd->value;
	} else {
		*((double *)pgm_read_word(&cfgArray[i].target)) = cmd->value * MM_PER_INCH;
	}
	return (TG_OK);
}

static uint8_t _get_nul(const INDEX_T i, cmdObj *cmd) 
{ 
	cmd->value_type = VALUE_TYPE_NULL;
	return (TG_OK);
}

static uint8_t _get_ui8(const INDEX_T i, cmdObj *cmd)
{
	cmd->value = (double)*((uint8_t *)pgm_read_word(&cfgArray[i].target));
	cmd->value_type = VALUE_TYPE_INT32;
	return (TG_OK);
}

static uint8_t _get_int(const INDEX_T i, cmdObj *cmd)
{
	cmd->value = (double)*((uint32_t *)pgm_read_word(&cfgArray[i].target));
	cmd->value_type = VALUE_TYPE_INT32;
	return (TG_OK);
}

static uint8_t _get_dbl(const INDEX_T i, cmdObj *cmd)
{
	cmd->value = *((double *)pgm_read_word(&cfgArray[i].target));
	cmd->value_type = VALUE_TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_dbu(const INDEX_T i, cmdObj *cmd)
{
	cmd->value = *((double *)pgm_read_word(&cfgArray[i].target));
	if (cm_get_inches_mode() == true) cmd->value *= INCH_PER_MM;
	cmd->value_type = VALUE_TYPE_FLOAT;
	return (TG_OK);
}

static uint8_t _get_dbls(const INDEX_T i, cmdObj *cmd)
{
	_get_dbl(i,cmd);
	sprintf(cmd->string, "%0.0f", cmd->value);
	cmd->value_type = VALUE_TYPE_STRING;
	return (TG_OK);
}

static uint8_t _get_ui8_value(const INDEX_T i) 
{ 
	cmdObj cmd;
	(((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i, &cmd));
	return ((uint8_t)cmd.value);
}

static uint8_t _get_int_value(const INDEX_T i) 
{ 
	cmdObj cmd;
	(((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i, &cmd));
	return ((uint32_t)cmd.value);
}

static double _get_dbl_value(const INDEX_T i) 
{
	cmdObj cmd;
	(((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i, &cmd));
	return (cmd.value);
}

static double _get_dbu_value(const INDEX_T i) 
{
	cmdObj cmd;
	(((fptrCmd)(pgm_read_word(&cfgArray[i].get)))(i, &cmd));
	if (cm_get_inches_mode() == true) cmd.value *= INCH_PER_MM;
	return (cmd.value);
}

static char msg_units0[] PROGMEM = " mm";
static char msg_units1[] PROGMEM = " in";
static char msg_units2[] PROGMEM = " deg";
static PGM_P msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };

static void _print_nul(const INDEX_T i) {}

static void _print_ui8(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(i,format), (uint8_t)_get_ui8_value(i));
}

static void _print_int(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(i,format), (uint32_t)_get_int_value(i));
}

static void _print_dbl(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(i,format), (double)_get_dbl_value(i));
}

static void _print_lin(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(i,format), _get_dbu_value(i), (PGM_P)pgm_read_word(&msg_units[cm_get_inches_mode()]));
}

static void _print_rot(const INDEX_T i)
{
	char format[CMD_FORMAT_LEN+1];
	fprintf(stderr, _get_format(i,format), _get_dbl_value(i), (PGM_P)pgm_read_word(&msg_units[2]));
}

/***************************************************************************** 
 * more accessors and other functions
 * _get_format() - returns format string as above
 * _get_axis() 	 - returns the axis an index applies to or -1 if na
 * _get_motor()  - returns the axis an index applies to or -1 if na
 *
 *  NOTE: Axis and motor functions rely on the token naming conventions
 */
static char *_get_format(const INDEX_T i, char *format)
{
	char *ptr;
	char tmp[CMD_STRING_FIELD_LEN];

	strncpy_P(tmp,(PGM_P)pgm_read_word(&cfgArray[i].string), CMD_STRING_FIELD_LEN);
	ptr = strstr(tmp,",");					// find the first separating comma
	ptr = strstr(++ptr,",");				// find the second comma
	ptr++;
	while (*ptr == ' ') ptr++;				// find the first non-whitespace
	strncpy(format, ptr, CMD_FORMAT_LEN+1);
	return (format);
}

static int8_t _get_axis(const INDEX_T i)
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

/****************************************************************************
 * EEPROM access functions:
 * cmd_read_NVM_record()  - return token and value by index number
 * cmd_write_NVM_record() - write token/value record to NVM by index
 */
uint8_t cmd_read_NVM_record(const INDEX_T i, cmdObj *cmd)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) return (TG_UNRECOGNIZED_COMMAND);
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (i * NVM_RECORD_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
	strncpy(cmd->token, (char *)&nvm_record, CMD_TOKEN_LEN);
	memcpy(&cmd->value, &nvm_record[CMD_TOKEN_LEN+1], sizeof(double));
	return (TG_OK);
}

uint8_t cmd_write_NVM_record(const INDEX_T i, const cmdObj *cmd)
{
	if ((i < 0) || (i >= CMD_MAX_INDEX)) return (TG_UNRECOGNIZED_COMMAND);
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (i * NVM_RECORD_LEN);
	strncpy((char *)&nvm_record, cmd->token, CMD_TOKEN_LEN);
	memcpy(&nvm_record[CMD_TOKEN_LEN+1], &cmd->value, sizeof(double));
	(void)EEPROM_WriteBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
	return(TG_OK);
}

uint8_t cmd_read_NVM_multi(const uint16_t addr_offset, char *token, int8_t *data, const uint16_t size)
{
//	int8_t token[CMD_TOKEN_+LEN];
	(void)EEPROM_ReadBytes((cfg.nvm_profile_base + addr_offset), (int8_t *)token, CMD_TOKEN_LEN);
	(void)EEPROM_ReadBytes((cfg.nvm_profile_base + addr_offset + CMD_TOKEN_LEN+1), data, size);
	return (TG_OK);
}

uint8_t cmd_write_NVM_multi(const uint16_t addr_offset, char *token, int8_t *data, const uint16_t size)
{
	return (TG_OK);
}

/*
uint8_t cmd_read_NVM(const uint16_t addr_offset, int8_t *data, const uint16_t size)
{
	(void)EEPROM_ReadBytes((cfg.nvm_profile_base + addr_offset), data, size);
	return (TG_OK);
}

uint8_t cmd_write_NVM(const uint16_t addr_offset, int8_t *data, const uint16_t size)
{
	(void)EEPROM_WriteBytes((cfg.nvm_profile_base + addr_offset), data, size);
	return (TG_OK);
}
*/

/****************************************************************************
 ***** Config Diagnostics ***************************************************
 ****************************************************************************/

/*
 * cfg_dump_NVM() 		- dump current NVM profile to stderr in 8 byte lines
 * _print_NVM_record()	- print a single record
 *
 *	Requires 'label' to be a program memory string. Usage example:
 *		cfg_dump_NVM(0,10,PSTR("Initial state"));
 */
#ifdef __DEBUG

static void _dump_NVM_record(const int16_t index, const int8_t *nvm_record);

void cfg_dump_NVM(const uint16_t start_index, const uint16_t end_index, char *label)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address;
	uint16_t i;

	fprintf_P(stderr, PSTR("\nDump NMV - %S\n"), label);
	for (i=start_index; i<end_index; i++) {
		nvm_address = cfg.nvm_profile_base + (i * NVM_RECORD_LEN);
		(void)EEPROM_ReadBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
		_dump_NVM_record(i, nvm_record);
	}
}

static void _dump_NVM_record(const int16_t index, const int8_t *nvm_record)
{
	char token[CMD_TOKEN_LEN+1];
	double value;

	strncpy(token, (char *)nvm_record, CMD_TOKEN_LEN);
	memcpy(&value, &nvm_record[CMD_TOKEN_LEN+1], sizeof(double));
	fprintf_P(stderr, PSTR("Index %d - %s %1.2f [%d %d %d %d %d %d %d %d]\n"), 
							index, token, value,
							nvm_record[0], nvm_record[1], nvm_record[2], nvm_record[3],
							nvm_record[4], nvm_record[5], nvm_record[6], nvm_record[7]);
}
#endif

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

	i = cmd_get_index_by_token("xfr");

/*
	for (i=0; i<CMD_MAX_INDEX; i++) {

		cmd_get(i, &cmd);

		cmd.value = 42;
		cmd_set(i, &cmd);

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
