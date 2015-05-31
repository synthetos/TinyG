/*
 * config_app.c - application-specific part of configuration data
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* This file contains application specific data for the config system:
 *	- application-specific functions and function prototypes
 *	- application-specific message and print format strings
 *	- application-specific config array
 *	- any other application-specific data or functions
 *
 * See config_app.h for a detailed description of config objects and the config table
 */

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "controller.h"
#include "canonical_machine.h"
#include "gcode_parser.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"
#include "planner.h"
#include "stepper.h"
#include "switch.h"
#include "pwm.h"
#include "report.h"
#include "hardware.h"
#include "test.h"
#include "util.h"
#include "help.h"
#include "network.h"
#include "xio.h"

#ifdef __cplusplus
extern "C"{
#endif

/*** structures ***/

cfgParameters_t cfg; 				// application specific configuration parameters

/***********************************************************************************
 **** application-specific internal functions **************************************
 ***********************************************************************************/
// See config.cpp/.h for generic variables and functions that are not specific to
// TinyG or the motion control application domain

// helpers (most helpers are defined immediately above their usage so they don't need prototypes here)

static stat_t _do_motors(nvObj_t *nv);		// print parameters for all motor groups
static stat_t _do_axes(nvObj_t *nv);		// print parameters for all axis groups
static stat_t _do_offsets(nvObj_t *nv);		// print offset parameters for G54-G59,G92, G28, G30
static stat_t _do_all(nvObj_t *nv);			// print all parameters

// communications settings and functions

static stat_t set_ec(nvObj_t *nv);			// expand CRLF on TX output
static stat_t set_ee(nvObj_t *nv);			// enable character echo
static stat_t set_ex(nvObj_t *nv);			// enable XON/XOFF and RTS/CTS flow control
static stat_t set_baud(nvObj_t *nv);		// set USB baud rate
static stat_t get_rx(nvObj_t *nv);			// get bytes in RX buffer
//static stat_t run_sx(nvObj_t *nv);		// send XOFF, XON

/***********************************************************************************
 **** CONFIG TABLE  ****************************************************************
 ***********************************************************************************
 *
 *	NOTES AND CAVEATS
 *
 *	- Token matching occurs from the most specific to the least specific. This means
 *	  that if shorter tokens overlap longer ones the longer one must precede the
 *	  shorter one. E.g. "gco" needs to come before "gc"
 *
 *	- Mark group strings for entries that have no group as nul -->  "".
 *	  This is important for group expansion.
 *
 *	- Groups do not have groups. Neither do uber-groups, e.g.
 *	  'x' is --> { "", "x",  	and 'm' is --> { "", "m",
 *
 *	- Be careful not to define groups longer than GROUP_LEN (3) and tokens longer
 *	  than TOKEN_LEN (5). (See config.h for lengths). The combined group + token
 *	  cannot exceed TOKEN_LEN. String functions working on the table assume these
 *	  rules are followed and do not check lengths or perform other validation.
 *
 *	NOTE: If the count of lines in cfgArray exceeds 255 you need to change index_t
 *	uint16_t in the config.h file.
 */

const cfgItem_t cfgArray[] PROGMEM = {
	// group token flags p, print_func,	 get_func,  set_func, target for get/set,   	default value
	{ "sys", "fb", _fipn,2, hw_print_fb, get_flt,   set_nul,  (float *)&cs.fw_build,   TINYG_FIRMWARE_BUILD }, // MUST BE FIRST!
	{ "sys", "fv", _fipn,3, hw_print_fv, get_flt,   set_nul,  (float *)&cs.fw_version, TINYG_FIRMWARE_VERSION },
	{ "sys", "hp", _fipn,0, hw_print_hp, get_flt,   set_flt,  (float *)&cs.hw_platform,TINYG_HARDWARE_PLATFORM },
	{ "sys", "hv", _fipn,0, hw_print_hv, get_flt,   hw_set_hv,(float *)&cs.hw_version, TINYG_HARDWARE_VERSION },
	{ "sys", "id", _fn,  0, hw_print_id, hw_get_id, set_nul,  (float *)&cs.null, 0 },  // device ID (ASCII signature)

	// dynamic model attributes for reporting purposes (up front for speed)
	{ "",   "n",   _fi, 0, cm_print_line, cm_get_mline,set_int,(float *)&cm.gm.linenum,0 },		// Model line number
	{ "",   "line",_fi, 0, cm_print_line, cm_get_line, set_int,(float *)&cm.gm.linenum,0 },		// Active line number - model or runtime line number
	{ "",   "vel", _f0, 2, cm_print_vel,  cm_get_vel,  set_nul,(float *)&cs.null, 0 },			// current velocity
	{ "",   "feed",_f0, 2, cm_print_feed, cm_get_feed, set_nul,(float *)&cs.null, 0 },			// feed rate
	{ "",   "stat",_f0, 0, cm_print_stat, cm_get_stat, set_nul,(float *)&cs.null, 0 },			// combined machine state
	{ "",   "macs",_f0, 0, cm_print_macs, cm_get_macs, set_nul,(float *)&cs.null, 0 },			// raw machine state
	{ "",   "cycs",_f0, 0, cm_print_cycs, cm_get_cycs, set_nul,(float *)&cs.null, 0 },			// cycle state
	{ "",   "mots",_f0, 0, cm_print_mots, cm_get_mots, set_nul,(float *)&cs.null, 0 },			// motion state
	{ "",   "hold",_f0, 0, cm_print_hold, cm_get_hold, set_nul,(float *)&cs.null, 0 },			// feedhold state
	{ "",   "unit",_f0, 0, cm_print_unit, cm_get_unit, set_nul,(float *)&cs.null, 0 },			// units mode
	{ "",   "coor",_f0, 0, cm_print_coor, cm_get_coor, set_nul,(float *)&cs.null, 0 },			// coordinate system
	{ "",   "momo",_f0, 0, cm_print_momo, cm_get_momo, set_nul,(float *)&cs.null, 0 },			// motion mode
	{ "",   "plan",_f0, 0, cm_print_plan, cm_get_plan, set_nul,(float *)&cs.null, 0 },			// plane select
	{ "",   "path",_f0, 0, cm_print_path, cm_get_path, set_nul,(float *)&cs.null, 0 },			// path control mode
	{ "",   "dist",_f0, 0, cm_print_dist, cm_get_dist, set_nul,(float *)&cs.null, 0 },			// distance mode
	{ "",   "frmo",_f0, 0, cm_print_frmo, cm_get_frmo, set_nul,(float *)&cs.null, 0 },			// feed rate mode
	{ "",   "tool",_f0, 0, cm_print_tool, cm_get_toolv,set_nul,(float *)&cs.null, 0 },			// active tool
//	{ "",   "tick",_f0, 0, tx_print_int,  get_int,     set_int,(float *)&rtc.sys_ticks, 0 },	// tick count

	{ "mpo","mpox",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// X machine position
	{ "mpo","mpoy",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// Y machine position
	{ "mpo","mpoz",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// Z machine position
	{ "mpo","mpoa",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// A machine position
	{ "mpo","mpob",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// B machine position
	{ "mpo","mpoc",_f0, 3, cm_print_mpo, cm_get_mpo, set_nul,(float *)&cs.null, 0 },			// C machine position

	{ "pos","posx",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// X work position
	{ "pos","posy",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// Y work position
	{ "pos","posz",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// Z work position
	{ "pos","posa",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// A work position
	{ "pos","posb",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// B work position
	{ "pos","posc",_f0, 3, cm_print_pos, cm_get_pos, set_nul,(float *)&cs.null, 0 },			// C work position

	{ "ofs","ofsx",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// X work offset
	{ "ofs","ofsy",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// Y work offset
	{ "ofs","ofsz",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// Z work offset
	{ "ofs","ofsa",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// A work offset
	{ "ofs","ofsb",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// B work offset
	{ "ofs","ofsc",_f0, 3, cm_print_ofs, cm_get_ofs, set_nul,(float *)&cs.null, 0 },			// C work offset

	{ "hom","home",_f0, 0, cm_print_home, cm_get_home, cm_run_home,(float *)&cs.null, 0 },		// homing state, invoke homing cycle
	{ "hom","homx",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_X], false },	// X homed - Homing status group
	{ "hom","homy",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_Y], false },	// Y homed
	{ "hom","homz",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_Z], false },	// Z homed
	{ "hom","homa",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_A], false },	// A homed
	{ "hom","homb",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_B], false },	// B homed
	{ "hom","homc",_f0, 0, cm_print_pos, get_ui8, set_nul,(float *)&cm.homed[AXIS_C], false },	// C homed

	{ "prb","prbe",_f0, 0, tx_print_nul, get_ui8, set_nul,(float *)&cm.probe_state, 0 },		// probing state
	{ "prb","prbx",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_X], 0 },
	{ "prb","prby",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_Y], 0 },
	{ "prb","prbz",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_Z], 0 },
	{ "prb","prba",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_A], 0 },
	{ "prb","prbb",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_B], 0 },
	{ "prb","prbc",_f0, 3, tx_print_nul, get_flt, set_nul,(float *)&cm.probe_results[AXIS_C], 0 },

	{ "jog","jogx",_f0, 0, tx_print_nul, get_nul, cm_run_jogx, (float *)&cm.jogging_dest, 0},
	{ "jog","jogy",_f0, 0, tx_print_nul, get_nul, cm_run_jogy, (float *)&cm.jogging_dest, 0},
	{ "jog","jogz",_f0, 0, tx_print_nul, get_nul, cm_run_jogz, (float *)&cm.jogging_dest, 0},
	{ "jog","joga",_f0, 0, tx_print_nul, get_nul, cm_run_joga, (float *)&cm.jogging_dest, 0},
//	{ "jog","jogb",_f0, 0, tx_print_nul, get_nul, cm_run_jogb, (float *)&cm.jogging_dest, 0},
//	{ "jog","jogc",_f0, 0, tx_print_nul, get_nul, cm_run_jogc, (float *)&cm.jogging_dest, 0},

	{ "pwr","pwr1",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},	// motor power enable readouts
	{ "pwr","pwr2",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},
	{ "pwr","pwr3",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},
	{ "pwr","pwr4",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},
#if (MOTORS >= 5)
	{ "pwr","pwr5",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},
#endif
#if (MOTORS >= 6)
	{ "pwr","pwr6",_f0, 0, st_print_pwr, st_get_pwr, set_nul, (float *)&cs.null, 0},
#endif

	// Reports, tests, help, and messages
	{ "", "sr",  _f0, 0, sr_print_sr,  sr_get,  sr_set,   (float *)&cs.null, 0 },	// status report object
	{ "", "qr",  _f0, 0, qr_print_qr,  qr_get,  set_nul,  (float *)&cs.null, 0 },	// queue report - planner buffers available
	{ "", "qi",  _f0, 0, qr_print_qi,  qi_get,  set_nul,  (float *)&cs.null, 0 },	// queue report - buffers added to queue
	{ "", "qo",  _f0, 0, qr_print_qo,  qo_get,  set_nul,  (float *)&cs.null, 0 },	// queue report - buffers removed from queue
	{ "", "er",  _f0, 0, tx_print_nul, rpt_er,  set_nul,  (float *)&cs.null, 0 },	// invoke bogus exception report for testing
	{ "", "qf",  _f0, 0, tx_print_nul, get_nul, cm_run_qf,(float *)&cs.null, 0 },	// queue flush
	{ "", "rx",  _f0, 0, tx_print_int, get_rx,  set_nul,  (float *)&cs.null, 0 },	// space in RX buffer
	{ "", "msg", _f0, 0, tx_print_str, get_nul, set_nul,  (float *)&cs.null, 0 },	// string for generic messages
//	{ "", "clc", _f0, 0, tx_print_nul, st_clc,  st_clc,   (float *)&cs.null, 0 },	// clear diagnostic step counters
	{ "", "clear",_f0,0, tx_print_nul, cm_clear,cm_clear, (float *)&cs.null, 0 },	// GET a clear to clear soft alarm
//	{ "", "sx",  _f0, 0, tx_print_nul, run_sx,  run_sx ,  (float *)&cs.null, 0 },	// send XOFF, XON test

	{ "", "test",_f0, 0, tx_print_nul, help_test, run_test, (float *)&cs.null,0 },	// run tests, print test help screen
	{ "", "defa",_f0, 0, tx_print_nul, help_defa, set_defaults,(float *)&cs.null,0 },	// set/print defaults / help screen
	{ "", "boot",_f0, 0, tx_print_nul, help_boot_loader,hw_run_boot, (float *)&cs.null,0 },

#ifdef __HELP_SCREENS
	{ "", "help",_f0, 0, tx_print_nul, help_config, set_nul, (float *)&cs.null,0 },  // prints config help screen
	{ "", "h",   _f0, 0, tx_print_nul, help_config, set_nul, (float *)&cs.null,0 },  // alias for "help"
#endif

	// Motor parameters
	{ "1","1ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_1].motor_map,	M1_MOTOR_MAP },
	{ "1","1sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_1].step_angle,	M1_STEP_ANGLE },
	{ "1","1tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_1].travel_rev,	M1_TRAVEL_PER_REV },
	{ "1","1mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_1].microsteps,	M1_MICROSTEPS },
	{ "1","1po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_1].polarity,	M1_POLARITY },
	{ "1","1pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_1].power_mode,	M1_POWER_MODE },
#ifdef __ARM
	{ "1","1pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_1].power_level,M1_POWER_LEVEL },
#endif
#if (MOTORS >= 2)
	{ "2","2ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_2].motor_map,	M2_MOTOR_MAP },
	{ "2","2sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_2].step_angle,	M2_STEP_ANGLE },
	{ "2","2tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_2].travel_rev,	M2_TRAVEL_PER_REV },
	{ "2","2mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_2].microsteps,	M2_MICROSTEPS },
	{ "2","2po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_2].polarity,	M2_POLARITY },
	{ "2","2pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_2].power_mode,	M2_POWER_MODE },
#ifdef __ARM
	{ "2","2pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_2].power_level,M2_POWER_LEVEL},
#endif
#endif
#if (MOTORS >= 3)
	{ "3","3ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_3].motor_map,	M3_MOTOR_MAP },
	{ "3","3sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_3].step_angle,	M3_STEP_ANGLE },
	{ "3","3tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_3].travel_rev,	M3_TRAVEL_PER_REV },
	{ "3","3mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_3].microsteps,	M3_MICROSTEPS },
	{ "3","3po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_3].polarity,	M3_POLARITY },
	{ "3","3pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_3].power_mode,	M3_POWER_MODE },
#ifdef __ARM
	{ "3","3pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_3].power_level,M3_POWER_LEVEL },
#endif
#endif
#if (MOTORS >= 4)
	{ "4","4ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_4].motor_map,	M4_MOTOR_MAP },
	{ "4","4sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_4].step_angle,	M4_STEP_ANGLE },
	{ "4","4tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_4].travel_rev,	M4_TRAVEL_PER_REV },
	{ "4","4mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_4].microsteps,	M4_MICROSTEPS },
	{ "4","4po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_4].polarity,	M4_POLARITY },
	{ "4","4pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_4].power_mode,	M4_POWER_MODE },
#ifdef __ARM
	{ "4","4pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_4].power_level,M4_POWER_LEVEL },
#endif
#endif
#if (MOTORS >= 5)
	{ "5","5ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_5].motor_map,	M5_MOTOR_MAP },
	{ "5","5sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_5].step_angle,	M5_STEP_ANGLE },
	{ "5","5tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_5].travel_rev,	M5_TRAVEL_PER_REV },
	{ "5","5mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_5].microsteps,	M5_MICROSTEPS },
	{ "5","5po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_5].polarity,	M5_POLARITY },
	{ "5","5pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_5].power_mode,	M5_POWER_MODE },
#ifdef __ARM
	{ "5","5pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_5].power_level,M5_POWER_LEVEL },
#endif
#endif
#if (MOTORS >= 6)
	{ "6","6ma",_fip, 0, st_print_ma, get_ui8, set_ui8,   (float *)&st_cfg.mot[MOTOR_6].motor_map,	M6_MOTOR_MAP },
	{ "6","6sa",_fip, 3, st_print_sa, get_flt, st_set_sa, (float *)&st_cfg.mot[MOTOR_6].step_angle,	M6_STEP_ANGLE },
	{ "6","6tr",_fipc,4, st_print_tr, get_flt, st_set_tr, (float *)&st_cfg.mot[MOTOR_6].travel_rev,	M6_TRAVEL_PER_REV },
	{ "6","6mi",_fip, 0, st_print_mi, get_ui8, st_set_mi, (float *)&st_cfg.mot[MOTOR_6].microsteps,	M6_MICROSTEPS },
	{ "6","6po",_fip, 0, st_print_po, get_ui8, set_01,    (float *)&st_cfg.mot[MOTOR_6].polarity,	M6_POLARITY },
	{ "6","6pm",_fip, 0, st_print_pm, get_ui8, st_set_pm, (float *)&st_cfg.mot[MOTOR_6].power_mode,	M6_POWER_MODE },
#ifdef __ARM
	{ "6","6pl",_fip, 3, st_print_pl, get_flt, st_set_pl, (float *)&st_cfg.mot[MOTOR_6].power_level,M6_POWER_LEVEL },
#endif
#endif
	// Axis parameters
	{ "x","xam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_X].axis_mode,		X_AXIS_MODE },
	{ "x","xvm",_fipc, 0, cm_print_vm, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].velocity_max,	X_VELOCITY_MAX },
	{ "x","xfr",_fipc, 0, cm_print_fr, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].feedrate_max,	X_FEEDRATE_MAX },
	{ "x","xtn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].travel_min,		X_TRAVEL_MIN },
	{ "x","xtm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].travel_max,		X_TRAVEL_MAX },
	{ "x","xjm",_fipc, 0, cm_print_jm, get_flt,   cm_set_xjm,(float *)&cm.a[AXIS_X].jerk_max,		X_JERK_MAX },
	{ "x","xjh",_fipc, 0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_X].jerk_homing,	X_JERK_HOMING },
	{ "x","xjd",_fipc, 4, cm_print_jd, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].junction_dev,	X_JUNCTION_DEVIATION },
	{ "x","xsn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.mode[0],					X_SWITCH_MODE_MIN },
	{ "x","xsx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.mode[1],					X_SWITCH_MODE_MAX },
//	{ "x","xsn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_X][SW_MIN].mode,	X_SWITCH_MODE_MIN },	// new style
//	{ "x","xsx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_X][SW_MAX].mode,	X_SWITCH_MODE_MAX },	// new style
	{ "x","xsv",_fipc, 0, cm_print_sv, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].search_velocity,X_SEARCH_VELOCITY },
	{ "x","xlv",_fipc, 0, cm_print_lv, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].latch_velocity,	X_LATCH_VELOCITY },
	{ "x","xlb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].latch_backoff,	X_LATCH_BACKOFF },
	{ "x","xzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].zero_backoff,	X_ZERO_BACKOFF },

	{ "y","yam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_Y].axis_mode,		Y_AXIS_MODE },
	{ "y","yvm",_fipc, 0, cm_print_vm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].velocity_max,	Y_VELOCITY_MAX },
	{ "y","yfr",_fipc, 0, cm_print_fr, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].feedrate_max,	Y_FEEDRATE_MAX },
	{ "y","ytn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].travel_min,		Y_TRAVEL_MIN },
	{ "y","ytm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].travel_max,		Y_TRAVEL_MAX },
	{ "y","yjm",_fipc, 0, cm_print_jm, get_flt,	  cm_set_xjm,(float *)&cm.a[AXIS_Y].jerk_max,		Y_JERK_MAX },
	{ "y","yjh",_fipc, 0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_Y].jerk_homing,	Y_JERK_HOMING },
	{ "y","yjd",_fipc, 4, cm_print_jd, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].junction_dev,	Y_JUNCTION_DEVIATION },
	{ "y","ysn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.mode[2],					Y_SWITCH_MODE_MIN },
	{ "y","ysx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.mode[3],					Y_SWITCH_MODE_MAX },
//	{ "y","ysn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_Y][SW_MIN].mode,	Y_SWITCH_MODE_MIN },	// new style
//	{ "y","ysx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_Y][SW_MAX].mode,	Y_SWITCH_MODE_MAX },	// new style
	{ "y","ysv",_fipc, 0, cm_print_sv, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].search_velocity,Y_SEARCH_VELOCITY },
	{ "y","ylv",_fipc, 0, cm_print_lv, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].latch_velocity,	Y_LATCH_VELOCITY },
	{ "y","ylb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].latch_backoff,	Y_LATCH_BACKOFF },
	{ "y","yzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].zero_backoff,	Y_ZERO_BACKOFF },

	{ "z","zam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_Z].axis_mode,		Z_AXIS_MODE },
	{ "z","zvm",_fipc, 0, cm_print_vm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].velocity_max,	Z_VELOCITY_MAX },
	{ "z","zfr",_fipc, 0, cm_print_fr, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].feedrate_max,	Z_FEEDRATE_MAX },
	{ "z","ztn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].travel_min,		Z_TRAVEL_MIN },
	{ "z","ztm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].travel_max,		Z_TRAVEL_MAX },
	{ "z","zjm",_fipc, 0, cm_print_jm, get_flt,	  cm_set_xjm,(float *)&cm.a[AXIS_Z].jerk_max,		Z_JERK_MAX },
	{ "z","zjh",_fipc, 0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_Z].jerk_homing, 	Z_JERK_HOMING },
	{ "z","zjd",_fipc, 4, cm_print_jd, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].junction_dev,	Z_JUNCTION_DEVIATION },
	{ "z","zsn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.mode[4],					Z_SWITCH_MODE_MIN },
	{ "z","zsx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.mode[5],					Z_SWITCH_MODE_MAX },
//	{ "z","zsn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_Z][SW_MIN].mode,	Z_SWITCH_MODE_MIN },	// new style
//	{ "z","zsx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_Z][SW_MAX].mode,	Z_SWITCH_MODE_MAX },	// new style
	{ "z","zsv",_fipc, 0, cm_print_sv, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].search_velocity,Z_SEARCH_VELOCITY },
	{ "z","zlv",_fipc, 0, cm_print_lv, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].latch_velocity,	Z_LATCH_VELOCITY },
	{ "z","zlb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].latch_backoff,	Z_LATCH_BACKOFF },
	{ "z","zzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].zero_backoff,	Z_ZERO_BACKOFF },

	{ "a","aam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_A].axis_mode,		A_AXIS_MODE },
	{ "a","avm",_fip,  0, cm_print_vm, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].velocity_max,	A_VELOCITY_MAX },
	{ "a","afr",_fip,  0, cm_print_fr, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].feedrate_max,	A_FEEDRATE_MAX },
	{ "a","atn",_fip,  3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_A].travel_min,		A_TRAVEL_MIN },
	{ "a","atm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].travel_max,		A_TRAVEL_MAX },
	{ "a","ajm",_fip,  0, cm_print_jm, get_flt,	  cm_set_xjm,(float *)&cm.a[AXIS_A].jerk_max,		A_JERK_MAX },
	{ "a","ajh",_fip,  0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_A].jerk_homing, 	A_JERK_HOMING },
	{ "a","ajd",_fip,  4, cm_print_jd, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].junction_dev,	A_JUNCTION_DEVIATION },
	{ "a","ara",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].radius,			A_RADIUS},
	{ "a","asn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.mode[6],					A_SWITCH_MODE_MIN },
	{ "a","asx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.mode[7],					A_SWITCH_MODE_MAX },
//	{ "a","asn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_A][SW_MIN].mode,	A_SWITCH_MODE_MIN },	// new style
//	{ "a","asx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_A][SW_MAX].mode,	A_SWITCH_MODE_MAX },	// new style
	{ "a","asv",_fip,  0, cm_print_sv, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].search_velocity,A_SEARCH_VELOCITY },
	{ "a","alv",_fip,  0, cm_print_lv, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].latch_velocity,	A_LATCH_VELOCITY },
	{ "a","alb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].latch_backoff,	A_LATCH_BACKOFF },
	{ "a","azb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].zero_backoff,	A_ZERO_BACKOFF },

	{ "b","bam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_B].axis_mode,		B_AXIS_MODE },
	{ "b","bvm",_fip,  0, cm_print_vm, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].velocity_max,	B_VELOCITY_MAX },
	{ "b","bfr",_fip,  0, cm_print_fr, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].feedrate_max,	B_FEEDRATE_MAX },
	{ "b","btn",_fip,  3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_B].travel_min,		B_TRAVEL_MIN },
	{ "b","btm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].travel_max,		B_TRAVEL_MAX },
	{ "b","bjm",_fip,  0, cm_print_jm, get_flt,	  cm_set_xjm,(float *)&cm.a[AXIS_B].jerk_max,		B_JERK_MAX },
	{ "b","bjd",_fip,  0, cm_print_jd, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].junction_dev,	B_JUNCTION_DEVIATION },
	{ "b","bra",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].radius,			B_RADIUS },
#ifdef __ARM	// B axis extended parameters
	{ "b","asn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_B][SW_MIN].mode,	B_SWITCH_MODE_MIN },
	{ "b","asx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_B][SW_MAX].mode,	B_SWITCH_MODE_MAX },
	{ "b","bsv",_fip,  0, cm_print_sv, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].search_velocity,B_SEARCH_VELOCITY },
	{ "b","blv",_fip,  0, cm_print_lv, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].latch_velocity,	B_LATCH_VELOCITY },
	{ "b","blb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].latch_backoff,	B_LATCH_BACKOFF },
	{ "b","bzb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].zero_backoff,	B_ZERO_BACKOFF },
	{ "b","bjh",_fip,  0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_B].jerk_homing,	B_JERK_HOMING },
#endif

	{ "c","cam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_C].axis_mode,		C_AXIS_MODE },
	{ "c","cvm",_fip,  0, cm_print_vm, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].velocity_max,	C_VELOCITY_MAX },
	{ "c","cfr",_fip,  0, cm_print_fr, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].feedrate_max,	C_FEEDRATE_MAX },
	{ "c","ctn",_fip,  3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_C].travel_min,		C_TRAVEL_MIN },
	{ "c","ctm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].travel_max,		C_TRAVEL_MAX },
	{ "c","cjm",_fip,  0, cm_print_jm, get_flt,	  cm_set_xjm,(float *)&cm.a[AXIS_C].jerk_max,		C_JERK_MAX },
	{ "c","cjd",_fip,  0, cm_print_jd, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].junction_dev,	C_JUNCTION_DEVIATION },
	{ "c","cra",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].radius,			C_RADIUS },
#ifdef __ARM	// C axis extended parameters
	{ "c","csn",_fip,  0, cm_print_sn, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_C][SW_MIN].mode,	C_SWITCH_MODE_MIN },
	{ "c","csx",_fip,  0, cm_print_sx, get_ui8,   sw_set_sw, (float *)&sw.s[AXIS_C][SW_MAX].mode,	C_SWITCH_MODE_MAX },
	{ "c","csv",_fip,  0, cm_print_sv, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].search_velocity,C_SEARCH_VELOCITY },
	{ "c","clv",_fip,  0, cm_print_lv, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].latch_velocity,	C_LATCH_VELOCITY },
	{ "c","clb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].latch_backoff,	C_LATCH_BACKOFF },
	{ "c","czb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].zero_backoff,	C_ZERO_BACKOFF },
	{ "c","cjh",_fip,  0, cm_print_jh, get_flt,	  cm_set_xjh,(float *)&cm.a[AXIS_C].jerk_homing, 	C_JERK_HOMING },
#endif

	// PWM settings
	{ "p1","p1frq",_fip, 0, pwm_print_p1frq, get_flt, set_flt,(float *)&pwm.c[PWM_1].frequency,		P1_PWM_FREQUENCY },
	{ "p1","p1csl",_fip, 0, pwm_print_p1csl, get_flt, set_flt,(float *)&pwm.c[PWM_1].cw_speed_lo,	P1_CW_SPEED_LO },
	{ "p1","p1csh",_fip, 0, pwm_print_p1csh, get_flt, set_flt,(float *)&pwm.c[PWM_1].cw_speed_hi,	P1_CW_SPEED_HI },
	{ "p1","p1cpl",_fip, 3, pwm_print_p1cpl, get_flt, set_flt,(float *)&pwm.c[PWM_1].cw_phase_lo,	P1_CW_PHASE_LO },
	{ "p1","p1cph",_fip, 3, pwm_print_p1cph, get_flt, set_flt,(float *)&pwm.c[PWM_1].cw_phase_hi,	P1_CW_PHASE_HI },
	{ "p1","p1wsl",_fip, 0, pwm_print_p1wsl, get_flt, set_flt,(float *)&pwm.c[PWM_1].ccw_speed_lo,	P1_CCW_SPEED_LO },
	{ "p1","p1wsh",_fip, 0, pwm_print_p1wsh, get_flt, set_flt,(float *)&pwm.c[PWM_1].ccw_speed_hi,	P1_CCW_SPEED_HI },
	{ "p1","p1wpl",_fip, 3, pwm_print_p1wpl, get_flt, set_flt,(float *)&pwm.c[PWM_1].ccw_phase_lo,	P1_CCW_PHASE_LO },
	{ "p1","p1wph",_fip, 3, pwm_print_p1wph, get_flt, set_flt,(float *)&pwm.c[PWM_1].ccw_phase_hi,	P1_CCW_PHASE_HI },
	{ "p1","p1pof",_fip, 3, pwm_print_p1pof, get_flt, set_flt,(float *)&pwm.c[PWM_1].phase_off,		P1_PWM_PHASE_OFF },

	// Coordinate system offsets (G54-G59 and G92)
	{ "g54","g54x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_X], G54_X_OFFSET },
	{ "g54","g54y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_Y], G54_Y_OFFSET },
	{ "g54","g54z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_Z], G54_Z_OFFSET },
	{ "g54","g54a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_A], G54_A_OFFSET },
	{ "g54","g54b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_B], G54_B_OFFSET },
	{ "g54","g54c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_C], G54_C_OFFSET },

	{ "g55","g55x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_X], G55_X_OFFSET },
	{ "g55","g55y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_Y], G55_Y_OFFSET },
	{ "g55","g55z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_Z], G55_Z_OFFSET },
	{ "g55","g55a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_A], G55_A_OFFSET },
	{ "g55","g55b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_B], G55_B_OFFSET },
	{ "g55","g55c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_C], G55_C_OFFSET },

	{ "g56","g56x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_X], G56_X_OFFSET },
	{ "g56","g56y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_Y], G56_Y_OFFSET },
	{ "g56","g56z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_Z], G56_Z_OFFSET },
	{ "g56","g56a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_A], G56_A_OFFSET },
	{ "g56","g56b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_B], G56_B_OFFSET },
	{ "g56","g56c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_C], G56_C_OFFSET },

	{ "g57","g57x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_X], G57_X_OFFSET },
	{ "g57","g57y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_Y], G57_Y_OFFSET },
	{ "g57","g57z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_Z], G57_Z_OFFSET },
	{ "g57","g57a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_A], G57_A_OFFSET },
	{ "g57","g57b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_B], G57_B_OFFSET },
	{ "g57","g57c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_C], G57_C_OFFSET },

	{ "g58","g58x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_X], G58_X_OFFSET },
	{ "g58","g58y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_Y], G58_Y_OFFSET },
	{ "g58","g58z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_Z], G58_Z_OFFSET },
	{ "g58","g58a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_A], G58_A_OFFSET },
	{ "g58","g58b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_B], G58_B_OFFSET },
	{ "g58","g58c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_C], G58_C_OFFSET },

	{ "g59","g59x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_X], G59_X_OFFSET },
	{ "g59","g59y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_Y], G59_Y_OFFSET },
	{ "g59","g59z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_Z], G59_Z_OFFSET },
	{ "g59","g59a",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_A], G59_A_OFFSET },
	{ "g59","g59b",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_B], G59_B_OFFSET },
	{ "g59","g59c",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_C], G59_C_OFFSET },

	{ "g92","g92x",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_X], 0 },// G92 handled differently
	{ "g92","g92y",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_Y], 0 },
	{ "g92","g92z",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_Z], 0 },
	{ "g92","g92a",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_A], 0 },
	{ "g92","g92b",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_B], 0 },
	{ "g92","g92c",_fi, 3, cm_print_cofs, get_flt, set_nul,(float *)&cm.gmx.origin_offset[AXIS_C], 0 },

	// Coordinate positions (G28, G30)
	{ "g28","g28x",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_X], 0 },// g28 handled differently
	{ "g28","g28y",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_Y], 0 },
	{ "g28","g28z",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_Z], 0 },
	{ "g28","g28a",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_A], 0 },
	{ "g28","g28b",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_B], 0 },
	{ "g28","g28c",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g28_position[AXIS_C], 0 },

	{ "g30","g30x",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_X], 0 },// g30 handled differently
	{ "g30","g30y",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_Y], 0 },
	{ "g30","g30z",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_Z], 0 },
	{ "g30","g30a",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_A], 0 },
	{ "g30","g30b",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_B], 0 },
	{ "g30","g30c",_fi, 3, cm_print_cpos, get_flt, set_nul,(float *)&cm.gmx.g30_position[AXIS_C], 0 },

	// this is a 128bit UUID for identifying a previously committed job state
	{ "jid","jida",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cs.job_id[0], 0},
	{ "jid","jidb",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cs.job_id[1], 0},
	{ "jid","jidc",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cs.job_id[2], 0},
	{ "jid","jidd",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cs.job_id[3], 0},

	// System parameters
	{ "sys","ja",  _fipnc,0, cm_print_ja,  get_flt,   set_flu,    (float *)&cm.junction_acceleration,JUNCTION_ACCELERATION },
	{ "sys","ct",  _fipnc,4, cm_print_ct,  get_flt,   set_flu,    (float *)&cm.chordal_tolerance,	CHORDAL_TOLERANCE },
	{ "sys","sl",  _fipn, 0, cm_print_sl,  get_ui8,   set_ui8,    (float *)&cm.soft_limit_enable,	SOFT_LIMIT_ENABLE },
	{ "sys","st",  _fipn, 0, sw_print_st,  get_ui8,   sw_set_st,  (float *)&sw.switch_type,			SWITCH_TYPE },
	{ "sys","mt",  _fipn, 2, st_print_mt,  get_flt,   st_set_mt,  (float *)&st_cfg.motor_power_timeout,MOTOR_IDLE_TIMEOUT},
	{ "",   "me",  _f0,   0, tx_print_str, st_set_me, st_set_me,  (float *)&cs.null, 0 },
	{ "",   "md",  _f0,   0, tx_print_str, st_set_md, st_set_md,  (float *)&cs.null, 0 },

	{ "sys","ej",  _fipn, 0, js_print_ej,  get_ui8,   set_01,     (float *)&cfg.comm_mode,			COMM_MODE },
	{ "sys","jv",  _fipn, 0, js_print_jv,  get_ui8,   json_set_jv,(float *)&js.json_verbosity,		JSON_VERBOSITY },
	{ "sys","js",  _fipn, 0, js_print_js,  get_ui8,   set_01,     (float *)&js.json_syntax, 		JSON_SYNTAX_MODE },
	{ "sys","tv",  _fipn, 0, tx_print_tv,  get_ui8,   set_01,     (float *)&txt.text_verbosity,		TEXT_VERBOSITY },
	{ "sys","qv",  _fipn, 0, qr_print_qv,  get_ui8,   set_0123,   (float *)&qr.queue_report_verbosity,QUEUE_REPORT_VERBOSITY },
	{ "sys","sv",  _fipn, 0, sr_print_sv,  get_ui8,   set_012,    (float *)&sr.status_report_verbosity,STATUS_REPORT_VERBOSITY },
	{ "sys","si",  _fipn, 0, sr_print_si,  get_int,   sr_set_si,  (float *)&sr.status_report_interval,STATUS_REPORT_INTERVAL_MS },
//	{ "sys","spi", _fipn, 0, xio_print_spi,get_ui8,   xio_set_spi,(float *)&xio.spi_state,			0 },

	{ "sys","ec",  _fipn, 0, cfg_print_ec,  get_ui8,   set_ec,     (float *)&cfg.enable_cr,			COM_EXPAND_CR },
	{ "sys","ee",  _fipn, 0, cfg_print_ee,  get_ui8,   set_ee,     (float *)&cfg.enable_echo,		COM_ENABLE_ECHO },
	{ "sys","ex",  _fipn, 0, cfg_print_ex,  get_ui8,   set_ex,     (float *)&cfg.enable_flow_control,COM_ENABLE_FLOW_CONTROL },
	{ "sys","baud",_fn,   0, cfg_print_baud,get_ui8,   set_baud,   (float *)&cfg.usb_baud_rate,		XIO_BAUD_115200 },
	{ "sys","net", _fipn, 0, cfg_print_net, get_ui8,   set_ui8,    (float *)&cs.network_mode,		NETWORK_MODE },

	// switch state readouts
/*
	{ "ss","ss0",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[0], 0 },
	{ "ss","ss1",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[1], 0 },
	{ "ss","ss2",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[2], 0 },
	{ "ss","ss3",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[3], 0 },
	{ "ss","ss4",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[4], 0 },
	{ "ss","ss5",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[5], 0 },
	{ "ss","ss6",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[6], 0 },
	{ "ss","ss7",  _f0, 0, print_ss, get_ui8, set_nul, (float *)&sw.state[7], 0 },
*/
	// NOTE: The ordering within the gcode defaults is important for token resolution
	{ "sys","gpl", _fipn, 0, cm_print_gpl, get_ui8, set_012, (float *)&cm.select_plane,	GCODE_DEFAULT_PLANE },
	{ "sys","gun", _fipn, 0, cm_print_gun, get_ui8, set_01,  (float *)&cm.units_mode,	GCODE_DEFAULT_UNITS },
	{ "sys","gco", _fipn, 0, cm_print_gco, get_ui8, set_ui8, (float *)&cm.coord_system,	GCODE_DEFAULT_COORD_SYSTEM },
	{ "sys","gpa", _fipn, 0, cm_print_gpa, get_ui8, set_012, (float *)&cm.path_control,	GCODE_DEFAULT_PATH_CONTROL },
	{ "sys","gdi", _fipn, 0, cm_print_gdi, get_ui8, set_01,  (float *)&cm.distance_mode,GCODE_DEFAULT_DISTANCE_MODE },
	{ "",   "gc",  _f0,   0, tx_print_nul, gc_get_gc, gc_run_gc,(float *)&cs.null, 0 }, // gcode block - must be last in this group

	// "hidden" parameters (not in system group)
//	{ "",   "ms",  _fip, 0, cm_print_ms,  get_flt, set_flt, (float *)&cm.estd_segment_usec,	NOM_SEGMENT_USEC },
//	{ "",   "ml",  _fipc,4, cm_print_ml,  get_flt, set_flu, (float *)&cm.min_segment_len,	MIN_LINE_LENGTH },
	{ "",   "ma",  _fipc,4, cm_print_ma,  get_flt, set_flu, (float *)&cm.arc_segment_len,	ARC_SEGMENT_LENGTH },
	{ "",   "fd",  _fip, 0, tx_print_ui8, get_ui8, set_01,  (float *)&js.json_footer_depth,	JSON_FOOTER_DEPTH },

	// User defined data groups
	{ "uda","uda0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[0], USER_DATA_A0 },
	{ "uda","uda1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[1], USER_DATA_A1 },
	{ "uda","uda2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[2], USER_DATA_A2 },
	{ "uda","uda3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[3], USER_DATA_A3 },

	{ "udb","udb0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[0], USER_DATA_B0 },
	{ "udb","udb1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[1], USER_DATA_B1 },
	{ "udb","udb2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[2], USER_DATA_B2 },
	{ "udb","udb3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[3], USER_DATA_B3 },

	{ "udc","udc0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[0], USER_DATA_C0 },
	{ "udc","udc1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[1], USER_DATA_C1 },
	{ "udc","udc2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[2], USER_DATA_C2 },
	{ "udc","udc3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[3], USER_DATA_C3 },

	{ "udd","udd0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[0], USER_DATA_D0 },
	{ "udd","udd1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[1], USER_DATA_D1 },
	{ "udd","udd2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[2], USER_DATA_D2 },
	{ "udd","udd3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[3], USER_DATA_D3 },

	// Diagnostic parameters
#ifdef __DIAGNOSTIC_PARAMETERS
	{ "_te","_tex",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_X], 0 },			// X target endpoint
	{ "_te","_tey",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_Y], 0 },
	{ "_te","_tez",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_Z], 0 },
	{ "_te","_tea",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_A], 0 },
	{ "_te","_teb",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_B], 0 },
	{ "_te","_tec",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_C], 0 },

	{ "_tr","_trx",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_X], 0 },			// X target runtime
	{ "_tr","_try",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_Y], 0 },
	{ "_tr","_trz",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_Z], 0 },
	{ "_tr","_tra",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_A], 0 },
	{ "_tr","_trb",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_B], 0 },
	{ "_tr","_trc",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_C], 0 },

#if (MOTORS >= 1)
	{ "_ts","_ts1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_1], 0 },		// Motor 1 target steps
	{ "_ps","_ps1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_1], 0 },	// Motor 1 position steps
	{ "_cs","_cs1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_1], 0 },	// Motor 1 commanded steps (delayed steps)
	{ "_es","_es1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_1], 0 },	// Motor 1 encoder steps
	{ "_xs","_xs1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_1].corrected_steps, 0 }, // Motor 1 correction steps applied
	{ "_fe","_fe1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_1], 0 },	// Motor 1 following error in steps
#endif
#if (MOTORS >= 2)
	{ "_ts","_ts2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_2], 0 },
	{ "_ps","_ps2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_2], 0 },
	{ "_cs","_cs2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_2], 0 },
	{ "_es","_es2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_2], 0 },
	{ "_xs","_xs2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_2].corrected_steps, 0 },
	{ "_fe","_fe2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_2], 0 },
#endif
#if (MOTORS >= 3)
	{ "_ts","_ts3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_3], 0 },
	{ "_ps","_ps3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_3], 0 },
	{ "_cs","_cs3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_3], 0 },
	{ "_es","_es3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_3], 0 },
	{ "_xs","_xs3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_3].corrected_steps, 0 },
	{ "_fe","_fe3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_3], 0 },
#endif
#if (MOTORS >= 4)
	{ "_ts","_ts4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_4], 0 },
	{ "_ps","_ps4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_4], 0 },
	{ "_cs","_cs4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_4], 0 },
	{ "_es","_es4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_4], 0 },
	{ "_xs","_xs4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_4].corrected_steps, 0 },
	{ "_fe","_fe4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_4], 0 },
#endif
#if (MOTORS >= 5)
	{ "_ts","_ts5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_5], 0 },
	{ "_ps","_ps5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_5], 0 },
	{ "_cs","_cs5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_5], 0 },
	{ "_es","_es5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_5], 0 },
	{ "_xs","_xs6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_6].corrected_steps, 0 },
	{ "_fe","_fe5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_5], 0 },
#endif
#if (MOTORS >= 6)
	{ "_ts","_ts6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_6], 0 },
	{ "_ps","_ps6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_6], 0 },
	{ "_cs","_cs6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_6], 0 },
	{ "_es","_es6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_6], 0 },
	{ "_xs","_xs5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_5].corrected_steps, 0 },
	{ "_fe","_fe6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_6], 0 },
#endif
	{ "",   "_dam",_f0, 0, tx_print_nul, cm_dam,  cm_dam, (float *)&cs.null, 0 },	// dump active model
#endif	//  __DIAGNOSTIC_PARAMETERS

	// Persistence for status report - must be in sequence
	// *** Count must agree with NV_STATUS_REPORT_LEN in config.h ***
	{ "","se00",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[0],0 },
	{ "","se01",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[1],0 },
	{ "","se02",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[2],0 },
	{ "","se03",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[3],0 },
	{ "","se04",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[4],0 },
	{ "","se05",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[5],0 },
	{ "","se06",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[6],0 },
	{ "","se07",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[7],0 },
	{ "","se08",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[8],0 },
	{ "","se09",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[9],0 },
	{ "","se10",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[10],0 },
	{ "","se11",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[11],0 },
	{ "","se12",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[12],0 },
	{ "","se13",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[13],0 },
	{ "","se14",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[14],0 },
	{ "","se15",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[15],0 },
	{ "","se16",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[16],0 },
	{ "","se17",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[17],0 },
	{ "","se18",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[18],0 },
	{ "","se19",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[19],0 },
	{ "","se20",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[20],0 },
	{ "","se21",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[21],0 },
	{ "","se22",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[22],0 },
	{ "","se23",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[23],0 },
	{ "","se24",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[24],0 },
	{ "","se25",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[25],0 },
	{ "","se26",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[26],0 },
	{ "","se27",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[27],0 },
	{ "","se28",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[28],0 },
	{ "","se29",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[29],0 },
// Count is 30, since se00 counts as one.

	// Group lookups - must follow the single-valued entries for proper sub-string matching
	// *** Must agree with NV_COUNT_GROUPS below ***
	// *** START COUNTING FROM HERE ***
	{ "","sys",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// system group
	{ "","p1", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// PWM 1 group

	{ "","1",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// motor groups
	{ "","2",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","3",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","4",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#if (MOTORS >= 5)
	{ "","5",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#endif
#if (MOTORS >= 6)
	{ "","6",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#endif

	{ "","x",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// axis groups
	{ "","y",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","z",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","a",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","b",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","c",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },

//	{ "","ss", _f0, 0, tx_print_nul, get_grp, set_nul,(float *)&cs.null,0 },	// switch states
	{ "","g54",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// coord offset groups
	{ "","g55",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g56",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g57",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g58",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g59",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
	{ "","g92",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// origin offsets
	{ "","g28",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// g28 home position
	{ "","g30",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// g30 home position

	{ "","mpo",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// machine position group
	{ "","pos",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// work position group
	{ "","ofs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// work offset group
	{ "","hom",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// axis homing state group
	{ "","prb",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// probing state group
	{ "","pwr",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// motor power enagled group
	{ "","jog",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// axis jogging state group
	{ "","jid",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// job ID group

	{ "","uda", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// user data group
	{ "","udb", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// user data group
	{ "","udc", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// user data group
	{ "","udd", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// user data group

#ifdef __DIAGNOSTIC_PARAMETERS
	{ "","_te",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// target axis endpoint group
	{ "","_tr",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// target axis runtime group
	{ "","_ts",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// target motor steps group
	{ "","_ps",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// position motor steps group
	{ "","_cs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// commanded motor steps group
	{ "","_es",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// encoder steps group
	{ "","_xs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// correction steps group
	{ "","_fe",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },	// following error group
#endif

	// Uber-group (groups of groups, for text-mode displays only)
	// *** Must agree with NV_COUNT_UBER_GROUPS below ****
	{ "", "m", _f0, 0, tx_print_nul, _do_motors, set_nul,(float *)&cs.null,0 },
	{ "", "q", _f0, 0, tx_print_nul, _do_axes,   set_nul,(float *)&cs.null,0 },
	{ "", "o", _f0, 0, tx_print_nul, _do_offsets,set_nul,(float *)&cs.null,0 },
	{ "", "$", _f0, 0, tx_print_nul, _do_all,    set_nul,(float *)&cs.null,0 }
};

/***** Make sure these defines line up with any changes in the above table *****/

#define NV_COUNT_UBER_GROUPS 	4 		// count of uber-groups, above
#define STANDARD_GROUPS 		33		// count of standard groups, excluding diagnostic parameter groups

#if (MOTORS >= 5)
#define MOTOR_GROUP_5			1
#else
#define MOTOR_GROUP_5			0
#endif

#if (MOTORS >= 6)
#define MOTOR_GROUP_6			1
#else
#define MOTOR_GROUP_6			0
#endif

#ifdef __DIAGNOSTIC_PARAMETERS
#define DIAGNOSTIC_GROUPS 		8		// count of diagnostic groups only
#else
#define DIAGNOSTIC_GROUPS 		0
#endif
#define NV_COUNT_GROUPS 		(STANDARD_GROUPS + MOTOR_GROUP_5 + MOTOR_GROUP_6 + DIAGNOSTIC_GROUPS)

/* <DO NOT MESS WITH THESE DEFINES> */
#define NV_INDEX_MAX (sizeof cfgArray / sizeof(cfgItem_t))
#define NV_INDEX_END_SINGLES		(NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS - NV_STATUS_REPORT_LEN)
#define NV_INDEX_START_GROUPS		(NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS)
#define NV_INDEX_START_UBER_GROUPS (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS)
/* </DO NOT MESS WITH THESE DEFINES> */

index_t	nv_index_max() { return ( NV_INDEX_MAX );}
uint8_t nv_index_is_single(index_t index) { return ((index <= NV_INDEX_END_SINGLES) ? true : false);}
uint8_t nv_index_is_group(index_t index) { return (((index >= NV_INDEX_START_GROUPS) && (index < NV_INDEX_START_UBER_GROUPS)) ? true : false);}
uint8_t nv_index_lt_groups(index_t index) { return ((index <= NV_INDEX_START_GROUPS) ? true : false);}

/***** APPLICATION SPECIFIC CONFIGS AND EXTENSIONS TO GENERIC FUNCTIONS *****/

/*
 * set_flu() - set floating point number with G20/G21 units conversion
 *
 * The number 'setted' will have been delivered in external units (inches or mm).
 * It is written to the target memory location in internal canonical units (mm).
 * The original nv->value is also changed so persistence works correctly.
 * Displays should convert back from internal canonical form to external form.
 */

stat_t set_flu(nvObj_t *nv)
{
	if (cm_get_units_mode(MODEL) == INCHES) {		// if in inches...
		nv->value *= MM_PER_INCH;					// convert to canonical millimeter units
	}
	*((float *)GET_TABLE_WORD(target)) = nv->value;	// write value as millimeters or degrees
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return(STAT_OK);
}

/*
 * preprocess_float() - pre-process floating point number for units display
 */

void preprocess_float(nvObj_t *nv)
{
	if (isnan((double)nv->value) || isinf((double)nv->value)) return; // illegal float values
	if (GET_TABLE_BYTE(flags) & F_CONVERT) {		// unit conversion required?
		if (cm_get_units_mode(MODEL) == INCHES) {
			nv->value *= INCHES_PER_MM;
		}
	}
}

/**** TinyG UberGroup Operations ****************************************************
 * Uber groups are groups of groups organized for convenience:
 *	- motors		- group of all motor groups
 *	- axes			- group of all axis groups
 *	- offsets		- group of all offsets and stored positions
 *	- all			- group of all groups
 *
 * _do_group_list()	- get and print all groups in the list (iteration)
 * _do_motors()		- get and print motor uber group 1-N
 * _do_axes()		- get and print axis uber group XYZABC
 * _do_offsets()	- get and print offset uber group G54-G59, G28, G30, G92
 * _do_all()		- get and print all groups uber group
 */

static stat_t _do_group_list(nvObj_t *nv, char list[][TOKEN_LEN+1]) // helper to print multiple groups in a list
{
	for (uint8_t i=0; i < NV_MAX_OBJECTS; i++) {
		if (list[i][0] == NUL)
            return (STAT_COMPLETE);

		nv_reset_nv_list();
		nv = nv_body;
		strncpy(nv->token, list[i], TOKEN_LEN);
		nv->index = nv_get_index((const char_t *)"", nv->token);
//		nv->valuetype = TYPE_PARENT;
		nv_get_nvObj(nv);
		nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
	}
	return (STAT_COMPLETE);
}

static stat_t _do_motors(nvObj_t *nv)	// print parameters for all motor groups
{
#if MOTORS == 2
	char list[][TOKEN_LEN+1] = {"1","2",""}; // must have a terminating element
#endif
#if MOTORS == 3
	char list[][TOKEN_LEN+1] = {"1","2","3",""}; // must have a terminating element
#endif
#if MOTORS == 4
	char list[][TOKEN_LEN+1] = {"1","2","3","4",""}; // must have a terminating element
#endif
#if MOTORS == 5
	char list[][TOKEN_LEN+1] = {"1","2","3","4","5",""}; // must have a terminating element
#endif
#if MOTORS == 6
	char list[][TOKEN_LEN+1] = {"1","2","3","4","5","6",""}; // must have a terminating element
#endif
	return (_do_group_list(nv, list));
}

static stat_t _do_axes(nvObj_t *nv)	// print parameters for all axis groups
{
	char list[][TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
	return (_do_group_list(nv, list));
}

static stat_t _do_offsets(nvObj_t *nv)	// print offset parameters for G54-G59,G92, G28, G30
{
	char list[][TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92","g28","g30",""}; // must have a terminating element
	return (_do_group_list(nv, list));
}

static stat_t _do_all(nvObj_t *nv)	// print all parameters
{
	strcpy(nv->token,"sys");			// print system group
	get_grp(nv);
	nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	_do_motors(nv);					// print all motor groups
	_do_axes(nv);						// print all axis groups

	strcpy(nv->token,"p1");			// print PWM group
	get_grp(nv);
	nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);

	return (_do_offsets(nv));			// print all offsets
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * Most of these can be found in their respective modules.
 ***********************************************************************************/

/**** COMMUNICATIONS FUNCTIONS ******************************************************
 * set_ec() - enable CRLF on TX
 * set_ee() - enable character echo
 * set_ex() - enable XON/XOFF or RTS/CTS flow control
 * set_baud() - set USB baud rate
 * get_rx()	- get bytes available in RX buffer
 *
 *	The above assume USB is the std device
 */

static stat_t _set_comm_helper(nvObj_t *nv, uint32_t yes, uint32_t no)
{
	if (fp_NOT_ZERO(nv->value)) {
		(void)xio_ctrl(XIO_DEV_USB, yes);
	} else {
		(void)xio_ctrl(XIO_DEV_USB, no);
	}
	return (STAT_OK);
}

static stat_t set_ec(nvObj_t *nv) 				// expand CR to CRLF on TX
{
	if (nv->value > true)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	cfg.enable_cr = (uint8_t)nv->value;
	return(_set_comm_helper(nv, XIO_CRLF, XIO_NOCRLF));
}

static stat_t set_ee(nvObj_t *nv) 				// enable character echo
{
	if (nv->value > true)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	cfg.enable_echo = (uint8_t)nv->value;
	return(_set_comm_helper(nv, XIO_ECHO, XIO_NOECHO));
}

static stat_t set_ex(nvObj_t *nv)				// enable XON/XOFF or RTS/CTS flow control
{
	if (nv->value > FLOW_CONTROL_RTS)
        return (STAT_INPUT_VALUE_RANGE_ERROR);
	cfg.enable_flow_control = (uint8_t)nv->value;
	return(_set_comm_helper(nv, XIO_XOFF, XIO_NOXOFF));
}

static stat_t get_rx(nvObj_t *nv)
{
#ifdef __AVR
	nv->value = (float)xio_get_usb_rx_free();
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
#endif
#ifdef __ARM
	nv->value = (float)254;				// ARM always says the serial buffer is available (max)
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
#endif
}

/* run_sx()	- send XOFF, XON --- test only
static stat_t run_sx(nvObj_t *nv)
{
	xio_putc(XIO_DEV_USB, XOFF);
	xio_putc(XIO_DEV_USB, XON);
	return (STAT_OK);
}
*/

/*
 * set_baud() - set USB baud rate
 *
 *	See xio_usart.h for valid values. Works as a callback.
 *	The initial routine changes the baud config setting and sets a flag
 *	Then it posts a user message indicating the new baud rate
 *	Then it waits for the TX buffer to empty (so the message is sent)
 *	Then it performs the callback to apply the new baud rate
 */
static const char msg_baud0[] PROGMEM = "0";
static const char msg_baud1[] PROGMEM = "9600";
static const char msg_baud2[] PROGMEM = "19200";
static const char msg_baud3[] PROGMEM = "38400";
static const char msg_baud4[] PROGMEM = "57600";
static const char msg_baud5[] PROGMEM = "115200";
static const char msg_baud6[] PROGMEM = "230400";
static const char *const msg_baud[] PROGMEM = { msg_baud0, msg_baud1, msg_baud2, msg_baud3, msg_baud4, msg_baud5, msg_baud6 };

static stat_t set_baud(nvObj_t *nv)
{
	uint8_t baud = (uint8_t)nv->value;
	if ((baud < 1) || (baud > 6)) {
		nv_add_conditional_message((const char_t *)"*** WARNING *** Unsupported baud rate specified");
//		nv_add_conditional_message(PSTR("*** WARNING *** Unsupported baud rate specified"));
		return (STAT_INPUT_VALUE_RANGE_ERROR);
	}
	cfg.usb_baud_rate = baud;
	cfg.usb_baud_flag = true;
	char_t message[NV_MESSAGE_LEN];
	sprintf_P(message, PSTR("*** NOTICE *** Resetting baud rate to %s"),GET_TEXT_ITEM(msg_baud, baud));
	nv_add_conditional_message(message);
	return (STAT_OK);
}

stat_t set_baud_callback(void)
{
	if (cfg.usb_baud_flag == false)
        return(STAT_NOOP);
	cfg.usb_baud_flag = false;
	xio_set_baud(XIO_DEV_USB, cfg.usb_baud_rate);
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

//static const char fmt_ic[] PROGMEM = "[ic]  ignore CR or LF on RX%8d [0=off,1=CR,2=LF]\n";
static const char fmt_ec[] PROGMEM = "[ec]  expand LF to CRLF on TX%6d [0=off,1=on]\n";
static const char fmt_ee[] PROGMEM = "[ee]  enable echo%18d [0=off,1=on]\n";
static const char fmt_ex[] PROGMEM = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";
static const char fmt_baud[] PROGMEM = "[baud] USB baud rate%15d [1=9600,2=19200,3=38400,4=57600,5=115200,6=230400]\n";
static const char fmt_net[] PROGMEM = "[net] network mode%17d [0=master]\n";
static const char fmt_rx[] PROGMEM = "rx:%d\n";

void cfg_print_ec(nvObj_t *nv) { text_print_ui8(nv, fmt_ec);}
void cfg_print_ee(nvObj_t *nv) { text_print_ui8(nv, fmt_ee);}
void cfg_print_ex(nvObj_t *nv) { text_print_ui8(nv, fmt_ex);}
void cfg_print_baud(nvObj_t *nv) { text_print_ui8(nv, fmt_baud);}
void cfg_print_net(nvObj_t *nv) { text_print_ui8(nv, fmt_net);}
void cfg_print_rx(nvObj_t *nv) { text_print_ui8(nv, fmt_rx);}

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif
