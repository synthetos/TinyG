/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * See config.h for a description of CONFIG INTERNALS.
 *
 * Adding a new config setting:
 *	- add the setting to the global cfg struct in config.h (or wherever)
 *	- define a default value for it in settings.h
 *---> in the SETTING-SPECIFIC REGION in this file:
 *	- add a non-colliding mnemonic to cfgMnemonics & *mnemonics (see note)
 *  - add a static apply function and its prototype (_applyMN)
 *	- add a display format string (fmtMN)
 *	- add init line(s) to the large struct array (in display order)
 *
 * Note: Mnemonics are 2 char ASCII strings and can't start w\an axis name
 *		  - so these are off limits for 1st chars: X,Y,Z,A,B,C,U,V,W
 */

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "tinyg.h"
#include "settings.h"
#include "stepper.h"
#include "xio.h"
#include "gcode.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "controller.h"
#include "xmega_eeprom.h"

/*
 * Settings structure
 *
 * 	Settings are managed as an array of settings structs
 *	Functions are bound to the struct only if they are not common to 
 *	  all settings.
 *	The struct array is intialized, which is how it picks up the defaults
 */
#define CFG_VERSION 1			// NVM version
#define MNEMONIC_LEN 3			// mnemonic length +1 for termination
#define NVM_RECORD_LEN 6		// NVM record length (fixed length)

struct cfgSetting {				// don;t mess with the order of the struct
	int8_t axis;				// key: axis 0-N, or -1 if non-axis setting
	int8_t mnemonic;			// key: numeric token for mnemonic
	char * PROGMEM fmt_str;		// ptr to display format string (see note)
	void (*apply)(struct cfgSetting *);// function to apply setting
	double value;				// setting value - must be in-cast to double
};
//	Note: could provide a display function instead of a fmt string, like:
//	void (*disp)(struct cfgSetting *);
// 	You could also save 2 bytes per struct by using a single *util 
//	function instead of having separate *apply and *disp functions
//	void (*util)(struct cfgSetting *, uint8_t func);

struct cfgSingleton {			// persistent working variables
	uint8_t status;				// return status
	uint8_t profile;			// active profile
	uint16_t nvm_base_addr;		// NVM base address
	uint16_t nvm_profile_base;	// NVM base address of current profile
	uint16_t list_idx;			// array index of current struct
	char mnem_str[MNEMONIC_LEN];// working storage for mnemonic
	struct cfgSetting s;		// static working struct
};
static struct cfgSingleton cs;

//--- STATIC FUNCTION PROTOTYPES -----------------------------------------

static uint8_t _cfg_normalize_config_block(char *block);
static uint8_t _cfg_parse_config_block(char *block, struct cfgSetting *s);
static void _cfg_show_setting(struct cfgSetting *s);
static uint8_t _cfg_print_settings(char c);
static int8_t _cfg_get_axis_num(char c);
static int8_t _cfg_get_mnemonic_num(const char *mnemonic);
static char *_cfg_get_mnemonic_string(const int8_t mnum);
static char _cfg_get_axis_char(int8_t axis);
static double _cfg_compute_steps_per_unit(double step_angle, 
										  double travel_rev, 
								  		  uint8_t microsteps);
static double _cfg_compute_rate(double step_angle, double travel_rev,
								uint16_t steps_per_sec,
								double radius, uint8_t axis);
static uint16_t _cfg_compute_steps(double step_angle, double travel_rev,
								double rate, double radius, uint8_t axis);

static int16_t _cfg_get_setting_index(const int8_t axis, const int8_t mnemonic);
static struct cfgSetting *_cfg_get_setting_by_key(const int8_t axis, const int8_t mnemonic);
static struct cfgSetting *_cfg_get_setting_by_index(uint16_t index);
static struct cfgSetting *_cfg_put_setting(const struct cfgSetting *s);
static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t axis, const int8_t mnemonic);
static struct cfgSetting *_cfg_get_NVM_setting_by_index(uint16_t index);
static uint8_t _cfg_put_NVM_setting(const struct cfgSetting *s);


/************************************************************************
 *** START SETTING-SPECIFIC REGION **************************************
 ************************************************************************/

//--- MNEMONICS ----------------------------------------------------------
// See DISPLAY FORMAT STRINGS for explanations of what these are.

enum cfgMnemonics { __ = 0,		// null record. Must be first
	P_,							// header record. Must be #1
	MA, MO, SR, FR, SA, SS,	 	// per-axis settings
	FS, MI, PO, PW, LI, TR, 	// per-axis settings 
	TM, RA, 					// per axis settings
	HE, HR, HC, HO, HB, 		// per-axis settings (homing vars)
	GL, GU, GP, GT, GF, GS, 	// non-axis; gcode settings
	MM, MT, JM, JU, JL, HM,		// non-axis; general settings
	EA, 
	_P							// trailer record. Must be last
};
#define MAX_MNEMONIC _P

// These 2 arrays must stay in alignment
char *mnemonics[] = { "__", 	// array of mnemonic strings
	"P_", 
	"MA", "MO", "SR", "FR", "SA", "SS", 
	"FS", "MI", "PO", "PW", "LI", "TR", 
	"TM", "RA", 
	"HE", "HR", "HC", "HO", "HB",
	"GL", "GU", "GP", "GT", "GF", "GS",
	"MM", "MT", "JM", "JU", "JL", "HM",
	"EA", 
	"_P" 
};	

//--- APPLY FUNCTION PROTOTYPES --------------------------------------

static void _applyP_(struct cfgSetting *s);	// profile start

static void _applyMA(struct cfgSetting *s);
static void _applyMO(struct cfgSetting *s);
static void _applySR(struct cfgSetting *s);
static void _applyFR(struct cfgSetting *s);
static void _applySA(struct cfgSetting *s);
static void _applySS(struct cfgSetting *s);
static void _applyFS(struct cfgSetting *s);
static void _applyMI(struct cfgSetting *s);
static void _applyPO(struct cfgSetting *s);
static void _applyPW(struct cfgSetting *s);
static void _applyLI(struct cfgSetting *s);
static void _applyTR(struct cfgSetting *s);
static void _applyTM(struct cfgSetting *s);
static void _applyRA(struct cfgSetting *s);
static void _applyHE(struct cfgSetting *s);
static void _applyHR(struct cfgSetting *s);
static void _applyHC(struct cfgSetting *s);
static void _applyHO(struct cfgSetting *s);
static void _applyHB(struct cfgSetting *s);

static void _applyGL(struct cfgSetting *s);
static void _applyGU(struct cfgSetting *s);
static void _applyGP(struct cfgSetting *s);
static void _applyGT(struct cfgSetting *s);
static void _applyGF(struct cfgSetting *s);
static void _applyGS(struct cfgSetting *s);

static void _applyMM(struct cfgSetting *s);
static void _applyMT(struct cfgSetting *s);
static void _applyJM(struct cfgSetting *s);
static void _applyJU(struct cfgSetting *s);
static void _applyJL(struct cfgSetting *s);
static void _applyHM(struct cfgSetting *s);
static void _applyEA(struct cfgSetting *s);

//--- DISPLAY FORMAT STRINGS ---------------------------------------------

char fmtP_[] PROGMEM = "Version.Profile    %3.2f   $%s%1.2f\n";		// print version.profile
char fmtMA[] PROGMEM = "Map axis to motor  %5.0f   $%c%s%1.0f\n";	// map axis to motor 0 through 3
char fmtMO[] PROGMEM = "Axis mode          %5.0f   $%c%s%1.0f\n";	// 0=normal, 1=slaved (depends on axis)
char fmtSR[] PROGMEM = "Seek rate          %5.0f   $%c%s%1.2f\n";	// max axis seek rate in units per minute
char fmtFR[] PROGMEM = "Feed rate          %5.0f   $%c%s%1.2f\n";	// max axis feed rate in units per minute
char fmtSA[] PROGMEM = "Step angle (deg)   %5.3f   $%c%s%1.0f\n";	// in degrees per step 
char fmtSS[] PROGMEM = "Seek steps / sec   %5.0f   $%c%s%1.0f\n";	// max motor steps in whole steps per second
char fmtFS[] PROGMEM = "Feed steps / sec   %5.0f   $%c%s%1.0f\n";	// max motor steps in whole steps per second
char fmtMI[] PROGMEM = "Microstep mode     %5.0f   $%c%s%1.0f\n";	// [1248]; [0] may be morphing (future)
char fmtPO[] PROGMEM = "Motor polarity     %5.0f   $%c%s%1.0f\n";	// 0=normal, 1=inverted
char fmtPW[] PROGMEM = "Power mgmt mode    %5.0f   $%c%s%1.0f\n";	// 1=zero power idle
char fmtLI[] PROGMEM = "Limit switch mode  %5.0f   $%c%s%1.0f\n";	// 0=off, 1=on
char fmtTR[] PROGMEM = "Travel / rev      %6.2f   $%c%s%1.0f\n";	// in mm per revolution
char fmtTM[] PROGMEM = "Travel maximum     %5.0f   $%c%s%1.0f\n";	// maximum dimension on this axis
char fmtRA[] PROGMEM = "Axis radius        %5.3f   $%c%s%1.0f\n";	// for feed rate computation (valid for A,B and C)
char fmtHE[] PROGMEM = "Homing enabled     %5.0f   $%c%s%1.0f\n";	// 1=enable homing for this axis
char fmtHR[] PROGMEM = "Homing seek rate   %5.0f   $%c%s%1.0f\n";	// first pass homing speed
char fmtHC[] PROGMEM = "Homing close rate  %5.0f   $%c%s%1.0f\n";	// second pass homing speed
char fmtHO[] PROGMEM = "Homing offset      %5.0f   $%c%s%1.0f\n";	// offset for min limit switch
char fmtHB[] PROGMEM = "Homing backoff     %5.0f   $%c%s%1.0f\n";	// homing backoff distance

char fmtGL[] PROGMEM = "Gcode: {G17/G18/G19}    Plane   %1.0f   $%s%1.0f\n";
char fmtGU[] PROGMEM = "Gcode: {G20/G21} Units (21=mm)  %1.0f   $%s%1.0f\n";
char fmtGP[] PROGMEM = "Gcode: {G61/G61.1/G64} Path     %1.0f   $%s%1.0f\n";
char fmtGT[] PROGMEM = "Gcode: {T} Tool                 %1.0f   $%s%1.0f\n";
char fmtGF[] PROGMEM = "Gcode: {F} Feed rate     %8.2f   $%s%1.0f\n";
char fmtGS[] PROGMEM = "Gcode: {S} Spindle speed %8.2f   $%s%1.0f\n";

char fmtMM[] PROGMEM = "Minimum segment len (mm)    %4.3f   $%s%1.0f\n";
char fmtMT[] PROGMEM = "Minimum segment time (uS)   %5.0f   $%s%1.0f\n";
char fmtJM[] PROGMEM = "Max linear jerk        %10.0f   $%s%1.0f\n";
char fmtJU[] PROGMEM = "Angular jerk upper thresh   %4.3f   $%s%1.0f\n";
char fmtJL[] PROGMEM = "Angular jerk lower thresh   %4.3f   $%s%1.0f\n";
char fmtHM[] PROGMEM = "Homing mode (1=power-on)        %1.0f   $%s%f\n";
char fmtEA[] PROGMEM = "Enable Acceleration             %1.0f   $%s%f\n";

//##### CONFIRM OR ADJUST THESE COUNTS IF YOU CHANGE THE ABOVE #####
#define COUNT_AXES AXES		// number of supported axes in this version
#define COUNT_PER_AXIS_SETTINGS 19 
#define COUNT_GCODE_SETTINGS 6		// non-axis Gcode settings
#define COUNT_GLOBAL_SETTINGS 7		// non-axis global settings
#define COUNT_HEADERS 2				// non-axis header and trailer records
#define COUNT_NON_AXIS (COUNT_GCODE_SETTINGS + COUNT_GLOBAL_SETTINGS + COUNT_HEADERS)
#define COUNT_SETTINGS ((COUNT_PER_AXIS_SETTINGS * COUNT_AXES) + COUNT_NON_AXIS)

/*---- SETTING LIST STRUCTURE --------------------------------------------
 *
 *	Initialize all settings structs and defaults
 *
 *	int8_t axis;				// key: axis 0-N, or -1 if non-axis setting
 *	int8_t mnemonic;			// key: numeric token for mnemonic
 *	char * PROGMEM fmt_str;		// ptr to display format string (see note)
 *	void (*apply)(struct cfgSetting *);// function to apply setting
 *	double value;				// setting value - must be in-cast to double
 */

struct cfgSetting cfgList[COUNT_SETTINGS] = { 
	// starting version.profile record - must be first
	{-1, P_, fmtP_, _applyP_, (double) CFG_VERSION },	// don;t mess

	{ X, MA, fmtMA, _applyMA, (double) X_AXIS_MAP },
	{ X, MO, fmtMO, _applyMO, (double) X_AXIS_MODE },
	{ X, SR, fmtSR, _applySR, (double) 0 },
	{ X, FR, fmtFR, _applyFR, (double) 0 },
	{ X, SA, fmtSA, _applySA, (double) X_STEP_ANGLE },
	{ X, SS, fmtSS, _applySS, (double) X_SEEK_STEPS },
	{ X, FS, fmtFS, _applyFS, (double) X_FEED_STEPS },
	{ X, MI, fmtMI, _applyMI, (double) X_MICROSTEPS },
	{ X, PO, fmtPO, _applyPO, (double) X_POLARITY },
	{ X, PW, fmtPW, _applyPW, (double) X_POWER_MODE },
	{ X, LI, fmtLI, _applyLI, (double) X_LIMIT_MODE },
	{ X, TR, fmtTR, _applyTR, (double) X_TRAVEL_PER_REV },
	{ X, TM, fmtTM, _applyTM, (double) X_TRAVEL_MAX },
	{ X, RA, fmtRA, _applyRA, (double) 0 },
	{ X, HE, fmtHE, _applyHE, (double) X_HOMING_ENABLE },
	{ X, HR, fmtHR, _applyHR, (double) X_HOMING_SEEK_RATE },
	{ X, HC, fmtHC, _applyHC, (double) X_HOMING_CLOSE_RATE },
	{ X, HO, fmtHO, _applyHO, (double) X_HOMING_OFFSET },
	{ X, HB, fmtHB, _applyHB, (double) X_HOMING_BACKOFF },

	{ Y, MA, fmtMA, _applyMA, (double) Y_AXIS_MAP },
	{ Y, MO, fmtMO, _applyMO, (double) Y_AXIS_MODE },
	{ Y, SR, fmtSR, _applySR, (double) 0 },
	{ Y, FR, fmtFR, _applyFR, (double) 0 },
	{ Y, SA, fmtSA, _applySA, (double) Y_STEP_ANGLE },
	{ Y, SS, fmtSS, _applySS, (double) Y_SEEK_STEPS },
	{ Y, FS, fmtFS, _applyFS, (double) Y_FEED_STEPS },
	{ Y, MI, fmtMI, _applyMI, (double) Y_MICROSTEPS },
	{ Y, PO, fmtPO, _applyPO, (double) Y_POLARITY },
	{ Y, PW, fmtPW, _applyPW, (double) Y_POWER_MODE },
	{ Y, LI, fmtLI, _applyLI, (double) Y_LIMIT_MODE },
	{ Y, TR, fmtTR, _applyTR, (double) Y_TRAVEL_PER_REV },
	{ Y, TM, fmtTM, _applyTM, (double) Y_TRAVEL_MAX },
	{ Y, RA, fmtRA, _applyRA, (double) 0 },
	{ Y, HE, fmtHE, _applyHE, (double) Y_HOMING_ENABLE },
	{ Y, HR, fmtHR, _applyHR, (double) Y_HOMING_SEEK_RATE },
	{ Y, HC, fmtHC, _applyHC, (double) Y_HOMING_CLOSE_RATE },
	{ Y, HO, fmtHO, _applyHO, (double) Y_HOMING_OFFSET },
	{ Y, HB, fmtHB, _applyHB, (double) Y_HOMING_BACKOFF },

	{ Z, MA, fmtMA, _applyMA, (double) Z_AXIS_MAP },
	{ Z, MO, fmtMO, _applyMO, (double) Z_AXIS_MODE },
	{ Z, SR, fmtSR, _applySR, (double) 0 },
	{ Z, FR, fmtFR, _applyFR, (double) 0 },
	{ Z, SA, fmtSA, _applySA, (double) Z_STEP_ANGLE },
	{ Z, SS, fmtSS, _applySS, (double) Z_SEEK_STEPS },
	{ Z, FS, fmtFS, _applyFS, (double) Z_FEED_STEPS },
	{ Z, MI, fmtMI, _applyMI, (double) Z_MICROSTEPS },
	{ Z, PO, fmtPO, _applyPO, (double) Z_POLARITY },
	{ Z, PW, fmtPW, _applyPW, (double) Z_POWER_MODE },
	{ Z, LI, fmtLI, _applyLI, (double) Z_LIMIT_MODE },
	{ Z, TR, fmtTR, _applyTR, (double) Z_TRAVEL_PER_REV },
	{ Z, TM, fmtTM, _applyTM, (double) Z_TRAVEL_MAX },
	{ Z, RA, fmtRA, _applyRA, (double) 0 },
	{ Z, HE, fmtHE, _applyHE, (double) Z_HOMING_ENABLE },
	{ Z, HR, fmtHR, _applyHR, (double) Z_HOMING_SEEK_RATE },
	{ Z, HC, fmtHC, _applyHC, (double) Z_HOMING_CLOSE_RATE },
	{ Z, HO, fmtHO, _applyHO, (double) Z_HOMING_OFFSET },
	{ Z, HB, fmtHB, _applyHB, (double) Z_HOMING_BACKOFF },

	{ A, MA, fmtMA, _applyMA, (double) A_AXIS_MAP },
	{ A, MO, fmtMO, _applyMO, (double) A_AXIS_MODE },
	{ A, SR, fmtSR, _applySR, (double) 0 },
	{ A, FR, fmtFR, _applyFR, (double) 0 },
	{ A, SA, fmtSA, _applySA, (double) A_STEP_ANGLE },
	{ A, SS, fmtSS, _applySS, (double) A_SEEK_STEPS },
	{ A, FS, fmtFS, _applyFS, (double) A_FEED_STEPS },
	{ A, MI, fmtMI, _applyMI, (double) A_MICROSTEPS },
	{ A, PO, fmtPO, _applyPO, (double) A_POLARITY },
	{ A, PW, fmtPW, _applyPW, (double) A_POWER_MODE },
	{ A, LI, fmtLI, _applyLI, (double) A_LIMIT_MODE },
	{ A, TR, fmtTR, _applyTR, (double) A_TRAVEL_PER_REV },
	{ A, TM, fmtTM, _applyTM, (double) A_TRAVEL_MAX },
	{ A, RA, fmtRA, _applyRA, (double) A_RADIUS },
	{ A, HE, fmtHE, _applyHE, (double) A_HOMING_ENABLE },
	{ A, HR, fmtHR, _applyHR, (double) A_HOMING_SEEK_RATE },
	{ A, HC, fmtHC, _applyHC, (double) A_HOMING_CLOSE_RATE },
	{ A, HO, fmtHO, _applyHO, (double) A_HOMING_OFFSET },
	{ A, HB, fmtHB, _applyHB, (double) A_HOMING_BACKOFF },

	{ NON_AXIS, GL, fmtGL, _applyGL, (double) GCODE_PLANE },
	{ NON_AXIS, GU, fmtGU, _applyGU, (double) GCODE_UNITS },
	{ NON_AXIS, GP, fmtGP, _applyGP, (double) GCODE_PATH_CONTROL },
	{ NON_AXIS, GT, fmtGT, _applyGT, (double) GCODE_TOOL },
	{ NON_AXIS, GF, fmtGF, _applyGF, (double) GCODE_FEED_RATE },
	{ NON_AXIS, GS, fmtGS, _applyGS, (double) GCODE_SPINDLE_SPEED },

	{ NON_AXIS, MM, fmtMM, _applyMM, (double) MIN_SEGMENT_LENGTH },
	{ NON_AXIS, MT, fmtMT, _applyMT, (double) MIN_SEGMENT_TIME },
	{ NON_AXIS, JM, fmtJM, _applyJM, (double) MAX_LINEAR_JERK },
	{ NON_AXIS, JU, fmtJU, _applyJU, (double) ANGULAR_JERK_UPPER_THRESHOLD },
	{ NON_AXIS, JL, fmtJL, _applyJL, (double) ANGULAR_JERK_LOWER_THRESHOLD },
	{ NON_AXIS, HM, fmtHM, _applyHM, (double) HOMING_MODE },
	{ NON_AXIS, EA, fmtEA, _applyEA, (double) ENABLE_ACCEL },

	// ending version.profile record - must be last
	{-1, _P, fmtP_, _applyP_, (double) CFG_VERSION }	// don;t mess
};

// some shorthands useful for the functions
#define _S cfgSetting
#define CFGS cfg.a[s->axis]		// further shorthand for "apply" functions

/*---- APPLY FUNCTIONS -------------------------------------------------*/
// Note: standard formatting conventions violated for density & readability
static void _applyP_(struct _S *s) { return; }	// header & trailer (null)

// PER AXIS APPLY FUNCTIONS
static void _applyMA(struct _S *s) { CFGS.map_axis = (uint8_t)s->value; }
static void _applyMO(struct _S *s) { CFGS.axis_mode = (uint8_t)s->value; }

static void _applySR(struct _S *s) { CFGS.max_seek_rate = s->value;
	CFGS.seek_steps = _cfg_compute_steps(CFGS.step_angle, CFGS.travel_rev,
									CFGS.max_seek_rate, CFGS.radius, s->axis);
}
static void _applyFR(struct _S *s) { CFGS.max_feed_rate = s->value;
	CFGS.feed_steps = _cfg_compute_steps(CFGS.step_angle, CFGS.travel_rev,
									CFGS.max_feed_rate, CFGS.radius, s->axis);
}
static void _applySA(struct _S *s) { CFGS.step_angle = s->value;
	CFGS.steps_per_unit = _cfg_compute_steps_per_unit(CFGS.step_angle, 
									CFGS.travel_rev, CFGS.microsteps);
	CFGS.max_seek_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.seek_steps, CFGS.radius, s->axis);
	CFGS.max_feed_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.feed_steps, CFGS.radius, s->axis); 
}
static void _applySS(struct _S *s) { CFGS.seek_steps = (uint16_t)s->value;
	CFGS.max_seek_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.seek_steps, CFGS.radius, s->axis);
}
static void _applyFS(struct _S *s) { CFGS.feed_steps = (uint16_t)s->value;
	CFGS.max_feed_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.feed_steps, CFGS.radius, s->axis);
}
static void _applyMI(struct _S *s) { CFGS.microsteps = (uint8_t)s->value;
	st_set_microsteps(s->axis, CFGS.microsteps);
	CFGS.steps_per_unit = _cfg_compute_steps_per_unit(CFGS.step_angle, 
									CFGS.travel_rev, CFGS.microsteps);
}
static void _applyPO(struct _S *s) { CFGS.polarity = (uint8_t)s->value;
	st_set_polarity(s->axis, CFGS.polarity);
}
static void _applyPW(struct _S *s) { CFGS.power_mode = (uint8_t)s->value;}
static void _applyLI(struct _S *s) { CFGS.limit_mode = (uint8_t)s->value;}
static void _applyTR(struct _S *s) { CFGS.travel_rev = s->value;
	CFGS.steps_per_unit = _cfg_compute_steps_per_unit(CFGS.step_angle, 
									CFGS.travel_rev, CFGS.microsteps);
	CFGS.max_seek_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.seek_steps, CFGS.radius, s->axis);
	CFGS.max_feed_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.feed_steps, CFGS.radius, s->axis);
}
static void _applyTM(struct _S *s) { CFGS.travel_max = s->value; }
static void _applyHE(struct _S *s) { CFGS.homing_enable = (uint8_t)s->value;}
static void _applyHR(struct _S *s) { CFGS.homing_rate = s->value; }
static void _applyHC(struct _S *s) { CFGS.homing_close = s->value; }
static void _applyHO(struct _S *s) { CFGS.homing_offset = s->value; }
static void _applyHB(struct _S *s) { CFGS.homing_backoff = s->value; }
static void _applyRA(struct _S *s) { CFGS.radius = (double)s->value;
	CFGS.max_seek_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.seek_steps, CFGS.radius, s->axis);
	CFGS.max_feed_rate = _cfg_compute_rate(CFGS.step_angle,CFGS.travel_rev,
									CFGS.feed_steps, CFGS.radius, s->axis);
}
// GCODE DEFAULT APPLY FUNCTIONS
static void _applyGL(struct _S *s) {// apply in either Gcode or enum form
	switch ((int)s->value) {
		case 17: cfg.gcode_plane = CANON_PLANE_XY; break;
		case 18: cfg.gcode_plane = CANON_PLANE_XZ; break;
		case 19: cfg.gcode_plane = CANON_PLANE_YZ; break;
		default: cfg.gcode_plane = (uint8_t)s->value;
	}
}
static void _applyGU(struct _S *s) {// apply in either Gcode or enum form
	switch ((int)s->value) {
		case 20: cfg.gcode_units = UNITS_INCHES; break;
		case 21: cfg.gcode_units = UNITS_MM; break;
		default: cfg.gcode_units = (uint8_t)s->value;
	}
//	cm_use_length_units((~cfg.gcode_units) & 0x01);	// must invert 0/1 sense
	cm_use_length_units(cfg.gcode_units ^ 0x01);	// invert 0/1 sense
}
static void _applyGP(struct _S *s) {// apply in either Gcode or enum form
	switch ((int)s->value*10) {
		case 610: cfg.gcode_path_control = PATH_EXACT_STOP; break;
		case 611: cfg.gcode_path_control = PATH_EXACT_PATH; break;
		case 640: cfg.gcode_path_control = PATH_CONTINUOUS; break;
		default: cfg.gcode_path_control = (uint8_t)s->value;
	}
}
static void _applyGT(struct _S *s) { cfg.gcode_tool = (uint8_t)s->value; }
static void _applyGF(struct _S *s) { cfg.gcode_feed_rate = s->value; }
static void _applyGS(struct _S *s) { cfg.gcode_spindle_speed = s->value; }

// NON-AXIS APPLY FUNCTIONS
static void _applyMM(struct _S *s) { cfg.min_segment_len = s->value; }
static void _applyMT(struct _S *s) { cfg.min_segment_time = s->value; }
static void _applyJM(struct _S *s) { cfg.max_linear_jerk = s->value; }
static void _applyJU(struct _S *s) { cfg.angular_jerk_upper = s->value; }
static void _applyJL(struct _S *s) { cfg.angular_jerk_lower = s->value; }
static void _applyHM(struct _S *s) { cfg.homing_mode = (uint8_t)s->value;}
static void _applyEA(struct _S *s) { cfg.accel_enabled = (uint8_t)s->value;}

//--- APPLY FUNCTION HELPERS -------------------------------

/*
 * _cfg_compute_travel() - compute mm of travel per microstep
 *
 *	This function will need to be rethought when microstep morphing is 
 *	implemented, as microsteps are calculated statically. Dang.
 */
static double _cfg_compute_steps_per_unit(double step_angle, double travel_rev,
								 		  uint8_t microsteps)
{
	return (360 / (step_angle / microsteps) / travel_rev);
}

/*
 * _cfg_compute_rate() - compute linear or rotary axis rates
 *
 *	Compute seek or feed rate by providing seek_steps or feed_steps:
 *
 *		rate = steps_per_sec * 60 / (360 / step_angle / travel_rev)
 *
 *	Rotary axis seek_rate and feed_rates are computed in mm/min
 *	by multiplying degrees/min by the axis radius value / one radian. 
 *	If you actually want rate in degrees / min set radius to one radian.
 *	Radius is ignored for linear axes,
 */

static double _cfg_compute_rate(double step_angle, double travel_rev,
								uint16_t steps_per_sec, // whole steps
								double radius, uint8_t axis)
{
	if ((axis < A) || (axis > C)) {
		return ((double)steps_per_sec * 60 / (360/step_angle/travel_rev));
	} else {
		return ((double)steps_per_sec * 60 / (360/step_angle/travel_rev) * 
												(radius / RADIAN));
	}
}

/*
 * _cfg_compute_steps() - compute motor whole steps required for a seek or feed rate 
 *
 *		steps_per_sec = rate / (60 / 360 / step_angle / travel_rev)
 *
 */

static uint16_t _cfg_compute_steps(double step_angle, double travel_rev,
								   double rate, double radius, uint8_t axis)
{
	if ((axis < A) || (axis > C)) {
		return (uint16_t)(rate / (60/360/step_angle/travel_rev));
	} else {
		return (uint16_t)(rate / (60/360/step_angle/travel_rev) / 
									(radius / RADIAN));
	}
}

/*************************************************************************
 *** END SETTING-SPECIFIC REGION *****************************************
 ************************************************************************/

//----- CORE CONFIG SYSTEM FUNCTIONS -------------------------------------

/*
 * cfg_init() - called once on system init
 *
 *	Will perform one of 3 actions:
 *	- if NVM is set up and current, load NVM into config ram
 *	- if NVM is not set up load it and RAM with hardwaired default settings
 *	- if NVM is out-of-date apply all old settings that are still 
 *	  		 applicable, then migrate new settings to NVM
 */

void cfg_init() 
{
	uint16_t i;
	struct cfgSetting *s;

	// initialize working variables
	cs.status = TG_OK;
	cs.profile = 0;					// first (and only) profile
	cs.nvm_base_addr = CFG_NVM_BASE;
	cs.nvm_profile_base = cs.nvm_base_addr;

#ifdef __DEBUG
	fprintf_P(fdev_usb, PSTR("....Initializing EEPROM settings\n"));
#endif
	// apply the hard-wired default values pre-loaded from settings.h
	for (i=0; i<COUNT_SETTINGS; i++) {
		cfgList[i].apply(&cfgList[i]);
	}
#ifdef __NO_EEPROM
	return;
#endif
#ifdef __DEBUG
	cfg_dump_NVM(0,10,PSTR("Initial NVM Contents"));
#endif

	// if NVM is not set up, persist default settings to NVM and exit
	s = _cfg_get_NVM_setting_by_key(NON_AXIS, P_);
	if (!((s->axis == NON_AXIS) && (s->mnemonic == P_))) {
		for (i=0; i<COUNT_SETTINGS; i++) {
			s = _cfg_get_setting_by_index(i);
			_cfg_put_NVM_setting(s);
		}
#ifdef __DEBUG
		cfg_dump_NVM(0,10,PSTR("After setup uninitialized NVM"));
#endif
		return;
	}

	// read settings from NVM into RAM - stop at trailer record or max
	for (i=0; i<COUNT_SETTINGS; i++) {
		if ((s = _cfg_get_NVM_setting_by_index(i)) == NULL) {
			break;
		}
		_cfg_put_setting(s);
	}

	// if NVM is out of revision, write settings back to NVM
	s = _cfg_get_NVM_setting_by_key(NON_AXIS, P_);
	if (s->value != CFG_VERSION) {
		for (i=0; i<COUNT_SETTINGS; i++) {
			s = _cfg_get_setting_by_index(i);
			_cfg_put_NVM_setting(s);
		}
	}
#ifdef __DEBUG	
	cfg_dump_NVM(0,10,PSTR("After init completed"));
#endif
}

/* 
 * cfg_config_parser()	- update a config setting from a text block
 *			  			- conditionally display the setting (if TRUE)
 *			 			- conditionally persist setting to NVM (if TRUE)
 */

uint8_t cfg_config_parser(char *block, uint8_t display, uint8_t persist)
{
	struct cfgSetting *s;		// pointer to setting in settings list

	/// '$$' = display ALL settings
	if ((block[0] == '$') && (block[1] == '$')) {
		return (_cfg_print_settings(block[0])); // based on 1st char in blk
	}
	// normalize the block in place - strip leading $ and any whitespace
	if ((_cfg_normalize_config_block(block)) <2) { // not a setting
		return (_cfg_print_settings(block[0]));
	}
	// parse the block into its basic parts
	ritorno(_cfg_parse_config_block(block, &cs.s)); 
	// update config value in corresponding parser structure (or die trying)
	if ((s = _cfg_put_setting(&cs.s)) == NULL) {
		return (cs.status);
	}
	if (display) {			// do conditional config display
		_cfg_show_setting(s);
	}
	if (persist) {			// do conditional persist
		_cfg_put_NVM_setting(s);
	}
	return (TG_OK);
}

/*
 * _cfg_normalize_config_block() - normalize a config block in place
 *
 *	Capitalize and pack all valid characters
 *	Remove all whitespace and invalid characters
 *	Strip comments (parens)
 *	Returns # of chars in the nornalized string, minus the NUL char
 *
 *	Valid characters (these are passed to config parser):
 *		digits						all digits are passed to parser
 *		lower case alpha			all alpha is passed - converted to upper
 *		upper case alpha			all alpha is passed
 *		- . ? 						sign, dot, question mark pass to parser
 *
 *	Invalid characters (these are stripped but don't cause failure):
 *		control characters			chars < 0x20 are all removed
 *		/ *	< = > | % #	+ _			expression chars removed from string
 *		( ) [ ] { } 				expression chars removed from string
 *		<sp> <tab> 					whitespace chars removed from string
 *		! % , ; ; @ 				removed
 *		^ ~ " ' <DEL>				removed
 *		$							removes leading $
 */

static uint8_t _cfg_normalize_config_block(char *block) 
{
	char c;
	uint8_t i=0; 		// index for incoming characters
	uint8_t j=0;		// index for normalized characters

	// normalize the block & prune comment(if any)
	while ((c = toupper(block[i++])) != 0) {// NUL character
		if ((isupper(c)) || (isdigit(c))) {	// capture common chars
		 	block[j++] = c; 
			continue;
		}
		if (strchr("-.?", c)) {				// catch valid non-alphanumerics
		 	block[j++] = c; 
			continue;
		}
		if (c == '(') {						// detect & handle comments
			j++;
			break;
		}
	}										// ignores any other characters
	block[j] = NUL;							// terminate block and end
	return (j);			
}

/*
 * _cfg_parse_config_block() - parse a normalized config block
 */

static uint8_t _cfg_parse_config_block(char *block, struct cfgSetting *s) 
{
	uint8_t i=0;			// block read index
	uint8_t j=0;			// mnemonic write index 

	// get the axis and set mnemonic starting point in block
	if ((s->axis = _cfg_get_axis_num(block[0])) != NON_AXIS) {
		i++;
	}
	// capture mnemonic
	while ((!isdigit(block[i])) && (j<MNEMONIC_LEN-1)) {
		cs.mnem_str[j++] = block[i++];
	}
	cs.mnem_str[j] = NUL;
	s->mnemonic = _cfg_get_mnemonic_num(cs.mnem_str);

	// capture value
	while (!isdigit(block[i])) {	// position to start of value string
		i++; 
	};
	if (!gc_read_double(block, &i, &s->value)) {
		return (TG_BAD_NUMBER_FORMAT);
	}
	return (TG_OK);
}

/*
 * _cfg_print_settings() - print settings based on the input char
 *
 * Perform display depending on what is typically in block[0]:
 *		"$"		- display all settings (from '$$' non-normalized buffer)
 *		"X"		- display axis settings (where X is the axis) 
 *		<NUL>	- display non-axis settings
 *		'E'		- dump EEPROM
 *		'H'		- display help screen
 */

static uint8_t _cfg_print_settings(char c)
{
	int8_t axis = _cfg_get_axis_num(c);

	if (c == 'H') {
		return (cfg_print_config_help());
	}
	if (c == 'E') {
		cfg_dump_NVM(0,50,PSTR("from command line"));
		return (TG_OK);
	}
	for (uint16_t i=0; i < COUNT_SETTINGS; i++) {	
		if ((c == '$') || (cfgList[i].axis == axis)) {
			_cfg_show_setting(&cfgList[i]);
		}
	}
	return (TG_OK);
}

/*
 * _cfg_show_setting() - show a single setting
 */

static void _cfg_show_setting(struct cfgSetting *s)
{
	// print the display string	
	if (s->axis != NON_AXIS) {
		printf_P(PSTR("%c axis - "), _cfg_get_axis_char(s->axis));
		printf_P((PGM_P)s->fmt_str, s->value, _cfg_get_axis_char(s->axis), 
					_cfg_get_mnemonic_string(s->mnemonic), s->value);
	} else {
		printf_P((PGM_P)s->fmt_str, s->value, 
					_cfg_get_mnemonic_string(s->mnemonic), s->value);
	}
}

/*
 * cfg_print_config_help() - config help screen
 */

uint8_t cfg_print_config_help(void)
{
	fprintf_P(stderr, PSTR("*** TinyG Configuration Help ***\n\
These commands are active for configuration:\n\
  $    Show general settings\n\
  $x   Show X axis settings (or whatever axis you want x,y,z,a...)\n\
  $$   Show all settings\n\
  $h   Show this help screen\n\n\
To update settings type in a token and a value:\n\n\
  $ <token> <value>\n\n\
For example $yfr800 to set the Y max feed rate to 800 mm/minute\n\
Input is very forgiving of caps, spaces and extra characters\n\n\
The value taken will be echoed back to the console\n\
Please log any issues at http://synthetos.com/forums\n\
Have fun\n"));

	return (TG_OK);
}

/*
 * _cfg_get_axis_num()  		- return axis number from axis char
 * _cfg_get_axis_char() 		- return axis char from axis number
 * _cfg_get_mnemonic_num()  	- return mnemonic number from string
 * _cfg_get_mnemonic_string() 	- return mnemonic string from number
 */

static int8_t _cfg_get_axis_num(char c)
{
	switch (c) {
		case 'X': return (X);
		case 'Y': return (Y);
		case 'Z': return (Z);
		case 'A': return (A);
		case 'B': return (B);
		case 'C': return (C);
		case 'U': return (U);
		case 'V': return (V);
		case 'W': return (W);
	}
	return (NON_AXIS);
}

static char _cfg_get_axis_char(int8_t axis)
{
	switch (axis) {
		case X: return 'X';
		case Y: return 'Y';
		case Z: return 'Z';
		case A: return 'A';
		case B: return 'B';
		case C: return 'C';
		case U: return 'U';
		case V: return 'V';
		case W: return 'W';
	}
	return (' ');
}

static int8_t _cfg_get_mnemonic_num(const char *mnemonic)
{
	for (uint8_t i=0; i<MAX_MNEMONIC; i++) {
		if (strcmp(mnemonics[i], mnemonic) == 0) {	// match
			return(i);
		}
	}
	return (-1);		// mnemonic not found
}

static char *_cfg_get_mnemonic_string(const int8_t mnum)
{
	if (mnum >= MAX_MNEMONIC) {
		return (NUL);
	} else {
		return (mnemonics[mnum]);
	}
}

//----- SETTINGS ACCESS PRIMITIVES ---------------------------------------
/*
 *	Access to settings records works like a really dumb, inefficient 
 *	key/value dictionary (ala Python). The key is a compound key 
 *	consisting of the axis + mnemonic.
 *
 *	There are 2 dictionaries: the in-memory structure array and the 
 *	non-volatile memory records. NVM records are binary versions of the 
 *	config text blocks that are re-hydrated on extraction so they can be 
 *	re-parsed to populate the config structs. They look like this:
 *
 *		<axis byte><mnemonic byte><value as a double>	(6 bytes total)
 *
 *	A profile in NVM starts and ends with profile records of this form:
 *
 *		<-1><P_><version#>.<profile#>		version.profile is a decimal #
 *		<-1><_P><version#>.<profile#>
 */

/*
 * _cfg_get_setting_index() - return index of setting or -1 if not found
 */

static int16_t _cfg_get_setting_index(const int8_t axis, const int8_t mnemonic)
{
	for (cs.list_idx=0; cs.list_idx<COUNT_SETTINGS; cs.list_idx++) {
		if (cfgList[cs.list_idx].axis != axis) {
			continue;
		}
		if (cfgList[cs.list_idx].mnemonic == mnemonic) {
			return(cs.list_idx);
		}
	}
	cs.status = TG_PARAMETER_NOT_FOUND;
	return(-1);		// key not found
}

/*
 * _cfg_get_setting_by_key() - return ptr to setting or NULL if not found
 */

static struct cfgSetting *_cfg_get_setting_by_key(const int8_t axis, const int8_t mnemonic)
{
	if (_cfg_get_setting_index(axis, mnemonic) == -1) {
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (NULL);
	}
	return(&cfgList[cs.list_idx]);
}

/*
 * _cfg_get_setting_by_index() - return ptr to next setting or NULL if end
 */

static struct cfgSetting *_cfg_get_setting_by_index(uint16_t index)
{
	cs.list_idx = index;
	if (index >= COUNT_SETTINGS) {
		cs.status = TG_PARAMETER_NOT_FOUND;
		return(NULL);
	}
	return(&cfgList[index]);
}

/*
 * _cfg_put_setting() - update a table setting from the setting passed in
 *					  - apply the new value by running the apply function
 *					  - returns ptr to updated setting in list (or NULL)
 */

static struct cfgSetting *_cfg_put_setting(const struct cfgSetting *s)
{
	struct cfgSetting *t;	// setting struct in table

	// locate setting matching the incoming struct
	// (optimization would be to check if the list_idx already
	//  matches the s->axis, s->mnemonic and skip the table scan)
	if ((t = _cfg_get_setting_by_key(s->axis, s->mnemonic)) == NULL) {
		cs.status = TG_PARAMETER_NOT_FOUND;
		return(NULL);
	}
	t->value = s->value;
	t->apply(t);
	return(t);
}

/*
 * _cfg_get_NVM_setting_by_key() - return S struct with value from NVM
 */
static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t axis, const int8_t mnemonic)
{
	// scan the settings table to find the list index
	if ((_cfg_get_setting_by_key(axis, mnemonic)) == NULL) {
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (NULL);
	}
	return (_cfg_get_NVM_setting_by_index(cs.list_idx));
}

/*
 * _cfg_get_NVM_setting_by_index() - return S struct by index into NVM
 */

static struct cfgSetting *_cfg_get_NVM_setting_by_index(const uint16_t index)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address = cs.nvm_profile_base + (index * NVM_RECORD_LEN);

	EEPROM_ReadBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
	cs.s.axis = nvm_record[0];
	cs.s.mnemonic = nvm_record[1];
	memcpy(&cs.s.value, &nvm_record[2], sizeof(double));
	if (cs.s.mnemonic > MAX_MNEMONIC) {
		cs.status = TG_PARAMETER_OVER_RANGE;
		return (NULL);
	}
	return (&cs.s);
}

/*
 * _cfg_put_NVM_setting() - write binary NVM record
 */

static uint8_t _cfg_put_NVM_setting(const struct cfgSetting *s)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address;

	if (_cfg_get_setting_index(s->axis, s->mnemonic) == -1) {
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (cs.status);
	}
	nvm_address = cs.nvm_profile_base + (cs.list_idx * NVM_RECORD_LEN);
	nvm_record[0] = s->axis;
	nvm_record[1] = s->mnemonic;
	memcpy(&nvm_record[2], &s->value, sizeof(double));
	EEPROM_WriteBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
	return(TG_OK);
}

/*
 * _cfg_dump_NVM() - dump NVM to stderr in 6 byte lines
 *
 *	Requires label as program memory string. Usage example
 *		cfg_dump_NVM(0,10,PSTR("Initial state"));
 */

void cfg_dump_NVM(uint16_t start_record, uint16_t end_record, char *label)
{
	uint16_t addr;
	int8_t axis, mnemonic;
	int8_t value_array[4];
	double value;

	fprintf_P(stderr, PSTR("\nDump NMV - %S\n"), label);

	for (uint16_t i=start_record; i<end_record;i++) {
		addr = i*NVM_RECORD_LEN;
		axis = EEPROM_ReadByte(addr);
		mnemonic = EEPROM_ReadByte(addr+1);
		value_array[0] = EEPROM_ReadByte(addr+2);
		value_array[1] = EEPROM_ReadByte(addr+3);
		value_array[2] = EEPROM_ReadByte(addr+4);
		value_array[3] = EEPROM_ReadByte(addr+5);
		memcpy(&value, &value_array[0], sizeof(double));

		fprintf_P(stderr, PSTR("Record %d - %d %d %d %d %d %d [%c%s%1.2f]\n"),
								i, axis, mnemonic, 
								value_array[0], value_array[1],
								value_array[2], value_array[3], 
								_cfg_get_axis_char(axis), 
								_cfg_get_mnemonic_string(mnemonic), value );
	}
}

//##########################################
//############## UNIT TESTS ################
//##########################################

#ifdef __UNIT_TESTS

//---- prototypes and statics -----
void _test_cfg_init(void);
void _test_cfg_get_setting(void);
void _test_cfg_NVM_operations(void);
void _test_cfg_config_parser(void);
static struct cfgSetting *_cfg_make_setting(const int8_t axis, const int8_t mnemonic, const double value);
static char testblock[40];	// scope this so you can debug it

//---- entry point ----
void cfg_unit_tests()
{
	_test_cfg_init();	// comment out what you dont's want to test now
//	_test_cfg_get_setting();
//	_test_cfg_NVM_operations();
//	_test_cfg_config_parser();
}

//---- test routines ----
// assumes cfgList array has already been set up

void _test_cfg_init()
{
	struct cfgSetting *s;

	// To test inits (by simulation) you MUST use nnvm as the xmega 
	// simulator2 does not support EEPROM simulation (nnvm is a RAM block)
	// Uncomment (define) __NNVM in xmega_eeprom.h
	 
	// The first init is done by the init system. If nnvm is used it will
	// perform an uninitialized reset. Trace this at cfg_init().

	// The second init (below) is an initialized "EEPROM" at current rev . 
	s = _cfg_make_setting(X, MA, 4);
	_cfg_put_NVM_setting(s);
	cfg_init();

	// The third init is an initialized but out-of-rev "EEPROM"
	s = _cfg_make_setting(-1, P_, 4);
	_cfg_put_NVM_setting(s);
	cfg_init();
}

void _test_cfg_get_setting()
{
	struct cfgSetting *s;
	uint8_t fiddle=0;

	s = _cfg_get_setting_by_key(NON_AXIS, P_);
	fiddle +=1;							// let s settle so we can see it
	s = _cfg_get_setting_by_key(NON_AXIS, _P);
	fiddle +=1;
}

void _test_cfg_NVM_operations()
{
	struct cfgSetting *s;

	s = _cfg_make_setting(X, SS, 1244);
	_cfg_put_NVM_setting(s);
	s = _cfg_get_NVM_setting_by_key(X, SS);
	_cfg_put_setting(s);
}

///---- strings for _test_cfg_config_parser ----
//he1234 (this record currently fails)

char configs_P[] PROGMEM = "\
$gu20\n\
$gp2\n\
X map axis to motor 1\n\
 xse1600 (leading space)\n\
xfs 1500.123456789\n\
x SR 1250\n\
Xmicrosteps 8\n\
Xpolarity 0\n\
Xtravel 400.00\n\
yRV 2.54\n\
XLI0\n\
apo0\n\
atr65535\n\
aTW65535\n\
aRE1.27\n\
aID1\n\
g17 (XY plane)\n\
g20 (inches mode)\n\
g28 (home on power-up)\n\
f400.00\n\
s12000\n\
t1 \n\
mm per arc segment 0.01\n\
aLI0\n";

// generate some strings for the parser and test NVM read and write

void _test_cfg_config_parser()
{
	char c;
	uint16_t i = 0;		// FLASH buffer index (allow for > 256 characters)
	uint16_t j = 0;		// RAM buffer index (block)

	// feed the parser one line at a time
	while ((c = pgm_read_byte(&configs_P[i++]))) {
		if (c != '\n') {					
			testblock[j++] = c;			// put characters into line
		} else {						// end of line
			testblock[j] = 0;			// terminate the string
			j = 0;						// ... and reset J for next pass
			cfg_config_parser(testblock, FALSE, FALSE);// parse line w/no display or NVM update
//			cfg_config_parser(testblock, TRUE, FALSE);// parse line w/display
//			cfg_config_parser(testblock, FALSE, TRUE);// parse line w/NVM
//			cfg_config_parser(testblock, TRUE, TRUE);// parse line w/both
		}
	}
}

//---- helper functions ----

static struct cfgSetting *_cfg_make_setting(const int8_t axis, const int8_t mnemonic, const double value)
{
	cs.s.axis = axis;
	cs.s.mnemonic = mnemonic;
	cs.s.value = value;
	return (&cs.s);
}


#endif
