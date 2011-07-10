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
 *	- add a non-colliding mnemonic to cfgMnemonics & mnemonics (see notes)
 *  - add a static apply function and its prototype (_applyMN)
 *	- add a display format string (fmtMN)
 *	- recount COUNT_SETTINGS and related defines
 *	- add init line(s) to the large struct array (see notes)
 *---> if the setting display differently in inches than mm do also:
 *	- add separate format string and apply function for inches mode
 *	- compute and add the conversion factor (which is almost always 25.4) 
 *
 * Notes:
 *	- The order of display is set by the order of the settingList struct.
 *	  None of the other orders matter but are kept sequenced for easier 
 *	  reading and code maintenance.
 * *
 *	- Mnemonics are 2 char ASCII strings and can't start w\an axis name
 *		  - so these are off limits for 1st chars: X,Y,Z,A,B,C,U,V,W
 *
 *	- Gcode defaults are held in the cfg struct as their "G" value, 
 *	  e.g. G20 is held as 20, G61.1 as 61.1, etc. These are converted
 *	  to the internal representations and loaded into the gcode model (gm)
 *	  by the apply functions. 
 *
 *  - Modes and Units: 
 *	  The system can be in either inches mode (G20) or mm mode (G21).
 *	  This afects how settings are displayed and entered.
 *	  ABC axes always use degrees regardless of prevailing unit mode
 */

#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "system.h"
#include "settings.h"
#include "stepper.h"
#include "xio.h"
#include "gcode.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "controller.h"
#include "xmega_eeprom.h"

/* Local defines */

#define CFG_PROFILE ((double)1.00) // NVM profile (use a recognizable number)
#define CFG_VERSION ((double)0.90) // NVM version
#define MNEMONIC_LEN 2			// mnemonic length
#define NVM_RECORD_LEN 6		// NVM record length (fixed length)

/*
 * Settings structure
 *
 * 	Settings are managed as an array of cfgSetting structs in cfgList. 
 *	The array is initialized to the hard-wired defaults in settings.h
 *	Settings are displayed using a display format string bound to the 
 *	stting struct, and applied to the program by running an "apply" 
 *	function that is also bound to the struct. cfgSetting actually has 
 *	*two* sets of format and apply bindings one set is used to display
 *	and apply settings in mm unit mode, the other for inches mode.
 *
 *	HACK ALERT: This enables an enormous hack to save RAM space - 
 *	If the addresses of the mm and inches apply functions are the same
 *	it's because the setting is insensitive to units; and therefore
 *	uses the same apply function in both mm or inches mode. If they
 *	are different it's becuase they have to be. Therefore, if they are
 *	different the routines know to convert in and out of (native) mm mode
 *	based on the inequality of these 2 addresses. This ugliness is hidden
 *	from the children in the following macro:
 */
#define CONVERSION_REQUIRED (s->app_mm != s->app_in)

struct cfgSetting {				// don't mess with the order of the struct
	int8_t axis;				// key: axis 0-N, or -1 if non-axis setting
	int8_t mnemonic;			// key: numeric token for mnemonic
	char * PROGMEM fmt_mm;		// ptr to format string when in mm mode
	char * PROGMEM fmt_in;		// ptr to format string when in in mode
	void (*app_mm)(struct cfgSetting *);// apply value when in mm mode
	void (*app_in)(struct cfgSetting *);// apply value when in in mode
	double value;				// setting value - must be in-cast to double
};

struct cfgSingleton {			// persistent working variables
	uint8_t status;				// return status
	uint8_t profile;			// active profile
	uint16_t nvm_base_addr;		// NVM base address
	uint16_t nvm_profile_base;	// NVM base address of current profile
	char mnem_str[MNEMONIC_LEN+1];// working storage for mnemonic
	struct cfgSetting s;		// shared storage for settings struct
};
static struct cfgSingleton cs;

//--- STATIC FUNCTION PROTOTYPES -----------------------------------------

static uint8_t _cfg_normalize_config_block(char *block);
static uint8_t _cfg_parse_config_block(char *block, struct cfgSetting *s);
static void _cfg_print_settings(const char c);
static void _cfg_print_setting(const struct cfgSetting *s);
static void _cfg_print_step_rates(const int8_t axis, const char axis_char);
static void _cfg_print_axis_mode(const int8_t axis, const char axis_char);
static void _print_NVM_record(const int16_t record_number, const int8_t *nvm_record);
static int8_t _cfg_get_axis_num(const char c);
static int8_t _cfg_get_mnemonic_num(const char *mnemonic);
static char *_cfg_get_mnemonic_string(const int8_t mnum);
static void _cfg_set_steps_per_unit(const struct cfgSetting *s);
static int16_t _cfg_get_setting_index(const int8_t axis, const int8_t mnemonic);
static double _cfg_get_setting_value_by_key(const int8_t axis, const int8_t mnemonic);
static struct cfgSetting *_cfg_get_setting_by_index(const uint16_t index);
static struct cfgSetting *_cfg_get_setting_by_key(const int8_t axis, const int8_t mnemonic);
static struct cfgSetting *_cfg_put_setting(const struct cfgSetting *s);
static void _cfg_apply_setting(struct cfgSetting *s);
static struct cfgSetting *_cfg_get_NVM_setting_by_index(const uint16_t index);
static uint8_t _cfg_put_NVM_setting(const struct cfgSetting *s);
//UNUSED static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t axis, const int8_t mnemonic);

/************************************************************************
 *** START SETTING-SPECIFIC REGION **************************************
 ************************************************************************/

/*--- MNEMONICS AND COUNTS ----------------------------------------------
 * All items in this section MUST stay in alignment. 
 * Be sure to confirm or adjust COUNT_SETTINGS is you change mnemonics.
 * or you will get "warning: excess elements in array initializer" error
 * Profile and version records must be first. Must end with the trailer.
 * See DISPLAY FORMAT STRINGS for explanations of what the mnemonics mean
 */
enum cfgMnemonics { P_, V_,		// profile and version records

	// per-axis settings
	MO, SR, FR, TR, TH,
	TS, SA, RO, MI, PO, 
	PW, LI, HE, HR, HC, 
	HO, HB,						// ---> per-axis count 17 (adjust #define)

	// non-axis settings
	GU, GL, GP, GD, EA, 
	JL, JR, JT, JB, MM, 
	MT, M1, M2, M3, M4, 
	HM, EC, IC, IL, EX, 
	EE,							// ---> non-axis count 21 (adjust #define)

	_P							// profile trailer record. Must be last
};

char *mnemonics[] = { "P_",	"V_",// must align with cfgMnemonics
	"MO", "SR", "FR", "TR", "TH", 
	"TS", "SA", "RO", "MI", "PO", 
	"PW", "LI", "HE", "HR", "HC", 
	"HO", "HB",
	"GU", "GL", "GP", "GD", "EA", 
	"JL", "JR", "JT", "JB", "MM", 
	"MT", "M1", "M2", "M3", "M4", 
	"HM", "EC", "IC", "IL", "EX", 
	"EE",
	"_P"
};
#define MAX_MNEMONIC _P

#define COUNT_AXES AXES				// count of supported axes
#define COUNT_PER_AXIS 17			// count of per-axis settings
#define COUNT_NON_AXIS 21			// count of non-axis settings
#define COUNT_HDR_TRLR 3			// the 2 headers and 1 trailer 
#define COUNT_SETTINGS ((COUNT_PER_AXIS * COUNT_AXES) + COUNT_NON_AXIS + COUNT_HDR_TRLR)

//--- "_APPLY()" FUNCTION PROTOTYPES ------------------------------------------

// per-axis settings - unit agnostic
static void _apply00(struct cfgSetting *s);	// null function
static void _applyMO(struct cfgSetting *s);
static void _applyPW(struct cfgSetting *s);
static void _applyLI(struct cfgSetting *s);
static void _applyHE(struct cfgSetting *s);
static void _applySA(struct cfgSetting *s);
static void _applyMI(struct cfgSetting *s);
static void _applyPO(struct cfgSetting *s);

// per-axis settings - mm units
static void _appmmSR(struct cfgSetting *s);
static void _appmmFR(struct cfgSetting *s);
static void _appmmTH(struct cfgSetting *s);
static void _appmmTS(struct cfgSetting *s);
static void _appmmRO(struct cfgSetting *s);
static void _appmmHR(struct cfgSetting *s);
static void _appmmHC(struct cfgSetting *s);
static void _appmmHO(struct cfgSetting *s);
static void _appmmHB(struct cfgSetting *s);
static void _appmmTR(struct cfgSetting *s);

// per-axis settings - inch units
static void _appinSR(struct cfgSetting *s);
static void _appinFR(struct cfgSetting *s);
static void _appinTR(struct cfgSetting *s);
static void _appinTH(struct cfgSetting *s);
static void _appinTS(struct cfgSetting *s);
static void _appinRO(struct cfgSetting *s);
static void _appinHR(struct cfgSetting *s);
static void _appinHC(struct cfgSetting *s);
static void _appinHO(struct cfgSetting *s);
static void _appinHB(struct cfgSetting *s);

// per-axis settings - degree units
/* these are pass thrus - use macros instead
static void _appdgSR(struct cfgSetting *s);
static void _appdgFR(struct cfgSetting *s);
static void _appdgTR(struct cfgSetting *s);
static void _appdgTH(struct cfgSetting *s);
static void _appdgTS(struct cfgSetting *s);
static void _appdgHR(struct cfgSetting *s);
static void _appdgHC(struct cfgSetting *s);
static void _appdgHO(struct cfgSetting *s);
static void _appdgHB(struct cfgSetting *s);
*/
#define _appdgSR _appmmSR
#define _appdgFR _appmmFR
#define _appdgTR _appmmTR
#define _appdgTH _appmmTH
#define _appdgTS _appmmTS
#define _appdgHR _appmmHR
#define _appdgHC _appmmHC
#define _appdgHO _appmmHO
#define _appdgHB _appmmHB

// non-axis settings - mm units
static void _appmmJL(struct cfgSetting *s);
static void _appmmMM(struct cfgSetting *s);

// non-axis settings - inch units
static void _appinJL(struct cfgSetting *s);
static void _appinMM(struct cfgSetting *s);

// non-axis settings - unit agnostic
static void _applyGC(struct cfgSetting *s);
static void _applyEA(struct cfgSetting *s);
static void _applyJR(struct cfgSetting *s);
static void _applyJT(struct cfgSetting *s);
static void _applyJB(struct cfgSetting *s);
static void _applyMT(struct cfgSetting *s);
static void _applyHM(struct cfgSetting *s);

static void _applyM1(struct cfgSetting *s);
static void _applyM2(struct cfgSetting *s);
static void _applyM3(struct cfgSetting *s);
static void _applyM4(struct cfgSetting *s);

static void _applyEC(struct cfgSetting *s);
static void _applyIC(struct cfgSetting *s);
static void _applyIL(struct cfgSetting *s);
static void _applyEX(struct cfgSetting *s);
static void _applyEE(struct cfgSetting *s);

//--- DISPLAY FORMAT STRINGS ---------------------------------------------
// There are 4 possibilities:			   naming convention used
//	  agnostic 	- applies to all units		fmtXX, _applyXX 
//	  mm 		- applies to mm units		fmmXX, _appmmXX
//	  inches 	- applies to inches units	finXX, _appinXX
//	  degrees	- applies to degrees units	fdgXX, _appdgXX

char fmtP_[] PROGMEM = "Profile %1.2f [%s%1.2f]\n";	// print profile number
char fmtV_[] PROGMEM = "Version %1.2f [%s%1.2f]\n";	// print version number

// per-axis settings - unit agnostic (applies to all axes)
char fmtMO[] PROGMEM = "Axis mode          %5.0f [0-10]       $%c%s%1.0f\n";
char fmtMI[] PROGMEM = "Microsteps         %5.0f [1,2,4,8]    $%c%s%1.0f\n";
char fmtPO[] PROGMEM = "Motor polarity     %5.0f [0,1]        $%c%s%1.0f\n";
char fmtPW[] PROGMEM = "Power mgmt mode    %5.0f [0,1]        $%c%s%1.0f\n";
char fmtLI[] PROGMEM = "Limit switch mode  %5.0f [0,1]        $%c%s%1.0f\n";
char fmtHE[] PROGMEM = "Homing enabled     %5.0f [0,1]        $%c%s%1.0f\n";
char fmtSA[] PROGMEM = "Step angle         %5.3f degrees      $%c%s%1.2f\n";

// per-axis settings - mm mode (applies to XYZ axes)
char fmmSR[] PROGMEM = "Seek rate       %8.0f mm/min       $%c%s%1.0f\n";
char fmmFR[] PROGMEM = "Feed rate       %8.0f mm/min       $%c%s%1.0f\n";
char fmmTH[] PROGMEM = "Travel hard limit  %5.0f mm           $%c%s%1.0f\n";
char fmmTS[] PROGMEM = "Travel soft limit  %5.0f mm           $%c%s%1.0f\n";
char fmmHR[] PROGMEM = "Homing seek rate   %5.0f mm/min       $%c%s%1.0f\n";
char fmmHC[] PROGMEM = "Homing close rate  %5.0f mm/min       $%c%s%1.0f\n";
char fmmHO[] PROGMEM = "Homing offset      %5.0f mm           $%c%s%1.0f\n";
char fmmHB[] PROGMEM = "Homing backoff     %5.0f mm           $%c%s%1.0f\n";
char fmmTR[] PROGMEM = "Travel/rev      %8.2f mm           $%c%s%1.0f\n";
char fmmRO[] PROGMEM = "Rotary Circumf  %8.3f mm           $%c%s%1.3f\n";

// per-axis settings - inches mode (applies to XYZ axes)
char finSR[] PROGMEM = "Seek rate          %5.2f in/min       $%c%s%1.2f\n";
char finFR[] PROGMEM = "Feed rate          %5.2f in/min       $%c%s%1.2f\n";
char finTH[] PROGMEM = "Travel hard limit  %5.0f inches       $%c%s%1.0f\n";
char finTS[] PROGMEM = "Travel soft limit  %5.0f inches       $%c%s%1.0f\n";
char finHR[] PROGMEM = "Homing seek rate  %6.2f in/min       $%c%s%1.2f\n";
char finHC[] PROGMEM = "Homing close rate %6.2f in/min       $%c%s%1.2f\n";
char finHO[] PROGMEM = "Homing offset      %5.2f inches       $%c%s%1.2f\n";
char finHB[] PROGMEM = "Homing backoff     %5.2f inches       $%c%s%1.2f\n";
char finTR[] PROGMEM = "Travel/rev      %8.3f inches       $%c%s%1.3f\n";
char finRO[] PROGMEM = "Rotary Circumf  %8.3f inches       $%c%s%1.3f\n";

// per-axis settings - degrees mode (applies to ABC axes)
char fdgSR[] PROGMEM = "Seek rate       %8.0f deg/min      $%c%s%1.0f\n";
char fdgFR[] PROGMEM = "Feed rate       %8.0f deg/min      $%c%s%1.0f\n";
char fdgTH[] PROGMEM = "Travel hard limit  %5.0f degrees      $%c%s%1.0f\n";
char fdgTS[] PROGMEM = "Travel soft limit  %5.0f degrees      $%c%s%1.0f\n";
char fdgHR[] PROGMEM = "Homing seek rate   %5.0f deg/min      $%c%s%1.2f\n";
char fdgHC[] PROGMEM = "Homing close rate  %5.0f deg/min      $%c%s%1.2f\n";
char fdgHO[] PROGMEM = "Homing offset      %5.0f degrees      $%c%s%1.2f\n";
char fdgHB[] PROGMEM = "Homing backoff     %5.0f degrees      $%c%s%1.2f\n";
char fdgTR[] PROGMEM = "Travel/rev      %8.3f degrees      $%c%s%1.3f\n";

// non-axis settings - agnostic  (NOTE: removed spindle and tool in version 311)
char fmtGU[] PROGMEM = "Gcode: Units {G20,G21}         %2.0f [20,21]      $%s%1.0f\n";
char fmtGL[] PROGMEM = "Gcode: Plane {G17,G18,G19}     %2.0f [17,18,19]   $%s%1.0f\n";
char fmtGP[] PROGMEM = "Gcode: Path  {G61,G61.1,G64} %3.1f [61,61.1,64] $%s%1.1f\n";
char fmtGD[] PROGMEM = "Gcode: Distance Mode {G90,G91} %2.0f [90,91]      $%s%1.0f\n";

char fmtHM[] PROGMEM = "Homing mode                     %1.0f [0,1]        $%s%1.0f\n";
char fmtEA[] PROGMEM = "Enable Acceleration             %1.0f [0,1]        $%s%1.0f\n";
char fmtJR[] PROGMEM = "Rotary jerk maximum    %10.0f deg/min^3     $%s%1.0f\n";
char fmtJT[] PROGMEM = "Corner jerk top threshold   %5.3f              $%s%1.0f\n";
char fmtJB[] PROGMEM = "Corner jerk bot threshold   %5.3f              $%s%1.0f\n";
char fmtMT[] PROGMEM = "Min segment time            %5.0f uSec         $%s%1.0f\n";

// non-axis settings - mm mode
char fmmMM[] PROGMEM = "Min segment length          %5.3f mm           $%s%1.0f\n";
char fmmJL[] PROGMEM = "Linear jerk maximum    %10.0f mm/min^3     $%s%1.0f\n";

// non-axis settings - inches mode
char finMM[] PROGMEM = "Min segment length         %5.4f inches       $%s%1.4f\n";
char finJL[] PROGMEM = "Linear jerk maximum    %10.0f in/min^3     $%s%1.0f\n";

// non-axis settings - degrees mode
// No non-axis degrees settings as only ABC axes use degrees mode

// per-motor settings 
char fmtM1[] PROGMEM = "Map motor 1 to axis [0=x,1=y..] %1.0f [0-3]        $%s%1.0f\n";
char fmtM2[] PROGMEM = "Map motor 2 to axis             %1.0f [0-3]        $%s%1.0f\n";
char fmtM3[] PROGMEM = "Map motor 3 to axis             %1.0f [0-3]        $%s%1.0f\n";
char fmtM4[] PROGMEM = "Map motor 4 to axis             %1.0f [0-3]        $%s%1.0f\n";

// other settings 
char fmtEC[] PROGMEM = "Convert LF to CR LF (outgoing)  %1.0f [0,1]        $%s%1.0f\n";
char fmtIC[] PROGMEM = "Ignore Incoming CR              %1.0f [0,1]        $%s%1.0f\n";
char fmtIL[] PROGMEM = "Ignore Incoming LF              %1.0f [0,1]        $%s%1.0f\n";
char fmtEX[] PROGMEM = "Enable Xon/Xoff Flow Control    %1.0f [0,1]        $%s%1.0f\n";
char fmtEE[] PROGMEM = "Enable Echo                     %1.0f [0,1]        $%s%1.0f\n";


/*---- SETTING LIST STRUCTURE --------------------------------------------
 *
 *	Initialize all settings structs and defaults
 *
 * struct cfgSetting {
 *	int8_t axis;				// key: axis 0-N, or -1 if non-axis
 *	int8_t mnemonic;			// key: numeric token for mnemonic
 *	char * PROGMEM fmt_mm;		// ptr to format string for mm display
 *	char * PROGMEM fmt_in;		// ptr to format string for in display 
 *	void (*apply)(struct cfgSetting *);// mm apply function
 *	void (*appin)(struct cfgSetting *);// in apply function
 *	double inch_convert;		// conversion factor for inches mode
 *	double value;				// setting value
 *
 * The cfgList struct below is a rather brute-force way of handling the 
 *	setting structs and their initialization. Each struct is 30 bytes
 *	and there are quite a lot of them. Memory savings could be had by
 *	putting the format strings, apply functions and inch_convert value in
 *	program memory tables and indexing them by uint8 or uint16 values.
 */

struct cfgSetting cfgList[COUNT_SETTINGS] = { 
	// starting version.profile record - must be first
	{ NON_AXIS, P_, fmtP_, fmtP_, _apply00, _apply00,(double) CFG_PROFILE },// don't mess
	{ NON_AXIS, V_, fmtV_, fmtV_, _apply00, _apply00, (double) CFG_VERSION },// don't mess

	{ X, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) X_AXIS_MODE },
	{ X, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) X_SEEK_RATE_MAX },
	{ X, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) X_FEED_RATE_MAX },
	{ X, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) X_TRAVEL_PER_REV },
	{ X, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) X_TRAVEL_HARD_LIMIT },
	{ X, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) X_TRAVEL_SOFT_LIMIT },
	{ X, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) 0 },
	{ X, SA, fmtSA, fmtSA, _applySA, _applySA, (double) X_STEP_ANGLE },
	{ X, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) X_MICROSTEPS },
	{ X, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) X_POLARITY },
	{ X, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) X_POWER_MODE },
	{ X, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) X_LIMIT_MODE },
	{ X, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) X_HOMING_ENABLE },
	{ X, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) X_HOMING_SEEK_RATE },
	{ X, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) X_HOMING_CLOSE_RATE },
	{ X, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) X_HOMING_OFFSET },
	{ X, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) X_HOMING_BACKOFF },

	{ Y, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Y_AXIS_MODE },
	{ Y, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) Y_SEEK_RATE_MAX },
	{ Y, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) Y_FEED_RATE_MAX },
	{ Y, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) Y_TRAVEL_PER_REV },
	{ Y, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) Y_TRAVEL_HARD_LIMIT },
	{ Y, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) Y_TRAVEL_SOFT_LIMIT },
	{ Y, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) 0 },
	{ Y, SA, fmtSA, fmtSA, _applySA, _applySA, (double) Y_STEP_ANGLE },
	{ Y, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) Y_MICROSTEPS },
	{ Y, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) Y_POLARITY },
	{ Y, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) Y_POWER_MODE },
	{ Y, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Y_LIMIT_MODE },
	{ Y, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Y_HOMING_ENABLE },
	{ Y, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) Y_HOMING_SEEK_RATE },
	{ Y, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) Y_HOMING_CLOSE_RATE },
	{ Y, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) Y_HOMING_OFFSET },
	{ Y, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) Y_HOMING_BACKOFF },

	{ Z, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Z_AXIS_MODE },
	{ Z, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) Z_SEEK_RATE_MAX },
	{ Z, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) Z_FEED_RATE_MAX },
	{ Z, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) Z_TRAVEL_PER_REV },
	{ Z, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) Z_TRAVEL_HARD_LIMIT },
	{ Z, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) Z_TRAVEL_SOFT_LIMIT },
	{ Z, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) 0 },
	{ Z, SA, fmtSA, fmtSA, _applySA, _applySA, (double) Z_STEP_ANGLE },
	{ Z, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) Z_MICROSTEPS },
	{ Z, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) Z_POLARITY },
	{ Z, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) Z_POWER_MODE },
	{ Z, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Z_LIMIT_MODE },
	{ Z, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Z_HOMING_ENABLE },
	{ Z, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) Z_HOMING_SEEK_RATE },
	{ Z, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) Z_HOMING_CLOSE_RATE },
	{ Z, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) Z_HOMING_OFFSET },
	{ Z, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) Z_HOMING_BACKOFF },

	{ A, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) A_AXIS_MODE },
	{ A, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) A_SEEK_RATE_MAX },
	{ A, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) A_FEED_RATE_MAX },
	{ A, TR, fdgTR, fdgTR, _appdgTR, _appdgTR, (double) A_TRAVEL_PER_REV },
	{ A, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) A_TRAVEL_HARD_LIMIT },
	{ A, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) A_TRAVEL_SOFT_LIMIT },
	{ A, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) A_CIRCUMFERENCE },
	{ A, SA, fmtSA, fmtSA, _applySA, _applySA, (double) A_STEP_ANGLE },
	{ A, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) A_MICROSTEPS },
	{ A, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) A_POLARITY },
	{ A, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) A_POWER_MODE },
	{ A, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) A_LIMIT_MODE },
	{ A, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) A_HOMING_ENABLE },
	{ A, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) A_HOMING_SEEK_RATE },
	{ A, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) A_HOMING_CLOSE_RATE },
	{ A, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) A_HOMING_OFFSET },
	{ A, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) A_HOMING_BACKOFF },

	{ B, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) B_AXIS_MODE },
	{ B, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) B_SEEK_RATE_MAX },
	{ B, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) B_FEED_RATE_MAX },
	{ B, TR, fdgTR, fdgTR, _appdgTR, _appdgTR, (double) B_TRAVEL_PER_REV },
	{ B, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) B_TRAVEL_HARD_LIMIT },
	{ B, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) B_TRAVEL_SOFT_LIMIT },
	{ B, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) B_CIRCUMFERENCE },
	{ B, SA, fmtSA, fmtSA, _applySA, _applySA, (double) B_STEP_ANGLE },
	{ B, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) B_MICROSTEPS },
	{ B, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) B_POLARITY },
	{ B, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) B_POWER_MODE },
	{ B, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) B_LIMIT_MODE },
	{ B, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) B_HOMING_ENABLE },
	{ B, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) B_HOMING_SEEK_RATE },
	{ B, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) B_HOMING_CLOSE_RATE },
	{ B, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) B_HOMING_OFFSET },
	{ B, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) B_HOMING_BACKOFF },

	{ C, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) C_AXIS_MODE },
	{ C, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) C_SEEK_RATE_MAX },
	{ C, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) C_FEED_RATE_MAX },
	{ C, TR, fdgTR, fdgTR, _appdgTR, _appdgTR, (double) C_TRAVEL_PER_REV },
	{ C, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) C_TRAVEL_HARD_LIMIT },
	{ C, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) C_TRAVEL_SOFT_LIMIT },
	{ C, RO, fmmRO, finRO, _appmmRO, _appinRO, (double) C_CIRCUMFERENCE },
	{ C, SA, fmtSA, fmtSA, _applySA, _applySA, (double) C_STEP_ANGLE },
	{ C, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) C_MICROSTEPS },
	{ C, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) C_POLARITY },
	{ C, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) C_POWER_MODE },
	{ C, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) C_LIMIT_MODE },
	{ C, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) C_HOMING_ENABLE },
	{ C, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) C_HOMING_SEEK_RATE },
	{ C, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) C_HOMING_CLOSE_RATE },
	{ C, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) C_HOMING_OFFSET },
	{ C, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) C_HOMING_BACKOFF },

	{ NON_AXIS, GU, fmtGU, fmtGU, _applyGC, _applyGC, (double) GCODE_UNITS },
	{ NON_AXIS, GL, fmtGL, fmtGL, _applyGC, _applyGC, (double) GCODE_PLANE },
	{ NON_AXIS, GP, fmtGP, fmtGP, _applyGC, _applyGC, (double) GCODE_PATH_CONTROL },
	{ NON_AXIS, GD, fmtGD, fmtGD, _applyGC, _applyGC, (double) GCODE_DISTANCE_MODE },

	{ NON_AXIS, EA, fmtEA, fmtEA, _applyEA, _applyEA, (double) ENABLE_ACCEL },
	{ NON_AXIS, JL, fmmJL, finJL, _appmmJL, _appinJL, (double) MAX_LINEAR_JERK },
	{ NON_AXIS, JR, fmtJR, fmtJR, _applyJR, _applyJR, (double) MAX_ROTARY_JERK },
	{ NON_AXIS, JT, fmtJT, fmtJT, _applyJT, _applyJT, (double) CORNER_JERK_UPPER_THRESHOLD },
	{ NON_AXIS, JB, fmtJB, fmtJB, _applyJB, _applyJB, (double) CORNER_JERK_LOWER_THRESHOLD },
	{ NON_AXIS, MM, fmmMM, finMM, _appmmMM, _appinMM, (double) MIN_SEGMENT_LENGTH },
	{ NON_AXIS, MT, fmtMT, fmtMT, _applyMT, _applyMT, (double) MIN_SEGMENT_TIME },
	{ NON_AXIS, M1, fmtM1, fmtM1, _applyM1, _applyM1, (double) X },
	{ NON_AXIS, M2, fmtM2, fmtM2, _applyM2, _applyM2, (double) Y },
	{ NON_AXIS, M3, fmtM3, fmtM3, _applyM3, _applyM3, (double) Z },
	{ NON_AXIS, M4, fmtM4, fmtM4, _applyM4, _applyM4, (double) A },
	{ NON_AXIS, HM, fmtHM, fmtHM, _applyHM, _applyHM, (double) HOMING_MODE },

	{ NON_AXIS, EC, fmtEC, fmtEC, _applyEC, _applyEC, (double) FALSE },
	{ NON_AXIS, IC, fmtIC, fmtIC, _applyIC, _applyIC, (double) FALSE },
	{ NON_AXIS, IL, fmtIL, fmtIL, _applyIL, _applyIL, (double) FALSE },
	{ NON_AXIS, EX, fmtEX, fmtEX, _applyEX, _applyEX, (double) TRUE },
	{ NON_AXIS, EE, fmtEE, fmtEE, _applyEE, _applyEE, (double) TRUE },

	// ending version record - must be last
	{ NON_AXIS, _P, fmtP_, fmtP_, _apply00, _apply00, (double) CFG_VERSION }	// don't mess
};

/*---- APPLY FUNCTIONS -------------------------------------------------*/
// Note: some code style conventions abandoned for density & readability
static void _apply00(struct cfgSetting *s) { return; }	// null apply

// per-axis settings - unit agnostic
static void _applyMO(struct cfgSetting *s) { cfg.a[s->axis].axis_mode = (uint8_t)s->value;}
static void _applyPW(struct cfgSetting *s) { cfg.a[s->axis].power_mode = (uint8_t)s->value;}
static void _applyLI(struct cfgSetting *s) { cfg.a[s->axis].limit_mode = (uint8_t)s->value;}
static void _applyHE(struct cfgSetting *s) { cfg.a[s->axis].homing_enable = (uint8_t)s->value;}

static void _applySA(struct cfgSetting *s) {
	cfg.a[s->axis].step_angle = s->value;
	_cfg_set_steps_per_unit(s); 
}
static void _applyMI(struct cfgSetting *s) { 
	cfg.a[s->axis].microsteps = (uint8_t)s->value;
	st_set_microsteps(s->axis, cfg.a[s->axis].microsteps);
	_cfg_set_steps_per_unit(s); 
}
static void _applyPO(struct cfgSetting *s) { 
	cfg.a[s->axis].polarity = (uint8_t)s->value;
	st_set_polarity(s->axis, cfg.a[s->axis].polarity);
}

// per-axis settings - mm units
static void _appmmSR(struct cfgSetting *s) { cfg.a[s->axis].seek_rate_max = s->value;}
static void _appmmFR(struct cfgSetting *s) { cfg.a[s->axis].feed_rate_max = s->value; }
static void _appmmTH(struct cfgSetting *s) { cfg.a[s->axis].travel_hard_limit = s->value; }
static void _appmmTS(struct cfgSetting *s) { cfg.a[s->axis].travel_soft_limit = s->value; }
static void _appmmRO(struct cfgSetting *s) { cfg.a[s->axis].circumference = s->value; }
static void _appmmHR(struct cfgSetting *s) { cfg.a[s->axis].homing_rate = s->value; }
static void _appmmHC(struct cfgSetting *s) { cfg.a[s->axis].homing_close = s->value; }
static void _appmmHO(struct cfgSetting *s) { cfg.a[s->axis].homing_offset = s->value; }
static void _appmmHB(struct cfgSetting *s) { cfg.a[s->axis].homing_backoff = s->value; }

static void _appmmTR(struct cfgSetting *s) { 
	cfg.a[s->axis].travel_rev = s->value;
	_cfg_set_steps_per_unit(s); 
}

// per-axis settings - inches units
static void _appinSR(struct cfgSetting *s) { s->value *= 25.4; _appmmSR(s); }
static void _appinFR(struct cfgSetting *s) { s->value *= 25.4; _appmmFR(s); }
static void _appinTR(struct cfgSetting *s) { s->value *= 25.4; _appmmTR(s); }
static void _appinTH(struct cfgSetting *s) { s->value *= 25.4; _appmmTH(s); }
static void _appinTS(struct cfgSetting *s) { s->value *= 25.4; _appmmTS(s); }
static void _appinRO(struct cfgSetting *s) { s->value *= 25.4; _appmmRO(s); }
static void _appinHR(struct cfgSetting *s) { s->value *= 25.4; _appmmHR(s); }
static void _appinHC(struct cfgSetting *s) { s->value *= 25.4; _appmmHC(s); }
static void _appinHO(struct cfgSetting *s) { s->value *= 25.4; _appmmHO(s); }
static void _appinHB(struct cfgSetting *s) { s->value *= 25.4; _appmmHB(s); }

// per-axis settings - degree units
/* See function prototypes for macro definitions
static void _appdgSR(struct cfgSetting *s) { _appmmSR(s); }
static void _appdgFR(struct cfgSetting *s) { _appmmFR(s); }
static void _appdgTR(struct cfgSetting *s) { _appmmTR(s); }
static void _appdgTH(struct cfgSetting *s) { _appmmTH(s); }
static void _appdgTS(struct cfgSetting *s) { _appmmTS(s); }
static void _appdgHR(struct cfgSetting *s) { _appmmHR(s); }
static void _appdgHC(struct cfgSetting *s) { _appmmHC(s); }
static void _appdgHO(struct cfgSetting *s) { _appmmHO(s); }
static void _appdgHB(struct cfgSetting *s) { _appmmHB(s); }
*/

// non-axis settings - mm units
static void _appmmJL(struct cfgSetting *s) { cfg.linear_jerk_max = s->value; }
static void _appmmMM(struct cfgSetting *s) { cfg.min_segment_len = s->value; }

// non-axis settings - inch units
static void _appinJL(struct cfgSetting *s) { s->value *= 25.4; _appmmJL(s); }
static void _appinMM(struct cfgSetting *s) { s->value *= 25.4; _appmmMM(s); }

// non-axis settings - degree units 
// (none)

// non-axis settings - unit agnostic
static void _applyGC(struct cfgSetting *s) { // common function suffices
	switch ((int)s->value*10) {
		case 200: cm_use_length_units(TRUE); return; // set inches mode
		case 210: cm_use_length_units(FALSE); return;// set mm mode
		case 170: cm_select_plane(CANON_PLANE_XY); return;
		case 180: cm_select_plane(CANON_PLANE_XZ); return;
		case 190: cm_select_plane(CANON_PLANE_YZ); return;
		case 610: cm_set_motion_control_mode(PATH_EXACT_STOP); return;
		case 611: cm_set_motion_control_mode(PATH_EXACT_PATH); return;
		case 640: cm_set_motion_control_mode(PATH_CONTINUOUS); return;
		case 900: cm_set_distance_mode(TRUE); return; // set absolute mode
		case 910: cm_set_distance_mode(FALSE); return;// set incremental mode
	}
}

static void _applyEA(struct cfgSetting *s) { cfg.accel_enabled = (uint8_t)s->value;}
static void _applyJR(struct cfgSetting *s) { cfg.rotary_jerk_max = s->value; }
static void _applyJT(struct cfgSetting *s) { cfg.corner_jerk_upper = s->value; }
static void _applyJB(struct cfgSetting *s) { cfg.corner_jerk_lower = s->value; }
static void _applyMT(struct cfgSetting *s) { cfg.min_segment_time = s->value; }
static void _applyHM(struct cfgSetting *s) { cfg.homing_mode = (uint8_t)s->value;}

static void _applyM1(struct cfgSetting *s) { cfg.motor_map[MOTOR_1] = (uint8_t)s->value;}
static void _applyM2(struct cfgSetting *s) { cfg.motor_map[MOTOR_2] = (uint8_t)s->value;}
static void _applyM3(struct cfgSetting *s) { cfg.motor_map[MOTOR_3] = (uint8_t)s->value;}
static void _applyM4(struct cfgSetting *s) { cfg.motor_map[MOTOR_4] = (uint8_t)s->value;}


//----- SERIAL CONTROL APPLY FUNCTIONS -----
// assume USB is the std device
static void _applyEC(struct cfgSetting *s) { 
	if (s->value > EPSILON) {		// floating point test for non-zero
		(void)xio_cntl(XIO_DEV_USB, XIO_CRLF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOCRLF);
	}
}

static void _applyIC(struct cfgSetting *s) {
	if (s->value > EPSILON) {
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORECR);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORECR);
	}
}

static void _applyIL(struct cfgSetting *s) {
	if (s->value > EPSILON) {
		(void)xio_cntl(XIO_DEV_USB, XIO_IGNORELF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOIGNORELF);
	}
}

static void _applyEX(struct cfgSetting *s) {
	if (s->value > EPSILON) {
		(void)xio_cntl(XIO_DEV_USB, XIO_XOFF);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOXOFF);
	}
}

static void _applyEE(struct cfgSetting *s) {
	if (s->value > EPSILON) {
		(void)xio_cntl(XIO_DEV_USB, XIO_ECHO);
	} else {
		(void)xio_cntl(XIO_DEV_USB, XIO_NOECHO);
	}
}

//--- APPLY FUNCTION HELPERS ---------------------------------------------

/*
 * _cfg_set_steps_per_unit() - compute mm of travel per microstep
 *
 *		steps = (360 / (step_angle / microsteps) / travel_per_rev
 *
 *	This function will need to be rethought when microstep morphing is 
 *	implemented, as microsteps are calculated statically. Dang.
 */
static void _cfg_set_steps_per_unit(const struct cfgSetting *s)
{
	cfg.a[s->axis].steps_per_unit = (360 / 
				(cfg.a[s->axis].step_angle / cfg.a[s->axis].microsteps) /
				 cfg.a[s->axis].travel_rev);
	
}

/*************************************************************************
 *** END SETTING-SPECIFIC REGION *****************************************
 ************************************************************************/

//----- CORE CONFIG SYSTEM FUNCTIONS -------------------------------------

/*
 * cfg_init() - called once on system init
 * cfg_init_gcode_model()
 *
 *	Will perform one of 3 actions:
 *	- if NVM is set up and at current version: load NVM into config ram
 *	- if NVM is not set up: load RAM and NVM with hardwired default settings
 *	- if NVM is out-of-rev: apply all old settings that are still 
 *	  		 applicable, then migrate new settings to NVM
 */

void cfg_init() 
{
	uint16_t i = 0;
	struct cfgSetting *s;
	double profile = 0, version = 0;

	// initialize working variables
	cs.profile = 0;				// first (and currently only) profile
	cs.status = TG_OK;
	cs.nvm_base_addr = CFG_NVM_BASE;
	cs.nvm_profile_base = cs.nvm_base_addr;

#ifdef __DISABLE_EEPROM		// cutout for debug simulation
	// Apply the hard-wired default values from settings.h
	for (i=0; i<COUNT_SETTINGS; i++) {
		cfgList[i].apply(&cfgList[i]);
	}
	return;
#endif

	// get the profile and version numbers from NVM
	if ((s = _cfg_get_NVM_setting_by_index(P_)) != NULL) {
		profile = s->value;
	}
	if ((s = _cfg_get_NVM_setting_by_index(V_)) != NULL) {
		version = s->value;
	}
	// if NVM is initialized and NVM is at the current format version
	if (FLOAT_EQ(version, CFG_VERSION)) {
		for (i=0; i<COUNT_SETTINGS; i++) {
			_cfg_put_setting(_cfg_get_NVM_setting_by_index(i));
		}
	} else { 	// if NVM is uninitialized or out of revision
		fprintf_P(stderr, PSTR("....Initializing EEPROM\n"));
		fprintf_P(stderr, PSTR("EEPROM profile %4.2f, version %4.2f\n"), profile, version);
		for (i=0; i<COUNT_SETTINGS; i++) {
			s = _cfg_get_setting_by_index(i);	// read setting from RAM
			_cfg_put_setting(s);				// apply setting
			_cfg_put_NVM_setting(s);			// write setting to NVM
		}
	}	
#ifdef __dbSHOW_CONFIG_STATE
	cfg_dump_NVM(0,30,PSTR("Initialized NVM Contents"));
#endif
	return;
}

void cfg_init_gcode_model()
{
	struct cfgSetting *s;

	// in the below you could call either app_mm or app_in - makes no difference
	s = _cfg_get_setting_by_key(NON_AXIS, GU); s->app_mm(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GL); s->app_mm(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GP); s->app_mm(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GD); s->app_mm(s);
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
		_cfg_print_settings(block[0]); // based on 1st char in blk
		return (TG_OK);
	}
	// normalize the block in place - strip leading $ and any whitespace
	if ((_cfg_normalize_config_block(block)) < (MNEMONIC_LEN)) { // not a setting
		_cfg_print_settings(block[0]);
		return (TG_OK);
	}
	// parse the block into its basic parts
	ritorno(_cfg_parse_config_block(block, &cs.s)); 

	// update config value in corresponding parser structure (or die trying)
	if ((s = _cfg_put_setting(&cs.s)) == NULL) {
		return (cs.status);
	}
	if (display) {			// do conditional config display
		_cfg_print_setting(s);
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
 *	Returns # of chars in the normalized string, minus the NUL char
 *
 *	Valid characters (these are passed to config parser):
 *		digits					all digits are passed to parser
 *		lower case alpha		all lower case alpha converted to upper
 *		upper case alpha		all upper case alpha is passed
 *		- . ? 					sign, dot, question mark pass to parser
 *
 *	Invalid characters (these are stripped but don't cause failure):
 *		control characters		chars < 0x20 are all removed
 *		/ *	< = > | % #	+ _		expression chars removed from string
 *		( ) [ ] { } 			expression chars removed from string
 *		<sp> <tab> 				whitespace chars removed from string
 *		! % , ; ; @ 			removed
 *		^ ~ " ' <DEL>			removed
 *		$						removes leading $
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
	// capture a 2 character mnemonic
	while (j<MNEMONIC_LEN) {
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
 * cfg_get_axis_char() 			- return axis char from axis number
 * _cfg_get_axis_num()  			- return axis number from axis char
 * _cfg_get_mnemonic_num()  	- return mnemonic number from string
 * _cfg_get_mnemonic_string() 	- return mnemonic string from number
 */

static int8_t _cfg_get_axis_num(const char c)
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

char cfg_get_axis_char(const int8_t axis)
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
		return (mnemonics[mnum]);// pointer to mnemonic string in array
	}
}

//----- SETTINGS ACCESS PRIMITIVES ---------------------------------------
/*
 *	Access to settings records works like a really dumb, inefficient 
 *	key/value dictionary (ala Python). The key is a compound key 
 *	consisting of the axis + mnemonic.
 *
 *	There are 2 dictionaries: the in-memory config table (array of structs)
 *	and the non-volatile memory records. NVM records are binary versions of 
 *	the config text blocks that are re-hydrated on extraction so they can 
 *	be re-parsed to populate the config table. NVM records look like this:
 *
 *		<axis_byte><mnemonic_byte><value_as_a_double>	(6 bytes total)
 *
 *	A profile in NVM starts and ends with records of this form:
 *
 *		<-1><P_><profile#>		record 0 - profile number (as a double)
 *		<-1><V_><version#>		record 1 - version number (as a double)
 *		<-1><_P><version#>		record N - profile trailer (w/ profile #)
 */

/*
 * _cfg_get_setting_index() - return index of setting or -1 if not found
 *
 *	This is a linear scan of the config table by axis and mnemonic, so the
 *	table must be set up properly for this to work.
 */

static int16_t _cfg_get_setting_index(const int8_t axis, const int8_t mnemonic)
{
	uint16_t i;

	for (i=0; i<COUNT_SETTINGS; i++) {
		if ((cfgList[i].axis == axis) && (cfgList[i].mnemonic == mnemonic)) {
			return(i);
		}
	}
	DEBUG2(dbCONFIG, PSTR("_cfg_get_setting_index() not found for axis %d, mnemonic %d"), axis, mnemonic);
	cs.status = TG_PARAMETER_NOT_FOUND;
	return(-1);		// key not found
}

/*
 * _cfg_get_setting_by_index() - return ptr to next setting or NULL if end
 */

static struct cfgSetting *_cfg_get_setting_by_index(const uint16_t index)
{
	if (index >= COUNT_SETTINGS) {		// range check the index
		DEBUG1(dbCONFIG, PSTR("_cfg_get_setting_by_index failed for index %d"), index)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return(NULL);
	}
	return(&cfgList[index]);
}

/*
 * _cfg_get_setting_by_key() - return ptr to setting or NULL if not found
 */

static struct cfgSetting *_cfg_get_setting_by_key(const int8_t axis, const int8_t mnemonic)
{
	uint16_t i;

	if ((i = _cfg_get_setting_index(axis, mnemonic)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_get_setting_by_key() failed for axis %d, mnemonic %d"), axis, mnemonic)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (NULL);
	}
	return(&cfgList[i]);
}

/*
 * _cfg_get_setting_value_by_key() - return setting value
 */

static double _cfg_get_setting_value_by_key(const int8_t axis, const int8_t mnemonic)
{
	struct cfgSetting *s;
	s = _cfg_get_setting_by_key(axis, mnemonic);
	return s->value;
}

/*
 * _cfg_put_setting() - update a table setting from the setting passed in
 *					  - apply the new value by running the apply function
 *					  - return ptr to updated setting in list (or NULL)
 */

static struct cfgSetting *_cfg_put_setting(const struct cfgSetting *s)
{
	struct cfgSetting *t;	// setting struct in config table

	// locate setting matching the incoming struct
	// (optimization would be to check if the list_idx already
	//  matches the s->axis, s->mnemonic and skip the table scan)
	if ((t = _cfg_get_setting_by_key(s->axis, s->mnemonic)) == NULL) {
		DEBUG2(dbCONFIG, PSTR("_cfg_put_setting() failed for axis %d, mnemonic %d"), s->axis, s->mnemonic)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return(NULL);
	}
	t->value = s->value;	// write the new value to the config table
	_cfg_apply_setting(t);
	return(t);
}

static void _cfg_apply_setting(struct cfgSetting *s)
{
	if (cm_get_inches_mode() == FALSE) {
		s->app_mm(s);	// call the mm mode apply function
	} else {
		s->app_in(s);	// call the inches mode apply function
	}
}

/*
 * _cfg_get_NVM_setting_by_index() - return S struct by index into NVM
 */

static struct cfgSetting *_cfg_get_NVM_setting_by_index(const uint16_t index)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address = cs.nvm_profile_base + (index * NVM_RECORD_LEN);

	(void)EEPROM_ReadBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
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
 * _cfg_get_NVM_setting_by_key() - return S struct with value from NVM
 */
/* UNUSED
static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t axis, const int8_t mnemonic)
{
	uint16_t index;

	if ((index = _cfg_get_setting_index(axis, mnemonic)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_get_NVM_setting_by_key() failed for axis %d, mnemonic %d"), axis, mnemonic)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (NULL);
	}
	return (_cfg_get_NVM_setting_by_index(index));
}
*/

/*
 * _cfg_put_NVM_setting() - write setting to NVM as NVM record
 */

static uint8_t _cfg_put_NVM_setting(const struct cfgSetting *s)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address;
	uint16_t i;

	if ((i = _cfg_get_setting_index(s->axis, s->mnemonic)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_put_NVM_setting() failed for axis %d, mnemonic %d"), s->axis, s->mnemonic)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (cs.status);
	}
	nvm_address = cs.nvm_profile_base + (i * NVM_RECORD_LEN);
	nvm_record[0] = s->axis;
	nvm_record[1] = s->mnemonic;
	memcpy(&nvm_record[2], &s->value, sizeof(double));
	(void)EEPROM_WriteBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
	return(TG_OK);
}

//----- PRINT AND DISPLAY ROUTINES ---------------------------------------
/*
 * cfg_dump_NVM() 		- dump current NVM profile to stderr in 6 byte lines
 * _print_NVM_record()	- print a single record
 *
 *	Requires 'label' to be a program memory string. Usage example:
 *		cfg_dump_NVM(0,10,PSTR("Initial state"));
 */

void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, char *label)
{
	int8_t nvm_record[NVM_RECORD_LEN];
	uint16_t nvm_address;
	uint16_t i;

	fprintf_P(stderr, PSTR("\nDump NMV - %S\n"), label);
	for (i=start_record; i<end_record; i++) {
		nvm_address = cs.nvm_profile_base + (i * NVM_RECORD_LEN);
		(void)EEPROM_ReadBytes(nvm_address, nvm_record, NVM_RECORD_LEN);
		_print_NVM_record(i, nvm_record);
	}
}

static void _print_NVM_record(const int16_t record_number, const int8_t *nvm_record)
{
	double value;

	memcpy(&value, &nvm_record[2], sizeof(double));
	fprintf_P(stderr, PSTR("Record %d - %d %d %d %d %d %d [%c%s%1.2f]\n"),
							record_number,
							nvm_record[0], nvm_record[1], 
							nvm_record[2], nvm_record[3],
							nvm_record[4], nvm_record[5], 
							cfg_get_axis_char(nvm_record[0]), 
							_cfg_get_mnemonic_string(nvm_record[1]), 
							value );
}

/*
 * _cfg_print_settings()	- print settings based on the input char
 *
 * Print_settings() displays depending on what is typically in block[0]:
 *		"$"		- display all settings (from '$$' non-normalized buffer)
 *		"X"		- display axis settings (where X is the axis) 
 *		<NUL>	- display non-axis settings
 *		'E'		- dump EEPROM
 *		'H'		- display help screen
 */

static void _cfg_print_settings(const char c)
{
	int8_t axis = _cfg_get_axis_num(c);
	char axis_char = cfg_get_axis_char(axis);
	uint16_t i;

	switch(c) {
		case '$': {							// print all settings
			for (i=0; i<COUNT_SETTINGS; i++) {	
				_cfg_print_setting(&cfgList[i]);
			}
			return;
		}
		case 'X': case 'Y': case 'Z': 		// print settings for an axis
		case 'A': case 'B': case 'C': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if (cfgList[i].axis == axis) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			// Advisories
			fprintf_P(stderr, PSTR("Your %c axis settings translate to:\n"), axis_char);
			_cfg_print_axis_mode(axis, axis_char);
			_cfg_print_step_rates(axis, axis_char);
			return;
		}
		case 'H': { 						// print help
			cfg_print_config_help();
			return;
		}
		default: {							// print non-axis settings
			for (i=0; i<COUNT_SETTINGS; i++) {
				if (cfgList[i].axis == NON_AXIS) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			fprintf_P(stderr, PSTR("Type $h for configuration help\n"));
		}
	}
}

/*
 * _cfg_print_setting() - print a single setting
 */
static void _cfg_print_setting(const struct cfgSetting *s)
{
	double value = s->value;

	// hack to not display the axis radius values for linear axes
	if ((s->mnemonic == RO) && (s->axis <A)) {
		return;
	}

	if (cm_get_inches_mode() == FALSE) {// mm mode displays
		if (s->axis != NON_AXIS) {
			fprintf_P(stderr, PSTR("%c axis - "), 
						cfg_get_axis_char(s->axis));
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, 
						cfg_get_axis_char(s->axis), 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		} else {
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		}
	} else { // inch mode displays
		if (CONVERSION_REQUIRED) {
			value = s->value / 25.4;
		}
		if (s->axis != NON_AXIS) {
			fprintf_P(stderr, PSTR("%c axis - "), 
						cfg_get_axis_char(s->axis));
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, 
						cfg_get_axis_char(s->axis), 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		} else {
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		}
	}
}

/*
 * _cfg_print_step_rates() - print step rates resulting from the settings
 */
static void _cfg_print_step_rates(const int8_t axis, const char axis_char)
{
	double step_angle = _cfg_get_setting_value_by_key(axis, SA);
	double travel_rev = _cfg_get_setting_value_by_key(axis, TR);
	double seek_rate = _cfg_get_setting_value_by_key(axis, SR);
	double feed_rate = _cfg_get_setting_value_by_key(axis, FR);
	double seek_steps = (seek_rate / 60 / travel_rev) * (360 / step_angle);
	double feed_steps = (feed_rate / 60 / travel_rev) * (360 / step_angle);
	fprintf_P(stderr, PSTR(" %c max seek: %5.0f steps/sec\n"), axis_char, seek_steps);
	fprintf_P(stderr, PSTR(" %c max feed: %5.0f steps/sec\n"), axis_char, feed_steps);
}

/*
 * _cfg_print_axis_mode() - print axis mode meanings
 */
// put strings and string array in program memory
char pam00[] PROGMEM = "DISABLED";
char pam01[] PROGMEM = "ENABLED";
char pam02[] PROGMEM = "INHIBITED";
char pam03[] PROGMEM = "CIRCUMFERENCE MODE";
char pam04[] PROGMEM = "SLAVE X";
char pam05[] PROGMEM = "SLAVE Y";
char pam06[] PROGMEM = "SLAVE Z";
char pam07[] PROGMEM = "SLAVE XY";
char pam08[] PROGMEM = "SLAVE XZ";
char pam09[] PROGMEM = "SLAVE YZ";
char pam10[] PROGMEM = "SLAVE XYZ";

PGM_P cfgPAM[] PROGMEM = {
	pam00, pam01, pam02, pam03, 
	pam04, pam05, pam06, pam07,
	pam08, pam09, pam10
};

static void _cfg_print_axis_mode(const int8_t axis, const char axis_char) 
{
	double axis_mode = _cfg_get_setting_value_by_key(axis, MO);

	fprintf_P(stderr, PSTR(" %c axis mode: %S\n"), axis_char,
					  (PGM_P)pgm_read_word(&cfgPAM[(int)axis_mode]));
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

/******************************
 ***** Config Unit Tests ******
 ******************************/

#ifdef __UNIT_TESTS

//---- prototypes and statics -----
void _test_cfg_init(void);
void _test_cfg_write_test_pattern(void);
void _test_cfg_get_setting(void);
void _test_cfg_NVM_operations(void);
void _test_cfg_config_parser(void);
static struct cfgSetting *_cfg_make_setting(const int8_t axis, const int8_t mnemonic, const double value);
static char testblock[40];	// scope this so you can debug it

//---- entry point ----
void cfg_unit_tests()
{
//	_test_cfg_init();	// comment out what you dont's want to test now
//	_test_cfg_write_test_pattern();
//	_test_cfg_get_setting();
//	_test_cfg_NVM_operations();
//	_test_cfg_config_parser();
}

/*---- test routines ----
 * Assumes cfgList array has already been set up
 * To test inits (by simulation) you MUST use nnvm as the xmega 
 * simulator2 does not support EEPROM simulation (nnvm is a RAM block)
 * Uncomment (define) __NNVM in xmega_eeprom.h
 */

void _test_cfg_init()
{
	struct cfgSetting *s;

	// The first init is done by the init system. 
	// If nnvm is used it will perform an uninitialized reset. 
	// Trace this at cfg_init().

	// The second init (below) is an initialized "EEPROM" at current rev . 
	s = _cfg_make_setting(X, MA, 4);
	_cfg_put_NVM_setting(s);
	cfg_init();

	// The third init is an initialized but out-of-rev "EEPROM"
	s = _cfg_make_setting(-1, V_, 4);
	_cfg_put_NVM_setting(s);
	cfg_init();
}

void _test_cfg_write_test_pattern()
{
	uint16_t i;
	struct cfgSetting *s;

	for (i=0; i<COUNT_SETTINGS; i++) {
		s = _cfg_make_setting(i, i, i);
		_cfg_put_NVM_setting(s);
	}
	cfg_dump_NVM(0,COUNT_SETTINGS,PSTR("Show NVM Test Pattern"));
}

void _test_cfg_get_setting()
{
	struct cfgSetting *s;
	uint8_t fiddle=0;

	s = _cfg_get_setting_by_key(NON_AXIS, P_);
	fiddle +=1;							// let s settle so we can see it
	s = _cfg_get_setting_by_key(NON_AXIS, V_);
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
