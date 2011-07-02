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
 */

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

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
#define CONVERSION_REQUIRED (s->apply != s->appin)

struct cfgSetting {				// don't mess with the order of the struct
	int8_t axis;				// key: axis 0-N, or -1 if non-axis setting
	int8_t mnemonic;			// key: numeric token for mnemonic
	char * PROGMEM fmt_mm;		// ptr to format string for mm display
	char * PROGMEM fmt_in;		// ptr to format string for in display 
	void (*apply)(struct cfgSetting *);// mm apply function
	void (*appin)(struct cfgSetting *);// in apply function
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
static void _cfg_print_step_rates(const int8_t axis);
static void _print_NVM_record(const int16_t record_number, const int8_t *nvm_record);
static int8_t _cfg_get_axis_num(const char c);
static int8_t _cfg_get_mnemonic_num(const char *mnemonic);
static char *_cfg_get_mnemonic_string(const int8_t mnum);
static char _cfg_get_axis_char(const int8_t axis);
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
	MO, SR, FR, TR, TM, 		
	SA, RA, MI, PO, PW,
	LI, HE, HR, HC, HO,
	HB,							// ---> per-axis count 16 (adjust #define)

	// non-axis settings
	GU, GL, GP, GD, EA, 
	JM, JU, JL, MM, MT, 
	M1, M2, M3, M4, HM, 
	EC, IC, IL, EX, EE,			// ---> non-axis count 20 (adjust #define)

	_P							// profile trailer record. Must be last
};

char *mnemonics[] = { "P_",	"V_",// must align with cfgMnemonics
	"MO", "SR", "FR", "TR", "TM", 
	"SA", "RA", "MI", "PO", "PW", 
	"LI", "HE", "HR", "HC", "HO", 
	"HB",
	"GU", "GL", "GP", "GD", "EA", 
	"JM", "JU", "JL", "MM", "MT", 
	"M1", "M2", "M3", "M4", "HM", 
	"EC", "IC", "IL", "EX", "EE",
	"_P"
};
#define MAX_MNEMONIC _P

#define COUNT_AXES AXES				// count of supported axes
#define COUNT_PER_AXIS 16			// count of per-axis settings
#define COUNT_NON_AXIS 20			// count of non-axis settings
#define COUNT_HDR_TRLR 3			// the 2 headers and 1 trailer 
#define COUNT_SETTINGS ((COUNT_PER_AXIS * COUNT_AXES) + COUNT_NON_AXIS + COUNT_HDR_TRLR)

//--- APPLY FUNCTION PROTOTYPES ------------------------------------------

// mm mode functions are "_applyXX"
static void _apply00(struct cfgSetting *s);	// null function

static void _applyMO(struct cfgSetting *s);
static void _applySR(struct cfgSetting *s);
static void _applyFR(struct cfgSetting *s);
static void _applyTR(struct cfgSetting *s);
static void _applyTM(struct cfgSetting *s);
static void _applySA(struct cfgSetting *s);
static void _applyRA(struct cfgSetting *s);
static void _applyMI(struct cfgSetting *s);
static void _applyPO(struct cfgSetting *s);
static void _applyPW(struct cfgSetting *s);
static void _applyLI(struct cfgSetting *s);
static void _applyHE(struct cfgSetting *s);
static void _applyHR(struct cfgSetting *s);
static void _applyHC(struct cfgSetting *s);
static void _applyHO(struct cfgSetting *s);
static void _applyHB(struct cfgSetting *s);

static void _applyGC(struct cfgSetting *s); // common gcode function
static void _applyEA(struct cfgSetting *s);
static void _applyJM(struct cfgSetting *s);
static void _applyJU(struct cfgSetting *s);
static void _applyJL(struct cfgSetting *s);
static void _applyMM(struct cfgSetting *s);
static void _applyMT(struct cfgSetting *s);
static void _applyM1(struct cfgSetting *s);
static void _applyM2(struct cfgSetting *s);
static void _applyM3(struct cfgSetting *s);
static void _applyM4(struct cfgSetting *s);
static void _applyHM(struct cfgSetting *s);

static void _applyEC(struct cfgSetting *s);
static void _applyIC(struct cfgSetting *s);
static void _applyIL(struct cfgSetting *s);
static void _applyEX(struct cfgSetting *s);
static void _applyEE(struct cfgSetting *s);

// inches mode functions are "_appinXX"
static void _appinSR(struct cfgSetting *s);
static void _appinFR(struct cfgSetting *s);
static void _appinTR(struct cfgSetting *s);
static void _appinTM(struct cfgSetting *s);
static void _appinRA(struct cfgSetting *s);
static void _appinHR(struct cfgSetting *s);
static void _appinHC(struct cfgSetting *s);
static void _appinHO(struct cfgSetting *s);
static void _appinHB(struct cfgSetting *s);

static void _appinMM(struct cfgSetting *s);
static void _appinJM(struct cfgSetting *s);


//--- DISPLAY FORMAT STRINGS ---------------------------------------------

char fmtP_[] PROGMEM = "Profile %1.2f [%s%1.2f]\n";	// print profile number
char fmtV_[] PROGMEM = "Version %1.2f [%s%1.2f]\n";	// print version number

// mm mode per-axis settings
char fmtSR[] PROGMEM = "Seek rate          %5.0f mm/min       $%c%s%1.0f\n";// max axis seek rate in units per minute
char fmtFR[] PROGMEM = "Feed rate          %5.0f mm/min       $%c%s%1.0f\n";// max axis feed rate in units per minute
char fmtTM[] PROGMEM = "Travel max         %5.0f mm           $%c%s%1.0f\n";// maximum dimension on this axis
char fmtHR[] PROGMEM = "Homing seek rate   %5.0f mm/min       $%c%s%1.0f\n";// first pass homing speed
char fmtHC[] PROGMEM = "Homing close rate  %5.0f mm/min       $%c%s%1.0f\n";// second pass homing speed
char fmtHO[] PROGMEM = "Homing offset      %5.0f mm           $%c%s%1.0f\n";// offset for min limit switch
char fmtHB[] PROGMEM = "Homing backoff     %5.0f mm           $%c%s%1.0f\n";// homing backoff distance
char fmtTR[] PROGMEM = "Travel/rev      %8.2f mm           $%c%s%1.0f\n";	// in mm per revolution
char fmtRA[] PROGMEM = "Axis radius     %8.3f mm           $%c%s%1.3f\n";	// for feed rate computation (valid for A,B and C)
char fmtSA[] PROGMEM = "Step angle         %5.3f degrees      $%c%s%1.2f\n";// in degrees per step 
char fmtMO[] PROGMEM = "Axis mode          %5.0f [0,1]        $%c%s%1.0f\n";// 0=normal, 1=coordinated 
char fmtMI[] PROGMEM = "Microsteps         %5.0f [1,2,4,8]    $%c%s%1.0f\n";// [1248]; [0] may be morphing (future)
char fmtPO[] PROGMEM = "Motor polarity     %5.0f [0,1]        $%c%s%1.0f\n";// 0=normal, 1=inverted
char fmtPW[] PROGMEM = "Power mgmt mode    %5.0f [0,1]        $%c%s%1.0f\n";// 1=zero power idle
char fmtLI[] PROGMEM = "Limit switch mode  %5.0f [0,1]        $%c%s%1.0f\n";// 0=off, 1=on
char fmtHE[] PROGMEM = "Homing enabled     %5.0f [0,1]        $%c%s%1.0f\n";// 1=enable homing for this axis

// mm mode non-axis settings	(see version 311.x for spindle and tool)
char fmtGU[] PROGMEM = "Gcode: Units {G20,G21}         %2.0f [20,21]      $%s%1.0f\n";
char fmtGL[] PROGMEM = "Gcode: Plane {G17,G18,G19}     %2.0f [17,18,19]   $%s%1.0f\n";
char fmtGP[] PROGMEM = "Gcode: Path  {G61,G61.1,G64} %3.1f [61,61.1,64] $%s%1.1f\n";
char fmtGD[] PROGMEM = "Gcode: Distance Mode {G90,G91} %2.0f [90,91]      $%s%1.0f\n";

char fmtEA[] PROGMEM = "Enable Acceleration             %1.0f [0,1]        $%s%1.0f\n";
char fmtJM[] PROGMEM = "Max linear jerk        %10.0f mm/min^3     $%s%1.0f\n";
char fmtJU[] PROGMEM = "Angular jerk upper thresh   %5.3f              $%s%1.0f\n";
char fmtJL[] PROGMEM = "Angular jerk lower thresh   %5.3f              $%s%1.0f\n";
char fmtMM[] PROGMEM = "Min segment length          %5.3f mm           $%s%1.0f\n";
char fmtMT[] PROGMEM = "Min segment time            %5.0f uSec         $%s%1.0f\n";

char fmtM1[] PROGMEM = "Map motor 1 to axis             %1.0f [0-3]        $%s%1.0f\n";
char fmtM2[] PROGMEM = "Map motor 2 to axis             %1.0f [0-3]        $%s%1.0f\n";
char fmtM3[] PROGMEM = "Map motor 3 to axis             %1.0f [0-3]        $%s%1.0f\n";
char fmtM4[] PROGMEM = "Map motor 4 to axis             %1.0f [0-3]        $%s%1.0f\n";

char fmtHM[] PROGMEM = "Homing mode                     %1.0f [0,1]        $%s%1.0f\n";

char fmtEC[] PROGMEM = "Convert LF to CR LF (outgoing)  %1.0f [0,1]        $%s%1.0f\n";
char fmtIC[] PROGMEM = "Ignore Incoming CR              %1.0f [0,1]        $%s%1.0f\n";
char fmtIL[] PROGMEM = "Ignore Incoming LF              %1.0f [0,1]        $%s%1.0f\n";
char fmtEX[] PROGMEM = "Enable Xon/Xoff Flow Control    %1.0f [0,1]        $%s%1.0f\n";
char fmtEE[] PROGMEM = "Enable Echo                     %1.0f [0,1]        $%s%1.0f\n";

// inches mode settings
char finSR[] PROGMEM = "Seek rate          %5.2f in/min       $%c%s%1.2f\n";// max axis seek rate in units per minute
char finFR[] PROGMEM = "Feed rate          %5.2f in/min       $%c%s%1.2f\n";// max axis feed rate in units per minute
char finTM[] PROGMEM = "Travel max         %5.2f inches       $%c%s%1.2f\n";// maximum dimension on this axis
char finHR[] PROGMEM = "Homing seek rate   %5.2f in/min       $%c%s%1.2f\n";// first pass homing speed
char finHC[] PROGMEM = "Homing close rate  %5.2f in/min       $%c%s%1.2f\n";// second pass homing speed
char finHO[] PROGMEM = "Homing offset      %5.2f inches       $%c%s%1.2f\n";// offset for min limit switch
char finHB[] PROGMEM = "Homing backoff     %5.2f inches       $%c%s%1.2f\n";// homing backoff distance
char finTR[] PROGMEM = "Travel/rev      %8.3f inches       $%c%s%1.3f\n";	// in mm per revolution
char finRA[] PROGMEM = "Axis radius     %8.3f inches       $%c%s%1.3f\n";	// for feed rate computation (valid for A,B and C)
char finMM[] PROGMEM = "Min segment length         %5.4f inches       $%s%1.4f\n";
char finJM[] PROGMEM = "Max linear jerk        %10.0f in/min^3     $%s%1.0f\n";


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
	{ X, SR, fmtSR, finSR, _applySR, _appinSR, (double) X_SEEK_RATE },
	{ X, FR, fmtFR, finFR, _applyFR, _appinFR, (double) X_FEED_RATE },
	{ X, TR, fmtTR, finTR, _applyTR, _appinTR, (double) X_TRAVEL_PER_REV },
	{ X, TM, fmtTM, finTM, _applyTM, _appinTM, (double) X_TRAVEL_MAX },
	{ X, RA, fmtRA, finRA, _applyRA, _appinRA, (double) X_RADIUS },
	{ X, SA, fmtSA, fmtSA, _applySA, _applySA, (double) X_STEP_ANGLE },
	{ X, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) X_MICROSTEPS },
	{ X, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) X_POLARITY },
	{ X, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) X_POWER_MODE },
	{ X, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) X_LIMIT_MODE },
	{ X, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) X_HOMING_ENABLE },
	{ X, HR, fmtHR, finHR, _applyHR, _appinHR, (double) X_HOMING_SEEK_RATE },
	{ X, HC, fmtHC, finHC, _applyHC, _appinHC, (double) X_HOMING_CLOSE_RATE },
	{ X, HO, fmtHO, finHO, _applyHO, _appinHO, (double) X_HOMING_OFFSET },
	{ X, HB, fmtHB, finHB, _applyHB, _appinHB, (double) X_HOMING_BACKOFF },

	{ Y, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Y_AXIS_MODE },
	{ Y, SR, fmtSR, finSR, _applySR, _appinSR, (double) Y_SEEK_RATE },
	{ Y, FR, fmtFR, finFR, _applyFR, _appinFR, (double) Y_FEED_RATE },
	{ Y, TR, fmtTR, finTR, _applyTR, _appinTR, (double) Y_TRAVEL_PER_REV },
	{ Y, TM, fmtTM, finTM, _applyTM, _appinTM, (double) Y_TRAVEL_MAX },
	{ Y, RA, fmtRA, finRA, _applyRA, _appinRA, (double) Y_RADIUS },
	{ Y, SA, fmtSA, fmtSA, _applySA, _applySA, (double) Y_STEP_ANGLE },
	{ Y, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) Y_MICROSTEPS },
	{ Y, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) Y_POLARITY },
	{ Y, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) Y_POWER_MODE },
	{ Y, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Y_LIMIT_MODE },
	{ Y, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Y_HOMING_ENABLE },
	{ Y, HR, fmtHR, finHR, _applyHR, _appinHR, (double) Y_HOMING_SEEK_RATE },
	{ Y, HC, fmtHC, finHC, _applyHC, _appinHC, (double) Y_HOMING_CLOSE_RATE },
	{ Y, HO, fmtHO, finHO, _applyHO, _appinHO, (double) Y_HOMING_OFFSET },
	{ Y, HB, fmtHB, finHB, _applyHB, _appinHB, (double) Y_HOMING_BACKOFF },

	{ Z, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Z_AXIS_MODE },
	{ Z, SR, fmtSR, finSR, _applySR, _appinSR, (double) Z_SEEK_RATE },
	{ Z, FR, fmtFR, finFR, _applyFR, _appinFR, (double) Z_FEED_RATE },
	{ Z, TR, fmtTR, finTR, _applyTR, _appinTR, (double) Z_TRAVEL_PER_REV },
	{ Z, TM, fmtTM, finTM, _applyTM, _appinTM, (double) Z_TRAVEL_MAX },
	{ Z, RA, fmtRA, finRA, _applyRA, _appinRA, (double) Z_RADIUS },
	{ Z, SA, fmtSA, fmtSA, _applySA, _applySA, (double) Z_STEP_ANGLE },
	{ Z, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) Z_MICROSTEPS },
	{ Z, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) Z_POLARITY },
	{ Z, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) Z_POWER_MODE },
	{ Z, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Z_LIMIT_MODE },
	{ Z, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Z_HOMING_ENABLE },
	{ Z, HR, fmtHR, finHR, _applyHR, _appinHR, (double) Z_HOMING_SEEK_RATE },
	{ Z, HC, fmtHC, finHC, _applyHC, _appinHC, (double) Z_HOMING_CLOSE_RATE },
	{ Z, HO, fmtHO, finHO, _applyHO, _appinHO, (double) Z_HOMING_OFFSET },
	{ Z, HB, fmtHB, finHB, _applyHB, _appinHB, (double) Z_HOMING_BACKOFF },

	{ X, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) A_AXIS_MODE },
	{ A, SR, fmtSR, finSR, _applySR, _appinSR, (double) A_SEEK_RATE },
	{ A, FR, fmtFR, finFR, _applyFR, _appinFR, (double) A_FEED_RATE },
	{ A, TR, fmtTR, finTR, _applyTR, _appinTR, (double) A_TRAVEL_PER_REV },
	{ A, TM, fmtTM, finTM, _applyTM, _appinTM, (double) A_TRAVEL_MAX },
	{ A, RA, fmtRA, finRA, _applyRA, _appinRA, (double) A_RADIUS },
	{ A, SA, fmtSA, fmtSA, _applySA, _applySA, (double) A_STEP_ANGLE },
	{ A, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) A_MICROSTEPS },
	{ A, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) A_POLARITY },
	{ A, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) A_POWER_MODE },
	{ A, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) A_LIMIT_MODE },
	{ A, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) A_HOMING_ENABLE },
	{ A, HR, fmtHR, finHR, _applyHR, _appinHR, (double) A_HOMING_SEEK_RATE },
	{ A, HC, fmtHC, finHC, _applyHC, _appinHC, (double) A_HOMING_CLOSE_RATE },
	{ A, HO, fmtHO, finHO, _applyHO, _appinHO, (double) A_HOMING_OFFSET },
	{ A, HB, fmtHB, finHB, _applyHB, _appinHB, (double) A_HOMING_BACKOFF },

	{ NON_AXIS, GU, fmtGU, fmtGU, _applyGC, _applyGC, (double) GCODE_UNITS },
	{ NON_AXIS, GL, fmtGL, fmtGL, _applyGC, _applyGC, (double) GCODE_PLANE },
	{ NON_AXIS, GP, fmtGP, fmtGP, _applyGC, _applyGC, (double) GCODE_PATH_CONTROL },
	{ NON_AXIS, GD, fmtGD, fmtGD, _applyGC, _applyGC, (double) GCODE_DISTANCE_MODE },

	{ NON_AXIS, EA, fmtEA, fmtEA, _applyEA, _applyEA, (double) ENABLE_ACCEL },
	{ NON_AXIS, JM, fmtJM, finJM, _applyJM, _appinJM, (double) MAX_LINEAR_JERK },
	{ NON_AXIS, JU, fmtJU, fmtJU, _applyJU, _applyJU, (double) ANGULAR_JERK_UPPER_THRESHOLD },
	{ NON_AXIS, JL, fmtJL, fmtJL, _applyJL, _applyJL, (double) ANGULAR_JERK_LOWER_THRESHOLD },
	{ NON_AXIS, MM, fmtMM, finMM, _applyMM, _appinMM, (double) MIN_SEGMENT_LENGTH },
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

//----- MM PER-AXIS APPLY FUNCTIONS -----
static void _applyMO(struct cfgSetting *s) { cfg.a[s->axis].axis_mode = (uint8_t)s->value; }
static void _applySR(struct cfgSetting *s) { cfg.a[s->axis].seek_rate = s->value;}
static void _applyFR(struct cfgSetting *s) { cfg.a[s->axis].feed_rate = s->value; }
static void _applyTM(struct cfgSetting *s) { cfg.a[s->axis].travel_max = s->value; }
static void _applyRA(struct cfgSetting *s) { cfg.a[s->axis].radius = s->value; }
static void _applyPW(struct cfgSetting *s) { cfg.a[s->axis].power_mode = (uint8_t)s->value;}
static void _applyLI(struct cfgSetting *s) { cfg.a[s->axis].limit_mode = (uint8_t)s->value;}
static void _applyHE(struct cfgSetting *s) { cfg.a[s->axis].homing_enable = (uint8_t)s->value;}
static void _applyHR(struct cfgSetting *s) { cfg.a[s->axis].homing_rate = s->value; }
static void _applyHC(struct cfgSetting *s) { cfg.a[s->axis].homing_close = s->value; }
static void _applyHO(struct cfgSetting *s) { cfg.a[s->axis].homing_offset = s->value; }
static void _applyHB(struct cfgSetting *s) { cfg.a[s->axis].homing_backoff = s->value; }

static void _applyTR(struct cfgSetting *s) { 
	cfg.a[s->axis].travel_rev = s->value;
	_cfg_set_steps_per_unit(s); 
}

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

//----- GCODE DEFAULT APPLY FUNCTIONS -----
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

//----- MM NON-AXIS APPLY FUNCTIONS -----
static void _applyEA(struct cfgSetting *s) { cfg.accel_enabled = (uint8_t)s->value;}
static void _applyJM(struct cfgSetting *s) { cfg.max_linear_jerk = s->value; }
static void _applyJU(struct cfgSetting *s) { cfg.angular_jerk_upper = s->value; }
static void _applyJL(struct cfgSetting *s) { cfg.angular_jerk_lower = s->value; }
static void _applyMM(struct cfgSetting *s) { cfg.min_segment_len = s->value; }
static void _applyMT(struct cfgSetting *s) { cfg.min_segment_time = s->value; }

static void _applyM1(struct cfgSetting *s) { cfg.motor_map[MOTOR_1] = (uint8_t)s->value;}
static void _applyM2(struct cfgSetting *s) { cfg.motor_map[MOTOR_2] = (uint8_t)s->value;}
static void _applyM3(struct cfgSetting *s) { cfg.motor_map[MOTOR_3] = (uint8_t)s->value;}
static void _applyM4(struct cfgSetting *s) { cfg.motor_map[MOTOR_4] = (uint8_t)s->value;}

static void _applyHM(struct cfgSetting *s) { cfg.homing_mode = (uint8_t)s->value;}


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

//----- INCHES MODE NON-AXIS APPLY FUNCTIONS -----
static void _appinSR(struct cfgSetting *s) { s->value *= 25.4; _applySR(s); }
static void _appinFR(struct cfgSetting *s) { s->value *= 25.4; _applyFR(s); }
static void _appinTR(struct cfgSetting *s) { s->value *= 25.4; _applyTR(s); }
static void _appinTM(struct cfgSetting *s) { s->value *= 25.4; _applyTM(s); }
static void _appinRA(struct cfgSetting *s) { s->value *= 25.4; _applyRA(s); }
static void _appinHR(struct cfgSetting *s) { s->value *= 25.4; _applyHR(s); }
static void _appinHC(struct cfgSetting *s) { s->value *= 25.4; _applyHC(s); }
static void _appinHO(struct cfgSetting *s) { s->value *= 25.4; _applyHO(s); }
static void _appinHB(struct cfgSetting *s) { s->value *= 25.4; _applyHB(s); }
static void _appinMM(struct cfgSetting *s) { s->value *= 25.4; _applyMM(s); }
static void _appinJM(struct cfgSetting *s) { s->value *= 25.4; _applyJM(s); }


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

#ifdef __NO_EEPROM		// cutout for debug simulation
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

	s = _cfg_get_setting_by_key(NON_AXIS, GU);
	s->apply(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GL);
	s->apply(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GP);
	s->apply(s);
	s = _cfg_get_setting_by_key(NON_AXIS, GD);
	s->apply(s);
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
 * _cfg_get_axis_num()  		- return axis number from axis char
 * _cfg_get_axis_char() 		- return axis char from axis number
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

static char _cfg_get_axis_char(const int8_t axis)
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
		s->apply(s);	// call the mm mode apply function
	} else {
		s->appin(s);	// call the inches mode apply function
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
							_cfg_get_axis_char(nvm_record[0]), 
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
	uint16_t i;

	switch(c) {
		case '$': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				_cfg_print_setting(&cfgList[i]);
			}
			return;
		}
		case 'X': case 'Y': case 'Z': case 'A': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if (cfgList[i].axis == axis) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			_cfg_print_step_rates(axis);
			return;
		}
		case 'H': { 
			cfg_print_config_help();
			return;
		}
		default: {
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
	if ((s->mnemonic == RA) && (s->axis <A)) {
		return;
	}

	if (cm_get_inches_mode() == FALSE) {// mm mode displays
		if (s->axis != NON_AXIS) {
			fprintf_P(stderr, PSTR("%c axis - "), 
						_cfg_get_axis_char(s->axis));
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, 
						_cfg_get_axis_char(s->axis), 
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
						_cfg_get_axis_char(s->axis));
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, 
						_cfg_get_axis_char(s->axis), 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		} else {
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, 
						_cfg_get_mnemonic_string(s->mnemonic), value);
		}
	}
}

/*
 * _cfg_print_step_rates() - print step rates resulting from your settings
 */
static void _cfg_print_step_rates(const int8_t axis)
{
	char axis_char = _cfg_get_axis_char(axis);
	double step_angle = _cfg_get_setting_value_by_key(axis, SA);
	double travel_rev = _cfg_get_setting_value_by_key(axis, TR);
	double seek_rate = _cfg_get_setting_value_by_key(axis, SR);
	double feed_rate = _cfg_get_setting_value_by_key(axis, FR);
	double radius = _cfg_get_setting_value_by_key(axis, RA);
	double seek_steps = (seek_rate / 60 / travel_rev) * (360 / step_angle);
	double feed_steps = (feed_rate / 60 / travel_rev) * (360 / step_angle);
	if (axis > Z) {	// if rotary axis
		seek_steps = seek_steps / (radius / RADIAN);
		feed_steps = feed_steps / (radius / RADIAN);
	}
	fprintf_P(stderr, PSTR("Your %c axis settings translate to:\n"), axis_char);
	fprintf_P(stderr, PSTR(" Max %c seek steps/sec       %5.0f\n"), axis_char, seek_steps);
	fprintf_P(stderr, PSTR(" Max %c feed steps/sec       %5.0f\n"), axis_char, feed_steps);
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
