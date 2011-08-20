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
 *
 *  - Settings are specified as a 2 character mnemonic preceded by a group.
 *	  Groups are things like a axis specifier (e.g. Y), a motor specifier,
 *	  (e.g. 2), or the general group - which collects non-axis and 
 *	  non-motor settings 
 * *
 *	- Mnemonics are 2 char ASCII strings and can't start w\an axis name
 *	  or a motor number or PWM number (joint number)
 *		  - these are off limits for 1st chars: X,Y,Z,A,B,C,U,V,W
 *		  - these are also off limits: 			1,2,3,4,5,6,7,8,9,0
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
/*
  Still to do:
	- Steps per second - used travel per rev - but needs to come from the associated axis
 */

#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "util.h"
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
#include "help.h"

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
	int8_t grp;					// configuration group (see cfgGroups)
	int8_t mnem;				// key: numeric token for mnemonic
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
static void _cfg_print_rate_advisories(const int8_t grp, const char grp_char);
static void _cfg_print_axis_advisories(const int8_t grp, const char grp_char);
static int8_t _cfg_get_grp_num(const char c);
static char _cfg_get_grp_char(const int8_t grp);
static int8_t _cfg_get_mnemonic_num(const char *mnemonic);
static char *_cfg_get_mnemonic_string(const int8_t mnum);
static void _cfg_set_steps_per_unit(const struct cfgSetting *s);
static int16_t _cfg_get_setting_index(const int8_t grp, const int8_t mnem);
static double _cfg_get_setting_value_by_key(const int8_t grp, const int8_t mnem);
static struct cfgSetting *_cfg_get_setting_by_index(const uint16_t index);
static struct cfgSetting *_cfg_get_setting_by_key(const int8_t grp, const int8_t mnem);
static struct cfgSetting *_cfg_put_setting(const struct cfgSetting *s);
static void _cfg_apply_setting(struct cfgSetting *s);
static struct cfgSetting *_cfg_get_NVM_setting_by_index(const uint16_t index);
static uint8_t _cfg_put_NVM_setting(const struct cfgSetting *s);
//UNUSED static void _print_NVM_record(const int16_t record_number, const int8_t *nvm_record);
//UNUSED static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t axis, const int8_t mnemonic);

/************************************************************************
 *** START SETTING-SPECIFIC REGION **************************************
 ************************************************************************/

/*--- GROUPS, MNEMONICS AND COUNTS --------------------------------------
 * All items in this section MUST stay in alignment. 
 * Be sure to confirm or adjust COUNT_SETTINGS is you change mnemonics.
 * or you will get "warning: excess elements in array initializer" error
 * Profile and version records must be first. Must end with the trailer.
 * Or worse - it will just stop working after you make the changes.
 * See DISPLAY FORMAT STRINGS for explanations of what the mnemonics mean
 */
enum cfgGroups {
	CFG_GENERAL = -1,				// general groups are negative
	CFG_X,						// CFG_X thru CFG_C align w/tgAxisNum...
	CFG_Y,						// ...X=0, Y=1, etc.
	CFG_Z,
	CFG_A,
	CFG_B,
	CFG_C,
	CFG_M1,						// Stepper motors 1 - 4
	CFG_M2,
	CFG_M3,
	CFG_M4,
	CFG_PWM1,					// PWM channels are known as 5 and 6
	CFG_PWM2
};
#define CFG_MOTOR_BASE CFG_M1	// subtract base to get proper array index
#define CFG_PWM_BASE CFG_PWM1	// subtract base to get proper array index
#define AIDX(g) (g)				// convert grp to axis array index
#define MIDX(g) (g-CFG_M1)		// convert grp to motor array index
#define PIDX(g) (g-CFG_PWM1)	// convert grp to pwm array index

enum cfgMnemonics { P_, V_,		// <-- profile and version records

	// axis settings
	MO, SR, FR, TH, TS, 
	RA, LI, HE, HR, HC, 
	HO, HB,						// 12 axis settings (adjust #define)

	// motor settings
	MA, SA, TR, MI, PO, 
	PW, 						// 6 motor settings (adjust #define)

	// PWM settings
								// 0 PWM settings

	// general settings
	GU, GL, GP, GD, EA, 
	JL, MM, MT, HM, EC, 
	IC, IL, EX, EE,				// 15 general settings (adjust #define)

	_P							// profile trailer record. Must be last
};

char *mnemonics[] = { "P_",	"V_",// must align with cfgMnemonics, above
	"MO", "SR", "FR", "TH", "TS", 
	"RA", "LI", "HE", "HR", "HC", 
	"HO", "HB",
	"MA", "SA", "TR", "MI", "PO", 
	"PW", 
	"GU", "GL", "GP", "GD", "EA", 
	"JL", "MM", "MT", "HM", "EC", 
	"IC", "IL", "EX", "EE",
	"_P"
};
#define MAX_MNEMONIC _P

#define COUNT_AXES AXES				// count of supported axes
#define COUNT_MOTORS MOTORS			// count of supported motors
#define COUNT_PWM 0					// count of PWM channels
#define COUNT_AXIS_SETTINGS 12
#define COUNT_MOTOR_SETTINGS 6
#define COUNT_PWM_SETTINGS 0
#define COUNT_GENERAL 14
#define COUNT_HDR_TRLR 3			// the 2 headers and 1 trailer 
#define COUNT_SETTINGS ((COUNT_AXES * COUNT_AXIS_SETTINGS) + \
						(COUNT_MOTORS * COUNT_MOTOR_SETTINGS) + \
						(COUNT_PWM * COUNT_PWM_SETTINGS) + \
						 COUNT_GENERAL + COUNT_HDR_TRLR)

//--- "_APPLY()" FUNCTION PROTOTYPES ------------------------------------------

// axis settings - unit agnostic
static void _apply00(struct cfgSetting *s);	// null function
static void _applyMO(struct cfgSetting *s);
static void _applyLI(struct cfgSetting *s);
static void _applyHE(struct cfgSetting *s);

// axis settings - mm units
static void _appmmSR(struct cfgSetting *s);
static void _appmmFR(struct cfgSetting *s);
static void _appmmTH(struct cfgSetting *s);
static void _appmmTS(struct cfgSetting *s);
static void _appmmRA(struct cfgSetting *s);
static void _appmmHR(struct cfgSetting *s);
static void _appmmHC(struct cfgSetting *s);
static void _appmmHO(struct cfgSetting *s);
static void _appmmHB(struct cfgSetting *s);

// axis settings - inch units
static void _appinSR(struct cfgSetting *s);
static void _appinFR(struct cfgSetting *s);
static void _appinTH(struct cfgSetting *s);
static void _appinTS(struct cfgSetting *s);
static void _appinRA(struct cfgSetting *s);
static void _appinHR(struct cfgSetting *s);
static void _appinHC(struct cfgSetting *s);
static void _appinHO(struct cfgSetting *s);
static void _appinHB(struct cfgSetting *s);


// axis settings - degree units
/* these are pass thrus - use macros instead
static void _appdgSR(struct cfgSetting *s);
static void _appdgFR(struct cfgSetting *s);
static void _appdgTH(struct cfgSetting *s);
static void _appdgTS(struct cfgSetting *s);
static void _appdgHR(struct cfgSetting *s);
static void _appdgHC(struct cfgSetting *s);
static void _appdgHO(struct cfgSetting *s);
static void _appdgHB(struct cfgSetting *s);
*/
#define _appdgSR _appmmSR
#define _appdgFR _appmmFR
#define _appdgTH _appmmTH
#define _appdgTS _appmmTS
#define _appdgHR _appmmHR
#define _appdgHC _appmmHC
#define _appdgHO _appmmHO
#define _appdgHB _appmmHB

// motor settings
static void _applyMA(struct cfgSetting *s);	// unit agnostic settings
static void _applySA(struct cfgSetting *s);
static void _applyMI(struct cfgSetting *s);
static void _applyPO(struct cfgSetting *s);
static void _applyPW(struct cfgSetting *s);

static void _appmmTR(struct cfgSetting *s);	// settings using units
static void _appinTR(struct cfgSetting *s);
//static void _appdgTR(struct cfgSetting *s);
#define _appdgTR _appmmTR

// PWM settings - unit agnostic

// general settings - mm units
static void _appmmJL(struct cfgSetting *s);
static void _appmmMM(struct cfgSetting *s);

// general settings - inch units
static void _appinJL(struct cfgSetting *s);
static void _appinMM(struct cfgSetting *s);

// general settings - unit agnostic
static void _applyGC(struct cfgSetting *s);
static void _applyEA(struct cfgSetting *s);
static void _applyMT(struct cfgSetting *s);
static void _applyHM(struct cfgSetting *s);

static void _applyEC(struct cfgSetting *s);
static void _applyIC(struct cfgSetting *s);
static void _applyIL(struct cfgSetting *s);
static void _applyEX(struct cfgSetting *s);
static void _applyEE(struct cfgSetting *s);

//--- DISPLAY FORMAT STRINGS ---------------------------------------------
// There are 4 possibilities:			   naming convention used
//	 agnostic 	- applies to all units		fmtXX, _applyXX 
//	 mm 		- applies to mm units		fmmXX, _appmmXX
//	 inches 	- applies to inches units	finXX, _appinXX
//	 degrees	- applies to degrees units	fdgXX, _appdgXX

char fmtP_[] PROGMEM = "Profile %1.2f [%s%1.2f]\n";	// print profile number
char fmtV_[] PROGMEM = "Version %1.2f [%s%1.2f]\n";	// print version number

// axis settings - unit agnostic (applies to all axes)
char fmtMO[] PROGMEM = "Axis mode          %5.0f [0-10]       $%c%s%1.0f\n";
char fmtLI[] PROGMEM = "Limit switch mode  %5.0f [0,1]        $%c%s%1.0f\n";
char fmtHE[] PROGMEM = "Homing enabled     %5.0f [0,1]        $%c%s%1.0f\n";

// axis settings - mm mode (applies to XYZ axes)
char fmmSR[] PROGMEM = "Seek rate       %8.0f mm/min       $%c%s%1.0f\n";
char fmmFR[] PROGMEM = "Feed rate       %8.0f mm/min       $%c%s%1.0f\n";
char fmmTH[] PROGMEM = "Travel hard limit  %5.0f mm           $%c%s%1.0f\n";
char fmmTS[] PROGMEM = "Travel soft limit  %5.0f mm           $%c%s%1.0f\n";
char fmmHR[] PROGMEM = "Homing seek rate   %5.0f mm/min       $%c%s%1.0f\n";
char fmmHC[] PROGMEM = "Homing close rate  %5.0f mm/min       $%c%s%1.0f\n";
char fmmHO[] PROGMEM = "Homing offset      %5.0f mm           $%c%s%1.0f\n";
char fmmHB[] PROGMEM = "Homing backoff     %5.0f mm           $%c%s%1.0f\n";
char fmmRA[] PROGMEM = "Radius value    %8.3f mm           $%c%s%1.3f\n";

// axis settings - inches mode (applies to XYZ axes)
char finSR[] PROGMEM = "Seek rate          %5.2f in/min       $%c%s%1.2f\n";
char finFR[] PROGMEM = "Feed rate          %5.2f in/min       $%c%s%1.2f\n";
char finTH[] PROGMEM = "Travel hard limit  %5.0f inches       $%c%s%1.0f\n";
char finTS[] PROGMEM = "Travel soft limit  %5.0f inches       $%c%s%1.0f\n";
char finHR[] PROGMEM = "Homing seek rate  %6.2f in/min       $%c%s%1.2f\n";
char finHC[] PROGMEM = "Homing close rate %6.2f in/min       $%c%s%1.2f\n";
char finHO[] PROGMEM = "Homing offset      %5.2f inches       $%c%s%1.2f\n";
char finHB[] PROGMEM = "Homing backoff     %5.2f inches       $%c%s%1.2f\n";
char finRA[] PROGMEM = "Radius value    %8.3f inches       $%c%s%1.3f\n";

// axis settings - degrees mode (applies to ABC axes)
char fdgSR[] PROGMEM = "Seek rate       %8.0f deg/min      $%c%s%1.0f\n";
char fdgFR[] PROGMEM = "Feed rate       %8.0f deg/min      $%c%s%1.0f\n";
char fdgTH[] PROGMEM = "Travel hard limit  %5.0f degrees      $%c%s%1.0f\n";
char fdgTS[] PROGMEM = "Travel soft limit  %5.0f degrees      $%c%s%1.0f\n";
char fdgHR[] PROGMEM = "Homing seek rate   %5.0f deg/min      $%c%s%1.2f\n";
char fdgHC[] PROGMEM = "Homing close rate  %5.0f deg/min      $%c%s%1.2f\n";
char fdgHO[] PROGMEM = "Homing offset      %5.0f degrees      $%c%s%1.2f\n";
char fdgHB[] PROGMEM = "Homing backoff     %5.0f degrees      $%c%s%1.2f\n";


// motor settings
char fmtMA[] PROGMEM = "Mapped to axis        %1.0f [0=X,1=Y...] $%c%s%1.0f\n";
char fmtSA[] PROGMEM = "Step angle        %5.2f degrees      $%c%s%1.2f\n";
char fmtMI[] PROGMEM = "Microsteps        %5.0f [1,2,4,8]    $%c%s%1.0f\n";
char fmtPO[] PROGMEM = "Motor polarity    %5.0f [0,1]        $%c%s%1.0f\n";
char fmtPW[] PROGMEM = "Power mgmt mode   %5.0f [0,1]        $%c%s%1.0f\n";

char fmmTR[] PROGMEM = "Travel/rev     %8.2f mm           $%c%s%1.2f\n";
char finTR[] PROGMEM = "Travel/rev     %8.3f inches       $%c%s%1.3f\n";
char fdgTR[] PROGMEM = "Travel/rev     %8.2f degrees      $%c%s%1.2f\n";

// PWM settings

// general settings - unit agnostic  (NOTE: removed spindle and tool in version 311)
char fmtGU[] PROGMEM = "Gcode: Units {G20,G21}         %2.0f [20,21]      $%s%1.0f\n";
char fmtGL[] PROGMEM = "Gcode: Plane {G17,G18,G19}     %2.0f [17,18,19]   $%s%1.0f\n";
char fmtGP[] PROGMEM = "Gcode: Path  {G61,G61.1,G64} %3.1f [61,61.1,64] $%s%1.1f\n";
char fmtGD[] PROGMEM = "Gcode: Distance Mode {G90,G91} %2.0f [90,91]      $%s%1.0f\n";

char fmtHM[] PROGMEM = "Homing mode                     %1.0f [0,1]        $%s%1.0f\n";
char fmtEA[] PROGMEM = "Enable Acceleration             %1.0f [0,1]        $%s%1.0f\n";
char fmtJR[] PROGMEM = "Rotary jerk maximum    %10.0f deg/min^3     $%s%1.0f\n";
char fmtMT[] PROGMEM = "Min segment time            %5.0f uSec         $%s%1.0f\n";

// general settings - with units

char fmmMM[] PROGMEM = "Min segment length          %5.3f mm           $%s%1.4f\n";
char finMM[] PROGMEM = "Min segment length         %5.4f inches       $%s%1.4f\n";

char fmmJL[] PROGMEM = "Linear jerk maximum    %10.0f mm/min^3     $%s%1.0f\n";
char finJL[] PROGMEM = "Linear jerk maximum    %10.0f in/min^3     $%s%1.0f\n";

// communications settings (part of general) 
char fmtEC[] PROGMEM = "Convert LF to CR LF (outgoing)  %1.0f [0,1]        $%s%1.0f\n";
char fmtIC[] PROGMEM = "Ignore Incoming CR              %1.0f [0,1]        $%s%1.0f\n";
char fmtIL[] PROGMEM = "Ignore Incoming LF              %1.0f [0,1]        $%s%1.0f\n";
char fmtEX[] PROGMEM = "Enable Xon/Xoff Flow Control    %1.0f [0,1]        $%s%1.0f\n";
char fmtEE[] PROGMEM = "Enable Echo                     %1.0f [0,1]        $%s%1.0f\n";


/*---- SETTING LIST STRUCTURE --------------------------------------------
 *
 *	Initialize all settings structs and defaults
 *
 * The cfgList struct below is a rather brute-force way of handling the 
 *	cfgSetting structs and their initialization. Each struct is 14 bytes
 *	and there are quite a lot of them. I'm not going to worry about further 
 *	memory optimization at this point, but could be achieved with a combined
 *	"do-all" print and apply function (which I'm told is an "anti-pattern").
 */

struct cfgSetting cfgList[COUNT_SETTINGS] = { 
	// starting version.profile record - must be first. DOnt' mess with these.
	{ CFG_GENERAL, P_, fmtP_, fmtP_, _apply00, _apply00,(double) CFG_PROFILE },
	{ CFG_GENERAL, V_, fmtV_, fmtV_, _apply00, _apply00, (double) CFG_VERSION },

	{ CFG_M1, MA, fmtMA, fmtMA, _applyMA, _applyMA, (double) M1_MOTOR_MAP },
	{ CFG_M1, SA, fmtSA, fmtSA, _applySA, _applySA, (double) M1_STEP_ANGLE },
	{ CFG_M1, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) M1_TRAVEL_PER_REV },
	{ CFG_M1, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) M1_MICROSTEPS },
	{ CFG_M1, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) M1_POLARITY },
	{ CFG_M1, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) M1_POWER_MODE },

	{ CFG_M2, MA, fmtMA, fmtMA, _applyMA, _applyMA, (double) M2_MOTOR_MAP },
	{ CFG_M2, SA, fmtSA, fmtSA, _applySA, _applySA, (double) M2_STEP_ANGLE },
	{ CFG_M2, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) M2_TRAVEL_PER_REV },
	{ CFG_M2, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) M2_MICROSTEPS },
	{ CFG_M2, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) M2_POLARITY },
	{ CFG_M2, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) M2_POWER_MODE },

	{ CFG_M3, MA, fmtMA, fmtMA, _applyMA, _applyMA, (double) M3_MOTOR_MAP },
	{ CFG_M3, SA, fmtSA, fmtSA, _applySA, _applySA, (double) M3_STEP_ANGLE },
	{ CFG_M3, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) M3_TRAVEL_PER_REV },
	{ CFG_M3, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) M3_MICROSTEPS },
	{ CFG_M3, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) M3_POLARITY },
	{ CFG_M3, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) M3_POWER_MODE },

	{ CFG_M4, MA, fmtMA, fmtMA, _applyMA, _applyMA, (double) M4_MOTOR_MAP },
	{ CFG_M4, SA, fmtSA, fmtSA, _applySA, _applySA, (double) M4_STEP_ANGLE },
	{ CFG_M4, TR, fmmTR, finTR, _appmmTR, _appinTR, (double) M4_TRAVEL_PER_REV },
	{ CFG_M4, MI, fmtMI, fmtMI, _applyMI, _applyMI, (double) M4_MICROSTEPS },
	{ CFG_M4, PO, fmtPO, fmtPO, _applyPO, _applyPO, (double) M4_POLARITY },
	{ CFG_M4, PW, fmtPW, fmtPW, _applyPW, _applyPW, (double) M4_POWER_MODE },

	{ CFG_X, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) X_AXIS_MODE },
	{ CFG_X, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) X_SEEK_RATE_MAX },
	{ CFG_X, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) X_FEED_RATE_MAX },
	{ CFG_X, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) X_TRAVEL_HARD_LIMIT },
	{ CFG_X, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) X_TRAVEL_SOFT_LIMIT },
	{ CFG_X, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) 0 },
	{ CFG_X, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) X_LIMIT_MODE },
	{ CFG_X, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) X_HOMING_ENABLE },
	{ CFG_X, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) X_HOMING_SEEK_RATE },
	{ CFG_X, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) X_HOMING_CLOSE_RATE },
	{ CFG_X, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) X_HOMING_OFFSET },
	{ CFG_X, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) X_HOMING_BACKOFF },

	{ CFG_Y, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Y_AXIS_MODE },
	{ CFG_Y, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) Y_SEEK_RATE_MAX },
	{ CFG_Y, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) Y_FEED_RATE_MAX },
	{ CFG_Y, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) Y_TRAVEL_HARD_LIMIT },
	{ CFG_Y, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) Y_TRAVEL_SOFT_LIMIT },
	{ CFG_Y, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) 0 },
	{ CFG_Y, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Y_LIMIT_MODE },
	{ CFG_Y, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Y_HOMING_ENABLE },
	{ CFG_Y, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) Y_HOMING_SEEK_RATE },
	{ CFG_Y, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) Y_HOMING_CLOSE_RATE },
	{ CFG_Y, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) Y_HOMING_OFFSET },
	{ CFG_Y, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) Y_HOMING_BACKOFF },

	{ CFG_Z, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) Z_AXIS_MODE },
	{ CFG_Z, SR, fmmSR, finSR, _appmmSR, _appinSR, (double) Z_SEEK_RATE_MAX },
	{ CFG_Z, FR, fmmFR, finFR, _appmmFR, _appinFR, (double) Z_FEED_RATE_MAX },
	{ CFG_Z, TH, fmmTH, finTH, _appmmTH, _appinTH, (double) Z_TRAVEL_HARD_LIMIT },
	{ CFG_Z, TS, fmmTS, finTS, _appmmTS, _appinTS, (double) Z_TRAVEL_SOFT_LIMIT },
	{ CFG_Z, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) 0 },
	{ CFG_Z, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) Z_LIMIT_MODE },
	{ CFG_Z, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) Z_HOMING_ENABLE },
	{ CFG_Z, HR, fmmHR, finHR, _appmmHR, _appinHR, (double) Z_HOMING_SEEK_RATE },
	{ CFG_Z, HC, fmmHC, finHC, _appmmHC, _appinHC, (double) Z_HOMING_CLOSE_RATE },
	{ CFG_Z, HO, fmmHO, finHO, _appmmHO, _appinHO, (double) Z_HOMING_OFFSET },
	{ CFG_Z, HB, fmmHB, finHB, _appmmHB, _appinHB, (double) Z_HOMING_BACKOFF },

	{ CFG_A, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) A_AXIS_MODE },
	{ CFG_A, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) A_SEEK_RATE_MAX },
	{ CFG_A, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) A_FEED_RATE_MAX },
	{ CFG_A, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) A_TRAVEL_HARD_LIMIT },
	{ CFG_A, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) A_TRAVEL_SOFT_LIMIT },
	{ CFG_A, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) A_RADIUS },
	{ CFG_A, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) A_LIMIT_MODE },
	{ CFG_A, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) A_HOMING_ENABLE },
	{ CFG_A, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) A_HOMING_SEEK_RATE },
	{ CFG_A, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) A_HOMING_CLOSE_RATE },
	{ CFG_A, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) A_HOMING_OFFSET },
	{ CFG_A, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) A_HOMING_BACKOFF },

	{ CFG_B, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) B_AXIS_MODE },
	{ CFG_B, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) B_SEEK_RATE_MAX },
	{ CFG_B, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) B_FEED_RATE_MAX },
	{ CFG_B, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) B_TRAVEL_HARD_LIMIT },
	{ CFG_B, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) B_TRAVEL_SOFT_LIMIT },
	{ CFG_B, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) B_RADIUS },
	{ CFG_B, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) B_LIMIT_MODE },
	{ CFG_B, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) B_HOMING_ENABLE },
	{ CFG_B, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) B_HOMING_SEEK_RATE },
	{ CFG_B, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) B_HOMING_CLOSE_RATE },
	{ CFG_B, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) B_HOMING_OFFSET },
	{ CFG_B, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) B_HOMING_BACKOFF },

	{ CFG_C, MO, fmtMO, fmtMO, _applyMO, _applyMO, (double) C_AXIS_MODE },
	{ CFG_C, SR, fdgSR, fdgSR, _appdgSR, _appdgSR, (double) C_SEEK_RATE_MAX },
	{ CFG_C, FR, fdgFR, fdgFR, _appdgFR, _appdgFR, (double) C_FEED_RATE_MAX },
	{ CFG_C, TH, fdgTH, fdgTH, _appdgTH, _appdgTH, (double) C_TRAVEL_HARD_LIMIT },
	{ CFG_C, TS, fdgTS, fdgTS, _appdgTS, _appdgTS, (double) C_TRAVEL_SOFT_LIMIT },
	{ CFG_C, RA, fmmRA, finRA, _appmmRA, _appinRA, (double) C_RADIUS },
	{ CFG_C, LI, fmtLI, fmtLI, _applyLI, _applyLI, (double) C_LIMIT_MODE },
	{ CFG_C, HE, fmtHE, fmtHE, _applyHE, _applyHE, (double) C_HOMING_ENABLE },
	{ CFG_C, HR, fdgHR, fdgHR, _appdgHR, _appdgHR, (double) C_HOMING_SEEK_RATE },
	{ CFG_C, HC, fdgHC, fdgHC, _appdgHC, _appdgHC, (double) C_HOMING_CLOSE_RATE },
	{ CFG_C, HO, fdgHO, fdgHO, _appdgHO, _appdgHO, (double) C_HOMING_OFFSET },
	{ CFG_C, HB, fdgHB, fdgHB, _appdgHB, _appdgHB, (double) C_HOMING_BACKOFF },

	{ CFG_GENERAL, GU, fmtGU, fmtGU, _applyGC, _applyGC, (double) GCODE_UNITS },
	{ CFG_GENERAL, GL, fmtGL, fmtGL, _applyGC, _applyGC, (double) GCODE_PLANE },
	{ CFG_GENERAL, GP, fmtGP, fmtGP, _applyGC, _applyGC, (double) GCODE_PATH_CONTROL },
	{ CFG_GENERAL, GD, fmtGD, fmtGD, _applyGC, _applyGC, (double) GCODE_DISTANCE_MODE },

	{ CFG_GENERAL, EA, fmtEA, fmtEA, _applyEA, _applyEA, (double) ENABLE_ACCEL },
	{ CFG_GENERAL, JL, fmmJL, finJL, _appmmJL, _appinJL, (double) MAX_LINEAR_JERK },
	{ CFG_GENERAL, MM, fmmMM, finMM, _appmmMM, _appinMM, (double) MIN_SEGMENT_LENGTH },
	{ CFG_GENERAL, MT, fmtMT, fmtMT, _applyMT, _applyMT, (double) MIN_SEGMENT_USEC },
	{ CFG_GENERAL, HM, fmtHM, fmtHM, _applyHM, _applyHM, (double) HOMING_MODE },

	{ CFG_GENERAL, EC, fmtEC, fmtEC, _applyEC, _applyEC, (double) COM_APPEND_TX_CR },
	{ CFG_GENERAL, IC, fmtIC, fmtIC, _applyIC, _applyIC, (double) COM_IGNORE_RX_CR },
	{ CFG_GENERAL, IL, fmtIL, fmtIL, _applyIL, _applyIL, (double) COM_IGNORE_RX_LF },
	{ CFG_GENERAL, EX, fmtEX, fmtEX, _applyEX, _applyEX, (double) COM_ENABLE_XON },
	{ CFG_GENERAL, EE, fmtEE, fmtEE, _applyEE, _applyEE, (double) COM_ENABLE_ECHO },

	// ending version record - must be last
	{ CFG_GENERAL, _P, fmtP_, fmtP_, _apply00, _apply00, (double) CFG_VERSION }	// don't mess
};

/*---- APPLY FUNCTIONS -------------------------------------------------*/
// Note: some code style conventions abandoned for density & readability
static void _apply00(struct cfgSetting *s) { return; }	// null apply

// motor settings - unit agnostic
static void _applyMA(struct cfgSetting *s) { 
	cfg.m[MIDX(s->grp)].motor_map = (uint8_t)s->value;
}
static void _applySA(struct cfgSetting *s) {
	cfg.m[MIDX(s->grp)].step_angle = s->value;
	_cfg_set_steps_per_unit(s); 
}
static void _appmmTR(struct cfgSetting *s) { 
	cfg.m[MIDX(s->grp)].travel_rev = s->value;
	_cfg_set_steps_per_unit(s);
}
static void _appinTR(struct cfgSetting *s) { s->value *= 25.4; _appmmTR(s); }

static void _applyMI(struct cfgSetting *s) { 
	cfg.m[MIDX(s->grp)].microsteps = (uint8_t)s->value;
	st_set_microsteps(MIDX(s->grp), cfg.m[MIDX(s->grp)].microsteps);
	_cfg_set_steps_per_unit(s); 
}
static void _applyPO(struct cfgSetting *s) { 
	cfg.m[MIDX(s->grp)].polarity = (uint8_t)s->value;
	st_set_polarity((MIDX(s->grp)), 
		cfg.m[MIDX(s->grp)].polarity);
}
static void _applyPW(struct cfgSetting *s) { 
	cfg.m[MIDX(s->grp)].power_mode = (uint8_t)s->value;}

// axis settings - unit agnostic
static void _applyMO(struct cfgSetting *s) { cfg.a[s->grp].axis_mode = (uint8_t)s->value;}
static void _applyLI(struct cfgSetting *s) { cfg.a[s->grp].limit_mode = (uint8_t)s->value;}
static void _applyHE(struct cfgSetting *s) { cfg.a[s->grp].homing_enable = (uint8_t)s->value;}

// axis settings - mm units
static void _appmmSR(struct cfgSetting *s) { cfg.a[s->grp].seek_rate_max = s->value;}
static void _appmmFR(struct cfgSetting *s) { cfg.a[s->grp].feed_rate_max = s->value; }
static void _appmmTH(struct cfgSetting *s) { cfg.a[s->grp].travel_hard_limit = s->value; }
static void _appmmTS(struct cfgSetting *s) { cfg.a[s->grp].travel_soft_limit = s->value; }
static void _appmmRA(struct cfgSetting *s) { cfg.a[s->grp].radius = s->value; }
static void _appmmHR(struct cfgSetting *s) { cfg.a[s->grp].homing_rate = s->value; }
static void _appmmHC(struct cfgSetting *s) { cfg.a[s->grp].homing_close = s->value; }
static void _appmmHO(struct cfgSetting *s) { cfg.a[s->grp].homing_offset = s->value; }
static void _appmmHB(struct cfgSetting *s) { cfg.a[s->grp].homing_backoff = s->value; }

// axis settings - inches units
static void _appinSR(struct cfgSetting *s) { s->value *= 25.4; _appmmSR(s); }
static void _appinFR(struct cfgSetting *s) { s->value *= 25.4; _appmmFR(s); }
static void _appinTH(struct cfgSetting *s) { s->value *= 25.4; _appmmTH(s); }
static void _appinTS(struct cfgSetting *s) { s->value *= 25.4; _appmmTS(s); }
static void _appinRA(struct cfgSetting *s) { s->value *= 25.4; _appmmRA(s); }
static void _appinHR(struct cfgSetting *s) { s->value *= 25.4; _appmmHR(s); }
static void _appinHC(struct cfgSetting *s) { s->value *= 25.4; _appmmHC(s); }
static void _appinHO(struct cfgSetting *s) { s->value *= 25.4; _appmmHO(s); }
static void _appinHB(struct cfgSetting *s) { s->value *= 25.4; _appmmHB(s); }

// axis settings - degree units
/* See function prototypes for macro definitions
static void _appdgSR(struct cfgSetting *s) { _appmmSR(s); }
static void _appdgFR(struct cfgSetting *s) { _appmmFR(s); }
static void _appdgTH(struct cfgSetting *s) { _appmmTH(s); }
static void _appdgTS(struct cfgSetting *s) { _appmmTS(s); }
static void _appdgHR(struct cfgSetting *s) { _appmmHR(s); }
static void _appdgHC(struct cfgSetting *s) { _appmmHC(s); }
static void _appdgHO(struct cfgSetting *s) { _appmmHO(s); }
static void _appdgHB(struct cfgSetting *s) { _appmmHB(s); }
*/

// general settings - mm units
static void _appmmJL(struct cfgSetting *s) { cfg.linear_jerk_max = s->value; }
static void _appmmMM(struct cfgSetting *s) { cfg.min_segment_len = s->value; }

// general settings - inch units
static void _appinJL(struct cfgSetting *s) { s->value *= 25.4; _appmmJL(s); }
static void _appinMM(struct cfgSetting *s) { s->value *= 25.4; _appmmMM(s); }

// general settings - degree units 
// (none)

// general settings - unit agnostic
// common function for gcode settings
static void _applyGC(struct cfgSetting *s) { 
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
		case 910: cm_set_distance_mode(FALSE); return;// set incremental
	}
}

static void _applyEA(struct cfgSetting *s) { cfg.accel_enabled = (uint8_t)s->value;}
static void _applyMT(struct cfgSetting *s) { cfg.min_segment_time = s->value; }
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

//--- APPLY FUNCTION HELPERS ---------------------------------------------

/*
 * _cfg_set_steps_per_unit() - compute mm of travel per microstep
 *
 *		steps = (360 / (step_angle / microsteps) / travel_per_rev
 *
 *	This function will need to be rethought if microstep morphing is 
 *	implemented, as microsteps are calculated statically. Dang.
 */
static void _cfg_set_steps_per_unit(const struct cfgSetting *s)
{
	cfg.m[MIDX(s->grp)].steps_per_unit = (360 / 
				(cfg.m[MIDX(s->grp)].step_angle / 
				 cfg.m[MIDX(s->grp)].microsteps) /
				 cfg.m[MIDX(s->grp)].travel_rev);
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

#ifdef __DISABLE_EEPROM_INIT		// cutout for debug simulation
	// Apply the hard-wired default values from settings.h
	for (i=0; i<COUNT_SETTINGS; i++) {
		cfgList[i].app_mm(&cfgList[i]);
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
			_cfg_put_setting(_cfg_get_NVM_setting_by_index(i)); // set and apply setting
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

/* 
 * cfg_init_gcode_model()
 */

void cfg_init_gcode_model()
{
	struct cfgSetting *s;

	// in the below you could call either app_mm or app_in - makes no difference
	s = _cfg_get_setting_by_key(CFG_GENERAL, GU); s->app_mm(s); // unit mode
	s = _cfg_get_setting_by_key(CFG_GENERAL, GL); s->app_mm(s); // plane select
	s = _cfg_get_setting_by_key(CFG_GENERAL, GP); s->app_mm(s); // path control
	s = _cfg_get_setting_by_key(CFG_GENERAL, GD); s->app_mm(s); // distance mode
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
		fprintf_P(stderr, PSTR("#### Unknown config string: %s\n"), block);
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
	if ((s->grp = _cfg_get_grp_num(block[0])) != CFG_GENERAL) {
		i++;
	}
	// capture a 2 character mnemonic
	while (j<MNEMONIC_LEN) {
		cs.mnem_str[j++] = block[i++];
	}
	cs.mnem_str[j] = NUL;
	s->mnem = _cfg_get_mnemonic_num(cs.mnem_str);

	// capture value
	while ((!isnumber(block[i]))) {	// position to start of value string
		i++; 
	};
	if (!gc_read_double(block, &i, &s->value)) {
		return (TG_BAD_NUMBER_FORMAT);
	}
	return (TG_OK);
}

/*
 * _cfg_get_grp_num()  			- return axis or motor number from group char
 * _cfg_get_grp_char() 			- return axis or motor char from group #
 * _cfg_get_mnemonic_num()  	- return mnemonic number from string
 * _cfg_get_mnemonic_string() 	- return mnemonic string from number
 */
static int8_t _cfg_get_grp_num(const char c)
{
	switch (c) {
		case 'X': return (CFG_X);
		case 'Y': return (CFG_Y);
		case 'Z': return (CFG_Z);
		case 'A': return (CFG_A);
		case 'B': return (CFG_B);
		case 'C': return (CFG_C);
		case '1': return (CFG_M1);
		case '2': return (CFG_M2);
		case '3': return (CFG_M3);
		case '4': return (CFG_M4);
		case '5': return (CFG_PWM1);
		case '6': return (CFG_PWM1);
	}
	return (CFG_GENERAL);
}

char cfg_get_configuration_group_char(int8_t group)
{
	return(_cfg_get_grp_char(group));
}

char _cfg_get_grp_char(const int8_t grp)
{
	switch (grp) {
		case CFG_X: return 'X';
		case CFG_Y: return 'Y';
		case CFG_Z: return 'Z';
		case CFG_A: return 'A';
		case CFG_B: return 'B';
		case CFG_C: return 'C';
		case CFG_M1: return '1';
		case CFG_M2: return '2';
		case CFG_M3: return '3';
		case CFG_M4: return '4';
		case CFG_PWM1: return '5';
		case CFG_PWM2: return '6';
	}
	return (' ');
}

static int8_t _cfg_get_mnemonic_num(const char *mnem)
{
	for (uint8_t i=0; i<MAX_MNEMONIC; i++) {
		if (strcmp(mnemonics[i], mnem) == 0) {	// match
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

static int16_t _cfg_get_setting_index(const int8_t grp, const int8_t mnem)
{
	uint16_t i;

	for (i=0; i<COUNT_SETTINGS; i++) {
		if ((cfgList[i].grp == grp) && (cfgList[i].mnem == mnem)) {
			return(i);
		}
	}
	DEBUG2(dbCONFIG, PSTR("_cfg_get_setting_index() not found for %d, mnemonic %d"), grp, mnem);
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

static struct cfgSetting *_cfg_get_setting_by_key(const int8_t grp, const int8_t mnem)
{
	uint16_t i;

	if ((i = _cfg_get_setting_index(grp, mnem)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_get_setting_by_key() failed for %d, mnemonic %d"), grp, mnem)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (NULL);
	}
	return(&cfgList[i]);
}

/*
 * _cfg_get_setting_value_by_key() - return setting value
 */

static double _cfg_get_setting_value_by_key(const int8_t grp, const int8_t mnem)
{
	struct cfgSetting *s;
	s = _cfg_get_setting_by_key(grp, mnem);
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
	//  matches the s->grp, s->mnemonic and skip the table scan)
	if ((t = _cfg_get_setting_by_key(s->grp, s->mnem)) == NULL) {
		DEBUG2(dbCONFIG, PSTR("_cfg_put_setting() failed for %d, mnemonic %d"), s->grp, s->mnem)
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
	cs.s.grp = nvm_record[0];
	cs.s.mnem = nvm_record[1];
	memcpy(&cs.s.value, &nvm_record[2], sizeof(double));
	if (cs.s.mnem > MAX_MNEMONIC) {
		cs.status = TG_PARAMETER_OVER_RANGE;
		return (NULL);
	}
	return (&cs.s);
}

/*
 * _cfg_get_NVM_setting_by_key() - return S struct with value from NVM
 */
/* UNUSED
static struct cfgSetting *_cfg_get_NVM_setting_by_key(const int8_t grp, const int8_t mnem)
{
	uint16_t index;

	if ((index = _cfg_get_setting_index(grp, mnem)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_get_NVM_setting_by_key() failed for %d, mnemonic %d"), grp, mnem)
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

	if ((i = _cfg_get_setting_index(s->grp, s->mnem)) == -1) {
		DEBUG2(dbCONFIG, PSTR("_cfg_put_NVM_setting() failed for axis %d, mnemonic %d"), s->grp, s->mnem)
		cs.status = TG_PARAMETER_NOT_FOUND;
		return (cs.status);
	}
	nvm_address = cs.nvm_profile_base + (i * NVM_RECORD_LEN);
	nvm_record[0] = s->grp;
	nvm_record[1] = s->mnem;
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
/* UNUSED - FOR DEBUG ONLY
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
							_cfg_get_grp_char(nvm_record[0]), 
							_cfg_get_mnemonic_string(nvm_record[1]), 
							value );
}
*/

/*
 * _cfg_print_settings()	- print settings based on the input char
 *
 * Print_settings() displays depending on what is typically in block[0]:
 *		"1"		- display settings for a single motor '1'
 *		"X"		- display settings for a single axis 'X'
 *		"M"		- display settings for all motors
 *		"N"		- display settings for all axes
 *		"G"		- display general settings
 *		<NUL>	- display general settings
 *		"$"		- display all settings (from '$$' non-normalized buffer)
 *		'H'		- display help screen
 */

static void _cfg_print_settings(const char c)
{
	int8_t grp = _cfg_get_grp_num(c);
	char grp_char = _cfg_get_grp_char(grp);
	uint16_t i;

	switch(c) {
		// print settings for a single motor
		case '1': case '2': case '3': case '4':	{
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if (cfgList[i].grp == grp) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			return;
		}
		// print settings for a single axis
		case 'X': case 'Y': case 'Z': case 'A': case 'B': case 'C': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if (cfgList[i].grp == grp) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			// print axis advisories
			fprintf_P(stderr, PSTR("%c axis settings translate to:\n"), grp_char);
			_cfg_print_axis_advisories(grp, grp_char);
			_cfg_print_rate_advisories(grp, grp_char);
			return;
		}
		// print settings for all motors
		case 'M': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if ((cfgList[i].grp == CFG_M1) || 
					(cfgList[i].grp == CFG_M2) ||
					(cfgList[i].grp == CFG_M3) ||
					(cfgList[i].grp == CFG_M4)) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			return;
		}
		// print settings for all axes
		case 'N': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				if ((cfgList[i].grp == CFG_X) ||
					(cfgList[i].grp == CFG_Y) ||
					(cfgList[i].grp == CFG_Z) ||
					(cfgList[i].grp == CFG_A) ||
					(cfgList[i].grp == CFG_B) ||
					(cfgList[i].grp == CFG_C)) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			return;
		}
		// print all settings
		case '$': {
			for (i=0; i<COUNT_SETTINGS; i++) {	
				_cfg_print_setting(&cfgList[i]);
			}
			return;
		}
		// print help screen
		case 'H': {
			help_print_config_help();
			return;
		}
		// print general settings
		default: {
			for (i=0; i<COUNT_SETTINGS; i++) {
				if (cfgList[i].grp == CFG_GENERAL) {
					_cfg_print_setting(&cfgList[i]);
				}
			}
			fprintf_P(stderr, PSTR("Type $h for configuration help\n"));
		}
	}
}

/*
 * _cfg_print_setting() - print a single setting
 *
 *	Has some hacks to specialize displays for certain types of settings
 */
static void _cfg_print_setting(const struct cfgSetting *s)
{
	double value = s->value;
	char grp_char = _cfg_get_grp_char(s->grp);
	char *mnem = _cfg_get_mnemonic_string(s->mnem);

	// hack to not display the axis radius values for linear axes
	if ((s->mnem == RA) && (s->grp <A)) {
		return;
	}
	if (cm_get_inches_mode() == FALSE) {// mm mode displays
		if (s->grp == CFG_GENERAL) {
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, mnem, value);
		} else if (s->grp < CFG_MOTOR_BASE) {
			fprintf_P(stderr, PSTR("%c axis - "), grp_char); 
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, grp_char, mnem, value);
		} else {
			fprintf_P(stderr, PSTR("Motor %c - "), grp_char); 
			fprintf_P(stderr, (PGM_P)s->fmt_mm, value, grp_char, mnem, value);
		}
	} else { // inch mode displays
		if (CONVERSION_REQUIRED) {
			value = s->value / 25.4;
		}
		if (s->grp == CFG_GENERAL) {
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, mnem, value);
		} else if (s->grp < CFG_MOTOR_BASE) {
			fprintf_P(stderr, PSTR("%c axis - "), grp_char); 
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, grp_char, mnem, value);
		} else {
			fprintf_P(stderr, PSTR("Motor %c - "), grp_char); 
			fprintf_P(stderr, (PGM_P)s->fmt_in, value, grp_char, mnem, value);
		}
	}
}

/*
 * _cfg_print_axis_advisories() - print axis mode meanings
 * _cfg_print_rate_advisories() - print step rates resulting from settings
 *
 *	These routines are not essential, but are very handy.
 */
 // put axis advisory strings and string array in program memory
char pam00[] PROGMEM = "DISABLED";
char pam01[] PROGMEM = "STANDARD";
char pam02[] PROGMEM = "INHIBITED";
char pam03[] PROGMEM = "RADIUS";
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

static void _cfg_print_axis_advisories(const int8_t axis, const char axis_char) 
{
	double axis_mode = _cfg_get_setting_value_by_key(axis, MO);

	fprintf_P(stderr, PSTR("%c axis mode: %S\n"), axis_char,
					  (PGM_P)pgm_read_word(&cfgPAM[(int)axis_mode]));
}

static void _cfg_print_rate_advisories(const int8_t axis, const char axis_char)
{
//	uint8_t motor = axis - CFG_MOTOR_BASE;

	double step_angle = _cfg_get_setting_value_by_key(axis, SA);
	double travel_rev = _cfg_get_setting_value_by_key(axis, TR);
	double seek_rate = _cfg_get_setting_value_by_key(axis, SR);
	double feed_rate = _cfg_get_setting_value_by_key(axis, FR);
	double seek_steps = (seek_rate / 60 / travel_rev) * (360 / step_angle);
	double feed_steps = (feed_rate / 60 / travel_rev) * (360 / step_angle);
	fprintf_P(stderr, PSTR("%c max seek: %5.0f steps/sec\n"), axis_char, seek_steps);
	fprintf_P(stderr, PSTR("%c max feed: %5.0f steps/sec\n"), axis_char, feed_steps);
	if (feed_rate > seek_rate) {
		fprintf_P(stderr, PSTR("You may be interested to know that the feed rate exceeds the seek rate\n"));
	}
}

/*
 * cfg_print_config_help() - config help screen
 */
/*
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
*/
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

	s = _cfg_get_setting_by_key(CFG_GENERAL, P_);
	fiddle +=1;							// let s settle so we can see it
	s = _cfg_get_setting_by_key(CFG_GENERAL, V_);
	fiddle +=1;							// let s settle so we can see it
	s = _cfg_get_setting_by_key(CFG_GENERAL, _P);
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
	cs.s.grp = axis;
	cs.s.mnemonic = mnemonic;
	cs.s.value = value;
	return (&cs.s);
}


#endif
