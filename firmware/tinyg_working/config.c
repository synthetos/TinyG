/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the 
 * terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License along 
 * with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

/* How it works:
 *
 *	'C' enter config mode from control mode
 *	'Q' quit config mode (return to control mode)
 *	'?' dump config to console
 *	'H' show help screen
 *
 *	Configuration parameters are set one line at a time.
 *	Whitespace is ignored and not used for delimiting.
 *	Non-alpha and non numeric characters are ignored (except newline).
 *	Parameter strings are case insensitive. 
 *	Tags can have extra letters for readability.
 *	Comments are in parentheses and cause the rest of the line to be ignored.
 *
 *	Per-axis parameters have an axis letter followed by a 2 letter tag 
 *	followed by the parameter value. Examples:
 *		X SE 1500 (set X axis max seek rate to 1500 steps per second)
 *		zseek1800.99 (set Z axis max seek rate to 1800 steps per second)
 *
 *	General parameters are formatted as needed, and are explained separately
 *		AR 0.01  	(arc steps per mm)
 *
 *	------ Supported parameters ------
 *
 * 	In the examples below 'X' means any supported axis: X, Y, Z or A.
 *	[nnnn] is the range or list of values supported. The []'s are not typed.
 *  .00 indicates a floating point value - all others are integers.
 *
 * Non-axis config parameters
 * 
 *		  MM [0.00-1.00]	Millimeters per arc segment 
 *							Current driver resolution is between 0.05 and 0.01 mm
 *
 * Per-axis parameters
 *
 *		X SE [0-65535]		Maximum seek steps per second
 *							In whole steps (not microsteps)
 *							A practical limit will be < 2000 steps/sec
 *
 *		X FE [0-65535]		Maximum feed steps per second. As above
 *
 *		X DE [0.00-360.00]	Degrees per step
 *							Commonly 1.8
 *							A practical limit will be 7.5
 *
 *		X MI [-1,1,2,4,8]	Microstep mode
 *							1-8 is whole to 1/8 step
 *							-1 is morphing microsteps with rotational speed
 *							(microstep morphing is not yet implemented)
 *							(other morphing modes may be supported as well)
 *
 *		X PO [0,1]			Axis motor polarity
 *							0 = normal polarity
 *							1 = reverse polarity
 *
 *		X TR [0-65535]		Maximum axis travel in mm (table size)
 *
 *		X RE [0-9999.99]	Travel per revolution in mm (mm per revolution)
 *
 *		X ID [0,1]			Idle mode
 *							0 = no idle mode
 *							1 = low power idle mode enabled
 *
 *		X LI [0,1]			Limit switch mode
 *							0 = no limit switches
 *							1 = limit switches enabled 
 *							(may need more modes than this)
 *
 *		X MA [0-4]			Map axis to motor number
 *							0 = axis disabled
 *							1-4 = motor number 1-4
 *							Can also be used to support axis slaving
 *
 * Motor numbers, axis mapping, and axis slaving [to be implemented]
 *	
 *	Axis letters are logical values that map down to the physical motors.
 *	The default mapping is X=1, Y=2, Z=3, A=4
 *	Per-axis settings can be specified by axis or by motor number. 
 *	The following are valid and equivalent (assuming default mapping, above):
 *
 *		X SE 1500	(set motor 1 seek rate to 1500 steps per second)
 *		1se1500		(set motor 1 seek rate to 1500 steps per second)
 *
 *	Axis slaving is supported by mapping an axis letter to 2 or more motors:
 *
 *		XMA1		(map X axis to motor 1)
 *		XMA2		(map X axis to motor 2)
 *
 *	...would slave motors 1 and 2 to the X axis. Gcode X movements will be 
 *	executed by both motors. Config settings addressed to motor 1  set 
 *	motor 1, to motor 2 set motor 2, and to X set parameters for both 
 *	motor 1 and motor 2.
 *
 * Computed parameters
 *
 *	There are also a set of parameters that are computed from the above and 
 *	are displayed for convenience
 *
 *		steps per mm by axis
 *		steps per inch by axis
 *		maximum seek rate in mm/minute and inches/minute
 *		maximum feed rate in mm/minute and inches/minute
 *
 * G code configuration
 *
 *	Config accepts the following G codes which become the power-on defaults 
 *
 *		G20/G21			Select inches (G20) or millimeters mode (G21)
 *		G17/G18/G19		Plane selection
 *
 *	Examples of valid config lines:
 *
 *		X SE 1800			(Set X maximum seek to 1800 whole steps / second)
 *		XSE1800				(Same as above)
 *		xseek1800			(Same as above)
 *		xseek+1800			(Same as above)
 *		xseek 1800.00		(Same as above)
 *		xseek 1800.99		(OK, but will be truncated to 1800 integer value)
 *		X FE [1800]			(OK, but the [] brackets are superfluous)
 *		ZID1(set low power idle mode on Z axis, & show no space needed for comment)
 *		zmicrsteps 4 		(sets Z microsteps to 1/4, misspelling is intentional)
 *		G20					(Set Gcode to default to inches mode) 
 *		mm_per_arc_segment 0.01 (underscores)
 *		mm per arc segment 0.01	(spaces)
 *		MM0.01
 *
 *	Examples of invalid config lines:
 *
 *		SE 1800				(No axis specified)
 *		XSE1800	(config is OK, but has illegal (embedded) comment (see Note))
 *		SE 1800 X			(Axis specifier must be first)
 *		SEX 1800			(SEX is unsupported -axis specifier must be first)
 *		FEX 1800			(FEX is also unsupported)
 *		C LI 1				(C axis not currently supported -nor is B)
 *		X FE -100			(Negative feed step rate -you can try it...)
 *		X FE 100000			(Exceeds number range)
 *							(Find a motor this fast and we'll bump the data size)
 *
 *  NOTE: Technically embedded comments are against the RS274NGC spec, but the 
 *		  dumb-as-dirt comment parser will actually allow them. Still, it's 
 *		  best not to use them as they are out of spec.
 */
/*
 * CONFIG INTERNALS
 *
 *	Config is a collection of settings for:
 *		(1) Gcode defaults
 *		(2) non-axis machine settings
 *		(3) per-axis machine settings (4 axes are defined)
 *
 *	Config is stored and used at run-time in the cfg struct (in binary form).
 *	Config is persisted to EEPROM as a set of ASCII config records ("records")
 *	Functions exist to move settings between the two.
 *	A baseline config is defined in hardware.h. It is loaded at power-up
 *	  before attempting to read the EEPROM so the cfg struct always has some
 *	  degree of sanity even if the EEPROM fails or is not initialized.
 *	In addition to hardware.h / user entered settings there are a set of 
 *	  computed settings in cfg that are derived from the config settings. 
 *	  These are recomputed every time a config change occurs.
 *
 *	EEPROM has a header record of format:
 *		'%'	first character
 *		"NNN"   format revision level EEPROM_FORMAT_REVISION (not build number!)
 *		"l:NN"  record length specifier (CFG_RECORD_LEN == 12)
 *
 *	EEPROM has a trailer record of format:
 *		'%'	first character
 *		"END"
 *
 * 	Reset performs the following actions:
 *		- load config struct with compiled hardware.h default settings
 *		- if EEPROM is not initialized:
 *			- initialize EEPROM
 *			- write the default config to EEPROM
 *			- exit
 *		- if EEPROM is initialized but is not the current revision:
 *			- read settings from EEPROM into config struct
 *			- initialize EEPROM (with new revision and trailer)
 *			- write config struct back to the EEPROM
 *			- exit
 *		- else (EEPROM is intialized and current):
 *			- read settings from EEPROM into config struct
 *			  Note that not all settings are required to be in EEPROM,
 *			  and only those settings in EEPROM will be loaded 
 *			  (and overwrite the hardware,h settings).
 *
 *	Parsing a setting from the command line performs the following actions:
 *		- normalize and parse the input into a record and fielded values
 *		- update the cfg struct
 *		- write the record into EEPROM
 */
/* TODO:
	- help screen
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>					// for memset(), strchr()
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "xio.h"
#include "gcode.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
#include "controller.h"
#include "xmega_eeprom.h"

// prototypes for local helper functions
void _cfg_computed(void); 
void _cfg_normalize_config_block(char *block);
char *_cfg_create_config_record(char *block);
uint8_t _cfg_tokenize_config_record();

void _cfg_load_default_settings(void);
void _cfg_write_config_struct_to_eeprom(uint16_t address);

void _cfg_write_config_record(uint8_t param, double value, char axis);
uint16_t _cfg_compute_record_address(uint16_t address);
void _cfg_write_config_record_to_eeprom(const uint16_t address);
char *_cfg_read_config_record_from_eeprom(const uint16_t address);
void _cfg_print_config_record(char *record);
void _cfg_print_axis(uint8_t axis);

// Config parameter tokens and config record constants
// These values are used to tokenize config strings and 
// to compute the EEPROM record addresses (_cfg_write_record())

enum cfgTokens {
	CFG_ZERO_TOKEN,				// header record must always be location zero

	// Gcode default settings
	CFG_GCODE_PLANE,			// use gcCanonicalPlane enum (0-2)
	CFG_GCODE_UNITS,			// 0=inches (G20), 1=mm (G21)
	CFG_GCODE_HOMING_MODE,		// TRUE = homing cycle on startup
	CFG_GCODE_FEED_RATE,		// Default F value
	CFG_GCODE_SPINDLE_SPEED,	// default S value
	CFG_GCODE_TOOL,				// default T value

	// machine default settings
	CFG_MM_PER_ARC_SEGMENT,

	// per-axis settings 
	CFG_MAP_AXIS_TO_MOTOR,		// must be the first axis setting 
	CFG_SEEK_STEPS_MAX,
	CFG_FEED_STEPS_MAX,
	CFG_DEGREES_PER_STEP,
	CFG_MICROSTEP_MODE,
	CFG_POLARITY,
	CFG_TRAVEL_MAX,
	CFG_TRAVEL_WARN,			// stop homing cycle if above this value
	CFG_TRAVEL_PER_REV,
	CFG_IDLE_MODE,
	CFG_LIMIT_SWITCH_MODE,

	CFG_LAST_TOKEN				// must always be last token enum
};

#define CFG_EEPROM_BASE 0x0000	// base address of usable EEPROM
#define CFG_RECORD_LEN 12		// length of ASCII EEPROM strings (see note)
#define CFG_NON_AXIS_BASE CFG_MM_PER_ARC_SEGMENT // start of non-axis params
#define CFG_AXIS_BASE CFG_MAP_AXIS_TO_MOTOR // start of axis parameters
#define CFG_AXIS_COUNT (CFG_LAST_TOKEN - CFG_AXIS_BASE) // # of axis params
#define CFG_HEADER_RECORD_ADDR CFG_EEPROM_BASE
#define CFG_TRAILER_RECORD (CFG_AXIS_BASE + (4*CFG_AXIS_COUNT))
#define CFG_TRAILER_RECORD_ADDR (CFG_TRAILER_RECORD * CFG_RECORD_LEN)

/* Note: A CFG_RECORD_LEN of 12 will accommodate numbers up to 8 digits long 
	- seven if it has a decimal point, 6 if it also has a minus sign. Numbers
	with more digits will be truncated from the right. This should suffice 
	for any reasonable setting, but if not the record length must be increased
 */

// local data
struct cfgConfigParser {
	uint8_t status;				// parser status
	uint8_t param;				// tokenized parameter number
	double value;				// setting value
	int8_t axis;				// internal axis # (0-3) or -1 if none
	char axis_char;				// axis character or space if none
	uint16_t profile_base;		// base address of current profile
	char record[CFG_RECORD_LEN+1];// config record for EEPROM
};
static struct cfgConfigParser cp;

/*
 * cfg_init() - initialize config system
 */

void cfg_init() 
{
	cp.profile_base = CFG_EEPROM_BASE;	// first (and only) profile
	cfg_reset();			// reset config from compiled defaults
}

/* 
 * cfg_reset() - reset configs (but not necessarily the entire config system)
 *
 * 	Reset performs the following actions:
 *		- load config struct with compiled hardware.h default settings
 *		- if EEPROM is not initialized:
 *			- initialize EEPROM
 *			- write the default config to EEPROM
 *			- exit
 *		- if EEPROM is initialized but is not the current revision:
 *			- read settings from EEPROM into config struct
 *			- initialize EEPROM (with new revision and trailer)
 *			- write config struct back to the EEPROM
 *			- exit
 *		- else (EEPROM is intialized and current):
 *			- read settings from EEPROM into config struct
 *			  Note that not all settings are required to be in EEPROM,
 *			  and only those settings in EEPROM will be loaded 
 *			  (and thus overwrite the hardware.h settings).
 */

void cfg_reset()
{
	uint16_t address = cp.profile_base;

	// load hardware.h default settings
	_cfg_load_default_settings();

	// see if EEPROM is initialized and tak approriate action
	EEPROM_ReadString(cp.profile_base, cp.record, CFG_RECORD_LEN);

	// if the header is not initialzed, set it up and exit
	if (cp.record[0] != '%') {
		_cfg_write_config_struct_to_eeprom(cp.profile_base);
		return;
	}

	// if the header is initialzed but the wrong revision...
//	if (cp.record[0] != '%') {
//		// initialize EEPROM for config records
//		EEPROM_WriteString(CFG_HEADER_RECORD_ADDR, CFG_HEADER, TRUE);
//		EEPROM_WriteString(CFG_TRAILER_RECORD_ADDR, CFG_TRAILER, TRUE);
//		_cfg_write_config_struct_to_eeprom();
//		return;
//	}

	// if the header is initialized, read the EEPROM configs into the struct
	for (uint16_t i = 0; i < CFG_TRAILER_RECORD; i++) {
		EEPROM_ReadString(address, cp.record, CFG_RECORD_LEN);
		cfg_parse(cp.record);
		address += CFG_RECORD_LEN;
	}
	return;
}

/*
 * _cfg_write_config_struct_to_eeprom()
 *
 * Write entire config structure to EEPROM
 * Also writes header and trailer records
 */

void _cfg_write_config_struct_to_eeprom(uint16_t address)
{
	// write header and trailer records
	EEPROM_WriteString((address + CFG_HEADER_RECORD_ADDR), CFG_HEADER, TRUE);
	EEPROM_WriteString((address + CFG_TRAILER_RECORD_ADDR), CFG_TRAILER, TRUE);

	_cfg_write_config_record(CFG_GCODE_PLANE, (17 + cfg.gcode_plane), 0);
	_cfg_write_config_record(CFG_GCODE_UNITS, (20 + cfg.gcode_units), 0);
	_cfg_write_config_record(CFG_GCODE_HOMING_MODE, 28, 0);
	_cfg_write_config_record(CFG_GCODE_FEED_RATE, 400.50, 0); 
	_cfg_write_config_record(CFG_GCODE_SPINDLE_SPEED, 12000, 0);
	_cfg_write_config_record(CFG_GCODE_TOOL, 1, 0);

	_cfg_write_config_record(CFG_MM_PER_ARC_SEGMENT, 0.01, 0); 
/*
	for (uint8_t axis=X; axis<=A; axis++) {

		_cfg_print_axis(axis);
		_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 1, 'X'); 
		_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 2, 'Y'); 
		_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 3, 'Z'); 
		_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 4, 'A'); 

		_cfg_write_config_record(CFG_SEEK_STEPS_MAX, 1500, 'X'); 
		_cfg_write_config_record(CFG_FEED_STEPS_MAX, 1200, 'X'); 
		_cfg_write_config_record(CFG_DEGREES_PER_STEP, 1.8, 'X'); 
		_cfg_write_config_record(CFG_MICROSTEP_MODE, -1, 'X'); 
		_cfg_write_config_record(CFG_POLARITY, 0, 'X'); 
		_cfg_write_config_record(CFG_TRAVEL_MAX, 400, 'X'); 
		_cfg_write_config_record(CFG_TRAVEL_WARN, 425, 'X'); 
		_cfg_write_config_record(CFG_TRAVEL_PER_REV, 1.27, 'X'); 
		_cfg_write_config_record(CFG_IDLE_MODE, 1, 'X'); 
		_cfg_write_config_record(CFG_LIMIT_SWITCH_MODE, 0, 'X'); 
	}
*/
}

/*
 * _cfg_write_config_record() - make a confg record string from a cp struct
 *
 *	cp.param		set to proper enum
 *	cp.value		value loaded as a double
 *	cp.axis_char	axis character (must be null for non-axis settings)
 *
 *	For Gcode settings to work cp.value must be set to proper Gcode number:
 *
 *		17 	select XY plane	(cfg.plane = CANON_PLANE_XY  (0))
 *		18 	select XZ plane	(cfg.plane = CANON_PLANE_XZ  (1))
 *		19 	select YZ plane	(cfg.plane = CANON_PLANE_YZ  (2))
 *		20 	units in mm 	(cfg.units = 0)
 *		21 	units in inches	(cfg.units = 1)
 *		28 	home on startup	(cfg homing_mode = 1)
 */

// put record format strings in program memory
char cfgMakeRecord00[] PROGMEM = "HEADER%c%f";
char cfgMakeRecord01[] PROGMEM = "G%1.0f";		// Plane G17/G18/G19
char cfgMakeRecord02[] PROGMEM = "G%1.0f";		// Units G20/G21
char cfgMakeRecord03[] PROGMEM = "G%1.0f";		// G28  Power-on homing
char cfgMakeRecord04[] PROGMEM = "F%1.3f";		// F Feed rate
char cfgMakeRecord05[] PROGMEM = "S%1.2f";		// S Spindle speed
char cfgMakeRecord06[] PROGMEM = "T%1.0f";		// T Tool
char cfgMakeRecord07[] PROGMEM = "MM%1.3f";		// MM per arc segment
char cfgMakeRecord08[] PROGMEM = "%cMA%1.0f";	// Map axis to motor
char cfgMakeRecord09[] PROGMEM = "%cSE%1.0f";	// Seek steps per second
char cfgMakeRecord10[] PROGMEM = "%cFE%1.0f";	// Feed steps / sec
char cfgMakeRecord11[] PROGMEM = "%cDE%1.3f";	// Degrees per step
char cfgMakeRecord12[] PROGMEM = "%cMI%1.0f";	// Microstep mode
char cfgMakeRecord13[] PROGMEM = "%cPO%1.0f";	// Polarity
char cfgMakeRecord14[] PROGMEM = "%cTR%1.0f";	// Travel max (mm)
char cfgMakeRecord15[] PROGMEM = "%cTW%1.0f";	// Travel Warning
char cfgMakeRecord16[] PROGMEM = "%cRE%1.3f";	// mm per REvolution
char cfgMakeRecord17[] PROGMEM = "%cID%1.0f";	// Idle mode
char cfgMakeRecord18[] PROGMEM = "%cLI%1.0f";	// Limit switches on

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P cfgMkStrings[] PROGMEM = {	
	cfgMakeRecord00,
	cfgMakeRecord01,
	cfgMakeRecord02,
	cfgMakeRecord03,
	cfgMakeRecord04,
	cfgMakeRecord05,
	cfgMakeRecord06,
	cfgMakeRecord07,
	cfgMakeRecord08,
	cfgMakeRecord09,
	cfgMakeRecord10,
	cfgMakeRecord11,
	cfgMakeRecord12,
	cfgMakeRecord13,
	cfgMakeRecord14,
	cfgMakeRecord15,
	cfgMakeRecord16,
	cfgMakeRecord17,
	cfgMakeRecord18
};

void _cfg_write_config_record(uint8_t param, double value, char axis)
{
	uint16_t address;

	cp.param = param;
	cp.axis = axis;

	if (param < CFG_AXIS_BASE) {
		sprintf_P(cp.record,(PGM_P)pgm_read_word(&cfgMkStrings[param]), value);
	} else {
		sprintf_P(cp.record,(PGM_P)pgm_read_word(&cfgMkStrings[param]), axis, value);
	}
//	address = _cfg_compute_record_address(cp.profile_base, cp.param, cp.axis);
	EEPROM_WriteString(address, cp.record, TRUE);
}

/* 
 * cfg_parse() - parse a config line
 *			   - write into config record and persist to EEPROM
 */

int cfg_parse(char *block)
{
	cp.status = TG_OK;

	// normalize the block in place
	_cfg_normalize_config_block(block);

	// dispatch on special characters in the first location
	switch (block[0]) {
		case  0:  return(TG_OK);			// ignore comments (stripped)
		case 'Q': return(TG_QUIT);			// quit config mode
		case 'H': cfg_print_help_screen(); return(TG_OK);
		case '?': cfg_print_config_records(); return(TG_OK);
	}

	// create a well-formed config record from the normalized block
	_cfg_create_config_record(block);		
	
	// parse the config record into a parser structure (or die trying)
	if (_cfg_tokenize_config_record()) {
		tg_print_status(cp.status, block);
	}

	// load value based on parameter type (cp.param)
	switch (cp.param) {
		case CFG_GCODE_PLANE:		break;
		case CFG_GCODE_UNITS:	  	break;
		case CFG_GCODE_HOMING_MODE:	break;
		case CFG_GCODE_FEED_RATE:	break;
		case CFG_GCODE_SPINDLE_SPEED: break;
		case CFG_GCODE_TOOL:		break;

		case CFG_MM_PER_ARC_SEGMENT: cfg.mm_per_arc_segment = cp.value; break;

		case CFG_MAP_AXIS_TO_MOTOR:	break;
		case CFG_SEEK_STEPS_MAX: 	CFG(cp.axis).seek_steps_sec = (uint16_t)cp.value; break;
		case CFG_FEED_STEPS_MAX: 	CFG(cp.axis).feed_steps_sec = (uint16_t)cp.value; break;
		case CFG_DEGREES_PER_STEP:	CFG(cp.axis).degree_per_step = cp.value; break;
		case CFG_MICROSTEP_MODE:	CFG(cp.axis).microstep = (uint8_t)cp.value; break;
		case CFG_POLARITY:			CFG(cp.axis).polarity = (uint8_t)cp.value; 
					  		 		st_set_polarity(cp.axis, CFG(cp.axis).polarity); break;
		case CFG_TRAVEL_MAX: 		CFG(cp.axis).mm_travel = cp.value; break;
		case CFG_TRAVEL_WARN: 		break;
		case CFG_TRAVEL_PER_REV: 	CFG(cp.axis).mm_per_rev = cp.value; break;
		case CFG_IDLE_MODE: 		CFG(cp.axis).low_pwr_idle = (uint8_t)cp.value; break;
		case CFG_LIMIT_SWITCH_MODE: CFG(cp.axis).limit_enable = (uint8_t)cp.value; break;

		default: cp.status = TG_UNRECOGNIZED_COMMAND;	// error return
	}

	// save config record in EEPROM
	_cfg_write_config_record_to_eeprom(cp.profile_base);

	// generate & (re)populate computed config values
	_cfg_computed();

	// do config displays
//	_cfg_read_config_record_from_eeprom(cp.profile_base);
	_cfg_print_config_record(cp.record);

	return (cp.status);
}

/* 
 * _cfg_computed() - helper function to generate computed config values 
 *	call this every time you change any configs
 */

inline void _cfg_computed() 
{
	// = 360 / (degree_per_step/microstep) / mm_per_rev
	for (uint8_t i=X; i<=A; i++) {
		cfg.a[i].steps_per_mm = (360 / (cfg.a[i].degree_per_step / 
										cfg.a[i].microstep)) / 
										cfg.a[i].mm_per_rev;
	}
	// max_feed_rate = 60 * feed_steps_sec / (360/degree_per_step/mm_per_rev)
	cfg.max_feed_rate = ((60 * (double)cfg.a[X].feed_steps_sec) /
						 (360/cfg.a[X].degree_per_step/cfg.a[X].mm_per_rev));

	// max_seek_rate = 60 * seek_steps_sec / (360/degree_per_step/mm_per_rev)
	cfg.max_seek_rate = ((60 * (double)cfg.a[X].seek_steps_sec) /
						 (360/cfg.a[X].degree_per_step/cfg.a[X].mm_per_rev));
}

/*
 * _cfg_normalize_config_block() - normalize a config block in place
 *
 *	Normalize a block for further processing. 
 *	Normalization is command agnostic - no knowledge or state.
 *	Capitalizes and packs all valid characters (no whitespace)
 *	Removes all invalid characters
 *	Strips comments. Comments supported as below:
 *	  supported:	CONFIG
 *	  supported:	comment
 *	  supported:	CONFIG comment
 *	  unsupported:	CONFIG CONFIG
 *	  unsupported:	comment CONFIG
 *	  unsupported:	CONFIG comment CONFIG
 *
 *	Valid characters (these are passed to config parser):
 *		digits						all digits are passed to parser
 *		lower case alpha			all alpha is passed - converted to upper
 *		upper case alpha			all alpha is passed
 *		- . ? 						sign, decimal & question passed to parser
 *
 *	Invalid characters (these are stripped but don't cause failure):
 *		control characters			chars < 0x20 are all removed
 *		/ *	< = > | % #	+			expression chars removed from string
 *		( ) [ ] { } 				expression chars removed from string
 *		<sp> <tab> 					whitespace chars removed from string
 *		! $ % ,	; ; @ 				removed
 *		^ _ ~ " ' <DEL>				removed
 */

void _cfg_normalize_config_block(char *block) 
{
	char c;
	uint8_t i=0; 		// index for incoming characters
	uint8_t j=0;		// index for normalized characters

	// normalize the block & prune the comment(if any)
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
			break;
		}
	}										// ignores any other characters
	block[j] = 0;							// terminate block and end
}

/*
 * _cfg_create_config_record() - create an EEPROM record from a block
 *
 *	Converts a block into a record: parser and EEPROM friendly form
 *	Block must be pre-normalized (uppercase, no WS | comments | illegal chars)
 *	Keys off syntax only, does not validate comamnd semantics
 *
 *	Handles these command forms:
 *	 - Gcode settings 		Letter followed by value,  e.g's. G20, F333.333
 *	 - Non-axis settings 	2 letters followed by value, e.g. MM0.01
 *	 - Axis settings 
 *		letter type			3 letters followed by value, e.g. XSE1500
 *		number type			1 number + 2 letters +value, e.g. 1SE1500
 *
 *	The non-axis and axis forms can have extra ASCII. Gcode configs cannot
 */

char *_cfg_create_config_record(char *block)
{
	uint8_t i=0; 				// char index into block
	uint8_t j=0; 				// char index into record

	// cutout for null block
	if (!block[0]) {
		return (0);
	}
	memcpy(cp.record, block, CFG_RECORD_LEN-1);// initialize record string

	if (isdigit(block[1])) {			// handle Gcode settings
		return (block);					// OK as-is. Return it.
	}
	switch(block[0]) {
		case 'M': { i=1; j=2; break; }	// handle non-axis settings (only 1 for now)
		default:  { i=2; j=3; break; }	// handle axis settings
	}
	while (isupper(block[++i])) {		// position to value by advancing
	}									//...past any remaining tag alphas
	while (block[0] && j < CFG_RECORD_LEN-1) {	// copy value to EEPROM record
		cp.record[j++] = block[i++];
	}
	cp.record[j] = 0;					// terminate string
	return (cp.record);
}

/*
 * _cfg_tokenize_config_record() - parse record into struct
 *
 *	Block must be normalized w/comments removed
 */

uint8_t _cfg_tokenize_config_record()
{
	uint8_t i=0; 							// char index into record
	char *end;								// pointer to end of value

	// get the value
	while (isupper(cp.record[++i])) {		// locate start of value
	}										// & skip leading digit
	cp.value = strtod(&cp.record[i], &end); // extract the value
	cp.axis = -1;
	cp.axis_char = ' ';							// initialize it to nothing

	// tokenize everything
	switch(cp.record[0]) {
		// gcode settings
		case 'G': {
			switch ((int)cp.value) {
				case 17: { cp.param = CFG_GCODE_PLANE; cp.value = CANON_PLANE_XY; break; }
				case 18: { cp.param = CFG_GCODE_PLANE; cp.value = CANON_PLANE_XZ; break; }
				case 19: { cp.param = CFG_GCODE_PLANE; cp.value = CANON_PLANE_YZ; break; }
				case 20: { cp.param = CFG_GCODE_UNITS; cp.value = 0; break; }
				case 21: { cp.param = CFG_GCODE_UNITS; cp.value = 1; break; }
				case 28: { cp.param = CFG_GCODE_HOMING_MODE; cp.value = 0; break; }
			}
			return (TG_OK);
		}
		case 'F': { cp.param = CFG_GCODE_FEED_RATE; break; }
		case 'S': { cp.param = CFG_GCODE_SPINDLE_SPEED; break ;}
		case 'T': { cp.param = CFG_GCODE_TOOL; break ;}

		// non-axis settings
		case 'M': { cp.param = CFG_MM_PER_ARC_SEGMENT; break; }

		// axis and mapped axis settings
		case 'X': { cp.axis = 0; cp.axis_char = 'X'; break;} // axis settings
		case 'Y': { cp.axis = 1; cp.axis_char = 'Y'; break;}
		case 'Z': { cp.axis = 2; cp.axis_char = 'Z'; break;}
		case 'A': { cp.axis = 3; cp.axis_char = 'A'; break;}
		case '1': { cp.axis = 0; cp.axis_char = '1'; break;} // by motor number
		case '2': { cp.axis = 1; cp.axis_char = '2'; break;}
		case '3': { cp.axis = 2; cp.axis_char = '3'; break;}
		case '4': { cp.axis = 3; cp.axis_char = '4'; break;}

		// has to have been one of the above or it's an error
		default: return(TG_UNRECOGNIZED_COMMAND);
	}
	// pick apart the axis settings
	switch(cp.record[1]) {
		case 'S': { cp.param = CFG_SEEK_STEPS_MAX; break; }
		case 'F': { cp.param = CFG_FEED_STEPS_MAX; break; }
		case 'D': { cp.param = CFG_DEGREES_PER_STEP; break; }
		case 'P': { cp.param = CFG_POLARITY; break; }
		case 'T': 
			switch (cp.record[2]) {
				case 'R': { cp.param = CFG_TRAVEL_MAX; break; }
				case 'W': { cp.param = CFG_TRAVEL_WARN; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} break;
		case 'R': { cp.param = CFG_TRAVEL_PER_REV; break; }
		case 'I': { cp.param = CFG_IDLE_MODE; break; }
		case 'L': { cp.param = CFG_LIMIT_SWITCH_MODE; break; }
		case 'M': 
			switch (cp.record[2]) {
				case 'I': { cp.param = CFG_MICROSTEP_MODE; break; }
				case 'A': { cp.param = CFG_MAP_AXIS_TO_MOTOR; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} break;
		default: return(TG_UNRECOGNIZED_COMMAND);
	}
	return (TG_OK);
}

/*
 * _cfg_print_config_record()
 *
 *  Takes a config record as input - record must obey record formatting
 *	Uses global cp struct to tokenize and extract values from record
 */

// put record format strings in program memory
char cfgShowRecord00[] PROGMEM = "HEADER%s%d";
char cfgShowRecord01[] PROGMEM = "%c Gcode: {G17/G18/G19}    Plane:  %1.0f\n";
char cfgShowRecord02[] PROGMEM = "%c Gcode: {G20/G21} Units (1=mm):  %1.0f\n";
char cfgShowRecord03[] PROGMEM = "%c Gcode: {G28}  Power-on homing:  %1.0f\n";
char cfgShowRecord04[] PROGMEM = "%c Gcode: {F} Feed rate:       %8.2f\n";
char cfgShowRecord05[] PROGMEM = "%c Gcode: {S} Spindle speed:   %8.2f\n";
char cfgShowRecord06[] PROGMEM = "%c Gcode: {T} Tool:                %1.0f\n";
char cfgShowRecord07[] PROGMEM = "%c MM(illimeters) / arc segment:  %6.3f\n";
char cfgShowRecord08[] PROGMEM = "MAp %c axis to motor number: %7.0f\n";
char cfgShowRecord09[] PROGMEM = "  %c axis - SEek steps / sec:  %5.0f\n";
char cfgShowRecord10[] PROGMEM = "  %c axis - FEed steps / sec:  %5.0f\n";
char cfgShowRecord11[] PROGMEM = "  %c axis - DEgrees per step:  %5.0f\n";
char cfgShowRecord12[] PROGMEM = "  %c axis - MIcrostep mode:    %5.0f\n";
char cfgShowRecord13[] PROGMEM = "  %c axis - POlarity:          %5.0f\n";
char cfgShowRecord14[] PROGMEM = "  %c axis - TRavel max:        %5.0f\n";
char cfgShowRecord15[] PROGMEM = "  %c axis - Travel Warning:    %5.0f\n";
char cfgShowRecord16[] PROGMEM = "  %c axis - mm per REvolution: %5.0f\n";
char cfgShowRecord17[] PROGMEM = "  %c axis - IDle mode          %5.0f\n";
char cfgShowRecord18[] PROGMEM = "  %c axis - LImit switches on: %5.0f\n";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P cfgShowRecordStrings[] PROGMEM = {	
	cfgShowRecord00,
	cfgShowRecord01,
	cfgShowRecord02,
	cfgShowRecord03,
	cfgShowRecord04,
	cfgShowRecord05,
	cfgShowRecord06,
	cfgShowRecord07,
	cfgShowRecord08,
	cfgShowRecord09,
	cfgShowRecord10,
	cfgShowRecord11,
	cfgShowRecord12,
	cfgShowRecord13,
	cfgShowRecord14,
	cfgShowRecord15,
	cfgShowRecord16,
	cfgShowRecord17,
	cfgShowRecord18
};

void _cfg_print_config_record(char *record)
{
	// tokenize the record and return if failed
	if (_cfg_tokenize_config_record()) {
		tg_print_status(cp.status, record);
		return;
	}
	// otherwise print it
//	printf_P(PSTR("\n%s    "), cp.record);
	printf_P((PGM_P)pgm_read_word(&cfgShowRecordStrings[cp.param]), cp.axis_char, cp.value);
}

/*
 * cfg_print_config_records() - dump configs from EEPROM to stderr
 */

void cfg_print_config_records()
{
	uint16_t record_addr = cp.profile_base;

	for (uint16_t i = 0; i < CFG_TRAILER_RECORD; i++) {
		EEPROM_ReadString(record_addr, cp.record, CFG_RECORD_LEN);
		_cfg_print_config_record(cp.record);
		record_addr += CFG_RECORD_LEN;
	}
}

/* 
 * _cfg_read_config_record_from_eeprom()
 *
 * Uses cp struct to read 
 */

char *_cfg_read_config_record_from_eeprom(uint16_t address)
{
	address = _cfg_compute_record_address(address);
//	EEPROM_ReadString(address, cp.record, FALSE);
  	return(cp.record);
}

/* 
 * _cfg_write_config_struct_to_eeprom()
 *
 * Write entire config structure to EEPROM
 * Also writes header and trailer records
 */

//void _cfg_write_config_struct_to_eeprom(uint16_t address)
//{
//	address = _cfg_compute_record_address(address);
//	EEPROM_WriteString(address, cp.record, TRUE);
//}

/* 
 * _cfg_write_config_record_to_eeprom()
 *
 * Configuration records are written to EEPROM using the following scheme:
 *	- header record - identifies revision and carries record length
 *	- Gcode settings		(identified by token < CFG_AXIS_BASE)
 *	- non-axis settings		(identified by token < CFG_AXIS_BASE)
 *	- per-axis settings		(identified by token >= CFG_AXIS_BASE)
 *
 *	The base address of the record-set is provided as an argument to 
 *	support writing and reading multiple machine profiles.
 */

void _cfg_write_config_record_to_eeprom(uint16_t address)
{
	address = _cfg_compute_record_address(address);
	EEPROM_WriteString(address, cp.record, TRUE);
}

/* config EEPROM address function */

inline uint16_t _cfg_compute_record_address(uint16_t address)
{
	if (cp.param < CFG_AXIS_BASE) {
		address = address + (cp.param * CFG_RECORD_LEN);
	} else {
		address = address + (CFG_AXIS_BASE 
						  + cp.axis * CFG_AXIS_COUNT 
						  + cp.param - CFG_AXIS_BASE) 
						  * CFG_RECORD_LEN;
	}
	return (address);
}

/*
 * cfg_print_help_screen() - send config help screen to stderr
 */

void cfg_print_help_screen()
{
	printf_P(PSTR("Configuration Help\n"));
	return;
}


/*
 * cfg_print_config_struct() - dump configs from internal structure to stderr
 */

char cfgMsgXaxis[] PROGMEM = "X";
char cfgMsgYaxis[] PROGMEM = "Y";
char cfgMsgZaxis[] PROGMEM = "Z";
char cfgMsgAaxis[] PROGMEM = "A";

PGM_P cfgMsgs[] PROGMEM = {	// put string pointer array in program memory
	cfgMsgXaxis,
	cfgMsgYaxis,
	cfgMsgZaxis,
	cfgMsgAaxis
};

void cfg_print_config_struct()
{
	printf_P(PSTR("\n***** CONFIGURATION ****\n"));
	printf_P(PSTR("G-code Model Configuration Values ---\n"));
	printf_P(PSTR("  mm_per_arc_segment:   %5.3f mm / segment\n"), cfg.mm_per_arc_segment);
	printf_P(PSTR(" (maximum_seek_rate:  %7.3f mm / minute)\n"), cfg.max_seek_rate);
	printf_P(PSTR(" (maximum_feed_rate:  %7.3f mm / minute)\n\n"), cfg.max_feed_rate);

	for (uint8_t axis=X; axis<=A; axis++) {
		_cfg_print_axis(axis);
	}
}

void _cfg_print_axis(uint8_t	axis)
{
	printf_P(PSTR("%S Axis Configuration Values\n"),(PGM_P)pgm_read_word(&cfgMsgs[axis]));
	printf_P(PSTR("  seek_steps_sec:  %4d    steps / second (whole steps)\n"), CFG(axis).seek_steps_sec);
	printf_P(PSTR("  feed_steps_sec:  %4d    steps / second (whole steps)\n"), CFG(axis).feed_steps_sec);
	printf_P(PSTR("  microsteps:      %4d    microsteps / whole step\n"), CFG(axis).microstep);
	printf_P(PSTR("  degree_per_step: %7.2f degrees / step (whole steps)\n"), CFG(axis).degree_per_step);
	printf_P(PSTR("  mm_revolution:   %7.2f millimeters / revolution\n"), CFG(axis).mm_per_rev);
	printf_P(PSTR("  mm_travel:       %7.2f millimeters total travel\n"), CFG(axis).mm_travel);
	printf_P(PSTR("  limit_enable:    %4d    1=enabled, 0=disabled\n"), CFG(axis).limit_enable);
	printf_P(PSTR("  low_pwr_idle:    %4d    1=enabled, 0=disabled\n"), CFG(axis).low_pwr_idle);
	printf_P(PSTR("  polarity:        %4d    1=inverted, 0=normal\n"), CFG(axis).polarity);
	printf_P(PSTR(" (steps_per_mm:    %7.2f microsteps / millimeter)\n\n"), CFG(axis).steps_per_mm);
}

/* 
 * _cfg_load_default_settings() - load compiled default settings into strict
 */

void _cfg_load_default_settings()
{
	cfg.gcode_plane = CANON_PLANE_XY;
	cfg.gcode_units = 1;
	cfg.homing_mode = 0;

	cfg.mm_per_arc_segment = MM_PER_ARC_SEGMENT;

	cfg.a[X].seek_steps_sec = X_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Y].seek_steps_sec = Y_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Z].seek_steps_sec = Z_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[A].seek_steps_sec = A_SEEK_WHOLE_STEPS_PER_SEC;

	cfg.a[X].feed_steps_sec = X_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Y].feed_steps_sec = Y_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Z].feed_steps_sec = Z_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[A].feed_steps_sec = A_FEED_WHOLE_STEPS_PER_SEC;

	cfg.a[X].degree_per_step = X_DEGREE_PER_WHOLE_STEP;
	cfg.a[Y].degree_per_step = Y_DEGREE_PER_WHOLE_STEP;
	cfg.a[Z].degree_per_step = Z_DEGREE_PER_WHOLE_STEP;
	cfg.a[A].degree_per_step = A_DEGREE_PER_WHOLE_STEP;

	cfg.a[X].mm_per_rev = X_MM_PER_REVOLUTION;
	cfg.a[Y].mm_per_rev = Y_MM_PER_REVOLUTION;
	cfg.a[Z].mm_per_rev = Z_MM_PER_REVOLUTION;
	cfg.a[A].mm_per_rev = A_MM_PER_REVOLUTION;
	
	cfg.a[X].mm_travel = X_MM_TRAVEL;
	cfg.a[Y].mm_travel = Y_MM_TRAVEL;
	cfg.a[Z].mm_travel = Z_MM_TRAVEL;
	cfg.a[A].mm_travel = A_MM_TRAVEL;
	
	cfg.a[X].microstep = X_MICROSTEPS;
	cfg.a[Y].microstep = Y_MICROSTEPS;
	cfg.a[Z].microstep = Z_MICROSTEPS;
	cfg.a[A].microstep = A_MICROSTEPS;

	cfg.a[X].polarity = X_POLARITY;
	cfg.a[Y].polarity = Y_POLARITY;
	cfg.a[Z].polarity = Z_POLARITY;
	cfg.a[A].polarity = A_POLARITY;

	cfg.a[X].limit_enable = X_LIMIT_ENABLE;
	cfg.a[Y].limit_enable = Y_LIMIT_ENABLE;
	cfg.a[Z].limit_enable = Z_LIMIT_ENABLE;
	cfg.a[A].limit_enable = A_LIMIT_ENABLE;

	cfg.a[X].low_pwr_idle = X_LOW_POWER_IDLE;
	cfg.a[Y].low_pwr_idle = Y_LOW_POWER_IDLE;
	cfg.a[Z].low_pwr_idle = Z_LOW_POWER_IDLE;
	cfg.a[A].low_pwr_idle = A_LOW_POWER_IDLE;

	_cfg_computed();		// generate computed values from the above
}

#ifdef __UNIT_TESTS

void _cfg_test_parse(void);
void _cfg_test_write_config_record(void);

void cfg_tests()
{
//	_cfg_test_parse();
	_cfg_test_write_config_record();
}

void _cfg_test_write_config_record()
{
	_cfg_write_config_record(CFG_GCODE_PLANE, 17, 0); // G17
	_cfg_write_config_record(CFG_GCODE_PLANE, 18, 0); // G18
	_cfg_write_config_record(CFG_GCODE_PLANE, 19, 0); // G19
	_cfg_write_config_record(CFG_GCODE_UNITS, 20, 0); 	// G20
	_cfg_write_config_record(CFG_GCODE_UNITS, 21, 0); 	// G21
	_cfg_write_config_record(CFG_GCODE_HOMING_MODE, 28, 0); // G28 startup homing
	_cfg_write_config_record(CFG_GCODE_FEED_RATE, 400.50, 0); 
	_cfg_write_config_record(CFG_GCODE_SPINDLE_SPEED, 12000, 0);
	_cfg_write_config_record(CFG_GCODE_TOOL, 1, 0);

	_cfg_write_config_record(CFG_MM_PER_ARC_SEGMENT, 0.01, 0); 

	_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 1, 'X'); 
	_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 2, 'Y'); 
	_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 3, 'Z'); 
	_cfg_write_config_record(CFG_MAP_AXIS_TO_MOTOR, 4, 'A'); 

	_cfg_write_config_record(CFG_SEEK_STEPS_MAX, 1500, 'X'); 
	_cfg_write_config_record(CFG_FEED_STEPS_MAX, 1200, 'X'); 
	_cfg_write_config_record(CFG_DEGREES_PER_STEP, 1.8, 'X'); 
	_cfg_write_config_record(CFG_MICROSTEP_MODE, -1, 'X'); 
	_cfg_write_config_record(CFG_POLARITY, 0, 'X'); 
	_cfg_write_config_record(CFG_TRAVEL_MAX, 400, 'X'); 
	_cfg_write_config_record(CFG_TRAVEL_WARN, 425, 'X'); 
	_cfg_write_config_record(CFG_TRAVEL_PER_REV, 1.27, 'X'); 
	_cfg_write_config_record(CFG_IDLE_MODE, 1, 'X'); 
	_cfg_write_config_record(CFG_LIMIT_SWITCH_MODE, 0, 'X'); 

	return;
}

//he1234 (this record currently fails)

char configs_P[] PROGMEM = "\
g17 (XY plane)\n\
g20 (inches mode)\n\
g28 (home on power-up)\n\
f400.00\n\
s12000\n\
t1 \n\
mm per arc segment 0.01\n\
X map axis to motor 1\n\
 xse1891 (leading space)\n\
x feed steps 1892.123456789\n\
XDE1.8\n\
Xmicrosteps -1\n\
Xpolarity 0\n\
Xtravel 400.00\n\
XTW warning 425.00\n\
yRE 1.27\n\
XID1\n\
XLI0\n\
yma2\n\
yse1500\n\
yfe1200\n\
yde1.8\n\
ymi8\n\
ypo1\n\
ytr400\n\
yTW425\n\
yRE1.27\n\
yID1\n\
yLI0\n\
zma3\n\
zse1500\n\
zfe1200\n\
zde1.8\n\
zmi8\n\
zpo0\n\
ztr10\n\
zTW12.5\n\
zRE1.27\n\
zID1\n\
zLI0\n\
ama4\n\
ase1500\n\
afe1200\n\
ade1.8\n\
ami8\n\
apo0\n\
atr65535\n\
aTW65535\n\
aRE1.27\n\
aID1\n\
aLI0\n";

/* generate some strings for the parser and test EEPROM read and write */

void _cfg_test_parse()
{
	char testblock[40];
	char c;
	uint16_t i = 0;		// FLASH buffer index (allow for > 256 characters)
	uint16_t j = 0;		// RAM buffer index (block)

	// feed the parser one line at a time
	while (TRUE) {
		c = pgm_read_byte(&configs_P[i++]);
		if (c == '\n') {					// read a complete line
			testblock[j] = 0;				// terminate the string
			cfg_parse(testblock);			// parse line 
			j = 0;			
		} else if (c == 0) {				// handle the last line
			testblock[j] = 0;
			cfg_parse(testblock);
			break;			
		} else {
			testblock[j++] = c;				// put characters into line
		}
	}
}

#endif
