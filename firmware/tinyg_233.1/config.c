/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * See config.h for a description of CONFIG INTERNALS.
 *	
 * Adding a new config element (  is not very simple :(
 *	- add into cfg struct
 *	- put hardwired defaults into settings.h
 *	- adjust _cfg_load_hardwired_settings()
 *	- adjust cfgTokens to match. Check subsequent defines are still valid
 *	- add mnemonic to table on _cfg_sprintf_as_record(). extend cfgFmt[]
 *	- add to cfg_parse()
 *	- add to _cfg_tokenize_config_record()
 *	- add to _cfg_print_config_record(). extend cfgShow[]. line up COMPUTED_VALUES
 *	- add to _cfg_write_profile_to_NVM()
 *	- attempt compile it and see what you broke by changing the cfg struct
 */
/* TODO:
	- help screen
 */

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "system.h"
#include "settings.h"
#include "stepper.h"
#include "xio.h"
#include "gcode.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "motion_control.h"			// needed only for unit tests
#include "xmega_eeprom.h"

// prototypes for local helper functions
static void _cfg_migrate_config_data(uint16_t address, uint8_t old_length);
static void _cfg_computed(void); 
static void _cfg_normalize_config_block(char *block);
static char *_cfg_format_config_record(char *block);
static uint8_t _cfg_tokenize_config_record();

static void _cfg_load_hardwired_settings(void);

static void _cfg_write_profile_to_NVM(uint16_t base_addr);
static void _cfg_sprintf_as_record(char *record, uint8_t token, char axis, double value);

static void _cfg_write_as_record_to_NVM(uint16_t base_addr, uint8_t token, uint8_t axis, double value);
static void _cfg_write_record_to_NVM(uint16_t base_addr, char *record, uint8_t token, uint8_t axis);
static void _cfg_read_record_from_NVM(uint16_t base_addr, char *record, uint8_t token, uint8_t axis);
static void _cfg_write_to_NVM(uint16_t address, char *record);
static void _cfg_read_from_NVM(uint16_t address, char *record, uint8_t size);
static uint16_t _cfg_compute_record_address(const uint16_t base_addr, uint8_t token, uint8_t axis);
static void _cfg_print_config_record_by_number(uint8_t token);
static void _cfg_print_config_record(char *record);
static void _cfg_print_computed_values(void); 

/* 
 * Config token tokens and config record constants
 *
 * These values are used to tokenize config strings and to compute the NVM 
 * record addresses (_cfg_write_record())
 *
 * There is no trailer token. The trailer record is handled as a special case.
 */

enum cfgTokens {
	// headers and trailers for revision and profile control
	CFG_LENGTH_TOKEN,			// location 0 must be record length value
	CFG_REVISION_TOKEN,			// location 1 must be header revision rec.
	CFG_PROFILE_TOKEN,			// location 2 must be profile number

	// Gcode default settings
	CFG_GCODE_PLANE,			// default gcCanonicalPlane enum (0-2)
	CFG_GCODE_UNITS,			// default 0=inches (G20), 1=mm (G21)
	CFG_GCODE_TOOL,				// default T value
	CFG_GCODE_FEED_RATE,		// default F value
	CFG_GCODE_SPINDLE_SPEED,	// default S value

	// machine default settings
	CFG_MM_PER_ARC_SEGMENT,
	CFG_MAX_LINEAR_JERK,
	CFG_ANGULAR_JERK_UPPER,
	CFG_ANGULAR_JERK_LOWER,
	CFG_HOMING_MODE,
	CFG_A_RADIUS,				// A axis radius for feed rate computation

	// per-axis settings - 4 sets: one for each axis
		CFG_MAP_AXIS_TO_MOTOR,	// the map must be the first axis setting.
		CFG_AXIS_MODE,			// the rest are ordered by convention and 
		CFG_SEEK_STEPS_MAX,		//...the order will be visible to the user 
		CFG_FEED_STEPS_MAX,		//...so try not to change it too much
		CFG_STEP_ANGLE,			// motor parameter
		CFG_MICROSTEP_MODE,		// [1248] will ass morphing [0] at some point
		CFG_POLARITY,			// 0=normal, 1=inverted
		CFG_POWER_MODE,			// 0=none, 1=low power idle
		CFG_LIMIT_MODE,			// 0=off

		CFG_TRAVEL_PER_REV,		// in mm per revolution
		CFG_TRAVEL_MAX,			// work envelope extent
		CFG_TRAVEL_WARN,		// warn the user if travel exceeds this value

		CFG_HOMING_ENABLE,		// 1=enable homing for this axis
		CFG_HOMING_RATE,		// homing feed rate
		CFG_HOMING_OFFSET,		// zero offset for min limit switch
		CFG_HOMING_BACKOFF,		// homing backoff distance

	CFG_TRAILER_TOKEN			// must be last
};

#define CFG_NON_AXIS_BASE CFG_LENGTH_TOKEN		// start of tokens
#define CFG_PER_AXIS_BASE CFG_MAP_AXIS_TO_MOTOR // start of axis tokens
#define CFG_PER_AXIS_COUNT (CFG_TRAILER_TOKEN - CFG_PER_AXIS_BASE)

#define CFG_TRAILER_RECORD (CFG_PER_AXIS_BASE + (4*CFG_PER_AXIS_COUNT))
#define CFG_TRAILER_RECORD_ADDR (CFG_TRAILER_RECORD * CFG_RECORD_LEN)
#define TOO_MANY_RECORDS CFG_TRAILER_RECORD + 10	// safety feature

#define CFG_NON_AXIS_START CFG_NON_AXIS_BASE
#define CFG_NON_AXIS_END CFG_A_RADIUS
#define CFG_X_AXIS_START CFG_PER_AXIS_BASE
#define CFG_X_AXIS_END (CFG_X_AXIS_START + CFG_PER_AXIS_COUNT-1)
#define CFG_Y_AXIS_START (CFG_X_AXIS_END +1)
#define CFG_Y_AXIS_END (CFG_Y_AXIS_START + CFG_PER_AXIS_COUNT-1)
#define CFG_Z_AXIS_START (CFG_Y_AXIS_END +1)
#define CFG_Z_AXIS_END (CFG_Z_AXIS_START + CFG_PER_AXIS_COUNT-1)
#define CFG_A_AXIS_START (CFG_Z_AXIS_END +1)
#define CFG_A_AXIS_END (CFG_A_AXIS_START + CFG_PER_AXIS_COUNT-1)


/* Note: A CFG_RECORD_LEN of 12 will accommodate numbers up to 8 digits
 *	- 7 if it has a decimal point, 6 if it also has a minus sign. 
 *	Numbers with more digits will be truncated from the right. 
 *	This should suffice for any reasonable setting, but if not the 
 *	record length must be increased.
 */

struct cfgConfigParser {
	uint8_t status;				// parser status
	uint8_t token;				// tokenized parameter number
	uint8_t axis;				// internal axis number (0-3 = X-A)
	double value;				// setting value
	uint16_t base_addr;			// base address in NVM (of current profile)
	char record[CFG_RECORD_LEN];// config record for NVM
};
static struct cfgConfigParser cp;

#define GETAXISCHAR(a) ((a==0)?('X'): ((a==1)?('Y'): ((a==2)?('Z'):('A'))))

/*
 * cfg_init() - initialize config system
 */

void cfg_init() 
{
	cp.base_addr = CFG_NVM_BASE;// first (and only) profile
	cfg_reset();				// reset config w/compiled hardwired values
}

/* 
 * cfg_reset() - reset configs (but not the entire config system)
 *
 * 	Reset performs the following actions:
 *		- load config struct with hardwired settings from hardware.h
 *		- if NVM is not initialized:
 *			- initialize NVM
 *			- write the hardwired settings to NVM
 *			- exit
 *		- if NVM is initialized but is not the current revision:
 *			- read settings from NVM into config struct (profile)
 *			- initialize NVM (with new header and trailer)
 *			- write config struct back to the NVM
 *			- exit
 *		- else (NVM is intialized and current):
 *			- read settings from NVM into config struct
 *			  Note that not all settings are required to be in NVM,
 *			  and only those settings in NVM will be loaded 
 *			  (and thus overwrite the hardware.h settings).
 */

void cfg_reset()
{
	uint16_t address = cp.base_addr;
	uint8_t record_len;				// temp record length

	// start by loading hardware.h hardwired settings into cfg struct
	_cfg_load_hardwired_settings();

#ifdef __SIMULATION_MODE
	return;
#endif

	// see if NVM is initialized and take approriate action
	// note the bootstrapped read uses raw reads instead of record reads
	_cfg_read_from_NVM(cp.base_addr, cp.record, CFG_RECORD_LEN);

	// if the header is not initialized, set up EEPROM and exit
	if (cp.record[0] != '_') {
		_cfg_write_profile_to_NVM(cp.base_addr);
		return;
	}

	// check for current config format revision & migrate if out-of-rev
	_cfg_tokenize_config_record();	// tokenize record from previous read 
	record_len = (uint8_t)cp.value;
	_cfg_read_from_NVM((cp.base_addr + record_len), cp.record, cp.value);
	_cfg_tokenize_config_record();	// get revision number

	// if the header is initialized read the NVM configs into the struct
	if (cp.value == CFG_REVISION) {
		for (uint16_t i = 0; i < CFG_TRAILER_RECORD; i++) {
			_cfg_read_from_NVM(address, cp.record, CFG_RECORD_LEN);
			cfg_parse(cp.record, FALSE, FALSE); // don't persist or display
			address += CFG_RECORD_LEN;
		}
	} else { // header is out-of-rev - migrate the old EEPROM data
		_cfg_migrate_config_data(cp.base_addr, record_len);
	}
}

/*
 * _cfg_migrate_config_data() - migrate config data in place
 *
 *	This can get complicated if the record lengths are different
 *	For now this assumes the record lengths are the same (punt)
 * 
 * 	old_length is the length of the records currently in EEPROM
 */

static void _cfg_migrate_config_data(uint16_t address, uint8_t old_length)
{
	uint16_t record_count=0;

	do {
		if (++record_count >= TOO_MANY_RECORDS) { // exit corrupt EEPROM
			break;
		}
		_cfg_read_from_NVM(address, cp.record, old_length);
		cfg_parse(cp.record, FALSE, FALSE); // don't persist or display data
		address += old_length;
	} while (cp.token != CFG_TRAILER_TOKEN);
	_cfg_write_profile_to_NVM(cp.base_addr);
}


/* 
 * cfg_parse() - parse a config line
 *			   - write into config record and persist to NVM
 *
 * Processing steps:
 *	  - normalize config string (block) protocol agnostic cleanup)
 *	  - format config block into a well-formed config record
 *	  - tokenize the record and extract parameter enum, axis & value
 *	  - update config structure with new value
 *	  - persist record to NVM
 *	  - display the update
 */

int cfg_parse(char *block, uint8_t persist, uint8_t display)
{
	cp.status = TG_OK;

	// cutout for header and trailer blocks. Don't parse them
	if (block[0] == '_') {
		return (TG_OK);
	}

	// normalize the block in place
	_cfg_normalize_config_block(block);

	// dispatch on special characters in the first byte location
	switch (block[0]) {
		case  0:  return(TG_OK);			// ignore comments (stripped)
		case 'Q': return(TG_QUIT);			// quit config mode
///		case 'H': cfg_print_help_screen(); return(TG_OK);
		case '?': cfg_print_config_records(block); return(TG_OK);
	}

	// create a well-formed config record from the normalized block
	_cfg_format_config_record(block);		
	
	// parse the config record into a parser structure (or die trying)
	if (_cfg_tokenize_config_record()) {
		tg_print_status(cp.status, block);
	}

	// load value into cfg struct based on parameter type
	switch (cp.token) {

		// gcode defaults
		case CFG_GCODE_PLANE:		cfg.gcode_plane = (uint8_t)cp.value; break;
		case CFG_GCODE_UNITS:	  	cfg.gcode_units = (uint8_t)cp.value; break;
		case CFG_GCODE_TOOL:		cfg.gcode_tool = (uint8_t)cp.value; break;
		case CFG_GCODE_FEED_RATE:	cfg.gcode_feed_rate = cp.value; break;
		case CFG_GCODE_SPINDLE_SPEED: cfg.gcode_spindle_speed = cp.value; break;

		// non-axis settings
		case CFG_MM_PER_ARC_SEGMENT:cfg.mm_per_arc_segment = cp.value; break;
		case CFG_MAX_LINEAR_JERK:	cfg.max_linear_jerk = cp.value * 1000; break;
		case CFG_ANGULAR_JERK_UPPER:cfg.angular_jerk_upper = cp.value; break;
		case CFG_ANGULAR_JERK_LOWER:cfg.angular_jerk_lower = cp.value; break;
		case CFG_HOMING_MODE:		cfg.homing_mode = (uint8_t)cp.value; break;
		case CFG_A_RADIUS:			cfg.a_radius = cp.value; break;

		// per-axis settings
		case CFG_MAP_AXIS_TO_MOTOR:	CFG(cp.axis).map_axis = (uint8_t)cp.value; break;
		case CFG_AXIS_MODE:			CFG(cp.axis).axis_mode = (uint8_t)cp.value; break;

		case CFG_SEEK_STEPS_MAX: 	CFG(cp.axis).seek_steps_sec = (uint16_t)cp.value; break;
		case CFG_FEED_STEPS_MAX: 	CFG(cp.axis).feed_steps_sec = (uint16_t)cp.value; break;
		case CFG_STEP_ANGLE:		CFG(cp.axis).step_angle = cp.value; break;

		case CFG_MICROSTEP_MODE:	CFG(cp.axis).microstep_mode = (uint8_t)cp.value;
									st_set_microsteps(cp.axis, CFG(cp.axis).microstep_mode); break;

		case CFG_POLARITY:			CFG(cp.axis).polarity = (uint8_t)cp.value; 
					  		 		st_set_polarity(cp.axis, CFG(cp.axis).polarity); break;

		case CFG_POWER_MODE: 		CFG(cp.axis).power_mode = (uint8_t)cp.value; break;
		case CFG_LIMIT_MODE: 		CFG(cp.axis).limit_mode = (uint8_t)cp.value; break;

		case CFG_TRAVEL_PER_REV: 	CFG(cp.axis).travel_rev = cp.value; break;
		case CFG_TRAVEL_MAX: 		CFG(cp.axis).travel_max = cp.value; break;
		case CFG_TRAVEL_WARN: 		CFG(cp.axis).travel_warn = cp.value; break;

		case CFG_HOMING_ENABLE: 	CFG(cp.axis).homing_enable = (uint8_t)cp.value; break;
		case CFG_HOMING_RATE: 		CFG(cp.axis).homing_rate = cp.value; break;
		case CFG_HOMING_OFFSET: 	CFG(cp.axis).homing_offset = cp.value; break;
		case CFG_HOMING_BACKOFF: 	CFG(cp.axis).homing_backoff = cp.value; break;

		default: cp.status = TG_UNRECOGNIZED_COMMAND;	// error return
	}
	_cfg_computed();		// generate & (re)populate computed config values

	if (persist) {			// save config record in NVM
		_cfg_write_record_to_NVM(cp.base_addr, cp.record, cp.token, cp.axis);
	}
	if (display) {			// do config displays
		_cfg_read_record_from_NVM (cp.base_addr, cp.record, cp.token, cp.axis);
		_cfg_print_config_record(cp.record);
	}
	return (cp.status);
}

/* 
 * _cfg_computed() - helper function to generate computed config values 
 *	
 *	This routine should be called every time you change any configs
 *
 *	The A axis max_seek_rate and max_feed_rate are computed in mm/min
 *	by multiplying degrees/min by the A radius value / one radian. 
 *	If you actually want A in degrees / min set A radius to one radian.
 *
 *	This function will need to be rethought when microstep morphing is 
 *	implemented, as microsteps are calculated statically. Dang.
 */

inline static void _cfg_computed() 
{
	// linear axes are in mm/min, A axis is in degrees/min becuase travel/rev is as well.
	for (uint8_t i = X; i <= A; i++) {
		//  = 360 / (step_angle / microstep) / travel_per_rev
		CFG(i).steps_per_unit = (360 / (CFG(i).step_angle / 
								 CFG(i).microstep_mode)) / CFG(i).travel_rev;

		//  = 60 * seek_steps_sec / (360 / step_angle / travel_rev)
		CFG(i).max_seek_rate = ((60 * (double)CFG(i).seek_steps_sec) /
							 	(360/CFG(i).step_angle / CFG(i).travel_rev));

		//  = 60 * feed_steps_sec / (360 / step_angle / travel_rev)
		CFG(i).max_feed_rate = ((60 * (double)CFG(i).feed_steps_sec) /
							   (360/CFG(i).step_angle / CFG(i).travel_rev));
	}
	CFG(A).max_seek_rate *= (cfg.a_radius / RADIAN); // A axis to mm/min
	CFG(A).max_feed_rate *= (cfg.a_radius / RADIAN);
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
 *		- . ? 						sign, dot, question mark pass to parser
 *
 *	Invalid characters (these are stripped but don't cause failure):
 *		control characters			chars < 0x20 are all removed
 *		/ *	< = > | % #	+			expression chars removed from string
 *		( ) [ ] { } 				expression chars removed from string
 *		<sp> <tab> 					whitespace chars removed from string
 *		! $ % ,	; ; @ 				removed
 *		^ ~ " ' <DEL>				removed
 *		_							underscore is removed, but has already
 *									been trapped in header and trailers 
 *									before this routine has been called.
 */

static void _cfg_normalize_config_block(char *block) 
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
 * _cfg_format_config_record() - create an NVM record from a block
 *
 *	Converts a block into a record: parser and NVM friendly form
 *	Block must be pre-normalized (uppercase, no WS, comments, illegal chars)
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

static char *_cfg_format_config_record(char *block)
{
	uint8_t i=0; 				// char index into block
	uint8_t j=0; 				// char index into record

	// cutout for null block
	if (!block[0]) {
		return (0);
	}
	memcpy(cp.record, block, CFG_RECORD_LEN);// initialize record string

	if (isdigit(block[1])) {			// handle Gcode settings
		return (block);					// OK as-is. Return it.
	}
	switch(block[0]) {
		case 'M': { i=1; j=2; break; }	// handle non-axis settings (only 1 for now)
		default:  { i=2; j=3; break; }	// handle axis settings
	}
	while (isupper(block[++i])) {		// position to value by advancing
	}									//...past any remaining tag alphas
	while (block[0] && j < CFG_RECORD_LEN) {	// copy value to NVM record
		cp.record[j++] = block[i++];
	}
	cp.record[j] = 0;					// terminate string
	return (cp.record);
}

/*
 * _cfg_tokenize_config_record() - parse record into a parser token struct
 *
 *	Return TG_UNRECOGNIZED_COMMAND if there are any errors
 *	Block must be normalized w/comments removed.
 */

static uint8_t _cfg_tokenize_config_record()
{
	uint8_t i=0; 							// char index into record
	char *end;								// pointer to end of value

	// get the value
	while (isupper(cp.record[++i])) {		// locate start of value
	}										// & skip leading digit
	cp.value = strtod(&cp.record[i], &end); // extract the value
	cp.axis = -1;

	// get the token enum and the axis, if there is one
	switch(cp.record[0]) {
		// gcode settings
		case 'G': {
			switch ((int)cp.value) {
				case 17: { cp.token = CFG_GCODE_PLANE; cp.value = CANON_PLANE_XY; break; }
				case 18: { cp.token = CFG_GCODE_PLANE; cp.value = CANON_PLANE_XZ; break; }
				case 19: { cp.token = CFG_GCODE_PLANE; cp.value = CANON_PLANE_YZ; break; }
				case 20: { cp.token = CFG_GCODE_UNITS; cp.value = 0; break; }
				case 21: { cp.token = CFG_GCODE_UNITS; cp.value = 1; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);
		}
		case 'T': { cp.token = CFG_GCODE_TOOL; break ;}
		case 'F': { cp.token = CFG_GCODE_FEED_RATE; break; }
		case 'S': { cp.token = CFG_GCODE_SPINDLE_SPEED; break ;}

		// non-axis settings
		case 'M':
			switch (cp.record[1]) {
				case 'M': { cp.token = CFG_MM_PER_ARC_SEGMENT; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);

		case 'J': 
			switch (cp.record[1]) {
				case 'M': { cp.token = CFG_MAX_LINEAR_JERK; break; }
				case 'U': { cp.token = CFG_ANGULAR_JERK_UPPER; break; }
				case 'L': { cp.token = CFG_ANGULAR_JERK_LOWER; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);

		case 'H': 
			switch (cp.record[1]) {
				case 'O': { cp.token = CFG_HOMING_MODE; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);

		case 'R': 
			switch (cp.record[1]) {
				case 'A': { cp.token = CFG_A_RADIUS; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);

		case '_': 
			switch (cp.record[1]) {
				case 'L': { cp.token = CFG_LENGTH_TOKEN; break; }
				case 'R': { cp.token = CFG_REVISION_TOKEN; break; }
				case 'P': { cp.token = CFG_PROFILE_TOKEN; break; }
				case 'T': { cp.token = CFG_TRAILER_TOKEN; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			} return (TG_OK);

		// axis and mapped axis settings by axis letter and by motor number
		case 'X': case '1': { cp.axis = 0; break;}
		case 'Y': case '2': { cp.axis = 1; break;}
		case 'Z': case '3': { cp.axis = 2; break;}
		case 'A': case '4': { cp.axis = 3; break;}

		// has to have been one of the above or it's an error
		default: return(TG_UNRECOGNIZED_COMMAND);
	}
	// pick apart the axis settings if an axis was set
	if (cp.axis != -1) {
		switch(cp.record[1]) {
			case 'S':
				switch (cp.record[2]) {
					case 'E': { cp.token = CFG_SEEK_STEPS_MAX; break; }
					case 'T': { cp.token = CFG_STEP_ANGLE; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'F': 
				switch (cp.record[2]) {
					case 'E': { cp.token = CFG_FEED_STEPS_MAX; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'P': 
				switch (cp.record[2]) {
					case 'O': { cp.token = CFG_POLARITY; break; }
					case 'W': { cp.token = CFG_POWER_MODE; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'T': 
				switch (cp.record[2]) {
					case 'M': { cp.token = CFG_TRAVEL_MAX; break; }
					case 'W': { cp.token = CFG_TRAVEL_WARN; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);


			case 'H': 
				switch (cp.record[2]) {
					case 'E': { cp.token = CFG_HOMING_ENABLE; break; }
					case 'R': { cp.token = CFG_HOMING_RATE; break; }
					case 'O': { cp.token = CFG_HOMING_OFFSET; break; }
					case 'B': { cp.token = CFG_HOMING_BACKOFF; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'R':
				switch (cp.record[2]) {
					case 'E': { cp.token = CFG_TRAVEL_PER_REV; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'L':
				switch (cp.record[2]) {
					case 'I': { cp.token = CFG_LIMIT_MODE; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);

			case 'M': 
				switch (cp.record[2]) {
					case 'A': { cp.token = CFG_MAP_AXIS_TO_MOTOR; break; }
					case 'O': { cp.token = CFG_AXIS_MODE; break; }
					case 'I': { cp.token = CFG_MICROSTEP_MODE; break; }
					default: return(TG_UNRECOGNIZED_COMMAND);
				} return (TG_OK);
			default: return(TG_UNRECOGNIZED_COMMAND);
		}
	}
	return (TG_OK);
}

/*
 * cfg_print_config_records() - dump configs from NVM to stderr
 * _cfg_print_config_record_by_number()
 * _cfg_print_config_record()
 * _cfg_print_computed_values()
 *
 *  Takes a config record as input - record must obey record formatting
 *	Uses global cp struct to tokenize and extract values from record
 */

// put record print format strings in program memory
char cfs00[] PROGMEM = "Length   %3.0f";
char cfs01[] PROGMEM = "Revision %3.0f";
char cfs02[] PROGMEM = "Profile  %3.0f";

char cfs03[] PROGMEM = "  Gcode: {G17/G18/G19}    Plane   %1.0f";
char cfs04[] PROGMEM = "  Gcode: {G20/G21} Units (1=mm)   %1.0f";
char cfs05[] PROGMEM = "  Gcode: {T} Tool                 %1.0f";
char cfs06[] PROGMEM = "  Gcode: {F} Feed rate     %8.2f";
char cfs07[] PROGMEM = "  Gcode: {S} Spindle speed %8.2f";

char cfs08[] PROGMEM = "  Millimeters / arc segment   %5.3f";
char cfs09[] PROGMEM = "  Max linear jerk /1000	 %10.0f";
char cfs10[] PROGMEM = "  Angular jerk upper thresh   %1.3f";
char cfs11[] PROGMEM = "  Angular jerk lower thresh   %1.3f";
char cfs12[] PROGMEM = "  Homing mode (1=power-on)        %1.0f";
char cfs13[] PROGMEM = "  A axis radius              %5.3f";

char cfs14[] PROGMEM = "%c axis mapped to motor number  %4.0f";
char cfs15[] PROGMEM = "  %c axis - Axis mode          %5.0f";
char cfs16[] PROGMEM = "  %c axis - Seek steps / sec   %5.0f";
char cfs17[] PROGMEM = "  %c axis - Feed steps / sec   %5.0f";
char cfs18[] PROGMEM = "  %c axis - Step angle (deg)   %5.3f";
char cfs19[] PROGMEM = "  %c axis - Microstep mode     %5.0f";
char cfs20[] PROGMEM = "  %c axis - Motor polarity     %5.0f";
char cfs21[] PROGMEM = "  %c axis - Power mgmt mode    %5.0f";
char cfs22[] PROGMEM = "  %c axis - Limit switch mode  %5.0f";

char cfs23[] PROGMEM = "  %c axis - Travel / rev       %5.2f";
char cfs24[] PROGMEM = "  %c axis - Travel maximum     %5.0f";
char cfs25[] PROGMEM = "  %c axis - Travel warning     %5.0f";

char cfs26[] PROGMEM = "  %c axis - Homing enabled     %5.0f";
char cfs27[] PROGMEM = "  %c axis - Homing feed rate   %5.0f";
char cfs28[] PROGMEM = "  %c axis - Homing offset      %5.0f";
char cfs29[] PROGMEM = "  %c axis - Homing backoff     %5.0f";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P cfgShow[] PROGMEM = {	
	cfs00, cfs01, cfs02, cfs03, cfs04, cfs05, cfs06, cfs07, cfs08, cfs09,
	cfs10, cfs11, cfs12, cfs13, cfs14, cfs15, cfs16, cfs17,	cfs18, cfs19,
	cfs20, cfs21, cfs22, cfs23, cfs24, cfs25, cfs26, cfs27, cfs28, cfs29
};

// Loose strings - not in array
char cfsMSG[] PROGMEM = "**** Type ?X to see X axis values - similar for other axes ****\n";

void cfg_print_config_records(char *block)
{
	uint16_t i;

	if ((block[1] == 'X') || (block[1] == '*')){
		for (i = CFG_X_AXIS_START; i < CFG_X_AXIS_END; i++) {
			_cfg_print_config_record_by_number(i);
		}
	} else if ((block[1] == 'Y') || (block[1] == '*')){
		for (i = CFG_Y_AXIS_START; i < CFG_Y_AXIS_END; i++) {
			_cfg_print_config_record_by_number(i);
		}
	} else if ((block[1] == 'Z') || (block[1] == '*')){
		for (i = CFG_Z_AXIS_START; i < CFG_Z_AXIS_END; i++) {
			_cfg_print_config_record_by_number(i);
		}
	} else if ((block[1] == 'A') || (block[1] == '*')){
		for (i = CFG_A_AXIS_START; i < CFG_A_AXIS_END; i++) {
			_cfg_print_config_record_by_number(i);
		}
	} else {
		printf_P(cfsMSG);
		for (i = CFG_NON_AXIS_START; i < CFG_NON_AXIS_END; i++) {
			_cfg_print_config_record_by_number(i);
		}
		_cfg_print_computed_values();
	}
}

static void _cfg_print_config_record_by_number(uint8_t token) 
{
	uint16_t address = (cp.base_addr + (token * CFG_RECORD_LEN));
	_cfg_read_from_NVM(address, cp.record, CFG_RECORD_LEN);
	_cfg_print_config_record(cp.record);
}

inline static void _cfg_print_config_record(char *record)
{
	_cfg_tokenize_config_record();
	if (cp.token < CFG_PER_AXIS_BASE) {
		printf_P((PGM_P)pgm_read_word(&cfgShow[cp.token]), cp.value);
	} else {
		char axis = GETAXISCHAR(cp.axis);
		printf_P((PGM_P)pgm_read_word(&cfgShow[cp.token]), axis, cp.value);
	}
	printf_P(PSTR("     %-12s\n"), record);	// Must agree with CFG_RECORD_LEN
}

// strings for print computed values
char cvsMSG[] PROGMEM = "Derived: max seek rate, max feed rate (change steps / sec to set)\n";
char cvsMM[] PROGMEM = "  %c axis - %7.2f        %7.2f mm/min\n";
char cvsIN[] PROGMEM = "  %c axis - %7.2f        %7.2f in/min\n";
char cvsDEG[] PROGMEM = "  %c axis - %7.2f        %7.2f degrees/min\n";

static void _cfg_print_computed_values() 
{
	printf_P((PGM_P)&cvsMSG);

	for (uint8_t i=X; i<=Z; i++) {
		char axis = GETAXISCHAR(i);
		if (cfg.gcode_units) {	// TRUE = mm
			printf_P((PGM_P)&cvsMM, axis, CFG(i).max_seek_rate, CFG(i).max_feed_rate);
		} else {
			printf_P((PGM_P)&cvsIN, axis, CFG(i).max_seek_rate, CFG(i).max_feed_rate);
		}
	}
	printf_P((PGM_P)&cvsDEG, 'A', CFG(A).max_seek_rate, CFG(A).max_feed_rate);
}

/*
 * _cfg_sprintf_as_record() - make a formatted config record from parameters
 *
 *	record			string to write to
 *	token			token enumneration
 *	value			value loaded as a double
 *	axis			axis (numeric)
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
// THE FORMAT OF THE HEADER/TRAILER TOKENS SHOULD NEVER CHANGE! MUST BE "_x"
char cff00[] PROGMEM = "_L%1.0f";	// record length token	 (header)
char cff01[] PROGMEM = "_R%1.0f";	// config revision token (header)
char cff02[] PROGMEM = "_P%1.0f";	// profile number token	 (header)

char cff03[] PROGMEM = "G%1.0f";	// Plane G17/G18/G19
char cff04[] PROGMEM = "G%1.0f";	// Units G20/G21
char cff05[] PROGMEM = "T%1.0f";	// T Tool
char cff06[] PROGMEM = "F%1.3f";	// F Feed rate
char cff07[] PROGMEM = "S%1.2f";	// S Spindle speed

char cff08[] PROGMEM = "MM%1.3f";	// MM per arc segment
char cff09[] PROGMEM = "JM%6.0f";	// max linear jerk / 1000
char cff10[] PROGMEM = "JU%1.4f";	// angular jerk upper threshold
char cff11[] PROGMEM = "JL%1.4f";	// angular jerk lower threshold
char cff12[] PROGMEM = "HO%1.3f";	// Homing mode
char cff13[] PROGMEM = "RA%1.3f";	// A radius

char cff14[] PROGMEM = "%cMA%1.0f";	// Map axis to motor
char cff15[] PROGMEM = "%cMO%1.0f";	// Axis operating mode
char cff16[] PROGMEM = "%cSE%1.0f";	// Seek max steps per second
char cff17[] PROGMEM = "%cFE%1.0f";	// Feed max steps per sec
char cff18[] PROGMEM = "%cST%1.3f";	// Step angle (degrees per step)
char cff19[] PROGMEM = "%cMI%1.0f";	// Microstep mode
char cff20[] PROGMEM = "%cPO%1.0f";	// Polarity
char cff21[] PROGMEM = "%cPW%1.0f";	// Power management mode
char cff22[] PROGMEM = "%cLI%1.0f";	// Limit switch mode

char cff23[] PROGMEM = "%cRE%1.3f";	// Travel per revolution (mm, degrees)
char cff24[] PROGMEM = "%cTM%1.0f";	// Travel max (mm)
char cff25[] PROGMEM = "%cTW%1.0f";	// Travel warning

char cff26[] PROGMEM = "%cHE%1.0f";	// Homing enable
char cff27[] PROGMEM = "%cHR%1.0f";	// Homing rate
char cff28[] PROGMEM = "%cHO%1.0f";	// Homing offset (mm)
char cff29[] PROGMEM = "%cHB%1.0f";	// Homing backoff (mm)

char cff30[] PROGMEM = "_T%1.0f";	// trailer token (trailer)

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P cfgFmt[] PROGMEM = {	
	cff00, cff01, cff02, cff03, cff04, cff05, cff06, cff07, cff08, cff09,
	cff10, cff11, cff12, cff13, cff14, cff15, cff16, cff17, cff18, cff19,
	cff20, cff21, cff22, cff23, cff24, cff25, cff26, cff27, cff28, cff29,
	cff30
};

static void _cfg_sprintf_as_record(char *record, uint8_t token, char axis, double value)
{
	if (token == CFG_TRAILER_TOKEN) {
		sprintf_P(record,(PGM_P)pgm_read_word(&cfgFmt[token]), value);
	} else if (token < CFG_PER_AXIS_BASE) {
		sprintf_P(record,(PGM_P)pgm_read_word(&cfgFmt[token]), value);
	} else {
		char axisc = GETAXISCHAR(axis);
		sprintf_P(record,(PGM_P)pgm_read_word(&cfgFmt[token]), axisc, value);
	}
}

/*
 * _cfg_write_profile_to_NVM()
 *
 * Write headers, trailer, and entire config structure to NVM
 */

static void _cfg_write_profile_to_NVM(uint16_t base_addr)
{
	// write header and trailer records
	_cfg_write_as_record_to_NVM(base_addr, CFG_LENGTH_TOKEN, 0, CFG_RECORD_LEN);
	_cfg_write_as_record_to_NVM(base_addr, CFG_REVISION_TOKEN, 0, CFG_REVISION);
	_cfg_write_as_record_to_NVM(base_addr, CFG_PROFILE_TOKEN, 0, CFG_PROFILE);
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAILER_TOKEN, 0, CFG_PROFILE);

	// Gcode settings
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, (17 + cfg.gcode_plane));
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, (20 + cfg.gcode_units));
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_TOOL, 0, cfg.gcode_tool);
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_FEED_RATE, 0, cfg.gcode_feed_rate); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_SPINDLE_SPEED, 0, cfg.gcode_spindle_speed);

	// non-axis machine settings
	_cfg_write_as_record_to_NVM(base_addr, CFG_MM_PER_ARC_SEGMENT, 0, cfg.mm_per_arc_segment); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAX_LINEAR_JERK, 0, cfg.max_linear_jerk/1000); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_ANGULAR_JERK_UPPER, 0, cfg.angular_jerk_upper); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_ANGULAR_JERK_LOWER, 0, cfg.angular_jerk_lower); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_MODE, 0, cfg.homing_mode);
	_cfg_write_as_record_to_NVM(base_addr, CFG_A_RADIUS, 0, cfg.a_radius);

	// per-axis settings
	for (uint8_t axis = 0; axis <= 3; axis++) {	// cycle thru axes X,Y,Z,A

		_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, axis,
											   CFG(axis).map_axis+1); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_AXIS_MODE, axis,
											   CFG(axis).axis_mode); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_SEEK_STEPS_MAX, axis,
											   CFG(axis).seek_steps_sec); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_FEED_STEPS_MAX, axis,
											   CFG(axis).feed_steps_sec); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_STEP_ANGLE, axis,
											   CFG(axis).step_angle); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_MICROSTEP_MODE, axis,
											   CFG(axis).microstep_mode); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_POLARITY, axis,
											   CFG(axis).polarity);

		_cfg_write_as_record_to_NVM(base_addr, CFG_POWER_MODE, axis,
											   CFG(axis).power_mode); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_LIMIT_MODE, axis,
											   CFG(axis).limit_mode); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_PER_REV, axis,
											   CFG(axis).travel_rev); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_MAX, axis,
											   CFG(axis).travel_max); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_WARN, axis,
											   CFG(axis).travel_warn); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_ENABLE, axis,
											   CFG(axis).homing_enable); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_RATE, axis,
											   CFG(axis).homing_rate); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_OFFSET, axis,
											   CFG(axis).homing_offset); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_BACKOFF, axis,
											   CFG(axis).homing_backoff);
	}
}

/*
 * _cfg_write_as_record_to_NVM() - format a config record and write to NVM
 *
 *	base_addr		base address in NVM for this profile
 *	token			token enumneration
 *	value			value loaded as a double
 *	axis			axis (numeric)
 */

static void _cfg_write_as_record_to_NVM(uint16_t base_addr, uint8_t token, uint8_t axis, double value)
{
	uint16_t address;
	char record[CFG_RECORD_LEN];

	_cfg_sprintf_as_record(record, token, axis, value);
	address = _cfg_compute_record_address(base_addr, token, axis);
	_cfg_write_to_NVM(address, record);
}

/* 
 * _cfg_write_record_to_NVM()  - write NVM record given token & axis values
 * _cfg_read_record_from_NVM() - read NVM record given token & axis values
 * _cfg_write_to_NVM() 		   - raw write to NVM w/no address calculation
 * _cfg_read_from_NVM() 	   - raw read from NVM w/no address calculation
 *
 *	The base address of the record-set is provided as an argument to 
 *	support writing and reading multiple machine profiles.
 */

static void _cfg_write_record_to_NVM(uint16_t base_addr, char *record, uint8_t token, uint8_t axis)
{
	uint16_t address = _cfg_compute_record_address(base_addr, token, axis);
	_cfg_write_to_NVM(address, record);
}

static void _cfg_read_record_from_NVM(uint16_t base_addr, char *record, uint8_t token, uint8_t axis)
{
	uint16_t address = _cfg_compute_record_address(base_addr, token, axis);
	_cfg_read_from_NVM(address, record, CFG_RECORD_LEN);
  	return;
}

inline static void _cfg_write_to_NVM(uint16_t address, char *record)
{
	EEPROM_WriteString(address, record, TRUE);
}

inline static void _cfg_read_from_NVM(uint16_t address, char *record, uint8_t size)
{
	EEPROM_ReadString(address, record, size);
}

/*
 * _cfg_compute_record_address() - compute NVM address
 *
 * The record address in NVM is computed as follows:
 *	- header record - identifies revision and carries record length
 *	- Gcode settings		(identified by token < CFG_AXIS_BASE)
 *	- non-axis settings		(identified by token < CFG_AXIS_BASE)
 *	- per-axis settings		(identified by token >= CFG_AXIS_BASE)
 *	- trailer record is a pre-calculated value
 */

inline static uint16_t _cfg_compute_record_address(const uint16_t base_addr, uint8_t token, uint8_t axis)
{
	if (token == CFG_TRAILER_TOKEN) {
		return (base_addr + CFG_TRAILER_RECORD_ADDR);
	} else if (token < CFG_PER_AXIS_BASE) {
		return (base_addr + (token * CFG_RECORD_LEN));
	} else {
		return (base_addr + (CFG_PER_AXIS_BASE + 
							(axis * CFG_PER_AXIS_COUNT) + 
							 token - CFG_PER_AXIS_BASE) * 
							 CFG_RECORD_LEN);
	}
}

/* 
 * _cfg_load_hardwired_settings() - load compiled hardwired settings into strict
 */

void static _cfg_load_hardwired_settings()
{
	cfg.gcode_plane = CANON_PLANE_XY;
	cfg.gcode_units = GCODE_UNITS;
	cfg.gcode_path_control = GCODE_PATH_CONTROL;
	cfg.gcode_tool = GCODE_TOOL;
	cfg.gcode_feed_rate = GCODE_FEED_RATE;
	cfg.gcode_spindle_speed = GCODE_SPINDLE_SPEED;

	cfg.mm_per_arc_segment = MM_PER_ARC_SEGMENT;
	cfg.min_segment_time = MIN_SEGMENT_TIME;
	cfg.max_linear_jerk = MAX_LINEAR_JERK;
	cfg.angular_jerk_upper = ANGULAR_JERK_UPPER_THRESHOLD;
	cfg.angular_jerk_lower = ANGULAR_JERK_LOWER_THRESHOLD;
	cfg.homing_mode = HOMING_MODE;
	cfg.a_radius = A_RADIUS;

	cfg.a[X].map_axis = X_MAP_AXIS;
	cfg.a[Y].map_axis = Y_MAP_AXIS;
	cfg.a[Z].map_axis = Z_MAP_AXIS;
	cfg.a[A].map_axis = A_MAP_AXIS;

	cfg.a[X].axis_mode = X_AXIS_MODE;
	cfg.a[Y].axis_mode = Y_AXIS_MODE;
	cfg.a[Z].axis_mode = Z_AXIS_MODE;
	cfg.a[A].axis_mode = A_AXIS_MODE;

	cfg.a[X].seek_steps_sec = X_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Y].seek_steps_sec = Y_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Z].seek_steps_sec = Z_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[A].seek_steps_sec = A_SEEK_WHOLE_STEPS_PER_SEC;

	cfg.a[X].feed_steps_sec = X_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Y].feed_steps_sec = Y_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Z].feed_steps_sec = Z_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[A].feed_steps_sec = A_FEED_WHOLE_STEPS_PER_SEC;

	cfg.a[X].step_angle = X_STEP_ANGLE;
	cfg.a[Y].step_angle = Y_STEP_ANGLE;
	cfg.a[Z].step_angle = Z_STEP_ANGLE;
	cfg.a[A].step_angle = A_STEP_ANGLE;

	cfg.a[X].microstep_mode = X_MICROSTEP_MODE;
	cfg.a[Y].microstep_mode = Y_MICROSTEP_MODE;
	cfg.a[Z].microstep_mode = Z_MICROSTEP_MODE;
	cfg.a[A].microstep_mode = A_MICROSTEP_MODE;

	cfg.a[X].polarity = X_POLARITY;
	cfg.a[Y].polarity = Y_POLARITY;
	cfg.a[Z].polarity = Z_POLARITY;
	cfg.a[A].polarity = A_POLARITY;

	cfg.a[X].power_mode = X_POWER_MODE;
	cfg.a[Y].power_mode = Y_POWER_MODE;
	cfg.a[Z].power_mode = Z_POWER_MODE;
	cfg.a[A].power_mode = A_POWER_MODE;

	cfg.a[X].limit_mode = X_LIMIT_MODE;
	cfg.a[Y].limit_mode = Y_LIMIT_MODE;
	cfg.a[Z].limit_mode = Z_LIMIT_MODE;
	cfg.a[A].limit_mode = A_LIMIT_MODE;

	cfg.a[X].travel_rev = X_TRAVEL_PER_REV;
	cfg.a[Y].travel_rev = Y_TRAVEL_PER_REV;
	cfg.a[Z].travel_rev = Z_TRAVEL_PER_REV;
	cfg.a[A].travel_rev = A_TRAVEL_PER_REV;
	
	cfg.a[X].travel_max = X_TRAVEL_MAX;
	cfg.a[Y].travel_max = Y_TRAVEL_MAX;
	cfg.a[Z].travel_max = Z_TRAVEL_MAX;
	cfg.a[A].travel_max = A_TRAVEL_MAX;
	
	cfg.a[X].travel_warn = X_TRAVEL_WARN;
	cfg.a[Y].travel_warn = Y_TRAVEL_WARN;
	cfg.a[Z].travel_warn = Z_TRAVEL_WARN;
	cfg.a[A].travel_warn = A_TRAVEL_WARN;

	cfg.a[X].homing_enable = X_HOMING_ENABLE;
	cfg.a[Y].homing_enable = Y_HOMING_ENABLE;
	cfg.a[Z].homing_enable = Z_HOMING_ENABLE;
	cfg.a[A].homing_enable = A_HOMING_ENABLE;

	cfg.a[X].homing_rate = X_HOMING_RATE;
	cfg.a[Y].homing_rate = Y_HOMING_RATE;
	cfg.a[Z].homing_rate = Z_HOMING_RATE;
	cfg.a[A].homing_rate = A_HOMING_RATE;

	cfg.a[X].homing_offset = X_HOMING_OFFSET;
	cfg.a[Y].homing_offset = Y_HOMING_OFFSET;
	cfg.a[Z].homing_offset = Z_HOMING_OFFSET;
	cfg.a[A].homing_offset = A_HOMING_OFFSET;

	cfg.a[X].homing_backoff = X_HOMING_BACKOFF;
	cfg.a[Y].homing_backoff = Y_HOMING_BACKOFF;
	cfg.a[Z].homing_backoff = Z_HOMING_BACKOFF;
	cfg.a[A].homing_backoff = A_HOMING_BACKOFF;

	_cfg_computed();		// generate computed values from the above
}

/*
 * cfg_print_help_screen() - send config help screen to stderr
 */

void cfg_print_help_screen()
{
	printf_P(PSTR("Configuration Help\n"));
	return;
}

//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _cfg_test_reset(void);
void _cfg_test_parse(void);
void _cfg_test_write_record(void);

void cfg_tests()
{
	_cfg_test_reset();
//	_cfg_test_parse();
//	_cfg_test_write_record();
}

void _cfg_test_reset()
{
	// The first reset is done by the init system. If nnvm is used it will
	// perform an uninitialized reset. Trace this at cfg_reset()
//	cfg_reset();

	// The second reset finds an initialized "EEPROM" at current revision . 
	// It reads the EEPROM into the cfg struct
//	cfg_reset();

	// The third reset is initialized but out-of-rev
	// It initiates a config migration followed by a config load
	_cfg_write_to_NVM(CFG_NVM_BASE + CFG_RECORD_LEN, "_R999");
	cfg_reset();
}

void _cfg_test_write_record()	// (outdated)
{
	uint16_t base_addr = 0;

	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 17); // G17
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 18); // G18
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 19); // G19
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, 20); // G20
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, 21); // G21
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_TOOL, 0, 1);
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_FEED_RATE, 0, 400.50); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_SPINDLE_SPEED, 0, 12000);

	_cfg_write_as_record_to_NVM(base_addr, CFG_MM_PER_ARC_SEGMENT, 0, MM_PER_ARC_SEGMENT); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_HOMING_MODE, 0, CFG_HOMING_MODE);

	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, X, 1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, Y, 2); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, Z, 3); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, A, 4); 

	_cfg_write_as_record_to_NVM(base_addr, CFG_SEEK_STEPS_MAX, X, 1500); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_FEED_STEPS_MAX, X, 1200); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_STEP_ANGLE, X, 1.8); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MICROSTEP_MODE, X, -1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_POLARITY, X, 0); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_MAX, X, 400); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_WARN, X, 425); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_PER_REV, X, 1.27); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_POWER_MODE, X, 1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_LIMIT_MODE, X, 0); 

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
 xse1600 (leading space)\n\
x feed steps 1500.123456789\n\
XDE1.8\n\
Xmicrosteps 8\n\
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

/* generate some strings for the parser and test NVM read and write */

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
			cfg_parse(testblock, TRUE, TRUE);// parse line 
			j = 0;			
		} else if (c == 0) {				// handle the last line
			testblock[j] = 0;
			cfg_parse(testblock, TRUE, TRUE);
			break;			
		} else {
			testblock[j++] = c;				// put characters into line
		}
	}
}

#endif
