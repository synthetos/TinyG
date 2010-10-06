/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

/* TODO:
	- help screen
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "xio.h"
#include "gcode.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
#include "controller.h"
#include "canonical_machine.h"
#include "xmega_eeprom.h"

// prototypes for local helper functions
void _cfg_computed(void); 
void _cfg_normalize_config_block(char *block);
char *_cfg_format_config_record(char *block);
uint8_t _cfg_tokenize_config_record();

void _cfg_load_hardwired_settings(void);

void _cfg_write_profile_to_NVM(uint16_t base_addr);
void _cfg_sprintf_as_record(char *record, uint8_t param, char axis, double value);

void _cfg_write_as_record_to_NVM(uint16_t base_addr, uint8_t param, uint8_t axis, double value);

void _cfg_write_record_to_NVM(uint16_t base_addr, char *record, uint8_t param, uint8_t axis);
void _cfg_read_record_from_NVM(uint16_t base_addr, char *record, uint8_t param, uint8_t axis);
void _cfg_write_to_NVM(uint16_t address, char *record);
void _cfg_read_from_NVM(uint16_t address, char *record, uint8_t size);
uint16_t _cfg_compute_record_address(uint16_t address, uint8_t param, uint8_t axis);

void _cfg_print_config_record(char *record);
void _cfg_print_axis(uint8_t axis);

/* 
 * Config parameter tokens and config record constants
 *
 * These values are used to tokenize config strings and to compute the NVM 
 * record addresses (_cfg_write_record())
 */

enum cfgTokens {
	CFG_HEADER_TOKEN,			// location zero reserved for header record

	// Gcode default settings
	CFG_GCODE_PLANE,			// default gcCanonicalPlane enum (0-2)
	CFG_GCODE_UNITS,			// default 0=inches (G20), 1=mm (G21)
	CFG_GCODE_HOMING_MODE,		// TRUE = do a homing cycle on startup
	CFG_GCODE_FEED_RATE,		// default F value
	CFG_GCODE_SPINDLE_SPEED,	// default S value
	CFG_GCODE_TOOL,				// default T value

	// machine default settings
	CFG_MM_PER_ARC_SEGMENT,		// the only global machine setting for now

	// per-axis settings 
	CFG_MAP_AXIS_TO_MOTOR,		// the map must be the first axis setting.
	CFG_SEEK_STEPS_MAX,			// the rest are ordered by convention and 
	CFG_FEED_STEPS_MAX,			//...the order will be visible to the user 
	CFG_DEGREES_PER_STEP,		//...so try not to change it too much
	CFG_MICROSTEP_MODE,
	CFG_POLARITY,
	CFG_TRAVEL_MAX,
	CFG_TRAVEL_WARN,			// warn the user if travel exceeds this value
	CFG_TRAVEL_PER_REV,			// in mm per revolution
	CFG_IDLE_MODE,
	CFG_LIMIT_SWITCH_MODE,

	CFG_TRAILER_TOKEN			// must always be last token enum
};

#define CFG_NVM_BASE 0x0000	// base address of usable NVM
#define CFG_RECORD_LEN 12		// length of ASCII NVM strings (see note)

#define CFG_NON_AXIS_BASE CFG_GCODE_PLANE		// start of non-axis parms
#define CFG_AXIS_BASE CFG_MAP_AXIS_TO_MOTOR 	// start of axis parameters
#define CFG_AXIS_COUNT (CFG_TRAILER_TOKEN - CFG_AXIS_BASE) // # of axis parms

#define CFG_HEADER_RECORD_ADDR CFG_NVM_BASE
#define CFG_TRAILER_RECORD (CFG_AXIS_BASE + (4*CFG_AXIS_COUNT))
#define CFG_TRAILER_RECORD_ADDR (CFG_TRAILER_RECORD * CFG_RECORD_LEN)

/* Note: A CFG_RECORD_LEN of 12 will accommodate numbers up to 8 digits
	- seven if it has a decimal point, 6 if it also has a minus sign. 
	Numbers with more digits will be truncated from the right. 
	This should suffice for any reasonable setting, but if not the 
	record length must be increased.
 */

struct cfgConfigParser {
	uint8_t status;				// parser status
	uint8_t param;				// tokenized parameter number
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

	// load hardware.h hardwired settings into cfg struct
	_cfg_load_hardwired_settings();

	// see if NVM is initialized and take approriate action
	_cfg_read_from_NVM(cp.base_addr, cp.record, CFG_RECORD_LEN);

	// if the header is not initialzed, set it up and exit
	if (cp.record[0] != '%') {
		_cfg_write_profile_to_NVM(cp.base_addr);
		return;
	}

	// if the header is initialzed but the wrong revision...
//	if (cp.record[0] != '%') {
//		// initialize NVM for config records
//		_cfg_write_to_NVM(CFG_HEADER_RECORD_ADDR, CFG_HEADER, TRUE);
//		_cfg_write_to_NVM(CFG_TRAILER_RECORD_ADDR, CFG_TRAILER, TRUE);
//		_cfg_write_profile_to_NVM(cp.base_addr);
//		return;
//	}

	// if the header is initialized, read the NVM configs into the struct
	for (uint16_t i = 0; i < CFG_TRAILER_RECORD; i++) {
		_cfg_read_from_NVM(address, cp.record, CFG_RECORD_LEN);
		cfg_parse(cp.record);
		address += CFG_RECORD_LEN;
	}
	return;
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

int cfg_parse(char *block)
{
	cp.status = TG_OK;

	// cutout for header and trailer blocks. Don't parse them
	if (block[0] == '%') {
		return (TG_OK);
	}

	// normalize the block in place
	_cfg_normalize_config_block(block);

	// dispatch on special characters in the first byte location
	switch (block[0]) {
		case  0:  return(TG_OK);			// ignore comments (stripped)
		case 'Q': return(TG_QUIT);			// quit config mode
		case 'H': cfg_print_help_screen(); return(TG_OK);
		case '?': cfg_print_config_records(); return(TG_OK);
	}

	// create a well-formed config record from the normalized block
	_cfg_format_config_record(block);		
	
	// parse the config record into a parser structure (or die trying)
	if (_cfg_tokenize_config_record()) {
		tg_print_status(cp.status, block);
	}

	// load value into cfg struct based on parameter type
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

	// generate & (re)populate computed config values
	_cfg_computed();

	// save config record in NVM
	_cfg_write_record_to_NVM(cp.base_addr, cp.record, cp.param, cp.axis);

	// do config displays
	_cfg_read_record_from_NVM (cp.base_addr, cp.record, cp.param, cp.axis);
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

	cm_set_traverse_rate(cfg.max_seek_rate); 
//	cm_set_feed_rate(CFG(cp.axis).seek_steps_sec); 

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

char *_cfg_format_config_record(char *block)
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

		// axis and mapped axis settings by axxess letter and by motor number
		case 'X': case '1': { cp.axis = 0; break;}
		case 'Y': case '2': { cp.axis = 1; break;}
		case 'Z': case '3': { cp.axis = 2; break;}
		case 'A': case '4': { cp.axis = 3; break;}

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
 * cfg_print_config_records() - dump configs from NVM to stderr
 *
 * Also display computed values at end
 */

void cfg_print_config_records()
{
	uint16_t address = cp.base_addr;

	tg_alive();		// header record

	for (uint16_t i = 0; i < CFG_TRAILER_RECORD; i++) {

		_cfg_read_from_NVM(address, cp.record, CFG_RECORD_LEN);
		_cfg_print_config_record(cp.record);
		address += CFG_RECORD_LEN;
	}
	printf_P(PSTR(" (maximum_seek_rate:  %7.3f mm / minute)\n"), cfg.max_seek_rate);
	printf_P(PSTR(" (maximum_feed_rate:  %7.3f mm / minute)\n"), cfg.max_feed_rate);
}

/*
 * _cfg_print_config_record()
 *
 *  Takes a config record as input - record must obey record formatting
 *	Uses global cp struct to tokenize and extract values from record
 */

// put record format strings in program memory
char cfgShowRecord00[] PROGMEM = "HEADER%s%d";
char cfgShowRecord01[] PROGMEM = "  Gcode: {G17/G18/G19}    Plane   %1.0f";
char cfgShowRecord02[] PROGMEM = "  Gcode: {G20/G21} Units (1=mm)   %1.0f";
char cfgShowRecord03[] PROGMEM = "  Gcode: {G28}  Power-on homing   %1.0f";
char cfgShowRecord04[] PROGMEM = "  Gcode: {F} Feed rate        %8.2f";
char cfgShowRecord05[] PROGMEM = "  Gcode: {S} Spindle speed    %8.2f";
char cfgShowRecord06[] PROGMEM = "  Gcode: {T} Tool                 %1.0f";
char cfgShowRecord07[] PROGMEM = "  MM(illimeters) / arc segment   %6.3f";
char cfgShowRecord08[] PROGMEM = "%c axis mapped to motor number  %4.0f";
char cfgShowRecord09[] PROGMEM = "  %c axis - Seek steps / sec   %5.0f";
char cfgShowRecord10[] PROGMEM = "  %c axis - Feed steps / sec   %5.0f";
char cfgShowRecord11[] PROGMEM = "  %c axis - Degrees per step   %5.3f";
char cfgShowRecord12[] PROGMEM = "  %c axis - Microstep mode     %5.0f";
char cfgShowRecord13[] PROGMEM = "  %c axis - Polarity           %5.0f";
char cfgShowRecord14[] PROGMEM = "  %c axis - Travel max         %5.0f";
char cfgShowRecord15[] PROGMEM = "  %c axis - Travel warning     %5.0f";
char cfgShowRecord16[] PROGMEM = "  %c axis - mm per revolution  %5.2f";
char cfgShowRecord17[] PROGMEM = "  %c axis - Idle mode          %5.0f";
char cfgShowRecord18[] PROGMEM = "  %c axis - Limit switches     %5.0f";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P rShowStrings[] PROGMEM = {	
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
	if (_cfg_tokenize_config_record()) {	// populates cp.struct used below
		tg_print_status(cp.status, record);
		return;
	}

	// otherwise print it
	if (cp.param < CFG_AXIS_BASE) {
		printf_P((PGM_P)pgm_read_word(&rShowStrings[cp.param]), cp.value);
	} else {
		char axis = GETAXISCHAR(cp.axis);
		printf_P((PGM_P)pgm_read_word(&rShowStrings[cp.param]), axis, cp.value);
	}
	printf_P(PSTR("     %-12s\n"), record);	// Must use 12 instead of CFG_RECORD_LEN

}

/*
 * _cfg_sprintf_as_record() - make a formatted config record from parameters
 *
 *	record			string to write to
 *	param			parameter enumneration
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
char cfgFmtRecord00[] PROGMEM = "HEADER%c%f";
char cfgFmtRecord01[] PROGMEM = "G%1.0f";		// Plane G17/G18/G19
char cfgFmtRecord02[] PROGMEM = "G%1.0f";		// Units G20/G21
char cfgFmtRecord03[] PROGMEM = "G%1.0f";		// G28  Power-on homing
char cfgFmtRecord04[] PROGMEM = "F%1.3f";		// F Feed rate
char cfgFmtRecord05[] PROGMEM = "S%1.2f";		// S Spindle speed
char cfgFmtRecord06[] PROGMEM = "T%1.0f";		// T Tool
char cfgFmtRecord07[] PROGMEM = "MM%1.3f";		// MM per arc segment
char cfgFmtRecord08[] PROGMEM = "%cMA%1.0f";	// Map axis to motor
char cfgFmtRecord09[] PROGMEM = "%cSE%1.0f";	// Seek steps per second
char cfgFmtRecord10[] PROGMEM = "%cFE%1.0f";	// Feed steps / sec
char cfgFmtRecord11[] PROGMEM = "%cDE%1.3f";	// Degrees per step
char cfgFmtRecord12[] PROGMEM = "%cMI%1.0f";	// Microstep mode
char cfgFmtRecord13[] PROGMEM = "%cPO%1.0f";	// Polarity
char cfgFmtRecord14[] PROGMEM = "%cTR%1.0f";	// Travel max (mm)
char cfgFmtRecord15[] PROGMEM = "%cTW%1.0f";	// Travel Warning
char cfgFmtRecord16[] PROGMEM = "%cRE%1.3f";	// mm per REvolution
char cfgFmtRecord17[] PROGMEM = "%cID%1.0f";	// Idle mode
char cfgFmtRecord18[] PROGMEM = "%cLI%1.0f";	// Limit switches on

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P rFmtStrings[] PROGMEM = {	
	cfgFmtRecord00,
	cfgFmtRecord01,
	cfgFmtRecord02,
	cfgFmtRecord03,
	cfgFmtRecord04,
	cfgFmtRecord05,
	cfgFmtRecord06,
	cfgFmtRecord07,
	cfgFmtRecord08,
	cfgFmtRecord09,
	cfgFmtRecord10,
	cfgFmtRecord11,
	cfgFmtRecord12,
	cfgFmtRecord13,
	cfgFmtRecord14,
	cfgFmtRecord15,
	cfgFmtRecord16,
	cfgFmtRecord17,
	cfgFmtRecord18
};

void _cfg_sprintf_as_record(char *record, uint8_t param, char axis, double value)
{
	if (param < CFG_AXIS_BASE) {
		sprintf_P(record,(PGM_P)pgm_read_word(&rFmtStrings[param]), value);
	} else {
		char axisc = GETAXISCHAR(axis);
		sprintf_P(record,(PGM_P)pgm_read_word(&rFmtStrings[param]), axisc, value);
	}
}

/*
 * _cfg_write_profile_to_NVM()
 *
 * Write entire config structure to NVM
 * Also writes header and trailer records
 */

void _cfg_write_profile_to_NVM(uint16_t base_addr)
{
	// write header and trailer records
	_cfg_write_to_NVM((base_addr + CFG_HEADER_RECORD_ADDR), CFG_HEADER);
	_cfg_write_to_NVM((base_addr + CFG_TRAILER_RECORD_ADDR), CFG_TRAILER);

	// Gcode settings
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, (17 + cfg.gcode_plane));
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, (20 + cfg.gcode_units));
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_HOMING_MODE, 0, 28);
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_FEED_RATE, 0, cfg.max_feed_rate); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_SPINDLE_SPEED, 0, cfg.spindle_speed);
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_TOOL, 0, cfg.gcode_tool);

	// non-axis machine settings
	_cfg_write_as_record_to_NVM(base_addr, CFG_MM_PER_ARC_SEGMENT, 0, cfg.mm_per_arc_segment); 

	// per-axis settings
	for (uint8_t axis = 0; axis <= 3; axis++) {	// cycle thru axes X,Y,Z,A

		_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, axis,
											   CFG(axis).map_axis+1); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_SEEK_STEPS_MAX, axis,
											   CFG(axis).seek_steps_sec); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_FEED_STEPS_MAX, axis,
											   CFG(axis).feed_steps_sec); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_DEGREES_PER_STEP, axis,
											   CFG(axis).degree_per_step); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_MICROSTEP_MODE, axis,
											   CFG(axis).microstep); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_POLARITY, axis,
											   CFG(axis).polarity);
										 
		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_MAX, axis,
											   CFG(axis).mm_travel); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_WARN, axis,
											   CFG(axis).mm_travel); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_PER_REV, axis,
											   CFG(axis).mm_per_rev); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_IDLE_MODE, axis,
											   CFG(axis).low_pwr_idle); 

		_cfg_write_as_record_to_NVM(base_addr, CFG_LIMIT_SWITCH_MODE, axis,
											   CFG(axis).limit_enable); 
	}
}

/*
 * _cfg_write_as_record_to_NVM() - format a config record and write to NVM
 *
 *	base_addr		base address in NVM for this profile
 *	param			parameter enumneration
 *	value			value loaded as a double
 *	axis			axis (numeric)
 */

void _cfg_write_as_record_to_NVM(uint16_t base_addr, uint8_t param, uint8_t axis, double value)
{
	uint16_t address;
	char record[CFG_RECORD_LEN];

	_cfg_sprintf_as_record(record, param, axis, value);
	address = _cfg_compute_record_address(base_addr, param, axis);
	_cfg_write_to_NVM(address, record);
}

/* 
 * _cfg_write_record_to_NVM()  - write NVM record given param & axis values
 * _cfg_read_record_from_NVM() - read NVM record given param & axis values
 * _cfg_write_to_NVM() 		   - raw write to NVM w/no address calculation
 * _cfg_read_from_NVM() 	   - raw read from NVM w/no address calculation
 *
 *	The base address of the record-set is provided as an argument to 
 *	support writing and reading multiple machine profiles.
 */

void _cfg_write_record_to_NVM(uint16_t base_addr, char *record, uint8_t param, uint8_t axis)
{
	uint16_t address = _cfg_compute_record_address(base_addr, param, axis);
	_cfg_write_to_NVM(address, record);
}

void _cfg_read_record_from_NVM(uint16_t base_addr, char *record, uint8_t param, uint8_t axis)
{
	uint16_t address = _cfg_compute_record_address(base_addr, param, axis);
	_cfg_read_from_NVM(address, record, CFG_RECORD_LEN);
  	return;
}

inline void _cfg_write_to_NVM(uint16_t address, char *record)
{
	EEPROM_WriteString(address, record, TRUE);
}

inline void _cfg_read_from_NVM(uint16_t address, char *record, uint8_t size)
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
 *	- trailer record is a known value
 */

inline uint16_t _cfg_compute_record_address(uint16_t base_addr, uint8_t param, uint8_t axis)
{
	if (param == CFG_TRAILER_RECORD) {
		return (base_addr + CFG_TRAILER_RECORD_ADDR);
	} else if (param < CFG_AXIS_BASE) {
		return (base_addr += (param * CFG_RECORD_LEN));
	} else {
		return (base_addr += (CFG_AXIS_BASE + (axis * CFG_AXIS_COUNT) + param - CFG_AXIS_BASE) * CFG_RECORD_LEN);
	}
}

/*
 * cfg_print_profile() - dump configs from internal structure to stderr
 */
/*
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

void cfg_print_profile()
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
*/
/* 
 * _cfg_load_hardwired_settings() - load compiled hardwired settings into strict
 */

void _cfg_load_hardwired_settings()
{
	cfg.gcode_plane = CANON_PLANE_XY;
	cfg.gcode_units = GCODE_UNITS;
	cfg.homing_mode = HOMING_MODE;
	cfg.spindle_speed = SPINDLE_SPEED;
	cfg.gcode_tool = GCODE_TOOL;

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

/*
 * cfg_print_help_screen() - send config help screen to stderr
 */

void cfg_print_help_screen()
{
	printf_P(PSTR("Configuration Help\n"));
	return;
}



#ifdef __UNIT_TESTS

void _cfg_test_parse(void);
void _cfg_test_write_record(void);

void cfg_tests()
{
//	_cfg_test_parse();
//	_cfg_test_write_record();
}

void _cfg_test_write_record()
{
	uint16_t base_addr = 0;

	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 17); // G17
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 18); // G18
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_PLANE, 0, 19); // G19
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, 20); // G20
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_UNITS, 0, 21); // G21
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_HOMING_MODE, 0, 28); // G28 startup homing
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_FEED_RATE, 0, 400.50); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_SPINDLE_SPEED, 0, 12000);
	_cfg_write_as_record_to_NVM(base_addr, CFG_GCODE_TOOL, 0, 1);

	_cfg_write_as_record_to_NVM(base_addr, CFG_MM_PER_ARC_SEGMENT, 0, MM_PER_ARC_SEGMENT); 

	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, X, 1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, Y, 2); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, Z, 3); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MAP_AXIS_TO_MOTOR, A, 4); 

	_cfg_write_as_record_to_NVM(base_addr, CFG_SEEK_STEPS_MAX, X, 1500); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_FEED_STEPS_MAX, X, 1200); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_DEGREES_PER_STEP, X, 1.8); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_MICROSTEP_MODE, X, -1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_POLARITY, X, 0); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_MAX, X, 400); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_WARN, X, 425); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_TRAVEL_PER_REV, X, 1.27); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_IDLE_MODE, X, 1); 
	_cfg_write_as_record_to_NVM(base_addr, CFG_LIMIT_SWITCH_MODE, X, 0); 

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
