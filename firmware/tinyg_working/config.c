/*
 * config.c - eeprom and compile time configuration handling 
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>					// for memset(), strchr()
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
#include "xmega_nvm.h"

// prototypes for local helper functions
void _cfg_computed(void); 
void _cfg_normalize_config_block(char *block);
uint8_t _cfg_parse_config_block(char *block);
void _cfg_dump_axis(uint8_t	axis);

// local data
struct cfgConfigParser {
	int8_t axis;				// axis value or -1 if none
	uint8_t param;				// tokenized parameteer
	double value;				// value parsed
	char block[40];				// 
};
static struct cfgConfigParser cp;

// Config parameter tokens

enum cfgTokens {
	CPTOK_NONE,
	CPTOK_MM_PER_ARC_SEGMENT,
	CPTOK_SEEK_STEPS_MAX,
	CPTOK_FEED_STEPS_MAX,
	CPTOK_DEGREES_PER_STEP,
	CPTOK_MICROSTEP_MODE,
	CPTOK_POLARITY,
	CPTOK_TRAVEL_MAX,
	CPTOK_TRAVEL_WARN,				// stop homing cycle if above this value
	CPTOK_TRAVEL_PER_REVOLUTION,
	CPTOK_IDLE_MODE,
	CPTOK_LIMIT_SWITCH_MODE,
	CPTOK_MAP_AXIS_TO_MOTOR,
	CPTOK_GCODE_UNITS,
	CPTOK_GCODE_PLANE
};

/* 
 * cfg_init() - initialize config system 
 */

void cfg_init() 
{
	cfg_reset();
}

/* 
 * cg_config_parser() - parse a config line
 *					  - write into config record and persist to EEPROM
 *
 * How it works:
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
 *	General config parameters
 * 
 *		  AR [0.00-1.00]	Millimeters per arc segment 
 *							Current driver resolution is between 0.05 and 0.01 mm
 *
 *	Per-axis parameters
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
 *							1 = limit switches enabled (may need more modes than this)
 *
 *		X MA [0-4]			Map axis to motor #
 *							0 = axis disabled
 *							This setting will also be used to support axis slaving
 *
 *	Computed parameters
 *
 *	There are also a set of parameters that are computed from the above and 
 *	are displayed for convenience
 *
 *		steps_per_mm
 *		steps_per_inch
 *		maximum_seek_rate in mm/minute and inches/minute
 *		maximum_feed_rate in mm/minute and inches/minute
 *
 *	G code configuration
 *
 *	Config accepts the following G codes which become the power-on defaults 
 *
 *		G20/G21			Select inches (G20) or millimeters mode (G21)
 *		G17/G18/G19		Plane selection
 *
 *	Examples of valid config lines:
 *
 *		X SE 1800			Set X maximum seek to 1800 whole steps / second 
 *		XSE1800				Same as above
 *		xseek1800			Same as above
 *		xseek+1800			Same as above
 *		xseek 1800.00		Same as above
 *		xseek 1800.99		OK, but will be truncated to 1800 (integer value)
 *		X FE [1800]			OK, but the [] brackets are superfluous
 *		ZID1 (set low power idle mode on Z axis and show how comments work)
 *		zmicrsteps 4 (sets Z microsteps to 1/4, misspelling is intentional)
 *		G20					Set Gcode to default to inches mode 
 *
 *	Examples of invalid config lines:
 *
 *		SE 1800				No axis specified
 *		SE 1800 X			Axis specifier must be first
 *		SEX 1800			No SEX allowed (axis specifier must be first)
 *		X FE -100			Can't have a negative feed step rate
 *		X FE 100000			Find a motor this fast and I'll bump the data size
 *		C LI 1				C axis not currently supported (nor is B)
 */

int cfg_parse2(char *block)
{
//	char *val = 0;				// pointer to normalized value 
//	char *end = 0;				// pointer to end of value
//	uint8_t	axis = 0;			// axis index

	cfg.status = TG_OK;
	_cfg_normalize_config_block(block);		// normalize text in place
	if (block[0] == 0) { 					// ignore comments (stripped)
		return(TG_OK);
	}
	if (block[0] == 'Q') {					// quit config mode
		return(TG_QUIT);
	}
	if (block[0] == '?') {					// dump state
		cfg_dump();
		return(TG_OK);
	}
	_cfg_parse_config_block(block);

	CPTOK_MM_PER_ARC_SEGMENT,
	CPTOK_MICROSTEP_MODE,
	,
	CPTOK_TRAVEL_MAX,
	CPTOK_TRAVEL_WARN,				// stop homing cycle if above this value
	CPTOK_TRAVEL_PER_REVOLUTION,
	CPTOK_IDLE_MODE,
	CPTOK_LIMIT_SWITCH_MODE,
	CPTOK_MAP_AXIS_TO_MOTOR,
	CPTOK_GCODE_UNITS,
	CPTOK_GCODE_PLANE

	// dispatch based on parameter type
	switch (cp.param) {
		case CPTOK_SEEK_STEPS_MAX: CFG(cp.axis).seek_steps_sec = (uint16_t)cp.value; break;
		case CPTOK_FEED_STEPS_MAX: CFG(cp.axis).feed_steps_sec = (uint16_t)cp.value; break;
		case CPTOK_DEGREES_PER_STEP: CFG(cp.axis).degree_per_step = cp.value; break;
		case CPTOK_POLARITY: CFG(cp.axis).degree_per_step = (uint16_t)cp.value; break;

			case 'P': CFG(axis).polarity = (uint8_t)atoi(val);
					  st_set_polarity(axis, CFG(axis).polarity);
					  break;

			case 'M': 
				if (block[2] == 'I') {
					CFG(axis).microstep = (uint8_t)atoi(val); break;
				} else if (block[3] == 'R') {
					CFG(axis).mm_per_rev = strtod(val, &end); break;
				} else if (block[3] == 'T') {
					CFG(axis).mm_travel = strtod(val, &end); break;
				}
			case 'L': 
				if (block[2] == 'O') {
					CFG(axis).low_pwr_idle = (uint8_t)atoi(val); break;
				} else if (block[2] == 'I') {
					CFG(axis).limit_enable = (uint8_t)atoi(val); break;
				}

			default: cfg.status = TG_UNRECOGNIZED_COMMAND;	// error return
		}
	}
//	cfg_write();
	return (cfg.status);
}

/*
 * _cfg_normalize_config_block() - normalize a config block (text line) in place
 *
 *	Comments always terminate the block (embedded comments are not supported)
 *	Processing: split string into command and comment portions. Valid choices:
 *	  supported:	command
 *	  supported:	comment
 *	  supported:	command comment
 *	  unsupported:	command command
 *	  unsupported:	comment command
 *	  unsupported:	command comment command
 *
 *	Valid characters (these are passed to config parser):
 *		digits						all digits are passed to parser
 *		lower case alpha			all alpha is passed - converted to upper
 *		upper case alpha			all alpha is passed
 *		- . 						sign and decimal chars passed to parser
 *
 *	Invalid characters (these are stripped but don't cause failure):
 *		control characters			chars < 0x20 are all removed
 *		/ *	< = > | % #	+			expression chars removed from string
 *		( ) [ ] { } 				expression chars removed from string
 *		<sp> <tab> 					whitespace chars removed from string
 *		! $ % ,	; ; ? @ 			removed
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
		if (strchr("-.", c)) {				// catch valid non-alphanumerics
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
 * _cfg_parse_config_block() - parse block into struct
 */

uint8_t _cfg_parse_config_block(char *block)
{
	uint8_t i=0; 				// char index into block
	char *end;					// pointer to end of value

	switch(block[i++]) {
		case 'X': { cp.axis = X; break;}
		case 'Y': { cp.axis = Y; break;}
		case 'Z': { cp.axis = Z; break;}
		case 'A': { cp.axis = A; break;}
		default:  { cp.axis = -1; i--; }
	}
	switch(block[i++]) {
		case 'A': { cp.param = CPTOK_MM_PER_ARC_SEGMENT; break; }
		case 'S': { cp.param = CPTOK_SEEK_STEPS_MAX; break; }
		case 'F': { cp.param = CPTOK_FEED_STEPS_MAX; break; }
		case 'D': { cp.param = CPTOK_DEGREES_PER_STEP; break; }
		case 'P': { cp.param = CPTOK_POLARITY; break; }
		case 'T': { cp.param = CPTOK_TRAVEL_MAX; break; }
		case 'R': { cp.param = CPTOK_TRAVEL_PER_REVOLUTION; break; }
		case 'I': { cp.param = CPTOK_IDLE_MODE; break; }
		case 'L': { cp.param = CPTOK_LIMIT_SWITCH_MODE; break; }
		case 'M': 
			switch (block[i]) {
				case 'I': { cp.param = CPTOK_MICROSTEP_MODE; break; }
				case 'A': { cp.param = CPTOK_MAP_AXIS_TO_MOTOR; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			}
//		case 'G': 
//			switch (block[i]) {
//				case 'I': { cp.param = CPTOK_GCODE_UNITS; break; }
//				case 'A': { cp.param = CPTOK_GCODE_PLANE; break; }
//				default: return(TG_UNRECOGNIZED_COMMAND);
//			}
		default: return(TG_UNRECOGNIZED_COMMAND);
	}
	while (isupper(block[++i])) {	// advance past any remaining chars
	}
	cp.value = strtod(&block[i], &end);
	return (TG_OK);
}

/* 
 * cfg_parse() - parse a config string into the config record
 *
 * YACLHCP - yet-another-crappy-little-hard-coded-parser for reading config values
 *	Config string may consist of one or more tag=value pairs 
 *
 * Supported tags (axes X,Y,Z,A are supported - only X is shown)
 *	mm_arc_segment		0.1		arc drawing resolution in millimeters per segment 
 *	x_seek_steps_sec	1800	max seek whole steps per second for X axis
 *	x_feed_steps_sec	1200	max feed whole steps per second for X axis
 *	x_degree_step		1.8		degrees per whole step for X axis
 *	x_mm_rev			2.54	millimeters of travel per revolution of X axis
 *	x_mm_travel			406		millimeters of travel in X dimension (total envelope)
 * 	x_microstep			8		microsteps to apply for X axis steps
 *	x_polarity_invert	0		0=normal drive polarity, 1=inverted polarity
 *	x_low_pwr_idle		1		1=low power idle mode, 0=full power idle mode 
 *	x_limit_enable		1		1=max limit switch enabled, 0=not enabled
 *
 * Parsing rules:
 *	Tags are case insensitive 
 *	Punctuation and whitespace are ignored 
 *	Tags and values are separated by '=' sign
 *	Whitespace is optional and ignored
 *	Values are read as floating point numbers and cast to proper internal types
 *	Integers received as fractional numbers are truncated (this shouldn't happen)
 *	Comments are in parentheses and cause the remainder of the line to be ignored 
 *	Tags are only parsed to the point of uniqueness; the following are equivalent:
 *		z_seek_steps_sec
 *		z-seek-steps-sec
 *		z_seek_steps_*&*&^@#*&^_sec
 *		zSeekStepsSec
 *		zs
 *		ZS
 *
 *	Examples:
 *		mm=0.01					(set mm per arc segment to 0.01)
 *		xd = 0.9				(set x axis motor ot 0.9 degrees per step
 *		y_low_power_idle = 1	(enable low power idle on Y axis)
 *		y_lo_poweridle = 1		(enable low power idle on Y axis)
 *		ylo=1					(enable low power idle on Y axis)
 *		zlim=0					(disable Z axis limit switch)
 *		y_mm_revolution = 1.27	(mm per revolution of the Y axis)
 *		ymmr=1.27				(mm per revolution of the Y axis)
 */

int cfg_parse(char *text)
{
	char c;
	char *val = 0;				// pointer to normalized value 
	char *end = 0;				// pointer to end of value
	uint8_t i = 0;				// buffer read index (buf must be < 256 chars)
	uint8_t j = 0;				// buffer write index
	uint8_t	axis = 0;			// axis index

	// normalize and split text in place
	while ((c = toupper(text[i++])) != 0) {
		if (c == '=') {							// handle separator
			text[j++] = 0;						// terminate tag at separator
			val = &text[j];						// new string starts at value
		} else if ( (c == '-') || 				// capture special characters
					(c == '+') || 
					(c == '.') || 
					(c == '?') || 
					(c == '(')) { 
			text[j++] = c;
		} else if (c >= 'A' && c <= 'Z') {		// capture letters
			text[j++] = c;
		} else if (c >= '0' && c <= '9') {		// capture numbers
			text[j++] = c;
		}
	}
	text[j++] = 0;								// nul terminate line
	end = &text[j];								// needed for string-to-double

	// pick off tag characters starting with first character
	cfg.status = TG_OK;
	switch (text[0]) {
		case '?': cfg_dump(); cfg.status = TG_OK; break;
		case '(': cfg.status = TG_OK; break;		// ignore comment lines
		case 'Q': cfg.status = TG_QUIT; break;
		case 'M': cfg.mm_per_arc_segment = strtod(val, &end); 
				  cfg.status = TG_OK; 
				  break;

		case 'X': axis = X; break;
		case 'Y': axis = Y; break;
		case 'Z': axis = Z; break;
		case 'A': axis = A; break;

		default: cfg.status = TG_UNRECOGNIZED_COMMAND; 	// error return
	}
	if (cfg.status == TG_OK) {
		cfg.status = TG_OK;							// pre-emptive setting
		switch (text[1]) {
			case 'S': CFG(axis).seek_steps_sec = (uint16_t)atoi(val); break;
			case 'F': CFG(axis).feed_steps_sec = (uint16_t)atoi(val); break;
			case 'D': CFG(axis).degree_per_step = strtod(val, &end); break;
			case 'P': CFG(axis).polarity = (uint8_t)atoi(val);
					  st_set_polarity(axis, CFG(axis).polarity);
					  break;

			case 'M': 
				if (text[2] == 'I') {
					CFG(axis).microstep = (uint8_t)atoi(val); break;
				} else if (text[3] == 'R') {
					CFG(axis).mm_per_rev = strtod(val, &end); break;
				} else if (text[3] == 'T') {
					CFG(axis).mm_travel = strtod(val, &end); break;
				}
			case 'L': 
				if (text[2] == 'O') {
					CFG(axis).low_pwr_idle = (uint8_t)atoi(val); break;
				} else if (text[2] == 'I') {
					CFG(axis).limit_enable = (uint8_t)atoi(val); break;
				}

			default: cfg.status = TG_UNRECOGNIZED_COMMAND;	// error return
		}
	}
//	cfg_write();
	return (cfg.status);
}

/*
 * cfg_dump() - dump configs to stdout
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

void cfg_dump()
{
	printf_P(PSTR("\n***** CONFIGURATION [version %d] ****\n"), cfg.config_version);
	printf_P(PSTR("G-code Model Configuration Values ---\n"));
	printf_P(PSTR("  mm_per_arc_segment:   %5.3f mm / segment\n"), cfg.mm_per_arc_segment);
	printf_P(PSTR(" (maximum_seek_rate:  %7.3f mm / minute)\n"), cfg.max_seek_rate);
	printf_P(PSTR(" (maximum_feed_rate:  %7.3f mm / minute)\n\n"), cfg.max_feed_rate);

	for (uint8_t axis=X; axis<=A; axis++) {
		_cfg_dump_axis(axis);
	}
}

void _cfg_dump_axis(uint8_t	axis)
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
 * config_reset() - load default settings into config 
 */

void cfg_reset()
{
	cfg.config_version = EEPROM_DATA_VERSION;
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
 * _cfg_computed() - helper function to generate computed config values 
 *	call this every time you change any configs
 */

void _cfg_computed() 
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
 * cfg_read() - read config data from EEPROM into the config struct 
 */

int cfg_read()
{
/*
	uint8_t version = eeprom_get_char(0);	// Check version-byte of eeprom

	if (version != EEPROM_DATA_VERSION) {	// Read config-record and check checksum
		return(FALSE); 
	} 
  	if (!(memcpy_from_eeprom_with_checksum
		((char*)&cfg, 0, sizeof(struct cfgStructGlobal)))) {
    	return(FALSE);
  	}
*/
  	return(TRUE);
}

/* 
 * cfg_write() - write config struct to program ROM 
 */

void cfg_write()
{
//	eeprom_put_char(0, CONFIG_VERSION);
//	memcpy_to_eeprom_with_checksum(0, (char*)&cfg, sizeof(struct cfgStructGlobal));
}

/* 
 * cfg_test() - generate some strings for the parser and test EEPROM read and write 
 */

char configs_P[] PROGMEM = "\
mm_per_arc_segment = 0.2 \n\
x_seek_steps_sec = 1000 \n\
y_seek_steps_sec = 1100 \n\
z_seek_steps_sec = 1200 \n\
a_seek_steps_sec = 1300 \n\
x_feed_steps_sec = 600 \n\
y_feed_steps_sec = 700 \n\
z_feed_steps_sec = 800 \n\
a_feed_steps_sec = 900 \n\
x_degree_step = 0.9	\n\
x_mm_rev = 5.0 \n\
x_mm_travel	= 410 \n\
z_microstep	= 2	 \n\
x_low_pwr_idle = 0 \n\
x_limit_enable=	0";

void cfg_test()
{
	char text[40];
	int i = 0;					// ROM buffer index (int allows for > 256 chars)
	int j = 0;					// RAM buffer index (text)
	char c;

	// feed the parser one line at a time
	while (TRUE) {
		c = pgm_read_byte(&configs_P[i++]);
		if (c == 0) {									// last line
			text[j] = 0;
			cfg_parse(text);
			break;			
		} else if ((c == '\r') || (c == '\n') || (c == ';')) {	// line complete
			text[j] = 0;							// terminate the string
			cfg_parse(text);						// parse line 
			j = 0;			
		} else if (c <= ' ') { 							// toss whitespace & ctrls
		} else {
			text[j++] = c;							// put characters into line
		}
	}
}

/* 
 * cfg_test2() - generate some strings for the parser and test EEPROM read and write 
 */

char configs2_P[] PROGMEM = "\
xseek1892\n";

void cfg_test2()
{
//	char text[40];
	int i = 0;					// ROM buffer index (int allows for > 256 chars)
	int j = 0;					// RAM buffer index (text)
	char c;

	// feed the parser one line at a time
	while (TRUE) {
		c = pgm_read_byte(&configs2_P[i++]);
		if (c == 0) {								// last line
			cp.block[j] = 0;
			cfg_parse2(cp.block);
			break;			
		} else if (c == '\n') {						// line complete
			cp.block[j] = 0;							// terminate the string
			cfg_parse2(cp.block);						// parse line 
			j = 0;			
		} else {
			cp.block[j++] = c;							// put characters into line
		}
	}
}

