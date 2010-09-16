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

#include <stdio.h>
#include <ctype.h>
#include <string.h>					// for memset(), strchr()
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
#include "xmega_eeprom.h"

// prototypes for local helper functions
void _cfg_normalize_config_block(char *block);
char *_cfg_create_config_record(char *block);
uint8_t _cfg_parse_config_record();
void _cfg_display_config_record(char *record);

void _cfg_computed(void); 
uint16_t _cfg_compute_record_address(uint16_t address);
void _cfg_write_config_record_to_eeprom(const uint16_t address);
char *_cfg_read_config_record_from_eeprom(const uint16_t address);

void _cfg_dump_axis(uint8_t	axis);

// Config parameter tokens and config record constants
// These values are used to tokenize config strings and 
// to compute the EEPROM record addresses (_cfg_write_record())

enum cfgTokens {
	CFG_RECORD_HEADER,			// header record must always be zero
	
	// Gcode default settings
	CFG_GCODE_INCH_MODE,		// TRUE = inches, FALSE = mm 
	CFG_GCODE_POWERUP_HOME,		// TRUE = homing cycle on startup
	CFG_GCODE_PLANE,			// use gcCanonicalPlane enum (0-2)
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
	CFG_RECORD_LAST				// last record
};
#define CFG_EEPROM_BASE 0		// base address of usable EEPROM
#define CFG_RECORD_LEN 12		// length of ASCII EEPROM strings (see note)
#define CFG_AXIS_BASE CFG_MAP_AXIS_TO_MOTOR // start of axis records
#define CFG_AXIS_COUNT (CFG_RECORD_LAST - CFG_AXIS_BASE)

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
//	uint16_t address;			// debug value
	uint16_t profile_base;		// base address of current profile
	char record[CFG_RECORD_LEN+1];// config record for EEPROM
	char block[40];				// TEMP
};
static struct cfgConfigParser cp;

/*
 * cfg_init() - initialize config system 
 */

void cfg_init() 
{
	cfg_reset();
	cp.profile_base = CFG_EEPROM_BASE;	// first (and only) profile
}

/* 
 * cfg_parse() - parse a config line
 *			   - write into config record and persist to EEPROM
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
 *		steps_per_mm
 *		steps_per_inch
 *		maximum_seek_rate in mm/minute and inches/minute
 *		maximum_feed_rate in mm/minute and inches/minute
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
 *		mm_per_arc_segment 0.01
 *		MM0.01
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

int cfg_parse(char *block)
{
	cp.status = TG_OK;

	// normalize the block in place
	_cfg_normalize_config_block(block);

	// dispatch on special characters in the first location
	switch (block[0]) {
		case  0:  return(TG_OK);			// ignore comments (stripped)
		case 'Q': return(TG_QUIT);			// quit config mode
		case 'H': cfg_help(); return(TG_OK);
		case '?': cfg_show(); return(TG_OK);
	}

	// create a well-formed config record from the normalized block
	_cfg_create_config_record(block);		
	
	// parse the config record into a parser structure
	_cfg_parse_config_record();

	// load value based on parameter type (cp.param)
	switch (cp.param) {
		case CFG_MM_PER_ARC_SEGMENT: cfg.mm_per_arc_segment = cp.value; break;

		case CFG_SEEK_STEPS_MAX: 	CFG(cp.axis).seek_steps_sec = (uint16_t)cp.value; break;
		case CFG_FEED_STEPS_MAX: 	CFG(cp.axis).feed_steps_sec = (uint16_t)cp.value; break;
		case CFG_DEGREES_PER_STEP:	CFG(cp.axis).degree_per_step = cp.value; break;
		case CFG_POLARITY:			CFG(cp.axis).polarity = (uint8_t)cp.value; 
					  		 		st_set_polarity(cp.axis, CFG(cp.axis).polarity);
							 		break;

		case CFG_MICROSTEP_MODE:	CFG(cp.axis).microstep = (uint8_t)cp.value; break;
		case CFG_IDLE_MODE: 		CFG(cp.axis).low_pwr_idle = (uint8_t)cp.value; break;
		case CFG_LIMIT_SWITCH_MODE: CFG(cp.axis).limit_enable = (uint8_t)cp.value; break;
		case CFG_TRAVEL_PER_REV: 	CFG(cp.axis).mm_per_rev = cp.value; break;
		case CFG_TRAVEL_MAX: 		CFG(cp.axis).mm_travel = cp.value; break;

		default: cp.status = TG_UNRECOGNIZED_COMMAND;	// error return
	}

	// save config record in EEPROM
	_cfg_write_config_record_to_eeprom(cp.profile_base);

	_cfg_read_config_record_from_eeprom(cp.profile_base);
	_cfg_display_config_record(cp.record);

	return (cp.status);
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
	cp.record[CFG_RECORD_LEN-1] = '\n';	// write ending newline
	cp.record[CFG_RECORD_LEN] = 0;		// terminate. (NULL is needed even
										// though its not written to EEPROM)
	// handle Gcode settings
	if (isdigit(block[1])) {
		return (block);				// sufficiently normalized. return as-is
	}
	// handle non-Gcode settings
	switch(block[0]) {
		// handle non-axis settings
		case 'M': { i=1; j=2; break; }	// set read and write pointers
		// handle axis settings
		default:  { i=2; j=3; break; }
	}
	while (isupper(block[++i])) {	// position to start of value by advancing
	}								// ... past any remaining tag alphas
	while (block[0] && j < CFG_RECORD_LEN-1) {	// copy value to EEPROM record
		cp.record[j++] = block[i++];
	}
	while (j < CFG_RECORD_LEN-1) {	// space fill remainder of record
		cp.record[j++] = ' ';		// ...but preserve the newline (the -1) 
	}
	return (cp.record);
}

/*
 * _cfg_parse_config_record() - parse record into struct
 *
 *	Block must be normalized w/comments removed
 */

uint8_t _cfg_parse_config_record()
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
				case 20: { cp.param = CFG_GCODE_INCH_MODE; cp.value = TRUE; break; }
				case 21: { cp.param = CFG_GCODE_INCH_MODE; cp.value = FALSE; break; }
				case 28: { cp.param = CFG_GCODE_POWERUP_HOME; cp.value = TRUE; break; }
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
		case 'T': { cp.param = CFG_TRAVEL_MAX; break; }
		case 'R': { cp.param = CFG_TRAVEL_PER_REV; break; }
		case 'I': { cp.param = CFG_IDLE_MODE; break; }
		case 'L': { cp.param = CFG_LIMIT_SWITCH_MODE; break; }
		case 'M': 
			switch (cp.record[2]) {
				case 'I': { cp.param = CFG_MICROSTEP_MODE; break; }
				case 'A': { cp.param = CFG_MAP_AXIS_TO_MOTOR; break; }
				default: return(TG_UNRECOGNIZED_COMMAND);
			}
		default: return(TG_UNRECOGNIZED_COMMAND);
	}
	return (TG_OK);
}

/*
 * _cfg_display_config_record()
 *
 *  Takes a config record as input - record must obey record formatting
 *	Uses global cp struct to tokenize and extract values from record
 */

// put record format strings in program memory
char cfgRecord00[] PROGMEM = "HEADER%s%d\n";
char cfgRecord01[] PROGMEM = "%c Gcode: {G20/G21}  Inches mode:  %1.0f\n";
char cfgRecord02[] PROGMEM = "%c Gcode: {G28}  Power-on homing:  %1.0f\n";
char cfgRecord03[] PROGMEM = "%c Gcode: {G17/G18/G19}    Plane:  %1.0f\n";
char cfgRecord04[] PROGMEM = "%c Gcode: {F} Feed rate:       %8.2f\n";
char cfgRecord05[] PROGMEM = "%c Gcode: {S} Spindle speed:   %8.2f\n";
char cfgRecord06[] PROGMEM = "%c Gcode: {T} Tool                 %1.0f\n";
char cfgRecord07[] PROGMEM = "%c MM(illimeters) / arc segment:  %6.3f\n";
char cfgRecord08[] PROGMEM = "  MAp %c axis to motor number  %5.0f\n";
char cfgRecord09[] PROGMEM = "  %c axis - SEek steps / sec:  %5.0f\n";
char cfgRecord10[] PROGMEM = "  %c axis - FEed steps / sec:  %5.0f\n";
char cfgRecord11[] PROGMEM = "  %c axis - DEgrees per step:  %5.0f\n";
char cfgRecord12[] PROGMEM = "  %c axis - MIcrostep mode:    %5.0f\n";
char cfgRecord13[] PROGMEM = "  %c axis - motor POlarity:    %5.0f\n";
char cfgRecord14[] PROGMEM = "  %c axis - TRavel max:        %5.0f\n";
char cfgRecord15[] PROGMEM = "  %c axis - Travel Warning:    %5.0f\n";
char cfgRecord16[] PROGMEM = "  %c axis - mm per REvolution  %5.0f\n";
char cfgRecord17[] PROGMEM = "  %c axis - IDle_mode          %5.0f\n";
char cfgRecord18[] PROGMEM = "  %c axis - LImit_sw_mode:     %5.0f\n";
char cfgRecord19[] PROGMEM = "  %c axis - feed_steps_sec:    %5.0f\n";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P cfgRecordStrings[] PROGMEM = {	
	cfgRecord00,
	cfgRecord01,
	cfgRecord02,
	cfgRecord03,
	cfgRecord04,
	cfgRecord05,
	cfgRecord06,
	cfgRecord07,
	cfgRecord08,
	cfgRecord09,
	cfgRecord10,
	cfgRecord11,
	cfgRecord12,
	cfgRecord13,
	cfgRecord14,
	cfgRecord15,
	cfgRecord16,
	cfgRecord17,
	cfgRecord18,
	cfgRecord19
};

void _cfg_display_config_record(char *record)
{
	_cfg_parse_config_record(record);		// tokenize the record

//	printf_P(PSTR("  seek_steps_sec:  %4f    steps / second (whole steps)\n"), cp.value);
//	printf_P(PSTR("%S: %d\n"),(PGM_P)pgm_read_word(&cfgRecordStrings[cp.param]), cp.value);
//	printf_P(PSTR("%c: %4.0f\n"), cp.axis_char, cp.value);

//	printf_P(PSTR("\n%s"), cp.record);
	printf_P((PGM_P)pgm_read_word(&cfgRecordStrings[cp.param]), cp.axis_char, cp.value);
}
/* 
 * cfg_reset() - load default settings into config 
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
 * cfg_show() - dump configs to stderr
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

void cfg_show()
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
 * cfg_help() - send config help screen to stderr
 */

void cfg_help()
{
	printf_P(PSTR("Configuration Help\n"));
	return;
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
/*
	if (cp.param < CFG_AXIS_BASE) {
		address = address + (cp.param * CFG_RECORD_LEN);
	} else {

		address = address + (CFG_AXIS_BASE 
						  + cp.axis * CFG_AXIS_COUNT 
						  + cp.param - CFG_AXIS_BASE) 
						  * CFG_RECORD_LEN;
	}
*/
	address = _cfg_compute_record_address(address);
	EEPROM_WriteString(address, cp.record, FALSE);
}

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


#ifdef __UNIT_TESTS

/* 
 * cfg_test() - generate some strings for the parser and test EEPROM read and write 
 */
/*
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
*/

char configs_P[] PROGMEM = "\
he1234\n\
g20 (inches mode)\n\
g17 (XY plane)\n\
g28 (home on power-up)\n\
f400.00\n\
s12000\n\
t1 \n\
mm per arc segment 0.01\n\
X map axis to motor 1\n\
 xse1891\n\
x feed steps 1892.123456789\n\
YDE1.8\n\
Xmicrosteps -1\n\
Xpolarity 0\n\
Xtravel 400.00\n\
XTW warning 425.00\n\
ZRE 1.27\n\
XID1\n\
XLI0\n";

void cfg_test()
{

//	char text[40];
	int i = 0;					// ROM buffer index (int allows for > 256 chars)
	int j = 0;					// RAM buffer index (text)
	char c;

	// feed the parser one line at a time
	while (TRUE) {
		c = pgm_read_byte(&configs_P[i++]);
		if (c == 0) {								// last line
			cp.block[j] = 0;
			cfg_parse(cp.block);
			break;			
		} else if (c == '\n') {						// line complete
			cp.block[j] = 0;							// terminate the string
			cfg_parse(cp.block);						// parse line 
			j = 0;			
		} else {
			cp.block[j++] = c;							// put characters into line
		}
	}
}

#endif
