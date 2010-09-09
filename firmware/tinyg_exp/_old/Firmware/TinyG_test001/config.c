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
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "hardware.h"
#include "xmega_eeprom.h"


void _cfg_computed(void); // prototypes for local functions (helper functions) 
void _cfg_dump_axis(uint8_t	axis);
void _cfg_print_status(uint8_t status_code, char *textbuf);

#define status(a) 	cfg.status = a

/* 
 * cfg_init() - initialize config system 
 */

void cfg_init() 
{
	cfg_reset();
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
	while ((c = text[i++]) != 0) {
		if (c == '=') {							// handle separator
			text[j++] = 0;						// terminate tag at separator
			val = &text[j];						// new string starts at value
		} else if ( (c == '-') || 				// pass special characters
					(c == '+') || 
					(c == '.') || 
					(c == '?') || 
					(c == '(')) { 
			text[j++] = c;
		} else if (c >= 'a' && c <= 'z') {		// convert lower to upper
			text[j++] = c-'a'+'A';
		} else if (c >= 'A' && c <= 'Z') {		// pass upper case letters
			text[j++] = c;
		} else if (c >= '0' && c <= '9') {		// pass numbers
			text[j++] = c;
		}
	}
	text[j++] = 0;								// terminate line
	end = &text[j];								// needed for string-to-double

	// pick off tag characters starting with first character
	cfg.status = TG_OK;
	switch (text[0]) {
		case '?': cfg_dump(); status (TG_OK); break;
		case '(': status (TG_OK);	break;			// ignore comment lines
		case 'Q': status (TG_QUIT); break;
		case 'M': cfg.mm_per_arc_segment = strtod(val, &end); 
				  status(TG_OK); 
				  break;

		case 'X': axis = X_AXIS; break;
		case 'Y': axis = Y_AXIS; break;
		case 'Z': axis = Z_AXIS; break;
		case 'A': axis = A_AXIS; break;

		default: status (TG_UNRECOGNIZED_COMMAND); 	// error return
	}
	if (cfg.status == TG_OK) {
		status(TG_OK);							// pre-emptive setting
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

			default: status (TG_UNRECOGNIZED_COMMAND);	// error return
		}
	}
	_cfg_print_status(cfg.status, text);
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
	printf_P(PSTR("  mm_per_arc_segment: %5.3f mm / segment\n"), cfg.mm_per_arc_segment);
	printf_P(PSTR(" (default_seek_rate:  %5.3f mm / second)\n"), cfg.default_seek_rate);
	printf_P(PSTR(" (default_feed_rate:  %5.3f mm / second)\n\n"), cfg.default_feed_rate);

	for (uint8_t axis = 0; axis <= A_AXIS; axis++) {
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
	cfg.config_version = CONFIG_VERSION;
	cfg.mm_per_arc_segment = MM_PER_ARC_SEGMENT;

	cfg.a[X_AXIS].seek_steps_sec = X_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Y_AXIS].seek_steps_sec = Y_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[Z_AXIS].seek_steps_sec = Z_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.a[A_AXIS].seek_steps_sec = A_SEEK_WHOLE_STEPS_PER_SEC;

	cfg.a[X_AXIS].feed_steps_sec = X_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Y_AXIS].feed_steps_sec = Y_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[Z_AXIS].feed_steps_sec = Z_FEED_WHOLE_STEPS_PER_SEC;
	cfg.a[A_AXIS].feed_steps_sec = A_FEED_WHOLE_STEPS_PER_SEC;

	cfg.a[X_AXIS].degree_per_step = X_DEGREE_PER_WHOLE_STEP;
	cfg.a[Y_AXIS].degree_per_step = Y_DEGREE_PER_WHOLE_STEP;
	cfg.a[Z_AXIS].degree_per_step = Z_DEGREE_PER_WHOLE_STEP;
	cfg.a[A_AXIS].degree_per_step = A_DEGREE_PER_WHOLE_STEP;

	cfg.a[X_AXIS].mm_per_rev = X_MM_PER_REVOLUTION;
	cfg.a[Y_AXIS].mm_per_rev = Y_MM_PER_REVOLUTION;
	cfg.a[Z_AXIS].mm_per_rev = Z_MM_PER_REVOLUTION;
	cfg.a[A_AXIS].mm_per_rev = A_MM_PER_REVOLUTION;
	
	cfg.a[X_AXIS].mm_travel = X_MM_TRAVEL;
	cfg.a[Y_AXIS].mm_travel = Y_MM_TRAVEL;
	cfg.a[Z_AXIS].mm_travel = Z_MM_TRAVEL;
	cfg.a[A_AXIS].mm_travel = A_MM_TRAVEL;
	
	cfg.a[X_AXIS].microstep = X_MICROSTEPS;
	cfg.a[Y_AXIS].microstep = Y_MICROSTEPS;
	cfg.a[Z_AXIS].microstep = Z_MICROSTEPS;
	cfg.a[A_AXIS].microstep = A_MICROSTEPS;

	cfg.a[X_AXIS].polarity = X_POLARITY;
	cfg.a[Y_AXIS].polarity = Y_POLARITY;
	cfg.a[Z_AXIS].polarity = Z_POLARITY;
	cfg.a[A_AXIS].polarity = A_POLARITY;

	cfg.a[X_AXIS].limit_enable = X_LIMIT_ENABLE;
	cfg.a[Y_AXIS].limit_enable = Y_LIMIT_ENABLE;
	cfg.a[Z_AXIS].limit_enable = Z_LIMIT_ENABLE;
	cfg.a[A_AXIS].limit_enable = A_LIMIT_ENABLE;

	cfg.a[X_AXIS].low_pwr_idle = X_LOW_POWER_IDLE;
	cfg.a[Y_AXIS].low_pwr_idle = Y_LOW_POWER_IDLE;
	cfg.a[Z_AXIS].low_pwr_idle = Z_LOW_POWER_IDLE;
	cfg.a[A_AXIS].low_pwr_idle = A_LOW_POWER_IDLE;

	_cfg_computed();		// generate computed values from the above
}

/* 
 * _cfg_computed() - helper function to generate computed config values 
 *	call this every time you change any configs
 */

void _cfg_computed() 
{
	// = 360 / (degree_per_step/microstep) / mm_per_rev
	for (int i = X_AXIS; i <= A_AXIS; i++) {
		cfg.a[i].steps_per_mm = (360 / (cfg.a[i].degree_per_step / 
										cfg.a[i].microstep)) / 
										cfg.a[i].mm_per_rev;
	}

	// = feed_steps_sec / (360/degree_per_step/microstep)
	cfg.default_feed_rate = (cfg.a[X_AXIS].feed_steps_sec * 
							  cfg.a[X_AXIS].microstep) / 
							 (360/(cfg.a[X_AXIS].degree_per_step / 
							 	   cfg.a[X_AXIS].microstep));

	// = seek_steps_sec / (360/degree_per_step/microstep)
	cfg.default_seek_rate = (cfg.a[X_AXIS].seek_steps_sec * 
							  cfg.a[X_AXIS].microstep) / 
							 (360/(cfg.a[X_AXIS].degree_per_step / 
							 	   cfg.a[X_AXIS].microstep));
}

/* 
 * cfg_read() - read config data from EEPROM into the config struct 
 */

int cfg_read()
{
	uint8_t version = eeprom_get_char(0);	// Check version-byte of eeprom

	if (version != CONFIG_VERSION) {		// Read config-record and check checksum
		return(FALSE); 
	} 
  	if (!(memcpy_from_eeprom_with_checksum
		((char*)&cfg, 0, sizeof(struct cfgStructGlobal)))) {
    	return(FALSE);
  	}
  	return(TRUE);
}

/* 
 * cfg_write() - write config struct to program ROM 
 */

void cfg_write()
{
//	eeprom_put_char(0, CONFIG_VERSION);
	memcpy_to_eeprom_with_checksum(0, (char*)&cfg, sizeof(struct cfgStructGlobal));
}

/*
 * _cfg_print_status
 */

void _cfg_print_status(uint8_t status_code, char *textbuf)
{
	switch(status_code) {
		case TG_OK: {
#ifdef __DEBUG
			printf_P(PSTR("Config command: %s\n"), textbuf);
#endif
			break;
		};
		case TG_CONTINUE: 
			printf_P(PSTR("Config Continuation for: %s\n"), textbuf); 
			break;

		case TG_QUIT: 
			printf_P(PSTR("Quitting Config Mode\n")); 
			break;

		case TG_BAD_NUMBER_FORMAT: 
			printf_P(PSTR("Bad Number Format: %s\n"), textbuf); 
			break;

		case TG_UNRECOGNIZED_COMMAND: 
			printf_P(PSTR("Unrecognized Command: %s\n"), textbuf); 
			break;

		case TG_FLOATING_POINT_ERROR: 
			printf_P(PSTR("Floating Point Error: %s\n"), textbuf); 
			break;

		case TG_ARC_ERROR:
			printf_P(PSTR("Illegal Arc Statement: %s\n"), textbuf); 
			break;
	}
	return;
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
