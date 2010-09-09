/*
  config.c - eeprom and compile time configuration handling 
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  TinyG Notes:
  Modified Grbl to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

  Enhanced the $ config with mnemonic system. See confih.h
*/

//#include "xmega_init.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

#include "tinyg.h"
#include "config.h"
#include "xmega_eeprom.h"
#include "xio.h"
#include "xio_wiring_compat.h"

void _config_computed(void);		// prototypes for local functions (helper functions) 

/* 
 * config_init() - initialize config system 
 */

void config_init() 
{
/*
	if(read_settings()) {
    	printPgmString(PSTR("'$' to dump current settings\r\n"));
	} else {
    	printPgmString(("EEPROM blank. Rewrote default settings:\r\n"));
    	reset_settings();
    	write_settings();
    	dump_settings();
	}
*/
	config_reset();
}

/* 
 * config_reset() - load default settings into config 
 */

void config_reset()
{
	cfg.config_version = CONFIG_VERSION;
	cfg.mm_per_arc_segment = MM_PER_ARC_SEGMENT;

	cfg.seek_steps_sec[X_AXIS] = X_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.seek_steps_sec[Y_AXIS] = Y_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.seek_steps_sec[Z_AXIS] = Z_SEEK_WHOLE_STEPS_PER_SEC;
	cfg.seek_steps_sec[A_AXIS] = A_SEEK_WHOLE_STEPS_PER_SEC;
	
	cfg.feed_steps_sec[X_AXIS] = X_FEED_WHOLE_STEPS_PER_SEC;
	cfg.feed_steps_sec[Y_AXIS] = Y_FEED_WHOLE_STEPS_PER_SEC;
	cfg.feed_steps_sec[Z_AXIS] = Z_FEED_WHOLE_STEPS_PER_SEC;
	cfg.feed_steps_sec[A_AXIS] = A_FEED_WHOLE_STEPS_PER_SEC;
	
	cfg.degree_per_step[X_AXIS] = X_DEGREE_PER_WHOLE_STEP;
	cfg.degree_per_step[Y_AXIS] = Y_DEGREE_PER_WHOLE_STEP;
	cfg.degree_per_step[Z_AXIS] = Z_DEGREE_PER_WHOLE_STEP;
	cfg.degree_per_step[A_AXIS] = A_DEGREE_PER_WHOLE_STEP;
	
	cfg.mm_per_rev[X_AXIS] = X_MM_PER_REVOLUTION;
	cfg.mm_per_rev[Y_AXIS] = Y_MM_PER_REVOLUTION;
	cfg.mm_per_rev[Z_AXIS] = Z_MM_PER_REVOLUTION;
	cfg.mm_per_rev[A_AXIS] = A_MM_PER_REVOLUTION;
	
	cfg.mm_travel[X_AXIS] = X_MM_TRAVEL;
	cfg.mm_travel[Y_AXIS] = Y_MM_TRAVEL;
	cfg.mm_travel[Z_AXIS] = Z_MM_TRAVEL;
	cfg.mm_travel[A_AXIS] = A_MM_TRAVEL;
	
	cfg.microstep[X_AXIS] = X_MICROSTEPS;
	cfg.microstep[Y_AXIS] = Y_MICROSTEPS;
	cfg.microstep[Z_AXIS] = Z_MICROSTEPS;
	cfg.microstep[A_AXIS] = A_MICROSTEPS;

	cfg.limit_enable[X_AXIS] = X_LIMIT_ENABLE;
	cfg.limit_enable[Y_AXIS] = Y_LIMIT_ENABLE;
	cfg.limit_enable[Z_AXIS] = Z_LIMIT_ENABLE;
	cfg.limit_enable[A_AXIS] = A_LIMIT_ENABLE;
	
	cfg.low_pwr_idle[X_AXIS] = X_LOW_POWER_IDLE_ENABLE;
	cfg.low_pwr_idle[Y_AXIS] = Y_LOW_POWER_IDLE_ENABLE;
	cfg.low_pwr_idle[Z_AXIS] = Z_LOW_POWER_IDLE_ENABLE;
	cfg.low_pwr_idle[A_AXIS] = A_LOW_POWER_IDLE_ENABLE;
	
	_config_computed();		// generate computed values from the above
}

/* 
 * _config_computed() - helper function to generate computed config values 
 *	call this every time you change any configs
 */

void _config_computed() 
{
	// 360 / (degree_per_step/microstep) / mm_per_rev
	for (int i = X_AXIS; i <= A_AXIS; i++) {
		cfg.steps_per_mm[i] = (360 / (cfg.degree_per_step[i] / cfg.microstep[i])) / cfg.mm_per_rev[i];
	}

	// feed_steps_sec / (360/degree_per_step/microstep)
	cfg.default_feed_rate = (cfg.feed_steps_sec[X_AXIS] * cfg.microstep[X_AXIS]) / 
							(360/(cfg.degree_per_step[X_AXIS] / cfg.microstep[X_AXIS]));

	// seek_steps_sec / (360/degree_per_step/microstep)
	cfg.default_seek_rate = (cfg.seek_steps_sec[X_AXIS] * cfg.microstep[X_AXIS]) / 
							(360/(cfg.degree_per_step[X_AXIS] / cfg.microstep[X_AXIS]));
}

/* 
 * config_read() - read config data from program ROM into the config struct 
 */

int config_read()
{
	uint8_t version = eeprom_get_char(0);	// Check version-byte of eeprom

	if (version != CONFIG_VERSION) {		// Read config-record and check checksum
		return(FALSE); 
	} 
  	if (!(memcpy_from_eeprom_with_checksum((char*)&cfg, 0, sizeof(struct Config)))) {
    	return(FALSE);
  	}
  	return(TRUE);
}

/* 
 * config_write() - write config struct to program ROM 
 */

void config_write()
{
//	eeprom_put_char(0, CONFIG_VERSION);
	memcpy_to_eeprom_with_checksum(0, (char*)&cfg, sizeof(struct Config));

}

/* config_parse() - parse a config string into the config record
 *
 *	YACLHCP - yet another crappy little hard coded parser for reading config values
 *	Config string may consist of one or more tag=value pairs 
 *
 *	Supported tags (axes X,Y,Z,A are supported - only X is shown)
 *	mm_arc_segment		0.1		arc drawing resolution in millimeters per segment 
 *	x_seek_steps_sec	1800	max seek whole steps per second for X axis
 *	x_feed_steps_sec	1200	max feed whole steps per second for X axis
 *	x_degree_step		1.8		degrees per whole step for X axis
 *	x_mm_rev			2.54	millimeters of travel per revolution of X axis
 *	x_mm_travel			406		millimeters of travel in X dimension (total envelope)
 * 	x_microstep			8		microsteps to apply for X axis steps
 *	x_low_pwr_idle		1		1=low power idle mode, 0=full power idle mode 
 *	x_limit_enable		1		1=max limit switch enabled, 0=not enabled
 *
 *	Tags are case and punctuation insensitive 
 *	All whitespace is ignored 
 *	Tags are only parsed to the point of uniqueness
 *	The following are equivalent
 *	  -	z_seek_steps_sec
 *	  -	z-seek-steps-sec
 * 	  -	z_seek_steps_*&*&^@#*&^_sec
 *	  -	zSeekStepsSec
 *	  -	zs
 *	  _	ZS
 *
 *	Tags and values are separated by '=' sign
 *	Whitespace is optional and ignored
 *	Values are read as floating point numbers and cast to proper internal types
 *	Integers received as fractional numbers are truncated (this shouldn't happen)
 */

int config_parse(char *text)
{
	char c;
	char *val = 0;				// pointer to normalized value 
	char *end = 0;				// pointer to end of value
	int i = 0;					// ROM buffer index (int allows for > 256 chars)
	int j = 0;					// RAM buffer index (text)
	int	axis = 0;				// axis index

	// normalize and split text in place
	while ((c = text[i++]) != 0) {
		if (c == '=') {									// handle separator
			text[j++] = 0;							// terminate tag at separator
			val = &text[j];							// new string starts at value
		} else if ((c == '-') || (c == '+') || (c == '.')) { // save special characters
			text[j++] = c;		
		} else if (c >= 'a' && c <= 'z') {				// convert lower to upper and save
			text[j++] = c-'a'+'A';
		} else if (c >= 'A' && c <= 'Z') {				// save upper case letters
			text[j++] = c;
		} else if (c >= '0' && c <= '9') {				// save numbers
			text[j++] = c;
		}
	}
	text[j++] = 0;
	end = &text[j];				// needed for string-to-double

	// pick off tag characters starting with first character
	switch (text[0]) {
		case 'M': cfg.mm_per_arc_segment = strtod(val, &end); return(0);
		case 'X': axis = X_AXIS; break;
		case 'Y': axis = Y_AXIS; break;
		case 'Z': axis = Z_AXIS; break;
		case 'A': axis = A_AXIS; break;
		default: return (1); 	// error
	}
	switch (text[1]) {
		case 'S': cfg.seek_steps_sec[axis] = (uint16_t)atoi(val); return(0);
		case 'F': cfg.feed_steps_sec[axis] = (uint16_t)atoi(val); return(0);
		case 'D': cfg.degree_per_step[axis] = strtod(val, &end); return(0);
		case 'M': 
			if (text[2] == 'I') {
				cfg.microstep[axis] = (uint8_t)atoi(val); return(0);
			} else if (text[3] == 'R') {
				cfg.mm_per_rev[axis] = strtod(val, &end); return(0);
			} else if (text[3] == 'T') {
				cfg.mm_travel[axis] = strtod(val, &end); return(0);
			}
		case 'L': 
			if (text[2] == 'O') {
				cfg.low_pwr_idle[axis] = (uint8_t)atoi(val); return(0);
			} else if (text[2] == 'I') {
				cfg.limit_enable[axis] = (uint8_t)atoi(val); return(0);
			}
		default: return (1);	// error
	}

//	config_write();
//	printPgmString(PSTR("Stored new setting\r\n"));
	return (0);
}

/* 
 * config_test() - generate some strings for the parser and test EEPROM read and write 
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

//	char text[40];
//	char c;
//	int j = 0;					// RAM buffer index (text)

void config_test()
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
			config_parse(text);
			break;			
		} else if ((c == '\r') || (c == '\n') || (c == ';')) {	// line complete
			text[j] = 0;							// terminate the string
			config_parse(text);						// parse line 
			j = 0;			
		} else if (c <= ' ') { 							// toss whitespace & ctrls
		} else {
			text[j++] = c;							// put characters into line
		}
	}
}
