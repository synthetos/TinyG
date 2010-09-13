/*
 * config.h - configuration sub-system
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

#ifndef config_h
#define config_h

/*
 * Global Scope Functions
 */

void cfg_init(void);			// initialize config struct by reading from EEPROM
void cfg_reset(void);			// reset config values to defaults
int cfg_parse(char *text);		// parse a tag=value config string
int cfg_read(void);				// read config record from EEPROM
void cfg_write(void);			// write config record to EEPROM
void cfg_dump(void);
void cfg_test(void);			// unit tests for config routines
void cfg_test2(void);			// unit tests for config routines

/*
 * Global scope config structs
 */

struct cfgStructAxis {
 // motor configuration
  	uint8_t microstep;			// microsteps to apply for each axis (ex: 8)
 	uint8_t low_pwr_idle;		// 1=low power idle mode, 0=full power idle mode
	uint8_t polarity;			// 0=normal polarity, 1=reverse motor direction
	uint16_t seek_steps_sec;	// max seek whole steps per second (ex: 1600)
	uint16_t feed_steps_sec;	// max feed whole steps per second (ex: 1200)
	double degree_per_step;		// degrees per whole step (ex: 1.8)
 // machine configuration
	double mm_per_rev;			// millimeters of travel per revolution (ex: 2.54)
	double mm_travel;			// millimeters of travel max in N dimension (ex: 400)
	double steps_per_mm;		// # steps (actually usteps)/mm of travel (COMPUTED)
	uint8_t limit_enable;		// 1=limit switches enabled, 0=not enabled
};

struct cfgStructGlobal {
	uint8_t config_version;		// config format version. starts at 100
	uint8_t status;				// interpreter status
 // model configuration
	double mm_per_arc_segment;	// arc drawing resolution in millimeters per segment
	double max_feed_rate;		// mm of trav in mm/min (COMPUTED)
	double max_seek_rate;		// mm of trav in mm/min (COMPUTED)
 // axis structs
	struct cfgStructAxis a[4];	// holds axes X,Y,Z,A
};

struct cfgStructGlobal cfg; 	// declared in the header to make it global
#define CFG(x) cfg.a[x]			// handy macro for referencing the axis values, 
								// e.g: CFG(X_AXIS).steps_per_mm
#endif
