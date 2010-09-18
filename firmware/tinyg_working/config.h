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
 * Config header and trailer records
 * Header and trailer records must start with '%' sign
 * and be no more than CFG_RECORD_LEN-1 characters long
 * EEPROM_FORMAT_REVISION and CFG_HEADER revision must agree.
 */
#define EEPROM_FORMAT_REVISION 100 // Used to migrate data in firmware upgrades
#define CFG_HEADER  "%100L12P0"	// revision 100, record len = 12, profile = 0
#define CFG_TRAILER "%END"		// end of profile

/*
 * Global Scope Functions
 */

void cfg_init(void);			// init config system
void cfg_reset(void);			// reset configs
int cfg_parse(char *text);		// parse config record
void cfg_print_config_records(void);// print contents of config EEPROM
void cfg_print_config_struct(void);// print contents of config structure
void cfg_print_help_screen(void);
void cfg_tests(void);			// unit tests for config routines

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
	// Gcode defaults
	uint8_t gcode_plane;		// result of G17/G18/G19
	uint8_t gcode_units;		// 0=inches (G20), 1=mm (G21)
	uint8_t homing_mode;		// 0=off, 1=power-on (G28)
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
