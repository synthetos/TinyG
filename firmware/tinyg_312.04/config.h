/*
 * config.h - configuration sub-system
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
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
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef config_h
#define config_h

#define CFG_NVM_BASE 0x0000		// base address of usable NVM

/*
 * Global Scope Functions
 */
void cfg_init(void);
void cfg_init_gcode_model(void);
uint8_t cfg_config_parser(char *block, uint8_t display, uint8_t persist);
uint8_t cfg_print_config_help(void);
void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, char *label);

#ifdef __UNIT_TESTS
void cfg_unit_tests(void);
#endif

/*
 * Global scope config structs
 */

// Global Config structure - per-axis structures
struct cfgStructAxis {
	uint8_t map_axis;			// axis is mapped to what motor (0-3 internally)
	uint8_t axis_mode;			// 0=normal. A and Z have special modes
	double seek_rate;			// mm of trav in mm/min
	double feed_rate;			// mm of trav in mm/min
	double travel_rev;			// mm or deg of travel per motor revolution
	double step_angle;			// degrees per whole step (ex: 1.8)
	double travel_max;			// m of travel max in N dimension (ex: 400)
	double radius;				// radius for rotary axis feedrate computation
								// not used for linear axes
	double steps_per_unit;		// steps (usteps)/mm or deg of travel

  	uint8_t microsteps;			// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;			// 0=normal polarity, 1=reverse motor direction
 	uint8_t power_mode;			// 1=low power idle mode, 0=full power idle mode
	uint8_t limit_mode;			// 1=limit switches enabled, 0=not enabled

	uint8_t homing_enable;		// homing enabled for this axis
	double homing_rate;			// homing seek rate
	double homing_close;		// homing close rate
	double homing_offset;		// offset from zero at minimum
	double homing_backoff;		// axis backoff
};

// Global Config structure - main structure
struct cfgStructGlobal {
	// Gcode defaults
	uint8_t gcode_units;		// default units 20,21 (in,mm)
	uint8_t gcode_plane;		// default plane 17,18,19
	double gcode_path_control;	// default path control 61,61.1,64

	// non-axis settings / globals
	double min_segment_len;		// arc and line drawing resolution in mm
	double min_segment_time;	// minimum segment time in microseconds
	double max_linear_jerk;		// linear jerk constant
	double angular_jerk_upper;	// angular jerk upper threshold >continuous
	double angular_jerk_lower;	// angular jerk lower threshold <exact stop

	uint8_t	motor_map[MOTORS];	// array to map motors to axes

	uint8_t homing_mode;		// 0=off, 1=power-on (G28)
	uint8_t homing_state;		// HOMING state
	uint8_t cycle_active;		// TRUE while cycle active (e.g. homing)
	uint8_t accel_enabled;		// enable acceleration control

 // axis structs
	struct cfgStructAxis a[AXES];// holds axes X,Y,Z,A [B,C,U,V,W]
};

struct cfgStructGlobal cfg; 	// declared in the header to make it global
#define CFG(x) cfg.a[x]			// handy macros for referencing axis values,
								// e.g: CFG(X_AXIS).steps_per_mm
#endif

/*
 * CONFIG INTERNALS !!! OUTDATED !!!!
 *
 *	Config is a collection of settings for:
 *		(1) Gcode defaults
 *		(2) non-axis machine settings
 *		(3) per-axis machine settings (4 axes are defined)
 *
 *	Settings are loaded into the cfg struct startup time (in binary form)
 *	Config is persisted to EEPROM as a set of ASCII config records ("records")
 *
 *	Records are of the form:
 *
 *	 <axis><token><value><null>
 *		axis	single letter [XYZA1234] Omitted for non-axis records
 *		token	two letters. See _cfg_sprintf_as_record() for definitions
 *		value	ascii formatted number, float or int as approriate
 *		null	strings are null terminated
 *
 *	Functions exist to move settings between the EEPROM and cfg.
 *
 *	A baseline (hardwired) config is defined in hardware.h. It is loaded 
 *		at power-up before attempting to read the EEPROM so the cfg struct 
 *		always has some degree of sanity even if the EEPROM fails or is not 
 *		initialized.
 *
 *	In addition to the stored settings there is a set of computed settings 
 *		in cfg that are derived from the config settings. These are 
 *		recomputed every time a config change occurs.
 *
 *	EEPROM is organized into configuration profiles, each containing a full 
 *		set of records. Currently only one profile is supported.
 *
 *	A profile is organized as so (with examples):
 *		length record	 _L11	location zero sets record length for profile
 *		revision record	 _R101	location one defines config format revision
 *		profile record	 _P1	location two indicates the profile number
 *		non-axis records		settings that do not have axis numbers
 *		per-axis records		4 sets of axis settings 
 *		trailer record	 _T0
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
 *		- if EEPROM is intialized and current:
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

