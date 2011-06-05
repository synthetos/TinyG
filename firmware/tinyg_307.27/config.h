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
uint8_t cfg_config_parser(char *block, uint8_t display, uint8_t persist);
uint8_t cfg_print_config_help(void);
void cfg_dump_NVM(uint16_t start_record, uint16_t end_record, char *label);

#ifdef __UNIT_TESTS
void cfg_unit_tests(void);
#endif

/*
 * Global scope config structs
 */

// Global Config structure - per-axis structures
struct cfgStructAxis {
	// motor settings
	uint8_t map_axis;			// axis is mapped to what motor (0-3 internally)
	uint8_t axis_mode;			// 0=normal. A and Z have special modes
	uint16_t seek_steps;		// max seek whole steps per second (ex: 1600)
	uint16_t feed_steps;		// max feed whole steps per second (ex: 1200)
	double step_angle;			// degrees per whole step (ex: 1.8)
  	uint8_t microsteps;			// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;			// 0=normal polarity, 1=reverse motor direction
	double radius;				// radius for rotary axis feedrate computation
								// not used for linear axes
	// machine settings
 	uint8_t power_mode;			// 1=low power idle mode, 0=full power idle mode
	uint8_t limit_mode;			// 1=limit switches enabled, 0=not enabled

	double travel_rev;			// mm or deg of travel per motor revolution
	double travel_max;			// m of travel max in N dimension (ex: 400)
	double travel_warn;			// point to warn of travel violation

	uint8_t homing_enable;		// homing enabled for this axis
	double homing_rate;			// homing seek rate
	double homing_close;		// homing close rate
	double homing_offset;		// offset from zero at minimum
	double homing_backoff;		// axis backoff

	// computed values per axis (not stored)
	double steps_per_unit;		// steps (usteps)/mm or deg of travel
	double max_seek_rate;		// mm of trav in mm/min
	double max_feed_rate;		// mm of trav in mm/min
};

// Global Config structure - main structure
struct cfgStructGlobal {
	// Gcode defaults
	uint8_t gcode_plane;		// result of G17/G18/G19
	uint8_t gcode_units;		// 0=inches (G20), 1=mm (G21)
	uint8_t gcode_path_control;	// default path control mode
	uint8_t gcode_tool;
	double gcode_feed_rate;
	double gcode_spindle_speed;

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
	struct cfgStructAxis a[AXES];// holds axes X,Y,Z,A,B,C,U,V,W
};

struct cfgStructGlobal cfg; 	// declared in the header to make it global
#define CFG(x) cfg.a[x]			// handy macros for referencing axis values,
								// e.g: CFG(X_AXIS).steps_per_mm
#endif

/* 
 * How to use the config system:
 *
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
 *		X ST [0.00-360.00]	Step angle
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

