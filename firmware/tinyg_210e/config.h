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

/* 
 * How to use the config system:
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
 *	Config is stored and used at run-time in the cfg struct (in binary form).
 *	Config is persisted to EEPROM as a set of ASCII config records ("records")
 *	Functions exist to move settings between the two.
 *	A baseline config is defined in hardware.h. It is loaded at power-up
 *	  before attempting to read the EEPROM so the cfg struct always has some
 *	  degree of sanity even if the EEPROM fails or is not initialized.
 *	In addition to hardware.h / user entered settings there are a set of 
 *	  computed settings in cfg that are derived from the config settings. 
 *	  These are recomputed every time a config change occurs.
 *
 *	EEPROM has a header record of format:
 *		'%'	first character
 *		"NNN"   format revision level EEPROM_FORMAT_REVISION (not build number!)
 *		"l:NN"  record length specifier (CFG_RECORD_LEN == 12)
 *
 *	EEPROM has a trailer record of format:
 *		'%'	first character
 *		"END"
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
 *		- else (EEPROM is intialized and current):
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
	uint8_t map_axis;			// axis is mapped to what motor (0-3 internally)
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
	uint8_t gcode_tool;
	uint16_t spindle_speed;

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
