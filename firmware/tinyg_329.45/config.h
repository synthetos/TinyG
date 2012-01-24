/*
 * config.h - configuration sub-system
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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

#define NVM_RECORD_LEN 8			// NVM record length (fixed length)
#define NVM_BASE_ADDR 0x0000		// base address of usable NVM

// Choose one: (this sets the max # of tokens data type in all places that need it)
#define INDEX_T int16_t				// change to this when > 127 records
//#define INDEX_T int8_t			// OK as long as there are less than 127 records
//...and you are way past that now.

/**** Command definitions and objects (used by config and JSON) ****/

#define CMD_TOKEN_LEN 3				// mnemonic token string
#define CMD_NAME_LEN 24				// friendly name string
#define CMD_FORMAT_LEN 48			// print formatting string
#define CMD_VALUE_STRING_LEN 64		// original value string or value as a string
#define JSON_OVERHEAD 16			// allowance for additional JSON characters

#define CMD_NAMES_FIELD_LEN (CMD_TOKEN_LEN + CMD_NAME_LEN +2)
#define CMD_STRING_FIELD_LEN (CMD_TOKEN_LEN + CMD_NAME_LEN + CMD_FORMAT_LEN +3)
//#define JSON_STRING_LEN (CMD_NAME_LEN + CMD_VALUE_STRING_LEN + JSON_OVERHEAD)
#define JSON_STRING_LEN (CHAR_BUFFER_SIZE)

enum cmdValueType {					// value typing for config and JSON
	VALUE_TYPE_ERROR = -2,			// was unable to process the record
	VALUE_TYPE_NULL = -1,			// value is 'null'
	VALUE_TYPE_FALSE = false,		// value is 'false'
	VALUE_TYPE_TRUE = true,			// value is 'true'
	VALUE_TYPE_NUMBER,				// value is a number
	VALUE_TYPE_STRING,				// value is in vstring field
	VALUE_TYPE_PARENT				// object is a parent to a sub-object
};

struct cmdObject {					// depending on use, not all elements may be populated
	INDEX_T index;					// index of tokenized name, or -1 if no token
	uint8_t nesting_level;			// 0 is root
	struct cmdObject *nx;			// pointer to next object or NULL if last (or only) object
	uint8_t	status;					// return status for this object
	int8_t value_type;				// see cfgValueType
	double value;					// numeric value (if applicable)
	char token[CMD_TOKEN_LEN+1];	// mnemonic token 
	char name[CMD_NAME_LEN+1];		// JSON name field or friendly name
	char vstring[CMD_VALUE_STRING_LEN+1];// value string (if applicable)
};									// OK, so it's not REALLY an object
typedef struct cmdObject cmdObj;
typedef uint8_t (*fptrCmd)(const INDEX_T i, cmdObj *cmd); // required for cmd table access

/*
 * Global Scope Functions
 */

void cfg_init(void);
uint8_t cfg_config_parser(char *str);
void cfg_init_gcode_model(void);

// cmd accessors
struct cmdObject *cmd_new_object(cmdObj *cmd);
uint8_t cmd_get(const INDEX_T i, cmdObj *cmd);	// entry point for gets
uint8_t cmd_set(const INDEX_T i, cmdObj *cmd);	// entry point for sets
void cmd_print(const INDEX_T i);				// entry point for print
INDEX_T cmd_get_index(const char *str);
INDEX_T cmd_get_index_by_token(const char *str);
char *cmd_get_token(const INDEX_T i, char *string);
uint8_t cmd_read_NVM(const INDEX_T i, cmdObj *cmd);
uint8_t cmd_write_NVM(const INDEX_T i, const cmdObj *cmd);

#ifdef __DEBUG
void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, char *label);
#endif

/**** Global scope config structures ****/

// main configuration parameter table
struct cfgAxisParameters {
	uint8_t axis_mode;				// see tgAxisMode in gcode.h
	double feedrate_max;			// max velocity in mm/min or deg/min
	double velocity_max;			// max velocity in mm/min or deg/min
	double travel_max;				// work envelope w/warned or rejected blocks
	double jerk_max;				// max jerk (Jm) in mm/min^3
	double junction_dev;			// aka cornering delta
	double radius;					// radius in mm for rotary axis modes

	// homing cycle settings
	uint8_t switch_mode;			// 1=limit switches enabled, 0=not enabled
	double homing_travel;			// distance between crashes or switches
	double homing_search_velocity;	// homing search velocity
	double homing_latch_velocity;	// homing latch velocity
	double homing_zero_offset;		// machine coordinate system zero offset from switches (backoff value)
	double homing_work_offset;		// work coorinate system zero offset from machine zero
};

struct cfgMotorParameters {
	uint8_t	motor_map;				// map motor to axis
  	uint8_t microsteps;				// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor direction
 	uint8_t power_mode;				// 1=lo power idle mode, 0=full power idle mode
	double step_angle;				// degrees per whole step (ex: 1.8)
	double travel_rev;				// mm or deg of travel per motor revolution
	double steps_per_unit;			// steps (usteps)/mm or deg of travel
};

struct cfgParameters {
	double profile;					// configuration profile in effect
	double version;					// configuration version for migration

	uint16_t nvm_base_addr;			// NVM base address
	uint16_t nvm_profile_base;		// NVM base address of current profile

	// general settings / globals
	double min_segment_len;			// line drawing resolution in mm
	double arc_segment_len;			// arc drawing resolution in mm
	double estd_segment_usec;		// approximate segment time in microseconds
	double corner_acceleration;		// centripetal acceleration max for cornering
	uint8_t enable_acceleration;	// enable acceleration control
	uint8_t status_report_enabled;
	uint8_t status_report_interval;	// in MS. set non-zero to enable

	// gcode power-on default settings
	double select_plane;			// defaults are not the same as the gm versions
	double inches_mode;
	double path_control;
	double absolute_mode;

	// Note: configurable serial IO settings are located in XIO

	// axis and motor structs
	struct cfgAxisParameters a[AXES];	// settings for axes X,Y,Z,A B,C
	struct cfgMotorParameters m[MOTORS];// settings for motors 1-4

	// command object memory allocation and support
//	cmdObj cmd[CMD_OBJECT_ARRAY_SIZE];
};
struct cfgParameters cfg; 			// declared in the header to make it global
#define CFG(x) cfg.a[x]				// handy macro for referencing axis values,
									// e.g: CFG(X_AXIS).steps_per_mm

/* unit test setup */
//#define __UNIT_TEST_CONFIG		// uncomment to enable config unit tests
#ifdef __UNIT_TEST_CONFIG
void cfg_unit_tests(void);
#define	CONFIG_UNITS cfg_unit_tests();
#else
#define	CONFIG_UNITS
#endif // __UNIT_TESTS_CONFIG

#endif
