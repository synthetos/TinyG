/*
 * config.h - configuration sub-system
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef config_h
#define config_h

#include <stdbool.h>

/**** cmdObj lists ****
 *
 * 	Commands and groups of commands are processed internally a doubly linked list
 *	cmdObj_t structures. This isolates the command and config internals from the 
 *	details of communications, parsing and display in text mode and JSON mode.
 *	The first element of the list is designated the response header element ("r") 
 *	but the list can also be serialized as a simple object by skipping over the header
 *
 *	To use the cmd list first reset it by calling cmd_reset_list(). This initializes
 *	the header, marks the the objects as TYPE_EMPTY, resets the shared string, and 
 *	terminates the last element by setting its NX pointer to NULL. When you use the 
 *	list you can terminate your own last element, or just leave the EMPTY elements 
 *	to be skipped over during outpout serialization.
 * 
 * 	We don;t use recursion so parent/child nesting relationships are captured in a 
 *	'depth' variable, This must remain consistent if the curlies  are to work out. 
 *	In general you should not have to track depth explicitly if you use cmd_reset_list()
 *	or the accessor functions like cmd_add_integer() or cmd_add_message(). 
 *	If you see problems with curlies check the depth values in the lists.
 *
 *	Use the cmd_print_list() dispatcher for all JSON and text output. Do not simply 
 *	run through printf.
 */
/*	Cmd object string handling
 *
 *	It's very expensive to allocate sufficient string space to each cmdObj, so cmds 
 *	use a cheater's malloc. A single string of length CMD_SHARED_STRING_LEN is shared
 *	by all cmdObjs for all strings. The observation is that the total rendered output
 *	in JSON or text mode cannot exceed the size of the output buffer (typ 256 bytes),
 *	So some number less than that is sufficient for shared strings. This is all mediated 
 *	through cmd_copy_string() and cmd_copy_string_P(), and cmd_reset_list().
 */
/*	Other Notes:
 *
 *	CMD_BODY_LEN needs to allow for one parent JSON object and enough children
 *	to complete the largest possible operation - usually the status report.
 */
									// chose one based on # of elements in cmdArray
//typedef uint8_t index_t;			// use this if there are < 255 indexed objects
typedef uint16_t index_t;			// use this if there are > 255 indexed objects
#define NO_MATCH (index_t)0xFFFF
									// cmdObj defines
#define CMD_GROUP_LEN 3				// max length of group prefix
#define CMD_TOKEN_LEN 5				// mnemonic token string: group prefix + short token
#define CMD_FORMAT_LEN 80			// print formatting string max length
#define CMD_MESSAGE_LEN 80			// sufficient space to contain end-user messages
#define CMD_FOOTER_LEN 18			// sufficient space to contain a JSON footer array
#define CMD_SHARED_STRING_LEN 200	// shared string for string values

									// cmdObj list defines
#define CMD_BODY_LEN 25				// body elements - allow for 1 parent + N children 
#define CMD_LIST_LEN (CMD_BODY_LEN+2)// +2 allows for a header and a footer
#define CMD_MAX_OBJECTS (CMD_BODY_LEN-1)// maximum number of objects in a body string

#define CMD_STATUS_REPORT_LEN 24	// max number of status report elements - see cfgArray
									// must also line up in cfgArray, se00 - seXX

#define NVM_VALUE_LEN 4				// NVM value length (double, fixed length)
#define NVM_BASE_ADDR 0x0000		// base address of usable NVM

#define IGNORE_OFF 0				// accept either CR or LF as termination on RX text line
#define IGNORE_CR 1					// ignore CR on RX
#define IGNORE_LF 2					// ignore LF on RX

enum objType {						// object / value typing for config and JSON
	TYPE_EMPTY = 0,					// object has no value (which is not the same as "NULL")
	TYPE_NULL,						// value is 'null' (meaning the JSON null value)
	TYPE_BOOL,						// value is "true" (1) or "false"(0)
	TYPE_INTEGER,					// value is a uint32_t
	TYPE_FLOAT,						// value is a floating point number
	TYPE_STRING,					// value is in string field
	TYPE_ARRAY,						// value is array element count, values are CSV ASCII in string field
	TYPE_PARENT						// object is a parent to a sub-object
};

enum cmdType {						// classification of commands
	CMD_TYPE_NULL = 0,
	CMD_TYPE_CONFIG,				// configuration commands
	CMD_TYPE_GCODE,					// gcode
	CMD_TYPE_REPORT					// SR, QR and any other report
};

enum tgCommunicationsMode {
	TEXT_MODE = 0,					// text command line mode
	JSON_MODE,						// strict JSON construction
	JSON_MODE_RELAXED				// relaxed JSON construction (future)
};

enum textVerbosity {
	TV_SILENT = 0,					// no response is provided
	TV_PROMPT,						// returns prompt only and exception messages
	TV_MESSAGES,					// returns prompt only and exception messages
	TV_CONFIGS,						// returns prompt, messages and echo config commands. Gcode blocks are not echoed 
	TV_VERBOSE						// returns all prompts, messages, configs and gcode blocks
};

enum jsonVerbosity {
	JV_SILENT = 0,					// no response is provided for any command
	JV_FOOTER,						// responses contain  footer only; no command echo, gcode blocks or messages
	JV_CONFIGS,						// echo configs; gcode blocks are not echoed; messages are not echoed
	JV_MESSAGES,					// echo configs; gcode messages only (if present); no block echo or line numbers
	JV_LINENUM,						// echo configs; gcode blocks return messages and line numbers as present
	JV_VERBOSE						// echos all configs and gcode blocks, line numbers and messages
};

enum qrVerbosity {					// planner queue enable and verbosity
	QR_OFF = 0,						// no response is provided
	QR_FILTERED,					// queue depth reported only above hi-water mark and below lo-water mark  
	QR_VERBOSE						// queue depth reported for all buffers
};

enum srVerbosity {					// status report enable and verbosity
	SR_OFF = 0,						// no reports
	SR_FILTERED,					// reports only values that have changed from the last report
	SR_VERBOSE						// reports all values specified
};

enum jsonFormats {					// json output print modes
	JSON_NO_PRINT = 0,				// don't print anything if you find yourself in JSON mode
	JSON_OBJECT_FORMAT,				// print just the body as a json object
	JSON_RESPONSE_FORMAT			// print the header/body/footer as a response object
};

enum textFormats {					// text output print modes
	TEXT_NO_PRINT = 0,				// don't print anything if you find yourself in TEXT mode
	TEXT_INLINE_PAIRS,				// print key:value pairs as comma separated pairs
	TEXT_INLINE_VALUES,				// print values as commas separated values
	TEXT_MULTILINE_FORMATTED		// print formatted values on separate lines with formatted print per line
};

typedef struct cmdString {			// shared string object
	uint16_t magic_start;
	uint8_t wp;						// current string array index
	char string[CMD_SHARED_STRING_LEN];
	uint16_t magic_end;
} cmdStr_t;

typedef struct cmdObject {			// depending on use, not all elements may be populated
	struct cmdObject *pv;			// pointer to previous object or NULL if first object
	struct cmdObject *nx;			// pointer to next object or NULL if last object
	index_t index;					// index of tokenized name, or -1 if no token (optional)
	int8_t depth;					// depth of object in the tree. 0 is root (-1 is invalid)
	int8_t type;					// see cmdType
	double value;					// numeric value
	char token[CMD_TOKEN_LEN+1];	// full mnemonic token for lookup
	char group[CMD_GROUP_LEN+1];	// group prefix or NUL if not in a group
	char (*stringp)[];				// pointer to array of characters from shared character array
} cmdObj_t; 						// OK, so it's not REALLY an object

typedef uint8_t (*fptrCmd)(cmdObj_t *cmd);// required for cmd table access
typedef void (*fptrPrint)(cmdObj_t *cmd);// required for PROGMEM access

// static allocation and definitions
cmdStr_t cmdStr;
cmdObj_t cmd_list[CMD_LIST_LEN];	// JSON header element
#define cmd_header cmd_list
#define cmd_body  (cmd_list+1)

/*
 * Global Scope Functions
 */

void cfg_init(void);
uint8_t cfg_text_parser(char *str);
uint8_t cfg_baud_rate_callback(void);

// main entry points for core access functions
uint8_t cmd_get(cmdObj_t *cmd);		// get value
uint8_t cmd_set(cmdObj_t *cmd);		// set value
void cmd_print(cmdObj_t *cmd);		// formatted print
void cmd_persist(cmdObj_t *cmd);	// persistence

// helpers
index_t cmd_get_index(const char *group, const char *token);
uint8_t cmd_get_type(cmdObj_t *cmd);
uint8_t cmd_set_jv(cmdObj_t *cmd);
uint8_t cmd_set_tv(cmdObj_t *cmd);
uint8_t cmd_persist_offsets(uint8_t flag);

// object and list functions
void cmd_get_cmdObj(cmdObj_t *cmd);
cmdObj_t *cmd_reset_obj(cmdObj_t *cmd);
cmdObj_t *cmd_reset_list(void);
uint8_t cmd_copy_string(cmdObj_t *cmd, const char *src);
uint8_t cmd_copy_string_P(cmdObj_t *cmd, const char *src_P);
cmdObj_t *cmd_add_object(char *token);
cmdObj_t *cmd_add_integer(char *token, const uint32_t value);
cmdObj_t *cmd_add_float(char *token, const double value);
cmdObj_t *cmd_add_string(char *token, const char *string);
cmdObj_t *cmd_add_string_P(char *token, const char *string);
cmdObj_t *cmd_add_message(const char *string);
cmdObj_t *cmd_add_message_P(const char *string);

void cmd_print_list(uint8_t status, uint8_t text_flags, uint8_t json_flags);
uint8_t cmd_group_is_prefixed(char *group);
uint8_t cmd_index_is_group(index_t index);

uint8_t cmd_read_NVM_value(cmdObj_t *cmd);
uint8_t cmd_write_NVM_value(cmdObj_t *cmd);

#ifdef __DEBUG
void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, char *label);
#endif

/**** Global scope config structures ****/

// main configuration parameter table
typedef struct cfgAxisParameters {
	uint8_t axis_mode;				// see tgAxisMode in gcode.h
	double feedrate_max;			// max velocity in mm/min or deg/min
	double velocity_max;			// max velocity in mm/min or deg/min
	double travel_max;				// work envelope w/warned or rejected blocks
	double jerk_max;				// max jerk (Jm) in mm/min^3
	double junction_dev;			// aka cornering delta
	double radius;					// radius in mm for rotary axis modes
	double search_velocity;			// homing search velocity
	double latch_velocity;			// homing latch velocity
	double latch_backoff;			// backoff from switches prior to homing latch movement
	double zero_backoff;			// backoff from switches for machine zero
	double jerk_homing;				// homing jerk (Jh) in mm/min^3
} cfgAxis_t;

typedef struct cfgMotorParameters {
	uint8_t	motor_map;				// map motor to axis
  	uint8_t microsteps;				// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor direction
 	uint8_t power_mode;				// 1=lo power idle mode, 0=full power idle mode
	double step_angle;				// degrees per whole step (ex: 1.8)
	double travel_rev;				// mm or deg of travel per motor revolution
	double steps_per_unit;			// steps (usteps)/mm or deg of travel
} cfgMotor_t;

typedef struct cfgPWMParameters {
  	double frequency;				// base frequency for PWM driver, in Hz
	double cw_speed_lo;             // minimum clockwise spindle speed [0..N]
    double cw_speed_hi;             // maximum clockwise spindle speed
    double cw_phase_lo;             // pwm phase at minimum CW spindle speed, clamped [0..1]
    double cw_phase_hi;             // pwm phase at maximum CW spindle speed, clamped [0..1]
	double ccw_speed_lo;            // minimum counter-clockwise spindle speed [0..N]
    double ccw_speed_hi;			// maximum counter-clockwise spindle speed
    double ccw_phase_lo;			// pwm phase at minimum CCW spindle speed, clamped [0..1]
    double ccw_phase_hi;			// pwm phase at maximum CCW spindle speed, clamped
    double phase_off;               // pwm phase when spindle is disabled
} cfgPWM_t;

typedef struct cfgParameters {
	uint16_t magic_start;			// magic number to test memory integity
	double fw_build;				// tinyg firmware build number
	double fw_version;				// tinyg firmware version number
	double hw_version;				// tinyg hardware compatibility

	uint16_t nvm_base_addr;			// NVM base address
	uint16_t nvm_profile_base;		// NVM base address of current profile

	// hidden settings				// not part of system group, but still accessible
	double min_segment_len;			// line drawing resolution in mm
	double arc_segment_len;			// arc drawing resolution in mm
	double chordal_tolerance;		// arc chordal accuracy setting in mm
	double estd_segment_usec;		// approximate segment time in microseconds
//	uint8_t enable_acceleration;	// enable acceleration control

	// system group settings
	double junction_acceleration;	// centripetal acceleration max for cornering
//	double max_spindle_speed;		// in RPM

	// gcode power-on default settings - defaults are not the same as the gm state
	uint8_t coord_system;			// G10 active coordinate system default
	uint8_t select_plane;			// G17,G18,G19 reset default
	uint8_t units_mode;				// G20,G21 reset default
	uint8_t path_control;			// G61,G61.1,G64 reset default
	uint8_t distance_mode;			// G90,G91 reset default

	// communications settings		// these first 4 are shadow settigns for XIO cntrl bits
	uint8_t ignore_crlf;			// ignore CR or LF on RX
	uint8_t enable_cr;				// enable CR in CRFL expansion on TX
	uint8_t enable_echo;			// enable text-mode echo
	uint8_t enable_xon;				// enable XON/XOFF mode
	uint8_t comm_mode;				// TG_TEXT_MODE or TG_JSON_MODE

	uint8_t enable_qr;				// queue reports enabled and verbosity level
	uint8_t qr_hi_water;
	uint8_t qr_lo_water;
	uint8_t json_verbosity;			// see enum in this file for settings
	uint8_t text_verbosity;			// see enum in this file for settings
	uint8_t usb_baud_rate;			// see xio_usart.h for XIO_BAUD values
	uint8_t usb_baud_flag;			// technically this belongs in the controller singleton

	uint8_t echo_json_footer;		// flags for JSON responses serialization
	uint8_t echo_json_configs;
	uint8_t echo_json_messages;
	uint8_t echo_json_linenum;
	uint8_t echo_json_gcode_block;

	uint8_t echo_text_prompt;		// flags for text mode response construction
	uint8_t echo_text_messages;
	uint8_t echo_text_configs;
	uint8_t echo_text_gcode_block;

	// status report configs
	uint8_t status_report_verbosity;					// see enum in this file for settings
	uint32_t status_report_interval;					// in MS. set non-zero to enable
	index_t status_report_list[CMD_STATUS_REPORT_LEN];	// status report elements to report
	double status_report_value[CMD_STATUS_REPORT_LEN];	// previous values for filtered reporting

	// coordinate systems and offsets
	double offset[COORDS+1][AXES];	// persistent coordinate offsets: absolute + G54,G55,G56,G57,G58,G59

	// motor and axis structs
	cfgMotor_t m[MOTORS];			// settings for motors 1-4
	cfgAxis_t a[AXES];				// settings for axes X,Y,Z,A B,C
	cfgPWM_t p;						// settings for PWM p

	uint16_t magic_end;
} cfgParameters_t;
cfgParameters_t cfg;

/* unit test setup */
//#define __UNIT_TEST_CONFIG		// uncomment to enable config unit tests
#ifdef __UNIT_TEST_CONFIG
void cfg_unit_tests(void);
#define	CONFIG_UNITS cfg_unit_tests();
#else
#define	CONFIG_UNITS
#endif // __UNIT_TEST_CONFIG

#endif
