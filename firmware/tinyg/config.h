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

/**** Command definitions and objects (used by config and JSON) ****/

// Choose one: This sets the index size into the cmdArray

//#define INDEX_T int16_t				// use this if there are  > 127 indexed objects
//#define INDEX_T int8_t			// use this if there are < 127 indexed objects
//#define NO_INDEX -1					// defined as no match
//alternate
#define INDEX_T uint8_t				// use this if there are < 255 indexed objects
#define NO_INDEX 0xFF				// defined as no match

#define CMD_GROUP_LEN 3				// max length of group prefix
#define CMD_TOKEN_LEN 5				// mnemonic token string: group prefix + short token
//#define CMD_INFIX_LEN 4				// token minus group prefix
#define CMD_STRING_LEN 80			// original value string or value as a string
#define CMD_FORMAT_LEN 64			// print formatting string
#define CMD_STATUS_REPORT_LEN 12	// max number of status report elements - see cfgArray
									// must also line up in cfgArray, se00 - seXX

/**** cmdObj lists ****
 *
 * 	Commands and groups of commands are processed internally as lists of cmdObj's.
 * 	This isolates the command and config internals from the details of text mode, 
 *	JSON mode and other communications issues. Commands live as an array of 
 *	objects in the body. The body is wrapped by a header that vuagely resembles
 *	an HTTP response header. Structure is:
 *	 - header	(depth 0 - contains the response parent ("r") and body parent ("body")
 *	 - body		(depth 2 - contains the meat of the command / config item(s)
 *	 - status	(depth 1 - contains the response status code and status message
 *	 - checksum	(depth 1 - contains checksum of all previous elements and a terminating object
 *
 *	Depending on the operation, a list will be processed for a variety of starting
 *	points: The header start, the body start, status or checksum elements.
 *
 *	Lists are linked together as doubly linked list (although I have yet to find 
 *	a use for the backwards pointer and may remove it). The last element of the list 
 *	has a null "next" pointer.
 * 
 *	List objects that are unused carry a value type of CMD_TYPE_EMPTY. 
 *
 * 	Because we don't have recursion parent/child nesting relationships are 
 *	captured in a 'depth' variable, This must remain consistent if the curlies 
 *	are to work out. In general you should not have to track depth explicitly 
 *	if you use cmd_clear_cmdObj or functions that call it (including cmd_get_cmdObj(), 
 *	cmd_clear_body(), cmd_clear_message() and some other low-level routines. 
 *	cmd_clear_cmdObj sets depth correctly based on the object's predecessor. 
 *	If you see problems with curlies check the depth values in the lists.
 *
 *	Use cmd_print_list() for all JSON and text output. Do not simply run these
 *	through printf. This function does some housekeeping including clearing the 
 *	body and message after the output string is queued.
 *
 *	Notes:
 *
 *	CMD_BODY_LEN needs to allow for one parent JSON object and enough children
 *	to complete the largest possible operation. Right now this is the axis group 
 *	query which has 20 elements for the rotary axes. 
 *
 *	CMD_TOTAL_LEN - this is the biggest memory hog in the whole system with 
 *	the possible exception of the planner queue. It is dominated by the size 
 *	of CMD_NAME_LEN and CMD_VALUE_STRING_LEN which are statically allocated 
 *	and should be as short as possible. 
 */
/*  Tokens, grousp and infixes 
 *
 * 	These 3 short strings are kept in the cmdObj:
 *	  - token
 *	  - group
 *	  - infix
 *
 *	The token is the full mnemonic token used for cmdArray lookup. It is either 
 *	taken directly from the input or assembled from group+token during input
 *	in parent/child JSON cases.
 *	
 *	The group is the group prefix - e.g. xyzabc, 1-4, p1, g54-g59. It may also 
 *	be "sys", but this is a special case. Whereas other group prefixes can be pre-
 *	pended to the infix to make the token, sys is the exception.
 *
 *	The infix is the characters remaining in the token once the group has been stripped.
 *	The infix is used for serializing JSON responses in parent/child cases.
 */
#define CMD_HEADER_LEN 1			// "b" header
#define CMD_BODY_LEN 25				// body elements - includes one terminator
#define CMD_FOOTER_LEN 2			// footer element (includes terminator element)

#define CMD_MAX_OBJECTS (CMD_BODY_LEN-1)// maximum number of objects in a body string
#define CMD_TOTAL_LEN (CMD_HEADER_LEN + CMD_BODY_LEN + CMD_FOOTER_LEN)
#define CMD_NAMES_FIELD_LEN (CMD_TOKEN_LEN + CMD_STRING_LEN +2)
#define CMD_STRING_FIELD_LEN (CMD_TOKEN_LEN + CMD_STRING_LEN + CMD_FORMAT_LEN +3)
#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)

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
	TG_TEXT_MODE = 0,				// default
	TG_JSON_MODE
//	TG_GRBL_MODE
};

enum jsonVerbosity {
	JV_SILENT = 0,					// no response is provided for any command
	JV_FOOTER_ONLY,					// response contains no body - footer only
	JV_OMIT_GCODE_BODY,				// body returned for configs; omitted for Gcode commands
	JV_GCODE_LINENUM_ONLY,			// body returned for configs; Gcode returns line number as 'n', otherwise body is omitted
	JV_GCODE_MESSAGES,				// body returned for configs; Gcode returns line numbers and messages only
	JV_VERBOSE						// body returned for configs and Gcode - Gcode comments removed
};

enum textVerbosity {
	TV_SILENT = 0,					// no response is provided
	TV_PROMPT,						// returns prompt only and exception messages
	TV_MESSAGES,					// returns prompt and all messages
	TV_VERBOSE						// returns prompt, echos command and all messages
};

enum qrEnable {						// planner queue enable and verbosity
	QR_OFF = 0,						// no response is provided
	QR_FILTERED,					// queue depth reported only above hi-water mark and below lo-water mark  
	QR_VERBOSE						// queue depth reported for all buffers
};

enum textReports {					// text output print modes
	TEXT_INLINE_PAIRS,				// print key:value pairs as comma separated pairs
	TEXT_INLINE_VALUES,				// print values as commas separated values
	TEXT_MULTILINE_FORMATTED		// print formatted values on separate lines with formatted print per line
};

struct cmdObject {					// depending on use, not all elements may be populated
	struct cmdObject *pv;			// pointer to previous object or NULL if first object
	struct cmdObject *nx;			// pointer to next object or NULL if last object
	INDEX_T index;					// index of tokenized name, or -1 if no token (optional)
	int8_t depth;					// depth of object in the tree. 0 is root (-1 is invalid)
	int8_t type;					// see cmdType
	double value;					// numeric value
	char token[CMD_TOKEN_LEN+1];	// full mnemonic token for lookup
	char group[CMD_GROUP_LEN+1];	// group prefix or NUL if not in a group
	char string[CMD_STRING_LEN+1];	// string storage (See note below)
}; 									// OK, so it's not REALLY an object
typedef struct cmdObject cmdObj;	// handy typedef for command onjects
typedef uint8_t (*fptrCmd)(cmdObj *cmd);// required for cmd table access
typedef void (*fptrPrint)(cmdObj *cmd);	// required for PROGMEM access
#define CMD_OBJ_CORE (sizeof(cmdObj) - (CMD_STRING_LEN+1))

// Allocate cmdObj lists
cmdObj cmd_header[CMD_HEADER_LEN];	// JSON header element
cmdObj cmd_body[CMD_BODY_LEN];		// cmd_body[0] is the root object
cmdObj cmd_footer[CMD_FOOTER_LEN];	// JSON footer element

/*
 * Global Scope Functions
 */

void cfg_init(void);
uint8_t cfg_text_parser(char *str);
uint8_t cfg_baud_rate_callback(void);

uint8_t cmd_get(cmdObj *cmd);		// main entry point for GETs
uint8_t cmd_set(cmdObj *cmd);		// main entry point for SETs
void cmd_print(cmdObj *cmd);		// main entry point for formatted print
void cmd_persist(cmdObj *cmd);		// main entry point for persistence

cmdObj *cmd_new_obj(cmdObj *cmd);
void cmd_get_cmdObj(cmdObj *cmd);
INDEX_T cmd_get_index(const char *group, const char *token);
uint8_t cmd_get_type(cmdObj *cmd);
uint8_t cmd_persist_offsets(uint8_t flag);

void cmd_new_list(void);
void cmd_new_body(cmdObj *cmd);
uint8_t cmd_add_object(char *token);
uint8_t cmd_add_string(char *token, char *string);
uint8_t cmd_add_integer(char *token, uint32_t value);
uint8_t cmd_add_float(char *token, double value);
void cmd_print_list(uint8_t status, uint8_t textmode);

uint8_t cmd_read_NVM_value(cmdObj *cmd);
uint8_t cmd_write_NVM_value(cmdObj *cmd);

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
	double search_velocity;			// homing search velocity
	double latch_velocity;			// homing latch velocity
	double latch_backoff;			// backoff from switches prior to homing latch movement
	double zero_backoff;			// backoff from switches for machine zero
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

struct cfgPWMParameters {
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
};

struct cfgParameters {
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
//	uint8_t outmap[MOTORS];			// array for mapping output bits

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

	// status report configs
	uint32_t status_report_interval;// in MS. set non-zero to enable
	INDEX_T status_report_list[CMD_STATUS_REPORT_LEN];

	// coordinate systems and offsets
	double offset[COORDS+1][AXES];	// persistent coordinate offsets: absolute + G54,G55,G56,G57,G58,G59

	// motor and axis structs
	struct cfgMotorParameters m[MOTORS];// settings for motors 1-4
	struct cfgAxisParameters a[AXES];	// settings for axes X,Y,Z,A B,C
	struct cfgPWMParameters p;			// settings for PWM p
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
#endif // __UNIT_TEST_CONFIG

#endif
