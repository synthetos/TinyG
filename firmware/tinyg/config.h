/*
 * config.h - configuration sub-system generic part (see config_app for application part)
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CONFIG_H_ONCE
#define CONFIG_H_ONCE

/***** PLEASE NOTE *****
#include "config_app.h"	// is present at the end of this file
*/

#ifdef __cplusplus
extern "C"{
#endif

/**** Config System Overview and Usage ***
 *
 *	--- Config objects and the config list ---
 *
 *	The config system provides a structured way to get, set and print configuration variables.
 *	The config system operates as a list of "objects" (OK, so they are not really objects)
 *	that encapsulate each variable. The list may also may have header and footer objects.
 *
 *	The list is populated by the text_parser or the JSON_parser depending on the mode.
 *	This way the internals don't care about how the variable is represented or communicated
 *	externally as all internal operations occur on the nvObjs, not the wire form (text or JSON).
 */
/*	--- Config variables, tables and strings ---
 *
 *	Each configuration value is identified by a short mnemonic string (token). The token
 *	is resolved to an index into the cfgArray which is an array of structures with the
 *	static assignments for each variable. The cfgArray contains typed data in program
 *	memory (PROGMEM, in the AVR).
 *
 *	Each cfgItem has:
 *	 - group string identifying what group the variable is part of; or "" if no group
 *	 - token string - the token for that variable - pre-pended with the group (if present)
 *	 - operations flags - e.g. if the value should be initialized and/or persisted to NVM
 *	 - function pointer for formatted print() method for text-mode readouts
 *	 - function pointer for get() method - gets value from memory
 *	 - function pointer for set() method - sets value and runs functions
 *	 - target - memory location that the value is written to / read from
 *	 - default value - for cold initialization
 *
 *	Additionally an NVM array contains values persisted to EEPROM as floats; indexed by cfgArray index
 *
 *	The following rules apply to mnemonic tokens
 *	 - are up to 5 alphnuneric characters and cannot contain whitespace or separators
 *	 - must be unique (non colliding).
 *	 - axis tokens start with the axis letter and are typically 3 characters including the axis letter
 *	 - motor tokens start with the motor digit and are typically 3 characters including the motor digit
 *	 - non-axis or non-motor tokens are 2-5 characters and by convention generally should not start
 *		with: xyzabcuvw0123456789 (but there can be exceptions)
 *
 *  "Groups" are collections of values that mimic REST resources. Groups include:
 *	 - axis groups prefixed by "xyzabc"		("uvw" are reserved)
 *	 - motor groups prefixed by "1234"		("56789" are reserved)
 *	 - PWM groups prefixed by p1, p2 	    (p3 - p9 are reserved)
 *	 - coordinate system groups prefixed by g54, g55, g56, g57, g59, g92
 *	 - a system group is identified by "sys" and contains a collection of otherwise unrelated values
 *
 *	"Uber-groups" are groups of groups that are only used for text-mode printing - e.g.
 *	 - group of all axes groups
 *	 - group of all motor groups
 *	 - group of all offset groups
 *	 - group of all groups
 */
/*  --- Making changes and adding new values
 *
 *	Adding a new value to config (or changing an existing one) involves touching the following places:
 *
 *	 - Create a new record in cfgArray[]. Use existing ones for examples.
 *
 *	 - Create functions for print, get, and set. You can often use existing generic fucntions for
 *	   get and set, and sometimes print. If print requires any custom text it requires it's own function
 *	   Look in the modules for examples - e.g. at the end of canoonical_machine.c
 *
 *	 - The ordering of group displays is set by the order of items in cfgArray. None of the other
 *	   orders matter but are generally kept sequenced for easier reading and code maintenance. Also,
 *	   Items earlier in the array will resolve token searches faster than ones later in the array.
 *
 *	   Note that matching will occur from the most specific to the least specific, meaning that
 *	   if tokens overlap the longer one should be earlier in the array: "gco" should precede "gc".
 */

/**** nvObj lists ****
 *
 * 	Commands and groups of commands are processed internally a doubly linked list of nvObj_t
 *	structures. This isolates the command and config internals from the details of communications,
 *	parsing and display in text mode and JSON mode.
 *
 *	The first element of the list is designated the response header element ("r") but the list
 *	can also be serialized as a simple object by skipping over the header
 *
 *	To use the nvObj list first reset it by calling nv_reset_nv_list(). This initializes the
 *	header, marks the the objects as TYPE_EMPTY (-1), resets the shared string, relinks all objects
 *	with NX and PV pointers, and makes the last element the terminating element by setting its NX
 *	pointer to NULL. The terminating element may carry data, and will be processed.
 *
 *	When you use the list you can terminate your own last element, or just leave the EMPTY elements
 *	to be skipped over during output serialization.
 *
 * 	We don't use recursion so parent/child nesting relationships are captured in a 'depth' variable,
 *	This must remain consistent if the curlies are to work out. You should not have to track depth
 *	explicitly if you use nv_reset_nv_list() or the accessor functions like nv_add_integer() or
 *	nv_add_message(). If you see problems with curlies check the depth values in the lists.
 *
 *	Use the nv_print_list() dispatcher for all JSON and text output. Do not simply run through printf.
 */
/*	Token and Group Fields
 *
 *	The nvObject struct (nvObj_t) has strict rules on the use of the token and group fields.
 *	The following forms are legal which support the use cases listed:
 *
 *	Forms
 *	  - group is NUL; token is full token including any group profix
 *	  - group is populated; token is carried without the group prefix
 *	  - group is populated; token is NUL - indicates a group operation
 *
 *  Use Cases
 *	  - Lookup full token in cfgArray to get the index. Concatenates grp+token as key
 *	  - Text-mode displays. Concatenates grp+token for display, may also use grp alone
 *	  - JSON-mode display for single - element value e.g. xvm. Concatenate as above
 *	  - JSON-mode display of a parent/child group. Parent is named grp, children nems are tokens
 */
/*	--- nv object string handling ---
 *
 *	It's very expensive to allocate sufficient string space to each nvObj, so nv uses a cheater's
 *	malloc. A single string of length NV_SHARED_STRING_LEN is shared by all nvObjs for all strings.
 *	The observation is that the total rendered output in JSON or text mode cannot exceed the size of
 *	the output buffer (typ 256 bytes), So some number less than that is sufficient for shared strings.
 *	This is all mediated through nv_copy_string(), nv_copy_string_P(), and nv_reset_nv_list().
 */
/*  --- Setting nvObj indexes ---
 *
 *	It's the responsibility of the object creator to set the index. Downstream functions
 *	all expect a valid index. Set the index by calling nv_get_index(). This also validates
 *	the token and group if no lookup exists. Setting the index is an expensive operation
 *	(linear table scan), so there are some exceptions where the index does not need to be set.
 *	These cases are put in the code, commented out, and explained.
 */
/*	--- Other Notes:---
 *
 *	NV_BODY_LEN needs to allow for one parent JSON object and enough children to complete the
 *	largest possible operation - usually the status report.
 */

/***********************************************************************************
 **** DEFINITIONS AND SETTINGS *****************************************************
 ***********************************************************************************/

// Sizing and footprints				// chose one based on # of elements in cfgArray
//typedef uint8_t index_t;				// use this if there are < 256 indexed objects
typedef uint16_t index_t;				// use this if there are > 255 indexed objects

										// defines allocated from stack (not-pre-allocated)
#define NV_FORMAT_LEN 128				// print formatting string max length
#define NV_MESSAGE_LEN 128				// sufficient space to contain end-user messages

										// pre-allocated defines (take RAM permanently)
#define NV_SHARED_STRING_LEN 512		// shared string for string values
#define NV_BODY_LEN 30					// body elements - allow for 1 parent + N children
										// (each body element takes about 30 bytes of RAM)

// Stuff you probably don't want to change

#define GROUP_LEN 3						// max length of group prefix
#define TOKEN_LEN 5						// mnemonic token string: group prefix + short token
#define NV_FOOTER_LEN 18				// sufficient space to contain a JSON footer array
#define NV_LIST_LEN (NV_BODY_LEN+2)		// +2 allows for a header and a footer
#define NV_MAX_OBJECTS (NV_BODY_LEN-1)	// maximum number of objects in a body string
#define NO_MATCH (index_t)0xFFFF
#define NV_STATUS_REPORT_LEN NV_MAX_OBJECTS // max number of status report elements - see cfgArray
											// **** must also line up in cfgArray, se00 - seXX ****

enum tgCommunicationsMode {
	TEXT_MODE = 0,						// text command line mode
	JSON_MODE,							// strict JSON construction
	JSON_MODE_RELAXED					// relaxed JSON construction (future)
};

enum flowControl {
	FLOW_CONTROL_OFF = 0,				// flow control disabled
	FLOW_CONTROL_XON,					// flow control uses XON/XOFF
	FLOW_CONTROL_RTS					// flow control uses RTS/CTS
};

/*
enum lineTermination {					// REMOVED. Too easy to make the board non-responsive (not a total brick, but close)
	IGNORE_OFF = 0,						// accept either CR or LF as termination on RX text line
	IGNORE_CR,							// ignore CR on RX
	IGNORE_LF							// ignore LF on RX
};
*/
/*
enum tgCommunicationsSticky {
	NOT_STICKY = 0,						// communications mode changes automatically
	STICKY								// communications mode does not change
};
*/

enum valueType {						// value typing for config and JSON
	TYPE_EMPTY = -1,					// value struct is empty (which is not the same as "NULL")
	TYPE_NULL = 0,						// value is 'null' (meaning the JSON null value)
	TYPE_BOOL,							// value is "true" (1) or "false"(0)
	TYPE_INTEGER,						// value is a uint32_t
	TYPE_DATA,							// value is blind cast to uint32_t
	TYPE_FLOAT,							// value is a floating point number
	TYPE_STRING,						// value is in string field
	TYPE_ARRAY,							// value is array element count, values are CSV ASCII in string field
	TYPE_PARENT							// object is a parent to a sub-object
};

/**** operations flags and shorthand ****/

#define F_INITIALIZE	0x01			// initialize this item (run set during initialization)
#define F_PERSIST 		0x02			// persist this item when set is run
#define F_NOSTRIP		0x04			// do not strip the group prefix from the token
#define F_CONVERT		0x08			// set if unit conversion is required

#define _f0				0x00
#define _fi				(F_INITIALIZE)
#define _fp				(F_PERSIST)
#define _fn				(F_NOSTRIP)
#define _fc				(F_CONVERT)
#define _fip			(F_INITIALIZE | F_PERSIST)
#define _fipc			(F_INITIALIZE | F_PERSIST | F_CONVERT)
#define _fipn			(F_INITIALIZE | F_PERSIST | F_NOSTRIP)
#define _fipnc			(F_INITIALIZE | F_PERSIST | F_NOSTRIP | F_CONVERT)

/**** Structures ****/

typedef struct nvString {				// shared string object
	uint16_t magic_start;
  #if (NV_SHARED_STRING_LEN < 256)
	uint8_t wp;							// use this string array index value if string len < 256 bytes
  #else
	uint16_t wp;						// use this string array index value is string len > 255 bytes
  #endif
	char_t string[NV_SHARED_STRING_LEN];
	uint16_t magic_end;					// guard to detect string buffer underruns
} nvStr_t;

typedef struct nvObject {				// depending on use, not all elements may be populated
	struct nvObject *pv;				// pointer to previous object or NULL if first object
	struct nvObject *nx;				// pointer to next object or NULL if last object
	index_t index;						// index of tokenized name, or -1 if no token (optional)
	int8_t depth;						// depth of object in the tree. 0 is root (-1 is invalid)
	int8_t valuetype;					// see valueType enum
	int8_t precision;					// decimal precision for reporting (JSON)
	float value;						// numeric value
	char_t group[GROUP_LEN+1];			// group prefix or NUL if not in a group
	char_t token[TOKEN_LEN+1];			// full mnemonic token for lookup
	char_t (*stringp)[];				// pointer to array of characters from shared character array
} nvObj_t; 								// OK, so it's not REALLY an object

typedef uint8_t (*fptrCmd)(nvObj_t *nv);// required for cfg table access
typedef void (*fptrPrint)(nvObj_t *nv);	// required for PROGMEM access

typedef struct nvList {
	uint16_t magic_start;
	nvObj_t list[NV_LIST_LEN];			// list of nv objects, including space for a JSON header element
	uint16_t magic_end;
} nvList_t;

typedef struct cfgItem {
	char_t group[GROUP_LEN+1];			// group prefix (with NUL termination)
	char_t token[TOKEN_LEN+1];			// token - stripped of group prefix (w/NUL termination)
	uint8_t flags;						// operations flags - see defines below
	int8_t precision;					// decimal precision for display (JSON)
	fptrPrint print;					// print binding: aka void (*print)(nvObj_t *nv);
	fptrCmd get;						// GET binding aka uint8_t (*get)(nvObj_t *nv)
	fptrCmd set;						// SET binding aka uint8_t (*set)(nvObj_t *nv)
	float *target;						// target for writing config value
	float def_value;					// default value for config item
} cfgItem_t;

/**** static allocation and definitions ****/

extern nvStr_t nvStr;
extern nvList_t nvl;
extern const cfgItem_t cfgArray[];

//#define nv_header nv.list
#define nv_header (&nvl.list[0])
#define nv_body   (&nvl.list[1])

/**** Prototypes for generic config functions - see individual modules for application-specific functions  ****/

void config_init(void);
stat_t set_defaults(nvObj_t *nv);			// reset config to default values
void config_init_assertions(void);
stat_t config_test_assertions(void);

// main entry points for core access functions
stat_t nv_get(nvObj_t *nv);					// main entry point for get value
stat_t nv_set(nvObj_t *nv);					// main entry point for set value
void nv_print(nvObj_t *nv);					// main entry point for print value
stat_t nv_persist(nvObj_t *nv);				// main entry point for persistence

// helpers
uint8_t nv_get_type(nvObj_t *nv);
index_t nv_get_index(const char_t *group, const char_t *token);
index_t	nv_index_max(void);					// (see config_app.c)
uint8_t nv_index_is_single(index_t index);	// (see config_app.c)
uint8_t nv_index_is_group(index_t index);	// (see config_app.c)
uint8_t nv_index_lt_groups(index_t index);	// (see config_app.c)
uint8_t nv_group_is_prefixed(char_t *group);

// generic internal functions and accessors
stat_t set_nul(nvObj_t *nv);				// set nothing (no operation)
stat_t set_ui8(nvObj_t *nv);				// set uint8_t value
stat_t set_01(nvObj_t *nv);					// set a 0 or 1 value with validation
stat_t set_012(nvObj_t *nv);				// set a 0, 1 or 2 value with validation
stat_t set_0123(nvObj_t *nv);				// set a 0, 1, 2 or 3 value with validation
stat_t set_int(nvObj_t *nv);				// set uint32_t integer value
stat_t set_data(nvObj_t *nv);				// set uint32_t integer value blind cast
stat_t set_flt(nvObj_t *nv);				// set floating point value

stat_t get_nul(nvObj_t *nv);				// get null value type
stat_t get_ui8(nvObj_t *nv);				// get uint8_t value
stat_t get_int(nvObj_t *nv);				// get uint32_t integer value
stat_t get_data(nvObj_t *nv);				// get uint32_t integer value blind cast
stat_t get_flt(nvObj_t *nv);				// get floating point value

stat_t set_grp(nvObj_t *nv);				// set data for a group
stat_t get_grp(nvObj_t *nv);				// get data for a group

// nvObj and list functions
void nv_get_nvObj(nvObj_t *nv);
nvObj_t *nv_reset_nv(nvObj_t *nv);
nvObj_t *nv_reset_nv_list(void);
stat_t nv_copy_string(nvObj_t *nv, const char_t *src);
nvObj_t *nv_add_object(const char_t *token);
nvObj_t *nv_add_integer(const char_t *token, const uint32_t value);
nvObj_t *nv_add_float(const char_t *token, const float value);
nvObj_t *nv_add_string(const char_t *token, const char_t *string);
nvObj_t *nv_add_conditional_message(const char_t *string);
void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags);

// application specific helpers and functions (config_app.c)
stat_t set_flu(nvObj_t *nv);				// set floating point number with G20/G21 units conversion
void preprocess_float(nvObj_t *nv);			// pre-process float values for units and illegal values

// diagnostics
void nv_dump_nv(nvObj_t *nv);

/*********************************************************************************************
 **** PLEASE NOTICE THAT CONFIG_APP.H IS HERE ************************************************
 *********************************************************************************************/
#include "config_app.h"

#ifdef __cplusplus
}
#endif

#endif // End of include guard: CONFIG_H_ONCE
