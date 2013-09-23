/*
 * config_app.h - application-specific part of configuration data
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CONFIG_APP_H_ONCE
#define CONFIG_APP_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

//#include <stdbool.h>

/**** Config System Overview and Usage ***
 *
 *	--- Config objects and the config list ---
 *
 *	The config system provides a structured way to access and set configuration variables.
 *	It also provides a way to get an arbitrary variable for reporting. Config operates
 *	as a collection of "objects" (OK, so they are not really objects) that encapsulate
 *	each variable. The objects are collected into a list (the body), which also may have  
 *	header and footer objects. This way the internals don't care about how the variable
 *	is represented or communicated externally as all operations occur on the cmdObj list. 
 *	The list is populated by the text_parser or the JSON_parser depending on the mode.
 *	The lists are also used for responses and are read out (printed) by a text-mode or
 *	JSON serialization function.
 */
/*	--- Config variables, tables and strings ---
 *
 *	Each configuration value is identified by a short mnemonic string (token). The token 
 *	is resolved to an index into the cfgArray which is an array of structures with the 
 *	static assignments for each variable. The cfgArray contains typed data in program 
 *	memory (PROGMEM).
 * 
 *	Each cfgItem has:
 *	 - group string identifying what group the variable is part of; or "" if no group
 *	 - token string - the token for that variable - pre-pended with the group (if present)
 *	 - operations flags - e.g. if the value should be initialized and/or persisted to NVM
 *	 - pointer to a formatted print string also in program memory (Used only for text mode)
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
 *	 - Add a formatting string to fmt_XXX strings. Not needed if there is no text-mode print function
 *	   of you are using one of the generic print strings.
 * 
 *	 - Create a new record in cfgArray[]. Use existing ones for examples. You can usually use existing
 *	   functions for get and set; or create a new one if you need a specialized function.
 *
 *	   The ordering of group displays is set by the order of items in cfgArray. None of the other 
 *	   orders matter but are generally kept sequenced for easier reading and code maintenance. Also,
 *	   Items earlier in the array will resolve token searches faster than ones later in the array.
 *
 *	   Note that matching will occur from the most specific to the least specific, meaning that
 *	   if tokens overlap the longer one should be earlier in the array: "gco" should precede "gc".
 */
/*  --- Rules, guidelines and random stuff
 *
 *	It's the responsibility of the object creator to set the index. Downstream functions
 *	all expect a valid index. Set the index by calling cmd_get_index(). This also validates
 *	the token and group if no lookup exists.
 */

/**** cmdObj lists ****
 *
 * 	Commands and groups of commands are processed internally a doubly linked list of
 *	cmdObj_t structures. This isolates the command and config internals from the 
 *	details of communications, parsing and display in text mode and JSON mode.
 *
 *	The first element of the list is designated the response header element ("r") 
 *	but the list can also be serialized as a simple object by skipping over the header
 *
 *	To use the cmd list first reset it by calling cmd_reset_list(). This initializes
 *	the header, marks the the objects as TYPE_EMPTY (-1), resets the shared string, 
 *	relinks all objects with NX and PV pointers, and makes the last element the 
 *	terminating element by setting its NX pointer to NULL. The terminating element 
 *	may carry data, and will be processed.
 *
 *	When you use the list you can terminate your own last element, or just leave the 
 *	EMPTY elements to be skipped over during output serialization.
 * 
 * 	We don't use recursion so parent/child nesting relationships are captured in a 
 *	'depth' variable, This must remain consistent if the curlies are to work out. 
 *	In general you should not have to track depth explicitly if you use cmd_reset_list()
 *	or the accessor functions like cmd_add_integer() or cmd_add_message(). 
 *	If you see problems with curlies check the depth values in the lists.
 *
 *	Use the cmd_print_list() dispatcher for all JSON and text output. Do not simply 
 *	run through printf.
 */
/*	Token and Group Fields
 * 
 *	The cmdObject struct (cmdObj_t) has strict rules on the use of the token and group fields.
 *	The follwing forms are legal which support the use cases listed:
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

 /***********************************************************************************
  **** APPLICATION_SPECIFIC DEFINITIONS AND SETTINGS ********************************
  ***********************************************************************************/

enum cmdType {						// classification of commands
	CMD_TYPE_NULL = 0,
	CMD_TYPE_CONFIG,				// configuration commands
	CMD_TYPE_GCODE,					// gcode
	CMD_TYPE_REPORT,				// SR, QR and any other report
	CMD_TYPE_MESSAGE,				// cmd object carries a message
	CMD_TYPE_LINENUM				// cmd object carries a gcode line number
};

/*** Print format strings ***/

const char_t PROGMEM WEAK fmt_nul[] = "";
const char_t PROGMEM WEAK fmt_ui8[] = "%d\n";	// generic format for ui8s
const char_t PROGMEM WEAK fmt_flt[] = "%f\n";	// generic format for floats
const char_t PROGMEM WEAK fmt_str[] = "%s\n";	// generic format for string message (with no formatting)
/*
// System group and ungrouped formatting strings
const char_t PROGMEM WEAK fmt_fb[] = "[fb]  firmware build%18.2f\n";
const char_t PROGMEM WEAK fmt_fv[] = "[fv]  firmware version%16.2f\n";
const char_t PROGMEM WEAK fmt_hv[] = "[hv]  hardware version%16.2f\n";
const char_t PROGMEM WEAK fmt_id[] = "[id]  TinyG ID%30s\n";

const char_t PROGMEM WEAK fmt_ja[] = "[ja]  junction acceleration%8.0f%S\n";
const char_t PROGMEM WEAK fmt_ct[] = "[ct]  chordal tolerance%16.3f%S\n";

const char_t PROGMEM WEAK fmt_ml[] = "[ml]  min line segment%17.3f%S\n";
const char_t PROGMEM WEAK fmt_ma[] = "[ma]  min arc segment%18.3f%S\n";
const char_t PROGMEM WEAK fmt_ms[] = "[ms]  min segment time%13.0f uSec\n";

const char_t PROGMEM WEAK fmt_st[] = "[st]  switch type%18d [0=NO,1=NC]\n";
const char_t PROGMEM WEAK fmt_si[] = "[si]  status interval%14.0f ms\n";

//const char_t PROGMEM WEAK fmt_ic[] = "[ic]  ignore CR or LF on RX%8d [0=off,1=CR,2=LF]\n";
const char_t PROGMEM WEAK fmt_ec[] = "[ec]  expand LF to CRLF on TX%6d [0=off,1=on]\n";
const char_t PROGMEM WEAK fmt_ee[] = "[ee]  enable echo%18d [0=off,1=on]\n";
const char_t PROGMEM WEAK fmt_ex[] = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";

const char_t PROGMEM WEAK fmt_fs[] = "[fs]  footer style%17d [0=new,1=old]\n";
const char_t PROGMEM WEAK fmt_ej[] = "[ej]  enable json mode%13d [0=text,1=JSON]\n";
const char_t PROGMEM WEAK fmt_jv[] = "[jv]  json verbosity%15d [0=silent,1=footer,2=messages,3=configs,4=linenum,5=verbose]\n";
const char_t PROGMEM WEAK fmt_tv[] = "[tv]  text verbosity%15d [0=silent,1=verbose]\n";
const char_t PROGMEM WEAK fmt_sv[] = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";
const char_t PROGMEM WEAK fmt_qv[] = "[qv]  queue report verbosity%7d [0=off,1=filtered,2=verbose]\n";
const char_t PROGMEM WEAK fmt_baud[] = "[baud] USB baud rate%15d [1=9600,2=19200,3=38400,4=57600,5=115200,6=230400]\n";
const char_t PROGMEM WEAK fmt_net[] = "[net]  network mode%16d [0=master]\n";

const char_t PROGMEM WEAK fmt_qr[] = "qr:%d\n";
const char_t PROGMEM WEAK fmt_rx[] = "rx:%d\n";

const char_t PROGMEM WEAK fmt_mt[] = "[mt]  motor idle timeout%14.2f Sec\n";
const char_t PROGMEM WEAK fmt_me[] = "motors energized\n";
const char_t PROGMEM WEAK fmt_md[] = "motors de-energized\n";

// Gcode model values for reporting purposes
const char_t PROGMEM WEAK fmt_vel[]  = "Velocity:%17.3f%S/min\n";
const char_t PROGMEM WEAK fmt_line[] = "Line number:%10.0f\n";
const char_t PROGMEM WEAK fmt_feed[] = "Feed rate:%16.3f%S/min\n";
const char_t PROGMEM WEAK fmt_stat[] = "Machine state:       %s\n"; // combined machine state
const char_t PROGMEM WEAK fmt_macs[] = "Raw machine state:   %s\n"; // raw machine state
const char_t PROGMEM WEAK fmt_cycs[] = "Cycle state:         %s\n";
const char_t PROGMEM WEAK fmt_mots[] = "Motion state:        %s\n";
const char_t PROGMEM WEAK fmt_hold[] = "Feedhold state:      %s\n";
const char_t PROGMEM WEAK fmt_home[] = "Homing state:        %s\n";
const char_t PROGMEM WEAK fmt_unit[] = "Units:               %s\n"; // units mode as ASCII string
const char_t PROGMEM WEAK fmt_coor[] = "Coordinate system:   %s\n";
const char_t PROGMEM WEAK fmt_momo[] = "Motion mode:         %s\n";
const char_t PROGMEM WEAK fmt_plan[] = "Plane:               %s\n";
const char_t PROGMEM WEAK fmt_path[] = "Path Mode:           %s\n";
const char_t PROGMEM WEAK fmt_dist[] = "Distance mode:       %s\n";
const char_t PROGMEM WEAK fmt_frmo[] = "Feed rate mode:      %s\n";
const char_t PROGMEM WEAK fmt_tool[] = "Tool number          %d\n";
//const char_t PROGMEM WEAK fmt_ss[]   = "Switch %s state:     %d\n";

const char_t PROGMEM WEAK fmt_pos[]  = "%c position:%15.3f%S\n";
const char_t PROGMEM WEAK fmt_mpos[] = "%c machine posn:%11.3f%S\n";
const char_t PROGMEM WEAK fmt_ofs[]  = "%c work offset:%12.3f%S\n";
const char_t PROGMEM WEAK fmt_hom[]  = "%c axis homing state:%2.0f\n";

// Motor print formatting strings
const char_t PROGMEM WEAK fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
const char_t PROGMEM WEAK fmt_0sa[] = "[%s%s] m%s step angle%20.3f%S\n";
const char_t PROGMEM WEAK fmt_0tr[] = "[%s%s] m%s travel per revolution%9.3f%S\n";
const char_t PROGMEM WEAK fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8]\n";
const char_t PROGMEM WEAK fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
const char_t PROGMEM WEAK fmt_0pm[] = "[%s%s] m%s power management%10d [0=remain powered,1=power down when idle]\n";

// Axis print formatting strings
const char_t PROGMEM WEAK fmt_Xam[] = "[%s%s] %s axis mode%18d %S\n";
const char_t PROGMEM WEAK fmt_Xfr[] = "[%s%s] %s feedrate maximum%15.3f%S/min\n";
const char_t PROGMEM WEAK fmt_Xvm[] = "[%s%s] %s velocity maximum%15.3f%S/min\n";
const char_t PROGMEM WEAK fmt_Xtm[] = "[%s%s] %s travel maximum%17.3f%S\n";
const char_t PROGMEM WEAK fmt_Xjm[] = "[%s%s] %s jerk maximum%15.0f%S/min^3 * 1 million\n";
const char_t PROGMEM WEAK fmt_Xjh[] = "[%s%s] %s jerk homing%16.0f%S/min^3 * 1 million\n";
const char_t PROGMEM WEAK fmt_Xjd[] = "[%s%s] %s junction deviation%14.4f%S (larger is faster)\n";
const char_t PROGMEM WEAK fmt_Xra[] = "[%s%s] %s radius value%20.4f%S\n";
const char_t PROGMEM WEAK fmt_Xsn[] = "[%s%s] %s switch min%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
const char_t PROGMEM WEAK fmt_Xsx[] = "[%s%s] %s switch max%17d [0=off,1=homing,2=limit,3=limit+homing]\n";
const char_t PROGMEM WEAK fmt_Xsv[] = "[%s%s] %s search velocity%16.3f%S/min\n";
const char_t PROGMEM WEAK fmt_Xlv[] = "[%s%s] %s latch velocity%17.3f%S/min\n";
const char_t PROGMEM WEAK fmt_Xlb[] = "[%s%s] %s latch backoff%18.3f%S\n";
const char_t PROGMEM WEAK fmt_Xzb[] = "[%s%s] %s zero backoff%19.3f%S\n";

// PWM strings
const char_t PROGMEM WEAK fmt_p1frq[] = "[p1frq] pwm frequency   %15.3f Hz\n";
const char_t PROGMEM WEAK fmt_p1csl[] = "[p1csl] pwm cw speed lo %15.3f RPM\n";
const char_t PROGMEM WEAK fmt_p1csh[] = "[p1csh] pwm cw speed hi %15.3f RPM\n";
const char_t PROGMEM WEAK fmt_p1cpl[] = "[p1cpl] pwm cw phase lo %15.3f [0..1]\n";
const char_t PROGMEM WEAK fmt_p1cph[] = "[p1cph] pwm cw phase hi %15.3f [0..1]\n";
const char_t PROGMEM WEAK fmt_p1wsl[] = "[p1wsl] pwm ccw speed lo%15.3f RPM\n";
const char_t PROGMEM WEAK fmt_p1wsh[] = "[p1wsh] pwm ccw speed hi%15.3f RPM\n";
const char_t PROGMEM WEAK fmt_p1wpl[] = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
const char_t PROGMEM WEAK fmt_p1wph[] = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
const char_t PROGMEM WEAK fmt_p1pof[] = "[p1pof] pwm phase off   %15.3f [0..1]\n";

// Coordinate system offset print formatting strings
const char_t PROGMEM WEAK fmt_cofs[] = "[%s%s] %s %s offset%20.3f%S\n";
const char_t PROGMEM WEAK fmt_cloc[] = "[%s%s] %s %s location%18.3f%S\n";

// Gcode model power-on reset default values
const char_t PROGMEM WEAK fmt_gpl[] = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
const char_t PROGMEM WEAK fmt_gun[] = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
const char_t PROGMEM WEAK fmt_gco[] = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
const char_t PROGMEM WEAK fmt_gpa[] = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
const char_t PROGMEM WEAK fmt_gdi[] = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";
*/
/***********************************************************************************
 **** APPLICATION_SPECIFIC CONFIG STRUCTURE(S) *************************************
 ***********************************************************************************/

typedef struct cfgParameters {
	uint16_t magic_start;			// magic number to test memory integrity

	// communications settings
	uint8_t usb_baud_rate;			// see xio_usart.h for XIO_BAUD values
	uint8_t usb_baud_flag;			// technically this belongs in the controller singleton

	uint8_t comm_mode;				// TG_TEXT_MODE or TG_JSON_MODE
//	uint8_t ignore_crlf;			// ignore CR or LF on RX --- these 4 are shadow settings for XIO cntrl bits
	uint8_t enable_cr;				// enable CR in CRFL expansion on TX
	uint8_t enable_echo;			// enable text-mode echo
	uint8_t enable_flow_control;	// enable XON/XOFF or RTS/CTS flow control

	// text mode settings
	uint8_t text_verbosity;			// see enum in this file for settings

	// Non-volatile RAM
	uint16_t nvm_base_addr;			// NVM base address
	uint16_t nvm_profile_base;		// NVM base address of current profile

	uint16_t magic_end;
} cfgParameters_t;
extern cfgParameters_t cfg;


/***********************************************************************************
 **** EXPOSED APPLICATION SPECIFIC FUNCTIONS ***************************************
 ***********************************************************************************/

stat_t set_flu(cmdObj_t *cmd);
stat_t get_flu(cmdObj_t *cmd);

void print_lin(cmdObj_t *cmd);
void print_rot(cmdObj_t *cmd);

char_t get_axis_char(int8_t axis);
int8_t get_motor(const index_t i);
int8_t get_pos_axis(const index_t i);

stat_t set_baud_callback(void);



#ifdef __cplusplus
}
#endif

#endif //End of include guard: CONFIG_APP_H_ONCE
