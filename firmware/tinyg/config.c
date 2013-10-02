/*
 * config.c - application independent configuration handling 
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
/*
 *	See config.h for a Config system overview and a bunch of details.
 */

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "report.h"
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"
#include "text_parser.h"
#include "hardware.h"
#include "help.h"
#include "util.h"
#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

#ifdef __cplusplus
extern "C"{
#endif

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmdStr_t cmdStr;
cmdObj_t cmd_list[CMD_LIST_LEN];	// JSON header element

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/* Primary access points to cmd functions
 * These gatekeeper functions check index ranges so others don't have to
 *
 * cmd_set() 	- Write a value or invoke a function - operates on single valued elements or groups
 * cmd_get() 	- Build a cmdObj with the values from the target & return the value
 *			   	  Populate cmd body with single valued elements or groups (iterates)
 * cmd_print()	- Output a formatted string for the value.
 * cmd_persist()- persist value to NVM. Takes special cases into account
 */

stat_t cmd_set(cmdObj_t *cmd)
{
	if (cmd->index >= cmd_index_max()) { return (STAT_INTERNAL_RANGE_ERROR);}
	return (((fptrCmd)GET_TABLE_WORD(set))(cmd));
}

stat_t cmd_get(cmdObj_t *cmd)
{
	if (cmd->index >= cmd_index_max()) { return(STAT_INTERNAL_RANGE_ERROR);}
	return (((fptrCmd)GET_TABLE_WORD(get))(cmd));
}

void cmd_print(cmdObj_t *cmd)
{
	if (cmd->index >= cmd_index_max()) return;
	((fptrCmd)GET_TABLE_WORD(print))(cmd);
}

void cmd_persist(cmdObj_t *cmd)
{
#ifdef __DISABLE_PERSISTENCE	// cutout for faster simulation in test
	return;
#endif
	if (cmd_index_lt_groups(cmd->index) == false) return;
	if (GET_TABLE_BYTE(flags) & F_PERSIST) cmd_write_NVM_value(cmd);
}

/******************************************************************************
 * config_init()  - called once on hard reset
 *
 * Performs one of 2 actions:
 *	(1) if NVM is set up or out-of-rev load RAM and NVM with settings.h defaults
 *	(2) if NVM is set up and at current config version use NVM data for config
 *
 *	You can assume the cfg struct has been zeroed by a hard reset. 
 *	Do not clear it as the version and build numbers have already been set by tg_init()
 */
void config_init()
{
	cmdObj_t *cmd = cmd_reset_list();
	cmdStr.magic_start = MAGICNUM;
	cmdStr.magic_end = MAGICNUM;
	cfg.magic_start = MAGICNUM;
	cfg.magic_end = MAGICNUM;

	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode
	hw.nvm_base_addr = NVM_BASE_ADDR;
	hw.nvm_profile_base = hw.nvm_base_addr;
	cmd->index = 0;							// this will read the first record in NVM

	cmd_read_NVM_value(cmd);
	if (cmd->value != cs.fw_build) {
		cmd->value = true;					// case (1) NVM is not setup or not in revision
		set_defaults(cmd);	
	} else {								// case (2) NVM is setup and in revision
		rpt_print_loading_configs_message();
		for (cmd->index=0; cmd_index_is_single(cmd->index); cmd->index++) {
			if (GET_TABLE_BYTE(flags) & F_INITIALIZE) {
				strcpy_P(cmd->token, cfgArray[cmd->index].token);	// read the token from the array
				cmd_read_NVM_value(cmd);
				cmd_set(cmd);
			}
		}
		sr_init_status_report();
	}
}

/*
 * set_defaults() - reset NVM with default values for active profile
 */
stat_t set_defaults(cmdObj_t *cmd) 
{
	if (fp_FALSE(cmd->value)) {				// failsafe. Must set true or no action occurs
		help_defa(cmd);
		return (STAT_OK);
	}
	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode

	for (cmd->index=0; cmd_index_is_single(cmd->index); cmd->index++) {
		if (GET_TABLE_BYTE(flags) & F_INITIALIZE) {
			cmd->value = GET_TABLE_FLOAT(def_value);
			strcpy_P(cmd->token, cfgArray[cmd->index].token);
			cmd_set(cmd);
			cmd_persist(cmd);				// persist must occur when no other interrupts are firing
		}
	}
	rpt_print_initializing_message();		// don't start TX until all the NVM persistence is done
	sr_init_status_report();				// reset status reports
	return (STAT_OK);
}

/***** Generic Internal Functions *********************************************/

/* Generic gets()
 *	get_nul() - get nothing (returns STAT_NOOP)
 *	get_ui8() - get value as 8 bit uint8_t w/o unit conversion
 *	get_int() - get value as 32 bit integer w/o unit conversion
 *	get_flt() - get value as float w/o unit conversion
 *	get_format() - internal accessor for printf() format string
 */
stat_t get_nul(cmdObj_t *cmd) 
{ 
	cmd->objtype = TYPE_NULL;
	return (STAT_NOOP);
}

stat_t get_ui8(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint8_t *)GET_TABLE_WORD(target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_int(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint32_t *)GET_TABLE_WORD(target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_flt(cmdObj_t *cmd)
{
	cmd->value = *((float *)GET_TABLE_WORD(target));
	cmd->precision = (int8_t)GET_TABLE_WORD(precision);
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

/* Generic sets()
 *	set_nul() - set nothing (returns STAT_NOOP)
 *	set_ui8() - set value as 8 bit uint8_t value w/o unit conversion
 *	set_01()  - set a 0 or 1 uint8_t value with validation
 *	set_012() - set a 0, 1 or 2 uint8_t value with validation
 *	set_0123()- set a 0, 1, 2 or 3 uint8_t value with validation
 *	set_int() - set value as 32 bit integer w/o unit conversion
 *	set_flt() - set value as float w/o unit conversion
 */
stat_t set_nul(cmdObj_t *cmd) { return (STAT_NOOP);}

stat_t set_ui8(cmdObj_t *cmd)
{
	*((uint8_t *)GET_TABLE_WORD(target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_01(cmdObj_t *cmd)
{
	if (cmd->value > 1) return (STAT_INPUT_VALUE_UNSUPPORTED);	// if
	return (set_ui8(cmd));										// else
}

stat_t set_012(cmdObj_t *cmd)
{
	if (cmd->value > 2) return (STAT_INPUT_VALUE_UNSUPPORTED);	// if
	return (set_ui8(cmd));										// else
}

stat_t set_0123(cmdObj_t *cmd)
{
	if (cmd->value > 3) return (STAT_INPUT_VALUE_UNSUPPORTED);	// if
	return (set_ui8(cmd));										// else
}

stat_t set_int(cmdObj_t *cmd)
{
	*((uint32_t *)GET_TABLE_WORD(target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_flt(cmdObj_t *cmd)
{
	*((float *)GET_TABLE_WORD(target)) = cmd->value;
	cmd->precision = GET_TABLE_WORD(precision);
	cmd->objtype = TYPE_FLOAT;
	return(STAT_OK);
}

/***** GCODE SPECIFIC EXTENSIONS TO GENERIC FUNCTIONS *****/

/*
 * get_flu() - get floating point number with G20/G21 units conversion
 *
 * The number "getted" will be in internal canonical units (mm), which is  
 * returned in external units (inches or mm) 
 */

stat_t get_flu(cmdObj_t *cmd)
{
	get_flt(cmd);
	if (cm_get_units_mode(MODEL) == INCHES) cmd->value *= INCH_PER_MM;
	return (STAT_OK);
}

/*
 * set_flu() - set floating point number with G20/G21 units conversion
 *
 * The number "setted" will have been delivered in external units (inches or mm).
 * It is written to the target memory location in internal canonical units (mm),
 * but the original cmd->value is not changed so display works correctly.
 */

stat_t set_flu(cmdObj_t *cmd)
{
	float tmp_value = cmd->value;
	if (cm_get_units_mode(MODEL) == INCHES) tmp_value *= MM_PER_INCH; // convert to canonical units
	*((float *)GET_TABLE_WORD(target)) = tmp_value;
	cmd->precision = GET_TABLE_WORD(precision);
	cmd->objtype = TYPE_FLOAT;
	return(STAT_OK);
}

/******************************************************************************
 * Group operations
 *
 *	Group operations work on parent/child groups where the parent is one of:
 *	  axis group 			x,y,z,a,b,c
 *	  motor group			1,2,3,4
 *	  PWM group				p1
 *	  coordinate group		g54,g55,g56,g57,g58,g59,g92
 *	  system group			"sys" - a collection of otherwise unrelated variables
 *
 *	Text mode can only GET groups. For example:
 *	  $x					get all members of an axis group
 *	  $1					get all members of a motor group
 *	  $<grp>				get any named group from the above lists
 *
 *	In JSON groups are carried as parent / child objects & can get and set elements:
 *	  {"x":""}						get all X axis parameters
 *	  {"x":{"vm":""}}				get X axis velocity max 
 *	  {"x":{"vm":1000}}				set X axis velocity max
 *	  {"x":{"vm":"","fr":""}}		get X axis velocity max and feed rate 
 *	  {"x":{"vm":1000,"fr";900}}	set X axis velocity max and feed rate
 *	  {"x":{"am":1,"fr":800,....}}	set multiple or all X axis parameters
 */

/*
 * get_grp() - read data from axis, motor, system or other group
 *
 *	get_grp() is a group expansion function that expands the parent group and 
 *	returns the values of all the children in that group. It expects the first 
 *	cmdObj in the cmdBody to have a valid group name in the token field. This 
 *	first object will be set to a TYPE_PARENT. The group field is left nul -  
 *	as the group field refers to a parent group, which this group has none.
 *
 *	All subsequent cmdObjs in the body will be populated with their values.
 *	The token field will be populated as will the parent name in the group field. 
 *
 *	The sys group is an exception where the childern carry a blank group field, 
 *	even though the sys parent is labeled as a TYPE_PARENT.
 */

stat_t get_grp(cmdObj_t *cmd)
{
	char_t *parent_group = cmd->token;		// token in the parent cmd object is the group
	char_t group[CMD_GROUP_LEN+1];			// group string retrieved from cfgArray child
	cmd->objtype = TYPE_PARENT;				// make first object the parent 
	for (index_t i=0; cmd_index_is_single(i); i++) {
		strcpy_P(group, cfgArray[i].group);  // don't need strncpy as it's always terminated
		if (strcmp(parent_group, group) != 0) continue;
		(++cmd)->index = i;
		cmd_get_cmdObj(cmd);
	}
	return (STAT_OK);
}

/*
 * set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and 
 *	a setter. It iterates the group children and either gets the value or sets
 *	the value for each depending on the cmd->objtype.
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

stat_t set_grp(cmdObj_t *cmd)
{
	if (cfg.comm_mode == TEXT_MODE) return (STAT_UNRECOGNIZED_COMMAND);
	for (uint8_t i=0; i<CMD_MAX_OBJECTS; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if (cmd->objtype == TYPE_EMPTY) break;
		else if (cmd->objtype == TYPE_NULL)	// NULL means GET the value
			cmd_get(cmd);
		else {
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (STAT_OK);
}

/*
 * cmd_group_is_prefixed() - hack
 *
 *	This little function deals with the exception cases that some groups don't use
 *	the parent token as a prefix to the child elements; SR being a good example.
 */
uint8_t cmd_group_is_prefixed(char_t *group)
{
	if (strcmp("sr",group) == 0) return (false);
	if (strcmp("sys",group) == 0) return (false);
	return (true);
}

/******************************************************************************
 ***** cmdObj functions *******************************************************
 ******************************************************************************/

/******************************************************************************
 * cmdObj helper functions and other low-level cmd helpers
 */

/* cmd_get_index() - get index from mnenonic token + group
 *
 * cmd_get_index() is the most expensive routine in the whole config. It does a 
 * linear table scan of the PROGMEM strings, which of course could be further 
 * optimized with indexes or hashing.
 */
index_t cmd_get_index(const char_t *group, const char_t *token)
{
	char_t c;
	char_t str[CMD_TOKEN_LEN+1];
	strcpy(str, group);
	strcat(str, token);

	index_t index_max = cmd_index_max();

	for (index_t i=0; i < index_max; i++) {
		if ((c = (char_t)pgm_read_byte(&cfgArray[i].token[0])) != str[0]) {	// 1st character mismatch 
			continue;
		}
		if ((c = (char_t)pgm_read_byte(&cfgArray[i].token[1])) == NUL) {
			if (str[1] == NUL) return(i);									// one character match
		}
		if (c != str[1]) continue;											// 2nd character mismatch
		if ((c = (char_t)pgm_read_byte(&cfgArray[i].token[2])) == NUL) {
			if (str[2] == NUL) return(i);									// two character match
		}
		if (c != str[2]) continue;											// 3rd character mismatch
		if ((c = (char_t)pgm_read_byte(&cfgArray[i].token[3])) == NUL) {
			if (str[3] == NUL) return(i);									// three character match
		}
		if (c != str[3]) continue;											// 4th character mismatch
		if ((c = (char_t)pgm_read_byte(&cfgArray[i].token[4])) == NUL) {
			if (str[4] == NUL) return(i);									// four character match
		}
		if (c != str[4]) continue;											// 5th character mismatch
		return (i);															// five character match
	}
	return (NO_MATCH);
}

/* 
 * cmd_get_type() - returns command type as a CMD_TYPE enum
 */

uint8_t cmd_get_type(cmdObj_t *cmd)
{
	if (cmd->token[0] == NUL) return (CMD_TYPE_NULL);
	if (strcmp("gc", cmd->token) == 0) return (CMD_TYPE_GCODE);
	if (strcmp("sr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("qr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("msg",cmd->token) == 0) return (CMD_TYPE_MESSAGE);
	if (strcmp("err",cmd->token) == 0) return (CMD_TYPE_MESSAGE); 	// errors are reported as messages
	if (strcmp("n",  cmd->token) == 0) return (CMD_TYPE_LINENUM);
	return (CMD_TYPE_CONFIG);
}

/*
 * cmd_persist_offsets() - write any changed G54 (et al) offsets back to NVM
 */

stat_t cmd_persist_offsets(uint8_t flag)
{
	if (flag == true) {
		cmdObj_t cmd;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf(cmd.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				cmd.index = cmd_get_index((const char_t *)"", cmd.token);
				cmd.value = cm.offset[i][j];
				cmd_persist(&cmd);				// only writes changed values
			}
		}
	}
	return (STAT_OK);
}

/******************************************************************************
 * cmdObj low-level object and list operations
 * cmd_get_cmdObj()		- setup a cmd object by providing the index
 * cmd_reset_obj()		- quick clear for a new cmd object
 * cmd_reset_list()		- clear entire header, body and footer for a new use
 * cmd_copy_string()	- used to write a string to shared string storage and link it
 * cmd_add_object()		- write contents of parameter to  first free object in the body
 * cmd_add_integer()	- add an integer value to end of cmd body (Note 1)
 * cmd_add_float()		- add a floating point value to end of cmd body
 * cmd_add_string()		- add a string object to end of cmd body
 * cmd_add_conditional_message() - add a message to cmd body if messages are enabled
 *
 *	Note: Functions that return a cmd pointer point to the object that was modified or
 *	a NULL pointer if there was an error.
 *
 *	Note: Adding a really large integer (like a checksum value) may lose precision due
 *	to the cast to a float. Sometimes it's better to load an integer as a string if 
 *	all you want to do is display it.
 *
 *	Note: A trick is to cast all string constants for cmd_copy_string(), cmd_add_object(), 
 *	cmd_add_string() and cmd_add_conditional_message() to (const char_t *). Examples:
 *
 *		cmd_add_string((const char_t *)"msg", string);
 *
 *	On the AVR this will save a little static RAM. The "msg" string will occupy flash 
 *	as an initializer and be instantiated in stack RAM when the function executes. 
 *	On the ARM (however) this will put the string into flash and skip RAM allocation.
 */

void cmd_get_cmdObj(cmdObj_t *cmd)
{
	if (cmd->index >= cmd_index_max()) { return; }	// sanity

	index_t tmp = cmd->index;
	cmd_reset_obj(cmd);
	cmd->index = tmp;

	strcpy_P(cmd->token, cfgArray[cmd->index].token); // token field is always terminated
	strcpy_P(cmd->group, cfgArray[cmd->index].group); // group field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (cmd->group[0] != NUL) {
		if (GET_TABLE_BYTE(flags) & F_NOSTRIP) {
			cmd->group[0] = NUL;
		} else {
			strcpy(cmd->token, &cmd->token[strlen(cmd->group)]); // strip group from the token
		}
	}
	((fptrCmd)GET_TABLE_WORD(get))(cmd);	// populate the value
}
 
cmdObj_t *cmd_reset_obj(cmdObj_t *cmd)		// clear a single cmdObj structure
{
	cmd->objtype = TYPE_EMPTY;				// selective clear is much faster than calling memset
	cmd->index = 0;
	cmd->value = 0;
	cmd->precision = 0;
	cmd->token[0] = NUL;
	cmd->group[0] = NUL;
	cmd->stringp = NULL;

	if (cmd->pv == NULL) { 					// set depth correctly
		cmd->depth = 0;
	} else {
		if (cmd->pv->objtype == TYPE_PARENT) { 
			cmd->depth = cmd->pv->depth + 1;
		} else {
			cmd->depth = cmd->pv->depth;
		}
	}
	return (cmd);							// return pointer to cmd as a convenience to callers
}

cmdObj_t *cmd_reset_list()					// clear the header and response body
{
	cmdStr.wp = 0;							// reset the shared string
	cmdObj_t *cmd = cmd_list;				// set up linked list and initialize elements	
	for (uint8_t i=0; i<CMD_LIST_LEN; i++, cmd++) {
		cmd->pv = (cmd-1);					// the ends are bogus & corrected later
		cmd->nx = (cmd+1);
		cmd->index = 0;
		cmd->depth = 1;						// header and footer are corrected later
		cmd->objtype = TYPE_EMPTY;
		cmd->token[0] = NUL;
	}
	(--cmd)->nx = NULL;
	cmd = cmd_list;							// setup response header element ('r')
	cmd->pv = NULL;
	cmd->depth = 0;
	cmd->objtype = TYPE_PARENT;
	strcpy(cmd->token, "r");
	return (cmd_body);						// this is a convenience for calling routines
}

stat_t cmd_copy_string(cmdObj_t *cmd, const char_t *src)
{
	if ((cmdStr.wp + strlen(src)) > CMD_SHARED_STRING_LEN) { return (STAT_BUFFER_FULL);}
	char_t *dst = &cmdStr.string[cmdStr.wp];
	strcpy(dst, src);						// copy string to current head position
	cmdStr.wp += strlen(src)+1;				// advance head for next string
	cmd->stringp = (char_t (*)[])dst;
	return (STAT_OK);
}

/* UNUSED
stat_t cmd_copy_string_P(cmdObj_t *cmd, const char_t *src_P)
{
	char_t buf[CMD_SHARED_STRING_LEN];
	strncpy_P(buf, src_P, CMD_SHARED_STRING_LEN);
	return (cmd_copy_string(cmd, buf));
}
*/

cmdObj_t *cmd_add_object(const char_t *token)  // add an object to the body using a token
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index((const char_t *)"",token)) == NO_MATCH) { return (NULL);}
		cmd_get_cmdObj(cmd);				// populate the object from the index
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_integer(const char_t *token, const uint32_t value)// add an integer object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = (float) value;
		cmd->objtype = TYPE_INTEGER;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_float(const char_t *token, const float value)	// add a float object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL);		// not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = value;
		cmd->objtype = TYPE_FLOAT;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_string(const char_t *token, const char_t *string) // add a string object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL);		// not supposed to find a NULL; here for safety
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		if (cmd_copy_string(cmd, string) != STAT_OK) { return (NULL);}
		cmd->index = cmd_get_index((const char_t *)"", cmd->token);
		cmd->objtype = TYPE_STRING;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_conditional_message(const char_t *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (js.echo_json_messages != true)) { return (NULL);}
	return(cmd_add_string((const char_t *)"msg", string));
}

/* UNUSED
cmdObj_t *cmd_add_string_P(const char_t *token, const char_t *string)
{
	char_t message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, string);
	return(cmd_add_string(token, message));
}
*/
/*
cmdObj_t *cmd_add_conditional_message_P(const char_t *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (js.echo_json_messages != true)) { return (NULL);}
//	return(cmd_add_string_P("msg", string));
	return(cmd_add_string((const char_t *)"msg", string));
}
*/

/**** cmd_print_list() - print cmd_array as JSON or text **********************
 *
 * 	Generate and print the JSON and text mode output strings. Use this function 
 *	for all text and JSON output that wants to be in a response header. 
 *	Don't just printf stuff.
 *
 *	Inputs:
 *	  json_flags = JSON_OBJECT_FORMAT - print just the body w/o header or footer
 *	  json_flags = JSON_RESPONSE_FORMAT - print a full "r" object with footer
 *
 *	  text_flags = TEXT_INLINE_PAIRS - print text as name/value pairs on a single line
 *	  text_flags = TEXT_INLINE_VALUES - print text as comma separated values on a single line
 *	  text_flags = TEXT_MULTILINE_FORMATTED - print text one value per line with formatting string
 */

void cmd_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cfg.comm_mode == JSON_MODE) {
		json_print_list(status, json_flags);
	} else {
		text_print_list(status, text_flags);
	}
}

/************************************************************************************
 ***** EEPROM access functions ******************************************************
 ************************************************************************************
 * cmd_read_NVM_value()	 - return value (as float) by index
 * cmd_write_NVM_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */
stat_t cmd_read_NVM_value(cmdObj_t *cmd)
{
#ifdef __AVR
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	uint16_t nvm_address = hw.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
#endif // __AVR
#ifdef __ARM
	//+++++ No ARM persistence yet
#endif // __ARM
	return (STAT_OK);
}

stat_t cmd_write_NVM_value(cmdObj_t *cmd)
{
#ifdef __AVR
	if (cm.cycle_state != CYCLE_OFF) return (STAT_FILE_NOT_OPEN);	// can't write when machine is moving
	float tmp = cmd->value;
	ritorno(cmd_read_NVM_value(cmd));
	if (cmd->value != tmp) {		// catches the isnan() case as well
		cmd->value = tmp;
		int8_t nvm_byte_array[NVM_VALUE_LEN];
		memcpy(&nvm_byte_array, &tmp, NVM_VALUE_LEN);
		uint16_t nvm_address = hw.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
#endif // __AVR
#ifdef __ARM
	//+++++ No ARM persistence yet
#endif // __ARM
	return (STAT_OK);
}

/****************************************************************************
 ***** Config Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_CONFIG

#define NVMwr(i,v) { cmd.index=i; cmd.value=v; cmd_write_NVM_value(&cmd);}
#define NVMrd(i)   { cmd.index=i; cmd_read_NVM_value(&cmd); printf("%f\n", (char *)cmd.value);}

void cfg_unit_tests()
{

// NVM tests
/*	cmdObj_t cmd;
	NVMwr(0, 329.01)
	NVMwr(1, 111.01)
	NVMwr(2, 222.02)
	NVMwr(3, 333.03)
	NVMwr(4, 444.04)
	NVMwr(10, 10.10)
	NVMwr(100, 100.100)
	NVMwr(479, 479.479)

	NVMrd(0)
	NVMrd(1)
	NVMrd(2)
	NVMrd(3)
	NVMrd(4)
	NVMrd(10)
	NVMrd(100)
	NVMrd(479)
*/

// config table tests

	index_t i;
//	float val;

//	print_configs("$", NUL);					// no filter (show all)
//	print_configs("$", 'g');					// filter for general parameters
//	print_configs("$", '1');					// filter for motor 1
//	print_configs("$", 'x');					// filter for x axis

	i = cmd_get_index((const char_t *)"fb");
	i = cmd_get_index((const char_t *)"xfr");
	i = cmd_get_index((const char_t *)"g54");

//	i = get_pos_axis(55);
//	i = get_pos_axis(73);
//	i = get_pos_axis(93);
//	i = get_pos_axis(113);

/*
	for (i=0; i<CMD_MAX_INDEX; i++) {

		cmd_get(&cmd);

		cmd.value = 42;
		cmd_set(&cmd);

		val = get_flt_value(i);
		cmd_get_token(i, cmd.token);

//		get_friendly(i, string);
		get_format(i, cmd.vstring);
		get_axis(i);							// uncomment main function to test
		get_motor(i);
		cmd_set(i, &cmd);
		cmd_print(i);
	}

	_parse_config_string("$1po 1", &c);			// returns a number
	_parse_config_string("XFR=1200", &c);		// returns a number
	_parse_config_string("YFR 1300", &c);		// returns a number
	_parse_config_string("zfr	1400", &c);		// returns a number
	_parse_config_string("afr", &c);			// returns a null
	_parse_config_string("Bfr   ", &c);			// returns a null
	_parse_config_string("cfr=wordy", &c);		// returns a null

//	i = cfgget_config_index("gc");
//	i = cfgget_config_index("gcode");
//	i = cfgget_config_index("c_axis_mode");
//	i = cfgget_config_index("AINT_NOBODY_HOME");
	i = cfgget_config_index("firmware_version");
*/
}

#endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
