/*
 * config.c - application independent configuration handling 
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
/*
 *	See config.h for a Config system overview and a bunch of details.
 */
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>			// precursor for xio.h
#include <avr/pgmspace.h>	// precursor for xio.h

#include "tinyg.h"			// config reaches into almost everything
#include "util.h"
#include "config.h"
#include "report.h"
#include "settings.h"
#include "controller.h"
#include "canonical_machine.h"
#include "gcode_parser.h"
#include "json_parser.h"
#include "text_parser.h"
#include "planner.h"
#include "stepper.h"
#include "gpio.h"
#include "test.h"
#include "help.h"
#include "system.h"
#include "network.h"
#include "xio/xio.h"
#include "xmega/xmega_eeprom.h"

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmdStr_t cmdStr;
cmdObj_t cmd_list[CMD_LIST_LEN];	// JSON header element

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

//static void _do_group_list(cmdObj_t *cmd, char list[][CMD_TOKEN_LEN+1]); // helper to print multiple groups in a list

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

#define ASSERT_CMD_INDEX(a) if (cmd->index >= CMD_INDEX_MAX) return (a);

stat_t cmd_set(cmdObj_t *cmd)
{
//	ASSERT_CMD_INDEX(STAT_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].set)))(cmd));
}

stat_t cmd_get(cmdObj_t *cmd)
{
//	ASSERT_CMD_INDEX(STAT_UNRECOGNIZED_COMMAND);
	return (((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd));
}

void cmd_print(cmdObj_t *cmd)
{
//	if (cmd->index >= CMD_INDEX_MAX) return;
	((fptrPrint)(pgm_read_word(&cfgArray[cmd->index].print)))(cmd);
}

void cmd_persist(cmdObj_t *cmd)
{
#ifdef __DISABLE_PERSISTENCE	// cutout for faster simulation in test
	return;
#endif
	if (cmd_index_lt_groups(cmd->index) == false) return;
	if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_PERSIST) {
		cmd_write_NVM_value(cmd);
	}
}

/******************************************************************************
 * cfg_init() - called once on hard reset
 * set_defa() - reset NVM with default values for active profile
 *
 * Performs one of 2 actions:
 *	(1) if NVM is set up or out-of-rev load RAM and NVM with settings.h defaults
 *	(2) if NVM is set up and at current config version use NVM data for config
 *
 *	You can assume the cfg struct has been zeroed by a hard reset. 
 *	Do not clear it as the version and build numbers have already been set by tg_init()
 */
void cfg_init()
{
	cmdObj_t *cmd = cmd_reset_list();
	cmdStr.magic_start = MAGICNUM;
	cmdStr.magic_end = MAGICNUM;
	cfg.magic_start = MAGICNUM;
	cfg.magic_end = MAGICNUM;

	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode
	cfg.nvm_base_addr = NVM_BASE_ADDR;
	cfg.nvm_profile_base = cfg.nvm_base_addr;
	cmd->index = 0;							// this will read the first record in NVM

	cmd_read_NVM_value(cmd);
	if (cmd->value != cs.fw_build) {
		cmd->value = true;					// case (1) NVM is not setup or not in revision
		set_defa(cmd);	
	} else {								// case (2) NVM is setup and in revision
		rpt_print_loading_configs_message();
		for (cmd->index=0; cmd_index_is_single(cmd->index); cmd->index++) {
			if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_INITIALIZE) {
				strcpy_P(cmd->token, cfgArray[cmd->index].token);	// read the token from the array
				cmd_read_NVM_value(cmd);
				cmd_set(cmd);
			}
		}
		rpt_init_status_report();
	}
}

// set_defa() is both a helper and called directly from the $defa=1 command
stat_t set_defa(cmdObj_t *cmd) 
{
	if (cmd->value != true) {				// failsafe. Must set true or no action occurs
		print_defaults_help(cmd);
		return (STAT_OK);
	}
	cm_set_units_mode(MILLIMETERS);			// must do inits in MM mode

	for (cmd->index=0; cmd_index_is_single(cmd->index); cmd->index++) {
		if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_INITIALIZE) {
			cmd->value = (float)pgm_read_float(&cfgArray[cmd->index].def_value);
			strcpy_P(cmd->token, cfgArray[cmd->index].token);
			cmd_set(cmd);
			cmd_persist(cmd);				// persist must occur when no other interrupts are firing
		}
	}
	rpt_print_initializing_message();		// don't start TX until all the NVM persistence is done
	rpt_init_status_report();				// reset status reports
	return (STAT_OK);
}

/*
 * cfg_cycle_check() - check if in a machining cycle and toss command if so
 */

stat_t cfg_cycle_check(void)
{
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
	cmd->value = (float)*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_int(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->objtype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_flt(cmdObj_t *cmd)
{
	cmd->value = *((float *)pgm_read_word(&cfgArray[cmd->index].target));
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT;
	return (STAT_OK);
}

char_t *get_format(const index_t i, char_t *format)
{
	strncpy_P(format, (PGM_P)pgm_read_word(&cfgArray[i].format), CMD_FORMAT_LEN);
	return (format);
}

/* ARM version: REPLACED BY A MACRO - See config.h
char *get_format(const index_t index)
{
	return ((char *)cfgArray[index].format);
}
*/

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
	*((uint8_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_01(cmdObj_t *cmd)
{
	if (cmd->value > 1) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (set_ui8(cmd));
	}
}

stat_t set_012(cmdObj_t *cmd)
{
	if (cmd->value > 2) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (set_ui8(cmd));
	}
}

stat_t set_0123(cmdObj_t *cmd)
{
	if (cmd->value > 3) { 
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (set_ui8(cmd));
	}
}

stat_t set_int(cmdObj_t *cmd)
{
	*((uint32_t *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->objtype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_flt(cmdObj_t *cmd)
{
	*((float *)pgm_read_word(&cfgArray[cmd->index].target)) = cmd->value;
	cmd->precision = (int8_t)pgm_read_word(&cfgArray[cmd->index].precision);
	cmd->objtype = TYPE_FLOAT;
	return(STAT_OK);
}

/* Generic prints()
 *	print_nul() - print nothing
 *	print_str() - print string value
 *	print_ui8() - print uint8_t value w/no units or unit conversion
 *	print_int() - print integer value w/no units or unit conversion
 *	print_flt() - print float value w/no units or unit conversion
 */
void print_nul(cmdObj_t *cmd) {}

void print_str(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), *cmd->stringp);
}

void print_ui8(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), (uint8_t)cmd->value);
}

void print_int(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), (uint32_t)cmd->value);
}

void print_flt(cmdObj_t *cmd)
{
	cmd_get(cmd);
	char_t format[CMD_FORMAT_LEN+1];
	fprintf(stderr, get_format(cmd->index, format), cmd->value);
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
 *	This little function deals with the fact that some groups don't use the parent 
 *	token as a prefix to the child elements; SR being a good example.
 */
uint8_t cmd_group_is_prefixed(char_t *group)
{
	if (strstr("sr,sys",group) != NULL) {	// you can extend like this: "sr,sys,xyzzy"
		return (false);
	}
	return (true);
}

/******************************************************************************
 ***** cmdObj functions *******************************************************
 ******************************************************************************/

/******************************************************************************
 * cmdObj helper functions and other low-level cmd helpers
 * cmd_get_type()		 - returns command type as a CMD_TYPE enum
 * cmd_persist_offsets() - write any changed G54 (et al) offsets back to NVM
 * cmd_get_index() 		 - get index from mnenonic token + group
 */
uint8_t cmd_get_type(cmdObj_t *cmd)
{
	if (cmd->token[0] == NUL) return (CMD_TYPE_NULL);
	if (strcmp("gc", cmd->token) == 0) return (CMD_TYPE_GCODE);
	if (strcmp("sr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("qr", cmd->token) == 0) return (CMD_TYPE_REPORT);
	if (strcmp("msg",cmd->token) == 0) return (CMD_TYPE_MESSAGE);
	if (strcmp("n",  cmd->token) == 0) return (CMD_TYPE_LINENUM);
	return (CMD_TYPE_CONFIG);
}

stat_t cmd_persist_offsets(uint8_t flag)
{
	if (flag == true) {
		cmdObj_t cmd;
		for (uint8_t i=1; i<=COORDS; i++) {
			for (uint8_t j=0; j<AXES; j++) {
				sprintf(cmd.token, "g%2d%c", 53+i, ("xyzabc")[j]);
				cmd.index = cmd_get_index("", cmd.token);
				cmd.value = cfg.offset[i][j];
				cmd_persist(&cmd);				// only writes changed values
			}
		}
	}
	return (STAT_OK);
}

/* 
 * cmd_get_index() is the most expensive routine in the whole config. It does a linear table scan 
 * of the PROGMEM strings, which of course could be further optimized with indexes or hashing.
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

/******************************************************************************
 * cmdObj low-level object and list operations
 * cmd_get_cmdObj()		- setup a cmd object by providing the index
 * cmd_reset_obj()		- quick clear for a new cmd object
 * cmd_reset_list()		- clear entire header, body and footer for a new use
 * cmd_copy_string()	- used to write a string to shared string storage and link it
 * cmd_copy_string_P()	- same, but for progmem string sources
 * cmd_add_object()		- write contents of parameter to  first free object in the body
 * cmd_add_integer()	- add an integer value to end of cmd body (Note 1)
 * cmd_add_float()		- add a floating point value to end of cmd body
 * cmd_add_string()		- add a string object to end of cmd body
 * cmd_add_string_P()	- add a program memory string as a string object to end of cmd body
 * cmd_add_message()	- add a message to cmd body
 * cmd_add_message_P()	- add a program memory message the the cmd body
 *
 *	Note: Functions that return a cmd pointer point to the object that was modified
 *	or a NULL pointer if there was an error
 *
 *	Note Adding a really large integer (like a checksum value) may lose precision 
 *	due to the cast to a float. Sometimes it's better to load an integer as a 
 *	string if all you want to do is display it.
 */

void cmd_get_cmdObj(cmdObj_t *cmd)
{
	if (cmd->index >= cmd_index_max()) { return; }	// sanity

	index_t tmp = cmd->index;
	cmd_reset_obj(cmd);
	cmd->index = tmp;

	strcpy_P(cmd->token, cfgArray[cmd->index].token);	// token field is always terminated
	strcpy_P(cmd->group, cfgArray[cmd->index].group);	// group field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (cmd->group[0] != NUL) {
		if (pgm_read_byte(&cfgArray[cmd->index].flags) & F_NOSTRIP) {
			cmd->group[0] = NUL;
		} else {
			strcpy(cmd->token, &cmd->token[strlen(cmd->group)]); // strip group from the token
		}
	}
	((fptrCmd)(pgm_read_word(&cfgArray[cmd->index].get)))(cmd);	// populate the value
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
	return (cmd);
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

stat_t cmd_copy_string_P(cmdObj_t *cmd, const char_t *src_P)
{
	char_t buf[CMD_SHARED_STRING_LEN];
	strncpy_P(buf, src_P, CMD_SHARED_STRING_LEN);
	return (cmd_copy_string(cmd, buf));
}

cmdObj_t *cmd_add_object(const char_t *token)  // add an object to the body using a token
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->objtype != TYPE_EMPTY) {
			if ((cmd = cmd->nx) == NULL) return(NULL); // not supposed to find a NULL; here for safety
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index("",token)) == NO_MATCH) { return (NULL);}
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
		cmd->index = cmd_get_index("", cmd->token);
		cmd->objtype = TYPE_STRING;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_string_P(const char_t *token, const char_t *string)
{
	char_t message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, string);
	return(cmd_add_string(token, message));
}

cmdObj_t *cmd_add_message(const char_t *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (cfg.echo_json_messages != true)) { return (NULL);}
	return(cmd_add_string("msg", string));
}

cmdObj_t *cmd_add_message_P(const char_t *string)	// conditionally add a message object to the body
{
	if ((cfg.comm_mode == JSON_MODE) && (cfg.echo_json_messages != true)) { return (NULL);}
	char_t message[CMD_MESSAGE_LEN]; 
	sprintf_P(message, string);
	return(cmd_add_string("msg", message));
}

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
		switch (json_flags) {
			case JSON_NO_PRINT: { break; } 
			case JSON_OBJECT_FORMAT: { json_print_object(cmd_body); break; }
			case JSON_RESPONSE_FORMAT: { json_print_response(status); break; }
		}
	} else {
		switch (text_flags) {
			case TEXT_NO_PRINT: { break; } 
			case TEXT_INLINE_PAIRS: { text_print_inline_pairs(cmd_body); break; }
			case TEXT_INLINE_VALUES: { text_print_inline_values(cmd_body); break; }
			case TEXT_MULTILINE_FORMATTED: { text_print_multiline_formatted(cmd_body);}
		}
	}
}

/*
void cmd_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cfg.comm_mode == JSON_MODE) {
		switch (json_flags) {
			case JSON_NO_PRINT: { break; } 
			case JSON_OBJECT_FORMAT: { json_print_object(cmd_body); break; }
			case JSON_RESPONSE_FORMAT: { json_print_response(status); break; }
		}
	} else {
		switch (text_flags) {
			case TEXT_NO_PRINT: { break; } 
			case TEXT_INLINE_PAIRS: { _print_text_inline_pairs(); break; }
			case TEXT_INLINE_VALUES: { _print_text_inline_values(); break; }
			case TEXT_MULTILINE_FORMATTED: { _print_text_multiline_formatted();}
		}
	}
}

void _print_text_inline_pairs()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%s:%1.3f"), cmd->token, cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%s:%1.0f"), cmd->token, cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s:%s"), cmd->token, *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}		
	}
}

void _print_text_inline_values()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->objtype) {
			case TYPE_PARENT: 	{ if ((cmd = cmd->nx) == NULL) return; continue;} // NULL means parent with no child
			case TYPE_FLOAT:	{ fprintf_P(stderr,PSTR("%1.3f"), cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf_P(stderr,PSTR("%1.0f"), cmd->value); break;}
			case TYPE_STRING:	{ fprintf_P(stderr,PSTR("%s"), *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf_P(stderr,PSTR("\n")); return; }
		}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype != TYPE_EMPTY) { fprintf_P(stderr,PSTR(","));}
	}
}

void _print_text_multiline_formatted()
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		if (cmd->objtype != TYPE_PARENT) { cmd_print(cmd);}
		if ((cmd = cmd->nx) == NULL) return;
		if (cmd->objtype == TYPE_EMPTY) break;
	}
}
*/

/******************************************************************************
 ***** EEPROM access functions ************************************************
 ******************************************************************************
 * cmd_read_NVM_value()	 - return value (as float) by index
 * cmd_write_NVM_value() - write to NVM by index, but only if the value has 
 * 	changed (see 331.09 or earlier for token/value record-oriented routines)
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */
stat_t cmd_read_NVM_value(cmdObj_t *cmd)
{
	int8_t nvm_byte_array[NVM_VALUE_LEN];
	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
	return (STAT_OK);
}

stat_t cmd_write_NVM_value(cmdObj_t *cmd)
{
	if (cm.cycle_state != CYCLE_OFF) return (STAT_FILE_NOT_OPEN);	// can't write when machine is moving
	float tmp = cmd->value;
	ritorno(cmd_read_NVM_value(cmd));
	if (cmd->value != tmp) {		// catches the isnan() case as well
		cmd->value = tmp;
		int8_t nvm_byte_array[NVM_VALUE_LEN];
		memcpy(&nvm_byte_array, &tmp, NVM_VALUE_LEN);
		uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
	}
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

	i = cmd_get_index("fb");
	i = cmd_get_index("xfr");
	i = cmd_get_index("g54");

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

