/*
 * config.c - application independent configuration handling
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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
#include "persistence.h"
#include "hardware.h"
#include "settings.h"
#include "help.h"
#include "util.h"
#include "xio.h"

static void _set_defa(nvObj_t *nv);

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

nvStr_t nvStr;
nvList_t nvl;

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/* Primary access points to functions bound to text mode / JSON functions
 * These gatekeeper functions check index ranges so others don't have to
 *
 * nv_set() 	- Write a value or invoke a function - operates on single valued elements or groups
 * nv_get() 	- Build a nvObj with the values from the target & return the value
 *			   	  Populate nv body with single valued elements or groups (iterates)
 * nv_print()	- Output a formatted string for the value.
 * nv_persist() - persist value to non-volatile storage. Takes special cases into account
 *
 *	!!!! NOTE: nv_persist() cannot be called from an interrupt on the AVR due to the AVR1008 EEPROM workaround
 */
stat_t nv_set(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	return (((fptrCmd)GET_TABLE_WORD(set))(nv));
}

stat_t nv_get(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	return (((fptrCmd)GET_TABLE_WORD(get))(nv));
}

void nv_print(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) {
        return;
    }
	((fptrCmd)GET_TABLE_WORD(print))(nv);
}

stat_t nv_persist(nvObj_t *nv)	// nv_persist() cannot be called from an interrupt on the AVR due to the AVR1008 EEPROM workaround
{
	if (nv_index_lt_groups(nv->index) == false) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
	if (GET_TABLE_BYTE(flags) & F_PERSIST) {
        return(write_persistent_value(nv));
    }
	return (STAT_OK);
}

/************************************************************************************
 * config_init() - called once on hard reset
 *
 * Performs one of 2 actions:
 *	(1) if persistence is set up or out-of-rev load RAM and NVM with settings.h defaults
 *	(2) if persistence is set up and at current config version use NVM data for config
 *
 *	You can assume the cfg struct has been zeroed by a hard reset.
 *	Do not clear it as the version and build numbers have already been set by controller_init()
 *
 * NOTE: Config assertions are handled from the controller
 */
void config_init()
{
	nvObj_t *nv = nv_reset_nv_list(NUL);
	config_init_assertions();

#ifdef __ARM
// ++++ The following code is offered until persistence is implemented.
// ++++ Then you can use the AVR code (or something like it)
	cfg.comm_mode = JSON_MODE;					// initial value until EEPROM is read
	_set_defa(nv);
#endif
#ifdef __AVR
	cm_set_units_mode(MILLIMETERS);				// must do inits in millimeter mode
	nv->index = 0;								// this will read the first record in NVM

	read_persistent_value(nv);
	if (fp_NE(nv->value_flt, cs.fw_build)) {    // case (1) NVM is not setup or not in revision
		_set_defa(nv);
	} else {									// case (2) NVM is setup and in revision
		rpt_print_loading_configs_message();
		for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {
			if (GET_TABLE_BYTE(flags) & F_INITIALIZE) {
				strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);	// read the token from the array
				read_persistent_value(nv);
                if (GET_TABLE_BYTE(flags) & F_FLOAT) {
                    nv->valuetype = TYPE_FLOAT;
                } else {
                    nv->valuetype = TYPE_INTEGER;
                }
				nv_set(nv);
			}
		}
        // +++++ Status Reports
//	    sr_init_status_report(false);                // reset status reports
        sr_init_status_report_P(PSTR(""));      // load status report setup from NVram
	}
#endif

    nvl.container_index = nv_get_index("", "txt");  // cache the index of the txt container
    nvl.tid_index = nv_get_index("", "tid");        // cache the index of the tid
}

/*
 * set_defaults() - reset persistence with default values for machine profile
 * _set_defa() - helper function and called directly from config_init()
 */

static void _set_defa(nvObj_t *nv)
{
	cm_set_units_mode(MILLIMETERS);				// must do inits in MM mode
	for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {
		if (GET_TABLE_BYTE(flags) & F_INITIALIZE) {
    		nv->value_flt = GET_TABLE_FLOAT(default_value); // get default as float
            if (!(GET_TABLE_BYTE(flags) & F_FLOAT)) {
                nv->value_int = (uint32_t)nv->value_flt;    // cast in place to int if required
                nv->valuetype = TYPE_INTEGER;
            } else {
                nv->valuetype = TYPE_FLOAT;
            }
			strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
			nv_set(nv);                         // sets value
			nv_persist(nv);
		}
	}
    // +++++ Status Reports
//	sr_init_status_report(true);                // reset status reports
    sr_init_status_report_P(SR_DEFAULTS);       // reset status reports for defaults
	rpt_print_initializing_message();           // don't start TX until all the NVM persistence is done
}

stat_t set_defaults(nvObj_t *nv)
{
	// failsafe. nv->value_int must be true or no action occurs
	if (nv->value_int != true) {
        return(help_defa(nv));
    }
	_set_defa(nv);

	// The values in nv are now garbage. Mark the nv as $defa so it displays nicely.
//	strncpy(nv->token, "defa", TOKEN_LEN);		// correct, but not required
//	nv->index = nv_get_index("", nv->token);	    // correct, but not required
	nv->valuetype = TYPE_INTEGER;
	nv->value_int = 1;
	return (STAT_OK);
}

/*
 * config_init_assertions()
 * config_test_assertions() - check memory integrity of config sub-system
 */

void config_init_assertions()
{
	cfg.magic_start = MAGICNUM;
	cfg.magic_end = MAGICNUM;
	nvl.magic_start = MAGICNUM;
	nvl.magic_end = MAGICNUM;
	nvStr.magic_start = MAGICNUM;
	nvStr.magic_end = MAGICNUM;
}

stat_t config_test_assertions()
{
	if ((cfg.magic_start	!= MAGICNUM) || (cfg.magic_end != MAGICNUM)) return (STAT_CONFIG_ASSERTION_FAILURE);
	if ((nvl.magic_start	!= MAGICNUM) || (nvl.magic_end != MAGICNUM)) return (STAT_CONFIG_ASSERTION_FAILURE);
	if ((nvStr.magic_start	!= MAGICNUM) || (nvStr.magic_end != MAGICNUM)) return (STAT_CONFIG_ASSERTION_FAILURE);
	if (global_string_buf[MESSAGE_LEN-1] != NUL) return (STAT_CONFIG_ASSERTION_FAILURE);
	return (STAT_OK);
}

/***** Generic Internal Functions *********************************************/

/* Generic gets()
 *	get_nul()  - get nothing (returns STAT_PARAMETER_CANNOT_BE_READ)
 *  get_str()  - get value from stringp[] (no action required)
 *	get_ui8()  - get value as uint8_t
 *	get_u16()  - get value as uint16_t
 *	get_u32()  - get value as uint32_t
 *	get_data() - get value as 32 bit integer blind cast
 *	get_flt()  - get value as float
 *	get_format() - internal accessor for printf() format string
 */
stat_t get_nul(nvObj_t *nv)
{
	nv->valuetype = TYPE_NULL;
	return (STAT_PARAMETER_CANNOT_BE_READ);
}

stat_t get_str(nvObj_t *nv)
{
    nv->valuetype = TYPE_STRING;
    return (STAT_OK);
}

stat_t get_ui8(nvObj_t *nv)
{
	nv->value_int = *((uint8_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_u16(nvObj_t *nv)
{
	nv->value_int = *((uint16_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_u32(nvObj_t *nv)
{
    nv->value_int = *((uint32_t *)GET_TABLE_WORD(target));
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t get_data(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value_flt;
	*v = *((uint32_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_DATA;
	return (STAT_OK);
}

stat_t get_flt(nvObj_t *nv)
{
	nv->value_flt = *((float *)GET_TABLE_WORD(target));
	nv->precision = (int8_t)GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

/* Generic sets()
 *	set_nul()   - set nothing (returns STAT_PARAMETER_IS_READ_ONLY)
 *	set_not()   - set nothing (returns STAT_OK)
 *	set_ui8()   - set value as uint8_t value
 *	set_u16()   - set value as uint16 value
 *	set_u32()   - set value as uint32 value
 *	set_01()    - set a 0 or 1 uint8_t value with validation
 *	set_012()   - set a 0, 1 or 2 uint8_t value with validation
 *	set_0123()  - set a 0, 1, 2 or 3 uint8_t value with validation
 *	set_data()  - set value as 32 bit integer blind cast
 *	set_flt()   - set value as float
 */
stat_t set_nul(nvObj_t *nv) { return (STAT_PARAMETER_IS_READ_ONLY); }

stat_t set_not(nvObj_t *nv) { return (STAT_OK); }

stat_t set_ui8(nvObj_t *nv)
{
	*((uint8_t *)GET_TABLE_WORD(target)) = (uint8_t )nv->value_int;
//	nv->valuetype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_u16(nvObj_t *nv)
{
	*((uint16_t *)GET_TABLE_WORD(target)) = (uint16_t )nv->value_int;
//	nv->valuetype = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_u32(nvObj_t *nv)
{
    *((uint32_t *)GET_TABLE_WORD(target)) = (uint32_t )nv->value_int;
 //   nv->valuetype = TYPE_INTEGER;
    return(STAT_OK);
}

stat_t set_01(nvObj_t *nv)
{
	if (nv->value_int > 1) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_012(nvObj_t *nv)
{
	if (nv->value_int > 2) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_0123(nvObj_t *nv)
{
	if (nv->value_int > 3) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	return (set_ui8(nv));
}

stat_t set_data(nvObj_t *nv)
{
	uint32_t *v = (uint32_t*)&nv->value_flt;
	*((uint32_t *)GET_TABLE_WORD(target)) = *v;
	nv->valuetype = TYPE_DATA;
	return(STAT_OK);
}

stat_t set_flt(nvObj_t *nv)
{
	*((float *)GET_TABLE_WORD(target)) = nv->value_flt;
	nv->precision = GET_TABLE_WORD(precision);
	nv->valuetype = TYPE_FLOAT;
	return(STAT_OK);
}

/************************************************************************************
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
 *	get_grp() is a group expansion function that expands the parent group and returns
 *	the values of all the children in that group. It expects the first nvObj in the
 *	nvBody to have a valid group name in the token field. This first object will be set
 *	to a TYPE_PARENT. The group field is left nul - as the group field refers to a parent
 *	group, which this group has none.
 *
 *	All subsequent nvObjs in the body will be populated with their values (unless there
 *  are no more nvObj struct available). The token field will be populated as will the
 *  parent name in the group field.
 *
 *	The sys group is an exception where the children carry a blank group field, even though
 *	the sys parent is labeled as a TYPE_PARENT.
 */

stat_t get_grp(nvObj_t *nv)
{
    nvObj_t *nv_parent = nv;                        // used for error recovery
    nvObj_t *nv_tmp;                                // used for error recovery
	char *parent = nv->token;				        // token in the parent nv object is the group
    uint8_t child_depth = nv->depth+1;
	char group[GROUP_LEN+1];						// group string retrieved from cfgArray child
	nv->valuetype = TYPE_PARENT;					// make first object the parent

    // scan the config_app table looking for tokens belonging to the parent group
	for (index_t i=0; nv_index_is_single(i); i++) {
		strcpy_P(group, cfgArray[i].group);			// don't need strncpy as it's always terminated
		if (strcmp(parent, group) != 0) {           // if no match try the next one
            continue;
        }
        nv_tmp = nv;
        if ((nv = nv_get_next_nvObj(nv)) == NULL) { // get next nvObj or fail
            do {                                    // undo the group expansion
                nv_reset_nv(nv_tmp);
            } while (--nv_tmp >= nv_parent);
            return (STAT_JSON_TOO_MANY_PAIRS);
        }
		nv->index = i;
		nv_populate_nvObj_by_index(nv);
        nv->depth = child_depth;
	}
	return (STAT_OK);
}

/*
 * set_grp() - get or set one or more values in a group
 *
 *	This functions is called "_set_group()" but technically it's a getter and
 *	a setter. It iterates the group children and either gets the value or sets
 *	the value for each depending on the nv->valuetype.
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

stat_t set_grp(nvObj_t *nv)
{
	if (cs.comm_mode == TEXT_MODE) {
        return (STAT_INVALID_OR_MALFORMED_COMMAND);
    }
	for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) {
		if ((nv = nv->nx) == NULL) break;
		if (nv->valuetype == TYPE_EMPTY) break;
		else if (nv->valuetype == TYPE_NULL)		// NULL means GET the value
			nv_get(nv);
		else {
			nv_set(nv);
			nv_persist(nv);
		}
	}
	return (STAT_OK);
}

/*
 * nv_group_is_prefixed() - hack
 *
 *	This little function deals with the exception cases that some groups don't use
 *	the parent token as a prefix to the child elements; SR being a good example.
 */
uint8_t nv_group_is_prefixed(char *group)
{
	if (strcmp_P(PSTR("sr"), group) == 0) { return (false); }
	if (strcmp_P(PSTR("sys"), group) == 0) { return (false); }
	return (true);
}

/*
 * nv_index_is_container() - return true if the NV object is a container (txt)
 */

bool nv_index_is_container(index_t index)
{
    return (nvl.container_index == index);
}

/***********************************************************************************
 ***** nvObj functions ************************************************************
 ***********************************************************************************/

/***********************************************************************************
 * nvObj helper functions and other low-level nv helpers
 */

/* nv_get_index() - get index from mnenonic token + group
 *
 * nv_get_index() is the most expensive routine in the whole config. It does a
 * linear table scan of the PROGMEM strings, which of course could be further
 * optimized with indexes or hashing.
 */
index_t nv_get_index(const char *group, const char *token)
{
	char c;
	char str[TOKEN_LEN + GROUP_LEN+1];	// should actually never be more than TOKEN_LEN+1
	strncpy(str, group, GROUP_LEN+1);
	strncat(str, token, TOKEN_LEN+1);

	index_t i;
	index_t index_max = nv_index_max();

	for (i=0; i < index_max; i++) {
		if ((c = GET_TOKEN_BYTE(token[0])) != str[0]) {	continue; }					// 1st character mismatch
		if ((c = GET_TOKEN_BYTE(token[1])) == NUL) { if (str[1] == NUL) return(i);}	// one character match
		if (c != str[1]) continue;													// 2nd character mismatch
		if ((c = GET_TOKEN_BYTE(token[2])) == NUL) { if (str[2] == NUL) return(i);}	// two character match
		if (c != str[2]) continue;													// 3rd character mismatch
		if ((c = GET_TOKEN_BYTE(token[3])) == NUL) { if (str[3] == NUL) return(i);}	// three character match
		if (c != str[3]) continue;													// 4th character mismatch
		if ((c = GET_TOKEN_BYTE(token[4])) == NUL) { if (str[4] == NUL) return(i);}	// four character match
		if (c != str[4]) continue;													// 5th character mismatch
		return (i);																	// five character match
	}
	return (NO_MATCH);
}

/*
 * nv_get_type() - returns command type as a NV_TYPE enum
 */

uint8_t nv_get_type(nvObj_t *nv)
{
	if (nv->token[0] == NUL) return (NV_TYPE_NULL);
	if (strcmp_P(PSTR("gc"), nv->token) == 0) { return (NV_TYPE_GCODE); }
	if (strcmp_P(PSTR("sr"), nv->token) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(PSTR("qr"), nv->token) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(PSTR("msg"),nv->token) == 0) { return (NV_TYPE_MESSAGE); }
	if (strcmp_P(PSTR("err"),nv->token) == 0) { return (NV_TYPE_MESSAGE); }  // errors are reported as messages
	if (strcmp_P(PSTR("n"),  nv->token) == 0) { return (NV_TYPE_LINENUM); }
	return (NV_TYPE_CONFIG);
}

/******************************************************************************
 * nvObj low-level object and list operations
 * nv_get_next_nvObj()  - get the next available nvObj, or NULL if none available
 * nv_populate_nvObj_by_index() - setup a nv object by providing the index
 * nv_reset_nv()		- quick clear for a new nv object
 * nv_reset_nv_list()	- clear entire header, body and footer for a new use
 * nv_relink_nv_list()  - relink nx and pv pointers removing EMPTY and SKIP
 * nv_copy_string()		- used to write a string to shared string storage and link it
 * nv_add_object()		- write contents of parameter to  first free object in the body
 * nv_add_integer()		- add an integer value to end of nv body (Note 1)
 * nv_add_float()		- add a floating point value to end of nv body
 * nv_add_string()		- add a string object to end of nv body
 * nv_add_conditional_message() - add a message to nv body if messages are enabled
 *
 *	Note: Functions that return a nv pointer point to the object that was modified or
 *	a NULL pointer if there was an error.
 *
 *	Note: Adding a really large integer (like a checksum value) may lose precision due
 *	to the cast to a float. Sometimes it's better to load an integer as a string if
 *	all you want to do is display it.
 *
 *	Note: A trick is to cast all string constants for nv_copy_string(), nv_add_object(),
 *	nv_add_string() and nv_add_conditional_message() to (const char *). Examples:
 *
 *		nv_add_string((const char *)"msg", string);
 *
 *	On the AVR this will save a little static RAM. The "msg" string will occupy flash
 *	as an initializer and be instantiated in stack RAM when the function executes.
 *	On the ARM (however) this will put the string into flash and skip RAM allocation.
 */

nvObj_t *nv_get_next_nvObj(nvObj_t *nv)
{
    if (++nv >= NV_TID) {
        return ((nvObj_t *)NULL);
    }
    return nv;
}

/*
 * nv_populate_nvObj_by_index() - fill in details of an nvObj given the index field
 */

void nv_populate_nvObj_by_index(nvObj_t *nv)
{
	if (nv->index >= nv_index_max()) { return; }	// sanity

	index_t tmp = nv->index;
	nv_reset_nv(nv);
	nv->index = tmp;

	strcpy_P(nv->token, cfgArray[nv->index].token); // token field is always terminated
	strcpy_P(nv->group, cfgArray[nv->index].group); // group field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (nv->group[0] != NUL) {
		if (GET_TABLE_BYTE(flags) & F_NOSTRIP) {
			nv->group[0] = NUL;
		} else {
			strcpy(nv->token, &nv->token[strlen(nv->group)]); // strip group from the token
		}
	}
	((fptrCmd)GET_TABLE_WORD(get))(nv);		// populate the value
}

nvObj_t *nv_reset_nv(nvObj_t *nv)			// clear a single nvObj structure
{                                           // depth and pointers are NOT affected
	nv->valuetype = TYPE_EMPTY;				// selective clear is much faster than calling memset
	nv->index = 0;
	nv->value_int = 0;
	nv->precision = 0;
	nv->token[0] = NUL;
	nv->group[0] = NUL;
	nv->stringp = NULL;
	return (nv);							// return pointer to nv as a convenience to callers
}

/*
 * nv_reset_nv_list() - clear the NV list and set up as a parent or a plain list
 *
 *  - Clear all nvObjs in the nvList:
 *      - set nvObj[i].valuetype = TYPE_EMPTY
 *      - zero all values and NUL terminate all strings
 *
 *  - nvObj[0] is reserved for 'r' header (NV_HEAD)
 *      - set nvObj[0].valuetype = TYPE_EMPTY
 *      - set nvObj[0].depth = 0
 *
 *  - if "parent" arg is NUL:
 *      - set nvObj[1..N].depth = 1             // set all depths to 1
 *
 *  - else if "parent" arg is non-NUL:
 *      - set nvObj[1].valuetype = TYPE_PARENT
 *      - set nvObj[1].token = "parent"
 *      - set nvObj[1].depth = 1
 *      - set nvObj[2..N].depth = 2             // set all remaining depths to 2
 *
 *  -  return nv pointing to NV_BODY (nvObj[1])
 */

nvObj_t *nv_reset_nv_list(char *parent)
{
    // set up linked list and initialize elements
	nvStr.wp = 0;							    // reset the shared string
	nvObj_t *nv = NV_HEAD;                      // nvl.list[0]
    uint8_t depth = (*parent != NUL) ? 2 : 1;   // element depth = 2 if there is a parent
	for (uint8_t i=0; i<NV_LIST_LEN; i++, nv++) {
		nv->pv = (nv-1);	                    // the ends are bogus & corrected later
		nv->nx = (nv+1);
		nv->index = 0;
		nv->depth = depth;
		nv->precision = 0;
		nv->valuetype = TYPE_EMPTY;
		nv->token[0] = NUL;
	}
	(--nv)->nx = NULL;                          // correct the ends
	nv = nvl.list;
	nv->pv = NULL;

    // reserve r{} element
    nv->depth = 0;

    // setup parent element if one was requested. This is a convenience for calling routines
    nv = NV_BODY;
    if (*parent != NUL) {
	    nv->depth = 1;
	    nv->valuetype = TYPE_PARENT;
	    strcpy(nv->token, parent);
    }
    return (NV_BODY);
}

/*
 * nv_relink_nv_list() - relink nx and pv pointers removing EMPTY and SKIP
 *
 *  Relink list and return pointer to first non-zero element, or NULL if not found
 */
nvObj_t *nv_relink_nv_list()
{
    nvObj_t *nv = NV_HEAD;  // nvl.list     // read pointer - advances for each loop iteration
    nvObj_t *pv;                            // previous non-EMPTY/SKIP pair
    nvObj_t *hd;                            // adjusted head
    uint8_t i=0, j=0;

    // skip past empty/skip leading elements
    for ( ; j<NV_LIST_LEN; j++, i++) {
        if (nv->valuetype >= TYPE_NULL) {
            break;
        }
        nv = nv->nx;                        // skip over TYPE_EMPTY and TYPE_SKIP
    }
    if (j == NV_LIST_LEN) {                 // empty list
        return (NULL);
    }
    hd = nv;                                // mark the head
    pv = nv;
	for ( ; i<NV_LIST_LEN; i++, nv++) {
        if (nv->valuetype < TYPE_NULL) {    // handle TYPE_EMPTY and TYPE_SKIP
            continue;
        }
        pv->nx = nv;                        // Note: the first pair is messed up but gets corrected
        nv->pv = pv;
        pv = nv;
	}
    hd->pv = NULL;                          // correct the ends
    pv->nx = NULL;
    return (hd);
}

/*
 * nv_copy_string() - copy srouce string to the string allocation buffer and set pointer in NV obj
 */
stat_t nv_copy_string(nvObj_t *nv, const char *src)
{
	if ((nvStr.wp + strlen(src)) > NV_SHARED_STRING_LEN)
        return (STAT_BUFFER_FULL);

	char *dst = &nvStr.string[nvStr.wp];
	strcpy(dst, src);						// copy string to current head position
											// string has already been tested for overflow, above
	nvStr.wp += strlen(src)+1;				// advance head for next string
	nv->stringp = (char (*)[])dst;
	return (STAT_OK);
}

/* UNUSED
stat_t nv_copy_string_P(nvObj_t *nv, const char *src_P)
{
	char buf[NV_SHARED_STRING_LEN];
	strncpy_P(buf, src_P, NV_SHARED_STRING_LEN);
	return (nv_copy_string(nv, buf));
}
*/

static void _add_object_helper(nvObj_t *nv, const char *token, valueType valuetype)
{
	nv->valuetype = valuetype;
	strncpy(nv->token, token, TOKEN_LEN);
    if (nv->pv->valuetype == TYPE_PARENT) {
        nv->depth = nv->pv->depth + 1;
    } else {
        nv->depth = nv->pv->depth;
    }
}

nvObj_t *nv_add_object(const char *token)  // add an object to the body using a token
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv->nx) == NULL) {
                return (NULL);               // not supposed to find a NULL; here for safety
            }
			continue;
		}
		// load the index from the token or die trying
		if ((nv->index = nv_get_index((const char *)"",token)) == NO_MATCH) { return (NULL);}
		nv_populate_nvObj_by_index(nv);
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_integer(const char *token, const uint32_t value)// add an integer object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv->nx) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
//		strncpy(nv->token, token, TOKEN_LEN);
//		nv->valuetype = TYPE_INTEGER;
        _add_object_helper(nv, token, TYPE_INTEGER);
		nv->value_int = value;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_data(const char *token, const uint32_t value)// add an integer object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv->nx) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
//		strcpy(nv->token, token);
//		nv->valuetype = TYPE_DATA;
        _add_object_helper(nv, token, TYPE_DATA);
		float *v = (float*)&value;
		nv->value_flt = *v;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_float(const char *token, const float value)	// add a float object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv->nx) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
//		strncpy(nv->token, token, TOKEN_LEN);
//		nv->valuetype = TYPE_FLOAT;
        _add_object_helper(nv, token, TYPE_FLOAT);
		nv->value_flt = value;
		return (nv);
	}
	return (NULL);
}

// ASSUMES A RAM STRING. If you need to post a FLASH string use pstr2str to convert it to a RAM string
nvObj_t *nv_add_string(const char *token, const char *string) // add a string object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv->nx) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
//		strncpy(nv->token, token, TOKEN_LEN);
//		nv->valuetype = TYPE_STRING;
        _add_object_helper(nv, token, TYPE_STRING);
		if (nv_copy_string(nv, string) != STAT_OK) {
            return (NULL);
        }
		nv->index = nv_get_index((const char *)"", nv->token);
        return (nv);
	}
	return (NULL);
}

/*
 * nv_add_conditional_message() - queue a RAM string as a message in the response (conditionally)
 * nv_add_conditional_message_P()
 *
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

nvObj_t *nv_add_conditional_message_P(const char *msg_P)// conditionally add a FLASH message object to the body
{
    char msg[STATUS_MESSAGE_LEN];
    sprintf_P(msg, msg_P);
    return(nv_add_conditional_message(msg));
}

nvObj_t *nv_add_conditional_message(const char *msg)	// conditionally add a message object to the body
{
	if ((cs.comm_mode == JSON_MODE) && (js.echo_json_messages != true)) { return (NULL);}
	return(nv_add_string((const char *)"msg", msg));
}

/**** nv_print_list() - print nv_array as JSON or text **********************
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

void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cs.comm_mode == JSON_MODE) {
		json_print_list(status, json_flags);
	} else {
		text_print_list(status, text_flags);
	}
}

/****************************************************************************
 ***** Diagnostics **********************************************************
 ****************************************************************************/

void nv_dump_nv(nvObj_t *nv)
{
	printf_P (PSTR("i:%d, d:%d, t:%d, p:%d, v:%f, g:%s, t:%s, s:%s\n"),
			 nv->index,
			 nv->depth,
			 nv->valuetype,
			 nv->precision,
			 (double)nv->value_flt,
			 nv->group,
			 nv->token,
			 (char *)nv->stringp);
}
