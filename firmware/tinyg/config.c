/*
 * config.c - application independent configuration handling
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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
	if (cfg_has_flag(nv->index, F_PERSIST)) {
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
            if (cfg_has_flag(nv->index, F_INITIALIZE)) {
				strncpy_P(nv->token, cfgArray[nv->index].token, TOKEN_LEN);	// read the token from the array
				read_persistent_value(nv);
                if (cfg_is_type(nv->index) == TYPE_FLOAT) {
                    nv->valuetype = TYPE_FLOAT;
                } else {
                    nv->valuetype = TYPE_INTEGER;
                }
				nv_set(nv);
			}
		}
        sr_init_status_report_P(PSTR(""));      // load status report setup from NVram
	}
#endif
}

/*
 * set_defaults() - reset persistence with default values for machine profile
 * _set_defa() - helper function and called directly from config_init()
 */

static void _set_defa(nvObj_t *nv)
{
	cm_set_units_mode(MILLIMETERS);				// must do inits in MM mode
	for (nv->index=0; nv_index_is_single(nv->index); nv->index++) {
        if (cfg_has_flag(nv->index, F_INITIALIZE)) {
    		nv->value_flt = GET_TABLE_FLOAT(default_value); // get default as float
            if (cfg_is_type(nv->index) != TYPE_FLOAT) {
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
    sr_init_status_report_P(SR_DEFAULTS);       // reset status reports to defaults
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
    nv_reset_nv_list("r");
	strncpy(nv->token, "defa", TOKEN_LEN);
	nv->index = nv_get_index("", nv->token);	// correct, but not required
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
    if ((BAD_MAGIC(cfg.magic_start)) ||
        (BAD_MAGIC(cfg.magic_end)) ||
        (BAD_MAGIC(nvl.magic_start)) ||
        (BAD_MAGIC(nvl.magic_end)) ||
        (BAD_MAGIC(nvStr.magic_start)) ||
        (BAD_MAGIC(nvStr.magic_end)) ) {

        return(cm_panic_P(STAT_CONFIG_ASSERTION_FAILURE, PSTR("config_test_assertions()")));
    }
    return (STAT_OK);
}

/***** Generic Internal Functions *********************************************
 *
 * cfg_has_flag() - test for flag set in cfgArray table
 * cfg_is_type()  - return data type of config item
 */

bool cfg_has_flag(index_t index, uint8_t bitmask)
{
#ifdef __AVR
    return (pgm_read_byte(&cfgArray[index].flags) & bitmask);
#else
    return (cfgArray[index].flags) & bitmask);
#endif
}

valueType cfg_is_type(index_t index)
{
#ifdef __AVR
    return (pgm_read_byte(&cfgArray[index].flags) & F_TYPE_MASK);
#else
    return (cfgArray[index].flags) & F_TYPE_MASK);
#endif
}

/* Generic gets()
 *	get_nul()  - get nothing (returns STAT_PARAMETER_CANNOT_BE_READ)
 *  get_str()  - get value from str (no action required)
 *	get_ui8()  - get value as uint8_t
 *	get_u16()  - get value as uint16_t
 *	get_u32()  - get value as uint32_t
 *  get_int()  - get value as a signed integer (up to 32 bits)
 *	get_flt()  - get value as float
 *	get_data() - get value as 32 bit integer blind cast
 */
stat_t get_nul(nvObj_t *nv)
{
	nv->valuetype = TYPE_NULL;
	return (STAT_NOOP);
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

stat_t get_int(nvObj_t *nv)
{
//	nv->value_int = *((int32_t *)GET_TABLE_WORD(target));
	nv->value_int = *((int8_t *)GET_TABLE_WORD(target));
	nv->valuetype = TYPE_SIGNED;
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
 *  set_str()   - set string value - just for test, performs no action
 */
stat_t set_nul(nvObj_t *nv) { return (STAT_PARAMETER_IS_READ_ONLY); }

stat_t set_not(nvObj_t *nv) { return (STAT_OK); }

stat_t set_ui8(nvObj_t *nv)
{
	*((uint8_t *)GET_TABLE_WORD(target)) = (uint8_t )nv->value_int;
	return(STAT_OK);
}

stat_t set_u16(nvObj_t *nv)
{
	*((uint16_t *)GET_TABLE_WORD(target)) = (uint16_t )nv->value_int;
	return(STAT_OK);
}

stat_t set_u32(nvObj_t *nv)
{
    *((uint32_t *)GET_TABLE_WORD(target)) = (uint32_t )nv->value_int;
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

stat_t set_str(nvObj_t *nv)
{
    return (STAT_OK);
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
 *	  {"x":n}                       get all X axis parameters
 *	  {"x":{"vm":n}}                get X axis velocity max
 *	  {"x":{"vm":1000}}             set X axis velocity max
 *	  {"x":{"vm":n,"fr":n}}         get X axis velocity max and feed rate
 *	  {"x":{"vm":1000,"fr";900}}    set X axis velocity max and feed rate
 *	  {"x":{"am":1,"fr":800,....}}  set multiple or all X axis parameters
 */

/*
 * get_grp() - read data from axis, motor, system or other group
 *
 *	get_grp() is a group expansion function that expands the parent group and returns
 *	the values of all the children in that group. It expects the first nvObj in the
 *	nvBody to have a valid group name in the token field. This first object will be set
 *	to a TYPE_PARENT. The group field is left NUL - as the group field refers to a parent
 *	group, which this group has none.
 *
 *	All subsequent nvObjs in the body will be populated with their values (unless there
 *  are no more nvObj struct available). The token field will be populated as will the
 *  parent name in the group field.
 *
 *	The sys group is an exception where the children carry a blank group field, even though
 *	the sys parent is labeled as a TYPE_PARENT.
 *
 *  Example: {x:n}
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
        if ((nv = nv_next(nv)) == NULL) {           // get next nvObj or fail
            do {                                    // undo the group expansion
                nv_reset_nv(nv_tmp);
            } while (--nv_tmp >= nv_parent);
            return (STAT_JSON_TOO_MANY_PAIRS);
        }
		nv_populate_nv_by_index(nv, i);
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
 *    SET example:   {x:{vm:10000, fr:8000}}
 *    GET example:   {x:{vm:n, fr:n}}
 *    Mixed example: {x:{vm:10000, fr:n}}
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */

stat_t set_grp(nvObj_t *nv)
{
	if (cs.comm_mode == TEXT_MODE) {
        return (STAT_UNRECOGNIZED_NAME);
    }
	for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) {
		if ((nv = nv_next(nv)) == NULL) break;
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
 * nv_group_is_typesafe() - hack
 *
 *	These little functions deal with exception cases for some groups.
 *  It would be better put flags on these cfg items directly and remove these
 *  functions, which we will probably do once we bump the flags byte to 16 bits.
 *
 *  nv_group_is_prefixed() returns false if the member of the group does not
 *  use the parent token as a prefix to the child elements. SYS is a good example.
 *
 *  nv_group_is_typesafe() returns false if JSON type checking should be disabled
 *  when collecting children for this group.
 */

bool nv_group_is_prefixed(char *group)
{
	if ((strcmp_P(group, PSTR("sys"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("set"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("srs"))) == 0) { return (false); }
	if ((strcmp_P(group, PSTR("sr"))) == 0)  { return (false); }
	return (true);
}

bool nv_group_is_typesafe(char *group)
{
    if ((strcmp_P(group, PSTR("set"))) == 0) { return (false); }
    if ((strcmp_P(group, PSTR("sr"))) == 0)  { return (false); }
    return (true);
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
	if (nv->token[0] == NUL) { return (NV_TYPE_NULL); }
	if (strcmp_P(nv->token, PSTR("gc")) == 0) { return (NV_TYPE_GCODE); }

	if (strcmp_P(nv->token, PSTR("sr")) == 0) { return (NV_TYPE_REPORT); }
//	if (strcmp_P(nv->token, PSTR("sr")) == 0) {
//        if (nv->valuetype == TYPE_PARENT) {
//            return (NV_TYPE_REPORT);            // actual SR{}s
//        } else {
//	        return (NV_TYPE_CONFIG);            // (sr:f} and {sr:t}
//        }
//    }

	if (strcmp_P(nv->token, PSTR("er")) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(nv->token, PSTR("qr")) == 0) { return (NV_TYPE_REPORT); }
	if (strcmp_P(nv->token, PSTR("n"))  == 0) { return (NV_TYPE_LINENUM); }
	if (strcmp_P(nv->token, PSTR("msg")) == 0) { return (NV_TYPE_MESSAGE); }
	if (strcmp_P(nv->token, PSTR("err")) == 0) { return (NV_TYPE_MESSAGE); }  // errors are reported as messages
	return (NV_TYPE_CONFIG);
}

/******************************************************************************
 * nvObj low-level object and list operations
 *
 * nv_prev()            - return pointer to previous NV or NULL if at start
 * nv_next()            - return pointer to next NV or NULL if at end
 * nv_next_empty()      - return pointer to next empty NV or NULL if none
 *
 * nv_reset_nv()		- quick clear for a new nv object
 * nv_reset_nv_list()	- clear entire header, body and footer for a new use
 * nv_relink_nv_list()  - relink nx and pv pointers removing EMPTY and SKIP
 * nv_populate_nvObj_by_index() - setup a nv object by providing the index
 *
 * nv_copy_string()		- used to write a string to shared string storage and link it
 * nv_add_object()		- write contents of parameter to  first free object in the body
 * nv_add_integer()		- add an integer value to end of nv body (Note 1)
 * nv_add_float()		- add a floating point value to end of nv body
 * nv_add_string()		- add a string object to end of nv body
 * nv_add_message()     - add a message to NV body if messages are enabled
 * nv_add_message_P()   - add a PSTR message to NV body if messages are enabled
 *
 *	Note: Functions that return a nv pointer point to the object that was modified or
 *	a NULL pointer if there was an error.
 *
 *	Note: Adding a really large integer (like a checksum value) may lose precision due
 *	to the cast to a float. Sometimes it's better to load an integer as a string if
 *	all you want to do is display it.
 *
 *	Note: A trick is to cast all string constants for nv_copy_string(), nv_add_object(),
 *	nv_add_string() and nv_add_message() to (const char *). Examples:
 *
 *		nv_add_string((const char *)"msg", string);
 *
 *	On the AVR this will save a little static RAM. The "msg" string will occupy flash
 *	as an initializer and be instantiated in stack RAM when the function executes.
 *	On the ARM (however) this will put the string into flash and skip RAM allocation.
 */

/*
 * Wrappers for nx/pv allowing later substitution w/more space efficient methods
 * Return pointer to next or previous NV object or NULL it at end or beginning
 */
nvObj_t *nv_prev(nvObj_t *nv) { return (nv->pv); }
nvObj_t *nv_next(nvObj_t *nv) { return (nv->nx); }

nvObj_t *nv_next_empty(nvObj_t *nv) {
    do {
        if (nv->valuetype == TYPE_EMPTY) {
            return (nv);
        }
    } while ((nv = nv->nx) != NULL);
    return (NULL);
}

nvObj_t *nv_reset_nv(nvObj_t *nv)			// clear a single nvObj structure
{                                           // depth and pointers are NOT affected
	nv->valuetype = TYPE_EMPTY;				// selective clear is actually faster than calling memset()
	nv->index = NO_MATCH;
	nv->value_int = 0xFFFFFFFF;
	nv->precision = 0;
	nv->token[0] = NUL;
	nv->group[0] = NUL;
	nv->str = NULL;
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
    uint8_t nv_num=0;
	nvObj_t *nv = NV_HEAD;                      // nvl.list[0]
    uint8_t depth = (*parent != NUL) ? 2 : 1;   // element depth = 2 if there is a parent
	for (uint8_t i=0; i<NV_LIST_LEN; i++, nv++) {
		nv->pv = (nv-1);	                    // the ends are bogus & corrected later
		nv->nx = (nv+1);
        nv->nv_num = nv_num++;
		nv->depth = depth;
		nv->index = 0;
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
//        if (nv->valuetype >= TYPE_NULL) {
//            break;
//        }
//        nv = nv_next(nv);                   // skip over TYPE_EMPTY and TYPE_SKIP

        if ((nv->valuetype == TYPE_EMPTY) || (nv->valuetype == TYPE_SKIP)) {
            nv = nv_next(nv);
            continue;
        }
        break;
    }
    if (j == NV_LIST_LEN) {                 // empty list
        return (NULL);
    }
    hd = nv;                                // mark the head
    pv = nv;
	for ( ; i<NV_LIST_LEN; i++, nv++) {
        // skip over TYPE_EMPTY and TYPE_SKIP
//        if (nv->valuetype < TYPE_NULL) {
        if ((nv->valuetype == TYPE_EMPTY) || (nv->valuetype == TYPE_SKIP)) { // skip past
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
 * nv_populate_nv_by_index() - fill in details of an nvObj given the index field
 */

void nv_populate_nv_by_index(nvObj_t *nv, index_t index)
{
	if (index >= nv_index_max()) { return; } // sanity

	nv->str = NULL;
	nv->index = index;
    nv->valuetype = cfg_is_type(index);
    if (nv->valuetype == TYPE_FLOAT) {
       	nv->precision = (int8_t)GET_TABLE_WORD(precision);
    }
	strcpy_P(nv->token, cfgArray[nv->index].token);         // NB: token field is always NUL terminated

    if (cfg_has_flag(nv->index, F_NOSTRIP)) {
        nv->group[0] = NUL;                                 // don't carry the group
    } else {
	    strcpy_P(nv->group, cfgArray[nv->index].group);     // NB: group string is always NUL terminated
        strcpy(nv->token, &nv->token[strlen(nv->group)]);   // strip group from the token
    }
    // read the getter from program memory; run it to get the value into nv->value_int or nv->value_flt
	((fptrCmd)GET_TABLE_WORD(get))(nv);		                // populate the value
}

/*
 * nv_copy_string() - copy source string to the string allocation buffer and set pointer in NV obj
 */
stat_t nv_copy_string(nvObj_t *nv, const char *src)
{
	if ((nvStr.wp + strlen(src)) > NV_SHARED_STRING_LEN) {
        return (STAT_BUFFER_FULL);
    }
	char *dst = &(nvStr.string[nvStr.wp]);
	strcpy(dst, src);						// copy string to current head position
											// string has already been tested for overflow, above
	nvStr.wp += strlen(src)+1;				// advance head for next string
	nv->str = dst;
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

    if ((nv_prev(nv))->valuetype == TYPE_PARENT) {
        nv->depth = (nv_prev(nv))->depth + 1;
    } else {
        nv->depth = (nv_prev(nv))->depth;
    }
}

nvObj_t *nv_add_object(const char *token)  // add an object to the body using a token
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
                return (NULL);               // not supposed to find a NULL; here for safety
            }
			continue;
		}
		// load the index from the token or die trying
		if ((nv->index = nv_get_index((const char *)"",token)) == NO_MATCH) { return (NULL);}
		nv_populate_nv_by_index(nv, nv->index);
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_integer(const char *token, const uint32_t value)// add TYPE_INTEGER object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_INTEGER);
		nv->value_int = value;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_data(const char *token, const uint32_t value)// add TYPE_DATA object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
        _add_object_helper(nv, token, TYPE_DATA);
		float *v = (float*)&value;
		nv->value_flt = *v;
		return (nv);
	}
	return (NULL);
}

nvObj_t *nv_add_float(const char *token, const float value)	// add TYPE_FLOAT object to the body
{
	nvObj_t *nv = NV_BODY;
	for (uint8_t i=0; i<NV_BODY_LEN; i++) {
		if (nv->valuetype != TYPE_EMPTY) {
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
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
			if ((nv = nv_next(nv)) == NULL) {
    			return (NULL);               // not supposed to find a NULL; here for safety
			}
			continue;
		}
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
 * nv_add_message() - queue a RAM string as a message in the response (conditionally)
 * nv_add_message_P()
 *
 *  Adds a message to the NV list if in JSON mode and messages are enabled for display
 *	Note: If you need to post a FLASH string use pstr2str to convert it to a RAM string
 */

nvObj_t *nv_add_message(const char *msg)
{
	if ((cs.comm_mode == JSON_MODE) && (js.echo_json_messages != true)) { return (NULL);}
	return(nv_add_string((const char *)"msg", msg));
}

nvObj_t *nv_add_message_P(const char *msg_P)
{
    char msg[LINE_MSG_LEN];
    strcpy_P(msg, msg_P);
    return(nv_add_message(msg));
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
