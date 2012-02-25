/*
 * canonical_machine.c - rs274/ngc status report and other reporting functions.
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S Hart, Jr.
 */
/* TinyG is free software: you can redistribute it and/or modify it 
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
//#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "json_parser.h"
#include "controller.h"
#include "planner.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "report.h"
#include "xio/xio.h"
#include "xmega/xmega_rtc.h"

static void _run_csv_status_report();

/*****************************************************************************
 * Status Reports
 *
 *	Status report behaviors
 *
 *	Configuration:
 *
 *		Status reports are configurable only from JSON. There is no way to set 
 *		the status report config from text mode or grbl mode. SRs are configured
 *		by sending a status report SET object, e.g:
 *
 *		  {"sr":{"line":true,"posx":true,"posy":true....."motm":true,"stat":true}}
 *
 *	Status report formats: The following formats exist for status reports:
 *
 *	  -	JSON format: Returns a JSON object as above, but with the values filled in. 
 *		In JSON form all values are returned as numeric values or enumerations. 
 *		E.g. "posx" is returned as 124.523 and "unit" is returned as 0 for 
 *		inches (G20) and 1 for mm (G21).
 *
 *	  - CSV format: Returns a single line of comma separated token:value pairs.
 *		Values are returned as numeric values or English text.
 *		E.g. "posx" is still returned as 124.523 but "unit" is returned as 
 *		"inch" for inches (G20) and "mm" for mm (G21).
 *
 *	  - Multi-line format: Returns a multi-line report where each value occupies 
 *		one line. Each line contains explanatory English text. Enumerated values are
 *		returned as English text as per CSV form.
 *	
 *	Status report invocation: Status reports can be invoked in the following ways:
 *
 *	  - Ad-hoc request in JSON mode. Issue {"sr":""} (or equivalent). Returns a 
 *		JSON format report.
 *
 *	  - Automatic status reports in JSON mode. Returns JSON format reports 
 *		according to "si" setting.
 *
 *	  - Ad-hoc request in text mode. Triggered by sending ?<cr>. Returns status 
 *		report in multi-line format. Additionally, a line starting with ? will put 
 *		the system into text mode.
 *
 *	  - Automatic status reports in text mode return CSV format according to si setting
 *
 *	  - grbl mode forms are not yet defined.
 */

/* rpt_init_status_report()
 *
 *	Call this function to completely re-initialze the status report
 *	Sets SR to hard-coded default and re-initializes sr values in NVM
 */
void rpt_init_status_report(uint8_t persist_flag)
{
	cmdObj cmd;
	uint8_t i=0;
	char sr_defaults[][CMD_TOKEN_LEN+1] = { SR_DEFAULTS };	// see config.h

	cmd.index = cmd_get_index_by_token("sr00");		// find first persistence index
	for (; i < (sizeof(sr_defaults)/(CMD_TOKEN_LEN+1)); i++) {
		cmd.value = cmd_get_index_by_token(sr_defaults[i]);
		cfg.status_report_spec[i] = cmd.value;
		if (persist_flag == true) {
			cmd_write_NVM_value(&cmd);
			cmd.index++;
		}
	}
	for (; i < CMD_STATUS_REPORT_LEN; i++) {	// fill rest of spec with -1
		cmd.value = -1;
		cfg.status_report_spec[i] = cmd.value;
		if (persist_flag == true) {
			cmd_write_NVM_value(&cmd);
			cmd.index++;
		}
	}
	cm.status_report_counter = cfg.status_report_interval;
}

/*	rpt_decr_status_report()  - decrement sr counter
 *	rpt_try_status_report()   - send a status report if it's time to send one
 *	rpt_force_status_report() - force status report to send on next try attempt
 *	rpt_run_status_report()	  - send a status report
 */
void rpt_decr_status_report() 
{
	if (cm.status_report_counter != 0) cm.status_report_counter--; // stick at zero
}

void rpt_force_status_report()
{
	cm.status_report_counter = 0; // report will be called from controller dispatcher
}

uint8_t rpt_try_status_report()	  // called by controller dispatcher
{
	if ((cm.machine_state != MACHINE_RESET) && 
		(cfg.status_report_interval > 0) && (cm.status_report_counter == 0)) {
		rpt_run_status_report();
		cm.status_report_counter = (cfg.status_report_interval / RTC_PERIOD);	// RTC fires every 10 ms
		return (TG_OK);
	}
	return (TG_NOOP);
}

void rpt_run_status_report()
{
	if (tg.communications_mode == TG_JSON_MODE) {
		rpt_run_json_status_report();
		js_make_json_string(cmd_array, tg.out_buf);
		fprintf_P(stderr, PSTR("%s"), tg.out_buf);
	} else {
		_run_csv_status_report();
	}
}

static void _run_csv_status_report() 		// single line status report
{
	cmdObj cmd;

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		cmd.index = cfg.status_report_spec[i];
		if (cmd.index < 1) continue;		// trap 0 and -1 cases
		if (i != 0) fprintf_P(stderr,PSTR(","));
		cmd_get_cmd(&cmd);
		if (cmd.value_type == VALUE_TYPE_FLOAT) {
			fprintf_P(stderr,PSTR("%s:%1.3f"), cmd.token, cmd.value);
		} else if (cmd.value_type == VALUE_TYPE_INTEGER) {
			fprintf_P(stderr,PSTR("%s:%1.0f"), cmd.token, cmd.value);
		} else if (cmd.value_type == VALUE_TYPE_STRING) {
			fprintf_P(stderr,PSTR("%s:%s"), cmd.token, cmd.string_value);
		}
	}
	fprintf_P(stderr,PSTR("\n"));
}

void rpt_run_multiline_status_report()		// multiple line status report
{
	cmdObj *cmd = cmd_array;
	fprintf_P(stderr,PSTR("\n"));
	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		cmd->index = cfg.status_report_spec[i];
		if (cmd->index < 1) continue;		// trap 0 and -1 cases
		cmd_print(cmd);
	}
}

void rpt_run_json_status_report() 				// JSON status report
{
	cmdObj *cmd = cmd_array;

	cmd->value_type = VALUE_TYPE_PARENT; 		// setup the parent object
	strcpy(cmd->token, "sr");
	cmd++;

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = cfg.status_report_spec[i]) == -1) { continue;}
		if (cmd->index == 0) { break;}
		cmd_get_cmd(cmd);
		(cmd-1)->nx = cmd; // set the next object of the previous object to be this object
		cmd++;
	}
}

/*****************************************************************************
 * rpt_print_configs()  - config print function
 *
 *	The following $ forms are supported:
 *	 single group:
 *	 $		print general settings (all non-axis and non-motor commands)
 *	 $g		print general settings (same as above)
 *	 $x		print axis settings (xyzabc)
 *	 $1		print motor settings (1234)
 *	 $g54	print ofsets for G54 (through G59)
 *
 *   multiple group:
 *	 $n		print all axis settings
 *	 $m		print all motor settings
 *	 $g5	print all offsets (G54 - G59)	
 *	 $$		print all settings
 */
 /*
void rpt_print_configs(char *str) 
{
	INDEX_T i;
	char groups[AXES+MOTORS+2];	// AXES + MOTORS + a general group + a terminator
	char *ptr = groups;
	char token[CMD_TOKEN_LEN+1];
//	char exclude[] = {"g17,g18,g19,g20,g21,g61,g61.1,g64,g90,g91"};	// don't print these tokens

	// special case to print coordinate system offsets
	if (str[1] == 'o') {
		for (i=0; i<cmd_get_max_index(); i++) {
			cmd_get_token(i, token);
			if ((token[0] == 'g') && (token[1] == '5')) {
				cmd_print(i);
			}
		}
		return;
	}
	// setup the groups specifier string
	groups[1] = NUL; // optimistically terminate string for NUL and default cases
	switch (str[1]) {
		case ('n'): { strcpy(groups, "xyzabc"); break;}
		case ('m'): { strcpy(groups, "1234"); break;}
		case ('$'): { strcpy(groups, "xyzabc1234g"); break;}
		case (NUL): { groups[0] = 'g'; break;}
		default: 	{ groups[0] = str[1];}
	}
	// now print everything in the group list
	while (*ptr != NUL) {
		for (i=0; i<cmd_get_max_index(); i++) {
//			if (strstr(exclude, cmd_get_token(i, cmd.token)) != NULL) continue;
			if (cmd_get_group(i) == *ptr) {
				cmd_print(i);
			}
		}
		ptr++;
	}
}
*/

/****************************************************************************
 ***** Report Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TEST_REPORT

void sr_unit_tests(void)
{
	sr_init();
	tg.communications_mode = TG_JSON_MODE;
	sr_run_status_report();
}


#endif
