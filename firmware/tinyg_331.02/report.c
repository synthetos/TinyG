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

static void _text_status_report();

/*****************************************************************************
 * Status Reports
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
	INDEX_T sr_index = cmd_get_index_by_token("sr00");	// find first persistence index
	char sr_defaults[][CMD_TOKEN_LEN+1] = {"line","xpos","ypos","zpos","apos","vel","unit","stat"};

	for (; i < (sizeof(sr_defaults)/(CMD_TOKEN_LEN+1)); i++) {
		cmd.value = cmd_get_index_by_token(sr_defaults[i]);
		cfg.status_report_spec[i] = cmd.value;
		if (persist_flag == true) cmd_write_NVM_value(sr_index++, &cmd);
	}
	for (; i < CMD_STATUS_REPORT_LEN; i++) {	// fill rest of spec with -1
		cmd.value = -1;
		cfg.status_report_spec[i] = cmd.value;
		if (persist_flag == true) cmd_write_NVM_value(sr_index++, &cmd);
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
	if ((cm.machine_state == MACHINE_RUN) && 
		(cfg.status_report_interval > 0) && 
		(cm.status_report_counter == 0)) {
		rpt_run_status_report();
		cm.status_report_counter = cfg.status_report_interval;
		return (TG_OK);
	}
	return (TG_NOOP);
}

void rpt_run_status_report()
{
	if (tg.communications_mode == TG_JSON_MODE) {
		rpt_json_status_report();
		js_make_json_string(cmd_array, tg.out_buf);
		fprintf_P(stderr, PSTR("%s"), tg.out_buf);
	} else {
		_text_status_report();
	}
}

static void _text_status_report() 
{
	INDEX_T i;
	cmdObj cmd;

	for (uint8_t j=0; j<CMD_STATUS_REPORT_LEN; j++) {
		i = cfg.status_report_spec[j];
		if (i < 1) continue;		// trap 0 and -1 cases
		if (j != 0) fprintf_P(stderr,PSTR(","));
		cmd_get_cmd(i, &cmd);
		if (cmd.value_type == VALUE_TYPE_FLOAT) {
			fprintf_P(stderr,PSTR("%s:%1.3f"), cmd.token, cmd.value);
		} else if (cmd.value_type == VALUE_TYPE_INT32) {
			fprintf_P(stderr,PSTR("%s:%1.0f"), cmd.token, cmd.value);
		} else if (cmd.value_type == VALUE_TYPE_STRING) {
			fprintf_P(stderr,PSTR("%s:%s"), cmd.token, cmd.string);			
		}
	}
	fprintf_P(stderr,PSTR("\n"));
}

void rpt_json_status_report() 
{
	INDEX_T i;
	cmdObj *cmd = cmd_array;

	cmd->value_type = VALUE_TYPE_PARENT; // setup the parent object
	strcpy(cmd->token, "sr");

	for (uint8_t j=0; j<CMD_STATUS_REPORT_LEN; j++) {
		if ((i = cfg.status_report_spec[j]) == -1) continue;
		if (i == 0) break;
		cmd_get_cmd(i, ++cmd);
		(cmd-1)->nx = cmd; // set the next object of the previous object to be this object
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
void rpt_print_configs(char *str) 
{
	INDEX_T i;
	char groups[AXES+MOTORS+2];	// AXES + MOTORS + a general group + a terminator
	char *ptr = groups;
	char token[CMD_TOKEN_LEN+1];
//	char exclusions[] = {"g17,g18,g19,g20,g21,g61,g61.1,g64,g90,g91"};	// don't print these tokens

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
//			if (strstr(exclusions, cmd_get_token(i, cmd.token)) != NULL) continue;
			if (cmd_get_group(i) == *ptr) {
				cmd_print(i);
			}
		}
		ptr++;
	}
}

/*****************************************************************************
 * rpt_print_machine_state()
 */
// Independent format strings - not in an array
static char msg_PosX[] PROGMEM = "Position X:%11.3f %s\n";
static char msg_PosY[] PROGMEM = "Position Y:%11.3f %s\n";
static char msg_PosZ[] PROGMEM = "Position Z:%11.3f %s\n";
static char msg_PosA[] PROGMEM = "Position A:%11.3f degrees\n";
static char msg_PosB[] PROGMEM = "Position B:%11.3f degrees\n";
static char msg_PosC[] PROGMEM = "Position C:%11.3f degrees\n";
static char msg_OfsI[] PROGMEM = "Offset I:%13.3f %s\n";
static char msg_OfsJ[] PROGMEM = "Offset J:%13.3f %s\n";
static char msg_OfsK[] PROGMEM = "Offset K:%13.3f %s\n";
static char msg_Feed[] PROGMEM = "Feed Rate:%12.3f %s \\ min\n";
//static char msg_Limit[] PROGMEM = "Limit Switches: %3.0f %s\n";

// Format strings with indexing arrays
static char msg_g20[] PROGMEM = "Units:           G20 - inches mode\n";
static char msg_g21[] PROGMEM = "Units:           G21 - millimeter mode\n";
static PGM_P msgUnitsMode[] PROGMEM = { msg_g20, msg_g21 };

static char msg_g53[] PROGMEM = "Coord system:    Absolute coordinates\n";
static char msg_g54[] PROGMEM = "Coord system:    G54\n";
static char msg_g55[] PROGMEM = "Coord system:    G55\n";
static char msg_g56[] PROGMEM = "Coord system:    G56\n";
static char msg_g57[] PROGMEM = "Coord system:    G57\n";
static char msg_g58[] PROGMEM = "Coord system:    G58\n";
static char msg_g59[] PROGMEM = "Coord system:    G59\n";
static PGM_P msgCoordSystem[] PROGMEM = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static char msg_g00[] PROGMEM = "Motion mode:     G0  - linear traverse (seek)\n";
static char msg_g01[] PROGMEM = "Motion mode:     G1  - linear feed\n";
static char msg_g02[] PROGMEM = "Motion mode:     G2  - clockwise arc feed\n";
static char msg_g03[] PROGMEM = "Motion mode:     G3  - counter clockwise arc feed\n";
static char msg_g80[] PROGMEM = "Motion mode:     G80 - cancel motion mode (none active)\n";
static PGM_P msgMotionMode[] PROGMEM = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static char msg_g17[] PROGMEM = "Plane selection: G17 - XY plane\n";
static char msg_g18[] PROGMEM = "Plane selection: G18 - XZ plane\n";
static char msg_g19[] PROGMEM = "Plane selection: G19 - YZ plane\n";
static PGM_P msgPlaneSelect[] PROGMEM = { msg_g17, msg_g18, msg_g19 };

static char msg_g90[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
static char msg_g91[] PROGMEM = "Distance mode:   G91 - incremental distance\n";
static PGM_P msgDistanceMode[] PROGMEM = { msg_g90, msg_g91 };

static char msg_g94[] PROGMEM = "Feed rate mode:  G94 - units per minute\n";
static char msg_g93[] PROGMEM = "Feed rate mode:  G93 - inverse time\n";
static PGM_P msgFeedRateMode[] PROGMEM = { msg_g94, msg_g93 };

static char msg_ms0[] PROGMEM = "Machine state:   Reset\n";
static char msg_ms1[] PROGMEM = "Machine state:   Run\n";
static char msg_ms2[] PROGMEM = "Machine state:   Stop\n";
static char msg_ms3[] PROGMEM = "Machine state:   Feedhold\n";
static char msg_ms4[] PROGMEM = "Machine state:   End Feedhold\n";
static char msg_ms5[] PROGMEM = "Machine state:   Homing\n";
static PGM_P msgMachineState[] PROGMEM = { msg_ms0, msg_ms1, msg_ms2, msg_ms3, msg_ms4, msg_ms5 };

void rpt_print_machine_state()
{
	double conversion = 1;
	char units[8] = "mm";

	if (gm.units_mode == INCHES_MODE) {
		conversion = (INCH_PER_MM);	
		strncpy(units,"inches", 8);
	}
	cm_get_model_position_vector(vector);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgUnitsMode[(gm.units_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgCoordSystem[(gm.coord_system)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMotionMode[(gm.motion_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgPlaneSelect[(gm.select_plane)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgDistanceMode[(gm.distance_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgFeedRateMode[(gm.inverse_feed_rate_mode)]));
	fprintf_P(stderr,(PGM_P)msg_Feed, gm.feed_rate * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosX, vector[X] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosY, vector[Y] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosZ, vector[Z] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosA, vector[A]);
	fprintf_P(stderr,(PGM_P)msg_PosB, vector[B]);
	fprintf_P(stderr,(PGM_P)msg_PosC, vector[C]);
	fprintf_P(stderr,(PGM_P)msg_OfsI, gm.arc_offset[0] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsJ, gm.arc_offset[1] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsK, gm.arc_offset[2] * conversion, units);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMachineState[(cm.machine_state)]));
//	fprintf_P(stderr,(PGM_P)msg_Limit, ls.min[X]);
}

/*
static void _print_rate_advisories(const int8_t axis, const char axis_char)
{
//	uint8_t motor = axis - CMD_MOTOR_BASE;

	double step_angle = _cfg_get_setting_value_by_key(axis, SA);
	double travel_rev = _cfg_get_setting_value_by_key(axis, TR);
	double seek_rate = _cfg_get_setting_value_by_key(axis, SR);
	double feed_rate = _cfg_get_setting_value_by_key(axis, FR);
	double seek_steps = (seek_rate / 60 / travel_rev) * (360 / step_angle);
	double feed_steps = (feed_rate / 60 / travel_rev) * (360 / step_angle);
	fprintf_P(stderr, PSTR("%c max seek: %5.0f steps/sec\n"), axis_char, seek_steps);
	fprintf_P(stderr, PSTR("%c max feed: %5.0f steps/sec\n"), axis_char, feed_steps);
	if (feed_rate > seek_rate) {
		fprintf_P(stderr, PSTR("You may be interested to know that the feed rate exceeds the seek rate\n"));
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
