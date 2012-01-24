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
#include "planner.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "report.h"

/*****************************************************************************
 * Status Reports
 * sr_init()
 * sr_decr_status_report()	- decrement sr counter
 * sr_try_status_report() 	- send a status report if it's time to send one
 * sr_force_status_report() - force status report to send on next try attempt
 * sr_run_status_report() 	- send a status report
 */

static char st_ms0[] PROGMEM = "\"reset\"";		// used for status reports
static char st_ms1[] PROGMEM = "\"run\"";
static char st_ms2[] PROGMEM = "\"stop\"";
static char st_ms3[] PROGMEM = "\"hold\"";
static char st_ms4[] PROGMEM = "\"resume\"";
static char st_ms5[] PROGMEM = "\"homing\"";
static PGM_P stMachineState[] PROGMEM = { st_ms0, st_ms1, st_ms2, st_ms3, st_ms4, st_ms5 };

void sr_init() 
{ 
	cm.status_report_counter = cfg.status_report_interval; 
}

void sr_decr_status_report() 
{ 
	if (cm.status_report_counter != 0) {	// stick at zero
		cm.status_report_counter--; 
	}
}

void sr_force_status_report()
{
	cm.status_report_counter = 0;
}

uint8_t sr_try_status_report() 
{
	if ((cfg.status_report_enabled >= true) && (cm.status_report_counter == 0)) {
		sr_run_status_report();
		cm.status_report_counter = cfg.status_report_interval;
		return (TG_OK);
	}
	return (TG_NOOP);
}

void sr_run_status_report()
{
	mp_get_current_position_vector(vector);
	double velocity = mp_get_current_velocity();
	uint8_t distance_mode = 1;			// indicating mm mode

	if (gm.inches_mode == TRUE) {
		vector[X] = vector[X] * (INCH_PER_MM);
		vector[Y] = vector[Y] * (INCH_PER_MM);
		vector[Z] = vector[Z] * (INCH_PER_MM);
		velocity = velocity * (INCH_PER_MM);
		distance_mode = 0;
	}
	fprintf_P(stderr,PSTR("{\"sr\":{"));		// parent object
	fprintf_P(stderr,PSTR("\"ln\":%1.0f"), mp_get_current_linenum()); 	// notice it has no leading comma / space
	fprintf_P(stderr,PSTR(", \"xwp\":%1.4f"), vector[X]);
	fprintf_P(stderr,PSTR(", \"ywp\":%1.4f"), vector[Y]);
	fprintf_P(stderr,PSTR(", \"zwp\":%1.4f"), vector[Z]);

//	if (cfg.status_report_enabled > 1) {
//		fprintf_P(stderr,PSTR(", \"awp\":%1.4f"), vector[A]);
//		fprintf_P(stderr,PSTR(", \"bwp\":%1.4f"), vector[B]);
//		fprintf_P(stderr,PSTR(", \"cwp\":%1.4f"), vector[C]);
//	}
	fprintf_P(stderr,PSTR(", \"vel\":%5.2f"), velocity);
	fprintf_P(stderr,PSTR(", \"gu\":%d"), distance_mode);
	fprintf_P(stderr,PSTR(", \"ms\":%S"),(PGM_P)pgm_read_word(&stMachineState[cm.machine_state]));

	fprintf_P(stderr,PSTR("}}\n"));		// terminating curlies
}

/*
 * cm_print_machine_state()
 */

// Independent format strings - not in an array
static char msg_PosX[] PROGMEM = "Position X:   %8.3f %s\n";
static char msg_PosY[] PROGMEM = "Position Y:   %8.3f %s\n";
static char msg_PosZ[] PROGMEM = "Position Z:   %8.3f %s\n";
static char msg_PosA[] PROGMEM = "Position A:   %8.3f degrees\n";
static char msg_PosB[] PROGMEM = "Position B:   %8.3f degrees\n";
static char msg_PosC[] PROGMEM = "Position C:   %8.3f degrees\n";
static char msg_OfsI[] PROGMEM = "Offset I:     %8.3f %s\n";
static char msg_OfsJ[] PROGMEM = "Offset J:     %8.3f %s\n";
static char msg_OfsK[] PROGMEM = "Offset K:     %8.3f %s\n";
static char msg_Feed[] PROGMEM = "Feed Rate:    %8.3f %s \\ min\n";
//static char msg_Limit[] PROGMEM = "Limit Switches: %3.0f %s\n";

// Format strings with indexing arrays
static char msg_g21[] PROGMEM = "Units:           G21 - millimeter mode\n";
static char msg_g20[] PROGMEM = "Units:           G20 - inches mode\n";
static PGM_P msgUnitsMode[] PROGMEM = { msg_g21, msg_g20 };

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

static char msg_g91[] PROGMEM = "Distance mode:   G91 - incremental distance\n";
static char msg_g90[] PROGMEM = "Distance mode:   G90 - absolute distance\n";
static PGM_P msgDistanceMode[] PROGMEM = { msg_g91, msg_g90 };

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

void sr_print_machine_state()
{
	double conversion = 1;
	char units[8] = "mm";

	if (gm.inches_mode == true) {
		conversion = (INCH_PER_MM);	
		strncpy(units,"inches", 8);
	}
	mp_get_current_position_vector(vector);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgUnitsMode[(gm.inches_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMotionMode[(gm.motion_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgPlaneSelect[(gm.select_plane)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgDistanceMode[(gm.absolute_mode)]));
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgFeedRateMode[(gm.inverse_feed_rate_mode)]));
	fprintf_P(stderr,(PGM_P)msg_Feed, gm.feed_rate * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosX, vector[X] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosY, vector[Y] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosZ, vector[Z] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_PosA, vector[A]);
	fprintf_P(stderr,(PGM_P)msg_PosB, vector[B]);
	fprintf_P(stderr,(PGM_P)msg_PosC, vector[C]);
	fprintf_P(stderr,(PGM_P)msg_OfsI, gm.offset[0] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsJ, gm.offset[1] * conversion, units);
	fprintf_P(stderr,(PGM_P)msg_OfsK, gm.offset[2] * conversion, units);
	fprintf_P(stderr,(PGM_P)pgm_read_word(&msgMachineState[(cm.machine_state)]));
//	fprintf_P(stderr,(PGM_P)msg_Limit, ls.min[X]);
}

