/*
 * controller.c - tinyg controller and top level parser
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
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
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <ctype.h>				// for parsing
#include <string.h>				// for memset
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "util.h"				// #2
#include "config.h"				// #3
#include "controller.h"
#include "settings.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"	// uses homing cycle
#include "arc.h"
#include "planner.h"
#include "stepper.h"			// needed for stepper kill and terminate
#include "report.h"
#include "test.h"

#include "system.h"
#include "gpio.h"
#include "help.h"
#include "xio/xio.h"

#include <util/delay.h>			// debug


static void _controller_HSM(void);
static uint8_t _sync_to_tx_buffer(void);
static uint8_t _sync_to_planner(void);
static uint8_t _dispatch(void);
static void _dispatch_return(uint8_t status, char *buf);
static void _prompt_without_message(void);
static void _prompt_with_message(uint8_t status, char *buf);

static uint8_t _abort_handler(void);
static uint8_t _feedhold_handler(void);
static uint8_t _cycle_start_handler(void);

/*
 * tg_init() - controller init
 * tg_reset() - application-level reset
 * tg_announce() - announce that TinyG is alive
 * tg_ready() - final part of announcement - syste is ready fo input
 * tg_application_startup() - application start and restart
 *
 *	The controller init is split in two: the actual init, and tg_alive()
 *	which should be issued once the rest of the application is initialized.
 */

void tg_init(uint8_t default_src) 
{
	tg.version = TINYG_VERSION_NUMBER;
	tg.build = TINYG_BUILD_NUMBER;

	tg.default_src = default_src;
	xio_set_stdin(tg.default_src);
	xio_set_stdout(tg.default_src);
	xio_set_stderr(STD_ERROR);
	tg_set_active_source(tg.default_src);	// set initial active source
	tg.communications_mode = TG_TEXT_MODE;
}

void tg_reset(void)
{
	mp_flush_planner();
	tg_system_reset();
	tg_application_reset();
}

void tg_announce(void)
{
	fprintf_P(stderr, PSTR("#### TinyG version %0.2f (build %0.2f) \"%s\" ####\n" ), 
		tg.version, tg.build, TINYG_VERSION_NAME);
//	fprintf_P(stderr,PSTR("#### %s Profile ####\n"), TINYG_CONFIGURATION_PROFILE);
}

void tg_ready(void)
{
	fprintf_P(stderr, PSTR("Type h for help\n"));
	_prompt_without_message();
}

void tg_application_startup(void)
{
	tg_canned_startup();			// pre-load input buffers (for test)
}

/* 
 * tg_controller() - top-level controller
 *
 * The order of the dispatched tasks is very important. 
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon. 
 *
 * Tasks must be written as continuations as they will be called repeatedly, 
 * and are called even if they are not currently active. 
 *
 * The DISPATCH macro calls the function and returns to the controller parent 
 * if not finished (TG_EAGAIN), preventing later routines from running 
 * (they remain blocked). Any other condition - OK or ERR - drops through 
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return TG_NOOP
 *
 * Useful reference on state machines:
 * http://johnsantic.com/comp/state.html, "Writing Efficient State Machines in C"
 */
#define	DISPATCH(func) if (func == TG_EAGAIN) return; 

void tg_controller()
{
	while (TRUE) { _controller_HSM();}
}

static void _controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
	DISPATCH(gpio_switch_handler());	// limit and homing switch handler
	DISPATCH(_abort_handler());
	DISPATCH(_feedhold_handler());
	DISPATCH(_cycle_start_handler());

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(rpt_status_report_callback());	// conditionally send status report
	DISPATCH(mp_plan_hold_callback());		// plan a feedhold 
	DISPATCH(mp_end_hold_callback());		// end a feedhold
	DISPATCH(ar_arc_callback());			// arc generation runs behind lines
	DISPATCH(cm_homing_callback());			// G28.1 continuation

//----- command readers and parsers ------------------------------------//
	DISPATCH(_sync_to_tx_buffer());		// sync with TX buffer (pseudo-blocking)
	DISPATCH(_sync_to_planner());		// sync with planning queue
	DISPATCH(_dispatch());				// read and execute next command
}

/***************************************************************************** 
 * _dispatch() 			- dispatch line read from active input device
 * _dispatch_return()	- perform returns and prompting for commands
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control 
 */

static uint8_t _dispatch()
{
	uint8_t status;

	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	if ((status = xio_gets(tg.src, tg.in_buf, sizeof(tg.in_buf))) != TG_OK) {
		if (status == TG_EOF) {					// EOF can come from file devices only
			fprintf_P(stderr, PSTR("End of command file\n"));
			tg_reset_source();					// reset to default source
		}
		// Note that TG_EAGAIN, TG_NOOP etc. will just flow through
		return (status);
	}

	// dispatch the new text line
	switch (toupper(tg.in_buf[0])) {
//		case '^': { sig_abort(); break; }		// debug char for abort tests
//		case '@': { sig_feedhold(); break;}		// debug char for feedhold tests
//		case '#': { sig_cycle_start(); break;}	// debug char for cycle start tests

		case NUL: { 							// blank line (just a CR)
			_dispatch_return(TG_OK, tg.in_buf); 
			break;
		}
		case 'H': { 							// help screen
			help_print_general_help();
			_dispatch_return(TG_OK, tg.in_buf);
			break;
		}
		case '$': case '?':{ 					// text-mode config and query
			if (tg.communications_mode != TG_GRBL_MODE) {
				tg.communications_mode = TG_TEXT_MODE;
			}
			_dispatch_return(cfg_config_parser(tg.in_buf), tg.in_buf);
			break;
		}
		case '{': { 							// JSON input
			tg.communications_mode = TG_JSON_MODE;
			_dispatch_return(js_json_parser(tg.in_buf, tg.out_buf), tg.out_buf); 
			break;
		}
		default: {								// Gcode - is anything else
			if (tg.communications_mode == TG_JSON_MODE) {
				tg_make_json_gcode_response(gc_gcode_parser(tg.in_buf), tg.in_buf, tg.out_buf);
				_dispatch_return(NUL, tg.out_buf);	// status is ignored
			} else {
				_dispatch_return(gc_gcode_parser(tg.in_buf), tg.in_buf);
			}
		}
	}
	return (TG_OK);
}

void _dispatch_return(uint8_t status, char *buf)
{
	if (tg.communications_mode == TG_JSON_MODE) {
		fprintf(stderr, "%s", buf);
		return;
	}
	if (tg.communications_mode == TG_GRBL_MODE) {
		if (status == TG_OK) {
			fprintf_P(stderr, PSTR("ok"));
		} else {
			fprintf_P(stderr, PSTR("err"));
		}
		return;
	} 
	if (tg.communications_mode == TG_TEXT_MODE) {
		// for these status codes just send a prompt 
		switch (status) {
			case TG_OK: case TG_EAGAIN: case TG_NOOP: case TG_ZERO_LENGTH_MOVE:{ 
				_prompt_without_message(); 
				break; 
			}
			default: { 	// for everything else
				_prompt_with_message(status, buf); 
				break;
			}
		}
	}
}

/* 
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 */

static uint8_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

static uint8_t _sync_to_planner()
{
	if (mp_test_write_buffer() == FALSE) { 		// got a buffer you can use?
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

/*
 * tg_make_json_gcode_response() - generate JSON response object for Gcode
 */
void tg_make_json_gcode_response(uint8_t status, char *block, char *out_buf)
{
	cmdObj *cmd = cmd_array;

	cmd_new_object(cmd);						// parent gcode response
	sprintf_P(cmd->token, PSTR("gc"));
	cmd->value_type = VALUE_TYPE_PARENT;
	cmd++;

	cmd_new_object(cmd);						// child gcode string echo
	sprintf_P(cmd->token, PSTR("gc"));
	sprintf(cmd->string_value, block);
	cmd->value_type = VALUE_TYPE_STRING;
	(cmd-1)->nx = cmd;
	cmd++;

	cmd_new_object(cmd);						// status as an integer
	sprintf_P(cmd->token, PSTR("st"));
	cmd->value = status;
	cmd->value_type = VALUE_TYPE_INTEGER;
	(cmd-1)->nx = cmd;
	cmd++;

	cmd_new_object(cmd);						// status as message
	sprintf_P(cmd->token, PSTR("msg"));
	tg_get_status_message(status, cmd->string_value);
	cmd->value_type = VALUE_TYPE_STRING;
	(cmd-1)->nx = cmd;

	js_make_json_string(cmd_array, out_buf);
}

/**** Prompting **************************************************************
 * tg_get_status_message()
 * _prompt_with_message()
 * _prompt_without_message()
 *
 *	Handles response formatting and prompt generation.
 *	Aware of communications mode: COMMAND_LINE_MODE, JSON_MODE, GRBL_MODE
 */

/* The number of elements in the indexing array must match the # of strings
 * Reference for putting display strings and string arrays in program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */
char msg_stat00[] PROGMEM = "OK";
char msg_stat01[] PROGMEM = "Error";
char msg_stat02[] PROGMEM = "Eagain";
char msg_stat03[] PROGMEM = "Noop";
char msg_stat04[] PROGMEM = "Complete";
char msg_stat05[] PROGMEM = "End of line";
char msg_stat06[] PROGMEM = "End of file";
char msg_stat07[] PROGMEM = "File not open";
char msg_stat08[] PROGMEM = "Max file size exceeded";
char msg_stat09[] PROGMEM = "No such device";
char msg_stat10[] PROGMEM = "Buffer empty";
char msg_stat11[] PROGMEM = "Buffer full - fatal";
char msg_stat12[] PROGMEM = "Buffer full - non-fatal";
char msg_stat13[] PROGMEM = "Quit";
char msg_stat14[] PROGMEM = "Unrecognized command";
char msg_stat15[] PROGMEM = "Number range error";
char msg_stat16[] PROGMEM = "Expected command letter";
char msg_stat17[] PROGMEM = "JSON sysntax error";
char msg_stat18[] PROGMEM = "Input exceeds max length";
char msg_stat19[] PROGMEM = "Output exceeds max length";
char msg_stat20[] PROGMEM = "Internal error";
char msg_stat21[] PROGMEM = "Bad number format";
char msg_stat22[] PROGMEM = "Floating point error";
char msg_stat23[] PROGMEM = "Arc specification error";
char msg_stat24[] PROGMEM = "Zero length line";
char msg_stat25[] PROGMEM = "Gcode block skipped";
char msg_stat26[] PROGMEM = "Gcode input error";
char msg_stat27[] PROGMEM = "Gcode feedrate error";
char msg_stat28[] PROGMEM = "Gcode axis word missing";
char msg_stat29[] PROGMEM = "Gcode modal group violation";
char msg_stat30[] PROGMEM = "Homing cycle failed";
char msg_stat31[] PROGMEM = "Max travel exceeded";
char msg_stat32[] PROGMEM = "Max spindle speed exceeded";
PGM_P msgStatus[] PROGMEM = {	
	msg_stat00, msg_stat01, msg_stat02, msg_stat03, msg_stat04, 
	msg_stat05, msg_stat06, msg_stat07, msg_stat08, msg_stat09,
	msg_stat10, msg_stat11, msg_stat12, msg_stat13, msg_stat14, 
	msg_stat15, msg_stat16, msg_stat17, msg_stat18, msg_stat19,
	msg_stat20, msg_stat21, msg_stat22, msg_stat23, msg_stat24,
	msg_stat25, msg_stat26, msg_stat27, msg_stat28, msg_stat29,
	msg_stat30, msg_stat31, msg_stat32
};

char pr1[] PROGMEM = "tinyg";
char pr_in[] PROGMEM = "[inch] ok> ";
char pr_mm[] PROGMEM = "[mm] ok> ";

char *tg_get_status_message(uint8_t status, char *msg) 
{
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgStatus[status]), STATUS_MESSAGE_LEN);
	return (msg);
}

static void _prompt_with_message(uint8_t status, char *buf)
{
	fprintf_P(stderr, PSTR("%S: %s \n"),(PGM_P)pgm_read_word(&msgStatus[status]),buf);
	_prompt_without_message();
}

static void _prompt_without_message()
{
	if (cm_get_units_mode() == INCHES) {
		fprintf_P(stderr, PSTR("%S%S"), pr1, pr_in);
	} else {
		fprintf_P(stderr, PSTR("%S%S"), pr1, pr_mm);
	}

//##################### BEGIN DIAGNOSTIC #######################
	fprintf_P(stderr,PSTR("Programmed X =[%1.3f] "),cm_get_runtime_work_position(X));
// 100 is step/mm=( 360*micro_step/(step angle * travel_per_revolution)) 360*1/(0.72*5)=100
	double x_factor = (360 * cfg.m[X].microsteps / (cfg.m[X].step_angle * cfg.m[X].travel_rev));
	fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),x_cnt/x_factor); 
//	fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),x_cnt/100); 

	fprintf_P(stderr,PSTR("Programmed Y =[%1.3f] "),cm_get_runtime_work_position(Y));
	double y_factor = (360 * cfg.m[Y].microsteps / (cfg.m[Y].step_angle * cfg.m[Y].travel_rev));
	fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),y_cnt/y_factor); 
//	fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),y_cnt/100); 

    fprintf_P(stderr,PSTR("Programmed Z =[%1.3f] "),cm_get_runtime_work_position(Z));
	double z_factor = (360 * cfg.m[Z].microsteps / (cfg.m[Z].step_angle * cfg.m[Z].travel_rev));
    fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),z_cnt/z_factor); 
//    fprintf_P(stderr,PSTR("Real=[%1.3f] mm\n"),z_cnt/100); 

//##################### EMD DIAGNOSTIC #######################

}

/**** Input source controls ****
 * tg_reset_source() - reset source to default input device (see note)
 * tg_set_active_source() - set current input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 *	be expanded to also set the stdout/stderr console device so the prompt
 *	and other messages are sent to the active device.
 */

void tg_reset_source()
{
	tg_set_active_source(tg.default_src);
}

void tg_set_active_source(uint8_t dev)
{
	tg.src = dev;							// dev = XIO device #. See xio.h
	if (tg.src == XIO_DEV_PGM) {
		tg.prompt_enabled = false;
	} else {
		tg.prompt_enabled = true;
	}
}

/**** Main loop signal handlers ****
 * _abort_handler()
 * _feedhold_handler()
 * _cycle_start_handler()
 */

static uint8_t _abort_handler(void)
{
	if (sig.sig_abort == false) { return (TG_NOOP);}
	sig.sig_abort = false;
	tg_reset();							// stop all activity and reset
	return (TG_EAGAIN);					// best to restart the control loop
}

static uint8_t _feedhold_handler(void)
{
	if (sig.sig_feedhold == false) { return (TG_NOOP);}
	sig.sig_feedhold = false;
	cm_feedhold();
	return (TG_EAGAIN);
}

static uint8_t _cycle_start_handler(void)
{
	if (sig.sig_cycle_start == FALSE) { return (TG_NOOP);}
	sig.sig_cycle_start = FALSE;
	cm_cycle_start();
	return (TG_EAGAIN);
}
