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

// local helpers
static void _controller_HSM(void);
static uint8_t _sync_to_tx_buffer(void);
static uint8_t _sync_to_planner(void);
static uint8_t _dispatch(void);
static void _dispatch_return(uint8_t status, char *buf);
static void _prompt_ok(void);
static void _prompt_error(uint8_t status, char *buf);
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
}

void tg_reset(void)
{
	mp_flush_planner();
	tg_system_reset();
	tg_application_reset();
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
void tg_controller() { while (TRUE) { _controller_HSM();}}
#define	DISPATCH(func) if (func == TG_EAGAIN) return; 
static void _controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
	DISPATCH(gpio_switch_handler());		// limit and homing switch handler
	DISPATCH(_abort_handler());				// abort signal
	DISPATCH(_feedhold_handler());			// feedhold signal
	DISPATCH(_cycle_start_handler());		// cycle start signal

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(rpt_status_report_callback());	// conditionally send status report
	DISPATCH(mp_plan_hold_callback());		// plan a feedhold 
	DISPATCH(mp_end_hold_callback());		// end a feedhold
	DISPATCH(ar_arc_callback());			// arc generation runs behind lines
	DISPATCH(cm_homing_callback());			// G28.1 continuation

//----- command readers and parsers ------------------------------------//
	DISPATCH(_sync_to_tx_buffer());			// sync with TX buffer (pseudo-blocking)
	DISPATCH(_sync_to_planner());			// sync with planning queue
	DISPATCH(_dispatch());					// read and execute next command
}

/**** Signal handlers ****
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

/**** Sync routines ****
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
		case 'H': { 							// intercept help screens
			cfg.communications_mode = TG_TEXT_MODE;
			help_print_general_help();
			_dispatch_return(TG_OK, tg.in_buf);
			break;
		}
		case '$': case '?':{ 					// text-mode config and query
			cfg.communications_mode = TG_TEXT_MODE;
			_dispatch_return(cfg_config_parser(tg.in_buf), tg.in_buf);
			break;
		}
		case '{': { 							// JSON input
			cfg.communications_mode = TG_JSON_MODE;
			_dispatch_return(js_json_parser(tg.in_buf, tg.out_buf), tg.out_buf); 
			break;
		}
		default: {								// anything else must be Gcode
			_dispatch_return(gc_gcode_parser(tg.in_buf), tg.in_buf);
		}
	}
	return (TG_OK);
}

void _dispatch_return(uint8_t status, char *buf)
{
	if (cfg.communications_mode == TG_JSON_MODE) {
		cmd_print_list(status, TEXT_INLINE_PAIRS);
		return;
	}
	switch (status) {
		case TG_OK: case TG_EAGAIN: case TG_NOOP: case TG_ZERO_LENGTH_MOVE:{ 
			_prompt_ok(); 
			break;
		}
		default: {
			_prompt_error(status, buf);
		}
	}
}

/**** System Prompts **************************************************************
 * tg_get_status_message()
 * _prompt_ok()
 * _prompt_error()
 */

/* These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 * Reference for putting display strings and string arrays in program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */
char msg_sc00[] PROGMEM = "OK";
char msg_sc01[] PROGMEM = "Error";
char msg_sc02[] PROGMEM = "Eagain";
char msg_sc03[] PROGMEM = "Noop";
char msg_sc04[] PROGMEM = "Complete";
char msg_sc05[] PROGMEM = "Terminated";
char msg_sc06[] PROGMEM = "Aborted";
char msg_sc07[] PROGMEM = "End of line";
char msg_sc08[] PROGMEM = "End of file";
char msg_sc09[] PROGMEM = "File not open";
char msg_sc10[] PROGMEM = "Max file size exceeded";
char msg_sc11[] PROGMEM = "No such device";
char msg_sc12[] PROGMEM = "Buffer empty";
char msg_sc13[] PROGMEM = "Buffer full - fatal";
char msg_sc14[] PROGMEM = "Buffer full - non-fatal";
char msg_sc15[] PROGMEM = "#15";
char msg_sc16[] PROGMEM = "#16";
char msg_sc17[] PROGMEM = "#17";
char msg_sc18[] PROGMEM = "#18";
char msg_sc19[] PROGMEM = "#19";

char msg_sc20[] PROGMEM = "Internal error";
char msg_sc21[] PROGMEM = "Internal range error";
char msg_sc22[] PROGMEM = "Floating point error";
char msg_sc23[] PROGMEM = "Divide by zero";
char msg_sc24[] PROGMEM = "#24";
char msg_sc25[] PROGMEM = "#25";
char msg_sc26[] PROGMEM = "#26";
char msg_sc27[] PROGMEM = "#27";
char msg_sc28[] PROGMEM = "#28";
char msg_sc29[] PROGMEM = "#29";
char msg_sc30[] PROGMEM = "#30";
char msg_sc31[] PROGMEM = "#31";
char msg_sc32[] PROGMEM = "#32";
char msg_sc33[] PROGMEM = "#33";
char msg_sc34[] PROGMEM = "#34";
char msg_sc35[] PROGMEM = "#35";
char msg_sc36[] PROGMEM = "#36";
char msg_sc37[] PROGMEM = "#37";
char msg_sc38[] PROGMEM = "#38";
char msg_sc39[] PROGMEM = "#39";

char msg_sc40[] PROGMEM = "Unrecognized command";
char msg_sc41[] PROGMEM = "Expected command letter";
char msg_sc42[] PROGMEM = "Bad number format";
char msg_sc43[] PROGMEM = "Input exceeds max length";
char msg_sc44[] PROGMEM = "Input value too small";
char msg_sc45[] PROGMEM = "Input value too large";
char msg_sc46[] PROGMEM = "Input value range error";
char msg_sc47[] PROGMEM = "Input value unsupported";
char msg_sc48[] PROGMEM = "JSON syntax error";
char msg_sc49[] PROGMEM = "JSON input has too many pairs";
char msg_sc50[] PROGMEM = "Out of buffer space";
char msg_sc51[] PROGMEM = "#51";
char msg_sc52[] PROGMEM = "#52";
char msg_sc53[] PROGMEM = "#53";
char msg_sc54[] PROGMEM = "#54";
char msg_sc55[] PROGMEM = "#55";
char msg_sc56[] PROGMEM = "#56";
char msg_sc57[] PROGMEM = "#57";
char msg_sc58[] PROGMEM = "#58";
char msg_sc59[] PROGMEM = "#59";

char msg_sc60[] PROGMEM = "Zero length move";
char msg_sc61[] PROGMEM = "Gcode block skipped";
char msg_sc62[] PROGMEM = "Gcode input error";
char msg_sc63[] PROGMEM = "Gcode feedrate error";
char msg_sc64[] PROGMEM = "Gcode axis word missing";
char msg_sc65[] PROGMEM = "Gcode modal group violation";
char msg_sc66[] PROGMEM = "Homing cycle failed";
char msg_sc67[] PROGMEM = "Max travel exceeded";
char msg_sc68[] PROGMEM = "Max spindle speed exceeded";
char msg_sc69[] PROGMEM = "Arc specification error";

PGM_P msgStatusMessage[] PROGMEM = {
	msg_sc00, msg_sc01, msg_sc02, msg_sc03, msg_sc04, msg_sc05, msg_sc06, msg_sc07, msg_sc08, msg_sc09,
	msg_sc10, msg_sc11, msg_sc12, msg_sc13, msg_sc14, msg_sc15, msg_sc16, msg_sc17, msg_sc18, msg_sc19,
	msg_sc20, msg_sc21, msg_sc22, msg_sc23, msg_sc24, msg_sc25, msg_sc26, msg_sc27, msg_sc28, msg_sc29,
	msg_sc30, msg_sc31, msg_sc32, msg_sc33, msg_sc34, msg_sc35, msg_sc36, msg_sc37, msg_sc38, msg_sc39,
	msg_sc40, msg_sc41, msg_sc42, msg_sc43, msg_sc44, msg_sc45, msg_sc46, msg_sc47, msg_sc48, msg_sc49,
	msg_sc50, msg_sc51, msg_sc52, msg_sc53, msg_sc54, msg_sc55, msg_sc56, msg_sc57, msg_sc58, msg_sc59,
	msg_sc60, msg_sc61, msg_sc62, msg_sc63, msg_sc64, msg_sc65, msg_sc66, msg_sc67, msg_sc68, msg_sc69
};

char prompt1[] PROGMEM = "tinyg";
char prompt_inch[] PROGMEM = "[inch] ok> ";
char prompt_mm[] PROGMEM = "[mm] ok> ";

char *tg_get_status_message(uint8_t status, char *msg) 
{
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgStatusMessage[status]), STATUS_MESSAGE_LEN);
	return (msg);
}

static void _prompt_ok()
{
	if (cm_get_units_mode() == INCHES) {
		fprintf_P(stderr, PSTR("%S%S"), prompt1, prompt_inch);
	} else {
		fprintf_P(stderr, PSTR("%S%S"), prompt1, prompt_mm);
	}
}

static void _prompt_error(uint8_t status, char *buf)
{
	fprintf_P(stderr, PSTR("error: %S: %s \n"),(PGM_P)pgm_read_word(&msgStatusMessage[status]),buf);
}

/**** Application Messages *********************************************************
 * tg_get_message() 		 - returns a canned application message in a pre-allocated string
 * tg_print_message()        - print a character string passed as argument
 * tg_print_message_number() - print a canned message by number
 * tg_print_configuration_profile()
 * tg_print_system_ready()
 *
 * Messages are collected in this one place to manage alternate display options
 * such as text mode or JSON mode.
 */

char msg_ap00[] PROGMEM = "<null message>";
char msg_ap01[] PROGMEM = "Loading configs from EEPROM";

PGM_P msgApplicationMessage[] PROGMEM = {
	msg_ap00, msg_ap01
};

void tg_print_message(char *msg)
{
	cmd_add_string("msg", msg);
	cmd_print_list(TG_OK, TEXT_INLINE_VALUES);
}

void tg_print_message_number(uint8_t msgnum) 
{
	char msg[APPLICATION_MESSAGE_LEN];
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgApplicationMessage[msgnum]), APPLICATION_MESSAGE_LEN);
	tg_print_message(msg);
}

void tg_print_configuration_profile(void)
{
	cmd_add_string("msg", INIT_CONFIGURATION_MESSAGE); // see settings.h & sub-headers
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);
}

void tg_print_system_ready(void)
{
	cmd_add_token("fv");
	cmd_add_token("fb");
	cmd_add_string("msg", "SYSTEM READY");
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);
}
