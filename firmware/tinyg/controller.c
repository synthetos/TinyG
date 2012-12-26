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
#include <string.h>
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "config.h"				// #2
#include "controller.h"
#include "settings.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "arc.h"
#include "planner.h"
#include "report.h"
#include "system.h"
#include "gpio.h"
#include "help.h"
#include "xio/xio.h"

// local helpers
static void _controller_HSM(void);
static uint8_t _dispatch(void);
static void _text_response(const uint8_t status, const char *buf);

static uint8_t _shutdown_handler(void);
static uint8_t _reset_handler(void);
static uint8_t _feedhold_handler(void);
static uint8_t _cycle_start_handler(void);
static uint8_t _sync_to_tx_buffer(void);
static uint8_t _sync_to_planner(void);

/*
 * tg_init() - controller init
 */

void tg_init(uint8_t default_src) 
{
	cfg.fw_build = TINYG_BUILD_NUMBER;
	cfg.fw_version = TINYG_VERSION_NUMBER;
	cfg.hw_version = TINYG_HARDWARE_VERSION;

	tg.default_src = default_src;
	xio_set_stdin(tg.default_src);
	xio_set_stdout(tg.default_src);
	xio_set_stderr(STD_ERROR);
	tg_set_active_source(tg.default_src);	// set initial active source
}

/* 
 * tg_controller() - MAIN LOOP - top-level controller
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

void tg_controller() 
{ 
	while (true) { 
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == TG_EAGAIN) return; 
static void _controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
											// Order is important:
	DISPATCH(_reset_handler());				// 1. reset signal
	DISPATCH(_shutdown_handler());			// 2. limit switch has been thrown
	DISPATCH(_feedhold_handler());			// 3. feedhold signal
	DISPATCH(_cycle_start_handler());		// 4. cycle start signal

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(rpt_status_report_callback());	// conditionally send status report
	DISPATCH(rpt_queue_report_callback());	// conditionally send queue report
	DISPATCH(mp_plan_hold_callback());		// plan a feedhold
	DISPATCH(mp_end_hold_callback());		// end a feedhold
	DISPATCH(ar_arc_callback());			// arc generation runs behind lines
	DISPATCH(cm_homing_callback());			// G28.2 continuation

//----- command readers and parsers ------------------------------------//
	DISPATCH(_sync_to_planner());			// ensure there is at least one free buffer in planning queue
	DISPATCH(_sync_to_tx_buffer());			// sync with TX buffer (pseudo-blocking)
	DISPATCH(cfg_baud_rate_callback());		// perform baud rate update (must be after TX sync)
	DISPATCH(_dispatch());					// read and execute next command
}

/*
 * _shutdown_handler()
 *
 *	Shutdown is triggered by an active limit switch firing. This causes the 
 *	canonical machine to run the shutdown functions and set the machine state 
 *	to MACHINE_SHUTDOWN.
 *
 *	Once shutdown occurs the only thing this handler does is blink an LED 
 *	(spindle CW/CCW LED). The system can only be cleared by performing a reset.
 *
 *	This function returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) can be processed.
 */

#define LED_COUNTER 100000

static uint8_t _shutdown_handler(void)
{
//	if (sw.limit_thrown == false) return (TG_NOOP);
	if (gpio_get_limit_thrown() == false) return (TG_NOOP);

	// first time through perform the shutdown
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) {
		cm_shutdown();

	// after that just flash the LED
	} else {
		if (--tg.led_counter < 0) {
			tg.led_counter = LED_COUNTER;
			if (tg.led_state == 0) {
				gpio_led_on(INDICATOR_LED);
				tg.led_state = 1;
			} else {
				gpio_led_off(INDICATOR_LED);
				tg.led_state = 0;
			}
		}
	}
	return (TG_EAGAIN);	 // EAGAIN prevents any other actions from running
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
	tg.linelen = strlen(tg.in_buf)+1;
	cmd_new_body(cmd_body);					// clear the cmd body to get ready for use

	// dispatch the new text line
	switch (toupper(tg.in_buf[0])) {
//		case '^': { sig_reset(); break; }		// debug char for reset tests
//		case '@': { sig_feedhold(); break;}		// debug char for feedhold tests
//		case '#': { sig_cycle_start(); break;}	// debug char for cycle start tests

		case NUL: { 							// blank line (just a CR)
			if (cfg.comm_mode != TG_JSON_MODE) {
				_text_response(TG_OK, tg.in_buf);
			}
			break;
		}
		case 'H': { 							// intercept help screens
			cfg.comm_mode = TG_TEXT_MODE;
			print_general_help();
			_text_response(TG_OK, tg.in_buf);
			break;
		}
		case '$': case '?':{ 					// text-mode configs
			cfg.comm_mode = TG_TEXT_MODE;
			_text_response(cfg_text_parser(tg.in_buf), tg.in_buf);
			break;
		}
		case '{': { 							// JSON input
			cfg.comm_mode = TG_JSON_MODE;
			js_json_parser(tg.in_buf);
			break;
		}
		default: {								// anything else must be Gcode
			if (cfg.comm_mode != TG_JSON_MODE) {
				_text_response(gc_gcode_parser(tg.in_buf), tg.in_buf);
			} else {
				strncpy(tg.out_buf, tg.in_buf, INPUT_BUFFER_LEN);	// use output buffer as a temp
				sprintf(tg.in_buf,"{\"gc\":\"%s\"}\n", tg.out_buf);
				js_json_parser(tg.in_buf); 
			}
		}
	}
	return (TG_OK);
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
static const char msg_sc00[] PROGMEM = "OK";
static const char msg_sc01[] PROGMEM = "Error";
static const char msg_sc02[] PROGMEM = "Eagain";
static const char msg_sc03[] PROGMEM = "Noop";
static const char msg_sc04[] PROGMEM = "Complete";
static const char msg_sc05[] PROGMEM = "Terminated";
static const char msg_sc06[] PROGMEM = "Hard reset";
static const char msg_sc07[] PROGMEM = "End of line";
static const char msg_sc08[] PROGMEM = "End of file";
static const char msg_sc09[] PROGMEM = "File not open";
static const char msg_sc10[] PROGMEM = "Max file size exceeded";
static const char msg_sc11[] PROGMEM = "No such device";
static const char msg_sc12[] PROGMEM = "Buffer empty";
static const char msg_sc13[] PROGMEM = "Buffer full - fatal";
static const char msg_sc14[] PROGMEM = "Buffer full - non-fatal";
static const char msg_sc15[] PROGMEM = "Initializing";
static const char msg_sc16[] PROGMEM = "#16";
static const char msg_sc17[] PROGMEM = "#17";
static const char msg_sc18[] PROGMEM = "#18";
static const char msg_sc19[] PROGMEM = "#19";

static const char msg_sc20[] PROGMEM = "Internal error";
static const char msg_sc21[] PROGMEM = "Internal range error";
static const char msg_sc22[] PROGMEM = "Floating point error";
static const char msg_sc23[] PROGMEM = "Divide by zero";
static const char msg_sc24[] PROGMEM = "#24";
static const char msg_sc25[] PROGMEM = "#25";
static const char msg_sc26[] PROGMEM = "#26";
static const char msg_sc27[] PROGMEM = "#27";
static const char msg_sc28[] PROGMEM = "#28";
static const char msg_sc29[] PROGMEM = "#29";
static const char msg_sc30[] PROGMEM = "#30";
static const char msg_sc31[] PROGMEM = "#31";
static const char msg_sc32[] PROGMEM = "#32";
static const char msg_sc33[] PROGMEM = "#33";
static const char msg_sc34[] PROGMEM = "#34";
static const char msg_sc35[] PROGMEM = "#35";
static const char msg_sc36[] PROGMEM = "#36";
static const char msg_sc37[] PROGMEM = "#37";
static const char msg_sc38[] PROGMEM = "#38";
static const char msg_sc39[] PROGMEM = "#39";

static const char msg_sc40[] PROGMEM = "Unrecognized command";
static const char msg_sc41[] PROGMEM = "Expected command letter";
static const char msg_sc42[] PROGMEM = "Bad number format";
static const char msg_sc43[] PROGMEM = "Input exceeds max length";
static const char msg_sc44[] PROGMEM = "Input value too small";
static const char msg_sc45[] PROGMEM = "Input value too large";
static const char msg_sc46[] PROGMEM = "Input value range error";
static const char msg_sc47[] PROGMEM = "Input value unsupported";
static const char msg_sc48[] PROGMEM = "JSON syntax error";
static const char msg_sc49[] PROGMEM = "JSON input has too many pairs";
static const char msg_sc50[] PROGMEM = "Out of buffer space";
static const char msg_sc51[] PROGMEM = "#51";
static const char msg_sc52[] PROGMEM = "#52";
static const char msg_sc53[] PROGMEM = "#53";
static const char msg_sc54[] PROGMEM = "#54";
static const char msg_sc55[] PROGMEM = "#55";
static const char msg_sc56[] PROGMEM = "#56";
static const char msg_sc57[] PROGMEM = "#57";
static const char msg_sc58[] PROGMEM = "#58";
static const char msg_sc59[] PROGMEM = "#59";

static const char msg_sc60[] PROGMEM = "Zero length move";
static const char msg_sc61[] PROGMEM = "Gcode block skipped";
static const char msg_sc62[] PROGMEM = "Gcode input error";
static const char msg_sc63[] PROGMEM = "Gcode feedrate error";
static const char msg_sc64[] PROGMEM = "Gcode axis word missing";
static const char msg_sc65[] PROGMEM = "Gcode modal group violation";
static const char msg_sc66[] PROGMEM = "Homing cycle failed";
static const char msg_sc67[] PROGMEM = "Max travel exceeded";
static const char msg_sc68[] PROGMEM = "Max spindle speed exceeded";
static const char msg_sc69[] PROGMEM = "Arc specification error";

PGM_P const msgStatusMessage[] PROGMEM = {
	msg_sc00, msg_sc01, msg_sc02, msg_sc03, msg_sc04, msg_sc05, msg_sc06, msg_sc07, msg_sc08, msg_sc09,
	msg_sc10, msg_sc11, msg_sc12, msg_sc13, msg_sc14, msg_sc15, msg_sc16, msg_sc17, msg_sc18, msg_sc19,
	msg_sc20, msg_sc21, msg_sc22, msg_sc23, msg_sc24, msg_sc25, msg_sc26, msg_sc27, msg_sc28, msg_sc29,
	msg_sc30, msg_sc31, msg_sc32, msg_sc33, msg_sc34, msg_sc35, msg_sc36, msg_sc37, msg_sc38, msg_sc39,
	msg_sc40, msg_sc41, msg_sc42, msg_sc43, msg_sc44, msg_sc45, msg_sc46, msg_sc47, msg_sc48, msg_sc49,
	msg_sc50, msg_sc51, msg_sc52, msg_sc53, msg_sc54, msg_sc55, msg_sc56, msg_sc57, msg_sc58, msg_sc59,
	msg_sc60, msg_sc61, msg_sc62, msg_sc63, msg_sc64, msg_sc65, msg_sc66, msg_sc67, msg_sc68, msg_sc69
};


char *tg_get_status_message(uint8_t status, char *msg) 
{
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgStatusMessage[status]), STATUS_MESSAGE_LEN);
	return (msg);
}

/************************************************************************************
 * _text_response() - text mode responses
 *
 *	Outputs prompt, status and message strings
 */
static const char prompt_mm[] PROGMEM = "mm";
static const char prompt_in[] PROGMEM = "inch";
static const char prompt_ok[] PROGMEM = "tinyg [%S] ok> ";
static const char prompt_err[] PROGMEM = "tinyg [%S] error: %S %s\n";

static void _text_response(const uint8_t status, const char *buf)
{
	if (cfg.text_verbosity == TV_SILENT) return;	// skip all this

	// deliver the prompt
	const char *Units;		// becomes pointer to progmem string
	if (cm_get_units_mode() != INCHES) Units = (PGM_P)&prompt_mm;
	else Units = (PGM_P)&prompt_in;
	if ((status == TG_OK) || (status == TG_EAGAIN) || (status == TG_NOOP) || (status == TG_ZERO_LENGTH_MOVE)) {
		fprintf_P(stderr, (PGM_P)&prompt_ok, Units);
	} else {
		fprintf_P(stderr, (PGM_P)prompt_err, Units, (PGM_P)pgm_read_word(&msgStatusMessage[status]), buf);
	}

	// deliver echo and messages
	cmdObj *cmd = cmd_body;		// if there is a message it will aways be in the second object
	if ((cfg.text_verbosity >= TV_MESSAGES) && (cmd->token[0] == 'm')) {
		fprintf(stderr, "%s\n", cmd->string);
	}
}

/**** Application Messages *********************************************************
 * tg_print_message()        - print a character string passed as argument
 * tg_print_message_value()  - print a message with a value
 * tg_print_message_number() - print a canned message by number
 *
 * tg_print_loading_configs_message()
 * tg_print_initializing_message()
 * tg_print_system_ready_message()
 */

void tg_print_message(char *msg)
{
	cmd_add_string("msg", msg);
	cmd_print_list(TG_OK, TEXT_INLINE_VALUES);
}
/*
void tg_print_message_value(char *msg, double value)
{
	cmd_add_string("msg", msg);
	cmd_add_float("v", value);
	cmd_print_list(TG_OK, TEXT_INLINE_VALUES);
}
*/
/*
void tg_print_message_number(uint8_t msgnum) 
{
	char msg[APPLICATION_MESSAGE_LEN];
	strncpy_P(msg,(PGM_P)pgm_read_word(&msgApplicationMessage[msgnum]), APPLICATION_MESSAGE_LEN);
	tg_print_message(msg);
}
*/

void tg_print_loading_configs_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string("msg", "Loading configs from EEPROM");
	cmd_print_list(TG_INITIALIZING, TEXT_MULTILINE_FORMATTED);
#endif
}

void tg_print_initializing_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string("msg", INIT_CONFIGURATION_MESSAGE); // see settings.h & sub-headers
	cmd_print_list(TG_INITIALIZING, TEXT_MULTILINE_FORMATTED);
#endif
}

void tg_print_system_ready_message(void)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_add_object("fv");
	cmd_add_object("fb");
	cmd_add_string("msg", "SYSTEM READY");
	cmd_print_list(TG_OK, TEXT_MULTILINE_FORMATTED);
	_text_response(TG_OK, "");				// prompt
#endif
}

/**** Utilities ****
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 * tg_reset_source() - reset source to default input device (see note)
 * tg_set_active_source() - set current input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 *	be expanded to also set the stdout/stderr console device so the prompt
 *	and other messages are sent to the active device.
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
	if (mp_get_planner_buffers_available() == 0) { 
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

void tg_reset_source()
{
	tg_set_active_source(tg.default_src);
}

void tg_set_active_source(uint8_t dev)
{
	tg.src = dev;						// dev = XIO device #. See xio.h
}

/**** Signal handlers ****
 * _reset_handler()
 * _feedhold_handler()
 * _cycle_start_handler()
 */

static uint8_t _reset_handler(void)
{
	if (sig.sig_reset == false) { return (TG_NOOP);}
//	sig.sig_reset = false;				// why bother?
	tg_reset();							// hard reset - identical to hitting RESET button
	return (TG_EAGAIN);
}

static uint8_t _feedhold_handler(void)
{
	if (sig.sig_feedhold == false) { return (TG_NOOP);}
	sig.sig_feedhold = false;
	cm_feedhold();
	return (TG_EAGAIN);					// best to restart the control loop
}

static uint8_t _cycle_start_handler(void)
{
	if (sig.sig_cycle_start == false) { return (TG_NOOP);}
	sig.sig_cycle_start = false;
	cm_cycle_start();
	return (TG_EAGAIN);					// best to restart the control loop
}
