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
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"			// needed for stepper kill and terminate
#include "report.h"

#include "system.h"
#include "gpio.h"
#include "help.h"
#include "xio/xio.h"

#include <util/delay.h>			// debug

/* 
 * Canned gcode files for testing - enable only one of the U set
 * 	If you want to enable more than one of these you need to change the 
 *	name of the char[] array to something other than "gcode_file" and edit
 *	_tg_test_T or _tg_test_U to recognize it.
 */
// 'T' test
#include "gcode/gcode_startup_tests.h" // system tests and other assorted test code
// 'U' test
#include "gcode/gcode_test001.h"
//#include "gcode/gcode_star_1x1.h"
//#include "gcode/gcode_square_pocket.h"
//#include "gcode/gcode_reilly_111115.h"
//#include "gcode/PROFILE_zen_7x12_002.h"

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

static void _set_active_source(uint8_t dev);
static void _canned_startup(void);
static uint8_t _test_T(void);
static uint8_t _test_U(void);

/*
 * tg_init() - controller init
 * tg_reset() - application-level reset
 * tg_announce() - announce that TinyG is alive
 * tg_application_startup() - application start and restart
 *
 *	The controller init is split in two: the actual init, and tg_alive()
 *	which should be issued once the rest of he application is initialized.
 */

void tg_init(uint8_t default_src) 
{
	tg.version = TINYG_VERSION_NUMBER;
	tg.build = TINYG_BUILD_NUMBER;

	tg.default_src = default_src;
	xio_set_stdin(tg.default_src);
	xio_set_stdout(tg.default_src);
	xio_set_stderr(STD_ERROR);
	_set_active_source(tg.default_src);	// set initial active source
	tg.communications_mode = TG_TEXT_MODE;
}

void tg_reset(void)
{
	tg_application_init();		// also sets cm.machine_state = MACHINE_RESET;
//	tg_application_startup();	// application startup sequence
}

void tg_announce(void)
{
	fprintf_P(stderr, 
		PSTR("\n#### TinyG version %0.2f (build %0.2f) \"%s\" ####\n" ), 
		tg.version, tg.build, TINYG_VERSION_NAME);
}

void tg_ready(void)
{
	fprintf_P(stderr, PSTR("Type h for help\n"));
	_prompt_without_message();
}

void tg_application_startup(void)
{
	_canned_startup();			// pre-load input buffers (for test)
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
	while (TRUE) {
		_controller_HSM();
	}
}

static void _controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
	DISPATCH(gp_switch_handler());		// limit and homing switch handler
	DISPATCH(_abort_handler());
	DISPATCH(_feedhold_handler());
	DISPATCH(_cycle_start_handler());

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(sr_try_status_report());	// conditionally send status report
	DISPATCH(mp_plan_hold());			// plan a feedhold 
	DISPATCH(mp_end_hold());			// end a feedhold
	DISPATCH(ar_run_arc());				// arc generation runs behind lines
	DISPATCH(cm_homing_callback());		// G30 continuation
	DISPATCH(cm_return_to_home_callback());	// G28 continuation

//----- command readers and parsers ------------------------------------//
	DISPATCH(_sync_to_tx_buffer());		// sync with TX buffer (pseudo-blocking)
	DISPATCH(_sync_to_planner());		// sync with planning queue
	DISPATCH(_dispatch());				// read and execute next command
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
 * _dispatch() 			- dispatch line read from active input device
 * _dispatch_return()	- perform returns and prompting for commands
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control 
 *
 * Note:	Q is the feed_increment value for a peck drilling (G83) cycle. 
 *			So you might have to watch this if you implement peck drilling
 *			or expect to see a leading Q value in a CGode file/block.
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

		case 'T': { _test_T(); break;}			// run test file #1
		case 'U': { _test_U(); break;}			// run test file #2
//		case 'R': { tg_reset(); break;}

		case NUL: { 							// blank line (just a CR)
			_dispatch_return(TG_OK, tg.in_buf); 
			break;
		}

		case 'H': { 							// help screen
			help_print_general_help();
			_dispatch_return(TG_OK, tg.in_buf);
			break;
		}

		case '?': {								// queries
			tg.communications_mode = TG_TEXT_MODE;
			sr_print_machine_state();
			_dispatch_return(TG_OK, tg.in_buf);
			break; 
		}
		
		case '$': { 							// command-line input
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

	//	case 'G': case 'M': case 'N': case 'F': case 'Q': case '(': case '%': case '\\':
		default: {								// Gcode - is anything else
			if (tg.communications_mode == TG_JSON_MODE) {
				tg_json_gcode_return(gc_gcode_parser(tg.in_buf), tg.in_buf, tg.out_buf);
			} else {
				_dispatch_return(gc_gcode_parser(tg.in_buf), tg.out_buf);
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
			case TG_OK: case TG_EAGAIN: case TG_NOOP: { 
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

void tg_json_gcode_return(uint8_t status, char *in_buf, char *out_buf)
{
	cmdObj *cmd = cmd_array;

	cmd_new_object(cmd);						// parent gcode response
	sprintf_P(cmd->token, PSTR("gc"));
	cmd->value_type = VALUE_TYPE_PARENT;
	cmd++;

	cmd_new_object(cmd);						// child gcode string echo
	sprintf_P(cmd->token, PSTR("gc"));
	sprintf(cmd->string, in_buf);
	cmd->value_type = VALUE_TYPE_STRING;
	(cmd-1)->nx = cmd;
	cmd++;

	cmd_new_object(cmd);						// status as an integer
	sprintf_P(cmd->token, PSTR("st"));
	cmd->value = status;
	cmd->value_type = VALUE_TYPE_INT32;
	(cmd-1)->nx = cmd;
	cmd++;

	cmd_new_object(cmd);						// status as message
	sprintf_P(cmd->token, PSTR("msg"));
	tg_get_status_message(status, cmd->string);
	cmd->value_type = VALUE_TYPE_STRING;
	(cmd-1)->nx = cmd;

	js_make_json_string(cmd_array, out_buf);
	_dispatch_return(status, out_buf);
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
char msg_stat15[] PROGMEM = "Expected command letter";
char msg_stat16[] PROGMEM = "JSON sysntax error";
char msg_stat17[] PROGMEM = "Input exceeds max length";
char msg_stat18[] PROGMEM = "Output exceeds max length";
char msg_stat19[] PROGMEM = "Internal error";
char msg_stat20[] PROGMEM = "Bad number format";
char msg_stat21[] PROGMEM = "Floating point error";
char msg_stat22[] PROGMEM = "Arc specification error";
char msg_stat23[] PROGMEM = "Zero length line";
char msg_stat24[] PROGMEM = "Max travel exceeded";
char msg_stat25[] PROGMEM = "Max spindle speed exceeded";
PGM_P msgStatus[] PROGMEM = {	
	msg_stat00, msg_stat01, msg_stat02, msg_stat03, msg_stat04, 
	msg_stat05, msg_stat06, msg_stat07, msg_stat08, msg_stat09,
	msg_stat10, msg_stat11, msg_stat12, msg_stat13, msg_stat14, 
	msg_stat15, msg_stat16, msg_stat17, msg_stat18, msg_stat19,
	msg_stat20, msg_stat21, msg_stat22, msg_stat23, msg_stat24,
	msg_stat25
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
	if (cm_get_inches_mode() == true) {
		fprintf_P(stderr, PSTR("%S%S"), pr1, pr_in);
	} else {
		fprintf_P(stderr, PSTR("%S%S"), pr1, pr_mm);
	}
}

/**** Input source controls ****
 * tg_reset_source() - reset source to default input device (see note)
 * _tg_set_source()	 - set current input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 *	be expanded to also set the stdout/stderr console device so the prompt
 *	and other messages are sent to the active device.
 */

void tg_reset_source()
{
	_set_active_source(tg.default_src);
}

static void _set_active_source(uint8_t dev)
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
//	INFO(PSTR("ABORT"));				// handy for debugging
	tg_reset();							// stop all activity and reset
	return (TG_EAGAIN);					// best to restart the control loop
}

static uint8_t _feedhold_handler(void)
{
	if (sig.sig_feedhold == false) { return (TG_NOOP);}
	sig.sig_feedhold = false;
//	INFO(PSTR("Feedhold"));				// handy for debugging
	cm_feedhold();
	return (TG_EAGAIN);
}

static uint8_t _cycle_start_handler(void)
{
	if (sig.sig_cycle_start == FALSE) { return (TG_NOOP);}
	sig.sig_cycle_start = FALSE;
//	INFO(PSTR("Cycle Start"));			// handy for debugging
	cm_cycle_start();
	return (TG_EAGAIN);
}

/***** TEST ROUTINES *****
 * Various test routines
 * _test_T() - 'T' runs a test file from program memory
 * _test_U() - 'U' runs a different test file from program memory
 * _canned_startup() - loads input buffer at reset
 */

static uint8_t _test_T(void)
{
	xio_open_pgm(PGMFILE(&startup_tests)); 		// collected system tests
	_set_active_source(XIO_DEV_PGM);
	return (TG_OK);
}

static uint8_t _test_U(void)
{
	xio_open_pgm(PGMFILE(&gcode_file)); 		// defined by the .h enabled
	_set_active_source(XIO_DEV_PGM);
	return (TG_OK);
}

/*
 * Pre-load the USB RX (input) buffer with some test strings that 
 * will be called on startup. Be mindful of the char limit on the 
 * read buffer (RX_BUFFER_SIZE)
 */

static void _canned_startup()
{
#ifdef __CANNED_STARTUP

/**** RUN TEST FILE ON STARTUP ***
 * Uncomment both Q and T lines to run a test file on startup
 * Will run test file active in _tg_test_T()   (see above routine)
 * Also requires uncommenting  #define __CANNED_STARTUP in tinyg.h
 */

//	xio_queue_RX_string_usb("Q\n");				// exits back to test mode
//	xio_queue_RX_string_usb("U\n");				// run second test file
//	xio_queue_RX_string_usb("T\n");				// run first test file

/* Other command sequences */
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("R\n");				// run a homing cycle
//	xio_queue_RX_string_usb("^\n");				// abort
//	xio_queue_RX_string_usb("!\n");				// feedhold
//	xio_queue_RX_string_usb("~\n");				// cycle start
//	xio_queue_RX_string_usb("\n\n");			// 2 null lines

/* G0's */
//	xio_queue_RX_string_usb("g0 x0.2\n");		// shortest drawable line
//	xio_queue_RX_string_usb("g0 x0\n");
//	xio_queue_RX_string_usb("g0 x2\n");
//	xio_queue_RX_string_usb("g0 x3\n");
//	xio_queue_RX_string_usb("g0 y3\n");
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z10 a10\n");
//	xio_queue_RX_string_usb("g0 x2000 y3000 z4000 a5000\n");

/* G1's */
//	xio_queue_RX_string_usb("g1 f300 x100\n");
//	xio_queue_RX_string_usb("g1 f10 x100\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f0 x10\n");

/* G2/G3's */
//	xio_queue_RX_string_usb("g3 f500 x100 y100 z25 i50 j50\n");	// arcs
//	xio_queue_RX_string_usb("g2 f2000 x50 y50 z2 i25 j25\n");	// arcs
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i8 j8\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i5 j5\n");
//	xio_queue_RX_string_usb("g2 f300 x3 y3 i1.5 j1.5\n");

/* G4 tests (dwells) */
//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p0.1\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z-10\n");

/* G92 tests */
//	xio_queue_RX_string_usb("g0 x10 y10\n");
//	xio_queue_RX_string_usb("g92 x0 y0\n");
//	xio_queue_RX_string_usb("g92.1\n");
//	xio_queue_RX_string_usb("g0 x5\n");

/* G28 and G30 homing tests */
//	xio_queue_RX_string_usb("g28x0y0z0\n");
//	xio_queue_RX_string_usb("g30x0y0z0\n");
//	xio_queue_RX_string_usb("g30x42\n");

/* Feedhold tests */
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x1 y1 z1\n");
//	xio_queue_RX_string_usb("!");				// issue feedhold
//	xio_queue_RX_string_usb("~");				// end feedhold
//	xio_queue_RX_string_usb("g0 x0 y0 z0\n");

/*
	xio_queue_RX_string_usb("N1 T1M6\n");
	xio_queue_RX_string_usb("N2 G17\n");
	xio_queue_RX_string_usb("N3 G21 (mm)\n");
	xio_queue_RX_string_usb("N4 (S8000)\n");
	xio_queue_RX_string_usb("N5 (M3)\n");
	xio_queue_RX_string_usb("N6 G92X0.327Y-33.521Z-1.000\n");
	xio_queue_RX_string_usb("N7 G0Z4.000\n");
	xio_queue_RX_string_usb("N8 F600.0\n");
	xio_queue_RX_string_usb("N9 G1X0.327Y-33.521\n");
	xio_queue_RX_string_usb("N10 G1Z-1.000\n");
*/

//	xio_queue_RX_string_usb("N1 G92X0.327Y-33.521Z-1.000\n");
//	xio_queue_RX_string_usb("N2 G1X0.327Y-33.521F600.0\n");
/*	xio_queue_RX_string_usb("N3 X0.654Y-33.526\n");
	xio_queue_RX_string_usb("N4 X0.980Y-33.534\n");
	xio_queue_RX_string_usb("N5 X1.304Y-33.546\n");
	xio_queue_RX_string_usb("N6 X1.626Y-33.562\n");
	xio_queue_RX_string_usb("N7 X1.946Y-33.580\n");
	xio_queue_RX_string_usb("N8 X2.262Y-33.602\n");
//	xio_queue_RX_string_usb("@\n");
	xio_queue_RX_string_usb("N9 X2.574Y-33.628\n");
	xio_queue_RX_string_usb("N10 X2.882Y-33.656\n");
	xio_queue_RX_string_usb("N11 X3.185Y-33.688\n");
	xio_queue_RX_string_usb("N12 X3.483Y-33.724\n");
	xio_queue_RX_string_usb("N13 X3.775Y-33.762\n");
//	xio_queue_RX_string_usb("#\n");
	xio_queue_RX_string_usb("N14 X4.060Y-33.805\n");
	xio_queue_RX_string_usb("N15 X4.339Y-33.850\n");
	xio_queue_RX_string_usb("N16 X4.610Y-33.898\n");
	xio_queue_RX_string_usb("N17 X4.874Y-33.950\n");
	xio_queue_RX_string_usb("N18 X5.130Y-34.005\n");
	xio_queue_RX_string_usb("N19 X5.376Y-34.064\n");
	xio_queue_RX_string_usb("N20 X5.614Y-34.125\n");
	xio_queue_RX_string_usb("N21 X5.842Y-34.190\n");
	xio_queue_RX_string_usb("N22 X6.060Y-34.257\n");
	xio_queue_RX_string_usb("N23 X6.268Y-34.328\n");
	xio_queue_RX_string_usb("N24 X6.466Y-34.402\n");
	xio_queue_RX_string_usb("N25 X6.652Y-34.479\n");
	xio_queue_RX_string_usb("N26 X6.827Y-34.559\n");
	xio_queue_RX_string_usb("N27 X6.990Y-34.642\n");
	xio_queue_RX_string_usb("N28 X7.141Y-34.728\n");
	xio_queue_RX_string_usb("N29 X7.280Y-34.817\n");
	xio_queue_RX_string_usb("N30 X7.407Y-34.909\n");
*/

//	xio_queue_RX_string_usb("g0 x50\n");
//	xio_queue_RX_string_usb("@\n");				// issue feedhold
//	xio_queue_RX_string_usb("#\n");				// end feedhold
/*
	xio_queue_RX_string_usb("g1 f1000\n");
	xio_queue_RX_string_usb("x2\n");
	xio_queue_RX_string_usb("y2\n");
	xio_queue_RX_string_usb("@\n");				// issue feedhold
	xio_queue_RX_string_usb("x0\n");
	xio_queue_RX_string_usb("#\n");				// end feedhold
	xio_queue_RX_string_usb("y0\n");
*/
/*
	xio_queue_RX_string_usb("g1f1000\n");
	xio_queue_RX_string_usb("x2\n");
	xio_queue_RX_string_usb("y2\n");
	xio_queue_RX_string_usb("x0\n");
	xio_queue_RX_string_usb("y0\n");
	xio_queue_RX_string_usb("x2\n");
	xio_queue_RX_string_usb("y2\n");
	xio_queue_RX_string_usb("x0\n");
	xio_queue_RX_string_usb("y0\n");
	xio_queue_RX_string_usb("x2\n");
	xio_queue_RX_string_usb("y2\n");
	xio_queue_RX_string_usb("x0\n");
	xio_queue_RX_string_usb("y0\n");
	xio_queue_RX_string_usb("@\n");				// issue feedhold
	xio_queue_RX_string_usb("#\n");				// end feedhold
//	xio_queue_RX_string_usb("!");				// issue feedhold
//	xio_queue_RX_string_usb("~");				// end feedhold
*/

/* Configs and controls */
//	xio_queue_RX_string_usb("$\n");				// print general group
//	xio_queue_RX_string_usb("$x\n");			// print x axis
//	xio_queue_RX_string_usb("$n\n");			// print all axes
//	xio_queue_RX_string_usb("$xam\n");			// print x axis mode

//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("$1tr\n");

//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("$xjm 1000000\n");
//	xio_queue_RX_string_usb("$xvm\n");			// config with no data
//	xio_queue_RX_string_usb("$ja\n");			// config with no data
//	xio_queue_RX_string_usb("$aam = 3\n");		// set A to radius mode
//	xio_queue_RX_string_usb("$aam 10\n");		// set A to SLAVE_XYZ mode
//	xio_queue_RX_string_usb("$ XSS=1200\n");	// some settings tests
//	xio_queue_RX_string_usb("$xHt\n");
//	xio_queue_RX_string_usb("$x\n");

//	xio_queue_RX_string_usb("$xfr=1000\n");
//	xio_queue_RX_string_usb("$2mi=4\n");

//	xio_queue_RX_string_usb("$gp 64\n");		// path control modes
//	xio_queue_RX_string_usb("$gp 61\n");
//	xio_queue_RX_string_usb("$gp 61.1\n");
//	xio_queue_RX_string_usb("$ec 0\n");			// disable CR (LF only)
//	xio_queue_RX_string_usb("$x\n");
//	xio_queue_RX_string_usb("g20\n$xvm 40\n");	// set inch mode, set SR
//	xio_queue_RX_string_usb("(MSGtest message in comment)\n");
//	xio_queue_RX_string_usb("g18\n");			// plane select

//	xio_queue_RX_string_usb("g0x0.1\n");
//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("$xps\n");			// query x position

/* JSON tests 
 * If you want to use all these you need to set RX buffer to 1024 in xio_usart.h
 */

/*	xio_queue_RX_string_usb("{\"x_feedrate\":1200}\n");
	xio_queue_RX_string_usb("{\"xfr\":1200, \"yfr\":1201, \"zfr\":600}\n");
	xio_queue_RX_string_usb("{\"err_1\":36000}\n");
	xio_queue_RX_string_usb("{\"1sa\":3.6.000}\n");
	xio_queue_RX_string_usb("{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");
	xio_queue_RX_string_usb("{\"config_version\":null}\n");	// simple null test
	xio_queue_RX_string_usb("{\"config_profile\":true}\n");	// simple true test
	xio_queue_RX_string_usb("{\"prompt\":false}\n");		// simple false test
	xio_queue_RX_string_usb("{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");// string test w/comment
	xio_queue_RX_string_usb("{\"x_feedrate\":1200}\n");		// numeric test
	xio_queue_RX_string_usb("{\"y_feedrate\":-1456}\n");	// numeric test
	xio_queue_RX_string_usb("{\"Z_velocity_maximum\":null}\n");// axis w/null
	xio_queue_RX_string_usb("{\"m1_microsteps\":null}\n");	// motor w/null
	xio_queue_RX_string_usb("{\"2mi\":8}\n");				// motor token w/null
	xio_queue_RX_string_usb("{\"no-token\":12345}\n");		// non-token w/number
	xio_queue_RX_string_usb("{\"firmware_version\":329.26,		\"config_version\":0.93}\n");
	xio_queue_RX_string_usb("{\"1mi\":8, \"2mi\":8,\"3mi\":8,\"4mi\":8}\n");	// 4 elements
	xio_queue_RX_string_usb("{\"status_report\":{\"ln\":true, \"x_pos\":true, \"y_pos\":true, \"z_pos\":true}}\n");
	xio_queue_RX_string_usb("{\"parent_case1\":{\"child_null\":null}}\n");	// parent w/single child
	xio_queue_RX_string_usb("{\"parent_case2\":{\"child_num\":23456}}\n");	// parent w/single child
	xio_queue_RX_string_usb("{\"parent_case3\":{\"child_str\":\"stringdata\"}}\n");// parent w/single child
	xio_queue_RX_string_usb("{\"err_1\":36000x\n}");		// illegal number 
	xio_queue_RX_string_usb("{\"err_2\":\"text\n}");		// no string termination
	xio_queue_RX_string_usb("{\"err_3\":\"12345\",}\n");	// bad } termination
	xio_queue_RX_string_usb("{\"err_4\":\"12345\"\n");		// no } termination
*/
//	xio_queue_RX_string_usb("{\"x\":\"\"}\n");				// group display
//	xio_queue_RX_string_usb("{\"1\":\"\"}\n");				// group display
//	xio_queue_RX_string_usb("{\"g\":\"\"}\n");				// group display
//	xio_queue_RX_string_usb("{\"x\":null}\n");				// group display
//	xio_queue_RX_string_usb("{\"  xfr  \":null}\n");		// JSON string normalization tests
	xio_queue_RX_string_usb("{\"gcode\":\"G1 x100 (Title Case Comment)   \"}\n");
//	xio_queue_RX_string_usb("{\"sr\":{\"ln\":true,\"vl\":true,\"ms\":true}}\n"); // set status report
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");				// get status report
//	xio_queue_RX_string_usb("g0 x10\n");

#endif
}

/***** DEBUG routines *****/

#ifdef __DEBUG
void tg_dump_controller_state()
{
	fprintf_P(stderr, PSTR("*** Controller state: line:%5d, block:%5f  %s\n"),
		tg.linenum, tg.linecount, &tg.buf);
} 
#endif

