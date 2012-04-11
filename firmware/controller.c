/*
 * controller.c - tinyg controller and top level parser
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
/* See tinyg_docs.txt for general notes. 
 * See tinyg_docs_developers.txt for coding details and explanations of:
 *	- basic controller operation and design notes
 *	- modeness (or not)
 *	- how to code continuations for use with the controller
 *
 * For the most current and complete info see:
 *	   http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info:
 *		(yes, the trailing ':' is required!)
 */

#include <ctype.h>				// for parsing
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "config.h"
#include "settings.h"
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"	// uses homing cycle
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"			// needed for stepper kill and terminate
#include "util.h"

#include "system.h"
#include "gpio.h"
#include "help.h"
#include "xio.h"

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

static void _tg_controller_HSM(void);
static uint8_t _tg_parser(char * buf);
static uint8_t _tg_run_prompt(void);
static uint8_t _tg_read_next_line(void);
static void _tg_prompt(void);
static void _tg_set_mode(uint8_t mode);
static void _tg_set_active_source(uint8_t dev);
static uint8_t _tg_kill_handler(void);
static uint8_t _tg_term_handler(void);
static uint8_t _tg_pause_handler(void);
static uint8_t _tg_resume_handler(void);
static uint8_t _tg_reset(void);
static uint8_t _tg_test_T(void);
static uint8_t _tg_test_U(void);
static void _tg_canned_startup(void);

/*
 * tg_init() - controller init
 * tg_alive() - announce that TinyG is alive
 * tg_application_startup() - application start and restart
 *
 *	The controller init is split in two: the actual init, and tg_alive()
 *	which should be issued once the rest of he application is initialized.
 */

void tg_init(uint8_t default_src) 
{
	tg.default_src = default_src;
	xio_set_stdin(tg.default_src);
	xio_set_stdout(tg.default_src);
	xio_set_stderr(STD_ERROR);
	_tg_set_active_source(tg.default_src);	// set initial active source
	_tg_set_mode(TG_GCODE_MODE);			// set initial operating mode
}

void tg_alive(void)
{
	fprintf_P(stderr, PSTR("#### "));
	tg_print_version_string();
	fprintf_P(stderr, PSTR(" ####\nType h for help\n"));
	_tg_prompt();
}

void tg_application_startup(void)
{
	tg.status = TG_OK;
	if (cfg.homing_mode == TRUE) { 	// conditionally run startup homing
		tg.status = cm_homing_cycle();
	}
	_tg_canned_startup();			// pre-load input buffers (for test)
}

/* 
 * tg_controller() - top-level controller
 *
 * The order of the dispatched tasks is very important. 
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon. 
 *
 * Tasks must be written as continuations as they will be called 
 * repeatedly, and are called even if they are not currently active. 
 *
 * The DISPATCH macro calls the function and returns to the controller 
 * parent if not finished (TG_EAGAIN), preventing later routines from 
 * running (they remain blocked). Any other condition - OK or ERR - 
 * drops through and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return TG_NOOP
 */
#define	DISPATCH(func) if (func == TG_EAGAIN) return; 

void tg_controller()
{
	while (TRUE) {
		_tg_controller_HSM();
	}
}

static void _tg_controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
	DISPATCH(sw_handler());			// limit and homing switch handler
	DISPATCH(_tg_kill_handler());	// complete processing of ENDs (M2)
	DISPATCH(_tg_term_handler());	// complete processing of ENDs (M2)
	DISPATCH(_tg_pause_handler());	// complete processing of STOPs
	DISPATCH(_tg_resume_handler());	// complete processing of STARTs

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(cm_try_status_report());// send status report
	DISPATCH(ar_run_arc());			// arc generation runs behind lines
	DISPATCH(cm_run_homing_cycle());// homing cycle

//----- command readers and parsers ------------------------------------//
	DISPATCH(_tg_run_prompt());		// manage sending command line prompt
	DISPATCH(_tg_read_next_line());	// read and execute next command
}

/* 
 * _tg_read_next_line() - non-blocking line read from active input device
 *
 *	Reads next command line and dispatches to currently active parser
 *	Manages various device and mode change conditions
 *	Also responsible for prompts and for flow control. 
 *	Accepts commands if the move queue has room - halts if it doesn't
 */

static uint8_t _tg_read_next_line()
{
	if (mp_test_write_buffer() == FALSE) { 	// got a buffer you can use?
		return (TG_EAGAIN);
	}
	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	if ((tg.status = xio_gets(tg.src, tg.buf, sizeof(tg.buf))) == TG_OK) {
		tg.status = _tg_parser(tg.buf);	// dispatch to active parser
		tg.prompted = FALSE;			// signals ready-for-next-line
	}
	if (tg.status == TG_QUIT) {			// handle case where parser detected QUIT
		_tg_set_mode(TG_TEST_MODE);
	}
	if (tg.status == TG_EOF) {			//(EOF can come from file devices only)
		fprintf_P(stderr, PSTR("End of command file\n"));
		tg_reset_source();				// reset to default src
	}
	// Note that TG_OK, TG_EAGAIN, TG_NOOP etc. will just flow through
	return (tg.status);
}

/* 
 * _tg_parser() - process top-level serial input 
 *
 *	tg_parser is the top-level of the input parser tree; dispatches other 
 *	parsers. Calls lower level parser based on mode
 *
 *	Keeps the system MODE, one of:
 *		- gcode mode
 *		- direct drive mode
 *		- test mode
 *
 *	In test mode it auto-detects mode by first character of input buffer
 *	Quits from a parser are handled by the controller (not individual parsers)
 *	Preserves and passes through return codes (status codes) from lower levels
 */

static uint8_t _tg_parser(char * buf)
{
	// auto-detect operating mode if not already set 
	if (tg.mode == TG_TEST_MODE) {
		switch (toupper(buf[0])) {
			case 'G': case 'M': case 'N': case 'F': case 'Q': 
			case '(': case '%': case '\\': case '$':
					  _tg_set_mode(TG_GCODE_MODE); break;
			case 'T': return (_tg_test_T());	// run whatever test u want
			case 'U': return (_tg_test_U());	// run 2nd test you want
			case 'R': return (_tg_reset());
		//	case 'D': _tg_set_mode(TG_DIRECT_DRIVE_MODE); break;
		//	case 'I': return (_tg_reserved());	// reserved
		//	case 'V': return (_tg_reserved());	// reserved
			case 'H': help_print_test_mode_help(); return (TG_OK);
			default:  _tg_set_mode(TG_TEST_MODE); break;
		}
	}
	// dispatch based on mode
	tg.status = TG_OK;
	switch (tg.mode) {
		case TG_GCODE_MODE: tg.status = gc_gcode_parser(buf); break;
	//	case TG_DIRECT_DRIVE_MODE: tg.status = dd_parser(buf); break;
	}
	return (tg.status);
}

/*
 * tg_reset_source()	Reset source to default input device (see note)
 * _tg_set_source()		Set current input source
 * _tg_set_mode()		Set current operating mode
 * _tg_reset()			Run power-up resets, including homing (table zero)
 *
 * Note: Once multiple serial devices are supported reset_source() should
 *	be expanded to also set the stdout/stderr console device so the prompt
 *	and other messages are sent to the active device.
 */

void tg_reset_source()
{
	_tg_set_active_source(tg.default_src);
}

static void _tg_set_active_source(uint8_t dev)
{
	tg.src = dev;							// dev = XIO device #. See xio.h
	if (tg.src == XIO_DEV_PGM) {
		tg.prompt_disabled = TRUE;
	} else {
		tg.prompt_disabled = FALSE;
	}
}

static void _tg_set_mode(uint8_t mode)
{
	tg.mode = mode;
}

static uint8_t _tg_reset(void)
{
	tg_application_startup();	// application startup sequence
	return (TG_OK);
}

/* 
 * _tg_run_prompt()		Conditionally display command line prompt
 * _tg_prompt() 		Display command line prompt
 *
 * We only want a prompt if the following conditions apply:
 *	- prompts are enabled (usually not enabled for direct-from-file reads)
 *	- system is ready for the next line of input
 *	- no prompt has been issued (issue only one)
 *
 * References for putting display strings in program memory:
 *	http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 *	http://johnsantic.com/comp/state.html, "Writing Efficient State Machines in C"
 */

static uint8_t _tg_run_prompt()
{
	if ((tg.prompt_disabled) || (tg.prompted)) { 
		return (TG_NOOP);			// exit w/continue if already prompted
	}
	_tg_prompt();
	return (TG_OK);
}

char tgModeStringGCode[] PROGMEM = "";	// put strings in program memory
char tgModeStringDumb[] PROGMEM = "DUMB";
char tgModeStringTest[] PROGMEM = "TEST"; 

PGM_P tgModeStrings[] PROGMEM = {	// put string pointer array in prog mem
	tgModeStringGCode,
	tgModeStringDumb,
	tgModeStringTest
};

static void _tg_prompt()
{
	fprintf_P(stderr, PSTR("tinyg%S"),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
	if (cm_get_inches_mode() == TRUE) {
		fprintf_P(stderr, PSTR("[inch] ok> "));
	} else {
		fprintf_P(stderr, PSTR("[mm] ok> "));
	}
//	fprintf_P(stderr, PSTR("ok\n"));	// send a grbl response
	tg.prompted = TRUE;					// set prompt state
}

/*
 * _tg_kill_handler()		Main loop signal handlers
 * _tg_term_handler()
 * _tg_pause_handler()
 * _tg_resume_handler()
 */

static uint8_t _tg_kill_handler(void)
{
	if (sig.sig_kill_flag == TRUE) { 
		sig.sig_kill_flag = FALSE;
		tg_reset_source();
		(void)cm_async_end();	// stop computing and generating motions
		return (TG_EAGAIN);		// best to restart the control loop
	} else {
		return (TG_NOOP);		
	}		
}

static uint8_t _tg_term_handler(void)
{
	return (_tg_kill_handler());
}

static uint8_t _tg_pause_handler(void)
{
	if (sig.sig_pause_flag == TRUE) { 
		sig.sig_pause_flag = FALSE;
		(void)cm_async_stop();
		return (TG_EAGAIN);
	} else {
		return (TG_NOOP);		
	}		
}

static uint8_t _tg_resume_handler(void)
{
	if (sig.sig_resume_flag == TRUE) {
		sig.sig_resume_flag = FALSE;
		(void)cm_async_start();
		return (TG_EAGAIN);
	} else {
		return (TG_NOOP);		
	}		
}

/*
 * tg_print_version_string()
 * 	see tinyg.h for TINYG_VERSION string
 */
void tg_print_version_string(void)
{
//	fprintf_P(stderr, PSTR("#### TinyG %S ####\n"), (PSTR(TINYG_VERSION)));
	fprintf_P(stderr, PSTR("TinyG %S"), (PSTR(TINYG_VERSION)));
}

/*
 * tg_print_status() - Send status message to stderr.
 */
char tgs00[] PROGMEM = "{00} OK";	// put strings in program memory
char tgs01[] PROGMEM = "{01} ERROR";
char tgs02[] PROGMEM = "{02} EAGAIN";
char tgs03[] PROGMEM = "{03} NOOP";
char tgs04[] PROGMEM = "{04} COMPLETE";
char tgs05[] PROGMEM = "{05} End of line";
char tgs06[] PROGMEM = "{06} End of file";
char tgs07[] PROGMEM = "{07} File not open";
char tgs08[] PROGMEM = "{08} Max file size exceeded";
char tgs09[] PROGMEM = "{09} No such device";
char tgs10[] PROGMEM = "{10} Buffer empty";
char tgs11[] PROGMEM = "{11} Buffer full - fatal";
char tgs12[] PROGMEM = "{12} Buffer full - non-fatal";
char tgs13[] PROGMEM = "{13} QUIT";
char tgs14[] PROGMEM = "{14} Unrecognized command";
char tgs15[] PROGMEM = "{15} Expected command letter";
char tgs16[] PROGMEM = "{16} Unsupported statement";
char tgs17[] PROGMEM = "{17} Input error";
char tgs18[] PROGMEM = "{18} Parameter not found";
char tgs19[] PROGMEM = "{19} Parameter under range";
char tgs20[] PROGMEM = "{20} Parameter over range";
char tgs21[] PROGMEM = "{21} Bad number format";
char tgs22[] PROGMEM = "{22} Floating point error";
char tgs23[] PROGMEM = "{23} Motion control error";
char tgs24[] PROGMEM = "{24} Arc specification error";
char tgs25[] PROGMEM = "{25} Zero length line";
char tgs26[] PROGMEM = "{26} Maximum feed rate exceeded";
char tgs27[] PROGMEM = "{27} Maximum seek rate exceeded";
char tgs28[] PROGMEM = "{28} Maximum table travel exceeded";
char tgs29[] PROGMEM = "{29} Maximum spindle speed exceeded";
char tgs30[] PROGMEM = "{30} Failed to converge";
char tgs31[] PROGMEM = "{31} Unused error string";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P tgStatus[] PROGMEM = {	
	tgs00, tgs01, tgs02, tgs03, tgs04, tgs05, tgs06, tgs07, tgs08, tgs09,
	tgs10, tgs11, tgs12, tgs13, tgs14, tgs15, tgs16, tgs17, tgs18, tgs19,
	tgs20, tgs21, tgs22, tgs23, tgs24, tgs25, tgs26, tgs27, tgs28, tgs29,
	tgs30, tgs31
};

void tg_print_status(const uint8_t status_code, const char *textbuf)
{
	switch (status_code) {		// don't send messages for these status codes
		case TG_OK: return;
		case TG_EAGAIN: return;
		case TG_NOOP: return;
		case TG_QUIT: return;
		case TG_ZERO_LENGTH_MOVE: return;
	}
	fprintf_P(stderr, PSTR("%S: %s\n"),(PGM_P)pgm_read_word(&tgStatus[status_code]), textbuf);
//	fprintf_P(stderr, PSTR("%S\n"),(PGM_P)pgm_read_word(&tgStatus[status_code])); // w/no text
}

/***** TEST ROUTINES *****
 * Various test routines
 * _tg_test_T() - 'T' runs a test file from program memory
 * _tg_test_U() - 'U' runs a different test file from program memory
 * _tg_canned_startup() - loads input buffer at reset
 */

static uint8_t _tg_test_T(void)
{
	xio_open_pgm(PGMFILE(&startup_tests)); 		// collected system tests
	_tg_set_active_source(XIO_DEV_PGM);
	_tg_set_mode(TG_GCODE_MODE);
	return (TG_OK);
}

static uint8_t _tg_test_U(void)
{
	xio_open_pgm(PGMFILE(&gcode_file)); 		// defined by the .h enabled
	_tg_set_active_source(XIO_DEV_PGM);
	_tg_set_mode(TG_GCODE_MODE);
	return (TG_OK);
}

/*
 * Pre-load the USB RX (input) buffer with some test strings that 
 * will be called on startup. Be mindful of the char limit on the 
 * read buffer (RX_BUFFER_SIZE)
 */

static void _tg_canned_startup()
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
//	xio_queue_RX_string_usb("!\n");				// stop
//	xio_queue_RX_string_usb("@\n");				// pause
//	xio_queue_RX_string_usb("%\n");				// resume

/* G0's */
//	xio_queue_RX_string_usb("g0 x0.2\n");		// shortest drawable line
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
//	xio_queue_RX_string_usb("g0 x5\n");

/* Configs and controls */
//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("$xjm6102\n");
//	xio_queue_RX_string_usb("$xsr\n");			// config with no data
//	xio_queue_RX_string_usb("$ja\n");			// config with no data
//	xio_queue_RX_string_usb("$amo3\n");			// set A to radius mode
//	xio_queue_RX_string_usb("$amo10\n");		// set A to SLAVE_XYZ mode
//	xio_queue_RX_string_usb("$arf1.2\n");		// set A rotary factor 
//	xio_queue_RX_string_usb("$ XSS=1200\n");	// some settings tests
//	xio_queue_RX_string_usb("$HM1\n");
//	xio_queue_RX_string_usb("$x\n");

//	xio_queue_RX_string_usb("$gp64\n");			// path control modes
//	xio_queue_RX_string_usb("$gp61\n");
//	xio_queue_RX_string_usb("$gp61.1\n");
//	xio_queue_RX_string_usb("$ec0\n");			// disable CR (LF only)
//	xio_queue_RX_string_usb("$x\n");
//	xio_queue_RX_string_usb("g20\n$xsr40\n");	// set inch mode, set SR
//	xio_queue_RX_string_usb("(MSGtest message in comment)\n");
//	xio_queue_RX_string_usb("g18\n");			// plane select

#endif
}

/***** DEBUG routines *****/

#ifdef __DEBUG
void tg_dump_controller_state()
{
	fprintf_P(stderr, PSTR("*** Controller state: line:%5f, block:%5f  %s\n"),
		tg.linenumber, tg.linecount, &tg.buf);
} 
#endif

