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
 */
/* ---- Controller Operation ----
 *
 *	The controller provides a simple process control scheme to manage 
 *	blocking of multiple "threads" in the application. The controller 
 *	is an event-driven hierarchical state machine (HSM) using inverted
 *	control to manage a set of cooperative run-to-completion kernel tasks.
 * 	(ref: http://www.state-machine.com/products)
 *
 *	More simply, it works as a set of aborting "superloops", one superloop
 *	per hierarchical state machine (or thread - sort of). Within each HSM
 *	the highest priority tasks are run first and progressively lower 
 *	priority tasks are run only if the higher priority tasks are not 
 *	blocked. No task ever actually blocks, but instead returns "busy" 
 *	(eagain) when it would ordinarily block. It must also provide a 
 *	re-entry point to resume the task once the blocking condition has 
 *	been removed.
 *
 *	For this scheme to work tasks must be written to run-to-completion 
 *	(non-blocking), and must offer re-entry points (continuations) to 
 *	resume operations that would have blocked (see line generator for an 
 *	example). A task returns TG_EAGAIN to indicate a blocking point. If 
 *	TG_EAGAIN is received the controller quits the loop (HSM) and starts 
 *	the next one in the round-robin (all HSMs are round robined).  
 *	Any other return code allows the controller to proceed down the task 
 *	list. See end notes in this file for how to write a continuation.
 *
 *	Interrupts run at the highest priority levels; kernel tasks are 
 *	organized into priority groups below the interrupt levels. The 
 *	priority of operations is:
 *
 *	- High priority ISRs
 *		- issue steps to motors / count dwell timings
 *		- dequeue and load next stepper move
 *
 *	- Medium priority ISRs
 *		- receive serial input (RX)
 *		- execute signals received by serial input
 *		- detect and flag limit switch closures
 *
 *	- Low priority ISRs
 *		- send serial output (TX)
 *
 *	- Main loop tasks
 *		These are divided up into layers depending on priority and blocking
 *		hierarchy. See tg_controller() for details.
 *
 *	Notes:
 *	  - Gcode and other command line flow control is managed cooperatively
 *		with the application sending Gcode or other commands. The 'ok' 
 *		chars in the prompt indicate that the controller is ready for the 
 *		next line. The sending app is supposed to honor this and not stuff
 *		lines down the pipe (which will choke the controller).
 *
 *	Futures: Using a super loop instead of an event system is a design 
 *	tradoff - or more to the point - a hack. If the flow of control gets 
 *	much more complicated it will make sense to replace this section 
 *	with an event driven dispatcher.
 */
/* ---- Modedness ----
 *
 *	TinyG appears to the command-line user as bing non-moded. 
 *	However, this is not entirely true. Separate modes exist for 
 *	entering test modes, and for reserved modes such as dumb mode 
 *	(direct drive) and other parsers that are planned.
 *
 *	To exit any mode hit Q as the first character of the command line
 *
 *	To re-enter Gcode or any other mode hit any of the following chars: 
 *		G,M,N,F,%,(	enter GCODE_MODE and perform that action
 *		D			enter DUMB mode (direct drive)
 *		T			execute primary test (whatever you link into it)
 *		U			execute secondary test (whatever you link into it)
 *		H			help screen (returns to TEST mode)
 *		I			<reserved>
 *		V			<reserved>
 *
 *	Once in the selected mode these characters are not active as mode 
 *	selects. 
 */

#include <ctype.h>				// for parsing
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h

#include "system.h"
#include "xio.h"
#include "tinyg.h"
#include "controller.h"
#include "gcode.h"				// calls out to gcode parser, etc.
#include "config.h"
#include "canonical_machine.h"	// uses homing cycle
#include "planner.h"
#include "direct_drive.h"
#include "stepper.h"			// needed for stepper kill and terminate
#include "limit_switches.h"

#include "encoder.h"			// for debug (en_toggle())
#include <util/delay.h>			// debug

/*
 * Canned gcode files for testing
 */

//#include "gcode/gcode_tests.h"		// system tests and other assorted test code
//#include "gcode/gcode_zoetrope.h"	// zoetrope moves. makes really cool sounds
//#include "gcode/gcode_contraptor_circle.h"
#include "gcode/gcode_mudflap.h"
//#include "gcode/gcode_braid2d.h"
//#include "gcode/gcode_hacdc.h"

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
static void _tg_print_test_help_screen(void);

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
{								// see tinyg.h for TINYG_VERSION string
	fprintf_P(stderr, PSTR("#### TinyG %S ####\nType h for help\n"), (PSTR(TINYG_VERSION)));
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
 * repeatedly, and often called even if they are not currently active. 
 * See end notes in this file for how to code continuations.
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
	DISPATCH(ls_handler());			// limit switch handler
	DISPATCH(_tg_kill_handler());	// complete processing of ENDs (M2)
	DISPATCH(_tg_term_handler());	// complete processing of ENDs (M2)
	DISPATCH(_tg_pause_handler());	// complete processing of STOPs
	DISPATCH(_tg_resume_handler());	// complete processing of STARTs

//----- low-level motor control ----------------------------------------//
	mp_move_dispatcher();			// run current or next move in queue

//----- machine cycles -------------------------------------------------//
	DISPATCH(cm_run_homing_cycle());// homing cycle

//----- command readers and parsers ------------------------------------//
	DISPATCH(_tg_run_prompt());		// manage prompts
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
	// test if it's OK to read the next line
	if (mp_check_for_write_buffers(MP_BUFFERS_NEEDED) == FALSE) {
		return (TG_EAGAIN);				// exit w/busy if not enough buffers
	}
	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	if ((tg.status = xio_gets(tg.src, tg.buf, sizeof(tg.buf))) == TG_OK) {
#ifdef __dbECHO_INPUT_LINE
		fprintf_P(stderr, PSTR("Got input line %s\n"), tg.buf);
#endif
//		_delay_ms(100);		// +++++ statement to debug xon/xoff
		tg.status = _tg_parser(tg.buf);	// dispatch to active parser
		tg.prompted = FALSE;			// signals ready-for-next-line
	}
	if (tg.status == TG_QUIT) {	// handle case where the parser detected a QUIT
		_tg_set_mode(TG_TEST_MODE);
	}
	if (tg.status == TG_EOF) {	//(EOF can come from file devices only)
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
			case 'D': _tg_set_mode(TG_DIRECT_DRIVE_MODE); break;
			case 'R': return (_tg_reset());
			case 'T': return (_tg_test_T());	// run whatever test u want
			case 'U': return (_tg_test_U());	// run 2nd test you want
//			case 'I': return (_tg_reserved());	// reserved
//			case 'V': return (_tg_reserved());	// reserved
			case 'H': _tg_print_test_help_screen(); return (TG_OK);
			default:  _tg_set_mode(TG_TEST_MODE); break;
		}
	}
	// dispatch based on mode
	tg.status = TG_OK;
	switch (tg.mode) {
		case TG_GCODE_MODE: tg.status = gc_gcode_parser(buf); break;
		case TG_DIRECT_DRIVE_MODE: tg.status = dd_parser(buf); break;
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

char tgModeStringGCode[] PROGMEM = "GCODE";// put strings in program memory
char tgModeStringDumb[] PROGMEM = "DUMB";
char tgModeStringTest[] PROGMEM = "TEST"; 

PGM_P tgModeStrings[] PROGMEM = {	// put string pointer array in prog mem
	tgModeStringGCode,
	tgModeStringDumb,
	tgModeStringTest
};

static void _tg_prompt()
{
	fprintf_P(stderr, PSTR("tinyg %S"),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
	if (cm_get_inches_mode() == TRUE) {
		fprintf_P(stderr, PSTR("[in] ok> "));
	} else {
		fprintf_P(stderr, PSTR("[mm] ok> "));
	}
	tg.prompted = TRUE;				// set prompt state
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
char tgs17[] PROGMEM = "{17} Parameter not found";
char tgs18[] PROGMEM = "{18} Parameter under range";
char tgs19[] PROGMEM = "{19} Parameter over range";
char tgs20[] PROGMEM = "{20} Bad number format";
char tgs21[] PROGMEM = "{21} Floating point error";
char tgs22[] PROGMEM = "{22} Motion control error";
char tgs23[] PROGMEM = "{23} Arc specification error";
char tgs24[] PROGMEM = "{24} Zero length line";
char tgs25[] PROGMEM = "{25} Maximum feed rate exceeded";
char tgs26[] PROGMEM = "{26} Maximum seek rate exceeded";
char tgs27[] PROGMEM = "{27} Maximum table travel exceeded";
char tgs28[] PROGMEM = "{28} Maximum spindle speed exceeded";
char tgs29[] PROGMEM = "{29} Failed to converge";
char tgs30[] PROGMEM = "{30} Unused error string";

// put string pointer array in program memory. MUST BE SAME COUNT AS ABOVE
PGM_P tgStatus[] PROGMEM = {	
	tgs00, tgs01, tgs02, tgs03, tgs04, tgs05, tgs06, tgs07, tgs08, tgs09,
	tgs10, tgs11, tgs12, tgs13, tgs14, tgs15, tgs16, tgs17, tgs18, tgs19,
	tgs20, tgs21, tgs22, tgs23, tgs24, tgs25, tgs26, tgs27, tgs28, tgs29,
	tgs30 
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

/*
 * _tg_print_test_help_screen() - Send help screen to stderr
 */

static void _tg_print_test_help_screen(void)
{
fprintf_P(stderr, PSTR("*** TinyG Test Screen Help ***\n\
Commands supported in TEST mode:\n\
  g    Re-enter Gcode mode with an of G, M, N, F, Q, $\n\
  t    Run a test (1 - n)\n\
  d    Enter direct drive mode\n\
  h    Show this help screen\n\
Please log any issues at http://synthetos.com/forums\n\
Have fun\n"));
}

/*************************************************************************
 ***** TEST ROUTINES *****************************************************
 *************************************************************************
 * Various test routines
 * _tg_test_T() - 'T' runs a test file from program memory
 * _tg_test_U() - 'U' runs a different test file from program memory
 * _tg_canned_startup() - loads input buffer at reset
 */

static uint8_t _tg_test_T(void)
{
//	xio_open_pgm(PGMFILE(&trajectory_cases_01));
//	xio_open_pgm(PGMFILE(&system_test01)); 		// collected system tests
//	xio_open_pgm(PGMFILE(&system_test01a)); 	// short version of 01
//	xio_open_pgm(PGMFILE(&system_test02)); 		// arcs only
//	xio_open_pgm(PGMFILE(&system_test03)); 		// lines only
//	xio_open_pgm(PGMFILE(&system_test04)); 		// decreasing 3d boxes
//	xio_open_pgm(PGMFILE(&straight_feed_test));
//	xio_open_pgm(PGMFILE(&arc_feed_test));
//	xio_open_pgm(PGMFILE(&contraptor_circle)); 	// contraptor circle test
//	xio_open_pgm(PGMFILE(&braid2d)); 			// braid test, part 1
	xio_open_pgm(PGMFILE(&mudflap)); 			// mudflap girl
//	xio_open_pgm(PGMFILE(hacdc));	 			// HacDC logo
	_tg_set_active_source(XIO_DEV_PGM);
	_tg_set_mode(TG_GCODE_MODE);
	return (TG_OK);
}

static uint8_t _tg_test_U(void)
{
//	xio_open_pgm(PGMFILE(&braid2d_part2)); 		// braid test, part 2
//	xio_open_pgm(PGMFILE(&contraptor_circle)); 	// contraptor circle test
	_tg_set_active_source(XIO_DEV_PGM);
	_tg_set_mode(TG_GCODE_MODE);
	return (TG_OK);
}

// TESTS AND CANNED STARTUP ROUTINES
// Pre-load the USB RX (input) buffer with some test strings
// Be mindful of the char limit on the RX_BUFFER_SIZE (circular buffer)

static void _tg_canned_startup()
{
#ifdef __CANNED_STARTUP
//	xio_queue_RX_string_usb("T\n");				// run test file
//	xio_queue_RX_string_usb("Q\n");				// back to test mode
//	xio_queue_RX_string_usb("U\n");				// run second test file
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("R\n");				// run a homing cycle
//	xio_queue_RX_string_usb("!\n");				// stop
//	xio_queue_RX_string_usb("@\n");				// pause
//	xio_queue_RX_string_usb("%\n");				// resume

/* Configs and controls */
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
//	xio_queue_RX_string_usb("g92 x0 y0 z0\n");	// G92 zero

/* Gn's */
//	xio_queue_RX_string_usb("g18\n");			// plane select

/* G0's */
//	xio_queue_RX_string_usb("g0 x20 y23 z40 a22\n");
//	xio_queue_RX_string_usb("g0 x100 y111 z123\n");
//	xio_queue_RX_string_usb("g0 x10 y11 z13\n");
//	xio_queue_RX_string_usb("g0 x10\n");
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("x0y0z0\n");
//	xio_queue_RX_string_usb("g0 x1000\n");
//	xio_queue_RX_string_usb("g0 x60\n");
//	xio_queue_RX_string_usb("g0 x-30\n");
//	xio_queue_RX_string_usb("g0 x1000\n");
//	xio_queue_RX_string_usb("g0 x2000 y3000 z4000 a5000\n");
//	xio_queue_RX_string_usb("g0 x360\n");
//	xio_queue_RX_string_usb("g0 a10\n");

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

	// grouped test to find bug in 311.05
//	xio_queue_RX_string_usb("g2f400x10y10i5j5\n");
//	xio_queue_RX_string_usb("g2x0y0i5j5\n");
//	xio_queue_RX_string_usb("g1f300x10\n");

/* G4 tests (dwells) */
//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p3\n");
//	xio_queue_RX_string_usb("g0 x0 y0 z0\n");

/* Axis tests */
	xio_queue_RX_string_usb("$amo3\n");
	xio_queue_RX_string_usb("g0 x3 a3\n");
	xio_queue_RX_string_usb("x0\n");

// mudflap simulation
/*
	xio_queue_RX_string_usb("(SuperCam Ver 2.2a SPINDLE)\n");
	xio_queue_RX_string_usb("G92 X0 Y0 Z0 (zero table - ash)\n");
	xio_queue_RX_string_usb("N1 G20	( set inches mode )\n");
	xio_queue_RX_string_usb("N1 G20\n");
	xio_queue_RX_string_usb("N5 G40 G17\n");
	xio_queue_RX_string_usb("N10 T1 M06\n");
	xio_queue_RX_string_usb("(N15 G90 G0 X0 Y0 Z0)\n");
	xio_queue_RX_string_usb("N20 S5000 M03\n");
	xio_queue_RX_string_usb("N25 G00 F30.0\n");
	xio_queue_RX_string_usb("N30 X0.076 Y0.341\n");
	xio_queue_RX_string_usb("N35 G00 Z-1.000 F90.0\n");
	xio_queue_RX_string_usb("N40 G01 Z-1.125 F30.0\n");
	xio_queue_RX_string_usb("N45 G01 F60.0\n");
	xio_queue_RX_string_usb("N50 X0.064 Y0.326\n");
	xio_queue_RX_string_usb("N55 X0.060 Y0.293\n");
	xio_queue_RX_string_usb("N60 X0.077 Y0.267\n");
	xio_queue_RX_string_usb("N65 X0.111 Y0.257\n");
	xio_queue_RX_string_usb("N70 X0.149 Y0.252\n");
	xio_queue_RX_string_usb("N75 X0.188 Y0.255\n");
*/
#endif
}


/* FURTHER NOTES

---- Generalized Serial Handler / Parser ----

  Want to do the following things:

	- Be able to interpret (and mix) various types of inputs, including:
		- Control commands from stdio - e.g. ^c, ^q/^p, ^n/^o...
		- Configuration commands for various sub-systems
		- Gcode blocks
		- Motion control commands (that bypass the Gcode layer)
		- Multi-DOF protocols TBD 
	- Accept and mix inputs from multiple sources:
		- USB
		- RS-485
		- Arduino serial port (Aux)
		- strings in program memory
		- EEPROM data
		- SD card data
	- Accept multiple types of line terminators including:
		- CR
		- LF
		- semicolon
		- NUL

---- Design notes ----

  	- XIO line readers are the lowest level (above single character read)
		From serial inputs: read single characters to assemble a string
		From in-memory strings: read characters from a string in program memory
		Either mode: read string to next terminator and return NULL terminated string 
		Do not otherwise process or normalize the string

	- tg_parser is the top-level parser / dispatcher
		Examine the head of the string to determine how to dispatch
		Supported dispatches:
		- Gcode block
		- Gcode configuration line
		- Direct drive (motion control) command
		- Network command / config (not implemented)

	- Individual parsers/interpreters are called from tg_parser
		These can assume:
		- They will only receive a single line (multi-line inputs have been split)
		- Tyey perform line normalization required for that dispatch type
		- Can run the current command to completion before receiving another command

	- Flow control
		Flow control is provided by the called routine running to completion 
		without blocking. If blocking could occur (e.g. move buffer is full)
		the routine should return and provide a continuation in the main 
		controller loop. This necessitates some careful state handling.

---- How To Code Continuations ----

	Continuations are used to manage points where the application would 
	ordinarily block. Call it application managed threading by way of an 
	inverted control loop. By coding using continuations the application 
	does not need an RTOS and is extremely responsive (there are no "ticks")

	Rules for writing a continuation task:
	  - A continuation is a pair of routines. The first is the main routine,
		the second the continuation. See mc_line() and mc_line_continue().

	  - The main routine is called first and should never block. It may 
	    have function arguments. It performs all initial actions and sets 
		up a static structure to hold data that is needed by the 
		continuation routine. The main routine should end by returning a 
		uint8_t TG_OK or an error code.

	  - The continuation task is a callback that is permanemtly registered 
	  	at the right level of the blocking heirarchy in the tg_controller 
		loop; where it will be called repeatedly by the controller. The 
		continuation cannot have input args - all necessary data must be 
		available in the static struct (or by some other means).

	  - Continuations should be coded as state machines. See the homing 
	  	cycle as an example. Common states used by most machines include: 
		OFF, NEW, or RUNNING. OFF means take no action (return NOOP). 
		The state on initial entry after the main routine should be NEW.
		RUNNING is a catch-all for simple routines. More complex state
		machines may have numerous other states.

	  - The continuation must return the following codes and may return 
	  	additional codes to indicate various exception conditions:

	 	TG_NOOP: No operation ocurred. This is the usual return from an 
			OFF state. All continuations must be callable with no effect 
			when they are OFF (as they are called repeatedly by the 
			controller whether or not they are active).

		TG_EAGAIN: The continuation is blocked or still processing. This one 
			is really important. As long as the continuation still has work 
			to do it must return TG_EAGAIN. Returning eagain causes the 
			tg_controller dispatcher to restart the controller loop from 
			the beginning, skipping all later routines. This enables 
			heirarchical blocking to be performed. The later routines will 
			not be run until the blocking conditions at the lower-level are
			removed.

		TG_OK; The continuation task  has just is completed - i.e. it has 
			just transitioned to OFF. TG_OK should only be returned only once. 
			The next state will be OFF, which will return NOOP.

		TG_COMPLETE: This additional state is used for nesting state 
			machines such as the homing cycle or other cycles (see the 
			homing cycle as an example of a nested state machine). 
			The lower-level routines called by a parent will return 
			TG_EAGAIN until they are done, then they return TG_OK. 
			The return codes from the continuation should be trapped by 
			a wrapper routine that manages the parent and child returns 
			When the parent REALLY wants to return it sends its wrapper 
			TG_COMPLETE, which is translated to an OK for the parent routine.
*/
