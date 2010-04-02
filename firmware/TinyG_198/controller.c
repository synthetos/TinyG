/*
 * controller.c - tinyg controller and top level parser
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * ---- Mode Auto-Detection behaviors ----
 *
 *	From Control mode a line starting with the following letters will enter modes:
 *
 *		G,M,N	enter GCODE_MODE (as will lower-case of the same)
 *		C,?		enter CONFIG_MODE
 *		D,A		enter DIRECT_DRIVE_MODE
 *		F		enter FILE_MODE (returns automatically after file selection)
 *		I		<reserved>
 *		V		<reserved>
 *
 *	Once in the selected mode these characters are not active as mode selects.
 *	Most modes use Q (Quit) to exit and return to control mode.
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "xio.h"						// must include the main xio.h file
#include "xio_usb.h"					//... and all the devices you are using
#include "xio_pgm.h"

#include "tinyg.h"
#include "controller.h"
#include "gcode.h"						// calls out to gcode parser, etc.
#include "config.h"						// calls out to config parser, etc.
#include "motion_control.h"
#include "direct_drive.h"
#include "stepper.h"					// needed for stepper kill and terminate

/*
 * Canned gcode files for testing
 */

#include "data_gcode_asst.h"			// assorted test code
#include "data_gcode_zoetrope.h"		// zoetrope moves. makes really cool sounds
#include "data_gcode_contraptor_circle.h"

/*
 * Local Scope Functions and Data
 */

enum tgControllerState {				// command execution state vector
	TG_STATE_READY_UNPROMPTED,			// ready for input, no prompt sent
	TG_STATE_READY_PROMPTED,			// ready for input, prompt has been sent
	TG_STATE_READING_COMMAND,			// reading a command - no command is active
	TG_STATE_RUNNING_COMMAND,			// controller is running a command
	TG_STATE_MAX						// NB: May be both running a command and 
										// reading next one. Running takes precedence
};

enum tgDeviceState {					// source channel state
	TG_SRC_INACTIVE,					// devivce won't receive input or signals
	TG_SRC_ACTIVE,						// active input source (only one at a time)
	TG_SRC_SIGNAL,						// read signals only
	TG_SRC_MAX
};

enum tgMode {
	TG_CONTROL_MODE,					// control mode only. No other modes active
	TG_CONFIG_MODE,						// read and set configurations
	TG_GCODE_MODE,						// gcode interpreter
	TG_DIRECT_DRIVE_MODE,				// direct drive motors
	TG_MAX_MODE
};

struct tgDevice {						// per-device struct
	uint8_t state;						// device state (tgDeviceState)
	uint8_t len;						// text buffer length
	void (*poll_func)(uint8_t d);		// polling function for scanning input channel
	char buf[CHAR_BUFFER_SIZE];			// text buffer
};

struct tgController {					// main controller struct
	uint8_t state;						// controller state (tgControllerState)
	uint8_t status;						// return status (controller level)
	uint8_t mode;						// current operating mode (tgMode)
	uint8_t source;						// active source device
	uint8_t source_default;				// default source device
	uint8_t	prompts;					// set TRUE to enable prompt lines
	uint8_t i;							// temp for indexes
	struct tgDevice dev[XIO_DEV_MAX];	// one entry per input device
};
static struct tgController tg;


static void _tg_prompt(void);
static void _tg_set_mode(uint8_t mode);
static void _tg_set_source(uint8_t d);
static void _tg_poll_active(uint8_t d);
static void _tg_poll_signal(uint8_t d);
static int _tg_test_file(void);


/*
 * tg_init()
 */

void tg_init() 
{
	xio_control(XIO_DEV_USB, XIO_SIG_FUNC, (int)&tg_signal); // bind sig handler
//	xio_control(XIO_DEV_AUX, XIO_SIG_FUNC, (int)&tg_signal);
//	xio_control(XIO_DEV_RS485, XIO_SIG_FUNC, (int)&tg_signal);

	tg.source_default = XIO_DEV_USB; 
	tg.state = TG_STATE_READY_UNPROMPTED;
	_tg_set_source(tg.source_default);		// set initial active source
	_tg_set_mode(TG_CONTROL_MODE);			// set initial operating mode

	printf_P(PSTR("TinyG - Version %S\n"), (PSTR(TINYG_VERSION)));
}

/* 
 * tg_controller() - top-level controller 
 *
 *	Main "super loop" for TinyG application. Responsibilities:
 *	  - send "receive ready" back to sources (*'s via _tg_prompt())
 *	  - run generators - re-enter line and arc generators if they would block
 *	  - receive lines and signals from IO devices (USB, RS485, PGM files)
 *
 *	Notes:
 *	  - Command flow control is managed cooperatively with the application sending
 *		the Gcode or other command. The '*' char in the prompt indicates that the 
 *		controller is ready for the next line. The app is supposed to honor this 
 *		and not stuff lines down the pipe (which will choke the controller).
 *
 *	  - The USB and RS485 readers are called even when the system is not ready so 
 *		they can still receive control characters (aka signals; e.g. ^c). 
 *		It's up the the calling app not to send lines during the not_ready interval.
 *
 *	Futures: Using a super loop instead of an event system is a design tradoff - or 
 *	more to the point - a hack. If the flow of control gets much more complicated 
 *	it will make sense to replace this section with an event driven dispatcher.
 */

void tg_controller()
{
//	uint8_t i = 1;

	_tg_prompt();		// Send a prompt - but only if controller is ready for input

	if ((tg.status = mc_line_continuation()) == TG_OK) { // Run the line generator 
		tg.state = TG_STATE_READY_UNPROMPTED;
		return;
	}

	if ((tg.status = mc_arc_continuation()) == TG_OK) {	 // Run the arc generator 
		tg.state = TG_STATE_READY_UNPROMPTED;
		return;
	}

	for (tg.i=1; tg.i < XIO_DEV_MAX; tg.i++) {	// Scan all input devices 
		tg.dev[tg.i].poll_func(tg.i);			//   ...(except /dev/null)
	}
}


/* 
 * tg_parser() - process top-level serial input 
 *
 *	tg_parser is the top-level of the input parser tree; dispatches other parsers
 *	Calls lower level parser based on mode

 *	Keeps the system MODE, one of:
 *		- control mode (no lines are interpreted, just control characters)
 *		- config mode
 *		- direct drive mode
 *		- gcode mode
 *
 *	In control mode it auto-detects mode by first character of input buffer
 *	Quits from a parser are handled by the controller (not the individual parsers)
 *	Preserves and passes through return codes (status codes) from lower levels
 */

int tg_parser(char * buf)
{
	// auto-detect mode if not already set 
	if (tg.mode == TG_CONTROL_MODE) {
		if (buf[0] >= 'a' && buf[0] <= 'z') {		// convert lower to upper
			buf[0] = buf[0]-'a'+'A';
		}
		switch (buf[0]) {
			case 'G': case 'M': case 'N': _tg_set_mode(TG_GCODE_MODE); break;
			case 'C': case '?': _tg_set_mode(TG_CONFIG_MODE); break;
			case 'D': _tg_set_mode(TG_DIRECT_DRIVE_MODE); break;
			case 'F': return (_tg_test_file());
			default:  _tg_set_mode(TG_CONTROL_MODE); break; //+++ put a help prompt here
		}
	}
	// dispatch based on mode
	tg.status = TG_OK;
	switch (tg.mode) {
//		case TG_CONTROL_MODE: 
//			break;

		case TG_CONFIG_MODE:
			tg.status = cfg_parse(buf);
			break;

		case TG_GCODE_MODE: 
			tg.status = gc_gcode_parser(buf);
			break;

		case TG_DIRECT_DRIVE_MODE:
			tg.status = dd_parser(buf);
			break;
	}
	return (tg.status);
}

/*
 * _tg_set_mode() - Set current operating mode
 */

void _tg_set_mode(uint8_t mode)
{
	tg.mode = mode;
}

/*
 * _tg_set_source()  Set current input source
 *
 * Note: Once multiple serial devices are supported this function should be 
 *	expanded to also set the stdout/stderr console device so the prompt and
 *	other messages are sent to the active device.
 */

void _tg_set_source(uint8_t d)
{
	tg.source = d;									// d = XIO device #. See xio.h
	tg.prompts = TRUE;

	// reset common settings for all devices
	for (uint8_t i=1; i < XIO_DEV_MAX; i++) {		// don't bother with /dev/null
		tg.dev[i].state = TG_SRC_SIGNAL;
		tg.dev[i].poll_func = &_tg_poll_signal;
		tg.dev[i].len = sizeof(tg.dev[i].buf);
	}
	tg.dev[XIO_DEV_PGM].state = TG_SRC_INACTIVE;	// program memory is an exception

	// make selected device active
	tg.dev[d].state = TG_SRC_ACTIVE;	
	tg.dev[d].poll_func = &_tg_poll_active;
	if (d == XIO_DEV_PGM) {
		tg.prompts = FALSE;							// no prompts for file input
	}
}

/* 
 * _tg_prompt() - conditionally display command line prompt
 *
 *	Note: Do not display command line prompt if not ready for next input line.
 *
 * ---- Mode Strings - for ASCII output ----
 *	This is an example of how to put a string table into program memory
 *	The order of strings in the table must match order of prModeTypes enum
 *	Access is by: (PGM_P)pgm_read_word(&(tgModeStrings[i]));
 *	  where i is the tgModeTypes enum, e.g. modeGCode
 *
 *	ref: http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 *	ref: http://johnsantic.com/comp/state.html, "Writing Efficient State Machines in C"
 */

char tgModeStringControl[] PROGMEM = "CONTROL MODE"; // put strings in program memory
char tgModeStringConfig[] PROGMEM = "CONFIG MODE";
char tgModeStringGCode[] PROGMEM = "G-CODE MODE";
char tgModeStringDirect[] PROGMEM = "DIRECT DRIVE";

PGM_P tgModeStrings[] PROGMEM = {	// put string pointer array in program memory
	tgModeStringControl,
	tgModeStringConfig,
	tgModeStringGCode,
	tgModeStringDirect
};

void _tg_prompt()
{
	if ((tg.prompts) && (tg.state == TG_STATE_READY_UNPROMPTED)) {
		printf_P(PSTR("TinyG [%S]*> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
//		printf_P(PSTR("TinyG [%S]*> \n"),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
		tg.state = TG_STATE_READY_PROMPTED;
	}
	// bastardized prompts for file sources
	if ((!tg.prompts) && (tg.state == TG_STATE_READY_UNPROMPTED)) {
		printf_P(PSTR("TinyG [%S]*> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
//		printf_P(PSTR("TinyG [%S]*> \n"),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
//		tg.state = TG_STATE_READY_PROMPTED;
	}

}

/* 
 * _tg_poll_active() - Perform a non-blocking line read from active input device
 */

static void _tg_poll_active(uint8_t d)
{
	if (tg.dev[d].state != TG_SRC_ACTIVE) {			// sanity check
		return;
	}

	// special handling for file sources
	if (tg.source == XIO_DEV_PGM) {
		if (tg.state == TG_STATE_READY_UNPROMPTED) {
			tg.state = TG_STATE_READY_PROMPTED; 	// issue "virtual prompt"
		} else {
			return;									// not ready for next line
		}
	}

	// read input line or return if not a completed line
	if ((tg.status = xio_fget_ln(d, tg.dev[d].buf, tg.dev[d].len)) == TG_OK) {
		tg.status = tg_parser(tg.dev[d].buf);	// dispatch to parser
	}
	switch (tg.status) {

		case TG_OK: {								// got a completed line
			tg.state = TG_STATE_READY_UNPROMPTED; 
			break;
		}

		case TG_NOOP: {
			break;
		}

		case TG_CONTINUE: { 						// returned without a new line
			tg.state = TG_STATE_READING_COMMAND; 
			break;
		}

		case TG_QUIT: {								// Quit returned from parser
			_tg_set_mode(TG_CONTROL_MODE);
//			tg.state = TG_STATE_READY_UNPROMPTED;	// left commented for clarity
//			break;
		}
					  	
		case TG_EOF: {								// file devices only
			printf_P(PSTR("End of command file\n"));
			_tg_set_source(tg.source_default);		// reset to default src
//			tg.state = TG_STATE_READY_UNPROMPTED;	// left commented for clarity
//			break;
		}
		default: {
			tg.state = TG_STATE_READY_UNPROMPTED;	// traps various error returns
		}
	}
}

/* 
 * _tg_poll_signal() - Perform a read from a signal-only device
 *
 *	If a signal is received it's dispatched from the low-level line reader
 *	Any line that's read is ignored (tossed)
 */

static void _tg_poll_signal(uint8_t d)
{
	if (tg.dev[d].state != TG_SRC_SIGNAL) {			// sanity check
		return;
	}
	tg.status = xio_fget_ln(d, tg.dev[d].buf, tg.dev[d].len);
}

/* 
 * tg_signal() - default signal handler to bind to the line readers
 */

int tg_signal(uint8_t sig)
{
	switch (sig) {
		case XIO_SIG_OK: break;
 		case XIO_SIG_EOL: break;

		case XIO_SIG_EOF:
			printf_P(PSTR("\r\nEnd of file encountered\r\n"));
//			tg.srcin = stdin;
			_tg_prompt();
			break;

		case XIO_SIG_WOULDBLOCK: break;
		case XIO_SIG_KILL: tg_kill(); break;
		case XIO_SIG_TERMINATE: tg_terminate(); break;
		case XIO_SIG_PAUSE: tg_pause(); break;
		case XIO_SIG_RESUME: tg_resume(); break;
		case XIO_SIG_SHIFTOUT: break;
		case XIO_SIG_SHIFTIN: break;
		default: break;
	}
	return (0);
}

void tg_kill()
{
	st_kill();
	return;
}

void tg_terminate()
{
	st_kill();
	return;
}

void tg_pause()
{
	return;
}

void tg_resume()
{
	return;
}


/*
 * _tg_test_file() - selects and starts playback from a memory file
 *
 * This is a shoirtcut for now. Ultimately the file handle, mode and device 
 * should be provided as args.
 */

int _tg_test_file()
{
	// Open a program memory file:
//	xio_pgm_open(PGMFILE(&g0_test1));		// simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test2));		// another simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test3));		// very short moves for single stepping
//	xio_pgm_open(PGMFILE(&radius_arc_test1));
//	xio_pgm_open(PGMFILE(&radius_arc_test2));
//	xio_pgm_open(PGMFILE(&square_test1));
//	xio_pgm_open(PGMFILE(&square_test2));
//	xio_pgm_open(PGMFILE(&square_test10));
//	xio_pgm_open(PGMFILE(&circle_test10));
//	xio_pgm_open(PGMFILE(&square_circle_test10));
//	xio_pgm_open(PGMFILE(&square_circle_test100));
//	xio_pgm_open(PGMFILE(&spiral_test50a));
//	xio_pgm_open(PGMFILE(&spiral_test5));

//	xio_pgm_open(PGMFILE(&contraptor_circle)); 	// contraptor circle test
	xio_pgm_open(PGMFILE(&zoetrope));

	// set mode and source for file mode
	_tg_set_mode(TG_GCODE_MODE);
	_tg_set_source(XIO_DEV_PGM);
	return (TG_OK);
}



/* FURTHER NOTES

---- Generalized Serial Handler / Parser ----

  Want to do the following things:
	- Be able to interpret (and mix) various types of inputs, including:
		- Control commands from stdio - e.g. ^c, ^q/^p, ^n/^o...
		- Configuration commands for various sub-systems
		- Gcode interpreter blocks
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

  Design notes:
  	- line readers are the lowest level (above single character read)
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
		controller loop. THis necessitates some careful state handling.
*/
