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
 *
 * ---- Controller Operation ----
 *
 *	The controller implements a simple process control scheme to manage blocking
 *	in the application. The controller works as a aborting "super loop", where 
 *	the highest priority tasks are run first and progressively lower priority 
 *  tasks are run only if the higher priority tasks are ready.
 *
 *	For this to work tasks must be written to run-to-completion (non blocking),
 *	and must offer re-entry points (continuations) to resume operations that would
 *	have blocked (see arc generator for an example). A task returns TG_EAGAIN to 
 *	indicate a blocking point. If EGTG_EAGAIN is received The controller quits
 *	the loop and starts over.Any other return code allows the controller to 
 *	proceed down the list. 
 *
 *	Interrupts run at the highest priority level and may interact with the tasks.
 *
 *	The priority of operations is:
 *
 *	- High priority ISRs
 *		- issue steps to motors
 *		- count dwell timings
 *		- dequeue and load next stepper move
 *
 *	- Medium priority ISRs
 *		- receive serial input (RX)
 *		- execute signals received by serial input
 *
 *	- Low priority ISRs
 *		- send serial output (TX)
 *
 *	- Top priority tasks
 *		- dequeue and load next stepper move (only if stalled by ISRs)
 *
 *  - Medium priority tasks
 *		- line generator continuation - queue line once move buffer is ready
 *		- arc generator continuation - queue arc segments once move buffer...
 *
 *  - Low priority tasks
 *		- read line from active input device. On completed line:
 *			- run gcode interpreter (or other parser)
 *			- run motion control called by the gcode interpreter
 *			- queue lines and arcs (line and arc generators)
 *		- send "receive ready" prompt back to input source (*'s via _tg_prompt())
 *			(does this once and only once a parser has returned)
 *
 *	Notes:
 *	  - Gcode and other command line flow control is managed cooperatively with 
 *		the application sending the Gcode or other command. The '*' char in the 
 *		prompt indicates that the controller is ready for the next line. 
 *		The sending app is supposed to honor this and not stuff lines down the pipe
 *		(which will choke the controller).
 *
 *	Futures: Using a super loop instead of an event system is a design tradoff - or 
 *	more to the point - a hack. If the flow of control gets much more complicated 
 *	it will make sense to replace this section with an event driven dispatcher.
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
#include "move_buffer.h"
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

struct tgDevice {						// per-device struct
	uint8_t flags;						// flags describe the device
	uint8_t len;						// text buffer length
	char buf[CHAR_BUFFER_SIZE];			// text buffer
};

struct tgController {					// main controller struct
	uint8_t state;						// controller state (tgControllerState)
	uint8_t status;						// return status (controller level)
	uint8_t mode;						// current operating mode (tgMode)
	uint8_t src;						// active source device
	uint8_t default_src;				// default source device
	uint8_t i;							// temp for indexes
	struct tgDevice dev[XIO_DEV_MAX];	// one entry per input device
};
static struct tgController tg;

enum tgMode {
	TG_CONTROL_MODE,					// control mode only. No other modes active
	TG_CONFIG_MODE,						// read and set configurations
	TG_GCODE_MODE,						// gcode interpreter
	TG_DIRECT_DRIVE_MODE,				// direct drive motors
	TG_MAX_MODE
};

enum tgControllerState {				// command execution state
	TG_READY_UNPROMPTED,				// ready for input, no prompt sent
	TG_READY_PROMPTED,					// ready for input, prompt has been sent
	TG_STATE_MAX
};

static int _tg_read_next_line(void);
static void _tg_prompt(void);
static void _tg_set_mode(uint8_t mode);
static void _tg_set_source(uint8_t d);
static int _tg_test_file(void);


/*
 * tg_init()
 */

void tg_init() 
{
	// initialize devices
	for (uint8_t i=1; i < XIO_DEV_MAX; i++) { // don't bother with /dev/null
		tg.dev[i].flags = XIO_FLAG_PROMPTS_bm;
		tg.dev[i].len = sizeof(tg.dev[i].buf);
	}
	tg.dev[XIO_DEV_PGM].flags = 0;			// no asterisks on file devices

	// set input source
	tg.default_src = XIO_DEV_USB; 			// hard-wire input to USB (for now)
	_tg_set_source(tg.default_src);			// set initial active source
	_tg_set_mode(TG_CONTROL_MODE);			// set initial operating mode
	tg.state = TG_READY_UNPROMPTED;

	// version string
	printf_P(PSTR("TinyG - Version %S\n"), (PSTR(TINYG_VERSION)));
}

/* 
 * tg_controller() - top-level controller.
 */

void tg_controller()
{
	// top priority tasks
	st_execute_move();

	// medium priority tasks
	if ((tg.status = mc_line_continue()) == TG_EAGAIN) {	// line generator
		return;
	}
	if ((tg.status = mc_arc_continue()) == TG_EAGAIN) {	 	// arc generator 
		return;
	}

	// low priority tasks
	if ((tg.status = _tg_read_next_line()) == TG_EAGAIN) {	// input line
		return;
	}
	_tg_prompt();		// Send a prompt - but only if controller is ready for input
}

/* 
 * _tg_read_next_line() - Perform a non-blocking line read from active input device
 */

static int _tg_read_next_line()
{
	// read input line or return if not a completed line
	if ((tg.status = xio_fget_ln(tg.src, tg.dev[tg.src].buf, tg.dev[tg.src].len)) == TG_OK) {
		tg.status = tg_parser(tg.dev[tg.src].buf);	// dispatch to parser
	}

	// Note: This switch statement could be reduced as most paths lead to
	//		 TG_READY_UNPROMPTED, but it's written for clarity instead.
	switch (tg.status) {

		case TG_EAGAIN: case TG_NOOP: break;		// no change of state

		case TG_OK: {								// finished a line OK
			tg.state = TG_READY_UNPROMPTED; 		// ready for next input line
			break;
		}
		case TG_QUIT: {								// Quit returned from parser
			_tg_set_mode(TG_CONTROL_MODE);
			tg.state = TG_READY_UNPROMPTED;
			break;
		}
		case TG_EOF: {								// EOF comes from file devs only
			printf_P(PSTR("End of command file\n"));
			tg_reset_source();						// reset to default src
			tg.state = TG_READY_UNPROMPTED;
			break;
		}
		default: {
			tg.state = TG_READY_UNPROMPTED;			// traps various error returns
		}
	}
	return (TG_OK);
}

/* 
 * tg_parser() - process top-level serial input 
 *
 *	tg_parser is the top-level of the input parser tree; dispatches other parsers
 *	Calls lower level parser based on mode
 *
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
		case TG_CONFIG_MODE: tg.status = cfg_parse(buf); break;
		case TG_GCODE_MODE: tg.status = gc_gcode_parser(buf); break;
		case TG_DIRECT_DRIVE_MODE: tg.status = dd_parser(buf); break;
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
	tg.src = d;									// d = XIO device #. See xio.h
}

/*
 * tg_reset_source()  Reset source to default input device
 */

void tg_reset_source()
{
	_tg_set_source(tg.default_src);
}

/* 
 * _tg_prompt() - conditionally display command line prompt
 *
 * We only want a prompt if the following conditions apply:
 *	- system is ready for the next line of input
 *	- no prompt has been issued (issue only one)
 *
 * Further, we only want an asterisk in the prompt if it's not a file device.
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
	if (tg.state == TG_READY_UNPROMPTED) {
		if (tg.dev[tg.src].flags && XIO_FLAG_PROMPTS_bm) {
			printf_P(PSTR("TinyG [%S]*> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
		}
		tg.state = TG_READY_PROMPTED;
	}
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
//	xio_pgm_open(PGMFILE(&dwell_test2));

	xio_pgm_open(PGMFILE(&contraptor_circle)); 	// contraptor circle test
//	xio_pgm_open(PGMFILE(&zoetrope));			// crazy noisy zoetrope file

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
