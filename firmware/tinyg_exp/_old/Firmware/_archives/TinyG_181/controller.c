/*
 * tg_controller.c - tinyg controller and top level routines
 * Copyright (c) 2010 Alden S. Hart, Jr.
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
 *	Most modes use Q (Quit) to exit and return to control mode
 */

#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"						// include the main xio.h file
#include "xio_usb.h"					//... and all the devices you are using
#include "xio_pgm.h"
#include "tinyg.h"
#include "controller.h"
#include "gcode.h"						// calls out to gcode parser, etc.
#include "config.h"						// calls out to config parser, etc.
#include "motion_control.h"
#include "stepper.h"					// needed for stepper kill and terminate

/*
 * include any canned gcode files
 */

#include "data_gcode_files.h"			// contraptor circles and a few other tests
#include "data_gcode_zoetrope.h"		// zoetrope moves

/*
 * Local Scope Functions and Data
 */

static void tg_prompt(void);
static void tg_set_mode(uint8_t mode);
static void tg_set_source(uint8_t src);
static void tg_start_file_mode(void);

struct tgState {						// main state struct for parsing and other
	uint8_t mode;						// current operating mode
	uint8_t source;						// current input source (enum)
	uint8_t src_stdin;					// standard input (enum)
	uint8_t src_stdout;					// standard output (enum)
	uint8_t src_stderr;					// standard error (enum)
	int status;
	FILE *srcin;						// current input source (stdio handle)
};
static struct tgState tg;

enum tgMode {							// used in place of a series of #defines
	TG_MODE_CONTROL,					// control mode only. No other modes active
	TG_MODE_CONFIG,						// configuration mode active
	TG_MODE_FILE,						// file mode - read from a file
	TG_MODE_DIRECT_DRIVE,				// direct drive motor mode active
	TG_MODE_GCODE,						// gcode mode active
	TG_MODE_IPA,						// International Phonetic Alphabet mode
};

enum tgSource {
	TG_SRC_NULL,						// no source selected
	TG_SRC_STDIN,						// set source to stdin
	TG_SRC_USB,							// USB device is line source
	TG_SRC_AUX,							// AUX device is line source (Arduino)
	TG_SRC_NET,							// network is line source (RS-485)
	TG_SRC_PGM							// lines rea from program memory file
};

/*
 * tg_init()
 */

void tg_init() 
{
	tg_set_mode(TG_MODE_CONTROL);		// set initial mode
	tg_set_source(TG_SRC_USB);			// set initial command line source

	tg.src_stdin = TG_SRC_USB;			// hard-wire USB to stdin
	tg.src_stdout = TG_SRC_USB;			// hard-wire USB to stdout
	tg.src_stderr = TG_SRC_USB;			// hard-wire USB to stderr

	printf_P(PSTR("TinyG - Version %S\n"), (PSTR(TINYG_VERSION)));

	// activate all inputs to process signals
	xio_usb_control(XIO_SIG_FUNC, (int)&tg_signal);
	xio_pgm_control(XIO_SIG_FUNC, (int)&tg_signal);

	tg_prompt();
}

/*
 * tg_set_mode()  Set current mode
 */

void tg_set_mode(uint8_t mode)
{
	tg.mode = mode;
}

/*
 * tg_set_source()  Set current line source device
 */

void tg_set_source(uint8_t src)
{
	if (src == TG_SRC_STDIN) {
		tg.source = tg.src_stdin;
	} else {
		tg.source = src;
	}

	// bind null handlers to all sources
	xio_usb_control(XIO_LINE_FUNC, (int)&xio_null_line);
	xio_pgm_control(XIO_LINE_FUNC, (int)&xio_null_line);

	// bind line handler to active source
	switch (tg.source) {
		case TG_SRC_NULL: {
			return;
		}
		case TG_SRC_USB: {
			xio_usb_control(XIO_LINE_FUNC, (int)&tg_parser);
			return;
		}
		case TG_SRC_PGM: {
			xio_pgm_control(XIO_LINE_FUNC, (int)&tg_parser);
			return;
		}
	}
}


/*
 * tg_start_file_mode() - selects and starts playback from a memory file
 */

void tg_start_file_mode()
{
	// Open a program memory file:
//	xio_pgm_open(PGMFILE(&g0_test1));		// simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test2));		// another simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test3));		// very short moves for single stepping
//	xio_pgm_open(PGMFILE(&radius_arc_test1));
//	xio_pgm_open(PGMFILE(&radius_arc_test2));
	xio_pgm_open(PGMFILE(&square_test1));

//	xio_pgm_open(PGMFILE(&contraptor_circle)); 	// contraptor circle test
//	xio_pgm_open(PGMFILE(&zoetrope));		// open the desired file

	// set mode and source for file mode
	tg_set_mode(TG_MODE_GCODE);
	tg_set_source(TG_SRC_PGM);
}

/* 
 * tg_prompt() - conditionally display command line prompt
 *
 *	Note: Do not display command line prompt if input is not from stdin.
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
char tgModeStringFile[] PROGMEM = "FILE PLAYBACK";
char tgModeStringDirect[] PROGMEM = "DIRECT DRIVE";
char tgModeStringGCode[] PROGMEM = "G-CODE MODE";

PGM_P tgModeStrings[] PROGMEM = {	// put string pointer array in program memory
	tgModeStringControl,
	tgModeStringConfig,
	tgModeStringFile,
	tgModeStringDirect,
	tgModeStringGCode
};

void tg_prompt()
{
	if (tg.source == TG_SRC_USB) {
		printf_P(PSTR("TinyG [%S]*> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
	}
}

/* 
 * tg_controller() - top-level controller 
 *
 *	Main entry point for TinyG application. Has these responsibilities:
 *		- initialize system and sources
 *		- kick start steppers - should ordinarily run by themselves
 *		- run generators - re-enter line and arc generators if they block
 *		- receive lines from IO devices
 *		- select mode - controls the operating mdoe of the system
 *		- invoke mode parsers and interpreters
 *		- select sources for input
 *		- send completions back to sources (*'s)
 */

void tg_controller()
{
	// run line generator
	if (mc_line_continuation() == TG_DONE) {	
		tg_prompt();
		return;		
	}

	// run arc generator
	if (mc_arc_continuation() == TG_DONE) {	
		tg_prompt();
		return;		
	}

	// non-blocking read line from USB (dispatches to tg_parser on completed line)
	if (xio_usb_readln() != TG_CONTINUE) {
		tg_prompt();
		return;
	}

	// blocking read line from PGM file (dispatches to tg_parser on completed line)
	if (xio_pgm_readln() == TG_EOF) {		// read from program memory "file"
		tg_set_source(TG_SRC_STDIN);		// EOF: return source to stdin
		tg_prompt();
		return;
	}
}

/* 
 * tg_parser() - process top-level serial input 
 *
 *	Top parser is the top-level of the input parser tree:
 *	Accepts a pointer to a command line buffer
 *	Keeps the system MODE, which to-date includes:
 *		- control mode (no lines are interpreted, just control characters)
 *		- config mode
 *		- direct drive mode
 *		- file playback mode
 *		- gcode mode
 *		- motion control mode
 *	Calls lower level interpreter based on mode
 *	Preserves and passes through return codes (status codes) from lower levels
 */

int tg_parser(char * buf)
{
	char tmp;

	// auto-detect mode if not already set 
	if (tg.mode == TG_MODE_CONTROL) {
		tmp = buf[0];
		if (tmp >= 'a' && tmp <= 'z') {		// convert lower to upper
			tmp = tmp-'a'+'A';
		}
		switch (tmp) {
			case 'G': case 'M': case 'N': tg_set_mode(TG_MODE_GCODE); break;
			case 'C': case '?': tg_set_mode(TG_MODE_CONFIG); break;
			case 'D': tg_set_mode(TG_MODE_DIRECT_DRIVE); break;
			case 'F': tg_set_mode(TG_MODE_FILE); break;
			default:  tg_set_mode(TG_MODE_CONTROL); break;
		}
	}
	// dispatch based on mode
	tg.status = TG_OK;
	switch (tg.mode) {
		case TG_MODE_CONTROL: break;

		case TG_MODE_CONFIG:
			if ((tg.status = cfg_parse(buf)) == CFG_STATUS_QUIT) {
				tg.mode = TG_MODE_CONTROL;
			}
			break;

		case TG_MODE_FILE:
			tg_start_file_mode(); 
			tg.mode = TG_MODE_CONTROL;
			break;

		case TG_MODE_GCODE: 
			if ((tg.status = gc_gcode_parser(buf)) == TG_QUIT) {
				tg.mode = TG_MODE_CONTROL;
			}
			break;

		case TG_MODE_DIRECT_DRIVE: break;

		default: break;
	}
	return (tg.status);
}

/* 
 * tg_signal() - process top-level signals 
 */

int tg_signal(uint8_t sig)
{
	switch (sig) {
		case XIO_SIG_OK: break;
 		case XIO_SIG_EOL: break;

		case XIO_SIG_EOF:
			printf_P(PSTR("\r\nEnd of file encountered\r\n"));
			tg.srcin = stdin;
			tg_prompt();
			break;

		case XIO_SIG_WOULDBLOCK: break;
		case XIO_SIG_KILL: st_kill(); break;
		case XIO_SIG_TERMINATE: st_kill(); break;
		case XIO_SIG_PAUSE: break;
		case XIO_SIG_RESUME: break;
		case XIO_SIG_SHIFTOUT: break;
		case XIO_SIG_SHIFTIN: break;
		default: break;
	}
	return (0);
}


/* FURTHER NOTES

---- Generalized Serial Handler / Parser ----

  Want to do the following things:
	- Be able to interpret (and mix) various types of inputs, including:
		- Control commands from stdio - e.g. ^c, ^q/^p, ^n/^o...
		- Configuration commands for various sub-systems
		- Gcode interpreter blocks
		- Motion control commands (that bypass the Gcode layer)
		- Multi-DOF protocols TBD (e.g. 20-axis IPA control protocol)
	- Accept and mix inputs from multiple sources:
		- USB
		- RS-485
		- Arduino serial port
		- strings in program memory
		- EEPROM data
	- Accept multiple types of line terminators including:
		- CR
		- LF
		- semicolon
		- NUL
	- Convert input strings with multiple command lines into multiple, single line cmds. 
		2 known cases:
		- multi-line progmem string: NULL terminated with embedded CRs
		- multi-command text line: CR terminated with semicolons separating commands

  Design:
  	- tg_read_line / tg_read_line_P is the lowest level (above single character read)
		From serial inputs: read a single character to assemble a string
		From in-memory strings: read characters from a string in program memory
		Either mode: read string to next terminator and return NULL terminated string 
		Does not attempt to normalize the string
	- tg_dispatch is the top-level dispatcher
		Examine the head of the string to determine how to dispatch
		Perform line normalization required for that dispatch type
		Supported dispatches:
		- Gcode block (does not send comments)
		- Gcode configuration line
		- Gcode MSG comment (not implemented)
		- Motion control command
		- Network command / config (not implemented)
		- IPA block (not implemented)
		- Ignored line (e.g. Gcode command)
		- Ill-formed line
	- Individual dispatchers are called from the topDispatch
		These can assume:
		- They will only receive a single line (multi-line inputs have been split)
		- The line will be normalized to their specification
		- Can run the current command to completion before receiving another command

	- Flow control
		Flow control is provided by sleeping at a low level in any sub-system called
		  by the dispatchers (e.g. Gcode motion control layer unable to write an XYZ
		  line because the XYZ line buffer is full). The system exits sleep mode on
		  any interrupt. All input processing is therefore blocked if any single 
		  sub-system is blocked.
*/
