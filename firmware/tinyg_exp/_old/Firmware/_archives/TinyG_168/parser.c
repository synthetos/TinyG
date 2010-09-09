/*
  tg_parser.c - TinyG top level parser
  Copyright (c) 2010 Alden S. Hart, Jr.

---- Mode Auto-Detection behaviors ----

	From Control mode a line starting with the following letters will enter modes:

		G,M,N	enter GCODE_MODE (as will lower-case of the same)
		C,?		enter CONFIG_MODE
		D		enter DIRECT_DRIVE_MODE
		F		enter FILE_MODE (returns automatically after file selection)
		I		<reserved>
		V		<reserved>

	Once in the selected mode these characters are not active as mode selects.
	Most modes use Q (Quit) to exit and return to control mode
*/

#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"							// include the main xio.h file
#include "xio_usb.h"						//... and all the devices you are using
#include "xio_pgm.h"
#include "tinyg.h"
#include "parser.h"
#include "gcode.h"							// calls out to gcode parser, etc.
#include "config.h"							// calls out to config parser, etc.
#include "stepper.h"						// needed for kill and terminate

/*
 * include any canned gcode files
 */

#include "data_gcode_files.h"			// contraptor circles and a few other tests
#include "data_gcode_zoetrope.h"		// zoetrope moves

/*
 * globals and setup
 */

#define BUF_LEN 80

struct tgState {						// main state struct for parsing and other
	uint8_t mode;						// current operating mode
	uint8_t signal;						// signal passed up from lower layer
	FILE *srcin;						// current input source
	char buf[BUF_LEN];					// parser main buffer
};
static struct tgState tg;

/*
 * tg_init()
 */

void tg_init() 
{
	tg.buf[0] = 0;						// initialize line buffer
	tg.mode = TG_CONTROL_MODE;
	tg.srcin = &dev_usb;				// set gcode input to USB port
	printf_P(PSTR("TinyG - Version %S\r\n"), (PSTR(TINYG_VERSION)));
	tg_prompt();
}

/*
 * tg_select_file_mode() - selects and starts playback from a memory file
 */

void tg_select_file_mode()
{
	// Open a program memory file:
//	xio_pgm_open(PGMFILE(&g0_test1));	// simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test2));	// another simple linear motion test
	xio_pgm_open(PGMFILE(&circle_test1)); // contraptor circle test
//	xio_pgm_open(PGMFILE(&zoetrope));		// open the desired file
	tg_prompt();
	tg.srcin = &dev_pgm;					// set gcode input to PGM file
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
	if (tg.srcin == stdin) {
		printf_P(PSTR("TinyG [%S]>> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));

	}
}

/* 
 * top_parser() - process top-level serial input 
 *
 * Discussion of the code block: if (fgets(textbuf, BUF_LEN-1, srcin)....
 *	This if{} branch is taken if the srcin device for gcode blocks hits the EOF
 *	or an error. (It does not attempt to distinguish between the two). The error 
 *	or EOF condition must be cleared and input is returned to the stdin device.
 */

void top_parser()
{
	char tmp;

	// get the line and look for end-of-file and control characters (and errors)
	if (fgets(tg.buf, BUF_LEN-1, tg.srcin) == NULL) {	
		// get the signal from the underlying device struct (do this once. Ugh)
		tg.signal = *(uint8_t *)tg.srcin->udata;
		switch (tg.signal) {
			case XIO_SIG_OK: break;

			case XIO_SIG_EOF:
				printf_P(PSTR("\r\nEnd of file encountered\r\n"));
				tg.srcin = stdin;
				tg_prompt();
				break;

			case XIO_SIG_WOULDBLOCK: break;
			case XIO_SIG_KILL: st_kill(); break;
			case XIO_SIG_TERMINATE: break;
			case XIO_SIG_PAUSE: break;
			case XIO_SIG_RESUME: break;
			case XIO_SIG_SHIFTOUT: break;
			case XIO_SIG_SHIFTIN: break;
			default: break;
		}
		clearerr(tg.srcin);
		return;
	}
	// auto-detect mode if not already set 
	if (tg.mode == TG_CONTROL_MODE) {
		tmp = tg.buf[0];
		if (tmp >= 'a' && tmp <= 'z') {		// convert lower to upper
			tmp = tmp-'a'+'A';
		}
		switch (tmp) {
			case 'G': case 'M': case 'N': tg.mode = TG_GCODE_MODE; break;
			case 'C': case '?': tg.mode = TG_CONFIG_MODE; break;
			case 'D': tg.mode = TG_DIRECT_DRIVE_MODE; break;
			case 'F': tg.mode = TG_FILE_MODE; break;
			default:  tg.mode = TG_CONTROL_MODE; break;
		}
	}
	// dispatch based on mode
	switch (tg.mode) {
		case TG_CONTROL_MODE: break;

		case TG_CONFIG_MODE:
			if (cfg_parse(tg.buf) == CFG_STATUS_QUIT) {
				tg.mode = TG_CONTROL_MODE;
			}; break;

		case TG_FILE_MODE:
			tg_select_file_mode(); 
			tg.mode = TG_CONTROL_MODE;
			break;

		case TG_GCODE_MODE: 
			if (gc_gcode_parser(tg.buf) == GC_STATUS_QUIT) {
				tg.mode = TG_CONTROL_MODE;
			}; break;

		case TG_DIRECT_DRIVE_MODE: break;
		default: break;
	}
	tg_prompt();
}


int top_parser2(char * buf)
{
	char tmp;

	// auto-detect mode if not already set 
	if (tg.mode == TG_CONTROL_MODE) {
		tmp = buf[0];
		if (tmp >= 'a' && tmp <= 'z') {		// convert lower to upper
			tmp = tmp-'a'+'A';
		}
		switch (tmp) {
			case 'G': case 'M': case 'N': tg.mode = TG_GCODE_MODE; break;
			case 'C': case '?': tg.mode = TG_CONFIG_MODE; break;
			case 'D': tg.mode = TG_DIRECT_DRIVE_MODE; break;
			case 'F': tg.mode = TG_FILE_MODE; break;
			default:  tg.mode = TG_CONTROL_MODE; break;
		}
	}
	// dispatch based on mode
	switch (tg.mode) {
		case TG_CONTROL_MODE: break;

		case TG_CONFIG_MODE:
			if (cfg_parse(buf) == CFG_STATUS_QUIT) {
				tg.mode = TG_CONTROL_MODE;
			}; break;

		case TG_FILE_MODE:
			tg_select_file_mode(); 
			tg.mode = TG_CONTROL_MODE;
			break;

		case TG_GCODE_MODE: 
			if (gc_gcode_parser(buf) == GC_STATUS_QUIT) {
				tg.mode = TG_CONTROL_MODE;
			}; break;

		case TG_DIRECT_DRIVE_MODE: break;
		default: break;
	}
	tg_prompt();
	return (0);
}

int top_signal(uint8_t sig)
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
		case XIO_SIG_TERMINATE: break;
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
