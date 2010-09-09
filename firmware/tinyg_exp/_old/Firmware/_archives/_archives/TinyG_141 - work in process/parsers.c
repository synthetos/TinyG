/*
  tg_parsers.c - TinyG top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.

---- control characters and signals ----

  Some control characters are trapped to signal the top_parser ("control mode").

  The mechanism is - a control character is trapped by the stdin get_char() routine.
  get_char() sets a flag in xio_signals and returns an error. The flag can be gotten
  to via the pointer set in __file.udata. Control characters are not echoed at the 
  get_char() level, but they may be by top_parser(); depends on what makes sense.
  
  top_parser() exhibits the following control code behaviors:

   ^c,^x,ESC	Abort current action
  				Sends a "STOP" to the currently active mode
				Does not echo control character
				Exits the current mode (reverts to control mode)
				Echos "STOP"

   ^h, DEL		Delete previous character
  				Only affects top_parser() buffer
				Echoed to stdout if ECHO mode is on

   ^n			Shift out - change to another mode
   				Next received character determines mode to enter
				  'C'onfig mode
				  'G'code mode
				  'D'rive mode (Direct to motor commands)
				  'I'pa mode
				  'F'ile mode (switch stdin to file - requires an address token)

   ^o			Shift in - revert to control mode
   				Exit current mode but do not abort currently executing command

   ^q			Pause
   				Pause reading input until ^s received
				No echo

   ^s			Resume
   				Resume reading input
				No echo

  Mode Auto-Detection behaviors:

	From Control mode a line starting with the following letters will enter modes:

		G,M,N	enter GCODE_MODE (as will lower-case of the same)
		C		enter CONFIG_MODE
		D		enter DIRECT_DRIVE_MODE
		F		enter FILE_MODE
		I		<reserved>
		V		<weserved>

	Once in the selected mode these charcaters are not active.
	Most modes use Q (Quit) to exit. This is performed by the mode.
*/

#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"							// include the main xio.h file
#include "xio_usb.h"						//... and all the devices you are using
#include "xio_pgm.h"
#include "tinyg.h"
#include "parsers.h"
#include "gcode.h"

/*
 * include any canned gcode files
 */

#include "data_gcode_contraptor_circles.h"

/*
 * globals and setup
 */

#define BUF_LEN 80

struct tgState {						// main state struct for parsing and other
	char buf[BUF_LEN];					// parser main buffer
	FILE *srcin;						// current input source
	uint8_t mode;						// current operating mode
};
static struct tgState tg;

/*
 * tg_init()
 */

void tg_init() 
{
	tg.buf[0] = 0;						// initialize line buffer
	tg.mode = TG_CONTROL_MODE;

//	tg.srcin = &dev_usb;				// set gcode input to USB port
	tg.srcin = &dev_pgm;				// set gcode input to memory file

	// Open a program memory file:
//	xio_pgm_open(PGMFILE(&g0_test1));	// simple linear motion test
	xio_pgm_open(PGMFILE(&circle_test1)); // contraptor circle test

	printf_P(PSTR("TinyG [TEST MODE] - Version "));
	printf_P(PSTR(TINYG_VERSION));		// Did you know that printf actually...
	printf_P(PSTR("\r\n"));				//...prints to stderr and not stdout?
	tg_prompt();
}

/* 
 * tg_prompt() - conditionally display command line prompt
 *
 *	Do not display command line prompt if input is not from stdin.
 */
//#define tg_prompt() printf_P(PSTR("TinyG>> ")) // macro instead of a 1 line function

void tg_prompt()
{
	if (tg.srcin == stdin) {
		printf_P(PSTR("TinyG>> "));
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
		printf_P(PSTR("\r\nEnd of file encountered\r\n"));
		clearerr(tg.srcin);
		tg.srcin = stdin;
		tg_prompt();
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
			case 'C': tg.mode = TG_CONFIG_MODE; break;
			case 'D': tg.mode = TG_DIRECT_DRIVE_MODE; break;
			case 'F': tg.mode = TG_FILE_MODE; break;
			default:  tg.mode = TG_CONTROL_MODE; break;
		}
	}
	// dispatch based on mode
	switch (tg.mode) {
		case TG_CONTROL_MODE: break;
		case TG_CONFIG_MODE: break;
		case TG_FILE_MODE: break;
		case TG_GCODE_MODE: gc_gcode_parser(tg.buf); break;
		case TG_DIRECT_DRIVE_MODE: break;
		default: break;
	}
	tg_prompt();
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
