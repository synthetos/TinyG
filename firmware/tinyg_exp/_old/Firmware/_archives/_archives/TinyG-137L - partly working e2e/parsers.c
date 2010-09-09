/*
  tg_parsers.c - TinyG top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.

  ---- Generalized Serial Handler / Parser ----

  Want to do the following things:
	- Be able to interpret (and mix) various types of inputs, including:
		- Gcode blocks
		- Gcode / machine configuration
		- Network and device configuration (e.g. RS-485 network)
		- Motion control commands (that bypass the Gcode layer)
		- Multi-DOF protocols TBD (e.g. 20-axis IPA control protocol)
	- Accept and mix inputs from multiple sources; currently:
		- USB
		- RS-485
		- strings in program memory
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

#include "xmega_support.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "xio.h"
#include "xio_pgm.h"
#include "tinyg.h"
#include "config.h"
#include "parsers.h"
#include "gcode.h"

/*
 * globals and setup
 */

#include "data_gcode_contraptor_circles.h"

#define BUF_LEN 40
static char textline[BUF_LEN];					// fails dismally if not static
extern FILE *srcin;
#define tg_prompt() printf_P(PSTR("TinyG>> ")) // macro instead of a 1 line function

/*
 * tg_init()
 */

void tg_init() 
{
	textline[0] = 0;					// initialize line buffer
	xio_pgm_open(&block_P);				// point program memory file to gcode block

	printf_P(PSTR("TinyG [TEST MODE] - Version "));
	printf_P(PSTR(TINYG_VERSION));		// Did you know that printf actually...
	printf_P(PSTR("\r\n"));				//...prints to stderr and not stdout?

	tg_prompt();
}

/* 
 * tg_process() - process serial prototol 
 */

void tg_process()
{
	uint8_t status;		// double duty, depending on where it's used

	if (fgets(textline, BUF_LEN-1, srcin) == 0) {
		if (textline[0] == NUL) {	// this means you hit end-of-file
			printf_P(PSTR("\r\nEnd of file encountered\r\n"));
			srcin = stdin;			// reset source to stdin
			return;
		}
	}
	tg_normalize_gcode_block(textline);
	printf_P(PSTR("\r\n EXEC>> "));
	printf(textline);
//	printf_P(PSTR("\r\n"));
	status = gc_execute_line(textline);			// execute cmd
	tg_print_gcstatus(status);
}

/*
 * tg_normalize_gcode_block() - normalize a block (line) of gcode in place
 */

char *tg_normalize_gcode_block(char *block) {

	char c;
	uint8_t i = 0; 		// index for incoming characters
	uint8_t j = 0;		// index for normalized characters

	while ((c = block[i++]) != NUL) {
		if (c <= ' ' ) {						// throw away WS & ctrl chars
			continue;
		} else if (c >= 'a' && c <= 'z') {		// convert lower to upper
			block[j++] = c-'a'+'A';
		} else {
			block[j++] = c;
		}
	}
	block[j] = 0;
	return block;
}

/*
 * tg_print_gcstatus
 */

void tg_print_gcstatus(uint8_t status_code)
{
	switch(status_code) {
		case GCSTATUS_OK:
			printf_P(PSTR("\r\nExecuting "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;
		
		case GCSTATUS_BAD_NUMBER_FORMAT:
			printf_P(PSTR("\r\nBad Number Format "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;

		case GCSTATUS_EXPECTED_COMMAND_LETTER:
			printf_P(PSTR("\r\nExpected Command Letter "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;

		case GCSTATUS_UNSUPPORTED_STATEMENT:
			printf_P(PSTR("\r\nUnsupported Statement "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;

		case GCSTATUS_MOTION_CONTROL_ERROR:
			printf_P(PSTR("\r\nMotion Control Error "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;

		case GCSTATUS_FLOATING_POINT_ERROR:
			printf_P(PSTR("\r\nFloating Point Error "));
			printf(textline);
			printf_P(PSTR("\r\n"));
			printf_P(PSTR("\r\n"));
			break;
	}
	return;
}

