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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "xmega_support.h"
#include "gcode.h"
#include "xmega_io.h"
#include "parsers.h"
#include "config.h"
#include "tinyg.h"

char textline[READ_BUFFER_DEFAULT_SIZE];
int i = 0;											// textline buffer index

void tg_prompt() 
{
	writepstr(FD_STDOUT, PSTR("TinyG>> "));			// write to console
}

void tg_init() 
{
	open(DEV_USB, IO_RDWR | IO_ECHO | IO_BAUD_115200);
//	fdusb = open(DEV_USB, IO_RDWR | IO_ECHO | IO_BAUD_115200);
//	writestr(fdusb, "USB test started\r\n");

	writepstr(FD_STDOUT,PSTR("\r\nTinyG [TEST MODE] - Version "));
	writepstr(FD_STDOUT,PSTR(TINYG_VERSION));
	writepstr(FD_STDOUT,PSTR("\r\n"));

//	printPgmString(PSTR("\r\nTinyG [TEST MODE] - Version "));
//	printPgmString(PSTR(TINYG_VERSION));
//	printPgmString(PSTR("\r\n"));
	textline[0] = 0;								// initialize line buffer
	tg_prompt();
}

void tg_read_line(char *textline)
{
	char c;
//	int i = 0;					// does not work if i is defined here. Don't know why.	

	while((c = serialRead()) != EOF) {			// 0x04 is ASCII ETX
		if((c == '\r') || (c == '\n') || (c == ';')) {  // line complete
			textline[i] = 0;						// terminate and echo the string
			printPgmString(PSTR("\r\n EXEC>> "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			tg_print_gcstatus(gc_execute_line(textline));	// execute cmd & show status
			i = 0;
			textline[i] = 0;						// reset the buffer
			tg_prompt();							
		} else if ((c == 0x08) || (c == 0x7F)) {  	// backspace or delete
			textline[--i] = 0;
		} else if (c <= ' ') { 						// throw away WS & ctrl chars
		} else if (c >= 'a' && c <= 'z') {			// convert lower to upper
			textline[i++] = c-'a'+'A';
		} else {
			textline[i++] = c;
		}
	}
}


/* tg_process() - process serial prototol */

void tg_process()
{
	char c;
//	int i = 0;					// does not work if i is defined here. Don't know why.	

	while((c = serialRead()) != 0x04) {				// 0x04 is ASCII ETX
		if((c == '\r') || (c == '\n') || (c == ';')) {  // Line complete. Execute!
			textline[i] = 0;						// terminate and echo the string
			printPgmString(PSTR("\r\n EXEC>> "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			tg_print_gcstatus(gc_execute_line(textline));	// execute cmd & show status
			i = 0;
			textline[i] = 0;						// reset the buffer
			tg_prompt();							
		} else if ((c == 0x08) || (c == 0x7F)) {  	// backspace or delete
			textline[--i] = 0;
		} else if (c <= ' ') { 						// throw away WS & ctrl chars
		} else if (c >= 'a' && c <= 'z') {			// convert lower to upper
			textline[i++] = c-'a'+'A';
		} else {
			textline[i++] = c;
		}
	}
}

/*
( Made using CamBam - http://www.cambam.co.uk )
( e-circles 4/10/2010 1:23:46 AM )
( T0 : 0.0 )
G21 
G90 
G64 
G40
G92 X0 Y0 Z0 (set zero)
G0 Z1.5
( T0 : 0.0 )
T0 M6
( Engrave1 )
G17
M3 S0
G0 X17.6075 Y35.6797
G1 F100.0 Z-0.5
G1 F200.0 X21.4068 Y35.2654
G2 X20.1819 Y32.7363 I-9.0526 J2.8233
G2 X18.0773 Y30.7072 I-6.54 J4.6773
G2 X15.1243 Y29.4444 I-4.7414 J7.0037
G2 X11.8677 Y29.0857 I-2.9605 J11.9147
G2 X7.7803 Y29.6697 I-0.3853 J11.899
G2 X4.31 Y31.6621 I2.4791 J8.3368
G2 X2.1243 Y35.0552 I6.0574 J6.3024
G2 X1.532 Y38.9227 I12.7433 J3.9306
G2 X2.1286 Y42.9079 I14.0281 J-0.063
G2 X4.3508 Y46.4175 I8.5166 J-2.9342
G2 X7.6794 Y48.45 I6.1647 J-6.3539
G2 X11.6635 Y49.084 I3.6279 J-9.9636
G2 X15.5393 Y48.4587 I0.3433 J-10.1968
G2 X18.7718 Y46.4716 I-2.8213 J-8.2124
G2 X20.9465 Y43.0285 I-6.1748 J-6.3083
G2 X21.5294 Y39.1209 I-13.2192 J-3.9692
G2 X21.509 Y38.2561 I-32.37 J0.3319
G1 X5.3313
G3 X5.8549 Y35.6831 I9.9322 J0.6816
G3 X7.3535 Y33.4277 I5.7532 J2.1971
G3 X11.8881 Y31.7522 I4.14 J4.2305
G3 X15.3402 Y32.689 I0.3404 J5.5742
G3 X16.7206 Y34.0389 I-2.9329 J4.3799
G3 X17.6075 Y35.6797 I-7.0816 J4.888
G0 Z1.5
*/

//char block_P[] PROGMEM = "g0 x10 y20 z30";
char block_P[] PROGMEM = "G21 \r\
G90 \r\
G0 Z1.5 \r\
G17 \r\
M3 S0 \r\
G0 X17.6075 Y35.6797 \r\
G1 F100.0 Z-0.5 \r\
G1 F200.0 X21.4068 Y35.2654 \r\
G2 X20.1819 Y32.7363 I-9.0526 J2.8233 \r\
G2 X18.0773 Y30.7072 I-6.54 J4.6773 \r\
G2 X15.1243 Y29.4444 I-4.7414 J7.0037 \r\
G2 X11.8677 Y29.0857 I-2.9605 J11.9147 \r\
G2 X7.7803 Y29.6697 I-0.3853 J11.899 \r\
G2 X4.31 Y31.6621 I2.4791 J8.3368 \r\
G2 X2.1243 Y35.0552 I6.0574 J6.3024 \r\
G2 X1.532 Y38.9227 I12.7433 J3.9306 \r\
G2 X2.1286 Y42.9079 I14.0281 J-0.063 \r\
G2 X4.3508 Y46.4175 I8.5166 J-2.9342 \r\
G2 X7.6794 Y48.45 I6.1647 J-6.3539 \r\
G2 X11.6635 Y49.084 I3.6279 J-9.9636 \r\
G2 X15.5393 Y48.4587 I0.3433 J-10.1968 \r\
G2 X18.7718 Y46.4716 I-2.8213 J-8.2124 \r\
G2 X20.9465 Y43.0285 I-6.1748 J-6.3083 \r\
G2 X21.5294 Y39.1209 I-13.2192 J-3.9692 \r\
G2 X21.509 Y38.2561 I-32.37 J0.3319 \r\
G1 X5.3313 \r\
G3 X5.8549 Y35.6831 I9.9322 J0.6816 \r\
G3 X7.3535 Y33.4277 I5.7532 J2.1971 \r\
G3 X11.8881 Y31.7522 I4.14 J4.2305 \r\
G3 X15.3402 Y32.689 I0.3404 J5.5742 \r\
G3 X16.7206 Y34.0389 I-2.9329 J4.3799 \r\
G3 X17.6075 Y35.6797 I-7.0816 J4.888 \r\
G0 Z1.5";

/* run_gcode_from_rom() 

   Provisional. Should take the flash pointer as an arg.

   Gcode in flash must be a single NULL terminated string with all Gcode blocks.
   Blocks are terminated with \r, \n or ';' (semicolon)

*/

void run_gcode_from_rom()
{
	int i = 0;					// ROM buffer index (int allows for > 256 chars)
	int j = 0;					// RAM buffer index (textline)
	char c;

	while ((c = pgm_read_byte(&block_P[i++])) != 0) {
		if ((c == '\r') || (c == '\n') || (c == ';') || (c == '(')) {  // Line complete. Execute!
			textline[j] = 0;							// terminate the string
			tg_print_gcstatus(gc_execute_line(textline));	// execute cmd & show status
			j = 0;			
		} else if (c <= ' ') { 							// toss whitespace & ctrls
		} else if (c == '(' && j == 0) {				// toss comment line
		} else if (c >= 'a' && c <= 'z') {				// convert lower to upper
			textline[j++] = c-'a'+'A';
		} else {
			textline[j++] = c;							// put numbers into line
		}
	}
}

void tg_print_gcstatus (uint8_t status_code)
{
	switch(status_code) {
		case GCSTATUS_OK:
			printPgmString(PSTR("Executing "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;
		
		case GCSTATUS_BAD_NUMBER_FORMAT:
			printPgmString(PSTR("Bad Number Format "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_EXPECTED_COMMAND_LETTER:
			printPgmString(PSTR("Expected Command Letter "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_UNSUPPORTED_STATEMENT:
			printPgmString(PSTR("Unsupported Statement "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_MOTION_CONTROL_ERROR:
			printPgmString(PSTR("Motion Control Error "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_FLOATING_POINT_ERROR:
			printPgmString(PSTR("FLoating Point Error "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;
	}
	return;
}

