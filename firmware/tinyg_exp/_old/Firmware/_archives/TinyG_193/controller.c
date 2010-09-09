/*
 * controller.c - tinyg controller and top level routines
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
 *
 *	Laziness? A lot of structs have a static struct pointer to an element in a 
 *	subsequent array and then use this pointer instead of passing the address
 *	of the array element to the called function. Embedding the pointer in the
 *	struct makes debugging with the simulator work better. 
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

#include "data_gcode_asst.h"			// contraptor circles and a few other tests
//#include "data_gcode_zoetrope.h"		// zoetrope moves
//#include "data_gcode_cjig.h"			// spiral cut
//#include "data_gcode_XYZ.h"

/*
 * Local Scope Functions and Data
 */

enum tgSource {							// textline input source
	TG_SRC_USB,							// USB device is line source
	TG_SRC_AUX,							// AUX device is line source (Arduino)
	TG_SRC_RS485,						// network is line source (RS-485)
	TG_SRC_PGM,							// read from program memory file
	TG_SRC_MAX							// must be last
};

enum tgControllerState {				// command execution state vector
	TG_STATE_READY_UNPROMPTED,			// controller is ready, no prompt sent
	TG_STATE_READY_PROMPTED,			// controller is ready, prompt has been sent
	TG_STATE_COMMAND_RUNNING			// controller is running a command
};

enum tgInputState {						// input channel state
	TG_INPUT_INACTIVE,					// input channel is OFF
	TG_INPUT_PRIMARY,					// primary input channel
	TG_INPUT_SECONDARY					// secondary input channel (signals only)
};

enum tgMode {							// used in place of a series of #defines
	TG_CONTROL_MODE,					// control mode only. No other modes active
	TG_CONFIG_MODE,						// configuration mode active
	TG_FILE_MODE,						// file mode - read from a file
	TG_DIRECT_DRIVE_MODE,				// direct drive motor mode active
	TG_GCODE_MODE,						// gcode mode active
	TG_IPA_MODE,						// International Phonetic Alphabet mode
};

struct tgInput {
	uint8_t state;						// input channel state
	uint8_t status;						// return status - input channel level
	uint8_t len;						// length of text buffer
	void (*poll_func)(uint8_t src);		// polling function for scanning readln
	int (*read_func)(char * buf);		// line reader function - collect a text line
//	int (*line_func)(char * buf);		// line dispatch function - interpreter
	char buf[CHAR_BUFFER_SIZE];			// line buffer for line reader
};
//static struct tgInput *in;				// declare a static pointer for general use

struct tgController {					// main controller struct
	uint8_t state;						// controller state (tgControllerState)
	uint8_t status;						// return status - controller level
	uint8_t	prompts;					// set TRUE to enable prompt lines
	uint8_t mode;						// current operating mode (tgMode)
	uint8_t source;						// current primary source (tgSource)
	struct tgInput input[TG_SRC_MAX];	// one slot per designated source
};
static struct tgController tg;


static void _tg_prompt(void);
static void _tg_set_mode(uint8_t mode);
static void _tg_set_source(uint8_t src);
static int _tg_start_file_mode(void);
static void _tg_poll_primary(uint8_t src);
static void _tg_poll_secondary(uint8_t src);


/*
 * tg_init()
 */

void tg_init() 
{
	tg.state = TG_STATE_READY_UNPROMPTED;
	tg.prompts = TRUE;

	// initialize USB input
	tg.input[TG_SRC_USB].read_func = (fptr_int_char_p)&xio_usb_readln;
	xio_usb_control(XIO_LINE_FUNC, (int)&xio_null_line);
	xio_usb_control(XIO_SIG_FUNC, (int)&tg_signal);

	// initialize AUX input (arduino)
//	tg.input[TG_SRC_AUX].read_func = (fptr_int_char_p)&xio_aux_readln;
//	xio_aux_control(XIO_LINE_FUNC, (int)&xio_null_line);
//	xio_aux_control(XIO_SIG_FUNC, (int)&tg_signal);

	// initialize RS485 input (network port)
//	tg.input[TG_SRC_RS485].read_func = (fptr_int_char_p)&xio_rs485_readln;
//	xio_rs485_control(XIO_LINE_FUNC, (int)&xio_null_line);
//	xio_rs485_control(XIO_SIG_FUNC, (int)&tg_signal);

	// initialize program memory file input
	tg.input[TG_SRC_PGM].read_func = (fptr_int_char_p)&xio_pgm_readln;
	xio_pgm_control(XIO_LINE_FUNC, (int)&xio_null_line);
	xio_pgm_control(XIO_SIG_FUNC, (int)&tg_signal);

	_tg_set_mode(TG_CONTROL_MODE);		// set initial mode
	_tg_set_source(TG_SRC_USB);			// set initial primary source

	printf_P(PSTR("TinyG - Version %S\n"), (PSTR(TINYG_VERSION)));
}

/*
 * _tg_set_mode() - Set current mode
 */

void _tg_set_mode(uint8_t mode)
{
	tg.mode = mode;
}

/*
 * _tg_set_source()  Set current line source device
 */

void _tg_set_source(uint8_t src)
{
	tg.source = src;
	tg.prompts = TRUE;

	// initialize common settings for all inputs
	for (uint8_t i = 0; i < TG_SRC_MAX; i++) {
		tg.input[i].status = 0;
		tg.input[i].state = TG_INPUT_SECONDARY;
		tg.input[i].poll_func = &_tg_poll_secondary;
		tg.input[i].len = sizeof(tg.input[i].buf);
	}
	tg.input[TG_SRC_PGM].state = TG_INPUT_INACTIVE;	// this one's an exception

	// bind line handler to active source, other settings
	switch (tg.source) {
		case TG_SRC_USB: {
			tg.input[TG_SRC_USB].poll_func = &_tg_poll_primary;
			return;
		}
		case TG_SRC_PGM: {
			tg.input[TG_SRC_PGM].poll_func = &_tg_poll_primary;
			tg.prompts = FALSE;
			return;
		}
	}
}

/* 
 * tg_controller() - top-level controller 
 *
 *	Main "super loop" for TinyG application. Has these responsibilities:
 *	  - receive lines from IO devices (USB, RS485, program memory files)
 *	  - run generators - re-enter line and arc generators if they would block
 *	  - send "system ready" back to sources (*'s via _tg_prompt())
 *
 *	Notes:
 *
 *	  - Mode parsers and signal handlers are invoked as callbacks from line readers
 *
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
	// Send a prompt - but only if controller is ready for one.
	_tg_prompt();

	// Run the line generator 
	tg.status = mc_line_continuation();
	switch (tg.status) {
		case TG_CONTINUE: break;
		case TG_DONE: 	  tg.state = TG_STATE_READY_UNPROMPTED; break;
	}

	// Run the arc generator 
	tg.status = mc_arc_continuation();
	switch (tg.status) {
		case TG_CONTINUE: break;
		case TG_DONE: 	  tg.state = TG_STATE_READY_UNPROMPTED; break;
	}

	// Scan all input devices
	for (uint8_t i = 0; i < TG_SRC_MAX; i++) {
		tg.input[i].poll_func(i);
	}
}

/* 
 * _tg_poll_primary() - Perform a non-blocking readln from a primary input.
 */

static void _tg_poll_primary(uint8_t src)
{
	if (tg.input[src].state != TG_INPUT_PRIMARY) {
		return;
	}
	tg.input[src].status = tg.input[src].read_func(tg.input[src].buf);// line reader
	if (tg.input[src].status == TG_DONE) {
		tg.input[src].status = tg_parser(tg.input[src].buf);
	}
	switch (tg.input[src].status) {
		case TG_CONTINUE: tg.state = TG_STATE_COMMAND_RUNNING; break;
		case TG_DONE:	  tg.state = TG_STATE_READY_UNPROMPTED; break;
		case TG_QUIT:	  tg.state = TG_STATE_READY_UNPROMPTED;
						  _tg_set_mode(TG_CONTROL_MODE);
						  break;	
		default:		  tg.state = TG_STATE_READY_UNPROMPTED;	// error conditions
	}
}

/* 
 * _tg_poll_secondary() - Perform a readln from a secondary input (signals only).
 */

static void _tg_poll_secondary(uint8_t src)
{
/*
	if (in->state != TG_INPUT_SECONDARY) {
		return;
	}
	in->status = in->read_func();			// call the line reader
	if (in->status == TG_DONE) {
		in->status = in->line_func(in->buf);
	}
	switch (in->status) {
		case TG_CONTINUE: tg.state = TG_STATE_COMMAND_RUNNING; break;
		case TG_DONE:	  tg.state = TG_STATE_READY_UNPROMPTED; break;
		case TG_QUIT:	  tg.state = TG_STATE_READY_UNPROMPTED;
						  _tg_set_mode(TG_CONTROL_MODE);
						  break;	
		default:		  tg.state = TG_STATE_READY_UNPROMPTED;	// error conditions
	// Perform a read_line from the file system port (effectively non-blocking).
	// Dispatches to tg_parser on completed line if PGM is the command SRC.
	// Completed lines return controller to a READY_UNPROMPTED state, 
	// but since it's in file mode no prompt will actually be sent
	//
	if (tg.filemode == TRUE) {
		tg.status = xio_pgm_readln();
		switch (tg.status) {
			case TG_CONTINUE: tg.state = TG_STATE_COMMAND_RUNNING; break;
			case TG_DONE:	  tg.state = TG_STATE_READY_UNPROMPTED; break;

			case TG_EOF: case TG_QUIT: 
							  printf_P(PSTR("End of command file\n"));
							  _tg_set_mode(TG_CONTROL_MODE);
						  	  _tg_set_source(TG_SRC_STDIN);	// return source to stdin
							  tg.state = TG_STATE_READY_UNPROMPTED;
						  	  break;

			default: 		  printf_P(PSTR("File exec error\n"));
							  _tg_set_mode(TG_CONTROL_MODE);
						  	  _tg_set_source(TG_SRC_STDIN);	// return source to stdin
							  tg.state = TG_STATE_READY_UNPROMPTED;
						  	  break;
	}
*/
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
 *	Quits from a parser are handled by the controller (not the individual parsers)
 */

int tg_parser(char * buf)
{
	char tmp;

	// auto-detect mode if not already set 
	if (tg.mode == TG_CONTROL_MODE) {
		tmp = buf[0];
		if (tmp >= 'a' && tmp <= 'z') {		// convert lower to upper
			tmp = tmp-'a'+'A';
		}
		switch (tmp) {
			case 'G': case 'M': case 'N': _tg_set_mode(TG_GCODE_MODE); break;
			case 'C': case '?': _tg_set_mode(TG_CONFIG_MODE); break;
			case 'D': _tg_set_mode(TG_DIRECT_DRIVE_MODE); break;
			case 'F': _tg_set_mode(TG_FILE_MODE); break;
			default:  _tg_set_mode(TG_CONTROL_MODE); break;
		}
	}
	// dispatch based on mode
	tg.status = TG_OK;
	switch (tg.mode) {
		case TG_CONTROL_MODE: break;

		case TG_CONFIG_MODE:
			tg.status = cfg_parse(buf);
			break;

		case TG_FILE_MODE:
			tg.status = _tg_start_file_mode();
			tg.mode = TG_CONTROL_MODE;				// change back to control mode
			break;

		case TG_GCODE_MODE: 
			tg.status = gc_gcode_parser(buf);
			break;

		case TG_DIRECT_DRIVE_MODE: break;

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
 * _tg_prompt() - conditionally display command line prompt
 *
 *	Note: Do not display command line prompt if system is not ready for next line.
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

void _tg_prompt()
{
	if ((tg.prompts) && (tg.state == TG_STATE_READY_UNPROMPTED)) {
		printf_P(PSTR("TinyG [%S]*> "),(PGM_P)pgm_read_word(&tgModeStrings[tg.mode]));
		tg.state = TG_STATE_READY_PROMPTED;
	}
}

/*
 * _tg_start_file_mode() - selects and starts playback from a memory file
 */

int _tg_start_file_mode()
{
	// Open a program memory file:
//	xio_pgm_open(PGMFILE(&g0_test1));		// simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test2));		// another simple linear motion test
//	xio_pgm_open(PGMFILE(&g0_test3));		// very short moves for single stepping
//	xio_pgm_open(PGMFILE(&radius_arc_test1));
//	xio_pgm_open(PGMFILE(&radius_arc_test2));
//	xio_pgm_open(PGMFILE(&square_test1));
//	xio_pgm_open(PGMFILE(&square_test10));
	xio_pgm_open(PGMFILE(&square_circle_test10));
//	xio_pgm_open(PGMFILE(&square_circle_test100));
//	xio_pgm_open(PGMFILE(&spiral_test50a));
//	xio_pgm_open(PGMFILE(&spiral_test5));

//	xio_pgm_open(PGMFILE(&contraptor_circle)); 	// contraptor circle test
//	xio_pgm_open(PGMFILE(&zoetrope));
//	xio_pgm_open(PGMFILE(&cjig));
//	xio_pgm_open(PGMFILE(&xyz));

	// set mode and source for file mode
	_tg_set_mode(TG_GCODE_MODE);
	_tg_set_source(TG_SRC_PGM);
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
