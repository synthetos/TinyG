/*
  tinyg.h - tinyg main header - GLOBALS
  Copyright (c) 1020 Alden S Hart Jr.
*/

#ifndef tinyg_h
#define tinyg_h

#define TINYG_VERSION "build 172"

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/* The following enums are unified status return codes for various TinyG functions.
 * This is necessary as a lot of things return via callbacks that return ints.
 * Shared mostly by the interpreters
 */
enum tgStatus {
	TG_OK,						// 0 always = general purpose OK return
	TG_CONTINUE,				// 1 always = continuation in progress (generator, readln)
	TG_OFF,						// generator is off (eg not generating line/arc)
	TG_NEW,						// generator in first pass (transient state)
	TG_DONE,					// generator is done
	TG_QUIT,					// encountered a quit command
	TG_INCOMPLETE_LINE,			// buffer overrun - not a properly terminated line
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_UNSUPPORTED_STATEMENT,	// a different kind of malformed line to parser
	TG_BAD_NUMBER_FORMAT,
	TG_FLOATING_POINT_ERROR,
	TG_MOTION_CONTROL_ERROR
};

#define CHAR_BUFFER_SIZE 80	// unified buffer size. 255 maximum.

//#define __DEBUG TRUE		// set debug mode (comment out to undefine)
//#define __RILEY TRUE		// set RILEY mode (comment out to undefine)

#endif
