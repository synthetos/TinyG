/*
 * tinyg.h - tinyg main header - GLOBALS
 * Copyright (c) 2010 Alden S Hart, Jr.
 */

#ifndef tinyg_h
#define tinyg_h

#define TINYG_VERSION "build 191"		// See also CONFIG_VERSION in config.h

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/* The following enums are unified status return codes for various TinyG functions.
 * This is necessary as a lot of things return via callbacks that return ints.
 * The first fixed returns are used for flow control. The rest are up for grabs
 * Shared mostly by the interpreters
 */
enum tgStatus {
	// this block of 4 should remain fixed and in this order
	TG_OK,						// 0 (fixed) = general purpose OK return
	TG_DONE,					// 1 (fixed) = generator is done
	TG_CONTINUE,				// 2 (fixed) = continuation in progress (generator, readln)
	TG_EOF,						// 3 (fixed) = end of file reached

	// have at it for the rest
	TG_GENERIC_ERROR,			// generic error return
	TG_OFF,						// generator is off (eg not generating line/arc)
	TG_NEW,						// generator in first pass (transient state)
	TG_QUIT,					// encountered a quit command
	TG_BUFFER_FULL,				// buffer is full (also used to terminate too-long text line)
	TG_UNRECOGNIZED_COMMAND,	// parser didn't recognize the command
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_ZERO_LENGTH_LINE,		// XYZ line is zero length 
	TG_UNSUPPORTED_STATEMENT,	// a different kind of malformed line to parser
	TG_EAGAIN,					// 11 - function would block here (11 by convention)
	TG_BAD_NUMBER_FORMAT,
	TG_FLOATING_POINT_ERROR,
	TG_MOTION_CONTROL_ERROR,	
	TG_ARC_ERROR				// don't forget the comma if you extend this list
};

#define CHAR_BUFFER_SIZE 80	// unified buffer size. 255 maximum.

//#define __DEBUG TRUE		// set debug mode (comment out to undefine)
//#define __RILEY TRUE		// set RILEY mode (comment out to undefine)

#endif
