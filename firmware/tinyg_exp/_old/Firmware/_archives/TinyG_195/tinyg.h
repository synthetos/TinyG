/*
 * tinyg.h - tinyg main header - GLOBALS
 * Copyright (c) 2010 Alden S Hart, Jr.
 */

#ifndef tinyg_h
#define tinyg_h

#define TINYG_VERSION "build 195"		// See also CONFIG_VERSION in config.h

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define CHAR_BUFFER_SIZE 80				// unified buffer size. 255 maximum.

/* The following enums are unified status return codes for various TinyG functions.
 * This is necessary as some funcs return via callbacks & return codes get jumbled up.
 * The first fixed codes are used for flow control. The rest are up for grabs.
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
	TG_BAD_NUMBER_FORMAT,		// Gcode failure
	TG_FLOATING_POINT_ERROR,	// Gcode failure
	TG_MOTION_CONTROL_ERROR,	// Motion control failure
	TG_ARC_ERROR,				// don't forget the comma if you extend this list
	TG_UNRECOGNIZED_DEVICE		// no device with this ID
};

/*
 * Common typedefs
 */										// pointers to functions:
typedef void (*fptr_void_void) (void); 	// returns void, void args
typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_void) (void); 	// returns int, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)

/*
 * Various debug and other compile-time switches
 */
//#define __DEBUG TRUE		// set debug mode (comment out to undefine)
//#define __RILEY TRUE		// set RILEY mode (comment out to undefine)
#define __FAKE_STEPPERS		// disables stepper ISR load for faster debugging

#endif
