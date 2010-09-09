/*
 * tinyg.h - tinyg main header - GLOBALS
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is derived from Simen Svale Skogsrud's Grbl code from March 2010
 * It ports the code to the Atmel xmega chip and adds some capabilities
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
 */

#ifndef tinyg_h
#define tinyg_h

#define TINYG_VERSION "build 198"		// See also CONFIG_VERSION in config.h

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
	// this block should remain fixed and in this order
	TG_OK,				//	0	function completed successfully with no errors
	TG_NOOP,			//	1	function had no-operation	
	TG_CONTINUE,		//	2	function requires continuation (call again)
	TG_QUIT,			//	3 	function returns QUIT (mode)
	TG_EOF,				//	4	end-of-file reached
	TG_ERROR,			//	5 	generic error return (errors start here)

	// have at it for the rest
	TG_BUFFER_FULL,				// buffer is full (also used to terminate too-long text line)
	TG_UNRECOGNIZED_COMMAND,	// parser didn't recognize the command
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_ZERO_LENGTH_LINE,		// XYZ line is zero length 
	TG_UNSUPPORTED_STATEMENT,	// a different kind of malformed line to parser
	TG_EAGAIN,					// 11 - function would block here (11 by convention)
	TG_BAD_NUMBER_FORMAT,		// number format error
	TG_FLOATING_POINT_ERROR,	// number conversion error
	TG_MOTION_CONTROL_ERROR,	// motion control failure
	TG_ARC_ERROR,				// arc specification error
	TG_UNRECOGNIZED_DEVICE,		// no device with this ID
	TG_MAX_ERRNO
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
//#define __ECHO TRUE		// set to echo Gcode commands. If false, only prompts returned
//#define __DEBUG TRUE		// set debug mode (comment out to undefine)
//#define __RILEY TRUE		// set RILEY mode (comment out to undefine)
//#define __FAKE_STEPPERS	// disables stepper ISR load for faster debugging

#endif
