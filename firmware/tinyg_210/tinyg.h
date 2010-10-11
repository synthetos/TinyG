/*
 * tinyg.h - tinyg main header - Application GLOBALS (see hardware.h for HW config)
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

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef max
#define max(a, b) (((a)>(b))?(a):(b))
#endif
#ifndef min
#define min(a, b) (((a)<(b))?(a):(b))
#endif

// Operating Mode: (chose only one)
#define __NORMAL_MODE					// normal operation - receive from USB
//#define __RELAY_MODE					// receive from USB, relay to rs485
//#define __SLAVE_MODE					// receive from rs485


// Global Settings
#define TINYG_VERSION "build 210f - \"aphasia\""


// Constants
#define ONE_MINUTE_OF_MICROSECONDS 60000000.0	// from GRBL - Thanks Simen!
#define TICKS_PER_MICROSECOND (F_CPU/1000000)	// from GRBL
#define MM_PER_INCH 25.4						// from GRBL


/* Define axes */

enum tgAxisNum {		// define axis numbers and array indexes from 0 to 3
		X,
		Y,
		Z,
		A,
};

/* TinyG return codes
 * The following return codes are unified for various TinyG functions.
 * The first codes (up to the line) are aligned with the XIO codes.
 * Please don't change them without checking the corresponding values in xio.h
 * If you mess with this be sure to change the strings in tg_print_status
 */

//----- codes aligned with XIO subsystem...
#define TG_OK 0							// function completed OK
#define TG_ERR 1						// generic error return (EPERM)
#define TG_EAGAIN 2						// function would block here (call again)
#define TG_NOOP 3						// function had no-operation	
#define TG_EOL 4						// function returned end-of-line
#define TG_EOF 5						// function returned end-of-file 
#define TG_FILE_NOT_OPEN 6
#define TG_FILE_SIZE_EXCEEDED 7
#define TG_NO_SUCH_DEVICE 8
#define TG_BUFFER_EMPTY 9
#define TG_BUFFER_FULL_FATAL 10 
#define TG_BUFFER_FULL_NON_FATAL 11
//----- ...to here

#define TG_QUIT 12						// function returned QUIT
#define TG_UNRECOGNIZED_COMMAND 13		// parser didn't recognize the command
#define TG_EXPECTED_COMMAND_LETTER 14	// malformed line to parser
#define TG_UNSUPPORTED_STATEMENT 15		// a different kind of malformed line
#define TG_PARAMETER_OVER_RANGE 16		// parameter is too large
#define TG_BAD_NUMBER_FORMAT 17			// number format error
#define TG_FLOATING_POINT_ERROR 18		// number conversion error
#define TG_MOTION_CONTROL_ERROR 19		// motion control failure
#define TG_ARC_SPECIFICATION_ERROR 20	// arc specification error
#define TG_ZERO_LENGTH_LINE 21			// XYZ line is zero length 
#define TG_MAX_FEED_RATE_EXCEEDED 22
#define TG_MAX_SEEK_RATE_EXCEEDED 23
#define TG_MAX_TRAVEL_EXCEEDED 24
#define TG_MAX_SPINDLE_SPEED_EXCEEDED 25

/*
 * Common typedefs (see xio.h for some others)
 */										// pointers to functions:
typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)

/*
 * Various debug and other compile-time switches
 */

#ifdef __SLAVE_MODE
#define DEFAULT_SOURCE XIO_DEV_RS485
#else
#define DEFAULT_SOURCE XIO_DEV_USB		// default source device
#endif

#define __UNIT_TESTS		// uncomment to compile the unit tests into the code
//#define __DEBUG			// uncomment to enable debug logging
//#define __ECHO TRUE		// set to echo Gcode commands. If false, only prompts returned
//#define __FAKE_STEPPERS	// disables stepper ISR load for faster debugging

#endif
