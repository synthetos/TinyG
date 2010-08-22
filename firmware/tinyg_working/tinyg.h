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

// Operating Mode: (chose only one)
//#define __NORMAL_MODE					// normal operation - receive from USB
#define __RELAY_MODE					// receive from USB, relay to rs485
//#define __SLAVE_MODE					// receive from rs485


// System Constants
#define TINYG_VERSION "build 208"
#define EEPROM_DATA_VERSION 100	// Used to migrate old data during firmware upgrades

#define MM_PER_ARC_SEGMENT 0.05
#define ONE_MINUTE_OF_MICROSECONDS 60000000.0	// from GRBL - Thanks Simen!
#define TICKS_PER_MICROSECOND (F_CPU/1000000)	// from GRBL
#define INCHES_PER_MM (1.0/25.4)				// from GRBL
#define MINIMUM_TICKS_PER_STEP (0xC00)		// too small and the steppers freeze	

/* Define axes */

enum tgAxisNum {				// define axis numbers and array indexes from 0 to 3
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
#define TG_NO_SUCH_DEVICE 7
#define TG_BUFFER_EMPTY 8
#define TG_BUFFER_FULL_FATAL 9 
#define TG_BUFFER_FULL_NON_FATAL 10
//----- ...to here

#define TG_QUIT 11						// function returned QUIT
#define TG_UNRECOGNIZED_COMMAND 12		// parser didn't recognize the command
#define TG_EXPECTED_COMMAND_LETTER 13	// malformed line to parser
#define TG_UNSUPPORTED_STATEMENT 14		// a different kind of malformed line
#define TG_PARAMETER_OVER_RANGE 15		// parameter is too large
#define TG_BAD_NUMBER_FORMAT 16			// number format error
#define TG_FLOATING_POINT_ERROR 17		// number conversion error
#define TG_MOTION_CONTROL_ERROR 18		// motion control failure
#define TG_ARC_SPECIFICATION_ERROR 19	// arc specification error
#define TG_ZERO_LENGTH_LINE 20			// XYZ line is zero length 


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

//#define __ECHO TRUE		// set to echo Gcode commands. If false, only prompts returned
//#define __DEBUG TRUE		// set debug mode (comment out to undefine)
//#define __FAKE_STEPPERS	// disables stepper ISR load for faster debugging

#endif
