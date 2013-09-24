/*
 * tinyg.h - tinyg main header - Application GLOBALS 
 *			 (see also system.h and settings.h)
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * Is this code over documented? Possibly. 
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc

 Project setup notes:
 ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=117023
 `
 `  Yes it's definitely worth making WinAVR work. To install WinAVR for the project use 
 `  Project-Configuration Options and under Custom Options untick the "Use toolchain" box 
    then set the top one to \winavr\bin\avr-gcc.exe 	(C:\WinAVR-20100110\bin\avr-gcc.exe)
    and the lower one to 	\winavr\utils\bin\make.exe	(C:\WinAVR-20100110\utils\bin\make.exe)
 */

#ifndef TINYG_H_ONCE
#define TINYG_H_ONCE

// common system includes
#include <ctype.h>					
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


//#include "MotatePins.h"

/****** REVISIONS ******/

#define TINYG_FIRMWARE_BUILD   		392.70	// Moved stepper config interface functions to stepper module
#define TINYG_FIRMWARE_VERSION		0.97	// major version
#define TINYG_HARDWARE_PLATFORM		1		// hardware platform indicator (1 = Xmega series)
#define TINYG_HARDWARE_VERSION		8		// default board revision number
#define TINYG_HARDWARE_VERSION_MAX	8


/****** COMPILE-TIME SETTINGS ******/

#define __TEXT_MODE							// enables text support in modules


/****** DEVELOPMENT SETTINGS ******/

#define __CANNED_STARTUP					// run any canned startup moves
#define __DISABLE_PERSISTENCE				// disable EEPROM writes for faster simulation
#define __SUPPRESS_STARTUP_MESSAGES 		// what it says
#define __SUPPRESS_CANNED_TESTS 			// to save space in debug compiles
//#define __UNIT_TESTS						// master enable for unit tests; uncomment modules in .h files
//#define __DEBUG							// complies debug functions found in test.c

// UNIT_TESTS exist for various modules are can be enabled at the end of their .h files

// bringing in new functionality
//#define __PLAN_R2							// comment out to use R1 planner functions

#ifndef WEAK
#define WEAK  __attribute__ ((weak))
#endif

/******************************************************************************
 ***** PLATFORM COMPATIBILITY *************************************************
 ******************************************************************************/

//#ifdef __AVR
#include <avr/pgmspace.h>
//#endif

/******************************************************************************
 ***** TINYG APPLICATION DEFINITIONS ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;		// magic number size
#define MAGICNUM 0x12EF			// used for memory integrity assertions

#define STD_IN 	XIO_DEV_USB		// default IO settings
#define STD_OUT	XIO_DEV_USB
#define STD_ERR	XIO_DEV_USB

/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used

#define AXES 	6				// number of axes supported in this version
#define MOTORS	4				// number of motors on the board
#define COORDS	6				// number of supported coordinate systems (1-6)
#define PWMS	2				// number of supported PWM channels

// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c

// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used
#define AXIS_X	0
#define AXIS_Y	1
#define AXIS_Z	2
#define AXIS_A	3
#define AXIS_B	4
#define AXIS_C	5
#define AXIS_U 	6				// reserved
#define AXIS_V 	7				// reserved
#define AXIS_W 	8				// reserved

#define MOTOR_1	0 				// define motor numbers and array indexes
#define MOTOR_2	1				// must be defines. enums don't work
#define MOTOR_3	2
#define MOTOR_4	3

#define PWM_1	0
#define PWM_2	1

//#ifdef __AVR_GCC

/*************************************************************************
 * String handling help - strings are handled as uint8_t's typedef'd to char_t
 */
typedef char char_t;					// C version 
//typedef uint8_t char_t;				// C++ version uses uint8_t as char_t

typedef const char PROGMEM *char_P;		// access to PROGMEM arrays of PROGMEM strings
//typedef const char *char_P;			// ARM/C++ version requires this typedef instead

//#define SysTickTimer.getValue SysTickTimer_getValue

//#endif // __AVR_GCC

/* 
 * STATUS CODES
 *
 * The first code range (0-19) is aligned with the XIO codes and must be so.
 * Please don't change them without checking the corresponding values in xio.h
 *
 * Any changes to the ranges also require changing the message strings and 
 * string array in report.c
 *
 * ritorno is a handy way to provide exception returns 
 * It returns only if an error occurred. (ritorno is Italian for return) 
 */

typedef uint8_t stat_t;
#define STATUS_MESSAGE_LEN 48			// status message string storage allocation
char status_message[STATUS_MESSAGE_LEN];// allocate string for global use

stat_t status_code;						// allocate a variable for this macro
//extern stat_t status_code;			// declared in main.cpp
#define ritorno(a) if((status_code=a) != STAT_OK) { return(status_code); }

// OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
#define	STAT_OK 0						// function completed OK
#define	STAT_ERROR 1					// generic error return (EPERM)
#define	STAT_EAGAIN 2					// function would block here (call again)
#define	STAT_NOOP 3						// function had no-operation
#define	STAT_COMPLETE 4					// operation is complete
#define STAT_TERMINATE 5				// operation terminated (gracefully)
#define STAT_RESET 6					// operation was hard reset (sig kill)
#define	STAT_EOL 7						// function returned end-of-line
#define	STAT_EOF 8						// function returned end-of-file 
#define	STAT_FILE_NOT_OPEN 9
#define	STAT_FILE_SIZE_EXCEEDED 10
#define	STAT_NO_SUCH_DEVICE 11
#define	STAT_BUFFER_EMPTY 12
#define	STAT_BUFFER_FULL 13
#define	STAT_BUFFER_FULL_FATAL 14
#define	STAT_INITIALIZING 15			// initializing - not ready for use
#define	STAT_ENTERING_BOOT_LOADER 16	// this code actually emitted from boot loader, not TinyG
#define	STAT_ERROR_17 17
#define	STAT_ERROR_18 18
#define	STAT_ERROR_19 19				// NOTE: XIO codes align to here

// Internal errors and startup messages
#define	STAT_INTERNAL_ERROR 20			// unrecoverable internal error
#define	STAT_INTERNAL_RANGE_ERROR 21	// number range other than by user input
#define	STAT_FLOATING_POINT_ERROR 22	// number conversion error
#define	STAT_DIVIDE_BY_ZERO 23
#define	STAT_INVALID_ADDRESS 24
#define	STAT_READ_ONLY_ADDRESS 25
#define	STAT_INIT_FAIL 26
#define	STAT_ALARMED 27
#define	STAT_MEMORY_FAULT 28
#define	STAT_ERROR_29 29
#define	STAT_ERROR_30 30
#define	STAT_ERROR_31 31
#define	STAT_ERROR_32 32
#define	STAT_ERROR_33 33
#define	STAT_ERROR_34 34
#define	STAT_ERROR_35 35
#define	STAT_ERROR_36 36
#define	STAT_ERROR_37 37
#define	STAT_ERROR_38 38
#define	STAT_ERROR_39 39

// Input errors (400's, if you will)
#define	STAT_UNRECOGNIZED_COMMAND 40		// parser didn't recognize the command
#define	STAT_EXPECTED_COMMAND_LETTER 41		// malformed line to parser
#define	STAT_BAD_NUMBER_FORMAT 42			// number format error
#define	STAT_INPUT_EXCEEDS_MAX_LENGTH 43	// input string is too long 
#define	STAT_INPUT_VALUE_TOO_SMALL 44		// input error: value is under minimum
#define	STAT_INPUT_VALUE_TOO_LARGE 45		// input error: value is over maximum
#define	STAT_INPUT_VALUE_RANGE_ERROR 46		// input error: value is out-of-range
#define	STAT_INPUT_VALUE_UNSUPPORTED 47		// input error: value is not supported
#define	STAT_JSON_SYNTAX_ERROR 48			// JSON input string is not well formed
#define	STAT_JSON_TOO_MANY_PAIRS 49			// JSON input string has too many JSON pairs
#define	STAT_JSON_TOO_LONG 50				// JSON output exceeds buffer size
#define	STAT_NO_BUFFER_SPACE 51				// Buffer pool is full and cannot perform this operation
#define	STAT_CONFIG_NOT_TAKEN 52			// configuration value not taken while in machining cycle
#define	STAT_ERROR_53 53
#define	STAT_ERROR_54 54
#define	STAT_ERROR_55 55
#define	STAT_ERROR_56 56
#define	STAT_ERROR_57 57
#define	STAT_ERROR_58 58
#define	STAT_ERROR_59 59

// Gcode and machining errors
#define	STAT_MINIMUM_LENGTH_MOVE_ERROR 60	// move is less than minimum length
#define	STAT_MINIMUM_TIME_MOVE_ERROR 61		// move is less than minimum time
#define	STAT_GCODE_BLOCK_SKIPPED 62			// block is too short - was skipped
#define	STAT_GCODE_INPUT_ERROR 63			// general error for gcode input 
#define	STAT_GCODE_FEEDRATE_ERROR 64		// move has no feedrate
#define	STAT_GCODE_AXIS_WORD_MISSING 65		// command requires at least one axis present
#define	STAT_MODAL_GROUP_VIOLATION 66		// gcode modal group error
#define	STAT_HOMING_CYCLE_FAILED 67			// homing cycle did not complete
#define	STAT_MAX_TRAVEL_EXCEEDED 68
#define	STAT_MAX_SPINDLE_SPEED_EXCEEDED 69
#define	STAT_ARC_SPECIFICATION_ERROR 70		// arc specification error
#define	STAT_SOFT_LIMIT_EXCEEDED 71			// soft limit error
#define	STAT_COMMAND_NOT_ACCEPTED 72		// command cannot be accepted at this time
#define	STAT_PROBING_CYCLE_FAILED 73		// probing cycle did not complete

/*** Alarm States ***/
#define ALARM_LIMIT_OFFSET 0
#define ALARM_MEMORY_OFFSET 10

#endif // End of include guard: TINYG2_H_ONCE
