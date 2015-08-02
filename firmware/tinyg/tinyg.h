/*
 * tinyg.h - tinyg main header
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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
/* Is this code over documented? Possibly.
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc
 */
/* Xmega project setup notes:
 * from: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=117023
 * "Yes it's definitely worth making WinAVR work. To install WinAVR for the project use
 * Project-Configuration Options and under Custom Options untick the "Use toolchain" box
 * then set the top one to \winavr\bin\avr-gcc.exe  (C:\WinAVR-20100110\bin\avr-gcc.exe)
 * and the lower one to \winavr\utils\bin\make.exe  (C:\WinAVR-20100110\utils\bin\make.exe)"
 */
/*
 *  g2sync notes (in rough order to do them):
 *    - change out combined state for the dynamic routine
 *    - split canonical machine init into init and reset parts
 *    - install gcode_input struct with booleans
 *    - install constants
 *    - add state management for alarm, shutdown and panic
 *      - go back through files and change to new alarm, shutdown and panic
 *      - change out soft limits for new alarms
 *    - install digital input system as gpio file
 *      - change release to 0.98
 *      - refactor limits and homing for new inputs
 *    - move spindle and coolant to separate files
 *    - remove Kahan references
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

#ifndef TINYG_FIRMWARE_BUILD
#define TINYG_FIRMWARE_BUILD        444.05	// re-arranged the config_app table to agree with g2

#endif
#define TINYG_FIRMWARE_VERSION		0.97					// firmware major version
#define TINYG_HARDWARE_PLATFORM		HW_PLATFORM_TINYG_XMEGA	// see hardware.h
#define TINYG_HARDWARE_VERSION		HW_VERSION_TINYGV8		// see hardware.h
#define TINYG_HARDWARE_VERSION_MAX	TINYG_HARDWARE_VERSION

/****** COMPILE-TIME SETTINGS ******/

#define __STEP_CORRECTION
//#define __NEW_SWITCHES					// Using v9 style switch code
//#define __JERK_EXEC						// Use computed jerk (versus forward difference based exec)
//#define __KAHAN							// Use Kahan summation in aline exec functions

#define __TEXT_MODE							// enables text mode	(~10Kb)
#define __HELP_SCREENS						// enables help screens (~3.5Kb)
#define __CANNED_TESTS 						// enables $tests 		(~12Kb)
#define __TEST_99 							// enables diagnostic test 99 (independent of other tests)

/****** DEVELOPMENT SETTINGS ******/

#define __DIAGNOSTIC_PARAMETERS				// enables system diagnostic parameters (_xx) in config_app
//#define __DEBUG_SETTINGS					// special settings. See settings.h
//#define __CANNED_STARTUP					// run any canned startup moves

//#define __SIMULATION						// for software-only simulations
#ifdef __SIMULATION
  #undef  __TEXT_MODE
  #undef  __HELP_SCREENS
  #undef  __CANNED_TESTS
#ifndef __CANNED_STARTUP
  #define __CANNED_STARTUP					// run any canned startup moves
#endif
  #define __DISABLE_PERSISTENCE				// disable EEPROM writes for faster simulation
  #define __SUPPRESS_STARTUP_MESSAGES
  #define __SUPPRESS_STATUS_REPORTS
  #define __SUPPRESS_QUEUE_REPORTS
  #define __SUPRESS_DIAGNOSTIC_DISPLAYS
  #define __SILENCE_JSON_RESPONSES
#endif

//#ifndef WEAK
//#define WEAK  __attribute__ ((weak))
//#endif

/************************************************************************************
 ***** PLATFORM COMPATIBILITY *******************************************************
 ************************************************************************************/
#undef __AVR
#define __AVR
//#undef __ARM
//#define __ARM

/*********************
 * AVR Compatibility *
 *********************/
#ifdef __AVR

#include <avr/pgmspace.h>		// defines PROGMEM and PSTR
																    // gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  pgm_read_word(&cfgArray[nv->index].a)    // get word value from cfgArray
#define GET_TABLE_BYTE(a)  pgm_read_byte(&cfgArray[nv->index].a)    // get byte value from cfgArray
#define GET_TABLE_FLOAT(a) pgm_read_float(&cfgArray[nv->index].a)   // get float value from cfgArray
#define GET_TOKEN_BYTE(a)  pgm_read_byte(&cfgArray[i].a)            // get token byte value from cfgArray

// populate the shared buffer with the token string given the index
#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token);

// get text from an array of strings in PGM and convert to RAM string
#define GET_TEXT_ITEM(b,a) strncpy_P(global_string_buf,(const char *)pgm_read_word(&b[a]), MESSAGE_LEN-1)

// get units from array of strings in PGM and convert to RAM string
#define GET_UNITS(a) strncpy_P(global_string_buf,(const char *)pgm_read_word(&msg_units[cm_get_units_mode(a)]), MESSAGE_LEN-1)

// IO settings
#define STD_IN 	XIO_DEV_USB		// default IO settings
#define STD_OUT	XIO_DEV_USB
#define STD_ERR	XIO_DEV_USB

// String compatibility
#define strtof strtod			// strtof is not in the AVR lib

#endif // __AVR

/*********************
 * ARM Compatibility *
 *********************/
#ifdef __ARM
								// Use macros to fake out AVR's PROGMEM and other AVRisms.
#define PROGMEM					// ignore PROGMEM declarations in ARM/GCC++
#define PSTR (const char *)		// AVR macro is: PSTR(s) ((const PROGMEM char *)(s))

													// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(i,a) cfgArray[i].a	        // get token byte value from cfgArray

#define GET_TOKEN_STRING(i,a) cfgArray[(index_t)i].a
//#define GET_TOKEN_STRING(i,a) (char_t)cfgArray[i].token)// populate the token string given the index
//#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token); // populate the token string given the index

#define GET_TEXT_ITEM(b,a) b[a]						// get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]

// IO settings
#define DEV_STDIN 0				// STDIO defaults - stdio is not yet used in the ARM version
#define DEV_STDOUT 0
#define DEV_STDERR 0

/* String compatibility
 *
 * AVR GCC has "_P" variants that take PROGMEM strings as args.
 * On the ARM/GCC++ the _P functions are just aliases of the non-P variants.
 *
 * Note that we have to be sure to cast non char variables to char types when used
 * with standard functions. We must maintain const when it's required as well.
 *
 *      Example: char *ret = strcpy((char *)d, (const char *)s);
 *      The compiler will be your guide when you get it wrong. :)
 *
 * Avoid redefining global defines if possible The following inline jump functions are better.
 */
inline char* strcpy_P(char* d, const char* s) { return (char *)strcpy((char *)d, (const char *)s); }
inline char* strncpy_P(char* d, const char* s, size_t l) { return (char *)strncpy((char *)d, (const char *)s, l); }

// These we'll allow for the sake of not having to pass the variadic variables...
#define printf_P printf
#define fprintf_P fprintf
#define sprintf_P sprintf

#endif // __ARM

/******************************************************************************
 ***** TINYG APPLICATION DEFINITIONS ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;		// magic number size
#define MAGICNUM 0x12EF			// used for memory integrity assertions

/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used

#define AXES		6			// number of axes supported in this version
#define HOMING_AXES	4			// number of axes that can be homed (assumes Zxyabc sequence)
#define MOTORS		4			// number of motors on the board
#define COORDS		6			// number of supported coordinate systems (1-6)
#define PWMS		2			// number of supported PWM channels

// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    AXIS_B,
    AXIS_C,
    AXIS_U,                     // reserved
    AXIS_V,                     // reserved
    AXIS_W                      // reserved
} cmAxes;

typedef enum {
    OFS_I = 0,
    OFS_J,
    OFS_K
} cmIJKOffsets;

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6
} cmMotors;

typedef enum {
    PWM_1 = 0,
    PWM_2
} cmPWMs;

/******************************************************************************
 ***** STATUS CODE DEFINITIONS ************************************************
 ******************************************************************************/
// NB: must follow platform compatibility to manage PROGMEM and PSTR

#include "error.h"
#endif // End of include guard: TINYG2_H_ONCE
