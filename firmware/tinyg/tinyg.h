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
#define TINYG_FIRMWARE_BUILD        440.20	// arc test

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

typedef char char_t;			// ARM/C++ version uses uint8_t as char_t

																	// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  pgm_read_word(&cfgArray[nv->index].a)	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  pgm_read_byte(&cfgArray[nv->index].a)	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) pgm_read_float(&cfgArray[nv->index].a)	// get float value from cfgArray
#define GET_TOKEN_BYTE(a)  (char_t)pgm_read_byte(&cfgArray[i].a)	// get token byte value from cfgArray

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

typedef char char_t;			// In the ARM/GCC++ version char_t is typedef'd to uint8_t
								// because in C++ uint8_t and char are distinct types and
								// we want chars to behave as uint8's

													// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(i,a) (char_t)cfgArray[i].a	// get token byte value from cfgArray

#define GET_TOKEN_STRING(i,a) cfgArray[(index_t)i].a
//#define GET_TOKEN_STRING(i,a) (char_t)cfgArray[i].token)// populate the token string given the index

#define GET_TEXT_ITEM(b,a) b[a]						// get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]

// IO settings
#define DEV_STDIN 0				// STDIO defaults - stdio is not yet used in the ARM version
#define DEV_STDOUT 0
#define DEV_STDERR 0

/* String compatibility
 *
 * The ARM stdio functions we are using still use char as input and output. The macros
 * below do the casts for most cases, but not all. Vararg functions like the printf()
 * family need special handling. These like char * as input and require casts as per:
 *
 *   printf((const char *)"Good Morning Hoboken!\n");
 *
 * The AVR also has "_P" variants that take PROGMEM strings as args.
 * On the ARM/GCC++ the _P functions are just aliases of the non-P variants.
 */
#define strncpy(d,s,l) (char_t *)strncpy((char *)d, (char *)s, l)
#define strpbrk(d,s) (char_t *)strpbrk((char *)d, (char *)s)
#define strcpy(d,s) (char_t *)strcpy((char *)d, (char *)s)
#define strcat(d,s) (char_t *)strcat((char *)d, (char *)s)
#define strstr(d,s) (char_t *)strstr((char *)d, (char *)s)
#define strchr(d,s) (char_t *)strchr((char *)d, (char)s)
#define strcmp(d,s) strcmp((char *)d, (char *)s)
#define strtod(d,p) strtod((char *)d, (char **)p)
#define strtof(d,p) strtof((char *)d, (char **)p)
#define strlen(s) strlen((char *)s)
#define isdigit(c) isdigit((char)c)
#define isalnum(c) isalnum((char)c)
#define tolower(c) (char_t)tolower((char)c)
#define toupper(c) (char_t)toupper((char)c)

#define printf_P printf		// these functions want char * as inputs, not char_t *
#define fprintf_P fprintf	// just sayin'
#define sprintf_P sprintf
#define strcpy_P strcpy

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

#define AXIS_X		0
#define AXIS_Y		1
#define AXIS_Z		2
#define AXIS_A		3
#define AXIS_B		4
#define AXIS_C		5
#define AXIS_U		6			// reserved
#define AXIS_V		7			// reserved
#define AXIS_W		8			// reserved

#define MOTOR_1		0 			// define motor numbers and array indexes
#define MOTOR_2		1			// must be defines. enums don't work
#define MOTOR_3		2
#define MOTOR_4		3
#define MOTOR_5		4
#define MOTOR_6		5

#define PWM_1		0
#define PWM_2		1

/************************************************************************************
 * STATUS CODES
 *
 * The first code range (0-19) is aligned with the XIO codes and must be so.
 * Please don't change them without checking the corresponding values in xio.h
 *
 * Status codes are divided into ranges for clarity and extensibility. At some point
 * this may break down and the whole thing will get messy(er), but it's advised not
 * to change the values of existing status codes once they are in distribution.
 *
 * Ranges are:
 *
 *	 0 - 19		OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
 *
 *  20 - 99		Generic internal and application errors. Internal errors start at 20 and work up,
 *				Assertion failures start at 99 and work down.
 *
 * 100 - 129	Generic data and input errors - not specific to Gcode or TinyG
 *
 * 130 -		Gcode and TinyG application errors and warnings
 *
 * See main.c for associated message strings. Any changes to the codes may also require
 * changing the message strings and string array in main.c
 *
 * Most of the status codes (except STAT_OK) below are errors which would fail the command,
 * and are returned by the failed command and reported back via JSON or text.
 * Some status codes are warnings do not fail the command. These can be used to generate
 * an exception report. These are labeled as WARNING
 */

typedef uint8_t stat_t;
extern stat_t status_code;				// allocated in main.c

#define MESSAGE_LEN 80					// global message string storage allocation
extern char global_string_buf[];				// allocated in main.c

char *get_status_message(stat_t status);

// ritorno is a handy way to provide exception returns
// It returns only if an error occurred. (ritorno is Italian for return)
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
#define	STAT_FUNCTION_IS_STUBBED 17
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
#define	STAT_FAILED_TO_GET_PLANNER_BUFFER 28
#define STAT_GENERIC_EXCEPTION_REPORT 29	// used for test

#define	STAT_PREP_LINE_MOVE_TIME_IS_INFINITE 30
#define	STAT_PREP_LINE_MOVE_TIME_IS_NAN 31
#define	STAT_FLOAT_IS_INFINITE 32
#define	STAT_FLOAT_IS_NAN 33
#define	STAT_PERSISTENCE_ERROR 34
#define	STAT_BAD_STATUS_REPORT_SETTING 35
#define	STAT_ERROR_36 36
#define	STAT_ERROR_37 37
#define	STAT_ERROR_38 38
#define	STAT_ERROR_39 39

#define	STAT_ERROR_40 40
#define	STAT_ERROR_41 41
#define	STAT_ERROR_42 42
#define	STAT_ERROR_43 43
#define	STAT_ERROR_44 44
#define	STAT_ERROR_45 45
#define	STAT_ERROR_46 46
#define	STAT_ERROR_47 47
#define	STAT_ERROR_48 48
#define	STAT_ERROR_49 49

#define	STAT_ERROR_50 50
#define	STAT_ERROR_51 51
#define	STAT_ERROR_52 52
#define	STAT_ERROR_53 53
#define	STAT_ERROR_54 54
#define	STAT_ERROR_55 55
#define	STAT_ERROR_56 56
#define	STAT_ERROR_57 57
#define	STAT_ERROR_58 58
#define	STAT_ERROR_59 59

#define	STAT_ERROR_60 60
#define	STAT_ERROR_61 61
#define	STAT_ERROR_62 62
#define	STAT_ERROR_63 63
#define	STAT_ERROR_64 64
#define	STAT_ERROR_65 65
#define	STAT_ERROR_66 66
#define	STAT_ERROR_67 67
#define	STAT_ERROR_68 68
#define	STAT_ERROR_69 69

#define	STAT_ERROR_70 70
#define	STAT_ERROR_71 71
#define	STAT_ERROR_72 72
#define	STAT_ERROR_73 73
#define	STAT_ERROR_74 74
#define	STAT_ERROR_75 75
#define	STAT_ERROR_76 76
#define	STAT_ERROR_77 77
#define	STAT_ERROR_78 78
#define	STAT_ERROR_79 79

#define	STAT_ERROR_80 80
#define	STAT_ERROR_81 81
#define	STAT_ERROR_82 82
#define	STAT_ERROR_83 83
#define	STAT_ERROR_84 84
#define	STAT_ERROR_85 85
#define	STAT_ERROR_86 86
#define	STAT_ERROR_87 87
#define	STAT_ERROR_88 88
#define	STAT_ERROR_89 89

// Assertion failures - build down from 99 until they meet the system internal errors

#define	STAT_CONFIG_ASSERTION_FAILURE 90
#define	STAT_XIO_ASSERTION_FAILURE 91
#define	STAT_ENCODER_ASSERTION_FAILURE 92
#define	STAT_STEPPER_ASSERTION_FAILURE 93
#define	STAT_PLANNER_ASSERTION_FAILURE 94
#define	STAT_CANONICAL_MACHINE_ASSERTION_FAILURE 95
#define	STAT_CONTROLLER_ASSERTION_FAILURE 96
#define	STAT_STACK_OVERFLOW 97
#define	STAT_MEMORY_FAULT 98					// generic memory corruption detected by magic numbers
#define	STAT_GENERIC_ASSERTION_FAILURE 99		// generic assertion failure - unclassified

// Application and data input errors

// Generic data input errors
#define	STAT_UNRECOGNIZED_NAME 100              // parser didn't recognize the name
#define	STAT_INVALID_OR_MALFORMED_COMMAND 101   // malformed line to parser
#define	STAT_BAD_NUMBER_FORMAT 102              // number format error
#define	STAT_UNSUPPORTED_TYPE 103               // An otherwise valid number or JSON type is not supported
#define	STAT_PARAMETER_IS_READ_ONLY 104         // input error: parameter cannot be set
#define	STAT_PARAMETER_CANNOT_BE_READ 105       // input error: parameter cannot be set
#define	STAT_COMMAND_NOT_ACCEPTED 106			// command cannot be accepted at this time
#define	STAT_INPUT_EXCEEDS_MAX_LENGTH 107       // input string is too long
#define	STAT_INPUT_LESS_THAN_MIN_VALUE 108      // input error: value is under minimum
#define	STAT_INPUT_EXCEEDS_MAX_VALUE 109        // input error: value is over maximum

#define	STAT_INPUT_VALUE_RANGE_ERROR 110        // input error: value is out-of-range
#define	STAT_JSON_SYNTAX_ERROR 111              // JSON input string is not well formed
#define	STAT_JSON_TOO_MANY_PAIRS 112            // JSON input string has too many JSON pairs
#define	STAT_JSON_TOO_LONG 113					// JSON input or output exceeds buffer size
#define	STAT_ERROR_114 114
#define	STAT_ERROR_115 115
#define	STAT_ERROR_116 116
#define	STAT_ERROR_117 117
#define	STAT_ERROR_118 118
#define	STAT_ERROR_119 119

#define	STAT_ERROR_120 120
#define	STAT_ERROR_121 121
#define	STAT_ERROR_122 122
#define	STAT_ERROR_123 123
#define	STAT_ERROR_124 124
#define	STAT_ERROR_125 125
#define	STAT_ERROR_126 126
#define	STAT_ERROR_127 127
#define	STAT_ERROR_128 128
#define	STAT_ERROR_129 129

// Gcode errors and warnings (Most originate from NIST - by concept, not number)
// Fascinating: http://www.cncalarms.com/

#define	STAT_GCODE_GENERIC_INPUT_ERROR 130				// generic error for gcode input
#define	STAT_GCODE_COMMAND_UNSUPPORTED 131				// G command is not supported
#define	STAT_MCODE_COMMAND_UNSUPPORTED 132				// M command is not supported
#define	STAT_GCODE_MODAL_GROUP_VIOLATION 133			// gcode modal group error
#define	STAT_GCODE_AXIS_IS_MISSING 134					// command requires at least one axis present
#define STAT_GCODE_AXIS_CANNOT_BE_PRESENT 135			// error if G80 has axis words
#define STAT_GCODE_AXIS_IS_INVALID 136					// an axis is specified that is illegal for the command
#define STAT_GCODE_AXIS_IS_NOT_CONFIGURED 137			// WARNING: attempt to program an axis that is disabled
#define STAT_GCODE_AXIS_NUMBER_IS_MISSING 138			// axis word is missing its value
#define STAT_GCODE_AXIS_NUMBER_IS_INVALID 139	 		// axis word value is illegal

#define STAT_GCODE_ACTIVE_PLANE_IS_MISSING 140			// active plane is not programmed
#define STAT_GCODE_ACTIVE_PLANE_IS_INVALID 141			// active plane selected is not valid for this command
#define	STAT_GCODE_FEEDRATE_NOT_SPECIFIED 142			// move has no feedrate
#define STAT_GCODE_INVERSE_TIME_MODE_CANNOT_BE_USED 143	// G38.2 and some canned cycles cannot accept inverse time mode
#define STAT_GCODE_ROTARY_AXIS_CANNOT_BE_USED 144		// G38.2 and some other commands cannot have rotary axes
#define STAT_GCODE_G53_WITHOUT_G0_OR_G1 145				// G0 or G1 must be active for G53
#define STAT_REQUESTED_VELOCITY_EXCEEDS_LIMITS 146
#define STAT_CUTTER_COMPENSATION_CANNOT_BE_ENABLED 147
#define STAT_PROGRAMMED_POINT_SAME_AS_CURRENT_POINT 148
#define	STAT_SPINDLE_SPEED_BELOW_MINIMUM 149

#define	STAT_SPINDLE_SPEED_MAX_EXCEEDED 150
#define	STAT_S_WORD_IS_MISSING 151
#define	STAT_S_WORD_IS_INVALID 152
#define	STAT_SPINDLE_MUST_BE_OFF 153
#define	STAT_SPINDLE_MUST_BE_TURNING 154				// some canned cycles require spindle to be turning when called
#define	STAT_ARC_SPECIFICATION_ERROR 155				// generic arc specification error
#define STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE 156	// arc is missing axis (axes) required by selected plane
#define STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE 157 // one or both offsets are not specified
#define STAT_ARC_RADIUS_OUT_OF_TOLERANCE 158			// WARNING - radius arc is too small or too large - accuracy in question
#define STAT_ARC_ENDPOINT_IS_STARTING_POINT 159

#define STAT_P_WORD_IS_MISSING 160						// P must be present for dwells and other functions
#define STAT_P_WORD_IS_INVALID 161						// generic P value error
#define STAT_P_WORD_IS_ZERO 162
#define STAT_P_WORD_IS_NEGATIVE 163						// dwells require positive P values
#define STAT_P_WORD_IS_NOT_AN_INTEGER 164				// G10s and other commands require integer P numbers
#define STAT_P_WORD_IS_NOT_VALID_TOOL_NUMBER 165
#define STAT_D_WORD_IS_MISSING 166
#define STAT_D_WORD_IS_INVALID 167
#define STAT_E_WORD_IS_MISSING 168
#define STAT_E_WORD_IS_INVALID 169

#define STAT_H_WORD_IS_MISSING 170
#define STAT_H_WORD_IS_INVALID 171
#define STAT_L_WORD_IS_MISSING 172
#define STAT_L_WORD_IS_INVALID 173
#define STAT_Q_WORD_IS_MISSING 174
#define STAT_Q_WORD_IS_INVALID 175
#define STAT_R_WORD_IS_MISSING 176
#define STAT_R_WORD_IS_INVALID 177
#define STAT_T_WORD_IS_MISSING 178
#define STAT_T_WORD_IS_INVALID 179

#define	STAT_ERROR_180 180									// reserved for Gcode errors
#define	STAT_ERROR_181 181
#define	STAT_ERROR_182 182
#define	STAT_ERROR_183 183
#define	STAT_ERROR_184 184
#define	STAT_ERROR_185 185
#define	STAT_ERROR_186 186
#define	STAT_ERROR_187 187
#define	STAT_ERROR_188 188
#define	STAT_ERROR_189 189

#define	STAT_ERROR_190 190
#define	STAT_ERROR_191 191
#define	STAT_ERROR_192 192
#define	STAT_ERROR_193 193
#define	STAT_ERROR_194 194
#define	STAT_ERROR_195 195
#define	STAT_ERROR_196 196
#define	STAT_ERROR_197 197
#define	STAT_ERROR_198 198
#define	STAT_ERROR_199 199

// TinyG errors and warnings

#define STAT_GENERIC_ERROR 200
#define	STAT_MINIMUM_LENGTH_MOVE 201					// move is less than minimum length
#define	STAT_MINIMUM_TIME_MOVE 202						// move is less than minimum time
#define	STAT_MACHINE_ALARMED 203						// machine is alarmed. Command not processed
#define	STAT_LIMIT_SWITCH_HIT 204						// a limit switch was hit causing shutdown
#define	STAT_PLANNER_FAILED_TO_CONVERGE 205				// trapezoid generator can through this exception
#define	STAT_ERROR_206 206
#define	STAT_ERROR_207 207
#define	STAT_ERROR_208 208
#define	STAT_ERROR_209 209

#define	STAT_ERROR_210 210
#define	STAT_ERROR_211 211
#define	STAT_ERROR_212 212
#define	STAT_ERROR_213 213
#define	STAT_ERROR_214 214
#define	STAT_ERROR_215 215
#define	STAT_ERROR_216 216
#define	STAT_ERROR_217 217
#define	STAT_ERROR_218 218
#define	STAT_ERROR_219 219

#define	STAT_SOFT_LIMIT_EXCEEDED 220					// soft limit error - axis unspecified
#define	STAT_SOFT_LIMIT_EXCEEDED_XMIN 221				// soft limit error - X minimum
#define	STAT_SOFT_LIMIT_EXCEEDED_XMAX 222				// soft limit error - X maximum
#define	STAT_SOFT_LIMIT_EXCEEDED_YMIN 223				// soft limit error - Y minimum
#define	STAT_SOFT_LIMIT_EXCEEDED_YMAX 224				// soft limit error - Y maximum
#define	STAT_SOFT_LIMIT_EXCEEDED_ZMIN 225				// soft limit error - Z minimum
#define	STAT_SOFT_LIMIT_EXCEEDED_ZMAX 226				// soft limit error - Z maximum
#define	STAT_SOFT_LIMIT_EXCEEDED_AMIN 227				// soft limit error - A minimum
#define	STAT_SOFT_LIMIT_EXCEEDED_AMAX 228				// soft limit error - A maximum
#define	STAT_SOFT_LIMIT_EXCEEDED_BMIN 229				// soft limit error - B minimum

#define	STAT_SOFT_LIMIT_EXCEEDED_BMAX 220				// soft limit error - B maximum
#define	STAT_SOFT_LIMIT_EXCEEDED_CMIN 231				// soft limit error - C minimum
#define	STAT_SOFT_LIMIT_EXCEEDED_CMAX 232				// soft limit error - C maximum
#define	STAT_ERROR_233 233
#define	STAT_ERROR_234 234
#define	STAT_ERROR_235 235
#define	STAT_ERROR_236 236
#define	STAT_ERROR_237 237
#define	STAT_ERROR_238 238
#define	STAT_ERROR_239 239

#define	STAT_HOMING_CYCLE_FAILED 240					// homing cycle did not complete
#define	STAT_HOMING_ERROR_BAD_OR_NO_AXIS 241
#define	STAT_HOMING_ERROR_ZERO_SEARCH_VELOCITY 242
#define	STAT_HOMING_ERROR_ZERO_LATCH_VELOCITY 243
#define	STAT_HOMING_ERROR_TRAVEL_MIN_MAX_IDENTICAL 244
#define	STAT_HOMING_ERROR_NEGATIVE_LATCH_BACKOFF 245
#define	STAT_HOMING_ERROR_SWITCH_MISCONFIGURATION 246
#define	STAT_ERROR_247 247
#define	STAT_ERROR_248 248
#define	STAT_ERROR_249 249

#define	STAT_PROBE_CYCLE_FAILED 250						// probing cycle did not complete
#define STAT_PROBE_ENDPOINT_IS_STARTING_POINT 251
#define	STAT_JOGGING_CYCLE_FAILED 252					// jogging cycle did not complete

// !!! Do not exceed 255 without also changing stat_t typedef

#endif // End of include guard: TINYG2_H_ONCE
