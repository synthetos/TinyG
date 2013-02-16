/*
 * tinyg.h - tinyg main header - Application GLOBALS 
 *			 (see also system.h and settings.h)
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of7
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * Is this code over documented? Possibly. 
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc
 */
#ifndef tinyg_h
#define tinyg_h

// NOTE: This header requires <stdio.h> be included previously

#define TINYG_BUILD_NUMBER   	368.12		// Added offsets to status
#define TINYG_VERSION_NUMBER	0.95		// major version
#define TINYG_HARDWARE_VERSION	7.00		// board revision number

/****** DEVELOPMENT SETTINGS ******/

//#define __CANNED_STARTUP					// run any canned startup moves
//#define __DISABLE_PERSISTENCE				// disable EEPROM writes for faster simulation
//#define __SUPPRESS_STARTUP_MESSAGES 		// what it says
//#define __UNIT_TESTS						// master enable for unit tests; uncomment modules in .h files
//#define __DEBUG							// complies debug functions found in test.c

// UNIT_TESTS exist for various modules are can be enabled at the end of their .h files

// bringing in new functionality
//#define __PLAN_R2							// comment out to use R1 planner functions

/****** OPERATING SETTINGS *******/

// Operating Mode: (chose only one)
#define __STANDALONE_MODE					// normal operation - receive from USB
//#define __MASTER_MODE						// receive from USB, relay to rs485
//#define __SLAVE_MODE						// receive from rs485

#ifdef __SLAVE_MODE
#define STD_INPUT XIO_DEV_RS485
#define STD_ERROR XIO_DEV_USB
#else 
#define STD_INPUT XIO_DEV_USB
#define STD_ERROR XIO_DEV_USB
#endif

/*************************************************************************
 * TinyG application-specific prototypes, defines and globals
 */

#define AXES 6					// number of axes supported in this version
#define MOTORS 4				// number of motors on the board
#define COORDS 6				// number of supported coordinate systems (1-6)
#define PWMS 2					// number of supported PWM channels

// If you change COORDS you must adjust the entries in cfgArray table in config.c

/* Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used
 * 	 NB: Using defines can have side effects if anythign else in the code uses A, B, X... etc.
 *   The "side effect safe" min and max routines had this side effect.
 * Alternate enum is: enum tgAxes { X=0, Y, Z, A, B, C };
 */

#define X	0
#define Y	1
#define Z	2
#define A	3
#define B	4
#define C	5
#define U 	6				// reserved
#define V 	7				// reserved
#define W 	8				// reserved

#define MOTOR_1	0 			// define motor numbers and array indexes
#define MOTOR_2	1			// must be defines. enums don't work
#define MOTOR_3	2
#define MOTOR_4	3

#define PWM_1	0
#define PWM_2	1

typedef uint16_t magicNum_t;
#define MAGICNUM 0x12EF		// used for memory integrity assertions

/* TinyG status codes
 * The first code range (0-19) is aligned with the XIO codes and must be so.
 * Please don't change them without checking the corresponding values in xio.h
 *
 * Any changes to the ranges also require changing the message strings and 
 * string array in controller.c
 */
 
// OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
#define	TG_OK 0							// function completed OK
#define	TG_ERROR 1						// generic error return (EPERM)
#define	TG_EAGAIN 2						// function would block here (call again)
#define	TG_NOOP 3						// function had no-operation
#define	TG_COMPLETE 4					// operation is complete
#define TG_TERMINATE 5					// operation terminated (gracefully)
#define TG_RESET 6						// operation was hard reset (sig kill)
#define	TG_EOL 7						// function returned end-of-line
#define	TG_EOF 8						// function returned end-of-file 
#define	TG_FILE_NOT_OPEN 9
#define	TG_FILE_SIZE_EXCEEDED 10
#define	TG_NO_SUCH_DEVICE 11
#define	TG_BUFFER_EMPTY 12
#define	TG_BUFFER_FULL 13
#define	TG_BUFFER_FULL_FATAL 14
#define	TG_INITIALIZING 15				// initializing - not ready for use
#define	TG_ERROR_16 16
#define	TG_ERROR_17 17
#define	TG_ERROR_18 18
#define	TG_ERROR_19 19					// NOTE: XIO codes align to here

// Internal errors and startup messages
#define	TG_INTERNAL_ERROR 20			// unrecoverable internal error
#define	TG_INTERNAL_RANGE_ERROR 21		// number range other than by user input
#define	TG_FLOATING_POINT_ERROR 22		// number conversion error
#define	TG_DIVIDE_BY_ZERO 23
#define	TG_INVALID_ADDRESS 24
#define	TG_READ_ONLY_ADDRESS 25
#define	TG_INIT_FAIL 26
#define	TG_SHUTDOWN 27
#define	TG_MEMORY_CORRUPTION 28
#define	TG_ERROR_29 29
#define	TG_ERROR_30 30
#define	TG_ERROR_31 31
#define	TG_ERROR_32 32
#define	TG_ERROR_33 33
#define	TG_ERROR_34 34
#define	TG_ERROR_35 35
#define	TG_ERROR_36 36
#define	TG_ERROR_37 37
#define	TG_ERROR_38 38
#define	TG_ERROR_39 39

// Input errors (400's, if you will)
#define	TG_UNRECOGNIZED_COMMAND 40		// parser didn't recognize the command
#define	TG_EXPECTED_COMMAND_LETTER 41	// malformed line to parser
#define	TG_BAD_NUMBER_FORMAT 42			// number format error
#define	TG_INPUT_EXCEEDS_MAX_LENGTH 43	// input string is too long 
#define	TG_INPUT_VALUE_TOO_SMALL 44		// input error: value is under minimum
#define	TG_INPUT_VALUE_TOO_LARGE 45		// input error: value is over maximum
#define	TG_INPUT_VALUE_RANGE_ERROR 46	// input error: value is out-of-range
#define	TG_INPUT_VALUE_UNSUPPORTED 47	// input error: value is not supported
#define	TG_JSON_SYNTAX_ERROR 48			// JSON input string is not well formed
#define	TG_JSON_TOO_MANY_PAIRS 49		// JSON input string has too many JSON pairs
#define	TG_JSON_TOO_LONG 50				// JSON output exceeds buffer size
#define	TG_NO_BUFFER_SPACE 51			// Buffer pool is full and cannot perform this operation
#define	TG_ERROR_52 52
#define	TG_ERROR_53 53
#define	TG_ERROR_54 54
#define	TG_ERROR_55 55
#define	TG_ERROR_56 56
#define	TG_ERROR_57 57
#define	TG_ERROR_58 58
#define	TG_ERROR_59 59

// Gcode and machining errors
#define	TG_ZERO_LENGTH_MOVE 60			// move is zero length
#define	TG_GCODE_BLOCK_SKIPPED 61		// block is too short - was skipped
#define	TG_GCODE_INPUT_ERROR 62			// general error for gcode input 
#define	TG_GCODE_FEEDRATE_ERROR 63		// move has no feedrate
#define	TG_GCODE_AXIS_WORD_MISSING 64	// command requires at least one axis present
#define	TG_MODAL_GROUP_VIOLATION 65		// gcode modal group error
#define	TG_HOMING_CYCLE_FAILED 66		// homing cycle did not complete
#define	TG_MAX_TRAVEL_EXCEEDED 67
#define	TG_MAX_SPINDLE_SPEED_EXCEEDED 68
#define	TG_ARC_SPECIFICATION_ERROR 69	// arc specification error

#endif
