/*
 * test.h - tinyg test sets
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
#ifndef test_h
#define test_h

//#include <stdio.h>					// needed for FILE def'n

uint8_t run_test(nvObj_t *nv);
void run_canned_startup(void);

/***** DEBUG support ******
 *
 *	DEBUGs are print statements you probably only want enabled during
 *	debugging, and then probably only for one section of code or another.
 *
 *	DEBUG logging is enabled if __DEBUG is defined.
 *	__DEBUG enables a set of arbitrary __dbXXXXXX defines that control
 *	various debug regions, e.g. __dbCONFIG to enable debugging in config.c.
 *	Each __dbXXXXXX pairs with a dbXXXXXX global variable used as a flag.
 *	Each dbXXXXXX is initialized to TRUE or FALSE at startup in main.c.
 *	dbXXXXXX is used as a condition to enable or disable logging.
 *	No varargs, so you must use the one with the right number of variables.
 *	A closing semicolon is not required but is recommended for style.
 *
 *	DEBUG usage examples:
 *		DEBUG0(dbCONFIG, PSTR("String with no variables"));
 *		DEBUG1(dbCONFIG, PSTR("String with one variable: %f"), float_var);
 *		DEBUG2(dbCONFIG, PSTR("String with two variables: %4.2f, %d"), float_var, int_var);
 *
 *	DEBUG print statements are coded so they occupy no program space if
 *	they are not enabled. If you also use __dbXXXX defines to enable debug
 *	code these will - of course - be in the code regardless.
 *
 *	There are also a variety of module-specific diagnostic print statements
 *	that are enabled or not depending on whether __DEBUG is defined
 */

#ifdef __DEBUG
void dump_everything(void);
void roll_over_and_die(void);
void print_scalar(const char *label, float value);
void print_vector(const char *label, float vector[], uint8_t length);

// global allocation of debug control variables
	uint8_t dbECHO_GCODE_BLOCK;
	uint8_t dbALINE_CALLED;
	uint8_t dbSHOW_QUEUED_LINE;
	uint8_t dbSHOW_LIMIT_SWITCH;
	uint8_t dbSHOW_CONFIG_STATE;
	uint8_t dbCONFIG_DEBUG_ENABLED;
	uint8_t dbSHOW_LOAD_MOVE;

#define DEBUG0(dbXXXXXX,msg) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG1(dbXXXXXX,msg,a) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG2(dbXXXXXX,msg,a,b) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a,b); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG3(dbXXXXXX,msg,a,b,c) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a,b,c); \
								fprintf_P(stderr,PSTR("\n"));}}
#else
#define DEBUG0(dbXXXXXX,msg)
#define DEBUG1(dbXXXXXX,msg,a)
#define DEBUG2(dbXXXXXX,msg,a,b)
#define DEBUG3(dbXXXXXX,msg,a,b,c)
#endif	// __DEBUG

/***** Runtime Segment Data Logger Stuff *****
 *
 * This is independent of __DEBUG and does not need __DEBUG defined
 */
#ifdef __SEGMENT_LOGGER
#define SEGMENT_LOGGER_MAX 256

// segment logger structure and index
struct mpSegmentLog {
	uint8_t move_state;
	uint32_t linenum;
	uint32_t segments;
	float velocity;
	float microseconds;
//	float position_x;
//	float target_x;
//	float step_x;
//	float move_time;
//	float accel_time;
};
struct mpSegmentLog sl[SEGMENT_LOGGER_MAX];
uint16_t sl_index;

// function prototype and calling macro
void segment_logger(uint8_t move_state,
					uint32_t linenum,
					uint32_t segments,
					uint32_t segment_count,
					float velocity,
					float microseconds
//					float position_x,
//					float target_x,
//					float step_x,
//					float move_time,
//					float accel_time
					);

#define SEGMENT_LOGGER segment_logger(bf->move_state, \
									  mr.linenum, mr.segments, mr.segment_count, \
									  mr.segment_velocity, \
									  mr.microseconds);
/*
									  mr.microseconds, \
									  mr.position[X], \
									  mr.target[X], \
									  steps[X], \
									  mr.segment_move_time, \
									  mr.segment_accel_time);
*/
#else
#define SEGMENT_LOGGER
#endif	// __SEGMENT_LOGGER
#endif	// test_h
