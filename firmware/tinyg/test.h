/*
 * test.h - tinyg test sets
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
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
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
#ifndef test_h
#define test_h

//#include <stdio.h>					// needed for FILE def'n

uint8_t tg_test(cmdObj *cmd);
void tg_canned_startup(void);

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
 *		DEBUG1(dbCONFIG, PSTR("String with one variable: %f"), double_var);
 *		DEBUG2(dbCONFIG, PSTR("String with two variables: %4.2f, %d"), double_var, int_var);
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
void print_scalar(const char *label, double value);
void print_vector(const char *label, double vector[], uint8_t length);

#define __dbECHO_GCODE_BLOCK	// echos input to Gcode interpreter	(gcode.c)
//#define __dbALINE_CALLED		// shows call to mp_aline() 		(planner.c)
//#define __dbSHOW_QUEUED_LINE	// shows line being queued 			(motor_queue.c)
//#define __dbSHOW_LIMIT_SWITCH	// shows switch closures 			(limit_switches.c)
//#define __dbSHOW_CONFIG_STATE	// shows config settings			(config.c)
//#define __dbCONFIG_DEBUG_ENABLED// enable config.c debug statements
//#define __dbSHOW_LOAD_MOVE	// shows move being loaded 

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
	double velocity;
	double microseconds;
//	double position_x;
//	double target_x;
//	double step_x;
//	double move_time;
//	double accel_time;
};
struct mpSegmentLog sl[SEGMENT_LOGGER_MAX];
uint16_t sl_index;

// function prototype and calling macro
void segment_logger(uint8_t move_state, 
					uint32_t linenum,
					uint32_t segments, 
					uint32_t segment_count, 
					double velocity,
					double microseconds
//					double position_x, 
//					double target_x,
//					double step_x, 
//					double move_time,
//					double accel_time
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
