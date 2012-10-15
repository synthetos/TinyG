/*
 * planner.h - cartesian trajectory planning and motion execution
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

#ifndef planner_h
#define planner_h 

enum moveType {				// bf->move_type values 
	MOVE_TYPE_NULL = 0,		// null move - does a no-op
	MOVE_TYPE_LINE,			// simple line
	MOVE_TYPE_ALINE,		// acceleration planned line
	MOVE_TYPE_DWELL,		// delay with no movement
	MOVE_TYPE_MCODE,		// M code or other synchrouous command execution
	MOVE_TYPE_COMMAND,		// general command
	MOVE_TYPE_TOOL,			// T command
	MOVE_TYPE_SPINDLE_SPEED,// S command
	MOVE_TYPE_STOP,			// stop motors
	MOVE_TYPE_END			// stop motors and end program
};

enum moveState {
	MOVE_STATE_OFF = 0,		// move inactive (MUST BE ZERO)
	MOVE_STATE_NEW,			// general value if you need an initialization
	MOVE_STATE_RUN,			// general run state (for non-acceleration moves) 
	MOVE_STATE_RUN2,		// used for sub-states
	MOVE_STATE_HEAD,		// aline() acceleration portions
	MOVE_STATE_BODY,		// aline() cruise portions
	MOVE_STATE_TAIL,		// aline() deceleration portions
	MOVE_STATE_SKIP			// mark a skipped block
};
#define MOVE_STATE_RUN1 MOVE_STATE_RUN // a convenience

/***	Most of these factors are the result of a lot of tweaking. Change with caution.***/

/* The following must apply:
 *	  MM_PER_ARC_SEGMENT >= MIN_LINE_LENGTH >= MIN_SEGMENT_LENGTH 
 */
#define ARC_SEGMENT_LENGTH 0.1		// Arc segment size (mm).(0.03)
#define MIN_LINE_LENGTH 0.08		// Smallest line the system can plan (mm) (0.02)
#define MIN_SEGMENT_LENGTH 0.05		// Smallest accel/decel segment (mm). Set to produce ~10 ms segments (0.01)

/* ESTD_SEGMENT_USEC	 Microseconds per planning segment
 *	Should be experimentally adjusted if the MIN_SEGMENT_LENGTH is changed
 */
#define NOM_SEGMENT_USEC ((double)5000)			// nominal segment time
#define MIN_SEGMENT_USEC ((double)2500)			// minimum segment time
#define MIN_ARC_SEGMENT_USEC ((double)20000)	// minimum arc segment time
#define NOM_SEGMENT_TIME (MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_SEGMENT_TIME (MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_ARC_SEGMENT_TIME (MIN_ARC_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)

/* PLANNER_STARTUP_DELAY_SECONDS
 *	Used to introduce a short dwell before planning an idle machine.
 *  If you don;t do this the first block will always plan to zero as it will
 *	start executing before the next block arrives from the serial port.
 *	This cuases the machine to stutter once on startup.
 */
#define PLANNER_STARTUP_DELAY_SECONDS 0.05	// in seconds

/* PLANNER_BUFFER_POOL_SIZE
 *	Should be at least the number of buffers requires to support optimal 
 *	planning in the case of very short lines or arc segments. 
 *	Suggest 12 min. Limit is 255
 */
#define PLANNER_BUFFER_POOL_SIZE 24
//#define PLANNER_BUFFER_POOL_SIZE 8

/* Some parameters for _generate_trapezoid()
 * TRAPEZOID_ITERATION_MAX	 			Max iterations for convergence in the HT asymmetric case.
 * TRAPEZOID_ITERATION_ERROR_PERCENT	Error percentage for iteration convergence. As percent - 0.01 = 1%
 * TRAPEZOID_LENGTH_FIT_TOLERANCE		Tolerance for "exact fit" for H and T cases
 * TRAPEZOID_VELOCITY_TOLERANCE			Adaptive velocity tolerance term
 */
#define TRAPEZOID_ITERATION_MAX 10
#define TRAPEZOID_ITERATION_ERROR_PERCENT 0.10
#define TRAPEZOID_LENGTH_FIT_TOLERANCE (0.0001)	// allowable mm of error in planning phase
#define TRAPEZOID_VELOCITY_TOLERANCE (max(2,bf->entry_velocity/100))

/*
 *	Useful macros
 */

//#define MP_LINE(t,m) ((cfg.enable_acceleration == TRUE) ? mp_aline(t,m) : mp_line(t,m))
#define MP_LINE(t,m) (mp_aline(t,m))	// non-planned lines are disabled

/*
 * Global Scope Functions
 */

void mp_init(void);

uint8_t mp_isbusy(void);
void mp_flush_planner(void);
uint8_t mp_test_write_buffer(void);
double *mp_get_plan_position(double position[]);
void mp_set_plan_position(const double position[]);
void mp_set_axes_position(const double position[]);
void mp_set_axis_position(uint8_t axis, const double position);

double mp_get_runtime_position(uint8_t axis);
double mp_get_runtime_velocity(void);
double mp_get_runtime_linenum(void);
void mp_zero_segment_velocity(void);

uint8_t mp_exec_move(void);
void mp_sync_mcode(uint8_t mcode);
void mp_sync_command(uint8_t command, double parameter);

uint8_t mp_plan_hold_callback(void);
uint8_t mp_end_hold_callback(void);
uint8_t mp_dwell(const double seconds);
uint8_t mp_line(const double target[], const double minutes);
uint8_t mp_aline(const double target[], const double minutes);
uint8_t mp_go_home_cycle(void);

#ifdef __DEBUG
void mp_dump_running_plan_buffer(void);
void mp_dump_plan_buffer_by_index(uint8_t index);
void mp_dump_runtime_state(void);
#endif

//#define __UNIT_TEST_PLANNER	// start __UNIT_TEST_PLANNER
#ifdef __UNIT_TEST_PLANNER
void mp_unit_tests(void);
void mp_plan_arc_unit_tests(void);
#define	PLANNER_UNITS mp_unit_tests();
#else
#define	PLANNER_UNITS
#endif // end __UNIT_TEST_PLANNER

#endif
