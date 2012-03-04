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
 */

#ifndef planner_h
#define planner_h 

enum moveType {				// bf->move_type values 
	MOVE_TYPE_NULL = 0,		// null move - nothing should be NULL
	MOVE_TYPE_LINE,			// simple line
	MOVE_TYPE_ALINE,		// acceleration planned line
	MOVE_TYPE_DWELL,		// delay with no movement
	MOVE_TYPE_MCODE,		// M code execution
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
	MOVE_STATE_TAIL			// aline() deceleration portions
};
#define MOVE_STATE_RUN1 MOVE_STATE_RUN // a convenience

/*
 *	Most of these factors are the result of a lot of tweaking. Change with caution.
 */

/* MM_PER_ARC_SEGMENT	Arc segment size (mm). 
 * MIN_LINE_LENGTH		Smallest line the system can plan (mm)
 * MIN_SEGMENT_LENGTH	Smallest accel/decel segment (mm). Set to produce ~10 ms segments
 *
 * The following must apply:
 *	  MM_PER_ARC_SEGMENT >= MIN_LINE_LENGTH >= MIN_SEGMENT_LENGTH 
 */
#define MM_PER_ARC_SEGMENT 0.1		// 0.03
#define MIN_LINE_LENGTH 0.08		// 0.02
#define MIN_SEGMENT_LENGTH 0.05		// 0.01

/* ESTD_SEGMENT_USEC	 Microseconds per planning segment
 *	Should be experimentally adjusted if the MIN_SEGMENT_LENGTH is changed
 */
#define ESTD_SEGMENT_USEC 10000
#define MIN_ARC_SEGMENT_USEC 20000
#define MIN_SEGMENT_SEC (ESTD_SEGMENT_USEC/1000000)	// a convenience


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

/* PLANNER_ITERATION_MAX
 * PLANNER_ITERATION_ERROR_PERCENT
 *	Max iterations for convergence in the HT asymmetric case.
 *	Error percentage for iteration convergence. As percent - 0.01 = 1%
 */
#define PLANNER_ITERATION_MAX 10
#define PLANNER_ITERATION_ERROR_PERCENT 0.10

/* PLANNER_VELOCITY_TOLERANCE
 * PLANNER_LENGTH_TOLERANCE
 *	Tolerance below which velocities are considered equal for planning 
 *	purposes (mm/min)
 *	Tolerance below which lengths are considered equal for *comparison* 
 *	purposes only
 */
#define PLANNER_VELOCITY_TOLERANCE 2
#define PLANNER_LENGTH_TOLERANCE 0.05

/* PLANNER_LENGTH_FACTOR
 *	Length factor over which an HB or BT should be treated as an HT case.
 *	The amount over 1.00 is the maximum cruise length relative to the head or 
 *	tail length. For example, a setting of 1.5 and a head length of 0.4 mm would 
 *	plan lines up to 0.6 mm as HB cases. Longer than this would be planned as HT
 *	cases. This must be at least 1.00.
 */
#define PLANNER_LENGTH_FACTOR 1.25

/*
 *	Useful macros
 */

#define MP_LINE(t,m) ((cfg.enable_acceleration == TRUE) ? mp_aline(t,m) : mp_line(t,m))

/*
 * Global Scope Functions
 */

void mp_init(void);

uint8_t mp_isbusy(void);
void mp_flush_planner(void);
uint8_t mp_test_write_buffer(void);
double *mp_get_plan_position(double position[]);
void mp_set_plan_position(const double position[]);
void mp_set_axis_position(const double position[]);
double mp_get_runtime_position(uint8_t axis);
double mp_get_runtime_velocity(void);
double mp_get_runtime_linenum(void);
void mp_zero_segment_velocity(void);

uint8_t mp_exec_move(void);
void mp_queue_mcode(uint8_t mcode);

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

//#define __UNIT_TEST_PLANNER
#ifdef __UNIT_TEST_PLANNER
void mp_unit_tests(void);
void mp_plan_arc_unit_tests(void);
#define	PLANNER_UNITS mp_unit_tests();
#else
#define	PLANNER_UNITS
#endif // __UNIT_TEST_PLANNER

#endif
