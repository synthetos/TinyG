/*
 * planner.h - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
	MOVE_TYPE_ALINE,		// acceleration planned line
	MOVE_TYPE_DWELL,		// delay with no movement
	MOVE_TYPE_COMMAND,		// general command
	MOVE_TYPE_TOOL,			// T command
	MOVE_TYPE_SPINDLE_SPEED,// S command
	MOVE_TYPE_STOP,			// program stop
	MOVE_TYPE_END			// program end
};

enum moveState {
	MOVE_STATE_OFF = 0,		// move inactive (MUST BE ZERO)
	MOVE_STATE_NEW,			// general value if you need an initialization
	MOVE_STATE_RUN,			// general run state (for non-acceleration moves) 
	MOVE_STATE_RUN2,		// used for sub-states
	MOVE_STATE_HEAD,		// aline() acceleration portions
	MOVE_STATE_BODY,		// aline() cruise portions
	MOVE_STATE_TAIL,		// aline() deceleration portions
	MOVE_STATE_SKIP,		// mark a skipped block
	MOVE_STATE_END			// move is marked as done (used by dwells)
};
#define MOVE_STATE_RUN1 MOVE_STATE_RUN // a convenience

/*** Most of these factors are the result of a lot of tweaking. Change with caution.***/

/* The following must apply:
 *	  MM_PER_ARC_SEGMENT >= MIN_LINE_LENGTH >= MIN_SEGMENT_LENGTH 
 */
#define ARC_SEGMENT_LENGTH 	0.1		// Arc segment size (mm).(0.03)
#define MIN_LINE_LENGTH 	0.08	// Smallest line the system can plan (mm) (0.02)
#define MIN_SEGMENT_LENGTH 	0.05	// Smallest accel/decel segment (mm). Set to produce ~10 ms segments (0.01)

#define JERK_MATCH_PRECISION 1000	// precision to which jerk must match to be considered effectively the same

/* ESTD_SEGMENT_USEC	 Microseconds per planning segment
 *	Should be experimentally adjusted if the MIN_SEGMENT_LENGTH is changed
 */
#define NOM_SEGMENT_USEC 		((float)5000)		// nominal segment time
#define MIN_SEGMENT_USEC 		((float)2500)		// minimum segment time
#define MIN_ARC_SEGMENT_USEC	((float)10000)		// minimum arc segment time
#define NOM_SEGMENT_TIME 		(MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_SEGMENT_TIME 		(MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_ARC_SEGMENT_TIME 	(MIN_ARC_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_LENGTH_MOVE 		(EPSILON)
#define MIN_TIME_MOVE  			((float)0.0000001)

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
#define PLANNER_BUFFER_POOL_SIZE 28
#define PLANNER_BUFFER_HEADROOM 4			// buffers to reserve in planner before processing new input line

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
 *	Macros and typedefs
 */

//#define MP_LINE(t,m,o,n) ((cfg.enable_acceleration == TRUE) ? mp_aline(t,m,o,n) : mp_line(t,m))
#define MP_LINE(t,m,o,n) (mp_aline(t,m,o,n))	// non-planned lines are disabled

//typedef void (*cm_exec)(uint8_t, float);	// callback to canonical_machine execution function
//typedef void (*cm_exec)(float[], float[]);	// callback to canonical_machine execution function
// callback to canonical_machine execution function
// vars map to move_code, time, target[] and unit[], respectively
typedef void (*cm_exec)(uint8_t, float, float[], float[]);	

/*
 *	Planner structures
 */

// All the enums that equal zero must be zero. Don't change this

enum mpBufferState {			// bf->buffer_state values 
	MP_BUFFER_EMPTY = 0,		// struct is available for use (MUST BE 0)
	MP_BUFFER_LOADING,			// being written ("checked out")
	MP_BUFFER_QUEUED,			// in queue
	MP_BUFFER_PENDING,			// marked as the next buffer to run
	MP_BUFFER_RUNNING			// current running buffer
};

typedef struct mpBuffer {		// See Planning Velocity Notes for variable usage
	struct mpBuffer *pv;		// static pointer to previous buffer
	struct mpBuffer *nx;		// static pointer to next buffer
	stat_t (*bf_func)(struct mpBuffer *bf); // callback to buffer exec function
	cm_exec cm_func;			// callback to canonical machine execution function
	uint32_t linenum;			// runtime line number; or line index if not numbered
	uint8_t motion_mode;		// runtime motion mode for status reporting
	uint8_t buffer_state;		// used to manage queueing/dequeueing
	uint8_t move_type;			// used to dispatch to run routine
	uint8_t move_code;			// byte that can be used by used exec functions
	uint8_t move_state;			// move state machine sequence
	uint8_t replannable;		// TRUE if move can be replanned

	float target[AXES];			// target position in floating point
	float unit[AXES];			// unit vector for axis scaling & planning
	float work_offset[AXES];	// offset from the work coordinate system (for reporting only)

	float time;					// line, helix or dwell time in minutes
	float min_time;				// minimum time for the move - for rate override replanning
	float head_length;
	float body_length;
	float tail_length;
	float length;				// total length of line or helix in mm
								// *** SEE NOTES ON THESE VARIABLES, in aline() ***
	float entry_velocity;		// entry velocity requested for the move
	float cruise_velocity;		// cruise velocity requested & achieved
	float exit_velocity;		// exit velocity requested for the move

	float entry_vmax;			// max junction velocity at entry of this move
	float cruise_vmax;			// max cruise velocity requested for move
	float exit_vmax;			// max exit velocity possible (redundant)
	float delta_vmax;			// max velocity difference for this move
	float braking_velocity;		// current value for braking velocity

	float jerk;					// maximum linear jerk term for this move
	float recip_jerk;			// 1/Jm used for planning (compute-once)
	float cbrt_jerk;			// cube root of Jm used for planning (compute-once)
} mpBuf_t;
#define int_val move_code		// alias for uint8_t to the move_code
#define dbl_val time			// alias for float to the time variable

typedef struct mpBufferPool {	// ring buffer for sub-moves
	uint16_t magic_start;		// magic number to test memory integity	
	uint8_t buffers_available;	// running count of available buffers
	mpBuf_t *w;					// get_write_buffer pointer
	mpBuf_t *q;					// queue_write_buffer pointer
	mpBuf_t *r;					// get/end_run_buffer pointer
	mpBuf_t bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage
	uint16_t magic_end;
} mpBufferPool_t;

typedef struct mpMoveMasterSingleton {	// common variables for planning (move master)
	float position[AXES];		// final move position for planning purposes
	float ms_in_queue;			// total ms of movement & dwell in planner queue
	float prev_jerk;			// jerk values cached from previous move
	float prev_recip_jerk;
	float prev_cbrt_jerk;
#ifdef __UNIT_TEST_PLANNER
	float test_case;
	float test_velocity;
	float a_unit[AXES];
	float b_unit[AXES];
#endif
} mpMoveMasterSingleton_t;

typedef struct mpMoveRuntimeSingleton {	// persistent runtime variables
//	uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
	uint16_t magic_start;		// magic number to test memory integity	
	uint32_t linenum;			// runtime line/block number of BF being executed
	uint8_t motion_mode;		// runtime motion mode for status reports
	uint8_t move_state;			// state of the overall move
	uint8_t section_state;		// state within a move section

	float endpoint[AXES];		// final target for bf (used to correct rounding errors)
	float position[AXES];		// current move position
	float target[AXES];			// target move position
	float unit[AXES];			// unit vector for axis scaling & planning
	float work_offset[AXES];	// offset from the work coordinate system (for reporting only)

	float head_length;			// copies of bf variables of same name
	float body_length;
	float tail_length;
	float entry_velocity;
	float cruise_velocity;
	float exit_velocity;

	float length;				// length of line in mm
	float move_time;			// total running time (derived)
	float midpoint_velocity;	// velocity at accel/decel midpoint
	float jerk;					// max linear jerk

	float segments;				// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	float segment_move_time;	// actual time increment per aline segment
	float microseconds;			// line or segment time in microseconds
	float segment_length;		// computed length for aline segment
	float segment_velocity;		// computed velocity for aline segment
	float forward_diff_1;      // forward difference level 1 (Acceleration)
	float forward_diff_2;      // forward difference level 2 (Jerk - constant)
	uint16_t magic_end;
} mpMoveRuntimeSingleton_t;


// Allocate global scope structs
mpBufferPool_t mb;				// move buffer queue
mpMoveMasterSingleton_t mm;		// context for line planning
mpMoveRuntimeSingleton_t mr;	// context for line runtime

/*
 * Global Scope Functions
 */

void mp_init(void);
void mp_init_buffers(void);

void mp_flush_planner(void);
void mp_set_planner_position(uint8_t axis, const float position);
void mp_set_runtime_position(uint8_t axis, const float position);
//float *mp_get_plan_position_vector(float position[]);
//void mp_set_plan_position_vector(const float position[]);
//void mp_set_axes_position(const float position[]);
//void mp_set_axis_position(uint8_t axis, const float position);

stat_t mp_exec_move(void);
//void mp_queue_command(void(*cm_exec)(uint8_t, float), uint8_t int_val, float float_val);
//void mp_queue_command(void(*cm_exec)(float[], float[]), uint8_t int_val, float float_val);
//void mp_queue_command(void(*cm_exec)(float[], float[]), float int_val, float float_val);
//void mp_queue_command(void(*cm_exec)(void *, void *), void *int_val, void *float_val);
void mp_queue_command(void(*cm_exec)(uint8_t, float, float[], float[]), 
		uint8_t int_val, float float_val, float *vector, float *flag);
//void mp_queue_command_vector(void(*cm_exec)(uint8_t, float), float vector[], float flag[]);

stat_t mp_dwell(const float seconds);
void mp_end_dwell(void);
stat_t mp_aline(const float target[], const float minutes, const float work_offset[], const float min_time);
stat_t mp_plan_hold_callback(void);
stat_t mp_end_hold(void);
stat_t mp_feed_rate_override(uint8_t flag, float parameter);

// planner buffer handlers
uint8_t mp_get_planner_buffers_available(void);
void mp_clear_buffer(mpBuf_t *bf); 
void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp);
void mp_queue_write_buffer(const uint8_t move_type);
void mp_free_run_buffer(void);
mpBuf_t * mp_get_write_buffer(void); 
mpBuf_t * mp_get_run_buffer(void);
mpBuf_t * mp_get_first_buffer(void);
mpBuf_t * mp_get_last_buffer(void);
#define mp_get_prev_buffer(b) ((mpBuf_t *)(b->pv))
#define mp_get_next_buffer(b) ((mpBuf_t *)(b->nx))

// plan_line.c functions
uint8_t mp_isbusy(void);
uint8_t mp_get_runtime_motion_mode(void);
float mp_get_runtime_linenum(void);
float mp_get_runtime_velocity(void);
float mp_get_runtime_work_position(uint8_t axis);
float mp_get_runtime_machine_position(uint8_t axis);
float mp_get_runtime_work_offset(uint8_t axis);
float mp_get_runtime_work_scaling(uint8_t axis);
void mp_set_runtime_work_offset(float offset[]); 
void mp_zero_segment_velocity(void);

#ifdef __DEBUG
void mp_dump_running_plan_buffer(void);
void mp_dump_plan_buffer_by_index(uint8_t index);
void mp_dump_runtime_state(void);
#endif

/*** Unit tests ***/

//#define __UNIT_TEST_PLANNER	// uncomment to compile in planner unit tests
#ifdef __UNIT_TEST_PLANNER
void mp_unit_tests(void);
void mp_plan_arc_unit_tests(void);
#define	PLANNER_UNITS mp_unit_tests();
#else
#define	PLANNER_UNITS
#endif // end __UNIT_TEST_PLANNER

#endif

