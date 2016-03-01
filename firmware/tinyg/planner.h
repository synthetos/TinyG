/*
 * planner.h - cartesian trajectory planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PLANNER_H_ONCE
#define PLANNER_H_ONCE

#include "canonical_machine.h"	// used for GCodeState_t

/*
 * Enums and other type definitions
 *
 * All the enums that equal zero must be zero. Don't change them
 */

typedef void (*cm_exec_t)(float[], bool[]);	// callback to canonical_machine execution function

typedef enum {				        // bf->buffer_state values
    MP_BUFFER_EMPTY = 0,			// struct is available for use (MUST BE 0)
    MP_BUFFER_LOADING,				// being written ("checked out")
    MP_BUFFER_QUEUED,				// in queue
    MP_BUFFER_PENDING,				// marked as the next buffer to run
    MP_BUFFER_RUNNING				// current running buffer
} bufferState;

typedef enum {				        // bf->block_type values
	BLOCK_TYPE_NULL = 0,		    // null move - does a no-op
	BLOCK_TYPE_ALINE,		        // acceleration planned line
	BLOCK_TYPE_DWELL,		        // delay with no movement
	BLOCK_TYPE_COMMAND,		        // general command
	BLOCK_TYPE_TOOL,                // T command
	BLOCK_TYPE_SPINDLE_SPEED,       // S command
	BLOCK_TYPE_STOP,                // program stop
	BLOCK_TYPE_END                  // program end
} blockType;

typedef enum {
    BLOCK_IDLE = 0,                 // block is inactive (MUST BE ZERO)
    BLOCK_INITIALIZING,             // uninitialized run state
    BLOCK_RUNNING                   // block is in run state
} blockState;

typedef enum {
	SECTION_HEAD = 0,		        // acceleration
	SECTION_BODY,			        // cruise
	SECTION_TAIL			        // deceleration
} moveSection;
#define SECTIONS 3

typedef enum {
	SECTION_OFF = 0,		        // section inactive
	SECTION_NEW,			        // uninitialized section
	SECTION_1st_HALF,		        // first half of S curve
	SECTION_2nd_HALF		        // second half of S curve or running a BODY (cruise)
} sectionState;

/*** Most of these factors are the result of a lot of tweaking. Change with caution.***/

#define PLANNER_BUFFER_POOL_SIZE 32
#define PLANNER_BUFFER_HEADROOM 4			// buffers to reserve in planner before processing new input line

#define ARC_SEGMENT_LENGTH      ((float)0.1)		// Arc segment size (mm).(0.03)
#define MIN_ARC_RADIUS          ((float)0.1)

#define JERK_MULTIPLIER         ((float)1000000)
#define JERK_MATCH_PRECISION    ((float)1000)		// precision to which jerk must match to be considered effectively the same

#define NOM_SEGMENT_USEC        ((float)5000)		// nominal segment time
#define MIN_SEGMENT_USEC        ((float)2500)		// minimum segment time / minimum move time
#define MIN_ARC_SEGMENT_USEC    ((float)10000)		// minimum arc segment time

#define NOM_SEGMENT_TIME        (NOM_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_SEGMENT_TIME        (MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_ARC_SEGMENT_TIME    (MIN_ARC_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_TIME_MOVE           MIN_SEGMENT_TIME 	// minimum time a move can be is one segment
#define MIN_BLOCK_TIME          MIN_SEGMENT_TIME	// factor for minimum size Gcode block to process

#define MIN_SEGMENT_TIME_PLUS_MARGIN ((MIN_SEGMENT_USEC+1) / MICROSECONDS_PER_MINUTE)

/* Some parameters for _generate_trapezoid()
 * TRAPEZOID_ITERATION_MAX	 				Max iterations for convergence in the HT asymmetric case.
 * TRAPEZOID_ITERATION_ERROR_PERCENT		Error percentage for iteration convergence. As percent - 0.01 = 1%
 * TRAPEZOID_LENGTH_FIT_TOLERANCE			Tolerance for "exact fit" for H and T cases
 * TRAPEZOID_VELOCITY_TOLERANCE				Adaptive velocity tolerance term
 */
#define TRAPEZOID_ITERATION_MAX				10
#define TRAPEZOID_ITERATION_ERROR_PERCENT	((float)0.10)
#define TRAPEZOID_LENGTH_FIT_TOLERANCE		((float)0.0001)	// allowable mm of error in planning phase
#define TRAPEZOID_VELOCITY_TOLERANCE		(max(2,bf->entry_velocity/100))

/*
 *	Planner structures
 */

typedef struct mpBuffer {			// See Planning Velocity Notes for variable usage
	struct mpBuffer *pv;			// static pointer to previous buffer
	struct mpBuffer *nx;			// static pointer to next buffer
	stat_t (*bf_func)(struct mpBuffer *bf); // callback to buffer exec function
	cm_exec_t cm_func;				// callback to canonical machine execution function

	float naiive_move_time;

#ifdef __BITFIELDS
    bufferState buffer_state : 3 ;  // used to manage queuing/dequeuing
    blockType block_type     : 3 ;  // used to dispatch to run routine
    blockState block_state   : 2 ;  // move state machine sequence
	uint8_t replannable      : 1 ;  // TRUE if move can be re-planned
	uint8_t jerk_axis        : 3 ;  // rate limiting axis used to compute jerk for the move
#else
    bufferState buffer_state ;  // used to manage queuing/dequeuing
    blockType block_type     ;  // used to dispatch to run routine
    blockState block_state   ;  // move state machine sequence
    uint8_t replannable      ;  // TRUE if move can be re-planned
    uint8_t jerk_axis        ;  // rate limiting axis used to compute jerk for the move
#endif

//	uint8_t block_code;				// byte that can be used by used exec functions

	float unit[AXES];				// unit vector for axis scaling & planning
    bool axis_flags[AXES];          // set true for axes participating in the move & for command parameters

	float length;					// total length of line or helix in mm
	float head_length;
	float body_length;
	float tail_length;
									// *** SEE NOTES ON THESE VARIABLES, in aline() ***
	float entry_velocity;			// entry velocity requested for the move
	float cruise_velocity;			// cruise velocity requested & achieved
	float exit_velocity;			// exit velocity requested for the move

	float entry_vmax;				// max junction velocity at entry of this move
	float cruise_vmax;				// max cruise velocity requested for move
	float exit_vmax;				// max exit velocity possible (redundant)
	float delta_vmax;				// max velocity difference for this move
	float braking_velocity;			// current value for braking velocity

	float jerk;						// maximum linear jerk term for this move
	float recip_jerk;				// 1/Jm used for planning (computed and cached)
	float cbrt_jerk;				// cube root of Jm used for planning (computed and cached)

	GCodeState_t gm;				// Gode model state - passed from model, used by planner and runtime

} mpBuf_t;

typedef struct mpBufferPool {		// ring buffer for sub-moves
	magic_t magic_start;			// magic number to test memory integrity
	uint8_t buffers_available;		// running count of available buffers
	mpBuf_t *w;						// get_write_buffer pointer
	mpBuf_t *q;						// queue_write_buffer pointer
	mpBuf_t *r;						// get/end_run_buffer pointer
	mpBuf_t bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage
	magic_t magic_end;
} mpBufferPool_t;

typedef struct mpMoveMasterSingleton { // common variables for planning (move master)
	magic_t magic_start;			// magic number to test memory integrity
	float position[AXES];			// final move position for planning purposes

	float jerk;						// jerk values cached from previous block
	float recip_jerk;
	float cbrt_jerk;

	magic_t magic_end;
} mpMoveMasterSingleton_t;

typedef struct mpMoveRuntimeSingleton {	// persistent runtime variables
//	uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
	magic_t magic_start;                // magic number to test memory integrity
	blockState block_state;             // state of the overall move
	moveSection section;                // what section is the move in?
	sectionState section_state;         // state within a move section

	float unit[AXES];                   // unit vector for axis scaling & planning
    bool axis_flags[AXES];              // set true for axes participating in the move
	float target[AXES];                 // final target for bf (used to correct rounding errors)
	float position[AXES];               // current move position
	float waypoint[SECTIONS][AXES];     // head/body/tail endpoints for correction

	float target_steps[MOTORS];         // current MR target (absolute target as steps)
	float position_steps[MOTORS];       // current MR position (target from previous segment)
	float commanded_steps[MOTORS];      // will align with next encoder sample (target from 2nd previous segment)
	float encoder_steps[MOTORS];        // encoder position in steps - ideally the same as commanded_steps
	float following_error[MOTORS];      // difference between encoder_steps and commanded steps

	float head_length;                  // copies of bf variables of same name
	float body_length;
	float tail_length;

	float entry_velocity;
	float cruise_velocity;
	float exit_velocity;

	float segments;                     // number of segments in line (also used by arc generation)
	uint32_t segment_count;             // count of running segments
	float segment_velocity;             // computed velocity for aline segment
	float segment_time;                 // actual time increment per aline segment

	float forward_diff_1;               // forward difference level 1
	float forward_diff_2;               // forward difference level 2
	float forward_diff_3;               // forward difference level 3
	float forward_diff_4;               // forward difference level 4
	float forward_diff_5;               // forward difference level 5

	GCodeState_t gm;                    // gcode model state currently executing

	magic_t magic_end;
} mpMoveRuntimeSingleton_t;

// Reference global scope structures
extern mpBufferPool_t mb;				// move buffer queue
extern mpMoveMasterSingleton_t mm;		// context for line planning
extern mpMoveRuntimeSingleton_t mr;		// context for line runtime

/*
 * Global Scope Functions
 */

void planner_init(void);
void planner_reset(void);
void planner_init_assertions(void);
stat_t planner_test_assertions(void);

void mp_halt_runtime(void);
void mp_flush_planner(void);
void mp_set_planner_position(uint8_t axis, const float position);
void mp_set_runtime_position(uint8_t axis, const float position);
void mp_set_steps_to_runtime_position(void);

void mp_queue_command(void(*cm_exec)(float[], bool[]), float *value, bool *flags);
stat_t mp_runtime_command(mpBuf_t *bf);

stat_t mp_dwell(const float seconds);
void mp_end_dwell(void);
//void mp_request_out_of_band_dwell(float seconds);
//stat_t mp_exec_out_of_band_dwell(void);

// planner functions and helpers
uint8_t mp_get_planner_buffers_available(void);
bool mp_planner_is_full(void);
bool mp_has_runnable_buffer(void);
//bool mp_is_phat_city_time(void);

// planner buffer primitives
void mp_init_buffers(void);

//mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf);	// use the macro instead
//mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf);
#define mp_get_prev_buffer(b) ((mpBuf_t *)(b->pv))
#define mp_get_next_buffer(b) ((mpBuf_t *)(b->nx))

mpBuf_t * mp_get_write_buffer(void);
void mp_commit_write_buffer(const uint8_t block_type);
mpBuf_t * mp_get_run_buffer(void);
uint8_t mp_free_run_buffer(void);

mpBuf_t * mp_get_first_buffer(void);
mpBuf_t * mp_get_last_buffer(void);

//void mp_unget_write_buffer(void);
void mp_clear_buffer(mpBuf_t *bf);
void mp_copy_buffer(mpBuf_t *bf, const mpBuf_t *bp);

// plan_line.c functions
void mp_zero_segment_velocity(void);
float mp_get_runtime_velocity(void);
float mp_get_runtime_absolute_position(uint8_t axis);
float mp_get_runtime_work_position(uint8_t axis);
void mp_set_runtime_work_offset(float offset[]);
float* mp_get_planner_position_vector(void);
bool mp_get_runtime_busy(void);
bool mp_runtime_is_idle(void);

stat_t mp_aline(GCodeState_t *gm_in);
stat_t mp_plan_hold_callback(void);

stat_t mp_end_hold(void);
//stat_t mp_feed_rate_override(uint8_t flag, float parameter);

// plan_zoid.c functions
void mp_calculate_trapezoid(mpBuf_t *bf);
float mp_get_target_length(const float Vi, const float Vf, const mpBuf_t *bf);
float mp_get_target_velocity(const float Vi, const float L, const mpBuf_t *bf);

// plan_exec.c functions
stat_t mp_exec_move(void);
stat_t mp_exec_aline(mpBuf_t *bf);
void mp_exit_hold_state(void);

#endif	// End of include Guard: PLANNER_H_ONCE
