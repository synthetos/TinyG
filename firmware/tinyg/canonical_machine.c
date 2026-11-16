/*
 * canonical_machine.c - rs274/ngc canonical machine.
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2016 Robert Giseburt
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
/*
 * 	This code is a loose implementation of Kramer, Proctor and Messina's canonical
 *	machining functions as described in the NIST RS274/NGC v3
 *
 *	The canonical machine is the layer between the Gcode parser and the motion control
 *	code for a specific robot. It keeps state and executes commands - passing the
 *	stateless commands to the motion planning layer.
 */
/* --- System state contexts - Gcode models ---
 *
 *	Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *	There are 3 temporal contexts for system state:
 *	  - The gcode model in the canonical machine (the MODEL context, held in gm)
 *	  - The gcode model used by the planner (PLANNER context, held in bf's and mm)
 *	  - The gcode model used during motion for reporting (RUNTIME context, held in mr)
 *
 *	It's a bit more complicated than this. The 'gm' struct contains the core Gcode model
 *	context. This originates in the canonical machine and is copied to each planner buffer
 *	(bf buffer) during motion planning. Finally, the gm context is passed to the runtime
 *	(mr) for the RUNTIME context. So at last count the Gcode model exists in as many as
 *	30 copies in the system. (1+28+1)
 *
 *	Depending on the need, any one of these contexts may be called for reporting or by
 *	a function. Most typically, all new commends from the gcode parser work form the MODEL
 *	context, and status reports pull from the RUNTIME while in motion, and from MODEL when
 *	at rest. A convenience is provided in the ACTIVE_MODEL pointer to point to the right
 *	context.
 */
/* --- Synchronizing command execution ---
 *
 *	Some gcode commands only set the MODEL state for interpretation of the current Gcode
 *	block. For example, cm_set_feed_rate(). This sets the MODEL so the move time is
 *	properly calculated for the current (and subsequent) blocks, so it's effected
 *	immediately.
 *
 *	"Synchronous commands" are commands that affect the runtime need to be synchronized
 *	with movement. Examples include G4 dwells, program stops and ends, and most M commands.
 *	These are queued into the planner queue and execute from the queue. Synchronous commands
 *	work like this:
 *
 *	  - Call the cm_xxx_xxx() function which will do any input validation and return an
 *		error if it detects one.
 *
 *	  - The cm_ function calls mp_queue_command(). Arguments are a callback to the _exec_...()
 *		function, which is the runtime execution routine, and any arguments that are needed
 *		by the runtime. See typedef for *exec in planner.h for details
 *
 *	  - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *	  - When planner execution reaches the buffer it executes the callback w/ the args.
 *		Take careful note that the callback executes under an interrupt, so beware of
 *		variables that may need to be volatile.
 *
 *	Note:
 *	  - The synchronous command execution mechanism uses 2 vectors in the bf buffer to store
 *		and return values for the callback. It's obvious, but impractical to pass the entire
 *		bf buffer to the callback as some of these commands are actually executed locally
 *		and have no buffer.
 */

#include "tinyg.h"			// #1
#include "config.h"			// #2
#include "canonical_machine.h"
#include "cm_internal.h"
#include "controller.h"
#include "text_parser.h"

#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "runtime.h"
#include "feedlimit.h"			// FL_CLS_COOLANT (feed-limit advisor step 3)
#include "override.h"
#include "report.h"
#include "gpio.h"
#include "switch.h"
#include "hardware.h"
#include "util.h"
#include "xio.h"			// for serial queue flush

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmSingleton_t cm;		// canonical machine controller singleton

/*************************************************************************
 * CANONICAL MACHINING FUNCTIONS
 *	Values are passed in pre-unit_converted state (from gn structure)
 *	All operations occur on gm (current model state)
 *
 * These are organized by section number (x.x.x) in the order they are
 * found in NIST RS274 NGCv3
 ************************************************************************/

/******************************************
 * Initialization and Termination (4.3.2) *
 ******************************************/
/*
 * canonical_machine_init() - Config init cfg_init() must have been run beforehand
 * canonical_machine_reset()
 */

void canonical_machine_init()
{
// If you can assume all memory has been zeroed by a hard reset you don't need this memset code.
// Do not reset canonicalMachineSingleton once it's been initialized - which would be:
//	memset(&cm, 0, sizeof(cm));	                    // do not reset canonicalMachineSingleton once it's been initialized
	memset(&cm.gm, 0, sizeof(GCodeState_t));	    // clear the Gcode MODEL of all values, pointers and status
	memset(&cm.gn, 0, sizeof(GCodeInput_t));        // clear the Gcode new data struct
	memset(&cm.gf, 0, sizeof(GCodeFlags_t));        // clear the Gcode data flags struct

	canonical_machine_init_assertions();		    // establish assertions
	ACTIVE_MODEL = MODEL;						    // setup initial Gcode model pointer

	// sub-system inits
	spindle_init();
	cm_arc_init();
    canonical_machine_reset();                      // always finish an init with reset
}

void canonical_machine_reset()
{
	// set gcode defaults
	cm_set_units_mode(cm.default_units_mode);
	cm_set_coord_system(cm.default_coord_system);
	cm_select_plane(cm.default_select_plane);
	cm_set_path_control(cm.default_path_control);
	cm_set_distance_mode(cm.default_distance_mode);
	cm_set_arc_distance_mode(INCREMENTAL_MODE);     // always the default
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);   // always the default
//    cm_reset_overrides();                         // set overrides to initial conditions

    // NOTE: Should unhome axes here

	// reset request flags
//	cm.feedhold_requested = false;                  //+++++ TO BE REMOVED
//	cm.queue_flush_requested = false;               //+++++ TO BE REMOVED
//	cm.cycle_start_requested = false;               //+++++ TO BE REMOVED

    cm.queue_flush_state = FLUSH_OFF;
    cm.queue_flush_input = false;
    cm.queue_flush_input_due = false;
    cm.program_paused = false;
    cm.end_hold_requested = false;
    cm.limit_requested = 0;                     // resets switch closures that occurred during initialization
    cm.safety_interlock_disengaged = 0;         // ditto
    cm.safety_interlock_reengaged = 0;          // ditto
    cm.shutdown_requested = 0;                  // ditto

	// signal that the machine is ready for action
    cm.cycle_state = CYCLE_OFF;
    cm.motion_state = MOTION_STOP;
    cm.hold_state = FEEDHOLD_OFF;
	cm.gmx.block_delete_switch = true;
	cm.gm.motion_mode = MOTION_MODE_CANCEL;     // never start a machine in a motion mode
	cm.machine_state = MACHINE_READY;
}

/*
 * canonical_machine_init_assertions()
 * canonical_machine_test_assertions() - test assertions, return error code if violation exists
 */

void canonical_machine_init_assertions(void)
{
	cm.magic_start = MAGICNUM;
	cm.magic_end = MAGICNUM;
	cm.gmx.magic_start = MAGICNUM;
	cm.gmx.magic_end = MAGICNUM;
	arc.magic_start = MAGICNUM;
	arc.magic_end = MAGICNUM;
}

stat_t canonical_machine_test_assertions(void)
{
    if ((BAD_MAGIC(cm.magic_start)) || (BAD_MAGIC(cm.magic_end)) ||
        (BAD_MAGIC(cm.gmx.magic_start)) || (BAD_MAGIC(cm.gmx.magic_end)) ||
        (BAD_MAGIC(arc.magic_start)) || (BAD_MAGIC(arc.magic_end))) {
        return(cm_panic_P(STAT_CANONICAL_MACHINE_ASSERTION_FAILURE, PSTR("canonical_machine_test_assertions()")));
    }
    return (STAT_OK);
}

/*****************************
 * Free Space Motion (4.3.4) *
 *****************************/
/*
 * cm_straight_traverse() - G0 linear rapid
 */

/*
 * _queue_line() - plan a move (cm.gm, or a refused short move) inside a machining cycle (G0/G1)
 *
 *	The cycle starts before the move is queued so a short move cannot finish and end the cycle
 *	before it was started. But mp_aline() plans before it reserves its buffer, and meanwhile the
 *	runtime can finish the previous last block and end the cycle (plan_exec.c, mp_free_run_buffer()
 *	then cm_cycle_end()). So once the move is queued, check again with interrupts off: restart the
 *	cycle if anything is queued or running, end it if nothing is (a too-short or rejected move).
 */
static stat_t _queue_line(GCodeState_t *gm)
{
	cm_cycle_start();
	stat_t status = mp_aline(gm);
	if (cm_is_alarmed() != STAT_OK) {			// mp_aline can alarm or panic (failed reservation): the
		return (status);						// cycle repair below must not overwrite that state
	}
#ifdef __AVR
	uint8_t sreg = SREG;
	cli();
#endif
	bool runnable = mp_has_runnable_buffer();
	if (runnable) {
		cm_cycle_start();						// cheap; must not race the runtime's cycle end
	}
#ifdef __AVR
	SREG = sreg;
#endif
	if (!runnable) {
		cm_cycle_end();							// only the main loop queues, so this can't race a new move
	}
	if ((status == STAT_MINIMUM_LENGTH_MOVE) || (status == STAT_MINIMUM_TIME_MOVE)) {
		status = STAT_OK;						// too short to plan; not an error
	}
	return (status);
}

/*
 * cm_run_short_move() - plan a move mp_aline() refused as too short, before anything else is queued
 * cm_short_move_callback() - plan it once at most the running block is queued
 *
 *	mp_aline() refuses a move under the minimum block time while others are queued, and the model still
 *	advances (cm_finalize_move()), so the next move starts from the planner position and covers it. When
 *	no move follows, the tool would stop short while the position report (the model at rest) claims the
 *	endpoint (N87). The rest of the way is planned at the minimum time: at a dwell or a queued command,
 *	before G28.3 rebases the model and planner, or when the queue has drained to the running block (the
 *	machine stops there anyway). Rebases from the runtime (flush, homing, probing) discard it on purpose.
 */
void cm_run_short_move()
{
	if (!mm.short_move_pending || (cm_is_alarmed() != STAT_OK) || (cm.queue_flush_state != FLUSH_OFF)) {
		return;									// a flush resyncs the model and clears it
	}
	mm.short_move_pending = false;
	GCodeState_t gm;
	memcpy(&gm, &cm.gm, sizeof(GCodeState_t));
	copy_vector(gm.target, cm.gmx.position);	// where the model already is
	if ((gm.feed_rate_mode == INVERSE_TIME_MODE) || !(gm.feed_rate > 0)) {
		gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;	// no feed to use (G93 clears it after each move)
	}
	gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
	mm.short_move_run = true;
	_queue_line(&gm);
	mm.short_move_run = false;
}

stat_t cm_short_move_callback()
{
	if (mm.short_move_pending && (mp_get_planner_buffers_available() >= PLANNER_BUFFER_POOL_SIZE - 1)) {
		cm_run_short_move();
	}
	return (STAT_OK);
}

static stat_t _straight_traverse(const float target[], const bool flags[], const bool machine_coordinates)
{
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;

    // it's legal for a G0 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] || flags[AXIS_Y] || flags[AXIS_Z] ||
          flags[AXIS_A] || flags[AXIS_B] || flags[AXIS_C])) {
        return(STAT_OK);
    }
    if (machine_coordinates) {
        copy_vector(cm.gm.target, cm.gmx.position);
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flags[axis] && (cm.a[axis].axis_mode != AXIS_DISABLED)) {
                cm.gm.target[axis] = target[axis];  // stored machine coordinates are already mm/degrees
            }
        }
    } else {
        ritorno(cm_set_model_target(target, flags));
    }

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) {
        return (cm_alarm(status, cs.saved_buf));
    }
	// prep and plan the move
	cm_set_work_offsets();				        // capture the fully resolved offsets to the state

    status = _queue_line(&cm.gm);               // plan the move inside a machining cycle
    if (status != STAT_OK) {
        copy_vector(cm.gm.target, cm.gmx.position);
        return (status);
    }
    cm_finalize_move();
	return (status);
}

stat_t cm_straight_traverse(const float target[], const bool flags[])
{
    return (_straight_traverse(target, flags, false));
}

/*
 * cm_set_g28_position()  - G28.1
 * cm_goto_g28_position() - G28
 * cm_set_g30_position()  - G30.1
 * cm_goto_g30_position() - G30
 */

static stat_t _goto_stored_position(const float stored[], const float target[], const bool flags[])
{
    // Earlier commands in this line have used part of the dispatch reserve.
    // Require only these two legs, before either is queued.
    if (mp_get_planner_buffers_available() < 2) {
        return (STAT_BUFFER_FULL);
    }
    uint8_t saved_override = cm.gm.absolute_override;
    cm_set_absolute_override(false);
    stat_t status = cm_straight_traverse(target, flags); // intermediate point uses work coordinates
    if (status == STAT_OK) {
        cm_set_absolute_override(true);
        status = _straight_traverse(stored, FLAGS_ALL, true);
    }
    cm_set_absolute_override(saved_override);
    return (status);
}

stat_t cm_goto_g28_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g28_position, target, flags));
}

stat_t cm_goto_g30_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm.gmx.g30_position, target, flags));
}

/*******************************
 * Machining Functions (4.3.6) *
 *******************************/
/*
 * cm_arc_feed() - SEE plan_arc.c(pp)
 */

/*
 * cm_dwell() - G4, P parameter (seconds)
 */
stat_t cm_dwell(const float seconds)
{
	if (seconds < 0) { return (STAT_P_WORD_IS_NEGATIVE); }
    // _exec_dwell converts seconds to uint32 microseconds before passing the timer prep.
    if (!isfinite(seconds) || (seconds > 4294.0f)) {
        nv_add_message_P(PSTR("dwell maximum 4294 seconds"));
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
	mp_dwell(seconds);
	return (STAT_OK);
}

/*
 * cm_straight_feed() - G1
 */
stat_t cm_straight_feed(const float target[], const bool flags[])
{
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

    // it's legal for a G1 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] || flags[AXIS_Y] || flags[AXIS_Z] ||
          flags[AXIS_A] || flags[AXIS_B] || flags[AXIS_C])) {
        return(STAT_OK);
    }
	// trap zero feed rate condition
	if ((cm.gm.feed_rate_mode == INVERSE_TIME_MODE) ? (cm.gm.feed_rate <= 0) : fp_ZERO(cm.gm.feed_rate)) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}
	ritorno(cm_set_model_target(target, flags));

	// test soft limits
	stat_t status = cm_test_soft_limits(cm.gm.target);
	if (status != STAT_OK) {
        return (cm_alarm(status, cs.saved_buf));
    }
	// prep and plan the move
	cm_set_work_offsets();                      // capture the fully resolved offsets to the state
    status = _queue_line(&cm.gm);               // plan the move inside a machining cycle
    if (status != STAT_OK) {
        copy_vector(cm.gm.target, cm.gmx.position);
        return (status);
    }
	cm_finalize_move();
	return (status);
}

/*****************************
 * Spindle Functions (4.3.7) *
 *****************************/
// see spindle.c, spindle.h
