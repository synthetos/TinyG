/*
 * cycle_feedhold.c - machine lifecycle, feedhold, resume and queue flush.
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
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

// command execution callbacks from planner queue
static void _exec_program_finalize(float *value, bool *flags);
static void _exec_program_end(float *value, bool *flags);
static void _exec_program_stop(float *value, bool *flags);

static void _stop_outputs(void);
// A new stop request vetoes resume even if another '~' arrives during the planner walk.
static volatile bool hold_resume_cancelled;


/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

/*************************************
 * Internal getters and setters      *
 * Canonical Machine State functions *
 *************************************/
/*
 * cm_set_motion_state() - adjusts active model pointer as well
 */
void cm_set_motion_state(const cmMotionState motion_state)
{
    if (motion_state == MOTION_RUN) {
        rt_qt_invalidate();                     // backstop: motion starting (or resuming) ends quiescence
    }
    cm.motion_state = motion_state;

    switch (motion_state) {
        case (MOTION_STOP): { ACTIVE_MODEL = MODEL; break; }
        case (MOTION_RUN):  { ACTIVE_MODEL = RUNTIME; break; }
        case (MOTION_HOLD): { ACTIVE_MODEL = RUNTIME; break; }
    }
    cm_position_changed();                      // a report may now read the other model (E166)
}

/*
 * cm_get_machine_state()
 * cm_get_cycle_state()
 * cm_get_motion_state()
 * cm_get_hold_state()
 * cm_get_homing_state()
 */
 cmMachineState  cm_get_machine_state() { return cm.machine_state;}
 cmCycleState    cm_get_cycle_state()   { return cm.cycle_state;}
 cmMotionState   cm_get_motion_state()  { return cm.motion_state;}
 cmFeedholdState cm_get_hold_state()    { return cm.hold_state;}
 cmHomingState   cm_get_homing_state()  { return cm.homing_state;}

/*
 * cm_get_combined_state() - combines raw states into something a user might want to see
 *
 *  Note:
 *  On issuing a gcode command we call cm_cycle_start() before the motion gets queued. We don't go
 *  to MOTION_RUN until the command is executed by mp_exec_aline(), planned, queued, and started.
 *  So MOTION_STOP must actually return COMBINED_RUN to address this case, even though under some
 *  circumstances it might actually ne an exception case. Therefore this assertion isn't valid:
 *      cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "mots2"));//"mots is stop but machine is in cycle"
 *      return (COMBINED_PANIC);
 */
cmCombinedState cm_get_combined_state()
{
    // A dwell (or final segment) can outlive its planner slot and stored STOP/END state.
    // Snapshot the runtime with the states: the HI loader can finish/start work between reads.
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    cmMachineState machine_state = cm.machine_state;
    cmCycleState cycle_state = cm.cycle_state;
    cmMotionState motion_state = cm.motion_state;
    bool pending = false;
    if (!cm.program_paused &&
        ((machine_state == MACHINE_READY) || (machine_state == MACHINE_PROGRAM_STOP) ||
         (machine_state == MACHINE_PROGRAM_END))) {
        pending = !mp_runtime_is_idle() || mp_has_runnable_buffer() ||
            ((st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER) &&
             (st_pre.block_type != BLOCK_TYPE_NULL));
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
    // Reporting must not restart a cycle, undo M0/M1, or overwrite an alarm.
    if (pending) { return (motion_state == MOTION_HOLD ? COMBINED_HOLD : COMBINED_RUN); }
    if (machine_state <= MACHINE_PROGRAM_END) {  // replaces first 5 machine_state cases
        return ((cmCombinedState)machine_state); //...where MACHINE_xxx == COMBINED_xxx
    }
    switch(machine_state) {
        case MACHINE_INTERLOCK:     { return (COMBINED_INTERLOCK); }
        case MACHINE_SHUTDOWN:      { return (COMBINED_SHUTDOWN); }
        case MACHINE_PANIC:         { return (COMBINED_PANIC); }
        case MACHINE_CYCLE: {
            switch(cycle_state) {
                case CYCLE_HOMING:  { return (COMBINED_HOMING); }
                case CYCLE_PROBE:   { return (COMBINED_PROBE); }
                case CYCLE_JOG:     { return (COMBINED_JOG); }
                case CYCLE_MACHINING: case CYCLE_OFF: {
                    switch(motion_state) {
                        case MOTION_STOP:     { return (COMBINED_RUN); }    // See NOTE_1, above
                        case MOTION_RUN:      { return (COMBINED_RUN); }
                        case MOTION_HOLD:     { return (COMBINED_HOLD); }
                        default: {
                            cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() mots bad"));    // "mots has impossible value"
                            return (COMBINED_PANIC);
                        }
                    }
                }
                default: {
                    cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() cycs bad"));    // "cycs has impossible value"
                    return (COMBINED_PANIC);
                }
            }
        }
        default: {
            cm_panic_P(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, PSTR("cm_get_combined_state() macs bad"));    // "macs has impossible value"
            return (COMBINED_PANIC);
        }
    }
}

uint8_t cm_get_runtime_busy() { return (mp_get_runtime_busy());}

/**************************
 * Alarms                 *
 **************************/

/*
 * cm_clear() - clear ALARM and SHUTDOWN states
 */

void cm_clear()
{
    if (cm.machine_state == MACHINE_ALARM) {
        cm.machine_state = MACHINE_PROGRAM_STOP;
        // An alarm latches a hold (E044). Once nothing is left to run (e.g. after a '%' flush)
        // no '~' will come, so clearing also ends the hold and the cycle; otherwise the cycle stays
        // MACHINING and persisted settings are refused as 'moving' until a reset (E144).
#ifdef __AVR
        uint8_t saved_sreg = SREG;
        cli();
#endif
        // Drained means no queued block, an idle runtime AND no prepared segment waiting for the
        // loader (the last block can be freed before its final segment loads); snapshot together.
        bool drained = !mp_has_runnable_buffer() && mp_runtime_is_idle() &&
                       (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER);
#ifdef __AVR
        SREG = saved_sreg;
#endif
        // once drained only the main loop (this code) can queue motion, so act outside the cli window
        if (drained) {
            if (cm.hold_state != FEEDHOLD_OFF) {
                cm.end_hold_requested = false;
                mp_exit_hold_state();
            }
            cm_cycle_end();                     // no-op if the cycle already ended
        }
    } else if (cm.machine_state == MACHINE_SHUTDOWN) {
        cm.machine_state = MACHINE_READY;
    }
}

/*
 * cm_is_alarmed() - return alarm status code or OK if no alarms
 */

stat_t cm_is_alarmed()
{
    if (cm.machine_state == MACHINE_ALARM)    { return (STAT_COMMAND_REJECTED_BY_ALARM); }
    if (cm.machine_state == MACHINE_PANIC)    { return (STAT_COMMAND_REJECTED_BY_PANIC); }
    if (cm.machine_state == MACHINE_SHUTDOWN) { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
    return (STAT_OK);
}

/*
 * cm_halt_all() - stop, spindle and coolant immediately
 * cm_halt_motion() - stop motion immediately. Does not affect spindle, coolant, or other IO
 *
 * Stop motors and reset all system states accordingly.
 * Does not de-energize motors as in some cases the motors must remain energized
 * in order to prevent an axis from crashing.
 */

void cm_halt_all(void)
{
    cm_halt_motion();
//	cm_spindle_off_immediate();
//	cm_coolant_off_immediate();
}

void cm_halt_motion(void)
{
    mp_halt_runtime();                  // stop the runtime. Do this immediately. (Reset is in cm_clear)
    canonical_machine_reset();          // reset Gcode model
	cm.cycle_state = CYCLE_OFF;         // Note: leaves machine_state alone
	cm.motion_state = MOTION_STOP;
	cm.hold_state = FEEDHOLD_OFF;
}

/*
 * cm_alarm()   - enter ALARM state
 * cm_alarm_P() - enter ALARM state - accepts message as program string
 *
 * An ALARM sets the ALARM machine state, starts a feedhold to stop motion, stops the
 * spindle, turns off coolant, clears out queued planner moves and serial input,
 * and rejects new action commands (gcode blocks, SET commands, and other actions)
 * until the alarm is cleared.
 *
 * ALARM is typically entered by a soft limit or a limit switch being hit. In the
 * limit switch case the INPUT_ACTION will override the feedhold - i.e. if the
 * input action is "FAST_STOP" or "HALT" that setting will take precedence over
 * the feedhold native to the alarm function.
 *
 * Gcode and machine state is preserved. It may be possible to recover the job from
 * an alarm, but in many cases this is not possible. Since ALARM attempts to preserve
 * Gcode and machine state it does not END the job.
 *
 * ALARM may also be invoked from the command line using {alarm:n} or $alarm
 * ALARM can be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * ALARMs will also clear on receipt of an M30 or M2 command if one is received
 * while draining the host command queue.
 */

stat_t cm_alarm_P(const stat_t status, const char *msg_P)
{
    char msg_buf[STATUS_MSG_LEN];
    strcpy_P(msg_buf, msg_P);
    return(cm_alarm(status, msg_buf));
}

// Apply the stop without reporting: also safe while a response is waiting for TX space.
static bool _enter_alarm(void)
{
    ov_cancel();
    if ((cm.machine_state == MACHINE_ALARM) ||
        (cm.machine_state == MACHINE_PANIC) ||
        (cm.machine_state == MACHINE_SHUTDOWN)) {
        return (false);                         // already inhibited
    }
	cm.machine_state = MACHINE_ALARM;

    // Nothing moving (idle, a dwell, moves not started yet) means no hold will ever start, so the
    // flush would wait forever while queued moves run in ALARM (E044). Decide that atomically with
    // the exec: with interrupts off, a stopped runtime is put straight into HOLD, which keeps
    // mp_exec_aline() from starting another move. A move already running takes the normal hold.
#ifdef __AVR
    uint8_t sreg = SREG;
    cli();
#endif
    bool idle = (cm.motion_state == MOTION_STOP) && (cm.hold_state == FEEDHOLD_OFF);
    bool segment_in_flight = false;
    if (idle) {
        segment_in_flight = st_aline_in_flight();   // a block's last segment can outlive MOTION_STOP
        cm_set_motion_state(MOTION_HOLD);
        cm.hold_state = FEEDHOLD_HOLD;          // no new move starts; a last segment finishes on its own
    }
#ifdef __AVR
    SREG = sreg;
#endif
    if (idle) {
        if (!segment_in_flight) {
            stepper_reset();                    // only a dwell (or nothing) is running: stop it now
        }
        cm.queue_flush_state = FLUSH_REQUESTED; // flush input and planner once the runtime is idle
    } else {
        cm_request_feedhold();                  // stop motion
        cm_request_planner_flush();             // flush the planner once runtime is not busy; host input is
                                                // kept and rejected line by line while alarmed (E145)
    }

    _stop_outputs();                     // alarms stop spindle and coolant independently of feedhold

    return (true);
}

stat_t cm_alarm(const stat_t status, const char *msg)
{
    if (!_enter_alarm()) { return (STAT_OK); }
    rpt_exception(status, msg);
    return (status);
}

/*
 * cm_shutdown() - enter shutdown state
 *
 * SHUTDOWN stops all motion, spindle and coolant immediately, sets a SHUTDOWN machine
 * state, clears out queued moves and serial input, and rejects new action commands
 * (gcode blocks, SET commands, and some others).
 *
 * Shutdown is typically invoked as an electrical input signal sent to the board as
 * part of an external emergency stop (Estop). Shutdown is meant to augment but not
 * replace the external Estop functions that shut down power to motors, spindles and
 * other moving parts.
 *
 * Shutdown may also be invoked from the command line using {shutd:n} or $shutd
 * Shutdown must be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * Shutdown does not clear on M30 or M2 Gcode commands
 */

stat_t cm_shutdown(const stat_t status, const char *msg)
{
    ov_cancel();
    if ((cm.machine_state == MACHINE_SHUTDOWN) || (cm.machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't shutdown if shutdown or panic'd
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    _stop_outputs();
    cm_queue_flush();                           // flush all queues and reset positions

    for (uint8_t i = 0; i < HOMING_AXES; i++) { // unhome axes and the machine
        cm.homed[i] = false;
    }
    cm.homing_state = HOMING_NOT_HOMED;

	cm.machine_state = MACHINE_SHUTDOWN;        // do this after all other activity
	rpt_exception(status, msg);	                // send exception report
    return (status);
}

/*
 * cm_panic()   - enter PANIC state
 * cm_panic_P() - enter PANIC state - accepts message as program string
 *
 * PANIC occurs if the firmware has detected an unrecoverable internal error
 * such as an assertion failure or a code condition that should never occur.
 * It sets PANIC machine state, and leaves the system inspect able (if possible).
 *
 * PANIC can only be exited by a hardware reset or soft reset (^x)
 */

stat_t cm_panic_P(const stat_t status, const char *msg_P)
{
    char msg_buf[STATUS_MSG_LEN];
    strcpy_P(msg_buf, msg_P);
    return(cm_panic(status, msg_buf));
}

stat_t cm_panic(const stat_t status, const char *msg)
{
    ov_cancel();
    if (cm.machine_state == MACHINE_PANIC) {    // only do this once
        return (STAT_OK);
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    _stop_outputs();
    cm_queue_flush();                           // flush all queues and reset positions

	cm.machine_state = MACHINE_PANIC;           // don't reset anything. Panics are not recoverable
	rpt_exception(status, msg);			        // send panic report
	return (status);
}

/*
 * cm_outputs_inhibited() - true in ALARM, SHUTDOWN and PANIC: spindle and coolant may only turn off
 *
 *  A queued M3/M7/M8 can still reach its exec function after an alarm (e.g. at the end of a running
 *  dwell, before the queue flush runs), so the exec functions check this at execution time.
 */
bool cm_outputs_inhibited()
{
    return ((cm.machine_state == MACHINE_ALARM) || (cm.machine_state == MACHINE_SHUTDOWN) ||
            (cm.machine_state == MACHINE_PANIC) || (cm.queue_flush_state != FLUSH_OFF) ||
            cm.job_kill_requested || cm.limit_requested || cm.shutdown_requested);
}

/**************************************************
 * Feedhold and Related Functions (no NIST ref #) *
 **************************************************/
/*
 * Feedholds, queue flushes and end_holds are all related. The request functions change
 * states to "_____REQUESTED". The sequencing callback interprets the flags as so:
 *
 *    - A feedhold request received during motion should be honored
 *    - A feedhold request received during a feedhold should be ignored
 *    - A feedhold request received during a motion stop should be ignored
 *
 *    - A queue flush request received during a motion stop should be honored
 *    - A queue flush request received during a feedhold should be honored.
 *      A request received during a feedhold should be deferred until cmFeedholdState
 *      enters FEEDHOLD_HOLD (i.e. until deceleration is complete and motors stop).
 *
 *    - An end_hold (cycle start) request should only be honored while in a feedhold
 *    - Said end_hold request received during a feedhold should be deferred until the
 *      feedhold enters a HOLD state (i.e. until deceleration is complete).
 *      If a queue flush request is also present the queue flush should be done first
 *
 *	Below the request level, feedholds work like this:
 *
 *    - The hold is initiated by calling cm_start_hold()
 *      cm.hold_state is set to FEEDHOLD_SYNC, motion_state is set to MOTION_HOLD
 *      The spindle is turned off (if it is on)
 *      The remainder of feedhold processing occurs in plan_exec.c / mp_exec_aline()
 *
 *	  - MOTION_HOLD and FEEDHOLD_SYNC tells mp_exec_aline() to begin feedhold processing
 *      after the current move segment is finished (< 5 ms later).
 *      See plan_exec.c for details of how feedholds are handled. In short:
 *
 *      - FEEDHOLD_SYNC causes the current move in mr to be replanned into a deceleration.
 *        If the distance remaining in the mr move is sufficient for a full deceleration
 *        then motion will stop in the current block. Otherwise the deceleration phase
 *        will extend across as many blocks necessary until one will stop.
 *
 *      - Once deceleration is complete hold_state transitions to FEEDHOLD_HOLD and the
 *        distance remaining in the bf last block is replanned up from zero velocity.
 *        The move in the bf block is NOT released (unlike normal operation), as it
 *        will be used again to restart from hold.
 *
 *      - When cm_end_hold() is called it releases the hold, restarts the move and restarts
 *        the spindle if the spindle is active.
 */
/*
 * cm_request_feedhold()
 * cm_request_end_hold() - cycle restart
 * cm_request_queue_flush()
 */
/*
 * cm_request_job_kill() - ^d from the RX ISR (E096)
 *
 *	The job kill is an ALARM (g2core Job-Exception-Handling): motion stops, spindle and coolant stop,
 *	the planner is flushed and commands are rejected until {clear:n}, $clear, M2 or M30, in or out of
 *	a feedhold. The ISR only flags it; cm_feedhold_sequencing_callback() raises the alarm.
 */
void cm_request_job_kill(void)
{
    rt_override_reset_job();
    rt_qt_invalidate();
    rt_job_end();                                   // the host job is over (advisor step 3)
    cm.job_kill_requested = true;
}

void cm_request_feedhold(void) {
    ov_cancel();
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    if (cm.hold_state == FEEDHOLD_HOLD) {
        cm.end_hold_requested = false;
        hold_resume_cancelled = true;
    }
    bool held = mp_hold_startup();
    if (!held && (cm.hold_state == FEEDHOLD_OFF)) {
        bool stopped = !st_aline_in_flight();
        bool dwell = stopped && (st_runtime_isbusy() ||
            ((st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER) &&
             (st_pre.block_type == BLOCK_TYPE_DWELL)));
        if (dwell || (stopped && (cm.motion_state == MOTION_STOP) &&
                      (cm.cycle_state == CYCLE_MACHINING))) {
            // A lone dwell can have freed its last planner buffer before its timer ends.
            if (dwell && !cm_is_alarmed()) { cm_cycle_start(); }
            cm_set_motion_state(MOTION_HOLD);
            cm.hold_state = FEEDHOLD_HOLD; // no moving segment needs deceleration
            held = true;
        } else if (cm.motion_state != MOTION_STOP) {
            cm.hold_state = FEEDHOLD_REQUESTED;
        }
    }
    if (held) {
        cs.controller_state = CONTROLLER_READY;
        cm.hold_fast = false;
        cm.hold_fast_request = false;
        sr_request_status_report(SR_REQUEST_ASAP);
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

// Cycle contacts and operator holds both stop only motion. A new contact still cancels resume.
void cm_request_cycle_feedhold(void)
{
    cm_request_feedhold();
}

void cm_request_end_hold(void)
{
    if (cm.hold_state != FEEDHOLD_OFF) {
        cm.end_hold_requested = true;
    }
}

void cm_request_queue_flush()
{
    cm_request_planner_flush();
    if (cm.queue_flush_state == FLUSH_REQUESTED) {  // honored now, or already pending from an alarm
        rt_qt_invalidate();
        rt_job_end();                               // a discarded job is over (advisor step 3)
        cm.queue_flush_input = true;                // the host asked: drop what it sent before the '%'
    }
}

void cm_request_planner_flush()
{
    ov_cancel();
    if ((cm.hold_state != FEEDHOLD_OFF) &&          // don't honor request unless you are in a feedhold
        (cm.queue_flush_state == FLUSH_OFF)) {      // ...and only once
        cm.queue_flush_state = FLUSH_REQUESTED;     // flushed from the main loop
    }
}

/*
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 */

/*
 * _run_requested_queue_flush() - complete a host-requested flush ('%' or alarm) from the main loop
 *
 *  Waits until motion has stopped and no hold is still decelerating, then flushes the planner and,
 *  for a host '%', drops all buffered input (lines received before the '%'). An alarm's flush keeps
 *  host input: those lines are rejected one by one while alarmed (E145). cm_queue_flush() consumes the
 *  host's request atomically with the end of the flush and cm_feedhold_sequencing_callback() drops the
 *  input. A hold that was requested but never
 *  started (nothing was moving) is cancelled rather than waited for. Direct callers of
 *  cm_queue_flush() (homing, panic) keep the host input unless a '%' was pending.
 */
// Hand a pending host '%' to queue_flush_input_due, atomically against the RX ISR that sets it (E145)
static void _take_input_flush_request()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    if (cm.queue_flush_input) {
        cm.queue_flush_input = false;
        cm.queue_flush_input_due = true;
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

// Drop the input a host '%' asked for; main loop only, never inside a direct flush (E145)
static void _service_input_flush()
{
    if (cm.queue_flush_input_due) {
        cm.queue_flush_input_due = false;
        xio_flush_input();
    }
}

static void _run_requested_queue_flush()
{
    hold_resume_cancelled = true;
    st_cancel_dwell();                             // discarding a job also discards its remaining dwell
    if (!mp_runtime_is_idle() ||
        ((cm.hold_state != FEEDHOLD_OFF) && (cm.hold_state != FEEDHOLD_REQUESTED) &&
         (cm.hold_state != FEEDHOLD_HOLD))) {
        return;
    }
    _take_input_flush_request();                    // drop a host '%''s input before the planner work below,
    _service_input_flush();                         // so a line sent right after the '%' is not cut (E145)
    cm_queue_flush();                               // a '%' landing meanwhile is handed off and serviced next
    if (cm.hold_state == FEEDHOLD_REQUESTED) {
        cm.hold_state = FEEDHOLD_OFF;
        cm.end_hold_requested = false;              // a '~' for the cancelled hold must not resume a later one
    }
}

// A blocked writer may be using the parser's input and report buffers. Only start
// the stop here; never flush input, resume, format a report or re-enter the dispatcher.
void cm_tx_blocked_callback(void)
{
    if (cm.job_kill_requested && _enter_alarm()) {
        cm.job_kill_report_pending = true;
    }
    if ((cm.hold_state == FEEDHOLD_REQUESTED) || (cm.hold_state == FEEDHOLD_SYNC)) {
        cm_start_hold();
    }
}

stat_t cm_feedhold_sequencing_callback()
{
    if (cm.job_kill_requested) {
        cm.job_kill_requested = false;
        xio_cancel_pgm_source();                    // return canned-file input to the host (N43)
        if (cm.job_kill_report_pending) {
            cm.job_kill_report_pending = false;
            rpt_exception(STAT_KILL_JOB, "job kill"); // stop already applied by the blocked writer
        } else {
            cm_alarm_P(STAT_KILL_JOB, PSTR("job kill"));
        }
    }
	if ((cm.hold_state == FEEDHOLD_REQUESTED) || (cm.hold_state == FEEDHOLD_SYNC)) {
		cm_start_hold();                            // feed won't run unless the machine is moving
	}
	if (cm.queue_flush_state == FLUSH_REQUESTED) {
        _run_requested_queue_flush();               // runs once motion has stopped
	}
	_service_input_flush();                         // a flush consumed a host '%': drop its input here (E145)
	if (cm.end_hold_requested) {
        if (cm.queue_flush_state == FLUSH_OFF) {    // either no flush or wait until it's done flushing
			cm_end_hold();
		}
	}
	return (STAT_OK);
}

/*
 * cm_start_hold() - start a feedhhold by signalling the exec
 * cm_end_hold()   - end a feedhold by returning the system to normal operation
 * cm_has_hold()   - return true if a hold condition exists (or a pending hold request)
 */

static void _stop_outputs(void)
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    hold_resume_cancelled = true;
    spindle_reset();
    float off[] = { 0 };
    cm_exec_flood_coolant_control(off, FLAGS_NONE);
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

void cm_start_hold()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // The exec may free the last block before its prepared segment reaches the loader.
    // Decide and latch together; an empty queue alone does not mean the motor has stopped.
    if ((cm.hold_state == FEEDHOLD_REQUESTED) || (cm.hold_state == FEEDHOLD_SYNC)) {
        bool runnable = mp_has_runnable_buffer();
        // SYNC ends on a line, and a flush cancels the dwell that would wake the exec for one (N85).
        // With a flush waiting and no line in the runtime this is already the stop.
        bool flushing = (cm.queue_flush_state == FLUSH_REQUESTED) && (mr.block_state == BLOCK_IDLE);
        if ((cm.hold_state == FEEDHOLD_REQUESTED) && runnable) {
            cm_set_motion_state(MOTION_HOLD);
            cm.hold_state = FEEDHOLD_SYNC;
        } else if ((!runnable || flushing) && mp_runtime_is_idle() &&
                   (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER)) {
            cm_set_motion_state(MOTION_HOLD);
            cm.hold_state = FEEDHOLD_HOLD;           // let pending '~' or '%' finish normally
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

void cm_end_hold()
{
    bool resume_ready = false;
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    if ((cm.hold_state == FEEDHOLD_HOLD) && cm.end_hold_requested) {
        if (cm_is_alarmed() || (cm.queue_flush_state != FLUSH_OFF) || cm.job_kill_requested ||
            cm.hold_fast_request || cm.limit_requested || cm.shutdown_requested) {
            hold_resume_cancelled = true;
            cm.end_hold_requested = false;
        } else {
            hold_resume_cancelled = false;
#ifdef __AVR
            // Never force interrupts on for a masked caller or walk the planner list masked.
            if (saved_sreg & 0x80) { resume_ready = true; }
#else
            resume_ready = true;
#endif
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
    if (!resume_ready) { return; }

    // HOLD and program_paused stay latched throughout this interruptible planner walk.
    stat_t status = mp_replan_held_block();
#ifdef __AVR
    cli();
#endif
    bool aborted = cm_is_alarmed() || (cm.queue_flush_state != FLUSH_OFF) || cm.job_kill_requested ||
                   cm.hold_fast_request || cm.limit_requested || cm.shutdown_requested;
    if ((status == STAT_OK) && (cm.hold_state == FEEDHOLD_HOLD) && cm.end_hold_requested &&
        !hold_resume_cancelled && !aborted) {
        cm.program_paused = false;
        cm.end_hold_requested = false;
        bool runnable = mp_has_runnable_buffer();
        if (runnable) { cm_cycle_start(); }        // before an executor can empty the queue (E165)
        mp_exit_hold_state();
        if (!runnable) { cm_cycle_end(); }         // a lone dwell or M0 can also resume with an empty queue
    } else if (aborted || (cm.hold_state != FEEDHOLD_HOLD)) {
        hold_resume_cancelled = true;
        cm.end_hold_requested = false;
    } else {
        // A second !, even followed by ~ during replanning, cancels this resume attempt.
        hold_resume_cancelled = true;
        if (status != STAT_OK) { cm.end_hold_requested = false; }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
    if (status != STAT_OK) { rpt_exception_P(status, PSTR("held move geometry")); }
}

bool cm_has_hold()
{
    return (cm.hold_state != FEEDHOLD_OFF);
}

/*
 * cm_queue_flush() - Flush planner queue and correct model positions
 *
 * This one's complicated. See here first:
 * https://github.com/synthetos/g2/wiki/Alarm-Processing
 * https://github.com/synthetos/g2/wiki/Job-Exception-Handling
 *
 * We want to use queue flush for a few different use cases, as per the above wiki pages.
 * The % behavior implements Exception Handling cases 1 and 2 - Stop a Single Move and
 * Stop Multiple Moves. This is complicated further by the processing in single USB and
 * dual USB being different. Also, the state handling is located in xio.cpp / readline(),
 * controller.cpp _dispatch_kernel() and cm_request_queue_flush(), below.
 * So it's documented here.
 *
 * Single or Dual USB Channels:
 *  - If a % is received outside of a feed hold or ALARM state, ignore it.
 *      Change the % to a ; comment symbol (xio)
 *      This behavior supports % as file beginnings and ends, and Inkscape comments.
 *
 * Single USB Channel Operation (TinyG v8's only mode)
 *  - Enter a feedhold (!)
 *  - Receive a queue flush (%) Both dispatch it and store a marker (ACK) in the input
 *      buffer in place of the the % (xio)
 *  - Execute the feedhold to a hold condition (plan_exec)
 *  - Execute the dispatched % to flush queues (canonical_machine)
 *  - Silently reject any commands up to the % in the input queue (controller)
 *  - When ETX is encountered transition to STOP state (controller/canonical_machine)
 *
 * Dual USB Channel Operation (Available on g2 only)
 *  - Same as above except that we expect the % to arrive on the control channel
 *  - The system will read and dump all commands in the data channel until either a
 *    clear is encountered ({clear:n} or $clear), or an ETX is encountered on either
 *    channel, but it really should be on the data channel to ensure all queued commands
 *    are dumped. It is the host's responsibility to both write the clear (or ETX), and
 *    to ensure that it either arrives on the data channel or that the data channel is
 *    empty before writing it to the control channel.
 */

void cm_queue_flush()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    hold_resume_cancelled = true;
    // Homing calls this directly as housekeeping, not as a request to abort a dwell.
    if ((cm.hold_state == FEEDHOLD_HOLD) || (cm.queue_flush_state == FLUSH_REQUESTED)) {
        st_cancel_dwell();
    }
    bool idle = mp_runtime_is_idle();
    if (idle) { st_prep_null(); }                  // a prepared dwell must not load after the flush
#ifdef __AVR
    SREG = saved_sreg;
#endif
    if (idle) {                                   // moving segments still finish through feedhold
        mp_flush_planner();

        for (uint8_t axis = AXIS_X; axis < AXES; axis++) { // set all positions
            cm_set_position(axis, mp_get_runtime_absolute_position(axis));
        }
#ifdef __AVR
        uint8_t hold_sreg = SREG;
        cli();
#endif
        cm.program_paused = false;
        cm.end_hold_requested = false;
        if (cm.hold_state == FEEDHOLD_HOLD) {
            mp_exit_hold_state();                 // empty queue: MOTION_STOP, including in ALARM
            if (!cm_is_alarmed()) { cm_cycle_end(); }
        }
#ifdef __AVR
        SREG = hold_sreg;
#endif
        // Consume a host '%' together with the state change: the RX ISR sets both (E145). A '%' that
        // lands before this is handed off; after it, the flush is over and the '%' is not honored.
        // Callers include panic paths, so the input pool is reset later, by the sequencing callback.
#ifdef __AVR
        uint8_t saved_sreg = SREG;
        cli();
#endif
        _take_input_flush_request();                // nests: restores the cleared SREG
        cm.queue_flush_state = FLUSH_OFF;
#ifdef __AVR
        SREG = saved_sreg;
#endif
        qr_request_queue_report(0);                 // request a queue report, since we've changed the number of buffers available
    }
}

/******************************
 * Program Functions (4.3.10) *
 ******************************/
/* This group implements stop, start, and end functions.
 * It is extended beyond the NIST spec to handle various situations.
 *
 * _exec_program_finalize()     - helper
 * cm_cycle_start()
 * cm_cycle_end()
 * cm_program_stop()            - M0
 * cm_optional_program_stop()   - M1
 * cm_program_end()             - M2, M30
 */
/*
 * Program and cycle state functions
 *
 * cm_program_stop() and cm_optional_program_stop() are synchronous Gcode commands
 * that are received through the interpreter. They cause all motion to stop
 * at the end of the current command, including spindle motion. All Gcode state
 * is otherwise left intact.
 *
 * Note that the stop actually occurs at the end of the immediately preceding
 * block - i.e. the stop is queued behind the last executing block.
 *
 *
 * cm_program_end() is a stop that also resets the machine to initial state.
 * The END behaviors are defined by NIST 3.6.1 are:
 *	1. Axis offsets are set to zero (like G92.2) and...
 *  1a. Origin offsets are set to the default (like G54)
 *	2. Selected plane is set to CANON_PLANE_XY (like G17)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *	5. Feed and speed overrides are set to ON (like M48)
 *	6. Cutter compensation is turned off (like G40)
 *	7. The spindle is stopped (like M5)
 *	8. The current motion mode is set to G_1 (like G1)
 *	9. Coolant is turned off (like M9)
 *
 * cm_program_end() performs things slightly differently than NIST:
 *	1. Axis offsets are set to G92.1 CANCEL offsets (instead of using G92.2 SUSPEND Offsets)
 *	1a. Set default coordinate system (uses $gco, not G54)
 *	2. Selected plane is set to default plane ($gpl) (instead of setting it to G54)
 *	3. Distance mode is set to MODE_ABSOLUTE (like G90)
 *	4. Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 * 	5. Not implemented
 *	6. Not implemented
 *	7. The spindle is stopped (like M5)
 *	8. Motion mode is canceled like G80 (not set to G1)
 *	9. Coolant is turned off (like M9)
 *	10 Default INCHES or MM units mode is restored ($gun)
 */

static void _exec_program_finalize(float *value, bool *flags)
{
    (void)flags;
	if (cm_is_alarmed() != STAT_OK) { return; }			// only $clear or a reset ends these (N84)
    hold_resume_cancelled = false;
	cm.machine_state = (uint8_t)value[0];               // set to one of STOP or END
	cm_set_motion_state(MOTION_STOP);
	if (cm.cycle_state == CYCLE_MACHINING) {
		cm.cycle_state = CYCLE_OFF;						// don't end cycle if homing, probing, etc.
	}
	cm.hold_state = FEEDHOLD_OFF;						// end feedhold (if in feed hold)
	cm.hold_fast = false;
	cm.hold_fast_request = false;
    cm.end_hold_requested = false;					    // cancel any pending end hold request
	mp_zero_segment_velocity();							// for reporting purposes

	// A program END's model resets were applied when the M2/M30 was parsed (cm_program_end()).
	// This runs from the stepper loader ISR, so it must not touch the gcode model or queue commands.
	sr_request_status_report(SR_REQUEST_ASAP);		    // request a final status report (not unfiltered)
}

void cm_cycle_start()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // Program-stop callbacks run in the HI loader, so masking only LO/MED is insufficient.
    if (!cm.program_paused) {
        cm.machine_state = MACHINE_CYCLE;
        if (cm.cycle_state == CYCLE_OFF) {					// don't (re)start homing, probe or other canned cycles
            cm.cycle_state = CYCLE_MACHINING;		// keep qi/qo: buffers queued before the cycle started count (E091)
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

void cm_cycle_end()
{
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    // Program-stop callbacks run in the HI loader, so masking only LO/MED is insufficient.
    if (!cm.program_paused) {
        if (cm.cycle_state != CYCLE_OFF) {
            float value[] = { (float)MACHINE_PROGRAM_STOP };
            _exec_program_finalize(value, FLAGS_ONE);
        }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
}

static void _exec_program_stop(float *value, bool *flags)
{
    _exec_program_finalize(value, flags);
    cm.program_paused = true;
    cm.hold_state = FEEDHOLD_HOLD;
    cm_set_motion_state(MOTION_HOLD);
}

void cm_program_stop()
{
    ov_cancel();
	float value[AXES] = { (float)MACHINE_PROGRAM_STOP };
	mp_queue_command(_exec_program_stop, value, FLAGS_ONE);
}

void cm_optional_program_stop()
{
    if (cm.optional_stop_enabled) { cm_program_stop(); }
}

void cm_program_end()
{
    ov_cancel();
	// Reset the model now, in the main loop, so blocks parsed after the M2/M30 keep their modes, and
	// queue ONE runtime command for the rest: the per-block planner demand stays bounded (E069, E088)
	cm.gmx.origin_offset_enable = 0;				// 1.  G92.1 - we do G91.1 instead of G92.2
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		cm.gmx.origin_offset[axis] = 0;
	}
//	cm_suspend_origin_offsets();					// 1.  G92.2 - as per NIST
	cm.gm.coord_system = cm.default_coord_system;	// 1a. reset to default coordinate system
	cm_select_plane(cm.default_select_plane);		// 2.  reset to default arc plane
	cm_set_distance_mode(cm.default_distance_mode); // 3.  reset to default distance mode
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);	// 4.  G94
	cm.gm.spindle_mode = SPINDLE_OFF;				// 7.  M5 (model; the output goes off at runtime)
//	cm_set_motion_mode(MOTION_MODE_STRAIGHT_FEED);	// 8.  NIST specifies G1...
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL);  // 8.  but we cancel motion mode. Safer.
	cm.gm.mist_coolant = false;						// 9.  M9 (model)
	cm.gm.flood_coolant = false;
//	cm_set_units_mode(cm.units_mode);				// 10. reset to default units mode +++ REMOVED +++

	float value[AXES] = { (float)MACHINE_PROGRAM_END, (float)cm.default_coord_system };	// mp_queue_command reads AXES values
	mp_queue_command(_exec_program_end, value, FLAGS_ONE);
}

/*
 * _exec_program_end() - runtime half of M2/M30, in queue order behind the moves before it
 *
 *	One queued command (it must not queue more): apply the reset work offsets, stop the spindle and
 *	coolant outputs directly, then finalize the program end.
 */
static void _exec_program_end(float *value, bool *flags)
{
    rt_override_reset_job();
	uint8_t coord_system = (uint8_t)value[1];
	float offsets[AXES];
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		offsets[axis] = cm.offset[coord_system][axis];	// origin offsets were reset with the model
	}
	mp_set_runtime_work_offset(offsets);
	spindle_reset();								// M5 now (immediate; S is kept)
	cm_coolant_off_immediate();						// M9 now
	rt_job_end();									// M2/M30 executed: no supply gaps after it (advisor step 3)
	_exec_program_finalize(value, flags);			// value[0] is MACHINE_PROGRAM_END
}

/**************************************
 * END OF CANONICAL MACHINE FUNCTIONS *
 **************************************/
