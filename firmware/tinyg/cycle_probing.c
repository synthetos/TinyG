/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * Part of TinyG project
 * 
 * Copyright (c) 2010 - 2013 Alden S Hart, Jr.
 * G38.2 cycle by Mike Estee
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
#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "switch.h"
#include "spindle.h"

/**** Probe singleton structure ****/

struct pbProbingSingleton {		// persistent homing runtime variables
    stat_t (*func)();// binding for callback function state machine
    
	// state saved from gcode model
    uint8_t saved_switch_type;  // saved switch type NC/NO
    uint8_t saved_switch_mode[NUM_SWITCHES];
    uint8_t probe_switch;       // what switch should we check?
    
    // probe destination
    float target[AXES];
    float flags[AXES];
    
	float saved_jerk[AXES];		// saved and restored for each axis
};
static struct pbProbingSingleton pb;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_start();
static stat_t _probing_finish();
static stat_t _probing_finalize_exit();
//static stat_t _probing_error_exit();

static stat_t _set_pb_func(uint8_t (*func)());


/*****************************************************************************
 * cm_probing_cycle_start()	- G38.2 homing cycle using limit switches
 * cm_probing_callback() 	- main loop callback for running the homing cycle
 *
 */
/*	--- Some further details ---
 *
 *	Note: When coding a cycle (like this one) you get to perform one queued 
 *	move per entry into the continuation, then you must exit. 
 *
 *	Another Note: When coding a cycle (like this one) you must wait until 
 *	the last move has actually been queued (or has finished) before declaring
 *	the cycle to be done. Otherwise there is a nasty race condition in the 
 *	tg_controller() that will accept the next command before the position of 
 *	the final move has been recorded in the Gcode model. That's what the call
 *	to cm_isbusy() is about.
 *
 *  ESTEE: is this still true???
 */

uint8_t cm_probe_cycle_start( float target[], float flags[] )
{
    // so optimistic... ;)
    // NOTE: it is *not* an error condition for the probe not to trigger.
    // it is an error for the limit or homing switches to fire, or for some other configuration error.
    cm.probe_state = PROBE_FAILED;
    
    // axis init
    for( int axis=0; axis<AXES; axis++ )
    {
        // clear the old probe position, note that realying on probe_result will not detect a probe to 0,0,0.
        cm.probe_results[axis] = 0.0;
        
        // save the jerk settings & switch to the jerk_homing settings
        pb.saved_jerk[axis] = cm.a[axis].jerk_max;         // save the max jerk value
        cm.a[axis].jerk_max = cm.a[axis].jerk_homing;	// use the homing jerk for probe
        
        // set endpoint
        pb.target[axis] = target[axis];
        pb.flags[axis] = flags[axis];
    }

    // switch the switch type mode for the probe
    // FIXME: we should be able to use the homing switch at this point too, can't because switch mode is global
    // and our probe is NO, not NC.
    pb.probe_switch = SW_MIN_Z;         // FIXME: hardcoded...
    for( int i=0; i<NUM_SWITCHES; i++ )
        pb.saved_switch_mode[i] = sw.mode[i];
    
    sw.mode[pb.probe_switch] = SW_MODE_HOMING;
    pb.saved_switch_type = sw.switch_type;  // save the switch type for recovery later.
    sw.switch_type = SW_TYPE_NORMALLY_OPEN; // contact probes are NO switches... usually.
    
    // re-init to pick up new switch settings
    switch_init();
    
    // start!
	pb.func = _probing_start; 			// bind initial processing function
	cm.cycle_state = CYCLE_PROBE;
	st_energize_motors();				// enable motors if not already enabled
    cm_spindle_control(SPINDLE_OFF);    // important! if previous command was an M3 this would be bad...
	return (STAT_OK);
}

// called when exiting on success or error
void _probe_restore_settings()
{
    mp_flush_planner(); 						// we should be stopped now, but in case of switch closure
    
    // restore switch settings
    sw.switch_type = pb.saved_switch_type;
    for( uint8_t i=0; i<NUM_SWITCHES; i++ )
        sw.mode[i] = pb.saved_switch_mode[i];
    
    // re-init to pick up changes
    switch_init();
    
    // restore axis jerk
    for( uint8_t axis=0; axis<AXES; axis++ )
        cm.a[axis].jerk_max = pb.saved_jerk[axis];
    
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
    cm.cycle_state = CYCLE_OFF;
	//cm_cycle_end();
    
    printf_P(PSTR("(cm.cycle_state %i)\n"), cm.cycle_state);
}

uint8_t cm_probe_callback(void)
{
	if (cm.cycle_state != CYCLE_PROBE) { return (STAT_NOOP);}	// exit if not in a probe cycle
	if (cm_get_runtime_busy() == true) { return (STAT_EAGAIN);}	// sync to planner move ends
	return (pb.func());                                         // execute the current homing move
}

static stat_t _probing_start()
{
    // initial probe state, don't probe if we're already contacted!
    int8_t probe = read_switch(pb.probe_switch);
    if( probe==SW_OPEN )
    {
        //cm_request_queue_flush();
        //mp_flush_planner();   // do we want to flush the planner here? we could already be at velocity from a previous move?
        //cm_request_cycle_start();
        
        ritorno(cm_straight_feed(pb.target, pb.flags));
    }
    
	return (_set_pb_func(_probing_finish));				// start the clear
}

static stat_t _probing_finish()
{
    int8_t probe = read_switch(pb.probe_switch);
    cm.probe_state = (probe==SW_CLOSED) ? PROBE_SUCCEDED : PROBE_FAILED;
    
    for( uint8_t axis=0; axis<AXES; axis++ )
        cm.probe_results[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
    
    // if we got here because of a feed hold we need to keep the model position correct
    //cm_set_model_position(STAT_OK);
    
    cm_request_queue_flush();
    
    printf_P(PSTR("{\"prb\":{\"e\":%i,\"x\":%.3g,\"y\":%.3g,\"z\":%.3g}}\n"),
             (int)cm.probe_state, cm.probe_results[AXIS_X], cm.probe_results[AXIS_Y], cm.probe_results[AXIS_Z]);
    
    return (_set_pb_func(_probing_finalize_exit));
}

static stat_t _probing_finalize_exit()
{
	_probe_restore_settings();
	return (STAT_OK);
}


/*
 * _probing_error_exit()
 */

//static stat_t _probing_error_exit(int8_t axis)
//{
//	// Generate the warning message. Since the error exit returns via the homing callback
//	// - and not the main controller - it requires its own display processing
//	cmd_reset_list();
//	if (axis == -2) {
//		cmd_add_conditional_message((const char_t *)"*** WARNING *** Probing error: Specified axis(es) cannot use probe");
//	} else {
//		char message[CMD_MESSAGE_LEN];
//		sprintf_P(message, PSTR("*** WARNING *** Probing error: %c axis settings misconfigured"), cm_get_axis_char(axis));
//		cmd_add_conditional_message((const char_t *)message);
//	}
//	cmd_print_list(STAT_PROBING_CYCLE_FAILED, TEXT_INLINE_VALUES, JSON_RESPONSE_FORMAT);
//
//	// clean up and exit
//	_probe_restore_settings();
//	return (STAT_PROBING_CYCLE_FAILED);
//}


/**** HELPERS ****************************************************************/
/*
 * _set_hm_func() - a convenience for setting the next dispatch vector and exiting
 */

uint8_t _set_pb_func(uint8_t (*func)())
{
	pb.func = func;
	return (STAT_EAGAIN);
}
