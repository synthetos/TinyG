/*
 * spindle.c - canonical machine spindle driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "spindle.h"
#include "gpio.h"
#include "planner.h"
#include "hardware.h"
#include "pwm.h"
#include "util.h"
#include "settings.h"
#include "stepper.h"
#ifdef __AVR
#include <avr/interrupt.h>
#endif

spindleConfig_t spindle_cfg;
static bool spindle_configured;

static void _exec_spindle_control(float *value, bool *flags);
static void _exec_spindle_speed(float *value, bool *flags);

void spindle_config_init(void)
{
    spindle_configured = false;
    spindle_cfg.enable_polarity = SPINDLE_ENABLE_POLARITY;
    spindle_cfg.direction_polarity = SPINDLE_DIR_POLARITY;
    spindle_cfg.pause_on_hold = SPINDLE_PAUSE_ON_HOLD;
    spindle_cfg.coolant_pause_on_hold = COOLANT_PAUSE_ON_HOLD;
    spindle_cfg.dwell_seconds = SPINDLE_DWELL_TIME;
}

void spindle_init_outputs(void)
{
    spindle_reset();
    gpio_init_outputs();
    spindle_configured = true;
}

stat_t spindle_set_polarity(nvObj_t *nv)
{
    if (nv->value_int > 1) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
#ifdef __AVR
    uint8_t saved_sreg = SREG;
    cli();
#endif
    bool busy = spindle_configured &&
                ((cm.gm.spindle_mode != SPINDLE_OFF) || mp_has_runnable_buffer() ||
                 !mp_runtime_is_idle() || (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER));
    stat_t status = STAT_COMMAND_NOT_ACCEPTED;
    if (!busy) {
        status = set_01(nv);
        if (status == STAT_OK) { spindle_reset(); }
    }
#ifdef __AVR
    SREG = saved_sreg;
#endif
    return (status);
}

stat_t spindle_set_dwell(nvObj_t *nv)
{
    if (!isfinite(nv->value_flt) || nv->value_flt < 0 || nv->value_flt > 3600) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    return (set_flt(nv));  // persisted SETs already pass the common idle gate
}

void spindle_control_immediate(uint8_t mode, float speed)
{
    cm_set_spindle_speed_parameter(MODEL, speed);
    float value[] = { (float)mode };
    _exec_spindle_control(value, FLAGS_NONE);
}

/*
 * spindle_init()
 * spindle_reset()
 */
/*
 * Spindle state readouts, named as in g2core: spe (enabled), spc (control: 0 off, 1 CW, 2 CCW),
 * spd (direction last driven, 0 CW, 1 CCW;
 * kept while the spindle is off), sps (speed in use: the S after the p1csl..p1csh or p1wsl..p1wsh clamp,
 * 0 while off; commanded, not measured). They report the outputs as last driven by the executors below,
 * not the model, which runs ahead of the outputs (M2/M30 turns the model off when it is parsed).
 */
static struct {
	volatile uint8_t mode;						// SPINDLE_OFF, SPINDLE_CW or SPINDLE_CCW, as driven
	volatile uint8_t ccw;						// direction last driven
	float speed;								// speed in use; written from the loader interrupt too
} spindle_out;

void spindle_get_runtime(uint8_t *mode, float *speed)
{
    uint8_t saved_sreg = SREG;
    cli();
    *mode = spindle_out.mode;
    *speed = spindle_out.speed;
    SREG = saved_sreg;
}

static void _record_speed(float speed)
{
	uint8_t saved_sreg = SREG;					// four bytes: written whole
	cli();
	spindle_out.speed = speed;
	SREG = saved_sreg;
}

static stat_t _get_state(nvObj_t *nv, uint8_t value)
{
	nv->value_int = value;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t spindle_get_spe(nvObj_t *nv) { return (_get_state(nv, spindle_out.mode != SPINDLE_OFF)); }
stat_t spindle_get_spd(nvObj_t *nv) { return (_get_state(nv, spindle_out.ccw)); }
stat_t spindle_get_spc(nvObj_t *nv) { return (_get_state(nv, spindle_out.mode)); }

stat_t spindle_get_sps(nvObj_t *nv)
{
	uint8_t saved_sreg = SREG;					// four bytes: read whole
	cli();
	float speed = spindle_out.speed;
	SREG = saved_sreg;
	nv->value_flt = speed;
	nv->precision = (int8_t)GET_TABLE_WORD(precision);	// a custom getter sets the row's precision itself (N81)
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

void spindle_init()
{
	if( pwm.c[PWM_1].frequency < 0 )
		pwm.c[PWM_1].frequency = 0;

    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
}

void spindle_reset()
{
    float value[] = { (float)SPINDLE_OFF };
    _exec_spindle_control(value, FLAGS_NONE);   // immediate, not queued: used by alarm, shutdown and panic
}

/*
 * cm_get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */
float cm_get_spindle_pwm( uint8_t spindle_mode )
{
	float speed_lo=0, speed_hi=0, phase_lo=0, phase_hi=0;
	if (spindle_mode == SPINDLE_CW ) {
		speed_lo = pwm.c[PWM_1].cw_speed_lo;
		speed_hi = pwm.c[PWM_1].cw_speed_hi;
		phase_lo = pwm.c[PWM_1].cw_phase_lo;
		phase_hi = pwm.c[PWM_1].cw_phase_hi;
	} else if (spindle_mode == SPINDLE_CCW ) {
		speed_lo = pwm.c[PWM_1].ccw_speed_lo;
		speed_hi = pwm.c[PWM_1].ccw_speed_hi;
		phase_lo = pwm.c[PWM_1].ccw_phase_lo;
		phase_hi = pwm.c[PWM_1].ccw_phase_hi;
	}

	if (spindle_mode==SPINDLE_CW || spindle_mode==SPINDLE_CCW ) {
		// clamp spindle speed to lo/hi range
		if( cm.gm.spindle_speed < speed_lo ) cm.gm.spindle_speed = speed_lo;
		if( cm.gm.spindle_speed > speed_hi ) cm.gm.spindle_speed = speed_hi;

		// normalize speed to [0..1]; only an exactly zero-width range ($p1csl == $p1csh) is all-on, not 0/0 (E149)
		float range = speed_hi - speed_lo;
		float speed = (range == 0) ? 1.0 : (cm.gm.spindle_speed - speed_lo) / range;  // NaN bounds stay NaN: refused
		return (speed * (phase_hi - phase_lo)) + phase_lo;
	} else {
		return pwm.c[PWM_1].phase_off;
	}
}

/*
 * cm_spindle_control() -  queue the spindle command to the planner buffer
 * cm_exec_spindle_control() - execute the spindle command (called from planner)
 */

stat_t cm_spindle_control(uint8_t spindle_mode)
{
	float value[AXES] = { (float)spindle_mode };
	mp_queue_command(_exec_spindle_control, value, FLAGS_ONE);
	return(STAT_OK);
}

//static void _exec_spindle_control(uint8_t spindle_mode, float f, float *vector, float *flag)
static void _exec_spindle_control(float *value, bool *flags)
{
	uint8_t spindle_mode = cm_outputs_inhibited() ? SPINDLE_OFF : (uint8_t)value[0];  // a queued M3/M4 may run after an alarm
	cm_set_spindle_mode(MODEL, spindle_mode);

#ifdef __AVR
    bool on = (spindle_mode == SPINDLE_CW) || (spindle_mode == SPINDLE_CCW);
    if (on) {
        bool direction_high = (spindle_mode == SPINDLE_CW) == spindle_cfg.direction_polarity;
        if (direction_high) { gpio_set_bit_on(SPINDLE_DIR); }
        else                { gpio_set_bit_off(SPINDLE_DIR); }
    }
    if (on == spindle_cfg.enable_polarity) { gpio_set_bit_on(SPINDLE_BIT); }
    else                                  { gpio_set_bit_off(SPINDLE_BIT); }
#endif // __AVR
#ifdef __ARM
	if (spindle_mode == SPINDLE_CW) {
		spindle_enable_pin.set();
		spindle_dir_pin.clear();
	} else if (spindle_mode == SPINDLE_CCW) {
		spindle_enable_pin.set();
		spindle_dir_pin.set();
	} else {
		spindle_enable_pin.clear();	// failsafe: any error causes stop
	}
#endif // __ARM

	// PWM spindle control
	pwm_set_duty(PWM_1, cm_get_spindle_pwm(spindle_mode) );
	spindle_out.mode = spindle_mode;			// the outputs as now driven (readouts)
	if (spindle_mode != SPINDLE_OFF) {
		spindle_out.ccw = (spindle_mode == SPINDLE_CCW);
	}
	_record_speed((spindle_mode != SPINDLE_OFF) ? cm.gm.spindle_speed : 0);
}

/*
 * cm_set_spindle_speed() 	- queue the S parameter to the planner buffer
 * cm_exec_spindle_speed() 	- execute the S command (called from the planner buffer)
 * _exec_spindle_speed()	- spindle speed callback from planner queue
 */
stat_t cm_set_spindle_speed(float speed)
{
//	if (speed > cfg.max_spindle speed) {    // test not currently performed
//        return (STAT_MAX_SPINDLE_SPEED_EXCEEDED);
//  }
	float value[AXES] = { speed };
	mp_queue_command(_exec_spindle_speed, value, FLAGS_ONE);
	return (STAT_OK);
}

void cm_exec_spindle_speed(float speed)
{
	cm_set_spindle_speed(speed);
}

static void _exec_spindle_speed(float *value, bool *flags)
{
	cm_set_spindle_speed_parameter(MODEL, value[0]);
	// M30 may already have reset the model while this earlier S was queued.
	// The output mode belongs to the command's place in the runtime queue.
	uint8_t mode = spindle_out.mode;
	pwm_set_duty(PWM_1, cm_get_spindle_pwm(mode));
	_record_speed(((mode == SPINDLE_CW) || (mode == SPINDLE_CCW)) ? cm.gm.spindle_speed : 0);
}
