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

#ifdef __cplusplus
extern "C"{
#endif

static void _exec_spindle_control(float *value, float *flag);
static void _exec_spindle_speed(float *value, float *flag);

/*
 * cm_spindle_init()
 */
void cm_spindle_init()
{
	if( pwm.c[PWM_1].frequency < 0 )
		pwm.c[PWM_1].frequency = 0;

    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
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

		// normalize speed to [0..1]
		float speed = (cm.gm.spindle_speed - speed_lo) / (speed_hi - speed_lo);
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
	float value[AXES] = { (float)spindle_mode, 0,0,0,0,0 };
	mp_queue_command(_exec_spindle_control, value, value);
	return(STAT_OK);
}

//static void _exec_spindle_control(uint8_t spindle_mode, float f, float *vector, float *flag)
static void _exec_spindle_control(float *value, float *flag)
{
	uint8_t spindle_mode = (uint8_t)value[0];
	cm_set_spindle_mode(MODEL, spindle_mode);

 #ifdef __AVR
	if (spindle_mode == SPINDLE_CW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_off(SPINDLE_DIR);
	} else if (spindle_mode == SPINDLE_CCW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_on(SPINDLE_DIR);
	} else {
		gpio_set_bit_off(SPINDLE_BIT);	// failsafe: any error causes stop
	}
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
}

/*
 * cm_set_spindle_speed() 	- queue the S parameter to the planner buffer
 * cm_exec_spindle_speed() 	- execute the S command (called from the planner buffer)
 * _exec_spindle_speed()	- spindle speed callback from planner queue
 */
stat_t cm_set_spindle_speed(float speed)
{
//	if (speed > cfg.max_spindle speed)
//        return (STAT_MAX_SPINDLE_SPEED_EXCEEDED);

	float value[AXES] = { speed, 0,0,0,0,0 };
	mp_queue_command(_exec_spindle_speed, value, value);
	return (STAT_OK);
}

void cm_exec_spindle_speed(float speed)
{
	cm_set_spindle_speed(speed);
}

static void _exec_spindle_speed(float *value, float *flag)
{
	cm_set_spindle_speed_parameter(MODEL, value[0]);
	pwm_set_duty(PWM_1, cm_get_spindle_pwm(cm.gm.spindle_mode) ); // update spindle speed if we're running
}

#ifdef __cplusplus
}
#endif
