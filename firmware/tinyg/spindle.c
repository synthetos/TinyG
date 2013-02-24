/*
 * spindle.c - canonical machine spindle driver
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/io.h>
#include <math.h>
#include "tinyg.h"
#include "gpio.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "spindle.h"
#include "planner.h"
#include "system.h"
#include "config.h"
#include "pwm.h"

static void _exec_spindle_control(uint8_t spindle_mode, double f);
static void _exec_spindle_speed(uint8_t i, double speed);

/* 
 * sp_init()
 */
void sp_init()
{
    if( cfg.p.frequency < 0 )
        cfg.p.frequency = 0;
    
    pwm_set_freq(PWM_1, cfg.p.frequency);
    pwm_set_duty(PWM_1, cfg.p.phase_off);
}

/*
 * cm_get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */
double cm_get_spindle_pwm( uint8_t spindle_mode )
{
    double speed_lo=0, speed_hi=0, phase_lo=0, phase_hi=0;
    if (spindle_mode==SPINDLE_CW ) {
        speed_lo = cfg.p.cw_speed_lo;
        speed_hi = cfg.p.cw_speed_hi;
        phase_lo = cfg.p.cw_phase_lo;
        phase_hi = cfg.p.cw_phase_hi;
    } else if (spindle_mode==SPINDLE_CCW ) {
        speed_lo = cfg.p.ccw_speed_lo;
        speed_hi = cfg.p.ccw_speed_hi;
        phase_lo = cfg.p.ccw_phase_lo;
        phase_hi = cfg.p.ccw_phase_hi;
    }
    
    if (spindle_mode==SPINDLE_CW || spindle_mode==SPINDLE_CCW ) {
        // clamp spindle speed to lo/hi range
        if( gm.spindle_speed < speed_lo ) gm.spindle_speed = speed_lo;
        if( gm.spindle_speed > speed_hi ) gm.spindle_speed = speed_hi;
        
        // normalize speed to [0..1]
        double speed = (gm.spindle_speed - speed_lo) / (speed_hi - speed_lo);
        return (speed * (phase_hi - phase_lo)) + phase_lo;
        
    } else
        return cfg.p.phase_off;
}

/*
 * cm_spindle_control() -  queue the spindle command to the planner buffer
 * cm_exec_spindle_control() - execute the spindle command (called from planner)
 */

uint8_t cm_spindle_control(uint8_t spindle_mode)
{
	mp_queue_command(_exec_spindle_control, spindle_mode, 0);
	return(TG_OK);
}
static void _exec_spindle_control(uint8_t spindle_mode, double f)
{
	cm_set_spindle_mode(spindle_mode);
 	if (spindle_mode == SPINDLE_CW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_off(SPINDLE_DIR);
	} else if (spindle_mode == SPINDLE_CCW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_on(SPINDLE_DIR);
	} else {
		gpio_set_bit_off(SPINDLE_BIT);	// failsafe: any error causes stop
	}
    
    // PWM spindle control
    pwm_set_duty(PWM_1, cm_get_spindle_pwm(spindle_mode) );
}

/*
 * cm_set_spindle_speed() - queue the S parameter to the planner buffer
 */

uint8_t cm_set_spindle_speed(double speed)
{
//	if (speed > cfg.max_spindle speed) {
//		return (TG_MAX_SPINDLE_SPEED_EXCEEDED);
//	}
	mp_queue_command(_exec_spindle_speed, 0, speed);
    return (TG_OK);
}
static void _exec_spindle_speed(uint8_t i, double speed)
{
//	cm_set_spindle_speed_parameter(speed);
    
    // update spindle speed if we're running
    pwm_set_duty(PWM_1, cm_get_spindle_pwm(gm.spindle_mode) );
}


/*
 * cm_exec_spindle_speed() - execute the S command (called from the planner buffer)
 */
void cm_exec_spindle_speed(double speed)
{
	// TODO: Link in S command and calibrations to allow dynamic spindle speed setting 
}
