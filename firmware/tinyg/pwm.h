/*
 * pwm.h - pulse width modulation drivers
 * This file is part of the TinyG project
 *
 * Copyright (c) 2012 - 2014 Alden S. Hart, Jr.
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

#ifndef PWM_H_ONCE
#define PWM_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

typedef struct pwmConfigChannel {
	float frequency;				// base frequency for PWM driver, in Hz
	float cw_speed_lo;				// minimum clockwise spindle speed [0..N]
	float cw_speed_hi;				// maximum clockwise spindle speed
	float cw_phase_lo;				// pwm phase at minimum CW spindle speed, clamped [0..1]
	float cw_phase_hi;				// pwm phase at maximum CW spindle speed, clamped [0..1]
	float ccw_speed_lo;				// minimum counter-clockwise spindle speed [0..N]
	float ccw_speed_hi;				// maximum counter-clockwise spindle speed
	float ccw_phase_lo;				// pwm phase at minimum CCW spindle speed, clamped [0..1]
	float ccw_phase_hi;				// pwm phase at maximum CCW spindle speed, clamped
	float phase_off;				// pwm phase when spindle is disabled
} pwmConfigChannel_t;

typedef struct pwmChannel {
	uint8_t ctrla;					// byte needed to active CTRLA (it's dynamic - rest are static)
#ifdef __AVR
	TC1_t *timer;					// assumes TC1 flavor timers used for PWM channels
#endif
} pwmChannel_t;

typedef struct pwmSingleton {
	pwmConfigChannel_t  c[PWMS];	// array of channel configs
	pwmChannel_t 		p[PWMS];	// array of PWM channels
} pwmSingleton_t;

extern pwmSingleton_t pwm;

/*** function prototypes ***/

void pwm_init(void);
stat_t pwm_set_freq(uint8_t channel, float freq);
stat_t pwm_set_duty(uint8_t channel, float duty);

#ifdef __TEXT_MODE

	void pwm_print_p1frq(nvObj_t *nv);
	void pwm_print_p1csl(nvObj_t *nv);
	void pwm_print_p1csh(nvObj_t *nv);
	void pwm_print_p1cpl(nvObj_t *nv);
	void pwm_print_p1cph(nvObj_t *nv);
	void pwm_print_p1wsl(nvObj_t *nv);
	void pwm_print_p1wsh(nvObj_t *nv);
	void pwm_print_p1wpl(nvObj_t *nv);
	void pwm_print_p1wph(nvObj_t *nv);
	void pwm_print_p1pof(nvObj_t *nv);

#else

	#define pwm_print_p1frq tx_print_stub
	#define pwm_print_p1csl tx_print_stub
	#define pwm_print_p1csh tx_print_stub
	#define pwm_print_p1cpl tx_print_stub
	#define pwm_print_p1cph tx_print_stub
	#define pwm_print_p1wsl tx_print_stub
	#define pwm_print_p1wsh tx_print_stub
	#define pwm_print_p1wpl tx_print_stub
	#define pwm_print_p1wph tx_print_stub
	#define pwm_print_p1pof tx_print_stub

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif

#endif	// End of include guard: PWM_H_ONCE
