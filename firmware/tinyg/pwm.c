/*
 * pwm.c - pulse width modulation drivers
 * This file is part of the TinyG project
 *
 * Copyright (c) 2012 - 2015 Alden S. Hart, Jr.
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
#include "hardware.h"
#include "text_parser.h"
#include "gpio.h"
#include "pwm.h"

#ifdef __AVR
#include <avr/interrupt.h>
#endif

#ifdef __cplusplus
extern "C"{
#endif

/***** PWM defines, structures and memory allocation *****/

pwmSingleton_t pwm;

// defines common to all PWM channels
//#define PWM_TIMER_TYPE	TC1_struct	// PWM uses TC1's
#define PWM_TIMER_t	TC1_t				// PWM uses TC1's
#define PWM_TIMER_DISABLE 0				// turn timer off (clock = 0 Hz)
#define PWM_MAX_FREQ (F_CPU/256)		// max frequency with 8-bits duty cycle precision
#define PWM_MIN_FREQ (F_CPU/64/65536)	// min frequency with supported prescaling

// channel specific defines
/* CLKSEL is used to configure default PWM clock operating ranges
 * These can be changed by pwm_freq() depending on the PWM frequency selected
 *
 * The useful ranges (assuming a 32 Mhz system clock) are:
 *	 TC_CLKSEL_DIV1_gc  - good for about 500 Hz to 125 Khz practical upper limit
 *   TC_CLKSEL_DIV2_gc  - good for about 250 Hz to  62 KHz
 *	 TC_CLKSEL_DIV4_gc  - good for about 125 Hz to  31 KHz
 *	 TC_CLKSEL_DIV8_gc  - good for about  62 Hz to  16 KHz
 *	 TC_CLKSEL_DIV64_gc - good for about   8 Hz to   2 Khz
 */
#define PWM1_CTRLA_CLKSEL	TC_CLKSEL_DIV1_gc	// starting clock select value
#define PWM1_CTRLB 			(3 | TC0_CCBEN_bm)	// single slope PWM enabled on channel B
#define PWM1_ISR_vect 		TCD1_CCB_vect		// must match timer assignments in system.h
#define PWM1_INTCTRLB		0					// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

#define PWM2_CTRLA_CLKSEL 	TC_CLKSEL_DIV1_gc
#define PWM2_CTRLB 			3					// single slope PWM enabled, no output channel
//#define PWM2_CTRLB 		(3 | TC0_CCBEN_bm)	// single slope PWM enabled on channel B
#define PWM2_ISR_vect		TCE1_CCB_vect		// must match timer assignments in system.h
#define PWM2_INTCTRLB		0					// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

/***** PWM code *****/
/*
 * pwm_init() - initialize pwm channels
 *
 *	Notes:
 *	  - Whatever level interrupts you use must be enabled in main()
 *	  - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 *	  - See system.h for timer and port assignments
 *    - Don't do this: memset(&TIMER_PWM1, 0, sizeof(PWM_TIMER_t)); // zero out the timer registers
 */
void pwm_init()
{
#ifdef __AVR
	gpio_set_bit_off(SPINDLE_PWM);

	// setup PWM channel 1
	memset(&pwm.p[PWM_1], 0, sizeof(pwmChannel_t));		// clear parent structure
	pwm.p[PWM_1].timer = &TIMER_PWM1;					// bind timer struct to PWM struct array
	pwm.p[PWM_1].ctrla = PWM1_CTRLA_CLKSEL;				// initialize starting clock operating range
	pwm.p[PWM_1].timer->CTRLB = PWM1_CTRLB;
	pwm.p[PWM_1].timer->INTCTRLB = PWM1_INTCTRLB;		// set interrupt level

	// setup PWM channel 2
	memset(&pwm.p[PWM_2], 0, sizeof(pwmChannel_t));		// clear all values, pointers and status
	pwm.p[PWM_2].timer = &TIMER_PWM2;
	pwm.p[PWM_2].ctrla = PWM2_CTRLA_CLKSEL;
	pwm.p[PWM_2].timer->CTRLB = PWM2_CTRLB;
	pwm.p[PWM_2].timer->INTCTRLB = PWM2_INTCTRLB;
#endif // __AVR
}

/*
 * ISRs for PWM timers
 */
#ifdef __AVR
ISR(PWM1_ISR_vect)
{
	return;
}

ISR(PWM2_ISR_vect)
{
	return;
}
#endif // __AVR
/*
#ifdef __ARM
MOTATE_TIMER_INTERRUPT
ISR(PWM1_ISR_vect)
{
	return;
}

ISR(PWM2_ISR_vect)
{
	return;
}
#endif // __ARM
*/
/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *	channel	- PWM channel
 *	freq	- PWM frequency in Khz as a float
 *
 *	Assumes 32MHz clock.
 *	Doesn't turn time on until duty cycle is set
 */

stat_t pwm_set_freq(uint8_t chan, float freq)
{
	if (chan > PWMS) { return (STAT_NO_SUCH_DEVICE);}
	if (freq > PWM_MAX_FREQ) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}
	if (freq < PWM_MIN_FREQ) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}

#ifdef __AVR
	// set the period and the prescaler
	float prescale = F_CPU/65536/freq;	// optimal non-integer prescaler value
	if (prescale <= 1) {
		pwm.p[chan].timer->PER = F_CPU/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV1_gc;
	} else if (prescale <= 2) {
		pwm.p[chan].timer->PER = F_CPU/2/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV2_gc;
	} else if (prescale <= 4) {
		pwm.p[chan].timer->PER = F_CPU/4/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV4_gc;
	} else if (prescale <= 8) {
		pwm.p[chan].timer->PER = F_CPU/8/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV8_gc;
	} else {
		pwm.p[chan].timer->PER = F_CPU/64/freq;
		pwm.p[chan].timer->CTRLA = TC_CLKSEL_DIV64_gc;
	}
#endif // __AVR

#ifdef __ARM
	if (chan == PWM_1) {
		spindle_pwm_pin.setFrequency(freq);
	} else if (chan == PWM_2) {
		secondary_pwm_pin.setFrequency(freq);
	}
#endif // __ARM

	return (STAT_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle
 *
 *	channel	- PWM channel
 *	duty	- PWM duty cycle from 0% to 100%
 *
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *
 *	The frequency must have been set previously
 */

stat_t pwm_set_duty(uint8_t chan, float duty)
{
	if (duty < 0.0) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}
	if (duty > 1.0) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}

	#ifdef __AVR
//  Ffrq = Fper/(2N(CCA+1))
//  Fpwm = Fper/((N(PER+1))
	float period_scalar = pwm.p[chan].timer->PER;
	pwm.p[chan].timer->CCB = (uint16_t)(period_scalar * duty) + 1;
	#endif // __AVR

	#ifdef __ARM
	if (chan == PWM_1) {
		spindle_pwm_pin = duty;
	} else if (chan == PWM_2) {
		secondary_pwm_pin = duty;
	}
	#endif // __ARM

	return (STAT_OK);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

// none


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_p1frq[] PROGMEM = "[p1frq] pwm frequency   %15.0f Hz\n";
static const char fmt_p1csl[] PROGMEM = "[p1csl] pwm cw speed lo %15.0f RPM\n";
static const char fmt_p1csh[] PROGMEM = "[p1csh] pwm cw speed hi %15.0f RPM\n";
static const char fmt_p1cpl[] PROGMEM = "[p1cpl] pwm cw phase lo %15.3f [0..1]\n";
static const char fmt_p1cph[] PROGMEM = "[p1cph] pwm cw phase hi %15.3f [0..1]\n";
static const char fmt_p1wsl[] PROGMEM = "[p1wsl] pwm ccw speed lo%15.0f RPM\n";
static const char fmt_p1wsh[] PROGMEM = "[p1wsh] pwm ccw speed hi%15.0f RPM\n";
static const char fmt_p1wpl[] PROGMEM = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] PROGMEM = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] PROGMEM = "[p1pof] pwm phase off   %15.3f [0..1]\n";

void pwm_print_p1frq(nvObj_t *nv) { text_print_flt(nv, fmt_p1frq);}
void pwm_print_p1csl(nvObj_t *nv) { text_print_flt(nv, fmt_p1csl);}
void pwm_print_p1csh(nvObj_t *nv) { text_print_flt(nv, fmt_p1csh);}
void pwm_print_p1cpl(nvObj_t *nv) { text_print_flt(nv, fmt_p1cpl);}
void pwm_print_p1cph(nvObj_t *nv) { text_print_flt(nv, fmt_p1cph);}
void pwm_print_p1wsl(nvObj_t *nv) { text_print_flt(nv, fmt_p1wsl);}
void pwm_print_p1wsh(nvObj_t *nv) { text_print_flt(nv, fmt_p1wsh);}
void pwm_print_p1wpl(nvObj_t *nv) { text_print_flt(nv, fmt_p1wpl);}
void pwm_print_p1wph(nvObj_t *nv) { text_print_flt(nv, fmt_p1wph);}
void pwm_print_p1pof(nvObj_t *nv) { text_print_flt(nv, fmt_p1pof);}

#endif //__TEXT_MODE

#ifdef __cplusplus
}
#endif
