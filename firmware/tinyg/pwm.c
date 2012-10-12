/*
 * pwm.c - pulse width modulation drivers
 * Part of TinyG project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
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

#include <stdlib.h>
#include <string.h>				// needed for memset
#include <avr/interrupt.h>
#include <avr/io.h>

#include "tinyg.h"
#include "system.h"
#include "pwm.h"

/***** PWM defines, structures and memory allocation *****/

// defines common to all PWM channels
#define PWM_TIMER_TYPE	TC1_struct	// PWM uses TC1's
#define PWM_TIMER_DISABLE 0			// turn timer off (clock = 0 Hz)
#define PWM_MAX_FREQ (F_CPU/256)	// max frequency with 8-bits duty cycle precision
#define PWM_MIN_FREQ (F_CPU/64/65536)// min frequency with supported prescaling

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
#define PWM1_CTRLA_CLKSEL TC_CLKSEL_DIV1_gc	// starting clock select value
#define PWM1_CTRLB (3 | TC0_CCBEN_bm)// single slope PWM enabled on channel B
#define PWM1_ISR_vect TCD1_CCB_vect	// must match timer assignments in system.h
#define PWM1_INTCTRLB		0		// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

#define PWM2_CTRLA_CLKSEL TC_CLKSEL_DIV1_gc
#define PWM2_CTRLB 			3		// single slope PWM enabled, no output channel
#define PWM2_ISR_vect TCE1_CCB_vect	// must match timer assignments in system.h
#define PWM2_INTCTRLB		0		// timer interrupt level (0=off, 1=lo, 2=med, 3=hi)

struct pwmStruct { 					// one per PWM channel
	uint8_t ctrla;					// byte needed to active CTRLA (it's dynamic - rest are static)
	struct TC1_struct *timer;		// assumes TC1 flavor timers used for PWM channels
};
static struct pwmStruct pwm[PWMS];	// array of PWMs (usually 2, see system.h)

/***** PWM code *****/
/* 
 * pwm_init() - initialize pwm channels
 *
 *	Notes: 
 *	  - Whatever level interrupts you use must be enabled in main()
 *	  - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 */
void pwm_init()
{
	// setup PWM channel 1
	memset(&pwm[PWM_1], 0, sizeof(struct pwmStruct));			// clear parent structure 
	memset(&pwm[PWM_1].timer, 0, sizeof(struct PWM_TIMER_TYPE));// zero out the timer registers
	pwm[PWM_1].timer = &TIMER_PWM1;			// bind timer structs to PWM struct array
	pwm[PWM_1].ctrla = PWM1_CTRLA_CLKSEL;	// initialize starting clock operating range
	pwm[PWM_1].timer->CTRLB = PWM1_CTRLB;
	pwm[PWM_1].timer->INTCTRLB = PWM1_INTCTRLB;// set interrupt level	

	// setup PWM channel 2
	memset(&pwm[PWM_2], 0, sizeof(struct pwmStruct));	// clear all values, pointers and status
	memset(&pwm[PWM_2].timer, 0, sizeof(struct PWM_TIMER_TYPE));// zero out the timer registers
	pwm[PWM_2].timer = &TIMER_PWM2;
	pwm[PWM_2].ctrla = PWM2_CTRLA_CLKSEL;
	pwm[PWM_2].timer->CTRLB = PWM2_CTRLB;
	pwm[PWM_2].timer->INTCTRLB = PWM2_INTCTRLB;
}

/*
 * ISRs for PWM timers
 */
ISR(PWM1_ISR_vect) 
{
	return;
}

ISR(PWM2_ISR_vect) 
{
	return;
}

/* 
 * pwm_set_freq() - set PWM channel frequency
 *
 *	channel	- PWM channel
 *	freq	- PWM frequency in Khz as a double
 *
 *	Assumes 32MHz clock.
 *	Doesn't turn time on until duty cycle is set
 */
uint8_t pwm_set_freq(uint8_t chan, double freq)
{
	if (chan > PWMS) { return (TG_NO_SUCH_DEVICE);}
	if (freq > PWM_MAX_FREQ) { return (TG_INPUT_VALUE_TOO_SMALL);}
	if (freq < PWM_MIN_FREQ) { return (TG_INPUT_VALUE_TOO_LARGE);}

	// set the period and the prescaler
	double prescale = F_CPU/65536/freq;	// optimal non-integer prescaler value
	if (prescale <= 1) { 
		pwm[chan].timer->PER = F_CPU/freq;
		pwm[chan].timer->CTRLA = TC_CLKSEL_DIV1_gc;
	} else if (prescale <= 2) { 
		pwm[chan].timer->PER = F_CPU/2/freq;
		pwm[chan].timer->CTRLA = TC_CLKSEL_DIV2_gc;
	} else if (prescale <= 4) { 
		pwm[chan].timer->PER = F_CPU/4/freq;
		pwm[chan].timer->CTRLA = TC_CLKSEL_DIV4_gc;
	} else if (prescale <= 8) { 
		pwm[chan].timer->PER = F_CPU/8/freq;
		pwm[chan].timer->CTRLA = TC_CLKSEL_DIV8_gc;
	} else { 
		pwm[chan].timer->PER = F_CPU/64/freq;
		pwm[chan].timer->CTRLA = TC_CLKSEL_DIV64_gc;
	}
	return (TG_OK);
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
uint8_t pwm_set_duty(uint8_t chan, double duty)
{
	if (duty < 0)   { return (TG_INPUT_VALUE_TOO_SMALL);}
	if (duty > 100) { return (TG_INPUT_VALUE_TOO_LARGE);}

	pwm[chan].timer->CCB = (uint16_t)(pwm[chan].timer->PER - pwm[chan].timer->PER / (duty/100));
	return (TG_OK);
}

//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_PWM

void pwm_unit_tests()
{
	pwm_init();

	pwm_set_freq(PWM_1,500);
	pwm_set_duty(PWM_1,100);
	pwm_set_duty(PWM_1,75);
	pwm_set_duty(PWM_1,51);
	pwm_set_duty(PWM_1,10);
	pwm_set_duty(PWM_1,0);

	pwm_set_freq(PWM_1,5000);
	pwm_set_duty(PWM_1,100);
	pwm_set_duty(PWM_1,75);
	pwm_set_duty(PWM_1,51);
	pwm_set_duty(PWM_1,10);
	pwm_set_duty(PWM_1,0);

	pwm_set_freq(PWM_1,100);
	pwm_set_duty(PWM_1,100);
	pwm_set_duty(PWM_1,75);
	pwm_set_duty(PWM_1,51);
	pwm_set_duty(PWM_1,10);
	pwm_set_duty(PWM_1,0);

}

#endif // __UNIT_TEST_PWM

