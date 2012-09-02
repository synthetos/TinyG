/*
 * xmega_rtc.h - real-time counter/clock
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../tinyg.h"
#include "../gpio.h"
#include "xmega_rtc.h"

/* 
 * rtc_init() - initialize and start the clock
 *
 * This routine follows the code in app note AVR1314.
 */

void rtc_init()
{
	OSC.CTRL |= OSC_RC32KEN_bm;							// Turn on internal 32kHz.
	do {} while ((OSC.STATUS & OSC_RC32KRDY_bm) == 0);	// Wait for 32kHz oscillator to stabilize.
	do {} while (RTC.STATUS & RTC_SYNCBUSY_bm);			// Wait until RTC is not busy

	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;	// Set internal 32kHz osc as RTC clock source
	do {} while (RTC.STATUS & RTC_SYNCBUSY_bm);			// Wait until RTC is not busy

	RTC.PER = RTC_PERIOD-1;								// overflow period
	RTC.CNT = 0;
	RTC.COMP = RTC_PERIOD-1;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;					// no prescale (1x)
	RTC.INTCTRL = RTC_COMPINTLVL;						// interrupt on compare

	rtc.clock_ticks = 0;								//  default RTC clock counter
}

/* 
 * rtc ISR 
 *
 * This used to have application-specific clocks and timers in it but that approach
 * was abandoned because I decided it was better to just provide callbacks to the 
 * relevant code modules to perform those functions.
 *
 * It is the responsibility of the callback code to ensure atomicity and volatiles
 * are observed correctly as the callback will be run at the interrupt level.
 *
 * Here's the code in case the main loop (non-interrupt) function needs to 
 * create a critical region for variables set or used by the callback:
 *
 *		#include "gpio.h"
 *		#include "xmega_rtc.h"
 *
 *		RTC.INTCTRL = RTC_OVFINTLVL_OFF_gc;	// disable interrupt
 * 		blah blah blah critical region
 *		RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;	// enable interrupt
 */

ISR(RTC_COMP_vect)
{
	// callbacks to whatever you need to happen on each RTC tick go here:
	gpio_switch_timer_callback();		// switch debouncing

	// here's the default RTC timer clock
	++rtc.clock_ticks;					// increment real time clock (unused)
}

void rtc_reset_rtc_clock()
{
//	RTC.INTCTRL = RTC_OVFINTLVL_OFF_gc;	// disable interrupt
	rtc.clock_ticks = 0;
//	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;	// enable interrupt
}
