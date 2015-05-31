/*
 * xmega_rtc.h - real-time counter/clock
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../tinyg.h"
#include "../config.h"
#include "../switch.h"
#include "xmega_rtc.h"

rtClock_t rtc;		// allocate clock control struct

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

	// the following must be in this order or it doesn;t work
	RTC.PER = RTC_MILLISECONDS-1;						// set overflow period to 10ms - approximate
	RTC.CNT = 0;
	RTC.COMP = RTC_MILLISECONDS-1;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;					// no prescale (1x)
	RTC.INTCTRL = RTC_COMPINTLVL;						// interrupt on compare
	rtc.rtc_ticks = 0;									// reset tick counter
	rtc.sys_ticks = 0;									// reset tick counter
	rtc.magic_end = MAGICNUM;
}

/*
 * rtc ISR
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
	rtc.sys_ticks = ++rtc.rtc_ticks*10;		// advance both tick counters as appropriate

	// callbacks to whatever you need to happen on each RTC tick go here:
	switch_rtc_callback();					// switch debouncing
}
