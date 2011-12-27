/*
 * xmega_rtc.h - real-time counter/clock
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "tinyg.h"
#include "gpio.h"
#include "xmega_rtc.h"

#define RTC_TICKS 10		// interrupt on every 10 ticks (~10 ms)

/* 
 * rtc_init() - initialize and start the clock
 *
 * The order of the instructions in this routine follows app note 1314.
 * Best not to mess with it.
 */

void rtc_init()
{
	do {
		// wait for syncbusy to clear
	} while (RTC.STATUS);

	// RTC register setup
	CLK.RTCCTRL	= 0x05;					// use internal RTC 32.768, ENABLE
	RTC.PER = RTC_TICKS;				// overflow period
	RTC.COMP = RTC_TICKS;				// overflow period
	RTC.CNT = 0;
	RTC.INTCTRL = RTC_COMPINTLVL_LO_gc;	// lo interrupt on overflow
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;	// no prescale (1x)

	// app derived clocks
	rtc.clock_ms = 0;
}

/* 
 * rtc ISR 
 */

ISR(RTC_COMP_vect)
{
	sw_rtc_callback();	// callback to timer module to handle ticks
	++rtc.clock_ms;		// increment real time clock (unused)
}

void rtc_reset_ms()
{
//	RTC.INTCTRL = RTC_OVFINTLVL_OFF_gc;	// disable interrupt
	rtc.clock_ms = 0;
//	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;	// enable interrupt
}

/**** WORK ABANDONED FROM  327.17 ****
See also use of switch_lockout_timer in gpio.c/.h

#define RTC_OVERFLOW_TICKS 10		// interrupt every N ticks; 1 tick ~= 1ms
#define RTC_SWITCH_LOCKOUT_TICKS 25	// lockout interval in ticks (max 255)
#define RTC_IDLE_DETECT_TICKS 25	// idle detection interval in ticks (max 255)

struct rtClock {
	volatile double elapsed_time;	// time kept as second w/fractions
	volatile uint8_t switch_lockout_timer;
	volatile uint8_t idle_detect_timer;
};
struct rtClock rtc;

void rtc_init(void);				// initialize and start general timer
void rtc_reset_all_timers(void);
void rtc_reset_elapsed_time(void);
void rtc_reset_switch_lockout_timer(void);
void rtc_reset_idle_detect_timer(void);


*
 *	rtc resets
 *
 *	rtc_reset_all_timers()	 			- reset all timers
 *	rtc_reset_elapsed_time() 			- reset elapsed time
 *	rtc_reset_switch_lockout_counter()	- set counter to TICKS value
 *	rtc_reset_idle_detect_counter()		- set counter to TICKS value
 *

void rtc_reset_all_timers()
{
	rtc_reset_elapsed_time();
	rtc_reset_switch_lockout_timer();
	rtc_reset_idle_detect_timer();
}

void rtc_reset_elapsed_time(void)
{
	RTC.INTCTRL = RTC_OVFINTLVL_OFF_gc;	// disable interrupt
	rtc.elapsed_time = 0;
	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;	// enable interrupt
}

void rtc_reset_switch_lockout_timer(void)
{
	rtc.switch_lockout_timer = RTC_SWITCH_LOCKOUT_TICKS; // no int disable/enable - only 1 byte 
}

void rtc_reset_idle_detect_timer(void)
{
	rtc.idle_detect_timer = RTC_IDLE_DETECT_TICKS; // no int disable/enable - only 1 byte 
}

*/
