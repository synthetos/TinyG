/*
 * xmega_rtc.h - real-time counter/clock
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * (option) any later version.
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
#include "limit_switches.h"
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


ISR(RTC_COMP_vect)
{
	if (ls.count) {		// takes count down to zero over time.
		ls.count--;
	}
	++rtc.clock_ms;
}

void rtc_reset_ms()
{
//	RTC.INTCTRL = RTC_OVFINTLVL_OFF_gc;	// disable interrupt
	rtc.clock_ms = 0;
//	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;	// enable interrupt
}

