/*
 * xmega_rtc.h - general purpose real-time clock
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef xmega_rtc_h
#define xmga_rtc_h


struct rtClock {
	volatile uint16_t clock_ms;
//	volatile uint16_t clock_sec;
};
struct rtClock rtc;

void rtc_init(void);		// initialize and start general timer

#endif
