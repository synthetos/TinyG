/*
 * signals.c  - tinyg signal handling
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
 *
 *-----
 *
 *	This file is isolated from the other xio files as it can have a lot 
 *	of application specific code.
 *
 */

#include <stdio.h>						// precursor for xio.h
#include <avr/pgmspace.h>				// precursor for xio.h
#include "tinyg.h"
#include "xio.h"

/*
 * sig_init()			init signals
 * sig_abort()			end program (hard)
 * sig_feedhold()		stop motion
 * sig_cycle_start()	start or resume motion
 */

void sig_init()
{
	sig.sig_abort = FALSE;
	sig.sig_feedhold = FALSE;
	sig.sig_cycle_start = FALSE;
}

inline void sig_abort()					// reset
{
	sig.sig_abort = TRUE;
}

inline void sig_feedhold()				// pause
{
	sig.sig_feedhold = TRUE;
}

inline void sig_cycle_start()			// start or resume
{
	sig.sig_cycle_start = TRUE;
}
