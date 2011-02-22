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

#include <stdio.h>
#include "tinyg.h"
#include "signals.h"

/*
 * sig_init()	init signals
 * sig_kill()	end program (hard)
 * sig_term()	end program (soft)
 * sig_pause()	stop motion
 * sig_resume()	resume motion
 */

void sig_init()
{
	sig_kill_flag = 0;
	sig_term_flag = 0;
	sig_pause_flag = 0;
	sig_resume_flag = 0;
}

void sig_kill()				// end program (hard)
{
	sig_kill_flag = TRUE;
}

void sig_term()				// end program (hard)
{
	sig_term_flag = TRUE;
}

void sig_pause()			// stop motion
{
	sig_pause_flag = TRUE;
}

void sig_resume()			// resume motion
{
	sig_resume_flag = TRUE;
}
