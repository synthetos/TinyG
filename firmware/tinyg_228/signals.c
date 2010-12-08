/*
 * signals.c  - tinyg signal handling
 *
 * Part of TinyG project
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
#include "controller.h"
#include "motion_control.h"
#include "canonical_machine.h"

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
//	cm_async_end();			// stop computing and generating motions
}

void sig_term()				// end program (hard)
{
	sig_term_flag = TRUE;
//	cm_async_end();
}

void sig_pause()			// stop motion
{
	sig_pause_flag = TRUE;
//	cm_async_stop();
}

void sig_resume()			// resume motion
{
	sig_resume_flag = TRUE;
//	cm_async_start();
}
