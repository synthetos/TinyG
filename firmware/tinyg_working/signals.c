/*
 * signals.c  - tinyg signal handling
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *-----
 *
 *	This file is isolated from the other xio files as it has a lot of application 
 *	specific code.
 *
 */

#include <stdio.h>
#include "tinyg.h"
#include "controller.h"
#include "motion_control.h"

/*
 * signal_etx() - trap and dispatch ^c
 */

void signal_etx()				// ^c
{
	tg_reset_source();			// return control to standard device
	mc_motion_stop();			// stop computing and generating motions
	return;
}

void tg_terminate()
{
//	tg_kill();
	return;
}

void tg_pause()
{
	return;
}

void tg_resume()
{
	return;
}

