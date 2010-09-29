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
#include "move_buffer.h"
#include "motion_control.h"
#include "stepper.h"					// needed for stepper kill and terminate

/*
 * signal_etx() - trap and dispatch ^c
 */

void signal_etx() {			// ^c
{
	tg_reset_source();					// return control to standard device
	mc_motion_stop();					// stop computing and generating motions
	mv_flush();							// empty and reset the move queue
	st_stop_steppers();					// stop the steppers
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

/* 
 * tg_signal() - default signal handler to bind to the line readers
 */
/*
int tg_signal(uint8_t sig)
{
	switch (sig) {
		case XIO_SIG_OK: break;
 		case XIO_SIG_EOL: break;

		case XIO_SIG_EOF:
			printf_P(PSTR("\r\nEnd of file encountered\r\n"));
			_tg_prompt();
			break;

		case XIO_SIG_WOULDBLOCK: break;
		case XIO_SIG_KILL: tg_kill(); break;
		case XIO_SIG_TERMINATE: tg_terminate(); break;
		case XIO_SIG_PAUSE: tg_pause(); break;
		case XIO_SIG_RESUME: tg_resume(); break;
		case XIO_SIG_SHIFTOUT: break;
		case XIO_SIG_SHIFTIN: break;
		default: break;
	}
	return (0);
*/
}

