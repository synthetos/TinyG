/*
 * signals.h - signal handlers
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
 */
#ifndef tg_signals_h
#define tg_signals_h

/*
 * Global Scope Variables and Functions
 */

uint8_t	sig_kill_flag;
uint8_t	sig_term_flag;
uint8_t	sig_pause_flag;
uint8_t	sig_resume_flag;

void sig_init(void);		// initial signal flags
void sig_kill(void);		// end program (hard)
void sig_term(void);		// end program (soft)
void sig_pause(void);		// pause motion
void sig_resume(void);		// resume motion

#endif
