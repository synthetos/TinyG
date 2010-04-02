/*
 * tg_controller.h - tinyg controller and top level parsers
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
#ifndef controller_h
#define controller_h

/*
 * Global Scope Functions
 */

void tg_init(void);
void tg_controller(void);
int tg_parser(char * buf);
int tg_signal(uint8_t sig);
void tg_kill(void);
void tg_terminate(void);
void tg_pause(void);
void tg_resume(void);

#endif
