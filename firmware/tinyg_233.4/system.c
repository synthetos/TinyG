/*
 * system.c - general hardware support functions
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
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
 *
 * ------
 * Notes:
 *	- add full interrupt tables and dummy interrupt routine (maybe)
 *	- add crystal oscillator failover
 *	- add watchdog timer functions
 *
 */

#include "system.h"
#include "xmega_init.h"

/*
 * hw_init() - lowest level hardware init
 */

void hw_init() 
{
	xmega_init();
}
