/*
 * direct_drive.h - commands to drive stepper motors directly (no Gcode)
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
 *
 *	Command				
 *	---------------------------------------------------------------------------
 *	A	Axes - xyza1234
 *	R	Rate (whole steps per second)
 *	D	Direction (1=CW, 0=CCW)
 *	T	Time (seconds until stop)
 *	S	Steps (whole step count until stop)
 *	M	Microsteps (One of: 1,2,4,8)
 *	E	Enable (1=Enable, 0=Disable)
 *	W	Wait (wait N seconds before executing this command)
 *	/	Comment (ignore all chars until end-of-line)
 *	*	Asterisk (is returned when command is complete)
 *
 *	All values are sticky - they set a mode and persist across command lines
 *	Cases where this doesn't make sense are described
 *
 *	Axes can specify one or more axes for a command.
 *	When we extend beyond 9 axes we will require commas or spaces
 *
 *	Rate is the number of whole steps per second
 *	Can be fractional (float)
 *	If microsteps is other than 1, the real step rate will be multiplied
 *
 *	Direction is 1 or 0, CW or CCW
 *
 *	Time is the seconds before stop
 *	Sticky - it continues to count down if another command is sent w/o a Time
 *	 0 = Stop now
 *	-1 = Infinite. Do not stop until overridden by Steps or another Time 
 *
 *	Steps is the number of whole steps to make before stopping 
 *	
 *	Microsteps changes how pulses are sent to the motors, but does not change 
 *	the Rate or Steps, which are applied as whole steps
 *
 *	Enable enables or disables the active axes
 *
 *	Wait N seconds before executing the command
 *	Lack of a W (or a W0) means execute command immediately
 *	Wait -1 means wait until previous command is complete (all axes stopped)
 *
 *	Syntax and behavior
 *		- upper and lower ase are equivalent
 *		- whitespace is ignored
 *		- lines are terminated with NEWLINE and executed on termination
 *		- a new command without a Wait will begin execution immediately
 *		- a new command with a Wait begins execution when current command is done
 *		- a command is considered complete when all axes are stopped
 *
 *	Some valid direct drive commands
 *
 *		Axyza r1200 t25.2 d1 m8 e1		// start everything running
 *		axz r1210 w5.23					// run X and Z faster to turn the car right
 *		w10 axyza t0					// wait another 10 seconds and stop all axes
 *		
 */

#ifndef diect_drive_h
#define diect_drive_h

//#include "tinyg.h"

/*
 * Global Scope Functions
 */

void dd_init(void);
int dd_parser(char *text);	// parse and execute a direct drive command

#endif
