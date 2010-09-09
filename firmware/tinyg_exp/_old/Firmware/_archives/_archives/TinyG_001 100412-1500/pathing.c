/*
  pathing.c - tests paths from motion_control layer and generate optimized paths
  Part of TinyG

  Copyright (c) 2010 Alden S. Hart, Jr.

  Allows the Gcode generation and motion control layers to generate more 
    idealized / naiive paths (lines, moves).
  
  Performs the following functions:

	- Speed testing and adjustment
		- determine the maximum speed the robot can move
		- modify commands to operate within the speed envelope - or throw exception

	- Work envelope testing
		- test line for exceeding min/max and either limit or throw exception

	- Exclusion volume testing 
		- test line for collision with an interior exclusion volume
		- generate a move that avoids the exclusion

	- Move stitching and corner contouring
		- figures out if splining is needed to transition from one move to the next
		- applies path coontourining for corners (What are the G codes for this?)

	- Acceleration / Deceleration 
		- Breaks moves into sequences to provide acelleration and deceleration

  Operates by maintaining robot state, examining the line buffer and possibly 
    replacing input lines with multiple output lines exhibiting more optimized
	behavior.
	
*/
