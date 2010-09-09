/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify it under the terms
  of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  Grbl is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
  PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with Grbl.  
  If not, see <http://www.gnu.org/licenses/>.
*/
/* This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the 
   NIST RS274/NGC Interpreter by Kramer, Proctor and Messina. 

   Commands omitted for the time being:
	- group 0 = {G10, G28, G30, G92, G92.1, G92.2, G92.3} (Non modal G-codes)
	- group 8 = {M7, M8, M9} coolant
	- group 9 = {M48, M49} enable/disable feed and speed override switches
	- group 12 = {G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} 
				  (coordinate system selection)
	- group 13 = {G61, G61.1, G64} path control mode

   Commands intentionally not supported:
	- Canned cycles
	- Tool radius compensation
	- A,B,C-axes
	- Multiple coordinate systems
	- Evaluation of expressions
	- Variables (Parameters)
	- Multiple home locations
	- Probing
	- Override control
*/
/* 
  TinyG Notes:
  Modified Grbl to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

  Added local gc_execute_line variables to round out ParserState (gc) struct, 
  	and put the variable inits in gc_init (or gc_execute_line) as appropriate
  Eliminated unsued variables and consolidated some code
  Broke out G2/G3 computations into helper routine
  Added gc_print_status to better support hand generated G code and experimentation

*/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/pgmspace.h>			// needed for exception strings
#include "nuts_bolts.h"
#include "gcode.h"					// must precede config.h
#include "config.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "errno.h"
#include "wiring_serial.h"			// needed for exception strings
#include "serial_protocol.h"

#define NEXT_ACTION_DEFAULT 	0
#define NEXT_ACTION_DWELL 		1
#define NEXT_ACTION_GO_HOME 	2

#define MOTION_MODE_RAPID_LINEAR 0 		// G0 
#define MOTION_MODE_LINEAR 		1 		// G1
#define MOTION_MODE_CW_ARC 		2  		// G2
#define MOTION_MODE_CCW_ARC 	3  		// G3
#define MOTION_MODE_CANCEL 		4 		// G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS  2

#define PROGRAM_FLOW_RUNNING	0
#define PROGRAM_FLOW_PAUSED		1
#define PROGRAM_FLOW_COMPLETED	2

#define SPINDLE_DIRECTION_CW	0
#define SPINDLE_DIRECTION_CCW	1

struct ParserState {
	uint8_t status_code;
	char letter;					// parser values
	double value;

	uint8_t program_flow;
	uint8_t motion_mode;			// G0, G1, G2, G3, G38.2, G80, G81, G82, G83, 
									// ...G84, G85, G86, G87, G88, G89
	uint8_t inverse_feed_rate_mode; // G93, G94
	uint8_t inches_mode;         	//0 = millimeter mode, 1 = inches mode {G20,G21}
	uint8_t absolute_mode;       	//0 = relative motion, 1 = absolute motion {G90,G91}
	uint8_t radius_mode;

  	double dwell_time; 				// (was 'p' in older code)
	double radius;					// radius value
	double feed_rate; 				// millimeters/second
	double seek_rate;				// millimeters/second
	double unit_converted_value;
	double inverse_feed_rate; 		// negative inverse_feed_rate means no inverse_feed_rate specified
	double position[3];				// where the interpreter considers the tool
	double target[3]; 				// where the move should go
	double offset[3];  

	uint8_t plane_axis_0; 			// axes of the selected plane
	uint8_t plane_axis_1; 
	uint8_t plane_axis_2; 

	uint8_t tool;
	int8_t spindle_direction;
	int16_t spindle_speed;			// RPM/100
	uint8_t absolute_override; 		// TRUE (1) = absolute motion for this block only {G53}
	uint8_t next_action; 			// One of the NEXT_ACTION_-constants
};

struct ParserState gc;

#define FAIL(status) gc.status_code = status;

int read_double(char *textline, int *i, double *double_ptr);
int next_statement(char *letter, double *double_ptr, char *line, int *i);
int gc_arc_move(void);

/* gc_init() */

void gc_init() {
	memset(&gc, 0, sizeof(gc));				// must set doubles independently (is this true?)
  	gc.dwell_time = 0; 						// was 'p' 
	gc.radius = 0;							// radius value

	gc.feed_rate = settings.default_feed_rate/60;
	gc.seek_rate = settings.default_seek_rate/60;
	gc.absolute_mode = TRUE;
	gc.inverse_feed_rate = -1; 	// negative inverse_feed_rate means no inverse_feed_rate specified
	gc.radius_mode = FALSE;
	gc.absolute_override = FALSE; 			// TRUE (1) = absolute motion for this block only {G53}
	gc.next_action = NEXT_ACTION_DEFAULT; 	// One of the NEXT_ACTION_-constants

	select_plane(X_AXIS, Y_AXIS, Z_AXIS);
}


/* select_plane() - select axis plane */

void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2) 
{
	gc.plane_axis_0 = axis_0;
	gc.plane_axis_1 = axis_1;
	gc.plane_axis_2 = axis_2;
}


inline float to_millimeters(double value) 
{
	return(gc.inches_mode ? (value * INCHES_PER_MM) : value);
}

/* theta(double x, double y)
	Find the angle in radians of deviance from the positive y axis. 
	negative angles to the left of y-axis, positive to the right.
*/
double theta(double x, double y)
{
	double theta = atan(x/fabs(y));

	if (y>0) {
		return(theta);
	} else {
		if (theta>0) 
	    {
			return(M_PI-theta);
    	} else {
			return(-M_PI-theta);
		}
	}
}

/* next_statement() - parse next block of Gcode

	Parses the next statement and leaves the counter on the first character following
	the statement. Returns 1 if there was a statement, 0 if end of string was reached
	or there was an error (check state.status_code).

*/
int next_statement(char *letter, double *double_ptr, char *textline, int *i) {
	if (textline[*i] == 0) {
		return(0); // No more statements
	}
  
	*letter = textline[*i];
	if((*letter < 'A') || (*letter > 'Z')) {
		FAIL(GCSTATUS_EXPECTED_COMMAND_LETTER);
		return(0);
	}
	(*i)++;
	if (!read_double(textline, i, double_ptr)) {
		return(0);
	};
	return(1);
}

/* read_double() - read a double from a Gcode statement 

	textline	string: line of RS274/NGC code being processed
	i			index into string array (position on the line)
	double_ptr	pointer to double to be read
*/

int read_double(char *textline, int *i, double *double_ptr) 
{
	char *start = textline + *i;
	char *end;
  
	*double_ptr = strtod(start, &end);
	if(end == start) { 
		FAIL(GCSTATUS_BAD_NUMBER_FORMAT); 
		return(0); 
	};
	*i = end - textline;
	return(1);
}

/************************************************************************************
  gc_execute_line() - executes one line of NULL terminated G-Code. 

	The line is assumed to contain only uppercase characters and signed floats 
	(no whitespace).

************************************************************************************/

uint8_t gc_execute_line(char *textline) 
{
	int i = 0;  		// index into Gcode block input line
	double dtmp = 0;	// used for getting parameter settings
  
	clear_vector(gc.target);
	clear_vector(gc.offset);

	gc.status_code = GCSTATUS_OK;

  // First: parse all statements
  
	if (textline[0] == '(') { 
		return(gc.status_code); 
	}
	if (textline[0] == '/') { 	// ignore block delete ++++++ Shouldn't we skip this line?
		i++; 
	} 
  	if (textline[0] == '$') { 	// This is a parameter line intended to change 
								// EEPROM-settings
    							// Parameter lines are on the form '$4=374.3'
								// or '$' to dump current settings
		i = 1;
		if(textline[i] == 0) {
			dump_settings(); return(GCSTATUS_OK); 
		}
	    read_double(textline, &i, &dtmp);

    	if(textline[i++] != '=') { 
			return(GCSTATUS_UNSUPPORTED_STATEMENT); 
		}
    	read_double(textline, &i, &gc.value);

    	if(textline[i] != 0) { 
			return(GCSTATUS_UNSUPPORTED_STATEMENT); 
		}
    	store_setting(dtmp, gc.value);
  	}
  
  // Pass 1: Commands
	while(next_statement(&gc.letter, &gc.value, textline, &i)) {
    	switch(gc.letter) {
			case 'G':
				switch((int)gc.value) {
					case 0:  { gc.motion_mode = MOTION_MODE_RAPID_LINEAR; break; }
					case 1:  { gc.motion_mode = MOTION_MODE_LINEAR; break; }
					case 2:  { gc.motion_mode = MOTION_MODE_CW_ARC; break; }
					case 3:  { gc.motion_mode = MOTION_MODE_CCW_ARC; break; }
					case 4:  { gc.next_action = NEXT_ACTION_DWELL; break; }
					case 17: { select_plane(X_AXIS, Y_AXIS, Z_AXIS); break; }
					case 18: { select_plane(X_AXIS, Z_AXIS, Y_AXIS); break; }
					case 19: { select_plane(Y_AXIS, Z_AXIS, X_AXIS); break; }
					case 20: { gc.inches_mode = TRUE; break; }
					case 21: { gc.inches_mode = FALSE; break; }
					case 28: { gc.next_action = NEXT_ACTION_GO_HOME; break; }
					case 30: { gc.next_action = NEXT_ACTION_GO_HOME; break; }
					case 53: { gc.absolute_override = TRUE; break; }
					case 80: { gc.motion_mode = MOTION_MODE_CANCEL; break; }
					case 90: { gc.absolute_mode = TRUE; break; }
					case 91: { gc.absolute_mode = FALSE; break; }
					case 93: { gc.inverse_feed_rate_mode = TRUE; break; }
					case 94: { gc.inverse_feed_rate_mode = FALSE; break; }
					default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
				}
				break;
      
			case 'M':
				switch((int)gc.value) {
					case 0: case 1: gc.program_flow = PROGRAM_FLOW_PAUSED; break;
					case 2: case 30: case 60: gc.program_flow = PROGRAM_FLOW_COMPLETED; break;
					case 3: gc.spindle_direction = 1; break;
					case 4: gc.spindle_direction = -1; break;
					case 5: gc.spindle_direction = 0; break;
        			default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
				}
				break;

			case 'T': gc.tool = trunc(gc.value); break;
		}
		if(gc.status_code) {
			break;
		}
	}
  
  // If there were any errors parsing this line return right away with the bad news
	if (gc.status_code) { 
		return(gc.status_code); 
	}

	i = 0;
	clear_vector(gc.offset);
	memcpy(gc.target, gc.position, sizeof(gc.target)); // target = gc.position

  // Pass 2: Parameters
	while(next_statement(&gc.letter, &gc.value, textline, &i)) {
		gc.unit_converted_value = to_millimeters(gc.value);
		switch(gc.letter) {
			case 'F': 
				if (gc.inverse_feed_rate_mode) {
					gc.inverse_feed_rate = gc.unit_converted_value; // seconds per motion for this motion only
				} else {
					gc.feed_rate = gc.unit_converted_value/60; // millimeters per second
				}
				break;
			case 'I': case 'J': case 'K': gc.offset[gc.letter-'I'] = gc.unit_converted_value; break;
			case 'P': gc.dwell_time = gc.value; break;
			case 'R': gc.radius = gc.unit_converted_value; gc.radius_mode = TRUE; break;
			case 'S': gc.spindle_speed = gc.value; break;
			case 'X': case 'Y': case 'Z':
				if (gc.absolute_mode || gc.absolute_override) {
					gc.target[gc.letter - 'X'] = gc.unit_converted_value;
				} else {
					gc.target[gc.letter - 'X'] += gc.unit_converted_value;
				}
 				break;
		}	
	}
  
  // If there were any errors parsing this line return right away with the bad news
  	if (gc.status_code) {
		return(gc.status_code); 
	}
    
  // Update spindle state
	if (gc.spindle_direction) {
    	spindle_run(gc.spindle_direction, gc.spindle_speed);
	} else {
		spindle_stop();
	}
  
  // Perform any physical actions
	switch (gc.next_action) {
    	case NEXT_ACTION_GO_HOME: mc_go_home(); break;
		case NEXT_ACTION_DWELL: mc_dwell(trunc(gc.dwell_time*1000)); break;
		case NEXT_ACTION_DEFAULT: 
 		switch (gc.motion_mode) {
			case MOTION_MODE_CANCEL: break;
			case MOTION_MODE_RAPID_LINEAR:
			case MOTION_MODE_LINEAR:
				mc_line(gc.target[X_AXIS], gc.target[Y_AXIS], gc.target[Z_AXIS], 
						(gc.inverse_feed_rate_mode) ? 
						gc.inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);
			break;
			case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC: gc_arc_move(); break;
		}
	}

	/* As far as the parser is concerned the position is now == target. In reality 
	   the motion control system might still be processing the action and the real 
	   tool position in any intermediate location.
	*/
	memcpy(gc.position, gc.target, sizeof(gc.target));
	return(gc.status_code);
}


/************************************************************************************
 gc_arc_move() - arc move helper routine 
************************************************************************************/

int gc_arc_move()
{
	if (gc.radius_mode) {
	
	/*  We need to calculate the center of the circle that has the designated 
		radius and passes through both the current position and the target position
		  
		This method calculates the following set of equations where:
		   [x,y] is the vector from current to target position, 
			d == magnitude of that vector, 
			h == hypotenuse of the triangle formed by the radius of the circle, 
				 the distance to the center of the travel vector. 
		  
		A vector perpendicular to the travel vector [-y,x] is scaled to the length
		of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2]
		to form the new point [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the 
		center of our arc.
          
          	d^2 == x^2 + y^2
          	h^2 == r^2 - (d/2)^2
          	i == x/2 - y/d*h
          	j == y/2 + x/d*h
          
                                                   O <- [i,j]
                                                -  |
                                      r      -     |
                                          -        |
                                       -           | h
                                    -              |
                      [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                | <------ d/2 ---->|
                    
          	C - Current position
          	T - Target position
          	O - center of circle that pass through both C and T
          	d - distance from C to T
          	r - designated radius
          	h - distance from center of CT to O
          
		Expanding the equations:

          	d -> sqrt(x^2 + y^2)
          	h -> sqrt(4 * r^2 - x^2 - y^2)/2
          	i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
          	j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
         
		Which can be written:
          
          	i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          	j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          
		Which we for size and speed reasons optimize to:

          	h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
          	i = (x - (y * h_x2_div_d))/2
          	j = (y + (x * h_x2_div_d))/2  
        */
        
		// Calculate the change in position along each selected axis
		double x = gc.target[gc.plane_axis_0]-gc.position[gc.plane_axis_0];
		double y = gc.target[gc.plane_axis_1]-gc.position[gc.plane_axis_1];
        
		clear_vector(&gc.offset);
		double h_x2_div_d = -sqrt(4 * gc.radius*gc.radius - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)

		// If r is smaller than d, the arc is now traversing the complex plane beyond
		// the reach of any real CNC, and thus - for practical reasons - we will 
		// terminate promptly:
		if(isnan(h_x2_div_d)) { 
			FAIL(GCSTATUS_FLOATING_POINT_ERROR); 
			return(gc.status_code); 
		}

		// Invert the sign of h_x2_div_d if circle is counter clockwise 
		// (see sketch below)
		if (gc.motion_mode == MOTION_MODE_CCW_ARC) {
			h_x2_div_d = -h_x2_div_d;
		}
        
		/*	The counter clockwise circle lies to the left of the target direction. 
			When offset is positive, the left hand circle will be generated - 
			when it is negative the right hand circle is generated.

           
                                         T  <-- Target position
                                                         
                                         ^ 
            Clockwise circles with       |     Clockwise circles with
			this center will have        |     this center will have
            > 180 deg of angular travel  |     < 180 deg of angular travel, 
                              \          |      which is a good thing!
                               \         |         /
        center of arc when  ->  x <----- | -----> x <- center of arc when 
        h_x2_div_d is positive           |             h_x2_div_d is negative
                                         |
                  
                                         C  <-- Current position

		*/
                

		// Negative R is g-code-alese for "I want a circle with more than 180 degrees
		// of travel" (go figure!), even though it is advised against ever generating
		// such circles in a single line of g-code. By inverting the sign of 
		// h_x2_div_d the center of the circles is placed on the opposite side of 
		// the line of travel and thus we get the unadvisably long arcs as prescribed.
		if (gc.radius < 0) { 
			h_x2_div_d = -h_x2_div_d; 
		}        
        
		// Complete the operation by calculating the actual center of the arc
		gc.offset[gc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
		gc.offset[gc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
	} 
    
    /*	This segment sets up an clockwise or counterclockwise arc from the current
		position to the target position around the center designated by the offset
		vector. All theta-values measured in radians of deviance from the positive 
		y-axis. 

                        | <- theta == 0
                      * * *
                    *       *
                  *           *
                  *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
                  *   /
                    C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))

	*/

	// calculate the theta (angle) of the current point
	double theta_start = theta(-gc.offset[gc.plane_axis_0], -gc.offset[gc.plane_axis_1]);
		
	// calculate the theta (angle) of the target point
	double theta_end = theta(gc.target[gc.plane_axis_0] - gc.offset[gc.plane_axis_0] - gc.position[gc.plane_axis_0], 
 		gc.target[gc.plane_axis_1] - gc.offset[gc.plane_axis_1] - gc.position[gc.plane_axis_1]);

	// ensure that the difference is positive so that we have clockwise travel
	if (theta_end < theta_start) {
		theta_end += 2*M_PI;
	}
	double angular_travel = theta_end-theta_start;

	// Invert angular motion if the g-code wanted a counterclockwise arc
	if (gc.motion_mode == MOTION_MODE_CCW_ARC) {
		angular_travel = angular_travel-2*M_PI;
	}

	// Find the radius
	double radius = hypot(gc.offset[gc.plane_axis_0], gc.offset[gc.plane_axis_1]);

	// Calculate the motion along the depth axis of the helix
	double depth = gc.target[gc.plane_axis_2]-gc.position[gc.plane_axis_2];

	// Trace the arc
	mc_arc(theta_start, angular_travel, radius, depth, gc.plane_axis_0, gc.plane_axis_1, gc.plane_axis_2, 
        (gc.inverse_feed_rate_mode) ? gc.inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);

      // Finish off with a line to make sure we arrive exactly where we think we are
	mc_line(gc.target[X_AXIS], gc.target[Y_AXIS], gc.target[Z_AXIS], 
		(gc.inverse_feed_rate_mode) ? gc.inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode);

	return (GCSTATUS_OK);
}



void gc_print_status (uint8_t status_code)
{
	switch(status_code) {
		case GCSTATUS_OK:
			printPgmString(PSTR("Executing "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;
		
		case GCSTATUS_BAD_NUMBER_FORMAT:
			printPgmString(PSTR("Bad Number Format "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_EXPECTED_COMMAND_LETTER:
			printPgmString(PSTR("Expected Command Letter "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_UNSUPPORTED_STATEMENT:
			printPgmString(PSTR("Unsupported Statement "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_MOTION_CONTROL_ERROR:
			printPgmString(PSTR("Motion Control Error "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;

		case GCSTATUS_FLOATING_POINT_ERROR:
			printPgmString(PSTR("FLoating Point Error "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			break;
	}
	return;
}

