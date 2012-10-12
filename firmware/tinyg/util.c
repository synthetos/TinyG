/*
 * util.c - a random assortment of useful functions
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/* util.c/.h contains a dog's breakfast of supporting functions that are 
 * not specific to tinyg: including:
 *
 *	  - math and min/max utilities and extensions 
 *	  - vector manipulation utilities
 *	  - support for debugging routines
 */  
#include <ctype.h>
#include <stdio.h>				// precursor for xio.h
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"

/**** Vector functions ****
 * copy_vector()			- copy vector of arbitrary length
 * copy_axis_vector()		- copy an axis vector
 * set_unit_vector()		- populate a unit vector by pos. & target
 * get_axis_vector_length()	- return the length of an axis vector
 * set_vector()				- load values into vector form
 * set_vector_by_axis()		- load a single value into a zero vector
 */

void copy_vector(double dest[], const double src[], uint8_t length) 
{
	for (uint8_t i=0; i<length; i++) {
		dest[i] = src[i];
	}
}

void copy_axis_vector(double dest[], const double src[]) 
{
	memcpy(dest, src, sizeof(double)*AXES);
}

double get_axis_vector_length(const double a[], const double b[]) 
{
	return (sqrt(square(a[X] - b[X]) +
				 square(a[Y] - b[Y]) +
				 square(a[Z] - b[Z]) +
				 square(a[A] - b[A]) +
				 square(a[B] - b[B]) +
				 square(a[C] - b[C])));
}

void set_unit_vector(double unit[], double target[], double position[])
{
	double recip_length = 1/get_axis_vector_length(target, position);
	unit[X] = (target[X] - position[X]) * recip_length;	// the compiler would
	unit[Y] = (target[Y] - position[Y]) * recip_length; // probably do this
	unit[Z] = (target[Z] - position[Z]) * recip_length; // for me but what
	unit[A] = (target[A] - position[A]) * recip_length; // the hey
	unit[B] = (target[B] - position[B]) * recip_length;
	unit[C] = (target[C] - position[C]) * recip_length;
}

double *set_vector(double x, double y, double z, double a, double b, double c)
{
	vector[X] = x;
	vector[Y] = y;
	vector[Z] = z;
	vector[A] = a;
	vector[B] = b;
	vector[C] = c;
	return (vector);
}

double *set_vector_by_axis(double value, uint8_t axis)
{
	clear_vector(vector);
	switch (axis) {
		case (X): vector[X] = value; break;
		case (Y): vector[Y] = value; break;
		case (Z): vector[Z] = value; break;
		case (A): vector[A] = value; break;
		case (B): vector[B] = value; break;
		case (C): vector[C] = value;
	}
	return (vector);
}

/**** Math and other general purpose functions ****/

/*
 * isnumber() - isdigit that also accepts plus, minus, and decimal point
 */

uint8_t isnumber(char c)
{
	if (c == '.') { return (true); }
	if (c == '-') { return (true); }
	if (c == '+') { return (true); }
	return (isdigit(c));
}

/* 
 * read_double() - read a double from a normalized char array
 *
 *	buf			normalized char array (line)
 *	i			char array index must point to start of number
 *	double_ptr	pointer to double to write value into
 *
 *	The line is normalized when it is all caps, has no white space,
 *	no non-alphnumeric characters, and no newline or CR.
 */

uint8_t read_double(char *buf, uint8_t *i, double *double_ptr) 
{
	char *start = buf + *i;
	char *end;
  
	*double_ptr = strtod(start, &end);
	if(end == start) { 
		return(false); 
	}
	*i = (uint8_t)(end - buf);
	return(true);
}

/* 
 * calculate_hash() - calculate 32-bit hash code for a string
 * 
 * This is the Java hashCode function. See http://en.wikipedia.org/wiki/Java_hashCode()
 */
uint32_t calculate_hash(char const *string) 
{	
	uint32_t h = 0;
    uint16_t len = strlen(string);

    for (uint16_t i=0; i<len; i++) {
		h = 31 * h + string[i];
    }
    return h;
}


