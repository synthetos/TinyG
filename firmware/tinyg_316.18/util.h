/*
 * util.h - a random assortment of useful functions
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
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
 */

#ifndef util_h
#define util_h

/*************************************************************************
 * general utility defines
 */

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef max
#define max(a, b) (((a)>(b))?(a):(b))
#endif
#ifndef min
#define min(a, b) (((a)<(b))?(a):(b))
#endif

#ifndef EPSILON
#define EPSILON 0.0001						// rounding error for floats
#endif
#ifndef FLOAT_EQ
#define FLOAT_EQ(a,b) (fabs(a-b) < EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef FLOAT_NE
#define FLOAT_NE(a,b) (fabs(a-b) > EPSILON)	// requires math.h to be included in each file used
#endif

#define MAX_LONG 2147483647
#define MAX_ULONG 4294967295

#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
#define cubert(a) pow((a), 0.33333333333333)
#define radical3 (1.73205080756888)
//for PI use M_PI as defined in math.h

/*
 * Global Scope Functions
 */

uint8_t isnumber(char c);

#endif


