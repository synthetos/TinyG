/*
 * util.c - a random assortment of useful functions
 * Part of Kinen project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <ctype.h>
#include <stdio.h>				// precursor for xio.h
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
//#include <string.h>
//#include <avr/pgmspace.h>		// precursor for xio.h

#include "util.h"

/**** Math and other general purpose functions ****/

double std_dev(double a[], uint8_t n, double *mean) 
{
	if(n == 0) { return (0);}
	double sum = 0;
	double sq_sum = 0;
	for(uint8_t i=0; i<n; ++i) {
		sum += a[i];
		sq_sum += square(a[i]);
	}
	*mean = sum / n;
	double variance = (sq_sum / n) - square(*mean);
	return sqrt(variance);
}

/* Slightly faster (*) multi-value min and max functions
 * 	min3() - return minimum of 3 numbers
 * 	min4() - return minimum of 4 numbers
 * 	max3() - return maximum of 3 numbers
 * 	max4() - return maximum of 4 numbers
 * 	isnumber() - isdigit that also accepts plus, minus, and decimal point
 *
 * Implementation tip: Order the min and max values from most to least likely in the calling args
 *
 * (*) Macro min4 is about 20uSec, inline function version is closer to 10 uSec
 * 	#define min3(a,b,c) (min(min(a,b),c))
 *	#define min4(a,b,c,d) (min(min(a,b),min(c,d)))
 *	#define max3(a,b,c) (max(max(a,b),c))
 *	#define max4(a,b,c,d) (max(max(a,b),max(c,d)))
 */
/*
inline double min3(double x1, double x2, double x3)
{
	double min = x1;
	if (x2 < min) { min = x2;} 
	if (x3 < min) { return (x3);} 
	return (min);
}

inline double min4(double x1, double x2, double x3, double x4)
{
	double min = x1;
	if (x2 < min) { min = x2;} 
	if (x3 < min) { min = x3;} 
	if (x4 < min) { return (x4);}
	return (min);
}

inline double max3(double x1, double x2, double x3)
{
	double max = x1;
	if (x2 > max) { max = x2;} 
	if (x3 > max) { return (x3);} 
	return (max);
}

inline double max4(double x1, double x2, double x3, double x4)
{
	double max = x1;
	if (x2 > max) { max = x2;} 
	if (x3 > max) { max = x3;} 
	if (x4 > max) { return (x4);}
	return (max);
}

uint8_t isnumber(char c)
{
	if (c == '.') { return (true); }
	if (c == '-') { return (true); }
	if (c == '+') { return (true); }
	return (isdigit(c));
}
*/
