/*
 * mathpack.c - supporting math routines
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "tinyg.h"
#include "config.h"
#include "mathpack.h"


/*
 * Local Scope Data and Functions
 */

#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
#define cubert(a) pow((a), 0.33333333333333)
#define radical3 (1.73205080756888)

/*
 * mp_period_solvers
 *
 *	Solves for time as a function of position, velocity, acceleration and jerk
 *	Equations in classic equation-of-motion form:
 *
 *		accel pd1	S = Vi*T + Jm*(T^3)/6
 *		accel pd2	S = Vh*T + Ah*(T^2)/2 - Jm*(T^3)/6
 *		decel pd1	S = Vi*T - Jm*(T^3)/6
 *		decel pd2	S = Vh*T - Ah*(T^2)/2 + Jm*(T^3)/6
 *
 *	Equations in canonical cubic form:
 *
 *		canonical	0 = t3 + bt2 + ct + d
 *		accel pd1	0 = (T^3) + (6V/Jm)*T - 6S/Jm
 *		accel pd2	0 = (T^3) + (-3*Ah/Jm)*(T^2) + (-6*Vh/Jm)*T + (6*S/Jm)
 *		decel pd1	0 = (T^3) + (-6V/Jm)*T + 6S/Jm
 *		decel pd2	0 = (T^3) + (-3*Ah/Jm)*(T^2) + (6*Vh/Jm)*T + (-6*S/Jm)
 *
 *	The selection of roots (x1, x3) is hard-wired for the short distances
 *	usually encountered for trajectory planning segments. ( <1mm ) 
 *	If the equations are used for longer distances care must be taken in 
 *	returning the correct root.
 */

double mp_period_a1_solver(double s, double v, double a, double jm)
{
	double x1, x2, x3;
	mp_cubic_solver(0, (6*v/jm), (-6*s/jm), &x1, &x2, &x3);
	return (x1);
}

double mp_period_a2_solver(double s, double v, double a, double jm)
{
	double x1, x2, x3;		// roots
	mp_cubic_solver((-3*a/jm), (-6*v/jm), (6*s/jm), &x1, &x2, &x3);
	return (x3);
}

double mp_period_d1_solver(double s, double v, double a, double jm)
{
	double x1, x2, x3;
	mp_cubic_solver(0, (-6*v/jm), (6*s/jm), &x1, &x2, &x3);
	return (x3);
}

double mp_period_d2_solver(double s, double v, double a, double jm)
{
	double x1, x2, x3;
	mp_cubic_solver((-3*a/jm), (6*v/jm), (-6*s/jm), &x1, &x2, &x3);
	return (x1);
}

/* 
 * mp_cubic_solver()
 *
 *	Solves a general cubic equation of the form x3 + bx2 + cx + d = 0
 *	From http://www.1728.com/cubic2.htm
 * 	Has rounding error as much as +/- 0.0000005
 *
 *	This code relies on the compiler to optimize the expressions (-Os mode). 
 *	It does this really well - your expression re-arranging only obscures things.
 */

double mp_cubic_solver(double b, double c, double d, double *x1, double *x2, double *x3)
{
	double f, g, h;					// base terms
	double i, j, k, m, n, p;		// intermediate terms

	f = (3*c - square(b))/3;
	g = (2*cube(b) - 9*b*c + 27*d)/27;
	h = square(g)/4 + cube(f)/27;

	if (h==0) {						// 3 real and equal roots
		*x1 = -cubert(d);
//		*x2 = *x1;					// x2 is never used
		*x3 = *x1;
	} else if (h<0) {				// 3 real roots
		i = sqrt(square(g)/4 -h);
		j = cubert(i);
		k = acos(-g/(2*i));
		m = cos(k/3);
		n = radical3 * sin(k/3);
		p = -b/3;
		*x1 = 2*j * cos(k/3) + p;
//		*x2 = -j*(m+n) + p;			// x2 is never used
		*x3 = -j*(m-n) + p;
	} else {						// 1 real root
		m = cubert(sqrt(h) - g/2);
		n = -cubert(sqrt(h) + g/2);
		*x1 = (m+n) - b/3;
	}
	return (*x1);
}


//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

void _mp_test_cubic(void);
void _mp_test_period_solvers(void); 

void mp_unit_tests()
{
//	_mp_test_cubic();
	_mp_test_period_solvers();
}

void _mp_test_cubic() 
{
	double x1, x2, x3;								// roots
													//	x1	x2	x3
	mp_cubic_solver(6, -4, -24, &x1, &x2, &x3);		//	 2	-6	-2	(3 real)
	mp_cubic_solver(-15, 81, -175, &x1, &x2, &x3);	//	 7	 i   i	(1 real)

}

void _mp_test_period_solvers() 
{															// expected result
	mp_period_a1_solver(1.04,  600, 244948.97, 50000000);	// x1 = 0.001666667	(100,000 uS)
	mp_period_a2_solver(1.30,  600, 244948.97, 50000000);	// x3 = 0.001666667	(100,000 uS)
	mp_period_d1_solver(1.96, 1200, 244948.97, 50000000);	// x3 = 0.001666667	(100,000 uS)
	mp_period_d2_solver(0.70,  600, 244948.97, 50000000);	// x1 = 0.001666667	(100,000 uS)

	mp_period_a1_solver(0.05,  0.1, 244948.97, 50000000);	// x1 ~ 0.001811492 (~108,000 uS)
	mp_period_a2_solver(0.10,  600, 244948.97, 50000000);	// x3 = 0.000166667   (10,000 uS)
	mp_period_d1_solver(0.20, 1200, 244948.97, 50000000);	// x3 = 0.000166667	  (10,000 uS)
	mp_period_d2_solver(0.10,  600, 244948.97, 50000000);	// x1 = 0.000166667   (10,000 uS)
}

#endif
