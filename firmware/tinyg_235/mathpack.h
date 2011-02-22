/*
 * mathpack.h - supporting math routines
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

#ifndef mathpack_h
#define mathpack_h 

/*
 * Global Scope Functions
 */

double mp_period_a1_solver(double s, double v, double a, double jm);
double mp_period_a2_solver(double s, double v, double a, double jm);
double mp_period_d1_solver(double s, double v, double a, double jm);
double mp_period_d2_solver(double s, double v, double a, double jm);
double mp_cubic_solver(double b, double c, double d, double *x1, double *x2, double *x3);

void mp_unit_tests(void);

#endif
