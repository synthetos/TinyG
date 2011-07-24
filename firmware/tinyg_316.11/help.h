/*
 * help.h - collected help and assorted display routines
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

#ifndef help_h
#define help_h

void help_print_test_help(void);
void help_print_config_help(void);
void help_print_gcode_help(void);

void dump_set_f_dda(double f_dda,
					double dda_substeps, 
					double major_axis_steps, 
					double microseconds,
					double f_dda_base);
#endif


