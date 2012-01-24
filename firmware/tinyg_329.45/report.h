/*
 * report.h - rs274/ngc status report and other reporting functions
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
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef report_h
#define report_h

void sr_init(void);
void sr_decr_status_report(void);
void sr_force_status_report(void);
uint8_t sr_try_status_report(void);
void sr_run_status_report(void);

void sr_print_machine_state(void);

#endif
