/*
 * kinematics.h - inverse kinematics routines
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
/*
 * At some point this whole thing ought to be renamed as the line buffer,
 * segment buffer, motor buffer, motor command buffer, joint buffer, or 
 * something that's more descriptive and not in conflict with the 
 * upper-level move buffer used by the planner.
 */

#ifndef kinematics_h
#define kinematics_h 

/*
 * Global Scope Functions
 */

uint8_t ik_kinematics(double travel[], double steps[], double microseconds);

//#ifdef __UNIT_TESTS
//void ik_unit_tests(void);
//#endif

#endif

