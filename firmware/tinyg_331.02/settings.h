/*
 * settings.h - default runtime settings
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
/*	The values in this file are the default settings that are loaded into 
 *	a virgin EEPROM, and can be changed using the config commands.
 *	After initial load the EEPROM values (or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change 
 *	are in system.h  Application settings that also shouldn't need 
 *	to be changed are in tinyg.h
 */

#ifndef settings_h
#define settings_h


/**** GENERAL SETTINGS ******************************************************/

/*** System setup and operation ***/

#define ENABLE_ACCELERATION 1			// 1 to enable

#define STATUS_REPORT_ENABLE 		1		// 0=off, 1=on
#define STATUS_REPORT_INTERVAL_MS	250		// ms
#define STATUS_REPORT_MIN_MS		100		// ms - enforce a viable minimum
#define STATUS_REPORT_MAX_MS		2500

/*** Gcode power-on defaults ***/

#define GCODE_DEFAULT_UNITS			MILLIMETER_MODE
#define GCODE_DEFAULT_COORD_SYSTEM	G54
#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

/*** Communications defaults ***/

#define COM_APPEND_TX_CR		FALSE
#define COM_IGNORE_RX_CR		FALSE
#define COM_IGNORE_RX_LF		FALSE
#define COM_ENABLE_XON			TRUE
#define COM_ENABLE_ECHO			TRUE

/**** MACHINE PROFILES ******************************************************/
// default machine profiles - chose only one:

#include "settings/settings_zen7x12.h"			// Zen Toolworks 7x12
//#include "settings/settings_probotixV90.h"		// Probotix FireballV90
//#include "settings/settings_lumenlabMicRoV3.h"	// Lumenlabs micRo v3
//#include "settings/settings_DIYLILCNC.h"			// Zen Toolworks 7x12

#endif
