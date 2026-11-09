/*
 * canonical_machine.h - rs274/ngc canonical machining functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart Jr.
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 */
/* This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* Shared modal snapshot fields. Intentionally no include guard: included inside
 * GCodeQueuedState_t and GCodeState_t so their byte-copy prefix cannot drift.
 * This fragment is not a standalone header.
 */
	uint32_t linenum;					// Gcode block line number
	float target[AXES]; 				// XYZABC where the move should go
	float work_offset[AXES];			// offset from the work coordinate system (for reporting only)
    float feed_rate;                    // F - normalized to millimeters/minute or in inverse time mode

#ifdef __BITFIELDS
    // keep all bitfields together
    uint8_t :0;  // alignment
	cmMotionMode motion_mode             : 3 ;  // Group1: G0, G1, G2, G3, G38.2, G80 - G89
    cmFeedRateMode feed_rate_mode        : 2 ;  // See cmFeedRateMode for settings
    cmCanonicalPlane select_plane        : 2 ;  // G17,G18,G19 - values to set plane to
    cmUnitsMode units_mode               : 2 ;  // G20,G21 - 0=inches (G20), 1 = mm (G21)
    cmPathControl path_control           : 2 ;  // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    cmDistanceMode distance_mode         : 1 ;  // G90=use absolute coords, G91=incremental movement
    cmDistanceMode arc_distance_mode     : 1 ;  // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    cmAbsoluteOverride absolute_override : 1 ;  // G53 TRUE = move using machine coordinates - this block only
    cmCoordSystem coord_system           : 3 ;  // G54-G59 - select coordinate system 1-6
    uint8_t tool                         : 5 ;  // M6 tool change - moves "tool_select" to "tool"
    uint8_t tool_select                  : 5 ;  // T value - T sets this value

	uint8_t spindle_mode                 : 2 ;  // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	uint8_t mist_coolant                 : 1 ;  // TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant                : 1 ;  // TRUE = flood on (M8), FALSE = off (M9)
#else 
    // keep all bitfields together
//    uint8_t :0;  // alignment
	uint8_t motion_mode;				// Group1: G0, G1, G2, G3, G38.2, G80 - G89
    cmFeedRateMode feed_rate_mode;      // See cmFeedRateMode for settings
    cmCanonicalPlane select_plane;      // G17,G18,G19 - values to set plane to
    cmUnitsMode units_mode;             // G20,G21 - 0=inches (G20), 1 = mm (G21)
    cmPathControl path_control;         // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    cmDistanceMode distance_mode;       // G90=use absolute coords, G91=incremental movement
    cmDistanceMode arc_distance_mode;   // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    cmAbsoluteOverride absolute_override; // G53 TRUE = move using machine coordinates - this block only
    cmCoordSystem coord_system;         // G54-G59 - select coordinate system 1-6
    uint8_t tool;                       // M6 tool change - moves "tool_select" to "tool"
    uint8_t tool_select;                // T value - T sets this value

    uint8_t spindle_mode;               // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
    uint8_t mist_coolant;               // TRUE = mist on (M7), FALSE = off (M9)
    uint8_t flood_coolant;              // TRUE = flood on (M8), FALSE = off (M9)
#endif

	float spindle_speed;				// in RPM

