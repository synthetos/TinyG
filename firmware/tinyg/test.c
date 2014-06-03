/*
 * test.c - tinyg test sets
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg.h"			// #1
#include "config.h"			// #2
#include "controller.h"
#include "planner.h"
#include "test.h"
#include "util.h"
#include "xio.h"

// regression test files
#ifdef __CANNED_TESTS

#include "tests/test_001_smoke.h" 			// basic functionality
#include "tests/test_002_homing.h"			// G28.1 homing cycles
#include "tests/test_003_squares.h"			// square moves
#include "tests/test_004_arcs.h"			// arc moves
#include "tests/test_005_dwell.h"			// dwells embedded in move sequences
#include "tests/test_006_feedhold.h"		// feedhold - requires manual ! and ~ entry
#include "tests/test_007_Mcodes.h"			// M codes synchronized w/moves (planner queue)
#include "tests/test_008_json.h"			// JSON parser and IO
#include "tests/test_009_inverse_time.h"	// inverse time mode
#include "tests/test_010_rotary.h"			// ABC axes
#include "tests/test_011_small_moves.h"		// small move test
#include "tests/test_012_slow_moves.h"		// slow move test
#include "tests/test_013_coordinate_offsets.h"	// what it says
#include "tests/test_014_microsteps.h"		// test all microstep settings
#include "tests/test_050_mudflap.h"			// mudflap test - entire drawing
#include "tests/test_051_braid.h"			// braid test - partial drawing

#endif

#ifdef __TEST_99
#include "tests/test_099.h"					// diagnostic test file. used to diagnose specific issues
#endif

/*
 * run_test() - system tests from FLASH invoked by $test=n command
 *
 * 	By convention the character array containing the test must have the same 
 *	name as the file name.
 */
uint8_t run_test(cmdObj_t *cmd)
{
	switch ((uint8_t)cmd->value) {
		case 0: { return (STAT_OK);}
#ifdef __CANNED_TESTS

		case 1: { xio_open(XIO_DEV_PGM, PGMFILE(&test_smoke),PGM_FLAGS); break;}
		case 2: { xio_open(XIO_DEV_PGM, PGMFILE(&test_homing),PGM_FLAGS); break;}
		case 3: { xio_open(XIO_DEV_PGM, PGMFILE(&test_squares),PGM_FLAGS); break;}
		case 4: { xio_open(XIO_DEV_PGM, PGMFILE(&test_arcs),PGM_FLAGS); break;}
		case 5: { xio_open(XIO_DEV_PGM, PGMFILE(&test_dwell),PGM_FLAGS); break;}
		case 6: { xio_open(XIO_DEV_PGM, PGMFILE(&test_feedhold),PGM_FLAGS); break;}
		case 7: { xio_open(XIO_DEV_PGM, PGMFILE(&test_Mcodes),PGM_FLAGS); break;}
		case 8: { xio_open(XIO_DEV_PGM, PGMFILE(&test_json),PGM_FLAGS); break;}
		case 9: { xio_open(XIO_DEV_PGM, PGMFILE(&test_inverse_time),PGM_FLAGS); break;}
		case 10: { xio_open(XIO_DEV_PGM, PGMFILE(&test_rotary),PGM_FLAGS); break;}
		case 11: { xio_open(XIO_DEV_PGM, PGMFILE(&test_small_moves),PGM_FLAGS); break;}
		case 12: { xio_open(XIO_DEV_PGM, PGMFILE(&test_slow_moves),PGM_FLAGS); break;}
		case 13: { xio_open(XIO_DEV_PGM, PGMFILE(&test_coordinate_offsets),PGM_FLAGS); break;}
		case 14: { xio_open(XIO_DEV_PGM, PGMFILE(&test_microsteps),PGM_FLAGS); break;}
		case 50: { xio_open(XIO_DEV_PGM, PGMFILE(&test_mudflap),PGM_FLAGS); break;}
		case 51: { xio_open(XIO_DEV_PGM, PGMFILE(&test_braid),PGM_FLAGS); break;}
#endif
#ifdef __TEST_99
		case 99: { xio_open(XIO_DEV_PGM, PGMFILE(&test_99),PGM_FLAGS); break;}
#endif
		default: {
			fprintf_P(stderr,PSTR("Test #%d not found\n"),(uint8_t)cmd->value);
			return (STAT_ERROR);
		}
	}
	tg_set_primary_source(XIO_DEV_PGM);
	return (STAT_OK);
}

/*
 * run_canned_startup() - run a string on startup
 *
 *	Pre-load the USB RX (input) buffer with some test strings that will be called 
 *	on startup. Be mindful of the char limit on the read buffer (RX_BUFFER_SIZE).
 *	It's best to create a test file for really complicated things.
 */
void run_canned_startup()	// uncomment in tinyg.h if you want to run this
{
#ifdef __CANNED_STARTUP

/* Run test 99 */
//	xio_queue_RX_string_usb("$test=99\n");		// run test file
//	xio_queue_RX_string_usb("{\"test\":99}\n");	// run test file
//	xio_queue_RX_string_usb("{test:99}\n");		// run test file

	xio_queue_RX_string_usb("g20\n");
	xio_queue_RX_string_usb("{x:n}\n");

/* Some useful sequences */
//	xio_queue_RX_string_usb("g28.2z0\n");
//	xio_queue_RX_string_usb("{\"jogx\":1}\n");
//	xio_queue_RX_string_usb("{\"uda\":{\"0\":\"0x4\"}}\n");
//	xio_queue_RX_string_usb("T4\n");
//	xio_queue_RX_string_usb("M6\n");
//	xio_queue_RX_string_usb("M6T4\n");
//	xio_queue_RX_string_usb("$tool\n");
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("\n\n");			// 2 null lines
//	xio_queue_RX_string_usb("%\n");				// opening percent character
//	xio_queue_RX_string_usb("$\n");				// display general group
//	xio_queue_RX_string_usb("?\n");				// report

/*	Test signals - Note: requires test chars to be enabled */
//	xio_queue_RX_string_usb("^\n");				// reset 
//	xio_queue_RX_string_usb("!\n");				// feedhold
//	xio_queue_RX_string_usb("~\n");				// cycle start

/* Configs and controls */
//	xio_queue_RX_string_usb("$\n");				// print general group
//	xio_queue_RX_string_usb("$x\n");			// print x axis
//	xio_queue_RX_string_usb("$1\n");			// print motor #1 group
//	xio_queue_RX_string_usb("$m\n");			// print all motor groups 
//	xio_queue_RX_string_usb("$n\n");			// print all axis groups
//	xio_queue_RX_string_usb("$o\n");			// print offset groups
//	xio_queue_RX_string_usb("$$\n");			// print everything
//	xio_queue_RX_string_usb("$xam\n");			// print x axis mode
//	xio_queue_RX_string_usb("$sys\n");			// print system settings
//	xio_queue_RX_string_usb("$unit\n");
//	xio_queue_RX_string_usb("$sr\n");

//	xio_queue_RX_string_usb("$xfr=1000\n");
//	xio_queue_RX_string_usb("$2mi=4\n");
//	xio_queue_RX_string_usb("$xjm 1000000\n");
//	xio_queue_RX_string_usb("$xvm\n");			// config with no data
//	xio_queue_RX_string_usb("$ja\n");			// config with no data
//	xio_queue_RX_string_usb("$aam = 3\n");		// set A to radius mode
//	xio_queue_RX_string_usb("$aam 10\n");		// set A to SLAVE_XYZ mode
//	xio_queue_RX_string_usb("(MSGtest message in comment)\n");
	xio_queue_RX_string_usb("{\"1tr\":1.23456}\n");

/* G0's */
//	xio_queue_RX_string_usb("g0 x0.0004\n");	// too short line
//	xio_queue_RX_string_usb("g0 x0.04\n");		// very short line
//	xio_queue_RX_string_usb("g0 x0.08\n");
//	xio_queue_RX_string_usb("g0 x0.12\n");
//	xio_queue_RX_string_usb("g0 x20\n");		// medium line
//	xio_queue_RX_string_usb("g0 x2000\n");		// exceed soft limits

//	xio_queue_RX_string_usb("g0 x0.04 y0.2\n");		// very short line
//	xio_queue_RX_string_usb("g0 x0.4 y0.12\n");		// very short line
//	xio_queue_RX_string_usb("g0 x0.1 y0.02\n");		// very short line
//	xio_queue_RX_string_usb("g0 x0.0 y-0.2\n");		// very short line

//	xio_queue_RX_string_usb("g0 z2\n");			// Z short line
//	xio_queue_RX_string_usb("g0 z-2\n");		// Z short line
//	xio_queue_RX_string_usb("g0 z20\n");		// Z medium line

//	xio_queue_RX_string_usb("g0 x0.2\n");		// shortest drawable line
//	xio_queue_RX_string_usb("g0 x0\n");
//	xio_queue_RX_string_usb("g0 x2\n");
//	xio_queue_RX_string_usb("g0 x3\n");
//	xio_queue_RX_string_usb("g0 y3\n");
//	xio_queue_RX_string_usb("g0 x10\n");
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z10 a10\n");
//	xio_queue_RX_string_usb("g0 x2000 y3000 z4000 a5000\n");

/* G1's */
//	xio_queue_RX_string_usb("g1 f300 x100\n");
//	xio_queue_RX_string_usb("g1 f10 x100\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f0 x10\n");

/* G2/G3's */
//	xio_queue_RX_string_usb("g3 f500 x100 y100 z25 i50 j50\n");
//	xio_queue_RX_string_usb("g2 x0 y0 i30 j30 f2000\n");
//	xio_queue_RX_string_usb("g2 f2000 x50 y50 z2 i25 j25\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i8 j8\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i5 j5\n");
//	xio_queue_RX_string_usb("g2 f300 x3 y3 i1.5 j1.5\n");

//	xio_queue_RX_string_usb("g17 g3 f3000 x3.0 y0.0 z0.5 i1.5 j6.0 k0.0\n"); // 13 segment XY arc
//	xio_queue_RX_string_usb("g18 g3 f3000 x3.0 y0.5 z0.0 i1.5.j0 0 k6.0\n"); // 13 segment XZ arc
//	xio_queue_RX_string_usb("g19 g3 f3000 x0.5 y3.0 z0.0 i0.0 j1.5 k6.0\n"); // 13 segment YZ arc

//	xio_queue_RX_string_usb("g2 f300 x3 y3 z1.2 i1.5 j1.5 k0.5\n");	// 51 segment arc
//	xio_queue_RX_string_usb("g2 f300 x3 y0 z0.5 i1.5 j6\n");		// 161 segment arc
//	xio_queue_RX_string_usb("g2 f300 x1 y0.5 z0.25 i0.5 j0.5\n");	// __ segment arc

//	xio_queue_RX_string_usb("g2 f300 i10 j10\n");				// G2 pocket arc
//	xio_queue_RX_string_usb("f400\n");							// set feed rate while in arc motion mode
//	xio_queue_RX_string_usb("g3 f300 i10 j10\n");				// G3 pocket arc

/* G4 tests (dwells) */
//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p0.1\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z-10\n");

/* G10 coordinate offsets */
//	xio_queue_RX_string_usb("g10 L2 p2 x10 y11 z12\n");

/* G20 / G21 units */
//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("g21\n");

/* G28 and G30 homing tests */
//	xio_queue_RX_string_usb("g28.2x0\n");
//	xio_queue_RX_string_usb("g28.1\n");			// G28.1 OK
//	xio_queue_RX_string_usb("g28.1x10y10\n");	// G28.1 specification error
//	xio_queue_RX_string_usb("g28.2x0y0z0\n");
//	xio_queue_RX_string_usb("g28.2y0\n");
//	xio_queue_RX_string_usb("g28.2x0y0z0a0\n");
//	xio_queue_RX_string_usb("g28.2 z0\n");
//	xio_queue_RX_string_usb("g30x0y0z0\n");
//	xio_queue_RX_string_usb("g30x42\n");

/* G48-G51 override tests */
//	xio_queue_RX_string_usb("m50 P1\n");		// enable feed override

/* G53 tests */
//	xio_queue_RX_string_usb("g56\n");			// assumes G55 is different than machine coords
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// move to zero in G55
//	xio_queue_RX_string_usb("g53 g0 x0 y0\n");	// should move off G55 zero back to machine zero

/* G54-G59 tests */
//	xio_queue_RX_string_usb("g54\n");
//	xio_queue_RX_string_usb("g55\n");
//	xio_queue_RX_string_usb("g10 p2 l2 x10 y10 z-10\n");

/* G92 tests */
//	xio_queue_RX_string_usb("g92 x20 y20\n");	// apply offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move diagonally to SouthWest
//	xio_queue_RX_string_usb("g92.1\n");			// cancel offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move NW back to original coordinates
//	xio_queue_RX_string_usb("g92.2\n");
//	xio_queue_RX_string_usb("g92.3\n");

/* Other Gcode tests */
//	xio_queue_RX_string_usb("g20\n");			// inch mode
//	xio_queue_RX_string_usb("g21\n");			// mm mode
//	xio_queue_RX_string_usb("g18\n");			// plane select
//	xio_queue_RX_string_usb("g10 l2 p4 x20 y20 z-10\n"); // test G10

/* M code tests */
//	xio_queue_RX_string_usb("m3\n");			// spindle CW
//	xio_queue_RX_string_usb("m4\n");			// spindle CCW
//	xio_queue_RX_string_usb("m5\n");			// spindle off
//	xio_queue_RX_string_usb("m7\n");			// mist coolant on
//	xio_queue_RX_string_usb("m8\n");			// flood coolant on
//	xio_queue_RX_string_usb("m9\n");			// all coolant off

/* Feedhold tests */
// Consider dropping PLANNER_BUFFER_POOL_SIZE down to something like 4 for these tests
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x1 y1 z1\n");

//	xio_queue_RX_string_usb("g0 x0.1\n");
//	xio_queue_RX_string_usb("g0 x0.2\n");
//	xio_queue_RX_string_usb("g0 x0.3\n");
//	xio_queue_RX_string_usb("g0 x0.4\n");
//	xio_queue_RX_string_usb("g0 x0.5\n");
//	xio_queue_RX_string_usb("@\n");		// issue feedhold - uncomment __ENABLE_DEBUG_CHARS
//	xio_queue_RX_string_usb("#\n");		// end feedhold - uncomment __ENABLE_DEBUG_CHARS

//	xio_queue_RX_string_usb("!");				// issue feedhold
//	xio_queue_RX_string_usb("~");				// end feedhold
//	xio_queue_RX_string_usb("g0 x0 y0 z0\n");
//	xio_queue_RX_string_usb("g0 x50\n");
//	xio_queue_RX_string_usb("g0 y5\n");
//	See 331.19 or earlier for some more lengthy feedhold tests

/* JSON TEST CASES */
// If you want to run multi-line cases you need to set RX buffer to 1024 in xio_usart.h

// JSON parser tests		  // set a group
//	xio_queue_RX_string_usb("{\"x\":{\"am\":2,\"vm\":601.000,\"fr\":1201.000,\"tm\":476.000,\"jm\":20000001.000,\"jd\":0.051,\"sn\":2,\"sv\":-502.000,\"lv\":101.000,\"lb\":2.001,\"zb\":1.001}}\n");
//	xio_queue_RX_string_usb("{\"x\":\"\"}\n"); // retrieve a group

//	xio_queue_RX_string_usb("{\"gc\":\"g0 x3 y4 z5.5 (comment line)\"}\n");
//	xio_queue_RX_string_usb("{\"xfr\":1200}\n");
//	xio_queue_RX_string_usb("{\"xfr\":1200, \"yfr\":1201, \"zfr\":600}\n");
//	xio_queue_RX_string_usb("{\"err_1\":36000}\n");
//	xio_queue_RX_string_usb("{\"1sa\":3.6.000}\n");
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");				// invoke a status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":true,\"posx\":true,\"posy\":true}}\n");	// set status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":null,\"posx\":null,\"posy\":null}}\n");	// set status report
//	xio_queue_RX_string_usb("{\"x\":{\"am\":2,\"vm\":601.000,\"fr\":1201.000,\"tm\":476.000,\"jm\":20000001.000,\"jd\":0.051,\"sm\":2,\"sv\":-502.000,\"lv\":101.000,\"lb\":2.001,\"zb\":1.001}}\n");

//	xio_queue_RX_string_usb("{\"x\":\"\"}\n");				// x axis group display
//	xio_queue_RX_string_usb("{\"c\":\"\"}\n");				// c axis group display
//	xio_queue_RX_string_usb("{\"1\":\"\"}\n");				// motor 1 group display
//	xio_queue_RX_string_usb("{\"sys\":\"\"}\n");			// system group display
//	xio_queue_RX_string_usb("{\"x\":null}\n");				// group display
//	xio_queue_RX_string_usb("{\"x\":{\"am\":1,\"fr\":800.000,\"vm\":800.000,\"tm\":100.000,\"jm\":100000000.000,\"jd\":0.050,\"sm\":1,\"sv\":800.000,\"lv\":100.000,\"zo\":3.000,\"abs\":0.000,\"pos\":0.000}}\n");
//	xio_queue_RX_string_usb("{\"sys\":{\"fv\":0.930,\"fb\":330.390,\"si\":250,\"gpl\":0,\"gun\":1,\"gco\":1,\"gpa\":2,\"gdi\":0,\"ea\":1,\"ja\":200000.000,\"ml\":0.080,\"ma\":0.100,\"mt\":10000.000,\"ic\":0,\"il\":0,\"ec\":0,\"ee\":1,\"ex\":1}}\n");

//	xio_queue_RX_string_usb("{\"  xfr  \":null}\n");		// JSON string normalization tests
//	xio_queue_RX_string_usb("{\"gcode\":\"G1 x100 (Title Case Comment)   \"}\n");
//	xio_queue_RX_string_usb("{\"sr\":{\"ln\":true,\"vl\":true,\"ms\":true}}\n");  // set status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":true,\"posx\":true,\"stat\":true}}\n"); // set status report
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");				// get status report
//	xio_queue_RX_string_usb("g0 x10\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0 x2\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g1 f243.543 x22.3456 y32.2134 z-0.127645\"}\n");

//	xio_queue_RX_string_usb("{\"gc\":\"n10000 g0 x20\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n100000 g0 x0\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n1000000 g0 x20\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n10000000 g0 x0\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n100000000 g0 x20\"}\n");

#endif // __CANNED_STARTUP
}

