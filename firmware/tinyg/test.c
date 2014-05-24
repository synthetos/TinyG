/*
 * test.c - tinyg test sets
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart Jr.
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
uint8_t run_test(nvObj_t *cmd)
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
//		case 96: { xio_open(XIO_DEV_PGM, PGMFILE(&test_96),PGM_FLAGS); break;}
//		case 97: { xio_open(XIO_DEV_PGM, PGMFILE(&test_97),PGM_FLAGS); break;}
//		case 98: { xio_open(XIO_DEV_PGM, PGMFILE(&test_98),PGM_FLAGS); break;}
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
 *
 *	Update 3/31/14 - Build 420.02
 *	Deleted a ton of test cases. Left some in as examples. Go back to 420.01 if you want these.
 */
void run_canned_startup()	// uncomment in tinyg.h if you want to run this
{
#ifdef __CANNED_STARTUP

/* Run test 99 */
//	xio_queue_RX_string_usb("$test=99\n");		// run test file (doesn't work if text mode is disabled)
//	xio_queue_RX_string_usb("{\"test\":99}\n");	// run test file
//	xio_queue_RX_string_usb("{test:98}\n");		// run test file
	xio_queue_RX_string_usb("{test:99}\n");		// run test file

/* Configs and controls */
//	xio_queue_RX_string_usb("$\n");				// print general group
//	xio_queue_RX_string_usb("$zam\n");			// print axis mode
//	xio_queue_RX_string_usb("$z\n");			// print an axis
//	xio_queue_RX_string_usb("$xfr=1000\n");
//	xio_queue_RX_string_usb("{\"jogx\":1}\n");
//	xio_queue_RX_string_usb("{\"uda\":{\"0\":\"0x4\"}}\n");
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("\n\n");			// 2 null lines
//	xio_queue_RX_string_usb("(MSGtest message in comment)\n");

/* Gcode tests */
//	xio_queue_RX_string_usb("g0 x0.0004\n");	// too short line
//	xio_queue_RX_string_usb("g0 x0.04 y0.2\n");	// very short line

//	xio_queue_RX_string_usb("g1 f300 x100\n");

//	xio_queue_RX_string_usb("g3 f500 x100 y100 z25 i50 j50\n");
//	xio_queue_RX_string_usb("g2 x0 y0 i30 j30 f2000\n");
//	xio_queue_RX_string_usb("g2 f2000 x50 y50 z2 i25 j25\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i8 j8\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i5 j5\n");
//	xio_queue_RX_string_usb("g2 f300 x3 y3 i1.5 j1.5\n");

//	xio_queue_RX_string_usb("g17 g3 f3000 x3.0 y0.0 z0.5 i1.5 j6.0 k0.0\n"); // 13 segment XY arc
//	xio_queue_RX_string_usb("g18 g3 f3000 x3.0 y0.5 z0.0 i1.5.j0 0 k6.0\n"); // 13 segment XZ arc
//	xio_queue_RX_string_usb("g19 g3 f3000 x0.5 y3.0 z0.0 i0.0 j1.5 k6.0\n"); // 13 segment YZ arc

//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p0.1\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z-10\n");

//	xio_queue_RX_string_usb("g10 L2 p2 x10 y11 z12\n");

//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("g21\n");

//	xio_queue_RX_string_usb("g28.2x0\n");
//	xio_queue_RX_string_usb("g28.1\n");			// G28.1 OK
//	xio_queue_RX_string_usb("g28.1x10y10\n");	// G28.1 specification error
//	xio_queue_RX_string_usb("g28.2x0y0z0\n");
//	xio_queue_RX_string_usb("g28.2y0\n");
//	xio_queue_RX_string_usb("g28.2x0y0z0a0\n");
//	xio_queue_RX_string_usb("g28.2 z0\n");
//	xio_queue_RX_string_usb("g30x0y0z0\n");
//	xio_queue_RX_string_usb("g30x42\n");

//	xio_queue_RX_string_usb("g56\n");			// assumes G55 is different than machine coords
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// move to zero in G55
//	xio_queue_RX_string_usb("g53 g0 x0 y0\n");	// should move off G55 zero back to machine zero
//	xio_queue_RX_string_usb("g54\n");
//	xio_queue_RX_string_usb("g55\n");
//	xio_queue_RX_string_usb("g10 p2 l2 x10 y10 z-10\n");

//	xio_queue_RX_string_usb("g92 x20 y20\n");	// apply offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move diagonally to SouthWest
//	xio_queue_RX_string_usb("g92.1\n");			// cancel offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move NW back to original coordinates
//	xio_queue_RX_string_usb("g92.2\n");
//	xio_queue_RX_string_usb("g92.3\n");

/* M code tests */
//	xio_queue_RX_string_usb("m3\n");			// spindle CW
//	xio_queue_RX_string_usb("m4\n");			// spindle CCW
//	xio_queue_RX_string_usb("m5\n");			// spindle off
//	xio_queue_RX_string_usb("m7\n");			// mist coolant on
//	xio_queue_RX_string_usb("m8\n");			// flood coolant on
//	xio_queue_RX_string_usb("m9\n");			// all coolant off

//	xio_queue_RX_string_usb("T4\n");
//	xio_queue_RX_string_usb("M6\n");
//	xio_queue_RX_string_usb("M6T4\n");

/* JSON TEST CASES */
// If you want to run multi-line cases you need to set RX buffer to 1024 in xio_usart.h

//	xio_queue_RX_string_usb("{\"x\":null}\n");	// X axis group display, strict
//	xio_queue_RX_string_usb("{x:null}\n");		// X axis group display, relaxed
//	xio_queue_RX_string_usb("{x:n}\n");			// X axis group display, relaxed, short
//	xio_queue_RX_string_usb("{c:n}\n");			// C axis group display, relaxed, short

							  // set a group
//	xio_queue_RX_string_usb("{x:{am:2,vm:601.000,fr:1201.000,tn:476.000,tm:476.000,jm:20000001.000,jd:0.051,sn:2,sv:-502.000,lv:101.000,lb:2.001,zb:1.001}}\n");
//	xio_queue_RX_string_usb("{x:n}\n"); // retrieve a group

#endif // __CANNED_STARTUP
}

