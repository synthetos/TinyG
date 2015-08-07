/*
 * test_001_smoke.h
 *
 * This test checks basic functionality:
 *	- motor 1 CW at full speed ~3 seconds
 *	- motor 1 CCW at full speed ~3 seconds
 *	- motor 2 CW at full speed ~3 seconds
 *	- motor 2 CCW at full speed ~3 seconds
 *	- motor 3 CW at full speed ~3 seconds
 *	- motor 3 CCW at full speed ~3 seconds
 *	- motor 4 CW at full speed ~3 seconds
 *	- motor 4 CCW at full speed ~3 seconds
 *	- all motors CW at full speed ~3 seconds
 *	- all motors CCW at full speed ~3 seconds
 *	- all motors CW at medium speed ~3 seconds
 *	- all motors CCW at medium speed ~3 seconds
 *	- all motors CW at slow speed ~3 seconds
 *	- all motors CCW at slow speed ~3 seconds
 *	- light LEDs 1,2 and 4 in sequence for about 1 second each:
 *	- short finishing move
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
/*
const char test_smoke[] PROGMEM = "\
(MSG**** Smoke Test [v1] ****)\n\
G00 G17 G21 G40 G49 G80 G90\n\
m3g4p1\n\
m5g4p1\n\
m4g4p1\n\
m3g4p1\n\
m5g4p1\n\
m7g4p1\n\
m9g4p1\n\
m8g4p1\n\
m9\n\
g0x0y0z0\n\
g00 x20\n\
x0\n\
y20\n\
y0\n\
z20\n\
z0\n\
a20\n\
a0\n\
G00 x20 y20 z20 a20\n\
G00 x0 y0 z0 a0\n\
G01 f30 x2 y2 z2 a2\n\
x0 y0 z0 a0\n\
g0x1\n\
g0x0\n\
m2";
*/
/*
G01 f200 x10 y10 z10 a10\n\
x0 y0 z0 a0\n\
*/
/*
G02 f10000 x0 y0 z40 i27 j27\n\
G03 f10000 x0 y0 z0 i27 j27\n\
g0x0y0z0\n\
*/

const char test_smoke[] PROGMEM = "\
(MSG**** Smoke Test - Allow at least 1 inch clearance in all directions ****)\n\
n10 g00 g17 g21 g40 g49 g80 g90 g91.1\n\
n20 m3g4p1 (msgTest outputs)\n\
n30 m5g4p1\n\
n40 m4g4p1\n\
n50 m3g4p1\n\
n60 m5g4p1\n\
n70 m7g4p1\n\
n80 m9g4p1\n\
n90 m8g4p1\n\
n100 m9\n\
n110 g0x0y0z0 (msgStraight lines)\n\
n120 g00 x20\n\
n130 x0\n\
n140 y20\n\
n150 y0\n\
n160 z20\n\
n170 z0\n\
n180 a20\n\
n190 a0\n\
n200 g00 x20 y20 z20 a20\n\
n210 g00 x0 y0 z0 a0\n\
n220 g01 f30 x2 y2 z2 a2\n\
n230 x0 y0 z0 a0\n\
n240 g0x1\n\
n250 g0x0\n\
n260 g00g17g21g40g49g80g90g91.1\n\
n270 g0x0y0\n\
n280 f500\n\
n290 g2x10y-10i10 (msgCW 270 degrees)\n\
n300 g0x0y0\n\
n310 g3x10y-10i10 (msgCCW 90 degrees)\n\
n320 g0x0y0\n\
n330 g2x20y0i10 (msgCW 180 degrees)\n\
n340 g0x0y0\n\
n350 g3x20y0i10 (msgCCW 180 degrees)\n\
n360 g0x0y0\n\
n370 F1200\n\
n380 g2x0y0i20 (msgCW 360 degrees)\n\
n390 g2x0y0i20 (msgCW 360 again)\n\
n400 f700 (msgChange feed rate while in G2 motion mode)\n\
n410 g2x0y0i20 (msgCW 360 3rd time)\n\
n420 f3000 (msgChange feed rate while in G2 motion mode)\n\
n430 g2x0y0i20 (msgCW 360 4th time)\n\
n440 f500\n\
n450 g3x0y0z20i20 (msgCCW 360 degrees with linear travel)\n\
n460 g3x0y0z0i20 (msgCCW 360 again)\n\
n470 f700 (msgChange feed rate while in G3 motion mode)\n\
n480 g3x0y0i20 (msgCCW 360 3rd time)\n\
n490 g18 g21 g90 g91.1\n\
n500 g0 x10 y0 z0\n\
n510 g2 i-10 f700 (msgG18 CW 360 degrees)\n\
n520 g80\n\
n530 g0x0y0z0\n\
n540 g0x2\n\
n550 g0x0\n\
n999 m30 (msgEND)";
