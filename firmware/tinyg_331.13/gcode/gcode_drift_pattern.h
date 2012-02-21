/* Pattern for diagnosing axis drift issue
 *
 * Set $1pw0 to diagnose X axis drift (assumes motor 1 is mapped to X asis)
 */
const char PROGMEM gcode_file[] = "\
$1pw0\n\
N1 G21 G90 G40\n\
N2 G1 F400 X10\n\
N3 G1 Y10\n\
N4 G1 X0\n\
N5 G1 Y0\n\
N6 M5\n\
N7 M30";
