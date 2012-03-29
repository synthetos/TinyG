/* gcode_boxes_400mm.h - data file containing gcode
 */
const char PROGMEM gcode_file[] = "\
T1M6\n\
G17\n\
G21 (mm)\n\
G92X0Y0Z-1.000\n\
G0Z4.000\n\
F400.0\n\
G1Z-1.000\n\
G1Y0.876\n\
X0.011\n\
Y12.327\n\
X-5.560\n\
G0Z4.000\n\
G0Y0.876\n\
G1Z-1.000\n\
G1Y-10.575\n\
X0.011\n\
Y0.876\n\
X-4.560\n\
G0Z4.000\n\
G0X0.011Y12.327\n\
G1Z-1.000\n\
G1X3.084Y15.011\n\
G0Z4.000\n\
G0X0.011Y0.876\n\
G1Z-1.000\n\
G1X3.084Y6.213\n\
Y15.011\n\
X1.286\n\
X-5.560Y12.327\n\
G0Z4.000\n\
G0X0.011Y-10.575\n\
G1Z-1.000\n\
G1X3.084Y-2.585\n\
Y6.213\n\
X0.011Y0.876\n\
G0Z4.000\n\
G0X0.000Y0.000\n\
M30";



