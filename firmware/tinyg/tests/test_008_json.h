/* 
 * test_008_json.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_json[] PROGMEM= "\
{\"gc\":\"g00g17g21g40g49g80g90\"}\n\
{\"gc\":\"g55\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"f500\"}\n\
{\"gc\":\"(MSGsquares)\"}\n\
{\"gc\":\"g0x20\"}\n\
{\"gc\":\"y20\"}\n\
{\"gc\":\"x0\"}\n\
{\"gc\":\"y0\"}\n\
{\"gc\":\"g1x10\"}\n\
{\"gc\":\"y10\"}\n\
{\"gc\":\"x0\"}\n\
{\"gc\":\"y0\"}\n\
{\"gc\":\"(MSGcircles)\"}\n\
{\"gc\":\"g2x10y-10i10\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"g3x10y-10i10\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"g2x20y0i10\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"g3x20y0i10\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"g2x0y0i10\"}\n\
{\"gc\":\"g3x0y0i10\"}\n\
{\"gc\":\"g54\"}\n\
{\"gc\":\"g0x0y0\"}\n\
{\"gc\":\"m30\"}";
