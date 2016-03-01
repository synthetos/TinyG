/*
 * util.c - a random assortment of useful functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
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
/* util contains a dog's breakfast of supporting functions that are not specific to tinyg:
 * including:
 *	  - math and min/max utilities and extensions
 *	  - vector manipulation utilities
 */

#include "tinyg.h"
#include "util.h"

#ifdef __AVR
#include "xmega/xmega_rtc.h"
#endif

bool FLAGS_NONE[AXES] = { false, false, false, false, false, false };
bool FLAGS_ONE[AXES]  = { true, false, false, false, false, false };
bool FLAGS_ALL[AXES]  = { true, true, true, true, true, true };

/**** Vector utilities ****
 * copy_vector()			- copy vector of arbitrary length
 * vector_equal()			- test if vectors are equal
 * get_axis_vector_length()	- return the length of an axis vector
 * set_vector()				- load values into vector form
 * set_vector_by_axis()		- load a single value into a zero vector
 */

float vector[AXES];	// statically allocated global for vector utilities

/*
void copy_vector(float dst[], const float src[])
{
	memcpy(dst, src, sizeof(dst));
}
*/

uint8_t vector_equal(const float a[], const float b[])
{
	if ((fp_EQ(a[AXIS_X], b[AXIS_X])) &&
		(fp_EQ(a[AXIS_Y], b[AXIS_Y])) &&
		(fp_EQ(a[AXIS_Z], b[AXIS_Z])) &&
		(fp_EQ(a[AXIS_A], b[AXIS_A])) &&
		(fp_EQ(a[AXIS_B], b[AXIS_B])) &&
		(fp_EQ(a[AXIS_C], b[AXIS_C]))) {
		return (true);
	}
	return (false);
}

float get_axis_vector_length(const float a[], const float b[])
{
	return (sqrt(square(a[AXIS_X] - b[AXIS_X]) +
				 square(a[AXIS_Y] - b[AXIS_Y]) +
				 square(a[AXIS_Z] - b[AXIS_Z]) +
				 square(a[AXIS_A] - b[AXIS_A]) +
				 square(a[AXIS_B] - b[AXIS_B]) +
				 square(a[AXIS_C] - b[AXIS_C])));
}

float *set_vector(float x, float y, float z, float a, float b, float c)
{
	vector[AXIS_X] = x;
	vector[AXIS_Y] = y;
	vector[AXIS_Z] = z;
	vector[AXIS_A] = a;
	vector[AXIS_B] = b;
	vector[AXIS_C] = c;
	return (vector);
}

float *set_vector_by_axis(float value, uint8_t axis)
{
	clear_vector(vector);
	switch (axis) {
		case (AXIS_X): vector[AXIS_X] = value; break;
		case (AXIS_Y): vector[AXIS_Y] = value; break;
		case (AXIS_Z): vector[AXIS_Z] = value; break;
		case (AXIS_A): vector[AXIS_A] = value; break;
		case (AXIS_B): vector[AXIS_B] = value; break;
		case (AXIS_C): vector[AXIS_C] = value;
	}
	return (vector);
}

/**** Math and other general purpose functions ****/

/* Slightly faster (*) multi-value min and max functions
 * 	min3() - return minimum of 3 numbers
 * 	min4() - return minimum of 4 numbers
 * 	max3() - return maximum of 3 numbers
 * 	max4() - return maximum of 4 numbers
 *
 * Implementation tip: Order the min and max values from most to least likely in the calling args
 *
 * (*) Macro min4 is about 20uSec, inline function version is closer to 10 uSec (Xmega 32 MHz)
 * 	#define min3(a,b,c) (min(min(a,b),c))
 *	#define min4(a,b,c,d) (min(min(a,b),min(c,d)))
 *	#define max3(a,b,c) (max(max(a,b),c))
 *	#define max4(a,b,c,d) (max(max(a,b),max(c,d)))
 */

float min3(float x1, float x2, float x3)
{
	float min = x1;
	if (x2 < min) { min = x2;}
	if (x3 < min) { return (x3);}
	return (min);
}

float min4(float x1, float x2, float x3, float x4)
{
	float min = x1;
	if (x2 < min) { min = x2;}
	if (x3 < min) { min = x3;}
	if (x4 < min) { return (x4);}
	return (min);
}

float max3(float x1, float x2, float x3)
{
	float max = x1;
	if (x2 > max) { max = x2;}
	if (x3 > max) { return (x3);}
	return (max);
}

float max4(float x1, float x2, float x3, float x4)
{
	float max = x1;
	if (x2 > max) { max = x2;}
	if (x3 > max) { max = x3;}
	if (x4 > max) { return (x4);}
	return (max);
}


/**** String utilities ****
 * isnumber() 	  - isdigit that also accepts plus, minus, and decimal point
 * str_escape()   - add escapes to a string - currently for quotes only
 * str_unescape() - remove escaped quotes from a string. Also remove leading "'s if any
 * str_concat()   - concatenate src to dst string and return pointer to NUL of dst string
 * str_asciify()  - turn string into good old American 7 but ASCII - obliterate smart quotes
 */

uint8_t isnumber(char c)
{
	if (c == '.') { return (true); }
	if (c == '-') { return (true); }
	if (c == '+') { return (true); }
	return (isdigit(c));
}

char *str_escape(char *dst, const char *src)
{
	char c;
	char *start_dst = dst;

	while ((c = *(src++)) != 0) {	// NUL
		if (c == '"') { *(dst++) = '\\'; }
		*(dst++) = c;
	}
	return (start_dst);
}

char *str_unescape(char *str)
{
    char c;
    char *wr = str;  // write pointer
	char *start_str = str;

    if (*str == '"') { str++; }     // skip leading quote in string

    while ((c = *(str++)) != 0) {	// NUL
        if ((c == '\\') && (*str == '"')) { 
            str++;
        }
        *wr++ = c;
    }
    wr--;                        // back up to last char before the NUL
    if (*wr == '"') { *wr = 0; }    // re-terminate
    return (start_str);
}

// remove or replace any 8-bit characters with good old American 7 bit ASCII
char *str_asciify(char *str)
{
    char *wr = str;                                     // write pointer
	char *start_str = str;

    do {
        if ((*str == 0xE2) && (*(str+1) == 0x80)) {     // replace so-called "smart" quotes everywhere
            str += 2;
            if ((*str == 0x9C) || (*str == 0x9D)) {
                *wr++ = '\"';
            }
        }
        *str = *str & 0x7F;         // should not be seeing any MSBs in ASCII. Make sure
    } while (str++, *str != 0);     // NUL
    return (start_str);
}

/*
 * str2float() - wrapped version of strtod with additional error checking
 * str2long()  - wrapped version of strtol with additional error checking
 */
// 
stat_t str2float(const char *str, float *value)
{ 
    char *end;
    *value = (float)strtod(str, &end);
    if (end == str) { return (STAT_BAD_NUMBER_FORMAT); } // failed to convert
    if (*end != 0) { return (STAT_BAD_NUMBER_FORMAT); }  // nonsense following float string
    if (isnan(*value)) { *value = 0; return (STAT_FLOAT_IS_NAN); }
    if (isinf(*value)) { *value = 0; return (STAT_FLOAT_IS_INFINITE); }
    return (STAT_OK);
}

stat_t str2long(const char *str, uint32_t *value)
{
    if (strchr(str, (int)'.') != NULL) { return (STAT_VALUE_TYPE_ERROR); } // is float
    char *end;
    *value = strtol(str, &end, 0);
    if (end == str) { return (STAT_BAD_NUMBER_FORMAT); } // failed to convert
    if (*end != 0) { return (STAT_BAD_NUMBER_FORMAT); }  // nonsense following int string
    return (STAT_OK);
}

/*
 * pstr2str() - return an AVR style progmem string as a RAM string. No effect on ARMs
 *
 *	This function deals with FLASH memory string confusion between the AVR serias and ARMs.
 *	AVRs typically have xxxxx_P() functions which take strings from FLASH as args.
 *	On ARMs there is no need for this as strings are handled identically in FLASH and RAM.
 *
 *	This function copies a string from FLASH to a pre-allocated RAM buffer - see main.c for
 *	allocation and max length. On the ARM it's a pass through that just returns the address
 *	of the input string
 */
char *pstr2str(const char *pgm_string)
{
#ifdef __AVR
	strncpy_P(text_item, pgm_string, TEXT_ITEM_LEN);
	return (text_item);
#endif
#ifdef __ARM
	return ((char *)pgm_string);
#endif
}

/*
 * adapted string concatenation functions
 *
 *  strcat_string()    - add a string to a base string with opening/closing quotes
 *  strcat_string_P()  - add a PSTR string to a base string w/quotes
 *  strcat_literal_P() - add a PSTR string to a base string with no opening / closing quotes
 *  strcat_integer()   - add an unsigned integer value to a string
 *  strcat_signed()    - add a signed integer value to a string
 *  strcat_float()     - add a floating point valu to a string with defined precision digits
 *
 *  All the above perform the same basic function:
 *    - add an element to a base string defined as the dst. 
 *    - terminate the string with NUL
 *    - return a pointer to the terminating NUL
 *    - numeric routines perform the approriate conversion
 *    _ _P routines accept PSTR strings as input
 */

char *strcat_string(char *str, const char *src)
{
    *str++ = '"';
    while (*src != 0) {
        *str++ = *src++;
    }
    *str++ = '"';
    return (str);
}

char *strcat_string_P(char *str, const char *src)
{
    return (str);   //++++ STUBBED
}

char *strcat_literal_P(char *str, const char *src)
{
#ifdef __AVR
    strcpy_P(str, src);
    while (*str++ != 0);
    return (--str);
#endif
#ifdef __ARM
    strcpy(str, src);
    while (*str++ != 0);
    return (--str);
#endif
}

char *strcat_integer(char *str, const uint32_t value)
{
//    inttoa(str, value);
//    while (*str++ != 0);
//    return (--str);

// The above inttoa() function adds ~1700 bytes to the FLASH footprint. 
// If that matters more that execution speed use the following code instead:
    str += sprintf_P(str, PSTR("%lu"), value);
    return (str);
}

char *strcat_signed(char *str, const int32_t value)
{
    str += sprintf_P(str, PSTR("%l"), value);
    return (str);   //++++ STUBBED
}

char *strcat_float(char *str, const float value, const uint8_t precision)
{
    fntoa(str, value, precision);
    while (*str++ != 0);
    return (--str);
}


/*
 * fntoa() - return ASCII string given a float and a decimal precision value
 *
 *	Returns length of string, less the terminating NUL character
 */
char fntoa(char *str, float n, uint8_t precision)
{
    // handle special cases
	if (isnan(n)) {
		strcpy(str, "nan");
		return (3);

	} else if (isinf(n)) {
		strcpy(str, "inf");
		return (3);

	} else if (precision == 0 ) { return((char)sprintf_P((char *)str, PSTR("%0.0f"), (double) n));
	} else if (precision == 1 ) { return((char)sprintf_P((char *)str, PSTR("%0.1f"), (double) n));
	} else if (precision == 2 ) { return((char)sprintf_P((char *)str, PSTR("%0.2f"), (double) n));
	} else if (precision == 3 ) { return((char)sprintf_P((char *)str, PSTR("%0.3f"), (double) n));
	} else if (precision == 4 ) { return((char)sprintf_P((char *)str, PSTR("%0.4f"), (double) n));
	} else if (precision == 5 ) { return((char)sprintf_P((char *)str, PSTR("%0.5f"), (double) n));
	} else if (precision == 6 ) { return((char)sprintf_P((char *)str, PSTR("%0.6f"), (double) n));
	} else if (precision == 7 ) { return((char)sprintf_P((char *)str, PSTR("%0.7f"), (double) n));
	} else					    { return((char)sprintf_P((char *)str, PSTR("%f"), (double) n)); }
}

/*
 * compute_checksum() - calculate the checksum for a string
 *
 *	Stops calculation on null termination or length value if non-zero.
 * 	This is based on the the Java hashCode function. See http://en.wikipedia.org/wiki/Java_hashCode()
 */
/*
#define HASHMASK 9999
uint16_t compute_checksum(char const *string, const uint16_t length)
{
	uint32_t h = 0;
	uint16_t len = strlen(string);
	if (length != 0) len = min(len, length);
    for (uint16_t i=0; i<len; i++) {
		h = 31 * h + string[i];
    }
    return (h % HASHMASK);
}
*/
/*
 * SysTickTimer_getValue() - this is a hack to get around some compatibility problems
 */

#ifdef __AVR
uint32_t SysTickTimer_getValue()
{
	return (rtc.sys_ticks);
}
#endif // __AVR

#ifdef __ARM
uint32_t SysTickTimer_getValue()
{
	return (SysTickTimer.getValue());
}
#endif // __ARM

/***********************************************
 **** Very Fast Number to ASCII Conversions ****
 ***********************************************/
/*
 * inttoa() - integer to ASCII
 *
 *  Taking advantage of the fact that most ints we display are 8 bit quantities,
 *  and we have plenty of FLASH
 */
// static ASCII numbers
static const char itoa_00[] PROGMEM = "0";
static const char itoa_01[] PROGMEM = "1";
static const char itoa_02[] PROGMEM = "2";
static const char itoa_03[] PROGMEM = "3";
static const char itoa_04[] PROGMEM = "4";
static const char itoa_05[] PROGMEM = "5";
static const char itoa_06[] PROGMEM = "6";
static const char itoa_07[] PROGMEM = "7";
static const char itoa_08[] PROGMEM = "8";
static const char itoa_09[] PROGMEM = "9";
static const char itoa_10[] PROGMEM = "10";
static const char itoa_11[] PROGMEM = "11";
static const char itoa_12[] PROGMEM = "12";
static const char itoa_13[] PROGMEM = "13";
static const char itoa_14[] PROGMEM = "14";
static const char itoa_15[] PROGMEM = "15";
static const char itoa_16[] PROGMEM = "16";
static const char itoa_17[] PROGMEM = "17";
static const char itoa_18[] PROGMEM = "18";
static const char itoa_19[] PROGMEM = "19";
static const char itoa_20[] PROGMEM = "20";
static const char itoa_21[] PROGMEM = "21";
static const char itoa_22[] PROGMEM = "22";
static const char itoa_23[] PROGMEM = "23";
static const char itoa_24[] PROGMEM = "24";
static const char itoa_25[] PROGMEM = "25";
static const char itoa_26[] PROGMEM = "26";
static const char itoa_27[] PROGMEM = "27";
static const char itoa_28[] PROGMEM = "28";
static const char itoa_29[] PROGMEM = "29";
static const char itoa_30[] PROGMEM = "30";
static const char itoa_31[] PROGMEM = "31";

static const char itoa_32[] PROGMEM = "32";
static const char itoa_33[] PROGMEM = "33";
static const char itoa_34[] PROGMEM = "34";
static const char itoa_35[] PROGMEM = "35";
static const char itoa_36[] PROGMEM = "36";
static const char itoa_37[] PROGMEM = "37";
static const char itoa_38[] PROGMEM = "38";
static const char itoa_39[] PROGMEM = "39";
static const char itoa_40[] PROGMEM = "40";
static const char itoa_41[] PROGMEM = "41";
static const char itoa_42[] PROGMEM = "42";
static const char itoa_43[] PROGMEM = "43";
static const char itoa_44[] PROGMEM = "44";
static const char itoa_45[] PROGMEM = "45";
static const char itoa_46[] PROGMEM = "46";
static const char itoa_47[] PROGMEM = "47";
static const char itoa_48[] PROGMEM = "48";
static const char itoa_49[] PROGMEM = "49";
static const char itoa_50[] PROGMEM = "50";
static const char itoa_51[] PROGMEM = "51";
static const char itoa_52[] PROGMEM = "52";
static const char itoa_53[] PROGMEM = "53";
static const char itoa_54[] PROGMEM = "54";
static const char itoa_55[] PROGMEM = "55";
static const char itoa_56[] PROGMEM = "56";
static const char itoa_57[] PROGMEM = "57";
static const char itoa_58[] PROGMEM = "58";
static const char itoa_59[] PROGMEM = "59";
static const char itoa_60[] PROGMEM = "60";
static const char itoa_61[] PROGMEM = "61";
static const char itoa_62[] PROGMEM = "62";
static const char itoa_63[] PROGMEM = "63";

static const char itoa_64[] PROGMEM = "64";
static const char itoa_65[] PROGMEM = "65";
static const char itoa_66[] PROGMEM = "66";
static const char itoa_67[] PROGMEM = "67";
static const char itoa_68[] PROGMEM = "68";
static const char itoa_69[] PROGMEM = "69";
static const char itoa_70[] PROGMEM = "70";
static const char itoa_71[] PROGMEM = "71";
static const char itoa_72[] PROGMEM = "72";
static const char itoa_73[] PROGMEM = "73";
static const char itoa_74[] PROGMEM = "74";
static const char itoa_75[] PROGMEM = "75";
static const char itoa_76[] PROGMEM = "76";
static const char itoa_77[] PROGMEM = "77";
static const char itoa_78[] PROGMEM = "78";
static const char itoa_79[] PROGMEM = "79";
static const char itoa_80[] PROGMEM = "80";
static const char itoa_81[] PROGMEM = "81";
static const char itoa_82[] PROGMEM = "82";
static const char itoa_83[] PROGMEM = "83";
static const char itoa_84[] PROGMEM = "84";
static const char itoa_85[] PROGMEM = "85";
static const char itoa_86[] PROGMEM = "86";
static const char itoa_87[] PROGMEM = "87";
static const char itoa_88[] PROGMEM = "88";
static const char itoa_89[] PROGMEM = "89";
static const char itoa_90[] PROGMEM = "90";
static const char itoa_91[] PROGMEM = "91";
static const char itoa_92[] PROGMEM = "92";
static const char itoa_93[] PROGMEM = "93";
static const char itoa_94[] PROGMEM = "94";
static const char itoa_95[] PROGMEM = "95";

static const char itoa_96[] PROGMEM = "96";
static const char itoa_97[] PROGMEM = "97";
static const char itoa_98[] PROGMEM = "98";
static const char itoa_99[] PROGMEM = "99";
static const char itoa_100[] PROGMEM = "100";
static const char itoa_101[] PROGMEM = "101";
static const char itoa_102[] PROGMEM = "102";
static const char itoa_103[] PROGMEM = "103";
static const char itoa_104[] PROGMEM = "104";
static const char itoa_105[] PROGMEM = "105";
static const char itoa_106[] PROGMEM = "106";
static const char itoa_107[] PROGMEM = "107";
static const char itoa_108[] PROGMEM = "108";
static const char itoa_109[] PROGMEM = "109";
static const char itoa_110[] PROGMEM = "110";
static const char itoa_111[] PROGMEM = "111";
static const char itoa_112[] PROGMEM = "112";
static const char itoa_113[] PROGMEM = "113";
static const char itoa_114[] PROGMEM = "114";
static const char itoa_115[] PROGMEM = "115";
static const char itoa_116[] PROGMEM = "116";
static const char itoa_117[] PROGMEM = "117";
static const char itoa_118[] PROGMEM = "118";
static const char itoa_119[] PROGMEM = "119";
static const char itoa_120[] PROGMEM = "120";
static const char itoa_121[] PROGMEM = "121";
static const char itoa_122[] PROGMEM = "122";
static const char itoa_123[] PROGMEM = "123";
static const char itoa_124[] PROGMEM = "124";
static const char itoa_125[] PROGMEM = "125";
static const char itoa_126[] PROGMEM = "126";
static const char itoa_127[] PROGMEM = "127";

static const char itoa_128[] PROGMEM = "128";
static const char itoa_129[] PROGMEM = "129";
static const char itoa_130[] PROGMEM = "130";
static const char itoa_131[] PROGMEM = "131";
static const char itoa_132[] PROGMEM = "132";
static const char itoa_133[] PROGMEM = "133";
static const char itoa_134[] PROGMEM = "134";
static const char itoa_135[] PROGMEM = "135";
static const char itoa_136[] PROGMEM = "136";
static const char itoa_137[] PROGMEM = "137";
static const char itoa_138[] PROGMEM = "138";
static const char itoa_139[] PROGMEM = "139";
static const char itoa_140[] PROGMEM = "140";
static const char itoa_141[] PROGMEM = "141";
static const char itoa_142[] PROGMEM = "142";
static const char itoa_143[] PROGMEM = "143";
static const char itoa_144[] PROGMEM = "144";
static const char itoa_145[] PROGMEM = "145";
static const char itoa_146[] PROGMEM = "146";
static const char itoa_147[] PROGMEM = "147";
static const char itoa_148[] PROGMEM = "148";
static const char itoa_149[] PROGMEM = "149";
static const char itoa_150[] PROGMEM = "150";
static const char itoa_151[] PROGMEM = "151";
static const char itoa_152[] PROGMEM = "152";
static const char itoa_153[] PROGMEM = "153";
static const char itoa_154[] PROGMEM = "154";
static const char itoa_155[] PROGMEM = "155";
static const char itoa_156[] PROGMEM = "156";
static const char itoa_157[] PROGMEM = "157";
static const char itoa_158[] PROGMEM = "158";
static const char itoa_159[] PROGMEM = "159";

static const char itoa_160[] PROGMEM = "160";
static const char itoa_161[] PROGMEM = "161";
static const char itoa_162[] PROGMEM = "162";
static const char itoa_163[] PROGMEM = "163";
static const char itoa_164[] PROGMEM = "164";
static const char itoa_165[] PROGMEM = "165";
static const char itoa_166[] PROGMEM = "166";
static const char itoa_167[] PROGMEM = "167";
static const char itoa_168[] PROGMEM = "168";
static const char itoa_169[] PROGMEM = "169";
static const char itoa_170[] PROGMEM = "170";
static const char itoa_171[] PROGMEM = "171";
static const char itoa_172[] PROGMEM = "172";
static const char itoa_173[] PROGMEM = "173";
static const char itoa_174[] PROGMEM = "174";
static const char itoa_175[] PROGMEM = "175";
static const char itoa_176[] PROGMEM = "176";
static const char itoa_177[] PROGMEM = "177";
static const char itoa_178[] PROGMEM = "178";
static const char itoa_179[] PROGMEM = "179";
static const char itoa_180[] PROGMEM = "180";
static const char itoa_181[] PROGMEM = "181";
static const char itoa_182[] PROGMEM = "182";
static const char itoa_183[] PROGMEM = "183";
static const char itoa_184[] PROGMEM = "184";
static const char itoa_185[] PROGMEM = "185";
static const char itoa_186[] PROGMEM = "186";
static const char itoa_187[] PROGMEM = "187";
static const char itoa_188[] PROGMEM = "188";
static const char itoa_189[] PROGMEM = "189";
static const char itoa_190[] PROGMEM = "190";
static const char itoa_191[] PROGMEM = "191";

static const char itoa_192[] PROGMEM = "192";
static const char itoa_193[] PROGMEM = "193";
static const char itoa_194[] PROGMEM = "194";
static const char itoa_195[] PROGMEM = "195";
static const char itoa_196[] PROGMEM = "196";
static const char itoa_197[] PROGMEM = "197";
static const char itoa_198[] PROGMEM = "198";
static const char itoa_199[] PROGMEM = "199";
static const char itoa_200[] PROGMEM = "200";
static const char itoa_201[] PROGMEM = "201";
static const char itoa_202[] PROGMEM = "202";
static const char itoa_203[] PROGMEM = "203";
static const char itoa_204[] PROGMEM = "204";
static const char itoa_205[] PROGMEM = "205";
static const char itoa_206[] PROGMEM = "206";
static const char itoa_207[] PROGMEM = "207";
static const char itoa_208[] PROGMEM = "208";
static const char itoa_209[] PROGMEM = "209";
static const char itoa_210[] PROGMEM = "210";
static const char itoa_211[] PROGMEM = "211";
static const char itoa_212[] PROGMEM = "212";
static const char itoa_213[] PROGMEM = "213";
static const char itoa_214[] PROGMEM = "214";
static const char itoa_215[] PROGMEM = "215";
static const char itoa_216[] PROGMEM = "216";
static const char itoa_217[] PROGMEM = "217";
static const char itoa_218[] PROGMEM = "218";
static const char itoa_219[] PROGMEM = "219";
static const char itoa_220[] PROGMEM = "220";
static const char itoa_221[] PROGMEM = "221";
static const char itoa_222[] PROGMEM = "222";
static const char itoa_223[] PROGMEM = "223";

static const char itoa_224[] PROGMEM = "224";
static const char itoa_225[] PROGMEM = "225";
static const char itoa_226[] PROGMEM = "226";
static const char itoa_227[] PROGMEM = "227";
static const char itoa_228[] PROGMEM = "228";
static const char itoa_229[] PROGMEM = "229";
static const char itoa_230[] PROGMEM = "230";
static const char itoa_231[] PROGMEM = "231";
static const char itoa_232[] PROGMEM = "232";
static const char itoa_233[] PROGMEM = "233";
static const char itoa_234[] PROGMEM = "234";
static const char itoa_235[] PROGMEM = "235";
static const char itoa_236[] PROGMEM = "236";
static const char itoa_237[] PROGMEM = "237";
static const char itoa_238[] PROGMEM = "238";
static const char itoa_239[] PROGMEM = "239";
static const char itoa_240[] PROGMEM = "240";
static const char itoa_241[] PROGMEM = "241";
static const char itoa_242[] PROGMEM = "242";
static const char itoa_243[] PROGMEM = "243";
static const char itoa_244[] PROGMEM = "244";
static const char itoa_245[] PROGMEM = "245";
static const char itoa_246[] PROGMEM = "246";
static const char itoa_247[] PROGMEM = "247";
static const char itoa_248[] PROGMEM = "248";
static const char itoa_249[] PROGMEM = "249";
static const char itoa_250[] PROGMEM = "250";
static const char itoa_251[] PROGMEM = "251";
static const char itoa_252[] PROGMEM = "252";
static const char itoa_253[] PROGMEM = "253";
static const char itoa_254[] PROGMEM = "254";
static const char itoa_255[] PROGMEM = "255";

static const char *const itoa_str[] PROGMEM = {
    itoa_00, itoa_01, itoa_02, itoa_03, itoa_04, itoa_05, itoa_06, itoa_07, itoa_08, itoa_09,
    itoa_10, itoa_11, itoa_12, itoa_13, itoa_14, itoa_15, itoa_16, itoa_17, itoa_18, itoa_19,
    itoa_20, itoa_21, itoa_22, itoa_23, itoa_24, itoa_25, itoa_26, itoa_27, itoa_28, itoa_29,
    itoa_30, itoa_31, itoa_32, itoa_33, itoa_34, itoa_35, itoa_36, itoa_37, itoa_38, itoa_39,
    itoa_40, itoa_41, itoa_42, itoa_43, itoa_44, itoa_45, itoa_46, itoa_47, itoa_48, itoa_49,
    itoa_50, itoa_51, itoa_52, itoa_53, itoa_54, itoa_55, itoa_56, itoa_57, itoa_58, itoa_59,
    itoa_60, itoa_61, itoa_62, itoa_63, itoa_64, itoa_65, itoa_66, itoa_67, itoa_68, itoa_69,
    itoa_70, itoa_71, itoa_72, itoa_73, itoa_74, itoa_75, itoa_76, itoa_77, itoa_78, itoa_79,
    itoa_80, itoa_81, itoa_82, itoa_83, itoa_84, itoa_85, itoa_86, itoa_87, itoa_88, itoa_89,
    itoa_90, itoa_91, itoa_92, itoa_93, itoa_94, itoa_95, itoa_96, itoa_97, itoa_98, itoa_99,
    itoa_100, itoa_101, itoa_102, itoa_103, itoa_104, itoa_105, itoa_106, itoa_107, itoa_108, itoa_109,
    itoa_110, itoa_111, itoa_112, itoa_113, itoa_114, itoa_115, itoa_116, itoa_117, itoa_118, itoa_119,
    itoa_120, itoa_121, itoa_122, itoa_123, itoa_124, itoa_125, itoa_126, itoa_127, itoa_128, itoa_129,
    itoa_130, itoa_131, itoa_132, itoa_133, itoa_134, itoa_135, itoa_136, itoa_137, itoa_138, itoa_139,
    itoa_140, itoa_141, itoa_142, itoa_143, itoa_144, itoa_145, itoa_146, itoa_147, itoa_148, itoa_149,
    itoa_150, itoa_151, itoa_152, itoa_153, itoa_154, itoa_155, itoa_156, itoa_157, itoa_158, itoa_159,
    itoa_160, itoa_161, itoa_162, itoa_163, itoa_164, itoa_165, itoa_166, itoa_167, itoa_168, itoa_169,
    itoa_170, itoa_171, itoa_172, itoa_173, itoa_174, itoa_175, itoa_176, itoa_177, itoa_178, itoa_179,
    itoa_180, itoa_181, itoa_182, itoa_183, itoa_184, itoa_185, itoa_186, itoa_187, itoa_188, itoa_189,
    itoa_190, itoa_191, itoa_192, itoa_193, itoa_194, itoa_195, itoa_196, itoa_197, itoa_198, itoa_199,
    itoa_200, itoa_201, itoa_202, itoa_203, itoa_204, itoa_205, itoa_206, itoa_207, itoa_208, itoa_209,
    itoa_210, itoa_211, itoa_212, itoa_213, itoa_214, itoa_215, itoa_216, itoa_217, itoa_218, itoa_219,
    itoa_220, itoa_221, itoa_222, itoa_223, itoa_224, itoa_225, itoa_226, itoa_227, itoa_228, itoa_229,
    itoa_230, itoa_231, itoa_232, itoa_233, itoa_234, itoa_235, itoa_236, itoa_237, itoa_238, itoa_239,
    itoa_240, itoa_241, itoa_242, itoa_243, itoa_244, itoa_245, itoa_246, itoa_247, itoa_248, itoa_249,
    itoa_250, itoa_251, itoa_252, itoa_253, itoa_254, itoa_255
};

static int _i2a(char *s, int n)
{
    div_t qr;
    int pos;

    if (n == 0) {
        return 0;
    }

    qr = div(n, 10);
    pos = _i2a(s, qr.quot);
    s[pos] = qr.rem + '0';
    return (pos + 1);
}

char inttoa(char *str, int n)
{
    if (n < 256) {
        strcpy(str, GET_TEXT_ITEM(itoa_str, n));
    } else {
        char *p = str;
        if (n < 0){
            *p++ = '-';
            n *= -1;
        } else if(n == 0) {
            *p++ = '0';
        }
        p[_i2a(p, n)]='\0';
        }
    return (strlen(str));
}
/*
void test_ftoa()
{
    int i;
    char str[32];
    int len;
    float d[] = {
        0.0,
        42.0,
        123.456789,
        -123.456789,
        1234.0000,
        -1234.0000,
        0.00018,
    };
    for (i = 0; i < 6; i++) {
        len = floattoa(str, d[i], 4, 10);
        printf("%f: %s len:%d precision:4\n", d[i], str, len);
    }
}
*/

static const float round_lookup[] = {
    0.5,          // precision 0
    0.05,         // precision 1
    0.005,        // precision 2
    0.0005,       // precision 3
    0.00005,      // precision 4
    0.000005,     // precision 5
    0.0000005,    // precision 6
    0.00000005,   // precision 7
};


// It's assumed that the string buffer contains at lest count_ non-\0 chars
/* C++ version (put in header file)
constexpr int c_strreverse(char * const t, const int count_, char hold = 0) {
    return count_>1 ? (hold=*t, *t=*(t+(count_-1)), *(t+(count_-1))=hold), c_strreverse(t+1, count_-2), count_ : count_;
}
  C version with recursion
int c_strreverse(char * const t, const int count_) {
    char hold = 0;
    return count_>1 ? (hold=*t, *t=*(t+(count_-1)), *(t+(count_-1))=hold), c_strreverse(t+1, count_-2), count_ : count_;
}
*/
//C version non-recursive
int c_strreverse(char *t, int count_) {
    char hold = 0;
    int c_ = count_;
    while (c_>1) {
        hold=*t;
        *t=*(t+(c_-1));
        *(t+(c_-1))=hold;
        t++;
        c_-=2;
    }
    return count_;
}

char floattoa(char *buffer, float in, int precision) {
    int maxlen = 16;                    // set an arbitrary maximum length for the output string
    int length = 0;
    char *b = buffer;

    if (in < 0.0) {                     // handle negative numbers
        *b++ = '-';
        in = -in;
        length++;
    }

    in += round_lookup[precision];      // round up the number to the precision
    int int_length = 0;
    int integer_part = (int)in;

    // do integer part
    while (integer_part > 0) {
        if (length++ > maxlen) {
            *buffer = 0;
            return 0;
        }
        int t = integer_part / 10;
        *b++ = '0' + (integer_part - (t*10));
        integer_part = t;
        int_length++;
    }
    if (length > 0) {
        c_strreverse(buffer, int_length);
    } else {
        *b++ = '0';
    }

    // do fractional part
    *b++ = '.';
    length++;
    float frac_part = in;
    frac_part -= (int)frac_part;
    while (precision-- > 0) {
        if (length++ > maxlen) {
            *buffer = 0;
            return 0;
        }
        frac_part *= 10.0;
        *b++ = ('0' + (int)frac_part);
        frac_part -= (int)frac_part;
    }

    // right strip trailing zeroes (OPTIONAL)
    while (*(b-1) == '0') {
        *(b--) = 0;
        length--;
    }
    if (*(b-1) == '.') {
        *(b--) = 0;
        length--;
    }
    return (char)length;
}
