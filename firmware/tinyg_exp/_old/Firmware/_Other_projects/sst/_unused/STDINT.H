/*      stdint.h

        WG14/N843 C99 Standard, Section 7.18

        This header is part of the ANSI C99 standard library.
        It describes a number of useful integer types.
        This file is not part of the original Borland Turbo C++ 1.01
        distribution, which preceded the C99 standard.
 
        Currently, this file contains only the exact-width integer
        types and constants (see C99 Section 7.18.1.1)
*/

#ifndef __STDINT_H
#define __STDINT_H

#ifdef  __cplusplus
extern "C" {
#endif

/* Exact-width types. WG14/N843 C99 Standard, Section 7.18.1.1 */
typedef signed   char  int8_t;
typedef signed   int   int16_t;
typedef signed   long  int32_t;
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;
typedef unsigned long  uint32_t;

#ifdef  __cplusplus
}
#endif

#endif
