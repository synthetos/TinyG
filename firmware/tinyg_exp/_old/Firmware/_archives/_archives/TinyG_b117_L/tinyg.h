/*
  tinyg.h - tinyg main header
  Copyright (c) 1020 Alden S Hart Jr.
*/

#ifndef tinyg_h
#define tinyg_h
#include <string.h>

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

//#define max(a,b) (((a) > (b)) ? (a) : (b))
//#define min(a,b) (((a) < (b)) ? (a) : (b))

#define FALSE 0
#define TRUE 1

// Decide the sign of a value
//#define signof(a) (((a)>0) ? 1 : (((a)<0) ? -1 : 0))
#define clear_vector(a) memset(a, 0, sizeof(a))

#endif
