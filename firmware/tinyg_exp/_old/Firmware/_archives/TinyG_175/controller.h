/*
  tg_controller.h - tinyg controller and top level routines
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef controller_h
#define controller_h

/*
 * Global Scope Functions
 */

void tg_init(void);
void tg_controller(void);
int tg_parser(char * buf);
int tg_signal(uint8_t sig);


#endif
