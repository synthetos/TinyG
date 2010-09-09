/*
  xmega_io_progmem.h - serial and "file" IO functions for xmega family
  USART module

  Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef xmega_io_progmem_h
#define xmega_io_progmem_h

#include "xmega_io.h"

/*
 * Local subsystem configs, constants, and device structures 
 */

/* 
 * Function prototypes and aliases
 */

int8_t xio_open_pgm(uint8_t dev, uint32_t control);
int8_t xio_close_pgm(struct fdUSART *f);
int8_t xio_control_pgm(struct fdUSART *f, uint32_t control, int16_t arg);
int16_t xio_read_pgm(struct fdUSART *fd_ptr, char *buf, int16_t size);
int16_t xio_write_pgm(struct fdUSART *fd_ptr, const char *buf, int16_t size);
char xio_getc_pgm(struct fdUSART *fd_ptr);
char xio_putc_pgm(struct fdUSART *fd_ptr, const char c);

#endif
