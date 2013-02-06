/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* I2C/TWI Module                                                       */
/*                                                                      */
/* i2c.h                                                                */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2010 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef __I2C_H
#define __I2C_H

#include "xboot.h"

// Globals
#ifdef USE_I2C
extern unsigned char first_byte;
#endif

// Defines
#define i2c_address_match() ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_APIF_bm) && \
        (I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_AP_bm))
#define i2c_char_received() ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIF_bm) && \
        !(I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIR_bm))
#define i2c_ready_data() ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIF_bm) && \
        (I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIR_bm))
#define i2c_got_ack() (I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_RXACK_bm)
#define i2c_send_ack() I2C_DEVICE.SLAVE.CTRLB = 0b00000011
#define i2c_send_nak() I2C_DEVICE.SLAVE.CTRLB = 0b00000110
#define i2c_end_transmission() I2C_DEVICE.SLAVE.CTRLB = 0b00000010
#define i2c_cur_char() I2C_DEVICE.SLAVE.DATA
#define i2c_send_char(c) I2C_DEVICE.SLAVE.DATA = (c)

// Prototypes
extern void i2c_init(void);
extern void i2c_deinit(void);

#endif // __I2C_H