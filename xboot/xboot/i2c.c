/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* I2C/TWI Module                                                       */
/*                                                                      */
/* i2c.c                                                                */
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

#include "i2c.h"

// Globals
#ifdef USE_I2C
unsigned char first_byte;
#endif

// Interrupts
#ifdef USE_INTERRUPTS
#ifdef USE_I2C
ISR(I2C_DEVICE_ISR)
{
        if ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_APIF_bm) && 
                (I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_AP_bm))
        {
                // Address match, send ACK
                I2C_DEVICE.SLAVE.CTRLB = 0b00000011;
                comm_mode = MODE_I2C;
                #ifdef USE_UART
                #ifdef __AVR_XMEGA__
                // disable I2C interrupt
                UART_DEVICE.CTRLA = 0;
                #endif // __AVR_XMEGA__
                #endif // USE_UART
                first_byte = 1;
        }
        if ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIF_bm) &&
                !(I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIR_bm))
        {
                // Data has arrived
                if (rx_char_cnt == 0)
                {
                        rx_buff0 = I2C_DEVICE.SLAVE.DATA;
                        rx_char_cnt = 1;
                }
                else
                {
                        rx_buff1 = I2C_DEVICE.SLAVE.DATA;
                        rx_char_cnt = 2;
                }
                I2C_DEVICE.SLAVE.CTRLB = 0b00000011;
        }
        if ((I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIF_bm) &&
                (I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_DIR_bm))
        {
                if (!first_byte && I2C_DEVICE.SLAVE.STATUS & TWI_SLAVE_RXACK_bm)
                {
                        I2C_DEVICE.SLAVE.CTRLB = 0b00000010; // end transaction
                }
                else
                {
                        first_byte = 0;
                        if (tx_char_cnt == 0)
                        {
                                // Wants data, but there is no data to send...
                                // also include NAK
                                I2C_DEVICE.SLAVE.DATA = '?';
                        }
                        else
                        {
                                I2C_DEVICE.SLAVE.DATA = tx_buff0;
                                tx_char_cnt = 0;
                        }
                        I2C_DEVICE.SLAVE.CTRLB = 0b00000110;
                }
        }
}
#endif // USE_I2C
#endif // USE_INTERRUPTS

void i2c_init(void)
{
        I2C_DEVICE.CTRL = 0;
        #if I2C_MATCH_ANY
        #ifdef USE_INTERRUPTS
        I2C_DEVICE.SLAVE.CTRLA = TWI_SLAVE_ENABLE_bm | TWI_SLAVE_PMEN_bm | TWI_SLAVE_INTLVL0_bm;
        #else
        I2C_DEVICE.SLAVE.CTRLA = TWI_SLAVE_ENABLE_bm | TWI_SLAVE_PMEN_bm;
        #endif // USE_INTERRUPTS
        #else
        #ifdef USE_INTERRUPTS
        I2C_DEVICE.SLAVE.CTRLA = TWI_SLAVE_ENABLE_bm | TWI_SLAVE_INTLVL0_bm;
        #else
        I2C_DEVICE.SLAVE.CTRLA = TWI_SLAVE_ENABLE_bm;
        #endif // USE_INTERRUPTS
        #endif
        #if I2C_GC_ENABLE
        I2C_DEVICE.SLAVE.ADDR = I2C_ADDRESS | 1;
        #else
        I2C_DEVICE.SLAVE.ADDR = I2C_ADDRESS;
        #endif
        I2C_DEVICE.SLAVE.ADDRMASK = 0;
}

void i2c_deinit(void)
{
        // Shut down I2C module and turn off interrupt
        I2C_DEVICE.SLAVE.CTRLA = 0;
        I2C_DEVICE.SLAVE.ADDR = 0;
        I2C_DEVICE.SLAVE.ADDRMASK = 0;
}



