/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* tested with ATXMEGA64A3, ATXMEGA128A1, ATXMEGA256A1, ATXMEGA32A4     */
/*                                                                      */
/* xboot.h                                                              */
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

#ifndef __XBOOT_H
#define __XBOOT_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

// RST.STATUS & RST_SRF_bm	

// token pasting
#define token_paste2_int(x, y) x ## y
#define token_paste2(x, y) token_paste2_int(x, y)
#define token_paste3_int(x, y, z) x ## y ## z
#define token_paste3(x, y, z) token_paste3_int(x, y, z)

// Version
#define XBOOT_VERSION_MAJOR 1
#define XBOOT_VERSION_MINOR 7
#define XBOOT_TINYG_REVISION 1		// added a revision for TinyG uses

// config.h
#ifdef USE_CONFIG_H
#include "config.h"
#endif // USE_CONFIG_H

// Configuration

// clock config
// DFLL for better stability
#define USE_DFLL
// use 32MHz osc if makefile calls for it
#if (F_CPU == 32000000L)
// defaults to 2MHz RC oscillator
// define USE_32MHZ_RC to override
#define USE_32MHZ_RC
#endif // F_CPU

#ifndef USE_CONFIG_H

// Begin Default Configuration Section
// --------------------------------------------------

// AVR1008 fixes
// Really only applicable to XMEGA 256a3 rev A and B devices
//#define USE_AVR1008_EEPROM

// bootloader entrace
#define USE_ENTER_DELAY
//#define USE_ENTER_PIN
#define USE_ENTER_UART
//#define USE_ENTER_I2C
//#define USE_ENTER_FIFO

// bootloader exit
//#define LOCK_SPM_ON_EXIT

// bootloader communication
#define USE_LED
#define USE_UART
//#define USE_UART_EN_PIN
//#define USE_I2C
//#define USE_I2C_ADDRESS_NEGOTIATION
//#define USE_ATTACH_LED
//#define USE_FIFO

// General Options
//#define USE_INTERRUPTS
//#define USE_WATCHDOG

// bootloader features
#define ENABLE_BLOCK_SUPPORT
#define ENABLE_FLASH_BYTE_SUPPORT
#define ENABLE_EEPROM_BYTE_SUPPORT
#define ENABLE_LOCK_BITS
#define ENABLE_FUSE_BITS
#define ENABLE_FLASH_ERASE_WRITE
#define ENABLE_CRC_SUPPORT

// API
#define ENABLE_API
#define USE_API_VERSION 1
#define ENABLE_API_LOW_LEVEL_FLASH
#define ENABLE_API_SPM_WRAPPER
#define ENABLE_API_FIRMWARE_UPDATE

// Code Protection
#define ENABLE_CODE_PROTECTION
#define ENABLE_EEPROM_PROTECTION
#define ENABLE_BOOTLOADER_PROTECTION

// ENTER_PIN
#define ENTER_PORT_NAME         C
#define ENTER_PIN               0
#define ENTER_PIN_STATE         0
#define ENTER_PIN_PUEN          1

// ENTER_DELAY
#define ENTER_BLINK_COUNT       3
#define ENTER_BLINK_WAIT        30000

// ENTER_UART
//#define ENTER_UART_NEED_SYNC

// ENTER_FIFO
//#define ENTER_FIFO_NEED_SYNC

// WATCHDOG
// Select only one
//#define WATCHDOG_TIMEOUT        WDT_PER_8CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_16CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_32CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_64CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_128CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_256CLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_512CLK_gc
#define WATCHDOG_TIMEOUT        WDT_PER_1KCLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_2KCLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_4KCLK_gc
//#define WATCHDOG_TIMEOUT        WDT_PER_8KCLK_gc

// LED
#define LED_PORT_NAME           A
#define LED_PIN                 0
#define LED_INV                 1

// UART
// Select BAUD rate, port name, and UART number
// Port name is only for XMEGA
// For ATMEGA, uart number is usually 0
// UART_U2X will double clock rate for ATMEGA
// Needed for high baud rates
#define UART_BAUD_RATE          115200
#define UART_PORT_NAME          C
#define UART_NUMBER             0
#define UART_U2X
#define UART_RX_PUEN

// UART RS485 Enable Output
#define UART_EN_PORT_NAME       C
#define UART_EN_PIN             4
#define UART_EN_PIN_INV         0

// FIFO
#define FIFO_DATA_PORT_NAME     C
#define FIFO_CTL_PORT_NAME      D
#define FIFO_RXF_N              3
#define FIFO_TXE_N              2
#define FIFO_RD_N               1
#define FIFO_WR_N               0
#define FIFO_BIT_REVERSE

// I2C
#define I2C_DEVICE_PORT         C

#define I2C_MATCH_ANY           1
#define I2C_ADDRESS             0x10
#define I2C_GC_ENABLE           1

// I2C Address Autonegotiation
// Note: only works on XMega chips for the time being
// There is no easy way to get this to work on regular
// ATMega chips as they have no unique part ID number
#define I2C_AUTONEG_DIS_PROMISC         1
#define I2C_AUTONEG_DIS_GC              0
#define I2C_AUTONEG_PORT_NAME           A
#define I2C_AUTONEG_PIN                 2

// Attach LED
#define ATTACH_LED_PORT_NAME            A
#define ATTACH_LED_PIN                  1
#define ATTACH_LED_INV                  1

// --------------------------------------------------
// End Default Configuration Section

#endif // USE_CONFIG_H

// ENTER_PIN
#define ENTER_PORT              token_paste2(PORT, ENTER_PORT_NAME)
#define ENTER_PIN_CTRL          token_paste3(ENTER_PORT.PIN, ENTER_PIN, CTRL)

// LED
#define LED_PORT                token_paste2(PORT, LED_PORT_NAME)

// UART RS485 Enable Output
#define UART_EN_PORT            token_paste2(PORT, UART_EN_PORT_NAME)

#if (UART_NUMBER == 0)
#define UART_RX_PIN             2
#define UART_TX_PIN             3
#else
#define UART_RX_PIN             6
#define UART_TX_PIN             7
#endif
#define UART_PORT               token_paste2(PORT, UART_PORT_NAME)
#define UART_DEVICE_PORT        token_paste2(UART_PORT_NAME, UART_NUMBER)
#define UART_DEVICE             token_paste2(USART, UART_DEVICE_PORT)
#define UART_DEVICE_RXC_ISR     token_paste3(USART, UART_DEVICE_PORT, _RXC_vect)
#define UART_DEVICE_DRE_ISR     token_paste3(USART, UART_DEVICE_PORT, _DRE_vect)
#define UART_DEVICE_TXC_ISR     token_paste3(USART, UART_DEVICE_PORT, _TXC_vect)
#define UART_RX_PIN_CTRL        token_paste3(UART_PORT.PIN, UART_RX_PIN, CTRL)
#define UART_TX_PIN_CTRL        token_paste3(UART_PORT.PIN, UART_TX_PIN, CTRL)

// BAUD Rate Values
// Known good at 2MHz
#if (F_CPU == 2000000L) && (UART_BAUD_RATE == 19200)
#define UART_BSEL_VALUE         12
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 38400)
#define UART_BSEL_VALUE         22
#define UART_BSCALE_VALUE       -2
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 57600)
#define UART_BSEL_VALUE         26
#define UART_BSCALE_VALUE       -3
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 115200)
#define UART_BSEL_VALUE         19
#define UART_BSCALE_VALUE       -4
#define UART_CLK2X              1
// Known good at 32MHz
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 19200)
#define UART_BSEL_VALUE         103
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 38400)
#define UART_BSEL_VALUE         51
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 57600)
#define UART_BSEL_VALUE         34
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 115200)
#define UART_BSEL_VALUE         16
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
// None of the above, so calculate something
#else
#warning Not using predefined BAUD rate, possible BAUD rate error!
#if (F_CPU == 2000000L)
#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 8) - 1)
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              1
#else
#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 16) - 1)
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#endif
#endif

// FIFO
#define FIFO_DATA_PORT          token_paste2(PORT, FIFO_DATA_PORT_NAME)
#define FIFO_CTL_PORT           token_paste2(PORT, FIFO_CTL_PORT_NAME)

// I2C
#define I2C_DEVICE              token_paste2(TWI, I2C_DEVICE_PORT)
#define I2C_DEVICE_ISR          token_paste3(TWI, I2C_DEVICE_PORT, _TWIS_vect)

// I2C Address Autonegotiation
#define I2C_AUTONEG_PORT                token_paste2(PORT, I2C_AUTONEG_PORT_NAME);

// Attach LED
#define ATTACH_LED_PORT                 token_paste2(PORT, ATTACH_LED_PORT_NAME)


#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE E2PAGESIZE
#endif

#ifndef EEPROM_BYTE_ADDRESS_MASK
#if EEPROM_PAGE_SIZE == 32
#define EEPROM_BYTE_ADDRESS_MASK 0x1f
#elif EEPROM_PAGE_SIZE == 16
#define EEPROM_BYTE_ADDRESS_MASK = 0x0f
#elif EEPROM_PAGE_SIZE == 8
#define EEPROM_BYTE_ADDRESS_MASK = 0x07
#elif EEPROM_PAGE_SIZE == 4
#define EEPROM_BYTE_ADDRESS_MASK = 0x03
#else
#error Unknown EEPROM page size!  Please add new byte address value!
#endif
#endif

#ifdef USE_INTERRUPTS
#ifndef NEED_INTERRUPTS
#define NEED_INTERRUPTS
#endif // NEED_INTERRUPTS
#endif // USE_INTERRUPTS

#ifdef USE_AVR1008_EEPROM
#ifndef NEED_INTERRUPTS
#define NEED_INTERRUPTS
#endif // NEED_INTERRUPTS
#endif // USE_AVR1008_EEPROM

#ifdef ENABLE_CODE_PROTECTION
#ifndef NEED_CODE_PROTECTION
#define NEED_CODE_PROTECTION
#endif // NEED_CODE_PROTECTION
#endif // ENABLE_CODE_PROTECTION

#ifdef ENABLE_EEPROM_PROTECTION
#ifndef NEED_CODE_PROTECTION
#define NEED_CODE_PROTECTION
#endif // NEED_CODE_PROTECTION
#endif // ENABLE_EEPROM_PROTECTION

// communication modes
#define MODE_UNDEF              0
#define MODE_UART               1
#define MODE_I2C                2
#define MODE_FIFO               3

// types
typedef uint32_t ADDR_T;

// Includes
#include "protocol.h"
#include "flash.h"
#include "eeprom_driver.h"
#include "uart.h"
#include "i2c.h"
#include "fifo.h"
#include "watchdog.h"
#include "api.h"

// globals
#ifdef USE_INTERRUPTS
extern volatile unsigned char comm_mode;

extern volatile unsigned char rx_buff0;
extern volatile unsigned char rx_buff1;
extern volatile unsigned char rx_char_cnt;

extern volatile unsigned char tx_buff0;
extern volatile unsigned char tx_char_cnt;
#else
extern unsigned char comm_mode;
#endif // USE_INTERRUPTS

// Functions
unsigned char __attribute__ ((noinline)) ow_slave_read_bit(void);
void __attribute__ ((noinline)) ow_slave_write_bit(unsigned char b);
void ow_slave_wait_bit(void);

unsigned char __attribute__ ((noinline)) get_char(void);
void __attribute__ ((noinline)) send_char(unsigned char c);
unsigned int __attribute__ ((noinline)) get_2bytes(void);

void clear_buffer(void);

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address);
void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address);

uint16_t crc16_block(uint32_t start, uint32_t length);
void install_firmware(void);

#endif // __XBOOT_H
