/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* XBoot Protocol Definition                                            */
/*                                                                      */
/* protocol.h                                                           */
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

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "xboot.h"

// Defines

#define CMD_SYNC                '\x1b'
#define CMD_CHECK_AUTOINCREMENT 'a'
#define CMD_SET_ADDRESS         'A'
#define CMD_SET_EXT_ADDRESS     'H'
#define CMD_CHIP_ERASE          'e'
#define CMD_CHECK_BLOCK_SUPPORT 'b'
#define CMD_BLOCK_LOAD          'B'
#define CMD_BLOCK_READ          'g'
#define CMD_READ_BYTE           'R'
#define CMD_WRITE_LOW_BYTE      'c'
#define CMD_WRITE_HIGH_BYTE     'C'
#define CMD_WRITE_PAGE          'm'
#define CMD_WRITE_EEPROM_BYTE   'D'
#define CMD_READ_EEPROM_BYTE    'd'
#define CMD_WRITE_LOCK_BITS     'l'
#define CMD_READ_LOCK_BITS      'r'
#define CMD_READ_LOW_FUSE_BITS  'F'
#define CMD_READ_HIGH_FUSE_BITS 'N'
#define CMD_READ_EXT_FUSE_BITS  'Q'
#define CMD_ENTER_PROG_MODE     'P'
#define CMD_LEAVE_PROG_MODE     'L'
#define CMD_EXIT_BOOTLOADER     'E'
#define CMD_PROGRAMMER_TYPE     'p'
#define CMD_DEVICE_CODE         't'
#define CMD_SET_LED             'x'
#define CMD_CLEAR_LED           'y'
#define CMD_SET_TYPE            'T'
#define CMD_PROGRAM_ID          'S'
#define CMD_VERSION             'V'
#define CMD_READ_SIGNATURE      's'
#define CMD_AUTONEG_START       '@'
#define CMD_AUTONEG_DONE        '#'

#define MEM_EEPROM              'E'
#define MEM_FLASH               'F'
#define MEM_USERSIG             'U'
#define MEM_PRODSIG             'P'

#define REPLY_ACK               '\r'
#define REPLY_YES               'Y'
#define REPLY_ERROR             '?'

#endif // __PROTOCOL_H

