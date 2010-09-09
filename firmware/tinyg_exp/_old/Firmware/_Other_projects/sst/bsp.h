/*****************************************************************************
* Product: SST example, 80x86, Turbo C++ 1.01
*
* Copyright (C) 2006 Miro Samek and Robert Ward. All rights reserved.
*
* This software may be distributed and modified under the terms of the GNU
* General Public License version 2 (GPL) as published by the Free Software
* Foundation and appearing in the file GPL.TXT included in the packaging of
* this file. Please note that GPL Section 2[b] requires that all works based
* on this software must also be made publicly available under the terms of
* the GPL ("Copyleft").
*
* Contact information:
* Email:    miro@quantum-leaps.com
* Internet: www.quantum-leaps.com
*****************************************************************************/
#ifndef bsp_h
#define bsp_h

/* Direct Video Access .....................................................*/
enum VideoColor {
    /* foreground */
    VIDEO_FGND_BLACK        = 0x00,
    VIDEO_FGND_BLUE         = 0x01,
    VIDEO_FGND_GREEN        = 0x02,
    VIDEO_FGND_CYAN         = 0x03,
    VIDEO_FGND_RED          = 0x04,
    VIDEO_FGND_PURPLE       = 0x05,
    VIDEO_FGND_BROWN        = 0x06,
    VIDEO_FGND_LIGHT_GRAY   = 0x07,
    VIDEO_FGND_DARK_GRAY    = 0x08,
    VIDEO_FGND_LIGHT_BLUE   = 0x09,
    VIDEO_FGND_LIGHT_GREEN  = 0x0A,
    VIDEO_FGND_LIGHT_CYAN   = 0x0B,
    VIDEO_FGND_LIGHT_RED    = 0x0C,
    VIDEO_FGND_LIGHT_PURPLE = 0x0D,
    VIDEO_FGND_YELLOW       = 0x0E,
    VIDEO_FGND_WHITE        = 0x0F,
    /* background */
    VIDEO_BGND_BLACK        = 0x00,
    VIDEO_BGND_BLUE         = 0x10,
    VIDEO_BGND_GREEN        = 0x20,
    VIDEO_BGND_CYAN         = 0x30,
    VIDEO_BGND_RED          = 0x40,
    VIDEO_BGND_PURPLE       = 0x50,
    VIDEO_BGND_BROWN        = 0x60,
    VIDEO_BGND_LIGHT_GRAY   = 0x70,

    VIDEO_BGND_BLINK        = 0x80
};
/*..........................................................................*/
void Video_clearScreen(uint8_t bgColor);
void Video_clearRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                     uint8_t bgColor);
void Video_printChAt(uint8_t x, uint8_t y, uint8_t color, char ch);
void Video_printStrAt(uint8_t x, uint8_t y, uint8_t color,
                      char const *str);
void Video_printNumAt(uint8_t x, uint8_t y, uint8_t color, uint32_t num);

#endif                                                             /* bsp_h */

