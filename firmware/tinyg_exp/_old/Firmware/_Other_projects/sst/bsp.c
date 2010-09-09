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
#include "sst_port.h"
#include "sst_exa.h"
#include "bsp.h"

#include <stdlib.h>                                          /* for _exit() */

/* Local-scope objects -----------------------------------------------------*/
static void interrupt (*l_dosTickISR)();
static void interrupt (*l_dosKbdISR)();

#define TICKS_PER_SEC   200

#define TICK_VECTOR     0x08
#define KBD_VECTOR      0x09

static void displayPreemptions(uint8_t pprev, uint8_t pnext);

/*..........................................................................*/
static void interrupt tickISR() {
    uint8_t pin;
    displayPreemptions(SST_currPrio_, TICK_ISR_PRIO);/* for testing, NOTE01 */
    SST_ISR_ENTRY(pin, TICK_ISR_PRIO);

    SST_post(TICK_TASK_A_PRIO, TICK_SIG, 0);     /* post the Tick to Task A */
    SST_post(TICK_TASK_B_PRIO, TICK_SIG, 0);     /* post the Tick to Task B */

    busyDelay();                                 /* for testing, see NOTE02 */
    SST_ISR_EXIT(pin, outportb(0x20, 0x20));
}
/*..........................................................................*/
static void interrupt kbdISR() {
    uint8_t pin;
    uint8_t key = inport(0x60);/*get scan code from the 8042 kbd controller */

    displayPreemptions(SST_currPrio_, KBD_ISR_PRIO); /* for testing, NOTE01 */
    SST_ISR_ENTRY(pin, KBD_ISR_PRIO);

    SST_post(KBD_TASK_PRIO, KBD_SIG, key);   /* post the Key to the KbdTask */

    busyDelay();                                 /* for testing, see NOTE02 */
    SST_ISR_EXIT(pin, outportb(0x20, 0x20));
}
/*..........................................................................*/
void SST_init(void) {
}
/*..........................................................................*/
void SST_start(void) {
                                      /* divisor for the 8254 timer/counter */
    uint16_t count = (uint16_t)(((1193180 * 2) / TICKS_PER_SEC + 1) >> 1);

                                       /* save the original DOS vectors ... */
    l_dosTickISR = getvect(TICK_VECTOR);
    l_dosKbdISR  = getvect(KBD_VECTOR);

    SST_INT_LOCK();                                  /* lock the interrupts */
    outportb(0x43, 0x36);             /* use mode-3 for timer 0 in the 8254 */
    outportb(0x40, count & 0xFF);              /* load low  byte of timer 0 */
    outportb(0x40, (count >> 8) & 0xFF);       /* load high byte of timer 0 */
    setvect(TICK_VECTOR, &tickISR);
    setvect(KBD_VECTOR, &kbdISR);
    SST_INT_UNLOCK();                              /* unlock the interrupts */
}
/*..........................................................................*/
void SST_onIdle(void) {
    static uint32_t onIdleCtr;
    Video_printNumAt(22,  19, VIDEO_FGND_YELLOW, ++onIdleCtr);
}
/*..........................................................................*/
void SST_exit(void) {
    SST_INT_LOCK();                                  /* lock the interrupts */
    outportb(0x43, 0x36);             /* use mode-3 for timer 0 in the 8254 */
    outportb(0x40, 0);                         /* load low  byte of timer 0 */
    outportb(0x40, 0);                         /* load high byte of timer 0 */
                                    /* restore the original DOS vectors ... */
    setvect(TICK_VECTOR, l_dosTickISR);
    setvect(KBD_VECTOR, l_dosKbdISR);
    SST_INT_UNLOCK();                              /* unlock the interrupts */
    _exit(0);                                                /* exit to DOS */
}

/*--------------------------------------------------------------------------*/
void displayPreemptions(uint8_t pprev, uint8_t pnext) {
    if (pnext == TICK_ISR_PRIO) {
        static uint32_t tmrIsrCtr;               /* timer interrupt counter */
        Video_printNumAt(22,  7, VIDEO_FGND_YELLOW, ++tmrIsrCtr);
    }
    else if (pnext == KBD_ISR_PRIO) {
        static uint32_t kbdIsrCtr;                 /* kbd interrupt counter */
        Video_printNumAt(22,  8, VIDEO_FGND_YELLOW, ++kbdIsrCtr);
    }

    if (pprev == TICK_ISR_PRIO) {           /* is this Tick ISR preemption? */
        static uint32_t tickPreCtr;          /* Tick ISR preemption counter */
        Video_printNumAt(30,  7, VIDEO_FGND_YELLOW, ++tickPreCtr);
    }
    else if (pprev == KBD_ISR_PRIO) {        /* is this kbd ISR preemption? */
        static uint32_t kbdPreCtr;            /* kbd ISR preemption counter */
        Video_printNumAt(30,  8, VIDEO_FGND_YELLOW, ++kbdPreCtr);
    }
    else {                                     /* must be a task preemption */
        static uint32_t preCtr[SST_MAX_PRIO + 2];
        Video_printNumAt(30, 19 - pprev, VIDEO_FGND_YELLOW,
                         ++preCtr[pprev]);
    }
}
/*--------------------------------------------------------------------------*/
void Video_clearScreen(uint8_t bgColor) {
    clrscr();
    Video_clearRect(0,  0, 80, 25, bgColor);
}
/*..........................................................................*/
void Video_clearRect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
                     uint8_t bgColor)
{
    for ( ; y1 < y2; ++y1) {
        uint8_t x;
        uint8_t far *pscr = (uint8_t far *)MK_FP(0xB800,
                                           (uint16_t)(((y1 * 80) + x1) * 2));
        for (x = x1; x < x2; ++x) {
            pscr[0] = ' ';                    /* Put space in the video RAM */
            pscr[1] = bgColor;      /* Put video attribute in the video RAM */
            pscr += 2;
        }
    }
}
/*..........................................................................*/
void Video_printChAt(uint8_t x, uint8_t y, uint8_t color, char ch) {
                               /* calculate position on the video RAM (VGA) */
    uint8_t far *pscr = (uint8_t far *)MK_FP(0xB800,
                                             (uint16_t)(((y * 80) + x) * 2));
    pscr[0] = ch;
    pscr[1] = color;
}
/*..........................................................................*/
void Video_printStrAt(uint8_t x, uint8_t y, uint8_t color,
                      char const *str)
{
                               /* calculate position on the video RAM (VGA) */
    uint8_t far *pscr = (uint8_t far *)MK_FP(0xB800,
                                             (uint16_t)(((y * 80) + x) * 2));
    while (*str != (uint8_t)0) {
        pscr[0] = *str++;                     /* Put character in video RAM */
        pscr[1] |= color;               /* Put video attribute in video RAM */
        pscr += 2;
    }
}
/*..........................................................................*/
void Video_printNumAt(uint8_t x, uint8_t y, uint8_t color, uint32_t num) {
    char buf[4];
    buf[3] = (char)0;
    buf[2] = (char)('0' + num % 10);
    num /= 10;
    buf[1] = (char)('0' + num % 10);
    num /= 10;
    buf[0] = (char)('0' + num % 10);
    if (buf[0] == '0') {
        buf[0] = ' ';
    }
    Video_printStrAt(x, y, color, buf);
}

/*****************************************************************************
* NOTE01:
* The function call to displayPreemptions() is added only to monitor the
* "asynchronous" preemptions within the SST.
*
* NOTE02:
* The call to busyDelay() is added only to extend the execution time
* to increase the chance of an "asynchronous" preemption.
*/
