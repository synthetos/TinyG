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

#include <stdlib.h>                                           /* for atol() */

static void setupScreen(void);
static SSTEvent tickTaskAQueue[2];
static SSTEvent tickTaskBQueue[2];
static SSTEvent kbdTaskQueue[2];

static uint32_t l_delayCtr = 0UL;

/*..........................................................................*/
int main(int argc, char *argv[]) {

    if (argc > 1) {                                     /* delay specified? */
        l_delayCtr = atol(argv[1]);
    }

    setupScreen();                                      /* setup the screen */

    SST_init();                                       /* initialize the SST */

    SST_task(&tickTaskA, TICK_TASK_A_PRIO,
            tickTaskAQueue, sizeof(tickTaskAQueue)/sizeof(tickTaskAQueue[0]),
            INIT_SIG, 0);

    SST_task(&tickTaskB, TICK_TASK_B_PRIO,
            tickTaskBQueue, sizeof(tickTaskBQueue)/sizeof(tickTaskBQueue[0]),
            INIT_SIG, 0);

    SST_task(&kbdTask, KBD_TASK_PRIO,
             kbdTaskQueue, sizeof(kbdTaskQueue)/sizeof(kbdTaskQueue[0]),
             INIT_SIG, 0);

    SST_run();                                   /* run the SST application */
    return 0;
}
/*..........................................................................*/
static void setupScreen(void) {
    Video_clearScreen(VIDEO_BGND_BLACK);
    Video_clearRect( 0,  5, 80,  6, VIDEO_BGND_LIGHT_GRAY);
    Video_clearRect( 0,  6, 40, 23, VIDEO_BGND_BLUE);
    Video_clearRect(40,  6, 80, 23, VIDEO_BGND_RED);
    Video_clearRect(43,  8, 77, 21, VIDEO_BGND_BLACK);
    Video_clearRect( 0, 23, 80, 24, VIDEO_BGND_LIGHT_GRAY);

    Video_printStrAt(31,  2, VIDEO_FGND_YELLOW,
                     "Super Simple Tasker");

    Video_printStrAt( 0,  5, VIDEO_FGND_BLUE,
                     " Task      Priority  Calls  Preemptions");
    Video_printStrAt( 1,  7, VIDEO_FGND_WHITE,  "tickISR   255");
    Video_printStrAt( 1,  8, VIDEO_FGND_WHITE,  "kbdISR    254");

    Video_printStrAt( 1, 10, VIDEO_FGND_WHITE,  "schedLock   9");
    Video_printStrAt( 1, 11, VIDEO_FGND_WHITE,  "task8       8");
    Video_printStrAt( 1, 12, VIDEO_FGND_WHITE,  "task7       7");
    Video_printStrAt( 1, 13, VIDEO_FGND_WHITE,  "task6       6");
    Video_printStrAt( 1, 14, VIDEO_FGND_WHITE,  "task5       5");
    Video_printStrAt( 1, 15, VIDEO_FGND_WHITE,  "task4       4");
    Video_printStrAt( 1, 16, VIDEO_FGND_WHITE,  "task3       3");
    Video_printStrAt( 1, 17, VIDEO_FGND_WHITE,  "task2       2");
    Video_printStrAt( 1, 18, VIDEO_FGND_WHITE,  "task1       1");
    Video_printStrAt( 1, 19, VIDEO_FGND_WHITE,  "idle Loop   0");

    Video_printStrAt(4, 23, VIDEO_FGND_BLUE,
         "* Copyright (c) 2006 Quantum Leaps, LLC "
         "* www.quantum-leaps.com *");
    Video_printStrAt(28, 24, VIDEO_FGND_LIGHT_RED,
         "<< Press Esc to quit >>");
}
/*..........................................................................*/
void busyDelay(void) {
    volatile uint32_t i = l_delayCtr;
    while (i-- > 0) {
    }
}
