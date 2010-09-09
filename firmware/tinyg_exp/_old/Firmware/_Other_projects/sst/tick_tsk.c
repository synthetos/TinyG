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

#include <stdlib.h>                                         /* for random() */

/*..........................................................................*/
void tickTaskA(SSTEvent e) {
    static uint32_t tickTaskACtr;
    static uint8_t colorA = VIDEO_BGND_LIGHT_GRAY;

    Video_printNumAt(22, 19 - TICK_TASK_A_PRIO,
                     VIDEO_FGND_YELLOW, ++tickTaskACtr);
    busyDelay();                                     /* for testing, NOTE01 */

    switch (e.sig) {
        case INIT_SIG: {
            Video_printStrAt( 1, 19 - TICK_TASK_A_PRIO,
                              VIDEO_FGND_WHITE, "tickTaskA");
            break;
        }
        case TICK_SIG: {
            uint8_t x, y;
            uint8_t mutex;

            mutex = SST_mutexLock(TICK_TASK_B_PRIO); /* the other tick task */
            x = random(34);
            y = random(13);
            SST_mutexUnlock(mutex);

            Video_printChAt(x + 43, y + 8, colorA, 'A');
            break;
        }
        case COLOR_SIG: {
            colorA = e.par;          /* color is delivered in the parameter */
            break;
        }
    }
}
/*..........................................................................*/
void tickTaskB(SSTEvent e) {
    static uint32_t tickTaskBCtr;
    static uint8_t colorB = VIDEO_BGND_LIGHT_GRAY;

    Video_printNumAt(22, 19 - TICK_TASK_B_PRIO,
                     VIDEO_FGND_YELLOW, ++tickTaskBCtr);
    busyDelay();                                     /* for testing, NOTE01 */

    switch (e.sig) {
        case INIT_SIG: {
            Video_printStrAt( 1, 19 - TICK_TASK_B_PRIO,
                              VIDEO_FGND_WHITE, "tickTaskB");
            break;
        }
        case TICK_SIG: {
            uint8_t x, y;
            uint8_t mutex;

            mutex = SST_mutexLock(TICK_TASK_A_PRIO); /* the other tick task */
            x = random(34);
            y = random(13);
            SST_mutexUnlock(mutex);

            Video_printChAt(x + 43, y + 8, colorB, 'B');
            break;
        }
        case COLOR_SIG: {
            colorB = e.par;          /* color is delivered in the parameter */
            break;
        }
    }
}

/*****************************************************************************
* NOTE01:
* The call to busyDelay() is added only to extend the execution time
* to increase the chance of an "asynchronous" preemption.
*/
