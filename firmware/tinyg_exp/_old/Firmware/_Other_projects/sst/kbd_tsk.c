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

/*..........................................................................*/
void kbdTask(SSTEvent e) {
    static uint32_t kbdTaskCtr;

    Video_printNumAt(22, 19 - KBD_TASK_PRIO, VIDEO_FGND_YELLOW, ++kbdTaskCtr);
    busyDelay();                                     /* for testing, NOTE01 */

    switch (e.sig) {
        case INIT_SIG: {
            Video_printStrAt( 1, 19 - KBD_TASK_PRIO,
                             VIDEO_FGND_WHITE, "kbdTask");
            break;
        }
        case KBD_SIG: {
            if (e.par == (SSTParam)0x81) {          /* is this the ESC key? */
                SST_exit();
            }
            else if ((e.par & 1) != 0) {      /* pick one of the Tick Tasks */
                SST_post(TICK_TASK_A_PRIO,     /* no synchronous preemption */
                         COLOR_SIG, e.par & 0xF);
            }
            else {
                SST_post(TICK_TASK_B_PRIO,        /* synchronous preemption */
                         COLOR_SIG, (e.par & 0xF));
            }
            break;
        }
    }
}

/*****************************************************************************
* NOTE01:
* The call to busyDelay() is added only to extend the execution time
* to increase the chance of an "asynchronous" preemption.
*/
