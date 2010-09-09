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
#ifndef sst_exa_h
#define sst_exa_h

void tickTaskA(SSTEvent e);
void tickTaskB(SSTEvent e);
void kbdTask(SSTEvent e);

enum Events {                         /* the events used in the application */
    INIT_SIG,                                       /* initialization event */
    TICK_SIG,
    KBD_SIG,
    COLOR_SIG
};

enum SSTPriorities {     /* the SST priorities don't need to be consecutive */
    /* task priorities... */
    TICK_TASK_A_PRIO = 2,
    KBD_TASK_PRIO    = 5,
    TICK_TASK_B_PRIO = 7,

    /* ISR priorities... */
    KBD_ISR_PRIO     = 0xFF - 1,
    TICK_ISR_PRIO    = 0xFF
};

#endif                                                         /* sst_exa_h */

