/*****************************************************************************
* SST platform-independent public interface
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
#ifndef sst_h
#define sst_h

#include <stdint.h>                 /* exact-width integer types, ANSI C'99 */

typedef uint8_t SSTSignal;
typedef uint8_t SSTParam;

typedef struct SSTEventTag SSTEvent;
struct SSTEventTag {
    SSTSignal sig;
    SSTParam  par;
};

typedef void (*SSTTask)(SSTEvent e);

void SST_init(void);
void SST_task(SSTTask task, uint8_t prio, SSTEvent *queue, uint8_t qlen,
              SSTSignal sig, SSTParam  par);
void SST_start(void);
void SST_run(void);
void SST_onIdle(void);
void SST_exit(void);

uint8_t SST_post(uint8_t prio, SSTSignal sig, SSTParam  par);

uint8_t SST_mutexLock(uint8_t prioCeiling);
void SST_mutexUnlock(uint8_t orgPrio);

void SST_schedule_(void);

                                            /* SST interrupt entry and exit */
#define SST_ISR_ENTRY(pin_, isrPrio_) do { \
    (pin_) = SST_currPrio_; \
    SST_currPrio_ = (isrPrio_); \
    SST_INT_UNLOCK(); \
} while (0)

#define SST_ISR_EXIT(pin_, EOI_command_) do { \
    SST_INT_LOCK(); \
    (EOI_command_); \
    SST_currPrio_ = (pin_); \
    SST_schedule_(); \
} while (0)


/* public-scope objects */
extern uint8_t SST_currPrio_;     /* current priority of the executing task */
extern uint8_t SST_readySet_;                              /* SST ready-set */

#endif                                                             /* sst_h */
