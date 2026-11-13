/* Separate instrumented images only; no profiling code in release or hooks. */
#ifndef TINYG_MOTION_PROFILE_H
#define TINYG_MOTION_PROFILE_H
#if defined(TINYG_MOTION_PROFILE) && defined(__AVR__)
#include <avr/io.h>
#include <stdint.h>
typedef struct {
    uint32_t calls, overflows;
    uint16_t maximum_ticks;
    uint8_t site, section_at_maximum;
} motionProfile_t;
extern volatile motionProfile_t motion_profile;
static inline void motion_profile_begin(void)
{
    TCC1.CTRLA = TC_CLKSEL_OFF_gc;
    TCC1.CTRLB = 0; /* no compare outputs, normal count */
    TCC1.INTCTRLA = 0; TCC1.INTCTRLB = 0;
    TCC1.PER = 65535; TCC1.CNT = 0;
    TCC1.INTFLAGS = TC1_OVFIF_bm;
    TCC1.CTRLA = TC_CLKSEL_DIV8_gc;
}
static inline void motion_profile_end(uint8_t section)
{
    TCC1.CTRLA = TC_CLKSEL_OFF_gc;
    uint16_t ticks = TCC1.CNT;
    uint8_t overflow = TCC1.INTFLAGS & TC1_OVFIF_bm;
    motion_profile.calls++;
    motion_profile.site = TINYG_MOTION_PROFILE;
    if (overflow) { motion_profile.overflows++; ticks = UINT16_MAX; }
    if (motion_profile.calls == 1 || ticks > motion_profile.maximum_ticks) {
        motion_profile.maximum_ticks = ticks;
        motion_profile.section_at_maximum = section;
    }
}
#endif
#endif
