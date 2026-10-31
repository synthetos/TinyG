#include "tinyg.h"
#include "config.h"
#include "stack_report.h"
#ifdef __AVR
#include <avr/io.h>
#endif

#define STACK_PAINT 0xa5

static uint16_t _stack_unused(const volatile uint8_t *start, const volatile uint8_t *limit)
{
    uint16_t unused = 0;
    while (((uintptr_t)start < (uintptr_t)limit) && (*start == STACK_PAINT)) {
        start++;
        unused++;
    }
    return unused;
}

#ifdef __AVR
extern uint8_t __heap_start;
static uint16_t lowest_plus_one;

// .init3 runs after SP setup and before .data/.bss initialization or any C call frames.
// Use only assembly, and fall through to .init4 rather than returning.
void stack_paint_boot(void) __attribute__((naked, used, section(".init3")));
void stack_paint_boot(void)
{
    __asm__ __volatile__(
        "ldi r30, lo8(__heap_start)\n\t"
        "ldi r31, hi8(__heap_start)\n\t"
        "ldi r26, lo8(%1)\n\t"
        "ldi r27, hi8(%1)\n\t"
        "ldi r24, %0\n\t"
        "1: cp r30, r26\n\t"
        "cpc r31, r27\n\t"
        "brsh 2f\n\t"
        "st Z+, r24\n\t"
        "rjmp 1b\n\t"
        "2:\n\t"
        : : "M" (STACK_PAINT), "n" (RAMEND + 1)
        : "r24", "r26", "r27", "r30", "r31"
    );
}
#endif

stat_t hw_get_fstk(nvObj_t *nv)
{
#ifdef __AVR
    // An interrupt may deepen the stack during a scan; this is an observed watermark,
    // not a proven bound. A stack byte equal to the paint can also hide prior use.
    uint16_t unused = _stack_unused(&__heap_start, (const volatile uint8_t *)(uintptr_t)SP);
    if ((lowest_plus_one == 0) || (unused + 1 < lowest_plus_one)) {
        lowest_plus_one = unused + 1;
    }
    nv->value_int = lowest_plus_one - 1;
    nv->valuetype = TYPE_INTEGER;
    return STAT_OK;
#else
    (void)nv;
    return STAT_COMMAND_NOT_ACCEPTED;
#endif
}
