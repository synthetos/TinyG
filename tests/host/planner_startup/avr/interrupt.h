#ifndef HOST_INTERRUPT_H
#define HOST_INTERRUPT_H
#include <stdint.h>
extern uint8_t SREG;
#define cli() (SREG &= (uint8_t)~128)
#endif
