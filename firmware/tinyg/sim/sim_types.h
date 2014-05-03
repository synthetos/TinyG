#ifndef SIM_SIM_TYPES_H_ONCE
#define SIM_SIM_TYPES_H_ONCE

typedef struct {
  uint32_t unused;
} USART_t;

typedef struct PORT_struct
{
    uint8_t DIR;  /* I/O Port Data Direction */
    uint8_t DIRSET;  /* I/O Port Data Direction Set */
    uint8_t DIRCLR;  /* I/O Port Data Direction Clear */
    uint8_t DIRTGL;  /* I/O Port Data Direction Toggle */
    uint8_t OUT;  /* I/O Port Output */
    uint8_t OUTSET;  /* I/O Port Output Set */
    uint8_t OUTCLR;  /* I/O Port Output Clear */
    uint8_t OUTTGL;  /* I/O Port Output Toggle */
    uint8_t IN;  /* I/O port Input */
    uint8_t INTCTRL;  /* Interrupt Control Register */
    uint8_t INT0MASK;  /* Port Interrupt 0 Mask */
    uint8_t INT1MASK;  /* Port Interrupt 1 Mask */
    uint8_t INTFLAGS;  /* Interrupt Flag Register */
    uint8_t reserved_0x0D;
    uint8_t reserved_0x0E;
    uint8_t reserved_0x0F;
    uint8_t PIN0CTRL;  /* Pin 0 Control Register */
    uint8_t PIN1CTRL;  /* Pin 1 Control Register */
    uint8_t PIN2CTRL;  /* Pin 2 Control Register */
    uint8_t PIN3CTRL;  /* Pin 3 Control Register */
    uint8_t PIN4CTRL;  /* Pin 4 Control Register */
    uint8_t PIN5CTRL;  /* Pin 5 Control Register */
    uint8_t PIN6CTRL;  /* Pin 6 Control Register */
    uint8_t PIN7CTRL;  /* Pin 7 Control Register */
} PORT_t;

#endif // SIM_SIM_TYPES_H_ONCE
