/* Raw PB3 (SS2 connector) ADC reporting; no temperature or voltage calibration. */
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "adc_report.h"
#ifdef __AVR
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stddef.h>
#include "xio.h"
#if XIO_DEV_SPI_COUNT > 1
#error ADC3 owns PB3; SPI device 2 cannot be enabled
#endif

#define ADC_POLL_LIMIT 1024

void adc_init(void)
{
    // Called once at boot before interrupts. Preserve the NVM command register.
    uint8_t command = NVM.CMD;
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    ADCB.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0));
    ADCB.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1));
    NVM.CMD = command;
    PORTB.DIRCLR = PIN3_bm;
    PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PR.PRPB &= ~PR_ADC_bm;
    ADCB.CTRLA = 0;
    // A3 errata forbids unsigned mode; use signed single-ended, no gain stage.
    ADCB.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
    ADCB.PRESCALER = ADC_PRESCALER_DIV32_gc;
    ADCB.REFCTRL = ADC_REFSEL_VCC_gc;    // VCC/1.6, about 2.06 V with a 3.3 V supply
    ADCB.EVCTRL = 0;
    ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;
    ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
    ADCB.CH0.INTCTRL = ADC_CH_INTLVL_OFF_gc;
    ADCB.CH0.INTFLAGS = ADC_CH_CHIF_bm;
    ADCB.CTRLA = ADC_ENABLE_bm;
    _delay_us(32);                     // reference startup: max 24 ADC clocks at 1 MHz
}

static stat_t _adc_wait_ready(const volatile uint8_t *flags)
{
    for (uint16_t remaining = ADC_POLL_LIMIT; remaining != 0; remaining--) {
        if (*flags & ADC_CH_CHIF_bm) { return STAT_OK; }
    }
    return STAT_COMMAND_NOT_ACCEPTED;
}

stat_t adc_read_raw(int16_t *sample)
{
    ADCB.CH0.INTFLAGS = ADC_CH_CHIF_bm;
    ADCB.CH0.CTRL |= ADC_CH_START_bm;
    ritorno(_adc_wait_ready(&ADCB.CH0.INTFLAGS));
    *sample = (int16_t)ADCB.CH0.RES;
    return STAT_OK;
}
#else
void adc_init(void) {}
stat_t adc_read_raw(int16_t *sample) { (void)sample; return STAT_COMMAND_NOT_ACCEPTED; }
#endif

stat_t hw_get_adc3(nvObj_t *nv)
{
    int16_t sample;
    ritorno(adc_read_raw(&sample));
    // TYPE_INTEGER is unsigned; preserve the signed count using an integral float.
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 0;
    nv->value_flt = sample;
    return STAT_OK;
}

stat_t adc_report(void)
{
    int16_t sample;
    ritorno(adc_read_raw(&sample));
    if (cs.comm_mode == TEXT_MODE) {
        fprintf_P(stderr, PSTR("ADC3:%d\n"), (int)sample);
    } else {
        char token[5];
        strcpy_P(token, PSTR("adc3"));
        nvObj_t *nv = nv_add_float(token, sample);
        if (nv == NULL) { return STAT_JSON_TOO_MANY_PAIRS; }
        nv->precision = 0;
        nv->depth = 1;
        nv->group[0] = NUL;
    }
    return STAT_OK;
}
