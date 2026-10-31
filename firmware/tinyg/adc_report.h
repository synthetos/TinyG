#ifndef ADC_REPORT_H_ONCE
#define ADC_REPORT_H_ONCE
void adc_init(void);
stat_t adc_read_raw(int16_t *sample);
stat_t hw_get_adc3(nvObj_t *nv);
stat_t adc_report(void);
#endif
