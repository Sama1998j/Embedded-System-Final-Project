#ifndef ADC_H
#define ADC_H

/* IR sensor usable range. Exposed so the obstacle threshold can be validated
 * against it. */
#define ADC_DIST_MIN_CM 10
#define ADC_DIST_MAX_CM 150

void adc_init(void);
void adc_update(void);          // run one scan and cache both raw results
double adc_read_battery(void);
int adc_read_distance(void);

#endif
