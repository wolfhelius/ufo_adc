#ifndef MY_ADC_H_
#define MY_ADC_H_

// Function Prototypes
void adc_init(void);

uint8_t ReadCalibrationByte(uint8_t index);
int16_t adc_compensate( int16_t adcvalue, int16_t factor, int32_t correction );

#endif /* MY_ADC_H_ */