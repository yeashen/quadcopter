#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h" 
	 
void adc_init(void);
u16 get_adc(u8 ch);
u16 get_adc_avg(u8 ch, u8 n);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
