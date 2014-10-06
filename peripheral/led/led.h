#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"
/*	 
#define LED1_ON()	GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define LED1_OFF()	GPIO_ResetBits(GPIOD, GPIO_Pin_13)	 

#define LED2_ON()	GPIO_SetBits(GPIOG, GPIO_Pin_14)
#define LED2_OFF()	GPIO_ResetBits(GPIOG, GPIO_Pin_14)
*/

#define LED1	PBout(12)
#define LED2	PBout(13)
#define LED3	PBout(14)
#define LED4	PBout(15)
	 
#define MOTOR1	PBout(4)
#define MOTOR2	PBout(5)
#define MOTOR3	PBout(0)
#define MOTOR4	PBout(1)
	 
void led_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
