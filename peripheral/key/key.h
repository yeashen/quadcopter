#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"

#define KEY0	GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)
#define KEY1	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)

#define KEY0_PRESS	1
#define KEY1_PRESS	2	 
	 
void key_init(void);
u8 key_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
