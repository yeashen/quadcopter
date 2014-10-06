#ifndef __EXINT_H
#define __EXINT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"
	 
void ExtInt_Init(void);
void EXTI0_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXINT_H */
