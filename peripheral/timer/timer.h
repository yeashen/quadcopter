#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"
	 
void timer2_init(u16 arr, u16 psc, u8 cmd);
void timer2_cap_init(u16 arr, u16 psc);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
