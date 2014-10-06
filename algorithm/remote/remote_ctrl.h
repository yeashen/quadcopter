#ifndef __REMOTE_CTRL_H
#define __REMOTE_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"

void RemoteData_Handle(u8 *data_buf,u8 num);	 

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_CTRL_H */
