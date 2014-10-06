#ifndef __SPI_H
#define __SPI_H

#include "sys.h"
#include "usart.h"
 				  
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�

#define SPI_PDEBUG
#ifdef SPI_PDEBUG
#define SPI_DEBUG(string,args...)	printf("[SPI_DEBUG] ");	\
									printf(string, ##args)
#else
#define SPI_DEBUG(string,args...)
#endif

#endif

