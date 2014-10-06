/******************************************************************************
  File			: remote_ctrl.c
  Description	: remote data analyze
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/9/15 21:00 PM	| Created
******************************************************************************/

#include "remote_ctrl.h"

int16_t throttle = 0;

void RemoteData_Handle(u8 *data_buf,u8 num)
{
	u8 i, sum = 0;
	
	for(i = 0; i < (num-1); i++)
		sum += *(data_buf + i);
	if(!(sum==*(data_buf+num-1)))		return;		//ÅÐ¶Ïsum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//ÅÐ¶ÏÖ¡Í·
	if(*(data_buf+2) == 0x01){
		throttle = (*(data_buf+3)<<8 + *(data_buf+4));
	}
	printf("Rev: %d\r\n", throttle);
}
