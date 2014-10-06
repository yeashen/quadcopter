/******************************************************************************
  File			: key.c
  Description	: key test
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/8/4 21:00 PM	| Created
******************************************************************************/

#include "key.h"
#include "delay.h"

/*-------------------------------------------------------------------
 * FUNC : key_init
 * DESC : key initial
 * PARM : N/A
 * RET	: N/A
 *-----------------------------------------------------------------*/
void key_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*-------------------------------------------------------------------
 * FUNC : key_scan
 * DESC : key scan press
 * PARM : N/A
 * RET	: res - press key value 
 *-----------------------------------------------------------------*/
u8 key_scan(void)
{
	if(KEY0 == 0||KEY1 == 0){
		delay_ms(10);
		if(KEY0 == 0)
			return KEY0_PRESS;
		else if(KEY1 == 0)
			return KEY1_PRESS;
	}
	return 0;
}
