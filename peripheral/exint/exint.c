/******************************************************************************
  File			: exint.c
  Description	: external interrupt test
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/8/4 21:00 PM	| Created
******************************************************************************/

#include "exint.h"
#include "delay.h"
#include "led.h"
#include "24l01.h"
#include "usart.h"

u8 recv_buf[3];
//extern bool recv_flag;
extern u16 motor_pwm_val;
extern bool isFisrtBoot;

/*-------------------------------------------------------------------
 * FUNC : ExtInt_Init
 * DESC : external interrupt initial
 * PARM : N/A
 * RET	: N/A
 *-----------------------------------------------------------------*/
void ExtInt_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	//外部引脚PA.2中断配置
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI2_IRQHandler(void)
{
/*
	if(NRF24L01_RxPacket(recv_buf) == 0){
		recv_buf[1] = 0;
		if(recv_buf[0] == 'L' || recv_buf[0] == 'R'){
			if(recv_buf[0] == 'L'){
				TIM_SetCompare1(TIM3, 0);
				TIM_SetCompare2(TIM3, 0);
			}else{
				TIM_SetCompare3(TIM3, 0);
				TIM_SetCompare4(TIM3, 0);
			}
		}
		else{
			if(recv_buf[0] == 'P'){		//启动电机
				isFisrtBoot = FALSE;
				motor_pwm_val = 800;
			}
			if(recv_buf[0] == 'O'){		//关闭电机
				isFisrtBoot = TRUE;
				motor_pwm_val = 0;
			}
			if((recv_buf[0] == 'U') || (recv_buf[0] == 'D')){
				if(motor_pwm_val > 999)
					motor_pwm_val = 999;
				if(motor_pwm_val <= 0)
					motor_pwm_val = 0;
				if((recv_buf[0] == 'U')){
					motor_pwm_val += 50;
					LED1 = !LED1;
				}else if(recv_buf[0] == 'D'){
					motor_pwm_val -= 50;
					LED2 = !LED2;
				}
			}
			TIM_SetCompare1(TIM3, motor_pwm_val);
			TIM_SetCompare2(TIM3, motor_pwm_val);
			TIM_SetCompare3(TIM3, motor_pwm_val);
			TIM_SetCompare4(TIM3, motor_pwm_val);
		}
	}
	*/
	EXTI_ClearITPendingBit(EXTI_Line2);
}

