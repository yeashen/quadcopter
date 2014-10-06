/******************************************************************************
  File			: pwm.c
  Description	: pwm test
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/8/4 21:00 PM	| Created
******************************************************************************/

#include "pwm.h"

/*-------------------------------------------------------------------
 * FUNC : TIMER1_PWM_init
 * DESC : timer1 pwm initial, Fout=72MHz/(arr+1)/(psc+1)
 * PARM : arr - reload value
 *		  psc - clock pre
 * RET	: N/A
 *-----------------------------------------------------------------*/
void TIMER3_PWM_init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	
	//PB.4 PB.5 PB.0 PB.1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*
	//PB.4 PB.5 PB.0 PB.1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); 

	/*
	
	*/
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	//TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	TIM_SetCompare1(TIM3, 0);
	TIM_SetCompare2(TIM3, 0);
	TIM_SetCompare3(TIM3, 0);
	TIM_SetCompare4(TIM3, 0);
}

/*-------------------------------------------------------------------
 * FUNC : TIMER3_PWM_Refresh
 * DESC : timer1 pwm value refresh
 * PARM : new_PWM_Value - new PWM value
 * RET	: N/A
 *-----------------------------------------------------------------*/
Motor_PWM PWM_Value;

void TIMER3_PWM_Refresh(Motor_PWM *new_PWM_Value)
{
	if(new_PWM_Value->pwm1 > PWM_MAX_VALUE) PWM_Value.pwm1 = PWM_MAX_VALUE;
	if(new_PWM_Value->pwm2 > PWM_MAX_VALUE) PWM_Value.pwm2 = PWM_MAX_VALUE;
	if(new_PWM_Value->pwm3 > PWM_MAX_VALUE) PWM_Value.pwm3 = PWM_MAX_VALUE;
	if(new_PWM_Value->pwm4 > PWM_MAX_VALUE) PWM_Value.pwm4 = PWM_MAX_VALUE;
	if(new_PWM_Value->pwm1 < 0)	PWM_Value.pwm1 = 0;
	if(new_PWM_Value->pwm2 < 0)	PWM_Value.pwm2 = 0;
	if(new_PWM_Value->pwm3 < 0)	PWM_Value.pwm3 = 0;
	if(new_PWM_Value->pwm4 < 0)	PWM_Value.pwm4 = 0;
	TIM_SetCompare1(TIM3, PWM_Value.pwm1);
	TIM_SetCompare2(TIM3, PWM_Value.pwm2);
	TIM_SetCompare3(TIM3, PWM_Value.pwm3);
	TIM_SetCompare4(TIM3, PWM_Value.pwm4);
}

