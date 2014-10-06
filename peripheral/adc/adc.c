/******************************************************************************
  File			: adc.c
  Description	: adc test
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/8/8 21:00 PM	| Created
******************************************************************************/

#include "adc.h"
#include "delay.h"

/*-------------------------------------------------------------------
 * FUNC : adc_init
 * DESC : adc initial
 * PARM : N/A
 * RET	: N/A
 *-----------------------------------------------------------------*/
void adc_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);	//分频 72M/6=12M
	
	//PA.1作为模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);	//开启复位校准
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1); //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));
}

/*-------------------------------------------------------------------
 * FUNC : get_adc
 * DESC : get adc value
 * PARM : ch - convert which channel
 * RET	: res - convert adc value
 *-----------------------------------------------------------------*/
u16 get_adc(u8 ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

/*-------------------------------------------------------------------
 * FUNC : get_adc_avg
 * DESC : get average adc value
 * PARM : ch - convert which channel
 * 		  n  - convert count 
 * RET	: res - convert average adc value with n times
 *-----------------------------------------------------------------*/
u16 get_adc_avg(u8 ch, u8 n)
{
	u32 temp = 0;
	u8 t;
	for(t = 0; t < n; t++){
		temp += get_adc(ch);
		delay_ms(5);
	}
	return temp/n;
}
