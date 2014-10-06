/******************************************************************************
  File			: timer.c
  Description	: timer test
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/8/5 21:00 PM	| Created
******************************************************************************/

#include "timer.h"
#include "led.h"
#include "delay.h"
#include "24l01.h"
#include "usart.h"
#include "adc.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "PID_Ctrl.h"
#include "value_struct.h"

#define q30  1073741824.0f

static int time_count = 0;
u8 NRF_TXBUF[32];

#define BYTE0(covTemp)		(*(char *)(&covTemp))
#define BYTE1(covTemp)		(*(char *)(&covTemp) + 1)
#define BYTE2(covTemp)		(*(char *)(&covTemp) + 2)
#define BYTE3(covTemp)		(*(char *)(&covTemp) + 3)

//MPUCovData c_data;

int16_t gyro[3], accel[3];
int16_t mag_x, mag_y, mag_z;
Angle quat_angle;
int16_t m_pitch, m_roll, m_yaw;
u16 adc_value;
u16 bat_vol;

unsigned long sensor_timestamp;
short sensors;
unsigned char more;
long quat[4];

/*-------------------------------------------------------------------
 * FUNC : timer2_init
 * DESC : timer 2 initial T=(arr+1)*(psc+1)/72MHz (us)
 * PARM : arr - reload value
 * 		  psc - clock pre
 *		  cmd - 1: enable timer 0: don't enable timer
 * RET	: N/A
 *-----------------------------------------------------------------*/
void timer2_init(u16 arr, u16 psc, u8 cmd)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	if(cmd)
		TIM_Cmd(TIM2, ENABLE);
} 

void timer2_cap_init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	timer2_init(arr, psc, 0);
	TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_CC1, ENABLE);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	
	TIM_Cmd(TIM2, ENABLE);
}


void data_load(void)
{
	u8 i = 0, sum = 0;
	
	NRF_TXBUF[0] = 0x88;	//帧头
	NRF_TXBUF[1] = 0xAF;//0xA1;	//FUN:功能地址
	NRF_TXBUF[2] = 0x1C;		//LEN:数据长度为28

#if 0
	c_data.c_accel.x = accel[0] * MPU6050_G_PER_LSB;
	c_data.c_accel.y = accel[1] * MPU6050_G_PER_LSB;
	c_data.c_accel.z = accel[2] * MPU6050_G_PER_LSB;
	c_data.c_gyro.x = gyro[0] * MPU6050_DEG_PER_LSB;
	c_data.c_gyro.y = gyro[1] * MPU6050_DEG_PER_LSB;
	c_data.c_gyro.z = gyro[2] * MPU6050_DEG_PER_LSB;

	NRF_TXBUF[3] = BYTE3(c_data.c_accel.x);	//X轴A数据
	NRF_TXBUF[4] = BYTE2(c_data.c_accel.x);		
	NRF_TXBUF[5] = BYTE1(c_data.c_accel.x);	
	NRF_TXBUF[6] = BYTE0(c_data.c_accel.x);	
	
	NRF_TXBUF[7] = BYTE3(c_data.c_accel.y);	//Y轴A数据
	NRF_TXBUF[8] = BYTE2(c_data.c_accel.y);	
	NRF_TXBUF[9] = BYTE1(c_data.c_accel.y);	
	NRF_TXBUF[10] = BYTE0(c_data.c_accel.y);	
	
	NRF_TXBUF[11] = BYTE3(c_data.c_accel.z);//Z轴A数据
	NRF_TXBUF[12] = BYTE2(c_data.c_accel.z);		
	NRF_TXBUF[13] = BYTE1(c_data.c_accel.z);	
	NRF_TXBUF[14] = BYTE0(c_data.c_accel.z);		

	NRF_TXBUF[15] = BYTE3(c_data.c_gyro.x);//X轴G数据
	NRF_TXBUF[16] = BYTE2(c_data.c_gyro.x);
	NRF_TXBUF[17] = BYTE1(c_data.c_gyro.x);
	NRF_TXBUF[18] = BYTE0(c_data.c_gyro.x);

	NRF_TXBUF[19] = BYTE3(c_data.c_gyro.y);//Y轴G数据
	NRF_TXBUF[20] = BYTE2(c_data.c_gyro.y);
	NRF_TXBUF[21] = BYTE1(c_data.c_gyro.y);
	NRF_TXBUF[22] = BYTE0(c_data.c_gyro.y);

	NRF_TXBUF[23] = BYTE3(c_data.c_gyro.z);//Z轴G数据
	NRF_TXBUF[24] = BYTE2(c_data.c_gyro.z);
	NRF_TXBUF[25] = BYTE1(c_data.c_gyro.z);
	NRF_TXBUF[26] = BYTE0(c_data.c_gyro.z);

	NRF_TXBUF[27] = BYTE3(bat_vol);		//电池电压
	NRF_TXBUF[28] = BYTE2(bat_vol);
	NRF_TXBUF[29] = BYTE1(bat_vol);
	NRF_TXBUF[30] = BYTE0(bat_vol);
#else
	NRF_TXBUF[3] = accel[0] >> 8;
	NRF_TXBUF[4] = accel[0];
	NRF_TXBUF[5] = accel[1] >> 8;
	NRF_TXBUF[6] = accel[1];
	NRF_TXBUF[7] = accel[2] >> 8;
	NRF_TXBUF[8] = accel[2];

	NRF_TXBUF[9] = gyro[0] >> 8;
	NRF_TXBUF[10] = gyro[0];
	NRF_TXBUF[11] = gyro[1] >> 8;
	NRF_TXBUF[12] = gyro[1];
	NRF_TXBUF[13] = gyro[2] >> 8;
	NRF_TXBUF[14] = gyro[2];

	NRF_TXBUF[15] = mag_x >> 8;
	NRF_TXBUF[16] = mag_x;
	NRF_TXBUF[17] = mag_y >> 8;
	NRF_TXBUF[18] = mag_y;
	NRF_TXBUF[19] = mag_z >> 8;
	NRF_TXBUF[20] = mag_z;

	NRF_TXBUF[21] = m_roll >> 8;
	NRF_TXBUF[22] = m_roll;
	NRF_TXBUF[23] = m_pitch >> 8;
	NRF_TXBUF[24] = m_pitch;
	NRF_TXBUF[25] = m_yaw >> 8;
	NRF_TXBUF[26] = m_yaw;
	
	NRF_TXBUF[27] = 0x0;
	NRF_TXBUF[28] = 0x0;

	NRF_TXBUF[29] = bat_vol >> 8;		//电池电压
	NRF_TXBUF[30] = bat_vol;

#endif
	
	for(i = 0; i < 31; i++)
		sum += NRF_TXBUF[i];
	
	NRF_TXBUF[31] = sum;			//SUM:校验和
}

void data_handle()
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	bat_vol = (float)adc_value * (3.3/4096) * 1000;
	if(sensors & INV_WXYZ_QUAT ){
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		quat_angle.pitch = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
		quat_angle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
		quat_angle.yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		m_pitch = quat_angle.pitch * 100;
		m_roll = quat_angle.roll * 100;
		m_yaw = quat_angle.yaw * 10;
	}
}

void TIM2_IRQHandler(void)
{
	time_count++;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		
		//10ms 定时任务
		if(time_count == 10){	
			dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
			//hmc_getMagnetic(&mag_x, &mag_y, &mag_z);
			adc_value = get_adc_avg(ADC_Channel_3, 1);
			data_handle();
			PID_Ctrl(&quat_angle, gyro);
#ifdef DATA_MONITOR
			data_load();
			if(NRF24L01_TxPacket(NRF_TXBUF, TX_PLOAD_WIDTH) == TX_OK){
				LED2 = !LED2;
			}
#endif
			time_count = 0;
		}
		
		//NRF24L01_Check_Event();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

