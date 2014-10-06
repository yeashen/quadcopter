#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usmart.h"
#include "exint.h"
#include "timer.h"
#include "adc.h"
#include "led.h"
#include "pwm.h"
#include "24l01.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bmp180.h"
#include "hmc5883l.h"
#include "PID_Ctrl.h"
#include "value_struct.h"

////////////////////////////全局变量定义区-开始/////////////////////////////////

extern u8 recv_buf[3];
char pbuf[50];
u16 motor_pwm_val = 0;
bool NRFisRunning = FALSE;
extern u16 bat_vol;
extern u16 adc_value;

////////////////////////////全局变量定义区-结束/////////////////////////////////

////////////////////////////宏定义区-开始///////////////////////////////////////

////////////////////////////宏定义区-结束///////////////////////////////////////

void dev_init(void)
{
	static signed char gyro_orientation[9] = {-1, 0, 0,
                                           	   0,-1, 0,
                                               0, 0, 1};
	int result;
	
	delay_init();
	NVIC_Configuration();
	uart1_init(9600);
	led_init();
	//ADC初始化
	adc_init();

	//外部中断初始化
	//ExtInt_Init();
	
	//PWM初始化
	TIMER3_PWM_init(999, 141);	//72MHz/(999+1)/(141+1)=500Hz

	//定时器2初始化
	timer2_init(999, 71, 0);	//T = (999+1)*(71+1)/72MHz = 1ms

	PID_init();

	//NRF24L01初始化
	NRF24L01_Init();
	if(NRF24L01_Check()){
		printf("NRF24L01 error!\r\n");
		return;
	}else{
		printf("NRF24L01 check OK...\r\n");
		NRF24L01_TX_Mode();
	}
	
#if defined(SW_I2C)
	SwI2C_init();
#else 
	HwI2C_Init();
#endif
	delay_ms(100);

	//MPU6050初始化
	if(mpu6050_check() != TRUE){
		printf("Check error! Please check!\r\n");
		return ;
	}else{
		printf("Check OK.\r\n");
	}

	//mpu6050_init();
	result = mpu_init(); 
	printf("mpu init result:%d \r\n", result);

	if(!result){	 		 
		printf("mpu initialization complete......\n ");	 	  //mpu_set_sensor
		
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			printf("mpu_set_sensor complete ......\n");
		else
			printf("mpu_set_sensor come across error ......\n");
		
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	   	  //mpu_configure_fifo
			printf("mpu_configure_fifo complete ......\n");
		else
			printf("mpu_configure_fifo come across error ......\n");
		
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  //mpu_set_sample_rate
			printf("mpu_set_sample_rate complete ......\n");
		else
			printf("mpu_set_sample_rate error ......\n");
		
		if(!dmp_load_motion_driver_firmware())   	  //dmp_load_motion_driver_firmvare
			printf("dmp_load_motion_driver_firmware complete ......\n");
		else
			printf("dmp_load_motion_driver_firmware come across error ......\n");
		
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
			printf("dmp_set_orientation complete ......\n");
		else
			printf("dmp_set_orientation come across error ......\n");
		
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))		   	  //dmp_enable_feature
			printf("dmp_enable_feature complete ......\n");
		else
			printf("dmp_enable_feature come across error ......\n");
		
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	  //dmp_set_fifo_rate
			printf("dmp_set_fifo_rate complete ......\n");
		else
			printf("dmp_set_fifo_rate come across error ......\n");
	  
		run_self_test();
		if(!mpu_set_dmp_state(1))
			printf("mpu_set_dmp_state complete ......\n");
		else
			printf("mpu_set_dmp_state come across error ......\n");
	}
	delay_ms(100);
}

int main()
{
	

// TODO: for test NRF24L01 and Motor
#if 0
	//bool isFisrtBoot = TRUE;
	u16 t = 0;
	//u8 key_buf[16] = {'0','1','2','3','4','5','6','7','8','9','L','R','U','D','P','O'};
	//u16 motor_pwm_val = 300;
	u16 adc_value;
	u16 bat_vol;
	
	delay_init();
	NVIC_Configuration();
	TIMER3_PWM_init(999, 0);
	TIM_SetCompare1(TIM3, motor_pwm_val);
	TIM_SetCompare2(TIM3, motor_pwm_val);
	TIM_SetCompare3(TIM3, motor_pwm_val);
	TIM_SetCompare4(TIM3, motor_pwm_val);
	uart1_init(9600);
	//ExtInt_Init();
	led_init();
	//adc_init();
	NRF24L01_Init();
	while(NRF24L01_Check()){
		printf("NRF24L01 error!\r\n");
		delay_ms(500);
	}
	printf("NRF24L01 check OK...\r\n");
	printf("NRF24L01 RX_MODE\r\n");
	NRF24L01_RX_Mode();
	while(1){
		if(NRF24L01_RxPacket(recv_buf) == 0){
			recv_buf[1] = 0;
			if(recv_buf[0] == 'P'){		//启动电机
				isFisrtBoot = FALSE;
				motor_pwm_val = 300;
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
		delay_ms(500);
		t++;
		if(t == 2){
			NRF24L01_RX_Mode();
			t = 0;
			if(isFisrtBoot){
				LED1 = !LED1;
				LED2 = !LED2;
				LED3 = !LED3;
				LED4 = !LED4;
			}else{
				//adc_value = get_adc_avg(ADC_Channel_3, 2);
				//printf("adc=%d	", adc_value);
				//bat_vol = (float)adc_value * (3.3/4096) * 10;
				//printf("voltage=%d \r\n", bat_vol);
				//if(bat_vol < 15){
				//	isFisrtBoot = TRUE;
					/*
					motor_pwm_val = 0;
					TIM_SetCompare1(TIM3, motor_pwm_val);
					TIM_SetCompare2(TIM3, motor_pwm_val);
					TIM_SetCompare3(TIM3, motor_pwm_val);
					TIM_SetCompare4(TIM3, motor_pwm_val);
					*/
				}
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			LED4 = 1;
		}
	}
#endif

// TODO:for test MPU6050	
#if 0
	u8 t = 0;

	MPUData m_data;
	MPUCovData c_data;
	float angx, angy, angz;
	int16_t temp, realTemp;

	c_data.c_accel_offset.x = -0.03;
	c_data.c_accel_offset.y = 0.01;
	c_data.c_accel_offset.z = 0.08;
	
	delay_init();
	NVIC_Configuration();
	uart1_init(9600);
	led_init();
#if defined(SW_I2C)
	SwI2C_init();
#else 
	HwI2C_Init();
#endif
	delay_ms(100);

	while(mpu6050_check() != TRUE){
		printf("Check error! Please again\r\n");
		delay_ms(1000);
	}
	printf("Check OK.\r\n");

	mpu6050_init();
	
	while(1){
		t++;
		if(t == 4){
			LED1 = !LED1;
			mpu6050_getMotion6(&m_data.m_accel.x, &m_data.m_accel.y, &m_data.m_accel.z, \
				&m_data.m_gyro.x, &m_data.m_gyro.y, &m_data.m_gyro.z);
			temp = mpu6050_getTemperature();
			
			delay_ms(10);
			//intf("AX=%d AY=%d AZ=%d\r\n", m_data.m_accel.x, m_data.m_accel.y, m_data.m_accel.z);
			c_data.c_accel.x = m_data.m_accel.x * MPU6050_G_PER_LSB + c_data.c_accel_offset.x;
			c_data.c_accel.y = m_data.m_accel.y * MPU6050_G_PER_LSB + c_data.c_accel_offset.y;
			c_data.c_accel.z = m_data.m_accel.z * MPU6050_G_PER_LSB + c_data.c_accel_offset.z;

			c_data.c_gyro.x = m_data.m_gyro.x * MPU6050_DEG_PER_LSB;
			c_data.c_gyro.y = m_data.m_gyro.y * MPU6050_DEG_PER_LSB;
			c_data.c_gyro.z = m_data.m_gyro.z * MPU6050_DEG_PER_LSB;

			angx = mpu6050_getAngle(c_data.c_accel.x, c_data.c_accel.y, c_data.c_accel.z, AXIS_X);
			angy = mpu6050_getAngle(c_data.c_accel.x, c_data.c_accel.y, c_data.c_accel.z, AXIS_Y);
			angz = mpu6050_getAngle(c_data.c_accel.x, c_data.c_accel.y, c_data.c_accel.z, AXIS_Z);

			realTemp = temp / MPU6050_TEMP_PER_LSB;

			printf("\r\nax=%.2f ay=%.2f az=%.2f \r\n", c_data.c_accel.x, c_data.c_accel.y, c_data.c_accel.z);
			printf("gx=%.2f gy=%.2f gz=%.2f \r\n", c_data.c_gyro.x, c_data.c_gyro.y, c_data.c_gyro.z);
			printf("temp=%d realtemp=%d \r\n", temp, realTemp);
			printf("angx=%.2f angy=%.2f angz=%.2f\r\n", angx, angy, angz);
			LED1 = !LED1;
			t = 0;
		}
		delay_ms(500);
	}
#endif	

// TODO: for test MPU6050 DMP with others

#if 0
	#define q30  1073741824.0f
	static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
	u8 t = 0;
	int result;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	char num[50];
	float Pitch,Roll,Yaw;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	
	delay_init();
	NVIC_Configuration();
	uart1_init(9600);
	led_init();
#if defined(SW_I2C)
	SwI2C_init();
#else 
	HwI2C_Init();
#endif
	delay_ms(100);

	while(mpu6050_check() != TRUE){
		printf("Check error! Please again\r\n");
		delay_ms(1000);
	}
	printf("Check OK.\r\n");

	//mpu6050_init();
	result = mpu_init(); 
	printf("mpu init result:%d \r\n", result);

	if(!result){	 		 
		printf("mpu initialization complete......\n ");	 	  //mpu_set_sensor
		
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			printf("mpu_set_sensor complete ......\n");
		else
			printf("mpu_set_sensor come across error ......\n");
		
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	   	  //mpu_configure_fifo
			printf("mpu_configure_fifo complete ......\n");
		else
			printf("mpu_configure_fifo come across error ......\n");
		
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  //mpu_set_sample_rate
			printf("mpu_set_sample_rate complete ......\n");
		else
			printf("mpu_set_sample_rate error ......\n");
		
		if(!dmp_load_motion_driver_firmware())   	  //dmp_load_motion_driver_firmvare
			printf("dmp_load_motion_driver_firmware complete ......\n");
		else
			printf("dmp_load_motion_driver_firmware come across error ......\n");
		
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
			printf("dmp_set_orientation complete ......\n");
		else
			printf("dmp_set_orientation come across error ......\n");
		
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))		   	  //dmp_enable_feature
			printf("dmp_enable_feature complete ......\n");
		else
			printf("dmp_enable_feature come across error ......\n");
		
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	  //dmp_set_fifo_rate
			printf("dmp_set_fifo_rate complete ......\n");
		else
			printf("dmp_set_fifo_rate come across error ......\n");
	  
		run_self_test();
		if(!mpu_set_dmp_state(1))
			printf("mpu_set_dmp_state complete ......\n");
		else
			printf("mpu_set_dmp_state come across error ......\n");
	}
	delay_ms(100);
	
	while(1){
		t++;
		if(t == 1){
			LED1 = !LED1;
			result = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
			if (sensors & INV_WXYZ_QUAT ){
				q0 = quat[0] / q30;	
				q1 = quat[1] / q30;
				q2 = quat[2] / q30;
				q3 = quat[3] / q30;
				Pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
				Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
				Yaw = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
				sprintf(num,"Pitch:%.2f, Roll:%.2f, Yaw:%.2f",Pitch,Roll,Yaw);	
				printf(num);
				printf("\r\n");
				printf("ax=%.2f ay=%.2f az=%.2f\r\n", accel[0]* MPU6050_G_PER_LSB, \
						accel[1]* MPU6050_G_PER_LSB, accel[2]* MPU6050_G_PER_LSB);
				printf("gx=%.2f gy=%.2f gz=%.2f\r\n", gyro[0]* MPU6050_DEG_PER_LSB,\
						gyro[1]* MPU6050_DEG_PER_LSB, gyro[2]* MPU6050_DEG_PER_LSB);
			}
			LED1 = !LED1;
			t = 0;
		}
		LED2 = !LED2;
		delay_ms(500);
	}

#endif 

// TODO: for test BMP180
#if 0
	long temp, pres;
	u8 t = 0;
	
	delay_init();
	NVIC_Configuration();
	uart1_init(9600);
	led_init();
#if defined(SW_I2C)
	SwI2C_init();
#else 
	HwI2C_Init();
#endif
	delay_ms(100);
	while(1){
		t++;
		if(t == 4){
			t = 0;
			bmp180_getData(&temp, &pres);
			printf("temp = %dC pres= %d Pa\r\n", temp, pres);
			LED1 = !LED1;
		}
		delay_ms(500);
	}
#endif

// TODO: for test HMC5883L
#if 0
	u8 t = 0;
	int16_t mag_x, mag_y, mag_z;
	int16_t st_x, st_y, st_z;
	uint16_t angle;
	
	delay_init();
	NVIC_Configuration();
	uart1_init(9600);
	led_init();
#if defined(SW_I2C)
	SwI2C_init();
#else 
	HwI2C_Init();
#endif
	delay_ms(100);

	while(mpu6050_check() != TRUE){
		printf("Check error! Please again\r\n");
		delay_ms(1000);
	}
	printf("Check OK.\r\n");

	mpu6050_init();
	hmc_init();
	hmc_selt_test(&st_x, &st_y, &st_z);
	printf("self_test-x=%d y=%d z=%d\r\n", st_x, st_y, st_z);
	
	while(1){
		t++;
		if(t == 4){
			LED1 = !LED1;
			hmc_getMagnetic(&mag_x, &mag_y, &mag_z);
			angle= atan2((double)mag_y,(double)mag_x) * (180 / 3.14) + 180;
			printf("\r\n mag_x=%d mag_y=%d mag_z=%d\r\n", mag_x, mag_y, mag_z);
			printf("angle=%d\r\n", (uint16_t)angle);
			LED1 = !LED1;
			t = 0;
		}
		delay_ms(500);
	}	
#endif

// TODO: for All without data 
#if 0
	//bool isFisrtBoot = TRUE;
	u16 t = 0;
	//u8 key_buf[16] = {'0','1','2','3','4','5','6','7','8','9','L','R','U','D','P','O'};
	//u16 motor_pwm_val = 0;
	dev_init();

	TIM_Cmd(TIM2, ENABLE);
		
	while(1){	
		t++;
		if(t == 10){
			NRF24L01_RX_Mode();
			t = 0;
			if(isFisrtBoot){
				LED1 = !LED1;
				LED2 = !LED2;
				LED3 = !LED3;
				LED4 = !LED4;
			}else{
				if(!isStandby){
					LED4 = !LED4;
					adc_value = get_adc_avg(ADC_Channel_3, 1);
					bat_vol = (float)adc_value * (3.3/4096) * 1000;
					if(bat_vol < 15){
						isStandby = TRUE;
						isFisrtBoot = TRUE;
						TIM_SetCompare1(TIM3, 0);
						TIM_SetCompare2(TIM3, 0);
						TIM_SetCompare3(TIM3, 0);
						TIM_SetCompare4(TIM3, 0);
					}
					delay_ms(100);
				}
				LED1 = 1;
				LED2 = 1;
				LED3 = 1;
				LED4 = 1;
			}
		}
		delay_ms(100);
	}
#endif

// TODO: for All device data debug
#if 1
	dev_init();

	//TIM_Cmd(TIM2, ENABLE);
		
	while(1){	
		if(bat_vol < 15){
			TIM_Cmd(TIM3, DISABLE);
			while(1){
				LED1 = !LED1;
				LED2 = !LED2;
				LED3 = !LED3;
				LED4 = !LED4;
				delay_ms(500);
			}
		}
		
	}
#endif

}
