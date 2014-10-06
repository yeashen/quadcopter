TODO:  test UCOS-II
#if SYSTEM_SUPPORT_UCOS
#include "includes.h" 

/////////////////////////UCOSII ��������///////////////////////////////////
//START ����
#define START_TASK_PRIO      			10 //�����������ȼ�

#define START_STK_SIZE  				64 //���������ջ��С

OS_STK START_TASK_STK[START_STK_SIZE];	//�����ջ	

void start_task(void *pdata);		//������
 			   
//��������
#define MEASURE_TASK_PRIO       			7 //�����������ȼ�

#define MEASURE_STK_SIZE  		    		64 //���������ջ��С

OS_STK MEASURE_TASK_STK[MEASURE_STK_SIZE];	//�����ջ


void measure_task(void *pdata);	    //������

//��������
#define SEND_TASK_PRIO       			6 	//�����������ȼ�

#define SEND_STK_SIZE  					64	//���������ջ��С

OS_STK SEND_TASK_STK[SEND_STK_SIZE];	//�����ջ

void send_task(void *pdata);		//������

////////////////////////////ȫ�ֱ���������-��ʼ/////////////////////////////////
float Pitch,Roll,Yaw;
short gyro[3], accel[3];
char pbuf[50];
char recv_buf[2];

u16 motor_pwm_val = 0;
bool NRFisRunning = FALSE;
bool isFisrtBoot = TRUE;

////////////////////////////ȫ�ֱ���������-����/////////////////////////////////

////////////////////////////�궨����-��ʼ///////////////////////////////////////
#define q30  1073741824.0f

////////////////////////////�궨����-����///////////////////////////////////////

int main(void)
{	
	delay_init();	    	
	NVIC_Configuration();	 
	led_init();
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//??????
	OSStart();	
}
	  
//��ʼ����
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 

	dev_init();
  	OS_ENTER_CRITICAL();			  
 	OSTaskCreate(measure_task,(void *)0,(OS_STK*)&MEASURE_TASK_STK[MEASURE_STK_SIZE-1],MEASURE_TASK_PRIO);						   
 	OSTaskCreate(send_task,(void *)0,(OS_STK*)&SEND_TASK_STK[SEND_STK_SIZE-1],SEND_TASK_PRIO);	 				   
	OSTaskSuspend(START_TASK_PRIO);	
	OS_EXIT_CRITICAL();				
}

//LED0����
void measure_task(void *pdata)
{	 
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned char more;
	short sensors;
	long quat[4];
	unsigned long sensor_timestamp;
	u16 adc_value;
	float tmp;
	
	
	while(1)
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
		if (sensors & INV_WXYZ_QUAT ){
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		Pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3; // pitch
		Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
		Yaw = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		sprintf(pbuf,"Pitch:%.2f, Roll:%.2f, Yaw:%.2f",Pitch,Roll,Yaw);	
		printf(pbuf);
		printf("\r\n");
		printf("ax=%.2f ay=%.2f az=%.2f\r\n", accel[0]* MPU6050_G_PER_LSB, \
				accel[1]* MPU6050_G_PER_LSB, accel[2]* MPU6050_G_PER_LSB);
		printf("gx=%.2f gy=%.2f gz=%.2f\r\n", gyro[0]* MPU6050_DEG_PER_LSB,\
				gyro[1]* MPU6050_DEG_PER_LSB, gyro[2]* MPU6050_DEG_PER_LSB);

		adc_value = get_adc_avg(ADC_Channel_3, 10);
		printf("adc=%d	", adc_value);
		tmp = (float)adc_value * (3.3/4096);
		
		LED1 = !LED1;
		
		if(isFisrtBoot){
			LED1 = !LED1;
			LED2 = !LED2;
			LED3 = !LED3;
			LED4 = !LED4;
		}else{
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			LED4 = 1;
		}
		delay_ms(500);
	};
}

//LED2����
void send_task(void *pdata)
{	  
	while(1)
	{
		LED2 = !LED2;
		delay_ms(500);
	};
}

#else