/******************************************************************************
  File			: PID_Ctrl.c
  Description	: PID Control algorithm
  Author		: Xiaoming Li
*******************************************************************************
  Modify List:
-------------------------------------------------------------------------------
  2014/9/18 21:00 PM	| Created
******************************************************************************/

#include "PID_Ctrl.h"

PID PID_roll, PID_pitch, PID_yaw;
float roll_i=0,pitch_i=0,yaw_p=0;	//为后面积分用
Motor_PWM PWM_Ctrl;

extern int16_t throttle;

void PID_init()
{
	PID_roll.P = 1;
	PID_roll.I = 0;
	PID_roll.D = 0;
	
	PID_pitch.P = 1;
	PID_pitch.I = 0;
	PID_pitch.D = 0;

	PID_yaw.P = 1;
	PID_yaw.I = 0;
	PID_yaw.D = 0;
}

/*-------------------------------------------------------------------
 * FUNC : PID_Ctrl
 * DESC : PID control algorithm
 * PARM : new_PWM_Value - new PWM value
 * RET	: N/A
 *-----------------------------------------------------------------*/
void PID_Ctrl(Angle *m_angle, int16_t *m_gyro/*, int16_t thro*/)
{
	Angle angle;
	angle.pitch = m_angle->pitch;
	angle.roll = m_angle->roll;
	/*
	//进行pitch积分，限定pitch积分范围
	pitch_i += angle.pitch;
	if(pitch_i > 2000)
		pitch_i = 2000;
	if(pitch_i < -2000)
		pitch_i = -2000;
	*/
	PID_pitch.out = PID_pitch.P * (angle.pitch - EXPECT) + PID_pitch.D * m_gyro[1];
	PID_roll.out = PID_roll.P * (angle.roll- EXPECT) + PID_roll.D * m_gyro[0];
	PID_yaw.out = PID_yaw.P * (angle.yaw- EXPECT - 180) + PID_yaw.D * m_gyro[2];

	PWM_Ctrl.pwm1 = throttle - 2048 + PID_pitch.out - PID_roll.out - PID_yaw.out;
	PWM_Ctrl.pwm2 = throttle - 2048 + PID_pitch.out + PID_roll.out + PID_yaw.out;
	PWM_Ctrl.pwm3 = throttle - 2048 - PID_pitch.out + PID_roll.out - PID_yaw.out;
	PWM_Ctrl.pwm4 = throttle - 2048 - PID_pitch.out + PID_roll.out + PID_yaw.out;
	
	TIMER3_PWM_Refresh(&PWM_Ctrl);
}
