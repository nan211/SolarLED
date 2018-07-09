/******************** (C) COPYRIGHT 2012  Team **************************
 * �ļ���  ��Current_PidCtl.c
 * ����    ��         
 * ʵ��ƽ̨��STM32F103C8T6
 * Ӳ�����ӣ�---------------------
 *          |  PB.08: (TIM4_CH3)  -- Charge_pwm  |
 *           ---------------------    			
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "Current_PidCtl.h"
#include "math.h"
#include "adc.h"
#include "misc.h"
#include "Usart1.h"

struct t_pid
{
    float SetCurrent;         //�����趨ֵ
    float ActualCurrent;      //����ʵ��ֵ
    float err;                //����ƫ��ֵ
    float err_last;           //������һ��ƫ��ֵ
    float Kp,Ki,Kd;           //������������֡�΢��ϵ��
    float Boost_Pwm_Duty;     //�����ѹֵ������ִ�����ı�����
    float integral;           //�������ֵ
    float umax;               //���ֵ   
    float umin;               //��Сֵ
}pid;

void CurrentPID_init(void)
{
    pid.SetCurrent = 0.0f;
    pid.ActualCurrent = 0.0f;
    pid.err = 0.0f;
    pid.err_last = 0.0f;
    pid.Boost_Pwm_Duty = 50.0f;
    pid.integral = 0.0f;
    pid.Kp = 0.4f;
    pid.Ki = 0.2f;       //ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
    pid.Kd = 0.2f;
    pid.umax = 3000;     //���ֵ
    pid.umin = 0;       //��Сֵ
}

//����ֵ�PID�����㷨
float CurrentPID_Realize(float SetCurrent)
{
    float index;
	
    pid.SetCurrent = SetCurrent;
	  pid.ActualCurrent = (float) GetLoadCurrent();    //��õ�ǰ���ص���
	
    pid.err = pid.SetCurrent - pid.ActualCurrent;
	
    if(fabs(pid.err) > SetCurrent)                        //ȡ������
    {
      index = 0.0;
    }
		else if(fabs(pid.err) < (SetCurrent * 0.8))           //�Ӵ����
		{
      index = 1.0;
      pid.integral += pid.err;
    }
		else
		{
      index = (SetCurrent - fabs(pid.err)) / (SetCurrent - SetCurrent * 0.8);   //����ֹ���
      pid.integral += pid.err;
    }
    pid.Boost_Pwm_Duty = pid.Kp * pid.err + index * pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
    pid.err_last = pid.err;
    
		pid.Boost_Pwm_Duty = pid.Boost_Pwm_Duty * 0.01;
    
		if(pid.Boost_Pwm_Duty > 900) 
       pid.Boost_Pwm_Duty = 900;
    else
     if(pid.Boost_Pwm_Duty < 0) 
			 pid.Boost_Pwm_Duty = 10;
			
		//ִ��PID
		TIM_SetCompare1( TIM1,pid.Boost_Pwm_Duty  );      //ִ�����
		printf("pid.Boost_Pwm_Duty:%d\r\n",(u16)pid.Boost_Pwm_Duty);
				
//		Dac1_Set_Vol(pid.voltage * 0.5);		
//    pid.ActualCurrent = (float) ADC_ConvertedValue / 4096 * 3300;
   
		return pid.ActualCurrent;
}

