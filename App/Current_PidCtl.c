/******************** (C) COPYRIGHT 2012  Team **************************
 * 文件名  ：Current_PidCtl.c
 * 描述    ：         
 * 实验平台：STM32F103C8T6
 * 硬件连接：---------------------
 *          |  PB.08: (TIM4_CH3)  -- Charge_pwm  |
 *           ---------------------    			
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "Current_PidCtl.h"
#include "math.h"
#include "adc.h"
#include "misc.h"
#include "Usart1.h"

struct t_pid
{
    float SetCurrent;         //定义设定值
    float ActualCurrent;      //定义实际值
    float err;                //定义偏差值
    float err_last;           //定义上一个偏差值
    float Kp,Ki,Kd;           //定义比例、积分、微分系数
    float Boost_Pwm_Duty;     //定义电压值（控制执行器的变量）
    float integral;           //定义积分值
    float umax;               //最大值   
    float umin;               //最小值
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
    pid.Ki = 0.2f;       //注意，和上几次相比，这里加大了积分环节的值
    pid.Kd = 0.2f;
    pid.umax = 3000;     //最大值
    pid.umin = 0;       //最小值
}

//变积分的PID控制算法
float CurrentPID_Realize(float SetCurrent)
{
    float index;
	
    pid.SetCurrent = SetCurrent;
	  pid.ActualCurrent = (float) GetLoadCurrent();    //获得当前负载电流
	
    pid.err = pid.SetCurrent - pid.ActualCurrent;
	
    if(fabs(pid.err) > SetCurrent)                        //取消积分
    {
      index = 0.0;
    }
		else if(fabs(pid.err) < (SetCurrent * 0.8))           //加大积分
		{
      index = 1.0;
      pid.integral += pid.err;
    }
		else
		{
      index = (SetCurrent - fabs(pid.err)) / (SetCurrent - SetCurrent * 0.8);   //变积分过程
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
			
		//执行PID
		TIM_SetCompare1( TIM1,pid.Boost_Pwm_Duty  );      //执行输出
		printf("pid.Boost_Pwm_Duty:%d\r\n",(u16)pid.Boost_Pwm_Duty);
				
//		Dac1_Set_Vol(pid.voltage * 0.5);		
//    pid.ActualCurrent = (float) ADC_ConvertedValue / 4096 * 3300;
   
		return pid.ActualCurrent;
}

