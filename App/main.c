/******************** (C) COPYRIGHT 2018 Team **************************
 * 文件名  ：main.c
 * 描述    ：        
 * 实验平台：STM32F103C8T6
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 2080620 : Infrared_Recv.c line271,line277,增加2条指令，用于恢复出厂设置和回读参数
 * 日期    ：2018/05/04
**********************************************************************************/
#include "UserApp.h"

#define MAIN_DEBUG_INFO 0x01  //开启串口打印调试信息，如果不需要，屏蔽该行

extern struct ConfigData_Type ConfigData;
extern uint8_t Ir_SendBuff[255];
extern bool Boost_Working;

extern __IO u32 TimingDelay;
extern __IO u32 ModeDelay;
extern uint8_t Power_DetectSomeBody;
extern uint8_t Power_DetectNoBody;
extern volatile u16 BoostPwm_Duty;
extern u16 Working_Current;  
uint16_t LED_DaleyTime = 1000;

/*******************************************************************
*main()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
int main(void)
{
	Hardware_Init();
	System_Init();
	ConfigData_Init();
	
	#ifdef  MAIN_DEBUG_INFO
	  printf("ConfigData.Config_Tag:0x%02X.\r\n",ConfigData.Config_Tag);
		printf("ConfigData.OverDischarge_BackVol:0x%02X.\r\n",ConfigData.OverDischarge_BackVol);
	  printf("ConfigData.OverDischarge_Vol:0x%02X.\r\n\r\n",ConfigData.OverDischarge_Vol);
	
	  Infrared_Send_NByte(0xAA, GUA_INFRARED_DATA_LEN,(u8 *)(&ConfigData.Config_Tag));
  #endif
  
  DISABLE_DIRECT_CHARGE;
  
	ConfigData.Contorller_Mode = MODE_POWERONTEST;
	ModeDelay = 60000;     //开机1分钟测试，1 * 60 * 1000
  Boost_Working = true;
	
  BoostPwm_Duty = 50;
  TIM_SetCompare1( TIM1,BoostPwm_Duty);  
	Start_Boot_PWM();
	
//	while(1)
//	{
//		//LED状态指示，通过设定不同的闪烁频率表示不同的状态
//    if(TimingDelay == 0x00)
//		{
//      FLIP_LED1;
//			TimingDelay = 1000;
//		}

//		Is_Infrared_Recv();
//	}
	
  while(1)
	{
		Is_Infrared_Recv();
    		
    //开机1分钟测试
    if(ConfigData.Contorller_Mode == MODE_POWERONTEST)
		{
		  WorkInPowerOnTest();
		}
		//休眠状态，需要遥控器开启
    else if(ConfigData.Contorller_Mode == MODE_STANDBY)      //休眠
		{
      WorkInStandby();	
		}
    //白天待机
		else if(ConfigData.Contorller_Mode == MODE_IDLE)          
		{
		  WorkInIdleMode();		
		}
		//光控延迟
    else if(ConfigData.Contorller_Mode == MODE_LIGHTCTL)   //光控延迟
		{
		  WorkInLightControlDelay();	
		}
		else
		{			
			//感应时间段1
      if(ConfigData.Contorller_Mode == MODE_TIMESLOT1)
			{
				WorkInTimeSlot1();				  				
			}
      //感应时间段2
			else if(ConfigData.Contorller_Mode == MODE_TIMESLOT2)
			{
				WorkInTimeSlot2();
			}
      //感应时间段3
			else if(ConfigData.Contorller_Mode == MODE_TIMESLOT3)
			{
				WorkInTimeSlot3();		
			}
			
			Is_LoadCurrent_Rated();
			Is_SomeBody_Detection();
		}
		
		//LED状态指示，通过设定不同的闪烁频率表示不同的状态
    if(TimingDelay == 0x00)
		{
      FLIP_LED1;
			FLIP_LED2;

			TimingDelay = LED_DaleyTime;
		}
	}
}
/******************* (C) COPYRIGHT 2018 Team *****END OF FILE************/


