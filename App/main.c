/******************** (C) COPYRIGHT 2018 Team **************************
 * �ļ���  ��main.c
 * ����    ��        
 * ʵ��ƽ̨��STM32F103C8T6
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * 2080620 : Infrared_Recv.c line271,line277,����2��ָ����ڻָ��������úͻض�����
 * ����    ��2018/05/04
**********************************************************************************/
#include "UserApp.h"

#define MAIN_DEBUG_INFO 0x01  //�������ڴ�ӡ������Ϣ���������Ҫ�����θ���

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
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
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
	ModeDelay = 60000;     //����1���Ӳ��ԣ�1 * 60 * 1000
  Boost_Working = true;
	
  BoostPwm_Duty = 50;
  TIM_SetCompare1( TIM1,BoostPwm_Duty);  
	Start_Boot_PWM();
	
//	while(1)
//	{
//		//LED״ָ̬ʾ��ͨ���趨��ͬ����˸Ƶ�ʱ�ʾ��ͬ��״̬
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
    		
    //����1���Ӳ���
    if(ConfigData.Contorller_Mode == MODE_POWERONTEST)
		{
		  WorkInPowerOnTest();
		}
		//����״̬����Ҫң��������
    else if(ConfigData.Contorller_Mode == MODE_STANDBY)      //����
		{
      WorkInStandby();	
		}
    //�������
		else if(ConfigData.Contorller_Mode == MODE_IDLE)          
		{
		  WorkInIdleMode();		
		}
		//����ӳ�
    else if(ConfigData.Contorller_Mode == MODE_LIGHTCTL)   //����ӳ�
		{
		  WorkInLightControlDelay();	
		}
		else
		{			
			//��Ӧʱ���1
      if(ConfigData.Contorller_Mode == MODE_TIMESLOT1)
			{
				WorkInTimeSlot1();				  				
			}
      //��Ӧʱ���2
			else if(ConfigData.Contorller_Mode == MODE_TIMESLOT2)
			{
				WorkInTimeSlot2();
			}
      //��Ӧʱ���3
			else if(ConfigData.Contorller_Mode == MODE_TIMESLOT3)
			{
				WorkInTimeSlot3();		
			}
			
			Is_LoadCurrent_Rated();
			Is_SomeBody_Detection();
		}
		
		//LED״ָ̬ʾ��ͨ���趨��ͬ����˸Ƶ�ʱ�ʾ��ͬ��״̬
    if(TimingDelay == 0x00)
		{
      FLIP_LED1;
			FLIP_LED2;

			TimingDelay = LED_DaleyTime;
		}
	}
}
/******************* (C) COPYRIGHT 2018 Team *****END OF FILE************/


