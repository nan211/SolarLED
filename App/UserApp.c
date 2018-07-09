/******************** (C) COPYRIGHT 2018 Team **************************
 * �ļ���  ��main.c
 * ����    ��        
 * ʵ��ƽ̨��STM32F103C8T6
 * ��汾  ��ST3.5.0 
 * ����    ��Travis 
 * ����    ��2018/05/04
**********************************************************************************/
#include "math.h"
#include "UserApp.h"
#include "Control_GPIO.h"

extern bool PIR_DetectionFlag; 
extern bool Ir_RecvComplete;                //���������ɱ�־λ

extern volatile u16 BoostPwm_Duty;
extern uint16_t VirtAddVarTab[NumbOfVar];   //�����洢�������ַ
extern uint16_t LED_DaleyTime ;
extern __IO u32 ModeDelay;

bool Boost_Working = false;
bool LightControlDelay = false;

u16 Curr_Current, Working_Current, Last_Current;

struct ConfigData_Type ConfigData;

uint8_t Ir_SendBuff[255];

uint8_t Power_DetectSomeBody = 0;
uint8_t Power_DetectNoBody = 0;

uint32_t PIR_DetectDelay_CNT = 0;    //��Ӧ����ӳټ�ʱ


/*******************************************************************
*Hardware_Init()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void Hardware_Init(void)
{
  PWR_PVD_Init();
	ADC1_Init();  
  USART1_Init();
  SysTick_Init();
	Boost_PWM_Init();              //Boost_PWMʹ��T1
	Charge_PWM_Init();             //Charge_PWMʹ��T4
	Contorl_GPIO_Config();
	Infrared_Send_Init();     //IR_Sendʹ��T2
  Infrared_Recv_Init();	    //IR_Recvʹ��T3  
}

/*******************************************************************
*System_Init()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void System_Init(void)
{
  LED1(ON);	
  LED2(OFF);
	
	DISABLE_OVERLOAD;        //LED���ر���ʧЧ
	DISABLE_DIRECT_CHARGE;   //�رճ������
	
  ConfigData_Init();
  
  Boost_Working = false;
  LightControlDelay = false;  

  PIR_DetectDelay_CNT = ConfigData.Detection_Delay * 1000;	
}


/*******************************************************************
*Is_LoadCurrent_Rated()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
u16 Is_LoadCurrent_Rated(void)
{	
  if(Boost_Working)
  {
    Curr_Current = GetLoadCurrent();    //��õ�ǰ���ص���
     
    #ifdef  USERAPP_DEBUG_INFO
        printf("\r\n��ǰ����:%d mA,�����:%d mA.%d\r\n\r\n",Curr_Current,ConfigData.Load_RatedCurrent,BoostPwm_Duty);
     #endif
	
    if(Curr_Current > (ConfigData.Load_RatedCurrent<<1))    //���ع���,�ȶ������1��	
    {
      ENABLE_OVERLOAD;      
      Stop_Boot_PWM();
      Boost_Working = false;
      LED_DaleyTime = 100;
      
      #ifdef  USERAPP_DEBUG_INFO
        printf("\r\n���ع���,��ǰ����:%d mA,�����:%d mA.%d\r\n\r\n",Curr_Current,ConfigData.Load_RatedCurrent,BoostPwm_Duty);
      #endif
      
      return LOAD_OVERRATED;
    }
    else	
    {      
      if(fabs((float)(Working_Current) - (float)(Curr_Current)) > Working_Current * 0.03)    //���ص�������
      {		
        if(Curr_Current < Working_Current)
        {				
          BoostPwm_Duty = BoostPwm_Duty + 1;
          
          if(BoostPwm_Duty > 800) 
            BoostPwm_Duty = 800;
        }
        else
        {
          BoostPwm_Duty = BoostPwm_Duty - 1;
          
          if(BoostPwm_Duty < 50) 
            BoostPwm_Duty = 50;
        }
        
        TIM_SetCompare1( TIM1,BoostPwm_Duty );      //ִ�����
				
				#ifdef  DEBUG_INFO
          printf("���ص���,ռ�ձ�:%d,���ص���:%d mA,�����:%d mA,ģʽ:%d,��ʱ:%d ms.\r\n\r\n",BoostPwm_Duty,Curr_Current,Working_Current,ConfigData.Contorller_Mode - 4,ModeDelay);
				#endif
      }
      else
      if(Curr_Current < 2)    //���ؿ�·
      {
        Stop_Boot_PWM();
        Boost_Working = false;
        LED_DaleyTime = 100;
        
        #ifdef  USERAPP_DEBUG_INFO
          printf("���ؿ�·,��ǰ����:%d mA\r\n",Curr_Current);
        #endif 
        
        return LOAD_OPEN;
      }
            
      return LOAD_RATED;
    }
	
  }
  else
  {
    return 0x00;
  }    
    
}



/*******************************************************************
*Is_SomeBody_Detection()
*�������ܣ�����Ƿ����͵��Ӧ������
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void Is_SomeBody_Detection(void)
{
	if( Boost_Working )  //ֻ����ѹ��·��ʼ����ʱ�ż��
	{
		 if(PIR_DetectionFlag)
		 {
			 Delay_ms(10);
			 
			 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0x00)  //���ⴥ������
			 {
				 PIR_DetectDelay_CNT = ConfigData.Detection_Delay * 1000;
				 
				 #ifdef  USERAPP_DEBUG_INFO
					 printf("\r\n���͵��Ӧ�����˿���,����LED����,����ʼ��ʱ��\r\n\r\n");
				 #endif
			 }
			 
			 PIR_DetectionFlag = false;
				 
		 }
		 
		if(PIR_DetectDelay_CNT > 0)
		{
			 Working_Current = ConfigData.Load_RatedCurrent * Power_DetectSomeBody / 100;
			 LED_DaleyTime = 200;  //200msLED�ӳ٣��ӿ������ٶȱ�ʾ���͵��Ӧ��Ч
			 
		}
		else
		{
			 Working_Current = ConfigData.Load_RatedCurrent * Power_DetectNoBody / 100;
			 LED_DaleyTime = 1000;  //1s			
		}
  }
}

/*******************************************************************
*Is_Infrared_Recv()
*�������ܣ�����Ƿ���յ�����ң����ָ��
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void Is_Infrared_Recv(void)
{
	if(Ir_RecvComplete)
	{
		Infrared_DeCode();
	}
}

/*******************************************************************
*SetDefault_ConfigData()
*�������ܣ��ָ�����Ĭ�ϲ���
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void SetDefault_ConfigData(void)
{		
	EE_FLASH_WriteVariable(Addr_Battery_Type,0x01);                //�������:0x01:�12,0x02:�24,  ��λ:V,  Ĭ��0x01
	EE_FLASH_WriteVariable(Addr_Detect_Time1,0x02);                //��Ӧʱ��1:0~15H,����1H,        ��λ:H,  Ĭ��0x02   2H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody1,0x64);             //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Power_Nobody1,0x32);               //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Detect_Time2,0x03);                //��Ӧʱ��2:0~15H,����1H,        ��λ:H,  Ĭ��0x03   3H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody2,0x50);             //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Power_Nobody2,0x2F);               //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Detect_Time3,0x08);                //��Ӧʱ��3:0~15H,����1H,        ��λ:H,  Ĭ��0x08   8H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody3,0x46);             //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x64   70%,0x46
	EE_FLASH_WriteVariable(Addr_Power_Nobody3,0x1E);               //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x1E	   50%,0x32

	EE_FLASH_WriteVariable(Addr_Detection_Delay,0x1E);             //���˸�Ӧ�ӳ�ʱ��:0~255s,����1s, ��λ:s, Ĭ��0x1E
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Vol,0x05);             //��ص��ѹ:5~11V,����1V,       ��λ:V,  Ĭ��0x05
	
	//EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,0x05);           //����ӳ�:0~50min,����1min,     ��λ:min,Ĭ��0x05		
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,0x01);            //������:����ӳ�1

	//EE_FLASH_WriteVariable(Addr_Load_Current,0x012C);            //���ص���:0~6000mA,����10mA,    ��λ:mA, Ĭ��0x012C ,300mA
	EE_FLASH_WriteVariable(Addr_Load_Current,0x0064);              //������:���ص���:100mA    
	
	EE_FLASH_WriteVariable(Addr_Auto_Power,0x01);
	EE_FLASH_WriteVariable(Addr_Charging_Freezing,0x02);           
	EE_FLASH_WriteVariable(Addr_OverDischarge_Vol,0x5A);         //Ĭ�Ϲ��ű���9.0V
	
	//EE_FLASH_WriteVariable(Addr_OverDischarge_BackVol,0x2EE0);
	EE_FLASH_WriteVariable(Addr_OverDischarge_BackVol,0x60);     //Ĭ�Ϲ��ŷ���9.6V
	
	EE_FLASH_WriteVariable(Addr_OverCharge_Vol,0x7E);            //Ĭ�Ϲ��屣��12.6V
	EE_FLASH_WriteVariable(Addr_OverCharge_BackVol,0x78);        //Ĭ�Ϲ��巵��12.0V  
	EE_FLASH_WriteVariable(Addr_Config_Tag,0xAA);
}

/*******************************************************************
*SaveUser_ConfigData()
*�������ܣ������û����ò���
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void SaveUser_ConfigData(void)
{		
	EE_FLASH_WriteVariable(Addr_Battery_Type,ConfigData.Battery_Type);                //�������:0x01:�12,0x02:�24,  ��λ:V,  Ĭ��0x01
	EE_FLASH_WriteVariable(Addr_Detect_Time1,ConfigData.Detect_Time1);                //��Ӧʱ��1:0~15H,����1H,        ��λ:H,  Ĭ��0x02   2H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody1,ConfigData.Power_SomeBody1);          //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Power_Nobody1,ConfigData.Power_Nobody1);              //���˸�Ӧ����1:0~100%,����10%,  ��λ:%,  Ĭ��0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Detect_Time2,ConfigData.Detect_Time2);                //��Ӧʱ��2:0~15H,����1H,        ��λ:H,  Ĭ��0x03   3H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody2,ConfigData.Power_SomeBody2);          //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Power_Nobody2,ConfigData.Power_Nobody2);              //���˸�Ӧ����2:0~100%,����10%,  ��λ:%,  Ĭ��0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Detect_Time3,ConfigData.Detect_Time3);                //��Ӧʱ��3:0~15H,����1H,        ��λ:H,  Ĭ��0x08   8H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody3,ConfigData.Power_SomeBody3);          //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x64   70%,0x46
	EE_FLASH_WriteVariable(Addr_Power_Nobody3,ConfigData.Power_Nobody3);              //���˸�Ӧ����3:0~100%,����10%,  ��λ:%,  Ĭ��0x1E	   50%,0x32

	EE_FLASH_WriteVariable(Addr_Detection_Delay,ConfigData.Detection_Delay);          //���˸�Ӧ�ӳ�ʱ��:0~255s,����1s, ��λ:s, Ĭ��0x1E
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Vol,ConfigData.Optical_Ctl_Vol);          //��ص��ѹ:5~11V,����1V,       ��λ:V,  Ĭ��0x05
	
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,ConfigData.Optical_Ctl_Delay);      //����ӳ�:0~50min,����1min,     ��λ:min,Ĭ��0x05		

	EE_FLASH_WriteVariable(Addr_Load_Current,ConfigData.Load_RatedCurrent);           //���ص���:0~6000mA,����10mA,    ��λ:mA, Ĭ��0x012C ,300mA    
	
	EE_FLASH_WriteVariable(Addr_Auto_Power,ConfigData.Auto_Power);
	EE_FLASH_WriteVariable(Addr_Charging_Freezing,ConfigData.Charging_Freezing);
	EE_FLASH_WriteVariable(Addr_OverDischarge_Vol,ConfigData.OverDischarge_Vol);
	
	EE_FLASH_WriteVariable(Addr_OverDischarge_BackVol,ConfigData.OverDischarge_BackVol);     
	
	EE_FLASH_WriteVariable(Addr_OverCharge_Vol,ConfigData.OverCharge_Vol);
	EE_FLASH_WriteVariable(Addr_OverCharge_BackVol,ConfigData.OverCharge_BackVol);
	EE_FLASH_WriteVariable(Addr_Config_Tag,ConfigData.Config_Tag);
}

/*******************************************************************
*GetDefault_ConfigData()
*�������ܣ� ��ȡ����Ĭ�����ò���
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void GetDefault_ConfigData(void)
{		
	EE_FLASH_ReadVariable(Addr_Battery_Type,(u16*)&ConfigData.Battery_Type);
	EE_FLASH_ReadVariable(Addr_Detect_Time1,(u16*)&ConfigData.Detect_Time1);
	EE_FLASH_ReadVariable(Addr_Power_SomeBody1,(u16*)&ConfigData.Power_SomeBody1);
	EE_FLASH_ReadVariable(Addr_Power_Nobody1,(u16*)&ConfigData.Power_Nobody1);
	EE_FLASH_ReadVariable(Addr_Detect_Time2,(u16*)&ConfigData.Detect_Time2);
	EE_FLASH_ReadVariable(Addr_Power_SomeBody2,(u16*)&ConfigData.Power_SomeBody2);
	EE_FLASH_ReadVariable(Addr_Power_Nobody2,(u16*)&ConfigData.Power_Nobody2);
	EE_FLASH_ReadVariable(Addr_Detect_Time2,(u16*)&ConfigData.Detect_Time3);
	EE_FLASH_ReadVariable(Addr_Power_SomeBody3,(u16*)&ConfigData.Power_SomeBody3);
	EE_FLASH_ReadVariable(Addr_Power_Nobody3,(u16*)&ConfigData.Power_Nobody3);
	EE_FLASH_ReadVariable(Addr_Detection_Delay,(u16*)&ConfigData.Detection_Delay);
	EE_FLASH_ReadVariable(Addr_Optical_Ctl_Vol,(u16*)&ConfigData.Optical_Ctl_Vol);
	EE_FLASH_ReadVariable(Addr_Optical_Ctl_Delay,(u16*)&ConfigData.Optical_Ctl_Delay);
	EE_FLASH_ReadVariable(Addr_Load_Current,(u16*)&ConfigData.Load_RatedCurrent);
	EE_FLASH_ReadVariable(Addr_Auto_Power,(u16*)&ConfigData.Auto_Power);
	EE_FLASH_ReadVariable(Addr_Charging_Freezing,(u16*)&ConfigData.Charging_Freezing);
	EE_FLASH_ReadVariable(Addr_OverDischarge_Vol,(u16*)&ConfigData.OverDischarge_Vol);
	EE_FLASH_ReadVariable(Addr_OverDischarge_BackVol,(u16*)&ConfigData.OverDischarge_BackVol);
	EE_FLASH_ReadVariable(Addr_OverCharge_Vol,(u16*)&ConfigData.OverCharge_Vol);
	EE_FLASH_ReadVariable(Addr_OverCharge_BackVol,(u16*)&ConfigData.OverCharge_BackVol);
	EE_FLASH_ReadVariable(Addr_Config_Tag,(u16*)&ConfigData.Config_Tag);
}

/*******************************************************************
*ConfigData_Init()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/
void ConfigData_Init(void)
{	
	EE_FLASH_Init();         //��ʼ��FLASH
	
	EE_FLASH_ReadVariable(Addr_Config_Tag,(u16*)&ConfigData.Config_Tag);    //���û�����ù���ʼʹ��Ĭ������
	
	if(ConfigData.Config_Tag == 0x00)
	{
  	#ifdef  USERAPP_DEBUG_INFO
	    printf("δ��ȡ����Ч���ã����ڻָ�����ֵ.....\r\n\r\n");
    #endif
		SetDefault_ConfigData();
		GetDefault_ConfigData();
	}
	else if(ConfigData.Config_Tag == 0xAA)
	{
    #ifdef  USERAPP_DEBUG_INFO
	    printf("������Ч���ã����ڶ�ȡ......\r\n\r\n");
    #endif
		GetDefault_ConfigData();		
	}	
}

/*******************************************************************
*WorkInPowerOnTest()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInPowerOnTest(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_STANDBY;
		Stop_Boot_PWM();
		ModeDelay =  5000;   //FOR TEST
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л������ߡ���\r\n");
		#endif
	}
	else
	{
		Working_Current = ConfigData.Load_RatedCurrent;
		Is_LoadCurrent_Rated();
	}
}

/*******************************************************************
*WorkInStandby()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInStandby(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_IDLE;
		Stop_Boot_PWM();
		Boost_Working = false;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л��������������\r\n");
		#endif
	}
}


/*******************************************************************
*WorkInIdleMode()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInIdleMode(void)
{
	if( GetSolarVolt() < ConfigData.Optical_Ctl_Vol)                 //������ڹ��ֵ,��ʼ����ӳ�
	{
		ConfigData.Contorller_Mode = MODE_LIGHTCTL;
		ModeDelay = ConfigData.Optical_Ctl_Delay * 60 * 1000;
		//ModeDelay =  30000;   //FOR TEST
		Stop_Boot_PWM();
		Boost_Working = false;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л�������ӳ١���\r\n");
		#endif
	}
	else
	{
		//��ع��䱣��
		if((GetBaterryVolt()) > ConfigData.OverCharge_Vol / 10)   
		{
			DISABLE_DIRECT_CHARGE;
		}
		else         //��ع��ű���.���¶����ϣ��������
		if(((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10) && (GetSTM32Temp() > 0))
		{
			ENABLE_DIRECT_CHARGE;
			LED_DaleyTime = 100;
		}       
	}	
}

/*******************************************************************
*WorkInLightControlDelay()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInLightControlDelay(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_TIMESLOT1;
		ModeDelay = ConfigData.Detect_Time1 * 60 * 60 * 1000;
		//ModeDelay =  5000;    //FOR TEST
		
		//��ص�ѹ���ڹ��ŷ���ֵ�Ż�����������
		if((GetBaterryVolt()) > ConfigData.OverDischarge_BackVol / 10)
		{
			 Start_Boot_PWM();
			 Boost_Working = true;
		}
		else
		{
			 Stop_Boot_PWM();
			 Boost_Working = false; 
			 LED_DaleyTime = 100;          
		}

		BoostPwm_Duty = 50;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л�����Ӧʱ���1����\r\n");
		#endif
	}	
}

/*******************************************************************
*WorkInTimeSlot1()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInTimeSlot1(void)
{
	Power_DetectSomeBody = ConfigData.Power_SomeBody1;
	Power_DetectNoBody = ConfigData.Power_Nobody1;
	
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_TIMESLOT2;
		ModeDelay = ConfigData.Detect_Time2 * 60 * 60 * 1000;
		//BoostPwm_Duty = 50;  //FOR TEST					
		//ModeDelay =  30000;     //FOR TEST
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л�����Ӧʱ���2����\r\n");
		#endif				
	}
	
	//��ع��ű���
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	}		
}

/*******************************************************************
*WorkInPowerOnTest()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInTimeSlot2(void)
{
	Power_DetectSomeBody = ConfigData.Power_SomeBody2;
	Power_DetectNoBody = ConfigData.Power_Nobody2;				
	
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_TIMESLOT3;
		ModeDelay = ConfigData.Detect_Time3 * 60 * 60 * 1000;
		//ModeDelay =  30000;  //FOR TEST
		//BoostPwm_Duty = 50;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л�����Ӧʱ���3����\r\n");
		#endif				
	}
	
	//��ع��ű���
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	} 	
}

/*******************************************************************
*WorkInPowerOnTest()
*�������ܣ�
*�����������
*���ز�������
*��д���ߣ�
*��дʱ�䣺
*���˵����
********************************************************************/

void WorkInTimeSlot3(void)
{
	Power_DetectSomeBody = ConfigData.Power_SomeBody3;
	Power_DetectNoBody = ConfigData.Power_Nobody3;

	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_IDLE;
		Stop_Boot_PWM();
		Boost_Working = false;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\nģʽ�л��������������\r\n");
		#endif				
	}

	//��ع��ű���
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	} 			
}
