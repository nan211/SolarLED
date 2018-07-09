/******************** (C) COPYRIGHT 2018 Team **************************
 * 文件名  ：main.c
 * 描述    ：        
 * 实验平台：STM32F103C8T6
 * 库版本  ：ST3.5.0 
 * 作者    ：Travis 
 * 日期    ：2018/05/04
**********************************************************************************/
#include "math.h"
#include "UserApp.h"
#include "Control_GPIO.h"

extern bool PIR_DetectionFlag; 
extern bool Ir_RecvComplete;                //红外接收完成标志位

extern volatile u16 BoostPwm_Duty;
extern uint16_t VirtAddVarTab[NumbOfVar];   //参数存储的虚拟地址
extern uint16_t LED_DaleyTime ;
extern __IO u32 ModeDelay;

bool Boost_Working = false;
bool LightControlDelay = false;

u16 Curr_Current, Working_Current, Last_Current;

struct ConfigData_Type ConfigData;

uint8_t Ir_SendBuff[255];

uint8_t Power_DetectSomeBody = 0;
uint8_t Power_DetectNoBody = 0;

uint32_t PIR_DetectDelay_CNT = 0;    //感应输出延迟计时


/*******************************************************************
*Hardware_Init()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void Hardware_Init(void)
{
  PWR_PVD_Init();
	ADC1_Init();  
  USART1_Init();
  SysTick_Init();
	Boost_PWM_Init();              //Boost_PWM使用T1
	Charge_PWM_Init();             //Charge_PWM使用T4
	Contorl_GPIO_Config();
	Infrared_Send_Init();     //IR_Send使用T2
  Infrared_Recv_Init();	    //IR_Recv使用T3  
}

/*******************************************************************
*System_Init()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void System_Init(void)
{
  LED1(ON);	
  LED2(OFF);
	
	DISABLE_OVERLOAD;        //LED过载保护失效
	DISABLE_DIRECT_CHARGE;   //关闭充电输入
	
  ConfigData_Init();
  
  Boost_Working = false;
  LightControlDelay = false;  

  PIR_DetectDelay_CNT = ConfigData.Detection_Delay * 1000;	
}


/*******************************************************************
*Is_LoadCurrent_Rated()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
u16 Is_LoadCurrent_Rated(void)
{	
  if(Boost_Working)
  {
    Curr_Current = GetLoadCurrent();    //获得当前负载电流
     
    #ifdef  USERAPP_DEBUG_INFO
        printf("\r\n当前电流:%d mA,额定电流:%d mA.%d\r\n\r\n",Curr_Current,ConfigData.Load_RatedCurrent,BoostPwm_Duty);
     #endif
	
    if(Curr_Current > (ConfigData.Load_RatedCurrent<<1))    //负载过流,比额定电流大1倍	
    {
      ENABLE_OVERLOAD;      
      Stop_Boot_PWM();
      Boost_Working = false;
      LED_DaleyTime = 100;
      
      #ifdef  USERAPP_DEBUG_INFO
        printf("\r\n负载过流,当前电流:%d mA,额定电流:%d mA.%d\r\n\r\n",Curr_Current,ConfigData.Load_RatedCurrent,BoostPwm_Duty);
      #endif
      
      return LOAD_OVERRATED;
    }
    else	
    {      
      if(fabs((float)(Working_Current) - (float)(Curr_Current)) > Working_Current * 0.03)    //负载电流调节
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
        
        TIM_SetCompare1( TIM1,BoostPwm_Duty );      //执行输出
				
				#ifdef  DEBUG_INFO
          printf("负载调节,占空比:%d,负载电流:%d mA,额定电流:%d mA,模式:%d,余时:%d ms.\r\n\r\n",BoostPwm_Duty,Curr_Current,Working_Current,ConfigData.Contorller_Mode - 4,ModeDelay);
				#endif
      }
      else
      if(Curr_Current < 2)    //负载开路
      {
        Stop_Boot_PWM();
        Boost_Working = false;
        LED_DaleyTime = 100;
        
        #ifdef  USERAPP_DEBUG_INFO
          printf("负载开路,当前电流:%d mA\r\n",Curr_Current);
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
*函数功能：检测是否热释电感应到有人
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void Is_SomeBody_Detection(void)
{
	if( Boost_Working )  //只有升压电路开始工作时才检测
	{
		 if(PIR_DetectionFlag)
		 {
			 Delay_ms(10);
			 
			 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0x00)  //红外触发消抖
			 {
				 PIR_DetectDelay_CNT = ConfigData.Detection_Delay * 1000;
				 
				 #ifdef  USERAPP_DEBUG_INFO
					 printf("\r\n热释电感应到有人靠近,调高LED亮度,并开始计时！\r\n\r\n");
				 #endif
			 }
			 
			 PIR_DetectionFlag = false;
				 
		 }
		 
		if(PIR_DetectDelay_CNT > 0)
		{
			 Working_Current = ConfigData.Load_RatedCurrent * Power_DetectSomeBody / 100;
			 LED_DaleyTime = 200;  //200msLED延迟，加快闪灯速度表示热释电感应起效
			 
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
*函数功能：检测是否接收到红外遥控器指令
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
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
*函数功能：恢复出厂默认参数
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void SetDefault_ConfigData(void)
{		
	EE_FLASH_WriteVariable(Addr_Battery_Type,0x01);                //电池类型:0x01:锂12,0x02:锂24,  单位:V,  默认0x01
	EE_FLASH_WriteVariable(Addr_Detect_Time1,0x02);                //感应时间1:0~15H,步长1H,        单位:H,  默认0x02   2H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody1,0x64);             //有人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Power_Nobody1,0x32);               //无人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Detect_Time2,0x03);                //感应时间2:0~15H,步长1H,        单位:H,  默认0x03   3H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody2,0x50);             //有人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Power_Nobody2,0x2F);               //无人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Detect_Time3,0x08);                //感应时间3:0~15H,步长1H,        单位:H,  默认0x08   8H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody3,0x46);             //有人感应功率3:0~100%,步长10%,  单位:%,  默认0x64   70%,0x46
	EE_FLASH_WriteVariable(Addr_Power_Nobody3,0x1E);               //无人感应功率3:0~100%,步长10%,  单位:%,  默认0x1E	   50%,0x32

	EE_FLASH_WriteVariable(Addr_Detection_Delay,0x1E);             //有人感应延迟时间:0~255s,步长1s, 单位:s, 默认0x1E
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Vol,0x05);             //光控点电压:5~11V,步长1V,       单位:V,  默认0x05
	
	//EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,0x05);           //光控延迟:0~50min,步长1min,     单位:min,默认0x05		
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,0x01);            //测试用:光控延迟1

	//EE_FLASH_WriteVariable(Addr_Load_Current,0x012C);            //负载电流:0~6000mA,步长10mA,    单位:mA, 默认0x012C ,300mA
	EE_FLASH_WriteVariable(Addr_Load_Current,0x0064);              //测试用:负载电流:100mA    
	
	EE_FLASH_WriteVariable(Addr_Auto_Power,0x01);
	EE_FLASH_WriteVariable(Addr_Charging_Freezing,0x02);           
	EE_FLASH_WriteVariable(Addr_OverDischarge_Vol,0x5A);         //默认过放保护9.0V
	
	//EE_FLASH_WriteVariable(Addr_OverDischarge_BackVol,0x2EE0);
	EE_FLASH_WriteVariable(Addr_OverDischarge_BackVol,0x60);     //默认过放返回9.6V
	
	EE_FLASH_WriteVariable(Addr_OverCharge_Vol,0x7E);            //默认过冲保护12.6V
	EE_FLASH_WriteVariable(Addr_OverCharge_BackVol,0x78);        //默认过冲返回12.0V  
	EE_FLASH_WriteVariable(Addr_Config_Tag,0xAA);
}

/*******************************************************************
*SaveUser_ConfigData()
*函数功能：保存用户设置参数
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void SaveUser_ConfigData(void)
{		
	EE_FLASH_WriteVariable(Addr_Battery_Type,ConfigData.Battery_Type);                //电池类型:0x01:锂12,0x02:锂24,  单位:V,  默认0x01
	EE_FLASH_WriteVariable(Addr_Detect_Time1,ConfigData.Detect_Time1);                //感应时间1:0~15H,步长1H,        单位:H,  默认0x02   2H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody1,ConfigData.Power_SomeBody1);          //有人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Power_Nobody1,ConfigData.Power_Nobody1);              //无人感应功率1:0~100%,步长10%,  单位:%,  默认0x64   100%,0x64
	EE_FLASH_WriteVariable(Addr_Detect_Time2,ConfigData.Detect_Time2);                //感应时间2:0~15H,步长1H,        单位:H,  默认0x03   3H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody2,ConfigData.Power_SomeBody2);          //有人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Power_Nobody2,ConfigData.Power_Nobody2);              //无人感应功率2:0~100%,步长10%,  单位:%,  默认0x50   80%,0x50
	EE_FLASH_WriteVariable(Addr_Detect_Time3,ConfigData.Detect_Time3);                //感应时间3:0~15H,步长1H,        单位:H,  默认0x08   8H
	EE_FLASH_WriteVariable(Addr_Power_SomeBody3,ConfigData.Power_SomeBody3);          //有人感应功率3:0~100%,步长10%,  单位:%,  默认0x64   70%,0x46
	EE_FLASH_WriteVariable(Addr_Power_Nobody3,ConfigData.Power_Nobody3);              //无人感应功率3:0~100%,步长10%,  单位:%,  默认0x1E	   50%,0x32

	EE_FLASH_WriteVariable(Addr_Detection_Delay,ConfigData.Detection_Delay);          //有人感应延迟时间:0~255s,步长1s, 单位:s, 默认0x1E
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Vol,ConfigData.Optical_Ctl_Vol);          //光控点电压:5~11V,步长1V,       单位:V,  默认0x05
	
	EE_FLASH_WriteVariable(Addr_Optical_Ctl_Delay,ConfigData.Optical_Ctl_Delay);      //光控延迟:0~50min,步长1min,     单位:min,默认0x05		

	EE_FLASH_WriteVariable(Addr_Load_Current,ConfigData.Load_RatedCurrent);           //负载电流:0~6000mA,步长10mA,    单位:mA, 默认0x012C ,300mA    
	
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
*函数功能： 获取出厂默认配置参数
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
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
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/
void ConfigData_Init(void)
{	
	EE_FLASH_Init();         //初始化FLASH
	
	EE_FLASH_ReadVariable(Addr_Config_Tag,(u16*)&ConfigData.Config_Tag);    //如果没有配置过则开始使用默认配置
	
	if(ConfigData.Config_Tag == 0x00)
	{
  	#ifdef  USERAPP_DEBUG_INFO
	    printf("未读取到有效配置，正在恢复出厂值.....\r\n\r\n");
    #endif
		SetDefault_ConfigData();
		GetDefault_ConfigData();
	}
	else if(ConfigData.Config_Tag == 0xAA)
	{
    #ifdef  USERAPP_DEBUG_INFO
	    printf("发现有效配置，正在读取......\r\n\r\n");
    #endif
		GetDefault_ConfigData();		
	}	
}

/*******************************************************************
*WorkInPowerOnTest()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/

void WorkInPowerOnTest(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_STANDBY;
		Stop_Boot_PWM();
		ModeDelay =  5000;   //FOR TEST
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\n模式切换至休眠……\r\n");
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
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/

void WorkInStandby(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_IDLE;
		Stop_Boot_PWM();
		Boost_Working = false;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\n模式切换至白天待机……\r\n");
		#endif
	}
}


/*******************************************************************
*WorkInIdleMode()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/

void WorkInIdleMode(void)
{
	if( GetSolarVolt() < ConfigData.Optical_Ctl_Vol)                 //输入低于光控值,开始光控延迟
	{
		ConfigData.Contorller_Mode = MODE_LIGHTCTL;
		ModeDelay = ConfigData.Optical_Ctl_Delay * 60 * 1000;
		//ModeDelay =  30000;   //FOR TEST
		Stop_Boot_PWM();
		Boost_Working = false;
		
		#ifdef  USERAPP_DEBUG_INFO
			printf("\r\n模式切换至光控延迟……\r\n");
		#endif
	}
	else
	{
		//电池过充保护
		if((GetBaterryVolt()) > ConfigData.OverCharge_Vol / 10)   
		{
			DISABLE_DIRECT_CHARGE;
		}
		else         //电池过放保护.且温度零上，开启充电
		if(((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10) && (GetSTM32Temp() > 0))
		{
			ENABLE_DIRECT_CHARGE;
			LED_DaleyTime = 100;
		}       
	}	
}

/*******************************************************************
*WorkInLightControlDelay()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
********************************************************************/

void WorkInLightControlDelay(void)
{
	if(ModeDelay == 0)
	{
		ConfigData.Contorller_Mode = MODE_TIMESLOT1;
		ModeDelay = ConfigData.Detect_Time1 * 60 * 60 * 1000;
		//ModeDelay =  5000;    //FOR TEST
		
		//电池电压大于过放返回值才会启动控制器
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
			printf("\r\n模式切换至感应时间段1……\r\n");
		#endif
	}	
}

/*******************************************************************
*WorkInTimeSlot1()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
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
			printf("\r\n模式切换至感应时间段2……\r\n");
		#endif				
	}
	
	//电池过放保护
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	}		
}

/*******************************************************************
*WorkInPowerOnTest()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
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
			printf("\r\n模式切换至感应时间段3……\r\n");
		#endif				
	}
	
	//电池过放保护
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	} 	
}

/*******************************************************************
*WorkInPowerOnTest()
*函数功能：
*输入参数：无
*返回参数：无
*编写作者：
*编写时间：
*相关说明：
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
			printf("\r\n模式切换至白天待机……\r\n");
		#endif				
	}

	//电池过放保护
	if((GetBaterryVolt())< ConfigData.OverDischarge_Vol / 10)
	{
		Boost_Working = false;
		Stop_Boot_PWM();
		LED_DaleyTime = 100;
	} 			
}
