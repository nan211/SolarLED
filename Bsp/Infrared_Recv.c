//******************************************************************************              
//name:             Infrared_Recv.c             
//introduce:        红外传感器接收驱动      
//author:   
//2080620 : Infrared_Recv.c line271,line277,增加2条指令，用于恢复出厂设置和回读参数
//changetime:       2016.11.16    
//******************************************************************************
#include "Infrared_Recv.h"
#include "Infrared_Send.h"
#include "Usart1.h"
#include "UserApp.h"

bool Ir_RecvComplete;      //接收完成标志位
bool Ir_RecvFlag;          //表示开始接收
bool PIR_DetectionFlag;          //表示开始接收

uint8_t Ir_RecvBuff[GUA_INFRARED_FRAME_LEN * 8 + 1];   //用于记录两个下降沿之间的时间
volatile uint16_t IrCode_idx;        //用于索引接收到的数值
volatile uint16_t IrTimecnt;

extern u8 gGUA_Infrared_Sending;
extern uint16_t Detection_Delay_CNT;
extern struct ConfigData_Type ConfigData;

void IR_Pin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
	EXTI_ClearITPendingBit(EXTI_Line5);                             //PIR中断
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5); 
  EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  EXTI_ClearITPendingBit(EXTI_Line6);                            //IR接收中断
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6); 
  EXTI_InitStructure.EXTI_Line=EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure); 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure);
}


/*TIM_Period--1000   TIM_Prescaler--71 -->中断周期为50us*/
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    
	  TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period = 100;		 								/* 自动重装载寄存器周期的值(计数值) 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72 计数周期为1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);									/* 开启时钟 */
    TIM_Cmd(TIM3, DISABLE);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
}

void TIM3_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  IrTimecnt++;
	
	
	if(IrTimecnt > 10000)  //低电平超过500ms退出
	{
		IrCode_idx = 0;
		IrTimecnt = 0;
		Ir_RecvComplete = false;
		Ir_RecvFlag = false;

		TIM_Cmd(TIM3, DISABLE);
		
		#ifdef  IR_RECV_DEBUG_INFO
      printf("红外接收超时退出...\r\n");
    #endif
	}	
	
}

//******************************************************************************        
//name:             EXTI9_5_IRQHandler        
//introduce:        红外接收引脚外部中断
//                  在下降沿触发的 IO 口中断函数中，需要实现统计两个下降沿之间的时间，并将其记录在数组中。
//                  下降沿第一次触发时，清除当前定时器中的计数值，以便统计时间。之后每一次下降沿触发就记录下当前计数值，
//                  然后再对其清零。如果该时间在同步头的时间区间内，对索引进行清零，表示重新开始接收数据。
//                  完整接收同步头和 32 个数据之后，表示接收完
//                  引导码：9ms低电平+4.5ms高电平              
//                  数据0 ：0.56ms低电平+0.565ms高电平
//                  数据1 ：0.56ms低电平+1.685ms高电平
//                  结束码： 0.263ms低电平作为结束位
//                  以上电平标准与发射端相反,计数周期为50us
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//****************************************************************************** 
void EXTI9_5_IRQHandler(void)
{
  uint16_t Ir_time;	
	
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		if(Ir_RecvFlag)
		{
				Ir_time = IrTimecnt; 
			
				if(Ir_time < 150 && Ir_time >= 120 ) // 接收到引导码13.5ms,过滤12ms<13.5ms<15ms
				{
						IrCode_idx = 0;  // 开始接收数据
				}			

				Ir_RecvBuff[IrCode_idx] = Ir_time;    // 获取计数时间
				IrTimecnt = 0;                        // 清零计数时间，以便下次统计
				IrCode_idx++;                         // 接收到一个数据，索引加1

				if(IrCode_idx == (GUA_INFRARED_FRAME_LEN * 8 + 1))       // 如果接收到N个数据
				
				{
						TIM_Cmd(TIM3, DISABLE);					  
						Ir_RecvComplete = true;
						Ir_RecvFlag = false;
					  IrCode_idx = 0;
						IrTimecnt = 0;
					  
				}
		}
		else   // 下降沿第一次触发
		{
				IrCode_idx = 0;
				IrTimecnt = 0;
				Ir_RecvFlag = true;				
			  TIM_Cmd(TIM3, ENABLE);
		}

		EXTI_ClearITPendingBit(EXTI_Line6);  // 清除中断标志
 }
	
 	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		PIR_DetectionFlag = true; 
		EXTI_ClearITPendingBit(EXTI_Line5);  // 清除中断标志
	}
}

//******************************************************************************        
//name:             GUA_Infrared_Send_Init        
//introduce:        由于中断函数中接收并记录下的数据是两个下降沿之间的时间，并不是红外所发送的数据。
//introduce:        因此需要根据红外协议，对 32 个时间进行判断，从而获得红外真正发送的数据。      
//                  这个函数需要在红外完整接收数据后执行，可通过判断接收完成标志位 receiveComplete 来实现 
//                  引导码：9ms低电平+4.5ms高电平
//                  数据0 ：0.56ms低电平+0.565ms高电平
//                  数据1 ：0.56ms低电平+1.685ms高电平
//                  结束码：0.263ms低电平作为结束位
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************

uint8_t Infrared_DeCode(void)
{
  uint8_t i,j,idx=1; //idx 从1 开始表示对同步头的时间不处理
  uint8_t data_temp,crc_temp;
	uint8_t InfraredRecv_code[GUA_INFRARED_FRAME_LEN];
	
	#ifdef  IR_RECV_DEBUG_INFO
     printf("\r\nremote_code:\r\n");
  #endif	
  
  Ir_RecvComplete = false;
	
	for(i=0; i<GUA_INFRARED_FRAME_LEN; i++)
  {
		for(j=0; j<8; j++)
		{				
			if(Ir_RecvBuff[idx] >=9 && Ir_RecvBuff[idx] < 13)   //表示 0,(1.12ms=0.56ms+0.56ms)，,50us*19=0.95ms < 接收到的低电平长度 < 50us*27=1.35ms
			{
					data_temp = 0;
			}
			else if(Ir_RecvBuff[idx] >=18 && Ir_RecvBuff[idx]<25) //表示 1,(2.224ms=0.56ms+1.685ms),50us*36=1.8ms < 接收到的低电平长度 < 50us*50=2.5ms
			{
					data_temp = 1;
			}
			
			InfraredRecv_code[i] <<= 1;
			InfraredRecv_code[i] |= data_temp;
			idx++;
		}
		
		if((i>1)&&(i<GUA_INFRARED_FRAME_LEN-1))
		{
		   crc_temp += InfraredRecv_code[i];
		}
		
		#ifdef  IR_RECV_DEBUG_INFO
			printf("0x%02X-",InfraredRecv_code[i]);
		#endif
			
  }
	
	#ifdef  IR_RECV_DEBUG_INFO
     printf("\r\n");
  #endif
	
	
	ConfigData.Data_CRC = ~crc_temp + 1;
	
	if(ConfigData.Data_CRC == InfraredRecv_code[GUA_INFRARED_DATA_LEN +2])  //CRC校验正确
	{	
		if((InfraredRecv_code[0] == 0xAA)&(InfraredRecv_code[1] == 0x55))  //更新参数
		{
			#ifdef  IR_RECV_DEBUG_INFO
				printf("ConfigData.Data_CRC: 0x%02X check correct!\r\n\r\n",ConfigData.Data_CRC);
			#endif
			
			ConfigData.Battery_Type          = InfraredRecv_code[2];
			ConfigData.Detect_Time1          = InfraredRecv_code[3];
			ConfigData.Power_SomeBody1       = InfraredRecv_code[4];
			ConfigData.Power_Nobody1         = InfraredRecv_code[5];
			ConfigData.Detect_Time2          = InfraredRecv_code[6];
			ConfigData.Power_SomeBody2       = InfraredRecv_code[7];
			ConfigData.Power_Nobody2         = InfraredRecv_code[8];
			ConfigData.Detect_Time3          = InfraredRecv_code[9];
			ConfigData.Power_SomeBody3       = InfraredRecv_code[10];
			ConfigData.Power_Nobody3         = InfraredRecv_code[11];
			ConfigData.Detection_Delay       = InfraredRecv_code[12];
			ConfigData.Optical_Ctl_Vol       = InfraredRecv_code[13];
			ConfigData.Optical_Ctl_Delay     = InfraredRecv_code[14];
			ConfigData.Load_RatedCurrent     = ((InfraredRecv_code[15]<<8)|InfraredRecv_code[16]) * 10;
			ConfigData.Auto_Power            = InfraredRecv_code[17];
			ConfigData.Charging_Freezing     = InfraredRecv_code[18];
			ConfigData.OverDischarge_Vol     = InfraredRecv_code[19];
			ConfigData.OverDischarge_BackVol = InfraredRecv_code[20];
			ConfigData.OverCharge_Vol        = InfraredRecv_code[21];
			ConfigData.OverCharge_BackVol    = InfraredRecv_code[22];
			ConfigData.Data_CRC              = InfraredRecv_code[23];
			
			SaveUser_ConfigData();
			
//			#ifdef  IR_RECV_DEBUG_INFO
//				printf("\r\nConfigData.Load_RatedCurrent: %d mA!\r\n",ConfigData.Load_RatedCurrent );
//			#endif
//			
  			
			Infrared_Send_NByte(0xAA, GUA_INFRARED_DATA_LEN, (u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;
		}
		else if((InfraredRecv_code[0] == 0xBB)&(InfraredRecv_code[1] == 0x44))  //返回参数
		{
		  GetDefault_ConfigData();
			Infrared_Send_NByte(0xBB, GUA_INFRARED_DATA_LEN,(u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;			
		}
		else if((InfraredRecv_code[0] == 0xDD)&(InfraredRecv_code[1] == 0x22))  //恢复出厂
		{
		  SetDefault_ConfigData();
		  GetDefault_ConfigData();
			Infrared_Send_NByte(0xDD, GUA_INFRARED_DATA_LEN,(u8 *)(&ConfigData.Battery_Type));
			ConfigData.Data_CRC = 0;			
		}

	}
	else
	{
		#ifdef  IR_RECV_DEBUG_INFO
		 printf("ConfigData.Data_CRC: 0x%02X check incorrect!\r\n",ConfigData.Data_CRC);
		#endif
	}

	
	Ir_RecvComplete = false;
	
  return InfraredRecv_code[2]; 

}

//******************************************************************************        
//name:             Infrared_Recv_Init        
//introduce:        红外接收初始化     
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Recv_Init(void)
{  
  IR_Pin_init();
  TIM3_Configuration();
}



