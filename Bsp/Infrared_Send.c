//******************************************************************************              
//name:             Infrared_Send.c             
//introduce:        红外传感器发送驱动      
//author:                                                     
//changetime:       2016.11.16    
//******************************************************************************
#include "Infrared_Send.h"
#include "Usart1.h"
#include "Control_GPIO.h"

/*********************外部变量************************/
//红外开关的标志位
u8 gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
u8 gGUA_Infrared_Sending = 0;  //红外发送标志
u8 GUA_USER_CODE;

//红外38K脉冲翻转次数
volatile unsigned short  gGUA_Infrared_Count;

/*
 * 函数名：IR_Send_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void IR_Send_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( GUA_INFRARED_SEND_RCC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GUA_INFRARED_SEND_PIN;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GUA_INFRARED_SEND_PORT, &GPIO_InitStructure);

  GPIO_SetBits(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN);	 // turn off all led
}



/*TIM_Period-->12+   TIM_Prescaler--71 -->中断周期为13us-38kHz*/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period = 12;		 					          /* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Prescaler = (72 -1);				        /* 时钟预分频数 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		    /* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     /* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);							            /* 清除溢出中断标志 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2, ENABLE);											                  /* 开启时钟 */
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);     //使能定时器2的时钟  
}


//******************************************************************************        
//name:             TIM2_IRQHandler        
//introduce:        定时器2中断服务函数,当红外的标志位打开时,能产生38K频率;能计数进中断次数。     
//parameter:        none       
//return:           none      
//author:                              
//changetime:       2016.11.16                      
//******************************************************************************  
void TIM2_IRQHandler(void)
{
	BitAction bGUA_Infrared_OnOff_Status = GUA_INFRARED_OFF;  
    
  //计数值计算  
  if(gGUA_Infrared_Count > 0)  
  {  
    gGUA_Infrared_Count--;  
  }  
    
  //红外标志被打开，则输出38K方波  
  if(gGUA_Infrared_Flag == GUA_INFRARED_FLAG_ON)  
  {  
    //读取当前红外输出状态  
    bGUA_Infrared_OnOff_Status = (BitAction)(GPIO_ReadOutputDataBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN));  
      
    //如果当前关闭，则打开  
    if(bGUA_Infrared_OnOff_Status == GUA_INFRARED_OFF)  
    {  
      GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_ON);
      LED2(ON);      
    }  
    //如果当前打开，则关闭  
    else  
    {  
      GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_OFF);
      LED2(OFF);			
    }	
  
  }  
  //红外标志被关闭  
  else  
  {  
    //关闭红外  
    GPIO_WriteBit(GUA_INFRARED_SEND_PORT, GUA_INFRARED_SEND_PIN, GUA_INFRARED_OFF);
    LED2(OFF);		
  }  
    	
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    
	}
}


//******************************************************************************        
//name:             GUA_Infrared_Send        
//introduce:        红外发送     
//parameter:        nGUA_Data:要发送的数据       
//return:           none      
//author:                                 
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Send_NByte(u8 GUA_USER_CODE,u8 DataLengh, u8 *nGUA_Data)
{
	u8 i,j = 0;
	u8 datatemp = 0;
	u8 rcr_temp= 0;
  
  //初始化红外，默认为关
	EXTI->IMR &= ~(EXTI_Line6);                               // 屏蔽红外接收,外部中断6
	
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);     //使能定时器2的时钟
	
  //9ms起始信号低电平,683*13us=8.879ms
  gGUA_Infrared_Count = 683;	
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
	while(gGUA_Infrared_Count);
  
  //4.5ms起始信号高电平，341*13=4.433ms
  gGUA_Infrared_Count = 341;
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
	while(gGUA_Infrared_Count);

  	
	#ifdef  INFRARED_SEND_DEBUG_INFO
    printf("Send Infrared_Data:\r\n");
  #endif
	

	//发送用户码
	datatemp = GUA_USER_CODE;
  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif 
  
	for(j = 0; j <  8; j++)
	{
		//通用的0.56ms低电平,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count); 
	
		//数值0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms高电平，43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//数值1
		else
		{
			//1.69ms高电平，128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//执行高电平
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count); 

		//数据移位
		datatemp <<= 1;
	}
	
	//发送用户码反码
	datatemp = (u8)~GUA_USER_CODE;
  	  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif
  
	for(j = 0; j <  8; j++)
	{
		//通用的0.56ms低电平,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count);    
	
		//数值0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms高电平，43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//数值1
		else
		{
			//1.69ms高电平，128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//执行高电平
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count);

		//数据移位
		datatemp <<= 1;
	}
	
  //循环发送NByte的数据
  for(i = 0; i < DataLengh ; i++)
  {
    datatemp =  *(nGUA_Data + i);
		rcr_temp = rcr_temp + datatemp;
		
	  #ifdef  INFRARED_SEND_DEBUG_INFO
      printf("0x%02X-",datatemp);
    #endif		
		
		for(j = 0; j <  8; j++)
		{
			//通用的0.56ms低电平,42*13us=0.546ms
			gGUA_Infrared_Count = 42;
			gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
			while(gGUA_Infrared_Count);
		
			//数值0
			if((datatemp & 0x80) == 0)
			{
				//0.565ms高电平，43*13us=0.559ms
				gGUA_Infrared_Count = 43;          
			}
			//数值1
			else
			{
				//1.69ms高电平，128*13us=1.664ms
				gGUA_Infrared_Count = 128;
			}
			
			//执行高电平
			gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
			while(gGUA_Infrared_Count); 

			//数据移位
			datatemp <<= 1;
		}		
  }
		
	//发送CRC
	datatemp = (u8)~rcr_temp + 1;
  	  
  #ifdef  INFRARED_SEND_DEBUG_INFO
    printf("0x%02X-",datatemp);
  #endif
  
	for(j = 0; j <  8; j++)
	{
		//通用的0.56ms低电平,42*13us=0.546ms
		gGUA_Infrared_Count = 42;
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
		while(gGUA_Infrared_Count);    
	
		//数值0
		if((datatemp & 0x80) == 0)
		{
			//0.565ms高电平，43*13us=0.559ms
			gGUA_Infrared_Count = 43;          
		}
		//数值1
		else
		{
			//1.69ms高电平，128*13us=1.664ms
			gGUA_Infrared_Count = 128;
		}
		
		//执行高电平
		gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;
		while(gGUA_Infrared_Count);

		//数据移位
		datatemp <<= 1;
	}
	
  
  //0.263ms低电平作为结束位，20*13=0.26ms
  gGUA_Infrared_Count = 20;
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_ON;
	while(gGUA_Infrared_Count); 
  
  //关闭红外
  gGUA_Infrared_Flag = GUA_INFRARED_FLAG_OFF;	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		    //不发送的时候，关掉38K定时器的时钟 	
	
	#ifdef  INFRARED_SEND_DEBUG_INFO
    printf("\r\n");
  #endif
	
	EXTI->IMR |= EXTI_Line6;                                      // 屏蔽红外接收,外部中断6
}

//******************************************************************************        
//name:             GUA_Infrared_Send_Init        
//introduce:        红外发送初始化     
//parameter:        none       
//return:           none      
//author:                                    
//changetime:       2016.11.16       
//******************************************************************************  
void Infrared_Send_Init(void)
{  
  IR_Send_GPIO_Config();
  TIM2_Configuration();
}

